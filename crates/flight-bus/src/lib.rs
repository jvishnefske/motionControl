//! The ~/src/middleware integration: the [`ModuleKind::Middleware`] modules.
//!
//! * [`ZenohBus`] ([`Backend::Real`]) opens a Zenoh peer session through
//!   `middleware::zenoh::ZenohTransport`.
//! * [`MemoryBus`] ([`Backend::Simulated`]) runs the in-process
//!   `middleware::transport::memory` hub. Its types are `!Send`, so it lives
//!   on a dedicated thread with a current-thread runtime and a `LocalSet`;
//!   the module talks to it over channels.
//!
//! Both do the same thing over their transport:
//!
//! * publish the latest vehicle telemetry at 10 Hz on
//!   `flight/<airframe>/telemetry` (JSON of [`Telemetry`]);
//! * publish every new [`TestReport`] on `flight/<airframe>/mission/report`
//!   (JSON). Reports come from a `watch` channel: [`Factory::report_sender`]
//!   is its sending side, which the orchestrator writes to;
//! * subscribe to `flight/**` and count what comes back (rate and last key
//!   are shown in [`Module::status`]), which proves the round trip;
//! * answer requests on `flight/<airframe>/cmd`: the body `status` returns
//!   the bus statistics as JSON ([`BusStats`]).

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{Context as _, anyhow};
use async_trait::async_trait;
use flight_core::{
    Airframe, Backend, LogLevel, Module, ModuleFactory, ModuleKind, ModuleStatus, SimContext,
    Telemetry, TestReport,
};
use middleware::transport::memory::MemoryBus as MwMemoryBus;
use middleware::zenoh::{SessionConfig, ZenohTransport};
use middleware::{
    HandlerError, Key, KeyFilter, Publisher, Request, RequestOptions, Requester, Responder,
    SenderId, Subscribe, Subscriber,
};
use serde::Serialize;
use tokio::sync::{mpsc, oneshot, watch};

/// Telemetry publish period (10 Hz).
pub const PUBLISH_PERIOD: Duration = Duration::from_millis(100);
/// Every key this crate uses starts with this segment; the subscriber
/// listens on `flight/**`.
pub const KEY_ROOT: Key<'static> = Key::from_static("flight");
/// Default listen endpoint of the Zenoh peer.
pub const DEFAULT_ZENOH_LISTEN: &str = "tcp/0.0.0.0:7447";

const RX_BUF: usize = 64 * 1024;
const SCRATCH: usize = 16 * 1024;

/// The key segment for an airframe (its serde name).
pub fn airframe_slug(af: Airframe) -> &'static str {
    match af {
        Airframe::Quadrotor => "quadrotor",
        Airframe::FixedWing => "fixed_wing",
    }
}

/// `flight/<airframe>/telemetry`.
pub fn telemetry_key(af: Airframe) -> Key<'static> {
    match af {
        Airframe::Quadrotor => Key::from_static("flight/quadrotor/telemetry"),
        Airframe::FixedWing => Key::from_static("flight/fixed_wing/telemetry"),
    }
}

/// `flight/<airframe>/mission/report`.
pub fn report_key(af: Airframe) -> Key<'static> {
    match af {
        Airframe::Quadrotor => Key::from_static("flight/quadrotor/mission/report"),
        Airframe::FixedWing => Key::from_static("flight/fixed_wing/mission/report"),
    }
}

/// `flight/<airframe>/cmd`.
pub fn cmd_key(af: Airframe) -> Key<'static> {
    match af {
        Airframe::Quadrotor => Key::from_static("flight/quadrotor/cmd"),
        Airframe::FixedWing => Key::from_static("flight/fixed_wing/cmd"),
    }
}

/// The `flight/**` filter.
pub fn flight_filter() -> KeyFilter<'static> {
    KeyFilter::Prefix(KEY_ROOT)
}

/// Statistics of a running bus, returned by the `status` command.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct BusStats {
    pub transport: String,
    pub airframe: String,
    /// Samples published (telemetry and reports).
    pub published: u64,
    /// Samples received on `flight/**`.
    pub received: u64,
    /// Receive rate over the last second or so.
    pub rate_hz: f64,
    pub last_key: Option<String>,
    pub commands_served: u64,
    pub last_error: Option<String>,
}

struct StatsCell {
    stats: BusStats,
    window_start: Instant,
    window_count: u64,
}

/// Counters shared between the bus tasks and the module.
struct Shared(Mutex<StatsCell>);

impl Shared {
    fn new(transport: &str, af: Airframe) -> Arc<Self> {
        Arc::new(Shared(Mutex::new(StatsCell {
            stats: BusStats {
                transport: transport.to_string(),
                airframe: airframe_slug(af).to_string(),
                ..BusStats::default()
            },
            window_start: Instant::now(),
            window_count: 0,
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut StatsCell) -> R) -> R {
        let mut g = self.0.lock().unwrap_or_else(|e| e.into_inner());
        f(&mut g)
    }

    fn on_received(&self, key: &str) {
        self.with(|c| {
            c.stats.received += 1;
            if c.stats.last_key.as_deref() != Some(key) {
                c.stats.last_key = Some(key.to_string());
            }
            Self::roll(c, Instant::now());
        });
    }

    fn roll(c: &mut StatsCell, now: Instant) {
        let dt = now.duration_since(c.window_start).as_secs_f64();
        if dt >= 1.0 {
            c.stats.rate_hz = (c.stats.received - c.window_count) as f64 / dt;
            c.window_start = now;
            c.window_count = c.stats.received;
        }
    }

    fn on_published(&self) {
        self.with(|c| c.stats.published += 1);
    }

    fn on_command(&self) {
        self.with(|c| c.stats.commands_served += 1);
    }

    fn on_error(&self, e: impl Into<String>) {
        let e = e.into();
        self.with(|c| c.stats.last_error = Some(e));
    }

    fn snapshot(&self) -> BusStats {
        self.with(|c| {
            let now = Instant::now();
            // Nothing received for a while: let the rate decay to the truth.
            if now.duration_since(c.window_start) >= Duration::from_secs(2) {
                Self::roll(c, now);
            }
            c.stats.clone()
        })
    }
}

fn summary(endpoint: &str, s: &BusStats) -> String {
    let mut out = format!(
        "{endpoint}: rx {} ({:.1}/s), tx {}, last {}",
        s.received,
        s.rate_hz,
        s.published,
        s.last_key.as_deref().unwrap_or("-")
    );
    if let Some(e) = &s.last_error {
        out.push_str(&format!(", error: {e}"));
    }
    out
}

// ---------------------------------------------------------------------------
// Transport-generic loops. Instantiated with concrete transports, so the Zenoh
// futures stay `Send` and the memory ones run on a LocalSet.

async fn publish_loop<P: Publisher>(
    mut p: P,
    af: Airframe,
    mut telemetry: watch::Receiver<Option<Telemetry>>,
    mut reports: watch::Receiver<Option<TestReport>>,
    shared: Arc<Shared>,
) {
    let mut tick = tokio::time::interval(PUBLISH_PERIOD);
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut reports_open = true;
    // Publish a report that was set before the bus started.
    reports.mark_changed();
    loop {
        tokio::select! {
            _ = tick.tick() => {
                let t = *telemetry.borrow_and_update();
                if let Some(t) = t {
                    send_json(&mut p, telemetry_key(af), &t, &shared).await;
                }
            }
            r = reports.changed(), if reports_open => {
                if r.is_err() {
                    reports_open = false;
                    continue;
                }
                let rep = reports.borrow_and_update().clone();
                if let Some(rep) = rep {
                    send_json(&mut p, report_key(af), &rep, &shared).await;
                }
            }
        }
    }
}

async fn send_json<P: Publisher, T: Serialize>(p: &mut P, key: Key<'_>, v: &T, shared: &Shared) {
    match serde_json::to_vec(v) {
        Ok(bytes) => match p.publish(key, &bytes).await {
            Ok(()) => shared.on_published(),
            Err(e) => shared.on_error(format!("publish {key}: {e:?}")),
        },
        Err(e) => shared.on_error(format!("encode: {e}")),
    }
}

async fn subscribe_loop<S: Subscriber>(mut s: S, shared: Arc<Shared>) {
    let mut buf = vec![0u8; RX_BUF];
    loop {
        match s.recv(&mut buf).await {
            Ok(sample) => shared.on_received(sample.key.as_str()),
            Err(e) => {
                shared.on_error(format!("recv: {e:?}"));
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        }
    }
}

/// The `flight/<airframe>/cmd` handler: `status` returns [`BusStats`] JSON.
fn handle_cmd(shared: &Shared, req: &Request<'_>, out: &mut [u8]) -> Result<usize, HandlerError> {
    let body = core::str::from_utf8(req.body).map_err(|_| HandlerError::InvalidRequest)?;
    match body.trim() {
        "status" => {
            shared.on_command();
            let json = serde_json::to_vec(&shared.snapshot()).map_err(|_| HandlerError::Internal)?;
            let slot = out.get_mut(..json.len()).ok_or(HandlerError::BufferTooSmall)?;
            slot.copy_from_slice(&json);
            Ok(json.len())
        }
        _ => Err(HandlerError::NotFound),
    }
}

async fn serve_loop<R: Responder>(mut r: R, shared: Arc<Shared>) {
    let mut scratch = vec![0u8; SCRATCH];
    let s2 = shared.clone();
    let mut handler = move |req: &Request<'_>, out: &mut [u8]| handle_cmd(&s2, req, out);
    loop {
        if let Err(e) = r.serve_one(&mut handler, &mut scratch).await {
            shared.on_error(format!("serve: {e:?}"));
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    }
}

// ---------------------------------------------------------------------------
// MemoryBus

type CmdReply = oneshot::Sender<anyhow::Result<String>>;

struct MemoryWorker {
    stop: oneshot::Sender<()>,
    cmds: mpsc::UnboundedSender<(String, CmdReply)>,
    thread: std::thread::JoinHandle<()>,
}

/// The simulated middleware: the in-process memory hub on its own thread.
pub struct MemoryBus {
    reports: watch::Receiver<Option<TestReport>>,
    shared: Option<Arc<Shared>>,
    worker: Option<MemoryWorker>,
    state: ModuleStatus,
}

impl MemoryBus {
    /// A bus publishing the reports seen on `reports`.
    pub fn new(reports: watch::Receiver<Option<TestReport>>) -> Self {
        MemoryBus { reports, shared: None, worker: None, state: ModuleStatus::Stopped }
    }

    /// Current statistics, `None` when stopped.
    pub fn stats(&self) -> Option<BusStats> {
        self.shared.as_ref().map(|s| s.snapshot())
    }

    /// Sends `cmd` as a request through the bus to `flight/<airframe>/cmd`
    /// (from a separate node on the hub) and returns the reply text.
    pub async fn command(&self, cmd: &str) -> anyhow::Result<String> {
        let w = self.worker.as_ref().ok_or_else(|| anyhow!("memory bus not running"))?;
        let (tx, rx) = oneshot::channel();
        w.cmds.send((cmd.to_string(), tx)).map_err(|_| anyhow!("memory bus thread gone"))?;
        rx.await.map_err(|_| anyhow!("memory bus thread gone"))?
    }
}

const NODE_PUB: SenderId = SenderId([1; 8]);
const NODE_SUB: SenderId = SenderId([2; 8]);
const NODE_CMD: SenderId = SenderId([3; 8]);
const NODE_REQ: SenderId = SenderId([4; 8]);

async fn memory_main(
    af: Airframe,
    telemetry: watch::Receiver<Option<Telemetry>>,
    reports: watch::Receiver<Option<TestReport>>,
    shared: Arc<Shared>,
    ready: oneshot::Sender<anyhow::Result<()>>,
    stop: oneshot::Receiver<()>,
    mut cmds: mpsc::UnboundedReceiver<(String, CmdReply)>,
) {
    let bus = MwMemoryBus::new();
    let setup = (|| -> Result<_, middleware::transport::memory::MemoryError> {
        let sub = bus.node(NODE_SUB).subscriber(flight_filter())?;
        let resp = bus.node(NODE_CMD).responder(KeyFilter::Exact(cmd_key(af)))?;
        Ok((sub, resp))
    })();
    let (sub, resp) = match setup {
        Ok(v) => v,
        Err(e) => {
            let _ = ready.send(Err(anyhow!("memory bus setup: {e:?}")));
            return;
        }
    };
    tokio::task::spawn_local(subscribe_loop(sub, shared.clone()));
    tokio::task::spawn_local(serve_loop(resp, shared.clone()));
    tokio::task::spawn_local(publish_loop(bus.node(NODE_PUB), af, telemetry, reports, shared.clone()));
    let mut req = bus.node(NODE_REQ);
    tokio::task::spawn_local(async move {
        let mut buf = vec![0u8; SCRATCH];
        let opts = RequestOptions::DEFAULT.with_timeout(Duration::from_secs(1));
        while let Some((cmd, reply)) = cmds.recv().await {
            let r = req
                .request_with(cmd_key(af), cmd.as_bytes(), &mut buf, &opts)
                .await
                .map_err(|e| anyhow!("request: {e:?}"))
                .and_then(|b| String::from_utf8(b.to_vec()).context("reply is not UTF-8"));
            let _ = reply.send(r);
        }
    });
    let _ = ready.send(Ok(()));
    let _ = stop.await;
    // Returning drops the LocalSet's tasks with the hub endpoints.
}

#[async_trait]
impl Module for MemoryBus {
    fn kind(&self) -> ModuleKind {
        ModuleKind::Middleware
    }
    fn backend(&self) -> Backend {
        Backend::Simulated
    }
    fn name(&self) -> &str {
        "memory-bus"
    }

    async fn start(&mut self, ctx: &SimContext) -> anyhow::Result<()> {
        self.stop().await?;
        self.state = ModuleStatus::Starting;
        let af = ctx.airframe;
        let shared = Shared::new("memory", af);
        let (stop_tx, stop_rx) = oneshot::channel();
        let (cmd_tx, cmd_rx) = mpsc::unbounded_channel();
        let (ready_tx, ready_rx) = oneshot::channel();
        let telemetry = ctx.telemetry.clone();
        let reports = self.reports.clone();
        let sh = shared.clone();
        let thread = std::thread::Builder::new()
            .name("memory-bus".into())
            .spawn(move || {
                let rt = match tokio::runtime::Builder::new_current_thread().enable_all().build() {
                    Ok(rt) => rt,
                    Err(e) => {
                        let _ = ready_tx.send(Err(anyhow!("runtime: {e}")));
                        return;
                    }
                };
                let local = tokio::task::LocalSet::new();
                local.block_on(&rt, memory_main(af, telemetry, reports, sh, ready_tx, stop_rx, cmd_rx));
            })
            .context("spawn memory-bus thread")?;
        let worker = MemoryWorker { stop: stop_tx, cmds: cmd_tx, thread };
        match ready_rx.await {
            Ok(Ok(())) => {
                self.worker = Some(worker);
                self.shared = Some(shared);
                self.state = ModuleStatus::Running(String::new());
                ctx.log("memory-bus", LogLevel::Info, format!("in-process bus up, publishing {}", telemetry_key(af)));
                Ok(())
            }
            Ok(Err(e)) => {
                let _ = tokio::task::spawn_blocking(move || worker.thread.join()).await;
                self.state = ModuleStatus::Failed(e.to_string());
                Err(e)
            }
            Err(_) => {
                let _ = tokio::task::spawn_blocking(move || worker.thread.join()).await;
                self.state = ModuleStatus::Failed("memory-bus thread died".into());
                Err(anyhow!("memory-bus thread died"))
            }
        }
    }

    async fn stop(&mut self) -> anyhow::Result<()> {
        if let Some(w) = self.worker.take() {
            let _ = w.stop.send(());
            drop(w.cmds);
            tokio::task::spawn_blocking(move || w.thread.join())
                .await
                .context("join memory-bus thread")?
                .map_err(|_| anyhow!("memory-bus thread panicked"))?;
        }
        self.shared = None;
        self.state = ModuleStatus::Stopped;
        Ok(())
    }

    fn status(&self) -> ModuleStatus {
        match (&self.state, &self.shared) {
            (ModuleStatus::Running(_), Some(s)) => ModuleStatus::Running(summary("memory", &s.snapshot())),
            (st, _) => st.clone(),
        }
    }
}

// ---------------------------------------------------------------------------
// ZenohBus

/// How the Zenoh peer session is set up.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ZenohConfig {
    /// Endpoints to listen on, default [`DEFAULT_ZENOH_LISTEN`].
    pub listen: Vec<String>,
    /// Endpoints to connect to (e.g. a zenohd router).
    pub connect: Vec<String>,
    /// Answer multicast scouts so other peers (and `mwctl scout`) find us.
    pub answer_scouts: bool,
    /// Also deliver this session's own publications to its `flight/**`
    /// subscriber, so the counter moves even with no other peer.
    pub local_delivery: bool,
}

impl Default for ZenohConfig {
    fn default() -> Self {
        ZenohConfig {
            listen: vec![DEFAULT_ZENOH_LISTEN.to_string()],
            connect: Vec::new(),
            answer_scouts: true,
            local_delivery: true,
        }
    }
}

impl ZenohConfig {
    /// Loopback only, no scouting: for tests and local runs.
    pub fn loopback(listen: impl Into<String>) -> Self {
        ZenohConfig { listen: vec![listen.into()], connect: Vec::new(), answer_scouts: false, local_delivery: true }
    }

    fn session_config(&self) -> SessionConfig {
        let mut c = SessionConfig::new();
        for l in &self.listen {
            c = c.listen(l.clone());
        }
        for e in &self.connect {
            c = c.connect(e.clone());
        }
        if self.answer_scouts {
            c = c.answer_scouts();
        }
        c
    }

    fn endpoint_label(&self) -> String {
        match (self.listen.first(), self.connect.first()) {
            (Some(l), _) => format!("zenoh {l}"),
            (None, Some(c)) => format!("zenoh -> {c}"),
            (None, None) => "zenoh (isolated)".to_string(),
        }
    }
}

/// The real middleware: a Zenoh peer session.
pub struct ZenohBus {
    config: ZenohConfig,
    reports: watch::Receiver<Option<TestReport>>,
    transport: Option<ZenohTransport>,
    tasks: Vec<tokio::task::JoinHandle<()>>,
    shared: Option<Arc<Shared>>,
    state: ModuleStatus,
}

impl ZenohBus {
    pub fn new(config: ZenohConfig, reports: watch::Receiver<Option<TestReport>>) -> Self {
        ZenohBus { config, reports, transport: None, tasks: Vec::new(), shared: None, state: ModuleStatus::Stopped }
    }

    /// Current statistics, `None` when stopped.
    pub fn stats(&self) -> Option<BusStats> {
        self.shared.as_ref().map(|s| s.snapshot())
    }

    async fn open(&mut self, ctx: &SimContext) -> anyhow::Result<()> {
        let af = ctx.airframe;
        let config = self.config.session_config().build().map_err(|e| anyhow!("zenoh config: {e}"))?;
        let t = ZenohTransport::open(config)
            .await
            .map_err(|e| anyhow!("zenoh open: {e}"))?
            .with_local_delivery(self.config.local_delivery);
        let shared = Shared::new("zenoh", af);
        let mut sub_t = t.clone();
        let sub = sub_t.subscribe(flight_filter()).await.map_err(|e| anyhow!("zenoh subscribe: {e}"))?;
        let resp = t.responder(cmd_key(af)).await.map_err(|e| anyhow!("zenoh responder: {e}"))?;
        self.tasks.push(tokio::spawn(subscribe_loop(sub, shared.clone())));
        self.tasks.push(tokio::spawn(serve_loop(resp, shared.clone())));
        self.tasks.push(tokio::spawn(publish_loop(
            t.clone(),
            af,
            ctx.telemetry.clone(),
            self.reports.clone(),
            shared.clone(),
        )));
        self.transport = Some(t);
        self.shared = Some(shared);
        Ok(())
    }
}

#[async_trait]
impl Module for ZenohBus {
    fn kind(&self) -> ModuleKind {
        ModuleKind::Middleware
    }
    fn backend(&self) -> Backend {
        Backend::Real
    }
    fn name(&self) -> &str {
        "zenoh-bus"
    }

    async fn start(&mut self, ctx: &SimContext) -> anyhow::Result<()> {
        self.stop().await?;
        self.state = ModuleStatus::Starting;
        match self.open(ctx).await {
            Ok(()) => {
                self.state = ModuleStatus::Running(String::new());
                ctx.log(
                    "zenoh-bus",
                    LogLevel::Info,
                    format!("{} up, publishing {}", self.config.endpoint_label(), telemetry_key(ctx.airframe)),
                );
                Ok(())
            }
            Err(e) => {
                let _ = self.stop().await;
                self.state = ModuleStatus::Failed(e.to_string());
                ctx.log("zenoh-bus", LogLevel::Error, e.to_string());
                Err(e)
            }
        }
    }

    async fn stop(&mut self) -> anyhow::Result<()> {
        for t in self.tasks.drain(..) {
            t.abort();
            let _ = t.await;
        }
        self.shared = None;
        if let Some(t) = self.transport.take() {
            t.session().close().await.map_err(|e| anyhow!("zenoh close: {e}"))?;
        }
        self.state = ModuleStatus::Stopped;
        Ok(())
    }

    fn status(&self) -> ModuleStatus {
        match (&self.state, &self.shared) {
            (ModuleStatus::Running(_), Some(s)) => {
                ModuleStatus::Running(summary(&self.config.endpoint_label(), &s.snapshot()))
            }
            (st, _) => st.clone(),
        }
    }
}

// ---------------------------------------------------------------------------
// Factory

/// Builds the middleware modules: Real is [`ZenohBus`], Simulated is
/// [`MemoryBus`]. Owns the report channel the buses publish from.
pub struct Factory {
    zenoh: ZenohConfig,
    reports: watch::Sender<Option<TestReport>>,
}

impl Default for Factory {
    fn default() -> Self {
        Self::new()
    }
}

impl Factory {
    pub fn new() -> Self {
        Factory { zenoh: ZenohConfig::default(), reports: watch::channel(None).0 }
    }

    pub fn with_zenoh_config(mut self, config: ZenohConfig) -> Self {
        self.zenoh = config;
        self
    }

    /// The sending side of the report channel: whatever is sent here is
    /// published on `flight/<airframe>/mission/report` by the running bus.
    pub fn report_sender(&self) -> watch::Sender<Option<TestReport>> {
        self.reports.clone()
    }

    /// Publishes `report` through whichever bus is running.
    pub fn push_report(&self, report: TestReport) {
        self.reports.send_replace(Some(report));
    }
}

impl ModuleFactory for Factory {
    fn available(&self, kind: ModuleKind) -> Vec<(Backend, Result<(), String>)> {
        match kind {
            ModuleKind::Middleware => vec![(Backend::Real, Ok(())), (Backend::Simulated, Ok(()))],
            _ => Vec::new(),
        }
    }

    fn create(&self, kind: ModuleKind, backend: Backend) -> Option<Box<dyn Module>> {
        match (kind, backend) {
            (ModuleKind::Middleware, Backend::Real) => {
                Some(Box::new(ZenohBus::new(self.zenoh.clone(), self.reports.subscribe())))
            }
            (ModuleKind::Middleware, Backend::Simulated) => Some(Box::new(MemoryBus::new(self.reports.subscribe()))),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests;
