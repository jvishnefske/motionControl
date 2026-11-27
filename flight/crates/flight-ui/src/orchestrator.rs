use std::collections::{HashMap, VecDeque};
use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Instant;

use anyhow::{anyhow, bail};
use flight_core::{
    Airframe, Backend, Check, CheckState, GeoPoint, LogLevel, LogLine, MavPorts, Mission, MissionItem,
    MissionTest, Module, ModuleFactory, ModuleKind, ModuleStatus, SimContext, Telemetry, TestReport,
    TestVerdict, VehicleControl,
};
use serde::Serialize;
use tokio::sync::{broadcast, oneshot, watch};

/// A boxed, `Send` future.
pub type BoxFuture<T> = Pin<Box<dyn Future<Output = T> + Send + 'static>>;

/// Runs one mission test against the vehicle: uploads, arms, starts,
/// evaluates, streams intermediate reports into the sender and returns the
/// final report. Any `Fn(Arc<dyn VehicleControl>, Mission,
/// watch::Sender<Option<TestReport>>) -> BoxFuture<TestReport>` is one; see
/// [`runner_fn`] for async closures.
pub trait MissionRunnerFn: Send + Sync {
    fn run(
        &self,
        vehicle: Arc<dyn VehicleControl>,
        mission: Mission,
        reports: watch::Sender<Option<TestReport>>,
    ) -> BoxFuture<TestReport>;
}

impl<F> MissionRunnerFn for F
where
    F: Fn(Arc<dyn VehicleControl>, Mission, watch::Sender<Option<TestReport>>) -> BoxFuture<TestReport>
        + Send
        + Sync,
{
    fn run(
        &self,
        vehicle: Arc<dyn VehicleControl>,
        mission: Mission,
        reports: watch::Sender<Option<TestReport>>,
    ) -> BoxFuture<TestReport> {
        self(vehicle, mission, reports)
    }
}

/// The shared mission runner handle the orchestrator takes.
pub type Runner = Arc<dyn MissionRunnerFn>;

/// Wraps an async closure as a [`Runner`]:
///
/// ```ignore
/// let runner = flight_ui::runner_fn(|v, m, tx| async move {
///     flight_mav::run_mission_test(&*v, &m, &tx).await
/// });
/// ```
pub fn runner_fn<F, Fut>(f: F) -> Runner
where
    F: Fn(Arc<dyn VehicleControl>, Mission, watch::Sender<Option<TestReport>>) -> Fut + Send + Sync + 'static,
    Fut: Future<Output = TestReport> + Send + 'static,
{
    struct Wrap<F>(F);
    impl<F, Fut> MissionRunnerFn for Wrap<F>
    where
        F: Fn(Arc<dyn VehicleControl>, Mission, watch::Sender<Option<TestReport>>) -> Fut + Send + Sync,
        Fut: Future<Output = TestReport> + Send + 'static,
    {
        fn run(
            &self,
            vehicle: Arc<dyn VehicleControl>,
            mission: Mission,
            reports: watch::Sender<Option<TestReport>>,
        ) -> BoxFuture<TestReport> {
            Box::pin((self.0)(vehicle, mission, reports))
        }
    }
    Arc::new(Wrap(f))
}

/// Called with every backend change (before the module restarts), e.g. to
/// tell the flight-launch factory which autopilot backend is active.
pub type BackendHook = Arc<dyn Fn(ModuleKind, Backend) + Send + Sync>;

/// Preset names, for the UI.
pub const PRESETS: [&str; 4] = ["all-sim", "px4-sih", "full-stack", "sim+qgc"];

/// The backend of every kind (in [`ModuleKind::ALL`] order) a preset sets.
pub fn preset_backends(name: &str) -> Option<[(ModuleKind, Backend); 4]> {
    use Backend::{Real, Simulated as Sim};
    use ModuleKind::*;
    let (mw, ph, ap, gs) = match name {
        "all-sim" => (Sim, Sim, Sim, Sim),
        "px4-sih" => (Sim, Sim, Real, Sim),
        "full-stack" => (Real, Real, Real, Real),
        "sim+qgc" => (Sim, Sim, Sim, Real),
        _ => return None,
    };
    Some([(Middleware, mw), (Physics, ph), (Autopilot, ap), (GroundStation, gs)])
}

const LOG_HISTORY: usize = 300;
const TRACK_MAX: usize = 4000;

struct Config {
    backends: HashMap<ModuleKind, Backend>,
    airframe: Airframe,
    home: GeoPoint,
    gui: bool,
    mission: String,
    ports: MavPorts,
}

#[derive(Default)]
struct Slot {
    module: Option<Box<dyn Module>>,
    name: Option<String>,
    busy: Option<&'static str>,
    error: Option<String>,
}

struct RunState {
    mission: String,
    started: Instant,
    cancel: Option<oneshot::Sender<()>>,
}

/// One row of the module menu.
#[derive(Debug, Clone, Serialize)]
pub struct ModuleView {
    pub kind: ModuleKind,
    pub label: &'static str,
    pub backend: Backend,
    /// The running module's name, e.g. "px4-sitl".
    pub name: Option<String>,
    pub status: ModuleStatus,
    /// "starting" / "stopping" while a switch is in progress.
    pub busy: Option<&'static str>,
    pub options: Vec<BackendOption>,
}

#[derive(Debug, Clone, Serialize)]
pub struct BackendOption {
    pub backend: Backend,
    pub available: bool,
    pub reason: Option<String>,
}

/// Everything the UI shows, pushed at ~5 Hz.
#[derive(Debug, Clone, Serialize)]
pub struct Snapshot {
    pub airframe: Airframe,
    pub airframes: Vec<(Airframe, &'static str)>,
    pub home: GeoPoint,
    pub gui: bool,
    pub mission: String,
    pub missions: Vec<&'static str>,
    pub presets: Vec<&'static str>,
    pub modules: Vec<ModuleView>,
    pub telemetry: Option<Telemetry>,
    pub report: Option<TestReport>,
    pub test_running: bool,
    pub test_elapsed_s: Option<f64>,
    /// Vehicle track, (north, east) metres from home.
    pub track: Vec<[f64; 2]>,
    /// Selected mission's targets: (north, east, alt) metres from home.
    pub waypoints: Vec<[f64; 3]>,
}

/// Owns the modules and runs the one-click mission test.
pub struct Orchestrator {
    factory: Arc<dyn ModuleFactory>,
    vehicle: Arc<dyn VehicleControl>,
    runner: Runner,
    hook: Option<BackendHook>,
    /// Serialises module switches.
    ops: tokio::sync::Mutex<()>,
    config: Mutex<Config>,
    slots: Mutex<HashMap<ModuleKind, Slot>>,
    run: Mutex<Option<RunState>>,
    log: broadcast::Sender<LogLine>,
    history: Arc<Mutex<VecDeque<LogLine>>>,
    report_tx: watch::Sender<Option<TestReport>>,
    sinks: Mutex<Vec<watch::Sender<Option<TestReport>>>>,
    track: Mutex<Vec<[f64; 2]>>,
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

impl Orchestrator {
    /// Every backend starts `Off`, quadrotor at PX4's default home, mission
    /// "box", headless. Call inside a tokio runtime (a log collector task is
    /// spawned when one is present).
    pub fn new(factory: Arc<dyn ModuleFactory>, vehicle: Arc<dyn VehicleControl>, runner: Runner) -> Self {
        let (log, _) = broadcast::channel(1024);
        let history = Arc::new(Mutex::new(VecDeque::with_capacity(LOG_HISTORY)));
        if let Ok(rt) = tokio::runtime::Handle::try_current() {
            let mut rx = log.subscribe();
            let h = history.clone();
            rt.spawn(async move {
                loop {
                    match rx.recv().await {
                        Ok(line) => {
                            let mut h = lock(&h);
                            if h.len() >= LOG_HISTORY {
                                h.pop_front();
                            }
                            h.push_back(line);
                        }
                        Err(broadcast::error::RecvError::Lagged(_)) => continue,
                        Err(broadcast::error::RecvError::Closed) => break,
                    }
                }
            });
        }
        Orchestrator {
            factory,
            vehicle,
            runner,
            hook: None,
            ops: tokio::sync::Mutex::new(()),
            config: Mutex::new(Config {
                backends: ModuleKind::ALL.iter().map(|k| (*k, Backend::Off)).collect(),
                airframe: Airframe::Quadrotor,
                home: GeoPoint::PX4_DEFAULT_HOME,
                gui: false,
                mission: Mission::BUILTIN[0].to_string(),
                ports: MavPorts::default(),
            }),
            slots: Mutex::new(HashMap::new()),
            run: Mutex::new(None),
            log,
            history,
            report_tx: watch::channel(None).0,
            sinks: Mutex::new(Vec::new()),
            track: Mutex::new(Vec::new()),
        }
    }

    /// Installs the backend-change hook.
    pub fn with_backend_hook(mut self, hook: BackendHook) -> Self {
        self.hook = Some(hook);
        self
    }

    pub fn with_airframe(self, airframe: Airframe) -> Self {
        lock(&self.config).airframe = airframe;
        self
    }

    pub fn with_home(self, home: GeoPoint) -> Self {
        lock(&self.config).home = home;
        self
    }

    pub fn with_gui(self, gui: bool) -> Self {
        lock(&self.config).gui = gui;
        self
    }

    pub fn with_ports(self, ports: MavPorts) -> Self {
        lock(&self.config).ports = ports;
        self
    }

    /// Sets the initial backend of `kind` without starting anything (use
    /// [`start_all`](Self::start_all) afterwards). Calls the hook.
    pub fn with_backend(self, kind: ModuleKind, backend: Backend) -> Self {
        lock(&self.config).backends.insert(kind, backend);
        self.notify(kind, backend);
        self
    }

    /// Every report (intermediate and final) is also sent here, e.g. the
    /// flight-bus factory's `report_sender()`.
    pub fn add_report_sink(&self, sink: watch::Sender<Option<TestReport>>) {
        sink.send_replace(self.report_tx.borrow().clone());
        lock(&self.sinks).push(sink);
    }

    /// The current mission report.
    pub fn reports(&self) -> watch::Receiver<Option<TestReport>> {
        self.report_tx.subscribe()
    }

    /// The log channel modules write to.
    pub fn log_sender(&self) -> broadcast::Sender<LogLine> {
        self.log.clone()
    }

    /// Recent log lines, oldest first.
    pub fn log_history(&self) -> Vec<LogLine> {
        lock(&self.history).iter().cloned().collect()
    }

    pub fn backend(&self, kind: ModuleKind) -> Backend {
        lock(&self.config).backends.get(&kind).copied().unwrap_or(Backend::Off)
    }

    pub fn airframe(&self) -> Airframe {
        lock(&self.config).airframe
    }

    pub fn home(&self) -> GeoPoint {
        lock(&self.config).home
    }

    pub fn selected_mission(&self) -> String {
        lock(&self.config).mission.clone()
    }

    pub fn test_running(&self) -> bool {
        lock(&self.run).is_some()
    }

    /// The SimContext a module started now gets.
    pub fn context(&self) -> SimContext {
        let c = lock(&self.config);
        SimContext {
            airframe: c.airframe,
            home: c.home,
            ports: c.ports,
            physics_backend: c.backends.get(&ModuleKind::Physics).copied().unwrap_or(Backend::Off),
            gui: c.gui,
            telemetry: self.vehicle.telemetry(),
            log: self.log.clone(),
        }
    }

    fn log_line(&self, level: LogLevel, text: impl Into<String>) {
        let _ = self.log.send(LogLine { source: "orchestrator".into(), level, text: text.into() });
    }

    fn notify(&self, kind: ModuleKind, backend: Backend) {
        if let Some(h) = &self.hook {
            h(kind, backend);
        }
    }

    /// `Ok` if the factory offers `backend` for `kind` and it is usable.
    fn check_available(&self, kind: ModuleKind, backend: Backend) -> anyhow::Result<()> {
        if backend == Backend::Off {
            return Ok(());
        }
        match self.factory.available(kind).into_iter().find(|(b, _)| *b == backend) {
            Some((_, Ok(()))) => Ok(()),
            Some((_, Err(why))) => bail!("{} {:?} unavailable: {why}", kind.label(), backend),
            None => bail!("{} has no {:?} backend", kind.label(), backend),
        }
    }

    fn is_running(&self, kind: ModuleKind) -> bool {
        let slots = lock(&self.slots);
        slots
            .get(&kind)
            .and_then(|s| s.module.as_ref())
            .is_some_and(|m| matches!(m.status(), ModuleStatus::Running(_) | ModuleStatus::Starting))
    }

    async fn stop_kind(&self, kind: ModuleKind) {
        let module = {
            let mut slots = lock(&self.slots);
            let slot = slots.entry(kind).or_default();
            slot.error = None;
            let m = slot.module.take();
            if m.is_some() {
                slot.busy = Some("stopping");
            }
            m
        };
        if let Some(mut m) = module {
            match m.stop().await {
                Ok(()) => self.log_line(LogLevel::Info, format!("stopped {}", m.name())),
                Err(e) => self.log_line(LogLevel::Warn, format!("stopping {}: {e:#}", m.name())),
            }
        }
        let mut slots = lock(&self.slots);
        let slot = slots.entry(kind).or_default();
        slot.busy = None;
        slot.name = None;
    }

    async fn start_kind(&self, kind: ModuleKind) -> anyhow::Result<()> {
        let backend = self.backend(kind);
        if backend == Backend::Off {
            return Ok(());
        }
        let Some(mut m) = self.factory.create(kind, backend) else {
            let e = format!("{} {:?}: the factory cannot build it", kind.label(), backend);
            lock(&self.slots).entry(kind).or_default().error = Some(e.clone());
            self.log_line(LogLevel::Error, e.clone());
            bail!(e);
        };
        let name = m.name().to_string();
        {
            let mut slots = lock(&self.slots);
            let slot = slots.entry(kind).or_default();
            slot.busy = Some("starting");
            slot.name = Some(name.clone());
            slot.error = None;
        }
        self.log_line(LogLevel::Info, format!("starting {name} ({})", kind.label()));
        let ctx = self.context();
        let r = m.start(&ctx).await;
        match &r {
            Ok(()) => self.log_line(LogLevel::Info, format!("{name} running")),
            Err(e) => self.log_line(LogLevel::Error, format!("{name} failed to start: {e:#}")),
        }
        let mut slots = lock(&self.slots);
        let slot = slots.entry(kind).or_default();
        slot.busy = None;
        if let Err(e) = &r {
            slot.error = Some(format!("{e:#}"));
        }
        slot.module = Some(m);
        r
    }

    /// Stops `kinds` (reverse start order), then starts those not `Off`
    /// (start order). Returns the first start error; the others still start.
    async fn restart(&self, kinds: &[ModuleKind]) -> anyhow::Result<()> {
        for k in ModuleKind::ALL.iter().rev().filter(|k| kinds.contains(k)) {
            self.stop_kind(*k).await;
        }
        let mut first = None;
        for k in ModuleKind::ALL.iter().filter(|k| kinds.contains(k)) {
            if let Err(e) = self.start_kind(*k).await {
                first.get_or_insert(e);
            }
        }
        first.map_or(Ok(()), Err)
    }

    /// Starts every kind whose backend is not `Off` that is not running.
    pub async fn start_all(&self) -> anyhow::Result<()> {
        let _g = self.ops.lock().await;
        let kinds: Vec<_> = ModuleKind::ALL.into_iter().filter(|k| !self.is_running(*k)).collect();
        self.restart(&kinds).await
    }

    /// Stops every module, reverse start order.
    pub async fn shutdown(&self) {
        let _g = self.ops.lock().await;
        self.cancel_run();
        for k in ModuleKind::ALL.iter().rev() {
            self.stop_kind(*k).await;
        }
    }

    /// Switches `kind` to `backend` in place: stops the old module, then
    /// starts the new one; everything else keeps running. A change of the
    /// autopilot or physics backend restarts the autopilot too, so PX4 picks
    /// Gazebo or SIH.
    pub async fn set_backend(&self, kind: ModuleKind, backend: Backend) -> anyhow::Result<()> {
        self.check_available(kind, backend)?;
        let _g = self.ops.lock().await;
        lock(&self.config).backends.insert(kind, backend);
        self.log_line(LogLevel::Info, format!("{} -> {:?}", kind.label(), backend));
        self.notify(kind, backend);
        let mut kinds = vec![kind];
        if matches!(kind, ModuleKind::Physics) {
            kinds.push(ModuleKind::Autopilot);
        }
        self.restart(&kinds).await
    }

    /// Sets several backends at once; see [`PRESETS`].
    pub async fn apply_preset(&self, name: &str) -> anyhow::Result<()> {
        let target = preset_backends(name).ok_or_else(|| anyhow!("unknown preset {name:?}"))?;
        let unavailable: Vec<String> =
            target.iter().filter_map(|(k, b)| self.check_available(*k, *b).err().map(|e| e.to_string())).collect();
        if !unavailable.is_empty() {
            bail!("preset {name}: {}", unavailable.join("; "));
        }
        let _g = self.ops.lock().await;
        let mut kinds = Vec::new();
        for (k, b) in target {
            let old = lock(&self.config).backends.insert(k, b);
            if old != Some(b) || !self.is_running(k) {
                kinds.push(k);
            }
            if old != Some(b) {
                self.notify(k, b);
            }
        }
        if kinds.contains(&ModuleKind::Physics) && !kinds.contains(&ModuleKind::Autopilot) {
            kinds.push(ModuleKind::Autopilot);
        }
        self.log_line(LogLevel::Info, format!("preset {name}"));
        self.restart(&kinds).await
    }

    /// Changes the airframe and restarts the middleware (its keys name the
    /// airframe), physics and autopilot. Refused during a mission test.
    pub async fn set_airframe(&self, airframe: Airframe) -> anyhow::Result<()> {
        if self.test_running() {
            bail!("a mission test is running");
        }
        let _g = self.ops.lock().await;
        {
            let mut c = lock(&self.config);
            if c.airframe == airframe {
                return Ok(());
            }
            c.airframe = airframe;
        }
        lock(&self.track).clear();
        self.log_line(LogLevel::Info, format!("airframe -> {}", airframe.label()));
        self.restart(&[ModuleKind::Middleware, ModuleKind::Physics, ModuleKind::Autopilot]).await
    }

    /// GUI flag for modules started from now on.
    pub fn set_gui(&self, gui: bool) {
        lock(&self.config).gui = gui;
    }

    /// Home for missions and modules started from now on.
    pub fn set_home(&self, home: GeoPoint) {
        lock(&self.config).home = home;
    }

    /// Selects the mission shown on the map and run by default.
    pub fn select_mission(&self, name: &str) -> anyhow::Result<()> {
        if !Mission::BUILTIN.contains(&name) {
            bail!("unknown mission {name:?}");
        }
        lock(&self.config).mission = name.to_string();
        Ok(())
    }

    /// The one-click mission test: starts the autopilot and physics if they
    /// are off (Simulated, or Real if that is all there is), builds the
    /// built-in mission, and runs the runner in a task that streams reports.
    /// Returns once the task is spawned; refuses a second concurrent run.
    pub fn run_mission_test(self: &Arc<Self>, mission_name: &str) -> anyhow::Result<()> {
        let (airframe, home) = {
            let c = lock(&self.config);
            (c.airframe, c.home)
        };
        let mission = Mission::builtin(mission_name, airframe, home)
            .ok_or_else(|| anyhow!("unknown mission {mission_name:?}"))?;
        let (cancel_tx, cancel_rx) = oneshot::channel();
        {
            let mut run = lock(&self.run);
            if let Some(r) = run.as_ref() {
                bail!("mission test {:?} is already running", r.mission);
            }
            *run = Some(RunState { mission: mission_name.to_string(), started: Instant::now(), cancel: Some(cancel_tx) });
        }
        lock(&self.config).mission = mission_name.to_string();
        let me = self.clone();
        tokio::spawn(async move {
            let report = me.test_task(mission, cancel_rx).await;
            me.publish_report(Some(report.clone()));
            let level = if report.verdict == TestVerdict::Passed { LogLevel::Info } else { LogLevel::Error };
            me.log_line(level, format!("mission test {}: {:?} in {:.1} s", report.mission, report.verdict, report.elapsed_s));
            *lock(&me.run) = None;
        });
        Ok(())
    }

    async fn test_task(self: &Arc<Self>, mission: Mission, mut cancel: oneshot::Receiver<()>) -> TestReport {
        let started = Instant::now();
        let initial = MissionTest::new(&mission).report().clone();
        self.publish_report(Some(initial.clone()));
        lock(&self.track).clear();
        self.log_line(LogLevel::Info, format!("mission test {} ({}) starting", mission.name, mission.airframe.label()));

        // Bring up what the mission needs.
        let needed = {
            let _g = self.ops.lock().await;
            let mut kinds = Vec::new();
            for kind in [ModuleKind::Physics, ModuleKind::Autopilot] {
                if self.backend(kind) == Backend::Off {
                    let pick = [Backend::Simulated, Backend::Real]
                        .into_iter()
                        .find(|b| self.check_available(kind, *b).is_ok());
                    match pick {
                        Some(b) => {
                            lock(&self.config).backends.insert(kind, b);
                            self.notify(kind, b);
                            self.log_line(LogLevel::Info, format!("{} was off, starting {:?}", kind.label(), b));
                        }
                        None => {
                            return failed(initial, started, format!("no usable backend for {}", kind.label()));
                        }
                    }
                }
                if !self.is_running(kind) {
                    kinds.push(kind);
                }
            }
            // Physics (re)started: the autopilot must follow it.
            if kinds.contains(&ModuleKind::Physics) && !kinds.contains(&ModuleKind::Autopilot) {
                kinds.push(ModuleKind::Autopilot);
            }
            self.restart(&kinds).await
        };
        if let Err(e) = needed {
            return failed(initial, started, format!("modules failed to start: {e:#}"));
        }

        // Forward intermediate reports to the sinks while the runner works.
        let (tx, mut rx) = watch::channel(Some(initial.clone()));
        let me = self.clone();
        let forward = tokio::spawn(async move {
            while rx.changed().await.is_ok() {
                let r = rx.borrow_and_update().clone();
                me.publish_report(r);
            }
        });
        let fut = self.runner.run(self.vehicle.clone(), mission, tx);
        let report = tokio::select! {
            r = fut => r,
            _ = &mut cancel => {
                let last = self.report_tx.borrow().clone().unwrap_or(initial);
                failed(last, started, "aborted: return to launch".into())
            }
        };
        let _ = forward.await;
        report
    }

    fn publish_report(&self, r: Option<TestReport>) {
        for s in lock(&self.sinks).iter() {
            s.send_replace(r.clone());
        }
        self.report_tx.send_replace(r);
    }

    fn cancel_run(&self) {
        if let Some(r) = lock(&self.run).as_mut()
            && let Some(c) = r.cancel.take()
        {
            let _ = c.send(());
        }
    }

    /// Commands return-to-launch and ends a running test as failed.
    pub async fn abort_test(&self) -> anyhow::Result<()> {
        self.log_line(LogLevel::Warn, "abort: return to launch");
        let rtl = self.vehicle.return_to_launch().await;
        if let Err(e) = &rtl {
            self.log_line(LogLevel::Error, format!("RTL failed: {e:#}"));
        }
        self.cancel_run();
        rtl
    }

    /// What the UI shows.
    pub fn snapshot(&self) -> Snapshot {
        let (airframe, home, gui, mission, backends) = {
            let c = lock(&self.config);
            (c.airframe, c.home, c.gui, c.mission.clone(), c.backends.clone())
        };
        let telemetry = *self.vehicle.telemetry().borrow();
        if let Some(t) = &telemetry {
            let (n, e) = home.ne_to(&t.position);
            let mut track = lock(&self.track);
            let moved = track.last().is_none_or(|p| (p[0] - n).hypot(p[1] - e) > 0.3);
            if moved {
                if track.len() >= TRACK_MAX {
                    track.drain(..TRACK_MAX / 4);
                }
                track.push([n, e]);
            }
        }
        let modules = {
            let slots = lock(&self.slots);
            ModuleKind::ALL
                .into_iter()
                .map(|kind| {
                    let slot = slots.get(&kind);
                    let status = match slot {
                        Some(Slot { busy: Some(_), .. }) => ModuleStatus::Starting,
                        Some(Slot { error: Some(e), module: None, .. }) => ModuleStatus::Failed(e.clone()),
                        Some(Slot { module: Some(m), .. }) => m.status(),
                        _ => ModuleStatus::Stopped,
                    };
                    let mut options = vec![BackendOption { backend: Backend::Off, available: true, reason: None }];
                    options.extend(self.factory.available(kind).into_iter().map(|(b, r)| BackendOption {
                        backend: b,
                        available: r.is_ok(),
                        reason: r.err(),
                    }));
                    ModuleView {
                        kind,
                        label: kind.label(),
                        backend: backends.get(&kind).copied().unwrap_or(Backend::Off),
                        name: slot.and_then(|s| s.name.clone()),
                        status,
                        busy: slot.and_then(|s| s.busy),
                        options,
                    }
                })
                .collect()
        };
        let waypoints = Mission::builtin(&mission, airframe, home)
            .map(|m| {
                m.items
                    .iter()
                    .map(|it| match (it, it.target()) {
                        (MissionItem::Takeoff { alt_m, .. }, _) => [0.0, 0.0, *alt_m],
                        (_, Some((lat, lon, alt))) => {
                            let (n, e) = home.ne_to(&GeoPoint { lat, lon, alt_m: 0.0 });
                            [n, e, alt]
                        }
                        (_, None) => [0.0, 0.0, 0.0],
                    })
                    .collect()
            })
            .unwrap_or_default();
        let (test_running, test_elapsed_s) = match lock(&self.run).as_ref() {
            Some(r) => (true, Some(r.started.elapsed().as_secs_f64())),
            None => (false, None),
        };
        Snapshot {
            airframe,
            airframes: Airframe::ALL.iter().map(|a| (*a, a.label())).collect(),
            home,
            gui,
            mission,
            missions: Mission::BUILTIN.to_vec(),
            presets: PRESETS.to_vec(),
            modules,
            telemetry,
            report: self.report_tx.borrow().clone(),
            test_running,
            test_elapsed_s,
            track: lock(&self.track).clone(),
            waypoints,
        }
    }
}

/// `report` ended as failed with an extra failed check saying why.
fn failed(mut report: TestReport, started: Instant, why: String) -> TestReport {
    let at = started.elapsed().as_secs_f64();
    report.verdict = TestVerdict::Failed;
    report.elapsed_s = report.elapsed_s.max(at);
    report.checks.push(Check { name: "run".into(), state: CheckState::Failed, at_s: Some(at), detail: why });
    report
}
