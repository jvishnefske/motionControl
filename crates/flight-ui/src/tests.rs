use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use axum::body::Body;
use axum::http::{Request, StatusCode, header};
use flight_core::{
    Backend, CheckState, GeoPoint, Mission, MissionTest, Module, ModuleFactory, ModuleKind, ModuleStatus,
    SimContext, Telemetry, TestReport, TestVerdict, VehicleControl,
};
use tokio::sync::{Notify, watch};
use tower::ServiceExt;

use crate::{Orchestrator, router, runner_fn};

type Events = Arc<Mutex<Vec<String>>>;

fn tag(kind: ModuleKind, backend: Backend) -> String {
    format!("{kind:?}/{backend:?}")
}

struct FakeModule {
    kind: ModuleKind,
    backend: Backend,
    name: String,
    events: Events,
    running: bool,
}

#[async_trait]
impl Module for FakeModule {
    fn kind(&self) -> ModuleKind {
        self.kind
    }
    fn backend(&self) -> Backend {
        self.backend
    }
    fn name(&self) -> &str {
        &self.name
    }
    async fn start(&mut self, ctx: &SimContext) -> anyhow::Result<()> {
        tokio::task::yield_now().await;
        let extra = if self.kind == ModuleKind::Autopilot { format!(" phys={:?}", ctx.physics_backend) } else { String::new() };
        self.events.lock().unwrap().push(format!("start {}{extra}", tag(self.kind, self.backend)));
        self.running = true;
        Ok(())
    }
    async fn stop(&mut self) -> anyhow::Result<()> {
        if self.running {
            self.events.lock().unwrap().push(format!("stop {}", tag(self.kind, self.backend)));
        }
        self.running = false;
        Ok(())
    }
    fn status(&self) -> ModuleStatus {
        if self.running { ModuleStatus::Running("ok".into()) } else { ModuleStatus::Stopped }
    }
}

#[derive(Default)]
struct FakeFactory {
    events: Events,
    /// (kind, backend) reported unavailable.
    unavailable: Vec<(ModuleKind, Backend)>,
}

impl ModuleFactory for FakeFactory {
    fn available(&self, kind: ModuleKind) -> Vec<(Backend, Result<(), String>)> {
        [Backend::Real, Backend::Simulated]
            .into_iter()
            .map(|b| (b, if self.unavailable.contains(&(kind, b)) { Err("not installed".into()) } else { Ok(()) }))
            .collect()
    }
    fn create(&self, kind: ModuleKind, backend: Backend) -> Option<Box<dyn Module>> {
        Some(Box::new(FakeModule {
            kind,
            backend,
            name: format!("fake-{kind:?}-{backend:?}").to_lowercase(),
            events: self.events.clone(),
            running: false,
        }))
    }
}

struct FakeVehicle {
    tel: watch::Sender<Option<Telemetry>>,
    rtl: AtomicUsize,
}

impl FakeVehicle {
    fn new() -> Arc<Self> {
        Arc::new(FakeVehicle { tel: watch::channel(None).0, rtl: AtomicUsize::new(0) })
    }
}

#[async_trait]
impl VehicleControl for FakeVehicle {
    fn telemetry(&self) -> watch::Receiver<Option<Telemetry>> {
        self.tel.subscribe()
    }
    async fn wait_ready(&self, _: Duration) -> anyhow::Result<()> {
        Ok(())
    }
    async fn upload_mission(&self, _: &Mission) -> anyhow::Result<()> {
        Ok(())
    }
    async fn arm(&self) -> anyhow::Result<()> {
        Ok(())
    }
    async fn start_mission(&self) -> anyhow::Result<()> {
        Ok(())
    }
    async fn return_to_launch(&self) -> anyhow::Result<()> {
        self.rtl.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
    async fn land(&self) -> anyhow::Result<()> {
        Ok(())
    }
}

/// A runner that publishes one intermediate report, then waits for `go`.
fn gated_runner(go: Arc<Notify>) -> crate::Runner {
    runner_fn(move |_v, m, tx| {
        let go = go.clone();
        async move {
            let mut r = MissionTest::new(&m).report().clone();
            r.checks[0].state = CheckState::Passed;
            tx.send_replace(Some(r.clone()));
            go.notified().await;
            r.verdict = TestVerdict::Passed;
            r
        }
    })
}

struct Rig {
    orch: Arc<Orchestrator>,
    events: Events,
    vehicle: Arc<FakeVehicle>,
    go: Arc<Notify>,
    hook: Arc<Mutex<Vec<(ModuleKind, Backend)>>>,
}

fn rig_with(unavailable: Vec<(ModuleKind, Backend)>) -> Rig {
    let events = Events::default();
    let factory = Arc::new(FakeFactory { events: events.clone(), unavailable });
    let vehicle = FakeVehicle::new();
    let go = Arc::new(Notify::new());
    let hook = Arc::new(Mutex::new(Vec::new()));
    let h = hook.clone();
    let orch = Orchestrator::new(factory, vehicle.clone(), gated_runner(go.clone()))
        .with_backend_hook(Arc::new(move |k, b| h.lock().unwrap().push((k, b))));
    Rig { orch: Arc::new(orch), events, vehicle, go, hook }
}

fn rig() -> Rig {
    rig_with(Vec::new())
}

fn take(e: &Events) -> Vec<String> {
    std::mem::take(&mut *e.lock().unwrap())
}

async fn wait_until(mut f: impl FnMut() -> bool) {
    for _ in 0..500 {
        if f() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    panic!("condition not reached");
}

#[tokio::test]
async fn set_backend_stops_old_before_starting_new() {
    let r = rig();
    let o = &r.orch;
    o.set_backend(ModuleKind::Middleware, Backend::Simulated).await.unwrap();
    o.set_backend(ModuleKind::Autopilot, Backend::Simulated).await.unwrap();
    assert_eq!(take(&r.events), ["start Middleware/Simulated", "start Autopilot/Simulated phys=Off"]);

    o.set_backend(ModuleKind::Autopilot, Backend::Real).await.unwrap();
    assert_eq!(take(&r.events), ["stop Autopilot/Simulated", "start Autopilot/Real phys=Off"]);

    // Physics change restarts the autopilot so it sees the new physics.
    o.set_backend(ModuleKind::Physics, Backend::Real).await.unwrap();
    assert_eq!(
        take(&r.events),
        ["stop Autopilot/Real", "start Physics/Real", "start Autopilot/Real phys=Real"]
    );

    // Middleware switch leaves the rest alone.
    o.set_backend(ModuleKind::Middleware, Backend::Real).await.unwrap();
    assert_eq!(take(&r.events), ["stop Middleware/Simulated", "start Middleware/Real"]);

    // Off stops.
    o.set_backend(ModuleKind::Middleware, Backend::Off).await.unwrap();
    assert_eq!(take(&r.events), ["stop Middleware/Real"]);
    assert_eq!(o.backend(ModuleKind::Middleware), Backend::Off);

    let hook = r.hook.lock().unwrap().clone();
    assert!(hook.contains(&(ModuleKind::Autopilot, Backend::Real)));
    assert_eq!(hook.last(), Some(&(ModuleKind::Middleware, Backend::Off)));

    let snap = o.snapshot();
    let ap = snap.modules.iter().find(|m| m.kind == ModuleKind::Autopilot).unwrap();
    assert_eq!(ap.backend, Backend::Real);
    assert_eq!(ap.status, ModuleStatus::Running("ok".into()));
    assert_eq!(ap.options.len(), 3);

    o.shutdown().await;
    assert_eq!(take(&r.events), ["stop Autopilot/Real", "stop Physics/Real"]);
}

#[tokio::test]
async fn unavailable_backend_is_refused() {
    let r = rig_with(vec![(ModuleKind::GroundStation, Backend::Real)]);
    let e = r.orch.set_backend(ModuleKind::GroundStation, Backend::Real).await.unwrap_err();
    assert!(e.to_string().contains("not installed"), "{e}");
    let e = r.orch.apply_preset("full-stack").await.unwrap_err();
    assert!(e.to_string().contains("not installed"), "{e}");
    assert!(take(&r.events).is_empty());
    assert_eq!(r.orch.backend(ModuleKind::Autopilot), Backend::Off);
}

#[tokio::test]
async fn presets_switch_only_what_changes() {
    let r = rig();
    let o = &r.orch;
    o.apply_preset("all-sim").await.unwrap();
    assert_eq!(
        take(&r.events),
        [
            "start Middleware/Simulated",
            "start Physics/Simulated",
            "start Autopilot/Simulated phys=Simulated",
            "start GroundStation/Simulated"
        ]
    );
    o.apply_preset("sim+qgc").await.unwrap();
    assert_eq!(take(&r.events), ["stop GroundStation/Simulated", "start GroundStation/Real"]);

    o.apply_preset("px4-sih").await.unwrap();
    assert_eq!(
        take(&r.events),
        [
            "stop GroundStation/Real",
            "stop Autopilot/Simulated",
            "start Autopilot/Real phys=Simulated",
            "start GroundStation/Simulated"
        ]
    );

    o.apply_preset("full-stack").await.unwrap();
    let ev = take(&r.events);
    // Everything stopped (reverse order) before anything started (start order).
    assert_eq!(
        ev,
        [
            "stop GroundStation/Simulated",
            "stop Autopilot/Real",
            "stop Physics/Simulated",
            "stop Middleware/Simulated",
            "start Middleware/Real",
            "start Physics/Real",
            "start Autopilot/Real phys=Real",
            "start GroundStation/Real"
        ]
    );
    for k in ModuleKind::ALL {
        assert_eq!(o.backend(k), Backend::Real);
    }
    assert!(o.apply_preset("nope").await.is_err());
}

#[tokio::test]
async fn set_airframe_restarts_vehicle_modules() {
    let r = rig();
    r.orch.apply_preset("all-sim").await.unwrap();
    take(&r.events);
    r.orch.set_airframe(flight_core::Airframe::FixedWing).await.unwrap();
    assert_eq!(
        take(&r.events),
        [
            "stop Autopilot/Simulated",
            "stop Physics/Simulated",
            "stop Middleware/Simulated",
            "start Middleware/Simulated",
            "start Physics/Simulated",
            "start Autopilot/Simulated phys=Simulated"
        ]
    );
    assert_eq!(r.orch.snapshot().airframe, flight_core::Airframe::FixedWing);
}

#[tokio::test]
async fn mission_test_starts_modules_and_refuses_a_second_run() {
    let r = rig();
    let o = &r.orch;
    let (sink, mut sink_rx) = watch::channel::<Option<TestReport>>(None);
    o.add_report_sink(sink);

    assert!(o.run_mission_test("nope").is_err());
    o.run_mission_test("box").unwrap();
    let e = o.run_mission_test("survey").unwrap_err();
    assert!(e.to_string().contains("already running"), "{e}");
    assert!(o.test_running());

    // Autopilot and physics were off: started as Simulated, physics first.
    wait_until(|| r.events.lock().unwrap().len() >= 2).await;
    assert_eq!(take(&r.events), ["start Physics/Simulated", "start Autopilot/Simulated phys=Simulated"]);
    assert_eq!(o.backend(ModuleKind::Physics), Backend::Simulated);

    // The intermediate report streams to the orchestrator and the sink.
    wait_until(|| o.reports().borrow().as_ref().is_some_and(|r| r.checks[0].state == CheckState::Passed)).await;
    assert_eq!(sink_rx.borrow_and_update().as_ref().unwrap().verdict, TestVerdict::Running);
    let snap = o.snapshot();
    assert!(snap.test_running);
    assert!(!snap.waypoints.is_empty());

    // Switching middleware mid-test is allowed; the airframe is not.
    o.set_backend(ModuleKind::Middleware, Backend::Simulated).await.unwrap();
    assert!(o.set_airframe(flight_core::Airframe::FixedWing).await.is_err());

    r.go.notify_one();
    wait_until(|| !o.test_running()).await;
    assert_eq!(o.reports().borrow().as_ref().unwrap().verdict, TestVerdict::Passed);
    assert_eq!(sink_rx.borrow().as_ref().unwrap().verdict, TestVerdict::Passed);

    // A new run is accepted after the first one ended, without restarts.
    take(&r.events);
    o.run_mission_test("survey").unwrap();
    wait_until(|| o.reports().borrow().as_ref().is_some_and(|r| r.mission == "survey")).await;
    r.go.notify_one();
    wait_until(|| !o.test_running()).await;
    assert!(take(&r.events).is_empty());
}

#[tokio::test]
async fn abort_commands_rtl_and_fails_the_test() {
    let r = rig();
    r.orch.run_mission_test("box").unwrap();
    wait_until(|| r.orch.reports().borrow().as_ref().is_some_and(|r| r.checks[0].state == CheckState::Passed)).await;
    r.orch.abort_test().await.unwrap();
    assert_eq!(r.vehicle.rtl.load(Ordering::SeqCst), 1);
    wait_until(|| !r.orch.test_running()).await;
    let rep = r.orch.reports().borrow().clone().unwrap();
    assert_eq!(rep.verdict, TestVerdict::Failed);
    assert!(rep.checks.last().unwrap().detail.contains("aborted"));
}

#[tokio::test]
async fn snapshot_tracks_the_vehicle() {
    let r = rig();
    let home = GeoPoint::PX4_DEFAULT_HOME;
    let mut t = Telemetry {
        time_s: 0.0,
        position: home,
        rel_alt_m: 0.0,
        vel_ned: [0.0; 3],
        attitude: [0.0; 3],
        armed: false,
        mode: flight_core::FlightMode::Hold,
        landed: true,
        mission_seq: 0,
        mission_reached: None,
        battery_pct: 100.0,
        home: Some(home),
    };
    r.vehicle.tel.send_replace(Some(t));
    assert_eq!(r.orch.snapshot().track.len(), 1);
    t.position = home.offset(10.0, 5.0);
    r.vehicle.tel.send_replace(Some(t));
    let s = r.orch.snapshot();
    assert_eq!(s.track.len(), 2);
    assert!((s.track[1][0] - 10.0).abs() < 0.01 && (s.track[1][1] - 5.0).abs() < 0.01);
}

async fn call(o: &Arc<Orchestrator>, method: &str, uri: &str, body: Option<serde_json::Value>) -> (StatusCode, serde_json::Value) {
    let mut req = Request::builder().method(method).uri(uri);
    let body = match body {
        Some(b) => {
            req = req.header(header::CONTENT_TYPE, "application/json");
            Body::from(b.to_string())
        }
        None => Body::empty(),
    };
    let resp = router(o.clone()).oneshot(req.body(body).unwrap()).await.unwrap();
    let status = resp.status();
    let bytes = axum::body::to_bytes(resp.into_body(), 1 << 20).await.unwrap();
    (status, serde_json::from_slice(&bytes).unwrap_or(serde_json::Value::Null))
}

#[tokio::test]
async fn http_routes() {
    let r = rig();
    let o = &r.orch;

    let resp = router(o.clone()).oneshot(Request::get("/").body(Body::empty()).unwrap()).await.unwrap();
    assert_eq!(resp.status(), StatusCode::OK);
    let html = axum::body::to_bytes(resp.into_body(), 1 << 20).await.unwrap();
    assert!(std::str::from_utf8(&html).unwrap().contains("Run mission test"));

    let (s, j) = call(o, "GET", "/api/state", None).await;
    assert_eq!(s, StatusCode::OK);
    assert_eq!(j["modules"].as_array().unwrap().len(), 4);
    assert_eq!(j["modules"][0]["kind"], "middleware");
    assert_eq!(j["airframe"], "quadrotor");

    let (s, j) =
        call(o, "POST", "/api/backend", Some(serde_json::json!({"kind":"autopilot","backend":"simulated"}))).await;
    assert_eq!((s, j["ok"].as_bool()), (StatusCode::OK, Some(true)));
    assert_eq!(take(&r.events), ["start Autopilot/Simulated phys=Off"]);

    let (s, _) = call(o, "POST", "/api/backend", Some(serde_json::json!({"kind":"autopilot","backend":"warp"}))).await;
    assert!(s.is_client_error());

    let (s, j) = call(o, "POST", "/api/preset", Some(serde_json::json!({"name":"bogus"}))).await;
    assert_eq!((s, j["ok"].as_bool()), (StatusCode::BAD_REQUEST, Some(false)));
    let (s, _) = call(o, "POST", "/api/preset", Some(serde_json::json!({"name":"all-sim"}))).await;
    assert_eq!(s, StatusCode::OK);

    let (s, _) = call(o, "POST", "/api/mission", Some(serde_json::json!({"name":"survey"}))).await;
    assert_eq!(s, StatusCode::OK);
    assert_eq!(o.selected_mission(), "survey");
    let (s, _) = call(o, "POST", "/api/gui", Some(serde_json::json!({"gui":true}))).await;
    assert_eq!(s, StatusCode::OK);
    assert!(o.snapshot().gui);

    let (s, _) = call(o, "POST", "/api/test/run", Some(serde_json::json!({}))).await;
    assert_eq!(s, StatusCode::OK);
    let (s, j) = call(o, "POST", "/api/test/run", Some(serde_json::json!({"mission":"box"}))).await;
    assert_eq!(s, StatusCode::CONFLICT);
    assert!(j["error"].as_str().unwrap().contains("already running"));
    let (s, _) = call(o, "POST", "/api/airframe", Some(serde_json::json!({"airframe":"fixed_wing"}))).await;
    assert_eq!(s, StatusCode::CONFLICT);

    let (s, _) = call(o, "POST", "/api/test/abort", Some(serde_json::json!({}))).await;
    assert_eq!(s, StatusCode::OK);
    assert_eq!(r.vehicle.rtl.load(Ordering::SeqCst), 1);
    wait_until(|| !o.test_running()).await;

    // SSE: the content type, and the first frame is a state event.
    let resp = router(o.clone()).oneshot(Request::get("/api/events").body(Body::empty()).unwrap()).await.unwrap();
    assert_eq!(resp.status(), StatusCode::OK);
    assert!(resp.headers()[header::CONTENT_TYPE].to_str().unwrap().starts_with("text/event-stream"));
    let mut body = resp.into_body().into_data_stream();
    let mut text = String::new();
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    while !text.contains("event: state") {
        let chunk = tokio::time::timeout_at(deadline, futures_next(&mut body)).await.expect("sse frame").expect("chunk");
        text.push_str(std::str::from_utf8(&chunk).unwrap());
    }
    assert!(text.contains("\"modules\""));
}

async fn futures_next(s: &mut axum::body::BodyDataStream) -> Option<axum::body::Bytes> {
    use tokio_stream::StreamExt;
    s.next().await.map(|r| r.unwrap())
}
