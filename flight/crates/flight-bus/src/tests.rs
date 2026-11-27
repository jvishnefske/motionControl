use super::*;
use flight_core::{Check, CheckState, FlightMode, GeoPoint, MavPorts, TestVerdict};
use tokio::sync::broadcast;

fn telemetry(t: f64) -> Telemetry {
    Telemetry {
        time_s: t,
        position: GeoPoint::PX4_DEFAULT_HOME,
        rel_alt_m: 10.0,
        vel_ned: [1.0, 2.0, 0.0],
        attitude: [0.0; 3],
        armed: true,
        mode: FlightMode::Mission,
        landed: false,
        mission_seq: 1,
        mission_reached: Some(0),
        battery_pct: 90.0,
        home: Some(GeoPoint::PX4_DEFAULT_HOME),
    }
}

fn report() -> TestReport {
    TestReport {
        mission: "box".into(),
        verdict: TestVerdict::Running,
        elapsed_s: 1.5,
        checks: vec![Check {
            name: "armed".into(),
            state: CheckState::Passed,
            at_s: Some(0.5),
            detail: String::new(),
        }],
        max_alt_error_m: 0.0,
    }
}

fn ctx(tel: watch::Receiver<Option<Telemetry>>) -> SimContext {
    SimContext {
        airframe: Airframe::Quadrotor,
        home: GeoPoint::PX4_DEFAULT_HOME,
        ports: MavPorts::default(),
        physics_backend: Backend::Simulated,
        gui: false,
        telemetry: tel,
        log: broadcast::channel(64).0,
    }
}

async fn wait_until(mut f: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(10);
    while !f() {
        assert!(Instant::now() < deadline, "condition not reached in time");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

#[test]
fn keys_and_filter() {
    assert_eq!(
        telemetry_key(Airframe::FixedWing).as_str(),
        "flight/fixed_wing/telemetry"
    );
    assert_eq!(
        report_key(Airframe::Quadrotor).as_str(),
        "flight/quadrotor/mission/report"
    );
    assert!(flight_filter().matches(cmd_key(Airframe::Quadrotor)));
    assert!(!flight_filter().matches(Key::from_static("flightx/a")));
    let f = Factory::new();
    assert!(f.create(ModuleKind::Middleware, Backend::Real).is_some());
    assert!(
        f.create(ModuleKind::Middleware, Backend::Simulated)
            .is_some()
    );
    assert!(f.create(ModuleKind::Autopilot, Backend::Real).is_none());
    assert_eq!(f.available(ModuleKind::Middleware).len(), 2);
}

#[tokio::test(flavor = "multi_thread")]
async fn memory_bus_round_trip_counts_and_answers_status() {
    let (tel_tx, tel_rx) = watch::channel(Some(telemetry(0.0)));
    let factory = Factory::new();
    let mut bus = MemoryBus::new(factory.report_sender().subscribe());
    assert_eq!(bus.status(), ModuleStatus::Stopped);
    bus.start(&ctx(tel_rx)).await.unwrap();

    wait_until(|| bus.stats().unwrap().received >= 3).await;
    let s = bus.stats().unwrap();
    assert_eq!(s.last_key.as_deref(), Some("flight/quadrotor/telemetry"));
    assert!(matches!(bus.status(), ModuleStatus::Running(d) if d.contains("rx")));

    // A report pushed through the factory shows up on the bus.
    factory.push_report(report());
    wait_until(|| {
        bus.stats().unwrap().last_key.as_deref() == Some("flight/quadrotor/mission/report")
    })
    .await;
    tel_tx.send_replace(Some(telemetry(1.0)));

    // Request/response through the hub.
    let reply = bus.command("status").await.unwrap();
    let v: serde_json::Value = serde_json::from_str(&reply).unwrap();
    assert_eq!(v["transport"], "memory");
    assert!(v["received"].as_u64().unwrap() >= 3);
    assert!(bus.command("bogus").await.is_err());

    bus.stop().await.unwrap();
    assert_eq!(bus.status(), ModuleStatus::Stopped);
    assert!(bus.command("status").await.is_err());
    bus.stop().await.unwrap(); // idempotent
    // Restartable.
    let (_t2, rx2) = watch::channel(Some(telemetry(0.0)));
    bus.start(&ctx(rx2)).await.unwrap();
    wait_until(|| bus.stats().unwrap().received >= 1).await;
    bus.stop().await.unwrap();
}

async fn free_port() -> u16 {
    let l = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    l.local_addr().unwrap().port()
}

#[tokio::test(flavor = "multi_thread")]
async fn zenoh_bus_publishes_to_a_second_session() {
    let endpoint = format!("tcp/127.0.0.1:{}", free_port().await);
    let (_tel_tx, tel_rx) = watch::channel(Some(telemetry(3.0)));
    let factory = Factory::new().with_zenoh_config(ZenohConfig::loopback(endpoint.clone()));
    let mut bus = factory
        .create(ModuleKind::Middleware, Backend::Real)
        .unwrap();
    bus.start(&ctx(tel_rx)).await.unwrap();

    let peer = SessionConfig::new().connect(endpoint).build().unwrap();
    let mut peer = ZenohTransport::open(peer)
        .await
        .unwrap()
        .with_timeout(Duration::from_secs(3));
    let mut sub = peer
        .subscriber(telemetry_key(Airframe::Quadrotor))
        .await
        .unwrap();
    let mut buf = vec![0u8; 4096];
    let sample = tokio::time::timeout(Duration::from_secs(10), sub.recv(&mut buf))
        .await
        .unwrap()
        .unwrap();
    let got: Telemetry = serde_json::from_slice(sample.payload).unwrap();
    assert_eq!(got, telemetry(3.0));

    // The peer publishes too; the bus counts it (local delivery also counts its own).
    peer.publish(Key::from_static("flight/test/ping"), b"{}")
        .await
        .unwrap();
    wait_until(|| matches!(bus.status(), ModuleStatus::Running(d) if d.contains("rx") && !d.contains("rx 0 "))).await;

    // Status over the cmd endpoint, retried while the queryable propagates.
    let mut out = vec![0u8; SCRATCH];
    let mut reply = None;
    for _ in 0..50 {
        if let Ok(b) = peer
            .request(cmd_key(Airframe::Quadrotor), b"status", &mut out)
            .await
        {
            reply = Some(b.to_vec());
            break;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    let v: serde_json::Value = serde_json::from_slice(&reply.expect("status reply")).unwrap();
    assert_eq!(v["transport"], "zenoh");
    assert!(v["published"].as_u64().unwrap() >= 1);

    bus.stop().await.unwrap();
    assert_eq!(bus.status(), ModuleStatus::Stopped);
    bus.stop().await.unwrap();
}
