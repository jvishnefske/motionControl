//! Launches real PX4 SITL (SIH physics, no Gazebo) if this machine has it:
//! `FLIGHT_PX4_DIR` or the `flight-px4-gz:latest` image.
//!
//!     cargo test -p flight-launch -- --ignored --nocapture

use std::time::Duration;

use flight_core::{
    Airframe, Backend, GeoPoint, MavPorts, ModuleFactory, ModuleKind, ModuleStatus, SimContext,
};
use flight_launch::Factory;
use tokio::sync::{broadcast, watch};

#[tokio::test]
#[ignore = "needs PX4 (FLIGHT_PX4_DIR or the flight-px4-gz image); binds udp 14540"]
async fn px4_sih_sends_mavlink_on_api_port() {
    let factory = Factory::from_env();
    let (_, avail) = &factory.available(ModuleKind::Autopilot)[0];
    if let Err(why) = avail {
        eprintln!("skipping: {why}");
        return;
    }
    let (log_tx, mut log_rx) = broadcast::channel(4096);
    let (_telem_tx, telem_rx) = watch::channel(None);
    let ctx = SimContext {
        airframe: Airframe::Quadrotor,
        home: GeoPoint::PX4_DEFAULT_HOME,
        ports: MavPorts::default(),
        physics_backend: Backend::Simulated,
        gui: false,
        telemetry: telem_rx,
        log: log_tx,
    };
    tokio::spawn(async move {
        while let Ok(l) = log_rx.recv().await {
            eprintln!("[{}] {}", l.source, l.text);
        }
    });

    // PX4's API link sends to 14540 once it has started.
    let sock = tokio::net::UdpSocket::bind("0.0.0.0:14540")
        .await
        .expect("bind udp 14540");
    let mut px4 = factory
        .create(ModuleKind::Autopilot, Backend::Real)
        .unwrap();
    px4.start(&ctx).await.expect("PX4 starts");
    assert!(
        matches!(px4.status(), ModuleStatus::Running(_)),
        "{:?}",
        px4.status()
    );

    let mut buf = [0u8; 512];
    let (n, from) = tokio::time::timeout(Duration::from_secs(20), sock.recv_from(&mut buf))
        .await
        .expect("MAVLink within 20 s")
        .unwrap();
    // MAVLink v2 (0xFD) or v1 (0xFE) frame start.
    assert!(
        n > 0 && (buf[0] == 0xFD || buf[0] == 0xFE),
        "{n} bytes from {from}: {:02x?}",
        &buf[..n.min(16)]
    );

    px4.stop().await.unwrap();
    assert_eq!(px4.status(), ModuleStatus::Stopped);
    px4.stop().await.unwrap();
}
