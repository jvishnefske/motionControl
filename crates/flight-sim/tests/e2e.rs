//! End to end: SimAutopilot <-> MAVLink over UDP <-> flight_mav::VehicleLink.
//! Each test uses its own port pair so they can run in parallel.

use std::time::Duration;

use flight_core::{
    Airframe, Backend, GeoPoint, MavPorts, Mission, Module, ModuleFactory, ModuleKind,
    ModuleStatus, SimContext, TestVerdict, VehicleControl,
};
use flight_mav::{VehicleLink, run_mission_test};
use flight_sim::{Factory, SimAutopilot, SimGroundStation};
use tokio::sync::{broadcast, watch};

fn ctx(airframe: Airframe, home: GeoPoint, ports: MavPorts) -> SimContext {
    SimContext {
        airframe,
        home,
        ports,
        physics_backend: Backend::Simulated,
        gui: false,
        telemetry: watch::channel(None).1,
        log: broadcast::channel(256).0,
    }
}

async fn fly_box(airframe: Airframe, ports: MavPorts, time_scale: f64) {
    let home = GeoPoint::PX4_DEFAULT_HOME;
    let mut ap = SimAutopilot::new().with_time_scale(time_scale);
    ap.start(&ctx(airframe, home, ports)).await.unwrap();
    let link = VehicleLink::connect(ports).await.unwrap();
    let mission = Mission::builtin("box", airframe, home).unwrap();
    let (tx, rx) = watch::channel(None);
    let report = tokio::time::timeout(
        Duration::from_secs(120),
        run_mission_test(link.as_ref(), &mission, &tx),
    )
    .await
    .expect("mission test did not finish");
    eprintln!(
        "{airframe:?}: {:?} in {:.1} sim s (limit {:.0}), max alt error {:.1} m, checks at {:?}",
        report.verdict,
        report.elapsed_s,
        mission.timeout_s,
        report.max_alt_error_m,
        report
            .checks
            .iter()
            .map(|c| c.at_s.unwrap_or(-1.0))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        report.verdict,
        TestVerdict::Passed,
        "{report:#?}\nautopilot: {:?}",
        ap.status()
    );
    assert_eq!(rx.borrow().as_ref(), Some(&report));
    // Judged in sim seconds, well inside the mission's limit.
    assert!(
        report.elapsed_s > 20.0 && report.elapsed_s < mission.timeout_s,
        "{report:#?}"
    );
    if airframe.is_fixed_wing() {
        // The plane is left loitering home in RTL (the mode change may be
        // one heartbeat behind the position that passed the test).
        let mut tel = link.telemetry();
        tokio::time::timeout(
            Duration::from_secs(3),
            tel.wait_for(|t| t.is_some_and(|t| t.mode == flight_core::FlightMode::Rtl && t.armed)),
        )
        .await
        .expect("plane not in RTL")
        .unwrap();
    }
    ap.stop().await.unwrap();
    assert_eq!(ap.status(), ModuleStatus::Stopped);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn quadrotor_box_mission_passes() {
    fly_box(
        Airframe::Quadrotor,
        MavPorts {
            gcs: 24550,
            api: 24540,
        },
        20.0,
    )
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn fixed_wing_box_mission_passes() {
    fly_box(
        Airframe::FixedWing,
        MavPorts {
            gcs: 25550,
            api: 25540,
        },
        20.0,
    )
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn link_follows_autopilot_switch() {
    let ports = MavPorts {
        gcs: 26550,
        api: 26540,
    };
    let home_a = GeoPoint::PX4_DEFAULT_HOME;
    let home_b = home_a.offset(1000.0, 500.0);
    let link = VehicleLink::connect(ports).await.unwrap();

    let mut ap = SimAutopilot::new().with_time_scale(5.0);
    ap.start(&ctx(Airframe::Quadrotor, home_a, ports))
        .await
        .unwrap();
    link.wait_ready(Duration::from_secs(10)).await.unwrap();
    let first = link.sessions_seen();
    assert_eq!(link.telemetry().borrow().unwrap().home, Some(round(home_a)));
    ap.stop().await.unwrap();

    // A different vehicle (new airframe and home) on the same ports, as when
    // PX4 is swapped for the sim autopilot or the other way round.
    let mut ap2 = SimAutopilot::new().with_time_scale(5.0);
    ap2.start(&ctx(Airframe::FixedWing, home_b, ports))
        .await
        .unwrap();
    let mut rx = link.telemetry();
    tokio::time::timeout(Duration::from_secs(15), async {
        loop {
            let at_b = rx
                .borrow()
                .and_then(|t| t.home)
                .is_some_and(|h| h.horizontal_distance_m(&home_b) < 1.0);
            if at_b {
                break;
            }
            rx.changed().await.unwrap();
        }
    })
    .await
    .expect("link did not pick up the new vehicle");
    link.wait_ready(Duration::from_secs(10)).await.unwrap();
    assert!(link.sessions_seen() > first);
    assert_eq!(link.target(), Some((1, 1)));
    // And it can be commanded.
    let m = Mission::builtin("box", Airframe::FixedWing, home_b).unwrap();
    link.upload_mission(&m).await.unwrap();
    ap2.stop().await.unwrap();

    // With nothing on the port the link notices the loss.
    tokio::time::timeout(Duration::from_secs(10), async {
        while link.target().is_some() {
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("vehicle loss not detected");
}

fn round(p: GeoPoint) -> GeoPoint {
    // HOME_POSITION carries 1e-7 deg and mm.
    GeoPoint {
        lat: (p.lat * 1e7).round() / 1e7,
        lon: (p.lon * 1e7).round() / 1e7,
        alt_m: (p.alt_m * 1000.0).round() / 1000.0,
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn ground_station_and_autopilot_see_each_other() {
    let ports = MavPorts {
        gcs: 27550,
        api: 27540,
    };
    let c = ctx(Airframe::Quadrotor, GeoPoint::PX4_DEFAULT_HOME, ports);
    let mut gs = SimGroundStation::new();
    gs.start(&c).await.unwrap();
    let mut ap = SimAutopilot::new();
    ap.start(&c).await.unwrap();
    tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            let ap_sees_gcs =
                matches!(ap.status(), ModuleStatus::Running(s) if s.contains("GCS connected"));
            if gs.vehicle_heartbeats() >= 2 && ap_sees_gcs {
                break;
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .unwrap_or_else(|_| panic!("gcs: {:?}, autopilot: {:?}", gs.status(), ap.status()));
    assert!(matches!(gs.status(), ModuleStatus::Running(s) if s.contains("vehicle heartbeats")));
    gs.stop().await.unwrap();
    ap.stop().await.unwrap();
    // Ports are free again right after stop.
    gs.start(&c).await.unwrap();
    ap.start(&c).await.unwrap();
    gs.stop().await.unwrap();
    ap.stop().await.unwrap();
}

#[tokio::test]
async fn factory_offers_simulated_backends() {
    let f = Factory;
    for kind in [
        ModuleKind::Autopilot,
        ModuleKind::Physics,
        ModuleKind::GroundStation,
    ] {
        assert_eq!(f.available(kind), vec![(Backend::Simulated, Ok(()))]);
        let mut m = f.create(kind, Backend::Simulated).unwrap();
        assert_eq!(m.kind(), kind);
        assert_eq!(m.backend(), Backend::Simulated);
        assert_eq!(m.status(), ModuleStatus::Stopped);
        if kind == ModuleKind::Physics {
            let c = ctx(
                Airframe::Quadrotor,
                GeoPoint::PX4_DEFAULT_HOME,
                MavPorts {
                    gcs: 28550,
                    api: 28540,
                },
            );
            m.start(&c).await.unwrap();
            assert!(matches!(m.status(), ModuleStatus::Running(s) if s.contains("SIH")));
            m.stop().await.unwrap();
        }
        assert!(f.create(kind, Backend::Real).is_none());
    }
    assert!(f.available(ModuleKind::Middleware).is_empty());
}
