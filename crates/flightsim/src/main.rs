//! `flightsim`: the web menu for one-click mission tests over PX4,
//! Gazebo, QGroundControl and ~/src/middleware, each switchable at runtime
//! between the real program and its Rust stand-in.

use std::net::SocketAddr;
use std::sync::Arc;

use flight_core::{Airframe, Backend, ModuleFactory, ModuleKind, Module, MavPorts};
use flight_ui::{Orchestrator, runner_fn};

/// Merges the per-crate factories: for each kind, every backend any of
/// them offers; `create` asks each in turn.
struct Composite(Vec<Arc<dyn ModuleFactory>>);

impl ModuleFactory for Composite {
    fn available(&self, kind: ModuleKind) -> Vec<(Backend, Result<(), String>)> {
        let mut out: Vec<(Backend, Result<(), String>)> = Vec::new();
        for f in &self.0 {
            for (b, ok) in f.available(kind) {
                match out.iter_mut().find(|(x, _)| *x == b) {
                    Some(slot) if slot.1.is_err() && ok.is_ok() => slot.1 = ok,
                    Some(_) => {}
                    None => out.push((b, ok)),
                }
            }
        }
        out.sort_by_key(|(b, _)| *b as u8);
        out
    }

    fn create(&self, kind: ModuleKind, backend: Backend) -> Option<Box<dyn Module>> {
        self.0.iter().find_map(|f| f.create(kind, backend))
    }
}

fn usage() -> ! {
    eprintln!(
        "usage: flightsim [--listen ADDR] [--airframe quadrotor|fixed_wing] [--preset NAME] [--gui] [--run MISSION]\n\
         presets: {}\n\
         env: FLIGHT_PX4_DIR (native PX4 checkout), FLIGHT_PX4_IMAGE, FLIGHT_GZ_WORLD, FLIGHT_QGC",
        flight_ui::PRESETS.join(", ")
    );
    std::process::exit(2)
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut listen: SocketAddr = "127.0.0.1:8080".parse()?;
    let mut airframe = Airframe::Quadrotor;
    let mut preset: Option<String> = None;
    let mut run: Option<String> = None;
    let mut gui = false;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = || args.next().unwrap_or_else(|| usage());
        match a.as_str() {
            "--listen" => listen = val().parse()?,
            "--airframe" => airframe = serde_json::from_value(serde_json::Value::String(val())).unwrap_or_else(|_| usage()),
            "--preset" => preset = Some(val()),
            "--run" => run = Some(val()),
            "--gui" => gui = true,
            _ => usage(),
        }
    }

    let launch = flight_launch::Factory::from_env();
    let bus = Arc::new(flight_bus::Factory::new());
    let factory = Arc::new(Composite(vec![
        Arc::new(flight_sim::Factory),
        Arc::new(launch.clone()),
        bus.clone(),
    ]));

    let ports = MavPorts::default();
    let vehicle = flight_mav::VehicleLink::connect(ports).await?;
    let runner = runner_fn(|v, m, tx| async move { flight_mav::run_mission_test(&*v, &m, &tx).await });
    let hook_launch = launch.clone();
    let orch = Arc::new(
        Orchestrator::new(factory, vehicle, runner)
            .with_airframe(airframe)
            .with_gui(gui)
            .with_ports(ports)
            .with_backend_hook(Arc::new(move |k, b| {
                if k == ModuleKind::Autopilot {
                    hook_launch.set_autopilot_backend(b);
                }
            })),
    );
    orch.add_report_sink(bus.report_sender());

    if let Some(p) = preset {
        orch.apply_preset(&p).await?;
    }
    if let Some(m) = run {
        orch.run_mission_test(&m)?;
    }

    println!("flightsim menu: http://{listen}/");
    let server = flight_ui::serve(listen, orch.clone());
    tokio::select! {
        r = server => r?,
        _ = tokio::signal::ctrl_c() => {}
    }
    orch.shutdown().await;
    Ok(())
}
