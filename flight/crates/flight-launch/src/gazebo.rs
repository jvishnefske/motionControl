//! Gazebo Harmonic (`gz sim`), headless server plus optional GUI.

use std::ffi::OsString;
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::Duration;

use async_trait::async_trait;
use flight_core::{
    Airframe, Backend, GeoPoint, LogLevel, Module, ModuleKind, ModuleStatus, SimContext, Telemetry,
};
use tokio::sync::watch;
use tokio::task::JoinHandle;

use crate::config::{LaunchConfig, Runner, which_in};
use crate::env::{self, GZ_CONTAINER, GZ_GUI_CONTAINER, GZ_IP};
use crate::proc::{Logger, Proc, SharedStatus, Spec, get_status, new_status, run_once, set_status};
use crate::px4::docker_run_args_for;

const WORLD_TIMEOUT: Duration = Duration::from_secs(90);
const FOLLOW_PERIOD: Duration = Duration::from_millis(100);

/// The autopilot backend currently selected, shared between the
/// [`Factory`](crate::Factory) (which the orchestrator tells through
/// `Factory::set_autopilot_backend`) and every [`GazeboSim`] it created.
/// `SimContext` only carries the physics backend, so this is how Gazebo
/// knows whether PX4 will spawn and drive the model (Real) or whether it
/// is a visualizer of `ctx.telemetry` (Simulated / Off). It is read live:
/// switching the autopilot while Gazebo runs removes or re-spawns the
/// visualizer model.
#[derive(Debug, Clone)]
pub struct AutopilotBackend(Arc<AtomicU8>);

fn encode(b: Backend) -> u8 {
    match b {
        Backend::Real => 0,
        Backend::Simulated => 1,
        Backend::Off => 2,
    }
}

fn decode(v: u8) -> Backend {
    match v {
        0 => Backend::Real,
        1 => Backend::Simulated,
        _ => Backend::Off,
    }
}

impl AutopilotBackend {
    pub fn new(b: Backend) -> Self {
        AutopilotBackend(Arc::new(AtomicU8::new(encode(b))))
    }
    pub fn get(&self) -> Backend {
        decode(self.0.load(Ordering::SeqCst))
    }
    pub fn set(&self, b: Backend) {
        self.0.store(encode(b), Ordering::SeqCst);
    }
}

impl Default for AutopilotBackend {
    /// PX4 real: the common case, and Gazebo then never spawns a model of its own.
    fn default() -> Self {
        AutopilotBackend::new(Backend::Real)
    }
}

/// How to run a `gz` CLI command against the running server.
#[derive(Clone)]
struct GzCli {
    program: OsString,
    prefix: Vec<String>,
    env: Vec<(String, String)>,
}

impl GzCli {
    fn new(runner: &Runner) -> Self {
        match runner {
            Runner::Native(dir) => GzCli {
                program: "gz".into(),
                prefix: vec![],
                env: native_gz_env(dir),
            },
            Runner::Docker(_) => GzCli {
                program: "docker".into(),
                prefix: vec![
                    "exec".into(),
                    GZ_CONTAINER.into(),
                    "flight-entrypoint".into(),
                    "gz".into(),
                ],
                env: vec![],
            },
        }
    }

    async fn run(&self, args: Vec<String>, timeout: Duration) -> Result<String, String> {
        let mut all = self.prefix.clone();
        all.extend(args);
        run_once(&self.program, &all, &self.env, timeout).await
    }
}

fn native_gz_env(dir: &std::path::Path) -> Vec<(String, String)> {
    let rp = std::env::var("GZ_SIM_RESOURCE_PATH").ok();
    let pp = std::env::var("GZ_SIM_SYSTEM_PLUGIN_PATH").ok();
    env::gz_env(dir, rp.as_deref(), pp.as_deref())
}

/// `gz sim -r -s <PX4 world>` (plus `gz sim -g` with `ctx.gui`), with
/// PX4's models, worlds, plugins and server config on the gz paths so PX4
/// standalone mode can spawn into it.
///
/// If the autopilot is not PX4 ([`AutopilotBackend`] not Real), Gazebo is a
/// visualizer: it spawns a static copy of the airframe's model and moves it
/// to `ctx.telemetry` at up to 10 Hz with `/world/<w>/set_pose`, best
/// effort (each update is one `gz service` call; failures are logged, not
/// fatal).
pub struct GazeboSim {
    cfg: LaunchConfig,
    autopilot: AutopilotBackend,
    status: SharedStatus,
    server: Option<Proc>,
    gui: Option<Proc>,
    follower: Option<JoinHandle<()>>,
}

impl GazeboSim {
    pub fn new(cfg: LaunchConfig, autopilot: AutopilotBackend) -> Self {
        GazeboSim {
            cfg,
            autopilot,
            status: new_status(),
            server: None,
            gui: None,
            follower: None,
        }
    }

    /// With a fixed autopilot backend.
    pub fn with_autopilot(cfg: LaunchConfig, autopilot: Backend) -> Self {
        Self::new(cfg, AutopilotBackend::new(autopilot))
    }

    fn fail(&self, msg: String) -> anyhow::Error {
        set_status(&self.status, ModuleStatus::Failed(msg.clone()));
        anyhow::anyhow!(msg)
    }

    async fn start_inner(&mut self, ctx: &SimContext, log: &Logger) -> anyhow::Result<String> {
        let runner = self.cfg.runner().map_err(|e| self.fail(e))?;
        let world = self.cfg.world.clone();
        let sdf = env::world_sdf(&runner.px4_dir(), &world);
        let server_args = env::gz_server_args(&sdf.display().to_string());
        let gz_env = [("GZ_IP".to_string(), GZ_IP.to_string())];
        let (server_spec, gui_spec) = match &runner {
            Runner::Native(dir) => {
                if which_in("gz", std::env::var_os("PATH").as_ref()).is_none() {
                    return Err(self.fail(
                        "gz not on PATH: install Gazebo Harmonic, or unset FLIGHT_PX4_DIR to use the docker image".into(),
                    ));
                }
                if !sdf.is_file() {
                    return Err(self.fail(format!(
                        "world {} not found (git submodule update --init Tools/simulation/gz in {})",
                        sdf.display(),
                        dir.display()
                    )));
                }
                let env = native_gz_env(dir);
                let mut s = Spec::new("gz", server_args);
                s.env = env.clone();
                let mut g = Spec::new("gz", env::gz_gui_args());
                g.env = env;
                (s, g)
            }
            Runner::Docker(image) => {
                let mut cmd = vec!["gz".to_string()];
                cmd.extend(server_args);
                let mut s = Spec::new(
                    "docker",
                    docker_run_args_for(GZ_CONTAINER, image, &gz_env, false, &cmd),
                );
                s.container = Some(GZ_CONTAINER.into());
                let mut gcmd = vec!["gz".to_string()];
                gcmd.extend(env::gz_gui_args());
                let mut g = Spec::new(
                    "docker",
                    docker_run_args_for(GZ_GUI_CONTAINER, image, &gz_env, true, &gcmd),
                );
                g.container = Some(GZ_GUI_CONTAINER.into());
                (s, g)
            }
        };

        self.server = Some(
            Proc::spawn(
                server_spec,
                "gz sim server",
                log.clone(),
                self.status.clone(),
            )
            .await?,
        );
        let cli = GzCli::new(&runner);
        self.wait_world(&cli, &world)
            .await
            .map_err(|e| self.fail(format!("Gazebo did not start: {e}")))?;
        log.log(LogLevel::Info, format!("Gazebo world {world} is up"));

        if ctx.gui {
            let mut g = gui_spec;
            // Closing the GUI window is not a physics failure.
            g.fail_module_on_exit = false;
            match Proc::spawn(g, "gz sim GUI", log.clone(), self.status.clone()).await {
                Ok(p) => self.gui = Some(p),
                Err(e) => log.log(LogLevel::Warn, format!("Gazebo GUI: {e}")),
            }
        }

        let follower = Follower {
            cli,
            world: world.clone(),
            airframe: ctx.airframe,
            home: ctx.home,
            telemetry: ctx.telemetry.clone(),
            autopilot: self.autopilot.clone(),
            log: log.clone(),
        };
        self.follower = Some(tokio::spawn(follower.run()));
        let role = if self.autopilot.get() == Backend::Real {
            "physics for PX4"
        } else {
            "visualizer"
        };
        Ok(format!("{}, world {world}, {role}", runner.describe()))
    }

    async fn wait_world(&self, cli: &GzCli, world: &str) -> Result<(), String> {
        let deadline = tokio::time::Instant::now() + WORLD_TIMEOUT;
        loop {
            if let Some(ex) = self.server.as_ref().and_then(Proc::exited) {
                return Err(format!("gz sim exited ({ex})"));
            }
            if let Ok(out) = cli
                .run(env::scene_info_args(world), Duration::from_secs(10))
                .await
                && out.contains("Service providers")
            {
                return Ok(());
            }
            if tokio::time::Instant::now() > deadline {
                return Err(format!(
                    "/world/{world}/scene/info not served after {} s",
                    WORLD_TIMEOUT.as_secs()
                ));
            }
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
    }
}

struct Follower {
    cli: GzCli,
    world: String,
    airframe: Airframe,
    home: GeoPoint,
    telemetry: watch::Receiver<Option<Telemetry>>,
    autopilot: AutopilotBackend,
    log: Logger,
}

impl Follower {
    async fn run(self) {
        let model = env::viz_model_name(self.airframe);
        let mut spawned = false;
        let mut origin_set = false;
        let mut failures = 0u32;
        let mut last: Option<Telemetry> = None;
        let mut tick = tokio::time::interval(FOLLOW_PERIOD);
        tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let svc = Duration::from_secs(6);
        loop {
            tick.tick().await;
            if self.autopilot.get() == Backend::Real {
                if spawned {
                    let _ = self
                        .cli
                        .run(env::remove_model_args(&self.world, &model), svc)
                        .await;
                    self.log.log(
                        LogLevel::Info,
                        format!("PX4 is the autopilot: removed visualizer model {model}"),
                    );
                    spawned = false;
                    origin_set = false;
                }
                continue;
            }
            if !spawned {
                if !origin_set {
                    if let Err(e) = self
                        .cli
                        .run(env::set_origin_args(&self.world, self.home), svc)
                        .await
                    {
                        self.log
                            .log(LogLevel::Warn, format!("set world origin: {e}"));
                    }
                    origin_set = true;
                }
                let _ = self
                    .cli
                    .run(env::remove_model_args(&self.world, &model), svc)
                    .await;
                match self
                    .cli
                    .run(env::spawn_viz_args(&self.world, self.airframe), svc)
                    .await
                {
                    Ok(_) => {
                        spawned = true;
                        last = None;
                        self.log.log(
                            LogLevel::Info,
                            format!("visualizer: spawned {model}, following telemetry"),
                        );
                    }
                    Err(e) => {
                        self.log.log(
                            LogLevel::Warn,
                            format!("visualizer: spawn {model} failed: {e}"),
                        );
                        tokio::time::sleep(Duration::from_secs(5)).await;
                    }
                }
                continue;
            }
            let Some(t) = *self.telemetry.borrow() else {
                continue;
            };
            if last == Some(t) {
                continue;
            }
            last = Some(t);
            let (pos, q) = env::telemetry_to_enu_pose(&self.home, &t);
            if let Err(e) = self
                .cli
                .run(
                    env::set_pose_args(&self.world, &model, pos, q),
                    Duration::from_secs(2),
                )
                .await
            {
                failures += 1;
                if failures == 1 || failures.is_multiple_of(100) {
                    self.log.log(
                        LogLevel::Warn,
                        format!("visualizer: set_pose failed ({failures}x): {e}"),
                    );
                }
            }
        }
    }
}

#[async_trait]
impl Module for GazeboSim {
    fn kind(&self) -> ModuleKind {
        ModuleKind::Physics
    }
    fn backend(&self) -> Backend {
        Backend::Real
    }
    fn name(&self) -> &str {
        "gz-sim"
    }

    async fn start(&mut self, ctx: &SimContext) -> anyhow::Result<()> {
        self.stop().await?;
        set_status(&self.status, ModuleStatus::Starting);
        let log = Logger {
            source: self.name().into(),
            tx: ctx.log.clone(),
        };
        match self.start_inner(ctx, &log).await {
            Ok(detail) => {
                set_status(&self.status, ModuleStatus::Running(detail));
                Ok(())
            }
            Err(e) => {
                let failed = get_status(&self.status);
                self.stop().await?;
                set_status(
                    &self.status,
                    match failed {
                        ModuleStatus::Failed(m) => ModuleStatus::Failed(m),
                        _ => ModuleStatus::Failed(e.to_string()),
                    },
                );
                Err(e)
            }
        }
    }

    async fn stop(&mut self) -> anyhow::Result<()> {
        let mut any = false;
        if let Some(f) = self.follower.take() {
            f.abort();
            any = true;
        }
        if let Some(mut g) = self.gui.take() {
            g.stop().await;
            any = true;
        }
        if let Some(mut s) = self.server.take() {
            s.stop().await;
            any = true;
        }
        if any {
            set_status(&self.status, ModuleStatus::Stopped);
        }
        Ok(())
    }

    fn status(&self) -> ModuleStatus {
        get_status(&self.status)
    }
}

impl Drop for GazeboSim {
    fn drop(&mut self) {
        if let Some(f) = self.follower.take() {
            f.abort();
        }
        // The Procs kill their process groups and containers in their own Drop.
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn autopilot_backend_is_shared() {
        let a = AutopilotBackend::default();
        let b = a.clone();
        assert_eq!(b.get(), Backend::Real);
        for x in [Backend::Simulated, Backend::Off, Backend::Real] {
            a.set(x);
            assert_eq!(b.get(), x);
        }
    }
}
