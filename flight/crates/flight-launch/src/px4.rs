//! PX4 SITL, instance 0.

use std::time::Duration;

use async_trait::async_trait;
use flight_core::{Backend, LogLevel, MavPorts, Module, ModuleKind, ModuleStatus, SimContext};

use crate::config::{LaunchConfig, Runner};
use crate::env::{self, DockerRun, PX4_CONTAINER};
use crate::proc::{Logger, Proc, SharedStatus, Spec, get_status, new_status, set_status};

/// Printed by px4 main.cpp when rcS finished.
const READY: &[&str] = &["Startup script returned successfully"];
/// rcS failed, or px4-rc.gzsim gave up on Gazebo.
const FAIL: &[&str] = &[
    "Startup script returned with return value",
    "Timed out waiting for Gazebo world",
    "gz_bridge failed",
    "Gazebo gz sim not found",
    "PX4 server already running",
];
/// px4-rc.gzsim waits up to 30 s for the world, plus start-up and spawn.
const START_TIMEOUT: Duration = Duration::from_secs(90);

/// PX4 SITL instance 0 (GCS link to udp 14550, API link 14580 -> 14540).
///
/// With `ctx.physics_backend == Backend::Real` PX4 runs in Gazebo
/// standalone mode and spawns its model into the world the Gazebo module
/// started; otherwise it uses its built-in SIH physics.
pub struct Px4Sitl {
    cfg: LaunchConfig,
    status: SharedStatus,
    proc: Option<Proc>,
}

impl Px4Sitl {
    pub fn new(cfg: LaunchConfig) -> Self {
        Px4Sitl {
            cfg,
            status: new_status(),
            proc: None,
        }
    }

    fn spec(runner: &Runner, ctx: &SimContext) -> Spec {
        let mut px4 = env::px4_env(ctx.airframe, ctx.physics_backend, ctx.home);
        let mut spec = match runner {
            Runner::Native(dir) => {
                if ctx.physics_backend == Backend::Real {
                    // Standalone px4-rc.gzsim does not source gz_env.sh: it
                    // needs PX4_GZ_MODELS for the spawn URI.
                    let rp = std::env::var("GZ_SIM_RESOURCE_PATH").ok();
                    let pp = std::env::var("GZ_SIM_SYSTEM_PLUGIN_PATH").ok();
                    px4.extend(env::gz_env(dir, rp.as_deref(), pp.as_deref()));
                }
                let build = env::px4_build_dir(dir);
                let mut s = Spec::new(build.join("bin/px4"), env::px4_native_args(dir));
                s.cwd = Some(build.join("rootfs"));
                s.env = px4;
                s
            }
            Runner::Docker(image) => {
                let cmd = vec!["px4".to_string()];
                let args = docker_run_args_for(PX4_CONTAINER, image, &px4, false, &cmd);
                let mut s = Spec::new("docker", args);
                s.container = Some(PX4_CONTAINER.into());
                s
            }
        };
        spec.ready = READY;
        spec.fail = FAIL;
        spec
    }
}

pub(crate) fn docker_run_args_for(
    name: &str,
    image: &str,
    env: &[(String, String)],
    gui: bool,
    cmd: &[String],
) -> Vec<String> {
    let display = std::env::var("DISPLAY").ok();
    let wayland = std::env::var("WAYLAND_DISPLAY").ok();
    let xdg = std::env::var("XDG_RUNTIME_DIR").ok();
    env::docker_run_args(&DockerRun {
        name,
        image,
        env,
        gui,
        dri: std::path::Path::new("/dev/dri").exists(),
        display: display.as_deref(),
        wayland_display: wayland.as_deref(),
        xdg_runtime_dir: xdg.as_deref(),
        cmd,
    })
}

#[async_trait]
impl Module for Px4Sitl {
    fn kind(&self) -> ModuleKind {
        ModuleKind::Autopilot
    }
    fn backend(&self) -> Backend {
        Backend::Real
    }
    fn name(&self) -> &str {
        "px4-sitl"
    }

    async fn start(&mut self, ctx: &SimContext) -> anyhow::Result<()> {
        self.stop().await?;
        set_status(&self.status, ModuleStatus::Starting);
        let log = Logger {
            source: self.name().into(),
            tx: ctx.log.clone(),
        };
        let runner = match self.cfg.runner() {
            Ok(r) => r,
            Err(e) => {
                set_status(&self.status, ModuleStatus::Failed(e.clone()));
                anyhow::bail!(e);
            }
        };
        if ctx.ports != MavPorts::default() {
            log.log(
                LogLevel::Warn,
                format!(
                    "PX4 instance 0 always uses udp 14550 (GCS) and 14540 (API); ignoring {:?}",
                    ctx.ports
                ),
            );
        }
        if let Runner::Native(dir) = &runner {
            let _ = std::fs::create_dir_all(env::px4_build_dir(dir).join("rootfs"));
        }
        let physics = if ctx.physics_backend == Backend::Real {
            format!("Gazebo {}", ctx.airframe.px4_gz_model())
        } else {
            format!("SIH {}", ctx.airframe.px4_sih_model())
        };
        let proc =
            match Proc::spawn(Self::spec(&runner, ctx), "PX4", log, self.status.clone()).await {
                Ok(p) => p,
                Err(e) => {
                    set_status(&self.status, ModuleStatus::Failed(e.to_string()));
                    return Err(e);
                }
            };
        let ready = proc.wait_ready(START_TIMEOUT).await;
        self.proc = Some(proc);
        match ready {
            Ok(()) => {
                set_status(
                    &self.status,
                    ModuleStatus::Running(format!("{}, {physics}", runner.describe())),
                );
                Ok(())
            }
            Err(why) => {
                self.stop().await?;
                let msg = format!("PX4 did not start: {why}");
                set_status(&self.status, ModuleStatus::Failed(msg.clone()));
                anyhow::bail!(msg)
            }
        }
    }

    async fn stop(&mut self) -> anyhow::Result<()> {
        if let Some(mut p) = self.proc.take() {
            p.stop().await;
            set_status(&self.status, ModuleStatus::Stopped);
        }
        Ok(())
    }

    fn status(&self) -> ModuleStatus {
        get_status(&self.status)
    }
}
