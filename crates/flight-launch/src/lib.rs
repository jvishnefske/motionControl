//! The real programs behind the module menu: PX4 SITL, Gazebo Harmonic and
//! QGroundControl, each a [`flight_core::Module`] with `Backend::Real`.
//!
//! PX4 and Gazebo run either natively from a PX4-Autopilot checkout
//! (`FLIGHT_PX4_DIR`, built with `make px4_sitl`) or from the
//! `flight-px4-gz:latest` image built from docker/px4-gz.Dockerfile
//! (`nix run .#build-px4-image`), with `--network host` so MAVLink UDP and
//! gz-transport behave as if native. See [`LaunchConfig`].
//!
//! Every child runs in its own process group; stop and drop kill the group
//! (and `docker rm -f` the container), so nothing is orphaned. Output lines
//! go to `ctx.log`; a child that dies on its own sets the module `Failed`.
//!
//! The Gazebo module needs to know whether PX4 is the autopilot (then PX4
//! spawns and drives the model) or not (then Gazebo visualizes
//! `ctx.telemetry`). `SimContext` does not say, so the orchestrator calls
//! [`Factory::set_autopilot_backend`] whenever it switches the autopilot;
//! the value is an [`AutopilotBackend`] shared with every [`GazeboSim`]
//! the factory created and read live.

mod config;
mod env;
mod gazebo;
mod proc;
mod px4;
mod qgc;

pub use config::{LaunchConfig, Runner, find_qgc, which_in};
pub use env::{
    CONTAINER_PX4_DIR, DEFAULT_IMAGE, DEFAULT_WORLD, DockerRun, GZ_CONTAINER, GZ_GUI_CONTAINER,
    GZ_IP, PX4_CONTAINER, docker_run_args, euler_to_quat, gz_env, gz_gui_args, gz_server_args,
    px4_build_dir, px4_env, px4_native_args, remove_model_args, scene_info_args, set_origin_args,
    set_pose_args, set_pose_req, spawn_viz_args, telemetry_to_enu_pose, viz_model_name, world_sdf,
};
pub use gazebo::{AutopilotBackend, GazeboSim};
pub use px4::Px4Sitl;
pub use qgc::{QGroundControl, locate_qgc, qgc_env};

use flight_core::{Backend, Module, ModuleFactory, ModuleKind};

/// Builds the Real backends of Autopilot, Physics and GroundStation.
#[derive(Debug, Clone, Default)]
pub struct Factory {
    cfg: LaunchConfig,
    autopilot: AutopilotBackend,
}

impl Factory {
    pub fn new(cfg: LaunchConfig) -> Self {
        Factory {
            cfg,
            autopilot: AutopilotBackend::default(),
        }
    }

    /// From `FLIGHT_PX4_DIR`, `FLIGHT_PX4_IMAGE`, `FLIGHT_GZ_WORLD`, `FLIGHT_QGC`.
    pub fn from_env() -> Self {
        Self::new(LaunchConfig::from_env())
    }

    pub fn config(&self) -> &LaunchConfig {
        &self.cfg
    }

    /// Tell the factory (and every Gazebo module it made) which autopilot
    /// backend is selected. Defaults to `Real`. Call it before starting
    /// the autopilot, and whenever it is switched.
    pub fn set_autopilot_backend(&self, backend: Backend) {
        self.autopilot.set(backend);
    }

    pub fn autopilot_backend(&self) -> Backend {
        self.autopilot.get()
    }

    /// The shared handle itself, e.g. to build a [`GazeboSim`] by hand.
    pub fn autopilot_handle(&self) -> AutopilotBackend {
        self.autopilot.clone()
    }
}

impl ModuleFactory for Factory {
    fn available(&self, kind: ModuleKind) -> Vec<(Backend, Result<(), String>)> {
        match kind {
            ModuleKind::Autopilot => vec![(Backend::Real, self.cfg.runner().map(|_| ()))],
            ModuleKind::Physics => {
                let r = self.cfg.runner().and_then(|r| match r {
                    Runner::Native(_) if which_in("gz", std::env::var_os("PATH").as_ref()).is_none() => {
                        Err("gz not on PATH: install Gazebo Harmonic, or unset FLIGHT_PX4_DIR to use the docker image".into())
                    }
                    _ => Ok(()),
                });
                vec![(Backend::Real, r)]
            }
            ModuleKind::GroundStation => vec![(Backend::Real, locate_qgc(&self.cfg).map(|_| ()))],
            ModuleKind::Middleware => vec![],
        }
    }

    fn create(&self, kind: ModuleKind, backend: Backend) -> Option<Box<dyn Module>> {
        if backend != Backend::Real {
            return None;
        }
        match kind {
            ModuleKind::Autopilot => Some(Box::new(Px4Sitl::new(self.cfg.clone()))),
            ModuleKind::Physics => Some(Box::new(GazeboSim::new(
                self.cfg.clone(),
                self.autopilot.clone(),
            ))),
            ModuleKind::GroundStation => Some(Box::new(QGroundControl::new(self.cfg.clone()))),
            ModuleKind::Middleware => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn creates_real_modules_only() {
        let f = Factory::new(LaunchConfig {
            px4_dir: Some("/nonexistent".into()),
            ..Default::default()
        });
        for kind in [
            ModuleKind::Autopilot,
            ModuleKind::Physics,
            ModuleKind::GroundStation,
        ] {
            let m = f.create(kind, Backend::Real).expect("real module");
            assert_eq!(m.kind(), kind);
            assert_eq!(m.backend(), Backend::Real);
            assert!(f.create(kind, Backend::Simulated).is_none());
            assert_eq!(f.available(kind).len(), 1);
        }
        assert!(f.create(ModuleKind::Middleware, Backend::Real).is_none());
        assert!(f.available(ModuleKind::Middleware).is_empty());
        // A bad FLIGHT_PX4_DIR is reported, not silently replaced by docker.
        let (_, r) = &f.available(ModuleKind::Autopilot)[0];
        assert!(r.as_ref().unwrap_err().contains("FLIGHT_PX4_DIR"));
    }

    #[test]
    fn autopilot_backend_reaches_gazebo() {
        let f = Factory::default();
        let h = f.autopilot_handle();
        f.set_autopilot_backend(Backend::Simulated);
        assert_eq!(h.get(), Backend::Simulated);
        assert_eq!(f.autopilot_backend(), Backend::Simulated);
    }
}
