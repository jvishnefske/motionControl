//! Rust stand-ins for the external programs, behind the
//! [`flight_core::Module`] trait:
//!
//! * [`SimAutopilot`] — a PX4-compatible MAVLink autopilot with vehicle
//!   dynamics for each airframe, on PX4 SITL instance 0's ports;
//! * [`SimPhysics`] — the "simulated physics" choice (PX4 SIH when PX4 is
//!   real, the Rust dynamics inside `SimAutopilot` when it is not);
//! * [`SimGroundStation`] — a heartbeat-only ground station;
//! * [`Factory`] — offers all three as `Backend::Simulated`.

// MAVLink struct literals end in `..Default::default()` on purpose: if any
// crate in the workspace enables mavlink's `emit-extensions`, the structs
// gain extension fields and these literals must keep compiling.
#![allow(clippy::needless_update)]

mod autopilot;
mod gcs;
pub mod mav;
pub mod vehicle;

use async_trait::async_trait;
use flight_core::{Backend, Module, ModuleFactory, ModuleKind, ModuleStatus, SimContext};

pub use autopilot::{COMPONENT_ID, SYSTEM_ID, SimAutopilot, local_ports};
pub use gcs::{GCS_COMPONENT_ID, GCS_SYSTEM_ID, SimGroundStation};

/// Simulated physics. There is nothing to run: with the real PX4 the
/// `sihsim_*` airframes use PX4's built-in SIH physics; with the sim
/// autopilot the dynamics live inside [`SimAutopilot`].
pub struct SimPhysics {
    status: ModuleStatus,
}

impl Default for SimPhysics {
    fn default() -> Self {
        Self::new()
    }
}

impl SimPhysics {
    pub fn new() -> SimPhysics {
        SimPhysics {
            status: ModuleStatus::Stopped,
        }
    }
}

#[async_trait]
impl Module for SimPhysics {
    fn kind(&self) -> ModuleKind {
        ModuleKind::Physics
    }

    fn backend(&self) -> Backend {
        Backend::Simulated
    }

    fn name(&self) -> &str {
        "sim-physics"
    }

    async fn start(&mut self, ctx: &SimContext) -> anyhow::Result<()> {
        self.status = ModuleStatus::Running(format!(
            "PX4 SIH ({}) or Rust dynamics in the sim autopilot",
            ctx.airframe.px4_sih_model()
        ));
        Ok(())
    }

    async fn stop(&mut self) -> anyhow::Result<()> {
        self.status = ModuleStatus::Stopped;
        Ok(())
    }

    fn status(&self) -> ModuleStatus {
        self.status.clone()
    }
}

/// Builds the simulated backends.
pub struct Factory;

impl ModuleFactory for Factory {
    fn available(&self, kind: ModuleKind) -> Vec<(Backend, Result<(), String>)> {
        match kind {
            ModuleKind::Autopilot | ModuleKind::Physics | ModuleKind::GroundStation => {
                vec![(Backend::Simulated, Ok(()))]
            }
            ModuleKind::Middleware => Vec::new(),
        }
    }

    fn create(&self, kind: ModuleKind, backend: Backend) -> Option<Box<dyn Module>> {
        if backend != Backend::Simulated {
            return None;
        }
        match kind {
            ModuleKind::Autopilot => Some(Box::new(SimAutopilot::new())),
            ModuleKind::Physics => Some(Box::new(SimPhysics::new())),
            ModuleKind::GroundStation => Some(Box::new(SimGroundStation::new())),
            ModuleKind::Middleware => None,
        }
    }
}
