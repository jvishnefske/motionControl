//! The runtime-swappable module contract.
//!
//! Every component of the sim is a [`Module`] of one [`ModuleKind`]. For each
//! kind a [`ModuleFactory`] can build one or more [`Backend`]s: the real
//! program (PX4 SITL, `gz sim`, QGroundControl, a Zenoh session) or a Rust
//! simulation of it. The orchestrator stops the running module of a kind and
//! starts another backend while the rest keeps running; that is the
//! one-click switch in the menu.

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use tokio::sync::{broadcast, watch};

use crate::types::{Airframe, GeoPoint, MavPorts, Telemetry};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ModuleKind {
    /// The flight controller: PX4 SITL, or the Rust autopilot.
    Autopilot,
    /// Vehicle physics and world: Gazebo, or simulated (PX4 SIH when the
    /// autopilot is real, the Rust dynamics model when it is simulated).
    Physics,
    /// Ground control station: QGroundControl, or a Rust GCS heartbeat.
    GroundStation,
    /// Telemetry/command bus: ~/src/middleware over Zenoh, or its in-memory backend.
    Middleware,
}

impl ModuleKind {
    /// Start order; stop runs in reverse.
    pub const ALL: [ModuleKind; 4] =
        [ModuleKind::Middleware, ModuleKind::Physics, ModuleKind::Autopilot, ModuleKind::GroundStation];

    pub fn label(self) -> &'static str {
        match self {
            ModuleKind::Autopilot => "Autopilot (PX4)",
            ModuleKind::Physics => "Physics (Gazebo)",
            ModuleKind::GroundStation => "Ground station (QGroundControl)",
            ModuleKind::Middleware => "Middleware bus",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Backend {
    /// The real external program.
    Real,
    /// A Rust implementation standing in for it.
    Simulated,
    /// Not running.
    Off,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "state", content = "detail", rename_all = "snake_case")]
pub enum ModuleStatus {
    Stopped,
    Starting,
    Running(String),
    Failed(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LogLevel {
    Info,
    Warn,
    Error,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LogLine {
    pub source: String,
    pub level: LogLevel,
    pub text: String,
}

/// What a module gets when it starts. Cheap to clone.
#[derive(Clone)]
pub struct SimContext {
    pub airframe: Airframe,
    pub home: GeoPoint,
    pub ports: MavPorts,
    /// Which backend each other kind currently uses, so e.g. the PX4 module
    /// can pick Gazebo (`gz_*`) or SIH (`sihsim_*`) physics.
    pub physics_backend: Backend,
    /// Start Gazebo / QGC with a GUI (false: headless).
    pub gui: bool,
    /// Latest vehicle telemetry (written by the mission runner's MAVLink link).
    pub telemetry: watch::Receiver<Option<Telemetry>>,
    /// Log sink shown in the UI.
    pub log: broadcast::Sender<LogLine>,
}

impl SimContext {
    pub fn log(&self, source: &str, level: LogLevel, text: impl Into<String>) {
        let _ = self.log.send(LogLine { source: source.to_string(), level, text: text.into() });
    }
}

#[async_trait]
pub trait Module: Send + Sync {
    fn kind(&self) -> ModuleKind;
    fn backend(&self) -> Backend;
    /// Short name for the UI, e.g. "px4-sitl" or "sim-autopilot".
    fn name(&self) -> &str;
    /// Start and return once the module is usable (or failed).
    async fn start(&mut self, ctx: &SimContext) -> anyhow::Result<()>;
    /// Stop; must be idempotent and leave no child process behind.
    async fn stop(&mut self) -> anyhow::Result<()>;
    fn status(&self) -> ModuleStatus;
}

/// Builds modules of a kind and backend.
pub trait ModuleFactory: Send + Sync {
    /// Backends this factory offers for `kind` (besides `Off`), with whether
    /// each is usable on this machine and why not.
    fn available(&self, kind: ModuleKind) -> Vec<(Backend, Result<(), String>)>;
    fn create(&self, kind: ModuleKind, backend: Backend) -> Option<Box<dyn Module>>;
}

/// Command and telemetry access to whatever vehicle is on the MAVLink API
/// port (PX4 SITL or the Rust autopilot; the caller cannot tell). Lives
/// across module switches: it keeps listening and picks up the new vehicle.
#[async_trait]
pub trait VehicleControl: Send + Sync {
    /// Latest telemetry; `None` until a vehicle has been heard.
    fn telemetry(&self) -> watch::Receiver<Option<Telemetry>>;
    /// Wait until a vehicle sends heartbeats, has a home position and a
    /// position estimate good enough to arm.
    async fn wait_ready(&self, timeout: std::time::Duration) -> anyhow::Result<()>;
    /// Clear, then upload the mission with the MAVLink mission protocol.
    async fn upload_mission(&self, mission: &crate::mission::Mission) -> anyhow::Result<()>;
    async fn arm(&self) -> anyhow::Result<()>;
    /// Switch to mission mode and start from item 0.
    async fn start_mission(&self) -> anyhow::Result<()>;
    async fn return_to_launch(&self) -> anyhow::Result<()>;
    async fn land(&self) -> anyhow::Result<()>;
}
