//! Shared vocabulary of the flight sim.
//!
//! * [`module`] — the [`Module`] trait every component implements (PX4,
//!   Gazebo, QGroundControl, the middleware bus), with a [`Backend`] chosen
//!   at runtime: the real program, a Rust simulation of it, or off.
//! * [`types`] — airframes, geo points, telemetry, MAVLink port layout.
//! * [`mission`] — mission items and the built-in missions per airframe.
//! * [`eval`] — the pure mission-test evaluator fed with telemetry.

pub mod eval;
pub mod mission;
pub mod module;
pub mod types;

pub use eval::{Check, CheckState, MissionTest, TestReport, TestVerdict};
pub use mission::{Mission, MissionItem};
pub use module::{Backend, LogLevel, LogLine, Module, ModuleFactory, ModuleKind, ModuleStatus, SimContext};
pub use types::{Airframe, FlightMode, GeoPoint, MavPorts, Telemetry};
