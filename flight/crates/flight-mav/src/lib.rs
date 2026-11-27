//! MAVLink client for the flight sim.
//!
//! * [`VehicleLink`] implements [`flight_core::VehicleControl`] over UDP on
//!   the API port, the way MAVSDK talks to PX4 SITL instance 0. It works the
//!   same against PX4 and the Rust sim autopilot and survives the vehicle
//!   being swapped on the port.
//! * [`run_mission_test`] flies a [`flight_core::Mission`] through any
//!   `VehicleControl` and judges it with [`flight_core::MissionTest`].

// MAVLink struct literals end in `..Default::default()` on purpose: if any
// crate in the workspace enables mavlink's `emit-extensions`, the structs
// gain extension fields and these literals must keep compiling.
#![allow(clippy::needless_update)]

pub mod codec;
mod link;
pub mod px4;
mod runner;

pub use link::{OWN_COMPONENT_ID, OWN_SYSTEM_ID, VehicleLink, mission_item_int};
pub use runner::{READY_TIMEOUT, run_mission_test};
