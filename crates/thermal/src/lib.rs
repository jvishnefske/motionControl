//! # Thermal Manager — PID Temperature Control
//!
//! Manages heater outputs using PID control with configurable parameters.
//! Portable: depends only on `printer-hal` traits, not on board-specific code.

#![no_std]

pub mod manager;
pub mod messages;
pub mod pid;

pub use manager::{HeaterFault, HeaterSafetyLimits, ThermalManager};
pub use messages::*;
pub use pid::PidController;
