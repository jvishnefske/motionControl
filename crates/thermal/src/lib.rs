//! # Thermal Manager — PID Temperature Control Actor
//!
//! Manages heater outputs using PID control with configurable parameters.
//! Runs as an async actor, receiving temperature setpoints and reporting
//! current temperatures via the event bus.

#![no_std]

pub mod pid;
pub mod messages;
pub mod manager;

pub use messages::*;
pub use manager::ThermalManager;
pub use pid::PidController;
