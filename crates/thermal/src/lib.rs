//! # Thermal Manager — PID Temperature Control Actor
//!
//! Manages heater outputs using PID control with configurable parameters.
//! Runs as an async actor, receiving temperature setpoints and reporting
//! current temperatures via the event bus.

#![no_std]

pub mod manager;
pub mod messages;
pub mod pid;

pub use manager::ThermalManager;
pub use messages::*;
pub use pid::PidController;
