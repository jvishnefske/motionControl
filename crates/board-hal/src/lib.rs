//! # Board HAL — Duet 3 Mini 5+ Hardware Abstraction
//!
//! Pin mappings, peripheral configuration, and driver abstractions
//! for the ATSAME54P20A-based Duet 3 Mini 5+ controller board.
//!
//! ## Hardware summary
//! - MCU: ATSAME54P20A (Cortex-M4F, 120MHz, 1MB Flash, 256KB RAM)
//! - 5x TMC2209 stepper drivers (shared UART bus, individual step/dir)
//! - 7x PWM outputs (heaters + fans)
//! - 3x thermistor ADC inputs
//! - MicroSD slot (SDHC peripheral)
//! - USB, CAN-FD, WiFi/Ethernet

#![no_std]

pub mod pins;
pub mod stepper;
pub mod thermistor;
pub mod tmc2209;

pub use pins::Duet3Pins;

// Re-export portable types from printer-hal for backward compat.
pub mod pwm_output {
    pub use printer_hal::{DutyCycle, PwmChannel};
}
pub use printer_hal;
