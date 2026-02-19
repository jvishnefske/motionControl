//! # Board HAL — BTT SKR Pico v1.0
//!
//! Pin mappings for the BigTreeTech SKR Pico v1.0 (Voron V0 mainboard).
//!
//! ## Hardware summary
//! - MCU: RP2040 (dual Cortex-M0+, 133MHz, 264KB SRAM, 2MB ext flash)
//! - 4x TMC2209 stepper drivers (soldered, shared UART bus)
//! - 2x heater MOSFET outputs (hotend + bed)
//! - 2x thermistor ADC inputs
//! - 3x fan outputs
//! - USB-C, UART to Raspberry Pi
//! - No SD card slot (UF2 bootloader flash)
//! - NeoPixel RGB LED
//! - Designed for Voron V0 / V0.1 / V0.2

#![no_std]

pub mod pins;
