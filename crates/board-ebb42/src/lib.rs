//! # Board HAL — BTT EBB42 v1.2 CAN Toolboard
//!
//! Pin mappings for the BigTreeTech EBB42 v1.2 CAN toolboard.
//!
//! ## Hardware summary
//! - MCU: STM32G0B1CBT6 (Cortex-M0+, 64MHz, 128KB Flash, 144KB SRAM)
//! - 1x TMC2209 stepper driver (extruder)
//! - 1x heater MOSFET output (hotend)
//! - 1x thermistor ADC input + 1x onboard NTC
//! - 2x fan outputs (part cooling + hotend)
//! - CAN-FD via FDCAN2 (PB0/PB1)
//! - ADXL345 accelerometer (SPI2)
//! - MAX31865 PT100/PT1000 (SPI1, optional)
//! - USB-C, endstop inputs, NeoPixel, BLTouch
//! - 8 MHz external crystal

#![no_std]

pub mod pins;

/// STM32 GPIO port identifier.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum Port {
    A,
    B,
    D,
}

/// GPIO pin identifier.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Pin {
    pub port: Port,
    pub pin: u8,
}

impl Pin {
    pub const fn new(port: Port, pin: u8) -> Self {
        Self { port, pin }
    }
}
