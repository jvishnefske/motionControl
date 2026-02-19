//! Pin definitions for the BTT EBB42 v1.2 CAN toolboard.
//!
//! Pin assignments sourced from the Klipper sample-bigtreetech-ebb-canbus-v1.2.cfg,
//! BTT GitHub schematics, and STM32G0B1 datasheet.

use crate::{Pin, Port};

/// Complete pin mapping for BTT EBB42 v1.2.
pub struct Ebb42Pins;

impl Ebb42Pins {
    // ── Stepper driver (TMC2209, extruder) ─────────────────────
    pub const STEP: Pin = Pin::new(Port::D, 0);
    pub const DIR: Pin = Pin::new(Port::D, 1); // active-low in Klipper
    pub const ENABLE: Pin = Pin::new(Port::D, 2); // active-low
    pub const TMC_UART: Pin = Pin::new(Port::A, 15); // half-duplex, addr 0x00

    // ── Heater output ──────────────────────────────────────────
    /// Hotend heater MOSFET (moved to PB13 in v1.2 for DFU safety).
    pub const HEATER: Pin = Pin::new(Port::B, 13); // max 5A

    // ── Thermistor ADC inputs ──────────────────────────────────
    /// TH0: hotend thermistor (100K NTC or PT1000 via jumper).
    pub const THERM_HOTEND: Pin = Pin::new(Port::A, 3); // ADC1_IN3
    /// Onboard board temperature NTC.
    pub const THERM_BOARD: Pin = Pin::new(Port::A, 2); // ADC1_IN2

    // ── Fan outputs ────────────────────────────────────────────
    /// FAN0: part cooling fan (max 1A).
    pub const FAN0: Pin = Pin::new(Port::A, 0);
    /// FAN1: hotend/heatbreak fan (max 1A).
    pub const FAN1: Pin = Pin::new(Port::A, 1);

    // ── CAN bus (FDCAN2) ───────────────────────────────────────
    /// CAN RX — FDCAN2_RX (AF3). NOTE: This is FDCAN2, not FDCAN1.
    pub const CAN_RX: Pin = Pin::new(Port::B, 0);
    /// CAN TX — FDCAN2_TX (AF3).
    pub const CAN_TX: Pin = Pin::new(Port::B, 1);

    // ── Endstop / probe inputs ─────────────────────────────────
    pub const ENDSTOP_1: Pin = Pin::new(Port::B, 7);
    pub const ENDSTOP_2: Pin = Pin::new(Port::B, 5);
    pub const ENDSTOP_3: Pin = Pin::new(Port::B, 6); // commonly probe
    pub const BLTOUCH_SENSOR: Pin = Pin::new(Port::B, 8); // input, pull-up
    pub const BLTOUCH_CONTROL: Pin = Pin::new(Port::B, 9); // servo PWM

    // ── ADXL345 accelerometer (SPI2) ───────────────────────────
    pub const ADXL_CS: Pin = Pin::new(Port::B, 12);
    pub const ADXL_SCK: Pin = Pin::new(Port::B, 10);
    pub const ADXL_MOSI: Pin = Pin::new(Port::B, 11);
    pub const ADXL_MISO: Pin = Pin::new(Port::B, 2);

    // ── MAX31865 PT100/PT1000 (SPI1, optional) ─────────────────
    pub const MAX31865_CS: Pin = Pin::new(Port::A, 4);
    pub const MAX31865_SCK: Pin = Pin::new(Port::A, 5);
    pub const MAX31865_MISO: Pin = Pin::new(Port::A, 6);
    pub const MAX31865_MOSI: Pin = Pin::new(Port::A, 7);

    // ── USB ────────────────────────────────────────────────────
    pub const USB_DM: Pin = Pin::new(Port::A, 11);
    pub const USB_DP: Pin = Pin::new(Port::A, 12);

    // ── NeoPixel ───────────────────────────────────────────────
    pub const NEOPIXEL: Pin = Pin::new(Port::D, 3);

    // ── Filament sensor ────────────────────────────────────────
    pub const FILAMENT_SWITCH: Pin = Pin::new(Port::B, 4);
    pub const FILAMENT_MOTION: Pin = Pin::new(Port::B, 3);
}
