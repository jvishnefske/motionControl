//! Pin definitions for the BTT SKR Pico v1.0.
//!
//! Pin assignments sourced from the Klipper generic-bigtreetech-skr-pico-v1.0.cfg,
//! BTT GitHub schematics, and RP2040 datasheet.
//!
//! All pins are RP2040 GPIO numbers (0-29).

/// Complete pin mapping for BTT SKR Pico v1.0.
pub struct SkrPicoPins;

impl SkrPicoPins {
    // ── Stepper X (TMC2209, UART addr 0) ───────────────────────
    pub const X_STEP: u8 = 11;
    pub const X_DIR: u8 = 10;
    pub const X_ENABLE: u8 = 12;
    pub const X_DIAG: u8 = 4; // sensorless homing / endstop

    // ── Stepper Y (TMC2209, UART addr 2) ───────────────────────
    pub const Y_STEP: u8 = 6;
    pub const Y_DIR: u8 = 5;
    pub const Y_ENABLE: u8 = 7;
    pub const Y_DIAG: u8 = 3; // sensorless homing / endstop

    // ── Stepper Z (TMC2209, UART addr 1) ───────────────────────
    /// Z1 and Z2 motor connectors are wired in parallel to the same driver.
    pub const Z_STEP: u8 = 19;
    pub const Z_DIR: u8 = 28;
    pub const Z_ENABLE: u8 = 2;
    pub const Z_DIAG: u8 = 25; // Z endstop

    // ── Stepper E0 (TMC2209, UART addr 3) ──────────────────────
    pub const E_STEP: u8 = 14;
    pub const E_DIR: u8 = 13;
    pub const E_ENABLE: u8 = 15;
    pub const E_DIAG: u8 = 16; // filament runout sensor

    // ── TMC2209 shared UART bus ────────────────────────────────
    /// All 4 drivers share this UART bus, differentiated by address.
    pub const TMC_UART_TX: u8 = 8;
    pub const TMC_UART_RX: u8 = 9;

    /// TMC2209 UART addresses (verified from Klipper config).
    pub const TMC_ADDR_X: u8 = 0;
    pub const TMC_ADDR_Y: u8 = 2;
    pub const TMC_ADDR_Z: u8 = 1;
    pub const TMC_ADDR_E: u8 = 3;

    // ── Heater outputs ─────────────────────────────────────────
    pub const HEATER_HOTEND: u8 = 23;
    pub const HEATER_BED: u8 = 21;

    // ── Thermistor ADC inputs ──────────────────────────────────
    pub const THERM_BED: u8 = 26; // ADC0
    pub const THERM_HOTEND: u8 = 27; // ADC1

    // ── Fan outputs (PWM, MOSFET) ──────────────────────────────
    /// Fans get Vcc directly (12-24V). No buck converter.
    pub const FAN1_PART: u8 = 17; // part cooling
    pub const FAN2_HOTEND: u8 = 18; // heatbreak fan
    pub const FAN3_CONTROLLER: u8 = 20; // controller/MCU fan

    // ── Endstop inputs ─────────────────────────────────────────
    /// Shared with DIAG pins via jumpers on PCB.
    pub const ENDSTOP_X: u8 = 4; // = X_DIAG
    pub const ENDSTOP_Y: u8 = 3; // = Y_DIAG
    pub const ENDSTOP_Z: u8 = 25; // = Z_DIAG

    // ── Probe / BLTouch ────────────────────────────────────────
    pub const PROBE_SIGNAL: u8 = 22; // BLTouch sensor / proximity
    pub const PROBE_SERVO: u8 = 29; // BLTouch control (also ADC3)

    // ── UART0 (Raspberry Pi communication) ─────────────────────
    pub const UART_TX: u8 = 0;
    pub const UART_RX: u8 = 1;

    // ── NeoPixel ───────────────────────────────────────────────
    pub const NEOPIXEL: u8 = 24;
}
