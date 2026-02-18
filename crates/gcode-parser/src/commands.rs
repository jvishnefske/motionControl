//! Typed G-code command enum.
//!
//! Every supported command is represented as a variant with its parsed parameters.
//! This keeps business logic clean — pattern match on the command, not raw strings.

use crate::types::*;

/// A fully parsed G-code command.
#[derive(Clone, Debug, defmt::Format)]
pub enum GCodeCommand {
    // ── Motion ────────────────────────────────────────────────────
    /// G0/G1: Linear move.
    LinearMove {
        axes: AxisValues,
        feedrate: Option<f32>,    // F parameter, mm/min
        endstop_action: Option<u8>, // H parameter (0-4)
    },

    /// G28: Home axes.
    Home {
        axes: AxisFlags,
    },

    /// G90: Set absolute positioning.
    AbsolutePositioning,

    /// G91: Set relative positioning.
    RelativePositioning,

    /// G92: Set position without movement.
    SetPosition {
        axes: AxisValues,
    },

    // ── Configuration ─────────────────────────────────────────────
    /// M92: Set steps per mm.
    SetStepsPerMm {
        axes: AxisValues,
    },

    /// M201: Set max per-axis accelerations (mm/s^2).
    SetMaxAccelPerAxis {
        axes: AxisValues,
    },

    /// M203: Set max feedrates (mm/min).
    SetMaxFeedrate {
        axes: AxisValues,
    },

    /// M204: Set default acceleration.
    SetAcceleration {
        print_accel: Option<f32>,   // P
        travel_accel: Option<f32>,  // T
    },

    /// M206: Set home offsets.
    SetHomeOffset {
        axes: AxisValues,
    },

    /// M208: Set axis limits.
    SetAxisLimits {
        axes: AxisValues,
        max: bool, // S0 = max, S1 = min
    },

    /// M350: Set microstepping.
    SetMicrostepping {
        axes: AxisValues,
        interpolation: Option<bool>, // I parameter
    },

    /// M569: Set stepper driver direction and mode.
    SetDriverConfig {
        driver: u8,                      // P parameter
        direction: Option<bool>,         // S parameter (true = forward)
        mode: Option<DriverMode>,        // D parameter
    },

    /// M584: Set drive mapping.
    SetDriveMapping {
        axes: AxisValues,
    },

    /// M906: Set motor currents (mA).
    SetMotorCurrent {
        axes: AxisValues,
        idle_percent: Option<f32>, // I parameter
    },

    // ── Temperature ───────────────────────────────────────────────
    /// M104: Set hotend temperature (no wait).
    SetHotendTemp {
        temp: f32,
        tool: Option<u8>,
    },

    /// M109: Set hotend temperature and wait.
    SetHotendTempWait {
        temp: f32,
        tool: Option<u8>,
    },

    /// M140: Set bed temperature (no wait) or configure bed heater.
    SetBedTemp {
        temp: Option<f32>,
        heater: Option<u8>, // H parameter for config
    },

    /// M190: Set bed temperature and wait.
    SetBedTempWait {
        temp: f32,
    },

    // ── Persistence ───────────────────────────────────────────────
    /// M500: Save parameters to SD card.
    SaveConfig,

    /// M501: Load parameters from SD card.
    LoadConfig,

    // ── Fan control ───────────────────────────────────────────────
    /// M106: Set fan speed.
    SetFanSpeed {
        fan: Option<u8>,   // P parameter
        speed: f32,        // S parameter (0.0 - 1.0)
    },

    /// M107: Turn fan off.
    FanOff {
        fan: Option<u8>,   // P parameter
    },

    // ── Misc ──────────────────────────────────────────────────────
    /// M112: Emergency stop.
    EmergencyStop,

    /// M114: Get current position.
    GetPosition,

    /// M115: Get firmware version.
    GetFirmwareVersion,

    /// M400: Wait for moves to finish.
    WaitForMoves,

    /// M503: Report current settings.
    ReportSettings,

    /// Comment or blank line — no action.
    Comment,

    /// Unrecognized command.
    Unknown {
        letter: u8,
        code: u16,
    },
}
