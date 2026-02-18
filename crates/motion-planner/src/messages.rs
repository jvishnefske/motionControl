//! Messages for the motion planner actor.

use gcode_parser::AxisValues;

/// Commands sent TO the motion planner.
#[derive(Clone, Debug, defmt::Format)]
pub enum MotionCommand {
    /// Execute a linear move (from G0/G1).
    LinearMove {
        target: AxisValues,
        feedrate_mm_min: f32,
        is_rapid: bool,
    },

    /// Home specified axes (from G28).
    Home {
        x: bool,
        y: bool,
        z: bool,
    },

    /// Set absolute positioning (G90).
    SetAbsolute,

    /// Set relative positioning (G91).
    SetRelative,

    /// Set current position without motion (G92).
    SetPosition {
        axes: AxisValues,
    },

    /// Configure steps per mm (M92).
    SetStepsPerMm {
        axes: AxisValues,
    },

    /// Set maximum feedrate (M203).
    SetMaxFeedrate {
        axes: AxisValues,
    },

    /// Set per-axis max acceleration (M201).
    SetMaxAccelPerAxis {
        axes: AxisValues,
    },

    /// Set default acceleration (M204).
    SetAcceleration {
        print_accel: Option<f32>,
        travel_accel: Option<f32>,
    },

    /// Set microstepping (M350).
    SetMicrostepping {
        axes: AxisValues,
        interpolation: Option<bool>,
    },

    /// Set motor current in mA (M906).
    SetMotorCurrent {
        axes: AxisValues,
        idle_percent: Option<f32>,
    },

    /// Set driver direction/mode (M569).
    SetDriverConfig {
        driver: u8,
        direction: Option<bool>,
        stealthchop: Option<bool>,
    },

    /// Wait for all pending moves to complete (M400).
    WaitForCompletion,

    /// Emergency stop — halt all motion immediately.
    EmergencyStop,

    /// Get current position — responds via status channel.
    ReportPosition,
}

/// Status updates FROM the motion planner.
#[derive(Clone, Debug, defmt::Format)]
pub enum MotionStatus {
    /// Current machine position.
    Position {
        x_mm: f32,
        y_mm: f32,
        z_mm: f32,
        e_mm: f32,
    },

    /// Homing completed for an axis.
    HomingComplete {
        x: bool,
        y: bool,
        z: bool,
    },

    /// All queued moves have been executed.
    MovesComplete,

    /// Motion error.
    Error(MotionError),
}

#[derive(Clone, Debug, defmt::Format)]
pub enum MotionError {
    NotHomed,
    SoftLimitExceeded,
    StallDetected { axis: u8 },
}
