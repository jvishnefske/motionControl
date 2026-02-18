//! Motion segment — the output of the planner, consumed by the step generator.
//!
//! Each segment represents a phase of motion (acceleration, cruise, deceleration)
//! with pre-computed step intervals for each axis.

/// A motion segment ready for step execution.
#[derive(Clone, Debug, defmt::Format)]
pub struct MotionSegment {
    /// Number of steps for each axis in this segment.
    pub steps: [i32; 4], // X, Y, Z, E

    /// Direction for each axis (true = positive).
    pub direction: [bool; 4],

    /// Initial step interval in microseconds (speed at segment start).
    pub initial_interval_us: u32,

    /// Final step interval in microseconds (speed at segment end).
    pub final_interval_us: u32,

    /// Acceleration in steps/s^2 (0 for cruise segments).
    pub accel_steps_s2: f32,

    /// Total duration of this segment in microseconds.
    pub duration_us: u32,

    /// Segment type for debugging/display.
    pub phase: SegmentPhase,
}

/// Phase of a trapezoidal motion profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum SegmentPhase {
    Accelerate,
    Cruise,
    Decelerate,
}
