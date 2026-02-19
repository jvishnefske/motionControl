//! # Motion Planner — Trapezoidal Acceleration with Look-Ahead
//!
//! Converts G-code motion commands into step-timed segments with
//! proper acceleration, deceleration, and multi-axis coordination.
//!
//! Runs as an async actor: receives motion commands via mailbox,
//! produces step events for the stepper driver ISR.

#![no_std]

pub mod messages;
pub mod planner;
pub mod segment;
pub mod step_generator;

pub use messages::*;
pub use planner::MotionPlanner;
pub use segment::*;
pub use step_generator::execute_segment;
