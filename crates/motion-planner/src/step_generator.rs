//! Step generator — converts motion segments into precisely timed step pulses.
//!
//! Uses Bresenham multi-axis step distribution. Portable: accepts a
//! `StepperDriver` trait for hardware abstraction.

use crate::segment::MotionSegment;
use printer_hal::StepperDriver;

/// Result of executing one segment.
#[derive(Clone, Copy, Debug)]
pub struct StepResult {
    /// Total master steps executed.
    pub total_steps: u32,
    /// Duration in microseconds.
    pub duration_us: u32,
}

/// Execute a single motion segment using Bresenham multi-axis step distribution.
///
/// Returns the step intervals (in microseconds) that the caller should wait
/// between each master step. The caller is responsible for the actual timing
/// (Embassy Timer, busy-wait, etc.) since timing mechanisms differ per platform.
///
/// `step_callback` is called for each master step with the interval to wait
/// *after* that step.
pub fn execute_segment<S: StepperDriver>(
    segment: &MotionSegment,
    driver: &mut S,
    mut step_callback: impl FnMut(u32),
) -> StepResult {
    let total_steps: i32 = segment
        .steps
        .iter()
        .map(|s| s.unsigned_abs() as i32)
        .max()
        .unwrap_or(0);

    if total_steps == 0 {
        return StepResult {
            total_steps: 0,
            duration_us: 0,
        };
    }

    // Set direction for all axes before stepping
    for axis in 0..4u8 {
        driver.set_direction(axis, segment.direction[axis as usize]);
    }

    // Bresenham multi-axis step distribution
    let mut accum = [0i32; 4];
    let mut current_interval = segment.initial_interval_us;
    let interval_delta = if total_steps > 1 {
        (segment.final_interval_us as i32 - segment.initial_interval_us as i32)
            / (total_steps - 1).max(1)
    } else {
        0
    };

    let mut total_duration_us: u32 = 0;

    for _step in 0..total_steps {
        for (axis, acc) in accum.iter_mut().enumerate() {
            let axis_steps = segment.steps[axis].unsigned_abs() as i32;
            *acc += axis_steps;
            if *acc >= total_steps {
                *acc -= total_steps;
                driver.step(axis as u8);
            }
        }

        let wait_us = current_interval.min(100_000); // cap at 100ms
        total_duration_us = total_duration_us.saturating_add(wait_us);

        step_callback(wait_us);

        current_interval = (current_interval as i32 + interval_delta).max(1) as u32;
    }

    StepResult {
        total_steps: total_steps as u32,
        duration_us: total_duration_us,
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::segment::SegmentPhase;
    use std::vec::Vec;

    struct MockStepper {
        steps: [i32; 4],
        directions: [bool; 4],
    }

    impl MockStepper {
        fn new() -> Self {
            Self {
                steps: [0; 4],
                directions: [true; 4],
            }
        }
    }

    impl StepperDriver for MockStepper {
        fn set_direction(&mut self, axis: u8, forward: bool) {
            self.directions[axis as usize] = forward;
        }

        fn step(&mut self, axis: u8) {
            if self.directions[axis as usize] {
                self.steps[axis as usize] += 1;
            } else {
                self.steps[axis as usize] -= 1;
            }
        }

        fn enable(&mut self, _axis: u8, _enabled: bool) {}
    }

    #[test]
    fn test_single_axis_move() {
        let segment = MotionSegment {
            steps: [100, 0, 0, 0],
            direction: [true, true, true, true],
            initial_interval_us: 1000,
            final_interval_us: 1000,
            accel_steps_s2: 0.0,
            duration_us: 100_000,
            phase: SegmentPhase::Cruise,
        };

        let mut driver = MockStepper::new();
        let mut intervals = Vec::new();

        let result = execute_segment(&segment, &mut driver, |us| intervals.push(us));

        assert_eq!(result.total_steps, 100);
        assert_eq!(driver.steps[0], 100);
        assert_eq!(driver.steps[1], 0);
        assert_eq!(intervals.len(), 100);
        // Cruise: all intervals should be 1000us
        assert!(intervals.iter().all(|&i| i == 1000));
    }

    #[test]
    fn test_multi_axis_bresenham() {
        // Move X=100, Y=50: Y should step every other master step
        let segment = MotionSegment {
            steps: [100, 50, 0, 0],
            direction: [true, true, true, true],
            initial_interval_us: 500,
            final_interval_us: 500,
            accel_steps_s2: 0.0,
            duration_us: 50_000,
            phase: SegmentPhase::Cruise,
        };

        let mut driver = MockStepper::new();
        let result = execute_segment(&segment, &mut driver, |_| {});

        assert_eq!(result.total_steps, 100);
        assert_eq!(driver.steps[0], 100);
        assert_eq!(driver.steps[1], 50);
    }

    #[test]
    fn test_negative_direction() {
        let segment = MotionSegment {
            steps: [-80, 40, 0, 0],
            direction: [false, true, true, true],
            initial_interval_us: 200,
            final_interval_us: 200,
            accel_steps_s2: 0.0,
            duration_us: 16_000,
            phase: SegmentPhase::Cruise,
        };

        let mut driver = MockStepper::new();
        let result = execute_segment(&segment, &mut driver, |_| {});

        assert_eq!(result.total_steps, 80);
        assert_eq!(driver.steps[0], -80);
        assert_eq!(driver.steps[1], 40);
    }

    #[test]
    fn test_acceleration_intervals_decrease() {
        let segment = MotionSegment {
            steps: [50, 0, 0, 0],
            direction: [true, true, true, true],
            initial_interval_us: 2000, // slow start
            final_interval_us: 200,    // fast end
            accel_steps_s2: 1000.0,
            duration_us: 55_000,
            phase: SegmentPhase::Accelerate,
        };

        let mut driver = MockStepper::new();
        let mut intervals = Vec::new();

        execute_segment(&segment, &mut driver, |us| intervals.push(us));

        assert_eq!(driver.steps[0], 50);
        // Intervals should be decreasing (accelerating)
        assert!(intervals.first().unwrap() > intervals.last().unwrap());
    }

    #[test]
    fn test_empty_segment() {
        let segment = MotionSegment {
            steps: [0, 0, 0, 0],
            direction: [true; 4],
            initial_interval_us: 1000,
            final_interval_us: 1000,
            accel_steps_s2: 0.0,
            duration_us: 0,
            phase: SegmentPhase::Cruise,
        };

        let mut driver = MockStepper::new();
        let result = execute_segment(&segment, &mut driver, |_| {});

        assert_eq!(result.total_steps, 0);
        assert_eq!(driver.steps, [0; 4]);
    }
}
