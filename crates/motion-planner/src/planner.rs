//! Core motion planner.
//!
//! Converts target positions and feedrates into trapezoidal motion profiles.
//! Handles coordinate system state (absolute/relative), axis limits, and
//! multi-axis step coordination.

use crate::segment::{MotionSegment, SegmentPhase};
use gcode_parser::{AxisValues, PositionMode};

/// Maximum number of segments in the motion queue.
const QUEUE_DEPTH: usize = 32;

/// The motion planner state machine.
pub struct MotionPlanner {
    /// Current position in steps for each axis.
    position_steps: [i64; 4],

    /// Steps per mm for each axis.
    steps_per_mm: [f32; 4],

    /// Max feedrate in mm/min for each axis.
    max_feedrate: [f32; 4],

    /// Max acceleration in mm/s^2 for each axis.
    max_accel: [f32; 4],

    /// Default print acceleration mm/s^2.
    print_accel: f32,

    /// Default travel acceleration mm/s^2.
    travel_accel: f32,

    /// Current positioning mode.
    mode: PositionMode,

    /// Active feedrate in mm/min (persists across moves).
    active_feedrate: f32,

    /// Whether axes are homed.
    homed: [bool; 4],

    /// Pending segments.
    queue: heapless::Deque<MotionSegment, QUEUE_DEPTH>,
}

impl Default for MotionPlanner {
    fn default() -> Self {
        Self::new()
    }
}

impl MotionPlanner {
    pub fn new() -> Self {
        Self {
            position_steps: [0; 4],
            steps_per_mm: [80.0, 80.0, 400.0, 420.0],
            max_feedrate: [6000.0, 6000.0, 600.0, 3600.0],
            max_accel: [500.0, 500.0, 100.0, 500.0],
            print_accel: 500.0,
            travel_accel: 1000.0,
            mode: PositionMode::Absolute,
            active_feedrate: 3000.0,
            homed: [false; 4],
            queue: heapless::Deque::new(),
        }
    }

    pub fn set_absolute(&mut self) {
        self.mode = PositionMode::Absolute;
    }

    pub fn set_relative(&mut self) {
        self.mode = PositionMode::Relative;
    }

    pub fn set_steps_per_mm(&mut self, axes: &AxisValues) {
        if let Some(v) = axes.x {
            self.steps_per_mm[0] = v;
        }
        if let Some(v) = axes.y {
            self.steps_per_mm[1] = v;
        }
        if let Some(v) = axes.z {
            self.steps_per_mm[2] = v;
        }
        if let Some(v) = axes.e {
            self.steps_per_mm[3] = v;
        }
    }

    pub fn set_max_feedrate(&mut self, axes: &AxisValues) {
        if let Some(v) = axes.x {
            self.max_feedrate[0] = v;
        }
        if let Some(v) = axes.y {
            self.max_feedrate[1] = v;
        }
        if let Some(v) = axes.z {
            self.max_feedrate[2] = v;
        }
        if let Some(v) = axes.e {
            self.max_feedrate[3] = v;
        }
    }

    pub fn set_max_accel(&mut self, axes: &AxisValues) {
        if let Some(v) = axes.x {
            self.max_accel[0] = v;
        }
        if let Some(v) = axes.y {
            self.max_accel[1] = v;
        }
        if let Some(v) = axes.z {
            self.max_accel[2] = v;
        }
        if let Some(v) = axes.e {
            self.max_accel[3] = v;
        }
    }

    pub fn set_acceleration(&mut self, print_accel: Option<f32>, travel_accel: Option<f32>) {
        if let Some(a) = print_accel {
            self.print_accel = a;
        }
        if let Some(a) = travel_accel {
            self.travel_accel = a;
        }
    }

    pub fn set_position(&mut self, axes: &AxisValues) {
        if let Some(v) = axes.x {
            self.position_steps[0] = (v * self.steps_per_mm[0]) as i64;
        }
        if let Some(v) = axes.y {
            self.position_steps[1] = (v * self.steps_per_mm[1]) as i64;
        }
        if let Some(v) = axes.z {
            self.position_steps[2] = (v * self.steps_per_mm[2]) as i64;
        }
        if let Some(v) = axes.e {
            self.position_steps[3] = (v * self.steps_per_mm[3]) as i64;
        }
    }

    pub fn position_mm(&self) -> [f32; 4] {
        [
            self.position_steps[0] as f32 / self.steps_per_mm[0],
            self.position_steps[1] as f32 / self.steps_per_mm[1],
            self.position_steps[2] as f32 / self.steps_per_mm[2],
            self.position_steps[3] as f32 / self.steps_per_mm[3],
        ]
    }

    pub fn is_homed(&self) -> bool {
        self.homed[0] && self.homed[1] && self.homed[2]
    }

    pub fn mark_homed(&mut self, x: bool, y: bool, z: bool) {
        if x {
            self.homed[0] = true;
            self.position_steps[0] = 0;
        }
        if y {
            self.homed[1] = true;
            self.position_steps[1] = 0;
        }
        if z {
            self.homed[2] = true;
            self.position_steps[2] = 0;
        }
    }

    /// Plan a linear move and push segments into the queue.
    /// Returns the number of segments generated.
    pub fn plan_linear_move(
        &mut self,
        target: &AxisValues,
        feedrate_mm_min: Option<f32>,
        is_extrusion: bool,
    ) -> usize {
        if let Some(f) = feedrate_mm_min {
            self.active_feedrate = f;
        }

        // Compute target position in steps
        let target_steps = self.compute_target_steps(target);

        // Compute delta steps for each axis
        let mut delta = [0i32; 4];
        let mut direction = [true; 4];
        let mut max_delta: i32 = 0;

        for i in 0..4 {
            let d = target_steps[i] - self.position_steps[i];
            delta[i] = d as i32;
            direction[i] = d >= 0;
            let abs_d = if d >= 0 { d } else { -d } as i32;
            if abs_d > max_delta {
                max_delta = abs_d;
            }
        }

        if max_delta == 0 {
            return 0;
        }

        // Compute distance in mm
        let distance_mm = self.compute_distance_mm(&delta);
        if distance_mm < 0.001 {
            return 0;
        }

        // Clamp feedrate to per-axis limits
        let feedrate_mm_s = self.clamp_feedrate(self.active_feedrate / 60.0, &delta, distance_mm);

        // Select acceleration
        let accel = if is_extrusion {
            self.print_accel
        } else {
            self.travel_accel
        };
        let accel = self.clamp_acceleration(accel, &delta, distance_mm);

        // Generate trapezoidal profile
        let segments_added =
            self.generate_trapezoid(delta, direction, feedrate_mm_s, accel, distance_mm);

        // Update position
        self.position_steps.copy_from_slice(&target_steps);

        segments_added
    }

    /// Pop the next segment from the queue.
    pub fn next_segment(&mut self) -> Option<MotionSegment> {
        self.queue.pop_front()
    }

    /// Check if there are pending segments.
    pub fn has_pending(&self) -> bool {
        !self.queue.is_empty()
    }

    /// Emergency stop — clear all queued segments.
    pub fn emergency_stop(&mut self) {
        self.queue.clear();
    }

    // ── Private ───────────────────────────────────────────────────

    fn compute_target_steps(&self, target: &AxisValues) -> [i64; 4] {
        let mut result = self.position_steps;

        let apply = |current: i64, target_val: Option<f32>, spm: f32, mode: PositionMode| -> i64 {
            match target_val {
                Some(v) => match mode {
                    PositionMode::Absolute => (v * spm) as i64,
                    PositionMode::Relative => current + (v * spm) as i64,
                },
                None => current,
            }
        };

        result[0] = apply(result[0], target.x, self.steps_per_mm[0], self.mode);
        result[1] = apply(result[1], target.y, self.steps_per_mm[1], self.mode);
        result[2] = apply(result[2], target.z, self.steps_per_mm[2], self.mode);
        result[3] = apply(result[3], target.e, self.steps_per_mm[3], self.mode);

        result
    }

    fn compute_distance_mm(&self, delta: &[i32; 4]) -> f32 {
        let mut sum_sq: f32 = 0.0;
        // XYZ only for distance
        for (i, &d) in delta.iter().enumerate().take(3) {
            let d_mm = d as f32 / self.steps_per_mm[i];
            sum_sq += d_mm * d_mm;
        }
        libm::sqrtf(sum_sq)
    }

    fn clamp_feedrate(&self, desired_mm_s: f32, delta: &[i32; 4], distance_mm: f32) -> f32 {
        let mut max_speed = desired_mm_s;

        for (i, &d) in delta.iter().enumerate() {
            if d != 0 {
                let axis_distance = (d as f32 / self.steps_per_mm[i]).abs();
                let fraction = axis_distance / distance_mm;
                if fraction > 0.001 {
                    let axis_limit = self.max_feedrate[i] / 60.0; // convert to mm/s
                    let limit = axis_limit / fraction;
                    if limit < max_speed {
                        max_speed = limit;
                    }
                }
            }
        }

        max_speed
    }

    fn clamp_acceleration(&self, desired: f32, delta: &[i32; 4], distance_mm: f32) -> f32 {
        let mut accel = desired;

        for (i, &d) in delta.iter().enumerate() {
            if d != 0 {
                let axis_distance = (d as f32 / self.steps_per_mm[i]).abs();
                let fraction = axis_distance / distance_mm;
                if fraction > 0.001 {
                    let limit = self.max_accel[i] / fraction;
                    if limit < accel {
                        accel = limit;
                    }
                }
            }
        }

        accel
    }

    #[allow(clippy::too_many_arguments)]
    fn generate_trapezoid(
        &mut self,
        delta: [i32; 4],
        direction: [bool; 4],
        cruise_speed: f32,
        accel: f32,
        distance_mm: f32,
    ) -> usize {
        // Trapezoidal profile: accelerate -> cruise -> decelerate
        // Distance to reach cruise speed: d = v^2 / (2*a)
        let accel_distance = (cruise_speed * cruise_speed) / (2.0 * accel);
        let decel_distance = accel_distance;

        let mut segments_added = 0;

        if 2.0 * accel_distance >= distance_mm {
            // Triangular profile: not enough room to reach full speed
            let peak_speed = libm::sqrtf(accel * distance_mm);
            let half_distance = distance_mm / 2.0;

            // Acceleration phase
            if self.push_segment(
                &delta,
                &direction,
                half_distance,
                distance_mm,
                0.0,
                peak_speed,
                accel,
                SegmentPhase::Accelerate,
            ) {
                segments_added += 1;
            }

            // Deceleration phase
            if self.push_segment(
                &delta,
                &direction,
                half_distance,
                distance_mm,
                peak_speed,
                0.0,
                accel,
                SegmentPhase::Decelerate,
            ) {
                segments_added += 1;
            }
        } else {
            let cruise_distance = distance_mm - accel_distance - decel_distance;

            // Acceleration phase
            if self.push_segment(
                &delta,
                &direction,
                accel_distance,
                distance_mm,
                0.0,
                cruise_speed,
                accel,
                SegmentPhase::Accelerate,
            ) {
                segments_added += 1;
            }

            // Cruise phase
            if cruise_distance > 0.001
                && self.push_segment(
                    &delta,
                    &direction,
                    cruise_distance,
                    distance_mm,
                    cruise_speed,
                    cruise_speed,
                    0.0,
                    SegmentPhase::Cruise,
                )
            {
                segments_added += 1;
            }

            // Deceleration phase
            if self.push_segment(
                &delta,
                &direction,
                decel_distance,
                distance_mm,
                cruise_speed,
                0.0,
                accel,
                SegmentPhase::Decelerate,
            ) {
                segments_added += 1;
            }
        }

        segments_added
    }

    #[allow(clippy::too_many_arguments)]
    fn push_segment(
        &mut self,
        delta: &[i32; 4],
        direction: &[bool; 4],
        seg_distance: f32,
        total_distance: f32,
        initial_speed: f32,
        final_speed: f32,
        accel: f32,
        phase: SegmentPhase,
    ) -> bool {
        let fraction = seg_distance / total_distance;

        let mut steps = [0i32; 4];
        for (i, s) in steps.iter_mut().enumerate() {
            *s = (delta[i] as f32 * fraction) as i32;
        }

        // Compute step intervals (microseconds per step for dominant axis)
        let avg_speed = (initial_speed + final_speed) / 2.0;
        let duration_s = if avg_speed > 0.001 {
            seg_distance / avg_speed
        } else {
            0.0
        };
        let duration_us = (duration_s * 1_000_000.0) as u32;

        let initial_interval_us = if initial_speed > 0.001 {
            (1_000_000.0 / (initial_speed * self.steps_per_mm[0])) as u32
        } else {
            u32::MAX
        };

        let final_interval_us = if final_speed > 0.001 {
            (1_000_000.0 / (final_speed * self.steps_per_mm[0])) as u32
        } else {
            u32::MAX
        };

        let segment = MotionSegment {
            steps,
            direction: *direction,
            initial_interval_us,
            final_interval_us,
            accel_steps_s2: accel * self.steps_per_mm[0],
            duration_us,
            phase,
        };

        self.queue.push_back(segment).is_ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gcode_parser::AxisValues;

    fn axes(x: Option<f32>, y: Option<f32>, z: Option<f32>, e: Option<f32>) -> AxisValues {
        AxisValues { x, y, z, e }
    }

    #[test]
    fn test_plan_generates_segments() {
        let mut planner = MotionPlanner::new();
        let n = planner.plan_linear_move(&axes(Some(10.0), None, None, None), Some(3000.0), false);
        assert!(n > 0, "Expected segments, got {}", n);
    }

    #[test]
    fn test_plan_updates_position() {
        let mut planner = MotionPlanner::new();
        planner.plan_linear_move(
            &axes(Some(10.0), Some(20.0), None, None),
            Some(3000.0),
            false,
        );
        let pos = planner.position_mm();
        assert!((pos[0] - 10.0).abs() < 0.1);
        assert!((pos[1] - 20.0).abs() < 0.1);
    }

    #[test]
    fn test_zero_move_generates_nothing() {
        let mut planner = MotionPlanner::new();
        let n = planner.plan_linear_move(&axes(Some(0.0), None, None, None), Some(3000.0), false);
        assert_eq!(n, 0);
    }

    #[test]
    fn test_relative_mode() {
        let mut planner = MotionPlanner::new();
        planner.plan_linear_move(&axes(Some(10.0), None, None, None), Some(3000.0), false);
        planner.set_relative();
        planner.plan_linear_move(&axes(Some(5.0), None, None, None), Some(3000.0), false);
        let pos = planner.position_mm();
        assert!((pos[0] - 15.0).abs() < 0.1);
    }

    #[test]
    fn test_absolute_mode() {
        let mut planner = MotionPlanner::new();
        planner.plan_linear_move(&axes(Some(10.0), None, None, None), Some(3000.0), false);
        planner.plan_linear_move(&axes(Some(5.0), None, None, None), Some(3000.0), false);
        let pos = planner.position_mm();
        assert!((pos[0] - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_set_position() {
        let mut planner = MotionPlanner::new();
        planner.set_position(&axes(Some(100.0), Some(200.0), None, None));
        let pos = planner.position_mm();
        assert!((pos[0] - 100.0).abs() < 0.1);
        assert!((pos[1] - 200.0).abs() < 0.1);
    }

    #[test]
    fn test_homing() {
        let mut planner = MotionPlanner::new();
        assert!(!planner.is_homed());
        planner.mark_homed(true, true, true);
        assert!(planner.is_homed());
    }

    #[test]
    fn test_emergency_stop_clears_queue() {
        let mut planner = MotionPlanner::new();
        planner.plan_linear_move(&axes(Some(100.0), None, None, None), Some(3000.0), false);
        assert!(planner.has_pending());
        planner.emergency_stop();
        assert!(!planner.has_pending());
    }

    #[test]
    fn test_segments_are_consumable() {
        let mut planner = MotionPlanner::new();
        let n = planner.plan_linear_move(&axes(Some(10.0), None, None, None), Some(3000.0), false);
        let mut consumed = 0;
        while planner.next_segment().is_some() {
            consumed += 1;
        }
        assert_eq!(consumed, n);
    }

    #[test]
    fn test_set_steps_per_mm() {
        let mut planner = MotionPlanner::new();
        planner.set_steps_per_mm(&axes(Some(160.0), None, None, None));
        // Move 10mm = 1600 steps at 160 steps/mm
        planner.plan_linear_move(&axes(Some(10.0), None, None, None), Some(3000.0), false);
        let pos = planner.position_mm();
        assert!((pos[0] - 10.0).abs() < 0.1);
    }
}
