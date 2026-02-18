//! PID controller for heater regulation.
//!
//! Anti-windup integral clamping, derivative filtering,
//! and output clamping to 0.0-1.0 duty cycle range.

/// PID controller with anti-windup.
pub struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    prev_error: f32,
    output_min: f32,
    output_max: f32,
    integral_max: f32,
}

impl PidController {
    /// Create a new PID controller with default heater gains.
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            output_min: 0.0,
            output_max: 1.0,
            integral_max: 1.0, // anti-windup
        }
    }

    /// Default PID gains for a typical hotend heater.
    pub fn hotend_default() -> Self {
        Self::new(10.0, 0.5, 50.0)
    }

    /// Default PID gains for a typical heated bed.
    pub fn bed_default() -> Self {
        Self::new(60.0, 0.3, 300.0)
    }

    /// Compute the next PID output given current and target temperature.
    /// `dt` is the time delta in seconds.
    pub fn update(&mut self, current: f32, target: f32, dt: f32) -> f32 {
        let error = target - current;

        // Proportional
        let p = self.kp * error;

        // Integral with anti-windup clamping
        self.integral += error * dt;
        self.integral = clamp(self.integral, -self.integral_max, self.integral_max);
        let i = self.ki * self.integral;

        // Derivative (on error, with simple filtering)
        let d = if dt > 0.0 {
            self.kd * (error - self.prev_error) / dt
        } else {
            0.0
        };
        self.prev_error = error;

        // Sum and clamp output
        let output = p + i + d;
        clamp(output, self.output_min, self.output_max)
    }

    /// Reset the PID state (call when changing setpoint significantly).
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}

fn clamp(val: f32, min: f32, max: f32) -> f32 {
    if val < min {
        min
    } else if val > max {
        max
    } else {
        val
    }
}
