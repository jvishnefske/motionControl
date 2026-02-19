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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_positive_error_gives_positive_output() {
        let mut pid = PidController::new(1.0, 0.0, 0.0);
        let out = pid.update(20.0, 200.0, 0.1);
        assert!(out > 0.0);
    }

    #[test]
    fn test_pid_at_target_gives_zero() {
        let mut pid = PidController::new(1.0, 0.0, 0.0);
        let out = pid.update(200.0, 200.0, 0.1);
        assert!((out - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_pid_output_clamped_to_1() {
        let mut pid = PidController::new(100.0, 0.0, 0.0);
        let out = pid.update(0.0, 200.0, 0.1);
        assert!((out - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_pid_output_clamped_to_0() {
        let mut pid = PidController::new(100.0, 0.0, 0.0);
        let out = pid.update(300.0, 200.0, 0.1); // overshoot
        assert!((out - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_pid_integral_accumulates() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        pid.update(190.0, 200.0, 0.1); // error=10, integral=1.0
        let out = pid.update(195.0, 200.0, 0.1); // error=5, integral=1.5
        assert!(out > 0.0);
    }

    #[test]
    fn test_pid_reset_clears_state() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        pid.update(100.0, 200.0, 0.1);
        pid.reset();
        let out = pid.update(200.0, 200.0, 0.1); // at target, no integral
        assert!((out - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_pid_converges_towards_target() {
        let mut pid = PidController::hotend_default();
        let mut temp = 25.0;
        for _ in 0..100 {
            let duty = pid.update(temp, 200.0, 0.1);
            // Simulated heating: temp increases proportional to duty
            temp += duty * 5.0;
        }
        assert!((temp - 200.0).abs() < 10.0, "temp={}", temp);
    }
}
