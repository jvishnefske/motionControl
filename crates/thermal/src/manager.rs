//! Thermal manager — owns all heater/fan state and runs PID loops.

use crate::pid::PidController;
use printer_hal::{DutyCycle, PwmChannel, TempChannel};

/// Number of heater channels.
const NUM_HEATERS: usize = 3;
/// Number of fan channels.
const NUM_FANS: usize = 4;

/// Per-heater state.
pub struct HeaterState {
    pub target_c: f32,
    pub current_c: f32,
    pub pid: PidController,
    pub enabled: bool,
    pub waiting: bool,
    pub pwm_output: DutyCycle,
}

impl HeaterState {
    fn new(pid: PidController) -> Self {
        Self {
            target_c: 0.0,
            current_c: 0.0,
            pid,
            enabled: false,
            waiting: false,
            pwm_output: DutyCycle::off(),
        }
    }
}

/// The thermal manager holds all heater and fan state.
pub struct ThermalManager {
    pub heaters: [HeaterState; NUM_HEATERS],
    pub fan_speeds: [DutyCycle; NUM_FANS],
}

impl Default for ThermalManager {
    fn default() -> Self {
        Self::new()
    }
}

impl ThermalManager {
    pub fn new() -> Self {
        Self {
            heaters: [
                HeaterState::new(PidController::bed_default()), // Bed
                HeaterState::new(PidController::hotend_default()), // Hotend 1
                HeaterState::new(PidController::hotend_default()), // Hotend 2
            ],
            fan_speeds: [DutyCycle::off(); NUM_FANS],
        }
    }

    fn heater_index(channel: TempChannel) -> usize {
        channel.index()
    }

    fn fan_index(channel: PwmChannel) -> Option<usize> {
        channel.fan_index()
    }

    /// Set the target temperature for a heater.
    pub fn set_target(&mut self, channel: TempChannel, temp_c: f32) {
        let idx = Self::heater_index(channel);
        let heater = &mut self.heaters[idx];

        if (temp_c - heater.target_c).abs() > 5.0 {
            heater.pid.reset();
        }

        heater.target_c = temp_c;
        heater.enabled = temp_c > 0.0;
        heater.waiting = false;
    }

    /// Set target and mark as waiting for temp to be reached.
    pub fn set_target_and_wait(&mut self, channel: TempChannel, temp_c: f32) {
        self.set_target(channel, temp_c);
        let idx = Self::heater_index(channel);
        self.heaters[idx].waiting = true;
    }

    /// Turn off a heater.
    pub fn heater_off(&mut self, channel: TempChannel) {
        let idx = Self::heater_index(channel);
        self.heaters[idx].target_c = 0.0;
        self.heaters[idx].enabled = false;
        self.heaters[idx].waiting = false;
        self.heaters[idx].pwm_output = DutyCycle::off();
        self.heaters[idx].pid.reset();
    }

    /// Set fan speed.
    pub fn set_fan_speed(&mut self, channel: PwmChannel, speed: f32) {
        if let Some(idx) = Self::fan_index(channel) {
            self.fan_speeds[idx] = DutyCycle::new(speed);
        }
    }

    /// Turn off a fan.
    pub fn fan_off(&mut self, channel: PwmChannel) {
        if let Some(idx) = Self::fan_index(channel) {
            self.fan_speeds[idx] = DutyCycle::off();
        }
    }

    /// Update a heater's current temperature reading and run PID.
    /// `dt` is the time since last update in seconds.
    /// Returns the new PWM duty cycle.
    pub fn update_heater(&mut self, channel: TempChannel, current_c: f32, dt: f32) -> DutyCycle {
        let idx = Self::heater_index(channel);
        let heater = &mut self.heaters[idx];

        heater.current_c = current_c;

        if !heater.enabled || current_c.is_nan() {
            heater.pwm_output = DutyCycle::off();
            return heater.pwm_output;
        }

        let output = heater.pid.update(current_c, heater.target_c, dt);
        heater.pwm_output = DutyCycle::new(output);
        heater.pwm_output
    }

    /// Check if a heater has reached its target temperature (within tolerance).
    pub fn is_at_target(&self, channel: TempChannel, tolerance: f32) -> bool {
        let idx = Self::heater_index(channel);
        let heater = &self.heaters[idx];

        if !heater.enabled {
            return true;
        }

        (heater.current_c - heater.target_c).abs() < tolerance
    }

    /// Check for thermal runaway (temperature diverging from target too far for too long).
    pub fn check_runaway(&self, channel: TempChannel, max_deviation: f32) -> bool {
        let idx = Self::heater_index(channel);
        let heater = &self.heaters[idx];

        if !heater.enabled {
            return false;
        }

        // Simple check: if heater is on full power but temp is far below target
        heater.pwm_output.fraction() > 0.95 && (heater.target_c - heater.current_c) > max_deviation
    }

    /// Emergency stop — turn off all heaters and fans.
    pub fn emergency_stop(&mut self) {
        for heater in &mut self.heaters {
            heater.target_c = 0.0;
            heater.enabled = false;
            heater.waiting = false;
            heater.pwm_output = DutyCycle::off();
            heater.pid.reset();
        }
        for fan in &mut self.fan_speeds {
            *fan = DutyCycle::off();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_set_target_enables_heater() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);
        assert!(mgr.heaters[1].enabled);
        assert_eq!(mgr.heaters[1].target_c, 200.0);
    }

    #[test]
    fn test_set_target_zero_disables() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);
        mgr.set_target(TempChannel::Hotend1, 0.0);
        assert!(!mgr.heaters[1].enabled);
    }

    #[test]
    fn test_set_target_and_wait() {
        let mut mgr = ThermalManager::new();
        mgr.set_target_and_wait(TempChannel::Bed, 60.0);
        assert!(mgr.heaters[0].waiting);
        assert!(mgr.heaters[0].enabled);
    }

    #[test]
    fn test_heater_off() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);
        mgr.heater_off(TempChannel::Hotend1);
        assert!(!mgr.heaters[1].enabled);
        assert_eq!(mgr.heaters[1].pwm_output.fraction(), 0.0);
    }

    #[test]
    fn test_fan_speed() {
        let mut mgr = ThermalManager::new();
        mgr.set_fan_speed(PwmChannel::Fan0, 0.75);
        assert!((mgr.fan_speeds[0].fraction() - 0.75).abs() < 0.001);
    }

    #[test]
    fn test_fan_off() {
        let mut mgr = ThermalManager::new();
        mgr.set_fan_speed(PwmChannel::Fan1, 1.0);
        mgr.fan_off(PwmChannel::Fan1);
        assert_eq!(mgr.fan_speeds[1].fraction(), 0.0);
    }

    #[test]
    fn test_update_heater_disabled_returns_zero() {
        let mut mgr = ThermalManager::new();
        let duty = mgr.update_heater(TempChannel::Hotend1, 25.0, 0.1);
        assert_eq!(duty.fraction(), 0.0);
    }

    #[test]
    fn test_update_heater_enabled_returns_nonzero() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);
        let duty = mgr.update_heater(TempChannel::Hotend1, 25.0, 0.1);
        assert!(duty.fraction() > 0.0);
    }

    #[test]
    fn test_is_at_target() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Bed, 60.0);
        mgr.heaters[0].current_c = 59.5;
        assert!(mgr.is_at_target(TempChannel::Bed, 2.0));
        mgr.heaters[0].current_c = 50.0;
        assert!(!mgr.is_at_target(TempChannel::Bed, 2.0));
    }

    #[test]
    fn test_emergency_stop() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);
        mgr.set_fan_speed(PwmChannel::Fan0, 1.0);
        mgr.emergency_stop();
        assert!(!mgr.heaters[1].enabled);
        assert_eq!(mgr.fan_speeds[0].fraction(), 0.0);
    }
}
