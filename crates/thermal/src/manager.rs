//! Thermal manager — owns all heater/fan state and runs PID loops.

use crate::pid::PidController;
use board_hal::thermistor::TempChannel;
use board_hal::pwm_output::{PwmChannel, DutyCycle};

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

impl ThermalManager {
    pub fn new() -> Self {
        Self {
            heaters: [
                HeaterState::new(PidController::bed_default()),     // Bed
                HeaterState::new(PidController::hotend_default()),  // Hotend 1
                HeaterState::new(PidController::hotend_default()),  // Hotend 2
            ],
            fan_speeds: [DutyCycle::off(); NUM_FANS],
        }
    }

    /// Map a TempChannel to a heater index.
    fn heater_index(channel: TempChannel) -> usize {
        match channel {
            TempChannel::Bed => 0,
            TempChannel::Hotend1 => 1,
            TempChannel::Hotend2 => 2,
        }
    }

    /// Map a PwmChannel to a fan index. Returns None for heater channels.
    fn fan_index(channel: PwmChannel) -> Option<usize> {
        match channel {
            PwmChannel::Fan0 => Some(0),
            PwmChannel::Fan1 => Some(1),
            PwmChannel::Fan2 => Some(2),
            PwmChannel::Fan3 => Some(3),
            _ => None,
        }
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
        heater.pwm_output.fraction() > 0.95
            && (heater.target_c - heater.current_c) > max_deviation
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
