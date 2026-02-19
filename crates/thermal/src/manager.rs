//! Thermal manager — owns all heater/fan state and runs PID loops.
//!
//! Each heater has per-channel safety limits. When a limit is violated the
//! heater enters a latched fault state (PWM forced to 0) until explicitly
//! cleared by a `ClearFault` command.

use crate::pid::PidController;
use printer_hal::{DutyCycle, PwmChannel, TempChannel};

/// Number of heater channels.
const NUM_HEATERS: usize = 3;
/// Number of fan channels.
const NUM_FANS: usize = 4;

// ── Safety limits ────────────────────────────────────────────────

/// Per-heater safety limits.  Chosen so that a single stuck MOSFET,
/// disconnected thermistor, or runaway PID loop is caught before the
/// printer sustains damage.
#[derive(Clone, Debug)]
pub struct HeaterSafetyLimits {
    /// Absolute maximum temperature (C).  Exceeding this instantly faults
    /// the heater.  Set below the damage threshold of the weakest
    /// component in the thermal path (PTFE liner, bed adhesive, wiring).
    pub max_temp_c: f32,

    /// Absolute minimum plausible temperature (C).  A reading below this
    /// indicates an open or shorted thermistor.
    pub min_temp_c: f32,

    /// Maximum heating rate (C/s).  A sustained rise faster than this
    /// suggests a stuck-on MOSFET or grossly wrong PID gains.
    pub max_rise_rate_c_per_s: f32,

    /// Maximum cooling rate (C/s, positive value).  A temperature drop
    /// faster than thermal mass allows means the sensor disconnected or
    /// its wiring is intermittent.
    pub max_fall_rate_c_per_s: f32,

    /// If the heater is enabled and the temperature hasn't risen by at
    /// least 2 C within this many seconds, the heater is considered
    /// broken (open element, blown fuse, loose connector).
    pub heating_timeout_s: f32,

    /// Existing runaway check threshold: fault if PWM > 95% and the
    /// temperature is more than this many degrees below target.
    pub runaway_deviation_c: f32,
}

impl HeaterSafetyLimits {
    /// Safe defaults for a heated bed (large thermal mass, slow response).
    ///
    /// - 130 C max: above this PEI/adhesive sheets delaminate and bed
    ///   wiring insulation softens.
    /// - 3 C/s max rise: a 200 W bed on a 300 mm plate rises ~1-2 C/s;
    ///   anything faster means the sensor is wrong or the MOSFET is stuck.
    /// - 120 s heating timeout: beds are slow — 60 C in ~60 s is typical.
    pub fn bed_default() -> Self {
        Self {
            max_temp_c: 130.0,
            min_temp_c: -10.0,
            max_rise_rate_c_per_s: 3.0,
            max_fall_rate_c_per_s: 5.0,
            heating_timeout_s: 120.0,
            runaway_deviation_c: 20.0,
        }
    }

    /// Safe defaults for a hotend heater (small thermal mass, fast response).
    ///
    /// - 285 C max: all-metal hotends are safe to ~300 C, but PTFE-lined
    ///   hotends release toxic fumes above ~260 C.  285 C leaves margin
    ///   for all-metal while still catching a runaway on PTFE.
    /// - 8 C/s max rise: a 40 W heater block rises ~4-5 C/s normally;
    ///   8 C/s catches stuck-on without false-tripping during PID overshoot.
    /// - 45 s heating timeout: a working hotend heater reaches 200 C from
    ///   ambient in ~30 s.
    pub fn hotend_default() -> Self {
        Self {
            max_temp_c: 285.0,
            min_temp_c: -10.0,
            max_rise_rate_c_per_s: 8.0,
            max_fall_rate_c_per_s: 10.0,
            heating_timeout_s: 45.0,
            runaway_deviation_c: 20.0,
        }
    }
}

// ── Fault tracking ───────────────────────────────────────────────

/// Reason a heater was faulted and locked out.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum HeaterFault {
    /// Temperature reading was NaN (open or shorted thermistor).
    SensorOpen,
    /// Temperature exceeded the absolute maximum limit.
    OverTemp,
    /// Temperature fell below the absolute minimum limit.
    UnderTemp,
    /// Temperature rose faster than `max_rise_rate_c_per_s`.
    RiseTooFast,
    /// Temperature fell faster than `max_fall_rate_c_per_s`.
    FallTooFast,
    /// Heater was enabled but temperature did not increase within the
    /// heating timeout window.
    HeatingTimeout,
    /// PWM at maximum but temperature far below target.
    ThermalRunaway,
}

// ── Per-heater state ─────────────────────────────────────────────

/// Per-heater state.
pub struct HeaterState {
    pub target_c: f32,
    pub current_c: f32,
    pub pid: PidController,
    pub enabled: bool,
    pub waiting: bool,
    pub pwm_output: DutyCycle,
    pub limits: HeaterSafetyLimits,
    /// Latched fault.  While `Some`, PWM is forced to 0 and the heater
    /// ignores `set_target` commands.  Cleared only by `clear_fault`.
    pub fault: Option<HeaterFault>,
    /// Previous temperature reading for rate-of-change detection.
    prev_c: f32,
    /// Set to `true` once we've recorded at least one reading.
    has_prev: bool,
    /// Accumulated seconds since enable with temperature not rising.
    heating_elapsed_s: f32,
    /// Temperature when the heater was enabled (for timeout check).
    baseline_c: f32,
    /// Whether the baseline has been captured for this heating cycle.
    baseline_set: bool,
}

impl HeaterState {
    fn new(pid: PidController, limits: HeaterSafetyLimits) -> Self {
        Self {
            target_c: 0.0,
            current_c: 0.0,
            pid,
            enabled: false,
            waiting: false,
            pwm_output: DutyCycle::off(),
            limits,
            fault: None,
            prev_c: 0.0,
            has_prev: false,
            heating_elapsed_s: 0.0,
            baseline_c: 0.0,
            baseline_set: false,
        }
    }
}

/// Minimum temperature rise (C) required within the heating timeout
/// window to prove the heater element is working.
const HEATING_MIN_RISE_C: f32 = 2.0;

// ── Manager ──────────────────────────────────────────────────────

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
                HeaterState::new(
                    PidController::bed_default(),
                    HeaterSafetyLimits::bed_default(),
                ),
                HeaterState::new(
                    PidController::hotend_default(),
                    HeaterSafetyLimits::hotend_default(),
                ),
                HeaterState::new(
                    PidController::hotend_default(),
                    HeaterSafetyLimits::hotend_default(),
                ),
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
    /// Ignored if the heater is in a faulted state.
    pub fn set_target(&mut self, channel: TempChannel, temp_c: f32) {
        let idx = Self::heater_index(channel);
        let heater = &mut self.heaters[idx];

        if heater.fault.is_some() {
            return;
        }

        if (temp_c - heater.target_c).abs() > 5.0 {
            heater.pid.reset();
        }

        let was_enabled = heater.enabled;
        heater.target_c = temp_c;
        heater.enabled = temp_c > 0.0;
        heater.waiting = false;

        // Reset heating timeout tracking when newly enabled.
        if heater.enabled && !was_enabled {
            heater.heating_elapsed_s = 0.0;
            heater.baseline_set = false;
        }
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

    /// Clear a latched fault and re-enable normal operation.
    /// The heater remains off — caller must issue a new `set_target`.
    pub fn clear_fault(&mut self, channel: TempChannel) {
        let idx = Self::heater_index(channel);
        let heater = &mut self.heaters[idx];
        heater.fault = None;
        heater.target_c = 0.0;
        heater.enabled = false;
        heater.waiting = false;
        heater.pwm_output = DutyCycle::off();
        heater.pid.reset();
        heater.has_prev = false;
        heater.heating_elapsed_s = 0.0;
        heater.baseline_set = false;
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

    /// Latch a fault on a heater — force PWM to 0 and lock out until
    /// explicitly cleared.
    fn latch_fault(&mut self, channel: TempChannel, fault: HeaterFault) {
        let idx = Self::heater_index(channel);
        let heater = &mut self.heaters[idx];
        heater.fault = Some(fault);
        heater.pwm_output = DutyCycle::off();
        heater.enabled = false;
        heater.waiting = false;
        heater.pid.reset();
    }

    /// Update a heater's current temperature reading and run PID.
    /// `dt` is the time since last update in seconds.
    /// Returns the new PWM duty cycle.
    pub fn update_heater(&mut self, channel: TempChannel, current_c: f32, dt: f32) -> DutyCycle {
        let idx = Self::heater_index(channel);
        let heater = &mut self.heaters[idx];

        heater.current_c = current_c;

        // Already faulted — stay at 0.
        if heater.fault.is_some() {
            heater.pwm_output = DutyCycle::off();
            return heater.pwm_output;
        }

        if !heater.enabled || current_c.is_nan() {
            heater.pwm_output = DutyCycle::off();
            return heater.pwm_output;
        }

        let output = heater.pid.update(current_c, heater.target_c, dt);
        heater.pwm_output = DutyCycle::new(output);
        heater.pwm_output
    }

    /// Run all safety checks on a heater.  Returns `Some(fault)` if a
    /// limit was violated and the heater has been latched off.
    ///
    /// Call once per PID tick, *after* `update_heater`.
    pub fn check_safety(&mut self, channel: TempChannel, dt: f32) -> Option<HeaterFault> {
        let idx = Self::heater_index(channel);
        let heater = &self.heaters[idx];

        // Skip checks on already-faulted or disabled heaters.
        if heater.fault.is_some() {
            return heater.fault;
        }

        let current = heater.current_c;

        // ── Sensor fault: NaN ────────────────────────────────────
        if current.is_nan() {
            self.latch_fault(channel, HeaterFault::SensorOpen);
            return Some(HeaterFault::SensorOpen);
        }

        // ── Absolute temperature limits (checked even if heater is off) ──
        if current > heater.limits.max_temp_c {
            self.latch_fault(channel, HeaterFault::OverTemp);
            return Some(HeaterFault::OverTemp);
        }
        if current < heater.limits.min_temp_c {
            self.latch_fault(channel, HeaterFault::UnderTemp);
            return Some(HeaterFault::UnderTemp);
        }

        // ── Rate-of-change checks (need at least two readings) ──
        if heater.has_prev {
            let delta = current - heater.prev_c;
            let rate = delta / dt;
            if rate > heater.limits.max_rise_rate_c_per_s {
                self.latch_fault(channel, HeaterFault::RiseTooFast);
                return Some(HeaterFault::RiseTooFast);
            }
            if -rate > heater.limits.max_fall_rate_c_per_s {
                self.latch_fault(channel, HeaterFault::FallTooFast);
                return Some(HeaterFault::FallTooFast);
            }
        }

        // Update previous reading *after* rate check.
        let heater = &mut self.heaters[idx];
        heater.prev_c = current;
        heater.has_prev = true;

        // The remaining checks only apply while the heater is actively heating.
        if !heater.enabled {
            return None;
        }

        // ── Heating timeout ──────────────────────────────────────
        // Record baseline on first tick after enable.
        if !heater.baseline_set {
            heater.baseline_c = current;
            heater.baseline_set = true;
        }
        heater.heating_elapsed_s += dt;
        if heater.heating_elapsed_s >= heater.limits.heating_timeout_s
            && (current - heater.baseline_c) < HEATING_MIN_RISE_C
        {
            self.latch_fault(channel, HeaterFault::HeatingTimeout);
            return Some(HeaterFault::HeatingTimeout);
        }

        // ── Thermal runaway (PWM maxed, temp far below target) ──
        // Only checked after the initial heat-up window — during startup
        // it's normal for PWM to be at 100% with a large deviation.
        // The heating timeout check above catches a dead heater element.
        let heater = &self.heaters[idx];
        if heater.heating_elapsed_s > heater.limits.heating_timeout_s
            && heater.pwm_output.fraction() > 0.95
            && (heater.target_c - current) > heater.limits.runaway_deviation_c
        {
            self.latch_fault(channel, HeaterFault::ThermalRunaway);
            return Some(HeaterFault::ThermalRunaway);
        }

        None
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

    /// Legacy runaway check — prefer `check_safety` which covers all fault modes.
    pub fn check_runaway(&self, channel: TempChannel, max_deviation: f32) -> bool {
        let idx = Self::heater_index(channel);
        let heater = &self.heaters[idx];

        if !heater.enabled {
            return false;
        }

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

    // ── Existing tests ───────────────────────────────────────────

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

    // ── Safety check tests ───────────────────────────────────────

    #[test]
    fn test_sensor_nan_faults_heater() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);
        mgr.update_heater(TempChannel::Hotend1, f32::NAN, 0.1);
        let fault = mgr.check_safety(TempChannel::Hotend1, 0.1);
        assert_eq!(fault, Some(HeaterFault::SensorOpen));
        assert!(mgr.heaters[1].fault.is_some());
        assert_eq!(mgr.heaters[1].pwm_output.fraction(), 0.0);
    }

    #[test]
    fn test_overtemp_faults_heater() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);
        // Simulate reading above hotend max (285 C)
        mgr.update_heater(TempChannel::Hotend1, 290.0, 0.1);
        let fault = mgr.check_safety(TempChannel::Hotend1, 0.1);
        assert_eq!(fault, Some(HeaterFault::OverTemp));
    }

    #[test]
    fn test_overtemp_bed_limit() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Bed, 100.0);
        // Bed max is 130 C
        mgr.update_heater(TempChannel::Bed, 131.0, 0.1);
        let fault = mgr.check_safety(TempChannel::Bed, 0.1);
        assert_eq!(fault, Some(HeaterFault::OverTemp));
    }

    #[test]
    fn test_undertemp_faults_heater() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);
        mgr.update_heater(TempChannel::Hotend1, -15.0, 0.1);
        let fault = mgr.check_safety(TempChannel::Hotend1, 0.1);
        assert_eq!(fault, Some(HeaterFault::UnderTemp));
    }

    #[test]
    fn test_rise_too_fast_faults() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);

        // First reading: establish baseline at 25 C
        mgr.update_heater(TempChannel::Hotend1, 25.0, 0.1);
        assert_eq!(mgr.check_safety(TempChannel::Hotend1, 0.1), None);

        // Second reading: jump to 26 C in 0.1 s = 10 C/s (exceeds 8 C/s limit)
        mgr.update_heater(TempChannel::Hotend1, 26.0, 0.1);
        let fault = mgr.check_safety(TempChannel::Hotend1, 0.1);
        assert_eq!(fault, Some(HeaterFault::RiseTooFast));
    }

    #[test]
    fn test_normal_rise_rate_ok() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);

        // 0.5 C in 0.1 s = 5.0 C/s — within hotend limit of 8 C/s
        mgr.update_heater(TempChannel::Hotend1, 25.0, 0.1);
        assert_eq!(mgr.check_safety(TempChannel::Hotend1, 0.1), None);

        mgr.update_heater(TempChannel::Hotend1, 25.5, 0.1);
        assert_eq!(mgr.check_safety(TempChannel::Hotend1, 0.1), None);
    }

    #[test]
    fn test_fall_too_fast_faults() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);

        // First reading at 200 C
        mgr.update_heater(TempChannel::Hotend1, 200.0, 0.1);
        assert_eq!(mgr.check_safety(TempChannel::Hotend1, 0.1), None);

        // Drop 2 C in 0.1 s = 20 C/s fall (exceeds 10 C/s limit)
        mgr.update_heater(TempChannel::Hotend1, 198.0, 0.1);
        let fault = mgr.check_safety(TempChannel::Hotend1, 0.1);
        assert_eq!(fault, Some(HeaterFault::FallTooFast));
    }

    #[test]
    fn test_heating_timeout_faults() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);

        // Simulate 46 seconds at constant 25 C (hotend timeout = 45 s)
        for _ in 0..460 {
            mgr.update_heater(TempChannel::Hotend1, 25.0, 0.1);
            let fault = mgr.check_safety(TempChannel::Hotend1, 0.1);
            if fault.is_some() {
                assert_eq!(fault, Some(HeaterFault::HeatingTimeout));
                return;
            }
        }
        panic!("Expected HeatingTimeout fault");
    }

    #[test]
    fn test_heating_timeout_not_triggered_when_rising() {
        let mut mgr = ThermalManager::new();
        // Use a low target so that after heat-up the runaway deviation
        // check (20 C) doesn't fire when we cross the timeout boundary.
        mgr.set_target(TempChannel::Hotend1, 35.0);

        // Temperature rises from 25 C to 28 C over 46 seconds.
        // That's > 2 C rise, proving the heater is alive.
        // Final deviation from target = 7 C (< 20 C runaway threshold).
        for i in 0..460 {
            let temp = 25.0 + (i as f32 * 3.0 / 460.0);
            mgr.update_heater(TempChannel::Hotend1, temp, 0.1);
            let fault = mgr.check_safety(TempChannel::Hotend1, 0.1);
            assert_eq!(
                fault, None,
                "unexpected fault at tick {}, temp={:.1}",
                i, temp
            );
        }
    }

    #[test]
    fn test_bed_heating_timeout_longer() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Bed, 60.0);

        // 50 seconds at constant temp — bed timeout is 120 s, should not fault yet
        for _ in 0..500 {
            mgr.update_heater(TempChannel::Bed, 25.0, 0.1);
            let fault = mgr.check_safety(TempChannel::Bed, 0.1);
            assert_eq!(fault, None);
        }
    }

    #[test]
    fn test_fault_latches_until_cleared() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);

        // Trigger overtemp
        mgr.update_heater(TempChannel::Hotend1, 290.0, 0.1);
        mgr.check_safety(TempChannel::Hotend1, 0.1);

        // set_target is ignored while faulted
        mgr.set_target(TempChannel::Hotend1, 100.0);
        assert!(!mgr.heaters[1].enabled);
        assert_eq!(mgr.heaters[1].fault, Some(HeaterFault::OverTemp));

        // update_heater returns 0 while faulted
        let duty = mgr.update_heater(TempChannel::Hotend1, 25.0, 0.1);
        assert_eq!(duty.fraction(), 0.0);

        // Clear fault — heater is off, ready for new target
        mgr.clear_fault(TempChannel::Hotend1);
        assert!(mgr.heaters[1].fault.is_none());
        assert!(!mgr.heaters[1].enabled);

        // Now set_target works again
        mgr.set_target(TempChannel::Hotend1, 200.0);
        assert!(mgr.heaters[1].enabled);
    }

    #[test]
    fn test_normal_operation_no_fault() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);

        // Simulate a realistic heat-up: ~5 C/s (within 8 C/s limit)
        let mut temp = 25.0;
        for _ in 0..350 {
            temp += 0.5; // 0.5 C per 0.1 s = 5 C/s
            if temp > 200.0 {
                temp = 200.0;
            }
            mgr.update_heater(TempChannel::Hotend1, temp, 0.1);
            let fault = mgr.check_safety(TempChannel::Hotend1, 0.1);
            assert_eq!(fault, None, "unexpected fault at temp={}", temp);
        }
    }

    #[test]
    fn test_overtemp_checked_even_when_disabled() {
        let mut mgr = ThermalManager::new();
        // Heater is not enabled, but sensor reads dangerously high
        mgr.update_heater(TempChannel::Hotend1, 300.0, 0.1);
        let fault = mgr.check_safety(TempChannel::Hotend1, 0.1);
        assert_eq!(fault, Some(HeaterFault::OverTemp));
    }

    #[test]
    fn test_thermal_runaway_via_check_safety() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 250.0);

        // Simulate past the heating timeout with rising temp (so timeout
        // doesn't trip).  Hotend timeout is 45 s.
        let mut temp = 25.0;
        for _ in 0..460 {
            temp += 0.5; // 5 C/s, well within 8 C/s limit
            if temp > 200.0 {
                temp = 200.0;
            }
            mgr.update_heater(TempChannel::Hotend1, temp, 0.1);
            mgr.check_safety(TempChannel::Hotend1, 0.1);
        }

        // Now force PWM to max and temp far below target — should trigger
        // runaway since we're past the heating timeout grace period.
        mgr.heaters[1].pwm_output = DutyCycle::full();
        mgr.heaters[1].current_c = 100.0;
        let fault = mgr.check_safety(TempChannel::Hotend1, 0.1);
        assert_eq!(fault, Some(HeaterFault::ThermalRunaway));
    }

    #[test]
    fn test_clear_fault_resets_rate_tracking() {
        let mut mgr = ThermalManager::new();
        mgr.set_target(TempChannel::Hotend1, 200.0);

        // Trigger a fault
        mgr.update_heater(TempChannel::Hotend1, 290.0, 0.1);
        mgr.check_safety(TempChannel::Hotend1, 0.1);
        assert!(mgr.heaters[1].fault.is_some());

        // Clear and verify rate tracking is reset
        mgr.clear_fault(TempChannel::Hotend1);
        assert!(!mgr.heaters[1].has_prev);
        assert_eq!(mgr.heaters[1].heating_elapsed_s, 0.0);
    }
}
