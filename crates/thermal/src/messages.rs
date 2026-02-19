//! Messages for the thermal manager actor.

use crate::manager::HeaterFault;
use printer_hal::{PwmChannel, TempChannel};

/// Commands sent TO the thermal manager.
#[derive(Clone, Debug, defmt::Format)]
pub enum ThermalCommand {
    /// Set target temperature for a heater (no wait).
    SetTarget { channel: TempChannel, temp_c: f32 },

    /// Set target temperature and signal when reached.
    SetTargetAndWait { channel: TempChannel, temp_c: f32 },

    /// Turn off a heater.
    HeaterOff { channel: TempChannel },

    /// Clear a latched heater fault so a new target can be set.
    ClearFault { channel: TempChannel },

    /// Set fan speed (0.0 - 1.0).
    SetFanSpeed { channel: PwmChannel, speed: f32 },

    /// Turn fan off.
    FanOff { channel: PwmChannel },

    /// Report current temperatures.
    ReportTemperatures,

    /// Emergency shutdown — turn off all heaters.
    EmergencyStop,
}

/// Status updates FROM the thermal manager.
#[derive(Clone, Debug, defmt::Format)]
pub enum ThermalStatus {
    /// Current temperature reading.
    Temperature {
        channel: TempChannel,
        current_c: f32,
        target_c: f32,
        pwm: f32,
    },

    /// Target temperature has been reached.
    TargetReached { channel: TempChannel },

    /// Thermal runaway detected.
    ThermalRunaway { channel: TempChannel, temp_c: f32 },

    /// Thermistor fault (open/short).
    SensorFault { channel: TempChannel },

    /// A heater safety limit was violated.  The heater is locked out
    /// until a `ClearFault` command is received.
    HeaterFaulted {
        channel: TempChannel,
        fault: HeaterFault,
    },
}
