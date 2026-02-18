//! PWM output abstraction for heaters and fans.
//!
//! Maps the 7 PWM-capable outputs on the Duet 3 Mini 5+
//! to logical heater and fan channels.

/// Identifies a PWM output channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum PwmChannel {
    /// OUT_0: Heated bed (15A fused)
    Bed,
    /// OUT_1: Hotend heater 1
    Heater1,
    /// OUT_2: Hotend heater 2
    Heater2,
    /// OUT_3: Fan 0 (heatbreak)
    Fan0,
    /// OUT_4: Fan 1 (part cooling)
    Fan1,
    /// OUT_5: Fan 2 (aux)
    Fan2,
    /// OUT_6: Fan 3 / laser / VFD
    Fan3,
}

/// Duty cycle as a fraction 0.0 to 1.0.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct DutyCycle(f32);

impl DutyCycle {
    pub fn new(value: f32) -> Self {
        Self(value.clamp(0.0, 1.0))
    }

    pub fn off() -> Self {
        Self(0.0)
    }

    pub fn full() -> Self {
        Self(1.0)
    }

    pub fn fraction(self) -> f32 {
        self.0
    }

    /// Convert to a timer compare value for a given period.
    pub fn to_compare(self, period: u32) -> u32 {
        (self.0 * period as f32) as u32
    }
}
