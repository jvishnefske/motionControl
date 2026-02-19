//! # Printer HAL — Platform-Agnostic Hardware Traits
//!
//! Defines the hardware abstraction boundary between portable printer logic
//! (motion planning, thermal control, file I/O) and platform-specific
//! implementations (Duet 3 firmware, WASM simulator, host tests).
//!
//! Library crates depend on these traits. Binary crates provide implementations.

#![no_std]

/// Stepper motor driver interface.
///
/// Called from the step generator at interrupt-level timing.
/// Implementations must be fast — no blocking I/O.
pub trait StepperDriver {
    /// Set direction for an axis (true = positive / forward).
    fn set_direction(&mut self, axis: u8, forward: bool);

    /// Pulse the step pin for an axis (rising edge triggers a step).
    fn step(&mut self, axis: u8);

    /// Enable or disable a stepper driver.
    fn enable(&mut self, axis: u8, enabled: bool);
}

/// Temperature sensor interface.
///
/// Reads from ADC-connected thermistors (or simulated sensors).
pub trait TemperatureSensor {
    /// Read current temperature in degrees Celsius for a channel.
    /// Returns `f32::NAN` on sensor fault (open/shorted thermistor).
    fn read_celsius(&mut self, channel: u8) -> f32;
}

/// PWM output for heaters and fans.
pub trait HeaterOutput {
    /// Set duty cycle for a heater channel (0.0 = off, 1.0 = full).
    fn set_heater_duty(&mut self, channel: u8, duty: f32);

    /// Set duty cycle for a fan channel (0.0 = off, 1.0 = full).
    fn set_fan_duty(&mut self, channel: u8, duty: f32);
}

/// File system access for G-code files and configuration.
pub trait FileSystem {
    /// Check if a file exists.
    fn exists(&mut self, path: &str) -> bool;

    /// Open a file for reading. Returns false if not found.
    fn open(&mut self, path: &str) -> bool;

    /// Read up to `buf.len()` bytes. Returns number of bytes read (0 = EOF).
    fn read(&mut self, buf: &mut [u8]) -> usize;

    /// Close the currently open file.
    fn close(&mut self);
}

/// Endstop switch reader.
pub trait EndstopReader {
    /// Returns true if the endstop for the given axis is triggered.
    fn is_triggered(&self, axis: u8) -> bool;
}

// ── Channel index types (portable, no board dependency) ────────

/// Identifies a temperature sensor channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum TempChannel {
    Bed,
    Hotend1,
    Hotend2,
}

impl TempChannel {
    pub fn index(self) -> usize {
        match self {
            Self::Bed => 0,
            Self::Hotend1 => 1,
            Self::Hotend2 => 2,
        }
    }

    pub const ALL: [TempChannel; 3] = [Self::Bed, Self::Hotend1, Self::Hotend2];
}

/// Identifies a PWM output channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum PwmChannel {
    /// Heated bed
    Bed,
    /// Hotend heater 1
    Heater1,
    /// Hotend heater 2
    Heater2,
    /// Fan 0 (heatbreak)
    Fan0,
    /// Fan 1 (part cooling)
    Fan1,
    /// Fan 2 (aux)
    Fan2,
    /// Fan 3 / laser / VFD
    Fan3,
}

impl PwmChannel {
    pub fn fan_index(self) -> Option<usize> {
        match self {
            Self::Fan0 => Some(0),
            Self::Fan1 => Some(1),
            Self::Fan2 => Some(2),
            Self::Fan3 => Some(3),
            _ => None,
        }
    }
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
