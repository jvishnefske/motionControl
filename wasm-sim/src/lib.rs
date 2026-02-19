//! # WASM Simulator — Mock HAL for Browser/Host Testing
//!
//! Provides mock implementations of `printer-hal` traits that log actions
//! and track state, enabling the full printer control stack to run without
//! hardware.

#![no_std]

use printer_hal::{EndstopReader, FileSystem, HeaterOutput, StepperDriver, TemperatureSensor};

/// Mock stepper that records steps and tracks position.
pub struct MockStepper {
    pub positions: [i64; 4],
    pub directions: [bool; 4],
    pub enabled: [bool; 4],
}

impl Default for MockStepper {
    fn default() -> Self {
        Self {
            positions: [0; 4],
            directions: [true; 4],
            enabled: [false; 4],
        }
    }
}

impl StepperDriver for MockStepper {
    fn set_direction(&mut self, axis: u8, forward: bool) {
        if (axis as usize) < 4 {
            self.directions[axis as usize] = forward;
        }
    }

    fn step(&mut self, axis: u8) {
        if (axis as usize) < 4 {
            if self.directions[axis as usize] {
                self.positions[axis as usize] += 1;
            } else {
                self.positions[axis as usize] -= 1;
            }
        }
    }

    fn enable(&mut self, axis: u8, enabled: bool) {
        if (axis as usize) < 4 {
            self.enabled[axis as usize] = enabled;
        }
    }
}

/// Mock temperature sensor with configurable readings.
pub struct MockTempSensor {
    pub readings: [f32; 3],
}

impl Default for MockTempSensor {
    fn default() -> Self {
        Self {
            readings: [25.0; 3],
        }
    }
}

impl TemperatureSensor for MockTempSensor {
    fn read_celsius(&mut self, channel: u8) -> f32 {
        if (channel as usize) < 3 {
            self.readings[channel as usize]
        } else {
            f32::NAN
        }
    }
}

/// Mock heater/fan output that records duty cycles.
pub struct MockHeaterOutput {
    pub heater_duties: [f32; 3],
    pub fan_duties: [f32; 4],
}

impl Default for MockHeaterOutput {
    fn default() -> Self {
        Self {
            heater_duties: [0.0; 3],
            fan_duties: [0.0; 4],
        }
    }
}

impl HeaterOutput for MockHeaterOutput {
    fn set_heater_duty(&mut self, channel: u8, duty: f32) {
        if (channel as usize) < 3 {
            self.heater_duties[channel as usize] = duty;
        }
    }

    fn set_fan_duty(&mut self, channel: u8, duty: f32) {
        if (channel as usize) < 4 {
            self.fan_duties[channel as usize] = duty;
        }
    }
}

/// Mock file system backed by static strings.
pub struct MockFileSystem {
    data: &'static [u8],
    cursor: usize,
    open: bool,
}

impl Default for MockFileSystem {
    fn default() -> Self {
        Self {
            data: b"",
            cursor: 0,
            open: false,
        }
    }
}

impl MockFileSystem {
    pub fn with_content(data: &'static [u8]) -> Self {
        Self {
            data,
            cursor: 0,
            open: false,
        }
    }
}

impl FileSystem for MockFileSystem {
    fn exists(&mut self, _path: &str) -> bool {
        !self.data.is_empty()
    }

    fn open(&mut self, _path: &str) -> bool {
        if self.data.is_empty() {
            return false;
        }
        self.cursor = 0;
        self.open = true;
        true
    }

    fn read(&mut self, buf: &mut [u8]) -> usize {
        if !self.open {
            return 0;
        }
        let remaining = &self.data[self.cursor..];
        let n = buf.len().min(remaining.len());
        buf[..n].copy_from_slice(&remaining[..n]);
        self.cursor += n;
        n
    }

    fn close(&mut self) {
        self.open = false;
        self.cursor = 0;
    }
}

/// Mock endstop reader — always not triggered unless configured.
#[derive(Default)]
pub struct MockEndstops {
    pub triggered: [bool; 4],
}

impl EndstopReader for MockEndstops {
    fn is_triggered(&self, axis: u8) -> bool {
        if (axis as usize) < 4 {
            self.triggered[axis as usize]
        } else {
            false
        }
    }
}
