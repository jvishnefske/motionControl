//! Stepper motor driver abstraction.
//!
//! Provides a hardware-agnostic interface for step/dir/enable control.
//! The actual GPIO manipulation is delegated to the firmware layer
//! which has access to the PAC peripherals.

use crate::pins::Pin;

/// Identifies one of the 5 stepper drivers on the Duet 3 Mini 5+.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum DriverId {
    Driver0 = 0,
    Driver1 = 1,
    Driver2 = 2,
    Driver3 = 3,
    Driver4 = 4,
}

impl DriverId {
    pub const ALL: [DriverId; 5] = [
        Self::Driver0,
        Self::Driver1,
        Self::Driver2,
        Self::Driver3,
        Self::Driver4,
    ];

    pub fn index(self) -> usize {
        self as usize
    }
}

/// Static configuration for a stepper driver's pins.
#[derive(Clone, Copy, Debug)]
pub struct StepperPins {
    pub step: Pin,
    pub dir: Pin,
    pub dir_inverted: bool,
    pub uart_addr: u8,
}

/// Pin configurations for all 5 drivers.
pub const STEPPER_CONFIGS: [StepperPins; 5] = [
    StepperPins {
        step: crate::pins::Duet3Pins::STEP_0,
        dir: crate::pins::Duet3Pins::DIR_0,
        dir_inverted: true, // !PB3
        uart_addr: 0,
    },
    StepperPins {
        step: crate::pins::Duet3Pins::STEP_1,
        dir: crate::pins::Duet3Pins::DIR_1,
        dir_inverted: false,
        uart_addr: 1,
    },
    StepperPins {
        step: crate::pins::Duet3Pins::STEP_2,
        dir: crate::pins::Duet3Pins::DIR_2,
        dir_inverted: false,
        uart_addr: 2,
    },
    StepperPins {
        step: crate::pins::Duet3Pins::STEP_3,
        dir: crate::pins::Duet3Pins::DIR_3,
        dir_inverted: false,
        uart_addr: 3,
    },
    StepperPins {
        step: crate::pins::Duet3Pins::STEP_4,
        dir: crate::pins::Duet3Pins::DIR_4,
        dir_inverted: false,
        uart_addr: 0, // shared address 0 with inverted select
    },
];

/// Runtime state for a stepper axis.
#[derive(Clone, Debug, defmt::Format)]
pub struct StepperState {
    pub position_steps: i64,
    pub steps_per_mm: f32,
    pub max_feedrate_mm_min: f32,
    pub max_accel_mm_s2: f32,
    pub current_ma: u16,
    pub microsteps: u16,
    pub interpolation: bool,
    pub stealthchop: bool,
    pub direction_forward: bool,
    pub homed: bool,
}

impl Default for StepperState {
    fn default() -> Self {
        Self {
            position_steps: 0,
            steps_per_mm: 80.0,
            max_feedrate_mm_min: 6000.0,
            max_accel_mm_s2: 500.0,
            current_ma: 800,
            microsteps: 16,
            interpolation: true,
            stealthchop: true,
            direction_forward: true,
            homed: false,
        }
    }
}

impl StepperState {
    pub fn position_mm(&self) -> f32 {
        self.position_steps as f32 / self.steps_per_mm
    }
}
