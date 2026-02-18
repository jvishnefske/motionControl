//! Core types used throughout the G-code parser.

/// Axis identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum Axis {
    X,
    Y,
    Z,
    E,
}

impl Axis {
    pub fn index(self) -> usize {
        match self {
            Axis::X => 0,
            Axis::Y => 1,
            Axis::Z => 2,
            Axis::E => 3,
        }
    }
}

/// Positioning mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum PositionMode {
    Absolute,
    Relative,
}

/// Stepper driver mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum DriverMode {
    ConstantOffTime,
    RandomOffTime,
    SpreadCycle,
    StealthChop,
}

/// Optional axis values — parsed from a G-code line.
#[derive(Clone, Copy, Debug, Default, defmt::Format)]
pub struct AxisValues {
    pub x: Option<f32>,
    pub y: Option<f32>,
    pub z: Option<f32>,
    pub e: Option<f32>,
}

impl AxisValues {
    pub fn get(&self, axis: Axis) -> Option<f32> {
        match axis {
            Axis::X => self.x,
            Axis::Y => self.y,
            Axis::Z => self.z,
            Axis::E => self.e,
        }
    }
}

/// Optional axis flags (for G28).
#[derive(Clone, Copy, Debug, Default, defmt::Format)]
pub struct AxisFlags {
    pub x: bool,
    pub y: bool,
    pub z: bool,
}

impl AxisFlags {
    pub fn any(&self) -> bool {
        self.x || self.y || self.z
    }

    pub fn all(&self) -> bool {
        self.x && self.y && self.z
    }
}
