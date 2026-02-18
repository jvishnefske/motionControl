//! # G-code Parser — Zero-Allocation, `no_std`
//!
//! Parses RepRapFirmware-compatible G-code lines into typed command enums.
//! Supports all motion, configuration, and temperature commands needed
//! for a Duet 3 firmware implementation.
//!
//! Reference: <https://docs.duet3d.com/User_manual/Reference/Gcodes>

#![no_std]

pub mod commands;
pub mod parser;
pub mod types;

pub use commands::GCodeCommand;
pub use parser::parse_line;
pub use types::*;
