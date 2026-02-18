//! # SD Card Reader — G-code File Loader
//!
//! Reads G-code configuration and job files from the SD card's FAT32
//! filesystem. Parses lines incrementally with zero heap allocation.
//!
//! ## File naming
//! Uses 8.3 short filenames (FAT limitation):
//! - `CONFIG.G` — main machine configuration
//! - `CONFIGO.G` — config-override (saved parameters)
//! - `HOMEALL.G` — home all axes macro
//! - `HOMEX.G`, `HOMEY.G`, `HOMEZ.G` — per-axis homing macros

#![no_std]

pub mod messages;
pub mod reader;

pub use messages::*;
pub use reader::LineReader;
