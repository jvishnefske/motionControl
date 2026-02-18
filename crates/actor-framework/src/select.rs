//! Re-export of embassy_futures::select for multi-source actors.
//!
//! Actors that need to respond to both incoming messages AND periodic timers
//! use `select` to race the two futures.

pub use embassy_futures::select::{select, select3, select4, Either, Either3, Either4};
