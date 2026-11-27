//! The orchestrator and its web menu.
//!
//! * [`Orchestrator`] owns one running [`Module`](flight_core::Module) per
//!   [`ModuleKind`], swaps backends in place, applies presets and runs the
//!   one-click mission test through a [`MissionRunnerFn`].
//! * [`serve`] / [`router`] put an axum server in front of it: one embedded
//!   HTML page, Server-Sent Events at `/api/events` and JSON POST actions.

mod orchestrator;
mod web;

pub use orchestrator::{
    BackendHook, BackendOption, BoxFuture, MissionRunnerFn, ModuleView, Orchestrator, PRESETS, Runner,
    Snapshot, preset_backends, runner_fn,
};
pub use web::{INDEX_HTML, router, serve};

#[cfg(test)]
mod tests;
