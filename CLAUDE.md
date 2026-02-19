# CLAUDE.md

## Project

Embedded Rust firmware for the Duet 3 Mini 5+ (ATSAME54P20A) using Embassy async actors.

## Build

```bash
cargo check --workspace          # type-check all crates
cargo test --workspace --target x86_64-unknown-linux-gnu  # run all tests on host
cargo clippy --workspace         # lint
cargo fmt --all                  # format
```

## Architecture

Portable library crates with HAL trait boundary:
- `printer-hal` — trait definitions (StepperDriver, TemperatureSensor, etc.)
- `gcode-parser` — zero-allocation G-code parsing
- `dispatcher` — command routing (gcode → motion/thermal/sdcard actors)
- `motion-planner` — trapezoidal profiles + Bresenham step generator
- `thermal` — PID controller + heater/fan manager
- `sdcard` — line reader + file protocol
- `actor-framework` — Embassy-based mailbox/event bus
- `board-hal` — Duet 3 Mini 5+ specific pin mappings + TMC2209

Binary crates:
- `firmware/` — hardware init + task spawn (thin wiring layer)
- `wasm-sim/` — mock HAL impls for browser/host testing

## Pull Request Policy

Pull requests should be automatically merged when all CI checks pass. Use `gh pr merge --auto --squash` after creating a PR to enable auto-merge.
