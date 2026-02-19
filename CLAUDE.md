# CLAUDE.md

## Project

Multi-board 3D printer firmware using Embassy async actors.
Supports Duet 3 Mini 5+ (ATSAME54P20A), BTT EBB42 (STM32G0B1), and BTT SKR Pico (RP2040).

## Build

```bash
cargo check --workspace          # type-check all crates
cargo test --workspace \
  --exclude duet3-firmware --exclude ebb42-firmware --exclude pico-firmware \
  --exclude actor-framework --exclude wasm-sim \
  --target x86_64-unknown-linux-gnu  # run all tests on host
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
Board-specific pin mapping crates:
- `board-hal` — Duet 3 Mini 5+ (ATSAME54P20A) pin mappings + TMC2209
- `board-ebb42` — BTT EBB42 v1.2 (STM32G0B1CBT6) CAN toolboard pins
- `board-pico` — BTT SKR Pico v1.0 (RP2040) mainboard pins

Binary crates (thin hardware wiring layers):
- `firmware/` — Duet 3 Mini 5+ mainboard firmware
- `firmware-ebb42/` — EBB42 CAN toolboard firmware (stepper + thermal + CAN)
- `firmware-pico/` — SKR Pico mainboard firmware (full stack)
- `wasm-sim/` — mock HAL impls for browser/host testing

## Pull Request Policy

Pull requests should be automatically merged when all CI checks pass. Use `gh pr merge --auto --squash` after creating a PR to enable auto-merge.
