# Design — Multi-Board 3D Printer Firmware

## Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│  Portable library crates (no_std, 71 tests on host)            │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐   │
│  │ gcode-parser │  │  dispatcher  │  │  motion-planner    │   │
│  │  (pure parse)│  │ (cmd routing)│  │  + step_generator  │   │
│  └──────────────┘  └──────────────┘  └────────────────────┘   │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐   │
│  │   thermal    │  │    sdcard    │  │  actor-framework   │   │
│  │  (PID + mgr) │  │ (config +   │  │  (mailbox, select) │   │
│  │              │  │  line reader)│  │                    │   │
│  └──────────────┘  └──────────────┘  └────────────────────┘   │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  printer-hal  (traits + NullFs — no board dependency)    │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘

┌───────────────────────────┐  ┌───────────────────────────┐
│  firmware/ (binary)       │  │  firmware-pico/ (binary)   │
│  Duet 3 Mini 5+ mainboard│  │  BTT SKR Pico mainboard   │
│  ATSAME54P20A Cortex-M4F │  │  RP2040 dual Cortex-M0+   │
│  board-hal pin mappings   │  │  board-pico pin mappings   │
│  SD card CONFIG.G         │  │  Flash CONFIG.G (planned)  │
└───────────────────────────┘  └───────────────────────────┘

┌───────────────────────────┐  ┌──────────────────────────────┐
│  firmware-ebb42/ (binary) │  │  wasm-sim/ (binary/lib)      │
│  BTT EBB42 CAN toolboard │  │  Browser mock HAL impls      │
│  STM32G0B1 Cortex-M0+    │  │  Embassy arch-wasm32         │
│  board-ebb42 pin mappings │  │  Canvas rendering, WebSerial │
│  Stepper + thermal + CAN │  │  No hardware dependencies    │
└───────────────────────────┘  └──────────────────────────────┘
```

## Supported Boards

| Board | MCU | Role | Crate | Firmware | Config Source |
|-------|-----|------|-------|----------|---------------|
| Duet 3 Mini 5+ Ethernet | ATSAME54P20A (Cortex-M4F, 120MHz) | Mainboard | `board-hal` | `firmware/` | SD card `CONFIG.G` |
| BTT EBB42 v1.2 | STM32G0B1CBT6 (Cortex-M0+, 64MHz) | CAN Toolboard | `board-ebb42` | `firmware-ebb42/` | CAN from mainboard |
| BTT SKR Pico v1.0 | RP2040 (dual Cortex-M0+, 133MHz) | Mainboard | `board-pico` | `firmware-pico/` | Flash partition (planned) |

Each board gets a single binary release. The binary reads `CONFIG.G` from storage
at boot and falls back to compiled-in defaults when no file is found.

## Release

Tag push (`v*`) triggers per-board `.bin` builds via GitHub Actions matrix:
- `duet3-mini5.bin` — `thumbv7em-none-eabihf`
- `ebb42.bin` — `thumbv6m-none-eabi`
- `skr-pico.bin` — `thumbv6m-none-eabi`

## Implementation Status

### 1. Motion Control

- [x] G-code parsing (G0, G1, G28, G90, G91, G92)
- [x] Trapezoidal acceleration planner
- [x] Multi-axis Bresenham step distribution
- [x] Per-axis feedrate and acceleration clamping
- [x] Absolute / relative positioning modes
- [x] Homing state tracking
- [x] Emergency stop (M112) — immediate queue clear
- [x] Wait for moves (M400)
- [x] Position reporting (M114)
- [ ] Endstop reading (homing actually hits a switch)
- [ ] Software endstops (min/max travel limits)
- [ ] Backlash compensation

### 2. Pressure Advance

Linear advance for extruder pressure compensation. Adjusts extruder steps
ahead of actual motion to compensate for filament compression in the Bowden
tube or hotend melt zone.

- [ ] Pressure advance factor (K) storage per extruder (M572)
- [ ] Extruder step lookahead — advance E steps based on instantaneous speed
- [ ] Deceleration compensation — retract extra E steps on deceleration
- [ ] Per-segment E-step adjustment in step generator
- [ ] Unit tests: verify E steps lead/lag XY motion by K × velocity

### 3. Input Shaping

Reduce ringing artifacts by filtering acceleration profiles to cancel
resonant frequencies of the printer frame and toolhead.

- [ ] Input shaper filter (ZV, MZV, EI, 2HUMP_EI, 3HUMP_EI types)
- [ ] Configurable shaper frequency and damping ratio per axis (M593)
- [ ] Shaped acceleration profile generation
- [ ] Acceleration limit adjustment based on shaper
- [ ] Unit tests: shaped profile impulse count and frequency response

### 4. Thermal Management

- [x] PID controller with anti-windup
- [x] Bed + dual hotend heater channels
- [x] Thermal runaway detection
- [x] Temperature set (M104) and set-and-wait (M109)
- [x] Bed temperature set (M140) and wait (M190)
- [x] Fan speed control (M106 / M107)
- [ ] ADC thermistor reading (wire HAL trait to hardware)
- [ ] PWM heater/fan output (wire HAL trait to hardware)
- [ ] PID auto-tune (M303)

### 5. Configuration

- [x] Steps per mm (M92)
- [x] Max feedrate (M203)
- [x] Max acceleration per axis (M201)
- [x] Default print/travel acceleration (M204)
- [x] Microstepping (M350)
- [x] Motor current (M906)
- [x] Driver direction/mode (M569)
- [x] Built-in config defaults per board
- [x] Config loader with FileSystem trait (`CONFIG.G` → fallback defaults)
- [x] Config override layering (`CONFIGO.G` on top of base config)
- [x] NullFs for boards without storage
- [ ] Config save to storage (M500)
- [ ] Config restore from storage (M501)

### 6. SD Card / File I/O

- [x] Line reader (zero-allocation, chunked input)
- [x] SD card actor message protocol
- [x] Portable config loader (`load_config_with_fallback`)
- [x] Comment stripping in config files
- [ ] SPI + FAT32 mount (board-specific HAL wiring)
- [ ] Stream G-code file for job execution
- [ ] Job pause / resume / cancel
- [ ] Job progress reporting

### 7. Communication

- [ ] USB serial G-code input
- [ ] Serial response output (ok, error, temperature reports)
- [ ] UART to Raspberry Pi (SKR Pico GPIO0/1)

### 8. CAN Toolboard Support

CAN-FD bus for distributed tool boards (Duet 3 ecosystem).

- [x] EBB42 firmware binary with CAN task structure
- [x] Stepper + thermal actor tasks on toolboard
- [x] CAN command/response message types defined
- [ ] CAN-FD driver (1 Mbit arbitration, 5 Mbit data phase)
- [ ] CBOR message serialization/deserialization (no_std, zero-alloc)
- [ ] Toolboard discovery and address assignment
- [ ] Remote heater/stepper/endstop/thermistor/fan control via CAN
- [ ] Heartbeat / watchdog — detect toolboard disconnect
- [ ] printer-hal trait proxying over CAN transport

### 9. Hardware Abstraction (printer-hal traits)

- [x] `StepperDriver` — set direction, pulse step, enable/disable
- [x] `TemperatureSensor` — read temperature from ADC channel
- [x] `HeaterOutput` — set PWM duty cycle for heater/fan
- [x] `FileSystem` — open/read/close files from storage
- [x] `EndstopReader` — read endstop switch state
- [x] `NullFs` — stub for boards without storage
- [ ] `Delay` — microsecond-precision timing for step pulses

### 10. Actor System

- [x] Typed mailboxes (embassy-sync channels)
- [x] System event bus (pub/sub broadcast)
- [x] Priority-based select (select, select3)
- [x] Dispatcher → planner → step generator pipeline
- [x] Thermal manager 10Hz PID loop
- [x] Status monitor (aggregates all actor outputs)

### 11. Board Support

- [x] Duet 3 Mini 5+ pin mappings (ATSAME54P20A)
- [x] BTT EBB42 v1.2 pin mappings (STM32G0B1CBT6)
- [x] BTT SKR Pico v1.0 pin mappings (RP2040)
- [x] Per-board firmware binaries with Embassy task spawn
- [x] Per-board compiled-in config defaults
- [x] Memory layouts (memory.x) for all three targets
- [ ] TMC2209 UART driver (shared bus on Pico, per-driver on Duet)
- [ ] RP2040 PIO for TMC2209 single-wire UART

### 12. WASM Simulation Target

- [x] Mock `StepperDriver` — logs steps, tracks position
- [x] Mock `TemperatureSensor` — simulated thermal model
- [x] Mock `HeaterOutput` — records duty cycles
- [x] Mock `FileSystem` — reads from embedded strings or JS
- [x] Mock `EndstopReader` — configurable trigger state
- [ ] Embassy `arch-wasm32` executor
- [ ] Browser entry point (wasm-bindgen)
- [ ] Simulated time (embassy-time with wasm tick source)

### 13. Portability & CI

- [x] All business logic in `no_std` library crates
- [x] Library crates depend only on `printer-hal` traits
- [x] Firmware binary: only hardware init + task spawn + HAL wiring
- [x] 71 unit tests run on host (`x86_64-unknown-linux-gnu`)
- [x] CI: fmt auto-commit on PRs, check, test, clippy
- [x] Release workflow: per-board `.bin` on tag push
- [ ] WASM target compiles with mock HAL

### 14. Safety

- [x] Thermal runaway detection
- [x] Emergency stop propagation to all actors
- [ ] Watchdog timer (hardware reset on firmware hang)
- [ ] Heater interlock (max on-time without temp change)
- [ ] Stepper timeout (disable after idle period)
