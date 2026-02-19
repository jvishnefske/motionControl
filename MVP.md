# MVP Functional Requirements — 3D Printer Control Firmware

## Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│  Portable library crates (no_std, test on host + WASM)         │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐   │
│  │ gcode-parser │  │  dispatcher  │  │  motion-planner    │   │
│  │  (pure parse)│  │ (cmd routing)│  │  + step_generator  │   │
│  └──────────────┘  └──────────────┘  └────────────────────┘   │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐   │
│  │   thermal    │  │    sdcard    │  │  actor-framework   │   │
│  │  (PID + mgr) │  │ (line reader)│  │  (mailbox, select) │   │
│  └──────────────┘  └──────────────┘  └────────────────────┘   │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  printer-hal  (trait definitions — no implementations)   │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘

┌───────────────────────────┐  ┌───────────────────────────┐
│  firmware/ (binary)       │  │  firmware-pico/ (binary)   │
│  Duet 3 ATSAME54 HAL     │  │  BTT SKR Pico RP2040      │
│  board-hal pin mappings   │  │  board-pico pin mappings   │
│  Embassy executor spawn   │  │  4x TMC2209 shared UART   │
│  Full mainboard stack     │  │  Full mainboard stack      │
└───────────────────────────┘  └───────────────────────────┘

┌───────────────────────────┐  ┌──────────────────────────────┐
│  firmware-ebb42/ (binary) │  │  wasm-sim/ (binary/lib)      │
│  BTT EBB42 STM32G0B1     │  │  Browser mock HAL impls      │
│  board-ebb42 pin mappings │  │  Embassy arch-wasm32         │
│  CAN toolboard (subset)  │  │  Canvas rendering, WebSerial │
│  Stepper + thermal + CAN │  │  No hardware dependencies    │
└───────────────────────────┘  └──────────────────────────────┘
```

## Supported Boards

| Board | MCU | Role | Crate | Firmware |
|-------|-----|------|-------|----------|
| Duet 3 Mini 5+ Ethernet | ATSAME54P20A (Cortex-M4F, 120MHz) | Mainboard | `board-hal` | `firmware/` |
| BTT EBB42 v1.2 | STM32G0B1CBT6 (Cortex-M0+, 64MHz) | CAN Toolboard | `board-ebb42` | `firmware-ebb42/` |
| BTT SKR Pico v1.0 | RP2040 (dual Cortex-M0+, 133MHz) | Mainboard | `board-pico` | `firmware-pico/` |

## MVP Checklist

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
- [ ] Backlash compensation (optional)

### 2. Pressure Advance

Linear advance for extruder pressure compensation. Adjusts extruder steps
ahead of actual motion to compensate for filament compression in the Bowden
tube or hotend melt zone.

- [ ] Pressure advance factor (K) storage per extruder (M572)
- [ ] Extruder step lookahead — advance E steps based on instantaneous speed
- [ ] Deceleration compensation — retract extra E steps on deceleration
- [ ] G-code: M572 D0 S0.06 — set advance factor per extruder
- [ ] Per-segment E-step adjustment in step generator
- [ ] Unit tests: verify E steps lead/lag XY motion by K × velocity

### 3. Input Shaping / Dynamic Speed Control

Reduce ringing (ghosting) artifacts by filtering acceleration profiles to
cancel resonant frequencies of the printer frame and toolhead.

- [ ] Input shaper filter (ZV, MZV, EI, 2HUMP_EI, 3HUMP_EI types)
- [ ] Configurable shaper frequency per axis (M593)
- [ ] Configurable damping ratio per axis
- [ ] Shaped acceleration profile generation (pre-filter trapezoid into shaped segments)
- [ ] G-code: M593 F45.0 — set input shaper frequency
- [ ] G-code: M593 P"mzv" — select shaper type
- [ ] Acceleration limit adjustment based on shaper (max_accel × shaper_factor)
- [ ] Dynamic speed scaling — reduce speed at sharp corners to limit vibration
- [ ] Unit tests: verify shaped profile has correct number of impulses
- [ ] Unit tests: verify frequency response attenuation at resonant frequency

### 4. Thermal Management

- [x] PID controller with anti-windup
- [x] Bed + dual hotend heater channels
- [x] Thermal runaway detection
- [x] Temperature set (M104) and set-and-wait (M109)
- [x] Bed temperature set (M140) and wait (M190)
- [x] Fan speed control (M106 / M107)
- [ ] Actual ADC thermistor reading (HAL trait)
- [ ] Actual PWM heater/fan output (HAL trait)
- [ ] PID auto-tune (M303)

### 5. Configuration

- [x] Steps per mm (M92)
- [x] Max feedrate (M203)
- [x] Max acceleration per axis (M201)
- [x] Default print/travel acceleration (M204)
- [x] Microstepping (M350)
- [x] Motor current (M906)
- [x] Driver direction/mode (M569)
- [x] Built-in config defaults
- [ ] SD card config load (CONFIG.G)
- [ ] Config save/restore (M500/M501)

### 6. SD Card / File I/O

- [x] Line reader (zero-allocation line parser)
- [x] SD card actor message protocol
- [ ] SPI init + FAT32 mount (HAL trait)
- [ ] Stream G-code file for job execution
- [ ] Job pause / resume / cancel
- [ ] Job progress reporting

### 7. Communication

- [ ] USB serial G-code input
- [ ] Serial response output (ok, error, temperature reports)

### 8. CAN Toolboard Support (CBOR)

CAN-FD bus for distributed tool boards (e.g., Duet 3 1HCL, 3HC, toolboard 1LC).
Uses CBOR encoding for compact, schema-flexible messages over CAN frames.

- [ ] CAN-FD driver (1 Mbit arbitration, 5 Mbit data phase)
- [ ] CBOR message serialization/deserialization (no_std, zero-alloc)
- [ ] Toolboard discovery and address assignment protocol
- [ ] Remote heater control — set target temp, read current temp via CAN
- [ ] Remote stepper control — send step/dir commands to toolboard drivers
- [ ] Remote endstop reading — poll toolboard endstop state
- [ ] Remote thermistor reading — stream ADC values over CAN
- [ ] Remote fan control — set fan PWM via CAN
- [ ] Heartbeat / watchdog — detect toolboard disconnect
- [ ] printer-hal trait proxying — wrap CAN transport as HAL trait impls
- [ ] G-code: M954 — configure CAN toolboard mapping
- [ ] Unit tests: CBOR encode/decode roundtrip
- [ ] Unit tests: mock CAN transport with loopback

### 9. Hardware Abstraction (printer-hal traits)

- [x] `StepperDriver` — set direction, pulse step, enable/disable
- [x] `TemperatureSensor` — read temperature from ADC channel
- [x] `HeaterOutput` — set PWM duty cycle for heater/fan
- [x] `FileSystem` — open/read/close files from storage
- [x] `EndstopReader` — read endstop switch state
- [ ] `Delay` — microsecond-precision timing for step pulses

### 10. Actor System

- [x] Typed mailboxes (embassy-sync channels)
- [x] System event bus (pub/sub broadcast)
- [x] Priority-based select (select, select3)
- [x] Dispatcher → planner → step generator pipeline
- [x] Thermal manager 10Hz PID loop
- [x] Status monitor (aggregates all actor outputs)

### 11. WASM Simulation Target

- [x] Mock `StepperDriver` — logs steps, tracks position
- [x] Mock `TemperatureSensor` — simulated thermal model
- [x] Mock `HeaterOutput` — records duty cycles
- [x] Mock `FileSystem` — reads from embedded strings or JS
- [x] Mock `EndstopReader` — configurable trigger state
- [ ] Embassy `arch-wasm32` executor
- [ ] Browser entry point (wasm-bindgen)
- [ ] Simulated time (embassy-time with wasm tick source)

### 12. Portability Requirements

- [x] All business logic in `no_std` library crates
- [x] Library crates depend only on `printer-hal` traits (not board-hal)
- [x] Firmware binary: only hardware init + task spawn + HAL impl wiring
- [x] Unit tests run on host (`x86_64-unknown-linux-gnu`)
- [ ] WASM target compiles with mock HAL

### 13. Safety

- [x] Thermal runaway detection
- [x] Emergency stop propagation to all actors
- [ ] Watchdog timer (hardware reset on firmware hang)
- [ ] Heater interlock (max on-time without temp change)
- [ ] Stepper timeout (disable after idle period)
