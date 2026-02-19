//! # BTT EBB42 v1.2 — CAN Toolboard Firmware
//!
//! This binary crate runs on the EBB42 CAN toolboard (STM32G0B1CBT6).
//! The toolboard is a peripheral device controlled by the mainboard over CAN-FD.
//!
//! Responsibilities:
//! - Single extruder stepper (TMC2209)
//! - Hotend heater + thermistor (PID loop)
//! - Part cooling + hotend fans
//! - ADXL345 accelerometer (input shaper tuning)
//! - CAN-FD communication with mainboard
//!
//! All business logic lives in library crates:
//! - `thermal` — PID control + heater/fan management
//! - `printer-hal` — platform-agnostic HAL traits

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};
use {defmt_rtt as _, panic_probe as _};

use actor_framework::select::{select, Either};
use printer_hal::TempChannel;
use thermal::{ThermalCommand, ThermalManager, ThermalStatus};

// ══════════════════════════════════════════════════════════════════
//  Static Channels (actor mailboxes)
// ══════════════════════════════════════════════════════════════════

/// Commands from mainboard (via CAN) to the stepper driver.
static STEPPER_CMD: Channel<CriticalSectionRawMutex, StepperCmd, 16> = Channel::new();

/// Commands from mainboard (via CAN) to thermal subsystem.
static THERMAL_CMD: Channel<CriticalSectionRawMutex, ThermalCommand, 8> = Channel::new();

/// Thermal status reports back to mainboard.
static THERMAL_STATUS: Channel<CriticalSectionRawMutex, ThermalStatus, 8> = Channel::new();

// ══════════════════════════════════════════════════════════════════
//  Toolboard-specific types
// ══════════════════════════════════════════════════════════════════

/// Stepper commands received over CAN from the mainboard.
#[derive(Clone, Debug, defmt::Format)]
pub enum StepperCmd {
    /// Execute a number of steps at a given interval.
    Steps {
        count: u32,
        interval_us: u32,
        forward: bool,
    },
    /// Enable or disable the stepper driver.
    Enable(bool),
    /// Emergency stop — immediately halt.
    Stop,
}

// ══════════════════════════════════════════════════════════════════
//  Task: Stepper Driver (single extruder motor)
// ══════════════════════════════════════════════════════════════════

/// EBB42 hardware stepper driver — TODO: wire to actual GPIO.
struct Ebb42Stepper;

impl printer_hal::StepperDriver for Ebb42Stepper {
    fn set_direction(&mut self, _axis: u8, _forward: bool) {
        // TODO: Set PD0 (E_DIR) via PAC GPIO
    }

    fn step(&mut self, _axis: u8) {
        // TODO: Toggle PD1 (E_STEP) via PAC GPIO
    }

    fn enable(&mut self, _axis: u8, _enabled: bool) {
        // TODO: Set PD2 (E_ENABLE) via PAC GPIO
    }
}

#[embassy_executor::task]
async fn stepper_task() {
    info!("EBB42 stepper task started");
    let cmd_rx = STEPPER_CMD.receiver();
    let mut _driver = Ebb42Stepper;

    loop {
        let cmd = cmd_rx.receive().await;
        match cmd {
            StepperCmd::Steps {
                count,
                interval_us,
                forward,
            } => {
                debug!(
                    "Step: {} steps, {}us interval, fwd={}",
                    count, interval_us, forward
                );
                // TODO: Execute steps using printer_hal::StepperDriver
                // with Timer::after for each step interval
            }
            StepperCmd::Enable(en) => {
                info!("Stepper enable={}", en);
                // TODO: driver.enable(0, en)
            }
            StepperCmd::Stop => {
                warn!("Stepper: Emergency stop");
                // TODO: driver.enable(0, false)
            }
        }
    }
}

// ══════════════════════════════════════════════════════════════════
//  Task: Thermal Manager (single hotend + 2 fans)
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::task]
async fn thermal_task() {
    info!("EBB42 thermal task started");
    let cmd_rx = THERMAL_CMD.receiver();
    let status_tx = THERMAL_STATUS.sender();
    let mut manager = ThermalManager::new();
    let mut ticker = Ticker::every(Duration::from_millis(100)); // 10Hz PID
    let dt: f32 = 0.1;

    loop {
        match select(cmd_rx.receive(), ticker.next()).await {
            Either::First(cmd) => match cmd {
                ThermalCommand::SetTarget { channel, temp_c } => {
                    info!("Thermal: Set {} to {}C", channel, temp_c);
                    manager.set_target(channel, temp_c);
                }
                ThermalCommand::SetTargetAndWait { channel, temp_c } => {
                    info!("Thermal: Set {} to {}C (wait)", channel, temp_c);
                    manager.set_target_and_wait(channel, temp_c);
                }
                ThermalCommand::HeaterOff { channel } => manager.heater_off(channel),
                ThermalCommand::ClearFault { channel } => {
                    info!("Thermal: clearing fault on {}", channel);
                    manager.clear_fault(channel);
                }
                ThermalCommand::SetFanSpeed { channel, speed } => {
                    manager.set_fan_speed(channel, speed)
                }
                ThermalCommand::FanOff { channel } => manager.fan_off(channel),
                ThermalCommand::ReportTemperatures => {
                    // EBB42 only has hotend thermistor
                    let h = &manager.heaters[TempChannel::Hotend1.index()];
                    status_tx
                        .send(ThermalStatus::Temperature {
                            channel: TempChannel::Hotend1,
                            current_c: h.current_c,
                            target_c: h.target_c,
                            pwm: h.pwm_output.fraction(),
                        })
                        .await;
                }
                ThermalCommand::EmergencyStop => {
                    manager.emergency_stop();
                    warn!("Thermal: Emergency stop — all heaters off");
                }
            },
            Either::Second(_tick) => {
                // PID update + safety checks — only hotend on EBB42
                let ch = TempChannel::Hotend1;
                // TODO: Read actual ADC thermistor (PA3 / ADC_IN3)
                let current = manager.heaters[ch.index()].current_c;
                let _duty = manager.update_heater(ch, current, dt);
                // TODO: Write duty to PB13 (HEATER_HOTEND) via PWM

                if let Some(fault) = manager.check_safety(ch, dt) {
                    warn!("Heater fault {:?} on hotend", fault);
                    status_tx
                        .send(ThermalStatus::HeaterFaulted { channel: ch, fault })
                        .await;
                }

                let idx = ch.index();
                if manager.heaters[idx].waiting && manager.is_at_target(ch, 2.0) {
                    manager.heaters[idx].waiting = false;
                    status_tx
                        .send(ThermalStatus::TargetReached { channel: ch })
                        .await;
                }
            }
        }
    }
}

// ══════════════════════════════════════════════════════════════════
//  Task: CAN-FD Communication
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::task]
async fn can_task() {
    info!("EBB42 CAN-FD task started");
    let stepper_tx = STEPPER_CMD.sender();
    let thermal_tx = THERMAL_CMD.sender();
    let _thermal_rx = THERMAL_STATUS.receiver();

    // TODO: Initialize FDCAN2 peripheral (PB0 = CAN_TX, PB1 = CAN_RX)
    //   - 1 Mbit/s arbitration, 5 Mbit/s data phase
    //   - Accept toolboard address filter
    //   - CBOR decode incoming frames → route to stepper/thermal commands
    //   - CBOR encode thermal status → transmit to mainboard

    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        // Placeholder: poll for CAN frames
        ticker.next().await;

        // TODO: Read CAN frame, decode CBOR, dispatch:
        //   - Stepper commands → stepper_tx
        //   - Thermal commands → thermal_tx
        //   - Heartbeat response → send back via CAN
        let _ = (&stepper_tx, &thermal_tx);
    }
}

// ══════════════════════════════════════════════════════════════════
//  Entry Point — Hardware Init + Task Spawn
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("═══════════════════════════════════════");
    info!("  EBB42 CAN Toolboard Firmware v0.1.0");
    info!("  STM32G0B1CBT6 @ 64MHz");
    info!("═══════════════════════════════════════");

    // TODO: Initialize STM32G0B1 peripherals via embassy-stm32:
    //   - Clock system (64MHz from 8MHz HSE crystal)
    //   - GPIO pins (PD0/PD1/PD2 stepper, PB13 heater, PA0/PA1 fans)
    //   - ADC for thermistors (PA3, PA2)
    //   - TIM for PWM (heater + fans)
    //   - SPI2 for ADXL345 (PB3/PB4/PB5/PA15)
    //   - SPI1 for MAX31865 (PA5/PA6/PA7/PA4) — optional
    //   - FDCAN2 (PB0/PB1)
    //   - USB (PA11/PA12) — for firmware update / fallback

    spawner.spawn(stepper_task()).unwrap();
    spawner.spawn(thermal_task()).unwrap();
    spawner.spawn(can_task()).unwrap();

    info!("All tasks spawned — waiting for CAN commands");

    // Heartbeat loop
    let mut ticker = Ticker::every(Duration::from_secs(10));
    loop {
        ticker.next().await;
        debug!("EBB42 heartbeat");
    }
}
