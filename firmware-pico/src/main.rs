//! # BTT SKR Pico v1.0 — Mainboard Firmware
//!
//! This binary crate runs the full firmware stack on the SKR Pico (RP2040).
//! Designed for Voron V0 / small CoreXY printers.
//!
//! Hardware features:
//! - 4x TMC2209 stepper drivers (X/Y/Z/E, shared UART bus)
//! - 2x heater outputs (hotend + bed)
//! - 3x fan outputs (part cooling, hotend, controller)
//! - 2x thermistor ADC inputs
//! - USB-C serial for G-code
//! - UART to Raspberry Pi
//! - NeoPixel RGB LED
//!
//! All business logic lives in library crates:
//! - `dispatcher` — G-code parsing and command routing
//! - `motion-planner` — trapezoidal profiles + step generation
//! - `thermal` — PID control + heater/fan management

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker, Timer};
use {defmt_rtt as _, panic_probe as _};

use actor_framework::event_bus::EventBus;
use actor_framework::select::{select, Either};
use dispatcher::{dispatch_line, DispatchAction};
use motion_planner::{MotionCommand, MotionPlanner, MotionSegment, MotionStatus};
use printer_hal::TempChannel;
use thermal::{ThermalCommand, ThermalManager, ThermalStatus};

// ══════════════════════════════════════════════════════════════════
//  Static Channels (actor mailboxes)
// ══════════════════════════════════════════════════════════════════

static MOTION_CMD: Channel<CriticalSectionRawMutex, MotionCommand, 16> = Channel::new();
static MOTION_STATUS: Channel<CriticalSectionRawMutex, MotionStatus, 8> = Channel::new();
static THERMAL_CMD: Channel<CriticalSectionRawMutex, ThermalCommand, 8> = Channel::new();
static THERMAL_STATUS: Channel<CriticalSectionRawMutex, ThermalStatus, 8> = Channel::new();
static STEP_QUEUE: Channel<CriticalSectionRawMutex, MotionSegment, 8> = Channel::new();
static GCODE_LINE: Channel<CriticalSectionRawMutex, heapless::String<256>, 16> = Channel::new();
static SYSTEM_EVENTS: EventBus<SystemEvent, 8, 4, 4> = EventBus::new();

#[derive(Clone, Debug, defmt::Format)]
pub enum SystemEvent {
    EmergencyStop,
    ThermalRunaway { channel: TempChannel },
    HomingComplete,
}

// ══════════════════════════════════════════════════════════════════
//  Task: G-code Dispatcher
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::task]
async fn gcode_dispatcher_task() {
    info!("GCode dispatcher started");
    let line_rx = GCODE_LINE.receiver();
    let motion_tx = MOTION_CMD.sender();
    let thermal_tx = THERMAL_CMD.sender();

    loop {
        let line = line_rx.receive().await;
        let (primary, secondary) = dispatch_line(line.as_str());

        for action in [Some(primary), secondary].into_iter().flatten() {
            match action {
                DispatchAction::Motion(cmd) => motion_tx.send(cmd).await,
                DispatchAction::Thermal(cmd) => thermal_tx.send(cmd).await,
                DispatchAction::SdCard(_) => {
                    // SKR Pico has no SD card slot
                    info!("SD card not available on SKR Pico");
                }
                DispatchAction::Log(msg) => info!("{}", msg),
                DispatchAction::EmergencyStop => {
                    warn!("EMERGENCY STOP");
                    motion_tx.send(MotionCommand::EmergencyStop).await;
                    thermal_tx.send(ThermalCommand::EmergencyStop).await;
                    if let Ok(pub_handle) = SYSTEM_EVENTS.publisher() {
                        pub_handle.publish(SystemEvent::EmergencyStop).await;
                    }
                }
                DispatchAction::Noop => {}
            }
        }
    }
}

// ══════════════════════════════════════════════════════════════════
//  Task: Motion Planner
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::task]
async fn motion_planner_task() {
    info!("Motion planner started");
    let cmd_rx = MOTION_CMD.receiver();
    let status_tx = MOTION_STATUS.sender();
    let step_tx = STEP_QUEUE.sender();
    let mut planner = MotionPlanner::new();

    loop {
        let cmd = cmd_rx.receive().await;

        match cmd {
            MotionCommand::LinearMove {
                target,
                feedrate_mm_min,
                is_rapid,
            } => {
                let has_extrusion = target.e.is_some();
                let feedrate = if feedrate_mm_min > 0.0 {
                    Some(feedrate_mm_min)
                } else {
                    None
                };
                let n = planner.plan_linear_move(&target, feedrate, has_extrusion && !is_rapid);
                for _ in 0..n {
                    if let Some(seg) = planner.next_segment() {
                        step_tx.send(seg).await;
                    }
                }
            }
            MotionCommand::Home { x, y, z } => {
                info!("Homing: X={} Y={} Z={}", x, y, z);
                planner.mark_homed(x, y, z);
                status_tx
                    .send(MotionStatus::HomingComplete { x, y, z })
                    .await;
            }
            MotionCommand::SetAbsolute => planner.set_absolute(),
            MotionCommand::SetRelative => planner.set_relative(),
            MotionCommand::SetPosition { axes } => planner.set_position(&axes),
            MotionCommand::SetStepsPerMm { axes } => planner.set_steps_per_mm(&axes),
            MotionCommand::SetMaxFeedrate { axes } => planner.set_max_feedrate(&axes),
            MotionCommand::SetMaxAccelPerAxis { axes } => planner.set_max_accel(&axes),
            MotionCommand::SetAcceleration {
                print_accel,
                travel_accel,
            } => planner.set_acceleration(print_accel, travel_accel),
            MotionCommand::SetMicrostepping { axes, .. } => {
                info!("Microstepping set: {:?}", axes);
                // TODO: Configure TMC2209 via shared UART (GPIO8/9)
            }
            MotionCommand::SetMotorCurrent { axes, .. } => {
                info!("Motor current set: {:?}", axes);
                // TODO: Configure TMC2209 via shared UART (GPIO8/9)
            }
            MotionCommand::SetDriverConfig {
                driver,
                direction,
                stealthchop,
            } => {
                info!(
                    "Driver {} config: dir={:?} stealth={:?}",
                    driver, direction, stealthchop
                );
                // TODO: Configure TMC2209 via shared UART (GPIO8/9)
            }
            MotionCommand::WaitForCompletion => {
                while planner.has_pending() {
                    Timer::after(Duration::from_millis(10)).await;
                }
                status_tx.send(MotionStatus::MovesComplete).await;
            }
            MotionCommand::EmergencyStop => {
                planner.emergency_stop();
                warn!("Motion: Emergency stop — queue cleared");
            }
            MotionCommand::ReportPosition => {
                let pos = planner.position_mm();
                status_tx
                    .send(MotionStatus::Position {
                        x_mm: pos[0],
                        y_mm: pos[1],
                        z_mm: pos[2],
                        e_mm: pos[3],
                    })
                    .await;
            }
        }
    }
}

// ══════════════════════════════════════════════════════════════════
//  Task: Step Generator
// ══════════════════════════════════════════════════════════════════

/// SKR Pico hardware stepper driver — TODO: wire to actual GPIO.
struct PicoStepper;

impl printer_hal::StepperDriver for PicoStepper {
    fn set_direction(&mut self, _axis: u8, _forward: bool) {
        // TODO: Set direction pin via RP2040 GPIO
        //   X=GPIO10, Y=GPIO5, Z=GPIO28, E=GPIO13
    }

    fn step(&mut self, _axis: u8) {
        // TODO: Toggle step pin via RP2040 GPIO
        //   X=GPIO11, Y=GPIO6, Z=GPIO19, E=GPIO14
    }

    fn enable(&mut self, _axis: u8, _enabled: bool) {
        // TODO: Set enable pin via RP2040 GPIO
        //   X=GPIO12, Y=GPIO7, Z=GPIO2, E=GPIO15
    }
}

#[embassy_executor::task]
async fn step_generator_task() {
    info!("Step generator started");
    let seg_rx = STEP_QUEUE.receiver();
    let mut driver = PicoStepper;

    loop {
        let segment = seg_rx.receive().await;

        motion_planner::execute_segment(&segment, &mut driver, |wait_us| {
            let _ = wait_us;
        });

        if segment.duration_us > 0 {
            Timer::after(Duration::from_micros(segment.duration_us as u64)).await;
        }
    }
}

// ══════════════════════════════════════════════════════════════════
//  Task: Thermal Manager
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::task]
async fn thermal_manager_task() {
    info!("Thermal manager started");
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
                ThermalCommand::SetFanSpeed { channel, speed } => {
                    manager.set_fan_speed(channel, speed)
                }
                ThermalCommand::FanOff { channel } => manager.fan_off(channel),
                ThermalCommand::ReportTemperatures => {
                    // SKR Pico has bed + hotend thermistors
                    for &ch in &[TempChannel::Bed, TempChannel::Hotend1] {
                        let h = &manager.heaters[ch.index()];
                        status_tx
                            .send(ThermalStatus::Temperature {
                                channel: ch,
                                current_c: h.current_c,
                                target_c: h.target_c,
                                pwm: h.pwm_output.fraction(),
                            })
                            .await;
                    }
                }
                ThermalCommand::EmergencyStop => {
                    manager.emergency_stop();
                    warn!("Thermal: Emergency stop — all heaters off");
                }
            },
            Either::Second(_tick) => {
                // PID update — bed + hotend on SKR Pico
                for &ch in &[TempChannel::Bed, TempChannel::Hotend1] {
                    // TODO: Read ADC thermistor
                    //   Bed = GPIO26 (ADC0), Hotend = GPIO27 (ADC1)
                    let current = manager.heaters[ch.index()].current_c;
                    let _duty = manager.update_heater(ch, current, dt);
                    // TODO: Write duty to heater PWM
                    //   Hotend = GPIO23, Bed = GPIO21

                    if manager.check_runaway(ch, 20.0) {
                        warn!("Thermal runaway on {:?}", ch);
                        manager.emergency_stop();
                        status_tx
                            .send(ThermalStatus::ThermalRunaway {
                                channel: ch,
                                temp_c: manager.heaters[ch.index()].current_c,
                            })
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
}

// ══════════════════════════════════════════════════════════════════
//  Task: Status Monitor
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::task]
async fn status_monitor_task() {
    info!("Status monitor started");
    let motion_rx = MOTION_STATUS.receiver();
    let thermal_rx = THERMAL_STATUS.receiver();

    loop {
        match select(motion_rx.receive(), thermal_rx.receive()).await {
            Either::First(status) => match status {
                MotionStatus::Position {
                    x_mm,
                    y_mm,
                    z_mm,
                    e_mm,
                } => info!("Position: X={} Y={} Z={} E={}", x_mm, y_mm, z_mm, e_mm),
                MotionStatus::HomingComplete { x, y, z } => {
                    info!("Homing complete: X={} Y={} Z={}", x, y, z)
                }
                MotionStatus::MovesComplete => info!("All moves complete"),
                MotionStatus::Error(e) => error!("Motion error: {:?}", e),
            },
            Either::Second(status) => match status {
                ThermalStatus::Temperature {
                    channel,
                    current_c,
                    target_c,
                    pwm,
                } => debug!(
                    "{:?}: {}C / {}C (PWM: {}%)",
                    channel,
                    current_c,
                    target_c,
                    pwm * 100.0
                ),
                ThermalStatus::TargetReached { channel } => {
                    info!("Temperature reached on {:?}", channel)
                }
                ThermalStatus::ThermalRunaway { channel, temp_c } => {
                    error!("THERMAL RUNAWAY on {:?} at {}C!", channel, temp_c)
                }
                ThermalStatus::SensorFault { channel } => {
                    error!("Sensor fault on {:?}!", channel)
                }
            },
        }
    }
}

// ══════════════════════════════════════════════════════════════════
//  Entry Point — Hardware Init + Task Spawn
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("═══════════════════════════════════════");
    info!("  SKR Pico Firmware v0.1.0");
    info!("  RP2040 @ 133MHz");
    info!("═══════════════════════════════════════");

    // TODO: Initialize RP2040 peripherals via embassy-rp:
    //   - Clock system (133MHz from 12MHz crystal)
    //   - GPIO pins (step/dir/enable for 4 axes)
    //   - PIO or UART for TMC2209 shared bus (GPIO8 TX, GPIO9 RX)
    //   - PWM for heaters (GPIO23 hotend, GPIO21 bed)
    //   - PWM for fans (GPIO17 part, GPIO18 hotend, GPIO20 controller)
    //   - ADC for thermistors (GPIO26/ADC0 bed, GPIO27/ADC1 hotend)
    //   - USB for host communication
    //   - UART0 (GPIO0/1) for Raspberry Pi

    // Load default configuration (no SD card on SKR Pico)
    let defaults: &[&str] = &[
        "M569 P0 S1 D3", // X driver
        "M569 P1 S1 D3", // Y driver
        "M569 P2 S1 D3", // Z driver
        "M569 P3 S1 D3", // E driver
        "M350 X16 Y16 Z16 E16 I1",
        "M92 X80 Y80 Z400 E420",
        "M203 X6000 Y6000 Z600 E3600",
        "M201 X500 Y500 Z100 E500",
        "M204 P500 T1000",
        "M906 X800 Y800 Z800 E800 I30",
        "G90",
    ];

    spawner.spawn(gcode_dispatcher_task()).unwrap();
    spawner.spawn(motion_planner_task()).unwrap();
    spawner.spawn(step_generator_task()).unwrap();
    spawner.spawn(thermal_manager_task()).unwrap();
    spawner.spawn(status_monitor_task()).unwrap();

    info!("All actors spawned — loading defaults");

    let line_tx = GCODE_LINE.sender();
    for line in defaults {
        let mut s = heapless::String::<256>::new();
        if s.push_str(line).is_ok() {
            line_tx.send(s).await;
        }
    }

    // Heartbeat + system event monitoring
    let mut ticker = Ticker::every(Duration::from_secs(10));

    if let Ok(mut sub) = SYSTEM_EVENTS.subscriber() {
        loop {
            match select(sub.next_message_pure(), ticker.next()).await {
                Either::First(event) => match event {
                    SystemEvent::EmergencyStop => {
                        error!("SYSTEM: Emergency stop activated!")
                    }
                    SystemEvent::ThermalRunaway { channel } => {
                        error!("SYSTEM: Thermal runaway on {:?}!", channel)
                    }
                    SystemEvent::HomingComplete => info!("SYSTEM: Homing complete"),
                },
                Either::Second(_) => debug!("Heartbeat — system running"),
            }
        }
    } else {
        loop {
            ticker.next().await;
            debug!("Heartbeat — system running");
        }
    }
}
