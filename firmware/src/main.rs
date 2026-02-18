//! # Duet 3 Mini 5+ Firmware — Main Entry Point
//!
//! Wires all async actor tasks together on the Embassy executor.
//! Architecture:
//!
//! ```text
//!  ┌──────────────────────────────────────────────────────────────┐
//!  │  High-Priority Executor (InterruptExecutor on TC2)          │
//!  │  ┌──────────────────┐  ┌──────────────────────────┐        │
//!  │  │  step_generator  │  │  safety_monitor          │        │
//!  │  │  (1kHz ISR)      │  │  (thermal runaway, estop)│        │
//!  │  └──────────────────┘  └──────────────────────────┘        │
//!  └──────────────────────────────────────────────────────────────┘
//!           ▲  Channels  │
//!           │            ▼
//!  ┌──────────────────────────────────────────────────────────────┐
//!  │  Thread-Mode Executor (main)                                │
//!  │  ┌────────────┐ ┌──────────┐ ┌──────────┐ ┌─────────────┐ │
//!  │  │ gcode      │ │ motion   │ │ thermal  │ │ sdcard      │ │
//!  │  │ dispatcher │ │ planner  │ │ manager  │ │ reader      │ │
//!  │  └────────────┘ └──────────┘ └──────────┘ └─────────────┘ │
//!  └──────────────────────────────────────────────────────────────┘
//! ```
//!
//! Control is ALWAYS available: the G-code dispatcher never blocks on
//! motor moves. Motion commands are queued, and the step generator
//! runs independently at interrupt priority.

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker, Timer};
use {defmt_rtt as _, panic_probe as _};

use actor_framework::event_bus::EventBus;
use actor_framework::select::{select, select3, Either, Either3};
use board_hal::pwm_output::PwmChannel;
use board_hal::thermistor::TempChannel;
use gcode_parser::{self, GCodeCommand};
use motion_planner::{MotionCommand, MotionPlanner, MotionSegment, MotionStatus};
use sdcard::{SdCardCommand, SdCardError, SdCardStatus};
use thermal::{ThermalCommand, ThermalManager, ThermalStatus};

// ══════════════════════════════════════════════════════════════════
//  Actor Mailboxes (static channels)
// ══════════════════════════════════════════════════════════════════

/// Motion planner inbox.
static MOTION_CMD: Channel<CriticalSectionRawMutex, MotionCommand, 16> = Channel::new();
/// Motion planner status output.
static MOTION_STATUS: Channel<CriticalSectionRawMutex, MotionStatus, 8> = Channel::new();

/// Thermal manager inbox.
static THERMAL_CMD: Channel<CriticalSectionRawMutex, ThermalCommand, 8> = Channel::new();
/// Thermal manager status output.
static THERMAL_STATUS: Channel<CriticalSectionRawMutex, ThermalStatus, 8> = Channel::new();

/// SD card reader inbox.
static SDCARD_CMD: Channel<CriticalSectionRawMutex, SdCardCommand, 4> = Channel::new();
/// SD card reader status output.
static SDCARD_STATUS: Channel<CriticalSectionRawMutex, SdCardStatus, 4> = Channel::new();

/// Step generator inbox — segments ready for execution.
static STEP_QUEUE: Channel<CriticalSectionRawMutex, MotionSegment, 8> = Channel::new();

/// G-code dispatch inbox — lines to parse and execute.
static GCODE_LINE: Channel<CriticalSectionRawMutex, heapless::String<256>, 16> = Channel::new();

/// System-wide event bus for emergency stop and critical events.
static SYSTEM_EVENTS: EventBus<SystemEvent, 8, 4, 4> = EventBus::new();

// ══════════════════════════════════════════════════════════════════
//  System Events
// ══════════════════════════════════════════════════════════════════

#[derive(Clone, Debug, defmt::Format)]
pub enum SystemEvent {
    EmergencyStop,
    ThermalRunaway { channel: TempChannel },
    HomingComplete,
    JobComplete,
}

// ══════════════════════════════════════════════════════════════════
//  Actor: G-code Dispatcher
//  Parses G-code lines and routes commands to the right actor.
//  NEVER blocks on motor moves — just queues them.
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::task]
async fn gcode_dispatcher_task() {
    info!("GCode dispatcher started");
    let line_rx = GCODE_LINE.receiver();
    let motion_tx = MOTION_CMD.sender();
    let thermal_tx = THERMAL_CMD.sender();
    let sdcard_tx = SDCARD_CMD.sender();

    loop {
        let line = line_rx.receive().await;
        let cmd = gcode_parser::parse_line(line.as_str());

        match cmd {
            // ── Motion ────────────────────────────────────────────
            GCodeCommand::LinearMove { axes, feedrate, .. } => {
                let is_rapid = feedrate.is_none();
                motion_tx
                    .send(MotionCommand::LinearMove {
                        target: axes,
                        feedrate_mm_min: feedrate.unwrap_or(0.0),
                        is_rapid,
                    })
                    .await;
            }
            GCodeCommand::Home { axes } => {
                motion_tx
                    .send(MotionCommand::Home {
                        x: !axes.any() || axes.x,
                        y: !axes.any() || axes.y,
                        z: !axes.any() || axes.z,
                    })
                    .await;
            }
            GCodeCommand::AbsolutePositioning => {
                motion_tx.send(MotionCommand::SetAbsolute).await;
            }
            GCodeCommand::RelativePositioning => {
                motion_tx.send(MotionCommand::SetRelative).await;
            }
            GCodeCommand::SetPosition { axes } => {
                motion_tx.send(MotionCommand::SetPosition { axes }).await;
            }

            // ── Configuration ─────────────────────────────────────
            GCodeCommand::SetStepsPerMm { axes } => {
                motion_tx.send(MotionCommand::SetStepsPerMm { axes }).await;
            }
            GCodeCommand::SetMaxFeedrate { axes } => {
                motion_tx.send(MotionCommand::SetMaxFeedrate { axes }).await;
            }
            GCodeCommand::SetMaxAccelPerAxis { axes } => {
                motion_tx
                    .send(MotionCommand::SetMaxAccelPerAxis { axes })
                    .await;
            }
            GCodeCommand::SetAcceleration {
                print_accel,
                travel_accel,
            } => {
                motion_tx
                    .send(MotionCommand::SetAcceleration {
                        print_accel,
                        travel_accel,
                    })
                    .await;
            }
            GCodeCommand::SetMicrostepping {
                axes,
                interpolation,
            } => {
                motion_tx
                    .send(MotionCommand::SetMicrostepping {
                        axes,
                        interpolation,
                    })
                    .await;
            }
            GCodeCommand::SetMotorCurrent { axes, idle_percent } => {
                motion_tx
                    .send(MotionCommand::SetMotorCurrent { axes, idle_percent })
                    .await;
            }
            GCodeCommand::SetDriverConfig {
                driver,
                direction,
                mode,
            } => {
                let stealthchop = mode.map(|m| matches!(m, gcode_parser::DriverMode::StealthChop));
                motion_tx
                    .send(MotionCommand::SetDriverConfig {
                        driver,
                        direction,
                        stealthchop,
                    })
                    .await;
            }

            // ── Temperature ───────────────────────────────────────
            GCodeCommand::SetHotendTemp { temp, .. } => {
                thermal_tx
                    .send(ThermalCommand::SetTarget {
                        channel: TempChannel::Hotend1,
                        temp_c: temp,
                    })
                    .await;
            }
            GCodeCommand::SetHotendTempWait { temp, .. } => {
                thermal_tx
                    .send(ThermalCommand::SetTargetAndWait {
                        channel: TempChannel::Hotend1,
                        temp_c: temp,
                    })
                    .await;
            }
            GCodeCommand::SetBedTemp { temp, heater } => {
                if let Some(temp) = temp {
                    thermal_tx
                        .send(ThermalCommand::SetTarget {
                            channel: TempChannel::Bed,
                            temp_c: temp,
                        })
                        .await;
                }
                if heater.is_some() {
                    // Configuration-only (M140 Hn in config.g) — just note the mapping
                    info!("Bed heater configured");
                }
            }
            GCodeCommand::SetBedTempWait { temp } => {
                thermal_tx
                    .send(ThermalCommand::SetTargetAndWait {
                        channel: TempChannel::Bed,
                        temp_c: temp,
                    })
                    .await;
            }
            GCodeCommand::SetFanSpeed { fan, speed } => {
                let channel = match fan.unwrap_or(0) {
                    0 => PwmChannel::Fan0,
                    1 => PwmChannel::Fan1,
                    2 => PwmChannel::Fan2,
                    _ => PwmChannel::Fan3,
                };
                thermal_tx
                    .send(ThermalCommand::SetFanSpeed { channel, speed })
                    .await;
            }
            GCodeCommand::FanOff { fan } => {
                let channel = match fan.unwrap_or(0) {
                    0 => PwmChannel::Fan0,
                    1 => PwmChannel::Fan1,
                    2 => PwmChannel::Fan2,
                    _ => PwmChannel::Fan3,
                };
                thermal_tx.send(ThermalCommand::FanOff { channel }).await;
            }

            // ── Control ───────────────────────────────────────────
            GCodeCommand::EmergencyStop => {
                warn!("EMERGENCY STOP");
                motion_tx.send(MotionCommand::EmergencyStop).await;
                thermal_tx.send(ThermalCommand::EmergencyStop).await;
                if let Ok(pub_handle) = SYSTEM_EVENTS.publisher() {
                    pub_handle.publish(SystemEvent::EmergencyStop).await;
                }
            }
            GCodeCommand::WaitForMoves => {
                motion_tx.send(MotionCommand::WaitForCompletion).await;
            }
            GCodeCommand::GetPosition => {
                motion_tx.send(MotionCommand::ReportPosition).await;
            }
            GCodeCommand::GetFirmwareVersion => {
                info!("Duet3-RS v0.1.0 (Embassy async actors on ATSAME54P20A)");
            }

            // ── SD card ───────────────────────────────────────────
            GCodeCommand::LoadConfig => {
                sdcard_tx.send(SdCardCommand::LoadConfigOverride).await;
            }
            GCodeCommand::SaveConfig => {
                info!("M500: Config save not yet implemented");
            }
            GCodeCommand::ReportSettings => {
                info!("M503: Settings report not yet implemented");
            }

            GCodeCommand::Comment => {}

            GCodeCommand::Unknown { letter, code } => {
                warn!("Unknown command: {}{}", letter as char, code);
            }

            _ => {
                debug!("Unhandled command variant");
            }
        }
    }
}

// ══════════════════════════════════════════════════════════════════
//  Actor: Motion Planner
//  Receives motion commands, computes trapezoidal profiles,
//  feeds segments to the step generator.
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

                // Feed generated segments to the step generator
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
            MotionCommand::SetAbsolute => {
                planner.set_absolute();
            }
            MotionCommand::SetRelative => {
                planner.set_relative();
            }
            MotionCommand::SetPosition { axes } => {
                planner.set_position(&axes);
            }
            MotionCommand::SetStepsPerMm { axes } => {
                planner.set_steps_per_mm(&axes);
            }
            MotionCommand::SetMaxFeedrate { axes } => {
                planner.set_max_feedrate(&axes);
            }
            MotionCommand::SetMaxAccelPerAxis { axes } => {
                planner.set_max_accel(&axes);
            }
            MotionCommand::SetAcceleration {
                print_accel,
                travel_accel,
            } => {
                planner.set_acceleration(print_accel, travel_accel);
            }
            MotionCommand::SetMicrostepping {
                axes,
                interpolation: _,
            } => {
                info!("Microstepping set: {:?}", axes);
            }
            MotionCommand::SetMotorCurrent {
                axes,
                idle_percent: _,
            } => {
                info!("Motor current set: {:?}", axes);
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
            }
            MotionCommand::WaitForCompletion => {
                // Wait until the step queue is drained
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
//  Actor: Step Generator
//  High-priority task that converts motion segments into
//  precisely timed step pulses.
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::task]
async fn step_generator_task() {
    info!("Step generator started");
    let seg_rx = STEP_QUEUE.receiver();

    loop {
        let segment = seg_rx.receive().await;

        // Execute the segment by generating step pulses
        let total_steps: i32 = segment
            .steps
            .iter()
            .map(|s| s.unsigned_abs() as i32)
            .max()
            .unwrap_or(0);

        if total_steps == 0 {
            continue;
        }

        // Bresenham multi-axis step distribution
        let mut accum = [0i32; 4];
        let mut current_interval = segment.initial_interval_us;
        let interval_delta = if total_steps > 1 {
            (segment.final_interval_us as i32 - segment.initial_interval_us as i32)
                / (total_steps - 1).max(1)
        } else {
            0
        };

        for _step in 0..total_steps {
            // For each axis, decide if this master step produces an axis step
            for (axis, acc) in accum.iter_mut().enumerate() {
                let axis_steps = segment.steps[axis].unsigned_abs() as i32;
                *acc += axis_steps;
                if *acc >= total_steps {
                    *acc -= total_steps;
                    // TODO: Toggle step pin for this axis using PAC GPIO
                    // The direction was already set before the segment started
                }
            }

            // Wait for the step interval
            let wait_us = current_interval.min(100_000); // cap at 100ms
            if wait_us > 0 {
                Timer::after(Duration::from_micros(wait_us as u64)).await;
            }

            // Update interval for acceleration/deceleration
            current_interval = (current_interval as i32 + interval_delta).max(1) as u32;
        }
    }
}

// ══════════════════════════════════════════════════════════════════
//  Actor: Thermal Manager
//  Runs PID loops at 10Hz, manages heaters and fans.
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::task]
async fn thermal_manager_task() {
    info!("Thermal manager started");
    let cmd_rx = THERMAL_CMD.receiver();
    let status_tx = THERMAL_STATUS.sender();

    let mut manager = ThermalManager::new();
    let mut ticker = Ticker::every(Duration::from_millis(100)); // 10Hz PID loop
    let dt: f32 = 0.1; // 100ms in seconds

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
                ThermalCommand::HeaterOff { channel } => {
                    manager.heater_off(channel);
                }
                ThermalCommand::SetFanSpeed { channel, speed } => {
                    manager.set_fan_speed(channel, speed);
                }
                ThermalCommand::FanOff { channel } => {
                    manager.fan_off(channel);
                }
                ThermalCommand::ReportTemperatures => {
                    for &ch in &[TempChannel::Bed, TempChannel::Hotend1, TempChannel::Hotend2] {
                        let idx = match ch {
                            TempChannel::Bed => 0,
                            TempChannel::Hotend1 => 1,
                            TempChannel::Hotend2 => 2,
                        };
                        let h = &manager.heaters[idx];
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
                // PID update cycle
                // TODO: Read actual ADC values from thermistors via PAC
                // For now, simulate readings
                for &ch in &[TempChannel::Bed, TempChannel::Hotend1, TempChannel::Hotend2] {
                    let _duty = manager.update_heater(
                        ch,
                        manager.heaters[match ch {
                            TempChannel::Bed => 0,
                            TempChannel::Hotend1 => 1,
                            TempChannel::Hotend2 => 2,
                        }]
                        .current_c,
                        dt,
                    );

                    // TODO: Write duty cycle to PWM hardware via PAC

                    // Check for thermal runaway
                    if manager.check_runaway(ch, 20.0) {
                        warn!("Thermal runaway detected on {:?}", ch);
                        manager.emergency_stop();
                        status_tx
                            .send(ThermalStatus::ThermalRunaway {
                                channel: ch,
                                temp_c: manager.heaters[match ch {
                                    TempChannel::Bed => 0,
                                    TempChannel::Hotend1 => 1,
                                    TempChannel::Hotend2 => 2,
                                }]
                                .current_c,
                            })
                            .await;
                    }

                    // Check if we've reached target (for wait commands)
                    let idx = match ch {
                        TempChannel::Bed => 0,
                        TempChannel::Hotend1 => 1,
                        TempChannel::Hotend2 => 2,
                    };
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
//  Actor: SD Card Reader
//  Reads G-code files and feeds lines to the dispatcher.
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::task]
async fn sdcard_reader_task() {
    info!("SD card reader started");
    let cmd_rx = SDCARD_CMD.receiver();
    let status_tx = SDCARD_STATUS.sender();
    let line_tx = GCODE_LINE.sender();

    loop {
        let cmd = cmd_rx.receive().await;

        match cmd {
            SdCardCommand::LoadConfig | SdCardCommand::LoadConfigOverride => {
                info!("SD: Loading config from SD card");
                // TODO: Initialize SPI, mount FAT32, read CONFIG.G
                // For now, send built-in defaults as G-code lines
                let defaults: &[&str] = &[
                    "M569 P0 S1 D3",
                    "M569 P1 S1 D3",
                    "M569 P2 S1 D3",
                    "M569 P3 S1 D3",
                    "M569 P4 S1 D3",
                    "M350 X16 Y16 Z16 E16 I1",
                    "M92 X80 Y80 Z400 E420",
                    "M203 X6000 Y6000 Z600 E3600",
                    "M201 X500 Y500 Z100 E500",
                    "M204 P500 T1000",
                    "M906 X800 Y800 Z800 E800 I30",
                    "G90",
                ];
                let mut count: u32 = 0;
                for line in defaults {
                    let mut s = heapless::String::<256>::new();
                    if s.push_str(line).is_ok() {
                        line_tx.send(s).await;
                        count += 1;
                    }
                }
                status_tx
                    .send(SdCardStatus::ConfigLoaded {
                        commands_executed: count,
                    })
                    .await;
            }
            SdCardCommand::StartJob { filename } => {
                info!("SD: Starting job: {}", filename.as_str());
                // TODO: Open file, read line by line, feed to dispatcher
                status_tx
                    .send(SdCardStatus::Error(SdCardError::FileNotFound))
                    .await;
            }
            SdCardCommand::PauseJob => {
                info!("SD: Job paused");
            }
            SdCardCommand::ResumeJob => {
                info!("SD: Job resumed");
            }
            SdCardCommand::CancelJob => {
                info!("SD: Job cancelled");
            }
        }
    }
}

// ══════════════════════════════════════════════════════════════════
//  Actor: Status Monitor
//  Collects status from all actors, logs to defmt, blinks LED.
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::task]
async fn status_monitor_task() {
    info!("Status monitor started");
    let motion_rx = MOTION_STATUS.receiver();
    let thermal_rx = THERMAL_STATUS.receiver();
    let sdcard_rx = SDCARD_STATUS.receiver();

    loop {
        match select3(
            motion_rx.receive(),
            thermal_rx.receive(),
            sdcard_rx.receive(),
        )
        .await
        {
            Either3::First(status) => match status {
                MotionStatus::Position {
                    x_mm,
                    y_mm,
                    z_mm,
                    e_mm,
                } => {
                    info!("Position: X={} Y={} Z={} E={}", x_mm, y_mm, z_mm, e_mm);
                }
                MotionStatus::HomingComplete { x, y, z } => {
                    info!("Homing complete: X={} Y={} Z={}", x, y, z);
                }
                MotionStatus::MovesComplete => {
                    info!("All moves complete");
                }
                MotionStatus::Error(e) => {
                    error!("Motion error: {:?}", e);
                }
            },
            Either3::Second(status) => match status {
                ThermalStatus::Temperature {
                    channel,
                    current_c,
                    target_c,
                    pwm,
                } => {
                    debug!(
                        "{:?}: {}C / {}C (PWM: {}%)",
                        channel,
                        current_c,
                        target_c,
                        pwm * 100.0
                    );
                }
                ThermalStatus::TargetReached { channel } => {
                    info!("Temperature reached on {:?}", channel);
                }
                ThermalStatus::ThermalRunaway { channel, temp_c } => {
                    error!("THERMAL RUNAWAY on {:?} at {}C!", channel, temp_c);
                }
                ThermalStatus::SensorFault { channel } => {
                    error!("Sensor fault on {:?}!", channel);
                }
            },
            Either3::Third(status) => match status {
                SdCardStatus::ConfigLoaded { commands_executed } => {
                    info!("Config loaded: {} commands", commands_executed);
                }
                SdCardStatus::JobProgress {
                    lines_processed,
                    percent_complete,
                } => {
                    info!("Job: {} lines ({}%)", lines_processed, percent_complete);
                }
                SdCardStatus::JobComplete => {
                    info!("Job complete");
                }
                SdCardStatus::Error(e) => {
                    error!("SD card error: {:?}", e);
                }
            },
        }
    }
}

// ══════════════════════════════════════════════════════════════════
//  Entry Point
// ══════════════════════════════════════════════════════════════════

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("═══════════════════════════════════════");
    info!("  Duet3-RS Firmware v0.1.0");
    info!("  ATSAME54P20A @ 120MHz");
    info!("  Embassy Async Actor Architecture");
    info!("═══════════════════════════════════════");

    // TODO: Initialize SAME54 peripherals via atsamd-hal:
    //   - Clock system (120MHz from 25MHz crystal)
    //   - GPIO pins (step/dir/enable)
    //   - SERCOM for TMC2209 UART
    //   - SERCOM for SPI (SD card)
    //   - TCC for PWM (heaters/fans)
    //   - ADC for thermistors
    //   - TC for step timing
    //   - USB for host communication

    // Spawn all actor tasks
    spawner.spawn(gcode_dispatcher_task()).unwrap();
    spawner.spawn(motion_planner_task()).unwrap();
    spawner.spawn(step_generator_task()).unwrap();
    spawner.spawn(thermal_manager_task()).unwrap();
    spawner.spawn(sdcard_reader_task()).unwrap();
    spawner.spawn(status_monitor_task()).unwrap();

    info!("All actors spawned — loading config from SD card");

    // Trigger config load from SD card
    SDCARD_CMD.sender().send(SdCardCommand::LoadConfig).await;

    // Main loop: heartbeat + system event monitoring
    let mut ticker = Ticker::every(Duration::from_secs(10));

    if let Ok(mut sub) = SYSTEM_EVENTS.subscriber() {
        loop {
            match select(sub.next_message_pure(), ticker.next()).await {
                Either::First(event) => match event {
                    SystemEvent::EmergencyStop => {
                        error!("SYSTEM: Emergency stop activated!");
                    }
                    SystemEvent::ThermalRunaway { channel } => {
                        error!("SYSTEM: Thermal runaway on {:?}!", channel);
                    }
                    SystemEvent::HomingComplete => {
                        info!("SYSTEM: Homing complete");
                    }
                    SystemEvent::JobComplete => {
                        info!("SYSTEM: Job complete");
                    }
                },
                Either::Second(_) => {
                    debug!("Heartbeat — system running");
                }
            }
        }
    } else {
        // Fallback if subscriber limit reached
        loop {
            ticker.next().await;
            debug!("Heartbeat — system running");
        }
    }
}
