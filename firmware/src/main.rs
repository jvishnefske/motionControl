//! # Duet 3 Mini 5+ Firmware — Hardware Wiring Layer
//!
//! This binary crate contains ONLY:
//! - Static channel/event-bus allocation
//! - Embassy task wrappers that call into portable library crates
//! - Hardware peripheral initialization (TODO: PAC/HAL setup)
//!
//! All business logic lives in library crates:
//! - `dispatcher` — G-code parsing and command routing
//! - `motion-planner` — trapezoidal profiles + step generation
//! - `thermal` — PID control + heater/fan management
//! - `sdcard` — line reader + file protocol

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
use dispatcher::{dispatch_line, DispatchAction};
use motion_planner::{MotionCommand, MotionPlanner, MotionSegment, MotionStatus};
use printer_hal::NullFs;
use printer_hal::TempChannel;
use sdcard::{SdCardCommand, SdCardError, SdCardStatus};
use thermal::{ThermalCommand, ThermalManager, ThermalStatus};

// ══════════════════════════════════════════════════════════════════
//  Static Channels (actor mailboxes)
// ══════════════════════════════════════════════════════════════════

static MOTION_CMD: Channel<CriticalSectionRawMutex, MotionCommand, 16> = Channel::new();
static MOTION_STATUS: Channel<CriticalSectionRawMutex, MotionStatus, 8> = Channel::new();
static THERMAL_CMD: Channel<CriticalSectionRawMutex, ThermalCommand, 8> = Channel::new();
static THERMAL_STATUS: Channel<CriticalSectionRawMutex, ThermalStatus, 8> = Channel::new();
static SDCARD_CMD: Channel<CriticalSectionRawMutex, SdCardCommand, 4> = Channel::new();
static SDCARD_STATUS: Channel<CriticalSectionRawMutex, SdCardStatus, 4> = Channel::new();
static STEP_QUEUE: Channel<CriticalSectionRawMutex, MotionSegment, 8> = Channel::new();
static GCODE_LINE: Channel<CriticalSectionRawMutex, heapless::String<256>, 16> = Channel::new();
static SYSTEM_EVENTS: EventBus<SystemEvent, 8, 4, 4> = EventBus::new();

#[derive(Clone, Debug, defmt::Format)]
pub enum SystemEvent {
    EmergencyStop,
    ThermalRunaway { channel: TempChannel },
    HomingComplete,
    JobComplete,
}

// ══════════════════════════════════════════════════════════════════
//  Task: G-code Dispatcher (thin wrapper around dispatcher crate)
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
        let (primary, secondary) = dispatch_line(line.as_str());

        for action in [Some(primary), secondary].into_iter().flatten() {
            match action {
                DispatchAction::Motion(cmd) => motion_tx.send(cmd).await,
                DispatchAction::Thermal(cmd) => thermal_tx.send(cmd).await,
                DispatchAction::SdCard(cmd) => sdcard_tx.send(cmd).await,
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
            }
            MotionCommand::SetMotorCurrent { axes, .. } => {
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
//  Task: Step Generator (uses portable step_generator from library)
// ══════════════════════════════════════════════════════════════════

/// Duet 3 hardware stepper driver — TODO: wire to actual GPIO.
struct Duet3Stepper;

impl printer_hal::StepperDriver for Duet3Stepper {
    fn set_direction(&mut self, _axis: u8, _forward: bool) {
        // TODO: Set direction pin via PAC GPIO
    }

    fn step(&mut self, _axis: u8) {
        // TODO: Toggle step pin via PAC GPIO
    }

    fn enable(&mut self, _axis: u8, _enabled: bool) {
        // TODO: Set enable pin via PAC GPIO
    }
}

#[embassy_executor::task]
async fn step_generator_task() {
    info!("Step generator started");
    let seg_rx = STEP_QUEUE.receiver();
    let mut driver = Duet3Stepper;

    loop {
        let segment = seg_rx.receive().await;

        motion_planner::execute_segment(&segment, &mut driver, |wait_us| {
            // Embassy timer wait is handled below — here we just
            // record the interval. In a real ISR-based implementation,
            // this would set the timer compare register.
            let _ = wait_us;
        });

        // For now, wait the segment duration (coarse timing).
        // Real implementation will use per-step Timer::after in the callback.
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
                    for &ch in &TempChannel::ALL {
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
                // PID update cycle
                for &ch in &TempChannel::ALL {
                    // TODO: Read actual ADC thermistor via printer_hal::TemperatureSensor
                    let current = manager.heaters[ch.index()].current_c;
                    let _duty = manager.update_heater(ch, current, dt);
                    // TODO: Write duty via printer_hal::HeaterOutput

                    if manager.check_runaway(ch, 20.0) {
                        warn!("Thermal runaway detected on {:?}", ch);
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
//  Task: SD Card Reader
// ══════════════════════════════════════════════════════════════════

/// Duet 3 Mini 5+ built-in defaults.
/// Used when CONFIG.G is not found on the SD card.
const DUET3_DEFAULTS: &[&str] = &[
    "M569 P0 S1 D3", // X driver
    "M569 P1 S1 D3", // Y driver
    "M569 P2 S1 D3", // Z driver
    "M569 P3 S1 D3", // E0 driver
    "M569 P4 S1 D3", // E1 driver
    "M350 X16 Y16 Z16 E16 I1",
    "M92 X80 Y80 Z400 E420",
    "M203 X6000 Y6000 Z600 E3600",
    "M201 X500 Y500 Z100 E500",
    "M204 P500 T1000",
    "M906 X800 Y800 Z800 E800 I30",
    "G90",
];

#[embassy_executor::task]
async fn sdcard_reader_task() {
    info!("SD card reader started");
    let cmd_rx = SDCARD_CMD.receiver();
    let status_tx = SDCARD_STATUS.sender();
    let line_tx = GCODE_LINE.sender();

    // TODO: Replace NullFs with actual SD card FileSystem impl
    // once SPI + FAT32 driver is wired up via board-hal.
    let mut fs = NullFs;

    loop {
        let cmd = cmd_rx.receive().await;

        match cmd {
            SdCardCommand::LoadConfig | SdCardCommand::LoadConfigOverride => {
                info!("SD: Loading config (CONFIG.G → fallback defaults)");

                let result = sdcard::load_config_with_fallback(&mut fs, DUET3_DEFAULTS, |line| {
                    let mut s = heapless::String::<256>::new();
                    if s.push_str(line).is_ok() {
                        // Can't .await inside closure — use try_send
                        let _ = line_tx.try_send(s);
                    }
                });

                if result.from_file {
                    info!(
                        "SD: Loaded CONFIG.G ({} commands)",
                        result.commands_executed
                    );
                } else {
                    info!(
                        "SD: No CONFIG.G found, using defaults ({} commands)",
                        result.commands_executed
                    );
                }

                status_tx
                    .send(SdCardStatus::ConfigLoaded {
                        commands_executed: result.commands_executed,
                    })
                    .await;
            }
            SdCardCommand::StartJob { filename } => {
                info!("SD: Starting job: {}", filename.as_str());
                status_tx
                    .send(SdCardStatus::Error(SdCardError::FileNotFound))
                    .await;
            }
            SdCardCommand::PauseJob => info!("SD: Job paused"),
            SdCardCommand::ResumeJob => info!("SD: Job resumed"),
            SdCardCommand::CancelJob => info!("SD: Job cancelled"),
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
                } => info!("Position: X={} Y={} Z={} E={}", x_mm, y_mm, z_mm, e_mm),
                MotionStatus::HomingComplete { x, y, z } => {
                    info!("Homing complete: X={} Y={} Z={}", x, y, z)
                }
                MotionStatus::MovesComplete => info!("All moves complete"),
                MotionStatus::Error(e) => error!("Motion error: {:?}", e),
            },
            Either3::Second(status) => match status {
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
            Either3::Third(status) => match status {
                SdCardStatus::ConfigLoaded { commands_executed } => {
                    info!("Config loaded: {} commands", commands_executed)
                }
                SdCardStatus::JobProgress {
                    lines_processed,
                    percent_complete,
                } => info!("Job: {} lines ({}%)", lines_processed, percent_complete),
                SdCardStatus::JobComplete => info!("Job complete"),
                SdCardStatus::Error(e) => error!("SD card error: {:?}", e),
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
    info!("  Duet3-RS Firmware v0.1.0");
    info!("  ATSAME54P20A @ 120MHz");
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

    spawner.spawn(gcode_dispatcher_task()).unwrap();
    spawner.spawn(motion_planner_task()).unwrap();
    spawner.spawn(step_generator_task()).unwrap();
    spawner.spawn(thermal_manager_task()).unwrap();
    spawner.spawn(sdcard_reader_task()).unwrap();
    spawner.spawn(status_monitor_task()).unwrap();

    info!("All actors spawned — loading config");
    SDCARD_CMD.sender().send(SdCardCommand::LoadConfig).await;

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
                    SystemEvent::JobComplete => info!("SYSTEM: Job complete"),
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
