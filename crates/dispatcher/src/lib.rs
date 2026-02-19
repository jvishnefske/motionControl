//! # G-code Dispatcher — Command Routing
//!
//! Parses G-code lines and produces typed commands for motion, thermal, and
//! SD card actors. Fully portable — no hardware or async runtime dependencies.

#![no_std]

use gcode_parser::{self, GCodeCommand};
use motion_planner::MotionCommand;
use printer_hal::{PwmChannel, TempChannel};
use sdcard::SdCardCommand;
use thermal::ThermalCommand;

/// A dispatched command ready for routing to the appropriate actor.
#[derive(Debug)]
pub enum DispatchAction {
    Motion(MotionCommand),
    Thermal(ThermalCommand),
    SdCard(SdCardCommand),
    Log(&'static str),
    EmergencyStop,
    Noop,
}

/// Parse a G-code line and return the action(s) to dispatch.
///
/// Returns a primary action and an optional secondary action
/// (e.g., emergency stop sends to both motion and thermal).
pub fn dispatch_line(line: &str) -> (DispatchAction, Option<DispatchAction>) {
    let cmd = gcode_parser::parse_line(line);
    dispatch_command(cmd)
}

/// Route a parsed GCodeCommand to the appropriate actor.
pub fn dispatch_command(cmd: GCodeCommand) -> (DispatchAction, Option<DispatchAction>) {
    match cmd {
        // ── Motion ────────────────────────────────────────────
        GCodeCommand::LinearMove { axes, feedrate, .. } => {
            let is_rapid = feedrate.is_none();
            (
                DispatchAction::Motion(MotionCommand::LinearMove {
                    target: axes,
                    feedrate_mm_min: feedrate.unwrap_or(0.0),
                    is_rapid,
                }),
                None,
            )
        }
        GCodeCommand::Home { axes } => (
            DispatchAction::Motion(MotionCommand::Home {
                x: !axes.any() || axes.x,
                y: !axes.any() || axes.y,
                z: !axes.any() || axes.z,
            }),
            None,
        ),
        GCodeCommand::AbsolutePositioning => {
            (DispatchAction::Motion(MotionCommand::SetAbsolute), None)
        }
        GCodeCommand::RelativePositioning => {
            (DispatchAction::Motion(MotionCommand::SetRelative), None)
        }
        GCodeCommand::SetPosition { axes } => (
            DispatchAction::Motion(MotionCommand::SetPosition { axes }),
            None,
        ),

        // ── Configuration ─────────────────────────────────────
        GCodeCommand::SetStepsPerMm { axes } => (
            DispatchAction::Motion(MotionCommand::SetStepsPerMm { axes }),
            None,
        ),
        GCodeCommand::SetMaxFeedrate { axes } => (
            DispatchAction::Motion(MotionCommand::SetMaxFeedrate { axes }),
            None,
        ),
        GCodeCommand::SetMaxAccelPerAxis { axes } => (
            DispatchAction::Motion(MotionCommand::SetMaxAccelPerAxis { axes }),
            None,
        ),
        GCodeCommand::SetAcceleration {
            print_accel,
            travel_accel,
        } => (
            DispatchAction::Motion(MotionCommand::SetAcceleration {
                print_accel,
                travel_accel,
            }),
            None,
        ),
        GCodeCommand::SetMicrostepping {
            axes,
            interpolation,
        } => (
            DispatchAction::Motion(MotionCommand::SetMicrostepping {
                axes,
                interpolation,
            }),
            None,
        ),
        GCodeCommand::SetMotorCurrent { axes, idle_percent } => (
            DispatchAction::Motion(MotionCommand::SetMotorCurrent { axes, idle_percent }),
            None,
        ),
        GCodeCommand::SetDriverConfig {
            driver,
            direction,
            mode,
        } => {
            let stealthchop = mode.map(|m| matches!(m, gcode_parser::DriverMode::StealthChop));
            (
                DispatchAction::Motion(MotionCommand::SetDriverConfig {
                    driver,
                    direction,
                    stealthchop,
                }),
                None,
            )
        }

        // ── Temperature ───────────────────────────────────────
        GCodeCommand::SetHotendTemp { temp, .. } => (
            DispatchAction::Thermal(ThermalCommand::SetTarget {
                channel: TempChannel::Hotend1,
                temp_c: temp,
            }),
            None,
        ),
        GCodeCommand::SetHotendTempWait { temp, .. } => (
            DispatchAction::Thermal(ThermalCommand::SetTargetAndWait {
                channel: TempChannel::Hotend1,
                temp_c: temp,
            }),
            None,
        ),
        GCodeCommand::SetBedTemp { temp, .. } => {
            if let Some(temp) = temp {
                (
                    DispatchAction::Thermal(ThermalCommand::SetTarget {
                        channel: TempChannel::Bed,
                        temp_c: temp,
                    }),
                    None,
                )
            } else {
                (DispatchAction::Log("Bed heater configured"), None)
            }
        }
        GCodeCommand::SetBedTempWait { temp } => (
            DispatchAction::Thermal(ThermalCommand::SetTargetAndWait {
                channel: TempChannel::Bed,
                temp_c: temp,
            }),
            None,
        ),
        GCodeCommand::SetFanSpeed { fan, speed } => {
            let channel = match fan.unwrap_or(0) {
                0 => PwmChannel::Fan0,
                1 => PwmChannel::Fan1,
                2 => PwmChannel::Fan2,
                _ => PwmChannel::Fan3,
            };
            (
                DispatchAction::Thermal(ThermalCommand::SetFanSpeed { channel, speed }),
                None,
            )
        }
        GCodeCommand::FanOff { fan } => {
            let channel = match fan.unwrap_or(0) {
                0 => PwmChannel::Fan0,
                1 => PwmChannel::Fan1,
                2 => PwmChannel::Fan2,
                _ => PwmChannel::Fan3,
            };
            (
                DispatchAction::Thermal(ThermalCommand::FanOff { channel }),
                None,
            )
        }

        // ── Control ───────────────────────────────────────────
        GCodeCommand::EmergencyStop => (DispatchAction::EmergencyStop, None),
        GCodeCommand::WaitForMoves => (
            DispatchAction::Motion(MotionCommand::WaitForCompletion),
            None,
        ),
        GCodeCommand::GetPosition => (DispatchAction::Motion(MotionCommand::ReportPosition), None),
        GCodeCommand::GetFirmwareVersion => (
            DispatchAction::Log("Duet3-RS v0.1.0 (Embassy async actors)"),
            None,
        ),

        // ── SD card ───────────────────────────────────────────
        GCodeCommand::LoadConfig => (
            DispatchAction::SdCard(SdCardCommand::LoadConfigOverride),
            None,
        ),
        GCodeCommand::SaveConfig => (
            DispatchAction::Log("M500: Config save not yet implemented"),
            None,
        ),
        GCodeCommand::ReportSettings => (
            DispatchAction::Log("M503: Settings report not yet implemented"),
            None,
        ),

        GCodeCommand::Comment => (DispatchAction::Noop, None),

        GCodeCommand::Unknown { .. } => (DispatchAction::Noop, None),

        _ => (DispatchAction::Noop, None),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dispatch_g1_move() {
        let (action, secondary) = dispatch_line("G1 X10 Y20 F3000");
        assert!(secondary.is_none());
        match action {
            DispatchAction::Motion(MotionCommand::LinearMove {
                target,
                feedrate_mm_min,
                is_rapid,
            }) => {
                assert_eq!(target.x, Some(10.0));
                assert_eq!(target.y, Some(20.0));
                assert_eq!(feedrate_mm_min, 3000.0);
                assert!(!is_rapid);
            }
            _ => panic!("Expected LinearMove"),
        }
    }

    #[test]
    fn test_dispatch_g0_rapid() {
        let (action, _) = dispatch_line("G0 X50");
        match action {
            DispatchAction::Motion(MotionCommand::LinearMove { is_rapid, .. }) => {
                assert!(is_rapid);
            }
            _ => panic!("Expected rapid LinearMove"),
        }
    }

    #[test]
    fn test_dispatch_g28_home_all() {
        let (action, _) = dispatch_line("G28");
        match action {
            DispatchAction::Motion(MotionCommand::Home { x, y, z }) => {
                assert!(x && y && z);
            }
            _ => panic!("Expected Home"),
        }
    }

    #[test]
    fn test_dispatch_m104_hotend() {
        let (action, _) = dispatch_line("M104 S200");
        match action {
            DispatchAction::Thermal(ThermalCommand::SetTarget { channel, temp_c }) => {
                assert_eq!(channel, TempChannel::Hotend1);
                assert_eq!(temp_c, 200.0);
            }
            _ => panic!("Expected SetTarget"),
        }
    }

    #[test]
    fn test_dispatch_m140_bed() {
        let (action, _) = dispatch_line("M140 S60");
        match action {
            DispatchAction::Thermal(ThermalCommand::SetTarget { channel, temp_c }) => {
                assert_eq!(channel, TempChannel::Bed);
                assert_eq!(temp_c, 60.0);
            }
            _ => panic!("Expected bed SetTarget"),
        }
    }

    #[test]
    fn test_dispatch_m106_fan() {
        let (action, _) = dispatch_line("M106 P1 S0.5");
        match action {
            DispatchAction::Thermal(ThermalCommand::SetFanSpeed { channel, speed }) => {
                assert_eq!(channel, PwmChannel::Fan1);
                assert!((speed - 0.5).abs() < 0.01);
            }
            _ => panic!("Expected SetFanSpeed"),
        }
    }

    #[test]
    fn test_dispatch_emergency_stop() {
        let (action, _) = dispatch_line("M112");
        assert!(matches!(action, DispatchAction::EmergencyStop));
    }

    #[test]
    fn test_dispatch_comment() {
        let (action, _) = dispatch_line("; this is a comment");
        assert!(matches!(action, DispatchAction::Noop));
    }

    #[test]
    fn test_dispatch_m92_config() {
        let (action, _) = dispatch_line("M92 X80 Y80 Z400 E420");
        match action {
            DispatchAction::Motion(MotionCommand::SetStepsPerMm { axes }) => {
                assert_eq!(axes.x, Some(80.0));
                assert_eq!(axes.z, Some(400.0));
            }
            _ => panic!("Expected SetStepsPerMm"),
        }
    }

    #[test]
    fn test_dispatch_g90_absolute() {
        let (action, _) = dispatch_line("G90");
        assert!(matches!(
            action,
            DispatchAction::Motion(MotionCommand::SetAbsolute)
        ));
    }

    #[test]
    fn test_dispatch_g91_relative() {
        let (action, _) = dispatch_line("G91");
        assert!(matches!(
            action,
            DispatchAction::Motion(MotionCommand::SetRelative)
        ));
    }
}
