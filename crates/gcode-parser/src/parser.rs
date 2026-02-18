//! Zero-allocation G-code line parser.
//!
//! Parses a single G-code line (as `&str`) into a `GCodeCommand`.
//! Handles comments (`;`), parameter extraction, and float parsing
//! without any heap allocation.

use crate::commands::GCodeCommand;
use crate::types::*;

/// Parse a single G-code line into a typed command.
pub fn parse_line(line: &str) -> GCodeCommand {
    let line = strip_comment(line).trim();
    if line.is_empty() {
        return GCodeCommand::Comment;
    }

    let bytes = line.as_bytes();
    if bytes.is_empty() {
        return GCodeCommand::Comment;
    }

    let letter = bytes[0].to_ascii_uppercase();
    let rest = &line[1..];

    let code = match parse_u16(rest) {
        Some((c, _)) => c,
        None => return GCodeCommand::Comment,
    };

    // Find where the code number ends and parameters begin
    let params = skip_number(rest);

    match letter {
        b'G' => parse_g_command(code, params),
        b'M' => parse_m_command(code, params),
        _ => GCodeCommand::Unknown { letter, code },
    }
}

fn parse_g_command(code: u16, params: &str) -> GCodeCommand {
    match code {
        0 | 1 => {
            let axes = parse_axis_values(params);
            let feedrate = parse_param_f32(params, b'F');
            let endstop_action = parse_param_f32(params, b'H').map(|v| v as u8);
            GCodeCommand::LinearMove {
                axes,
                feedrate,
                endstop_action,
            }
        }
        28 => {
            let axes = AxisFlags {
                x: has_param(params, b'X'),
                y: has_param(params, b'Y'),
                z: has_param(params, b'Z'),
            };
            GCodeCommand::Home { axes }
        }
        90 => GCodeCommand::AbsolutePositioning,
        91 => GCodeCommand::RelativePositioning,
        92 => GCodeCommand::SetPosition {
            axes: parse_axis_values(params),
        },
        _ => GCodeCommand::Unknown {
            letter: b'G',
            code,
        },
    }
}

fn parse_m_command(code: u16, params: &str) -> GCodeCommand {
    match code {
        92 => GCodeCommand::SetStepsPerMm {
            axes: parse_axis_values(params),
        },
        104 => GCodeCommand::SetHotendTemp {
            temp: parse_param_f32(params, b'S').unwrap_or(0.0),
            tool: parse_param_f32(params, b'T').map(|v| v as u8),
        },
        106 => GCodeCommand::SetFanSpeed {
            fan: parse_param_f32(params, b'P').map(|v| v as u8),
            speed: parse_param_f32(params, b'S').unwrap_or(1.0),
        },
        107 => GCodeCommand::FanOff {
            fan: parse_param_f32(params, b'P').map(|v| v as u8),
        },
        109 => GCodeCommand::SetHotendTempWait {
            temp: parse_param_f32(params, b'S').unwrap_or(0.0),
            tool: parse_param_f32(params, b'T').map(|v| v as u8),
        },
        112 => GCodeCommand::EmergencyStop,
        114 => GCodeCommand::GetPosition,
        115 => GCodeCommand::GetFirmwareVersion,
        140 => GCodeCommand::SetBedTemp {
            temp: parse_param_f32(params, b'S'),
            heater: parse_param_f32(params, b'H').map(|v| v as u8),
        },
        190 => GCodeCommand::SetBedTempWait {
            temp: parse_param_f32(params, b'S').unwrap_or(0.0),
        },
        201 => GCodeCommand::SetMaxAccelPerAxis {
            axes: parse_axis_values(params),
        },
        203 => GCodeCommand::SetMaxFeedrate {
            axes: parse_axis_values(params),
        },
        204 => GCodeCommand::SetAcceleration {
            print_accel: parse_param_f32(params, b'P'),
            travel_accel: parse_param_f32(params, b'T'),
        },
        206 => GCodeCommand::SetHomeOffset {
            axes: parse_axis_values(params),
        },
        208 => GCodeCommand::SetAxisLimits {
            axes: parse_axis_values(params),
            max: parse_param_f32(params, b'S').map_or(true, |v| v == 0.0),
        },
        350 => GCodeCommand::SetMicrostepping {
            axes: parse_axis_values(params),
            interpolation: parse_param_f32(params, b'I').map(|v| v != 0.0),
        },
        400 => GCodeCommand::WaitForMoves,
        500 => GCodeCommand::SaveConfig,
        501 => GCodeCommand::LoadConfig,
        503 => GCodeCommand::ReportSettings,
        569 => {
            let driver = parse_param_f32(params, b'P').unwrap_or(0.0) as u8;
            let direction = parse_param_f32(params, b'S').map(|v| v != 0.0);
            let mode = parse_param_f32(params, b'D').map(|v| match v as u8 {
                0 => DriverMode::ConstantOffTime,
                1 => DriverMode::RandomOffTime,
                2 => DriverMode::SpreadCycle,
                3 => DriverMode::StealthChop,
                _ => DriverMode::SpreadCycle,
            });
            GCodeCommand::SetDriverConfig {
                driver,
                direction,
                mode,
            }
        }
        584 => GCodeCommand::SetDriveMapping {
            axes: parse_axis_values(params),
        },
        906 => GCodeCommand::SetMotorCurrent {
            axes: parse_axis_values(params),
            idle_percent: parse_param_f32(params, b'I'),
        },
        _ => GCodeCommand::Unknown {
            letter: b'M',
            code,
        },
    }
}

// ── Helper functions ──────────────────────────────────────────────

/// Strip everything after `;` (comment marker).
fn strip_comment(line: &str) -> &str {
    match line.find(';') {
        Some(pos) => &line[..pos],
        None => line,
    }
}

/// Parse a u16 from the beginning of a string. Returns (value, chars_consumed).
fn parse_u16(s: &str) -> Option<(u16, usize)> {
    let bytes = s.as_bytes();
    let mut i = 0;
    // skip whitespace
    while i < bytes.len() && bytes[i] == b' ' {
        i += 1;
    }
    let start = i;
    let mut val: u16 = 0;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        val = val.wrapping_mul(10).wrapping_add((bytes[i] - b'0') as u16);
        i += 1;
    }
    if i == start {
        return None;
    }
    Some((val, i))
}

/// Skip past the command number to get to parameters.
fn skip_number(s: &str) -> &str {
    let bytes = s.as_bytes();
    let mut i = 0;
    while i < bytes.len() && (bytes[i].is_ascii_digit() || bytes[i] == b' ' || bytes[i] == b'.') {
        i += 1;
    }
    &s[i..]
}

/// Check if a parameter letter exists in the parameter string.
fn has_param(params: &str, letter: u8) -> bool {
    let upper = letter.to_ascii_uppercase();
    for b in params.as_bytes() {
        if b.to_ascii_uppercase() == upper {
            return true;
        }
    }
    false
}

/// Extract a float value following a parameter letter.
fn parse_param_f32(params: &str, letter: u8) -> Option<f32> {
    let upper = letter.to_ascii_uppercase();
    let bytes = params.as_bytes();
    let mut i = 0;

    while i < bytes.len() {
        if bytes[i].to_ascii_uppercase() == upper {
            i += 1;
            return Some(parse_f32_at(bytes, &mut i));
        }
        i += 1;
    }
    None
}

/// Parse axis values (X, Y, Z, E) from a parameter string.
fn parse_axis_values(params: &str) -> AxisValues {
    AxisValues {
        x: parse_param_f32(params, b'X'),
        y: parse_param_f32(params, b'Y'),
        z: parse_param_f32(params, b'Z'),
        e: parse_param_f32(params, b'E'),
    }
}

/// Parse a float starting at position `i` in `bytes`. Advances `i` past the number.
fn parse_f32_at(bytes: &[u8], i: &mut usize) -> f32 {
    // skip whitespace
    while *i < bytes.len() && bytes[*i] == b' ' {
        *i += 1;
    }

    let negative = if *i < bytes.len() && bytes[*i] == b'-' {
        *i += 1;
        true
    } else {
        false
    };

    let mut integer_part: i64 = 0;
    let mut has_integer = false;
    while *i < bytes.len() && bytes[*i].is_ascii_digit() {
        integer_part = integer_part * 10 + (bytes[*i] - b'0') as i64;
        *i += 1;
        has_integer = true;
    }

    let mut frac_part: f32 = 0.0;
    if *i < bytes.len() && bytes[*i] == b'.' {
        *i += 1;
        let mut divisor: f32 = 10.0;
        while *i < bytes.len() && bytes[*i].is_ascii_digit() {
            frac_part += (bytes[*i] - b'0') as f32 / divisor;
            divisor *= 10.0;
            *i += 1;
        }
    }

    if !has_integer && frac_part == 0.0 {
        return 0.0;
    }

    let val = integer_part as f32 + frac_part;
    if negative { -val } else { val }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_g0() {
        match parse_line("G0 X10 Y20.5 F3000") {
            GCodeCommand::LinearMove { axes, feedrate, .. } => {
                assert!((axes.x.unwrap() - 10.0).abs() < 0.01);
                assert!((axes.y.unwrap() - 20.5).abs() < 0.01);
                assert!(axes.z.is_none());
                assert!((feedrate.unwrap() - 3000.0).abs() < 0.01);
            }
            _ => panic!("Expected LinearMove"),
        }
    }

    #[test]
    fn test_parse_g1_with_extrusion() {
        match parse_line("G1 X90.6 Y13.8 E22.4 F1500") {
            GCodeCommand::LinearMove { axes, feedrate, .. } => {
                assert!((axes.x.unwrap() - 90.6).abs() < 0.1);
                assert!((axes.e.unwrap() - 22.4).abs() < 0.1);
                assert!((feedrate.unwrap() - 1500.0).abs() < 0.1);
            }
            _ => panic!("Expected LinearMove"),
        }
    }

    #[test]
    fn test_parse_g28_all() {
        match parse_line("G28") {
            GCodeCommand::Home { axes } => {
                assert!(!axes.any());
            }
            _ => panic!("Expected Home"),
        }
    }

    #[test]
    fn test_parse_g28_partial() {
        match parse_line("G28 X Y") {
            GCodeCommand::Home { axes } => {
                assert!(axes.x);
                assert!(axes.y);
                assert!(!axes.z);
            }
            _ => panic!("Expected Home"),
        }
    }

    #[test]
    fn test_parse_m92() {
        match parse_line("M92 X80 Y80 Z400 E420") {
            GCodeCommand::SetStepsPerMm { axes } => {
                assert!((axes.x.unwrap() - 80.0).abs() < 0.01);
                assert!((axes.z.unwrap() - 400.0).abs() < 0.01);
                assert!((axes.e.unwrap() - 420.0).abs() < 0.01);
            }
            _ => panic!("Expected SetStepsPerMm"),
        }
    }

    #[test]
    fn test_parse_m906() {
        match parse_line("M906 X800 Y800 Z800 E800 I30") {
            GCodeCommand::SetMotorCurrent { axes, idle_percent } => {
                assert!((axes.x.unwrap() - 800.0).abs() < 0.01);
                assert!((idle_percent.unwrap() - 30.0).abs() < 0.01);
            }
            _ => panic!("Expected SetMotorCurrent"),
        }
    }

    #[test]
    fn test_parse_comment() {
        match parse_line("; this is a comment") {
            GCodeCommand::Comment => {}
            _ => panic!("Expected Comment"),
        }
    }

    #[test]
    fn test_parse_inline_comment() {
        match parse_line("G0 X10 ; move to X10") {
            GCodeCommand::LinearMove { axes, .. } => {
                assert!((axes.x.unwrap() - 10.0).abs() < 0.01);
                assert!(axes.y.is_none());
            }
            _ => panic!("Expected LinearMove"),
        }
    }

    #[test]
    fn test_parse_m104() {
        match parse_line("M104 S200 T1") {
            GCodeCommand::SetHotendTemp { temp, tool } => {
                assert!((temp - 200.0).abs() < 0.01);
                assert_eq!(tool, Some(1));
            }
            _ => panic!("Expected SetHotendTemp"),
        }
    }

    #[test]
    fn test_parse_negative_values() {
        match parse_line("G1 X-10.5 Y-20") {
            GCodeCommand::LinearMove { axes, .. } => {
                assert!((axes.x.unwrap() - (-10.5)).abs() < 0.1);
                assert!((axes.y.unwrap() - (-20.0)).abs() < 0.1);
            }
            _ => panic!("Expected LinearMove"),
        }
    }

    #[test]
    fn test_parse_m569() {
        match parse_line("M569 P0 S1 D3") {
            GCodeCommand::SetDriverConfig { driver, direction, mode } => {
                assert_eq!(driver, 0);
                assert_eq!(direction, Some(true));
                assert_eq!(mode, Some(DriverMode::StealthChop));
            }
            _ => panic!("Expected SetDriverConfig"),
        }
    }

    #[test]
    fn test_parse_m112() {
        match parse_line("M112") {
            GCodeCommand::EmergencyStop => {}
            _ => panic!("Expected EmergencyStop"),
        }
    }

    #[test]
    fn test_parse_empty_line() {
        match parse_line("") {
            GCodeCommand::Comment => {}
            _ => panic!("Expected Comment"),
        }
    }
}
