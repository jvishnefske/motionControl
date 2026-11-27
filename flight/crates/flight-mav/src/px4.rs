//! PX4 custom-mode encoding (`px4_custom_mode.h`): the HEARTBEAT
//! `custom_mode` holds the main mode in bits 16-23 and the sub mode in bits
//! 24-31.

use flight_core::FlightMode;

pub const MAIN_MANUAL: u8 = 1;
pub const MAIN_ALTCTL: u8 = 2;
pub const MAIN_POSCTL: u8 = 3;
pub const MAIN_AUTO: u8 = 4;
pub const MAIN_ACRO: u8 = 5;
pub const MAIN_OFFBOARD: u8 = 6;
pub const MAIN_STABILIZED: u8 = 7;

pub const AUTO_READY: u8 = 1;
pub const AUTO_TAKEOFF: u8 = 2;
pub const AUTO_LOITER: u8 = 3;
pub const AUTO_MISSION: u8 = 4;
pub const AUTO_RTL: u8 = 5;
pub const AUTO_LAND: u8 = 6;
pub const AUTO_PRECLAND: u8 = 9;

/// `(main, sub)` -> HEARTBEAT `custom_mode`.
pub fn encode(main: u8, sub: u8) -> u32 {
    (u32::from(main) << 16) | (u32::from(sub) << 24)
}

/// HEARTBEAT `custom_mode` -> `(main, sub)`.
pub fn decode(custom_mode: u32) -> (u8, u8) {
    (
        ((custom_mode >> 16) & 0xff) as u8,
        ((custom_mode >> 24) & 0xff) as u8,
    )
}

pub fn flight_mode(custom_mode: u32) -> FlightMode {
    match decode(custom_mode) {
        (MAIN_MANUAL, _) | (MAIN_ACRO, _) => FlightMode::Manual,
        (MAIN_ALTCTL, _) => FlightMode::Altitude,
        (MAIN_POSCTL, _) => FlightMode::Position,
        (MAIN_OFFBOARD, _) => FlightMode::Offboard,
        (MAIN_STABILIZED, _) => FlightMode::Stabilized,
        (MAIN_AUTO, AUTO_TAKEOFF) => FlightMode::Takeoff,
        (MAIN_AUTO, AUTO_LOITER) | (MAIN_AUTO, AUTO_READY) => FlightMode::Hold,
        (MAIN_AUTO, AUTO_MISSION) => FlightMode::Mission,
        (MAIN_AUTO, AUTO_RTL) => FlightMode::Rtl,
        (MAIN_AUTO, AUTO_LAND) | (MAIN_AUTO, AUTO_PRECLAND) => FlightMode::Land,
        _ => FlightMode::Unknown,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn px4_encoding() {
        // AUTO.MISSION as PX4 reports it.
        assert_eq!(encode(MAIN_AUTO, AUTO_MISSION), 0x0404_0000);
        assert_eq!(decode(0x0404_0000), (4, 4));
        assert_eq!(flight_mode(encode(MAIN_AUTO, AUTO_RTL)), FlightMode::Rtl);
        assert_eq!(
            flight_mode(encode(MAIN_AUTO, AUTO_LOITER)),
            FlightMode::Hold
        );
        assert_eq!(flight_mode(encode(MAIN_POSCTL, 0)), FlightMode::Position);
        assert_eq!(flight_mode(0), FlightMode::Unknown);
    }
}
