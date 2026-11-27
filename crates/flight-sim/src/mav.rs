//! MAVLink framing for the sim side.
//!
//! The `mavlink` crate is built without `emit-extensions`, so the mission
//! protocol's `mission_type` extension field is not in the generated
//! structs. QGroundControl sends it (fence and rally plans use types 1 and
//! 2), so it is read from, and written to, the raw payload here.

use mavlink::common::MavMessage;
use mavlink::error::MessageReadError;
use mavlink::peek_reader::PeekReader;
use mavlink::{MavHeader, MavlinkVersion, Message};

/// PX4 custom-mode encoding: main mode in bits 16-23, sub mode in bits 24-31.
pub mod px4 {
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

    pub fn encode(main: u8, sub: u8) -> u32 {
        (u32::from(main) << 16) | (u32::from(sub) << 24)
    }

    pub fn name(custom_mode: u32) -> String {
        let main = ((custom_mode >> 16) & 0xff) as u8;
        let sub = ((custom_mode >> 24) & 0xff) as u8;
        match (main, sub) {
            (MAIN_MANUAL, _) => "MANUAL".into(),
            (MAIN_ALTCTL, _) => "ALTCTL".into(),
            (MAIN_POSCTL, _) => "POSCTL".into(),
            (MAIN_ACRO, _) => "ACRO".into(),
            (MAIN_OFFBOARD, _) => "OFFBOARD".into(),
            (MAIN_STABILIZED, _) => "STABILIZED".into(),
            (MAIN_AUTO, AUTO_READY) => "AUTO.READY".into(),
            (MAIN_AUTO, AUTO_TAKEOFF) => "AUTO.TAKEOFF".into(),
            (MAIN_AUTO, AUTO_LOITER) => "AUTO.LOITER".into(),
            (MAIN_AUTO, AUTO_MISSION) => "AUTO.MISSION".into(),
            (MAIN_AUTO, AUTO_RTL) => "AUTO.RTL".into(),
            (MAIN_AUTO, AUTO_LAND) => "AUTO.LAND".into(),
            _ => format!("custom {main}/{sub}"),
        }
    }
}

/// A received message plus the `mission_type` extension (0 when absent).
pub struct Frame {
    pub header: MavHeader,
    pub msg: MavMessage,
    pub mission_type: u8,
}

/// Payload offset of `mission_type` in the mission-protocol messages.
fn mission_type_offset(msg_id: u32) -> Option<usize> {
    match msg_id {
        40 | 44 | 51 => Some(4), // MISSION_REQUEST, MISSION_COUNT, MISSION_REQUEST_INT
        43 | 45 => Some(2),      // MISSION_REQUEST_LIST, MISSION_CLEAR_ALL
        47 => Some(3),           // MISSION_ACK
        39 | 73 => Some(37),     // MISSION_ITEM, MISSION_ITEM_INT
        _ => None,
    }
}

pub fn decode(datagram: &[u8]) -> Vec<Frame> {
    let mut reader = PeekReader::new(datagram);
    let mut out = Vec::new();
    for _ in 0..64 {
        let raw = match mavlink::read_any_raw_message::<MavMessage, _>(&mut reader) {
            Ok(r) => r,
            Err(MessageReadError::Io(_)) => break,
            Err(_) => continue,
        };
        let Ok(msg) = MavMessage::parse(raw.version(), raw.message_id(), raw.payload()) else {
            continue;
        };
        let mission_type = mission_type_offset(raw.message_id())
            .and_then(|o| raw.payload().get(o).copied())
            .unwrap_or(0);
        let header = MavHeader {
            system_id: raw.system_id(),
            component_id: raw.component_id(),
            sequence: raw.sequence(),
        };
        out.push(Frame {
            header,
            msg,
            mission_type,
        });
    }
    out
}

/// A MAVLink 2 frame for `msg`, with `mission_type` written into the
/// extension byte when the message has one and it is non-zero.
pub fn encode(header: MavHeader, msg: &MavMessage, mission_type: u8) -> Vec<u8> {
    let id = msg.message_id();
    let ext_at = mission_type_offset(id).filter(|_| mission_type != 0);
    let Some(ext_at) = ext_at else {
        let mut out = Vec::with_capacity(mavlink::MAX_FRAME_SIZE);
        let _ = mavlink::write_versioned_msg(&mut out, MavlinkVersion::V2, header, msg);
        return out;
    };
    let mut payload = [0u8; 255];
    msg.ser(MavlinkVersion::V2, &mut payload);
    payload[ext_at] = mission_type;
    // MAVLink 2 truncates trailing zero bytes (keeping at least one).
    let mut len = ext_at + 1;
    while len > 1 && payload[len - 1] == 0 {
        len -= 1;
    }
    let mut out = Vec::with_capacity(12 + len);
    out.extend_from_slice(&[
        0xfd,
        len as u8,
        0,
        0,
        header.sequence,
        header.system_id,
        header.component_id,
    ]);
    out.extend_from_slice(&id.to_le_bytes()[..3]);
    out.extend_from_slice(&payload[..len]);
    let crc = mavlink::calculate_crc(&out[1..], MavMessage::extra_crc(id));
    out.extend_from_slice(&crc.to_le_bytes());
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::common::{MISSION_ACK_DATA, MISSION_COUNT_DATA, MavMissionResult};

    #[test]
    fn mission_type_round_trips() {
        let h = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 9,
        };
        let count = MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
            count: 0,
            target_system: 255,
            target_component: 190,
            ..Default::default()
        });
        let f = decode(&encode(h, &count, 1));
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].mission_type, 1);
        assert_eq!(f[0].msg, count);
        let ack = MavMessage::MISSION_ACK(MISSION_ACK_DATA {
            target_system: 255,
            target_component: 190,
            mavtype: MavMissionResult::MAV_MISSION_ACCEPTED,
            ..Default::default()
        });
        let f = decode(&encode(h, &ack, 2));
        assert_eq!((f[0].mission_type, &f[0].msg), (2, &ack));
        let f = decode(&encode(h, &ack, 0));
        assert_eq!((f[0].mission_type, &f[0].msg), (0, &ack));
    }
}
