//! Datagram <-> MAVLink message helpers (MAVLink 2 out, 1 or 2 in).

use mavlink::common::MavMessage;
use mavlink::error::MessageReadError;
use mavlink::peek_reader::PeekReader;
use mavlink::{MavHeader, MavlinkVersion};

/// One MAVLink 2 frame for `msg`.
pub fn encode(header: MavHeader, msg: &MavMessage) -> Vec<u8> {
    let mut out = Vec::with_capacity(mavlink::MAX_FRAME_SIZE);
    // Writing into a Vec cannot fail.
    let _ = mavlink::write_versioned_msg(&mut out, MavlinkVersion::V2, header, msg);
    out
}

/// Every message in one UDP datagram. Frames with a bad CRC or an unknown
/// message id are skipped.
pub fn decode(datagram: &[u8]) -> Vec<(MavHeader, MavMessage)> {
    let mut reader = PeekReader::new(datagram);
    let mut out = Vec::new();
    // A datagram holds at most a few frames; the bound guards against a
    // reader that stops consuming.
    for _ in 0..64 {
        match mavlink::read_any_msg::<MavMessage, _>(&mut reader) {
            Ok(m) => out.push(m),
            Err(MessageReadError::Io(_)) => break,
            Err(_) => continue,
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::common::{HEARTBEAT_DATA, MavType};

    #[test]
    fn round_trip() {
        let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            mavtype: MavType::MAV_TYPE_GCS,
            ..Default::default()
        });
        let h = MavHeader {
            system_id: 7,
            component_id: 9,
            sequence: 3,
        };
        let mut buf = encode(h, &hb);
        buf.extend(encode(h, &hb));
        let got = decode(&buf);
        assert_eq!(got.len(), 2);
        assert_eq!(got[0].0, h);
        assert_eq!(got[0].1, hb);
        assert!(decode(&[0xfd, 1, 2, 3]).is_empty());
    }
}
