//! [`SimAutopilot`]: the Rust stand-in for PX4 SITL. It speaks MAVLink on
//! the same UDP ports as PX4 instance 0, so QGroundControl and the mission
//! runner connect to it exactly as they connect to PX4:
//!
//! * GCS link: bound to udp `gcs + 4020` (18570), streams to `127.0.0.1:gcs` (14550);
//! * API link: bound to udp `api + 40` (14580), streams to `127.0.0.1:api` (14540).
//!
//! Replies to requests go to the requester's address.

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::Context;
use async_trait::async_trait;
use flight_core::{
    Airframe, Backend, LogLevel, MavPorts, Module, ModuleKind, ModuleStatus, SimContext,
};
use mavlink::MavHeader;
use mavlink::common::*;
use mavlink::types::CharArray;
use tokio::net::UdpSocket;
use tokio::task::JoinHandle;

use crate::mav::{self, Frame, px4};
use crate::vehicle::{DT, Event, NavItem, Vehicle};

pub const SYSTEM_ID: u8 = 1;
pub const COMPONENT_ID: u8 = 1;

const GCS_LINK: usize = 0;
const API_LINK: usize = 1;
const LINK_NAMES: [&str; 2] = ["gcs", "api"];

/// PX4 instance 0 binds its GCS link to 18570 and its API link to 14580;
/// for other port layouts the same offsets are used.
pub fn local_ports(ports: MavPorts) -> (u16, u16) {
    (
        ports.gcs.checked_add(4020).unwrap_or(0),
        ports.api.checked_add(40).unwrap_or(0),
    )
}

/// Default streams per link: (message id, rate in sim Hz).
const DEFAULT_STREAMS: [(u32, f64); 10] = [
    (33, 10.0), // GLOBAL_POSITION_INT
    (30, 10.0), // ATTITUDE
    (74, 4.0),  // VFR_HUD
    (24, 2.0),  // GPS_RAW_INT
    (245, 2.0), // EXTENDED_SYS_STATE
    (1, 1.0),   // SYS_STATUS
    (147, 1.0), // BATTERY_STATUS
    (42, 1.0),  // MISSION_CURRENT
    (242, 0.5), // HOME_POSITION
    (32, 0.0),  // LOCAL_POSITION_NED (off unless requested)
];

#[derive(Clone, Copy)]
struct Stream {
    id: u32,
    default_interval: f64,
    interval: f64,
    next_due: f64,
}

struct Param {
    name: &'static str,
    value: f32,
    int: bool,
}

struct Upload {
    link: usize,
    peer: SocketAddr,
    sys: u8,
    comp: u8,
    count: u16,
    items: Vec<MISSION_ITEM_INT_DATA>,
    last_activity: Instant,
    retries: u32,
}

/// Everything behind the server mutex.
struct Server {
    v: Vehicle,
    seq: [u8; 2],
    dest: [SocketAddr; 2],
    streams: [Vec<Stream>; 2],
    items: Vec<MISSION_ITEM_INT_DATA>,
    upload: Option<Upload>,
    params: Vec<Param>,
    gcs_heartbeats: [u64; 2],
    last_gcs_heartbeat: [Option<Instant>; 2],
    out: Vec<(usize, SocketAddr, Vec<u8>)>,
}

fn param_id(name: &str) -> CharArray<16> {
    CharArray::from(name)
}

fn param_name(id: &CharArray<16>) -> String {
    let bytes: Vec<u8> = id.iter().copied().take_while(|&b| b != 0).collect();
    String::from_utf8_lossy(&bytes).into_owned()
}

fn deg_e7(v: f64) -> i32 {
    (v * 1e7).round() as i32
}

impl Server {
    fn new(airframe: Airframe, home: flight_core::GeoPoint, ports: MavPorts) -> Server {
        let v = Vehicle::new(airframe, home);
        let streams: Vec<Stream> = DEFAULT_STREAMS
            .iter()
            .map(|&(id, hz)| {
                let iv = if hz > 0.0 { 1.0 / hz } else { 0.0 };
                Stream {
                    id,
                    default_interval: iv,
                    interval: iv,
                    next_due: 0.0,
                }
            })
            .collect();
        let fw = airframe.is_fixed_wing();
        let params = vec![
            Param {
                name: "SYS_AUTOSTART",
                value: airframe.px4_autostart_sih() as f32,
                int: true,
            },
            Param {
                name: "MAV_SYS_ID",
                value: f32::from(SYSTEM_ID),
                int: true,
            },
            Param {
                name: "MAV_TYPE",
                value: if fw { 1.0 } else { 2.0 },
                int: true,
            },
            Param {
                name: "COM_RC_IN_MODE",
                value: 1.0,
                int: true,
            },
            Param {
                name: "BAT1_N_CELLS",
                value: 4.0,
                int: true,
            },
            Param {
                name: "NAV_ACC_RAD",
                value: airframe.acceptance_radius_m() as f32,
                int: false,
            },
            Param {
                name: "MIS_TAKEOFF_ALT",
                value: v.takeoff_alt as f32,
                int: false,
            },
            Param {
                name: "RTL_RETURN_ALT",
                value: v.rtl_alt as f32,
                int: false,
            },
            Param {
                name: if fw {
                    "FW_AIRSPD_TRIM"
                } else {
                    "MPC_XY_CRUISE"
                },
                value: airframe.cruise_speed_ms() as f32,
                int: false,
            },
            Param {
                name: "NAV_LOITER_RAD",
                value: 80.0,
                int: false,
            },
        ];
        Server {
            v,
            seq: [0; 2],
            dest: [
                SocketAddr::from(([127, 0, 0, 1], ports.gcs)),
                SocketAddr::from(([127, 0, 0, 1], ports.api)),
            ],
            streams: [streams.clone(), streams],
            items: Vec::new(),
            upload: None,
            params,
            gcs_heartbeats: [0; 2],
            last_gcs_heartbeat: [None; 2],
            out: Vec::new(),
        }
    }

    fn send_ext(&mut self, link: usize, to: SocketAddr, msg: &MavMessage, mission_type: u8) {
        let header = MavHeader {
            system_id: SYSTEM_ID,
            component_id: COMPONENT_ID,
            sequence: self.seq[link],
        };
        self.seq[link] = self.seq[link].wrapping_add(1);
        self.out
            .push((link, to, mav::encode(header, msg, mission_type)));
    }

    fn send(&mut self, link: usize, to: SocketAddr, msg: &MavMessage) {
        self.send_ext(link, to, msg, 0);
    }

    /// To both links' stream destinations.
    fn broadcast(&mut self, msg: &MavMessage) {
        for link in [GCS_LINK, API_LINK] {
            let to = self.dest[link];
            self.send(link, to, msg);
        }
    }

    // ---- telemetry messages ----------------------------------------------

    fn heartbeat(&self) -> MavMessage {
        let v = &self.v;
        let mut base = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        if v.armed {
            base |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        }
        base |= if v.mode.is_auto() {
            MavModeFlag::MAV_MODE_FLAG_AUTO_ENABLED
                | MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED
                | MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
        } else {
            MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                | MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
        };
        MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: v.custom_mode(),
            mavtype: if v.airframe.is_fixed_wing() {
                MavType::MAV_TYPE_FIXED_WING
            } else {
                MavType::MAV_TYPE_QUADROTOR
            },
            autopilot: MavAutopilot::MAV_AUTOPILOT_PX4,
            base_mode: base,
            system_status: if v.armed {
                MavState::MAV_STATE_ACTIVE
            } else {
                MavState::MAV_STATE_STANDBY
            },
            mavlink_version: 3,
        })
    }

    fn time_boot_ms(&self) -> u32 {
        (self.v.time_s * 1000.0) as u32
    }

    fn home_position(&self) -> MavMessage {
        let h = self.v.home;
        MavMessage::HOME_POSITION(HOME_POSITION_DATA {
            latitude: deg_e7(h.lat),
            longitude: deg_e7(h.lon),
            altitude: (h.alt_m * 1000.0) as i32,
            q: [1.0, 0.0, 0.0, 0.0],
            ..Default::default()
        })
    }

    fn autopilot_version(&self) -> MavMessage {
        use MavProtocolCapability as C;
        let caps = C::MAV_PROTOCOL_CAPABILITY_MISSION_INT
            | C::MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT
            | C::MAV_PROTOCOL_CAPABILITY_COMMAND_INT
            | C::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE
            | C::MAV_PROTOCOL_CAPABILITY_MAVLINK2
            | C::MAV_PROTOCOL_CAPABILITY_MISSION_FENCE
            | C::MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
        // PX4 v1.15.0, FIRMWARE_VERSION_TYPE_OFFICIAL.
        let version = (1 << 24) | (15 << 16) | 255;
        MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
            capabilities: caps,
            uid: 0x5157_0001,
            flight_sw_version: version,
            middleware_sw_version: version,
            ..Default::default()
        })
    }

    /// Build message `id` from the current state, if this sim knows it.
    fn message(&self, id: u32) -> Option<MavMessage> {
        let v = &self.v;
        let p = v.position();
        let t = self.time_boot_ms();
        let hdg = |yaw: f64| (yaw.to_degrees().rem_euclid(360.0) * 100.0) as u16 % 36000;
        let ground_speed = v.vn.hypot(v.ve);
        Some(match id {
            0 => self.heartbeat(),
            1 => {
                let all = MavSysStatusSensor::all();
                MavMessage::SYS_STATUS(SYS_STATUS_DATA {
                    onboard_control_sensors_present: all,
                    onboard_control_sensors_enabled: all,
                    onboard_control_sensors_health: all,
                    load: 250,
                    voltage_battery: (4.0 * (3.5 + 0.7 * v.battery_pct / 100.0) * 1000.0) as u16,
                    current_battery: if v.armed { 1500 } else { 50 },
                    battery_remaining: v.battery_pct.round() as i8,
                    ..Default::default()
                })
            }
            24 => MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
                time_usec: (v.time_s * 1e6) as u64,
                lat: deg_e7(p.lat),
                lon: deg_e7(p.lon),
                alt: (p.alt_m * 1000.0) as i32,
                eph: 80,
                epv: 120,
                vel: (ground_speed * 100.0) as u16,
                cog: hdg(v.ve.atan2(v.vn)),
                fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
                satellites_visible: 14,
            }),
            30 => MavMessage::ATTITUDE(ATTITUDE_DATA {
                time_boot_ms: t,
                roll: v.roll as f32,
                pitch: v.pitch as f32,
                yaw: v.yaw as f32,
                ..Default::default()
            }),
            32 => MavMessage::LOCAL_POSITION_NED(LOCAL_POSITION_NED_DATA {
                time_boot_ms: t,
                x: v.n as f32,
                y: v.e as f32,
                z: -v.h as f32,
                vx: v.vn as f32,
                vy: v.ve as f32,
                vz: -v.vu as f32,
            }),
            33 => MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
                time_boot_ms: t,
                lat: deg_e7(p.lat),
                lon: deg_e7(p.lon),
                alt: (p.alt_m * 1000.0) as i32,
                relative_alt: (v.h * 1000.0) as i32,
                vx: (v.vn * 100.0) as i16,
                vy: (v.ve * 100.0) as i16,
                vz: (-v.vu * 100.0) as i16,
                hdg: hdg(v.yaw),
            }),
            42 => MavMessage::MISSION_CURRENT(MISSION_CURRENT_DATA {
                seq: v.current as u16,
                ..Default::default()
            }),
            74 => MavMessage::VFR_HUD(VFR_HUD_DATA {
                airspeed: v.airspeed as f32,
                groundspeed: ground_speed as f32,
                alt: p.alt_m as f32,
                climb: v.vu as f32,
                heading: (v.yaw.to_degrees().rem_euclid(360.0)) as i16,
                throttle: if v.armed { 55 } else { 0 },
            }),
            147 => {
                let cell = (3500.0 + 700.0 * v.battery_pct / 100.0) as u16;
                let mut voltages = [u16::MAX; 10];
                voltages[..4].fill(cell);
                MavMessage::BATTERY_STATUS(BATTERY_STATUS_DATA {
                    current_consumed: -1,
                    energy_consumed: -1,
                    temperature: i16::MAX,
                    voltages,
                    current_battery: if v.armed { 1500 } else { 50 },
                    id: 0,
                    battery_function: MavBatteryFunction::MAV_BATTERY_FUNCTION_ALL,
                    mavtype: MavBatteryType::MAV_BATTERY_TYPE_LIPO,
                    battery_remaining: v.battery_pct.round() as i8,
                    ..Default::default()
                })
            }
            148 => self.autopilot_version(),
            242 => self.home_position(),
            245 => MavMessage::EXTENDED_SYS_STATE(EXTENDED_SYS_STATE_DATA {
                vtol_state: MavVtolState::MAV_VTOL_STATE_UNDEFINED,
                landed_state: if v.landed {
                    MavLandedState::MAV_LANDED_STATE_ON_GROUND
                } else {
                    MavLandedState::MAV_LANDED_STATE_IN_AIR
                },
            }),
            _ => return None,
        })
    }

    // ---- periodic --------------------------------------------------------

    fn emit_streams(&mut self) {
        let now = self.v.time_s;
        for link in [GCS_LINK, API_LINK] {
            for i in 0..self.streams[link].len() {
                let s = self.streams[link][i];
                if s.interval <= 0.0 || now < s.next_due {
                    continue;
                }
                let next = if now - s.next_due > s.interval {
                    now + s.interval
                } else {
                    s.next_due + s.interval
                };
                self.streams[link][i].next_due = next;
                if let Some(m) = self.message(s.id) {
                    let to = self.dest[link];
                    self.send(link, to, &m);
                }
            }
        }
    }

    fn emit_events(&mut self) {
        for ev in std::mem::take(&mut self.v.events) {
            match ev {
                Event::Reached(seq) => self.broadcast(&MavMessage::MISSION_ITEM_REACHED(
                    MISSION_ITEM_REACHED_DATA { seq },
                )),
                Event::Current(seq) => {
                    self.broadcast(&MavMessage::MISSION_CURRENT(MISSION_CURRENT_DATA {
                        seq,
                        ..Default::default()
                    }))
                }
                Event::Text(t) => self.broadcast(&MavMessage::STATUSTEXT(STATUSTEXT_DATA {
                    severity: MavSeverity::MAV_SEVERITY_INFO,
                    text: CharArray::from(t.as_str()),
                    ..Default::default()
                })),
            }
        }
    }

    fn step(&mut self, dt: f64) {
        self.v.step(dt);
        self.emit_events();
        self.emit_streams();
    }

    fn check_upload(&mut self) {
        let Some(up) = self.upload.as_mut() else {
            return;
        };
        if up.last_activity.elapsed() < Duration::from_millis(500) {
            return;
        }
        up.retries += 1;
        up.last_activity = Instant::now();
        let (link, peer, sys, comp, next) =
            (up.link, up.peer, up.sys, up.comp, up.items.len() as u16);
        if up.retries > 6 {
            self.upload = None;
            self.mission_ack(
                link,
                peer,
                sys,
                comp,
                MavMissionResult::MAV_MISSION_OPERATION_CANCELLED,
                0,
            );
            return;
        }
        self.request_item(link, peer, sys, comp, next);
    }

    fn status(&self) -> String {
        let v = &self.v;
        let gcs = self
            .last_gcs_heartbeat
            .iter()
            .flatten()
            .any(|t| t.elapsed() < Duration::from_secs(3));
        format!(
            "{} | {} | {} | alt {:.1} m | {:.1} m/s | wp {}/{} | battery {:.0}% | GCS {}",
            v.airframe.label(),
            px4::name(v.custom_mode()),
            if v.armed { "armed" } else { "disarmed" },
            v.h,
            v.vn.hypot(v.ve),
            v.current.min(v.mission.len()),
            v.mission.len(),
            v.battery_pct,
            if gcs { "connected" } else { "not seen" },
        )
    }

    // ---- requests ----------------------------------------------------------

    fn ack(&mut self, link: usize, to: SocketAddr, command: MavCmd, result: MavResult) {
        self.send(
            link,
            to,
            &MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
                command,
                result,
                ..Default::default()
            }),
        );
    }

    fn mission_ack(
        &mut self,
        link: usize,
        to: SocketAddr,
        sys: u8,
        comp: u8,
        r: MavMissionResult,
        mission_type: u8,
    ) {
        let ack = MavMessage::MISSION_ACK(MISSION_ACK_DATA {
            target_system: sys,
            target_component: comp,
            mavtype: r,
            ..Default::default()
        });
        self.send_ext(link, to, &ack, mission_type);
    }

    fn request_item(&mut self, link: usize, to: SocketAddr, sys: u8, comp: u8, seq: u16) {
        let req = MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
            seq,
            target_system: sys,
            target_component: comp,
            ..Default::default()
        });
        self.send(link, to, &req);
    }

    #[allow(deprecated)]
    fn nav_item(&self, it: &MISSION_ITEM_INT_DATA) -> NavItem {
        let pos = (it.x != 0 || it.y != 0)
            .then(|| self.v.local(f64::from(it.x) / 1e7, f64::from(it.y) / 1e7));
        let z = f64::from(it.z);
        let alt = match it.frame {
            MavFrame::MAV_FRAME_GLOBAL | MavFrame::MAV_FRAME_GLOBAL_INT => z - self.v.home.alt_m,
            _ => z,
        };
        NavItem {
            cmd: it.command,
            pos,
            alt: if it.command == MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH {
                f64::NAN
            } else {
                alt
            },
            acceptance: if it.command == MavCmd::MAV_CMD_NAV_WAYPOINT {
                f64::from(it.param2)
            } else {
                0.0
            },
            param1: f64::from(it.param1),
        }
    }

    fn set_mission(&mut self, items: Vec<MISSION_ITEM_INT_DATA>) {
        let nav = items.iter().map(|it| self.nav_item(it)).collect();
        self.items = items;
        self.v.set_mission(nav);
    }

    fn handle(&mut self, link: usize, from: SocketAddr, f: Frame) {
        let (sys, comp) = (f.header.system_id, f.header.component_id);
        if sys == SYSTEM_ID && comp == COMPONENT_ID {
            return;
        }
        let for_us =
            |ts: u8, tc: u8| (ts == 0 || ts == SYSTEM_ID) && (tc == 0 || tc == COMPONENT_ID);
        #[allow(deprecated)]
        match f.msg {
            MavMessage::HEARTBEAT(hb) => {
                if hb.mavtype == MavType::MAV_TYPE_GCS
                    || hb.autopilot == MavAutopilot::MAV_AUTOPILOT_INVALID
                {
                    self.gcs_heartbeats[link] += 1;
                    self.last_gcs_heartbeat[link] = Some(Instant::now());
                }
            }
            MavMessage::COMMAND_LONG(c) if for_us(c.target_system, c.target_component) => {
                let p = [
                    c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7,
                ];
                self.command(link, from, c.command, p);
            }
            MavMessage::COMMAND_INT(c) if for_us(c.target_system, c.target_component) => {
                let p = [
                    c.param1,
                    c.param2,
                    c.param3,
                    c.param4,
                    c.x as f32 / 1e7,
                    c.y as f32 / 1e7,
                    c.z,
                ];
                self.command(link, from, c.command, p);
            }
            MavMessage::SET_MODE(m) if m.target_system == SYSTEM_ID => {
                let (main, sub) = (
                    ((m.custom_mode >> 16) & 0xff) as u8,
                    ((m.custom_mode >> 24) & 0xff) as u8,
                );
                let _ = self.v.set_mode(main, sub);
            }
            // ---- mission protocol (server side) ----
            MavMessage::MISSION_COUNT(c) if for_us(c.target_system, c.target_component) => {
                if f.mission_type != 0 {
                    let r = if c.count == 0 {
                        MavMissionResult::MAV_MISSION_ACCEPTED
                    } else {
                        MavMissionResult::MAV_MISSION_UNSUPPORTED
                    };
                    self.mission_ack(link, from, sys, comp, r, f.mission_type);
                } else if c.count == 0 {
                    self.upload = None;
                    self.set_mission(Vec::new());
                    self.mission_ack(
                        link,
                        from,
                        sys,
                        comp,
                        MavMissionResult::MAV_MISSION_ACCEPTED,
                        0,
                    );
                } else {
                    self.upload = Some(Upload {
                        link,
                        peer: from,
                        sys,
                        comp,
                        count: c.count,
                        items: Vec::with_capacity(usize::from(c.count)),
                        last_activity: Instant::now(),
                        retries: 0,
                    });
                    self.request_item(link, from, sys, comp, 0);
                }
            }
            MavMessage::MISSION_ITEM_INT(it) if for_us(it.target_system, it.target_component) => {
                self.receive_item(link, from, it, f.mission_type);
            }
            MavMessage::MISSION_ITEM(it) if for_us(it.target_system, it.target_component) => {
                let int = MISSION_ITEM_INT_DATA {
                    param1: it.param1,
                    param2: it.param2,
                    param3: it.param3,
                    param4: it.param4,
                    x: (f64::from(it.x) * 1e7) as i32,
                    y: (f64::from(it.y) * 1e7) as i32,
                    z: it.z,
                    seq: it.seq,
                    command: it.command,
                    target_system: it.target_system,
                    target_component: it.target_component,
                    frame: it.frame,
                    current: it.current,
                    autocontinue: it.autocontinue,
                    ..Default::default()
                };
                self.receive_item(link, from, int, f.mission_type);
            }
            MavMessage::MISSION_REQUEST_LIST(r) if for_us(r.target_system, r.target_component) => {
                let count = if f.mission_type == 0 {
                    self.items.len() as u16
                } else {
                    0
                };
                let msg = MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
                    count,
                    target_system: sys,
                    target_component: comp,
                    ..Default::default()
                });
                self.send_ext(link, from, &msg, f.mission_type);
            }
            MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
                seq,
                target_system,
                target_component,
                ..
            })
            | MavMessage::MISSION_REQUEST(MISSION_REQUEST_DATA {
                seq,
                target_system,
                target_component,
                ..
            }) if for_us(target_system, target_component) => {
                match self.items.get(usize::from(seq)) {
                    Some(it) if f.mission_type == 0 => {
                        let mut it = it.clone();
                        it.target_system = sys;
                        it.target_component = comp;
                        it.current = u8::from(usize::from(seq) == self.v.current);
                        self.send(link, from, &MavMessage::MISSION_ITEM_INT(it));
                    }
                    _ => self.mission_ack(
                        link,
                        from,
                        sys,
                        comp,
                        MavMissionResult::MAV_MISSION_INVALID_SEQUENCE,
                        f.mission_type,
                    ),
                }
            }
            MavMessage::MISSION_CLEAR_ALL(c) if for_us(c.target_system, c.target_component) => {
                if f.mission_type == 0 || f.mission_type == 255 {
                    self.upload = None;
                    self.set_mission(Vec::new());
                }
                self.mission_ack(
                    link,
                    from,
                    sys,
                    comp,
                    MavMissionResult::MAV_MISSION_ACCEPTED,
                    f.mission_type,
                );
            }
            MavMessage::MISSION_SET_CURRENT(c) if for_us(c.target_system, c.target_component) => {
                if self.v.set_current(usize::from(c.seq)).is_ok() {
                    self.emit_events();
                }
            }
            // ---- parameters ----
            MavMessage::PARAM_REQUEST_LIST(r) if for_us(r.target_system, r.target_component) => {
                for i in 0..self.params.len() {
                    self.send_param(link, from, i);
                }
            }
            MavMessage::PARAM_REQUEST_READ(r) if for_us(r.target_system, r.target_component) => {
                let idx = if r.param_index >= 0 {
                    Some(r.param_index as usize).filter(|&i| i < self.params.len())
                } else {
                    let name = param_name(&r.param_id);
                    self.params.iter().position(|p| p.name == name)
                };
                if let Some(i) = idx {
                    self.send_param(link, from, i);
                }
            }
            MavMessage::PARAM_SET(s) if for_us(s.target_system, s.target_component) => {
                let name = param_name(&s.param_id);
                if let Some(i) = self.params.iter().position(|p| p.name == name) {
                    let p = &mut self.params[i];
                    p.value = if p.int {
                        s.param_value.to_bits() as i32 as f32
                    } else {
                        s.param_value
                    };
                    match p.name {
                        "RTL_RETURN_ALT" => self.v.rtl_alt = f64::from(p.value),
                        "MIS_TAKEOFF_ALT" => self.v.takeoff_alt = f64::from(p.value),
                        _ => {}
                    }
                    let to = self.dest[link];
                    self.send_param(link, to, i);
                    if from != to {
                        self.send_param(link, from, i);
                    }
                }
            }
            _ => {}
        }
    }

    fn send_param(&mut self, link: usize, to: SocketAddr, i: usize) {
        let p = &self.params[i];
        // PX4 encodes integer params bytewise in the float field.
        let (value, ty) = if p.int {
            (
                f32::from_bits(p.value as i32 as u32),
                MavParamType::MAV_PARAM_TYPE_INT32,
            )
        } else {
            (p.value, MavParamType::MAV_PARAM_TYPE_REAL32)
        };
        let msg = MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_value: value,
            param_count: self.params.len() as u16,
            param_index: i as u16,
            param_id: param_id(p.name),
            param_type: ty,
        });
        self.send(link, to, &msg);
    }

    fn receive_item(
        &mut self,
        link: usize,
        from: SocketAddr,
        it: MISSION_ITEM_INT_DATA,
        mission_type: u8,
    ) {
        if mission_type != 0 {
            return;
        }
        let Some(up) = self.upload.as_mut() else {
            return;
        };
        if up.peer != from {
            return;
        }
        let (sys, comp) = (up.sys, up.comp);
        let next = up.items.len() as u16;
        if it.seq != next {
            // Out of order: ask again for the one we need.
            self.request_item(link, from, sys, comp, next);
            return;
        }
        up.items.push(it);
        up.last_activity = Instant::now();
        up.retries = 0;
        if up.items.len() == usize::from(up.count) {
            let items = std::mem::take(&mut up.items);
            self.upload = None;
            self.set_mission(items);
            self.mission_ack(
                link,
                from,
                sys,
                comp,
                MavMissionResult::MAV_MISSION_ACCEPTED,
                0,
            );
            self.emit_events();
        } else {
            self.request_item(link, from, sys, comp, next + 1);
        }
    }

    // QGC still sends the deprecated REQUEST_AUTOPILOT_CAPABILITIES / GET_HOME_POSITION.
    #[allow(deprecated)]
    fn command(&mut self, link: usize, from: SocketAddr, cmd: MavCmd, p: [f32; 7]) {
        use MavResult::*;
        let ok = |r: Result<(), &'static str>| {
            if r.is_ok() {
                MAV_RESULT_ACCEPTED
            } else {
                MAV_RESULT_DENIED
            }
        };
        let mut then_send: Option<MavMessage> = None;
        let result = match cmd {
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => {
                ok(self.v.arm(p[0] > 0.5, p[1] as i32 == 21196))
            }
            MavCmd::MAV_CMD_DO_SET_MODE => {
                if (p[0] as u32) & MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.bits() as u32 == 0
                {
                    MAV_RESULT_UNSUPPORTED
                } else {
                    ok(self.v.set_mode(p[1] as u8, p[2] as u8))
                }
            }
            MavCmd::MAV_CMD_MISSION_START => {
                let first = (p[0].is_finite() && p[0] >= 0.0).then_some(p[0] as usize);
                ok(self.v.start_mission(first))
            }
            MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH => ok(self.v.rtl()),
            MavCmd::MAV_CMD_NAV_LAND => ok(self.v.land()),
            MavCmd::MAV_CMD_NAV_TAKEOFF => ok(self.v.takeoff(f64::from(p[6]))),
            MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL => {
                let id = p[0] as u32;
                let now = self.v.time_s;
                match self.streams[link].iter_mut().find(|s| s.id == id) {
                    Some(s) => {
                        s.interval = match p[1] {
                            x if x < 0.0 => 0.0,
                            0.0 => s.default_interval,
                            us => f64::from(us) / 1e6,
                        };
                        s.next_due = now;
                        MAV_RESULT_ACCEPTED
                    }
                    None => MAV_RESULT_UNSUPPORTED,
                }
            }
            MavCmd::MAV_CMD_REQUEST_MESSAGE => match self.message(p[0] as u32) {
                Some(m) => {
                    then_send = Some(m);
                    MAV_RESULT_ACCEPTED
                }
                None => MAV_RESULT_UNSUPPORTED,
            },
            MavCmd::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES => {
                then_send = Some(self.autopilot_version());
                MAV_RESULT_ACCEPTED
            }
            MavCmd::MAV_CMD_GET_HOME_POSITION => {
                then_send = Some(self.home_position());
                MAV_RESULT_ACCEPTED
            }
            _ => MAV_RESULT_UNSUPPORTED,
        };
        self.ack(link, from, cmd, result);
        if let Some(m) = then_send {
            self.send(link, from, &m);
        }
        self.emit_events();
    }
}

/// The Rust autopilot + vehicle dynamics as a [`Module`].
pub struct SimAutopilot {
    time_scale: f64,
    status: Arc<Mutex<ModuleStatus>>,
    tasks: Vec<JoinHandle<()>>,
}

impl Default for SimAutopilot {
    fn default() -> Self {
        Self::new()
    }
}

impl SimAutopilot {
    pub fn new() -> SimAutopilot {
        SimAutopilot {
            time_scale: 1.0,
            status: Arc::new(Mutex::new(ModuleStatus::Stopped)),
            tasks: Vec::new(),
        }
    }

    /// Run the sim `scale` times faster than real time (default 1).
    pub fn with_time_scale(mut self, scale: f64) -> SimAutopilot {
        self.time_scale = if scale.is_finite() && scale > 0.0 {
            scale
        } else {
            1.0
        };
        self
    }

    pub fn time_scale(&self) -> f64 {
        self.time_scale
    }

    async fn halt(&mut self) {
        for t in &self.tasks {
            t.abort();
        }
        // Await so the sockets are dropped (ports freed) before we return.
        for t in self.tasks.drain(..) {
            let _ = t.await;
        }
    }
}

impl Drop for SimAutopilot {
    fn drop(&mut self) {
        for t in &self.tasks {
            t.abort();
        }
    }
}

fn send_out(socks: &[Arc<UdpSocket>; 2], out: Vec<(usize, SocketAddr, Vec<u8>)>) {
    for (link, to, bytes) in out {
        let _ = socks[link].try_send_to(&bytes, to);
    }
}

async fn rx_loop(link: usize, socks: [Arc<UdpSocket>; 2], server: Arc<Mutex<Server>>) {
    let mut buf = vec![0u8; 65536];
    loop {
        let (n, from) = match socks[link].recv_from(&mut buf).await {
            Ok(v) => v,
            Err(_) => {
                // e.g. ICMP port unreachable surfacing on Linux; keep going.
                tokio::time::sleep(Duration::from_millis(5)).await;
                continue;
            }
        };
        let out = {
            let mut s = server.lock().unwrap();
            for f in mav::decode(&buf[..n]) {
                s.handle(link, from, f);
            }
            std::mem::take(&mut s.out)
        };
        send_out(&socks, out);
    }
}

async fn sim_loop(
    socks: [Arc<UdpSocket>; 2],
    server: Arc<Mutex<Server>>,
    scale: f64,
    status: Arc<Mutex<ModuleStatus>>,
) {
    // Wall tick: 50 Hz at real time, faster when scaled so every physics
    // step (and telemetry sample) still goes out separately.
    let tick = Duration::from_secs_f64((DT / scale).clamp(0.002, DT));
    let mut interval = tokio::time::interval(tick);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut last = Instant::now();
    let mut acc = 0.0;
    let mut last_hb: Option<Instant> = None;
    let mut last_hb_sim = f64::NEG_INFINITY;
    let mut last_state = (false, 0u32);
    let mut last_status = Instant::now();
    loop {
        interval.tick().await;
        let now = Instant::now();
        // No catching up after a stall: a late tick runs at most a few
        // steps, so sim time slows down instead of jumping. Telemetry then
        // stays dense in sim time (each burst of steps would otherwise be
        // coalesced by the receiver) and evaluation in sim time stays fair.
        acc = (acc + (now - last).as_secs_f64() * scale).min(4.0 * DT);
        last = now;
        let out = {
            let mut s = server.lock().unwrap();
            while acc >= DT {
                s.step(DT);
                acc -= DT;
            }
            // Heartbeat at 1 Hz in both wall and sim time, and right away
            // when the armed state or mode changes, so a fast sim never
            // reports a stale armed flag.
            let state = (s.v.armed, s.v.custom_mode());
            if state != last_state
                || s.v.time_s - last_hb_sim >= 1.0
                || last_hb.is_none_or(|t| t.elapsed() >= Duration::from_secs(1))
            {
                last_hb = Some(now);
                last_hb_sim = s.v.time_s;
                last_state = state;
                let hb = s.heartbeat();
                s.broadcast(&hb);
            }
            s.check_upload();
            if last_status.elapsed() >= Duration::from_millis(500) {
                last_status = now;
                *status.lock().unwrap() = ModuleStatus::Running(s.status());
            }
            std::mem::take(&mut s.out)
        };
        send_out(&socks, out);
    }
}

#[async_trait]
impl Module for SimAutopilot {
    fn kind(&self) -> ModuleKind {
        ModuleKind::Autopilot
    }

    fn backend(&self) -> Backend {
        Backend::Simulated
    }

    fn name(&self) -> &str {
        "sim-autopilot"
    }

    async fn start(&mut self, ctx: &SimContext) -> anyhow::Result<()> {
        self.halt().await;
        *self.status.lock().unwrap() = ModuleStatus::Starting;
        let (gcs_local, api_local) = local_ports(ctx.ports);
        let bind = async |port: u16| {
            UdpSocket::bind(("0.0.0.0", port))
                .await
                .map(Arc::new)
                .with_context(|| format!("bind udp {port}"))
        };
        let socks = match (bind(gcs_local).await, bind(api_local).await) {
            (Ok(g), Ok(a)) => [g, a],
            (Err(e), _) | (_, Err(e)) => {
                *self.status.lock().unwrap() = ModuleStatus::Failed(format!("{e:#}"));
                ctx.log("sim-autopilot", LogLevel::Error, format!("{e:#}"));
                return Err(e);
            }
        };
        let server = Arc::new(Mutex::new(Server::new(ctx.airframe, ctx.home, ctx.ports)));
        for link in [GCS_LINK, API_LINK] {
            self.tasks
                .push(tokio::spawn(rx_loop(link, socks.clone(), server.clone())));
        }
        self.tasks.push(tokio::spawn(sim_loop(
            socks,
            server.clone(),
            self.time_scale,
            self.status.clone(),
        )));
        let summary = server.lock().unwrap().status();
        *self.status.lock().unwrap() = ModuleStatus::Running(summary);
        ctx.log(
            "sim-autopilot",
            LogLevel::Info,
            format!(
                "{} at {:.6},{:.6}; {} link udp {gcs_local} -> 127.0.0.1:{}, {} link udp {api_local} -> 127.0.0.1:{}, time x{}",
                ctx.airframe.label(),
                ctx.home.lat,
                ctx.home.lon,
                LINK_NAMES[GCS_LINK],
                ctx.ports.gcs,
                LINK_NAMES[API_LINK],
                ctx.ports.api,
                self.time_scale
            ),
        );
        Ok(())
    }

    async fn stop(&mut self) -> anyhow::Result<()> {
        self.halt().await;
        *self.status.lock().unwrap() = ModuleStatus::Stopped;
        Ok(())
    }

    fn status(&self) -> ModuleStatus {
        self.status.lock().unwrap().clone()
    }
}
