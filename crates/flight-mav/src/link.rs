//! [`VehicleLink`]: a MAVLink client on the API port, like MAVSDK talking to
//! PX4 SITL instance 0 (PX4 sends from udp 14580 to 14540; we listen on
//! `udpin:0.0.0.0:{api}` and answer to whoever sent the last heartbeat).

use std::net::SocketAddr;
use std::sync::atomic::{AtomicU8, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{Context, anyhow, bail};
use async_trait::async_trait;
use flight_core::{
    FlightMode, GeoPoint, MavPorts, Mission, MissionItem, Telemetry, VehicleControl,
};
use mavlink::MavHeader;
use mavlink::common::*;
use tokio::net::UdpSocket;
use tokio::sync::{broadcast, watch};
use tokio::task::JoinHandle;
use tokio::time::timeout_at;

use crate::{codec, px4};

/// Our MAVLink identity (MAVSDK's default ground-station ids).
pub const OWN_SYSTEM_ID: u8 = 245;
pub const OWN_COMPONENT_ID: u8 = 190;

/// A vehicle is considered gone after this long without a heartbeat.
const HEARTBEAT_TIMEOUT: Duration = Duration::from_secs(3);

/// Stream rates requested from the vehicle (message id, Hz).
const STREAMS: [(u32, f32); 8] = [
    (33, 10.0), // GLOBAL_POSITION_INT
    (30, 10.0), // ATTITUDE
    (245, 2.0), // EXTENDED_SYS_STATE
    (42, 2.0),  // MISSION_CURRENT
    (24, 2.0),  // GPS_RAW_INT
    (1, 1.0),   // SYS_STATUS
    (147, 1.0), // BATTERY_STATUS
    (242, 0.5), // HOME_POSITION
];

/// What we know about the vehicle currently on the port. Replaced whenever a
/// different vehicle (new address, system id, type, a reboot, or a
/// heartbeat gap) shows up.
struct Session {
    peer: SocketAddr,
    system: u8,
    component: u8,
    mavtype: MavType,
    last_heartbeat: Instant,
    last_home_request: Instant,
    boot_ms_base: Option<u32>,
    last_boot_ms: u32,
    home: Option<GeoPoint>,
    fix_type: u8,
    have_position: bool,
    landed_known: bool,
    tel: Telemetry,
}

impl Session {
    fn new(peer: SocketAddr, h: &MavHeader, mavtype: MavType) -> Session {
        let now = Instant::now();
        Session {
            peer,
            system: h.system_id,
            component: h.component_id,
            mavtype,
            last_heartbeat: now,
            last_home_request: now,
            boot_ms_base: None,
            last_boot_ms: 0,
            home: None,
            fix_type: 0,
            have_position: false,
            landed_known: false,
            tel: Telemetry {
                time_s: 0.0,
                position: GeoPoint {
                    lat: 0.0,
                    lon: 0.0,
                    alt_m: 0.0,
                },
                rel_alt_m: 0.0,
                vel_ned: [0.0; 3],
                attitude: [0.0; 3],
                armed: false,
                mode: FlightMode::Unknown,
                landed: true,
                mission_seq: 0,
                mission_reached: None,
                battery_pct: 100.0,
                home: None,
            },
        }
    }

    fn alive(&self) -> bool {
        self.last_heartbeat.elapsed() < HEARTBEAT_TIMEOUT
    }
}

struct Inner {
    sock: UdpSocket,
    seq: AtomicU8,
    session: Mutex<Option<Session>>,
    sessions_seen: AtomicU64,
    tel_tx: watch::Sender<Option<Telemetry>>,
    msgs: broadcast::Sender<(MavHeader, MavMessage)>,
    configure: Mutex<Option<JoinHandle<()>>>,
}

/// MAVLink client for whatever autopilot is on the API port.
pub struct VehicleLink {
    inner: Arc<Inner>,
    tasks: Vec<JoinHandle<()>>,
}

impl Drop for VehicleLink {
    fn drop(&mut self) {
        for t in &self.tasks {
            t.abort();
        }
        if let Some(t) = self.inner.configure.lock().unwrap().take() {
            t.abort();
        }
    }
}

impl VehicleLink {
    /// Bind `0.0.0.0:{ports.api}` and start listening. Returns at once; use
    /// [`VehicleControl::wait_ready`] to wait for a vehicle.
    pub async fn connect(ports: MavPorts) -> anyhow::Result<Arc<VehicleLink>> {
        let sock = UdpSocket::bind(("0.0.0.0", ports.api))
            .await
            .with_context(|| format!("bind udp 0.0.0.0:{}", ports.api))?;
        let (tel_tx, _) = watch::channel(None);
        let (msgs, _) = broadcast::channel(4096);
        let inner = Arc::new(Inner {
            sock,
            seq: AtomicU8::new(0),
            session: Mutex::new(None),
            sessions_seen: AtomicU64::new(0),
            tel_tx,
            msgs,
            configure: Mutex::new(None),
        });
        let tasks = vec![
            tokio::spawn(rx_loop(inner.clone())),
            tokio::spawn(heartbeat_loop(inner.clone())),
        ];
        Ok(Arc::new(VehicleLink { inner, tasks }))
    }

    /// `(system, component)` of the vehicle currently heard, if any.
    pub fn target(&self) -> Option<(u8, u8)> {
        self.inner
            .session
            .lock()
            .unwrap()
            .as_ref()
            .filter(|s| s.alive())
            .map(|s| (s.system, s.component))
    }

    /// How many distinct vehicle sessions this link has seen (increments when
    /// the vehicle on the port is swapped or rebooted).
    pub fn sessions_seen(&self) -> u64 {
        self.inner.sessions_seen.load(Ordering::Relaxed)
    }

    /// Local address of the API socket.
    pub fn local_addr(&self) -> std::io::Result<SocketAddr> {
        self.inner.sock.local_addr()
    }

    /// What is still missing before the vehicle is ready, or `None` if ready.
    fn not_ready(&self) -> Option<&'static str> {
        let s = self.inner.session.lock().unwrap();
        match s.as_ref() {
            None => Some("no vehicle heartbeat"),
            Some(s) if !s.alive() => Some("no vehicle heartbeat"),
            Some(s) if !s.have_position => Some("no GLOBAL_POSITION_INT"),
            Some(s) if s.home.is_none() => Some("no home position"),
            Some(s) if s.fix_type < GpsFixType::GPS_FIX_TYPE_3D_FIX as u8 => Some("no 3D GPS fix"),
            Some(_) => None,
        }
    }

    /// Send a COMMAND_LONG and wait for an accepting COMMAND_ACK.
    pub async fn command(&self, cmd: MavCmd, params: [f32; 7]) -> anyhow::Result<()> {
        self.inner
            .command(cmd, params, 5, Duration::from_millis(1000))
            .await
    }
}

impl Inner {
    fn target(&self) -> anyhow::Result<(SocketAddr, u8, u8)> {
        let s = self.session.lock().unwrap();
        match s.as_ref() {
            Some(s) if s.alive() => Ok((s.peer, s.system, s.component)),
            _ => Err(anyhow!("no vehicle on the link")),
        }
    }

    fn send_to(&self, peer: SocketAddr, msg: &MavMessage) {
        let header = MavHeader {
            system_id: OWN_SYSTEM_ID,
            component_id: OWN_COMPONENT_ID,
            sequence: self.seq.fetch_add(1, Ordering::Relaxed),
        };
        let _ = self.sock.try_send_to(&codec::encode(header, msg), peer);
    }

    fn send(&self, msg: &MavMessage) -> anyhow::Result<(u8, u8)> {
        let (peer, sys, comp) = self.target()?;
        self.send_to(peer, msg);
        Ok((sys, comp))
    }

    /// Wait until `f` maps a message from system `sys` to `Some`, or until `deadline`.
    async fn wait_for<T>(
        &self,
        rx: &mut broadcast::Receiver<(MavHeader, MavMessage)>,
        sys: u8,
        deadline: tokio::time::Instant,
        mut f: impl FnMut(&MavMessage) -> Option<T>,
    ) -> anyhow::Result<Option<T>> {
        loop {
            match timeout_at(deadline, rx.recv()).await {
                Err(_) => return Ok(None),
                Ok(Err(broadcast::error::RecvError::Lagged(_))) => continue,
                Ok(Err(broadcast::error::RecvError::Closed)) => bail!("link closed"),
                Ok(Ok((h, m))) => {
                    if h.system_id != sys {
                        continue;
                    }
                    if let Some(v) = f(&m) {
                        return Ok(Some(v));
                    }
                }
            }
        }
    }

    async fn command(
        &self,
        cmd: MavCmd,
        p: [f32; 7],
        attempts: u8,
        wait: Duration,
    ) -> anyhow::Result<()> {
        for attempt in 0..attempts {
            let mut rx = self.msgs.subscribe();
            let (_, sys, comp) = self.target()?;
            let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                param1: p[0],
                param2: p[1],
                param3: p[2],
                param4: p[3],
                param5: p[4],
                param6: p[5],
                param7: p[6],
                command: cmd,
                target_system: sys,
                target_component: comp,
                confirmation: attempt,
            });
            self.send(&msg)?;
            let mut deadline = tokio::time::Instant::now() + wait;
            loop {
                let ack = self
                    .wait_for(&mut rx, sys, deadline, |m| match m {
                        MavMessage::COMMAND_ACK(a) if a.command == cmd => Some(a.result),
                        _ => None,
                    })
                    .await?;
                match ack {
                    None => break,
                    Some(MavResult::MAV_RESULT_ACCEPTED) => return Ok(()),
                    Some(MavResult::MAV_RESULT_IN_PROGRESS) => {
                        deadline = tokio::time::Instant::now() + wait * 5
                    }
                    Some(MavResult::MAV_RESULT_TEMPORARILY_REJECTED) if attempt + 1 < attempts => {
                        tokio::time::sleep(Duration::from_millis(300)).await;
                        break;
                    }
                    Some(r) => bail!("{cmd:?} rejected: {r:?}"),
                }
            }
        }
        bail!("{cmd:?}: no COMMAND_ACK after {attempts} attempts")
    }

    /// Request stream rates and the home position from a newly seen vehicle.
    async fn configure(self: Arc<Self>) {
        tokio::time::sleep(Duration::from_millis(100)).await;
        for (id, hz) in STREAMS {
            let interval_us = 1e6 / hz;
            let p = [id as f32, interval_us, 0.0, 0.0, 0.0, 0.0, 0.0];
            if let Err(e) = self
                .command(
                    MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                    p,
                    2,
                    Duration::from_millis(500),
                )
                .await
            {
                tracing::debug!("SET_MESSAGE_INTERVAL {id}: {e:#}");
            }
        }
        self.request_home();
    }

    /// Apply an acknowledged state change before the vehicle reports it.
    fn optimistic(&self, f: impl FnOnce(&mut Telemetry)) {
        if let Some(s) = self.session.lock().unwrap().as_mut() {
            f(&mut s.tel);
            if s.have_position {
                self.tel_tx.send_replace(Some(s.tel));
            }
        }
    }

    fn request_home(&self) {
        if let Ok((_, sys, comp)) = self.target() {
            let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                param1: 242.0,
                command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
                target_system: sys,
                target_component: comp,
                ..Default::default()
            });
            let _ = self.send(&msg);
        }
    }

    fn start_session(self: &Arc<Self>, slot: &mut Option<Session>, session: Session) {
        *slot = Some(session);
        self.sessions_seen.fetch_add(1, Ordering::Relaxed);
        self.tel_tx.send_replace(None);
        let me = self.clone();
        let task = tokio::spawn(me.configure());
        if let Some(old) = self.configure.lock().unwrap().replace(task) {
            old.abort();
        }
    }

    fn handle(self: &Arc<Self>, from: SocketAddr, h: MavHeader, m: &MavMessage) {
        if h.system_id == OWN_SYSTEM_ID && h.component_id == OWN_COMPONENT_ID {
            return;
        }
        let mut guard = self.session.lock().unwrap();
        if let MavMessage::HEARTBEAT(hb) = m {
            let is_vehicle = hb.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID
                && hb.mavtype != MavType::MAV_TYPE_GCS;
            if !is_vehicle {
                return;
            }
            let new_vehicle = match guard.as_ref() {
                None => true,
                Some(s) => {
                    s.peer != from
                        || s.system != h.system_id
                        || s.mavtype != hb.mavtype
                        || !s.alive()
                }
            };
            if new_vehicle {
                tracing::info!(
                    "vehicle sys {} comp {} ({:?}) at {from}",
                    h.system_id,
                    h.component_id,
                    hb.mavtype
                );
                self.start_session(&mut guard, Session::new(from, &h, hb.mavtype));
            }
        }
        let Some(s) = guard.as_mut() else { return };
        if s.system != h.system_id || s.peer != from {
            return;
        }
        let mut publish = true;
        match m {
            MavMessage::HEARTBEAT(hb) => {
                if h.component_id != s.component {
                    return;
                }
                s.last_heartbeat = Instant::now();
                s.tel.armed = hb
                    .base_mode
                    .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
                s.tel.mode = if hb.autopilot == MavAutopilot::MAV_AUTOPILOT_PX4 {
                    px4::flight_mode(hb.custom_mode)
                } else {
                    FlightMode::Unknown
                };
            }
            MavMessage::GLOBAL_POSITION_INT(p) => {
                if s.have_position && p.time_boot_ms + 1000 < s.last_boot_ms {
                    // The autopilot rebooted or was replaced on the same port.
                    tracing::info!("vehicle time went backwards; new session");
                    let fresh = Session::new(from, &h, s.mavtype);
                    self.start_session(&mut guard, fresh);
                    return;
                }
                let base = *s.boot_ms_base.get_or_insert(p.time_boot_ms);
                s.last_boot_ms = p.time_boot_ms;
                s.tel.time_s = f64::from(p.time_boot_ms.saturating_sub(base)) / 1000.0;
                s.tel.position = GeoPoint {
                    lat: f64::from(p.lat) / 1e7,
                    lon: f64::from(p.lon) / 1e7,
                    alt_m: f64::from(p.alt) / 1000.0,
                };
                s.tel.rel_alt_m = f64::from(p.relative_alt) / 1000.0;
                s.tel.vel_ned = [
                    f64::from(p.vx) / 100.0,
                    f64::from(p.vy) / 100.0,
                    f64::from(p.vz) / 100.0,
                ];
                if !s.landed_known {
                    s.tel.landed = s.tel.rel_alt_m < 0.5 && s.tel.ground_speed_ms() < 0.5;
                }
                s.have_position = true;
            }
            MavMessage::ATTITUDE(a) => {
                s.tel.attitude = [f64::from(a.roll), f64::from(a.pitch), f64::from(a.yaw)];
            }
            MavMessage::EXTENDED_SYS_STATE(e) => match e.landed_state {
                MavLandedState::MAV_LANDED_STATE_UNDEFINED => {}
                st => {
                    s.landed_known = true;
                    s.tel.landed = st == MavLandedState::MAV_LANDED_STATE_ON_GROUND;
                }
            },
            MavMessage::MISSION_CURRENT(c) => s.tel.mission_seq = c.seq,
            MavMessage::MISSION_ITEM_REACHED(r) => {
                s.tel.mission_reached =
                    Some(s.tel.mission_reached.map_or(r.seq, |old| old.max(r.seq)));
            }
            MavMessage::HOME_POSITION(hp) => {
                let home = GeoPoint {
                    lat: f64::from(hp.latitude) / 1e7,
                    lon: f64::from(hp.longitude) / 1e7,
                    alt_m: f64::from(hp.altitude) / 1000.0,
                };
                s.home = Some(home);
                s.tel.home = Some(home);
            }
            MavMessage::SYS_STATUS(st) if st.battery_remaining >= 0 => {
                s.tel.battery_pct = f32::from(st.battery_remaining);
            }
            MavMessage::BATTERY_STATUS(b) if b.battery_remaining >= 0 && b.id == 0 => {
                s.tel.battery_pct = f32::from(b.battery_remaining);
            }
            MavMessage::GPS_RAW_INT(g) => s.fix_type = g.fix_type as u8,
            _ => publish = false,
        }
        if publish && s.have_position {
            self.tel_tx.send_replace(Some(s.tel));
        }
    }
}

async fn rx_loop(inner: Arc<Inner>) {
    let mut buf = vec![0u8; 65536];
    loop {
        let (n, from) = match inner.sock.recv_from(&mut buf).await {
            Ok(v) => v,
            Err(e) => {
                tracing::debug!("api socket: {e}");
                tokio::time::sleep(Duration::from_millis(10)).await;
                continue;
            }
        };
        for (h, m) in codec::decode(&buf[..n]) {
            inner.handle(from, h, &m);
            let _ = inner.msgs.send((h, m));
        }
    }
}

/// 1 Hz GCS heartbeat to the vehicle, plus vehicle-loss and home watchdogs.
async fn heartbeat_loop(inner: Arc<Inner>) {
    let mut tick = tokio::time::interval(Duration::from_millis(250));
    let mut n = 0u64;
    loop {
        tick.tick().await;
        n += 1;
        let (peer, want_home) = {
            let mut guard = inner.session.lock().unwrap();
            match guard.as_mut() {
                Some(s) if !s.alive() => {
                    tracing::info!("vehicle sys {} lost", s.system);
                    *guard = None;
                    inner.tel_tx.send_replace(None);
                    (None, false)
                }
                Some(s) => {
                    let want =
                        s.home.is_none() && s.last_home_request.elapsed() > Duration::from_secs(2);
                    if want {
                        s.last_home_request = Instant::now();
                    }
                    (Some(s.peer), want)
                }
                None => (None, false),
            }
        };
        let Some(peer) = peer else { continue };
        if n.is_multiple_of(4) {
            let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: MavType::MAV_TYPE_GCS,
                autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
                base_mode: MavModeFlag::empty(),
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            });
            inner.send_to(peer, &hb);
        }
        if want_home {
            inner.request_home();
        }
    }
}

/// `flight_core::MissionItem` -> MISSION_ITEM_INT (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT).
// The _INT frame is deprecated in favour of its synonym, but it is what PX4,
// QGC and MAVSDK send for mission items.
#[allow(deprecated)]
pub fn mission_item_int(seq: u16, item: &MissionItem, target: (u8, u8)) -> MISSION_ITEM_INT_DATA {
    let deg = |v: f64| (v * 1e7).round() as i32;
    let (command, p, x, y, z) = match *item {
        MissionItem::Takeoff { lat, lon, alt_m } => (
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            [0.0, 0.0, 0.0, f32::NAN],
            deg(lat),
            deg(lon),
            alt_m as f32,
        ),
        MissionItem::Waypoint {
            lat,
            lon,
            alt_m,
            acceptance_m,
        } => (
            MavCmd::MAV_CMD_NAV_WAYPOINT,
            [0.0, acceptance_m as f32, 0.0, f32::NAN],
            deg(lat),
            deg(lon),
            alt_m as f32,
        ),
        MissionItem::Land { lat, lon } => (
            MavCmd::MAV_CMD_NAV_LAND,
            [0.0, 0.0, 0.0, f32::NAN],
            deg(lat),
            deg(lon),
            0.0,
        ),
        MissionItem::ReturnToLaunch => (MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 4], 0, 0, 0.0),
    };
    MISSION_ITEM_INT_DATA {
        param1: p[0],
        param2: p[1],
        param3: p[2],
        param4: p[3],
        x,
        y,
        z,
        seq,
        command,
        target_system: target.0,
        target_component: target.1,
        frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        current: u8::from(seq == 0),
        autocontinue: 1,
        ..Default::default()
    }
}

#[derive(Debug)]
enum UploadEvent {
    Request(u16),
    Ack(MavMissionResult),
}

// MISSION_REQUEST is deprecated but older senders still use it.
#[allow(deprecated)]
fn upload_event(m: &MavMessage) -> Option<UploadEvent> {
    let ours = |t: u8| t == OWN_SYSTEM_ID || t == 0;
    match m {
        MavMessage::MISSION_REQUEST_INT(r) if ours(r.target_system) => {
            Some(UploadEvent::Request(r.seq))
        }
        MavMessage::MISSION_REQUEST(r) if ours(r.target_system) => {
            Some(UploadEvent::Request(r.seq))
        }
        MavMessage::MISSION_ACK(a) if ours(a.target_system) => Some(UploadEvent::Ack(a.mavtype)),
        _ => None,
    }
}

#[async_trait]
impl VehicleControl for VehicleLink {
    fn telemetry(&self) -> watch::Receiver<Option<Telemetry>> {
        self.inner.tel_tx.subscribe()
    }

    async fn wait_ready(&self, timeout: Duration) -> anyhow::Result<()> {
        let deadline = Instant::now() + timeout;
        loop {
            let Some(why) = self.not_ready() else {
                return Ok(());
            };
            if Instant::now() >= deadline {
                bail!(
                    "vehicle not ready after {:.0} s: {why}",
                    timeout.as_secs_f64()
                );
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    }

    async fn upload_mission(&self, mission: &Mission) -> anyhow::Result<()> {
        let inner = &self.inner;
        let (_, sys, comp) = inner.target()?;
        let n = u16::try_from(mission.items.len()).context("mission too long")?;
        let wait = Duration::from_millis(1500);

        // 1. Clear.
        let mut cleared = false;
        for _ in 0..3 {
            let mut rx = inner.msgs.subscribe();
            inner.send(&MavMessage::MISSION_CLEAR_ALL(MISSION_CLEAR_ALL_DATA {
                target_system: sys,
                target_component: comp,
                ..Default::default()
            }))?;
            let deadline = tokio::time::Instant::now() + wait;
            match inner.wait_for(&mut rx, sys, deadline, upload_event).await? {
                Some(UploadEvent::Ack(MavMissionResult::MAV_MISSION_ACCEPTED)) => {
                    cleared = true;
                    break;
                }
                Some(UploadEvent::Ack(r)) => bail!("MISSION_CLEAR_ALL rejected: {r:?}"),
                _ => {}
            }
        }
        if !cleared {
            bail!("MISSION_CLEAR_ALL: no MISSION_ACK");
        }
        if n == 0 {
            return Ok(());
        }

        // 2. Count, then answer item requests until the final ACK.
        let items: Vec<MavMessage> = mission
            .items
            .iter()
            .enumerate()
            .map(|(i, it)| {
                MavMessage::MISSION_ITEM_INT(mission_item_int(i as u16, it, (sys, comp)))
            })
            .collect();
        let count = MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
            count: n,
            target_system: sys,
            target_component: comp,
            ..Default::default()
        });
        let mut rx = inner.msgs.subscribe();
        inner.send(&count)?;
        let mut last_sent: Option<u16> = None;
        let mut retries = 0;
        let overall = Instant::now() + Duration::from_secs(30);
        loop {
            if Instant::now() > overall {
                bail!("mission upload timed out");
            }
            let deadline = tokio::time::Instant::now() + wait;
            match inner.wait_for(&mut rx, sys, deadline, upload_event).await? {
                None => {
                    retries += 1;
                    if retries > 5 {
                        bail!(
                            "mission upload: vehicle stopped responding (last item sent: {last_sent:?})"
                        );
                    }
                    match last_sent {
                        None => inner.send(&count)?,
                        Some(seq) => inner.send(&items[usize::from(seq)])?,
                    };
                }
                Some(UploadEvent::Request(seq)) => {
                    let item = items
                        .get(usize::from(seq))
                        .ok_or_else(|| anyhow!("vehicle requested item {seq} of {n}"))?;
                    inner.send(item)?;
                    last_sent = Some(seq);
                    retries = 0;
                }
                Some(UploadEvent::Ack(MavMissionResult::MAV_MISSION_ACCEPTED)) => {
                    if last_sent == Some(n - 1) {
                        break;
                    }
                    // A late ACK for the clear; keep going.
                }
                Some(UploadEvent::Ack(r)) => bail!("mission rejected: {r:?}"),
            }
        }
        if let Some(s) = inner.session.lock().unwrap().as_mut() {
            s.tel.mission_reached = None;
            s.tel.mission_seq = 0;
        }
        Ok(())
    }

    async fn arm(&self) -> anyhow::Result<()> {
        self.command(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
        .context("arm")?;
        // The next heartbeat may be up to a second away; reflect the accepted
        // arm now so the telemetry never shows a climbing, disarmed vehicle.
        self.inner.optimistic(|t| t.armed = true);
        Ok(())
    }

    async fn start_mission(&self) -> anyhow::Result<()> {
        let custom = f32::from(px4::MAIN_AUTO);
        let sub = f32::from(px4::AUTO_MISSION);
        let flags = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.bits() as f32;
        self.command(
            MavCmd::MAV_CMD_DO_SET_MODE,
            [flags, custom, sub, 0.0, 0.0, 0.0, 0.0],
        )
        .await
        .context("set mode AUTO.MISSION")?;
        self.inner.optimistic(|t| t.mode = FlightMode::Mission);
        // PX4 already starts the mission on the mode switch; MISSION_START
        // additionally rewinds to item 0. Not fatal if a vehicle refuses it.
        if let Err(e) = self
            .command(
                MavCmd::MAV_CMD_MISSION_START,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
            .await
        {
            tracing::warn!("MISSION_START: {e:#}");
        }
        Ok(())
    }

    async fn return_to_launch(&self) -> anyhow::Result<()> {
        self.command(MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7])
            .await
            .context("RTL")
    }

    async fn land(&self) -> anyhow::Result<()> {
        let nan = f32::NAN;
        self.command(
            MavCmd::MAV_CMD_NAV_LAND,
            [0.0, 0.0, 0.0, nan, nan, nan, nan],
        )
        .await
        .context("land")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::Airframe;

    #[test]
    #[allow(deprecated)]
    fn items_map_to_mavlink() {
        let m = Mission::builtin("box", Airframe::Quadrotor, GeoPoint::PX4_DEFAULT_HOME).unwrap();
        let first = mission_item_int(0, &m.items[0], (1, 1));
        assert_eq!(first.command, MavCmd::MAV_CMD_NAV_TAKEOFF);
        assert_eq!(first.current, 1);
        assert_eq!(first.frame, MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        let wp = mission_item_int(1, &m.items[1], (1, 1));
        assert_eq!(wp.command, MavCmd::MAV_CMD_NAV_WAYPOINT);
        assert_eq!(wp.param2, 2.0);
        assert_eq!(wp.z, 15.0);
        let last = mission_item_int(5, m.items.last().unwrap(), (1, 1));
        assert_eq!(last.command, MavCmd::MAV_CMD_NAV_LAND);
        let rtl = mission_item_int(0, &MissionItem::ReturnToLaunch, (1, 1));
        assert_eq!(rtl.command, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH);
    }
}
