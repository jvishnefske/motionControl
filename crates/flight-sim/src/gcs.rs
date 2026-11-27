//! [`SimGroundStation`]: a minimal Rust ground station standing in for
//! QGroundControl. It listens on the GCS port, answers the vehicle with a
//! 1 Hz GCS heartbeat (so the vehicle sees a GCS link) and counts the
//! vehicle heartbeats it hears.

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::Context;
use async_trait::async_trait;
use flight_core::{Backend, LogLevel, Module, ModuleKind, ModuleStatus, SimContext};
use mavlink::MavHeader;
use mavlink::common::*;
use tokio::net::UdpSocket;
use tokio::task::JoinHandle;

use crate::mav;

/// QGroundControl's MAVLink identity.
pub const GCS_SYSTEM_ID: u8 = 255;
pub const GCS_COMPONENT_ID: u8 = 190;

#[derive(Default)]
struct Seen {
    heartbeats: u64,
    vehicle: Option<(u8, SocketAddr, MavType, u32)>,
}

pub struct SimGroundStation {
    seen: Arc<Mutex<Seen>>,
    state: Arc<Mutex<ModuleStatus>>,
    tasks: Vec<JoinHandle<()>>,
}

impl Default for SimGroundStation {
    fn default() -> Self {
        Self::new()
    }
}

impl SimGroundStation {
    pub fn new() -> SimGroundStation {
        SimGroundStation {
            seen: Arc::default(),
            state: Arc::new(Mutex::new(ModuleStatus::Stopped)),
            tasks: Vec::new(),
        }
    }

    /// Vehicle heartbeats heard since the last start.
    pub fn vehicle_heartbeats(&self) -> u64 {
        self.seen.lock().unwrap().heartbeats
    }

    async fn halt(&mut self) {
        for t in &self.tasks {
            t.abort();
        }
        for t in self.tasks.drain(..) {
            let _ = t.await;
        }
    }
}

impl Drop for SimGroundStation {
    fn drop(&mut self) {
        for t in &self.tasks {
            t.abort();
        }
    }
}

async fn rx_loop(sock: Arc<UdpSocket>, seen: Arc<Mutex<Seen>>) {
    let mut buf = vec![0u8; 65536];
    loop {
        let Ok((n, from)) = sock.recv_from(&mut buf).await else {
            tokio::time::sleep(Duration::from_millis(5)).await;
            continue;
        };
        for f in mav::decode(&buf[..n]) {
            if let MavMessage::HEARTBEAT(hb) = f.msg {
                if hb.autopilot == MavAutopilot::MAV_AUTOPILOT_INVALID
                    || hb.mavtype == MavType::MAV_TYPE_GCS
                {
                    continue;
                }
                let mut s = seen.lock().unwrap();
                s.heartbeats += 1;
                s.vehicle = Some((f.header.system_id, from, hb.mavtype, hb.custom_mode));
            }
        }
    }
}

async fn heartbeat_loop(sock: Arc<UdpSocket>, seen: Arc<Mutex<Seen>>) {
    let mut tick = tokio::time::interval(Duration::from_secs(1));
    let mut seq = 0u8;
    loop {
        tick.tick().await;
        let Some((_, peer, _, _)) = seen.lock().unwrap().vehicle else {
            continue;
        };
        let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GCS,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                | MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        });
        let header = MavHeader {
            system_id: GCS_SYSTEM_ID,
            component_id: GCS_COMPONENT_ID,
            sequence: seq,
        };
        seq = seq.wrapping_add(1);
        let _ = sock.try_send_to(&mav::encode(header, &hb, 0), peer);
    }
}

#[async_trait]
impl Module for SimGroundStation {
    fn kind(&self) -> ModuleKind {
        ModuleKind::GroundStation
    }

    fn backend(&self) -> Backend {
        Backend::Simulated
    }

    fn name(&self) -> &str {
        "sim-gcs"
    }

    async fn start(&mut self, ctx: &SimContext) -> anyhow::Result<()> {
        self.halt().await;
        *self.seen.lock().unwrap() = Seen::default();
        let port = ctx.ports.gcs;
        let sock = match UdpSocket::bind(("0.0.0.0", port))
            .await
            .with_context(|| format!("bind udp {port}"))
        {
            Ok(s) => Arc::new(s),
            Err(e) => {
                *self.state.lock().unwrap() = ModuleStatus::Failed(format!("{e:#}"));
                ctx.log("sim-gcs", LogLevel::Error, format!("{e:#}"));
                return Err(e);
            }
        };
        self.tasks
            .push(tokio::spawn(rx_loop(sock.clone(), self.seen.clone())));
        self.tasks
            .push(tokio::spawn(heartbeat_loop(sock, self.seen.clone())));
        *self.state.lock().unwrap() = ModuleStatus::Running(String::new());
        ctx.log(
            "sim-gcs",
            LogLevel::Info,
            format!("listening on udp {port}"),
        );
        Ok(())
    }

    async fn stop(&mut self) -> anyhow::Result<()> {
        self.halt().await;
        *self.state.lock().unwrap() = ModuleStatus::Stopped;
        Ok(())
    }

    fn status(&self) -> ModuleStatus {
        match &*self.state.lock().unwrap() {
            ModuleStatus::Running(_) => {
                let s = self.seen.lock().unwrap();
                ModuleStatus::Running(match s.vehicle {
                    None => "waiting for a vehicle heartbeat".into(),
                    Some((sys, addr, ty, mode)) => format!(
                        "{} vehicle heartbeats; sys {sys} {:?} at {addr}, {}",
                        s.heartbeats,
                        ty,
                        mav::px4::name(mode)
                    ),
                })
            }
            other => other.clone(),
        }
    }
}
