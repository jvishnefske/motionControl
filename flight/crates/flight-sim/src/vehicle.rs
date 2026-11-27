//! The Rust autopilot's brain and body: flight modes, mission execution and
//! a simple dynamics model per airframe. Pure and deterministic; the MAVLink
//! server in `autopilot.rs` drives it with fixed time steps.
//!
//! Frame: `n`/`e` metres from home, `h` metres above home (up positive).

use std::f64::consts::{FRAC_PI_2, PI};

use flight_core::{Airframe, GeoPoint};
use mavlink::common::MavCmd;

use crate::mav::px4;

pub const G: f64 = 9.81;
/// Physics step, sim seconds (50 Hz).
pub const DT: f64 = 0.02;

/// Fixed-wing limits.
pub const FW_MIN_AIRSPEED: f64 = 12.0;
pub const FW_CRUISE: f64 = 15.0;
const FW_MAX_BANK: f64 = 35.0 * PI / 180.0;
const FW_MAX_CLIMB: f64 = 5.0;
const FW_MAX_SINK: f64 = 4.0;
const FW_LOITER_RADIUS: f64 = 80.0;
const FW_LOOKAHEAD: f64 = 60.0;

/// Multicopter limits.
pub const MC_MAX_H_SPEED: f64 = 5.0;
pub const MC_MAX_V_SPEED: f64 = 3.0;
const MC_MAX_ACCEL: f64 = 4.0;

/// One executable mission item, already converted to the local frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavItem {
    pub cmd: MavCmd,
    /// Local position; `None` means "where the vehicle is".
    pub pos: Option<(f64, f64)>,
    /// Altitude above home (NaN: keep current).
    pub alt: f64,
    /// Acceptance radius (0: airframe default).
    pub acceptance: f64,
    pub param1: f64,
}

/// What the vehicle does; the reported PX4 custom mode derives from it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mode {
    Hold,
    Takeoff,
    Mission,
    Rtl,
    Land,
    /// A manual/assisted mode (MANUAL, POSCTL, ...): behaves like Hold.
    Other(u32),
}

impl Mode {
    pub fn custom_mode(self) -> u32 {
        use px4::*;
        match self {
            Mode::Hold => encode(MAIN_AUTO, AUTO_LOITER),
            Mode::Takeoff => encode(MAIN_AUTO, AUTO_TAKEOFF),
            Mode::Mission => encode(MAIN_AUTO, AUTO_MISSION),
            Mode::Rtl => encode(MAIN_AUTO, AUTO_RTL),
            Mode::Land => encode(MAIN_AUTO, AUTO_LAND),
            Mode::Other(cm) => cm,
        }
    }

    pub fn is_auto(self) -> bool {
        !matches!(self, Mode::Other(_))
    }
}

/// Things the MAVLink side should tell the ground station.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    Reached(u16),
    Current(u16),
    Text(String),
}

/// Guidance target for one step.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Guide {
    /// Sit on the ground.
    Ground,
    /// Climb to `alt`; fixed-wing rolls/climbs along `from` -> `to`.
    Takeoff {
        alt: f64,
        from: (f64, f64),
        to: (f64, f64),
    },
    /// Fly to a point, tracking the line from `from`.
    Goto {
        to: (f64, f64),
        alt: f64,
        from: (f64, f64),
    },
    /// Circle (fixed-wing) or hover (multicopter) at a point.
    Loiter { at: (f64, f64), alt: f64 },
    /// Descend to the ground at a point and land.
    Descend { at: (f64, f64) },
}

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub airframe: Airframe,
    pub home: GeoPoint,
    pub time_s: f64,
    pub n: f64,
    pub e: f64,
    pub h: f64,
    pub vn: f64,
    pub ve: f64,
    pub vu: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    /// Fixed-wing airspeed (multicopter: ground speed).
    pub airspeed: f64,
    pub armed: bool,
    pub landed: bool,
    pub mode: Mode,
    pub mission: Vec<NavItem>,
    pub current: usize,
    pub battery_pct: f64,
    // Parameters.
    pub rtl_alt: f64,
    pub takeoff_alt: f64,
    pub events: Vec<Event>,
    // Mode/item state.
    leg_start: (f64, f64),
    hold: (f64, f64, f64),
    takeoff_from: (f64, f64),
    takeoff_to: (f64, f64),
    item_started: bool,
    descending: bool,
    item_timer: f64,
    land_timer: f64,
    rtl_target_alt: f64,
    rtl_from: (f64, f64),
}

fn wrap_pi(a: f64) -> f64 {
    let mut a = (a + PI) % (2.0 * PI);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    a - PI
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (b.0 - a.0).hypot(b.1 - a.1)
}

fn bearing(from: (f64, f64), to: (f64, f64)) -> f64 {
    (to.1 - from.1).atan2(to.0 - from.0)
}

/// First-order lag of `x` toward `target` with time constant `tau`.
fn lag(x: f64, target: f64, tau: f64, dt: f64) -> f64 {
    x + (target - x) * (dt / tau).min(1.0)
}

impl Vehicle {
    pub fn new(airframe: Airframe, home: GeoPoint) -> Vehicle {
        let fw = airframe.is_fixed_wing();
        Vehicle {
            airframe,
            home,
            time_s: 0.0,
            n: 0.0,
            e: 0.0,
            h: 0.0,
            vn: 0.0,
            ve: 0.0,
            vu: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            airspeed: 0.0,
            armed: false,
            landed: true,
            mode: Mode::Hold,
            mission: Vec::new(),
            current: 0,
            battery_pct: 100.0,
            rtl_alt: if fw { 60.0 } else { 30.0 },
            takeoff_alt: if fw { 30.0 } else { 2.5 },
            events: Vec::new(),
            leg_start: (0.0, 0.0),
            hold: (0.0, 0.0, 0.0),
            takeoff_from: (0.0, 0.0),
            takeoff_to: (0.0, 0.0),
            item_started: false,
            descending: false,
            item_timer: 0.0,
            land_timer: 0.0,
            rtl_target_alt: 0.0,
            rtl_from: (0.0, 0.0),
        }
    }

    pub fn pos(&self) -> (f64, f64) {
        (self.n, self.e)
    }

    pub fn position(&self) -> GeoPoint {
        let mut p = self.home.offset(self.n, self.e);
        p.alt_m = self.home.alt_m + self.h;
        p
    }

    /// Local (north, east) of a lat/lon.
    pub fn local(&self, lat: f64, lon: f64) -> (f64, f64) {
        self.home.ne_to(&GeoPoint {
            lat,
            lon,
            alt_m: 0.0,
        })
    }

    pub fn custom_mode(&self) -> u32 {
        self.mode.custom_mode()
    }

    fn fw(&self) -> bool {
        self.airframe.is_fixed_wing()
    }

    fn text(&mut self, s: impl Into<String>) {
        self.events.push(Event::Text(s.into()));
    }

    // ---- commands -------------------------------------------------------

    pub fn arm(&mut self, arm: bool, force: bool) -> Result<(), &'static str> {
        if arm == self.armed {
            return Ok(());
        }
        if arm {
            if self.battery_pct < 5.0 {
                return Err("battery too low");
            }
            self.armed = true;
            self.land_timer = 0.0;
            self.text("Armed by external command");
        } else {
            if !self.landed && !force {
                return Err("in air");
            }
            self.disarm();
        }
        Ok(())
    }

    fn disarm(&mut self) {
        self.armed = false;
        self.land_timer = 0.0;
        self.text("Disarmed");
    }

    fn enter_hold(&mut self) {
        self.mode = Mode::Hold;
        self.hold = (self.n, self.e, self.h);
    }

    /// Set a PX4 custom mode.
    pub fn set_mode(&mut self, main: u8, sub: u8) -> Result<(), &'static str> {
        use px4::*;
        match (main, sub) {
            (MAIN_AUTO, AUTO_MISSION) => self.start_mission(None),
            (MAIN_AUTO, AUTO_LOITER) | (MAIN_AUTO, AUTO_READY) => {
                self.enter_hold();
                Ok(())
            }
            (MAIN_AUTO, AUTO_TAKEOFF) => self.takeoff(f64::NAN),
            (MAIN_AUTO, AUTO_RTL) => self.rtl(),
            (MAIN_AUTO, AUTO_LAND) => self.land(),
            (MAIN_MANUAL | MAIN_ALTCTL | MAIN_POSCTL | MAIN_ACRO | MAIN_STABILIZED, _) => {
                self.hold = (self.n, self.e, self.h);
                self.mode = Mode::Other(encode(main, 0));
                Ok(())
            }
            (MAIN_OFFBOARD, _) => Err("offboard not supported"),
            _ => Err("unknown mode"),
        }
    }

    /// Mission mode, optionally restarting at `first`. On the ground this
    /// arms and takes off, like PX4 starting a mission from the ground.
    pub fn start_mission(&mut self, first: Option<usize>) -> Result<(), &'static str> {
        if self.mission.is_empty() {
            return Err("no mission");
        }
        if let Some(i) = first {
            self.set_current(i)?;
        }
        if self.current >= self.mission.len() {
            self.set_current(0)?;
        }
        if !self.armed {
            self.arm(true, false)?;
        }
        if self.mode != Mode::Mission {
            self.mode = Mode::Mission;
            self.leg_start = self.pos();
            self.item_started = false;
            self.descending = false;
        }
        self.events.push(Event::Current(self.current as u16));
        Ok(())
    }

    pub fn set_current(&mut self, seq: usize) -> Result<(), &'static str> {
        if seq >= self.mission.len() {
            return Err("no such item");
        }
        self.current = seq;
        self.leg_start = self.pos();
        self.item_started = false;
        self.descending = false;
        self.item_timer = 0.0;
        self.events.push(Event::Current(seq as u16));
        Ok(())
    }

    pub fn set_mission(&mut self, items: Vec<NavItem>) {
        self.mission = items;
        self.current = 0;
        self.item_started = false;
        self.descending = false;
        if self.mode == Mode::Mission {
            self.leg_start = self.pos();
        }
        self.events.push(Event::Current(0));
    }

    pub fn takeoff(&mut self, alt: f64) -> Result<(), &'static str> {
        if !self.armed {
            return Err("not armed");
        }
        if alt.is_finite() && alt > 0.0 {
            self.takeoff_alt = alt;
        }
        self.mode = Mode::Takeoff;
        self.takeoff_from = self.pos();
        self.takeoff_to = (
            self.n + 1000.0 * self.yaw.cos(),
            self.e + 1000.0 * self.yaw.sin(),
        );
        Ok(())
    }

    pub fn rtl(&mut self) -> Result<(), &'static str> {
        if !self.armed || self.landed {
            return Err("not flying");
        }
        self.enter_rtl();
        Ok(())
    }

    fn enter_rtl(&mut self) {
        self.mode = Mode::Rtl;
        self.rtl_target_alt = self.h.max(self.rtl_alt);
        self.rtl_from = self.pos();
        self.descending = false;
    }

    pub fn land(&mut self) -> Result<(), &'static str> {
        if !self.armed || self.landed {
            return Err("not flying");
        }
        self.mode = Mode::Land;
        self.hold = (self.n, self.e, self.h);
        Ok(())
    }

    // ---- guidance -------------------------------------------------------

    fn hold_guide(&self) -> Guide {
        if self.landed {
            Guide::Ground
        } else {
            Guide::Loiter {
                at: (self.hold.0, self.hold.1),
                alt: self.hold.2,
            }
        }
    }

    fn advance(&mut self) {
        self.events.push(Event::Reached(self.current as u16));
        self.current += 1;
        self.leg_start = self.pos();
        self.item_started = false;
        self.descending = false;
        self.item_timer = 0.0;
        if self.current < self.mission.len() {
            self.events.push(Event::Current(self.current as u16));
        }
    }

    fn mission_guide(&mut self, dt: f64) -> Guide {
        // Bounded: each pass either returns or advances.
        for _ in 0..=self.mission.len() {
            let Some(&it) = self.mission.get(self.current) else {
                self.text("Mission finished, loitering");
                self.enter_hold();
                return self.hold_guide();
            };
            let here = self.pos();
            let target = it.pos.unwrap_or(here);
            let alt = if it.alt.is_finite() { it.alt } else { self.h };
            match it.cmd {
                MavCmd::MAV_CMD_NAV_TAKEOFF => {
                    if !self.item_started {
                        self.item_started = true;
                        self.takeoff_from = here;
                        self.takeoff_to = if dist(here, target) > 10.0 {
                            target
                        } else {
                            (
                                self.n + 1000.0 * self.yaw.cos(),
                                self.e + 1000.0 * self.yaw.sin(),
                            )
                        };
                    }
                    let tol = if self.fw() { 2.0 } else { 0.5 };
                    if !self.landed && self.h >= alt - tol {
                        self.advance();
                        continue;
                    }
                    return Guide::Takeoff {
                        alt,
                        from: self.takeoff_from,
                        to: self.takeoff_to,
                    };
                }
                MavCmd::MAV_CMD_NAV_WAYPOINT
                | MavCmd::MAV_CMD_NAV_LOITER_UNLIM
                | MavCmd::MAV_CMD_NAV_LOITER_TIME => {
                    if self.landed {
                        // No takeoff item: climb out toward the waypoint first.
                        return Guide::Takeoff {
                            alt,
                            from: here,
                            to: target,
                        };
                    }
                    let acc = if it.acceptance > 0.0 {
                        it.acceptance
                    } else {
                        self.airframe.acceptance_radius_m()
                    };
                    let alt_tol = if self.fw() { 10.0 } else { 1.0 };
                    let at_wp = self.item_started
                        || (dist(here, target) <= acc && (self.h - alt).abs() <= alt_tol);
                    if at_wp {
                        match it.cmd {
                            MavCmd::MAV_CMD_NAV_LOITER_UNLIM => {
                                self.item_started = true;
                                return Guide::Loiter { at: target, alt };
                            }
                            MavCmd::MAV_CMD_NAV_LOITER_TIME => {
                                self.item_started = true;
                                self.item_timer += dt;
                                if self.item_timer < it.param1 {
                                    return Guide::Loiter { at: target, alt };
                                }
                            }
                            _ => {}
                        }
                        self.advance();
                        continue;
                    }
                    return Guide::Goto {
                        to: target,
                        alt,
                        from: self.leg_start,
                    };
                }
                MavCmd::MAV_CMD_NAV_LAND => {
                    if self.fw() || self.descending || dist(here, target) < 1.0 {
                        self.descending = true;
                        return Guide::Descend { at: target };
                    }
                    return Guide::Goto {
                        to: target,
                        alt: self.h.max(1.0),
                        from: self.leg_start,
                    };
                }
                MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH => {
                    self.events.push(Event::Reached(self.current as u16));
                    self.current = self.mission.len();
                    self.enter_rtl();
                    return self.rtl_guide();
                }
                // DO_* and other non-navigation items: nothing to fly.
                _ => {
                    self.advance();
                    continue;
                }
            }
        }
        self.enter_hold();
        self.hold_guide()
    }

    fn rtl_guide(&mut self) -> Guide {
        if self.landed {
            return Guide::Ground;
        }
        let home = (0.0, 0.0);
        let alt = self.rtl_target_alt;
        if self.fw() {
            return Guide::Loiter { at: home, alt };
        }
        if self.descending || dist(self.pos(), home) < 1.0 {
            self.descending = true;
            return Guide::Descend { at: home };
        }
        if self.h < alt - 1.0 && dist(self.pos(), self.rtl_from) < 5.0 {
            return Guide::Goto {
                to: self.rtl_from,
                alt,
                from: self.rtl_from,
            };
        }
        Guide::Goto {
            to: home,
            alt,
            from: self.rtl_from,
        }
    }

    fn guide(&mut self, dt: f64) -> Guide {
        match self.mode {
            Mode::Hold | Mode::Other(_) => self.hold_guide(),
            Mode::Takeoff => {
                let tol = if self.fw() { 2.0 } else { 0.3 };
                if !self.landed && self.h >= self.takeoff_alt - tol {
                    self.enter_hold();
                    return self.hold_guide();
                }
                Guide::Takeoff {
                    alt: self.takeoff_alt,
                    from: self.takeoff_from,
                    to: self.takeoff_to,
                }
            }
            Mode::Mission => self.mission_guide(dt),
            Mode::Rtl => self.rtl_guide(),
            Mode::Land => {
                if self.landed {
                    Guide::Ground
                } else {
                    Guide::Descend {
                        at: (self.hold.0, self.hold.1),
                    }
                }
            }
        }
    }

    // ---- dynamics -------------------------------------------------------

    /// Advance by `dt` sim seconds.
    pub fn step(&mut self, dt: f64) {
        self.time_s += dt;
        self.battery_pct = (self.battery_pct - dt * if self.armed { 0.05 } else { 0.002 }).max(0.0);
        if !self.armed {
            // Disarmed: on the ground (or falling to it, if force-disarmed).
            self.h = (self.h - 5.0 * dt).max(0.0);
            self.vn = 0.0;
            self.ve = 0.0;
            self.vu = if self.h > 0.0 { -5.0 } else { 0.0 };
            self.airspeed = 0.0;
            self.roll = lag(self.roll, 0.0, 0.2, dt);
            self.pitch = lag(self.pitch, 0.0, 0.2, dt);
            self.landed = self.h <= 0.0;
            return;
        }
        let g = self.guide(dt);
        let was_landed = self.landed;
        if self.fw() {
            self.fw_step(g, dt);
        } else {
            self.mc_step(g, dt);
        }
        if was_landed && !self.landed {
            self.text("Takeoff detected");
        }
        if !was_landed && self.landed {
            self.text("Landing detected");
        }
        // Landing disarms (PX4 COM_DISARM_LAND).
        if self.landed && matches!(g, Guide::Descend { .. } | Guide::Ground) && self.airspeed < 0.5
        {
            self.land_timer += dt;
            if self.land_timer >= 1.0 {
                self.on_landed();
            }
        } else {
            self.land_timer = 0.0;
        }
    }

    fn on_landed(&mut self) {
        // Only auto-disarm after a flight that ended on the ground.
        let after_land = matches!(self.mode, Mode::Land | Mode::Rtl)
            || (self.mode == Mode::Mission
                && self
                    .mission
                    .get(self.current)
                    .is_some_and(|it| it.cmd == MavCmd::MAV_CMD_NAV_LAND));
        if !after_land {
            return;
        }
        if self.mode == Mode::Mission {
            self.events.push(Event::Reached(self.current as u16));
            self.current = self.mission.len();
            self.text("Mission finished");
        }
        self.disarm();
        self.descending = false;
    }

    fn mc_step(&mut self, g: Guide, dt: f64) {
        let here = self.pos();
        // Horizontal velocity toward a point, braking to stop on it.
        let toward = |to: (f64, f64), vmax: f64| -> (f64, f64) {
            let d = dist(here, to);
            if d < 0.01 {
                return (0.0, 0.0);
            }
            let speed = vmax.min((2.0 * 2.0 * d).sqrt()).min(1.5 * d);
            ((to.0 - here.0) / d * speed, (to.1 - here.1) / d * speed)
        };
        let alt_rate = |alt: f64, h: f64| (1.0 * (alt - h)).clamp(-MC_MAX_V_SPEED, MC_MAX_V_SPEED);
        let (sn, se, su) = match g {
            Guide::Ground => (0.0, 0.0, 0.0),
            Guide::Takeoff { alt, from, .. } => {
                let (a, b) = toward(from, 1.0);
                (a, b, alt_rate(alt, self.h).max(0.3))
            }
            Guide::Goto { to, alt, .. } => {
                let (a, b) = toward(to, MC_MAX_H_SPEED);
                (a, b, alt_rate(alt, self.h))
            }
            Guide::Loiter { at, alt } => {
                let (a, b) = toward(at, MC_MAX_H_SPEED);
                (a, b, alt_rate(alt, self.h))
            }
            Guide::Descend { at } => {
                let (a, b) = toward(at, 2.0);
                (a, b, if self.h > 5.0 { -2.0 } else { -0.8 })
            }
        };
        if g == Guide::Ground && self.landed {
            self.vn = 0.0;
            self.ve = 0.0;
            self.vu = 0.0;
            self.roll = lag(self.roll, 0.0, 0.2, dt);
            self.pitch = lag(self.pitch, 0.0, 0.2, dt);
            self.airspeed = 0.0;
            return;
        }
        // First-order velocity response with an acceleration limit.
        let (old_n, old_e) = (self.vn, self.ve);
        let mut dn = lag(self.vn, sn, 0.5, dt) - self.vn;
        let mut de = lag(self.ve, se, 0.5, dt) - self.ve;
        let dv = dn.hypot(de);
        if dv > MC_MAX_ACCEL * dt {
            dn *= MC_MAX_ACCEL * dt / dv;
            de *= MC_MAX_ACCEL * dt / dv;
        }
        self.vn += dn;
        self.ve += de;
        self.vu = lag(self.vu, su, 0.3, dt);
        // Attitude follows the acceleration it takes to get there.
        let (an, ae) = ((self.vn - old_n) / dt, (self.ve - old_e) / dt);
        let (sy, cy) = self.yaw.sin_cos();
        let fwd = an * cy + ae * sy;
        let right = -an * sy + ae * cy;
        self.pitch = lag(self.pitch, -(fwd / G).atan(), 0.15, dt);
        self.roll = lag(self.roll, (right / G).atan(), 0.15, dt);
        let speed = self.vn.hypot(self.ve);
        if speed > 1.0 {
            let err = wrap_pi(self.ve.atan2(self.vn) - self.yaw);
            self.yaw = wrap_pi(self.yaw + err.clamp(-FRAC_PI_2 * dt, FRAC_PI_2 * dt));
        }
        self.airspeed = speed;
        self.n += self.vn * dt;
        self.e += self.ve * dt;
        self.h += self.vu * dt;
        if self.h <= 0.0 {
            self.h = 0.0;
            self.vu = self.vu.max(0.0);
            if !self.landed && matches!(g, Guide::Descend { .. }) {
                self.landed = true;
                self.vn = 0.0;
                self.ve = 0.0;
                self.airspeed = 0.0;
            }
        } else if self.h > 0.1 {
            self.landed = false;
        }
    }

    /// Heading that tracks the line `from` -> `to` with a lookahead point.
    fn track_heading(&self, from: (f64, f64), to: (f64, f64), extend: bool) -> f64 {
        let here = self.pos();
        let len = dist(from, to);
        if len < 1.0 || (!extend && dist(here, to) < FW_LOOKAHEAD) {
            return bearing(here, to);
        }
        let u = ((to.0 - from.0) / len, (to.1 - from.1) / len);
        let along = (here.0 - from.0) * u.0 + (here.1 - from.1) * u.1;
        let s = along.max(0.0) + FW_LOOKAHEAD;
        if !extend && s >= len {
            return bearing(here, to);
        }
        bearing(here, (from.0 + u.0 * s, from.1 + u.1 * s))
    }

    /// Heading that converges on a clockwise circle around `c`.
    fn loiter_heading(&self, c: (f64, f64), r: f64) -> f64 {
        let here = self.pos();
        let d = dist(c, here);
        if d < 1.0 {
            return self.yaw;
        }
        let theta = bearing(c, here);
        theta + FRAC_PI_2 + (0.04 * (d - r)).atan()
    }

    fn fw_step(&mut self, g: Guide, dt: f64) {
        if self.landed {
            match g {
                Guide::Takeoff { to, .. } => {
                    // Ground roll toward the takeoff point, lift off at min airspeed.
                    self.yaw = bearing(self.pos(), to);
                    self.airspeed += 3.0 * dt;
                    if self.airspeed >= FW_MIN_AIRSPEED {
                        self.landed = false;
                        self.vu = 1.0;
                    }
                }
                _ => self.airspeed = (self.airspeed - 4.0 * dt).max(0.0),
            }
            if self.landed {
                self.h = 0.0;
                self.vu = 0.0;
                self.roll = lag(self.roll, 0.0, 0.2, dt);
                self.pitch = lag(self.pitch, 0.0, 0.2, dt);
                self.vn = self.airspeed * self.yaw.cos();
                self.ve = self.airspeed * self.yaw.sin();
                self.n += self.vn * dt;
                self.e += self.ve * dt;
                return;
            }
        }
        let (heading, vz_sp) = match g {
            Guide::Takeoff { alt, from, to } => (
                self.track_heading(from, to, true),
                (0.5 * (alt - self.h)).clamp(0.5, FW_MAX_CLIMB),
            ),
            Guide::Goto { to, alt, from } => (
                self.track_heading(from, to, false),
                (0.5 * (alt - self.h)).clamp(-FW_MAX_SINK, FW_MAX_CLIMB),
            ),
            Guide::Loiter { at, alt } => (
                self.loiter_heading(at, FW_LOITER_RADIUS),
                (0.5 * (alt - self.h)).clamp(-FW_MAX_SINK, FW_MAX_CLIMB),
            ),
            Guide::Descend { at } => (self.loiter_heading(at, FW_LOITER_RADIUS), -2.0),
            Guide::Ground => (self.yaw, -1.0),
        };
        self.airspeed = (self.airspeed + (FW_CRUISE - self.airspeed).clamp(-2.0 * dt, 2.0 * dt))
            .max(FW_MIN_AIRSPEED);
        self.vu = lag(self.vu, vz_sp, 1.0, dt);
        // Coordinated turn: bank from the commanded turn rate.
        let err = wrap_pi(heading - self.yaw);
        let rate_cmd = 0.6 * err;
        let roll_cmd = (self.airspeed * rate_cmd / G)
            .atan()
            .clamp(-FW_MAX_BANK, FW_MAX_BANK);
        self.roll = lag(self.roll, roll_cmd, 0.4, dt);
        self.yaw = wrap_pi(self.yaw + G * self.roll.tan() / self.airspeed * dt);
        let gamma = (self.vu / self.airspeed).clamp(-0.5, 0.5).asin();
        self.pitch = gamma + 0.05;
        let hs = self.airspeed * gamma.cos();
        self.vn = hs * self.yaw.cos();
        self.ve = hs * self.yaw.sin();
        self.n += self.vn * dt;
        self.e += self.ve * dt;
        self.h += self.vu * dt;
        if self.h <= 0.0 {
            // Touchdown; roll out on the ground.
            self.h = 0.0;
            self.vu = 0.0;
            self.landed = true;
            self.roll = 0.0;
            self.pitch = 0.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flight_core::{Mission, MissionItem};

    fn nav(v: &Vehicle, it: &MissionItem) -> NavItem {
        match *it {
            MissionItem::Takeoff { lat, lon, alt_m } => NavItem {
                cmd: MavCmd::MAV_CMD_NAV_TAKEOFF,
                pos: Some(v.local(lat, lon)),
                alt: alt_m,
                acceptance: 0.0,
                param1: 0.0,
            },
            MissionItem::Waypoint {
                lat,
                lon,
                alt_m,
                acceptance_m,
            } => NavItem {
                cmd: MavCmd::MAV_CMD_NAV_WAYPOINT,
                pos: Some(v.local(lat, lon)),
                alt: alt_m,
                acceptance: acceptance_m,
                param1: 0.0,
            },
            MissionItem::Land { lat, lon } => NavItem {
                cmd: MavCmd::MAV_CMD_NAV_LAND,
                pos: Some(v.local(lat, lon)),
                alt: 0.0,
                acceptance: 0.0,
                param1: 0.0,
            },
            MissionItem::ReturnToLaunch => NavItem {
                cmd: MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
                pos: None,
                alt: f64::NAN,
                acceptance: 0.0,
                param1: 0.0,
            },
        }
    }

    /// Fly the box mission offline; returns the sim time it took.
    fn fly(af: Airframe) -> (Vehicle, f64) {
        let home = GeoPoint::PX4_DEFAULT_HOME;
        let m = Mission::builtin("box", af, home).unwrap();
        let mut v = Vehicle::new(af, home);
        let items = m.items.iter().map(|it| nav(&v, it)).collect();
        v.set_mission(items);
        v.start_mission(None).unwrap();
        assert!(v.armed);
        let mut max_alt: f64 = 0.0;
        while v.time_s < m.timeout_s {
            v.step(DT);
            max_alt = max_alt.max(v.h);
            if v.current >= v.mission.len() {
                break;
            }
        }
        assert!(
            max_alt > 0.9 * m.items[0].target().unwrap().2,
            "max alt {max_alt}"
        );
        let t = v.time_s;
        (v, t)
    }

    #[test]
    fn quad_flies_box_and_lands() {
        let (mut v, t) = fly(Airframe::Quadrotor);
        assert!(t < 120.0, "took {t}");
        for _ in 0..200 {
            v.step(DT);
        }
        assert!(!v.armed && v.landed);
        assert!(v.n.hypot(v.e) < 2.0, "landed at {} {}", v.n, v.e);
    }

    #[test]
    fn plane_flies_box_and_loiters_home() {
        let (mut v, t) = fly(Airframe::FixedWing);
        assert!(t < 200.0, "took {t}");
        assert_eq!(v.mode, Mode::Rtl);
        let mut max_r: f64 = 0.0;
        for _ in 0..(120.0 / DT) as usize {
            v.step(DT);
            max_r = max_r.max(v.n.hypot(v.e));
            assert!(v.airspeed >= FW_MIN_AIRSPEED - 1e-9);
            assert!(v.roll.abs() <= FW_MAX_BANK + 1e-6);
        }
        assert!(max_r < 150.0, "loiter radius {max_r}");
        assert!((v.h - 60.0).abs() < 5.0, "alt {}", v.h);
    }

    #[test]
    fn mode_encoding() {
        assert_eq!(Mode::Mission.custom_mode(), 0x0404_0000);
        assert_eq!(Mode::Rtl.custom_mode(), 0x0504_0000);
        assert_eq!(Mode::Hold.custom_mode(), 0x0304_0000);
    }
}
