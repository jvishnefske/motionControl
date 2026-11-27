use serde::{Deserialize, Serialize};

/// Supported airframes. Each maps to a PX4 Gazebo model, a PX4 SIH model
/// (PX4's built-in physics, used when Gazebo is simulated) and a Rust
/// dynamics model (used when PX4 itself is simulated).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Airframe {
    /// Holybro X500 quadrotor.
    Quadrotor,
    /// RC Cessna fixed-wing.
    FixedWing,
}

impl Airframe {
    pub const ALL: [Airframe; 2] = [Airframe::Quadrotor, Airframe::FixedWing];

    pub fn label(self) -> &'static str {
        match self {
            Airframe::Quadrotor => "X500 quadrotor",
            Airframe::FixedWing => "RC Cessna fixed-wing",
        }
    }

    /// `PX4_SIM_MODEL` / make target suffix with Gazebo (gz) physics.
    pub fn px4_gz_model(self) -> &'static str {
        match self {
            Airframe::Quadrotor => "gz_x500",
            Airframe::FixedWing => "gz_rc_cessna",
        }
    }

    /// The Gazebo model name (without the `gz_` prefix).
    pub fn gz_model(self) -> &'static str {
        match self {
            Airframe::Quadrotor => "x500",
            Airframe::FixedWing => "rc_cessna",
        }
    }

    /// `PX4_SIM_MODEL` for PX4's simulation-in-hardware (no Gazebo).
    pub fn px4_sih_model(self) -> &'static str {
        match self {
            Airframe::Quadrotor => "sihsim_quadx",
            Airframe::FixedWing => "sihsim_airplane",
        }
    }

    /// PX4 `SYS_AUTOSTART` id of the gz airframe.
    pub fn px4_autostart_gz(self) -> u32 {
        match self {
            Airframe::Quadrotor => 4001,
            Airframe::FixedWing => 4003,
        }
    }

    /// PX4 `SYS_AUTOSTART` id of the SIH airframe.
    pub fn px4_autostart_sih(self) -> u32 {
        match self {
            Airframe::Quadrotor => 10040,
            Airframe::FixedWing => 10041,
        }
    }

    pub fn is_fixed_wing(self) -> bool {
        matches!(self, Airframe::FixedWing)
    }

    /// Horizontal acceptance radius for waypoints, metres.
    pub fn acceptance_radius_m(self) -> f64 {
        match self {
            Airframe::Quadrotor => 2.0,
            Airframe::FixedWing => 30.0,
        }
    }

    /// Cruise speed, m/s.
    pub fn cruise_speed_ms(self) -> f64 {
        match self {
            Airframe::Quadrotor => 5.0,
            Airframe::FixedWing => 15.0,
        }
    }
}

/// WGS-84 position; `alt_m` is AMSL.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GeoPoint {
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
}

pub const EARTH_RADIUS_M: f64 = 6_371_000.0;

impl GeoPoint {
    /// PX4's default SITL home (Zurich).
    pub const PX4_DEFAULT_HOME: GeoPoint = GeoPoint {
        lat: 47.397742,
        lon: 8.545594,
        alt_m: 488.0,
    };

    /// The point `north_m`/`east_m` metres away (flat-earth, fine for < 10 km).
    pub fn offset(&self, north_m: f64, east_m: f64) -> GeoPoint {
        let dlat = north_m / EARTH_RADIUS_M;
        let dlon = east_m / (EARTH_RADIUS_M * self.lat.to_radians().cos());
        GeoPoint {
            lat: self.lat + dlat.to_degrees(),
            lon: self.lon + dlon.to_degrees(),
            alt_m: self.alt_m,
        }
    }

    /// (north, east) metres from `self` to `other`.
    pub fn ne_to(&self, other: &GeoPoint) -> (f64, f64) {
        let n = (other.lat - self.lat).to_radians() * EARTH_RADIUS_M;
        let e = (other.lon - self.lon).to_radians() * EARTH_RADIUS_M * self.lat.to_radians().cos();
        (n, e)
    }

    pub fn horizontal_distance_m(&self, other: &GeoPoint) -> f64 {
        let (n, e) = self.ne_to(other);
        n.hypot(e)
    }
}

/// Flight mode as reported by the autopilot (PX4 custom modes, simplified).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FlightMode {
    Manual,
    Stabilized,
    Altitude,
    Position,
    Hold,
    Takeoff,
    Mission,
    Rtl,
    Land,
    Offboard,
    Unknown,
}

/// One telemetry snapshot of the vehicle, the same whether it came from
/// PX4 or the simulated autopilot (both are read over MAVLink).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Telemetry {
    /// Seconds since the vehicle link came up.
    pub time_s: f64,
    pub position: GeoPoint,
    /// Altitude above home, metres.
    pub rel_alt_m: f64,
    /// NED velocity, m/s.
    pub vel_ned: [f64; 3],
    /// Roll, pitch, yaw in radians.
    pub attitude: [f64; 3],
    pub armed: bool,
    pub mode: FlightMode,
    pub landed: bool,
    /// Current mission item index (MISSION_CURRENT).
    pub mission_seq: u16,
    /// Highest item index reported reached (MISSION_ITEM_REACHED), if any.
    pub mission_reached: Option<u16>,
    pub battery_pct: f32,
    pub home: Option<GeoPoint>,
}

impl Telemetry {
    pub fn ground_speed_ms(&self) -> f64 {
        self.vel_ned[0].hypot(self.vel_ned[1])
    }
}

/// UDP port layout, identical to PX4 SITL instance 0 so QGroundControl and
/// the mission runner connect the same way to the real and simulated
/// autopilot:
///
/// * the vehicle sends its GCS stream to `127.0.0.1:gcs` (QGC listens);
/// * the vehicle sends its API stream to `127.0.0.1:api` (the mission
///   runner listens with `udpin:0.0.0.0:api` and replies to the source).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct MavPorts {
    pub gcs: u16,
    pub api: u16,
}

impl Default for MavPorts {
    fn default() -> Self {
        MavPorts {
            gcs: 14550,
            api: 14540,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn offset_round_trips() {
        let h = GeoPoint::PX4_DEFAULT_HOME;
        let p = h.offset(100.0, -50.0);
        let (n, e) = h.ne_to(&p);
        assert!((n - 100.0).abs() < 0.01 && (e + 50.0).abs() < 0.01);
        assert!((h.horizontal_distance_m(&p) - 111.803).abs() < 0.01);
    }
}
