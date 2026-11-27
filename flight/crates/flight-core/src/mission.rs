use serde::{Deserialize, Serialize};

use crate::types::{Airframe, GeoPoint};

/// One mission item. Altitudes are relative to home (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum MissionItem {
    /// MAV_CMD_NAV_TAKEOFF at the current position (lat/lon given for fixed-wing climb-out direction).
    Takeoff { lat: f64, lon: f64, alt_m: f64 },
    /// MAV_CMD_NAV_WAYPOINT.
    Waypoint {
        lat: f64,
        lon: f64,
        alt_m: f64,
        acceptance_m: f64,
    },
    /// MAV_CMD_NAV_LAND at a position (multicopter).
    Land { lat: f64, lon: f64 },
    /// MAV_CMD_NAV_RETURN_TO_LAUNCH.
    ReturnToLaunch,
}

impl MissionItem {
    pub fn target(&self) -> Option<(f64, f64, f64)> {
        match *self {
            MissionItem::Takeoff { lat, lon, alt_m } => Some((lat, lon, alt_m)),
            MissionItem::Waypoint {
                lat, lon, alt_m, ..
            } => Some((lat, lon, alt_m)),
            MissionItem::Land { lat, lon } => Some((lat, lon, 0.0)),
            MissionItem::ReturnToLaunch => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Mission {
    pub name: String,
    pub airframe: Airframe,
    pub home: GeoPoint,
    pub items: Vec<MissionItem>,
    /// The test fails if the mission takes longer than this (wall clock, real time).
    pub timeout_s: f64,
}

impl Mission {
    /// Names of the built-in missions, for the UI.
    pub const BUILTIN: [&'static str; 2] = ["box", "survey"];

    /// A built-in mission for `airframe` around `home`.
    pub fn builtin(name: &str, airframe: Airframe, home: GeoPoint) -> Option<Mission> {
        let (scale, alt) = match airframe {
            Airframe::Quadrotor => (40.0, 15.0),
            Airframe::FixedWing => (300.0, 60.0),
        };
        let acc = airframe.acceptance_radius_m();
        let wp = |n: f64, e: f64| {
            let p = home.offset(n * scale, e * scale);
            MissionItem::Waypoint {
                lat: p.lat,
                lon: p.lon,
                alt_m: alt,
                acceptance_m: acc,
            }
        };
        let takeoff = {
            // Fixed-wing climbs out north; the takeoff point sets the heading.
            let p = if airframe.is_fixed_wing() {
                home.offset(scale, 0.0)
            } else {
                home
            };
            MissionItem::Takeoff {
                lat: p.lat,
                lon: p.lon,
                alt_m: alt,
            }
        };
        let mut items = vec![takeoff];
        match name {
            "box" => items.extend([wp(1.0, 0.0), wp(1.0, 1.0), wp(0.0, 1.0), wp(0.0, 0.0)]),
            "survey" => {
                for i in 0..4 {
                    let e = i as f64 * 0.5;
                    let (a, b) = if i % 2 == 0 { (0.0, 1.5) } else { (1.5, 0.0) };
                    items.extend([wp(a, e), wp(b, e)]);
                }
            }
            _ => return None,
        }
        match airframe {
            Airframe::Quadrotor => items.push(MissionItem::Land {
                lat: home.lat,
                lon: home.lon,
            }),
            Airframe::FixedWing => items.push(MissionItem::ReturnToLaunch),
        }
        let path_m = path_length_m(&home, &items);
        let timeout_s = 60.0 + 2.0 * path_m / airframe.cruise_speed_ms();
        Some(Mission {
            name: name.to_string(),
            airframe,
            home,
            items,
            timeout_s,
        })
    }
}

fn path_length_m(home: &GeoPoint, items: &[MissionItem]) -> f64 {
    let mut prev = *home;
    let mut total = 0.0;
    for it in items {
        let p = match it.target() {
            Some((lat, lon, _)) => GeoPoint {
                lat,
                lon,
                alt_m: 0.0,
            },
            None => *home,
        };
        total += prev.horizontal_distance_m(&p);
        prev = p;
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builtins_exist_for_every_airframe() {
        for af in Airframe::ALL {
            for name in Mission::BUILTIN {
                let m = Mission::builtin(name, af, GeoPoint::PX4_DEFAULT_HOME).unwrap();
                assert!(matches!(m.items[0], MissionItem::Takeoff { .. }));
                assert!(m.items.len() >= 4);
                assert!(m.timeout_s > 60.0);
            }
        }
        assert!(
            Mission::builtin("nope", Airframe::Quadrotor, GeoPoint::PX4_DEFAULT_HOME).is_none()
        );
    }
}
