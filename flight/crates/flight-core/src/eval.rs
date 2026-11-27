//! Pure mission-test evaluator: feed it telemetry, read the verdict.
//!
//! Checks, in order:
//! 1. `armed` — the vehicle armed.
//! 2. `takeoff` — reached 90 % of the takeoff altitude.
//! 3. `wp N` — each waypoint passed within its acceptance radius (horizontal)
//!    and the altitude tolerance, in mission order.
//! 4. `complete` — multicopter: landed and disarmed within 5 m of the land
//!    point; fixed-wing (RTL): back within 150 m of home.
//!
//! The test fails on timeout (`Mission::timeout_s`) or when the vehicle
//! disarms in the air before completing.

use serde::{Deserialize, Serialize};

use crate::mission::{Mission, MissionItem};
use crate::types::{GeoPoint, Telemetry};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CheckState {
    Pending,
    Passed,
    Failed,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Check {
    pub name: String,
    pub state: CheckState,
    /// Seconds into the test when it passed or failed.
    pub at_s: Option<f64>,
    pub detail: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TestVerdict {
    Running,
    Passed,
    Failed,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TestReport {
    pub mission: String,
    pub verdict: TestVerdict,
    pub elapsed_s: f64,
    pub checks: Vec<Check>,
    /// Largest altitude error seen while between takeoff and the last waypoint.
    pub max_alt_error_m: f64,
}

enum Goal {
    Armed,
    Altitude(f64),
    Reach { p: GeoPoint, alt: f64, acc: f64 },
    Landed(GeoPoint),
    Home(f64),
}

pub struct MissionTest {
    goals: Vec<Goal>,
    report: TestReport,
    next: usize,
    timeout_s: f64,
    home: GeoPoint,
    alt_tol: f64,
    was_airborne: bool,
}

impl MissionTest {
    pub fn new(m: &Mission) -> Self {
        let alt_tol = if m.airframe.is_fixed_wing() {
            15.0
        } else {
            3.0
        };
        let mut goals = vec![Goal::Armed];
        let mut names = vec!["armed".to_string()];
        let mut wp = 0;
        for it in &m.items {
            match *it {
                MissionItem::Takeoff { alt_m, .. } => {
                    goals.push(Goal::Altitude(alt_m * 0.9));
                    names.push(format!("takeoff to {alt_m:.0} m"));
                }
                MissionItem::Waypoint {
                    lat,
                    lon,
                    alt_m,
                    acceptance_m,
                } => {
                    wp += 1;
                    goals.push(Goal::Reach {
                        p: GeoPoint {
                            lat,
                            lon,
                            alt_m: 0.0,
                        },
                        alt: alt_m,
                        acc: acceptance_m,
                    });
                    names.push(format!("wp {wp}"));
                }
                MissionItem::Land { lat, lon } => {
                    goals.push(Goal::Landed(GeoPoint {
                        lat,
                        lon,
                        alt_m: 0.0,
                    }));
                    names.push("landed + disarmed".into());
                }
                MissionItem::ReturnToLaunch => {
                    goals.push(Goal::Home(150.0));
                    names.push("returned home".into());
                }
            }
        }
        let checks = names
            .into_iter()
            .map(|name| Check {
                name,
                state: CheckState::Pending,
                at_s: None,
                detail: String::new(),
            })
            .collect();
        MissionTest {
            goals,
            report: TestReport {
                mission: m.name.clone(),
                verdict: TestVerdict::Running,
                elapsed_s: 0.0,
                checks,
                max_alt_error_m: 0.0,
            },
            next: 0,
            timeout_s: m.timeout_s,
            home: m.home,
            alt_tol,
            was_airborne: false,
        }
    }

    pub fn report(&self) -> &TestReport {
        &self.report
    }

    /// Mark the test failed from outside (e.g. upload or arm rejected).
    pub fn abort(&mut self, elapsed_s: f64, why: &str) {
        if self.report.verdict != TestVerdict::Running {
            return;
        }
        self.report.elapsed_s = elapsed_s;
        self.fail(elapsed_s, why.to_string());
    }

    fn fail(&mut self, t: f64, detail: String) {
        if let Some(c) = self.report.checks.get_mut(self.next) {
            c.state = CheckState::Failed;
            c.at_s = Some(t);
            c.detail = detail;
        }
        self.report.verdict = TestVerdict::Failed;
    }

    /// Feed one telemetry sample taken `elapsed_s` after the test started.
    pub fn update(&mut self, t: &Telemetry, elapsed_s: f64) -> &TestReport {
        if self.report.verdict != TestVerdict::Running {
            return &self.report;
        }
        self.report.elapsed_s = elapsed_s;
        let home = t.home.unwrap_or(self.home);
        if t.rel_alt_m > 2.0 {
            self.was_airborne = true;
        }
        // Several goals may be met by one sample (e.g. armed + already high).
        while self.next < self.goals.len() {
            let met = match &self.goals[self.next] {
                Goal::Armed => t.armed,
                Goal::Altitude(a) => t.rel_alt_m >= *a,
                Goal::Reach { p, alt, acc } => {
                    let err = (t.rel_alt_m - alt).abs();
                    if err > self.report.max_alt_error_m && self.next > 1 {
                        self.report.max_alt_error_m = err;
                    }
                    t.position.horizontal_distance_m(p) <= *acc && err <= self.alt_tol
                }
                Goal::Landed(p) => {
                    !t.armed && t.rel_alt_m < 1.0 && t.position.horizontal_distance_m(p) <= 5.0
                }
                Goal::Home(r) => t.position.horizontal_distance_m(&home) <= *r,
            };
            if !met {
                break;
            }
            let c = &mut self.report.checks[self.next];
            c.state = CheckState::Passed;
            c.at_s = Some(elapsed_s);
            self.next += 1;
        }
        if self.next == self.goals.len() {
            self.report.verdict = TestVerdict::Passed;
        } else if elapsed_s > self.timeout_s {
            self.fail(elapsed_s, format!("timeout after {:.0} s", self.timeout_s));
        } else if self.was_airborne && !t.armed && t.rel_alt_m > 2.0 {
            self.fail(
                elapsed_s,
                format!("disarmed in the air at {:.1} m", t.rel_alt_m),
            );
        }
        &self.report
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Airframe, FlightMode};

    fn tel(p: GeoPoint, alt: f64, armed: bool) -> Telemetry {
        Telemetry {
            time_s: 0.0,
            position: p,
            rel_alt_m: alt,
            vel_ned: [0.0; 3],
            attitude: [0.0; 3],
            armed,
            mode: FlightMode::Mission,
            landed: alt < 0.5,
            mission_seq: 0,
            mission_reached: None,
            battery_pct: 100.0,
            home: None,
        }
    }

    #[test]
    fn perfect_quad_box_passes() {
        let home = GeoPoint::PX4_DEFAULT_HOME;
        let m = Mission::builtin("box", Airframe::Quadrotor, home).unwrap();
        let mut t = MissionTest::new(&m);
        t.update(&tel(home, 0.0, true), 1.0);
        t.update(&tel(home, 15.0, true), 5.0);
        for it in &m.items[1..] {
            if let Some((lat, lon, alt)) = it.target() {
                let p = GeoPoint {
                    lat,
                    lon,
                    alt_m: 0.0,
                };
                if alt > 0.0 {
                    t.update(&tel(p, alt, true), 10.0);
                } else {
                    t.update(&tel(p, 0.0, false), 60.0);
                }
            }
        }
        assert_eq!(t.report().verdict, TestVerdict::Passed, "{:#?}", t.report());
    }

    #[test]
    fn timeout_and_air_disarm_fail() {
        let home = GeoPoint::PX4_DEFAULT_HOME;
        let m = Mission::builtin("box", Airframe::Quadrotor, home).unwrap();
        let mut t = MissionTest::new(&m);
        t.update(&tel(home, 0.0, true), 1.0);
        assert_eq!(
            t.update(&tel(home, 0.0, true), m.timeout_s + 1.0).verdict,
            TestVerdict::Failed
        );

        let mut t = MissionTest::new(&m);
        t.update(&tel(home, 15.0, true), 1.0);
        let r = t.update(&tel(home, 10.0, false), 2.0);
        assert_eq!(r.verdict, TestVerdict::Failed);
        assert!(
            r.checks
                .iter()
                .any(|c| c.detail.contains("disarmed in the air"))
        );
    }
}
