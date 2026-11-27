//! Fly a mission and judge it with [`flight_core::MissionTest`].

use std::time::Duration;

use flight_core::{Mission, MissionTest, Telemetry, TestReport, TestVerdict, VehicleControl};
use tokio::sync::watch;
use tokio::time::Instant;

/// How long to wait for the vehicle to become ready.
pub const READY_TIMEOUT: Duration = Duration::from_secs(60);
/// Fail if no telemetry arrives for this long (wall clock).
const TELEMETRY_TIMEOUT: Duration = Duration::from_secs(5);

/// Wait for the vehicle, upload, arm, start, then feed telemetry into a
/// [`MissionTest`] until it passes or fails. Every intermediate report is
/// published on `report`.
///
/// The elapsed time given to the evaluator is the vehicle's own clock
/// (`Telemetry::time_s` deltas), so a sim running faster than real time is
/// judged in sim seconds.
pub async fn run_mission_test(
    link: &dyn VehicleControl,
    mission: &Mission,
    report: &watch::Sender<Option<TestReport>>,
) -> TestReport {
    let mut test = MissionTest::new(mission);
    report.send_replace(Some(test.report().clone()));

    let abort = |test: &mut MissionTest, why: String| {
        tracing::warn!("mission test {}: {why}", mission.name);
        test.abort(test.report().elapsed_s, &why);
        report.send_replace(Some(test.report().clone()));
        test.report().clone()
    };

    if let Err(e) = link.wait_ready(READY_TIMEOUT).await {
        return abort(&mut test, format!("{e:#}"));
    }
    if let Err(e) = link.upload_mission(mission).await {
        return abort(&mut test, format!("upload: {e:#}"));
    }
    if let Err(e) = link.arm().await {
        return abort(&mut test, format!("{e:#}"));
    }
    if let Err(e) = link.start_mission().await {
        return abort(&mut test, format!("{e:#}"));
    }

    let mut rx = link.telemetry();
    let mut elapsed = 0.0;
    let mut last_time: Option<f64> = None;
    let mut last_sample = Instant::now();
    // Wall-clock backstop in case the vehicle's clock stalls.
    let wall_limit = Instant::now() + Duration::from_secs_f64(mission.timeout_s * 2.0 + 60.0);
    let mut tick = tokio::time::interval(Duration::from_millis(100));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        // Every telemetry update, and at least 10 Hz.
        tokio::select! {
            r = rx.changed() => if r.is_err() {
                return abort(&mut test, "telemetry channel closed".into());
            },
            _ = tick.tick() => {}
        }
        let sample: Option<Telemetry> = *rx.borrow_and_update();
        let Some(t) = sample else {
            if last_sample.elapsed() > TELEMETRY_TIMEOUT {
                return abort(&mut test, "vehicle lost".into());
            }
            continue;
        };
        match last_time {
            // Only count forward steps of a plausible size; a reset clock
            // (vehicle swapped) or a gap does not advance the test.
            Some(prev) if t.time_s > prev && t.time_s - prev < 5.0 => elapsed += t.time_s - prev,
            Some(prev) if t.time_s == prev => {
                if last_sample.elapsed() > TELEMETRY_TIMEOUT {
                    return abort(&mut test, "telemetry stalled".into());
                }
                continue;
            }
            _ => {}
        }
        last_time = Some(t.time_s);
        last_sample = Instant::now();
        let r = test.update(&t, elapsed).clone();
        report.send_replace(Some(r.clone()));
        if r.verdict != TestVerdict::Running {
            return r;
        }
        if Instant::now() > wall_limit {
            return abort(&mut test, "wall-clock limit exceeded".into());
        }
    }
}
