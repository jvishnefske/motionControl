# flight

Real-time flight-control sim tying together **PX4**, **Gazebo**, **QGroundControl** and
**~/src/middleware**, with a web menu for one-click mission tests. Each component can
be switched at runtime between the real program and a Rust stand-in behind the same
trait (`flight_core::Module`).

```
nix develop            # rust, qgroundcontrol, docker client, just
just run               # = cargo run -p flightsim -- ; open http://127.0.0.1:8080/
flightsim --preset all-sim --run box          # headless one-click test
flightsim --airframe fixed_wing --preset px4-sih
nix run                # build + run flightsim through nix
```

## Modules

| kind | Real | Simulated (Rust) |
|---|---|---|
| Autopilot | PX4 SITL v1.18 (`flight-launch::Px4Sitl`) | `flight-sim::SimAutopilot`: MAVLink vehicle on PX4's ports with quad/fixed-wing dynamics |
| Physics | `gz sim` Harmonic (`GazeboSim`); visualizer that follows telemetry when the autopilot is simulated | PX4 SIH (`sihsim_*`) when PX4 is real, else the dynamics inside `SimAutopilot` |
| Ground station | QGroundControl (udp 14550) | `SimGroundStation`: GCS heartbeat |
| Middleware | `~/src/middleware` over Zenoh (`tcp/0.0.0.0:7447`) | `~/src/middleware` in-memory bus |

Telemetry and test reports go out on the bus as `flight/<airframe>/telemetry` and
`flight/<airframe>/mission/report`; `flight/<airframe>/cmd` answers `"status"`.

Presets: `all-sim`, `px4-sih` (real PX4 with SIH physics), `full-stack` (everything real),
`sim+qgc` (Rust autopilot with real QGC).

Airframes: X500 quadrotor (`gz_x500` / `sihsim_quadx`) and RC Cessna fixed-wing
(`gz_rc_cessna` / `sihsim_airplane`). Missions: `box`, `survey`.

## Mission test

`Run mission test` starts whatever autopilot and physics are needed and connects
over MAVLink, the same way for PX4 and the Rust autopilot. It then uploads the
mission, arms, switches to AUTO.MISSION and judges telemetry in real time with
`flight_core::MissionTest`. The checks are: armed, takeoff altitude, each waypoint
within its acceptance radius and altitude tolerance, and landed and disarmed (quad)
or back home (fixed-wing). A test fails on timeout or if the vehicle disarms in the air.

## Real PX4 / Gazebo

PX4 and gz Harmonic are not in nixpkgs, so there are two ways to run them:
- `just px4-image` (or `nix run .#build-px4-image`) builds `flight-px4-gz:latest`
  from `docker/px4-gz.Dockerfile`. That's about 10 GB, and it runs with `--network host`.
- `FLIGHT_PX4_DIR=/path/to/PX4-Autopilot`, a native checkout built with `make px4_sitl`.

Other environment variables: `FLIGHT_PX4_IMAGE`, `FLIGHT_GZ_WORLD`, `FLIGHT_QGC`.

QGroundControl from nixpkgs can't load the host's GL drivers on a non-NixOS system,
so it runs with Qt's software renderer. Headless runs (no `--gui`) go under `xvfb-run`;
the Nix package and the dev shell both put QGC and `xvfb-run` on PATH.

## Network exposure

Everything is local unless you opt in:
- The web menu accepts only loopback `Host` headers. A POST has to be JSON, and its
  `Origin` has to match the page's own.
- The MAVLink API socket binds `127.0.0.1` (`FLIGHT_MAV_BIND=0.0.0.0` opens it to the network).
- Zenoh listens on `tcp/127.0.0.1:7447` with scouting off. `FLIGHT_ZENOH_LISTEN`,
  `FLIGHT_ZENOH_CONNECT` and `FLIGHT_ZENOH_SCOUT=1` change that.
The menu greys out a Real backend that can't run on this machine and gives the reason.

## Crates

`flight-core` (traits, types, missions, evaluator) · `flight-mav` (MAVLink client and
mission runner) · `flight-sim` (Rust stand-ins) · `flight-launch` (PX4, gz and QGC
processes) · `flight-bus` (middleware) · `flight-ui` (orchestrator and web menu) ·
`flightsim` (binary).

The middleware is a path dependency, `../../middleware`. Nix takes it from the flake
input `middleware` (the committed HEAD of `/home/j/src/middleware`); to use a different
copy, pass `--override-input middleware <path-or-url>`.
