//! Pure construction of environments and argument lists. Nothing here
//! touches the filesystem or starts a process, so all of it is unit tested.
//!
//! The variable names follow PX4-Autopilot v1.18.0:
//!
//! * `ROMFS/px4fmu_common/init.d-posix/rcS`: `PX4_SYS_AUTOSTART` selects the
//!   airframe (`SYS_AUTOSTART`) and wins over `PX4_SIM_MODEL`.
//! * `.../airframes/4001_gz_x500`, `4003_gz_rc_cessna`: set
//!   `PX4_SIMULATOR=gz`, `PX4_GZ_WORLD=default` and `SIM_GZ_EN=1` as defaults.
//! * `.../airframes/10040_sihsim_quadx`, `10041_sihsim_airplane`: set
//!   `PX4_SIMULATOR=sihsim`, which `px4-rc.simulator` turns into
//!   `px4-rc.sihsim`; that copies `PX4_HOME_LAT/LON/ALT` into
//!   `SIH_LOC_LAT0/LON0/H0`.
//! * `.../px4-rc.gzsim`: with `PX4_GZ_STANDALONE` set it does not start
//!   Gazebo but waits (30 s) for `/world/$PX4_GZ_WORLD/scene/info`, sets the
//!   world origin from `PX4_HOME_LAT/LON/ALT` (all three or none) with
//!   `set_spherical_coordinates`, and spawns
//!   `file://$PX4_GZ_MODELS/${PX4_SIM_MODEL#*gz_}/model.sdf` as
//!   `<model>_<instance>` at `PX4_GZ_MODEL_POSE` ("x,y,z,roll,pitch,yaw").
//!   In standalone mode it does NOT source `gz_env.sh`, so `PX4_GZ_MODELS`
//!   must come from the caller.
//! * `src/modules/simulation/gz_bridge/gz_env.sh.in`: `PX4_GZ_MODELS`,
//!   `PX4_GZ_WORLDS`, `PX4_GZ_PLUGINS`, `PX4_GZ_SERVER_CONFIG`,
//!   `GZ_SIM_RESOURCE_PATH`, `GZ_SIM_SYSTEM_PLUGIN_PATH`,
//!   `GZ_SIM_SERVER_CONFIG_PATH`.
//! * `src/modules/simulation/gz_bridge/CMakeLists.txt`: `make px4_sitl
//!   gz_x500` runs `bin/px4` in `build/px4_sitl_default/rootfs` with
//!   `GZ_IP=127.0.0.1`; we set the same `GZ_IP` on both sides.
//! * `.../px4-rc.mavlink`: instance 0 sends the GCS link from udp 18570 to
//!   14550 and the onboard/API link from 14580 to 14540.

use std::path::{Path, PathBuf};

use flight_core::{Airframe, Backend, GeoPoint, Telemetry};

/// Gazebo world PX4 uses by default (`PX4_GZ_WORLD:=default` in the gz airframes).
pub const DEFAULT_WORLD: &str = "default";
/// Where docker/px4-gz.Dockerfile puts the PX4 checkout.
pub const CONTAINER_PX4_DIR: &str = "/opt/PX4-Autopilot";
/// Default image tag built by `nix run .#build-px4-image` / `just px4-image`.
pub const DEFAULT_IMAGE: &str = "flight-px4-gz:latest";
/// Container names, fixed so `stop()` can `docker rm -f` them.
pub const PX4_CONTAINER: &str = "flight-px4";
pub const GZ_CONTAINER: &str = "flight-gz";
pub const GZ_GUI_CONTAINER: &str = "flight-gz-gui";
/// gz-transport discovery address used by PX4's own make targets.
pub const GZ_IP: &str = "127.0.0.1";
/// Name of the model the visualizer spawns when PX4 is not real.
pub fn viz_model_name(airframe: Airframe) -> String {
    format!("flight_viz_{}", airframe.gz_model())
}

fn kv(k: &str, v: impl ToString) -> (String, String) {
    (k.to_string(), v.to_string())
}

/// The PX4 SITL environment for instance 0.
///
/// `physics == Backend::Real` means Gazebo is running (started by the
/// Gazebo module): PX4 runs in standalone mode and spawns its model into
/// the running world. Anything else uses PX4's SIH physics.
pub fn px4_env(airframe: Airframe, physics: Backend, home: GeoPoint) -> Vec<(String, String)> {
    let mut env = Vec::new();
    if physics == Backend::Real {
        env.push(kv("PX4_SYS_AUTOSTART", airframe.px4_autostart_gz()));
        env.push(kv("PX4_SIM_MODEL", airframe.px4_gz_model()));
        env.push(kv("PX4_GZ_STANDALONE", 1));
        env.push(kv("PX4_GZ_WORLD", DEFAULT_WORLD));
        env.push(kv("GZ_IP", GZ_IP));
    } else {
        env.push(kv("PX4_SYS_AUTOSTART", airframe.px4_autostart_sih()));
        env.push(kv("PX4_SIM_MODEL", airframe.px4_sih_model()));
    }
    env.push(kv("PX4_HOME_LAT", home.lat));
    env.push(kv("PX4_HOME_LON", home.lon));
    env.push(kv("PX4_HOME_ALT", home.alt_m));
    env
}

/// `build/px4_sitl_default` of a PX4 checkout.
pub fn px4_build_dir(px4_dir: &Path) -> PathBuf {
    px4_dir.join("build/px4_sitl_default")
}

/// What `gz_env.sh` (generated from gz_env.sh.in) exports, for a native
/// checkout. `existing_resource_path` / `existing_plugin_path` are the
/// caller's current `GZ_SIM_RESOURCE_PATH` / `GZ_SIM_SYSTEM_PLUGIN_PATH`.
pub fn gz_env(
    px4_dir: &Path,
    existing_resource_path: Option<&str>,
    existing_plugin_path: Option<&str>,
) -> Vec<(String, String)> {
    let models = px4_dir.join("Tools/simulation/gz/models");
    let worlds = px4_dir.join("Tools/simulation/gz/worlds");
    let plugins = px4_build_dir(px4_dir).join("src/modules/simulation/gz_plugins");
    let server_config = px4_dir.join("src/modules/simulation/gz_bridge/server.config");
    let join = |existing: Option<&str>, extra: &[&Path]| {
        let mut parts: Vec<String> = existing
            .filter(|s| !s.is_empty())
            .map(str::to_string)
            .into_iter()
            .collect();
        parts.extend(extra.iter().map(|p| p.display().to_string()));
        parts.join(":")
    };
    vec![
        kv("PX4_GZ_MODELS", models.display()),
        kv("PX4_GZ_WORLDS", worlds.display()),
        kv("PX4_GZ_PLUGINS", plugins.display()),
        kv("PX4_GZ_SERVER_CONFIG", server_config.display()),
        kv(
            "GZ_SIM_RESOURCE_PATH",
            join(existing_resource_path, &[&models, &worlds]),
        ),
        kv(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            join(existing_plugin_path, &[&plugins]),
        ),
        kv("GZ_SIM_SERVER_CONFIG_PATH", server_config.display()),
        kv("GZ_IP", GZ_IP),
    ]
}

/// Arguments of the native PX4 binary: daemon mode (no pxh shell on stdin),
/// working dir `rootfs`, startup files from `etc` (px4 main.cpp symlinks
/// `rootfs/etc` to it). Instance 0, so the default MAVLink ports.
pub fn px4_native_args(px4_dir: &Path) -> Vec<String> {
    let build = px4_build_dir(px4_dir);
    vec![
        "-d".into(),
        "-w".into(),
        build.join("rootfs").display().to_string(),
        build.join("etc").display().to_string(),
    ]
}

/// The world SDF inside a PX4 checkout.
pub fn world_sdf(px4_dir: &Path, world: &str) -> PathBuf {
    px4_dir
        .join("Tools/simulation/gz/worlds")
        .join(format!("{world}.sdf"))
}

/// `gz sim` server arguments: run immediately, server only.
pub fn gz_server_args(world_sdf: &str) -> Vec<String> {
    vec![
        "sim".into(),
        "--verbose=1".into(),
        "-r".into(),
        "-s".into(),
        world_sdf.into(),
    ]
}

/// `gz sim` GUI client arguments.
pub fn gz_gui_args() -> Vec<String> {
    vec!["sim".into(), "-g".into()]
}

/// Options of a `docker run`.
#[derive(Debug, Clone, Default)]
pub struct DockerRun<'a> {
    pub name: &'a str,
    pub image: &'a str,
    pub env: &'a [(String, String)],
    /// Forward the X11/Wayland display and the GPU.
    pub gui: bool,
    /// `/dev/dri` exists on the host (only used with `gui`).
    pub dri: bool,
    /// Host environment values forwarded when `gui`: DISPLAY, WAYLAND_DISPLAY, XDG_RUNTIME_DIR.
    pub display: Option<&'a str>,
    pub wayland_display: Option<&'a str>,
    pub xdg_runtime_dir: Option<&'a str>,
    pub cmd: &'a [String],
}

/// `docker run` arguments: foreground (logs stream to us), removed on
/// exit, host network (MAVLink UDP and gz-transport discovery as if native).
pub fn docker_run_args(o: &DockerRun<'_>) -> Vec<String> {
    let mut a: Vec<String> = vec![
        "run".into(),
        "--rm".into(),
        "--init".into(),
        "--name".into(),
        o.name.into(),
        "--network".into(),
        "host".into(),
    ];
    for (k, v) in o.env {
        a.push("-e".into());
        a.push(format!("{k}={v}"));
    }
    if o.gui {
        a.push("--ipc".into());
        a.push("host".into());
        a.push("-e".into());
        a.push("QT_X11_NO_MITSHM=1".into());
        if let Some(d) = o.display {
            a.push("-e".into());
            a.push(format!("DISPLAY={d}"));
            a.push("-v".into());
            a.push("/tmp/.X11-unix:/tmp/.X11-unix:ro".into());
        }
        if let (Some(w), Some(x)) = (o.wayland_display, o.xdg_runtime_dir) {
            a.push("-e".into());
            a.push(format!("WAYLAND_DISPLAY={w}"));
            a.push("-e".into());
            a.push(format!("XDG_RUNTIME_DIR={x}"));
            a.push("-v".into());
            a.push(format!("{x}/{w}:{x}/{w}"));
        }
        if o.dri {
            a.push("--device".into());
            a.push("/dev/dri".into());
        }
    }
    a.push(o.image.into());
    a.extend(o.cmd.iter().cloned());
    a
}

/// ZYX Euler angles (roll, pitch, yaw) to a quaternion (w, x, y, z).
pub fn euler_to_quat(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sr, cr) = (roll / 2.0).sin_cos();
    let (sp, cp) = (pitch / 2.0).sin_cos();
    let (sy, cy) = (yaw / 2.0).sin_cos();
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}

/// Telemetry (NED position from `origin`, FRD attitude in NED) to a Gazebo
/// world pose: ENU position in metres and the FLU-in-ENU orientation
/// (roll, -pitch, pi/2 - yaw) as (w, x, y, z).
pub fn telemetry_to_enu_pose(origin: &GeoPoint, t: &Telemetry) -> ([f64; 3], [f64; 4]) {
    let (north, east) = origin.ne_to(&t.position);
    let up = t.position.alt_m - origin.alt_m;
    let [roll, pitch, yaw] = t.attitude;
    let q = euler_to_quat(roll, -pitch, std::f64::consts::FRAC_PI_2 - yaw);
    ([east, north, up], q)
}

/// `gz.msgs.Pose` request text for `/world/<w>/set_pose`.
pub fn set_pose_req(model: &str, pos: [f64; 3], q: [f64; 4]) -> String {
    format!(
        "name: \"{model}\", position: {{x: {:.3}, y: {:.3}, z: {:.3}}}, orientation: {{w: {:.6}, x: {:.6}, y: {:.6}, z: {:.6}}}",
        pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3]
    )
}

/// `gz service` arguments with a request.
pub fn gz_service_args(
    service: &str,
    reqtype: &str,
    reptype: &str,
    timeout_ms: u32,
    req: &str,
) -> Vec<String> {
    vec![
        "service".into(),
        "-s".into(),
        service.into(),
        "--reqtype".into(),
        reqtype.into(),
        "--reptype".into(),
        reptype.into(),
        "--timeout".into(),
        timeout_ms.to_string(),
        "--req".into(),
        req.into(),
    ]
}

/// `/world/<w>/set_pose` call.
pub fn set_pose_args(world: &str, model: &str, pos: [f64; 3], q: [f64; 4]) -> Vec<String> {
    gz_service_args(
        &format!("/world/{world}/set_pose"),
        "gz.msgs.Pose",
        "gz.msgs.Boolean",
        300,
        &set_pose_req(model, pos, q),
    )
}

/// `/world/<w>/create` of a static copy of the airframe's model (it is
/// only moved by `set_pose`; gz-sim 8's Physics system applies
/// `WorldPoseCmd` to static models too).
pub fn spawn_viz_args(world: &str, airframe: Airframe) -> Vec<String> {
    let name = viz_model_name(airframe);
    let sdf = format!(
        "<sdf version=\"1.9\"><include><uri>model://{}</uri><name>{name}</name><static>true</static><pose>0 0 0 0 0 0</pose></include></sdf>",
        airframe.gz_model()
    );
    gz_service_args(
        &format!("/world/{world}/create"),
        "gz.msgs.EntityFactory",
        "gz.msgs.Boolean",
        5000,
        &format!("name: \"{name}\", allow_renaming: false, sdf: '{sdf}'"),
    )
}

/// `/world/<w>/remove` of a model.
pub fn remove_model_args(world: &str, model: &str) -> Vec<String> {
    gz_service_args(
        &format!("/world/{world}/remove"),
        "gz.msgs.Entity",
        "gz.msgs.Boolean",
        2000,
        &format!("name: \"{model}\", type: MODEL"),
    )
}

/// `/world/<w>/set_spherical_coordinates`, the same request px4-rc.gzsim sends.
pub fn set_origin_args(world: &str, home: GeoPoint) -> Vec<String> {
    gz_service_args(
        &format!("/world/{world}/set_spherical_coordinates"),
        "gz.msgs.SphericalCoordinates",
        "gz.msgs.Boolean",
        1000,
        &format!(
            "surface_model: EARTH_WGS84, latitude_deg: {}, longitude_deg: {}, elevation: {}",
            home.lat, home.lon, home.alt_m
        ),
    )
}

/// `gz service -i` of the scene info service: lists providers once the world is up.
pub fn scene_info_args(world: &str) -> Vec<String> {
    vec![
        "service".into(),
        "-i".into(),
        "--service".into(),
        format!("/world/{world}/scene/info"),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn get<'a>(env: &'a [(String, String)], k: &str) -> Option<&'a str> {
        env.iter()
            .find(|(key, _)| key == k)
            .map(|(_, v)| v.as_str())
    }

    #[test]
    fn px4_env_gazebo_standalone() {
        let env = px4_env(
            Airframe::Quadrotor,
            Backend::Real,
            GeoPoint::PX4_DEFAULT_HOME,
        );
        assert_eq!(get(&env, "PX4_SYS_AUTOSTART"), Some("4001"));
        assert_eq!(get(&env, "PX4_SIM_MODEL"), Some("gz_x500"));
        assert_eq!(get(&env, "PX4_GZ_STANDALONE"), Some("1"));
        assert_eq!(get(&env, "PX4_GZ_WORLD"), Some("default"));
        assert_eq!(get(&env, "GZ_IP"), Some("127.0.0.1"));
        assert_eq!(get(&env, "PX4_HOME_LAT"), Some("47.397742"));
        assert_eq!(get(&env, "PX4_HOME_LON"), Some("8.545594"));
        assert_eq!(get(&env, "PX4_HOME_ALT"), Some("488"));

        let fw = px4_env(
            Airframe::FixedWing,
            Backend::Real,
            GeoPoint::PX4_DEFAULT_HOME,
        );
        assert_eq!(get(&fw, "PX4_SYS_AUTOSTART"), Some("4003"));
        assert_eq!(get(&fw, "PX4_SIM_MODEL"), Some("gz_rc_cessna"));
    }

    #[test]
    fn px4_env_sih_without_gazebo() {
        for physics in [Backend::Simulated, Backend::Off] {
            let env = px4_env(
                Airframe::FixedWing,
                physics,
                GeoPoint {
                    lat: 1.5,
                    lon: -2.25,
                    alt_m: 10.0,
                },
            );
            assert_eq!(get(&env, "PX4_SYS_AUTOSTART"), Some("10041"));
            assert_eq!(get(&env, "PX4_SIM_MODEL"), Some("sihsim_airplane"));
            assert_eq!(get(&env, "PX4_GZ_STANDALONE"), None);
            assert_eq!(get(&env, "PX4_HOME_LAT"), Some("1.5"));
            assert_eq!(get(&env, "PX4_HOME_LON"), Some("-2.25"));
            assert_eq!(get(&env, "PX4_HOME_ALT"), Some("10"));
        }
        let q = px4_env(
            Airframe::Quadrotor,
            Backend::Simulated,
            GeoPoint::PX4_DEFAULT_HOME,
        );
        assert_eq!(get(&q, "PX4_SYS_AUTOSTART"), Some("10040"));
        assert_eq!(get(&q, "PX4_SIM_MODEL"), Some("sihsim_quadx"));
    }

    #[test]
    fn gz_env_matches_gz_env_sh() {
        let env = gz_env(Path::new("/px4"), Some("/mine"), None);
        assert_eq!(
            get(&env, "PX4_GZ_MODELS"),
            Some("/px4/Tools/simulation/gz/models")
        );
        assert_eq!(
            get(&env, "PX4_GZ_WORLDS"),
            Some("/px4/Tools/simulation/gz/worlds")
        );
        assert_eq!(
            get(&env, "GZ_SIM_RESOURCE_PATH"),
            Some("/mine:/px4/Tools/simulation/gz/models:/px4/Tools/simulation/gz/worlds")
        );
        assert_eq!(
            get(&env, "GZ_SIM_SYSTEM_PLUGIN_PATH"),
            Some("/px4/build/px4_sitl_default/src/modules/simulation/gz_plugins")
        );
        assert_eq!(
            get(&env, "GZ_SIM_SERVER_CONFIG_PATH"),
            Some("/px4/src/modules/simulation/gz_bridge/server.config")
        );
    }

    #[test]
    fn native_px4_args() {
        assert_eq!(
            px4_native_args(Path::new("/px4")),
            [
                "-d",
                "-w",
                "/px4/build/px4_sitl_default/rootfs",
                "/px4/build/px4_sitl_default/etc"
            ]
        );
    }

    #[test]
    fn gz_args() {
        assert_eq!(
            gz_server_args("/w/default.sdf"),
            ["sim", "--verbose=1", "-r", "-s", "/w/default.sdf"]
        );
        assert_eq!(gz_gui_args(), ["sim", "-g"]);
        assert_eq!(
            world_sdf(Path::new("/px4"), "default"),
            Path::new("/px4/Tools/simulation/gz/worlds/default.sdf")
        );
    }

    #[test]
    fn docker_args_headless_and_gui() {
        let env = vec![("A".to_string(), "1".to_string())];
        let cmd = vec!["px4".to_string()];
        let a = docker_run_args(&DockerRun {
            name: "n",
            image: "img",
            env: &env,
            cmd: &cmd,
            ..Default::default()
        });
        assert_eq!(
            a,
            [
                "run",
                "--rm",
                "--init",
                "--name",
                "n",
                "--network",
                "host",
                "-e",
                "A=1",
                "img",
                "px4"
            ]
        );
        let g = docker_run_args(&DockerRun {
            name: "n",
            image: "img",
            env: &env,
            cmd: &cmd,
            gui: true,
            dri: true,
            display: Some(":0"),
            ..Default::default()
        });
        assert!(g.windows(2).any(|w| w == ["-e", "DISPLAY=:0"]));
        assert!(g.windows(2).any(|w| w == ["--device", "/dev/dri"]));
        assert_eq!(g.last().map(String::as_str), Some("px4"));
    }

    fn telem(position: GeoPoint, attitude: [f64; 3]) -> Telemetry {
        Telemetry {
            time_s: 0.0,
            position,
            rel_alt_m: 0.0,
            vel_ned: [0.0; 3],
            attitude,
            armed: false,
            mode: flight_core::FlightMode::Unknown,
            landed: true,
            mission_seq: 0,
            mission_reached: None,
            battery_pct: 100.0,
            home: None,
        }
    }

    #[test]
    fn enu_pose_from_ned() {
        let home = GeoPoint::PX4_DEFAULT_HOME;
        let mut p = home.offset(10.0, -5.0);
        p.alt_m += 20.0;
        // Facing north in NED (yaw 0) is +90 deg about z in ENU.
        let (pos, q) = telemetry_to_enu_pose(&home, &telem(p, [0.0, 0.0, 0.0]));
        assert!(
            (pos[0] + 5.0).abs() < 0.01
                && (pos[1] - 10.0).abs() < 0.01
                && (pos[2] - 20.0).abs() < 1e-9
        );
        let h = std::f64::consts::FRAC_1_SQRT_2;
        assert!(
            (q[0] - h).abs() < 1e-9
                && q[1].abs() < 1e-9
                && q[2].abs() < 1e-9
                && (q[3] - h).abs() < 1e-9
        );
        // Facing east in NED (yaw 90 deg) is identity in ENU.
        let (_, q) =
            telemetry_to_enu_pose(&home, &telem(home, [0.0, 0.0, std::f64::consts::FRAC_PI_2]));
        assert!((q[0] - 1.0).abs() < 1e-9 && q[3].abs() < 1e-9);
        // Nose up (NED pitch > 0) is a negative rotation about FLU y.
        let (_, q) =
            telemetry_to_enu_pose(&home, &telem(home, [0.0, 0.2, std::f64::consts::FRAC_PI_2]));
        assert!(q[2] < 0.0);
    }

    #[test]
    fn gz_service_requests() {
        let a = set_pose_args("default", "m", [1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(&a[..3], ["service", "-s", "/world/default/set_pose"]);
        assert_eq!(a[4], "gz.msgs.Pose");
        assert_eq!(
            a.last().unwrap(),
            "name: \"m\", position: {x: 1.000, y: 2.000, z: 3.000}, orientation: {w: 1.000000, x: 0.000000, y: 0.000000, z: 0.000000}"
        );
        let s = spawn_viz_args("default", Airframe::Quadrotor);
        assert_eq!(s[2], "/world/default/create");
        assert!(s.last().unwrap().contains("model://x500</uri>"));
        assert!(s.last().unwrap().contains("<static>true</static>"));
        assert!(s.last().unwrap().starts_with("name: \"flight_viz_x500\""));
        let r = remove_model_args("default", "flight_viz_x500");
        assert_eq!(r.last().unwrap(), "name: \"flight_viz_x500\", type: MODEL");
        let o = set_origin_args(
            "default",
            GeoPoint {
                lat: 1.0,
                lon: 2.0,
                alt_m: 3.0,
            },
        );
        assert!(
            o.last()
                .unwrap()
                .contains("latitude_deg: 1, longitude_deg: 2, elevation: 3")
        );
        assert_eq!(
            scene_info_args("w"),
            ["service", "-i", "--service", "/world/w/scene/info"]
        );
    }
}
