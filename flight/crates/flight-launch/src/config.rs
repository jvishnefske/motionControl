//! Where PX4 and Gazebo come from: a native PX4 checkout or the container.

use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::process::Stdio;

use crate::env::{CONTAINER_PX4_DIR, DEFAULT_IMAGE, DEFAULT_WORLD, px4_build_dir};

/// How the external programs are found. Built from the environment by
/// [`LaunchConfig::from_env`]:
///
/// * `FLIGHT_PX4_DIR`: a PX4-Autopilot checkout with `build/px4_sitl_default`
///   built (`make px4_sitl`). Selects the native runner.
/// * `FLIGHT_PX4_IMAGE`: docker image (default `flight-px4-gz:latest`).
/// * `FLIGHT_GZ_WORLD`: Gazebo world name (default `default`).
/// * `FLIGHT_QGC`: QGroundControl binary or AppImage.
#[derive(Debug, Clone, PartialEq)]
pub struct LaunchConfig {
    pub px4_dir: Option<PathBuf>,
    pub image: String,
    pub world: String,
    pub qgc: Option<PathBuf>,
}

impl Default for LaunchConfig {
    fn default() -> Self {
        LaunchConfig {
            px4_dir: None,
            image: DEFAULT_IMAGE.into(),
            world: DEFAULT_WORLD.into(),
            qgc: None,
        }
    }
}

impl LaunchConfig {
    pub fn from_env() -> Self {
        let var = |k: &str| std::env::var_os(k).filter(|v| !v.is_empty());
        LaunchConfig {
            px4_dir: var("FLIGHT_PX4_DIR").map(PathBuf::from),
            image: var("FLIGHT_PX4_IMAGE")
                .map(|v| v.to_string_lossy().into_owned())
                .unwrap_or_else(|| DEFAULT_IMAGE.into()),
            world: var("FLIGHT_GZ_WORLD")
                .map(|v| v.to_string_lossy().into_owned())
                .unwrap_or_else(|| DEFAULT_WORLD.into()),
            qgc: var("FLIGHT_QGC").map(PathBuf::from),
        }
    }

    /// Pick the runner: native if `FLIGHT_PX4_DIR` is set, else docker if
    /// the image exists. Checks files and `docker image inspect` only.
    pub fn runner(&self) -> Result<Runner, String> {
        match &self.px4_dir {
            Some(dir) => check_native(dir).map(|()| Runner::Native(dir.clone())),
            None => match docker_image_exists(&self.image) {
                Ok(true) => Ok(Runner::Docker(self.image.clone())),
                Ok(false) => Err(format!(
                    "no PX4/Gazebo: docker image {} is not built (run `nix run .#build-px4-image` or `just px4-image`), \
                     or set FLIGHT_PX4_DIR to a PX4-Autopilot checkout built with `make px4_sitl`",
                    self.image
                )),
                Err(e) => Err(format!(
                    "no PX4/Gazebo: FLIGHT_PX4_DIR is not set and docker is unusable ({e}); \
                     set FLIGHT_PX4_DIR to a PX4-Autopilot checkout built with `make px4_sitl`, \
                     or install docker and run `nix run .#build-px4-image`"
                )),
            },
        }
    }
}

/// Where PX4 and `gz` run.
#[derive(Debug, Clone, PartialEq)]
pub enum Runner {
    /// A PX4-Autopilot checkout; `gz` from PATH.
    Native(PathBuf),
    /// The docker image (from docker/px4-gz.Dockerfile).
    Docker(String),
}

impl Runner {
    /// PX4 checkout path as seen by the process.
    pub fn px4_dir(&self) -> PathBuf {
        match self {
            Runner::Native(d) => d.clone(),
            Runner::Docker(_) => PathBuf::from(CONTAINER_PX4_DIR),
        }
    }

    pub fn describe(&self) -> String {
        match self {
            Runner::Native(d) => format!("native {}", d.display()),
            Runner::Docker(i) => format!("docker {i}"),
        }
    }
}

pub(crate) fn check_native(dir: &Path) -> Result<(), String> {
    let bin = px4_build_dir(dir).join("bin/px4");
    if bin.is_file() {
        Ok(())
    } else {
        Err(format!(
            "FLIGHT_PX4_DIR={}: {} not found; run `make px4_sitl` there (with Gazebo Harmonic installed for gz), \
             or unset FLIGHT_PX4_DIR to use the docker image",
            dir.display(),
            bin.display()
        ))
    }
}

/// `docker image inspect`: Ok(false) if the image is missing, Err if docker is.
pub(crate) fn docker_image_exists(image: &str) -> Result<bool, String> {
    match std::process::Command::new("docker")
        .args(["image", "inspect", "--format", "{{.Id}}", image])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .output()
    {
        Ok(o) if o.status.success() => Ok(true),
        Ok(o) => {
            let err = String::from_utf8_lossy(&o.stderr);
            if err.contains("No such image")
                || err.contains("No such object")
                || err.contains("not found")
            {
                Ok(false)
            } else {
                Err(err.trim().to_string())
            }
        }
        Err(e) => Err(format!("cannot run docker: {e}")),
    }
}

/// First executable named `name` on `path` (a PATH value).
pub fn which_in(name: &str, path: Option<&OsString>) -> Option<PathBuf> {
    let path = path?;
    std::env::split_paths(path)
        .map(|d| d.join(name))
        .find(|p| is_executable(p))
}

pub(crate) fn is_executable(p: &Path) -> bool {
    use std::os::unix::fs::PermissionsExt;
    p.metadata()
        .map(|m| m.is_file() && m.permissions().mode() & 0o111 != 0)
        .unwrap_or(false)
}

/// Find QGroundControl: `FLIGHT_QGC` (`override_path`), then
/// `QGroundControl` / `qgroundcontrol` on PATH (nixpkgs' package installs
/// `QGroundControl`), then a `QGroundControl*.AppImage` in
/// `~/Applications` or `~/Downloads`.
pub fn find_qgc(
    override_path: Option<&Path>,
    path: Option<&OsString>,
    home: Option<&Path>,
) -> Result<PathBuf, String> {
    if let Some(p) = override_path {
        return if is_executable(p) {
            Ok(p.to_path_buf())
        } else {
            Err(format!(
                "FLIGHT_QGC={} is not an executable file (chmod +x an AppImage)",
                p.display()
            ))
        };
    }
    for name in ["QGroundControl", "qgroundcontrol"] {
        if let Some(p) = which_in(name, path) {
            return Ok(p);
        }
    }
    let mut non_exec = None;
    if let Some(home) = home {
        for dir in ["Applications", "Downloads"] {
            let Ok(rd) = std::fs::read_dir(home.join(dir)) else {
                continue;
            };
            let mut found: Vec<PathBuf> = rd
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| {
                    p.file_name().map(|n| n.to_string_lossy()).is_some_and(|n| {
                        n.starts_with("QGroundControl")
                            && n.to_ascii_lowercase().ends_with(".appimage")
                    })
                })
                .collect();
            found.sort();
            // Newest name last (QGroundControl-x86_64.AppImage, versioned names sort by version).
            while let Some(p) = found.pop() {
                if is_executable(&p) {
                    return Ok(p);
                }
                non_exec.get_or_insert(p);
            }
        }
    }
    Err(match non_exec {
        Some(p) => format!(
            "found {} but it is not executable: chmod +x it",
            p.display()
        ),
        None => {
            "QGroundControl not found: `nix develop` (nixpkgs qgroundcontrol), `nix run .#qgc`, \
                 set FLIGHT_QGC, or put QGroundControl.AppImage in ~/Applications"
                .into()
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::PermissionsExt;

    fn tmpdir(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("flight-launch-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    fn touch(p: &Path, exec: bool) {
        std::fs::create_dir_all(p.parent().unwrap()).unwrap();
        std::fs::write(p, "#!/bin/sh\n").unwrap();
        std::fs::set_permissions(
            p,
            std::fs::Permissions::from_mode(if exec { 0o755 } else { 0o644 }),
        )
        .unwrap();
    }

    #[test]
    fn native_runner_needs_built_px4() {
        let d = tmpdir("native");
        let cfg = LaunchConfig {
            px4_dir: Some(d.clone()),
            ..Default::default()
        };
        let e = cfg.runner().unwrap_err();
        assert!(e.contains("make px4_sitl"), "{e}");
        touch(&d.join("build/px4_sitl_default/bin/px4"), true);
        assert_eq!(cfg.runner(), Ok(Runner::Native(d.clone())));
        std::fs::remove_dir_all(d).unwrap();
    }

    #[test]
    fn qgc_search_order() {
        let d = tmpdir("qgc");
        let bin = d.join("bin");
        let home = d.join("home");
        let path = OsString::from(bin.as_os_str());
        assert!(find_qgc(None, Some(&path), Some(&home)).is_err());

        touch(
            &home.join("Downloads/QGroundControl-x86_64.AppImage"),
            false,
        );
        let e = find_qgc(None, Some(&path), Some(&home)).unwrap_err();
        assert!(e.contains("chmod +x"), "{e}");

        touch(&home.join("Applications/QGroundControl.AppImage"), true);
        assert_eq!(
            find_qgc(None, Some(&path), Some(&home)).unwrap(),
            home.join("Applications/QGroundControl.AppImage")
        );

        touch(&bin.join("qgroundcontrol"), true);
        assert_eq!(
            find_qgc(None, Some(&path), Some(&home)).unwrap(),
            bin.join("qgroundcontrol")
        );

        touch(&bin.join("QGroundControl"), true);
        assert_eq!(
            find_qgc(None, Some(&path), Some(&home)).unwrap(),
            bin.join("QGroundControl")
        );

        let own = d.join("own-qgc");
        touch(&own, true);
        assert_eq!(find_qgc(Some(&own), Some(&path), Some(&home)).unwrap(), own);
        assert!(find_qgc(Some(&d.join("missing")), Some(&path), Some(&home)).is_err());
        std::fs::remove_dir_all(d).unwrap();
    }
}
