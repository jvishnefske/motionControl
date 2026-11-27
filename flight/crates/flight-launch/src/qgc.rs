//! QGroundControl.

use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::time::Duration;

use async_trait::async_trait;
use flight_core::{Backend, LogLevel, Module, ModuleKind, ModuleStatus, SimContext};

use crate::config::{LaunchConfig, find_qgc, which_in};
use crate::proc::{Logger, Proc, SharedStatus, Spec, get_status, new_status, set_status};

/// How long QGC must survive to count as started.
const SETTLE: Duration = Duration::from_secs(2);

/// Locate QGroundControl as [`find_qgc`] does, from the process environment.
pub fn locate_qgc(cfg: &LaunchConfig) -> Result<PathBuf, String> {
    let home = std::env::var_os("HOME").map(PathBuf::from);
    find_qgc(
        cfg.qgc.as_deref(),
        std::env::var_os("PATH").as_ref(),
        home.as_deref(),
    )
}

/// How to run QGC: program, arguments and environment.
///
/// * A nixpkgs build (`/nix/store/...`) on a non-NixOS host cannot load the
///   host's OpenGL drivers and aborts with "Could not initialize GLX", so it
///   is run on xcb with no GL integration and the Qt Quick software renderer.
/// * Headless (`gui == false`): under `xvfb-run` when one is given (Qt's
///   offscreen platform still wants GLX and aborts the same way), else on the
///   offscreen platform.
pub fn qgc_command(
    bin: &Path,
    gui: bool,
    xvfb_run: Option<&Path>,
) -> (OsString, Vec<String>, Vec<(String, String)>) {
    let lower = bin.to_string_lossy().to_ascii_lowercase();
    let appimage = lower.ends_with(".appimage");
    let nix = bin.starts_with("/nix/store");
    let mut env: Vec<(String, String)> = Vec::new();
    let software = |env: &mut Vec<(String, String)>| {
        env.push(("QT_QPA_PLATFORM".into(), "xcb".into()));
        env.push(("QT_XCB_GL_INTEGRATION".into(), "none".into()));
        env.push(("QT_QUICK_BACKEND".into(), "software".into()));
    };
    let (program, args) = match (gui, xvfb_run) {
        (false, Some(x)) => {
            software(&mut env);
            (x.as_os_str().to_owned(), vec!["-a".into(), bin.to_string_lossy().into_owned()])
        }
        _ => {
            if !gui {
                env.push(("QT_QPA_PLATFORM".into(), "offscreen".into()));
            } else if nix {
                software(&mut env);
            }
            (bin.as_os_str().to_owned(), vec![])
        }
    };
    if appimage {
        // Hosts without FUSE 2 (common now) can still run the AppImage.
        env.push(("APPIMAGE_EXTRACT_AND_RUN".into(), "1".into()));
    }
    (program, args, env)
}

/// QGroundControl; it autoconnects to the vehicle's GCS stream on udp 14550.
pub struct QGroundControl {
    cfg: LaunchConfig,
    status: SharedStatus,
    proc: Option<Proc>,
}

impl QGroundControl {
    pub fn new(cfg: LaunchConfig) -> Self {
        QGroundControl {
            cfg,
            status: new_status(),
            proc: None,
        }
    }
}

#[async_trait]
impl Module for QGroundControl {
    fn kind(&self) -> ModuleKind {
        ModuleKind::GroundStation
    }
    fn backend(&self) -> Backend {
        Backend::Real
    }
    fn name(&self) -> &str {
        "qgroundcontrol"
    }

    async fn start(&mut self, ctx: &SimContext) -> anyhow::Result<()> {
        self.stop().await?;
        set_status(&self.status, ModuleStatus::Starting);
        let log = Logger {
            source: self.name().into(),
            tx: ctx.log.clone(),
        };
        let bin = match locate_qgc(&self.cfg) {
            Ok(b) => b,
            Err(e) => {
                set_status(&self.status, ModuleStatus::Failed(e.clone()));
                anyhow::bail!(e);
            }
        };
        if ctx.ports.gcs != 14550 {
            log.log(
                LogLevel::Warn,
                format!("QGC autoconnects on udp 14550, not {}", ctx.ports.gcs),
            );
        }
        let xvfb = which_in("xvfb-run", std::env::var_os("PATH").as_ref());
        let (program, args, env) = qgc_command(&bin, ctx.gui, xvfb.as_deref());
        let headless = if ctx.gui { "" } else if xvfb.is_some() { ", headless (xvfb)" } else { ", offscreen" };
        let mut spec = Spec::new(program, args);
        spec.env = env;
        let proc = match Proc::spawn(spec, "QGroundControl", log, self.status.clone()).await {
            Ok(p) => p,
            Err(e) => {
                set_status(&self.status, ModuleStatus::Failed(e.to_string()));
                return Err(e);
            }
        };
        tokio::time::sleep(SETTLE).await;
        if let Some(ex) = proc.exited() {
            let msg = format!("QGroundControl exited at start ({ex})");
            set_status(&self.status, ModuleStatus::Failed(msg.clone()));
            anyhow::bail!(msg);
        }
        self.proc = Some(proc);
        set_status(
            &self.status,
            ModuleStatus::Running(format!("{}{headless}", bin.display())),
        );
        Ok(())
    }

    async fn stop(&mut self) -> anyhow::Result<()> {
        if let Some(mut p) = self.proc.take() {
            p.stop().await;
            set_status(&self.status, ModuleStatus::Stopped);
        }
        Ok(())
    }

    fn status(&self) -> ModuleStatus {
        get_status(&self.status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_for_nix_headless_and_appimage() {
        let get = |env: &[(String, String)], k: &str| env.iter().find(|(x, _)| x == k).map(|(_, v)| v.clone());
        let nix = Path::new("/nix/store/abc-qgroundcontrol/bin/QGroundControl");
        // Nix build with a GUI: software rendering, no xvfb.
        let (p, a, e) = qgc_command(nix, true, None);
        assert_eq!(p, nix.as_os_str());
        assert!(a.is_empty());
        assert_eq!(get(&e, "QT_XCB_GL_INTEGRATION").as_deref(), Some("none"));
        // Headless with xvfb-run.
        let (p, a, e) = qgc_command(nix, false, Some(Path::new("/usr/bin/xvfb-run")));
        assert_eq!(p, "/usr/bin/xvfb-run");
        assert_eq!(a, ["-a", nix.to_str().unwrap()]);
        assert_eq!(get(&e, "QT_QUICK_BACKEND").as_deref(), Some("software"));
        // Headless without xvfb: offscreen.
        let (_, _, e) = qgc_command(Path::new("/opt/QGroundControl"), false, None);
        assert_eq!(get(&e, "QT_QPA_PLATFORM").as_deref(), Some("offscreen"));
        // Native GUI build: untouched; AppImage extracts.
        assert!(qgc_command(Path::new("/opt/QGroundControl"), true, None).2.is_empty());
        let (_, _, e) = qgc_command(Path::new("/home/u/Applications/QGroundControl.AppImage"), true, None);
        assert_eq!(get(&e, "APPIMAGE_EXTRACT_AND_RUN").as_deref(), Some("1"));
    }
}
