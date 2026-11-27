//! QGroundControl.

use std::path::PathBuf;
use std::time::Duration;

use async_trait::async_trait;
use flight_core::{Backend, LogLevel, Module, ModuleKind, ModuleStatus, SimContext};

use crate::config::{LaunchConfig, find_qgc};
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

/// Environment for QGC: headless (`ctx.gui == false`) runs it on Qt's
/// offscreen platform so it still acts as the GCS on udp 14550.
pub fn qgc_env(gui: bool, is_appimage: bool) -> Vec<(String, String)> {
    let mut env = Vec::new();
    if !gui {
        env.push(("QT_QPA_PLATFORM".into(), "offscreen".into()));
    }
    if is_appimage {
        // Hosts without FUSE 2 (common now) can still run the AppImage.
        env.push(("APPIMAGE_EXTRACT_AND_RUN".into(), "1".into()));
    }
    env
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
        let appimage = bin
            .to_string_lossy()
            .to_ascii_lowercase()
            .ends_with(".appimage");
        let mut spec = Spec::new(bin.clone(), vec![]);
        spec.env = qgc_env(ctx.gui, appimage);
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
        let mode = if ctx.gui { "" } else { ", offscreen" };
        set_status(
            &self.status,
            ModuleStatus::Running(format!("{}{mode}", bin.display())),
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
    fn env_for_headless_and_appimage() {
        assert!(qgc_env(true, false).is_empty());
        assert_eq!(
            qgc_env(false, false),
            [("QT_QPA_PLATFORM".to_string(), "offscreen".to_string())]
        );
        assert!(
            qgc_env(true, true)
                .iter()
                .any(|(k, _)| k == "APPIMAGE_EXTRACT_AND_RUN")
        );
    }
}
