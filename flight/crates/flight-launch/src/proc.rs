//! Child processes in their own process group: output streamed into the
//! sim log, the group killed on stop and on drop, and an unexpected exit
//! reported as `ModuleStatus::Failed`.

use std::ffi::OsString;
use std::path::PathBuf;
use std::process::Stdio;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use flight_core::{LogLevel, LogLine, ModuleStatus};
use tokio::io::{AsyncBufReadExt, AsyncRead, BufReader};
use tokio::process::Command;
use tokio::sync::{broadcast, watch};
use tokio::task::JoinHandle;

pub(crate) type SharedStatus = Arc<Mutex<ModuleStatus>>;

pub(crate) fn new_status() -> SharedStatus {
    Arc::new(Mutex::new(ModuleStatus::Stopped))
}

pub(crate) fn set_status(status: &SharedStatus, s: ModuleStatus) {
    *status.lock().unwrap_or_else(|e| e.into_inner()) = s;
}

pub(crate) fn get_status(status: &SharedStatus) -> ModuleStatus {
    status.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// Log sink bound to a source name.
#[derive(Clone)]
pub(crate) struct Logger {
    pub source: String,
    pub tx: broadcast::Sender<LogLine>,
}

impl Logger {
    pub fn log(&self, level: LogLevel, text: impl Into<String>) {
        let _ = self.tx.send(LogLine {
            source: self.source.clone(),
            level,
            text: text.into(),
        });
    }
}

/// What to run.
#[derive(Debug, Clone)]
pub(crate) struct Spec {
    pub program: OsString,
    pub args: Vec<String>,
    pub env: Vec<(String, String)>,
    pub cwd: Option<PathBuf>,
    /// `docker rm -f` this container on stop/drop (the child is `docker run`).
    pub container: Option<String>,
    /// A stdout/stderr line containing one of these marks the process ready.
    pub ready: &'static [&'static str],
    /// A line containing one of these marks start-up as failed.
    pub fail: &'static [&'static str],
    /// Set the module status to `Failed` if the process exits by itself.
    pub fail_module_on_exit: bool,
}

impl Spec {
    pub fn new(program: impl Into<OsString>, args: Vec<String>) -> Self {
        Spec {
            program: program.into(),
            args,
            env: Vec::new(),
            cwd: None,
            container: None,
            ready: &[],
            fail: &[],
            fail_module_on_exit: true,
        }
    }
}

/// Readiness as seen in the output.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Signal {
    Pending,
    Ready,
    Failed(String),
}

pub(crate) struct Proc {
    pgid: i32,
    container: Option<String>,
    stopping: Arc<AtomicBool>,
    exited: watch::Receiver<Option<String>>,
    signal: watch::Receiver<Signal>,
    tasks: Vec<JoinHandle<()>>,
    done: bool,
}

fn level_of(line: &str, stderr: bool) -> LogLevel {
    let t = line.trim_start();
    if stderr
        || t.starts_with("ERROR")
        || t.starts_with("WARN")
        || t.starts_with("[Err]")
        || t.starts_with("[Wrn]")
    {
        LogLevel::Warn
    } else {
        LogLevel::Info
    }
}

fn pump<R: AsyncRead + Unpin + Send + 'static>(
    r: R,
    stderr: bool,
    log: Logger,
    ready: &'static [&'static str],
    fail: &'static [&'static str],
    signal: Arc<watch::Sender<Signal>>,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let mut lines = BufReader::new(r).lines();
        while let Ok(Some(line)) = lines.next_line().await {
            if *signal.borrow() == Signal::Pending {
                if fail.iter().any(|p| line.contains(p)) {
                    let _ = signal.send(Signal::Failed(line.clone()));
                } else if ready.iter().any(|p| line.contains(p)) {
                    let _ = signal.send(Signal::Ready);
                }
            }
            log.log(level_of(&line, stderr), line);
        }
    })
}

fn killpg(pgid: i32, sig: i32) {
    if pgid > 0 {
        // SAFETY: plain syscall on a process group we created.
        unsafe {
            libc::killpg(pgid, sig);
        }
    }
}

/// Remove a container, synchronously (used from Drop as well).
pub(crate) fn docker_rm_sync(name: &str) {
    let _ = std::process::Command::new("docker")
        .args(["rm", "-f", name])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
}

pub(crate) async fn docker_rm(name: &str) {
    let fut = Command::new("docker")
        .args(["rm", "-f", name])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    let _ = tokio::time::timeout(Duration::from_secs(15), fut).await;
}

impl Proc {
    /// Spawn `spec` in a new process group. `label` names it in messages.
    pub async fn spawn(
        spec: Spec,
        label: &str,
        log: Logger,
        status: SharedStatus,
    ) -> anyhow::Result<Proc> {
        if let Some(c) = &spec.container {
            // A container left over from a crash would make `docker run --name` fail.
            docker_rm(c).await;
        }
        let mut cmd = Command::new(&spec.program);
        cmd.args(&spec.args)
            .envs(spec.env.iter().map(|(k, v)| (k.as_str(), v.as_str())))
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .process_group(0)
            .kill_on_drop(false);
        if let Some(d) = &spec.cwd {
            cmd.current_dir(d);
        }
        let mut child = cmd.spawn().map_err(|e| {
            anyhow::anyhow!(
                "{label}: cannot run {}: {e}",
                spec.program.to_string_lossy()
            )
        })?;
        let pgid = child.id().map(|p| p as i32).unwrap_or(0);
        log.log(
            LogLevel::Info,
            format!(
                "{label}: started pid {pgid}: {} {}",
                spec.program.to_string_lossy(),
                spec.args.join(" ")
            ),
        );

        let (sig_tx, sig_rx) = watch::channel(Signal::Pending);
        let sig_tx = Arc::new(sig_tx);
        let mut tasks = Vec::new();
        if let Some(out) = child.stdout.take() {
            tasks.push(pump(
                out,
                false,
                log.clone(),
                spec.ready,
                spec.fail,
                sig_tx.clone(),
            ));
        }
        if let Some(err) = child.stderr.take() {
            tasks.push(pump(
                err,
                true,
                log.clone(),
                spec.ready,
                spec.fail,
                sig_tx.clone(),
            ));
        }

        let stopping = Arc::new(AtomicBool::new(false));
        let (exit_tx, exit_rx) = watch::channel(None);
        {
            let stopping = stopping.clone();
            let label = label.to_string();
            let fail_module = spec.fail_module_on_exit;
            tasks.push(tokio::spawn(async move {
                let desc = match child.wait().await {
                    Ok(s) => s.to_string(),
                    Err(e) => format!("wait failed: {e}"),
                };
                if !stopping.load(Ordering::SeqCst) {
                    let msg = format!("{label} exited on its own ({desc})");
                    log.log(
                        if fail_module {
                            LogLevel::Error
                        } else {
                            LogLevel::Warn
                        },
                        msg.clone(),
                    );
                    if fail_module {
                        set_status(&status, ModuleStatus::Failed(msg.clone()));
                    }
                    if *sig_tx.borrow() == Signal::Pending {
                        let _ = sig_tx.send(Signal::Failed(msg));
                    }
                }
                let _ = exit_tx.send(Some(desc));
            }));
        }
        Ok(Proc {
            pgid,
            container: spec.container,
            stopping,
            exited: exit_rx,
            signal: sig_rx,
            tasks,
            done: false,
        })
    }

    /// Exit description once the process has exited.
    pub fn exited(&self) -> Option<String> {
        self.exited.borrow().clone()
    }

    /// Wait for a ready line, a fail line, exit or `timeout`.
    pub async fn wait_ready(&self, timeout: Duration) -> Result<(), String> {
        let mut sig = self.signal.clone();
        let res = tokio::time::timeout(timeout, sig.wait_for(|s| *s != Signal::Pending)).await;
        match res {
            Ok(Ok(s)) => match &*s {
                Signal::Ready => Ok(()),
                Signal::Failed(why) => Err(why.clone()),
                Signal::Pending => unreachable!(),
            },
            Ok(Err(_)) => Err("output closed before the process was ready".into()),
            Err(_) => Err(format!("not ready after {} s", timeout.as_secs())),
        }
    }

    /// Kill the group (TERM, then KILL after a grace period) and remove the
    /// container. Idempotent.
    pub async fn stop(&mut self) {
        if self.done {
            return;
        }
        self.stopping.store(true, Ordering::SeqCst);
        if let Some(c) = &self.container {
            docker_rm(c).await;
        }
        killpg(self.pgid, libc::SIGTERM);
        let mut ex = self.exited.clone();
        if tokio::time::timeout(Duration::from_secs(5), ex.wait_for(Option::is_some))
            .await
            .is_err()
        {
            killpg(self.pgid, libc::SIGKILL);
            let _ =
                tokio::time::timeout(Duration::from_secs(2), ex.wait_for(Option::is_some)).await;
        }
        // Children that outlived the leader are still in the group.
        killpg(self.pgid, libc::SIGKILL);
        for t in self.tasks.drain(..) {
            t.abort();
        }
        self.done = true;
    }
}

impl Drop for Proc {
    fn drop(&mut self) {
        if self.done {
            return;
        }
        self.stopping.store(true, Ordering::SeqCst);
        killpg(self.pgid, libc::SIGKILL);
        if let Some(c) = &self.container {
            docker_rm_sync(c);
        }
        for t in self.tasks.drain(..) {
            t.abort();
        }
    }
}

/// Run a short command to completion (e.g. `gz service`), with a timeout.
/// Returns stdout+stderr on success, an error message otherwise.
pub(crate) async fn run_once(
    program: &OsString,
    args: &[String],
    env: &[(String, String)],
    timeout: Duration,
) -> Result<String, String> {
    let fut = Command::new(program)
        .args(args)
        .envs(env.iter().map(|(k, v)| (k.as_str(), v.as_str())))
        .stdin(Stdio::null())
        .kill_on_drop(true)
        .output();
    match tokio::time::timeout(timeout, fut).await {
        Ok(Ok(o)) => {
            let text = format!(
                "{}{}",
                String::from_utf8_lossy(&o.stdout),
                String::from_utf8_lossy(&o.stderr)
            );
            if o.status.success() {
                Ok(text)
            } else {
                Err(format!("{}: {}", o.status, text.trim()))
            }
        }
        Ok(Err(e)) => Err(e.to_string()),
        Err(_) => Err(format!("timed out after {} ms", timeout.as_millis())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn logger() -> (Logger, broadcast::Receiver<LogLine>) {
        let (tx, rx) = broadcast::channel(64);
        (
            Logger {
                source: "t".into(),
                tx,
            },
            rx,
        )
    }

    #[tokio::test]
    async fn ready_line_and_stop_kills_group() {
        let (log, mut rx) = logger();
        let status: SharedStatus = Arc::new(Mutex::new(ModuleStatus::Starting));
        let mut spec = Spec::new(
            "sh",
            vec![
                "-c".into(),
                "echo hello; echo READY; sleep 30 & wait".into(),
            ],
        );
        spec.ready = &["READY"];
        let mut p = Proc::spawn(spec, "sh", log, status.clone()).await.unwrap();
        p.wait_ready(Duration::from_secs(5)).await.unwrap();
        p.stop().await;
        assert!(p.exited().is_some());
        // Stopping on purpose is not a failure.
        assert_eq!(get_status(&status), ModuleStatus::Starting);
        let mut lines = Vec::new();
        while let Ok(l) = rx.try_recv() {
            lines.push(l.text);
        }
        assert!(lines.iter().any(|l| l == "hello"));
        p.stop().await; // idempotent
    }

    #[tokio::test]
    async fn exit_on_its_own_fails_module() {
        let (log, _rx) = logger();
        let status: SharedStatus = Arc::new(Mutex::new(ModuleStatus::Running("x".into())));
        let spec = Spec::new("sh", vec!["-c".into(), "exit 3".into()]);
        let p = Proc::spawn(spec, "sh", log, status.clone()).await.unwrap();
        assert!(p.wait_ready(Duration::from_secs(5)).await.is_err());
        let mut ex = p.exited.clone();
        ex.wait_for(Option::is_some).await.unwrap();
        match get_status(&status) {
            ModuleStatus::Failed(m) => assert!(m.contains("exit status: 3"), "{m}"),
            s => panic!("{s:?}"),
        }
    }

    #[tokio::test]
    async fn fail_line() {
        let (log, _rx) = logger();
        let status: SharedStatus = Arc::new(Mutex::new(ModuleStatus::Starting));
        let mut spec = Spec::new(
            "sh",
            vec!["-c".into(), "echo 'ERROR boom'; sleep 30".into()],
        );
        spec.ready = &["READY"];
        spec.fail = &["ERROR"];
        let mut p = Proc::spawn(spec, "sh", log, status).await.unwrap();
        assert_eq!(
            p.wait_ready(Duration::from_secs(5)).await,
            Err("ERROR boom".into())
        );
        p.stop().await;
    }

    #[test]
    fn levels() {
        assert_eq!(level_of("INFO  [px4] hi", false), LogLevel::Info);
        assert_eq!(level_of("ERROR [init] x", false), LogLevel::Warn);
        assert_eq!(level_of("anything", true), LogLevel::Warn);
    }
}
