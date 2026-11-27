use std::convert::Infallible;
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use axum::extract::State;
use axum::http::StatusCode;
use axum::response::sse::{Event, KeepAlive, Sse};
use axum::response::{Html, IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use flight_core::{Airframe, Backend, LogLine, ModuleKind};
use serde::Deserialize;
use serde_json::json;
use tokio_stream::StreamExt as _;
use tokio_stream::wrappers::{BroadcastStream, IntervalStream};

use crate::orchestrator::Orchestrator;

/// The single page, embedded at build time.
pub const INDEX_HTML: &str = include_str!("ui.html");

/// SSE state push period (5 Hz).
const STATE_PERIOD: Duration = Duration::from_millis(200);

type Orch = Arc<Orchestrator>;

/// The axum app:
///
/// | method | path               | body                                   |
/// |--------|--------------------|----------------------------------------|
/// | GET    | `/`                | the HTML page                          |
/// | GET    | `/api/state`       | → [`Snapshot`](crate::Snapshot) JSON   |
/// | GET    | `/api/events`      | SSE: `state` (snapshot, 5 Hz), `log` (a LogLine) |
/// | POST   | `/api/backend`     | `{"kind":"autopilot","backend":"real"}`|
/// | POST   | `/api/preset`      | `{"name":"all-sim"}`                   |
/// | POST   | `/api/airframe`    | `{"airframe":"fixed_wing"}`            |
/// | POST   | `/api/mission`     | `{"name":"survey"}` (select)           |
/// | POST   | `/api/gui`         | `{"gui":true}`                         |
/// | POST   | `/api/test/run`    | `{"mission":"box"}` (mission optional) |
/// | POST   | `/api/test/abort`  | `{}`                                   |
///
/// POSTs answer `{"ok":true}`, or `{"ok":false,"error":"..."}` with 409
/// (refused) / 400 (bad input).
pub fn router(orch: Orch) -> Router {
    Router::new()
        .route("/", get(|| async { Html(INDEX_HTML) }))
        .route("/api/state", get(state))
        .route("/api/events", get(events))
        .route("/api/backend", post(set_backend))
        .route("/api/preset", post(preset))
        .route("/api/airframe", post(airframe))
        .route("/api/mission", post(mission))
        .route("/api/gui", post(gui))
        .route("/api/test/run", post(run_test))
        .route("/api/test/abort", post(abort_test))
        .with_state(orch)
}

/// Serves [`router`] on `addr` until the process ends.
pub async fn serve(addr: SocketAddr, orch: Arc<Orchestrator>) -> anyhow::Result<()> {
    let listener = tokio::net::TcpListener::bind(addr).await?;
    tracing::info!("flight UI on http://{}", listener.local_addr()?);
    axum::serve(listener, router(orch)).await?;
    Ok(())
}

fn reply(r: anyhow::Result<()>) -> Response {
    match r {
        Ok(()) => Json(json!({ "ok": true })).into_response(),
        Err(e) => (StatusCode::CONFLICT, Json(json!({ "ok": false, "error": format!("{e:#}") }))).into_response(),
    }
}

fn bad(e: impl std::fmt::Display) -> Response {
    (StatusCode::BAD_REQUEST, Json(json!({ "ok": false, "error": e.to_string() }))).into_response()
}

async fn state(State(o): State<Orch>) -> impl IntoResponse {
    Json(o.snapshot())
}

fn state_event(o: &Orchestrator) -> Event {
    Event::default().event("state").json_data(o.snapshot()).unwrap_or_else(|_| Event::default().comment("encode"))
}

fn log_event(l: &LogLine) -> Event {
    Event::default().event("log").json_data(l).unwrap_or_else(|_| Event::default().comment("encode"))
}

async fn events(State(o): State<Orch>) -> Sse<impl tokio_stream::Stream<Item = Result<Event, Infallible>>> {
    let history: Vec<Event> = o.log_history().iter().map(log_event).collect();
    let logs = BroadcastStream::new(o.log_sender().subscribe())
        .filter_map(|r| r.ok())
        .map(|l| log_event(&l));
    let o2 = o.clone();
    let mut tick = tokio::time::interval(STATE_PERIOD);
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let states = IntervalStream::new(tick).map(move |_| state_event(&o2));
    let stream = tokio_stream::iter(history).chain(states.merge(logs)).map(Ok);
    Sse::new(stream).keep_alive(KeepAlive::default())
}

#[derive(Deserialize)]
struct BackendReq {
    kind: ModuleKind,
    backend: Backend,
}

async fn set_backend(State(o): State<Orch>, Json(r): Json<BackendReq>) -> Response {
    reply(o.set_backend(r.kind, r.backend).await)
}

#[derive(Deserialize)]
struct NameReq {
    name: String,
}

async fn preset(State(o): State<Orch>, Json(r): Json<NameReq>) -> Response {
    if crate::preset_backends(&r.name).is_none() {
        return bad(format!("unknown preset {:?}", r.name));
    }
    reply(o.apply_preset(&r.name).await)
}

#[derive(Deserialize)]
struct AirframeReq {
    airframe: Airframe,
}

async fn airframe(State(o): State<Orch>, Json(r): Json<AirframeReq>) -> Response {
    reply(o.set_airframe(r.airframe).await)
}

async fn mission(State(o): State<Orch>, Json(r): Json<NameReq>) -> Response {
    match o.select_mission(&r.name) {
        Ok(()) => reply(Ok(())),
        Err(e) => bad(e),
    }
}

#[derive(Deserialize)]
struct GuiReq {
    gui: bool,
}

async fn gui(State(o): State<Orch>, Json(r): Json<GuiReq>) -> Response {
    o.set_gui(r.gui);
    reply(Ok(()))
}

#[derive(Deserialize, Default)]
struct RunReq {
    mission: Option<String>,
}

async fn run_test(State(o): State<Orch>, body: Option<Json<RunReq>>) -> Response {
    let name = body.and_then(|Json(r)| r.mission).unwrap_or_else(|| o.selected_mission());
    if !flight_core::Mission::BUILTIN.contains(&name.as_str()) {
        return bad(format!("unknown mission {name:?}"));
    }
    reply(o.run_mission_test(&name))
}

async fn abort_test(State(o): State<Orch>) -> Response {
    reply(o.abort_test().await)
}
