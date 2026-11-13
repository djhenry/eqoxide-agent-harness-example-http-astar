//! The agent control routes, under the same `/v1/<group>/<action>` paths eqoxide's own HTTP API
//! used. Every route answers from the latest `Observation` or queues intent for the driver.

use crate::driver::{Control, VerbReply};
use axum::extract::State;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::{delete, get, post};
use axum::{Json, Router};
use eqoxide_agent_protocol::movement::AgentMovement;
use eqoxide_agent_protocol::observation::{Observation, VisibleEntity};
use eqoxide_agent_protocol::verb::{
    AgentVerb, CastRequest, ChatVerb, CombatVerb, InteractVerb, InventoryVerb, LifecycleVerb,
    PetVerb,
};
use harness_nav::status::{AggroAvoidOpts, NavStatus};
use harness_nav::walker::ZoneLinesError;
use serde::Deserialize;
use serde_json::{json, Value};
use std::sync::Arc;
use std::time::Duration;

type Ctl = Arc<Control>;

/// Default `duration_ms` of a manual move.
const MANUAL_DEFAULT_MS: u64 = 400;

pub fn router() -> Router<Ctl> {
    Router::new()
        .route("/v1/observe/state", get(observe_state))
        .route("/v1/observe/entities", get(observe_entities))
        .route("/v1/observe/nav", get(observe_nav))
        .route("/v1/observe/zone_lines", get(observe_zone_lines))
        .route("/v1/move/goto", post(goto))
        .route("/v1/move/follow", post(follow))
        .route("/v1/move/stop", post(stop))
        .route("/v1/move/manual", post(manual))
        .route("/v1/move/jump", post(jump))
        .route("/v1/combat/target", post(target))
        .route("/v1/combat/target/name", post(target_name))
        .route("/v1/combat/attack", post(attack_on))
        .route("/v1/combat/attack", delete(attack_off))
        .route("/v1/combat/consider", post(consider))
        .route("/v1/combat/cast", post(cast))
        .route("/v1/interact/sit", post(sit))
        .route("/v1/interact/stand", post(stand))
        .route("/v1/lifecycle/respawn", post(respawn))
        .route("/v1/lifecycle/camp", post(camp))
        .route("/v1/lifecycle/exit", post(exit))
        .route("/v1/inventory/move", post(inventory_move))
        .route("/v1/inventory/destroy", post(inventory_destroy))
        .route("/v1/inventory/consume", post(inventory_consume))
        .route("/v1/pet/command", post(pet_command))
        .route("/v1/chat/tell", post(chat_tell))
        .route("/v1/chat/ooc", post(chat_ooc))
        .route("/v1/chat/shout", post(chat_shout))
        .route("/v1/chat/group", post(chat_group))
        .route("/v1/chat/guild", post(chat_guild))
}

fn text(code: StatusCode, msg: impl Into<String>) -> Response {
    (code, msg.into()).into_response()
}

fn bad(msg: impl Into<String>) -> Response {
    text(StatusCode::BAD_REQUEST, msg)
}

/// A request body that is absent, or valid JSON for `T`. Malformed JSON and unknown keys are
/// `400`, never a silent default.
struct Body<T>(T);

#[axum::async_trait]
impl<T, S> axum::extract::FromRequest<S> for Body<T>
where
    T: serde::de::DeserializeOwned + Default,
    S: Send + Sync,
{
    type Rejection = Response;

    async fn from_request(req: axum::extract::Request, state: &S) -> Result<Self, Self::Rejection> {
        let bytes = axum::body::Bytes::from_request(req, state)
            .await
            .map_err(|e| bad(format!("unreadable body: {e}")))?;
        if bytes.iter().all(u8::is_ascii_whitespace) {
            return Ok(Body(T::default()));
        }
        serde_json::from_slice(&bytes)
            .map(Body)
            .map_err(|e| bad(format!("malformed body: {e}")))
    }
}

/// The latest observation, or the refusal for a session that has none or has ended. The `Err` is
/// the ready-to-send response, which handlers return as-is.
#[allow(clippy::result_large_err)]
fn live(c: &Control) -> Result<Observation, Response> {
    if let Some(why) = c.ended() {
        return Err(text(
            StatusCode::SERVICE_UNAVAILABLE,
            format!("no live session: {why}"),
        ));
    }
    c.latest()
        .ok_or_else(|| text(StatusCode::SERVICE_UNAVAILABLE, "no observation yet"))
}

/// As [`live`], and refuses while the character is dead.
#[allow(clippy::result_large_err)]
fn live_alive(c: &Control) -> Result<Observation, Response> {
    let obs = live(c)?;
    if obs.dead {
        return Err(text(StatusCode::CONFLICT, "the character is dead"));
    }
    Ok(obs)
}

fn dist(a: [f32; 3], b: [f32; 3]) -> f32 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// The visible spawn `name` resolves to: a live exact match, then a live substring match, then
/// the same two among corpses. Ties go to the nearest.
fn resolve<'a>(obs: &'a Observation, name: &str) -> Option<&'a VisibleEntity> {
    let needle = name.trim().to_lowercase();
    if needle.is_empty() {
        return None;
    }
    let nearest = |pred: &dyn Fn(&VisibleEntity) -> bool| {
        obs.visible
            .iter()
            .filter(|e| pred(e))
            .min_by(|a, b| dist(a.pos, obs.own.pos).total_cmp(&dist(b.pos, obs.own.pos)))
    };
    nearest(&|e| !e.dead && e.name.to_lowercase() == needle)
        .or_else(|| nearest(&|e| !e.dead && e.name.to_lowercase().contains(&needle)))
        .or_else(|| nearest(&|e| e.name.to_lowercase() == needle))
        .or_else(|| nearest(&|e| e.name.to_lowercase().contains(&needle)))
}

fn matched(e: &VisibleEntity) -> Value {
    json!({ "spawn_id": e.spawn_id, "name": e.name, "pos": e.pos, "dead": e.dead })
}

async fn observe_state(State(c): State<Ctl>) -> Response {
    match live(&c) {
        Ok(obs) => Json(obs).into_response(),
        Err(r) => r,
    }
}

async fn observe_entities(State(c): State<Ctl>) -> Response {
    match live(&c) {
        Ok(obs) => Json(obs.visible).into_response(),
        Err(r) => r,
    }
}

fn nav_json(s: &NavStatus, notices: Vec<String>) -> Value {
    json!({
        "state": s.state,
        "reason": s.reason,
        "goal_id": s.goal_id,
        "goal": s.goal,
        "tier": s.tier,
        "stalled": s.stall.is_some(),
        "local_planner_dead": s.local_planner_dead,
        "notices": notices,
    })
}

async fn observe_nav(State(c): State<Ctl>) -> Response {
    Json(nav_json(&c.nav_status(), c.recent_notices())).into_response()
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct GotoBody {
    name: Option<String>,
    x: Option<f32>,
    y: Option<f32>,
    z: Option<f32>,
    /// Brewall map coordinates, the negated server `x` and `y`.
    map_x: Option<f32>,
    map_y: Option<f32>,
    avoid_aggro: Option<bool>,
    aggro_buffer: Option<f32>,
}

fn apply_avoid(c: &Control, enabled: Option<bool>, buffer: Option<f32>) {
    if enabled.is_none() && buffer.is_none() {
        return;
    }
    let current = AggroAvoidOpts::default();
    c.slots().set_avoid(AggroAvoidOpts {
        enabled: enabled.unwrap_or(current.enabled),
        buffer: buffer.unwrap_or(current.buffer),
    });
}

fn target_entity(obs: &Observation) -> Option<&VisibleEntity> {
    let id = obs.own.target_id?;
    obs.visible.iter().find(|e| e.spawn_id == id)
}

async fn goto(State(c): State<Ctl>, Body(b): Body<GotoBody>) -> Response {
    let obs = match live_alive(&c) {
        Ok(o) => o,
        Err(r) => return r,
    };
    let raw = [b.x, b.y, b.z];
    let have_raw = raw.iter().filter(|v| v.is_some()).count();
    let have_map = [b.map_x, b.map_y].iter().filter(|v| v.is_some()).count();
    if b.name.is_some() && (have_raw + have_map) > 0 {
        return bad("conflicting target: give a name or coordinates, not both");
    }
    if (b.x.is_some() || b.y.is_some()) && have_map > 0 {
        return bad("conflicting target: give raw {x,y,z} or {map_x,map_y}, not both");
    }
    let (goal, entity): ((f32, f32, f32), Option<&VisibleEntity>) = if let Some(name) = &b.name {
        match resolve(&obs, name) {
            Some(e) => ((e.pos[0], e.pos[1], e.pos[2]), Some(e)),
            None => {
                return text(
                    StatusCode::NOT_FOUND,
                    format!("no visible entity matching '{name}'"),
                )
            }
        }
    } else if let (Some(x), Some(y), Some(z)) = (b.x, b.y, b.z) {
        ((x, y, z), None)
    } else if have_map == 2 {
        (
            (
                -b.map_x.unwrap(),
                -b.map_y.unwrap(),
                b.z.unwrap_or(obs.own.pos[2]),
            ),
            None,
        )
    } else if have_raw + have_map > 0 {
        return bad("partial target: give a complete {x,y,z} or {map_x,map_y}");
    } else {
        match target_entity(&obs) {
            Some(e) => ((e.pos[0], e.pos[1], e.pos[2]), Some(e)),
            None => return bad("no target; provide a name or coordinates"),
        }
    };
    apply_avoid(&c, b.avoid_aggro, b.aggro_buffer);
    c.cancel_manual();
    let goal_id = c.slots().goto(goal);
    Json(json!({
        "goal_id": goal_id,
        "goal": [goal.0, goal.1, goal.2],
        "distance": dist([goal.0, goal.1, goal.2], obs.own.pos),
        "matched": entity.map(matched),
    }))
    .into_response()
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct FollowBody {
    name: Option<String>,
    avoid_aggro: Option<bool>,
    aggro_buffer: Option<f32>,
}

async fn follow(State(c): State<Ctl>, Body(b): Body<FollowBody>) -> Response {
    let obs = match live_alive(&c) {
        Ok(o) => o,
        Err(r) => return r,
    };
    let entity = match &b.name {
        Some(name) => match resolve(&obs, name) {
            Some(e) => e,
            None => {
                return text(
                    StatusCode::NOT_FOUND,
                    format!("no visible entity matching '{name}'"),
                )
            }
        },
        None => match target_entity(&obs) {
            Some(e) => e,
            None => return bad("no target; provide a name"),
        },
    };
    if entity.dead {
        return text(
            StatusCode::CONFLICT,
            format!("'{}' is dead; nothing to follow", entity.name),
        );
    }
    apply_avoid(&c, b.avoid_aggro, b.aggro_buffer);
    c.cancel_manual();
    let goal_id = c
        .slots()
        .follow(&entity.name, (entity.pos[0], entity.pos[1], entity.pos[2]));
    Json(json!({ "goal_id": goal_id, "matched": matched(entity) })).into_response()
}

async fn stop(State(c): State<Ctl>) -> Response {
    c.stop();
    text(StatusCode::OK, "stopped")
}

/// The zone lines of the current zone as `{index, pos}`. Walk to a `pos` with `goto` and eqoxide
/// crosses once the character stands inside the line. `[]` means the zone has no zone line, and a
/// `409` means they are unknown.
async fn observe_zone_lines(State(c): State<Ctl>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    match c.zone_lines() {
        Some(Ok(lines)) => Json(
            lines
                .iter()
                .map(|l| json!({ "index": l.index, "pos": l.pos }))
                .collect::<Vec<_>>(),
        )
        .into_response(),
        Some(Err(ZoneLinesError::ZoneNotLoaded)) => {
            text(StatusCode::CONFLICT, "the zone's collision is not loaded")
        }
        Some(Err(ZoneLinesError::NoRegionData)) => text(
            StatusCode::CONFLICT,
            "the zone's region data is missing, so its zone lines are unknown",
        ),
        None => text(StatusCode::SERVICE_UNAVAILABLE, "no observation yet"),
    }
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct ManualBody {
    /// World `(east, north)` direction. Any magnitude.
    east: Option<f32>,
    north: Option<f32>,
    /// Swim axis, `-1..1`. Ignored on land.
    up: Option<f32>,
    jump: Option<bool>,
    duration_ms: Option<u64>,
}

fn manual_move(c: &Control, movement: AgentMovement, ms: u64) -> Response {
    if !movement.is_finite() {
        return bad("non-finite movement");
    }
    c.manual(movement, Duration::from_millis(ms));
    text(
        StatusCode::OK,
        format!(
            "manual move dir=({:.1},{:.1}) up={:.1} jump={} for {}ms",
            movement.dir[0],
            movement.dir[1],
            movement.up,
            movement.jump,
            ms.min(crate::driver::MANUAL_MAX_MS)
        ),
    )
}

async fn manual(State(c): State<Ctl>, Body(b): Body<ManualBody>) -> Response {
    let obs = match live_alive(&c) {
        Ok(o) => o,
        Err(r) => return r,
    };
    if obs.own.held {
        return text(
            StatusCode::CONFLICT,
            "the character is held; nothing was queued",
        );
    }
    let dir = [b.east.unwrap_or(0.0), b.north.unwrap_or(0.0)];
    let up = b.up.unwrap_or(0.0).clamp(-1.0, 1.0);
    let jump = b.jump.unwrap_or(false);
    if dir == [0.0, 0.0] && up == 0.0 && !jump {
        return bad("provide a direction {east,north}, {up:-1..1} (swim), and/or {\"jump\":true}");
    }
    let ms = b.duration_ms.unwrap_or(MANUAL_DEFAULT_MS);
    if ms == 0 {
        return bad("duration_ms must be at least 1");
    }
    manual_move(
        &c,
        AgentMovement {
            dir,
            up,
            jump,
            wish_heading: None,
        },
        ms,
    )
}

async fn jump(State(c): State<Ctl>) -> Response {
    let obs = match live_alive(&c) {
        Ok(o) => o,
        Err(r) => return r,
    };
    if obs.own.held {
        return text(
            StatusCode::CONFLICT,
            "the character is held; nothing was queued",
        );
    }
    manual_move(
        &c,
        AgentMovement {
            dir: [0.0, 0.0],
            up: 0.0,
            jump: true,
            wish_heading: None,
        },
        MANUAL_DEFAULT_MS,
    )
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct IdBody {
    id: Option<u32>,
}

async fn target(State(c): State<Ctl>, Body(b): Body<IdBody>) -> Response {
    let obs = match live_alive(&c) {
        Ok(o) => o,
        Err(r) => return r,
    };
    let Some(id) = b.id else {
        return bad("provide {\"id\":<spawn_id>}");
    };
    if !obs.visible.iter().any(|e| e.spawn_id == id) {
        return text(
            StatusCode::NOT_FOUND,
            format!("no visible spawn with id {id}"),
        );
    }
    send(
        &c,
        AgentVerb::Combat(CombatVerb::Target { spawn_id: id }),
        format!("targeting {id}"),
    )
    .await
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct NameBody {
    name: Option<String>,
}

async fn target_name(State(c): State<Ctl>, Body(b): Body<NameBody>) -> Response {
    let obs = match live_alive(&c) {
        Ok(o) => o,
        Err(r) => return r,
    };
    let Some(name) = b.name else {
        return bad("provide {\"name\":\"...\"}");
    };
    let Some(e) = resolve(&obs, &name) else {
        return text(
            StatusCode::NOT_FOUND,
            format!("no visible entity matching '{name}'"),
        );
    };
    let verb = AgentVerb::Combat(CombatVerb::Target {
        spawn_id: e.spawn_id,
    });
    send_with(&c, verb, || {
        Json(json!({ "matched": matched(e) })).into_response()
    })
    .await
}

async fn attack_on(State(c): State<Ctl>) -> Response {
    if let Err(r) = live_alive(&c) {
        return r;
    }
    send(
        &c,
        AgentVerb::Combat(CombatVerb::Attack { on: true }),
        "auto-attack on",
    )
    .await
}

async fn attack_off(State(c): State<Ctl>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    send(
        &c,
        AgentVerb::Combat(CombatVerb::Attack { on: false }),
        "auto-attack off",
    )
    .await
}

async fn consider(State(c): State<Ctl>, Body(b): Body<IdBody>) -> Response {
    let obs = match live_alive(&c) {
        Ok(o) => o,
        Err(r) => return r,
    };
    let Some(id) = b.id.or(obs.own.target_id) else {
        return bad("no target; provide {\"id\":<spawn_id>}");
    };
    if !obs.visible.iter().any(|e| e.spawn_id == id) {
        return text(
            StatusCode::NOT_FOUND,
            format!("no visible spawn with id {id}"),
        );
    }
    send(
        &c,
        AgentVerb::Combat(CombatVerb::Consider { spawn_id: id }),
        format!("considering {id}"),
    )
    .await
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct CastBody {
    gem: Option<u8>,
    target_id: Option<u32>,
    item_slot: Option<u32>,
}

async fn cast(State(c): State<Ctl>, Body(b): Body<CastBody>) -> Response {
    if let Err(r) = live_alive(&c) {
        return r;
    }
    let Some(gem) = b.gem else {
        return bad("provide {\"gem\":0-8}");
    };
    if gem > 8 {
        return bad("gem must be 0-8");
    }
    send(
        &c,
        AgentVerb::Combat(CombatVerb::Cast(CastRequest {
            gem,
            target_id: b.target_id,
            item_slot: b.item_slot,
        })),
        format!("casting gem {gem}"),
    )
    .await
}

async fn sit(State(c): State<Ctl>) -> Response {
    if let Err(r) = live_alive(&c) {
        return r;
    }
    send(&c, AgentVerb::Interact(InteractVerb::Sit), "sitting").await
}

async fn stand(State(c): State<Ctl>) -> Response {
    if let Err(r) = live_alive(&c) {
        return r;
    }
    send(&c, AgentVerb::Interact(InteractVerb::Stand), "standing").await
}

async fn respawn(State(c): State<Ctl>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    send(
        &c,
        AgentVerb::Lifecycle(LifecycleVerb::Respawn),
        "respawn requested",
    )
    .await
}

/// Queues `verb` and answers with eqoxide's verdict on it: `200 ok` only once eqoxide says it took
/// the verb, so a refusal is never reported as success.
async fn send(c: &Control, verb: AgentVerb, ok: impl Into<String>) -> Response {
    send_with(c, verb, || text(StatusCode::OK, ok)).await
}

/// As [`send`], with `accepted` building the success response.
async fn send_with(c: &Control, verb: AgentVerb, accepted: impl FnOnce() -> Response) -> Response {
    match c.verb(verb).await {
        VerbReply::Accepted => accepted(),
        VerbReply::Refused(why) => {
            let code = if why.starts_with("no item") {
                StatusCode::NOT_FOUND
            } else {
                StatusCode::CONFLICT
            };
            text(code, why)
        }
        VerbReply::NotQueued => text(
            StatusCode::TOO_MANY_REQUESTS,
            "too many verbs queued; nothing was queued",
        ),
        VerbReply::Lost(why) => text(StatusCode::SERVICE_UNAVAILABLE, why),
    }
}

async fn camp(State(c): State<Ctl>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    send(
        &c,
        AgentVerb::Lifecycle(LifecycleVerb::Camp),
        "camp toggled",
    )
    .await
}

async fn exit(State(c): State<Ctl>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    send(
        &c,
        AgentVerb::Lifecycle(LifecycleVerb::Exit),
        "exit requested: camping out, then shutting down",
    )
    .await
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct MoveBody {
    from: u32,
    to: u32,
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct MaybeMoveBody {
    from: Option<u32>,
    to: Option<u32>,
}

async fn inventory_move(State(c): State<Ctl>, Body(b): Body<MaybeMoveBody>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    let MoveBody { from, to } = match b {
        MaybeMoveBody {
            from: Some(from),
            to: Some(to),
        } => MoveBody { from, to },
        _ => return bad("provide {\"from\":N,\"to\":M}"),
    };
    send(
        &c,
        AgentVerb::Inventory(InventoryVerb::Move { from, to }),
        format!("moving item from slot {from} to slot {to}"),
    )
    .await
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct MaybeSlotBody {
    slot: Option<u32>,
}

async fn inventory_destroy(State(c): State<Ctl>, Body(b): Body<MaybeSlotBody>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    let Some(slot) = b.slot else {
        return bad("provide {\"slot\":N}");
    };
    send(
        &c,
        AgentVerb::Inventory(InventoryVerb::Destroy { slot }),
        format!("destroying the item in slot {slot}"),
    )
    .await
}

async fn inventory_consume(State(c): State<Ctl>, Body(b): Body<MaybeSlotBody>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    let Some(slot) = b.slot else {
        return bad("provide {\"slot\":N}");
    };
    send(
        &c,
        AgentVerb::Inventory(InventoryVerb::Consume { slot }),
        format!("consuming the item in slot {slot}"),
    )
    .await
}

/// `PET_*` command bytes (EQEmu zone/common.h).
const PET_ATTACK: u8 = 2;
const PET_FOLLOWME: u8 = 4;
const PET_GUARDHERE: u8 = 5;
const PET_SIT: u8 = 6;
const PET_BACKOFF: u8 = 28;

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct PetBody {
    command: Option<u8>,
    name: Option<String>,
}

async fn pet_command(State(c): State<Ctl>, Body(b): Body<PetBody>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    // `command` and `name` are two spellings of one command: giving both is two different commands
    // or one redundant one, and silently picking either would hide which.
    let command = match (b.command, b.name.as_deref()) {
        (Some(_), Some(_)) => return bad("give either \"command\" or \"name\", not both"),
        (Some(n), None) => n,
        (None, Some(name)) => {
            match name.to_ascii_lowercase().as_str() {
                "attack" => PET_ATTACK,
                "backoff" | "back_off" => PET_BACKOFF,
                "follow" => PET_FOLLOWME,
                "guard" => PET_GUARDHERE,
                "sit" => PET_SIT,
                _ => return bad(
                    "unknown pet command; use a PET_* number or attack|backoff|follow|guard|sit",
                ),
            }
        }
        (None, None) => {
            return bad("provide {\"command\":N} or {\"name\":\"attack|backoff|follow|guard|sit\"}")
        }
    };
    send(
        &c,
        AgentVerb::Pet(PetVerb::Command { command }),
        format!("pet command {command} queued"),
    )
    .await
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct TellBody {
    to: Option<String>,
    text: Option<String>,
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct TextBody {
    text: Option<String>,
}

async fn chat_tell(State(c): State<Ctl>, Body(b): Body<TellBody>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    let (Some(to), Some(text_)) = (b.to, b.text) else {
        return bad("tell requires non-empty 'to' and 'text'");
    };
    if to.trim().is_empty() || text_.trim().is_empty() {
        return bad("tell requires non-empty 'to' and 'text'");
    }
    let ok = format!("tell queued to {to}");
    send(&c, AgentVerb::Chat(ChatVerb::Tell { to, text: text_ }), ok).await
}

/// A broadcast chat route: `text` must be non-blank, then `make` builds the verb.
async fn broadcast(c: &Control, name: &str, b: TextBody, make: fn(String) -> ChatVerb) -> Response {
    if let Err(r) = live(c) {
        return r;
    }
    let Some(t) = b.text.filter(|t| !t.trim().is_empty()) else {
        return bad(format!("{name} requires 'text'"));
    };
    send(c, AgentVerb::Chat(make(t)), format!("{name} queued")).await
}

async fn chat_ooc(State(c): State<Ctl>, Body(b): Body<TextBody>) -> Response {
    broadcast(&c, "ooc", b, |text| ChatVerb::Ooc { text }).await
}

async fn chat_shout(State(c): State<Ctl>, Body(b): Body<TextBody>) -> Response {
    broadcast(&c, "shout", b, |text| ChatVerb::Shout { text }).await
}

async fn chat_group(State(c): State<Ctl>, Body(b): Body<TextBody>) -> Response {
    broadcast(&c, "group", b, |text| ChatVerb::Group { text }).await
}

async fn chat_guild(State(c): State<Ctl>, Body(b): Body<TextBody>) -> Response {
    broadcast(&c, "guild", b, |text| ChatVerb::Guild { text }).await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver::{self, ChannelLink, Control};
    use axum::body::Body as AxBody;
    use axum::http::Request;
    use eqoxide_agent_protocol::observation::{LegalActionMask, OwnState};
    use eqoxide_agent_protocol::step::Step;
    use eqoxide_agent_protocol::verb_result::{VerbOutcome, VerbResult};
    use harness_nav::status::NavSlots;
    use std::sync::mpsc;
    use tower::ServiceExt;

    fn entity(id: u32, name: &str, pos: [f32; 3], dead: bool) -> VisibleEntity {
        VisibleEntity {
            spawn_id: id,
            name: name.into(),
            is_npc: true,
            level: 1,
            race: "Rat".into(),
            pos,
            heading: 0.0,
            hp_pct: 100.0,
            dead,
        }
    }

    fn obs(visible: Vec<VisibleEntity>) -> Observation {
        Observation {
            own: OwnState {
                pos: [0.0, 0.0, 0.0],
                heading: 0.0,
                hp: 100,
                hp_max: 100,
                hp_verified: true,
                mana: 0,
                mana_max: 0,
                endurance: 0,
                endurance_max: 0,
                endurance_confirmed: true,
                casting: None,
                buffs: vec![],
                zone_name: "qeynos".into(),
                target_id: None,
                target_name: None,
                auto_attack: false,
                sitting: false,
                held: false,
                player_class: "Warrior".into(),
                player_level: 1,
            },
            visible,
            legal_actions: LegalActionMask {
                gems: [false; 9],
                abilities: vec![],
            },
            dead: false,
            terminated: false,
            truncated: false,
            visibility_available: true,
            tick: 0,
            verb_results: vec![],
            call_replies: vec![],
        }
    }

    /// A control with `o` published and a driver that cannot load any zone.
    fn session(o: Observation) -> (Arc<Control>, driver::Driver) {
        let control = Control::new(NavSlots::default());
        control.publish(&o);
        let drv = driver::Driver::new(control.clone(), Box::new(|_| None));
        (control, drv)
    }

    async fn call(c: &Arc<Control>, method: &str, uri: &str, body: &str) -> (StatusCode, String) {
        let req = Request::builder()
            .method(method)
            .uri(uri)
            .body(AxBody::from(body.to_string()))
            .unwrap();
        let resp = crate::router(c.clone()).oneshot(req).await.unwrap();
        let code = resp.status();
        let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX)
            .await
            .unwrap();
        (code, String::from_utf8_lossy(&bytes).into_owned())
    }

    #[tokio::test]
    async fn every_route_is_503_before_the_first_observation() {
        let c = Control::new(NavSlots::default());
        for (m, u) in [
            ("GET", "/v1/observe/state"),
            ("POST", "/v1/move/goto"),
            ("POST", "/v1/combat/attack"),
            ("POST", "/v1/interact/sit"),
        ] {
            assert_eq!(
                call(&c, m, u, "").await.0,
                StatusCode::SERVICE_UNAVAILABLE,
                "{m} {u}"
            );
        }
    }

    #[tokio::test]
    async fn goto_by_name_prefers_a_live_exact_match_and_sets_the_goal() {
        let (c, _) = session(obs(vec![
            entity(1, "a_rat00", [100.0, 0.0, 0.0], false),
            entity(2, "a_rat", [300.0, 0.0, 0.0], false),
            entity(3, "a_rat01", [10.0, 0.0, 0.0], true),
        ]));
        let (code, body) = call(&c, "POST", "/v1/move/goto", r#"{"name":"a_rat"}"#).await;
        assert_eq!(code, StatusCode::OK, "{body}");
        let v: Value = serde_json::from_str(&body).unwrap();
        assert_eq!(v["matched"]["spawn_id"], 2);
        assert_eq!(c.nav_status().goal, Some([300.0, 0.0, 0.0]));
    }

    #[tokio::test]
    async fn goto_rejects_conflicting_partial_and_unknown_bodies_without_setting_a_goal() {
        let (c, _) = session(obs(vec![entity(1, "a_rat00", [1.0, 0.0, 0.0], false)]));
        for body in [
            r#"{"name":"a_rat","x":1,"y":2,"z":3}"#,
            r#"{"x":1,"y":2}"#,
            r#"{"zone_id":4}"#,
            "{not json",
        ] {
            assert_eq!(
                call(&c, "POST", "/v1/move/goto", body).await.0,
                StatusCode::BAD_REQUEST,
                "{body}"
            );
        }
        assert_eq!(c.nav_status().goal, None);
    }

    #[tokio::test]
    async fn goto_map_coordinates_negate_to_server_coordinates() {
        let (c, _) = session(obs(vec![]));
        let (code, _) = call(
            &c,
            "POST",
            "/v1/move/goto",
            r#"{"map_x":10,"map_y":-20,"z":5}"#,
        )
        .await;
        assert_eq!(code, StatusCode::OK);
        assert_eq!(c.nav_status().goal, Some([-10.0, 20.0, 5.0]));
    }

    #[tokio::test]
    async fn a_dead_character_is_refused_with_409() {
        let mut o = obs(vec![]);
        o.dead = true;
        let (c, _) = session(o);
        for (m, u, b) in [
            ("POST", "/v1/move/goto", r#"{"x":1,"y":1,"z":1}"#),
            ("POST", "/v1/move/manual", r#"{"east":1}"#),
            ("POST", "/v1/combat/attack", ""),
            ("POST", "/v1/interact/sit", ""),
        ] {
            assert_eq!(call(&c, m, u, b).await.0, StatusCode::CONFLICT, "{m} {u}");
        }
    }

    #[tokio::test]
    async fn follow_refuses_a_corpse_and_stop_clears_the_goal() {
        let (c, _) = session(obs(vec![
            entity(1, "a_rat00", [5.0, 0.0, 0.0], false),
            entity(2, "a_bat00", [9.0, 0.0, 0.0], true),
        ]));
        assert_eq!(
            call(&c, "POST", "/v1/move/follow", r#"{"name":"a_bat"}"#)
                .await
                .0,
            StatusCode::CONFLICT
        );
        assert_eq!(
            call(&c, "POST", "/v1/move/follow", r#"{"name":"a_rat"}"#)
                .await
                .0,
            StatusCode::OK
        );
        assert!(c.nav_status().goal.is_some());
        assert_eq!(
            call(&c, "POST", "/v1/move/stop", "").await.0,
            StatusCode::OK
        );
    }

    #[tokio::test]
    async fn a_verb_route_rides_the_next_step_exactly_once() {
        let world = obs(vec![entity(7, "a_rat00", [5.0, 0.0, 0.0], false)]);
        let (c, mut drv) = session(world.clone());
        assert_eq!(
            call(&c, "POST", "/v1/combat/target", r#"{"id":99}"#)
                .await
                .0,
            StatusCode::NOT_FOUND
        );
        assert_eq!(
            call(&c, "POST", "/v1/combat/cast", r#"{"gem":9}"#).await.0,
            StatusCode::BAD_REQUEST
        );
        let (code, body, verb) = answered_in(
            &world,
            &c,
            &mut drv,
            "/v1/combat/target",
            r#"{"id":7}"#,
            Some(VerbOutcome::Accepted),
        )
        .await;
        assert_eq!(code, StatusCode::OK, "{body}");
        assert_eq!(
            verb,
            Some(AgentVerb::Combat(CombatVerb::Target { spawn_id: 7 }))
        );
        assert_eq!(drv.tick(&world).verb, None);

        // A verb eqoxide refuses is a 409 carrying its reason, never a 200.
        let (code, body, _) = answered_in(
            &world,
            &c,
            &mut drv,
            "/v1/combat/attack",
            "",
            Some(VerbOutcome::Refused("already attacking".into())),
        )
        .await;
        assert_eq!(code, StatusCode::CONFLICT, "{body}");
        assert_eq!(body, "already attacking");
    }

    #[tokio::test]
    async fn a_manual_move_drives_then_stands_still() {
        let (c, mut drv) = session(obs(vec![]));
        let (code, _) = call(
            &c,
            "POST",
            "/v1/move/manual",
            r#"{"east":1,"duration_ms":300}"#,
        )
        .await;
        assert_eq!(code, StatusCode::OK);
        assert_eq!(
            call(
                &c,
                "POST",
                "/v1/move/manual",
                r#"{"east":1,"duration_ms":0}"#
            )
            .await
            .0,
            StatusCode::BAD_REQUEST
        );
        let o = obs(vec![]);
        assert_eq!(drv.tick(&o).movement.unwrap().dir, [1.0, 0.0]);
        std::thread::sleep(Duration::from_millis(600));
        let stop = drv.tick(&o).movement.unwrap();
        assert_eq!((stop.dir, stop.jump), ([0.0, 0.0], false));
        assert_eq!(drv.tick(&o).movement, None);
    }

    #[tokio::test]
    async fn manual_with_nothing_to_do_and_a_held_character_are_refused() {
        let (c, _) = session(obs(vec![]));
        assert_eq!(
            call(&c, "POST", "/v1/move/manual", "{}").await.0,
            StatusCode::BAD_REQUEST
        );
        let mut held = obs(vec![]);
        held.own.held = true;
        let (c, _) = session(held);
        assert_eq!(
            call(&c, "POST", "/v1/move/jump", "").await.0,
            StatusCode::CONFLICT
        );
    }

    #[tokio::test]
    async fn auto_attack_engages_the_target_and_stands_still_once_it_is_off() {
        let (_, mut drv) = session(obs(vec![]));
        let mut o = obs(vec![entity(7, "a_rat00", [50.0, 0.0, 0.0], false)]);
        o.own.auto_attack = true;
        o.own.target_id = Some(7);
        assert!(drv.tick(&o).movement.unwrap().dir[0] > 0.0);
        o.own.auto_attack = false;
        assert_eq!(drv.tick(&o).movement.unwrap().dir, [0.0, 0.0]);
        assert_eq!(drv.tick(&o).movement, None);
    }

    #[test]
    fn the_driver_thread_answers_each_observation_and_reports_the_end_of_the_session() {
        let (obs_tx, obs_rx) = mpsc::channel();
        let (step_tx, step_rx) = mpsc::channel();
        let control = Control::new(NavSlots::default());
        let handle = driver::spawn(
            ChannelLink {
                observations: obs_rx,
                steps: step_tx,
            },
            control.clone(),
            Box::new(|_| None),
        );
        obs_tx.send(obs(vec![])).unwrap();
        let step: Step = step_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        assert_eq!(step.verb, None);
        assert!(control.latest().is_some());
        assert_eq!(control.ended(), None);
        drop(obs_tx);
        handle.join().unwrap();
        assert!(control
            .ended()
            .unwrap()
            .contains("observation stream ended"));
    }

    #[tokio::test]
    async fn zone_lines_is_409_while_the_zone_is_not_loaded_and_503_before_any_observation() {
        let c = Control::new(NavSlots::default());
        assert_eq!(
            call(&c, "GET", "/v1/observe/zone_lines", "").await.0,
            StatusCode::SERVICE_UNAVAILABLE
        );
        let (c, mut drv) = session(obs(vec![]));
        drv.tick(&obs(vec![]));
        let (code, body) = call(&c, "GET", "/v1/observe/zone_lines", "").await;
        assert_eq!(code, StatusCode::CONFLICT, "{body}");
    }

    #[tokio::test]
    async fn a_blank_name_matches_nothing_and_partial_raw_plus_map_is_a_conflict() {
        let (c, _) = session(obs(vec![entity(1, "a_rat00", [1.0, 0.0, 0.0], false)]));
        assert_eq!(
            call(&c, "POST", "/v1/combat/target/name", r#"{"name":"  "}"#)
                .await
                .0,
            StatusCode::NOT_FOUND
        );
        assert_eq!(
            call(
                &c,
                "POST",
                "/v1/move/goto",
                r#"{"x":1,"map_x":2,"map_y":3}"#
            )
            .await
            .0,
            StatusCode::BAD_REQUEST
        );
    }

    /// Sends `uri` and plays eqoxide: the verb it queues rides the next `Step` (returned with it),
    /// and the following `Observation` answers it with `outcome` when that is `Some`.
    async fn answered(
        c: &Arc<Control>,
        drv: &mut driver::Driver,
        uri: &str,
        body: &str,
        outcome: Option<VerbOutcome>,
    ) -> (StatusCode, String, Option<AgentVerb>) {
        answered_in(&obs(vec![]), c, drv, uri, body, outcome).await
    }

    /// As [`answered`], with `world` as the observation each step sees.
    async fn answered_in(
        world: &Observation,
        c: &Arc<Control>,
        drv: &mut driver::Driver,
        uri: &str,
        body: &str,
        outcome: Option<VerbOutcome>,
    ) -> (StatusCode, String, Option<AgentVerb>) {
        let (c2, uri, body) = (c.clone(), uri.to_string(), body.to_string());
        let req = tokio::spawn(async move { call(&c2, "POST", &uri, &body).await });
        let (verb, seq) = loop {
            let step = drv.tick(world);
            if step.verb.is_some() || req.is_finished() {
                break (step.verb, step.seq);
            }
            tokio::task::yield_now().await;
        };
        if let (Some(outcome), Some(seq)) = (outcome, seq) {
            let mut o = world.clone();
            o.verb_results = vec![VerbResult { seq, outcome }];
            drv.tick(&o);
        }
        let (code, body) = req.await.unwrap();
        (code, body, verb)
    }

    #[tokio::test]
    async fn a_refused_verb_is_a_409_and_an_accepted_one_is_a_200() {
        let (c, mut drv) = session(obs(vec![]));
        let (code, body, verb) = answered(
            &c,
            &mut drv,
            "/v1/inventory/move",
            r#"{"from":23,"to":30}"#,
            Some(VerbOutcome::Refused("no item in slot 23 to move".into())),
        )
        .await;
        assert_eq!(code, StatusCode::NOT_FOUND, "{body}");
        assert!(body.contains("no item in slot 23"));
        assert_eq!(
            verb,
            Some(AgentVerb::Inventory(InventoryVerb::Move {
                from: 23,
                to: 30
            }))
        );
        let (code, body, _) = answered(
            &c,
            &mut drv,
            "/v1/pet/command",
            r#"{"name":"attack"}"#,
            Some(VerbOutcome::Refused(
                "a pet command is already queued".into(),
            )),
        )
        .await;
        assert_eq!(code, StatusCode::CONFLICT, "{body}");
        let (code, _, verb) = answered(
            &c,
            &mut drv,
            "/v1/pet/command",
            r#"{"name":"Backoff"}"#,
            Some(VerbOutcome::Accepted),
        )
        .await;
        assert_eq!(code, StatusCode::OK);
        assert_eq!(verb, Some(AgentVerb::Pet(PetVerb::Command { command: 28 })));
    }

    #[tokio::test]
    async fn a_verb_dropped_on_a_zone_change_is_503_not_ok() {
        let (c, mut drv) = session(obs(vec![]));
        // The first zone of a session is not a change, so settle it before queueing.
        drv.tick(&obs(vec![]));
        let c2 = c.clone();
        let req = tokio::spawn(async move { call(&c2, "POST", "/v1/lifecycle/camp", "").await });
        // Wait for the verb to be queued, then change zone before it is sent.
        while c.verb_queue_len() == 0 {
            tokio::task::yield_now().await;
        }
        let mut other = obs(vec![]);
        other.own.zone_name = "elsewhere".into();
        assert_eq!(drv.tick(&other).verb, None);
        let (code, body) = req.await.unwrap();
        assert_eq!(code, StatusCode::SERVICE_UNAVAILABLE, "{body}");
    }

    #[tokio::test]
    async fn chat_and_pet_bodies_are_validated_before_anything_is_queued() {
        let (c, mut drv) = session(obs(vec![]));
        for (uri, body) in [
            ("/v1/chat/tell", r#"{"to":"  ","text":"hi"}"#),
            ("/v1/chat/tell", r#"{"text":"hi"}"#),
            ("/v1/chat/ooc", r#"{"text":" "}"#),
            ("/v1/chat/guild", ""),
            ("/v1/pet/command", r#"{"command":2,"name":"sit"}"#),
            ("/v1/pet/command", r#"{"name":"dance"}"#),
            ("/v1/pet/command", ""),
            ("/v1/inventory/move", r#"{"from":23}"#),
            ("/v1/inventory/destroy", ""),
            ("/v1/inventory/consume", r#"{"slot":"x"}"#),
        ] {
            assert_eq!(
                call(&c, "POST", uri, body).await.0,
                StatusCode::BAD_REQUEST,
                "{uri} {body}"
            );
        }
        assert_eq!(drv.tick(&obs(vec![])).verb, None);
    }

    #[tokio::test]
    async fn destroy_and_consume_queue_their_verbs_and_a_refusal_is_a_404_or_409() {
        let (c, mut drv) = session(obs(vec![]));
        let (code, body, verb) = answered(
            &c,
            &mut drv,
            "/v1/inventory/destroy",
            r#"{"slot":24}"#,
            Some(VerbOutcome::Accepted),
        )
        .await;
        assert_eq!(code, StatusCode::OK, "{body}");
        assert_eq!(
            verb,
            Some(AgentVerb::Inventory(InventoryVerb::Destroy { slot: 24 }))
        );
        let (code, body, verb) = answered(
            &c,
            &mut drv,
            "/v1/inventory/consume",
            r#"{"slot":25}"#,
            Some(VerbOutcome::Refused("no item in slot 25 to consume".into())),
        )
        .await;
        assert_eq!(code, StatusCode::NOT_FOUND, "{body}");
        assert_eq!(
            verb,
            Some(AgentVerb::Inventory(InventoryVerb::Consume { slot: 25 }))
        );
        let (code, body, _) = answered(
            &c,
            &mut drv,
            "/v1/inventory/consume",
            r#"{"slot":26}"#,
            Some(VerbOutcome::Refused(
                "Bread in slot 26 is not food or drink".into(),
            )),
        )
        .await;
        assert_eq!(code, StatusCode::CONFLICT, "{body}");
    }

    #[tokio::test]
    async fn a_chat_route_queues_the_matching_verb() {
        let (c, mut drv) = session(obs(vec![]));
        let (code, _, verb) = answered(
            &c,
            &mut drv,
            "/v1/chat/tell",
            r#"{"to":"Bob","text":"hi"}"#,
            Some(VerbOutcome::Accepted),
        )
        .await;
        assert_eq!(code, StatusCode::OK);
        assert_eq!(
            verb,
            Some(AgentVerb::Chat(ChatVerb::Tell {
                to: "Bob".into(),
                text: "hi".into()
            }))
        );
    }

    #[tokio::test]
    async fn every_new_verb_route_is_503_before_the_first_observation() {
        let c = Control::new(NavSlots::default());
        for uri in [
            "/v1/lifecycle/camp",
            "/v1/lifecycle/exit",
            "/v1/inventory/move",
            "/v1/inventory/destroy",
            "/v1/inventory/consume",
            "/v1/pet/command",
            "/v1/chat/tell",
            "/v1/chat/ooc",
        ] {
            assert_eq!(
                call(&c, "POST", uri, "{}").await.0,
                StatusCode::SERVICE_UNAVAILABLE,
                "{uri}"
            );
        }
    }

    #[tokio::test]
    async fn the_verb_queue_is_bounded_and_dropped_on_death() {
        let (c, mut drv) = session(obs(vec![entity(7, "a_rat00", [5.0, 0.0, 0.0], false)]));
        // Each waiting request holds one queue slot until eqoxide answers it.
        let waiting: Vec<_> = (0..driver::MAX_QUEUED_VERBS)
            .map(|_| {
                let c = c.clone();
                tokio::spawn(async move { call(&c, "POST", "/v1/combat/attack", "").await })
            })
            .collect();
        while c.verb_queue_len() < driver::MAX_QUEUED_VERBS {
            tokio::task::yield_now().await;
        }
        assert_eq!(
            call(&c, "POST", "/v1/combat/attack", "").await.0,
            StatusCode::TOO_MANY_REQUESTS
        );
        let mut dead = obs(vec![]);
        dead.dead = true;
        assert_eq!(drv.tick(&dead).verb, None);
        assert_eq!(drv.tick(&obs(vec![])).verb, None);
        // The dropped verbs' callers are told, not left to time out.
        for request in waiting {
            let (code, body) = request.await.unwrap();
            assert_eq!(code, StatusCode::SERVICE_UNAVAILABLE, "{body}");
        }
    }

    #[tokio::test]
    async fn stop_ends_a_manual_move_and_goto_overrides_one() {
        let (c, mut drv) = session(obs(vec![]));
        c.manual(
            AgentMovement {
                dir: [0.0, 1.0],
                up: 0.0,
                jump: false,
                wish_heading: None,
            },
            std::time::Duration::from_secs(5),
        );
        assert_eq!(
            call(&c, "POST", "/v1/move/stop", "").await.0,
            StatusCode::OK
        );
        // The manual move is gone: the tick after a stop does not keep running the character.
        assert_ne!(
            drv.tick(&obs(vec![])).movement.map(|m| m.dir),
            Some([0.0, 1.0])
        );
        c.manual(
            AgentMovement {
                dir: [0.0, 1.0],
                up: 0.0,
                jump: false,
                wish_heading: None,
            },
            std::time::Duration::from_secs(5),
        );
        assert_eq!(
            call(&c, "POST", "/v1/move/goto", r#"{"x":1,"y":2,"z":0}"#)
                .await
                .0,
            StatusCode::OK
        );
        assert_ne!(
            drv.tick(&obs(vec![])).movement.map(|m| m.dir),
            Some([0.0, 1.0])
        );
    }

    #[tokio::test]
    async fn a_zone_that_failed_to_load_is_retried_when_the_character_comes_back_to_the_old_one() {
        let control = Control::new(NavSlots::default());
        let loads = Arc::new(std::sync::Mutex::new(Vec::<String>::new()));
        let seen = loads.clone();
        let mut drv = driver::Driver::new(
            control,
            Box::new(move |z| {
                seen.lock().unwrap().push(z.to_string());
                None
            }),
        );
        let in_zone = |drv: &mut driver::Driver, name: &str| {
            let mut o = obs(vec![]);
            o.own.zone_name = name.into();
            drv.tick(&o);
        };
        in_zone(&mut drv, "qeynos");
        in_zone(&mut drv, "freportw");
        in_zone(&mut drv, "qeynos");
        assert_eq!(*loads.lock().unwrap(), vec!["qeynos", "freportw", "qeynos"]);
    }

    #[tokio::test]
    async fn an_observation_is_visible_only_after_the_driver_has_dropped_stale_verbs() {
        let control = Control::new(NavSlots::default());
        let mut drv = driver::Driver::new(control.clone(), Box::new(|_| None));
        drv.tick(&obs(vec![]));
        assert!(control.push_verb(AgentVerb::Interact(
            eqoxide_agent_protocol::verb::InteractVerb::Sit
        )));
        let mut dead = obs(vec![]);
        dead.dead = true;
        // The tick that sees the death clears the old verb and only then publishes it.
        let step = drv.tick(&dead);
        assert_eq!(step.verb, None);
        assert!(control.latest().unwrap().dead);
        // A verb queued in response to that observation is kept and sent on the next tick.
        assert!(control.push_verb(AgentVerb::Lifecycle(
            eqoxide_agent_protocol::verb::LifecycleVerb::Respawn
        )));
        assert!(drv.tick(&dead).verb.is_some());
    }

    #[tokio::test]
    async fn an_unserved_route_is_forwarded_as_a_call_and_answered_with_eqoxides_reply() {
        use eqoxide_agent_protocol::call::{CallReply, CallStatus};
        let (c, mut drv) = session(obs(vec![]));
        let c2 = c.clone();
        let request = tokio::spawn(async move {
            call(
                &c2,
                "POST",
                "/v1/merchant/buy?x=1",
                "{\"from\":23,\"to\":30}",
            )
            .await
        });
        // Tick until the call rides out in a Step.
        let step = loop {
            let step = drv.tick(&obs(vec![]));
            if !step.calls.is_empty() {
                break step;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert_eq!(step.calls.len(), 1);
        assert_eq!(step.calls[0].op, "POST /v1/merchant/buy");
        assert_eq!(
            step.calls[0].args,
            json!({"query": {"x": "1"}, "body": {"from": 23, "to": 30}})
        );
        let mut reply_obs = obs(vec![]);
        reply_obs.call_replies = vec![CallReply {
            id: step.calls[0].id,
            status: CallStatus::Conflict,
            body: json!("a move is already queued"),
        }];
        drv.tick(&reply_obs);
        let (code, body) = request.await.unwrap();
        assert_eq!(code, StatusCode::CONFLICT);
        assert_eq!(body, "\"a move is already queued\"");
    }

    #[tokio::test]
    async fn a_forwarded_call_is_refused_when_the_session_has_ended() {
        let (c, _) = session(obs(vec![]));
        c.end_for_test("link down");
        let (code, _) = call(&c, "GET", "/v1/observe/skills", "").await;
        assert_eq!(code, StatusCode::SERVICE_UNAVAILABLE);
    }
}
