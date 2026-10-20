//! The agent control routes, under the same `/v1/<group>/<action>` paths eqoxide's own HTTP API
//! used. Every route answers from the latest `Observation` or queues intent for the driver.

use crate::driver::Control;
use axum::extract::State;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::{delete, get, post};
use axum::{Json, Router};
use eqoxide_agent_protocol::movement::AgentMovement;
use eqoxide_agent_protocol::observation::{Observation, VisibleEntity};
use eqoxide_agent_protocol::verb::{AgentVerb, CastRequest, CombatVerb, InteractVerb, LifecycleVerb};
use harness_nav::status::{AggroAvoidOpts, NavStatus};
use harness_nav::walker::ZoneCrossError;
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
        .route("/v1/move/goto", post(goto))
        .route("/v1/move/follow", post(follow))
        .route("/v1/move/stop", post(stop))
        .route("/v1/move/zone_cross", post(zone_cross))
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
        serde_json::from_slice(&bytes).map(Body).map_err(|e| bad(format!("malformed body: {e}")))
    }
}

/// The latest observation, or the refusal for a session that has none or has ended.
fn live(c: &Control) -> Result<Observation, Response> {
    if let Some(why) = c.ended() {
        return Err(text(StatusCode::SERVICE_UNAVAILABLE, format!("no live session: {why}")));
    }
    c.latest().ok_or_else(|| text(StatusCode::SERVICE_UNAVAILABLE, "no observation yet"))
}

/// As [`live`], and refuses while the character is dead.
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
    let needle = name.to_lowercase();
    let nearest = |pred: &dyn Fn(&VisibleEntity) -> bool| {
        obs.visible.iter().filter(|e| pred(e)).min_by(|a, b| dist(a.pos, obs.own.pos).total_cmp(&dist(b.pos, obs.own.pos)))
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
    Json(nav_json(&c.nav_status(), c.take_notices())).into_response()
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
    c.slots().set_avoid(AggroAvoidOpts { enabled: enabled.unwrap_or(current.enabled), buffer: buffer.unwrap_or(current.buffer) });
}

fn target_entity<'a>(obs: &'a Observation) -> Option<&'a VisibleEntity> {
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
    if have_raw == 3 && have_map > 0 {
        return bad("conflicting target: give raw {x,y,z} or {map_x,map_y}, not both");
    }
    let (goal, entity): ((f32, f32, f32), Option<&VisibleEntity>) = if let Some(name) = &b.name {
        match resolve(&obs, name) {
            Some(e) => ((e.pos[0], e.pos[1], e.pos[2]), Some(e)),
            None => return text(StatusCode::NOT_FOUND, format!("no visible entity matching '{name}'")),
        }
    } else if let (Some(x), Some(y), Some(z)) = (b.x, b.y, b.z) {
        ((x, y, z), None)
    } else if have_map == 2 {
        ((-b.map_x.unwrap(), -b.map_y.unwrap(), b.z.unwrap_or(obs.own.pos[2])), None)
    } else if have_raw + have_map > 0 {
        return bad("partial target: give a complete {x,y,z} or {map_x,map_y}");
    } else {
        match target_entity(&obs) {
            Some(e) => ((e.pos[0], e.pos[1], e.pos[2]), Some(e)),
            None => return bad("no target; provide a name or coordinates"),
        }
    };
    apply_avoid(&c, b.avoid_aggro, b.aggro_buffer);
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
            None => return text(StatusCode::NOT_FOUND, format!("no visible entity matching '{name}'")),
        },
        None => match target_entity(&obs) {
            Some(e) => e,
            None => return bad("no target; provide a name"),
        },
    };
    if entity.dead {
        return text(StatusCode::CONFLICT, format!("'{}' is dead; nothing to follow", entity.name));
    }
    apply_avoid(&c, b.avoid_aggro, b.aggro_buffer);
    let goal_id = c.slots().follow(&entity.name, (entity.pos[0], entity.pos[1], entity.pos[2]));
    Json(json!({ "goal_id": goal_id, "matched": matched(entity) })).into_response()
}

async fn stop(State(c): State<Ctl>) -> Response {
    c.slots().stop();
    text(StatusCode::OK, "stopped")
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct ZoneCrossBody {
    /// A zone-line region index from the zone's region data. Absent picks the nearest line.
    line: Option<i32>,
}

async fn zone_cross(State(c): State<Ctl>, Body(b): Body<ZoneCrossBody>) -> Response {
    if let Err(r) = live_alive(&c) {
        return r;
    }
    let cc = c.clone();
    let answer = tokio::task::spawn_blocking(move || cc.zone_cross(b.line)).await.ok().flatten();
    match answer {
        Some(Ok(goal_id)) => Json(json!({ "goal_id": goal_id })).into_response(),
        Some(Err(ZoneCrossError::ZoneNotLoaded)) => text(StatusCode::CONFLICT, "the zone's collision is not loaded"),
        Some(Err(ZoneCrossError::NoRegionData)) => text(StatusCode::CONFLICT, "the zone's region data is missing, so its exits are unknown"),
        Some(Err(ZoneCrossError::NoZoneLines)) => text(StatusCode::NOT_FOUND, "the zone has no zone line"),
        None => text(StatusCode::SERVICE_UNAVAILABLE, "the driver did not answer"),
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
    text(StatusCode::OK, format!("manual move dir=({:.1},{:.1}) up={:.1} jump={} for {}ms", movement.dir[0], movement.dir[1], movement.up, movement.jump, ms.min(crate::driver::MANUAL_MAX_MS)))
}

async fn manual(State(c): State<Ctl>, Body(b): Body<ManualBody>) -> Response {
    let obs = match live_alive(&c) {
        Ok(o) => o,
        Err(r) => return r,
    };
    if obs.own.held {
        return text(StatusCode::CONFLICT, "the character is held; nothing was queued");
    }
    let dir = [b.east.unwrap_or(0.0), b.north.unwrap_or(0.0)];
    let up = b.up.unwrap_or(0.0).clamp(-1.0, 1.0);
    let jump = b.jump.unwrap_or(false);
    if dir == [0.0, 0.0] && up == 0.0 && !jump {
        return bad("provide a direction {east,north}, {up:-1..1} (swim), and/or {\"jump\":true}");
    }
    manual_move(&c, AgentMovement { dir, up, jump, wish_heading: None }, b.duration_ms.unwrap_or(MANUAL_DEFAULT_MS))
}

async fn jump(State(c): State<Ctl>) -> Response {
    let obs = match live_alive(&c) {
        Ok(o) => o,
        Err(r) => return r,
    };
    if obs.own.held {
        return text(StatusCode::CONFLICT, "the character is held; nothing was queued");
    }
    manual_move(&c, AgentMovement { dir: [0.0, 0.0], up: 0.0, jump: true, wish_heading: None }, MANUAL_DEFAULT_MS)
}

fn queue(c: &Control, verb: AgentVerb, ok: impl Into<String>) -> Response {
    c.push_verb(verb);
    text(StatusCode::OK, ok)
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
    let Some(id) = b.id else { return bad("provide {\"id\":<spawn_id>}") };
    if !obs.visible.iter().any(|e| e.spawn_id == id) {
        return text(StatusCode::NOT_FOUND, format!("no visible spawn with id {id}"));
    }
    queue(&c, AgentVerb::Combat(CombatVerb::Target { spawn_id: id }), format!("targeting {id}"))
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
    let Some(name) = b.name else { return bad("provide {\"name\":\"...\"}") };
    let Some(e) = resolve(&obs, &name) else {
        return text(StatusCode::NOT_FOUND, format!("no visible entity matching '{name}'"));
    };
    c.push_verb(AgentVerb::Combat(CombatVerb::Target { spawn_id: e.spawn_id }));
    Json(json!({ "matched": matched(e) })).into_response()
}

async fn attack_on(State(c): State<Ctl>) -> Response {
    if let Err(r) = live_alive(&c) {
        return r;
    }
    queue(&c, AgentVerb::Combat(CombatVerb::Attack { on: true }), "auto-attack on")
}

async fn attack_off(State(c): State<Ctl>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    queue(&c, AgentVerb::Combat(CombatVerb::Attack { on: false }), "auto-attack off")
}

async fn consider(State(c): State<Ctl>, Body(b): Body<IdBody>) -> Response {
    let obs = match live_alive(&c) {
        Ok(o) => o,
        Err(r) => return r,
    };
    let Some(id) = b.id.or(obs.own.target_id) else { return bad("no target; provide {\"id\":<spawn_id>}") };
    if !obs.visible.iter().any(|e| e.spawn_id == id) {
        return text(StatusCode::NOT_FOUND, format!("no visible spawn with id {id}"));
    }
    queue(&c, AgentVerb::Combat(CombatVerb::Consider { spawn_id: id }), format!("considering {id}"))
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
    let Some(gem) = b.gem else { return bad("provide {\"gem\":0-8}") };
    if gem > 8 {
        return bad("gem must be 0-8");
    }
    queue(&c, AgentVerb::Combat(CombatVerb::Cast(CastRequest { gem, target_id: b.target_id, item_slot: b.item_slot })), format!("casting gem {gem}"))
}

async fn sit(State(c): State<Ctl>) -> Response {
    if let Err(r) = live_alive(&c) {
        return r;
    }
    queue(&c, AgentVerb::Interact(InteractVerb::Sit), "sitting")
}

async fn stand(State(c): State<Ctl>) -> Response {
    if let Err(r) = live_alive(&c) {
        return r;
    }
    queue(&c, AgentVerb::Interact(InteractVerb::Stand), "standing")
}

async fn respawn(State(c): State<Ctl>) -> Response {
    if let Err(r) = live(&c) {
        return r;
    }
    queue(&c, AgentVerb::Lifecycle(LifecycleVerb::Respawn), "respawn requested")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver::{self, ChannelLink, Control};
    use axum::body::Body as AxBody;
    use axum::http::Request;
    use eqoxide_agent_protocol::observation::{LegalActionMask, OwnState};
    use eqoxide_agent_protocol::step::Step;
    use harness_nav::status::NavSlots;
    use std::sync::mpsc;
    use tower::ServiceExt;

    fn entity(id: u32, name: &str, pos: [f32; 3], dead: bool) -> VisibleEntity {
        VisibleEntity { spawn_id: id, name: name.into(), is_npc: true, level: 1, race: "Rat".into(), pos, heading: 0.0, hp_pct: 100.0, dead }
    }

    fn obs(visible: Vec<VisibleEntity>) -> Observation {
        Observation {
            own: OwnState {
                pos: [0.0, 0.0, 0.0], heading: 0.0, hp: 100, hp_max: 100, hp_verified: true,
                mana: 0, mana_max: 0, endurance: 0, endurance_max: 0, endurance_confirmed: true,
                casting: None, buffs: vec![], zone_name: "qeynos".into(), target_id: None, target_name: None,
                auto_attack: false, sitting: false, held: false, player_class: "Warrior".into(), player_level: 1,
            },
            visible,
            legal_actions: LegalActionMask { gems: [false; 9], abilities: vec![] },
            dead: false, terminated: false, truncated: false, visibility_available: true, tick: 0,
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
        let req = Request::builder().method(method).uri(uri).body(AxBody::from(body.to_string())).unwrap();
        let resp = crate::router(c.clone()).oneshot(req).await.unwrap();
        let code = resp.status();
        let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX).await.unwrap();
        (code, String::from_utf8_lossy(&bytes).into_owned())
    }

    #[tokio::test]
    async fn every_route_is_503_before_the_first_observation() {
        let c = Control::new(NavSlots::default());
        for (m, u) in [("GET", "/v1/observe/state"), ("POST", "/v1/move/goto"), ("POST", "/v1/combat/attack"), ("POST", "/v1/interact/sit")] {
            assert_eq!(call(&c, m, u, "").await.0, StatusCode::SERVICE_UNAVAILABLE, "{m} {u}");
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
        for body in [r#"{"name":"a_rat","x":1,"y":2,"z":3}"#, r#"{"x":1,"y":2}"#, r#"{"zone_id":4}"#, "{not json"] {
            assert_eq!(call(&c, "POST", "/v1/move/goto", body).await.0, StatusCode::BAD_REQUEST, "{body}");
        }
        assert_eq!(c.nav_status().goal, None);
    }

    #[tokio::test]
    async fn goto_map_coordinates_negate_to_server_coordinates() {
        let (c, _) = session(obs(vec![]));
        let (code, _) = call(&c, "POST", "/v1/move/goto", r#"{"map_x":10,"map_y":-20,"z":5}"#).await;
        assert_eq!(code, StatusCode::OK);
        assert_eq!(c.nav_status().goal, Some([-10.0, 20.0, 5.0]));
    }

    #[tokio::test]
    async fn a_dead_character_is_refused_with_409() {
        let mut o = obs(vec![]);
        o.dead = true;
        let (c, _) = session(o);
        for (m, u, b) in [("POST", "/v1/move/goto", r#"{"x":1,"y":1,"z":1}"#), ("POST", "/v1/move/manual", r#"{"east":1}"#), ("POST", "/v1/combat/attack", ""), ("POST", "/v1/interact/sit", "")] {
            assert_eq!(call(&c, m, u, b).await.0, StatusCode::CONFLICT, "{m} {u}");
        }
    }

    #[tokio::test]
    async fn follow_refuses_a_corpse_and_stop_clears_the_goal() {
        let (c, _) = session(obs(vec![entity(1, "a_rat00", [5.0, 0.0, 0.0], false), entity(2, "a_bat00", [9.0, 0.0, 0.0], true)]));
        assert_eq!(call(&c, "POST", "/v1/move/follow", r#"{"name":"a_bat"}"#).await.0, StatusCode::CONFLICT);
        assert_eq!(call(&c, "POST", "/v1/move/follow", r#"{"name":"a_rat"}"#).await.0, StatusCode::OK);
        assert!(c.nav_status().goal.is_some());
        assert_eq!(call(&c, "POST", "/v1/move/stop", "").await.0, StatusCode::OK);
    }

    #[tokio::test]
    async fn a_verb_route_rides_the_next_step_exactly_once() {
        let (c, mut drv) = session(obs(vec![entity(7, "a_rat00", [5.0, 0.0, 0.0], false)]));
        assert_eq!(call(&c, "POST", "/v1/combat/target", r#"{"id":99}"#).await.0, StatusCode::NOT_FOUND);
        assert_eq!(call(&c, "POST", "/v1/combat/target", r#"{"id":7}"#).await.0, StatusCode::OK);
        assert_eq!(call(&c, "POST", "/v1/combat/cast", r#"{"gem":9}"#).await.0, StatusCode::BAD_REQUEST);
        let o = obs(vec![]);
        assert_eq!(drv.tick(&o).verb, Some(AgentVerb::Combat(CombatVerb::Target { spawn_id: 7 })));
        assert_eq!(drv.tick(&o).verb, None);
    }

    #[tokio::test]
    async fn a_manual_move_drives_then_stands_still() {
        let (c, mut drv) = session(obs(vec![]));
        let (code, _) = call(&c, "POST", "/v1/move/manual", r#"{"east":1,"duration_ms":30}"#).await;
        assert_eq!(code, StatusCode::OK);
        let o = obs(vec![]);
        assert_eq!(drv.tick(&o).movement.unwrap().dir, [1.0, 0.0]);
        std::thread::sleep(Duration::from_millis(60));
        let stop = drv.tick(&o).movement.unwrap();
        assert_eq!((stop.dir, stop.jump), ([0.0, 0.0], false));
        assert_eq!(drv.tick(&o).movement, None);
    }

    #[tokio::test]
    async fn manual_with_nothing_to_do_and_a_held_character_are_refused() {
        let (c, _) = session(obs(vec![]));
        assert_eq!(call(&c, "POST", "/v1/move/manual", "{}").await.0, StatusCode::BAD_REQUEST);
        let mut held = obs(vec![]);
        held.own.held = true;
        let (c, _) = session(held);
        assert_eq!(call(&c, "POST", "/v1/move/jump", "").await.0, StatusCode::CONFLICT);
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
        let handle = driver::spawn(ChannelLink { observations: obs_rx, steps: step_tx }, control.clone(), Box::new(|_| None));
        obs_tx.send(obs(vec![])).unwrap();
        let step: Step = step_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        assert_eq!(step.verb, None);
        assert!(control.latest().is_some());
        assert_eq!(control.ended(), None);
        drop(obs_tx);
        handle.join().unwrap();
        assert!(control.ended().unwrap().contains("observation stream ended"));
    }

    #[tokio::test]
    async fn zone_cross_in_a_zone_that_did_not_load_is_409() {
        let (obs_tx, obs_rx) = mpsc::channel();
        let (step_tx, _step_rx) = mpsc::channel();
        let control = Control::new(NavSlots::default());
        driver::spawn(ChannelLink { observations: obs_rx, steps: step_tx }, control.clone(), Box::new(|_| None));
        obs_tx.send(obs(vec![])).unwrap();
        while control.latest().is_none() {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let feeder = std::thread::spawn(move || {
            for _ in 0..200 {
                if obs_tx.send(obs(vec![])).is_err() {
                    return;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        let (code, body) = call(&control, "POST", "/v1/move/zone_cross", "{}").await;
        assert_eq!(code, StatusCode::CONFLICT, "{body}");
        feeder.join().unwrap();
    }
}
