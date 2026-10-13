//! The path walker: plans a coarse and a fine route toward the active goal, steers pure-pursuit
//! along it, and decides arrival, stall back-off and recovery.
//!
//! It is driven by one [`Observation`] per call to [`Walker::tick`] and answers with the
//! [`AgentMovement`] to latch. Movement is "held key" semantics on the eqoxide side, so the walker
//! sends an explicit stand-still movement the tick after it stops driving, and nothing while idle.
//!
//! Tick-denominated thresholds in `steering` (stall, back-off, hop) assume about 150 ms between
//! calls.

use crate::collision::{CollisionAStar, PlanOutcome};
use crate::diagnostics::{ClearanceProbe, NavDebugSnapshot, NavDebugView, PlanDebug, WaterDebug};
use crate::status::*;
use crate::steering::*;
use eqoxide_agent_protocol::movement::AgentMovement;
use eqoxide_agent_protocol::observation::Observation;
use eqoxide_core::coord::eq_heading;
use eqoxide_core::physics::{fall_damage, fall_damage_ceiling};
use eqoxide_zone_geometry::collision::Collision;
use std::sync::Arc;
use std::time::Instant;

/// Buffer beyond the body radius that the committed coarse route is inflated off convex wall
/// corners by, so the walker takes one wide arc rather than hugging the apex.
const CORNER_BUFFER: f32 = 2.0;

/// Ticks between live clearance-probe refreshes for the diagnostics snapshot.
const CLEARANCE_REFRESH_TICKS: u32 = 8;

/// eqoxide scales swim `up` by this speed (u/s) when it turns a movement into a vertical wish.
const SWIM_UP_SPEED: f32 = 35.0;

/// Distance one tick may move the character before it is treated as a teleport and the route is
/// re-planned from the new position.
const TELEPORT_JUMP: f32 = 40.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LedgeFallVerdict {
    /// The worst-case damage would kill at the current HP: stop at the ledge.
    Stop { max_dmg: u32 },
    Survivable { max_dmg: u32 },
    /// HP is above the largest damage the fall model can report, so no drop trips the guard.
    Inert { max_dmg: u32, ceiling: u32 },
    NoHpReading,
}

pub fn classify_ledge_fall(drop_u: f32, cur_hp: i32) -> LedgeFallVerdict {
    let (_, max_dmg) = fall_damage(drop_u);
    if cur_hp <= 0 {
        return LedgeFallVerdict::NoHpReading;
    }
    if max_dmg >= cur_hp as u32 {
        return LedgeFallVerdict::Stop { max_dmg };
    }
    let ceiling = fall_damage_ceiling();
    if cur_hp as u32 > ceiling {
        return LedgeFallVerdict::Inert { max_dmg, ceiling };
    }
    LedgeFallVerdict::Survivable { max_dmg }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZoneCrossError {
    ZoneNotLoaded,
    /// The zone's region data (`.wtr`) is missing or unreadable, so its exits are unknown. Not the
    /// same claim as "this zone has no exits".
    NoRegionData,
    NoZoneLines,
}

/// Everything about the committed route needed to re-derive the published `nav_state` each tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommittedFacts {
    pub route: CommittedRoute,
    /// Clearance tier that answered a complete route; `None` for a partial.
    pub tier: Option<&'static str>,
    pub reason: Option<&'static str>,
}

/// What one tick knows about the character, taken from the observation.
#[derive(Clone, Copy)]
struct Pose {
    pos: [f32; 3],
    hp: i32,
}

pub struct Walker {
    slots: NavSlots,
    nav_debug: NavDebugView,
    collision: Option<Arc<Collision>>,
    /// Zone the loaded collision belongs to. Routing is refused while it differs from the
    /// observation's zone, so a stale grid is never used for the new zone.
    zone: Option<String>,
    planner: crate::planner::Planner,
    local_planner: crate::planner::LocalPlanner,

    /// Movement produced this tick, and the heading to face with it.
    out: Option<AgentMovement>,
    face: Option<f32>,
    /// Did the previous tick leave a movement latched on the eqoxide side?
    driving: bool,
    notices: Vec<String>,

    debug_seq: u64,
    last_plan: Option<Arc<PlanDebug>>,
    last_clearance: Option<ClearanceProbe>,
    clearance_countdown: u32,
    inert_fall_disclosed: bool,

    pub path: Vec<[f32; 3]>,
    pub path_i: usize,
    pub path_goal: Option<(f32, f32, f32)>,
    pub local_path: Vec<[f32; 3]>,
    pub local_from: [f32; 3],
    pub local_i: usize,
    pub stuck_best: f32,
    pub stuck_ticks: u32,
    pub stuck_i: usize,
    pub climb_mark: Option<f32>,
    pub nav_repaths: u32,
    pub nav_best_gdist: f32,
    /// Closest 3-D approach to the goal so far, and when it last improved. A walker that keeps
    /// moving without getting closer (a lap around a moat) is stopped as `blocked`/`no_progress`.
    pub nav_best_g3d: f32,
    pub nav_progress_at: Instant,
    pub exec: GoalVerdict,
    pub last_progress_at: Option<Instant>,
    pub committed: Option<CommittedFacts>,
    pub backoff_ticks: u32,
    pub backoff_dir: [f32; 2],
    pub local_stuck_ticks: u32,
    pub replan_coarse: bool,
    pub replan_cooldown: u32,
    pub proactive_replans: u32,
    pub last_walk_pos: [f32; 3],
    /// The planner moved the goal's z to a floor the caller never named.
    pub goal_snapped: bool,
    pub awaiting_first_plan: bool,
    /// `NavStatus::goal_id` when the current plan was posted, stamped into its debug record.
    pub plan_goal_id: u64,
}

impl Walker {
    pub fn new(slots: NavSlots, nav_debug: NavDebugView) -> Self {
        slots.status.lock().unwrap().local_planner_dead = false;
        Walker {
            slots,
            nav_debug,
            collision: None,
            zone: None,
            planner: crate::planner::Planner::spawn(),
            local_planner: crate::planner::LocalPlanner::spawn(),
            out: None,
            face: None,
            driving: false,
            notices: Vec::new(),
            debug_seq: 0,
            last_plan: None,
            last_clearance: None,
            clearance_countdown: 0,
            inert_fall_disclosed: false,
            path: Vec::new(),
            path_i: 0,
            path_goal: None,
            local_path: Vec::new(),
            local_from: [0.0; 3],
            local_i: 0,
            stuck_best: f32::MAX,
            stuck_ticks: 0,
            stuck_i: 0,
            climb_mark: None,
            nav_repaths: 0,
            nav_best_gdist: f32::MAX,
            nav_best_g3d: f32::MAX,
            nav_progress_at: Instant::now(),
            exec: GoalVerdict::fresh_for(0),
            last_progress_at: None,
            committed: None,
            backoff_ticks: 0,
            backoff_dir: [0.0, 0.0],
            local_stuck_ticks: 0,
            replan_coarse: false,
            replan_cooldown: 0,
            proactive_replans: 0,
            last_walk_pos: [0.0; 3],
            goal_snapped: false,
            awaiting_first_plan: false,
            plan_goal_id: 0,
        }
    }

    pub fn slots(&self) -> &NavSlots { &self.slots }

    pub fn debug_view(&self) -> &NavDebugView { &self.nav_debug }

    /// Messages the agent should see (a goal that was changed, a stop that needs explaining).
    /// Drained by the caller.
    pub fn take_notices(&mut self) -> Vec<String> { std::mem::take(&mut self.notices) }

    /// Install the collision grid for `zone`. A different zone from the one already loaded drops
    /// the current goal and route.
    pub fn set_zone(&mut self, zone: &str, collision: Arc<Collision>) {
        if self.zone.as_deref() != Some(zone) {
            self.reset_for_zone_change();
        }
        self.zone = Some(zone.to_string());
        self.collision = Some(collision);
    }

    /// Forget the loaded zone, so routing refuses until the next `set_zone`.
    pub fn clear_zone(&mut self) {
        self.reset_for_zone_change();
        self.zone = None;
        self.collision = None;
    }

    /// Walk to the zone line nearest `from`, where eqoxide's passive zone-line crossing fires once
    /// the character stands in it. Returns the new goal id. Crossing is not a verb: eqoxide sends
    /// the zone change itself, so a successful cross shows up as a changed `zone_name`, which
    /// drops the goal.
    ///
    /// `line` is a zone-line region index from the zone's region data; `None` picks the nearest.
    /// `Observation` carries no zone points, so the harness cannot tell which line leads where, or
    /// which lines eqoxide refuses to cross (the nearest line in a zone with teleport pads can be
    /// one): naming a line is how the caller steers around that.
    pub fn zone_cross(&mut self, from: [f32; 3], line: Option<i32>) -> Result<u64, ZoneCrossError> {
        let col = self.collision.clone().ok_or(ZoneCrossError::ZoneNotLoaded)?;
        let lines = col.zone_line_indices().map_err(|_| ZoneCrossError::NoRegionData)?;
        if lines.is_empty() {
            return Err(ZoneCrossError::NoZoneLines);
        }
        let (_, p) = col.find_zone_line_near(line, from).ok_or(ZoneCrossError::NoZoneLines)?;
        Ok(self.slots.goto((p[0], p[1], p[2])))
    }

    /// Advance one tick and return the movement to latch, if any.
    pub fn tick(&mut self, obs: &Observation) -> Option<AgentMovement> {
        self.out = None;
        self.face = None;
        let pose = Pose { pos: obs.own.pos, hp: obs.own.hp };

        if !self.halt_if_dead(obs) {
            self.drive_teleport_detect(pose);
            self.drive_chase(obs);
            if let Some(goal) = self.resolve_goal(pose) {
                self.drive_walk(obs, pose, goal);
            }
        }
        self.finish_tick()
    }

    /// Turn this tick's wish into the movement to send. A latched movement keeps applying until a
    /// newer one arrives, so the tick after driving stops must send an explicit stand-still.
    fn finish_tick(&mut self) -> Option<AgentMovement> {
        let out = match (self.out.take(), self.face, self.driving) {
            (Some(mut m), face, _) => {
                m.wish_heading = face.or(m.wish_heading);
                Some(m)
            }
            (None, Some(h), _) => Some(AgentMovement { dir: [0.0, 0.0], up: 0.0, jump: false, wish_heading: Some(h) }),
            (None, None, true) => Some(AgentMovement { dir: [0.0, 0.0], up: 0.0, jump: false, wish_heading: None }),
            (None, None, false) => None,
        };
        self.driving = out.is_some_and(|m| m.dir != [0.0, 0.0] || m.up != 0.0 || m.jump);
        out
    }

    fn clear_local_plan(&mut self) {
        self.local_path.clear();
        self.local_i = 0;
        self.local_stuck_ticks = 0;
        self.local_planner.cancel();
        self.set_nav_local(None);
    }

    fn local_says_no_way_through(&self) -> bool {
        self.slots.status.lock().unwrap().local.as_ref().is_some_and(|l| l.state == "no_way_through")
    }

    fn reset_for_zone_change(&mut self) {
        self.slots.stop();
        self.out = None;
        self.last_plan = None;
        self.last_clearance = None;
        self.path.clear();
        self.local_path.clear();
        self.local_i = 0;
        self.path_goal = None;
        self.path_i = 0;
        self.stuck_i = 0;
        self.stuck_best = f32::MAX;
        self.stuck_ticks = 0;
        self.nav_repaths = 0;
        self.proactive_replans = 0;
        self.nav_best_gdist = f32::MAX;
        self.nav_best_g3d = f32::MAX;
        self.nav_progress_at = Instant::now();
        self.reset_drive_state();
        self.backoff_ticks = 0;
        self.local_stuck_ticks = 0;
        self.replan_coarse = false;
        self.replan_cooldown = 0;
        self.planner.cancel();
        self.local_planner.cancel();
        self.awaiting_first_plan = false;
        self.set_nav_state_because("idle", Some(NAV_REASON_ZONED));
        self.publish_debug(None, None);
    }

    fn set_nav_state(&self, state: &str) { self.set_nav_state_because(state, None); }

    fn set_nav_state_because(&self, state: &str, reason: Option<&str>) {
        let mut s = self.slots.status.lock().unwrap();
        write_nav_state_locked(&mut s, state, reason);
    }

    fn reset_drive_state(&mut self) {
        let goal_id = self.slots.status.lock().unwrap().goal_id;
        self.exec = GoalVerdict::fresh_for(goal_id);
        self.last_progress_at = Some(Instant::now());
        self.committed = None;
        self.inert_fall_disclosed = false;
    }

    fn tick_drive_state(&mut self, progressed: bool) -> RouteExecution {
        let now = Instant::now();
        let goal_id = self.slots.status.lock().unwrap().goal_id;
        let new_journey = !self.exec.is_about(goal_id);
        self.exec = self.exec.tick(goal_id, progressed, self.nav_repaths);
        let exec = self.exec.as_of(goal_id);
        if new_journey || exec.quiet_ticks() == 0 || self.last_progress_at.is_none() {
            self.last_progress_at = Some(now);
        }
        exec
    }

    /// Re-derive the driving `nav_state` word from the committed route and the execution verdict.
    fn publish_drive_state(&self) {
        let Some(facts) = self.committed else { return };
        let mut s = self.slots.status.lock().unwrap();
        let exec = self.exec.as_of(s.goal_id);
        write_nav_state_locked(&mut s, driving_nav_state(facts.route, exec), facts.reason);
        s.tier = facts.tier;
        s.stall = match exec {
            RouteExecution::Advancing { .. } => None,
            RouteExecution::Stalled { quiet_ticks, repaths } => Some(NavStall {
                quiet_ticks,
                quiet_ms: self.last_progress_at.map_or(0, |t| t.elapsed().as_millis() as u64),
                repaths,
                route: facts.route.as_str(),
            }),
        };
    }

    fn set_nav_local(&self, local: Option<NavLocal>) {
        let mut s = self.slots.status.lock().unwrap();
        let local = if s.state == "idle" { None } else { local };
        if s.local != local {
            s.local = local;
        }
    }

    fn latch_local_planner_liveness(&self) {
        if self.local_planner.is_dead() {
            self.slots.status.lock().unwrap().local_planner_dead = true;
        }
    }

    fn publish_debug(&mut self, player: Option<[f32; 3]>, water: Option<WaterDebug>) {
        self.debug_seq += 1;
        let (state, reason, goal_id) = {
            let s = self.slots.status.lock().unwrap();
            (s.state.clone(), s.reason.clone(), s.goal_id)
        };
        let goal = self.slots.goto_target.lock().unwrap().map(|(x, y, z)| [x, y, z]);
        let snap = NavDebugSnapshot {
            seq: self.debug_seq,
            zone_model_loaded: self.collision.is_some(),
            nav_state: state,
            nav_reason: reason,
            goal_id,
            player,
            published_at: Instant::now(),
            goal,
            committed_coarse: self.path.clone(),
            committed_fine: self.local_path.clone(),
            plan: self.last_plan.clone(),
            pads: Vec::new(),
            clearance: self.last_clearance.clone(),
            water,
        };
        *self.nav_debug.lock().unwrap() = Some(Arc::new(snap));
    }

    /// Is the published idle snapshot still an accurate picture, so republishing can be skipped?
    fn debug_is_settled(&self, pos: [f32; 3]) -> bool {
        let snap = self.nav_debug.lock().unwrap();
        let Some(s) = snap.as_ref() else { return false };
        let live = self.slots.status.lock().unwrap();
        let moved = s.player.is_none_or(|p| (0..3).any(|i| (p[i] - pos[i]).abs() >= 0.5));
        !moved
            && s.zone_model_loaded == self.collision.is_some()
            && s.committed_coarse.is_empty() && s.committed_fine.is_empty() && s.goal.is_none()
            && s.nav_state == live.state && s.nav_reason == live.reason
    }

    fn refresh_clearance(&mut self, player: [f32; 3]) {
        if self.clearance_countdown > 0 {
            self.clearance_countdown -= 1;
            return;
        }
        self.clearance_countdown = CLEARANCE_REFRESH_TICKS;
        self.last_clearance = self.collision.as_ref().map(|c| c.clearance_probe(player[0], player[1], player[2]));
    }

    pub fn nav_state_is(&self, state: &str) -> bool {
        self.slots.status.lock().unwrap().state == state
    }

    fn advance_cursor(&mut self, p: [f32; 3]) {
        while self.path_i + 2 < self.path.len() {
            let (a, b) = (self.path[self.path_i], self.path[self.path_i + 1]);
            let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let l2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
            let t = if l2 < 1e-6 { 1.0 } else {
                ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1] + (p[2] - a[2]) * ab[2]) / l2
            };
            if t >= 1.0 { self.path_i += 1; } else { break; }
        }
        let walked_to = self.path_i;
        let col = self.collision.as_deref();
        let reachable = |a: [f32; 3], b: [f32; 3]| col.is_none_or(|c| resync_reachable(c, a, b));
        self.path_i = resync_cursor(&self.path, walked_to, p, reachable);
        if self.path_i > walked_to {
            self.stuck_i = self.stuck_i.max(self.path_i);
        }
    }

    fn notice(&mut self, msg: impl Into<String>) {
        let msg = msg.into();
        tracing::warn!("NAV: {msg}");
        self.notices.push(msg);
    }

    fn stop_nav(&mut self, pose: Pose, state: &str, reason: &str, msg: &str) {
        self.stop_nav_blocked(pose, state, reason, None, None, msg);
    }

    fn stop_nav_blocked(
        &mut self,
        pose: Pose,
        state: &str,
        reason: &str,
        goal_blk: Option<crate::traversability::Blockage>,
        frontier_blk: Option<crate::traversability::Blockage>,
        msg: &str,
    ) {
        self.notice(msg);
        self.set_nav_state_because(state, Some(reason));
        let to_nav = |b: crate::traversability::Blockage| NavBlockage { hazard: b.hazard.as_str(), at: b.at };
        {
            let mut s = self.slots.status.lock().unwrap();
            s.blocked_goal = goal_blk.map(to_nav);
            s.blocked_frontier = frontier_blk.map(to_nav);
        }
        self.path.clear();
        self.local_path.clear();
        self.local_i = 0;
        self.local_stuck_ticks = 0;
        self.local_planner.cancel();
        self.path_goal = None;
        self.planner.cancel();
        self.awaiting_first_plan = false;
        self.reset_drive_state();
        *self.slots.goto_target.lock().unwrap() = None;
        self.out = None;
        self.publish_debug(Some(pose.pos), None);
    }

    fn apply_local_plan(&mut self, reply: crate::planner::LocalReply) {
        let outcome = reply.outcome;
        self.local_path = outcome.steer().to_vec();
        self.local_from = reply.start;
        self.local_i = 0;
        let healthy = self.backoff_ticks == 0 && self.stuck_ticks < NAV_HOP_TICKS;
        if arms_coarse_replan(&outcome) && healthy && self.replan_cooldown == 0 {
            self.local_stuck_ticks += 1;
            if self.local_stuck_ticks >= NAV_LOCAL_STUCK_TICKS {
                self.replan_coarse = true;
                self.proactive_replans += 1;
                tracing::debug!(
                    "NAV: fine plan closed its window short of the carrot near ({:.0},{:.0}) ({}) — re-planning coarse (proactive #{})",
                    reply.start[0], reply.start[1], outcome.reason(), self.proactive_replans);
            }
        } else if outcome.threaded() {
            self.local_stuck_ticks = 0;
        }
        self.set_nav_local(Some(NavLocal {
            state: outcome.state().to_string(),
            reason: outcome.reason().to_string(),
            stuck_ticks: self.local_stuck_ticks,
            plan_us: reply.plan_us as u64,
        }));
    }

    fn inflate_committed(&self, route: &mut [[f32; 3]]) {
        if let Some(c) = &self.collision {
            c.inflate_route_off_corners(route, eqoxide_core::physics::PLAYER_RADIUS, CORNER_BUFFER);
        }
    }

    fn commit_route(&mut self, mut path: Vec<[f32; 3]>, facts: CommittedFacts, pose: Pose) {
        self.inflate_committed(&mut path);
        self.path = path;
        self.path_i = 0;
        self.stuck_i = 0;
        self.clear_local_plan();
        self.committed = Some(facts);
        self.publish_drive_state();
        self.publish_debug(Some(pose.pos), None);
    }

    /// Apply a finished coarse plan. Returns `true` when navigation ended and the caller must not
    /// keep driving this tick.
    fn apply_plan(&mut self, reply: crate::planner::PlanReply, pose: Pose, goal: (f32, f32, f32)) -> bool {
        self.awaiting_first_plan = false;
        let snapped = reply.goal_snapped;
        self.goal_snapped = snapped.is_some();
        {
            let (outcome_str, route_len, route_end) = match &reply.outcome {
                PlanOutcome::Route(p) => ("route", p.len(), p.last().copied()),
                PlanOutcome::Unreachable { .. } => ("unreachable", 0, None),
                PlanOutcome::Exhausted { progress, .. } => (
                    "exhausted",
                    progress.as_ref().map_or(0, |p| p.len()),
                    progress.as_ref().and_then(|p| p.last().copied()),
                ),
            };
            self.last_plan = Some(Arc::new(PlanDebug {
                gen: reply.gen,
                goal_id: self.plan_goal_id,
                start: reply.start,
                goal: reply.goal,
                outcome: outcome_str.to_string(),
                reason: reply.outcome.reason().to_string(),
                route_len,
                plan_ms: reply.plan_ms as u64,
                tight: reply.tight,
                goal_snapped: snapped.is_some(),
                goal_offset: route_goal_offset(route_end, reply.goal),
                trace: reply.trace.clone(),
            }));
        }
        match snapped {
            Some(crate::collision::GoalSnap::ToColumnFloor { z }) => self.notices.push(format!(
                "Goal z={:.0} is not on any floor — routing to the floor at z={:.0} instead.", goal.2, z)),
            Some(crate::collision::GoalSnap::ToWaterSurface { surface_z }) => self.notices.push(format!(
                "Goal z={:.0} is submerged — navigating to the water surface at z={:.0} above it; arrival will be at the surface.",
                goal.2, surface_z)),
            None => {}
        }
        match reply.outcome {
            PlanOutcome::Route(path) => {
                tracing::info!("NAV: plan #{} → route to ({:.0},{:.0}), {} waypoints, {}ms",
                    reply.gen, goal.0, goal.1, path.len(), reply.plan_ms);
                let facts = CommittedFacts {
                    route: CommittedRoute::Complete,
                    tier: Some(if reply.tight { "minimum" } else { "preferred" }),
                    reason: self.goal_snapped.then_some("goal_z_snapped"),
                };
                self.commit_route(path, facts, pose);
                false
            }
            PlanOutcome::Exhausted { limit, progress: Some(path) } => {
                tracing::warn!("NAV: plan #{} exhausted ({}) after {}ms — walking a partial route of {} waypoints toward ({:.0},{:.0})",
                    reply.gen, limit.as_str(), reply.plan_ms, path.len(), goal.0, goal.1);
                self.notices.push("Planner gave up before finding a full route — walking as far as it can, then re-planning".into());
                let facts = CommittedFacts { route: CommittedRoute::Partial, tier: None, reason: Some(limit.as_str()) };
                self.commit_route(path, facts, pose);
                false
            }
            PlanOutcome::Exhausted { limit, progress: None } => {
                self.stop_nav(pose, "search_exhausted", limit.as_str(), &format!(
                    "Path search to ({:.0},{:.0}) gave up ({}) after {}ms with no usable route. This is not \
                     'no route exists' — the search never finished. Try a nearer waypoint.",
                    goal.0, goal.1, limit.as_str(), reply.plan_ms));
                true
            }
            PlanOutcome::Unreachable { reason: why, goal_blocked_by, frontier_blocked_by } => {
                let blk = goal_blocked_by.or(frontier_blocked_by);
                let detail = blk.map(|b| format!(" — blocked by {} at ({:.0},{:.0},{:.0})",
                    b.hazard.as_str(), b.at[0], b.at[1], b.at[2])).unwrap_or_default();
                self.stop_nav_blocked(pose, "no_path", why.as_str(), goal_blocked_by, frontier_blocked_by, &format!(
                    "No route to ({:.0},{:.0}): {} (searched to completion in {}ms — a definitive no, not a timeout){}.",
                    goal.0, goal.1, why.as_str(), reply.plan_ms, detail));
                true
            }
        }
    }

    /// Halt on a dead character, or on a published HP of zero with a known maximum and no death
    /// flag. The two get different words: the second is a fact about the reading, not a death.
    fn halt_if_dead(&mut self, obs: &Observation) -> bool {
        let hp_zero = obs.own.hp_verified && obs.own.hp <= 0 && obs.own.hp_max > 0;
        if !obs.dead && !hp_zero {
            self.retire_life_halt();
            return false;
        }
        if self.slots.goto_target.lock().unwrap().take().is_some() {
            tracing::info!("NAV: player is dead — abandoning goal");
        }
        *self.slots.goto_entity.lock().unwrap() = None;
        self.out = None;
        self.path.clear();
        self.local_path.clear();
        self.local_i = 0;
        self.path_goal = None;
        self.path_i = 0;
        // A corpse must not act on a plan that lands after it died.
        self.planner.cancel();
        self.awaiting_first_plan = false;
        let (state, why) = if obs.dead {
            (NAV_STATE_DEAD, NAV_REASON_PLAYER_DEAD)
        } else {
            (NAV_STATE_HALTED_HP_ZERO, NAV_REASON_HP_ZERO_UNCONFIRMED)
        };
        self.set_nav_state_because(state, Some(why));
        self.publish_debug(Some(obs.own.pos), None);
        true
    }

    fn retire_life_halt(&mut self) {
        let current = self.slots.status.lock().unwrap().state.clone();
        let why = match current.as_str() {
            NAV_STATE_DEAD => NAV_REASON_RESPAWNED,
            NAV_STATE_HALTED_HP_ZERO => NAV_REASON_HP_RESTORED,
            _ => return,
        };
        self.set_nav_state_because("idle", Some(why));
    }

    /// Positions of living NPCs the route should keep clear of, excluding any near the goal.
    fn aggro_avoid(obs: &Observation, goal: (f32, f32, f32), enabled: bool) -> Vec<[f32; 2]> {
        if !enabled {
            return Vec::new();
        }
        const NEAR_GOAL_SQ: f32 = 55.0 * 55.0;
        obs.visible.iter()
            .filter(|e| e.is_npc && !e.dead)
            .filter(|e| {
                let (dx, dy) = (e.pos[0] - goal.0, e.pos[1] - goal.1);
                dx * dx + dy * dy > NEAR_GOAL_SQ
            })
            .map(|e| [e.pos[0], e.pos[1]])
            .collect()
    }

    /// Re-aim a follow goal at the followed entity's latest position. Only entities in the
    /// observation's visible set can be followed; one that leaves it ends the follow.
    fn drive_chase(&mut self, obs: &Observation) {
        let chase = self.slots.goto_entity.lock().unwrap().clone();
        let Some(name) = chase else { return };
        let mut target = self.slots.goto_target.lock().unwrap();
        if target.is_none() {
            *self.slots.goto_entity.lock().unwrap() = None;
        } else if let Some(e) = obs.visible.iter().find(|e| e.name == name && !e.dead) {
            *target = Some((e.pos[0], e.pos[1], e.pos[2]));
        } else {
            *target = None;
            *self.slots.goto_entity.lock().unwrap() = None;
        }
    }

    /// A position jump (server teleport or correction) invalidates the route.
    fn drive_teleport_detect(&mut self, pose: Pose) {
        let jumped = (pose.pos[0] - self.last_walk_pos[0]).hypot(pose.pos[1] - self.last_walk_pos[1]) > TELEPORT_JUMP;
        self.last_walk_pos = pose.pos;
        if jumped {
            self.path_goal = None;
            self.clear_local_plan();
        }
    }

    /// The current goto goal, or `None` after retiring any state that has no goal behind it.
    fn resolve_goal(&mut self, pose: Pose) -> Option<(f32, f32, f32)> {
        let goto = *self.slots.goto_target.lock().unwrap();
        if let Some(goal) = goto {
            return Some(goal);
        }
        self.path.clear();
        self.path_goal = None;
        self.planner.cancel();
        self.clear_local_plan();
        self.awaiting_first_plan = false;
        self.out = None;
        // Anything not terminal asserts work in flight, and there is no goal left to justify it.
        let current = self.slots.status.lock().unwrap().state.clone();
        if !nav_state_is_terminal(&current) {
            let why = match current.as_str() {
                NAV_STATE_DEAD => NAV_REASON_RESPAWNED,
                NAV_STATE_HALTED_HP_ZERO => NAV_REASON_HP_RESTORED,
                _ => NAV_REASON_GOAL_DROPPED,
            };
            self.set_nav_state_because("idle", Some(why));
        }
        if !self.debug_is_settled(pose.pos) {
            self.publish_debug(Some(pose.pos), None);
        }
        None
    }

    fn halt_no_world(&mut self, player: [f32; 3], reason: &str) {
        self.path.clear();
        self.path_i = 0;
        self.path_goal = None;
        self.clear_local_plan();
        self.planner.cancel();
        self.awaiting_first_plan = false;
        self.out = None;
        self.set_nav_state_because(NAV_STATE_ZONE_LOADING, Some(reason));
        self.publish_debug(Some(player), None);
    }

    fn post_plan(&mut self, obs: &Observation, pose: Pose, goal: (f32, f32, f32), c: Arc<Collision>) -> u64 {
        let av = *self.slots.avoid.lock().unwrap();
        let avoid = Self::aggro_avoid(obs, goal, av.enabled);
        let goal_region = c.zone_line_at([goal.0, goal.1, goal.2 + 1.0]);
        let gen = self.planner.request(crate::planner::PlanRequest {
            gen: 0,
            start: pose.pos,
            goal: [goal.0, goal.1, goal.2],
            avoid,
            aggro_buffer: av.buffer,
            goal_region,
            teleport_pads: Vec::new(),
            collision: c,
        });
        self.plan_goal_id = self.slots.status.lock().unwrap().goal_id;
        gen
    }

    fn drive_walk(&mut self, obs: &Observation, pose: Pose, goal: (f32, f32, f32)) {
        let [px, py, pz] = pose.pos;
        let Some(col) = self.collision.clone().filter(|_| self.zone.as_deref() == Some(obs.own.zone_name.as_str())) else {
            let reason = if self.collision.is_some() { "zone_changed" } else { "zone_assets_pending" };
            self.halt_no_world(pose.pos, reason);
            return;
        };

        if self.replan_cooldown > 0 {
            self.replan_cooldown -= 1;
        }
        self.refresh_clearance(pose.pos);

        let is_chase = self.slots.goto_entity.lock().unwrap().is_some();
        let in_flight = self.planner.in_flight_goal().map(|g| (g[0], g[1], g[2]));
        let decision = replan_decision(self.path_goal, goal, in_flight, self.replan_coarse, is_chase);
        if decision.reset_route {
            self.path.clear();
            self.clear_local_plan();
            self.path_i = 0;
            self.stuck_i = 0;
            self.backoff_ticks = 0;
            self.stuck_best = f32::MAX;
            self.stuck_ticks = 0;
            self.nav_repaths = 0;
            self.proactive_replans = 0;
            self.nav_best_gdist = f32::MAX;
            self.nav_best_g3d = f32::MAX;
            self.nav_progress_at = Instant::now();
            self.reset_drive_state();
            self.replan_cooldown = 0;
            self.replan_coarse = false;
            self.goal_snapped = false;
        }
        if decision.post {
            if !decision.reset_route {
                self.replan_coarse = false;
                self.local_stuck_ticks = 0;
                self.replan_cooldown = REPLAN_COOLDOWN_TICKS;
            }
            let gen = self.post_plan(obs, pose, goal, col.clone());
            self.path_goal = Some(goal);
            tracing::info!("NAV: posted plan #{gen} to ({:.0},{:.0})", goal.0, goal.1);
            if self.path.is_empty() {
                self.awaiting_first_plan = true;
                self.set_nav_state("planning");
                self.out = None;
            }
        }

        if let Some(reply) = self.planner.poll() {
            if self.apply_plan(reply, pose, goal) {
                return;
            }
        }
        if self.planner.is_dead() {
            self.stop_nav(pose, "no_path", "planner_dead", &format!(
                "The pathfinding worker thread has died — no route to ({:.0},{:.0}) or anywhere else can be planned \
                 for the rest of this session. This is a client fault, not an unreachable goal.", goal.0, goal.1));
            return;
        }
        if self.awaiting_first_plan {
            self.out = None;
            self.publish_debug(Some(pose.pos), None);
            return;
        }

        const LOOK_AHEAD: f32 = 5.0;
        self.advance_cursor(pose.pos);
        let have_path = !self.path.is_empty();
        let target: (f32, f32, f32) = if have_path {
            const LOCAL_BOUND: f32 = 40.0;
            let coarse = carrot_along(&self.path, self.path_i, pose.pos, LOOK_AHEAD).unwrap_or([goal.0, goal.1, pz]);
            if let Some(reply) = self.local_planner.poll() {
                self.apply_local_plan(reply);
            }
            if !self.local_path.is_empty() && (px - self.local_from[0]).hypot(py - self.local_from[1]) > LOCAL_BOUND {
                self.clear_local_plan();
            }
            let local_goal = carrot_along(&self.path, self.path_i, pose.pos, LOCAL_REACH).unwrap_or(coarse);
            self.local_planner.post_if_idle(crate::planner::LocalRequest {
                gen: 0,
                start: pose.pos,
                goal: local_goal,
                cell: LOCAL_CELL,
                bound: LOCAL_BOUND,
                carrot_tol: LOCAL_CELL * 2.0,
                collision: col.clone(),
            });
            if self.local_planner.is_dead() {
                self.set_nav_local(Some(NavLocal {
                    state: "planner_dead".into(),
                    reason: "local_planner_dead".into(),
                    stuck_ticks: 0,
                    plan_us: 0,
                }));
                self.latch_local_planner_liveness();
            }
            let los = |a: [f32; 3], b: [f32; 3]| col.carrot_los_clear(a, b, STEER_LOS_CLEARANCE);
            let aim = steer_target(
                &self.path, self.path_i, &self.local_path, &mut self.local_i,
                SteeringContext { from: pose.pos, look_ahead: LOOK_AHEAD, fallback: coarse }, los);
            (aim[0], aim[1], aim[2])
        } else {
            self.clear_local_plan();
            (goal.0, goal.1, pz)
        };

        // A climbable volume on the route overrides the aim: go to its centre until the dismount
        // is within reach, then to the dismount.
        const NAV_CLIMB_MIN_RISE: f32 = 2.0;
        let climb_edge = col.climb_edges().iter().find(|e| {
            e.volume.contains(pose.pos)
                && self.path.get(self.path_i..).is_some_and(|rest| rest.iter().any(|w| {
                    (w[0] - e.dismount[0]).abs() < 0.01
                        && (w[1] - e.dismount[1]).abs() < 0.01
                        && (w[2] - e.dismount[2]).abs() < 0.01
                }))
        }).cloned();
        let target = match &climb_edge {
            Some(e) if e.dismount[2] - pz > NAV_CLIMB_MIN_RISE => {
                let ctr = e.volume.center();
                (ctr[0], ctr[1], e.dismount[2])
            }
            Some(e) => (e.dismount[0], e.dismount[1], e.dismount[2]),
            None => target,
        };

        let dx = target.0 - px;
        let dy = target.1 - py;
        let dist = (dx * dx + dy * dy).sqrt();

        // Lethal-fall guard: stop at a ledge whose worst-case damage would kill.
        const FALL_TRIGGER: f32 = 18.0;
        let drop_to_target = pz - target.2;
        let water_landing = col.in_water([target.0, target.1, target.2 + 3.0]);
        if drop_to_target > FALL_TRIGGER && dist <= STOP_DIST + 8.0 && !water_landing {
            match classify_ledge_fall(drop_to_target, pose.hp) {
                LedgeFallVerdict::Stop { max_dmg } => {
                    tracing::info!("NAV: fall of {drop_to_target:.0}u (up to {max_dmg} dmg) would exceed {} hp — stopping at ledge", pose.hp);
                    self.notices.push("Fall too dangerous (HP too low) — stopped at the ledge".into());
                    self.set_nav_state_because("blocked", Some("fall_would_be_lethal"));
                    *self.slots.goto_target.lock().unwrap() = None;
                    self.out = None;
                    self.publish_debug(Some(pose.pos), None);
                    return;
                }
                LedgeFallVerdict::Inert { max_dmg, ceiling } => {
                    tracing::info!("NAV: fall of {drop_to_target:.0}u — the lethal-fall guard cannot fire at {} hp: \
                        reported fall damage is capped at {ceiling} (this fall reports up to {max_dmg}). Descending.", pose.hp);
                    if !self.inert_fall_disclosed {
                        self.inert_fall_disclosed = true;
                        self.notices.push(format!(
                            "Lethal-fall guard cannot fire at {} hp: reported fall damage is capped at {ceiling}, so no drop trips it. Descending unchecked.",
                            pose.hp));
                    }
                }
                LedgeFallVerdict::Survivable { .. } | LedgeFallVerdict::NoHpReading => {}
            }
        }

        let gdx = goal.0 - px;
        let gdy = goal.1 - py;
        let gdist = (gdx * gdx + gdy * gdy).sqrt();
        let goal_floor_z = col.resolve_goal_floor([goal.0, goal.1, goal.2]).unwrap_or(goal.2);
        let gdz = goal_floor_z - pz;
        let following = self.slots.goto_entity.lock().unwrap().is_some();
        match arrival_action(gdist, gdz, following) {
            ArrivalAction::FollowHold => {
                self.set_nav_state("following");
                self.path.clear();
                self.path_goal = None;
                self.out = None;
                self.face = Some(eq_heading(gdx, gdy));
                self.publish_debug(Some(pose.pos), None);
                return;
            }
            ArrivalAction::Arrived => {
                tracing::info!("NAV: arrived at ({px:.1},{py:.1},z~{pz:.1}) (goal floor z={goal_floor_z:.1}, |dz|={:.1})", gdz.abs());
                if self.goal_snapped {
                    self.set_nav_state_because("arrived", Some("goal_z_snapped"));
                } else {
                    self.set_nav_state("arrived");
                }
                *self.slots.goto_target.lock().unwrap() = None;
                self.out = None;
                self.reset_drive_state();
                self.face = Some(eq_heading(gdx, gdy));
                self.publish_debug(Some(pose.pos), None);
                return;
            }
            ArrivalAction::Drive => {}
        }

        const REPATH_RESET_DIST: f32 = 200.0;
        if gdist < self.nav_best_gdist - REPATH_RESET_DIST {
            self.nav_best_gdist = gdist;
            self.nav_repaths = 0;
            self.proactive_replans = 0;
        }

        // Route-level progress: a walker that keeps moving without getting closer to the goal is
        // stopped, which the cursor-based stall detector below cannot see.
        if have_path && !following {
            let g3d = (gdist * gdist + gdz * gdz).sqrt();
            let now = Instant::now();
            let cursor_advanced = self.path_i > self.stuck_i;
            let advancing_complete_route =
                self.committed.map(|c| c.route) == Some(CommittedRoute::Complete) && cursor_advanced;
            let closer = progress_improved(&mut self.nav_best_g3d, g3d, NAV_PROGRESS_EPS);
            self.tick_drive_state(cursor_advanced || closer);
            self.publish_drive_state();
            if advancing_complete_route || closer {
                self.nav_progress_at = now;
            } else if now.duration_since(self.nav_progress_at) >= NAV_NO_PROGRESS_WINDOW {
                self.stop_nav(pose, "blocked", "no_progress", &format!(
                    "No progress toward ({:.1},{:.1}) for {}s: the walker keeps moving but its closest approach \
                     to the goal has not improved (held at ~{:.0}u). A lap the route cannot escape, not a \
                     physical wedge. Approach from another direction or pick a reachable waypoint.",
                    goal.0, goal.1, NAV_NO_PROGRESS_WINDOW.as_secs(), self.nav_best_g3d));
                return;
            }
        }

        if self.proactive_replans >= PROACTIVE_REPLAN_CAP {
            self.stop_nav(pose, "blocked", "local_no_way_through", &format!(
                "Wedged near ({px:.1},{py:.1}) after {} proactive coarse re-plans that did not get the journey \
                 past this spot: the fine planner cannot thread the committed route here. Approach from \
                 another direction.", self.proactive_replans));
            return;
        }

        // Downhill back-off: walk the reverse direction, then re-plan from there.
        if self.backoff_ticks > 0 {
            self.backoff_ticks -= 1;
            self.out = Some(movement(self.backoff_dir, 0.0, false));
            if self.backoff_ticks == 0 {
                let gen = self.post_plan(obs, pose, goal, col.clone());
                self.stuck_ticks = 0;
                tracing::warn!("NAV: backed off — posted re-plan #{gen} (attempt {})", self.nav_repaths);
            }
            self.publish_debug(Some(pose.pos), None);
            return;
        }

        let climb = climb_edge.is_some();
        if have_path {
            // The whole ascent of a ladder is one coarse segment, so the cursor cannot see it:
            // credit vertical progress instead.
            const NAV_CLIMB_PROGRESS: f32 = 2.0;
            let climbed = climb && self.climb_mark.is_some_and(|z0| pz - z0 > NAV_CLIMB_PROGRESS);
            if !climb {
                self.climb_mark = None;
            } else if climbed || self.climb_mark.is_none() {
                self.climb_mark = Some(pz);
            }
            if self.path_i > self.stuck_i {
                self.stuck_i = self.path_i;
                self.stuck_ticks = 0;
            } else if climbed {
                self.stuck_ticks = 0;
            } else {
                self.stuck_ticks += 1;
                if self.stuck_ticks >= NAV_STUCK_TICKS {
                    self.stuck_ticks = 0;
                    if self.nav_repaths < 8 {
                        self.nav_repaths += 1;
                        self.backoff_ticks = NAV_BACKOFF_TICKS;
                        self.backoff_dir = if dist > 1e-3 { [-dx / dist, -dy / dist] } else { [0.0, 0.0] };
                        tracing::warn!("NAV: no progress near ({px:.1},{py:.1}) — backing off (attempt {})", self.nav_repaths);
                        return;
                    }
                    if self.local_says_no_way_through() {
                        self.stop_nav(pose, "blocked", "local_no_way_through", &format!(
                            "Wedged at ({px:.1},{py:.1}) after {} re-path attempts, and the fine planner has closed \
                             its whole window without finding a way along the committed route. The corridor is not \
                             threadable at the character's collision radius. Approach from another direction.",
                            self.nav_repaths));
                    } else {
                        self.stop_nav(pose, "blocked", "walker_stalled", &format!(
                            "Wedged at ({px:.1},{py:.1}) after {} re-path attempts — the route is planned and the fine \
                             planner can thread it, but the walker cannot physically follow it. The goal itself is \
                             reachable; this is a collision or steering wedge, not a routing failure.",
                            self.nav_repaths));
                    }
                    return;
                }
            }
        }

        let swim = col.in_water([px, py, pz]) || col.in_water([px, py, pz + 3.0]);
        let jump = match (self.path.get(self.path_i), self.path.get(self.path_i + 1)) {
            (Some(a), Some(b)) if self.path_i >= 1 => {
                let seg = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
                let to_takeoff = ((px - a[0]).powi(2) + (py - a[1]).powi(2)).sqrt();
                seg > JUMP_SEG_MIN && to_takeoff < JUMP_TAKEOFF_DIST
            }
            _ => false,
        };
        let swim_plane = if swim {
            col.water_surface(pose.pos).map(|surf| surf - crate::traversability::PLAYER_BODY.float_depth)
        } else {
            None
        };
        // Vertical wish in u/s; `up` is that as a fraction of the speed eqoxide scales it by.
        let up = match &climb_edge {
            Some(e) if pz < e.dismount[2] => 1.0,
            Some(_) => 0.0,
            None if swim => (swim_vspeed(target.2, pz, swim_plane) / SWIM_UP_SPEED).clamp(-1.0, 1.0),
            None => 0.0,
        };
        // Underwater a jump is swim-up on the eqoxide side, so only hop on land. eqoxide has no
        // one-shot barrier hop in AgentMovement; a plain jump stands in for it.
        let hop = self.stuck_ticks >= NAV_HOP_TICKS;
        let dir = if dist > 1e-3 { [dx / dist, dy / dist] } else { [0.0, 0.0] };
        self.out = Some(movement(dir, up, (jump || hop) && !swim));
        self.face = Some(eq_heading(dx, dy));
        self.publish_debug(Some(pose.pos), Some(WaterDebug { swimming: swim, swim_plane }));
    }
}

fn movement(dir: [f32; 2], up: f32, jump: bool) -> AgentMovement {
    AgentMovement { dir, up, jump, wish_heading: None }
}

fn write_nav_state_locked(s: &mut NavStatus, state: &str, reason: Option<&str>) {
    if state == "idle" {
        s.retire_to_idle(reason.expect("`idle` must name how it got there"));
    } else if s.state != state || s.reason.as_deref() != reason {
        s.transition_within_goal(state, reason);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use eqoxide_agent_protocol::observation::{LegalActionMask, OwnState};
    use eqoxide_assets::{MeshData, RenderMode, ZoneAssets};
    use std::time::Duration;

    const ZONE: &str = "testzone";

    fn flat_floor() -> Arc<Collision> {
        let (n0, n1, e0, e1) = (-300.0, 300.0, -300.0, 300.0);
        let slab = MeshData {
            positions: vec![[n0, 0.0, e0], [n0, 0.0, e1], [n1, 0.0, e1], [n1, 0.0, e0]],
            normals: vec![], uvs: vec![],
            indices: vec![0, 1, 2, 0, 2, 3],
            texture_name: None, base_color: [1.0; 4], center: [0.0; 3],
            render_mode: RenderMode::Opaque, anim: None,
        };
        Arc::new(Collision::build(&ZoneAssets { terrain: vec![slab], objects: vec![], textures: vec![] }, 32.0))
    }

    fn obs(pos: [f32; 3]) -> Observation {
        Observation {
            own: OwnState {
                pos, heading: 0.0, hp: 100, hp_max: 100, hp_verified: true,
                mana: 0, mana_max: 0, endurance: 0, endurance_max: 0, endurance_confirmed: false,
                casting: None, buffs: vec![], zone_name: ZONE.into(),
                target_id: None, target_name: None, auto_attack: false, sitting: false, held: false,
                player_class: "warrior".into(), player_level: 1,
            },
            visible: vec![],
            legal_actions: LegalActionMask { gems: [false; 9], abilities: vec![] },
            dead: false, terminated: false, truncated: false, visibility_available: true, tick: 0,
        }
    }

    fn walker() -> Walker {
        let mut w = Walker::new(NavSlots::default(), NavDebugView::default());
        w.set_zone(ZONE, flat_floor());
        w
    }

    /// Tick until the walker produces a movement, giving the planner thread time to answer.
    fn tick_until_moving(w: &mut Walker, o: &Observation) -> AgentMovement {
        for _ in 0..200 {
            if let Some(m) = w.tick(o) {
                if m.dir != [0.0, 0.0] {
                    return m;
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        panic!("no movement; nav state {:?}", w.slots().status());
    }

    #[test]
    fn zone_cross_walks_to_the_nearest_zone_line() {
        use eqoxide_core::region_map::RegionMap;
        let mut c = Arc::try_unwrap(flat_floor()).ok().unwrap();
        c.set_water(Some(Arc::new(RegionMap::zone_line_box(-5.0, 5.0, 100.0, 110.0, -2.0, 2.0, 7))));
        let mut w = Walker::new(NavSlots::default(), NavDebugView::default());
        w.set_zone(ZONE, Arc::new(c));
        w.zone_cross([0.0, 0.0, 0.0], None).expect("a line exists");
        let goal = w.slots().goto_target.lock().unwrap().unwrap();
        assert!(goal.0 > 99.0 && goal.0 < 111.0, "goal should be inside the line, got {goal:?}");
        let m = tick_until_moving(&mut w, &obs([0.0, 0.0, 0.0]));
        assert!(m.dir[0] > 0.9);
    }

    #[test]
    fn zone_cross_says_why_it_cannot() {
        let mut w = Walker::new(NavSlots::default(), NavDebugView::default());
        assert_eq!(w.zone_cross([0.0; 3], None), Err(ZoneCrossError::ZoneNotLoaded));
        w.set_zone(ZONE, flat_floor());
        assert_eq!(w.zone_cross([0.0; 3], None), Err(ZoneCrossError::NoRegionData));
    }

    #[test]
    fn without_a_goal_the_walker_sends_nothing() {
        let mut w = walker();
        assert!(w.tick(&obs([0.0, 0.0, 0.0])).is_none());
    }

    #[test]
    fn it_drives_toward_a_goal_on_open_ground() {
        let mut w = walker();
        w.slots().goto((80.0, 0.0, 0.0));
        let m = tick_until_moving(&mut w, &obs([0.0, 0.0, 0.0]));
        assert!(m.dir[0] > 0.9, "should head +x toward the goal, got {:?}", m.dir);
        assert!(m.wish_heading.is_some());
    }

    #[test]
    fn arriving_stands_still_once_then_goes_quiet() {
        let mut w = walker();
        w.slots().goto((30.0, 0.0, 0.0));
        tick_until_moving(&mut w, &obs([0.0, 0.0, 0.0]));
        let at_goal = obs([30.0, 0.0, 0.0]);
        let stop = w.tick(&at_goal).expect("explicit stand-still after driving");
        assert_eq!((stop.dir, stop.up, stop.jump), ([0.0, 0.0], 0.0, false));
        assert_eq!(w.slots().status().state, "arrived");
        assert!(w.tick(&at_goal).is_none());
    }

    #[test]
    fn a_dead_character_drops_the_goal_and_halts() {
        let mut w = walker();
        w.slots().goto((80.0, 0.0, 0.0));
        let mut o = obs([0.0, 0.0, 0.0]);
        o.dead = true;
        assert!(w.tick(&o).is_none());
        assert_eq!(w.slots().status().state, NAV_STATE_DEAD);
        assert!(w.slots().goto_target.lock().unwrap().is_none());
        o.dead = false;
        w.tick(&o);
        assert_eq!(w.slots().status().state, "idle");
        assert_eq!(w.slots().status().reason.as_deref(), Some(NAV_REASON_RESPAWNED));
    }

    #[test]
    fn an_unverified_zero_hp_reading_halts_without_claiming_death() {
        let mut w = walker();
        w.slots().goto((80.0, 0.0, 0.0));
        let mut o = obs([0.0, 0.0, 0.0]);
        o.own.hp = 0;
        w.tick(&o);
        assert_eq!(w.slots().status().state, NAV_STATE_HALTED_HP_ZERO);
    }

    #[test]
    fn routing_is_refused_until_the_zones_geometry_is_loaded() {
        let mut w = Walker::new(NavSlots::default(), NavDebugView::default());
        w.slots().goto((80.0, 0.0, 0.0));
        assert!(w.tick(&obs([0.0, 0.0, 0.0])).is_none());
        assert_eq!(w.slots().status().state, NAV_STATE_ZONE_LOADING);
    }

    #[test]
    fn a_stale_grid_is_not_used_for_a_different_zone() {
        let mut w = walker();
        w.slots().goto((80.0, 0.0, 0.0));
        let mut o = obs([0.0, 0.0, 0.0]);
        o.own.zone_name = "elsewhere".into();
        assert!(w.tick(&o).is_none());
        assert_eq!(w.slots().status().reason.as_deref(), Some("zone_changed"));
    }

    #[test]
    fn switching_zones_drops_the_goal() {
        let mut w = walker();
        w.slots().goto((80.0, 0.0, 0.0));
        w.set_zone("other", flat_floor());
        assert!(w.slots().goto_target.lock().unwrap().is_none());
        assert_eq!(w.slots().status().reason.as_deref(), Some(NAV_REASON_ZONED));
    }

    #[test]
    fn following_a_name_that_leaves_view_ends_the_follow() {
        let mut w = walker();
        w.slots().follow("Guard", (80.0, 0.0, 0.0));
        w.tick(&obs([0.0, 0.0, 0.0]));
        assert!(w.slots().goto_entity.lock().unwrap().is_none());
    }

    #[test]
    fn a_lethal_ledge_is_classified_by_worst_case_damage() {
        let (_, worst) = fall_damage(60.0);
        assert_eq!(classify_ledge_fall(60.0, worst as i32), LedgeFallVerdict::Stop { max_dmg: worst });
        assert_eq!(classify_ledge_fall(60.0, 0), LedgeFallVerdict::NoHpReading);
    }
}
