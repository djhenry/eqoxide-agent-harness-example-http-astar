//! The published navigation status and the command slots shared between the walker and whatever
//! serves the harness's API. The walker is the only writer of `status` once a goal is accepted;
//! `NavSlots::goto`/`follow`/`stop` are the only writers of a goal.

use std::sync::{Arc, Mutex};

/// A goal is finished for good in one of these states; anything else asserts work in flight and is
/// retired to `idle` once no goal justifies it.
pub const TERMINAL_NAV_STATES: [&str; 5] = ["idle", "arrived", "no_path", "search_exhausted", "blocked"];

pub fn nav_state_is_terminal(state: &str) -> bool {
    TERMINAL_NAV_STATES.contains(&state)
}

/// Published while no collision grid for the player's current zone is loaded. Not `blocked` (no
/// obstacle) and not `no_path` (no search ran).
pub const NAV_STATE_ZONE_LOADING: &str = "zone_loading";
/// Navigation halted because the observation says the character is dead.
pub const NAV_STATE_DEAD: &str = "dead";
/// Navigation halted on a published HP of zero with no death flag: a fact about the reading, not a
/// death.
pub const NAV_STATE_HALTED_HP_ZERO: &str = "halted_hp_zero";

pub const NAV_REASON_GOAL_DROPPED: &str = "goal_dropped";
pub const NAV_REASON_RESPAWNED: &str = "respawned";
pub const NAV_REASON_PLAYER_DEAD: &str = "player_dead";
pub const NAV_REASON_HP_ZERO_UNCONFIRMED: &str = "hp_zero_unconfirmed";
pub const NAV_REASON_HP_RESTORED: &str = "hp_restored";
pub const NAV_REASON_ZONED: &str = "zoned";

/// A named obstruction with a position. `hazard` is `floor` | `wall` | `water`.
#[derive(Clone, Debug, PartialEq)]
pub struct NavBlockage {
    pub hazard: &'static str,
    pub at: [f32; 3],
}

/// Evidence behind `navigating_stalled`: the walker has a route and is not executing it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NavStall {
    /// Consecutive ticks since the route cursor advanced or the closest approach improved.
    pub quiet_ticks: u32,
    /// The same window in wall-clock milliseconds, measured, not derived from `quiet_ticks`.
    pub quiet_ms: u64,
    pub repaths: u32,
    pub route: &'static str,
}

/// The fine (2 u) local planner's last outcome for the current route.
#[derive(Clone, Debug, PartialEq)]
pub struct NavLocal {
    pub state: String,
    pub reason: String,
    pub stuck_ticks: u32,
    pub plan_us: u64,
}

/// `state` is the status of goal `goal_id`, never of an earlier goal: each accepted command resets
/// `state` to `pending` and bumps `goal_id` under one lock.
#[derive(Clone, Debug, PartialEq)]
pub struct NavStatus {
    pub state: String,
    pub reason: Option<String>,
    pub goal_id: u64,
    pub goal: Option<[f32; 3]>,
    /// What blocks the goal itself / the nearest reachable frontier, on a terminal `no_path`.
    pub blocked_goal: Option<NavBlockage>,
    pub blocked_frontier: Option<NavBlockage>,
    /// Clearance tier of the committed route: `preferred` or `minimum`.
    pub tier: Option<&'static str>,
    pub local: Option<NavLocal>,
    /// `Some` exactly while `state == "navigating_stalled"`.
    pub stall: Option<NavStall>,
    /// The fine planner's worker thread has died. Latched for the life of the worker.
    pub local_planner_dead: bool,
}

impl Default for NavStatus {
    fn default() -> Self {
        NavStatus {
            state: "idle".into(), reason: None, goal_id: 0, goal: None, blocked_goal: None,
            blocked_frontier: None, tier: None, local: None, stall: None, local_planner_dead: false,
        }
    }
}

impl NavStatus {
    /// Retire the goal. `idle` must say how it got there; `reason: None` is reserved for boot.
    pub fn retire_to_idle(&mut self, why: &str) {
        self.state = "idle".into();
        self.reason = Some(why.into());
        self.goal = None;
        self.blocked_goal = None;
        self.blocked_frontier = None;
        self.tier = None;
        self.local = None;
        self.stall = None;
    }

    /// Move to a new word for the same goal. Per-route facts (tier, stall, blockage) belong to the
    /// previous word and are cleared; the goal and the fine planner's verdict carry over.
    pub fn transition_within_goal(&mut self, state: &str, why: Option<&str>) {
        debug_assert!(state != "idle", "`idle` retires the goal: use retire_to_idle");
        self.state = state.into();
        self.reason = why.map(str::to_string);
        self.blocked_goal = None;
        self.blocked_frontier = None;
        self.tier = None;
        self.stall = None;
    }

    fn stamp_fresh_goal(&mut self, goal: [f32; 3]) {
        self.goal_id += 1;
        self.state = "pending".into();
        self.reason = None;
        self.goal = Some(goal);
        self.blocked_goal = None;
        self.blocked_frontier = None;
        self.tier = None;
        self.local = None;
        self.stall = None;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AggroAvoidOpts {
    pub enabled: bool,
    pub buffer: f32,
}

impl Default for AggroAvoidOpts {
    fn default() -> Self {
        Self { enabled: true, buffer: 0.0 }
    }
}

/// Cheap-to-clone handle on the state the walker and the API share.
#[derive(Clone, Default)]
pub struct NavSlots {
    pub(crate) goto_target: Arc<Mutex<Option<(f32, f32, f32)>>>,
    pub(crate) goto_entity: Arc<Mutex<Option<String>>>,
    pub(crate) avoid: Arc<Mutex<AggroAvoidOpts>>,
    pub(crate) status: Arc<Mutex<NavStatus>>,
}

impl NavSlots {
    /// Walk to a fixed point. Returns the new goal id.
    pub fn goto(&self, goal: (f32, f32, f32)) -> u64 {
        *self.goto_entity.lock().unwrap() = None;
        let mut s = self.status.lock().unwrap();
        *self.goto_target.lock().unwrap() = Some(goal);
        s.stamp_fresh_goal([goal.0, goal.1, goal.2]);
        s.goal_id
    }

    /// Follow a named entity, starting from its last known position. Returns the new goal id.
    pub fn follow(&self, name: &str, at: (f32, f32, f32)) -> u64 {
        let mut s = self.status.lock().unwrap();
        *self.goto_entity.lock().unwrap() = Some(name.to_string());
        *self.goto_target.lock().unwrap() = Some(at);
        s.stamp_fresh_goal([at.0, at.1, at.2]);
        s.goal_id
    }

    /// Drop the current goal. The walker retires the status on its next tick.
    pub fn stop(&self) {
        *self.goto_target.lock().unwrap() = None;
        *self.goto_entity.lock().unwrap() = None;
    }

    pub fn set_avoid(&self, opts: AggroAvoidOpts) {
        *self.avoid.lock().unwrap() = opts;
    }

    pub fn status(&self) -> NavStatus {
        self.status.lock().unwrap().clone()
    }
}
