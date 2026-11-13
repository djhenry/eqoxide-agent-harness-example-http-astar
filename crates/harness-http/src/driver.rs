//! The loop that turns HTTP commands into `Step`s. One thread owns the `Link` to eqoxide, receives
//! each `Observation`, and answers it with exactly one `Step`. Route handlers never touch the
//! socket: they publish intent into [`Control`], and the driver reads it on the next observation.

use eqoxide_agent_protocol::call::{Call, CallReply, CallStatus};
use eqoxide_agent_protocol::movement::AgentMovement;
use eqoxide_agent_protocol::observation::Observation;
use eqoxide_agent_protocol::step::Step;
use eqoxide_agent_protocol::verb::AgentVerb;
use eqoxide_agent_protocol::verb_result::VerbOutcome;
use eqoxide_zone_geometry::collision::Collision;
use harness_combat::engage::engage;
use harness_nav::diagnostics::NavDebugView;
use harness_nav::status::{nav_state_is_terminal, NavSlots, NavStatus};
use harness_nav::walker::{Walker, ZoneLine, ZoneLinesError};
use harness_socket::{AgentClient, ClientError};
use std::collections::{HashMap, VecDeque};
use std::path::PathBuf;
#[cfg(test)]
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// A connected Agent Plugin API session, as far as the driver needs it.
pub trait Link: Send + 'static {
    fn recv_observation(&mut self) -> Result<Observation, ClientError>;
    fn send_step(&mut self, step: &Step) -> Result<(), ClientError>;
}

impl Link for AgentClient {
    fn recv_observation(&mut self) -> Result<Observation, ClientError> {
        AgentClient::recv_observation(self)
    }

    fn send_step(&mut self, step: &Step) -> Result<(), ClientError> {
        AgentClient::send_step(self, step)
    }
}

/// Longest `duration_ms` a manual move may request.
pub const MANUAL_MAX_MS: u64 = 5000;

const STAND_STILL: AgentMovement = AgentMovement {
    dir: [0.0, 0.0],
    up: 0.0,
    jump: false,
    wish_heading: None,
};

struct Manual {
    movement: AgentMovement,
    until: Instant,
}

#[derive(Default)]
struct Inner {
    latest: Option<Observation>,
    manual: Option<Manual>,
    /// Verbs waiting for a `Step`, oldest first, each with the `seq` its caller waits on (if any).
    verbs: VecDeque<(AgentVerb, Option<u64>)>,
    /// Callers waiting for eqoxide's answer to a verb already queued or sent, by `seq`.
    verb_waiters: HashMap<u64, tokio::sync::oneshot::Sender<VerbOutcome>>,
    next_seq: u64,
    zone_lines: Option<Result<Vec<ZoneLine>, ZoneLinesError>>,
    notices: VecDeque<String>,
    ended: Option<String>,
    /// Calls waiting for a `Step`, oldest first.
    calls: VecDeque<Call>,
    /// Callers waiting for the reply to a call already sent or queued, by call id.
    waiting: HashMap<u64, tokio::sync::oneshot::Sender<CallReply>>,
    next_call_id: u64,
}

/// Most verbs that may wait for a `Step`.
pub const MAX_QUEUED_VERBS: usize = 16;

/// Most calls that may be sent or queued and not yet answered. eqoxide answers a call past its own
/// bound (32) `Unavailable`; staying under it here means that never happens.
pub const MAX_CALLS_IN_FLIGHT: usize = 32;

/// Longest a caller waits for eqoxide's answer to a verb. A verb rides the next `Step`, one per
/// tick, so this covers a full queue.
pub const VERB_TIMEOUT: Duration = Duration::from_secs(10);

/// What became of a verb sent with [`Control::verb`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum VerbReply {
    /// eqoxide took the verb and handed it to the game.
    Accepted,
    /// eqoxide declined the verb, with the reason. Nothing happened.
    Refused(String),
    /// [`MAX_QUEUED_VERBS`] were already waiting; nothing was queued.
    NotQueued,
    /// No answer will come, and the string says what is known about whether the verb happened.
    Lost(String),
}

/// Longest a caller waits for a call's reply before it is given up on.
pub const CALL_TIMEOUT: Duration = Duration::from_secs(30);

/// Most walker notices kept for `GET /v1/observe/nav`.
pub const MAX_NOTICES: usize = 32;

/// Shortest wait before a zone whose collision failed to load is tried again.
const ZONE_RETRY: Duration = Duration::from_secs(5);

/// State shared between the driver thread and the route handlers.
pub struct Control {
    inner: Mutex<Inner>,
    slots: NavSlots,
}

impl Control {
    pub fn new(slots: NavSlots) -> Arc<Self> {
        Arc::new(Control {
            inner: Mutex::new(Inner::default()),
            slots,
        })
    }

    pub fn slots(&self) -> &NavSlots {
        &self.slots
    }

    /// The most recent `Observation`, or `None` before the first one arrives.
    pub fn latest(&self) -> Option<Observation> {
        self.inner.lock().unwrap().latest.clone()
    }

    /// Why the session ended, or `None` while it is live.
    pub fn ended(&self) -> Option<String> {
        self.inner.lock().unwrap().ended.clone()
    }

    pub fn nav_status(&self) -> NavStatus {
        self.slots.status()
    }

    /// The most recent messages the walker raised (a goal that was changed, a stop that needs
    /// explaining), oldest first. Reading does not consume them, so concurrent pollers each see
    /// them; only the newest [`MAX_NOTICES`] are kept.
    pub fn recent_notices(&self) -> Vec<String> {
        self.inner.lock().unwrap().notices.iter().cloned().collect()
    }

    /// Drives the character with `movement` for `duration`, replacing any nav goal and any earlier
    /// manual move. The character then stands still.
    pub fn manual(&self, movement: AgentMovement, duration: Duration) {
        self.slots.stop();
        let until = Instant::now() + duration.min(Duration::from_millis(MANUAL_MAX_MS));
        self.inner.lock().unwrap().manual = Some(Manual { movement, until });
    }

    /// Ends any manual move, so a nav goal set afterwards is the one that drives the character.
    pub fn cancel_manual(&self) {
        self.inner.lock().unwrap().manual = None;
    }

    /// Stops the character: drops the nav goal and any manual move.
    pub fn stop(&self) {
        self.slots.stop();
        self.cancel_manual();
    }

    /// Queues one verb. The driver sends one verb per `Step`, oldest first. `false` when
    /// [`MAX_QUEUED_VERBS`] are already waiting; nothing is queued then.
    pub fn push_verb(&self, verb: AgentVerb) -> bool {
        let mut inner = self.inner.lock().unwrap();
        if inner.verbs.len() >= MAX_QUEUED_VERBS {
            return false;
        }
        inner.verbs.push_back((verb, None));
        true
    }

    /// Queues `verb` and waits for eqoxide's answer to it, so a refusal reaches the caller instead of
    /// being reported as success.
    pub async fn verb(&self, verb: AgentVerb) -> VerbReply {
        let (tx, rx) = tokio::sync::oneshot::channel();
        let seq = {
            let mut inner = self.inner.lock().unwrap();
            if let Some(why) = &inner.ended {
                return VerbReply::Lost(format!("the session has ended: {why}"));
            }
            if inner.verbs.len() >= MAX_QUEUED_VERBS {
                return VerbReply::NotQueued;
            }
            let seq = inner.next_seq;
            inner.next_seq += 1;
            inner.verb_waiters.insert(seq, tx);
            inner.verbs.push_back((verb, Some(seq)));
            seq
        };
        match tokio::time::timeout(VERB_TIMEOUT, rx).await {
            Ok(Ok(VerbOutcome::Accepted)) => VerbReply::Accepted,
            Ok(Ok(VerbOutcome::Refused(why))) => VerbReply::Refused(why),
            Ok(Err(_)) => VerbReply::Lost(
                "the verb was dropped before eqoxide answered (the zone changed, the character died or respawned, or the session ended); it was not acted on".into(),
            ),
            Err(_) => {
                let mut inner = self.inner.lock().unwrap();
                inner.verb_waiters.remove(&seq);
                let before = inner.verbs.len();
                inner.verbs.retain(|(_, s)| *s != Some(seq));
                if inner.verbs.len() < before {
                    VerbReply::Lost("eqoxide did not take the verb in time; it was not sent".into())
                } else {
                    VerbReply::Lost("eqoxide did not answer the verb in time; it may or may not have happened".into())
                }
            }
        }
    }

    #[cfg(test)]
    pub(crate) fn verb_queue_len(&self) -> usize {
        self.inner.lock().unwrap().verbs.len()
    }

    /// Runs `op` on eqoxide and waits for its reply. A call that gets no reply within
    /// [`CALL_TIMEOUT`], or whose session ends first, is answered `Unavailable` here: the caller is
    /// never told a call happened when eqoxide did not say so.
    pub async fn call(&self, op: String, args: serde_json::Value) -> CallReply {
        let (tx, rx) = tokio::sync::oneshot::channel();
        let id = {
            let mut inner = self.inner.lock().unwrap();
            let id = inner.next_call_id;
            inner.next_call_id += 1;
            if let Some(why) = &inner.ended {
                return unavailable(id, format!("the session has ended: {why}"));
            }
            if inner.waiting.len() >= MAX_CALLS_IN_FLIGHT {
                return unavailable(id, "too many calls are waiting for a reply".into());
            }
            inner.waiting.insert(id, tx);
            inner.calls.push_back(Call { id, op, args });
            id
        };
        match tokio::time::timeout(CALL_TIMEOUT, rx).await {
            Ok(Ok(reply)) => reply,
            Ok(Err(_)) => unavailable(id, "the session ended before the call was answered".into()),
            Err(_) => {
                let mut inner = self.inner.lock().unwrap();
                inner.waiting.remove(&id);
                inner.calls.retain(|c| c.id != id);
                unavailable(
                    id,
                    "eqoxide did not answer the call in time; it may or may not have happened"
                        .into(),
                )
            }
        }
    }

    /// The zone lines of the zone the character is in, or `None` before the first observation.
    pub fn zone_lines(&self) -> Option<Result<Vec<ZoneLine>, ZoneLinesError>> {
        self.inner.lock().unwrap().zone_lines.clone()
    }

    /// Makes `obs` the latest observation without running the driver, for route tests. The driver
    /// itself publishes inside [`Driver::tick`], after it has loaded the zone and dropped stale
    /// verbs, so a caller who sees an observation never has a verb queued in response thrown away.
    #[cfg(test)]
    pub(crate) fn publish(&self, obs: &Observation) {
        self.inner.lock().unwrap().latest = Some(obs.clone());
    }

    #[cfg(test)]
    pub(crate) fn end_for_test(&self, why: &str) {
        self.end(why.into());
    }

    fn end(&self, why: String) {
        let mut inner = self.inner.lock().unwrap();
        inner.ended = Some(why);
        // Dropping the senders wakes every waiting caller.
        inner.waiting.clear();
        inner.calls.clear();
        inner.verb_waiters.clear();
    }
}

fn unavailable(id: u64, why: String) -> CallReply {
    CallReply {
        id,
        status: CallStatus::Unavailable,
        body: serde_json::Value::String(why),
    }
}

/// Loads the collision grid of a zone by name, `None` when the zone cannot be loaded.
pub type ZoneLoader = Box<dyn FnMut(&str) -> Option<Arc<Collision>> + Send>;

/// A [`ZoneLoader`] that reads zones from eqoxide's asset cache.
pub fn cache_loader(asset_cache_dir: PathBuf) -> ZoneLoader {
    Box::new(
        move |zone| match harness_nav::zone::load_zone_from_cache(&asset_cache_dir, zone) {
            Ok(c) => Some(Arc::new(c)),
            Err(e) => {
                tracing::warn!("zone '{zone}' not loaded: {e}");
                None
            }
        },
    )
}

/// Computes the `Step` that answers each `Observation`.
pub struct Driver {
    control: Arc<Control>,
    walker: Walker,
    loader: ZoneLoader,
    loaded_zone: Option<String>,
    /// The zone a load was last attempted for, and when; a failed load is retried after
    /// [`ZONE_RETRY`] rather than every tick.
    last_attempt: Option<(String, Instant)>,
    /// Death state seen on the previous tick, to drop verbs queued before a death or respawn.
    last_dead: bool,
    manual_active: bool,
    engaging: bool,
}

impl Driver {
    pub fn new(control: Arc<Control>, loader: ZoneLoader) -> Self {
        let walker = Walker::new(control.slots.clone(), NavDebugView::default());
        Driver {
            control,
            walker,
            loader,
            loaded_zone: None,
            last_attempt: None,
            last_dead: false,
            manual_active: false,
            engaging: false,
        }
    }

    /// Answers `obs` with one `Step`.
    pub fn tick(&mut self, obs: &Observation) -> Step {
        let zone_changed = self.sync_zone(obs);
        // Verbs queued against the old zone, or before a death or respawn, no longer mean what
        // the caller intended.
        let stale = zone_changed || obs.dead != self.last_dead;
        self.last_dead = obs.dead;
        let movement = self.movement(obs);
        let notices = self.walker.take_notices();
        let mut inner = self.control.inner.lock().unwrap();
        for n in notices {
            if inner.notices.len() >= MAX_NOTICES {
                inner.notices.pop_front();
            }
            inner.notices.push_back(n);
        }
        if stale {
            let dropped: Vec<u64> = inner.verbs.drain(..).filter_map(|(_, s)| s).collect();
            for seq in dropped {
                // Dropping the sender tells the caller its verb was not acted on.
                inner.verb_waiters.remove(&seq);
            }
        }
        inner.latest = Some(obs.clone());
        for reply in &obs.call_replies {
            if let Some(waiter) = inner.waiting.remove(&reply.id) {
                let _ = waiter.send(reply.clone());
            }
        }
        for r in &obs.verb_results {
            if let Some(waiter) = inner.verb_waiters.remove(&r.seq) {
                let _ = waiter.send(r.outcome.clone());
            }
        }
        let (verb, seq) = match inner.verbs.pop_front() {
            Some((v, s)) => (Some(v), s),
            None => (None, None),
        };
        let calls = inner.calls.drain(..).collect();
        Step {
            movement,
            verb,
            seq,
            calls,
        }
    }

    /// Loads the collision of the zone `obs` is in. A zone that fails to load is not recorded as
    /// loaded, so it is retried after [`ZONE_RETRY`]. Returns whether the character moved to a
    /// different zone since the last attempt.
    fn sync_zone(&mut self, obs: &Observation) -> bool {
        let zone = obs.own.zone_name.as_str();
        if self.loaded_zone.as_deref() == Some(zone) {
            return false;
        }
        let same_zone = self.last_attempt.as_ref().is_some_and(|(z, _)| z == zone);
        if same_zone
            && self
                .last_attempt
                .as_ref()
                .is_some_and(|(_, at)| at.elapsed() < ZONE_RETRY)
        {
            return false;
        }
        // The first zone of a session is not a change.
        let changed = self.last_attempt.is_some() && !same_zone;
        self.last_attempt = Some((zone.to_string(), Instant::now()));
        match (self.loader)(zone) {
            Some(collision) => {
                self.walker.set_zone(zone, collision);
                self.loaded_zone = Some(zone.to_string());
            }
            None => {
                self.walker.clear_zone();
                // Returning to the previous zone must reload it, not trust a stale marker.
                self.loaded_zone = None;
            }
        }
        // Zone lines are fixed points of the zone, computed once per load attempt.
        let lines = self.walker.zone_lines(obs.own.pos);
        self.control.inner.lock().unwrap().zone_lines = Some(lines);
        changed
    }

    /// The movement for this tick. A latched movement keeps applying on the eqoxide side, so the
    /// tick after a manual move or an engage ends sends an explicit stand-still.
    fn movement(&mut self, obs: &Observation) -> Option<AgentMovement> {
        let manual = {
            let mut inner = self.control.inner.lock().unwrap();
            match &inner.manual {
                Some(m) if Instant::now() < m.until => Some(m.movement),
                Some(_) => {
                    inner.manual = None;
                    None
                }
                None => None,
            }
        };
        if let Some(m) = manual {
            self.manual_active = true;
            self.engaging = false;
            return Some(m);
        }
        if self.manual_active {
            self.manual_active = false;
            return Some(STAND_STILL);
        }
        if let Some(m) = self.walker.tick(obs) {
            self.engaging = false;
            return Some(m);
        }
        if !nav_state_is_terminal(&self.control.slots.status().state) {
            return None;
        }
        match engage(obs) {
            Some(m) => {
                self.engaging = true;
                Some(m)
            }
            None if self.engaging => {
                self.engaging = false;
                Some(STAND_STILL)
            }
            None => None,
        }
    }
}

/// Runs the driver until the link fails, publishing every observation to `control`.
pub fn run(mut link: impl Link, mut driver: Driver) {
    let control = driver.control.clone();
    let why = loop {
        let obs = match link.recv_observation() {
            Ok(o) => o,
            Err(e) => break format!("observation stream ended: {e}"),
        };
        let step = driver.tick(&obs);
        if let Err(e) = link.send_step(&step) {
            break format!("step not sent: {e}");
        }
    };
    control.end(why);
}

/// Starts the driver on its own thread.
pub fn spawn(
    link: impl Link,
    control: Arc<Control>,
    loader: ZoneLoader,
) -> std::thread::JoinHandle<()> {
    let driver = Driver::new(control, loader);
    std::thread::spawn(move || run(link, driver))
}

/// A `Link` fed from a channel, for tests.
#[cfg(test)]
pub struct ChannelLink {
    pub observations: Receiver<Observation>,
    pub steps: Sender<Step>,
}

#[cfg(test)]
impl Link for ChannelLink {
    fn recv_observation(&mut self) -> Result<Observation, ClientError> {
        self.observations
            .recv()
            .map_err(|_| ClientError::ConnectionClosed)
    }

    fn send_step(&mut self, step: &Step) -> Result<(), ClientError> {
        self.steps
            .send(step.clone())
            .map_err(|_| ClientError::ConnectionClosed)
    }
}
