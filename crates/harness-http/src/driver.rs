//! The loop that turns HTTP commands into `Step`s. One thread owns the `Link` to eqoxide, receives
//! each `Observation`, and answers it with exactly one `Step`. Route handlers never touch the
//! socket: they publish intent into [`Control`], and the driver reads it on the next observation.

use eqoxide_agent_protocol::movement::AgentMovement;
use eqoxide_agent_protocol::observation::Observation;
use eqoxide_agent_protocol::step::Step;
use eqoxide_agent_protocol::verb::AgentVerb;
use eqoxide_zone_geometry::collision::Collision;
use harness_combat::engage::engage;
use harness_nav::diagnostics::NavDebugView;
use harness_nav::status::{nav_state_is_terminal, NavSlots, NavStatus};
use harness_nav::walker::{Walker, ZoneLine, ZoneLinesError};
use harness_socket::{AgentClient, ClientError};
use std::collections::VecDeque;
use std::path::PathBuf;
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

const STAND_STILL: AgentMovement = AgentMovement { dir: [0.0, 0.0], up: 0.0, jump: false, wish_heading: None };

struct Manual {
    movement: AgentMovement,
    until: Instant,
}

#[derive(Default)]
struct Inner {
    latest: Option<Observation>,
    manual: Option<Manual>,
    verbs: VecDeque<AgentVerb>,
    zone_lines: Option<Result<Vec<ZoneLine>, ZoneLinesError>>,
    notices: Vec<String>,
    ended: Option<String>,
}

/// State shared between the driver thread and the route handlers.
pub struct Control {
    inner: Mutex<Inner>,
    slots: NavSlots,
}

impl Control {
    pub fn new(slots: NavSlots) -> Arc<Self> {
        Arc::new(Control { inner: Mutex::new(Inner::default()), slots })
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

    /// Messages the walker raised since the last call (a goal that was changed, a stop that needs
    /// explaining).
    pub fn take_notices(&self) -> Vec<String> {
        std::mem::take(&mut self.inner.lock().unwrap().notices)
    }

    /// Drives the character with `movement` for `duration`, replacing any nav goal and any earlier
    /// manual move. The character then stands still.
    pub fn manual(&self, movement: AgentMovement, duration: Duration) {
        self.slots.stop();
        let until = Instant::now() + duration.min(Duration::from_millis(MANUAL_MAX_MS));
        self.inner.lock().unwrap().manual = Some(Manual { movement, until });
    }

    /// Queues one verb. The driver sends one verb per `Step`, oldest first.
    pub fn push_verb(&self, verb: AgentVerb) {
        self.inner.lock().unwrap().verbs.push_back(verb);
    }

    /// The zone lines of the zone the character is in, or `None` before the first observation.
    pub fn zone_lines(&self) -> Option<Result<Vec<ZoneLine>, ZoneLinesError>> {
        self.inner.lock().unwrap().zone_lines.clone()
    }

    pub(crate) fn publish(&self, obs: &Observation) {
        self.inner.lock().unwrap().latest = Some(obs.clone());
    }

    fn end(&self, why: String) {
        self.inner.lock().unwrap().ended = Some(why);
    }
}

/// Loads the collision grid of a zone by name, `None` when the zone cannot be loaded.
pub type ZoneLoader = Box<dyn FnMut(&str) -> Option<Arc<Collision>> + Send>;

/// A [`ZoneLoader`] that reads zones from eqoxide's asset cache.
pub fn cache_loader(asset_cache_dir: PathBuf) -> ZoneLoader {
    Box::new(move |zone| match harness_nav::zone::load_zone_from_cache(&asset_cache_dir, zone) {
        Ok(c) => Some(Arc::new(c)),
        Err(e) => {
            tracing::warn!("zone '{zone}' not loaded: {e}");
            None
        }
    })
}

/// Computes the `Step` that answers each `Observation`.
pub struct Driver {
    control: Arc<Control>,
    walker: Walker,
    loader: ZoneLoader,
    loaded_zone: Option<String>,
    manual_active: bool,
    engaging: bool,
}

impl Driver {
    pub fn new(control: Arc<Control>, loader: ZoneLoader) -> Self {
        let walker = Walker::new(control.slots.clone(), NavDebugView::default());
        Driver { control, walker, loader, loaded_zone: None, manual_active: false, engaging: false }
    }

    /// Answers `obs` with one `Step`.
    pub fn tick(&mut self, obs: &Observation) -> Step {
        self.sync_zone(obs);
        let movement = self.movement(obs);
        let notices = self.walker.take_notices();
        let mut inner = self.control.inner.lock().unwrap();
        inner.notices.extend(notices);
        let verb = inner.verbs.pop_front();
        Step { movement, verb }
    }

    fn sync_zone(&mut self, obs: &Observation) {
        let zone = obs.own.zone_name.as_str();
        if self.loaded_zone.as_deref() == Some(zone) {
            return;
        }
        match (self.loader)(zone) {
            Some(collision) => self.walker.set_zone(zone, collision),
            None => self.walker.clear_zone(),
        }
        self.loaded_zone = Some(zone.to_string());
        let lines = self.walker.zone_lines(obs.own.pos);
        self.control.inner.lock().unwrap().zone_lines = Some(lines);
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
        control.publish(&obs);
        let step = driver.tick(&obs);
        if let Err(e) = link.send_step(&step) {
            break format!("step not sent: {e}");
        }
    };
    control.end(why);
}

/// Starts the driver on its own thread.
pub fn spawn(link: impl Link, control: Arc<Control>, loader: ZoneLoader) -> std::thread::JoinHandle<()> {
    let driver = Driver::new(control, loader);
    std::thread::spawn(move || run(link, driver))
}

/// A `Link` fed from a channel, for tests.
pub struct ChannelLink {
    pub observations: Receiver<Observation>,
    pub steps: Sender<Step>,
}

impl Link for ChannelLink {
    fn recv_observation(&mut self) -> Result<Observation, ClientError> {
        self.observations.recv().map_err(|_| ClientError::ConnectionClosed)
    }

    fn send_step(&mut self, step: &Step) -> Result<(), ClientError> {
        self.steps.send(step.clone()).map_err(|_| ClientError::ConnectionClosed)
    }
}
