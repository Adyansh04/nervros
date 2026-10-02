//! Missions: the only way the agent moves the robot.
//!
//! The model calls `plan_mission` with a plan; it compiles against the robot's skill catalog and
//! world model, and the executor's `ValidateMission` checks the tree again. Nothing moves, and
//! the model gets the tree's hash back. `run_mission` with that hash is the act: the guard asks
//! the operator, then the tree goes to the executor's `ExecuteMission`. The call returns once the
//! executor accepts; the mission runs in the background, its progress goes out as session events,
//! and when it ends the model gets a report with the outcome and the goal checks.

pub mod advice;
pub mod camera;
pub mod catalog;
pub mod check;
pub mod history;
pub mod ledger;
pub mod plan;
pub mod preview;
pub mod sanity;
pub(crate) mod schedule;

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use nervros_ros::{Publisher, RobotPort};
use serde::Serialize;
use serde_json::{Value, json};

use self::catalog::Catalog;
use self::ledger::Ledger;
use self::plan::Compiled;
use self::sanity::Critic;
use crate::lock;
use crate::profile::{MissionConfig, Profile};
use crate::session::{Event, SessionHandle};
use crate::tools::Tool;

mod execution;
mod planning;
mod report;
#[cfg(test)]
mod tests;
mod tool;

use tool::RunMission;

const SERVICE_TIMEOUT: Duration = Duration::from_secs(10);
const WORLD_WAIT: Duration = Duration::from_secs(2);
/// Plans kept for `run_mission`, newest last.
const KEPT_PLANS: usize = 8;
/// Failed `plan_mission` calls before the model must ask the operator instead. Small local models
/// need about three to fix a plan from its problem list.
const MAX_PLAN_ATTEMPTS: u32 = 4;
/// How long the critic may think before a plan goes on without its opinion.
const CRITIC_TIMEOUT: Duration = Duration::from_secs(30);
/// A hash may be shortened to this many characters when it stays unique.
const MIN_HASH_PREFIX: usize = 8;
/// How long a failed catalog fetch stands before a turn asks again.
const CATALOG_RETRY: Duration = Duration::from_secs(30);
/// How often the executor is checked on while a mission runs.
const LIVENESS: Duration = Duration::from_secs(5);
/// Checks in a row without the executor saying it runs the mission before it is given up as
/// lost: a crashed executor sends no result, and its mission would hold the slot for good.
const LOST_AFTER: u32 = 6;
const EXECUTE: &str = "nervros_interfaces/action/ExecuteMission";
const STATE: &str = "nervros_interfaces/msg/RobotState";
const HEARTBEAT: &str = "nervros_interfaces/msg/Heartbeat";
const STATUSES: [&str; 5] = ["idle", "running", "success", "failure", "skipped"];

/// The executor sends its state at least once a second: an older one says how things were, not
/// how they are.
pub const STATE_FRESH: Duration = Duration::from_secs(3);

/// How a mission ended, in the order of the executor's `outcome` codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Outcome {
    /// Every step succeeded.
    Success,
    /// A step failed.
    Failure,
    /// A stop ended it: the operator's, the deadman's or the robot's own.
    Canceled,
    /// It ran past its time limit.
    Timeout,
    /// The executor refused the plan.
    Rejected,
    /// The executor or the link to it failed.
    Error,
}

impl Outcome {
    const BY_CODE: [Self; 6] = [
        Self::Success,
        Self::Failure,
        Self::Canceled,
        Self::Timeout,
        Self::Rejected,
        Self::Error,
    ];

    /// Its name, as the model, the log and the ledger read it.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Success => "success",
            Self::Failure => "failure",
            Self::Canceled => "canceled",
            Self::Timeout => "timeout",
            Self::Rejected => "rejected",
            Self::Error => "error",
        }
    }
}

impl std::fmt::Display for Outcome {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// What a hand holds when the executor cannot say: after a pick that was cut off, it trusts the
/// hand again only once a place with it succeeds.
pub const UNKNOWN_HELD: &str = "something unknown";

/// What a `RobotState` (as JSON) says one hand holds: an object id, [`UNKNOWN_HELD`], or empty
/// for nothing. The executor leaves `holding_*` empty for an unknown hand and says so in
/// `message`, as "<arm> hand unknown".
#[must_use]
pub fn held_by(state: &Value, arm: &str) -> String {
    let held = state[format!("holding_{arm}")].as_str().unwrap_or_default();
    let message = state["message"].as_str().unwrap_or_default();
    if held.is_empty() && message.contains(&format!("{arm} hand unknown")) {
        UNKNOWN_HELD.to_owned()
    } else {
        held.to_owned()
    }
}

/// The mission in flight.
struct Running {
    id: String,
}

/// The mission slot, from the moment a launch claims it until the executor takes the goal. Dropped
/// any earlier, by a failure or a stop, it frees the slot again.
struct Claim<'a> {
    missions: &'a Missions,
    id: String,
    kept: bool,
}

impl Drop for Claim<'_> {
    fn drop(&mut self) {
        if !self.kept {
            self.missions.release(&self.id);
        }
    }
}

/// Plans, the running mission and the counters that stop endless retries.
pub struct Missions {
    robot: Arc<dyn RobotPort>,
    /// The profile's places and the remembered ones.
    places: Arc<crate::places::Places>,
    profile: Profile,
    config: MissionConfig,
    catalog: Mutex<Option<Arc<Catalog>>>,
    /// When fetching the catalog last failed: a robot without an executor is not asked again
    /// before every turn.
    catalog_failed: Mutex<Option<Instant>>,
    planned: Mutex<VecDeque<Compiled>>,
    running: Mutex<Option<Running>>,
    plan_failures: AtomicU32,
    /// Missions of the operator's latest request that failed.
    run_failures: AtomicU32,
    /// The operator's messages in this session.
    said: AtomicU32,
    /// The plan that ran last in this request, and how it ended: run again unchanged on the
    /// model's own, it repeats a done task or fails the same way.
    last: Mutex<Option<(String, String)>>,
    session: OnceLock<SessionHandle>,
    /// Where the session's pulse goes: the executor's deadman, fed while the loop that serves
    /// Stop runs.
    heartbeat: Mutex<Option<Publisher>>,
    /// Who this agent is in its heartbeats, so the executor counts only its own.
    client: String,
    /// Where finished missions are kept, for track records, recall and replay.
    ledger: OnceLock<Arc<Ledger>>,
    /// What the operator said last: the request a mission is for.
    request: Mutex<String>,
    /// A second opinion on the model's plans, when a model has the role.
    critic: OnceLock<Arc<dyn Critic>>,
    /// A stronger model to ask when the model's plans keep failing, when one has the role.
    advisor: OnceLock<Arc<dyn advice::Advisor>>,
    /// The camera's view of a mission's end, when the robot has a camera and a vision model.
    vision: OnceLock<camera::Vision>,
    /// Whose resource locks a running mission holds, so no other act overlaps it.
    guard: OnceLock<Arc<crate::guard::Guard>>,
    /// Whether a plan for this request already went back for not matching the operator's words:
    /// once is a hint, twice would argue with a model that may be right.
    questioned: AtomicBool,
}

impl crate::session::Pulse for Missions {
    fn pulse(&self) {
        if let Some(publisher) = lock(&self.heartbeat).as_ref() {
            let since = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default();
            publisher.send(json!({
                "stamp": {"sec": since.as_secs(), "nanosec": since.subsec_nanos()},
                "client": self.client,
            }));
        }
    }
}

impl std::fmt::Debug for Missions {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Missions")
            .field("executor", &self.config.execute)
            .finish_non_exhaustive()
    }
}

impl Missions {
    /// For a profile with a `[mission]` section.
    #[must_use]
    pub fn new(
        profile: &Profile,
        places: Arc<crate::places::Places>,
        robot: Arc<dyn RobotPort>,
    ) -> Option<Arc<Self>> {
        let config = profile.mission.clone()?;
        Some(Arc::new(Self {
            robot,
            places,
            profile: profile.clone(),
            config,
            catalog: Mutex::default(),
            catalog_failed: Mutex::default(),
            planned: Mutex::default(),
            running: Mutex::default(),
            plan_failures: AtomicU32::new(0),
            run_failures: AtomicU32::new(0),
            said: AtomicU32::new(0),
            last: Mutex::default(),
            session: OnceLock::new(),
            heartbeat: Mutex::default(),
            client: format!("nervros {}", std::process::id()),
            ledger: OnceLock::new(),
            request: Mutex::default(),
            critic: OnceLock::new(),
            advisor: OnceLock::new(),
            vision: OnceLock::new(),
            guard: OnceLock::new(),
            questioned: AtomicBool::new(false),
        }))
    }

    /// Keeps finished missions in `ledger` from now on.
    #[must_use]
    pub fn with_ledger(self: Arc<Self>, ledger: Arc<Ledger>) -> Arc<Self> {
        // Set once, at start-up.
        let _ = self.ledger.set(ledger);
        self
    }

    /// Asks `critic` about each plan the model makes, before the operator sees it.
    #[must_use]
    pub fn with_critic(self: Arc<Self>, critic: Arc<dyn Critic>) -> Arc<Self> {
        // Set once, at start-up.
        let _ = self.critic.set(critic);
        self
    }

    /// Asks `advisor` how to fix a request's plans once they have failed twice.
    #[must_use]
    pub fn with_advisor(self: Arc<Self>, advisor: Arc<dyn advice::Advisor>) -> Arc<Self> {
        // Set once, at start-up.
        let _ = self.advisor.set(advisor);
        self
    }

    /// Holds `guard`'s locks on the whole robot while a mission runs: an act tool waits for the
    /// mission's end, and a mission for the act's.
    #[must_use]
    pub fn with_guard(self: Arc<Self>, guard: Arc<crate::guard::Guard>) -> Arc<Self> {
        // Set once, at start-up.
        let _ = self.guard.set(guard);
        self
    }

    /// Checks each successful mission's goals through the camera too.
    #[must_use]
    pub fn with_vision(self: Arc<Self>, vision: camera::Vision) -> Arc<Self> {
        // Set once, at start-up.
        let _ = self.vision.set(vision);
        self
    }

    /// The robot the missions run on.
    pub(crate) fn robot(&self) -> &Arc<dyn RobotPort> {
        &self.robot
    }

    /// Topics the profile keeps from being read, for the triggers that watch topics.
    pub(crate) fn read_deny(&self) -> &[String] {
        self.profile
            .ros_tools
            .as_ref()
            .map_or(&[], |c| c.read_deny.as_slice())
    }

    /// Where finished missions are kept, if anywhere.
    #[must_use]
    pub fn ledger(&self) -> Option<&Arc<Ledger>> {
        self.ledger.get()
    }

    /// `run_mission`, the one tool: it checks a plan, and runs it once the operator approves.
    #[must_use]
    pub fn tools(self: &Arc<Self>) -> [Arc<dyn Tool>; 1] {
        [Arc::new(RunMission(Arc::clone(self)))]
    }

    /// Connects to the session so missions can report; also loads the catalog, so the first
    /// turn's plan tool already lists the skills.
    pub fn attach(self: &Arc<Self>, session: SessionHandle) {
        let mut events = session.subscribe();
        if self.session.set(session).is_err() {
            return;
        }
        // Held weakly, so that the missions end with the session.
        let me = Arc::downgrade(self);
        let first = Arc::clone(self);
        tokio::spawn(async move {
            first.start_heartbeat().await;
            if let Err(e) = first.catalog().await {
                tracing::info!(error = %e, "no mission catalog yet");
            }
            drop(first);
            // The operator speaking resets the retry limits: it is a new request.
            loop {
                let event = events.recv().await;
                let Some(me) = me.upgrade() else { return };
                match event {
                    // Said while the agent works, it adds to the request rather than replacing it.
                    Ok(Event::Steer { text, .. }) => {
                        let mut request = lock(&me.request);
                        request.push(' ');
                        request.push_str(&text);
                    }
                    Ok(Event::User { text, .. }) => {
                        *lock(&me.request) = text;
                        me.said.fetch_add(1, Ordering::SeqCst);
                        me.plan_failures.store(0, Ordering::SeqCst);
                        me.run_failures.store(0, Ordering::SeqCst);
                        me.questioned.store(false, Ordering::SeqCst);
                        *lock(&me.last) = None;
                    }
                    Ok(_) | Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {}
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => return,
                }
            }
        });
    }

    /// Keeps a publisher for the heartbeat, if the profile names where; the session's pulse
    /// sends the beats.
    async fn start_heartbeat(&self) {
        let Some(topic) = &self.config.heartbeat else {
            return;
        };
        match self.robot.publisher(topic, HEARTBEAT).await {
            Ok(publisher) => {
                *lock(&self.heartbeat) = Some(publisher);
            }
            Err(e) => self.emit(Event::Notice {
                text: format!(
                    "no heartbeat on {topic} ({e}): a mission will not stop by itself if NervROS stops"
                ),
            }),
        }
    }

    /// Reports into the session, once one is attached.
    fn emit(&self, event: Event) {
        if let Some(s) = self.session.get() {
            s.emit(event);
        }
    }

    /// Whether a mission is running now.
    #[must_use]
    pub fn busy(&self) -> bool {
        lock(&self.running).is_some()
    }

    /// What the operator said last.
    pub(crate) fn request(&self) -> String {
        lock(&self.request).clone()
    }

    /// Frees the mission slot, if mission `id` still holds it.
    fn release(&self, id: &str) {
        let mut running = lock(&self.running);
        if running.as_ref().is_some_and(|r| r.id == id) {
            *running = None;
        }
    }
}
