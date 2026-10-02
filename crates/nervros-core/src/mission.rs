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

use std::borrow::Cow;
use std::collections::VecDeque;
use std::fmt::Write as _;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use nervros_ros::{Frame, Goal, GoalResult, Publisher, RobotPort, RosError};
use serde_json::{Value, json};
use tracing::Instrument as _;

use self::catalog::Catalog;
use self::check::Observed;
use self::ledger::{Ledger, MissionRecord, StepRecord};
use self::plan::{Author as By, Compiled, Plan, PlannedStep, Thing, World};
use self::sanity::{Concern, Critic, Verdict};
use crate::profile::{MissionConfig, Profile};
use crate::session::{Command, Event, SessionHandle};
use crate::tools::{Assessment, Risk, Status, Tool, ToolOutcome, ToolSpec};

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
const EXECUTE: &str = "nervros_interfaces/action/ExecuteMission";
const STATE: &str = "nervros_interfaces/msg/RobotState";
const HEARTBEAT: &str = "nervros_interfaces/msg/Heartbeat";
const OUTCOMES: [&str; 6] = [
    "success", "failure", "canceled", "timeout", "rejected", "error",
];
const STATUSES: [&str; 5] = ["idle", "running", "success", "failure", "skipped"];

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

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// The mission in flight.
struct Running {
    id: String,
}

/// Plans, the running mission and the counters that stop endless retries.
pub struct Missions {
    robot: Arc<dyn RobotPort>,
    /// The profile's places and the remembered ones.
    places: Arc<crate::places::Places>,
    profile: Profile,
    config: MissionConfig,
    catalog: Mutex<Option<Catalog>>,
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
    /// Whether it does: a mission that asked for a deadman without one would be stopped at once.
    heartbeat_live: AtomicBool,
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
            planned: Mutex::default(),
            running: Mutex::default(),
            plan_failures: AtomicU32::new(0),
            run_failures: AtomicU32::new(0),
            said: AtomicU32::new(0),
            last: Mutex::default(),
            session: OnceLock::new(),
            heartbeat: Mutex::default(),
            heartbeat_live: AtomicBool::new(false),
            client: format!("nervros {}", std::process::id()),
            ledger: OnceLock::new(),
            request: Mutex::default(),
            critic: OnceLock::new(),
            advisor: OnceLock::new(),
            vision: OnceLock::new(),
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
        let me = Arc::clone(self);
        tokio::spawn(async move {
            me.start_heartbeat().await;
            if let Err(e) = me.catalog().await {
                tracing::info!(error = %e, "no mission catalog yet");
            }
            // The operator speaking resets the retry limits: it is a new request.
            loop {
                match events.recv().await {
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
                self.heartbeat_live.store(true, Ordering::SeqCst);
            }
            Err(e) => self.emit(Event::Notice {
                text: format!(
                    "no heartbeat on {topic} ({e}): a mission will not stop by itself if NervROS stops"
                ),
            }),
        }
    }

    /// Asks the executor where a checked plan would take the robot, and tells the window: the
    /// approval card shows at once, and the picture follows when the planner has answered.
    async fn preview(&self, hash: &str) {
        let Some(service) = &self.config.preview else {
            return;
        };
        let Ok(compiled) = self.find(hash) else {
            return;
        };
        let reply = self
            .robot
            .call(
                service,
                "nervros_interfaces/srv/PreviewMission",
                json!({"tree_xml": compiled.xml}),
                SERVICE_TIMEOUT,
            )
            .await;
        match reply
            .map_err(|e| e.to_string())
            .and_then(|r| preview::parse(&r))
        {
            Ok(steps) => self.emit(Event::MissionPreview {
                hash: compiled.sha256,
                steps,
            }),
            Err(why) => tracing::info!(%why, "no preview of the plan"),
        }
    }

    /// Why the robot cannot run a plan that walks now, from the executor's own state.
    async fn cannot_walk(&self, compiled: &Compiled, catalog: &Catalog) -> Option<String> {
        let walks = compiled.steps.iter().any(|s| {
            s.skill == "GoToPlace"
                || catalog
                    .skill(&s.skill)
                    .is_some_and(|k| k.resources.iter().any(|r| r == "base"))
        });
        if !walks {
            return None;
        }
        let state = self
            .robot
            .latest(&self.config.state, STATE, Duration::from_secs(1))
            .await
            .ok()?;
        if state["can_move"].as_bool() != Some(false) {
            return None;
        }
        let reason = state["cannot_move_reason"]
            .as_str()
            .filter(|r| !r.is_empty())
            .unwrap_or("the robot says so");
        Some(format!(
            "the robot cannot walk now: {reason}. Tell the operator; plan no walk until they say it is fixed"
        ))
    }

    fn emit(&self, event: Event) {
        if let Some(s) = self.session.get() {
            s.emit(event);
        }
    }

    /// The catalog, fetched once and kept.
    async fn catalog(&self) -> Result<Catalog, String> {
        if let Some(c) = lock(&self.catalog).clone() {
            return Ok(c);
        }
        let reply = self
            .robot
            .call(
                &self.config.catalog,
                "nervros_interfaces/srv/GetCatalog",
                json!({}),
                SERVICE_TIMEOUT,
            )
            .await
            .map_err(|e| format!("the robot's skill catalog is not available: {e}"))?;
        let text = reply["catalog_json"].as_str().unwrap_or_default();
        let catalog =
            Catalog::parse(text).map_err(|e| format!("the skill catalog is malformed: {e}"))?;
        *lock(&self.catalog) = Some(catalog.clone());
        Ok(catalog)
    }

    async fn latest(&self, topic: Option<&crate::profile::TopicRef>) -> Value {
        match topic {
            Some(t) => self
                .robot
                .latest(&t.topic, &t.msg_type, WORLD_WAIT)
                .await
                .unwrap_or(Value::Null),
            None => Value::Null,
        }
    }

    /// What the plan compiler knows: places, rooms and objects with their positions, where the
    /// robot is and what it holds.
    fn world_of(&self, seen: &Observed) -> World {
        let things = |msg: &Value, list: &str, name: &str, at: &str| -> Vec<Thing> {
            msg[list]
                .as_array()
                .map(|items| {
                    items
                        .iter()
                        .filter(|o| o["state"].as_u64() != Some(2))
                        .map(|o| {
                            let text = |k: &str| o[k].as_str().unwrap_or_default().to_owned();
                            let p = o.pointer(at);
                            let xy = p.and_then(|p| Some((p["x"].as_f64()?, p["y"].as_f64()?)));
                            Thing {
                                id: text("id"),
                                name: text(name),
                                xy,
                            }
                        })
                        .collect()
                })
                .unwrap_or_default()
        };
        let holding = ["left", "right"]
            .into_iter()
            .map(|arm| (arm.to_owned(), held_by(&seen.state, arm)))
            .collect();
        World {
            places: self.places.all(),
            rooms: things(&seen.rooms, "rooms", "name", "/centroid"),
            objects: things(&seen.objects, "objects", "label", "/pose/position"),
            robot: seen.pose,
            holding,
        }
    }

    fn run_spec(&self) -> ToolSpec {
        let catalog = lock(&self.catalog).clone();
        let (skills, listing) = match &catalog {
            Some(c) => (
                json!({"type": "string", "enum": c.plan_skills()}),
                format!("{}\n{}", c.signatures().join(", "), c.describe()),
            ),
            None => (
                json!({"type": "string"}),
                "(the skill list loads on first use; call run_mission with check_only to get it)"
                    .to_owned(),
            ),
        };
        let description = format!(
            "Makes the robot do something: give the plan's steps. It is checked first, and any \
             problems come back to fix; then the operator approves it in the app and the robot \
             starts, so never ask them yourself. Each step names a skill and gives every one of its \
             arguments as {{name, value}}, such as {{\"skill\": \"GoToPlace\", \"args\": \
             [{{\"name\": \"place\", \"value\": \"kitchen\"}}]}}. Use ids from list_places and \
             find_objects. A walk up to an object that a step must be near is added for you. \
             check_only: true only checks it, for when the operator asks to see a plan; hash runs a \
             plan checked before. Skills, by their exact names: {listing}"
        );
        let parameters = json!({
            "type": "object",
            "properties": {
                "hash": {"type": "string", "description": "Instead of steps: a plan checked before"},
                "template": {"type": "string", "description": "Instead of steps: the name of a saved plan"},
                "check_only": {"type": "boolean", "description": "Only check the plan; do not run it"},
                "intent": {"type": "string", "description": "What the operator asked for, in a few words"},
                "steps": {"type": "array", "minItems": 1, "maxItems": plan::MAX_STEPS, "items": {
                    "type": "object",
                    "properties": {
                        "skill": skills,
                        "args": {"type": "array", "items": {"type": "object",
                            "properties": {"name": {"type": "string"}, "value": {"type": "string"}},
                            "required": ["name", "value"], "additionalProperties": false}},
                        "retries": {"type": "integer", "minimum": 0, "maximum": 2},
                        "timeout_s": {"type": "number", "description": "Optional; at most the skill's own"},
                        "optional": {"type": "boolean", "description": "A failure here does not fail the mission"},
                        "why": {"type": "string"}
                    },
                    "required": ["skill", "args"],
                    "additionalProperties": false
                }}
            },
            "additionalProperties": false
        });
        ToolSpec {
            timeout: SERVICE_TIMEOUT * 2,
            ..ToolSpec::new("run_mission", &description, parameters, Risk::Manipulation)
        }
    }

    async fn plan(&self, args: Value, by: By) -> ToolOutcome {
        if by == By::Model
            && let Some(refusal) = self.may_plan()
        {
            return refusal;
        }
        // As sent, for the advisor: the plan below loses what did not parse.
        let sent = args.clone();
        let mut args = args;
        // A plan saved by name runs as its steps.
        if let Some(name) = args["template"].as_str().map(str::to_owned) {
            match self.template(&name).await {
                Ok(saved) => args = json!({"intent": saved.intent, "steps": saved.steps}),
                Err(e) => return ToolOutcome::refused(e),
            }
        }
        // The tool's own switches, not the plan's.
        if let Some(fields) = args.as_object_mut() {
            fields.remove("check_only");
            fields.remove("hash");
        }
        let mut plan: Plan = match serde_json::from_value(args) {
            Ok(p) => p,
            Err(e) => {
                return self
                    .rejected(
                        &json!([{"step": "", "field": "", "message": e.to_string()}]),
                        &sent,
                        by,
                    )
                    .await;
            }
        };
        // The local model leaves the label out, and once resent an unchanged plan until refused.
        if plan.intent.trim().is_empty() {
            plan.intent = self.intent_of(&plan);
        }
        let catalog = match self.catalog().await {
            Ok(c) => c,
            Err(e) => return ToolOutcome::failed(e),
        };
        let world = self.world_of(&self.observe().await);
        let mut compiled = match plan::compile(&plan, &catalog, &world, by) {
            Ok(c) => c,
            Err(problems) => return self.rejected(&json!(problems), &sent, by).await,
        };
        let reply = match self
            .robot
            .call(
                &self.config.validate,
                "nervros_interfaces/srv/ValidateMission",
                json!({"tree_xml": compiled.xml}),
                SERVICE_TIMEOUT,
            )
            .await
        {
            Ok(r) => r,
            Err(e) => {
                return ToolOutcome::failed(format!("the executor could not check the plan: {e}"));
            }
        };
        if reply["ok"].as_bool() != Some(true) {
            let diagnostics = reply["diagnostics_json"]
                .as_str()
                .and_then(|d| serde_json::from_str::<Value>(d).ok())
                .unwrap_or_else(|| json!([{"message": "the executor rejected the plan"}]));
            return self.rejected(&diagnostics, &sent, by).await;
        }
        if by == By::Model {
            self.plan_failures.store(0, Ordering::SeqCst);
        }
        // Not the model's fault, so not counted against its attempts.
        if let Some(why) = self.cannot_walk(&compiled, &catalog).await {
            return ToolOutcome::refused(why);
        }
        let (blocking, concerns) = self.concerns(&compiled, &catalog, by).await;
        if blocking && by == By::Model && !self.questioned.swap(true, Ordering::SeqCst) {
            return questioned(&concerns);
        }
        self.add_tracks(&mut compiled.steps).await;
        let worst = reply["worst_case_duration_s"]
            .as_f64()
            .filter(|w| *w > 0.0)
            .unwrap_or(compiled.worst_case_s);
        self.checked(compiled, worst, concerns)
    }

    /// Ways a plan may not do what the operator asked, and whether any is plain enough to send
    /// the plan back: the rules first, then the critic on what they let through.
    async fn concerns(
        &self,
        compiled: &Compiled,
        catalog: &Catalog,
        by: By,
    ) -> (bool, Vec<Concern>) {
        let request = self.request();
        let found = sanity::check(&request, &compiled.steps);
        if !found.is_empty() {
            return (true, found);
        }
        let none = (false, Vec::new());
        // The operator's own edit needs no second opinion.
        let Some(critic) = self
            .critic
            .get()
            .filter(|_| by == By::Model && !request.is_empty())
        else {
            return none;
        };
        let prompt = sanity::critic_prompt(&request, &compiled.steps, catalog);
        let reply = match tokio::time::timeout(CRITIC_TIMEOUT, critic.judge(&prompt)).await {
            Ok(Ok(reply)) => reply,
            Ok(Err(e)) => {
                tracing::info!(error = %e, "no critic for this plan");
                return none;
            }
            Err(_) => {
                tracing::info!("the critic took too long");
                return none;
            }
        };
        let whole = |message: String| Concern {
            step: String::new(),
            message: if message.is_empty() {
                "the plan checker has doubts".to_owned()
            } else {
                message
            },
            fix: None,
        };
        match sanity::verdict(&reply) {
            Some(Verdict::Reject(why)) => (true, vec![whole(why)]),
            Some(Verdict::Ask(why)) => (false, vec![whole(why)]),
            Some(Verdict::Ok) => none,
            None => {
                tracing::info!(%reply, "the critic gave no verdict");
                none
            }
        }
    }

    /// A plan that passed every check: shown to the window, kept by its hash, and described to
    /// the model with what went wrong before with its steps.
    fn checked(&self, compiled: Compiled, worst: f64, concerns: Vec<Concern>) -> ToolOutcome {
        let noted: Vec<String> = concerns.iter().map(concern_line).collect();
        self.emit(Event::MissionPlanned {
            hash: compiled.sha256.clone(),
            intent: compiled.plan.intent.clone(),
            steps: compiled.steps.clone(),
            worst_case_s: worst,
            concerns,
        });
        let mut out = json!({
            "hash": compiled.sha256,
            "steps": compiled.steps.iter().map(|s| format!("{} {}", s.id, s.summary)).collect::<Vec<_>>(),
            "worst_case_s": worst.round(),
            "next": "only checked: to run it, call run_mission with this hash; the operator approves it then"
        });
        let history: Vec<String> = compiled
            .steps
            .iter()
            .filter_map(|s| {
                let track = s.track.as_ref()?;
                let failed = track.runs - track.succeeded;
                (failed > 0).then(|| {
                    format!(
                        "{} {}: failed {failed} of its last {} runs, last because {}",
                        s.id,
                        s.summary,
                        track.runs,
                        track
                            .last_failure
                            .as_deref()
                            .unwrap_or("of something unknown")
                    )
                })
            })
            .collect();
        if !history.is_empty() {
            out["history"] = json!(history);
        }
        if !noted.is_empty() {
            out["concerns"] = json!(noted);
        }
        let mut planned = lock(&self.planned);
        // The same plan compiles to the same tree: keep one copy, or its hash reads as ambiguous.
        planned.retain(|c| c.sha256 != compiled.sha256);
        planned.push_back(compiled);
        if planned.len() > KEPT_PLANS {
            planned.pop_front();
        }
        ToolOutcome::ok(out)
    }

    async fn rejected(&self, problems: &Value, plan: &Value, by: By) -> ToolOutcome {
        let lines: Vec<String> = problems
            .as_array()
            .map_or(&[][..], Vec::as_slice)
            .iter()
            .filter_map(|p| {
                let message = p["message"].as_str()?;
                Some(match p["step"].as_str().filter(|s| !s.is_empty()) {
                    Some(step) => format!("{step}: {message}"),
                    None => message.to_owned(),
                })
            })
            .collect();
        if by == By::Operator {
            return ToolOutcome::failed(if lines.is_empty() {
                "the edited plan failed its checks".to_owned()
            } else {
                lines.join("; ")
            });
        }
        let n = self.plan_failures.fetch_add(1, Ordering::SeqCst) + 1;
        let count = problems.as_array().map_or(1, Vec::len);
        if n == MAX_PLAN_ATTEMPTS {
            // Planning gives up here: what was asked goes in the skill-gap log.
            let why: Vec<&str> = problems
                .as_array()
                .map_or(&[][..], Vec::as_slice)
                .iter()
                .filter_map(|p| p["message"].as_str())
                .take(3)
                .collect();
            self.note_gap(
                &format!("plans kept failing their checks: {}", why.join("; ")),
                "",
            );
        }
        // The problems first: what a reader of the first line most needs.
        let mut message = if lines.is_empty() {
            format!(
                "the plan has {count} problem(s); fix them all and call run_mission again \
                 (attempt {n} of {MAX_PLAN_ATTEMPTS})"
            )
        } else {
            format!(
                "{}. Fix all {count} problem(s) and call run_mission again (attempt {n} of \
                 {MAX_PLAN_ATTEMPTS})",
                lines.iter().take(2).cloned().collect::<Vec<_>>().join("; ")
            )
        };
        if n == advice::ADVISE_AFTER
            && let Some(advice) = self.advice(plan, problems).await
        {
            let _ = write!(message, ". A stronger model suggests: {advice}");
        }
        ToolOutcome {
            status: Status::Failed,
            data: json!({"ok": false, "problems": problems}),
            message,
            images: Vec::new(),
        }
    }

    /// The advisor's word on a failed plan, when there is an advisor and it answers.
    async fn advice(&self, plan: &Value, problems: &Value) -> Option<String> {
        let advisor = self.advisor.get()?;
        let skills = self.catalog().await.ok()?.describe();
        let request = lock(&self.request).clone();
        let said = advisor
            .advise(&advice::prompt(&request, &skills, plan, problems))
            .await?;
        Some(crate::tools::clip(said.trim(), advice::ADVICE_CHARS))
    }

    pub(crate) fn find(&self, hash: &str) -> Result<Compiled, String> {
        let hash = hash.trim().to_ascii_lowercase();
        let planned = lock(&self.planned);
        let hits: Vec<&Compiled> = planned
            .iter()
            .filter(|c| hash.len() >= MIN_HASH_PREFIX && c.sha256.starts_with(&hash))
            .collect();
        match hits.as_slice() {
            [one] => Ok((*one).clone()),
            [] => {
                Err("no checked plan has this hash; give run_mission the steps instead".to_owned())
            }
            _ => Err("the hash is ambiguous; use the full hash".to_owned()),
        }
    }

    async fn run(self: &Arc<Self>, args: &Value) -> ToolOutcome {
        let compiled = match self.find(args["hash"].as_str().unwrap_or_default()) {
            Ok(c) => c,
            Err(e) => return ToolOutcome::refused(e),
        };
        if let Some(r) = lock(&self.running).as_ref() {
            return ToolOutcome::refused(format!(
                "mission {} is still running; wait for its report or stop it",
                r.id
            ));
        }
        if let Some(refusal) = self.unchanged(&compiled.sha256) {
            return refusal;
        }
        match self.launch(compiled).await {
            Ok(id) => ToolOutcome {
                status: Status::Accepted,
                data: json!({"mission_id": id}),
                message: "the mission is running; a report follows when it ends. Tell the operator it started."
                    .to_owned(),
                images: Vec::new(),
            },
            Err(e) => ToolOutcome::failed(e),
        }
    }

    /// Why the model may not plan for this request any more, or at all yet.
    fn may_plan(&self) -> Option<ToolOutcome> {
        if self.plan_failures.load(Ordering::SeqCst) >= MAX_PLAN_ATTEMPTS {
            return Some(ToolOutcome::refused(format!(
                "{MAX_PLAN_ATTEMPTS} plans in a row failed their checks; tell the operator what is missing instead"
            )));
        }
        let failed = self.run_failures.load(Ordering::SeqCst);
        if failed > self.config.max_replans {
            return Some(ToolOutcome::refused(format!(
                "this request already failed {failed} times; tell the operator what went wrong instead of retrying"
            )));
        }
        // A guess at what "it" is moves the wrong thing; asking costs a sentence.
        let unnamed = sanity::unnamed(&lock(&self.request));
        if let Some(word) = unnamed.filter(|_| self.said.load(Ordering::SeqCst) <= 1) {
            return Some(ToolOutcome::refused(format!(
                "the operator said \"{word}\" and nothing before it says what that is; ask \
                 them which thing they mean"
            )));
        }
        None
    }

    /// A name for a plan the model left unnamed: its first reason, else the operator's words.
    fn intent_of(&self, plan: &Plan) -> String {
        let asked = lock(&self.request)
            .trim()
            .trim_end_matches(['.', '!', '?'])
            .to_owned();
        plan.steps
            .iter()
            .map(|s| s.why.trim())
            .find(|w| !w.is_empty())
            .map(str::to_owned)
            .or_else(|| (!asked.is_empty()).then(|| crate::tools::clip(&asked, 60)))
            .unwrap_or_else(|| "the plan".to_owned())
    }

    /// The refusal for running the plan that just ran and failed, unchanged.
    fn unchanged(&self, hash: &str) -> Option<ToolOutcome> {
        let last = lock(&self.last);
        let (_, how) = last.as_ref().filter(|(ran, _)| ran == hash)?;
        Some(ToolOutcome::refused(format!(
            "this exact plan just ran and {how}; running it unchanged would repeat that. Change \
             the plan to deal with what the report said, or tell the operator"
        )))
    }

    /// Sends a checked plan to the executor and watches it; its id.
    async fn launch(self: &Arc<Self>, compiled: Compiled) -> Result<String, String> {
        let id = uuid::Uuid::now_v7().to_string();
        let goal = json!({
            "mission_id": id,
            "tree_xml": compiled.xml,
            "tree_sha256": compiled.sha256,
            "max_duration_s": 0.0,
            "heartbeat_timeout_s": if self.heartbeat_live.load(Ordering::SeqCst) {
                self.config.heartbeat_timeout_s
            } else {
                0.0
            },
            "heartbeat_client": self.client,
            "mode": 0
        });
        let goal = self
            .robot
            .send_goal(&self.config.execute, EXECUTE, goal, SERVICE_TIMEOUT)
            .await
            .map_err(|e| format!("the executor did not start the mission: {e}"))?;
        *lock(&self.running) = Some(Running { id: id.clone() });
        self.emit(Event::MissionStarted {
            id: id.clone(),
            hash: compiled.sha256.clone(),
        });
        let before = self.vision.get().and_then(camera::Vision::frame);
        let span = crate::telemetry::job("mission", &id, &compiled.plan.intent);
        tokio::spawn(
            Arc::clone(self)
                .watch(id.clone(), compiled, goal, before)
                .instrument(span),
        );
        Ok(id)
    }

    /// Runs a plan a schedule's approval covers: it repeats on purpose, so the guard against an
    /// unchanged rerun does not apply; `label` names the run in its report.
    ///
    /// # Errors
    ///
    /// A mission is running, or the executor did not start it.
    pub async fn run_scheduled(
        self: &Arc<Self>,
        mut compiled: Compiled,
        label: &str,
    ) -> Result<String, String> {
        if let Some(r) = lock(&self.running).as_ref() {
            return Err(format!("mission {} was still running", r.id));
        }
        compiled.plan.intent = label.to_owned();
        self.launch(compiled).await
    }

    /// Whether a mission is running now.
    #[must_use]
    pub fn busy(&self) -> bool {
        lock(&self.running).is_some()
    }

    /// Checks a plan for a schedule: its hash, and its steps as the operator reads them.
    ///
    /// # Errors
    ///
    /// The plan's problems, as the tool's outcome.
    pub async fn check(
        &self,
        intent: &str,
        steps: &Value,
    ) -> Result<(String, usize, f64), ToolOutcome> {
        let out = self
            .plan(json!({"intent": intent, "steps": steps}), By::Model)
            .await;
        if out.status != Status::Succeeded {
            return Err(out);
        }
        Ok((
            out.data["hash"].as_str().unwrap_or_default().to_owned(),
            out.data["steps"].as_array().map_or(0, Vec::len),
            out.data["worst_case_s"].as_f64().unwrap_or(0.0),
        ))
    }

    async fn watch(
        self: Arc<Self>,
        id: String,
        compiled: Compiled,
        goal: Goal,
        before: Option<Arc<Frame>>,
    ) {
        let started = Instant::now();
        let started_s = ledger::now_s();
        let mut times = StepTimes::default();
        let Goal {
            mut feedback,
            result,
            ..
        } = goal;
        tokio::pin!(result);
        let result = loop {
            tokio::select! {
                Some(fb) = feedback.recv() => self.progress(&id, &fb, &mut times),
                r = &mut result => break r,
            }
        };
        let elapsed = started.elapsed().as_secs_f64();
        let (outcome, step, reason) = summarise(
            result
                .map_err(|_| RosError::Middleware("the goal was dropped".into()))
                .and_then(|r| r),
        );
        *lock(&self.running) = None;
        tracing::Span::current().record("nervros.outcome", outcome.as_str());
        let how = if outcome == "success" {
            "succeeded".to_owned()
        } else {
            format!("failed at {step}: {reason}")
        };
        *lock(&self.last) = Some((compiled.sha256.clone(), how));
        self.emit(Event::MissionFinished {
            id: id.clone(),
            outcome: outcome.clone(),
            failed_step: step.clone(),
            reason: reason.clone(),
            elapsed_s: elapsed,
        });
        self.keep(MissionRecord {
            id: id.clone(),
            hash: compiled.sha256.clone(),
            intent: compiled.plan.intent.clone(),
            request: lock(&self.request).clone(),
            started: started_s,
            ended: ledger::now_s(),
            outcome: outcome.clone(),
            failed_step: step.clone(),
            reason: reason.clone(),
            steps: compiled
                .steps
                .iter()
                .map(|s| times.record(s, &step, &reason))
                .collect(),
        });
        let seen = self.observe().await;
        // What happened to what the failed step was about: often it was moved, not missed.
        let object = compiled
            .steps
            .iter()
            .find(|s| s.id == step && outcome != "success")
            .and_then(|s| check::step_object(s, &seen))
            .and_then(|o| o["id"].as_str());
        let story = match object {
            Some(id) => self.object_story(id).await,
            None => Vec::new(),
        };
        let camera = match self.vision.get().filter(|_| outcome == "success") {
            Some(vision) => {
                let verdicts = check::check(&compiled.goal, &seen);
                vision.check(&verdicts, &seen, before.as_ref()).await
            }
            None => camera::Checked::default(),
        };
        if let Some(image) = &camera.image {
            self.emit(Event::Snapshot {
                id: image.snapshot.clone(),
                jpeg: Arc::clone(&image.jpeg),
                width: image.width,
                height: image.height,
                marks: Vec::new(),
            });
        }
        let report = self.report(
            &id,
            &compiled,
            (&outcome, &step, &reason),
            elapsed,
            (&seen, &story, &camera.lines),
        );
        if let Some(s) = self.session.get() {
            s.send(Command::Report(report));
        }
    }

    /// What the model reads when a mission ends: how it went, the goal checks or why it failed
    /// and where its object was last seen, and what the hands hold now.
    fn report(
        &self,
        id: &str,
        compiled: &Compiled,
        (outcome, step, reason): (&str, &str, &str),
        elapsed: f64,
        (seen, story, camera): (&Observed, &[String], &[String]),
    ) -> String {
        let mut report = format!(
            "Mission {id} ({}) ended: {outcome} after {elapsed:.0} s.",
            compiled.plan.intent
        );
        let mut next = "";
        // Failures count per request, not in a row: a detour that works between two blocked
        // walks does not start the count again, and the operator's next message does.
        if outcome == "success" {
            let verdicts = check::check(&compiled.goal, seen);
            if !verdicts.is_empty() {
                let lines: Vec<String> = verdicts
                    .iter()
                    .map(|v| {
                        let mark = match v.ok {
                            Some(true) => "holds",
                            Some(false) => "DOES NOT hold",
                            None => "unchecked",
                        };
                        format!("{} {mark} ({})", v.predicate, v.detail)
                    })
                    .collect();
                let _ = write!(report, " Goal checks: {}.", lines.join("; "));
            }
            if !camera.is_empty() {
                let _ = write!(
                    report,
                    " Camera check, before and after: {}.",
                    camera.join("; ")
                );
            }
        } else {
            let n = self.run_failures.fetch_add(1, Ordering::SeqCst) + 1;
            let failed = compiled.steps.iter().find(|s| s.id == step);
            let what = failed.map_or_else(String::new, |s| format!(" {}", s.summary));
            let _ = write!(report, " Failed at {step}{what}: {reason}.");
            if let Some(line) = failed.and_then(|s| check::last_seen(s, seen, ledger::now_s())) {
                let _ = write!(report, " The world model: {line}.");
            }
            if !story.is_empty() {
                let _ = write!(report, " What happened to it: {}.", story.join(". "));
            }
            next = if n > self.config.max_replans {
                "This request has failed too often: do not retry. Tell the operator what went \
                 wrong and what would help."
            } else {
                "Find out why (robot_state, look, log_tail) and say it in one sentence. If a \
                 changed plan can work, run it now: the operator approves it. If not, say what is \
                 needed."
            };
        }
        if !seen.state.is_null() {
            let hands: Vec<String> = ["left", "right"]
                .iter()
                .filter_map(|h| {
                    let held = held_by(&seen.state, h);
                    (!held.is_empty()).then(|| format!("the {h} hand holds {held}"))
                })
                .collect();
            if !hands.is_empty() {
                let _ = write!(report, " Now {}.", hands.join(" and "));
            }
        }
        // What the model should do next goes on its own line, after what happened.
        if !next.is_empty() {
            report.push('\n');
            report.push_str(next);
        }
        report
    }

    /// What happened to the objects `query` names, a line each, from the world model's history;
    /// none when the profile names no history service or it does not answer.
    pub(crate) async fn object_story(&self, query: &str) -> Vec<String> {
        let Some(service) = self.profile.world.as_ref().and_then(|w| w.history.as_ref()) else {
            return Vec::new();
        };
        let request = json!({"query": query, "since": {"sec": 0, "nanosec": 0}, "max_events": 30});
        match self
            .robot
            .call(
                service,
                "canopy_msgs/srv/ObjectHistory",
                request,
                SERVICE_TIMEOUT,
            )
            .await
        {
            Ok(reply) => check::story(
                reply["events"].as_array().map_or(&[][..], Vec::as_slice),
                ledger::now_s(),
            ),
            Err(e) => {
                tracing::info!(error = %e, "no object history");
                Vec::new()
            }
        }
    }

    /// What the operator said last.
    pub(crate) fn request(&self) -> String {
        lock(&self.request).clone()
    }

    /// The plan saved as `name`, counted as used.
    async fn template(&self, name: &str) -> Result<ledger::Template, String> {
        let Some(ledger) = self.ledger.get().cloned() else {
            return Err("no plans are saved on this robot".to_owned());
        };
        let wanted = name.to_owned();
        tokio::task::spawn_blocking(move || ledger.use_template(&wanted))
            .await
            .map_err(|e| e.to_string())?
            .map_err(|e| e.to_string())?
            .ok_or_else(|| format!("no plan is saved as \"{name}\"; the plans tool lists them"))
    }

    /// Logs what the operator asked as something no skill could do.
    pub(crate) fn note_gap(&self, reason: &str, nearest: &str) {
        let Some(ledger) = self.ledger.get().cloned() else {
            return;
        };
        let (request, reason, nearest) = (self.request(), reason.to_owned(), nearest.to_owned());
        tokio::task::spawn_blocking(move || {
            if let Err(e) = ledger.note_gap(&request, &reason, &nearest) {
                tracing::warn!(error = %e, "the skill gap was not logged");
            }
        });
    }

    /// Each step's track record from the ledger, read off the async threads.
    async fn add_tracks(&self, steps: &mut [PlannedStep]) {
        let Some(ledger) = self.ledger.get().cloned() else {
            return;
        };
        let keys: Vec<(String, String)> = steps
            .iter()
            .map(|s| (s.skill.clone(), ledger::target_of(&s.args)))
            .collect();
        let tracks = tokio::task::spawn_blocking(move || {
            keys.iter()
                .map(|(skill, target)| ledger.track(skill, target).ok().flatten())
                .collect::<Vec<_>>()
        })
        .await
        .unwrap_or_default();
        for (step, track) in steps.iter_mut().zip(tracks) {
            step.track = track;
        }
    }

    /// Writes a finished mission to the ledger, off the async threads.
    fn keep(&self, record: MissionRecord) {
        if let Some(ledger) = self.ledger.get().cloned() {
            tokio::task::spawn_blocking(move || {
                if let Err(e) = ledger.record(&record) {
                    tracing::warn!(error = %e, mission = %record.id, "the mission was not recorded");
                }
            });
        }
    }

    /// Maps one feedback message's node events onto steps.
    fn progress(&self, id: &str, feedback: &Value, times: &mut StepTimes) {
        let elapsed = feedback["elapsed_s"].as_f64().unwrap_or(0.0);
        for e in feedback["events"].as_array().map_or(&[][..], Vec::as_slice) {
            let name = e["name"].as_str().unwrap_or_default();
            let path = e["path"].as_str().unwrap_or_default();
            let Some(step) = step_of(name).or_else(|| step_of(path)) else {
                continue;
            };
            let status = e["status"]
                .as_u64()
                .and_then(|s| STATUSES.get(usize::try_from(s).ok()?))
                .copied()
                .unwrap_or("unknown");
            // Finished nodes are reset to idle afterwards; that would hide how they ended.
            if status == "idle" {
                continue;
            }
            let node = if step_of(name).is_some() {
                times.note(&step, status, elapsed);
                String::new()
            } else {
                name.to_owned()
            };
            self.emit(Event::MissionProgress {
                id: id.to_owned(),
                step,
                node,
                path: path.to_owned(),
                status: status.to_owned(),
                elapsed_s: elapsed,
            });
        }
    }

    pub(crate) async fn observe(&self) -> Observed {
        let world = self.profile.world.as_ref();
        let rooms = self.latest(world.and_then(|w| w.rooms.as_ref())).await;
        let objects = self.latest(world.and_then(|w| w.objects.as_ref())).await;
        let (map, base) = (&self.profile.ros.map_frame, &self.profile.ros.base_frame);
        let pose = self
            .robot
            .transform(map, base)
            .ok()
            .map(|t| (t.translation[0], t.translation[1]));
        let state = self
            .robot
            .latest(&self.config.state, STATE, WORLD_WAIT)
            .await
            .unwrap_or(Value::Null);
        Observed {
            pose,
            places: self.places.all(),
            rooms,
            objects,
            state,
        }
    }
}

/// `(outcome, failed step, reason)` from the executor's result.
fn summarise(result: Result<GoalResult, RosError>) -> (String, String, String) {
    match result {
        Ok(r) => {
            let code = r.result["outcome"]
                .as_u64()
                .and_then(|c| usize::try_from(c).ok());
            let outcome = code.and_then(|c| OUTCOMES.get(c)).map_or_else(
                || format!("{:?}", r.status).to_lowercase(),
                |o| (*o).to_owned(),
            );
            let text = |k: &str| r.result[k].as_str().unwrap_or_default().to_owned();
            // A rejected mission says why in its diagnostics, not in the failure reason.
            let mut reason = text("failure_reason");
            if reason.is_empty() {
                reason = text("diagnostics_json");
            }
            (outcome, text("failed_step_id"), reason)
        }
        Err(e) => ("error".to_owned(), String::new(), e.to_string()),
    }
}

/// The `s<N>` a node name or path belongs to: the step subtrees are named `s<N>_<Skill>`.
/// When a step began, in seconds into the mission, and when and how it ended.
#[derive(Debug, Clone, Copy)]
struct StepTime {
    start: f64,
    end: Option<(f64, &'static str)>,
}

/// Each step's [`StepTime`], from the executor's events, for the ledger.
#[derive(Debug, Default)]
struct StepTimes(std::collections::BTreeMap<String, StepTime>);

impl StepTimes {
    fn note(&mut self, step: &str, status: &'static str, elapsed: f64) {
        let time = self.0.entry(step.to_owned()).or_insert(StepTime {
            start: elapsed,
            end: None,
        });
        if matches!(status, "success" | "failure") {
            time.end = Some((elapsed, status));
        }
    }

    /// A planned step as it went; `failed` is the step the mission failed at, with `reason`.
    fn record(&self, step: &PlannedStep, failed: &str, reason: &str) -> StepRecord {
        let (seconds, outcome) = match self.0.get(&step.id) {
            Some(&StepTime {
                start,
                end: Some((end, status)),
            }) => (Some(end - start), status),
            _ if step.id == failed => (None, "failure"),
            _ => (None, "skipped"),
        };
        StepRecord {
            id: step.id.clone(),
            skill: step.skill.clone(),
            target: ledger::target_of(&step.args),
            args: Value::Object(
                step.args
                    .iter()
                    .map(|a| (a.name.clone(), Value::String(a.value.clone())))
                    .collect(),
            ),
            seconds,
            outcome: outcome.to_owned(),
            reason: if outcome == "failure" {
                reason.to_owned()
            } else {
                String::new()
            },
        }
    }
}

fn step_of(text: &str) -> Option<String> {
    text.split(['/', ':'])
        .filter_map(|seg| {
            let rest = seg.strip_prefix('s')?;
            let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
            (!digits.is_empty() && rest[digits.len()..].starts_with('_'))
                .then(|| format!("s{digits}"))
        })
        .next_back()
}

/// `run_mission`.
struct RunMission(Arc<Missions>);

#[async_trait]
impl Tool for RunMission {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Owned(self.0.run_spec())
    }

    /// A plan is checked before anyone is asked: its problems go back to the model, and a sound
    /// one is shown, approved and run by its hash.
    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        if args.get("steps").is_none() && args.get("template").is_none() {
            // A plan by its hash that just failed is refused before anyone is asked to approve it.
            let hash = self.0.find(args["hash"].as_str()?).ok()?.sha256;
            return self.0.unchanged(&hash).map(Err);
        }
        if args["check_only"].as_bool() == Some(true) {
            return Some(Ok(Assessment {
                risk: Risk::Observe,
                resources: Vec::new(),
                reason: "checks a plan".to_owned(),
                args: None,
            }));
        }
        // Run now, it would be once; a schedule runs it now and then on its clock.
        if let Some(r) = sanity::repeat(&lock(&self.0.request)) {
            return Some(Err(ToolOutcome::refused(format!(
                "the operator asked for it every {} min: check one run of it with check_only, \
                 then give the steps to schedule, which runs it now and then on time",
                r.every_min
            ))));
        }
        Some(self.approvable(args.clone(), By::Model).await)
    }

    /// The operator's own plan, edited or made in the window, checked as the model's would be.
    async fn assess_operator(&self, args: Value) -> Option<Result<Assessment, ToolOutcome>> {
        Some(self.approvable(args, By::Operator).await)
    }

    /// A checked plan's hash dies with the session that checked it; its steps do not.
    fn ask_again(&self, args: &Value) -> Option<Value> {
        let plan = self.0.find(args["hash"].as_str()?).ok()?.plan;
        serde_json::to_value(plan).ok()
    }

    /// The skill list loads when the session starts; a first message sent at once would see none.
    async fn ready(&self) {
        let _ = tokio::time::timeout(SERVICE_TIMEOUT / 2, self.0.catalog()).await;
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        // Steps reach here only to be checked: a run's were replaced by their hash.
        if args.get("steps").is_some() || args.get("template").is_some() {
            return self.0.plan(args, By::Model).await;
        }
        self.0.run(&args).await
    }
}

impl RunMission {
    /// A plan checked and ready for the operator: run by its hash once approved, with its
    /// preview on the way.
    async fn approvable(&self, args: Value, by: By) -> Result<Assessment, ToolOutcome> {
        let out = self.0.plan(args, by).await;
        if out.status != Status::Succeeded {
            return Err(out);
        }
        let hash = out.data["hash"].as_str().unwrap_or_default().to_owned();
        if by == By::Model
            && let Some(refusal) = self.0.unchanged(&hash)
        {
            return Err(refusal);
        }
        let steps = out.data["steps"].as_array().map_or(0, Vec::len);
        let minutes = (out.data["worst_case_s"].as_f64().unwrap_or(0.0) / 60.0).ceil();
        let intent = self
            .0
            .find(&hash)
            .map_or_else(|_| "the plan".to_owned(), |c| c.plan.intent);
        let mut reason = format!("runs \"{intent}\": {steps} step(s), at most {minutes} min");
        if let Some(concerns) = out.data["concerns"].as_array() {
            for c in concerns.iter().filter_map(Value::as_str) {
                let _ = write!(reason, "; check: {c}");
            }
        }
        let missions = Arc::clone(&self.0);
        let previewed = hash.clone();
        tokio::spawn(async move { missions.preview(&previewed).await });
        Ok(Assessment {
            risk: Risk::Manipulation,
            resources: Vec::new(),
            reason,
            args: Some(json!({"hash": hash})),
        })
    }
}

/// A concern as one line, its step first.
fn concern_line(c: &Concern) -> String {
    if c.step.is_empty() {
        c.message.clone()
    } else {
        format!("{}: {}", c.step, c.message)
    }
}

/// The model's plan back to it once, for words it may not match; not a failed attempt.
fn questioned(concerns: &[Concern]) -> ToolOutcome {
    let lines: Vec<String> = concerns.iter().map(concern_line).collect();
    ToolOutcome {
        status: Status::Failed,
        message: format!(
            "{}. The plan may not do what the operator asked: fix it, or if it is right, send it \
             again unchanged and the operator sees these concerns when approving",
            lines.join("; ")
        ),
        data: json!({"ok": false, "concerns": lines}),
        images: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::catalog::tests::CATALOG;
    use crate::places::Places;
    use crate::session::Event;
    use nervros_ros::GoalStatus;
    use nervros_ros::fake::{FakeRobot, ScriptedRun};
    use std::path::Path;

    fn profile() -> Profile {
        profile_with("")
    }

    /// The test profile with `mission` lines added to its `[mission]` section.
    fn profile_with(mission: &str) -> Profile {
        let text = r#"
            [robot]
            name = "t"
            [world]
            objects = { topic = "/objects", type = "canopy_msgs/msg/WorldObjectArray" }
            history = "/x/history"
            [mission]
            execute = "/x/execute"
            validate = "/x/validate"
            catalog = "/x/catalog"
            stop = "/x/stop"
            state = "/x/state"
            max_replans = 1
            MISSION_EXTRA
            [[place]]
            name = "dock"
            pose = { x = 1.0, y = 2.0 }
            [models]
            file = "m.toml"
        "#;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("nervros.toml");
        std::fs::write(&path, text.replace("MISSION_EXTRA", mission)).unwrap();
        Profile::load(Path::new(&path)).unwrap()
    }

    fn robot(run: ScriptedRun) -> FakeRobot {
        let run = Arc::new(Mutex::new(Some(run)));
        FakeRobot::new()
            .with_service("/x/catalog", |_| Ok(json!({"catalog_json": CATALOG})))
            .with_service("/x/validate", |req| {
                let xml = req["tree_xml"].as_str().unwrap_or_default();
                Ok(json!({"ok": !xml.contains("Parallel"), "diagnostics_json": "[]", "worst_case_duration_s": 0.0}))
            })
            .with_action("/x/execute", move |_| lock(&run).take().unwrap_or_default())
            .with_topic(
                "/objects",
                json!({"objects": [
                    {"id": "O17", "label": "red mug", "state": 0,
                     "pose": {"position": {"x": 1.0, "y": 2.0}}, "size": {"x": 0.1, "y": 0.1}},
                    {"id": "O18", "label": "blue cup", "state": 0,
                     "pose": {"position": {"x": 1.1, "y": 2.0}}, "size": {"x": 0.1, "y": 0.1}}]}),
            )
            .with_topic("/x/state", json!({"holding_left": "", "holding_right": "O17"}))
            .with_transform(
                "map",
                "base_footprint",
                nervros_ros::Transform {
                    translation: [1.1, 2.0, 0.0],
                    ..nervros_ros::Transform::IDENTITY
                },
            )
    }

    fn steps() -> Value {
        json!({"intent": "fetch the mug", "goal": ["at(dock)", "holding(right, O17)"], "steps": [
            {"skill": "GoToPlace", "args": [{"name": "place", "value": "dock"}]},
            {"skill": "PickObject", "args": {"object_id": "O18", "phrase": "blue cup", "arm": "left"}}
        ]})
    }

    /// Waits up to five seconds for `ready`.
    async fn eventually(mut ready: impl FnMut() -> bool) {
        let wait = async {
            while !ready() {
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        };
        tokio::time::timeout(Duration::from_secs(5), wait)
            .await
            .expect("it never happened");
    }

    #[tokio::test]
    async fn a_mission_is_kept_recalled_saved_by_name_and_run_again_and_gaps_are_logged() {
        let run = || ScriptedRun {
            result: Ok(GoalResult {
                status: GoalStatus::Succeeded,
                result: json!({"outcome": 0, "failed_step_id": "", "failure_reason": ""}),
            }),
            ..ScriptedRun::default()
        };
        let robot: Arc<dyn RobotPort> = Arc::new(robot(run()));
        let ledger = Ledger::in_memory().unwrap();
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot)
            .unwrap()
            .with_ledger(Arc::clone(&ledger));
        let (tx, _commands) = tokio::sync::mpsc::unbounded_channel();
        let (events, _rx) = tokio::sync::broadcast::channel(64);
        missions.attach(SessionHandle::for_tests(&tx, events.clone()));
        let _ = events.send(Event::User {
            turn: 1,
            text: "fetch me the mug from the dock".to_owned(),
        });
        eventually(|| missions.request().contains("mug")).await;

        let hash = missions.plan(steps(), By::Model).await.data["hash"]
            .as_str()
            .unwrap()
            .to_owned();
        assert_eq!(
            missions.run(&json!({"hash": hash})).await.status,
            Status::Accepted
        );
        eventually(|| ledger.recent(1).is_ok_and(|r| !r.is_empty())).await;
        let kept = &ledger.recent(1).unwrap()[0];
        assert_eq!(
            (kept.outcome.as_str(), kept.request.as_str()),
            ("success", "fetch me the mug from the dock")
        );

        let [recall, plans, gap] = history::tools(&missions);
        let lately = recall.call(json!({"about": "missions"})).await;
        assert!(
            lately.data["missions"][0]
                .as_str()
                .unwrap()
                .contains("fetch the mug (success)")
        );
        let proven = recall
            .call(json!({"about": "plans", "query": "bring the mug"}))
            .await;
        assert!(
            proven.data["plans"][0]
                .as_str()
                .unwrap()
                .contains("GoToPlace(place=dock)"),
            "{}",
            proven.data
        );

        let saved = plans
            .call(json!({"action": "save", "name": "mug run", "hash": hash}))
            .await;
        assert_eq!(saved.status, Status::Succeeded, "{}", saved.message);
        let again = missions
            .plan(json!({"template": "mug run"}), By::Model)
            .await;
        assert_eq!(again.status, Status::Succeeded, "{}", again.message);
        assert_eq!(
            again.data["hash"],
            json!(hash),
            "the same plan, checked again"
        );

        gap.call(json!({"missing": "no skill puts a mug on a sofa", "nearest": "PlaceInto"}))
            .await;
        eventually(|| ledger.gaps(1).is_ok_and(|g| !g.is_empty())).await;
        assert_eq!(ledger.gaps(1).unwrap()[0].nearest, "PlaceInto");
    }

    #[tokio::test]
    async fn a_plan_runs_and_reports_with_goal_checks() {
        let run = ScriptedRun {
            feedback: vec![json!({"elapsed_s": 1.0, "events": [
                {"name": "s1_GoToPlace", "path": "Mission/s1_GoToPlace", "status": 1},
                {"name": "NavigateToPose", "path": "s1_GoToPlace/NavigateToPose", "status": 1},
                {"name": "s1_GoToPlace", "path": "s1_GoToPlace", "status": 2},
                {"name": "s1_GoToPlace", "path": "s1_GoToPlace", "status": 0}]})],
            result: Ok(GoalResult {
                status: GoalStatus::Succeeded,
                result: json!({"outcome": 0, "failed_step_id": "", "failure_reason": ""}),
            }),
            ..ScriptedRun::default()
        };
        let robot: Arc<dyn RobotPort> = Arc::new(robot(run));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let (tx, mut commands) = tokio::sync::mpsc::unbounded_channel();
        let (events, mut rx) = tokio::sync::broadcast::channel(64);
        missions.attach(SessionHandle::for_tests(&tx, events));

        let planned = missions.plan(steps(), By::Model).await;
        assert_eq!(planned.status, Status::Succeeded, "{}", planned.message);
        let hash = planned.data["hash"].as_str().unwrap().to_owned();
        let started = missions.run(&json!({"hash": &hash[..10]})).await;
        assert_eq!(started.status, Status::Accepted, "{}", started.message);

        let report = tokio::time::timeout(Duration::from_secs(5), commands.recv())
            .await
            .unwrap()
            .unwrap();
        let Command::Report(text) = report else {
            panic!("not a report")
        };
        assert!(text.contains("ended: success"), "{text}");
        // The checks come from the steps, not the model's goal list: the pick moves the base, so
        // no at(dock), and the fake's hands never change.
        assert!(
            text.contains("holding(right, O17) holds")
                && text.contains("holding(left, O18) DOES NOT hold")
                && !text.contains("at(dock)"),
            "{text}"
        );
        // Unchanged, the same plan is not run again on the model's own.
        let again = missions.run(&json!({"hash": hash})).await;
        assert_eq!(again.status, Status::Refused);
        assert!(
            again.message.contains("just ran and succeeded"),
            "{}",
            again.message
        );
        let mut seen = Vec::new();
        while let Ok(e) = rx.try_recv() {
            seen.push(e);
        }
        assert!(seen.iter().any(|e| matches!(e, Event::MissionProgress { step, node, .. } if step == "s1" && node == "NavigateToPose")));
        let last = seen.iter().rev().find_map(|e| match e {
            Event::MissionProgress { node, status, .. } if node.is_empty() => Some(status.as_str()),
            _ => None,
        });
        assert_eq!(last, Some("success"), "the reset to idle is not reported");
        assert!(
            seen.iter().any(
                |e| matches!(e, Event::MissionFinished { outcome, .. } if outcome == "success")
            )
        );
    }

    #[tokio::test]
    async fn a_walk_is_refused_while_the_robot_cannot_move_and_not_counted_as_the_models() {
        let robot = robot(ScriptedRun::default()).with_topic(
            "/x/state",
            json!({"holding_left": "", "holding_right": "", "can_move": false,
                   "cannot_move_reason": "fallen: tilted 104 degrees"}),
        );
        let robot: Arc<dyn RobotPort> = Arc::new(robot);
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();

        let out = missions.plan(steps(), By::Model).await;

        assert_eq!(out.status, Status::Refused, "{}", out.message);
        assert!(
            out.message.contains("fallen: tilted 104 degrees"),
            "{}",
            out.message
        );
        assert_eq!(missions.plan_failures.load(Ordering::SeqCst), 0);
    }

    #[tokio::test]
    async fn a_live_heartbeat_feeds_the_deadman_and_each_mission_asks_for_it() {
        let profile = profile_with("heartbeat = \"/x/heartbeat\"\nheartbeat_timeout_s = 1.0");
        let sent = Arc::new(Mutex::new(None));
        let goals = Arc::clone(&sent);
        let fake = Arc::new(
            robot(ScriptedRun::default()).with_action("/x/execute", move |goal| {
                *lock(&goals) = Some(goal.clone());
                ScriptedRun::default()
            }),
        );
        let robot: Arc<dyn RobotPort> = fake.clone();
        let missions = Missions::new(&profile, Places::new(&profile, None), robot).unwrap();
        let (tx, _commands) = tokio::sync::mpsc::unbounded_channel();
        let (events, _rx) = tokio::sync::broadcast::channel(64);
        missions.attach(SessionHandle::for_tests(&tx, events));

        // The session's loop pulses; here the test does.
        let beating = async {
            while !fake.published().iter().any(|(t, _)| t == "/x/heartbeat") {
                crate::session::Pulse::pulse(missions.as_ref());
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        };
        tokio::time::timeout(Duration::from_secs(5), beating)
            .await
            .expect("no heartbeat was published");
        let hash = missions.plan(steps(), By::Model).await.data["hash"]
            .as_str()
            .unwrap()
            .to_owned();
        let out = missions.run(&json!({"hash": hash})).await;
        assert_eq!(out.status, Status::Accepted, "{}", out.message);
        let goal = lock(&sent).clone().expect("no goal was sent");
        assert_eq!(goal["heartbeat_timeout_s"], json!(1.0));
        assert_eq!(goal["heartbeat_client"], json!(missions.client));
        let (_, beat) = fake
            .published()
            .into_iter()
            .find(|(t, _)| t == "/x/heartbeat")
            .unwrap();
        assert_eq!(beat["client"], json!(missions.client));
    }

    #[tokio::test]
    async fn without_a_heartbeat_a_mission_asks_for_no_deadman() {
        let sent = Arc::new(Mutex::new(None));
        let goals = Arc::clone(&sent);
        let robot = robot(ScriptedRun::default()).with_action("/x/execute", move |goal| {
            *lock(&goals) = Some(goal.clone());
            ScriptedRun::default()
        });
        let robot: Arc<dyn RobotPort> = Arc::new(robot);
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let hash = missions.plan(steps(), By::Model).await.data["hash"]
            .as_str()
            .unwrap()
            .to_owned();
        missions.run(&json!({"hash": hash})).await;
        let goal = lock(&sent).clone().expect("no goal was sent");
        assert_eq!(goal["heartbeat_timeout_s"], json!(0.0));
    }

    #[tokio::test]
    async fn a_plan_left_unanswered_is_asked_again_as_its_steps() {
        let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let [tool] = missions.tools();
        let checked = tool.assess(&steps()).await.unwrap().unwrap();
        let again = tool.ask_again(&checked.args.unwrap()).unwrap();
        assert!(
            again["steps"].is_array() && again.get("hash").is_none(),
            "{again}"
        );
        let out = missions.plan(again, By::Operator).await;
        assert_eq!(out.status, Status::Succeeded, "{}", out.message);
        assert!(tool.ask_again(&json!({"hash": "0000000000"})).is_none());
    }

    #[tokio::test]
    async fn a_plan_without_an_intent_takes_its_first_reason() {
        let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let plan = json!({"steps": [{"skill": "GoToPlace", "why": "go to the dock",
                                     "args": [{"name": "place", "value": "dock"}]}]});
        let out = missions.plan(plan, By::Model).await;
        assert_eq!(out.status, Status::Succeeded, "{}", out.message);
        let compiled = missions.find(out.data["hash"].as_str().unwrap()).unwrap();
        assert_eq!(compiled.plan.intent, "go to the dock");
    }

    #[tokio::test]
    async fn problems_go_back_to_the_model_and_attempts_are_capped() {
        let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let bad = json!({"intent": "x", "steps": [{"skill": "PickObject", "args": {"object_id": "O99"}}]});
        for n in 1..=MAX_PLAN_ATTEMPTS {
            let out = missions.plan(bad.clone(), By::Model).await;
            assert_eq!(out.status, Status::Failed);
            assert!(out.message.contains(&format!("attempt {n} of")));
        }
        assert_eq!(
            missions.plan(steps(), By::Model).await.status,
            Status::Refused
        );
    }

    #[tokio::test]
    async fn a_failure_reports_the_step_and_limits_replans() {
        let failing = || ScriptedRun {
            result: Ok(GoalResult {
                status: GoalStatus::Aborted,
                result: json!({"outcome": 1, "failed_step_id": "s2", "failure_reason": "grasp slipped"}),
            }),
            ..ScriptedRun::default()
        };
        let now = ledger::now_s();
        let history = move |req: &Value| {
            assert_eq!(req["query"], "O18", "the failed step's object");
            let at = |ago: f64| json!({"sec": (now - ago).floor(), "nanosec": 0});
            Ok(json!({"events": [
                {"id": "O18", "label": "blue cup", "kind": "appeared", "stamp": at(7200.0),
                 "room_id": "R2", "other_id": "", "detail": ""},
                {"id": "O18", "label": "blue cup", "kind": "missing", "stamp": at(600.0),
                 "room_id": "R2", "other_id": "", "detail": ""}]}))
        };
        let robot: Arc<dyn RobotPort> =
            Arc::new(robot(failing()).with_service("/x/history", history));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let (tx, mut commands) = tokio::sync::mpsc::unbounded_channel();
        let (events, _rx) = tokio::sync::broadcast::channel(64);
        missions.attach(SessionHandle::for_tests(&tx, events));
        let hash = missions.plan(steps(), By::Model).await.data["hash"]
            .as_str()
            .unwrap()
            .to_owned();
        assert_eq!(
            missions.run(&json!({"hash": hash})).await.status,
            Status::Accepted
        );
        let Some(Command::Report(text)) = commands.recv().await else {
            panic!("no report")
        };
        assert!(
            text.contains("Failed at s2 PickObject(") && text.contains("grasp slipped"),
            "{text}"
        );
        assert!(
            text.contains("run it now: the operator approves it"),
            "{text}"
        );
        assert!(
            text.contains(
                "What happened to it: O18 blue cup: appeared 2 h ago in R2; went missing 10 min ago."
            ),
            "{text}"
        );
        missions.run_failures.store(2, Ordering::SeqCst);
        assert_eq!(
            missions.plan(steps(), By::Model).await.status,
            Status::Refused
        );
    }

    #[tokio::test]
    async fn a_plan_is_checked_before_anyone_is_asked_to_run_it() {
        let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let [tool] = missions.tools();
        let sound = tool.assess(&steps()).await.unwrap().unwrap();
        assert_eq!(sound.risk, Risk::Manipulation);
        let hash = sound.args.unwrap()["hash"].as_str().unwrap().to_owned();
        assert_eq!(hash.len(), 64);
        let bad = json!({"intent": "x", "steps": [{"skill": "PickObject", "args": {"object_id": "O99"}}]});
        let problems = tool.assess(&bad).await.unwrap().unwrap_err();
        assert_eq!(problems.status, Status::Failed);
        assert!(
            problems.message.contains("call run_mission again"),
            "{}",
            problems.message
        );
        let mut only = steps();
        only["check_only"] = json!(true);
        assert_eq!(
            tool.assess(&only).await.unwrap().unwrap().risk,
            Risk::Observe
        );
        let checked = tool.call(only).await;
        assert_eq!(checked.data["hash"], hash);
        assert!(tool.assess(&json!({"hash": hash})).await.is_none());
    }

    /// Says the same about every plan.
    struct Says(&'static str);

    #[async_trait]
    impl Critic for Says {
        async fn judge(&self, _prompt: &str) -> Result<String, String> {
            Ok(self.0.to_owned())
        }
    }

    fn missions_asked(request: &str) -> Arc<Missions> {
        let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        *lock(&missions.request) = request.to_owned();
        missions
    }

    #[tokio::test]
    async fn a_first_request_for_it_is_asked_about_not_guessed() {
        let missions = missions_asked("Pick it up.");
        missions.said.store(1, Ordering::SeqCst);
        let out = missions.plan(steps(), By::Model).await;
        assert_eq!(out.status, Status::Refused);
        assert!(
            out.message.contains("which thing they mean"),
            "{}",
            out.message
        );
        missions.said.store(2, Ordering::SeqCst);
        let later = missions.plan(steps(), By::Model).await;
        assert_eq!(
            later.status,
            Status::Succeeded,
            "said after something: {}",
            later.message
        );
    }

    #[tokio::test]
    async fn a_repeat_goes_to_a_schedule_not_a_run() {
        let missions = missions_asked("Every minute, pick up the blue cup, 2 times in all.");
        let [tool] = missions.tools();
        let back = tool.assess(&steps()).await.unwrap().unwrap_err();
        assert_eq!(back.status, Status::Refused);
        assert!(back.message.contains("schedule"), "{}", back.message);
        let checked = tool.assess(&json!({"check_only": true, "steps": []})).await;
        assert!(matches!(checked, Some(Ok(_))), "checking one run is fine");
    }

    #[tokio::test]
    async fn an_unnamed_plan_takes_the_operators_words_and_a_failed_rerun_is_not_asked() {
        let missions = missions_asked("Fetch the blue cup for me.");
        let mut plan = steps();
        plan.as_object_mut().unwrap().remove("intent");
        let [tool] = missions.tools();
        let sound = tool.assess(&plan).await.unwrap().unwrap();
        let hash = sound.args.unwrap()["hash"].as_str().unwrap().to_owned();
        assert_eq!(
            missions.find(&hash).unwrap().plan.intent,
            "Fetch the blue cup for me"
        );
        *lock(&missions.last) = Some((hash.clone(), "failed at s2: grasp slipped".to_owned()));
        let again = tool.assess(&plan).await.unwrap().unwrap_err();
        assert!(
            again.message.contains("just ran and failed"),
            "{}",
            again.message
        );
        let by_hash = tool
            .assess(&json!({"hash": hash}))
            .await
            .unwrap()
            .unwrap_err();
        assert_eq!(by_hash.status, Status::Refused);
    }

    /// An advisor that keeps what it was asked.
    struct Advises(Mutex<Vec<String>>);

    #[async_trait]
    impl advice::Advisor for Advises {
        async fn advise(&self, prompt: &str) -> Option<String> {
            lock(&self.0).push(prompt.to_owned());
            Some("Find the object first: O99 is not in the world model.".to_owned())
        }
    }

    #[tokio::test]
    async fn a_stronger_model_is_asked_once_the_plans_have_failed_twice() {
        let advisor = Arc::new(Advises(Mutex::new(Vec::new())));
        let missions = missions_asked("bring me the cup")
            .with_advisor(Arc::clone(&advisor) as Arc<dyn advice::Advisor>);
        let bad = json!({"intent": "x", "steps": [{"skill": "PickObject", "args": {"object_id": "O99"}}]});
        let first = missions.plan(bad.clone(), By::Model).await;
        assert!(!first.message.contains("suggests"), "{}", first.message);
        let second = missions.plan(bad.clone(), By::Model).await;
        assert!(
            second.message.ends_with(
                "A stronger model suggests: Find the object first: O99 is not in the world model."
            ),
            "{}",
            second.message
        );
        let third = missions.plan(bad, By::Model).await;
        assert!(!third.message.contains("suggests"), "{}", third.message);
        let asked = lock(&advisor.0);
        assert_eq!(asked.len(), 1, "asked once");
        assert!(
            asked[0].starts_with("The operator asked: bring me the cup")
                && asked[0].contains("O99")
                && asked[0].contains("PickObject("),
            "{}",
            asked[0]
        );
    }

    #[tokio::test]
    async fn a_plan_against_the_words_goes_back_once_and_then_shows_its_concern() {
        let missions = missions_asked("Pick up the blue cup with your right hand");
        let [tool] = missions.tools();
        let back = tool.assess(&steps()).await.unwrap().unwrap_err();
        assert_eq!(back.status, Status::Failed);
        assert!(
            back.data["concerns"][0]
                .as_str()
                .unwrap()
                .starts_with("s2: the operator said the right hand"),
            "{}",
            back.data
        );
        assert_eq!(
            missions.plan_failures.load(Ordering::SeqCst),
            0,
            "not one of the model's attempts"
        );
        let sent_again = tool.assess(&steps()).await.unwrap().unwrap();
        assert!(
            sent_again
                .reason
                .contains("; check: s2: the operator said the right hand"),
            "{}",
            sent_again.reason
        );
    }

    #[tokio::test]
    async fn the_critic_sends_a_wrong_plan_back_and_flags_a_doubtful_one() {
        let rejecting = missions_asked("bring me the mug").with_critic(Arc::new(Says(
            r#"{"verdict": "reject", "reason": "it picks up the cup, not the mug"}"#,
        )));
        let [tool] = rejecting.tools();
        let back = tool.assess(&steps()).await.unwrap().unwrap_err();
        assert_eq!(back.data["concerns"][0], "it picks up the cup, not the mug");
        assert!(tool.assess(&steps()).await.unwrap().is_ok());

        let doubtful = missions_asked("bring me the mug").with_critic(Arc::new(Says(
            r#"{"verdict": "ask", "reason": "which mug?"}"#,
        )));
        let [tool] = doubtful.tools();
        let shown = tool.assess(&steps()).await.unwrap().unwrap();
        assert!(
            shown.reason.ends_with("; check: which mug?"),
            "{}",
            shown.reason
        );
    }

    #[tokio::test]
    async fn the_operator_edits_a_plan_without_spending_the_models_attempts() {
        let missions = missions_asked("Pick up the blue cup with your right hand")
            .with_critic(Arc::new(Says(r#"{"verdict": "reject", "reason": "no"}"#)));
        let [tool] = missions.tools();
        let bad = json!({"intent": "x", "steps": [{"skill": "PickObject", "args": {"object_id": "O99"}}]});
        let problems = tool.assess_operator(bad).await.unwrap().unwrap_err();
        assert!(problems.message.starts_with("s1"), "{}", problems.message);
        assert_eq!(missions.plan_failures.load(Ordering::SeqCst), 0);

        // Its own concern is shown, not sent back; the critic is not asked.
        let kept_left = tool.assess_operator(steps()).await.unwrap().unwrap();
        assert!(
            kept_left.reason.contains("right hand"),
            "{}",
            kept_left.reason
        );
        assert!(
            !kept_left.reason.contains("check: no"),
            "{}",
            kept_left.reason
        );
        let mut shorter = steps();
        shorter["steps"].as_array_mut().unwrap().remove(0);
        let edited = tool.assess_operator(shorter).await.unwrap().unwrap();
        assert_ne!(edited.args, kept_left.args, "an edit is a new plan");
    }

    #[tokio::test]
    async fn the_same_plan_twice_runs_by_its_hash() {
        let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let first = missions.plan(steps(), By::Model).await.data["hash"].clone();
        let second = missions.plan(steps(), By::Model).await.data["hash"].clone();
        assert_eq!(first, second);
        assert!(missions.find(first.as_str().unwrap()).is_ok());
    }

    #[tokio::test]
    async fn a_hand_the_executor_cannot_vouch_for_is_full() {
        let state = json!({"holding_left": "", "holding_right": "",
                           "message": "idle; left hand unknown (it may hold something)"});
        assert_eq!(held_by(&state, "left"), UNKNOWN_HELD);
        assert_eq!(held_by(&state, "right"), "");
        let robot: Arc<dyn RobotPort> =
            Arc::new(robot(ScriptedRun::default()).with_topic("/x/state", state));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let out = missions.plan(steps(), By::Model).await;
        assert_eq!(out.status, Status::Failed);
        let problems = out.data["problems"].to_string();
        assert!(
            problems.contains("left hand empty, but it holds something unknown"),
            "{problems}"
        );
    }

    #[test]
    fn steps_are_found_in_names_and_paths() {
        assert_eq!(step_of("s12_PickObject").as_deref(), Some("s12"));
        assert_eq!(
            step_of("Mission/s2_GoToPlace::7/NavigateToPose::9").as_deref(),
            Some("s2")
        );
        assert_eq!(step_of("sequence_1"), None);
        assert_eq!(step_of("Pick"), None);
    }
}
