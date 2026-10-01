//! Missions: the only way the agent moves the robot.
//!
//! The model calls `plan_mission` with a plan; it compiles against the robot's skill catalog and
//! world model, and the executor's `ValidateMission` checks the tree again. Nothing moves, and
//! the model gets the tree's hash back. `run_mission` with that hash is the act: the guard asks
//! the operator, then the tree goes to the executor's `ExecuteMission`. The call returns once the
//! executor accepts; the mission runs in the background, its progress goes out as session events,
//! and when it ends the model gets a report with the outcome and the goal checks.

pub mod catalog;
pub mod check;
pub mod plan;

use std::borrow::Cow;
use std::collections::VecDeque;
use std::fmt::Write as _;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use nervros_ros::{Goal, GoalResult, RobotPort, RosError};
use serde_json::{Value, json};

use self::catalog::Catalog;
use self::check::Observed;
use self::plan::{Compiled, Plan, Thing, World};
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
/// A hash may be shortened to this many characters when it stays unique.
const MIN_HASH_PREFIX: usize = 8;
const EXECUTE: &str = "nervros_interfaces/action/ExecuteMission";
const STATE: &str = "nervros_interfaces/msg/RobotState";
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
    run_failures: AtomicU32,
    /// The plan that ran last in this request, and how it ended: run again unchanged on the
    /// model's own, it repeats a done task or fails the same way.
    last: Mutex<Option<(String, String)>>,
    session: OnceLock<SessionHandle>,
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
            last: Mutex::default(),
            session: OnceLock::new(),
        }))
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
            if let Err(e) = me.catalog().await {
                tracing::info!(error = %e, "no mission catalog yet");
            }
            // The operator speaking resets the retry limits: it is a new request.
            loop {
                match events.recv().await {
                    Ok(Event::User { .. }) => {
                        me.plan_failures.store(0, Ordering::SeqCst);
                        me.run_failures.store(0, Ordering::SeqCst);
                        *lock(&me.last) = None;
                    }
                    Ok(_) | Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {}
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => return,
                }
            }
        });
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

    async fn plan(&self, args: Value) -> ToolOutcome {
        if self.plan_failures.load(Ordering::SeqCst) >= MAX_PLAN_ATTEMPTS {
            return ToolOutcome::refused(format!(
                "{MAX_PLAN_ATTEMPTS} plans in a row failed their checks; tell the operator what is missing instead"
            ));
        }
        let max_replans = self.config.max_replans;
        let failed = self.run_failures.load(Ordering::SeqCst);
        if failed > max_replans {
            return ToolOutcome::refused(format!(
                "this request already failed {failed} times; tell the operator what went wrong instead of retrying"
            ));
        }
        let mut args = args;
        // The tool's own switches, not the plan's.
        if let Some(fields) = args.as_object_mut() {
            fields.remove("check_only");
            fields.remove("hash");
        }
        let plan: Plan = match serde_json::from_value(args) {
            Ok(p) => p,
            Err(e) => {
                return self
                    .rejected(&json!([{"step": "", "field": "", "message": e.to_string()}]));
            }
        };
        let catalog = match self.catalog().await {
            Ok(c) => c,
            Err(e) => return ToolOutcome::failed(e),
        };
        let world = self.world_of(&self.observe().await);
        let compiled = match plan::compile(&plan, &catalog, &world) {
            Ok(c) => c,
            Err(problems) => return self.rejected(&json!(problems)),
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
            return self.rejected(&diagnostics);
        }
        self.plan_failures.store(0, Ordering::SeqCst);
        let worst = reply["worst_case_duration_s"]
            .as_f64()
            .filter(|w| *w > 0.0)
            .unwrap_or(compiled.worst_case_s);
        self.emit(Event::MissionPlanned {
            hash: compiled.sha256.clone(),
            intent: plan.intent.clone(),
            steps: compiled.steps.clone(),
            worst_case_s: worst,
        });
        let out = json!({
            "hash": compiled.sha256,
            "steps": compiled.steps.iter().map(|s| format!("{} {}", s.id, s.summary)).collect::<Vec<_>>(),
            "worst_case_s": worst.round(),
            "next": "only checked: to run it, call run_mission with this hash; the operator approves it then"
        });
        let mut planned = lock(&self.planned);
        // The same plan compiles to the same tree: keep one copy, or its hash reads as ambiguous.
        planned.retain(|c| c.sha256 != compiled.sha256);
        planned.push_back(compiled);
        if planned.len() > KEPT_PLANS {
            planned.pop_front();
        }
        ToolOutcome::ok(out)
    }

    fn rejected(&self, problems: &Value) -> ToolOutcome {
        let n = self.plan_failures.fetch_add(1, Ordering::SeqCst) + 1;
        let count = problems.as_array().map_or(1, Vec::len);
        ToolOutcome {
            status: Status::Failed,
            data: json!({"ok": false, "problems": problems}),
            message: format!(
                "the plan has {count} problem(s); fix them all and call run_mission again (attempt {n} of {MAX_PLAN_ATTEMPTS})"
            ),
            images: Vec::new(),
        }
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
        if let Some((hash, how)) = lock(&self.last).as_ref()
            && *hash == compiled.sha256
        {
            return ToolOutcome::refused(format!(
                "this exact plan just ran and {how}; running it unchanged would repeat that. Change \
                 the plan to deal with what the report said, or tell the operator"
            ));
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

    /// Sends a checked plan to the executor and watches it; its id.
    async fn launch(self: &Arc<Self>, compiled: Compiled) -> Result<String, String> {
        let id = uuid::Uuid::now_v7().to_string();
        let goal = json!({
            "mission_id": id,
            "tree_xml": compiled.xml,
            "tree_sha256": compiled.sha256,
            "max_duration_s": 0.0,
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
        tokio::spawn(Arc::clone(self).watch(id.clone(), compiled, goal));
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
        let out = self.plan(json!({"intent": intent, "steps": steps})).await;
        if out.status != Status::Succeeded {
            return Err(out);
        }
        Ok((
            out.data["hash"].as_str().unwrap_or_default().to_owned(),
            out.data["steps"].as_array().map_or(0, Vec::len),
            out.data["worst_case_s"].as_f64().unwrap_or(0.0),
        ))
    }

    async fn watch(self: Arc<Self>, id: String, compiled: Compiled, goal: Goal) {
        let started = Instant::now();
        let Goal {
            mut feedback,
            result,
            ..
        } = goal;
        tokio::pin!(result);
        let result = loop {
            tokio::select! {
                Some(fb) = feedback.recv() => self.progress(&id, &fb),
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
        let seen = self.observe().await;
        let mut report = format!(
            "Mission {id} ({}) ended: {outcome} after {elapsed:.0} s.",
            compiled.plan.intent
        );
        if outcome == "success" {
            self.run_failures.store(0, Ordering::SeqCst);
            let verdicts = check::check(&compiled.goal, &seen);
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
        } else {
            let n = self.run_failures.fetch_add(1, Ordering::SeqCst) + 1;
            let what = compiled
                .steps
                .iter()
                .find(|s| s.id == step)
                .map_or_else(String::new, |s| format!(" {}", s.summary));
            let _ = write!(report, " Failed at {step}{what}: {reason}.");
            if n > self.config.max_replans {
                report.push_str(
                    " This request has failed too often: do not retry. Tell the operator what went \
                     wrong and what would help.",
                );
            } else {
                report.push_str(
                    " Find out why (robot_state, look, log_tail) and say it in one sentence. If a \
                     changed plan can work, run it now: the operator approves it. If not, say what is \
                     needed.",
                );
            }
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
        if let Some(s) = self.session.get() {
            s.send(Command::Report(report));
        }
    }

    /// Maps one feedback message's node events onto steps.
    fn progress(&self, id: &str, feedback: &Value) {
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
                String::new()
            } else {
                name.to_owned()
            };
            self.emit(Event::MissionProgress {
                id: id.to_owned(),
                step,
                node,
                status: status.to_owned(),
                elapsed_s: elapsed,
            });
        }
    }

    async fn observe(&self) -> Observed {
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
        args.get("steps")?;
        if args["check_only"].as_bool() == Some(true) {
            return Some(Ok(Assessment {
                risk: Risk::Observe,
                resources: Vec::new(),
                reason: "checks a plan".to_owned(),
                args: None,
            }));
        }
        let out = self.0.plan(args.clone()).await;
        if out.status != Status::Succeeded {
            return Some(Err(out));
        }
        let steps = out.data["steps"].as_array().map_or(0, Vec::len);
        let minutes = (out.data["worst_case_s"].as_f64().unwrap_or(0.0) / 60.0).ceil();
        Some(Ok(Assessment {
            risk: Risk::Manipulation,
            resources: Vec::new(),
            reason: format!(
                "runs \"{}\": {steps} step(s), at most {minutes} min",
                args["intent"].as_str().unwrap_or("the plan")
            ),
            args: Some(json!({"hash": out.data["hash"]})),
        }))
    }

    /// The skill list loads when the session starts; a first message sent at once would see none.
    async fn ready(&self) {
        let _ = tokio::time::timeout(SERVICE_TIMEOUT / 2, self.0.catalog()).await;
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        // Steps reach here only to be checked: a run's were replaced by their hash.
        if args.get("steps").is_some() {
            return self.0.plan(args).await;
        }
        self.0.run(&args).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::catalog::tests::CATALOG;
    use crate::places::Places;
    use nervros_ros::GoalStatus;
    use nervros_ros::fake::{FakeRobot, ScriptedRun};
    use std::path::Path;

    fn profile() -> Profile {
        let text = r#"
            [robot]
            name = "t"
            [world]
            objects = { topic = "/objects", type = "canopy_msgs/msg/WorldObjectArray" }
            [mission]
            execute = "/x/execute"
            validate = "/x/validate"
            catalog = "/x/catalog"
            stop = "/x/stop"
            state = "/x/state"
            max_replans = 1
            [[place]]
            name = "dock"
            pose = { x = 1.0, y = 2.0 }
            [models]
            file = "m.toml"
        "#;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("nervros.toml");
        std::fs::write(&path, text).unwrap();
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

        let planned = missions.plan(steps()).await;
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
    async fn problems_go_back_to_the_model_and_attempts_are_capped() {
        let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let bad = json!({"intent": "x", "steps": [{"skill": "PickObject", "args": {"object_id": "O99"}}]});
        for n in 1..=MAX_PLAN_ATTEMPTS {
            let out = missions.plan(bad.clone()).await;
            assert_eq!(out.status, Status::Failed);
            assert!(out.message.contains(&format!("attempt {n} of")));
        }
        assert_eq!(missions.plan(steps()).await.status, Status::Refused);
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
        let robot: Arc<dyn RobotPort> = Arc::new(robot(failing()));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let (tx, mut commands) = tokio::sync::mpsc::unbounded_channel();
        let (events, _rx) = tokio::sync::broadcast::channel(64);
        missions.attach(SessionHandle::for_tests(&tx, events));
        let hash = missions.plan(steps()).await.data["hash"]
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
        missions.run_failures.store(2, Ordering::SeqCst);
        assert_eq!(missions.plan(steps()).await.status, Status::Refused);
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

    #[tokio::test]
    async fn the_same_plan_twice_runs_by_its_hash() {
        let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
        let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
        let first = missions.plan(steps()).await.data["hash"].clone();
        let second = missions.plan(steps()).await.data["hash"].clone();
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
        let out = missions.plan(steps()).await;
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
