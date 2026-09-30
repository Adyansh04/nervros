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
use crate::tools::{Risk, Status, Tool, ToolOutcome, ToolSpec};

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
    profile: Profile,
    config: MissionConfig,
    catalog: Mutex<Option<Catalog>>,
    planned: Mutex<VecDeque<Compiled>>,
    running: Mutex<Option<Running>>,
    plan_failures: AtomicU32,
    run_failures: AtomicU32,
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
    pub fn new(profile: &Profile, robot: Arc<dyn RobotPort>) -> Option<Arc<Self>> {
        let config = profile.mission.clone()?;
        Some(Arc::new(Self {
            robot,
            profile: profile.clone(),
            config,
            catalog: Mutex::default(),
            planned: Mutex::default(),
            running: Mutex::default(),
            plan_failures: AtomicU32::new(0),
            run_failures: AtomicU32::new(0),
            session: OnceLock::new(),
        }))
    }

    /// The two tools.
    #[must_use]
    pub fn tools(self: &Arc<Self>) -> [Arc<dyn Tool>; 2] {
        [
            Arc::new(PlanMission(Arc::clone(self))),
            Arc::new(RunMission {
                missions: Arc::clone(self),
                spec: ToolSpec {
                    timeout: SERVICE_TIMEOUT,
                    ..ToolSpec::new(
                        "run_mission",
                        "Runs a plan that plan_mission accepted, by its hash. The operator approves \
                         it first. Returns once the robot starts; a report follows when it ends.",
                        json!({
                            "type": "object",
                            "properties": {"hash": {"type": "string", "description": "The hash plan_mission returned"}},
                            "required": ["hash"],
                            "additionalProperties": false
                        }),
                        Risk::Manipulation,
                    )
                },
            }),
        ]
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
            .map(|arm| {
                let held = seen.state[format!("holding_{arm}")]
                    .as_str()
                    .unwrap_or_default();
                (arm.to_owned(), held.to_owned())
            })
            .collect();
        World {
            places: self.profile.places.clone(),
            rooms: things(&seen.rooms, "rooms", "name", "/centroid"),
            objects: things(&seen.objects, "objects", "label", "/pose/position"),
            robot: seen.pose,
            holding,
        }
    }

    fn plan_spec(&self) -> ToolSpec {
        let catalog = lock(&self.catalog).clone();
        let (skills, listing) = match &catalog {
            Some(c) => (
                json!({"type": "string", "enum": c.plan_skills()}),
                format!("{}\n{}", c.signatures().join(", "), c.describe()),
            ),
            None => (
                json!({"type": "string"}),
                "(the skill list loads on first use; call plan_mission to get it)".to_owned(),
            ),
        };
        let description = format!(
            "Checks a plan for the robot without moving it and returns the plan's hash; then call \
             run_mission with the hash. Each step names a skill and gives every one of its arguments \
             as {{name, value}}, such as {{\"skill\": \"GoToPlace\", \"args\": [{{\"name\": \"place\", \
             \"value\": \"kitchen\"}}]}}. Use ids from list_places and find_objects. A walk up to an \
             object that a step must be near is added for you. Skills, by their exact names: {listing}"
        );
        let parameters = json!({
            "type": "object",
            "properties": {
                "intent": {"type": "string", "description": "What the operator asked for, in a few words"},
                "goal": {"type": "array", "items": {"type": "string"},
                         "description": "What should hold at the end: at(place), holding(arm, object_id), inside(object_id, container_id), on(object_id, surface_id)"},
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
            "required": ["intent", "steps"],
            "additionalProperties": false
        });
        ToolSpec {
            timeout: SERVICE_TIMEOUT * 2,
            ..ToolSpec::new("plan_mission", &description, parameters, Risk::Observe)
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
            "next": "call run_mission with this hash; the operator approves it first"
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
                "the plan has {count} problem(s); fix them all and call plan_mission again (attempt {n} of {MAX_PLAN_ATTEMPTS})"
            ),
            images: Vec::new(),
        }
    }

    fn find(&self, hash: &str) -> Result<Compiled, String> {
        let hash = hash.trim().to_ascii_lowercase();
        let planned = lock(&self.planned);
        let hits: Vec<&Compiled> = planned
            .iter()
            .filter(|c| hash.len() >= MIN_HASH_PREFIX && c.sha256.starts_with(&hash))
            .collect();
        match hits.as_slice() {
            [one] => Ok((*one).clone()),
            [] => Err("no accepted plan has this hash; call plan_mission first".to_owned()),
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
        let id = uuid::Uuid::now_v7().to_string();
        let goal = json!({
            "mission_id": id,
            "tree_xml": compiled.xml,
            "tree_sha256": compiled.sha256,
            "max_duration_s": 0.0,
            "mode": 0
        });
        let goal = match self
            .robot
            .send_goal(&self.config.execute, EXECUTE, goal, SERVICE_TIMEOUT)
            .await
        {
            Ok(g) => g,
            Err(e) => {
                return ToolOutcome::failed(format!("the executor did not start the mission: {e}"));
            }
        };
        *lock(&self.running) = Some(Running { id: id.clone() });
        self.emit(Event::MissionStarted {
            id: id.clone(),
            hash: compiled.sha256.clone(),
        });
        tokio::spawn(Arc::clone(self).watch(id.clone(), compiled, goal));
        ToolOutcome {
            status: Status::Accepted,
            data: json!({"mission_id": id}),
            message: "the mission is running; a report follows when it ends. Tell the operator it started."
                .to_owned(),
            images: Vec::new(),
        }
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
            let verdicts = check::check(&compiled.plan.goal, &seen);
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
                    " This request has failed too often: do not retry; tell the operator.",
                );
            } else {
                report.push_str(" You may plan once more if the cause is clear.");
            }
        }
        if !seen.state.is_null() {
            let hands: Vec<String> = ["left", "right"]
                .iter()
                .filter_map(|h| {
                    let held = seen.state[format!("holding_{h}")]
                        .as_str()
                        .filter(|o| !o.is_empty())?;
                    Some(format!("the {h} hand holds {held}"))
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
            places: self.profile.places.clone(),
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

/// `plan_mission`.
struct PlanMission(Arc<Missions>);

#[async_trait]
impl Tool for PlanMission {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Owned(self.0.plan_spec())
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        self.0.plan(args).await
    }
}

/// `run_mission`.
struct RunMission {
    missions: Arc<Missions>,
    spec: ToolSpec,
}

#[async_trait]
impl Tool for RunMission {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        self.missions.run(&args).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::catalog::tests::CATALOG;
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
                json!({"objects": [{"id": "O17", "label": "red mug", "state": 0,
                    "pose": {"position": {"x": 1.0, "y": 2.0}}, "size": {"x": 0.1, "y": 0.1}}]}),
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
            {"skill": "PickObject", "args": {"object_id": "O17", "phrase": "red mug", "arm": "left"}}
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
        let missions = Missions::new(&profile(), robot).unwrap();
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
        assert!(
            text.contains("at(dock) holds") && text.contains("holding(right, O17) holds"),
            "{text}"
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
        let missions = Missions::new(&profile(), robot).unwrap();
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
        let missions = Missions::new(&profile(), robot).unwrap();
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
        assert!(text.contains("plan once more"), "{text}");
        missions.run_failures.store(2, Ordering::SeqCst);
        assert_eq!(missions.plan(steps()).await.status, Status::Refused);
    }

    #[tokio::test]
    async fn the_same_plan_twice_runs_by_its_hash() {
        let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
        let missions = Missions::new(&profile(), robot).unwrap();
        let first = missions.plan(steps()).await.data["hash"].clone();
        let second = missions.plan(steps()).await.data["hash"].clone();
        assert_eq!(first, second);
        assert!(missions.find(first.as_str().unwrap()).is_ok());
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
