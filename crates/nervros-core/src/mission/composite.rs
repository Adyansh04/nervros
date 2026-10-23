//! The composite executor: the robot's own executor, when it has one, together with the motions
//! NervROS's integrations run themselves, behind the executor contract.
//!
//! [`super::Missions`] talks to it as to any executor. A tree of the executor's skills alone goes to
//! the executor unchanged. A tree with native steps runs here, step by step: each run of executor
//! steps goes to the executor as a mission of its own, with the step elements copied as they are,
//! so their names, and the step ids the feedback carries, stay the plan's; native steps run in this
//! process, inside their `Timeout`, `RetryUntilSuccessful` and `ForceSuccess`. What a step hands on
//! goes into the arguments of the steps after it before they are sent.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use futures::stream::BoxStream;
use nervros_ros::{
    CancelFn, Frame, Goal, GoalResult, GoalStatus, Graph, GraphDetail, NodeEntities, Publisher,
    RobotPort, RosError, TfLink, TopicEndpoints, Transform,
};
use serde_json::{Map, Value, json};
use sha2::{Digest, Sha256};
use tokio::sync::{mpsc, oneshot, watch};

use crate::integrations::{Control, NativeSkill};
use crate::profile::{MissionConfig, PlanChecks, TurnSkill, WalkSkill};

/// The composite's own services, in place of the executor's.
pub(crate) const EXECUTE: &str = "/nervros/composite/execute_mission";
const VALIDATE: &str = "/nervros/composite/validate_mission";
const CATALOG: &str = "/nervros/composite/get_catalog";
const STOP: &str = "/nervros/composite/stop_all";
const PAUSE: &str = "/nervros/composite/pause_mission";
const STATE: &str = "/nervros/composite/robot_state";
const PREVIEW: &str = "/nervros/composite/preview_mission";
const EXECUTE_TYPE: &str = "nervros_interfaces/action/ExecuteMission";
const CALL_WAIT: Duration = Duration::from_secs(10);
/// How often a native step's progress goes out while it runs.
const TICK: Duration = Duration::from_secs(1);
/// `NodeEvent` statuses.
const RUNNING: u8 = 1;
const SUCCESS: u8 = 2;
const FAILURE: u8 = 3;
/// `ExecuteMission` outcomes.
const OUTCOME_SUCCESS: u8 = 0;
const OUTCOME_FAILURE: u8 = 1;
const OUTCOME_CANCELED: u8 = 2;
const OUTCOME_TIMEOUT: u8 = 3;
const OUTCOME_REJECTED: u8 = 4;

/// The `[mission]` config [`super::Missions`] uses with the composite: its own services in place of
/// the executor's, the rest as the robot's executor has it; checks of the natives' turn and walk
/// when the robot gives none.
pub(crate) fn config(real: Option<&MissionConfig>) -> MissionConfig {
    let checks = real.map_or_else(
        || PlanChecks {
            turn: Some(TurnSkill {
                skill: "TurnInPlace".to_owned(),
                degrees: "degrees".to_owned(),
                positive_left: true,
            }),
            walk: Some(WalkSkill {
                skill: "WalkStraight".to_owned(),
                metres: "distance_m".to_owned(),
                direction: "direction".to_owned(),
                max_m: 2.0,
            }),
            arm: None,
        },
        |r| r.checks.clone(),
    );
    MissionConfig {
        execute: EXECUTE.to_owned(),
        validate: VALIDATE.to_owned(),
        catalog: CATALOG.to_owned(),
        stop: STOP.to_owned(),
        pause: Some(PAUSE.to_owned()),
        // Native steps preview themselves, with or without the executor's preview.
        preview: Some(PREVIEW.to_owned()),
        define_skill: real.and_then(|r| r.define_skill.clone()),
        forget_skill: real.and_then(|r| r.forget_skill.clone()),
        state: STATE.to_owned(),
        max_replans: real.map_or(2, |r| r.max_replans),
        heartbeat: real.and_then(|r| r.heartbeat.clone()),
        heartbeat_timeout_s: real.map_or(2.0, |r| r.heartbeat_timeout_s),
        teleop: real.and_then(|r| r.teleop.clone()),
        teleop_cmd: real.and_then(|r| r.teleop_cmd.clone()),
        teleop_speed: real.map_or([0.5, 0.3, 0.8], |r| r.teleop_speed),
        lend: real.and_then(|r| r.lend.clone()),
        checks,
    }
}

/// One step of a mission tree.
#[derive(Debug, Clone, PartialEq)]
struct TreeStep {
    /// The step's element, wrappers and all, as written.
    xml: String,
    /// The macro or native skill it runs.
    template: String,
    /// `sN_Skill`.
    name: String,
    /// Its attributes but `ID` and `name`.
    args: BTreeMap<String, String>,
    timeout: Option<Duration>,
    attempts: u32,
    forced: bool,
}

/// The steps of a tree NervROS wrote.
fn parse(tree: &str) -> Result<Vec<TreeStep>, String> {
    let doc = roxmltree::Document::parse(tree)
        .map_err(|e| format!("the mission tree is not XML: {e}"))?;
    let sequence = doc
        .descendants()
        .find(|n| n.has_tag_name("Sequence") && n.attribute("name") == Some("mission"))
        .ok_or("the mission tree has no mission sequence")?;
    let mut steps = Vec::new();
    for child in sequence.children().filter(roxmltree::Node::is_element) {
        let (mut timeout, mut attempts, mut forced) = (None, 1, false);
        let mut node = child;
        loop {
            match node.tag_name().name() {
                "Timeout" => {
                    timeout = node
                        .attribute("msec")
                        .and_then(|m| m.parse().ok())
                        .map(Duration::from_millis);
                }
                "RetryUntilSuccessful" => {
                    attempts = node
                        .attribute("num_attempts")
                        .and_then(|a| a.parse().ok())
                        .unwrap_or(1);
                }
                "ForceSuccess" => forced = true,
                "SubTree" => break,
                other => return Err(format!("a mission step may not be a `{other}`")),
            }
            node = node
                .children()
                .find(roxmltree::Node::is_element)
                .ok_or("a mission step wraps nothing")?;
        }
        let args = node
            .attributes()
            .filter(|a| a.name() != "ID" && a.name() != "name")
            .map(|a| (a.name().to_owned(), a.value().to_owned()))
            .collect();
        steps.push(TreeStep {
            xml: tree[child.range()].to_owned(),
            template: node.attribute("ID").unwrap_or_default().to_owned(),
            name: node.attribute("name").unwrap_or_default().to_owned(),
            args,
            timeout,
            attempts,
            forced,
        });
    }
    Ok(steps)
}

/// A tree of these step elements, as NervROS writes one.
fn tree_of(steps: &[String]) -> String {
    let mut body = String::new();
    for s in steps {
        let _ = writeln!(body, "      {s}");
    }
    format!(
        "<root BTCPP_format=\"4\" main_tree_to_execute=\"Mission\">\n  \
         <BehaviorTree ID=\"Mission\">\n    <Sequence name=\"mission\">\n{body}    </Sequence>\n  \
         </BehaviorTree>\n</root>\n"
    )
}

fn sha(text: &str) -> String {
    format!("{:x}", Sha256::digest(text.as_bytes()))
}

/// `{key}` in a step's element, filled with what an earlier step handed on under `key`.
fn fill(xml: &str, outputs: &Map<String, Value>) -> String {
    let mut out = xml.to_owned();
    for (key, value) in outputs {
        let text = value
            .as_str()
            .map_or_else(|| value.to_string(), str::to_owned);
        out = out.replace(
            &format!("=\"{{{key}}}\""),
            &format!("=\"{}\"", escape(&text)),
        );
    }
    out
}

/// What is wrong with `value` for the catalog argument `arg`, as an executor would check it; a
/// key an earlier step hands on is checked when it is filled in.
fn arg_problem(arg: &Value, value: &str) -> Option<String> {
    let value = value.trim();
    if value.starts_with('{') && value.ends_with('}') {
        return None;
    }
    let choices: Vec<&str> = arg["enum"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .collect();
    if !choices.is_empty() && !choices.contains(&value) {
        return Some(format!("must be one of {}", choices.join(", ")));
    }
    let number = |v: &str| v.parse::<f64>().ok().filter(|n| n.is_finite());
    match arg["format"].as_str().unwrap_or_default() {
        "number" => {
            let Some(n) = number(value) else {
                return Some("must be a number".to_owned());
            };
            let (min, max) = (arg["min"].as_f64(), arg["max"].as_f64());
            (min.is_some_and(|m| n < m) || max.is_some_and(|m| n > m)).then(|| {
                format!(
                    "must be from {} to {}",
                    min.unwrap_or(f64::MIN),
                    max.unwrap_or(f64::MAX)
                )
            })
        }
        "station" => (value.split(';').filter_map(number).count() != 3
            || value.split(';').count() != 3)
            .then(|| "must be x;y;yaw".to_owned()),
        _ => None,
    }
}

fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('"', "&quot;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

/// The mission the composite runs.
struct Running {
    id: String,
    /// The step running here, empty while the executor runs one.
    step: String,
    stop: watch::Sender<bool>,
    /// Why it was stopped, as the stop said.
    stopped_for: String,
    paused: watch::Sender<bool>,
    /// The executor's goal while it runs a stretch of the mission.
    executor: Option<CancelFn>,
}

/// The composite executor; see the module.
pub(crate) struct Composite {
    inner: Arc<dyn RobotPort>,
    real: Option<MissionConfig>,
    natives: BTreeMap<String, Arc<dyn NativeSkill>>,
    running: Arc<Mutex<Option<Running>>>,
    boot: String,
}

impl Composite {
    /// Over the robot's port and its executor, if any; `natives` the integrations' motions,
    /// the ones the executor has under the same name left out when the catalog is merged.
    pub(crate) fn new(
        inner: Arc<dyn RobotPort>,
        real: Option<MissionConfig>,
        natives: Vec<Arc<dyn NativeSkill>>,
    ) -> Self {
        let natives = natives
            .into_iter()
            .filter_map(|n| Some((n.skill()["name"].as_str()?.to_owned(), n)))
            .collect();
        Self {
            inner,
            real,
            natives,
            running: Arc::default(),
            boot: uuid::Uuid::new_v4().to_string(),
        }
    }

    /// The executor's catalog and the natives it does not have, as `GetCatalog` answers.
    async fn catalog(&self) -> Result<Value, RosError> {
        let mut reply = match &self.real {
            Some(real) => {
                self.inner
                    .call(
                        &real.catalog,
                        "nervros_interfaces/srv/GetCatalog",
                        json!({}),
                        CALL_WAIT,
                    )
                    .await?
            }
            None => {
                json!({"catalog_json": "{\"catalog_version\": \"\", \"skills\": []}", "tree_nodes_model_xml": "", "bt_cpp_version": ""})
            }
        };
        let mut catalog: Value =
            serde_json::from_str(reply["catalog_json"].as_str().unwrap_or("{}")).map_err(|e| {
                RosError::Conversion {
                    name: CATALOG.to_owned(),
                    message: e.to_string(),
                }
            })?;
        let has: BTreeSet<String> = catalog["skills"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|s| s["name"].as_str().map(str::to_owned))
            .collect();
        let mut added = Vec::new();
        if let Some(skills) = catalog["skills"].as_array_mut() {
            for (name, native) in &self.natives {
                if !has.contains(name) {
                    skills.push(native.skill());
                    added.push(name.clone());
                }
            }
        }
        let version = sha(&format!(
            "{}{}",
            catalog["catalog_version"].as_str().unwrap_or_default(),
            added.join(",")
        ));
        catalog["catalog_version"] = json!(version[..12]);
        reply["catalog_json"] = json!(catalog.to_string());
        reply["catalog_version"] = json!(version[..12]);
        Ok(reply)
    }

    async fn catalog_of_executor(&self) -> Result<BTreeSet<String>, RosError> {
        let Some(real) = &self.real else {
            return Ok(BTreeSet::new());
        };
        let reply = self
            .inner
            .call(
                &real.catalog,
                "nervros_interfaces/srv/GetCatalog",
                json!({}),
                CALL_WAIT,
            )
            .await?;
        let catalog: Value = serde_json::from_str(reply["catalog_json"].as_str().unwrap_or("{}"))
            .unwrap_or_default();
        Ok(catalog["skills"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|s| s["name"].as_str().map(str::to_owned))
            .collect())
    }

    /// Which steps run here.
    async fn natives_in(&self, steps: &[TreeStep]) -> Vec<Option<Arc<dyn NativeSkill>>> {
        let executor = self.catalog_of_executor().await.unwrap_or_default();
        steps
            .iter()
            .map(|s| {
                (!executor.contains(&s.template))
                    .then(|| self.natives.get(&s.template).cloned())
                    .flatten()
            })
            .collect()
    }

    async fn validate(&self, request: &Value) -> Result<Value, RosError> {
        let tree = request["tree_xml"].as_str().unwrap_or_default();
        let steps = match parse(tree) {
            Ok(s) => s,
            Err(why) => {
                return Ok(
                    json!({"ok": false, "diagnostics_json": diagnostics(&why), "tree_sha256": sha(tree), "worst_case_duration_s": 0.0}),
                );
            }
        };
        let natives = self.natives_in(&steps).await;
        if natives.iter().all(Option::is_none) {
            return match &self.real {
                Some(real) => {
                    self.inner
                        .call(
                            &real.validate,
                            "nervros_interfaces/srv/ValidateMission",
                            request.clone(),
                            CALL_WAIT,
                        )
                        .await
                }
                None => Ok(
                    json!({"ok": false, "diagnostics_json": diagnostics("this robot has no executor for these skills"), "tree_sha256": sha(tree), "worst_case_duration_s": 0.0}),
                ),
            };
        }
        let mut problems = Vec::new();
        let mut worst = 0.0;
        let mut run: Vec<String> = Vec::new();
        for (step, native) in steps.iter().zip(&natives) {
            match native {
                Some(native) => {
                    if !run.is_empty() {
                        worst += self.validate_stretch(&run, &mut problems).await;
                        run.clear();
                    }
                    let skill = native.skill();
                    let limit = skill["max_duration_s"].as_f64().unwrap_or(60.0);
                    let each = step.timeout.map_or(limit, |t| t.as_secs_f64().min(limit));
                    worst += each * f64::from(step.attempts.max(1));
                    for arg in skill["args"].as_array().into_iter().flatten() {
                        let name = arg["name"].as_str().unwrap_or_default();
                        let why = match step.args.get(name) {
                            None => Some(format!("{} needs `{name}`", step.template)),
                            Some(v) => arg_problem(arg, v)
                                .map(|w| format!("{}: `{name}` {w}", step.template)),
                        };
                        if let Some(message) = why {
                            problems.push(json!({"code": "BAD_ARG", "line": 0, "node": step.name, "message": message}));
                        }
                    }
                    if let Some(why) = native.check(&step.args).await {
                        problems.push(json!({"code": "BAD_ARG", "line": 0, "node": step.name,
                            "message": format!("{}: {why}", step.template)}));
                    }
                    let topics = self.inner.graph().await.map(|g| g.topics);
                    for action in native.needs() {
                        if topics
                            .as_ref()
                            .is_ok_and(|t| !crate::integrations::has_action(t, &action))
                        {
                            problems.push(json!({"code": "UNAVAILABLE", "line": 0, "node": step.name, "message": format!("{} needs {action}, which is not running", step.template)}));
                        }
                    }
                }
                None => run.push(step.xml.clone()),
            }
        }
        if !run.is_empty() {
            worst += self.validate_stretch(&run, &mut problems).await;
        }
        Ok(json!({
            "ok": problems.is_empty(),
            "diagnostics_json": Value::Array(problems).to_string(),
            "tree_sha256": sha(tree),
            "worst_case_duration_s": worst,
        }))
    }

    /// Validates a stretch of executor steps with the executor; its worst case. A reference to
    /// what an earlier stretch hands on is checked as a literal.
    async fn validate_stretch(&self, steps: &[String], problems: &mut Vec<Value>) -> f64 {
        let Some(real) = &self.real else {
            problems.push(json!({"code": "UNKNOWN_MACRO", "line": 0, "node": "", "message": "this robot has no executor for these skills"}));
            return 0.0;
        };
        let literal: Vec<String> = steps.iter().map(|s| unbind(s)).collect();
        let reply = self
            .inner
            .call(
                &real.validate,
                "nervros_interfaces/srv/ValidateMission",
                json!({"tree_xml": tree_of(&literal)}),
                CALL_WAIT,
            )
            .await;
        match reply {
            Ok(r) => {
                if r["ok"].as_bool() != Some(true) {
                    let found: Vec<Value> =
                        serde_json::from_str(r["diagnostics_json"].as_str().unwrap_or("[]"))
                            .unwrap_or_default();
                    problems.extend(found);
                }
                r["worst_case_duration_s"].as_f64().unwrap_or(0.0)
            }
            Err(e) => {
                problems.push(json!({"code": "LOAD_ERROR", "line": 0, "node": "", "message": format!("the executor did not check its steps: {e}")}));
                0.0
            }
        }
    }

    /// Where the plan would take the robot: the executor's preview of the steps before the first
    /// native one, which it chains from where the robot stands, and each native step's own.
    async fn preview(
        &self,
        request: &Value,
        service_type: &str,
        timeout: Duration,
    ) -> Result<Value, RosError> {
        let tree = request["tree_xml"].as_str().unwrap_or_default();
        let steps = parse(tree).map_err(RosError::Rejected)?;
        let natives = self.natives_in(&steps).await;
        let real = self.real.as_ref().and_then(|r| r.preview.clone());
        if natives.iter().all(Option::is_none) {
            return match real {
                Some(real) => {
                    self.inner
                        .call(&real, service_type, request.clone(), timeout)
                        .await
                }
                None => Err(RosError::Unavailable(PREVIEW.to_owned())),
            };
        }
        let mut out = Vec::new();
        let lead = natives.iter().take_while(|n| n.is_none()).count();
        if let (true, Some(real)) = (lead > 0, &real) {
            let first: Vec<String> = steps[..lead].iter().map(|s| unbind(&s.xml)).collect();
            let ahead = json!({"tree_xml": tree_of(&first)});
            if let Ok(r) = self.inner.call(real, service_type, ahead, timeout).await
                && r["ok"].as_bool() == Some(true)
            {
                out.extend(r["steps"].as_array().cloned().unwrap_or_default());
            }
        }
        for (step, native) in steps.iter().zip(&natives) {
            let Some(native) = native else {
                continue;
            };
            if let Some(mut seen) = native.preview(&step.args).await {
                let id = step.name.split('_').next().unwrap_or_default();
                seen["step_id"] = json!(id);
                seen["name"] = json!(step.name);
                out.push(seen);
            }
        }
        Ok(json!({"ok": true, "message": "", "steps": out}))
    }

    async fn stop(&self, request: &Value) -> Result<Value, RosError> {
        if let Some(r) = crate::lock(&self.running).as_mut() {
            request["reason"]
                .as_str()
                .unwrap_or("stop")
                .clone_into(&mut r.stopped_for);
            r.stop.send_replace(true);
        }
        match &self.real {
            Some(real) => {
                self.inner
                    .call(
                        &real.stop,
                        "nervros_interfaces/srv/StopAll",
                        request.clone(),
                        CALL_WAIT,
                    )
                    .await
            }
            None => Ok(json!({"ok": true, "message": "stopped", "state_after": "stopped"})),
        }
    }

    async fn pause(&self, request: &Value) -> Result<Value, RosError> {
        let on = request["pause"].as_bool().unwrap_or(false);
        // A step NervROS runs pauses here. The executor pauses its own stretch, and a mission of
        // its skills alone, which goes to it straight and leaves nothing running here.
        let here = match crate::lock(&self.running).as_ref() {
            Some(r) if r.executor.is_none() => {
                r.paused.send_replace(on);
                Some(true)
            }
            Some(_) => Some(false),
            None => None,
        };
        if here == Some(true) {
            return Ok(json!({"ok": true, "message": if on { "paused" } else { "resumed" }}));
        }
        match self.real.as_ref().and_then(|r| r.pause.clone()) {
            Some(service) => {
                self.inner
                    .call(
                        &service,
                        "nervros_interfaces/srv/PauseMission",
                        request.clone(),
                        CALL_WAIT,
                    )
                    .await
            }
            None if here.is_none() => Ok(json!({"ok": false, "message": "no mission is running"})),
            None => Ok(json!({"ok": false, "message": "the executor cannot pause"})),
        }
    }

    /// The executor's state, with the mission the composite runs; one made here when there is no
    /// executor.
    async fn state(&self, wait: Duration, max_age: Duration) -> Result<Value, RosError> {
        let real = match &self.real {
            Some(real) => Some(
                self.inner
                    .latest_fresh(
                        &real.state,
                        "nervros_interfaces/msg/RobotState",
                        wait,
                        max_age,
                    )
                    .await?,
            ),
            None => None,
        };
        let mut state = real.unwrap_or_else(|| {
            json!({
                "boot_id": self.boot, "mission_id": "", "mission_step": "", "resources_held": [],
                "holding_left": "", "holding_right": "", "stopped": false, "message": "",
                "can_move": true, "cannot_move_reason": "", "tilt_deg": Value::Null,
                "teleop": false, "paused": false,
            })
        });
        if let Some(r) = crate::lock(&self.running).as_ref() {
            state["mission_id"] = json!(r.id);
            if !r.step.is_empty() {
                state["mission_step"] = json!(r.step);
                state["paused"] = json!(*r.paused.borrow());
            }
        }
        Ok(state)
    }

    async fn execute(&self, goal: Value, timeout: Duration) -> Result<Goal, RosError> {
        let tree = goal["tree_xml"].as_str().unwrap_or_default();
        let steps = parse(tree).map_err(RosError::Rejected)?;
        let natives = self.natives_in(&steps).await;
        if natives.iter().all(Option::is_none) {
            let Some(real) = &self.real else {
                return Err(RosError::Unavailable(
                    "an executor for these skills".to_owned(),
                ));
            };
            return self
                .inner
                .send_goal(&real.execute, EXECUTE_TYPE, goal, timeout)
                .await;
        }
        let id = goal["mission_id"].as_str().unwrap_or_default().to_owned();
        let (stop, stop_rx) = watch::channel(false);
        let (paused, paused_rx) = watch::channel(false);
        {
            let mut running = crate::lock(&self.running);
            if running.is_some() {
                return Err(RosError::Rejected("another mission is running".to_owned()));
            }
            *running = Some(Running {
                id: id.clone(),
                step: String::new(),
                stop: stop.clone(),
                stopped_for: String::new(),
                paused,
                executor: None,
            });
        }
        let (feedback_tx, feedback_rx) = mpsc::channel(64);
        let (result_tx, result_rx) = oneshot::channel();
        let run = Run {
            inner: Arc::clone(&self.inner),
            real: self.real.clone(),
            running: Arc::clone(&self.running),
            feedback: feedback_tx,
            ctl: Control {
                stop: stop_rx,
                paused: paused_rx,
            },
            goal,
            started: Instant::now(),
        };
        let plan: Vec<(TreeStep, Option<Arc<dyn NativeSkill>>)> =
            steps.into_iter().zip(natives).collect();
        let running = Arc::clone(&self.running);
        tokio::spawn(async move {
            let result = run.all(plan).await;
            *crate::lock(&running) = None;
            let status = match result["outcome"].as_u64() {
                Some(0) => GoalStatus::Succeeded,
                Some(2) => GoalStatus::Canceled,
                _ => GoalStatus::Aborted,
            };
            let _ = result_tx.send(Ok(GoalResult { status, result }));
        });
        let cancel_stop = stop;
        let cancel: CancelFn = Arc::new(move || {
            cancel_stop.send_replace(true);
            Box::pin(async { Ok(()) })
        });
        Ok(Goal::new(
            uuid::Uuid::new_v4(),
            feedback_rx,
            result_rx,
            cancel,
        ))
    }
}

/// A `{key}` reference to what another stretch handed on, as a literal the executor checks as any.
fn unbind(xml: &str) -> String {
    let mut out = String::with_capacity(xml.len());
    let mut rest = xml;
    while let Some(at) = rest.find("=\"{") {
        out.push_str(&rest[..at + 2]);
        rest = &rest[at + 3..];
        match rest.find("}\"") {
            Some(end) => {
                out.push_str(&rest[..end]);
                rest = &rest[end + 1..];
            }
            None => out.push('{'),
        }
    }
    out.push_str(rest);
    out
}

fn diagnostics(message: &str) -> String {
    json!([{"code": "BAD_STRUCTURE", "line": 0, "node": "", "message": message}]).to_string()
}

/// One mission the composite runs.
struct Run {
    inner: Arc<dyn RobotPort>,
    real: Option<MissionConfig>,
    running: Arc<Mutex<Option<Running>>>,
    feedback: mpsc::Sender<Value>,
    ctl: Control,
    goal: Value,
    started: Instant,
}

impl Run {
    fn elapsed(&self) -> f64 {
        self.started.elapsed().as_secs_f64()
    }

    async fn event(&self, step: &str, prev: u8, status: u8) {
        let _ = self
            .feedback
            .send(json!({
                "elapsed_s": self.elapsed(),
                "running_nodes": if status == RUNNING { vec![step.to_owned()] } else { Vec::new() },
                "events": [{"name": step, "path": step, "prev_status": prev, "status": status}],
                "paused": *self.ctl.paused.borrow(),
            }))
            .await;
    }

    fn result(&self, outcome: u8, step: &str, reason: &str, outputs: &Map<String, Value>) -> Value {
        json!({
            "outcome": outcome,
            "failed_node": step,
            "failed_step_id": step.split('_').next().unwrap_or_default(),
            "failure_reason": reason,
            "diagnostics_json": "[]",
            "cause": 0,
            "elapsed_s": self.elapsed(),
            "outputs_json": Value::Object(outputs.clone()).to_string(),
        })
    }

    /// Runs the plan stretch by stretch; the result in `ExecuteMission`'s shape.
    async fn all(&self, plan: Vec<(TreeStep, Option<Arc<dyn NativeSkill>>)>) -> Value {
        let mut outputs = Map::new();
        let mut stretch: Vec<TreeStep> = Vec::new();
        for (step, native) in plan {
            let Some(native) = native else {
                stretch.push(step);
                continue;
            };
            if !stretch.is_empty()
                && let Some(failed) = self
                    .executor(&std::mem::take(&mut stretch), &mut outputs)
                    .await
            {
                return failed;
            }
            if let Some(failed) = self.native(&step, &native, &mut outputs).await {
                return failed;
            }
        }
        if !stretch.is_empty()
            && let Some(failed) = self.executor(&stretch, &mut outputs).await
        {
            return failed;
        }
        self.result(OUTCOME_SUCCESS, "", "", &outputs)
    }

    /// A stretch of executor steps as a mission of their own; its result when it did not succeed.
    async fn executor(
        &self,
        steps: &[TreeStep],
        outputs: &mut Map<String, Value>,
    ) -> Option<Value> {
        let Some(real) = &self.real else {
            return Some(self.result(OUTCOME_REJECTED, "", "this robot has no executor", outputs));
        };
        if self.ctl.stopped() {
            return Some(self.result(
                OUTCOME_CANCELED,
                "",
                "stopped: before the executor's steps",
                outputs,
            ));
        }
        let elements: Vec<String> = steps.iter().map(|s| fill(&s.xml, outputs)).collect();
        let tree = tree_of(&elements);
        let mut goal = self.goal.clone();
        goal["tree_xml"] = json!(tree);
        goal["tree_sha256"] = json!(sha(&tree));
        goal["max_duration_s"] = json!(0.0);
        let sent = self
            .inner
            .send_goal(&real.execute, EXECUTE_TYPE, goal, CALL_WAIT)
            .await;
        let goal = match sent {
            Ok(g) => g,
            Err(e) => {
                return Some(self.result(
                    OUTCOME_FAILURE,
                    &steps[0].name,
                    &format!("the executor did not take its steps: {e}"),
                    outputs,
                ));
            }
        };
        if let Some(r) = crate::lock(&self.running).as_mut() {
            r.executor = Some(goal.canceller());
            r.step.clear();
        }
        let offset = self.elapsed();
        let cancel = goal.canceller();
        let Goal {
            mut feedback,
            result,
            ..
        } = goal;
        tokio::pin!(result);
        let mut stop = self.ctl.stop.clone();
        let done = loop {
            tokio::select! {
                Some(mut fb) = feedback.recv() => {
                    fb["elapsed_s"] = json!(offset + fb["elapsed_s"].as_f64().unwrap_or(0.0));
                    let _ = self.feedback.send(fb).await;
                }
                r = &mut result => break r,
                changed = stop.changed(), if !*stop.borrow() => {
                    if changed.is_ok() && *stop.borrow() {
                        let _ = cancel().await;
                    }
                }
            }
        };
        while let Ok(Some(mut fb)) =
            tokio::time::timeout(Duration::from_millis(50), feedback.recv()).await
        {
            fb["elapsed_s"] = json!(offset + fb["elapsed_s"].as_f64().unwrap_or(0.0));
            let _ = self.feedback.send(fb).await;
        }
        if let Some(r) = crate::lock(&self.running).as_mut() {
            r.executor = None;
        }
        let ended = match self.ended(done, &steps[0].name, outputs) {
            Ok(ended) => ended,
            Err(mission) => return Some(mission),
        };
        if let Ok(Value::Object(handed)) =
            serde_json::from_str(ended["outputs_json"].as_str().unwrap_or("{}"))
        {
            outputs.extend(handed);
        }
        if ended["outcome"].as_u64() == Some(u64::from(OUTCOME_SUCCESS)) {
            return None;
        }
        let mut failed = ended;
        failed["elapsed_s"] = json!(self.elapsed());
        failed["outputs_json"] = json!(Value::Object(outputs.clone()).to_string());
        Some(failed)
    }

    /// The executor's result for its stretch, or the mission's when it gave none: a goal cancelled
    /// under a stop can come back without the executor's own result.
    fn ended(
        &self,
        done: Result<Result<GoalResult, RosError>, oneshot::error::RecvError>,
        first: &str,
        outputs: &Map<String, Value>,
    ) -> Result<Value, Value> {
        match done {
            Ok(Ok(r)) if r.status == GoalStatus::Canceled && r.result["outcome"].is_null() => {
                let why = crate::lock(&self.running)
                    .as_ref()
                    .map_or_else(String::new, |r| r.stopped_for.clone());
                Err(self.result(OUTCOME_CANCELED, first, &format!("stopped: {why}"), outputs))
            }
            Ok(Ok(r)) => Ok(r.result),
            Ok(Err(e)) => Err(self.result(OUTCOME_FAILURE, first, &e.to_string(), outputs)),
            Err(_) => Err(self.result(
                OUTCOME_FAILURE,
                first,
                "the executor dropped the mission",
                outputs,
            )),
        }
    }

    /// A native step, with its wrappers; its result when the mission ends on it.
    async fn native(
        &self,
        step: &TreeStep,
        native: &Arc<dyn NativeSkill>,
        outputs: &mut Map<String, Value>,
    ) -> Option<Value> {
        let skill = native.skill();
        let output_names: BTreeSet<String> = skill["outputs"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|o| o["name"].as_str().map(str::to_owned))
            .collect();
        let mut args = BTreeMap::new();
        let mut keys = BTreeMap::new();
        for (name, value) in &step.args {
            if output_names.contains(name) {
                keys.insert(name.clone(), value.trim_matches(['{', '}']).to_owned());
                continue;
            }
            let value = match value.strip_prefix('{').and_then(|v| v.strip_suffix('}')) {
                Some(key) => match outputs.get(key) {
                    Some(v) => v.as_str().map_or_else(|| v.to_string(), str::to_owned),
                    None => {
                        return Some(self.result(
                            OUTCOME_FAILURE,
                            &step.name,
                            &format!("`{name}` takes what no earlier step handed on ({key})"),
                            outputs,
                        ));
                    }
                },
                None => value.clone(),
            };
            args.insert(name.clone(), value);
        }
        if let Some(r) = crate::lock(&self.running).as_mut() {
            r.step.clone_from(&step.name);
        }
        let limit = Duration::from_secs_f64(skill["max_duration_s"].as_f64().unwrap_or(60.0));
        let timeout = step.timeout.map_or(limit, |t| t.min(limit));
        let mut last = String::new();
        let mut timed_out = false;
        for attempt in 0..step.attempts.max(1) {
            if self.ctl.stopped() {
                break;
            }
            self.event(&step.name, 0, RUNNING).await;
            let run = tokio::time::timeout(timeout, native.run(&args, &self.ctl));
            tokio::pin!(run);
            let mut tick = tokio::time::interval(TICK);
            tick.tick().await;
            let ran = loop {
                tokio::select! {
                    r = &mut run => break r,
                    _ = tick.tick() => self.still(&step.name).await,
                }
            };
            match ran {
                Ok(Ok(handed)) => {
                    for (port, value) in handed {
                        if let Some(key) = keys.get(&port) {
                            outputs.insert(key.clone(), json!(value));
                        }
                    }
                    self.event(&step.name, RUNNING, SUCCESS).await;
                    return None;
                }
                Ok(Err(why)) => last = why,
                Err(_) => {
                    timed_out = true;
                    last = format!("it ran past its {:.0} s", timeout.as_secs_f64());
                }
            }
            self.event(&step.name, RUNNING, FAILURE).await;
            tracing::info!(step = %step.name, attempt, %last, "a native step failed");
        }
        if self.ctl.stopped() {
            // As an executor words it: "stopped:" and what the stop said.
            let why = crate::lock(&self.running)
                .as_ref()
                .map_or_else(String::new, |r| r.stopped_for.clone());
            let why = format!("stopped: {}", if why.is_empty() { last } else { why });
            return Some(self.result(OUTCOME_CANCELED, &step.name, &why, outputs));
        }
        if step.forced {
            return None;
        }
        let outcome = if timed_out {
            OUTCOME_TIMEOUT
        } else {
            OUTCOME_FAILURE
        };
        Some(self.result(outcome, &step.name, &last, outputs))
    }

    /// Says a step still runs, so the window's timer and the pause state stay current.
    async fn still(&self, step: &str) {
        let _ = self
            .feedback
            .send(json!({
                "elapsed_s": self.elapsed(),
                "running_nodes": [step],
                "events": [],
                "paused": *self.ctl.paused.borrow(),
            }))
            .await;
    }
}

#[async_trait]
impl RobotPort for Composite {
    async fn call(
        &self,
        service: &str,
        service_type: &str,
        request: Value,
        timeout: Duration,
    ) -> Result<Value, RosError> {
        match service {
            CATALOG => self.catalog().await,
            VALIDATE => self.validate(&request).await,
            STOP => self.stop(&request).await,
            PAUSE => self.pause(&request).await,
            PREVIEW => self.preview(&request, service_type, timeout).await,
            _ => {
                self.inner
                    .call(service, service_type, request, timeout)
                    .await
            }
        }
    }

    async fn send_goal(
        &self,
        action: &str,
        action_type: &str,
        goal: Value,
        timeout: Duration,
    ) -> Result<Goal, RosError> {
        if action == EXECUTE {
            return self.execute(goal, timeout).await;
        }
        self.inner
            .send_goal(action, action_type, goal, timeout)
            .await
    }

    async fn latest(&self, topic: &str, msg_type: &str, wait: Duration) -> Result<Value, RosError> {
        if topic == STATE {
            return self.state(wait, Duration::from_hours(1)).await;
        }
        self.inner.latest(topic, msg_type, wait).await
    }

    async fn latest_shared(
        &self,
        topic: &str,
        msg_type: &str,
        wait: Duration,
    ) -> Result<Arc<Value>, RosError> {
        if topic == STATE {
            return self
                .state(wait, Duration::from_hours(1))
                .await
                .map(Arc::new);
        }
        self.inner.latest_shared(topic, msg_type, wait).await
    }

    async fn latest_fresh(
        &self,
        topic: &str,
        msg_type: &str,
        wait: Duration,
        max_age: Duration,
    ) -> Result<Value, RosError> {
        if topic == STATE {
            return self.state(wait, max_age).await;
        }
        self.inner
            .latest_fresh(topic, msg_type, wait, max_age)
            .await
    }

    fn frames(&self, topic: &str) -> Result<watch::Receiver<Option<Arc<Frame>>>, RosError> {
        self.inner.frames(topic)
    }

    fn transform(&self, target: &str, source: &str) -> Result<Transform, RosError> {
        self.inner.transform(target, source)
    }

    async fn graph(&self) -> Result<Graph, RosError> {
        self.inner.graph().await
    }

    async fn service_available(&self, service: &str, service_type: &str, wait: Duration) -> bool {
        match service {
            CATALOG | VALIDATE | STOP | PAUSE => true,
            _ => {
                self.inner
                    .service_available(service, service_type, wait)
                    .await
            }
        }
    }

    async fn graph_detail(&self) -> Result<GraphDetail, RosError> {
        self.inner.graph_detail().await
    }

    async fn endpoints(&self, topic: &str) -> Result<TopicEndpoints, RosError> {
        self.inner.endpoints(topic).await
    }

    async fn node_entities(&self, node: &str) -> Result<NodeEntities, RosError> {
        self.inner.node_entities(node).await
    }

    async fn arrivals(
        &self,
        topic: &str,
        msg_type: &str,
    ) -> Result<BoxStream<'static, usize>, RosError> {
        self.inner.arrivals(topic, msg_type).await
    }

    async fn messages(
        &self,
        topic: &str,
        msg_type: &str,
    ) -> Result<BoxStream<'static, Result<Value, RosError>>, RosError> {
        self.inner.messages(topic, msg_type).await
    }

    async fn publish(
        &self,
        topic: &str,
        msg_type: &str,
        message: Value,
        count: usize,
        period: Duration,
    ) -> Result<usize, RosError> {
        self.inner
            .publish(topic, msg_type, message, count, period)
            .await
    }

    async fn publisher(&self, topic: &str, msg_type: &str) -> Result<Publisher, RosError> {
        self.inner.publisher(topic, msg_type).await
    }

    async fn latched_publisher(&self, topic: &str, msg_type: &str) -> Result<Publisher, RosError> {
        self.inner.latched_publisher(topic, msg_type).await
    }

    fn tf_links(&self) -> Vec<TfLink> {
        self.inner.tf_links()
    }
}

#[cfg(test)]
mod tests;
