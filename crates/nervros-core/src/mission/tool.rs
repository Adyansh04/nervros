//! The `run_mission` tool.

use std::borrow::Cow;
use std::fmt::Write as _;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::plan::Author as By;
use super::sanity::Concern;
use super::{Missions, SERVICE_TIMEOUT, plan, sanity};
use crate::lock;
use crate::tools::{Assessment, Risk, Status, Tool, ToolOutcome, ToolSpec};

impl Missions {
    /// `run_mission`'s spec, its skill list read off the catalog as it is now.
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
}

/// `run_mission`.
pub(super) struct RunMission(pub(super) Arc<Missions>);

#[async_trait]
impl Tool for RunMission {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Owned(self.0.run_spec())
    }

    /// A plan is checked before anyone is asked: its problems go back to the model, and a sound
    /// one is shown, approved and run by its hash.
    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        if args.get("steps").is_none() && args.get("template").is_none() {
            return self.by_hash(args);
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

    fn rule_view(&self, args: &Value) -> Option<Value> {
        self.0.rule_view(args["hash"].as_str()?)
    }

    /// A checked plan's hash dies with the session that checked it; its steps do not.
    fn ask_again(&self, args: &Value) -> Option<Value> {
        let plan = self.0.find(args["hash"].as_str()?).ok()?.plan;
        serde_json::to_value(plan).ok()
    }

    /// The skill list loads when the session starts; a first message sent at once would see none.
    async fn ready(&self) {
        self.0.catalog_ready().await;
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
    /// A run of a plan checked before, by its hash: refused before anyone is asked when there is
    /// no such plan, when it just ran unchanged, or when the operator asked for it again and
    /// again, which is a schedule's to do.
    fn by_hash(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let refused = |why: String| Some(Err(ToolOutcome::refused(why)));
        if args["check_only"].as_bool() == Some(true) {
            return refused(
                "a plan with a hash is checked already; give check_only the steps instead".into(),
            );
        }
        let Some(hash) = args["hash"].as_str() else {
            return refused(
                "give run_mission the plan's steps, or the hash of a plan checked in this session"
                    .into(),
            );
        };
        let compiled = match self.0.find(hash) {
            Ok(c) => c,
            Err(e) => return refused(e),
        };
        if let Some(refusal) = self.0.unchanged(&compiled.sha256) {
            return Some(Err(refusal));
        }
        if let Some(r) = sanity::repeat(&lock(&self.0.request)) {
            return refused(format!(
                "the operator asked for it every {} min: give the steps to schedule, which runs \
                 it now and then on time",
                r.every_min
            ));
        }
        let minutes = (compiled.worst_case_s / 60.0).ceil();
        Some(Ok(Assessment {
            risk: Risk::Manipulation,
            resources: Vec::new(),
            reason: format!(
                "runs \"{}\": {} step(s), at most {minutes} min",
                compiled.plan.intent,
                compiled.steps.len()
            ),
            args: Some(json!({"hash": compiled.sha256})),
        }))
    }

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
        // Marked as the operator's only here: a model's call by hash has its arguments replaced.
        let args = match by {
            By::Operator => json!({"hash": hash, "by": "operator"}),
            By::Model => json!({"hash": hash}),
        };
        Ok(Assessment {
            risk: Risk::Manipulation,
            resources: Vec::new(),
            reason,
            args: Some(args),
        })
    }
}

/// A concern as one line, its step first.
pub(super) fn concern_line(c: &Concern) -> String {
    if c.step.is_empty() {
        c.message.clone()
    } else {
        format!("{}: {}", c.step, c.message)
    }
}

/// The model's plan back to it once, for words it may not match; not a failed attempt.
pub(super) fn questioned(concerns: &[Concern]) -> ToolOutcome {
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
