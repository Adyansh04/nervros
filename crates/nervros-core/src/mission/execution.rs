//! Running a mission: launching it on the executor, watching it, and keeping how it went.

use std::sync::Arc;
use std::time::Duration;
use std::time::Instant;

use super::ledger::MissionRecord;
use super::ledger::StepRecord;
use super::plan::Author as By;
use super::plan::Compiled;
use super::plan::PlannedStep;
use super::{camera, check, ledger};
use nervros_ros::Frame;
use nervros_ros::Goal;
use nervros_ros::GoalResult;
use nervros_ros::RosError;
use serde_json::Value;
use serde_json::json;
use tracing::Instrument as _;

use super::Claim;
use super::EXECUTE;
use super::LIVENESS;
use super::LOST_AFTER;
use super::Missions;
use super::Outcome;
use super::Running;
use super::SERVICE_TIMEOUT;
use super::STATE;
use super::STATE_FRESH;
use super::STATUSES;
use crate::lock;
use crate::session::Command;
use crate::session::Event;
use crate::tools::Status;
use crate::tools::ToolOutcome;

impl Missions {
    /// Runs an approved plan, unless a mission runs or this one just failed the same way.
    pub(super) async fn run(self: &Arc<Self>, args: &Value) -> ToolOutcome {
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
        // The operator's own "Run again" is theirs to make; only the model's goes back.
        let by = if args["by"] == "operator" {
            By::Operator
        } else {
            By::Model
        };
        if by == By::Model
            && let Some(refusal) = self.unchanged(&compiled.sha256)
        {
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

    /// Sends a checked plan to the executor and watches it; its id. The slot is claimed before the
    /// goal goes out, so two launches at once cannot both pass.
    async fn launch(self: &Arc<Self>, compiled: Compiled) -> Result<String, String> {
        let id = uuid::Uuid::now_v7().to_string();
        {
            let mut running = lock(&self.running);
            if let Some(r) = running.as_ref() {
                return Err(format!(
                    "mission {} is still running; wait for its report or stop it",
                    r.id
                ));
            }
            *running = Some(Running { id: id.clone() });
        }
        let mut claim = Claim {
            missions: self,
            id: id.clone(),
            kept: false,
        };
        // Held until the mission ends; run_mission itself takes none, as it returns at once.
        let held = match self.guard.get() {
            Some(guard) => Some(
                guard
                    .lock(&crate::tools::Resource::ALL)
                    .map_err(|busy| busy.message)?,
            ),
            None => None,
        };
        let goal = json!({
            "mission_id": id,
            "tree_xml": compiled.xml,
            "tree_sha256": compiled.sha256,
            "max_duration_s": 0.0,
            // A mission that asked for a deadman with no beats coming would be stopped at once.
            "heartbeat_timeout_s": if lock(&self.heartbeat).is_some() {
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
        claim.kept = true;
        self.emit(Event::MissionStarted {
            id: id.clone(),
            hash: compiled.sha256.clone(),
        });
        let before = self.vision.get().and_then(camera::Vision::frame);
        let span = crate::telemetry::job("mission", &id, &compiled.plan.intent);
        tokio::spawn(
            Arc::clone(self)
                .watch(id.clone(), compiled, goal, before, held)
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

    /// Follows mission `id` to its end, then keeps it in the ledger and reports to the model.
    async fn watch(
        self: Arc<Self>,
        id: String,
        compiled: Compiled,
        goal: Goal,
        before: Option<Arc<Frame>>,
        held: Option<crate::guard::ResourceLock>,
    ) {
        let started = Instant::now();
        let started_s = crate::now_s();
        let mut times = StepTimes::default();
        let result = self.outcome_of(&id, goal, &mut times).await;
        let elapsed = started.elapsed().as_secs_f64();
        let (outcome, step, reason) = summarise(result);
        drop(held);
        self.release(&id);
        tracing::Span::current().record("nervros.outcome", outcome.as_str());
        let how = match outcome {
            Outcome::Success => "succeeded".to_owned(),
            Outcome::Canceled => "was stopped".to_owned(),
            _ => format!("failed at {step}: {reason}"),
        };
        *lock(&self.last) = Some((compiled.sha256.clone(), how));
        self.emit(Event::MissionFinished {
            id: id.clone(),
            outcome,
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
            ended: crate::now_s(),
            outcome: outcome.as_str().to_owned(),
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
            .find(|s| s.id == step && outcome != Outcome::Success)
            .and_then(|s| check::step_object(s, &seen))
            .and_then(|o| o["id"].as_str());
        let story = match object {
            Some(id) => self.object_story(id).await,
            None => Vec::new(),
        };
        let camera = match self.vision.get().filter(|_| outcome == Outcome::Success) {
            Some(vision) => {
                let verdicts = check::check(&compiled.goal, &seen);
                vision.check(&verdicts, &seen, before.as_ref()).await
            }
            None => camera::Checked::default(),
        };
        if let Some(image) = &camera.image {
            self.emit(image.into());
        }
        let report = self.report(
            &id,
            &compiled,
            (outcome, &step, &reason),
            elapsed,
            (&seen, &story, &camera.lines),
        );
        if let Some(s) = self.session.get() {
            s.send(Command::Report(report));
        }
    }

    /// How mission `id` ends: its result, or an error once the executor no longer says it runs it.
    /// Its steps' progress goes out as it comes, into `times` too.
    async fn outcome_of(
        &self,
        id: &str,
        goal: Goal,
        times: &mut StepTimes,
    ) -> Result<GoalResult, RosError> {
        let Goal {
            mut feedback,
            result,
            ..
        } = goal;
        tokio::pin!(result);
        let mut alive = tokio::time::interval(LIVENESS);
        alive.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        alive.tick().await;
        let mut unheard = 0;
        let result = loop {
            tokio::select! {
                Some(fb) = feedback.recv() => self.progress(id, &fb, times),
                r = &mut result => {
                    break r
                        .map_err(|_| RosError::Middleware("the goal was dropped".into()))
                        .and_then(|r| r);
                }
                _ = alive.tick() => {
                    unheard = if self.executor_runs(id).await { 0 } else { unheard + 1 };
                    if unheard >= LOST_AFTER {
                        break Err(RosError::Middleware(format!(
                            "the executor has not said it runs this mission for {} s: it may have \
                             restarted, or the link to it is down",
                            LIVENESS.as_secs() * u64::from(LOST_AFTER)
                        )));
                    }
                }
            }
        };
        // The executor's last word on the steps can come just behind its result.
        while let Ok(Some(fb)) =
            tokio::time::timeout(Duration::from_millis(50), feedback.recv()).await
        {
            self.progress(id, &fb, times);
        }
        result
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

    /// Whether the executor says, freshly, that it runs mission `id`.
    async fn executor_runs(&self, id: &str) -> bool {
        self.robot
            .latest_fresh(
                &self.config.state,
                STATE,
                Duration::from_secs(1),
                STATE_FRESH,
            )
            .await
            .is_ok_and(|s| s["mission_id"].as_str() == Some(id))
    }
}

/// `(outcome, failed step, reason)` from the executor's result.
fn summarise(result: Result<GoalResult, RosError>) -> (Outcome, String, String) {
    match result {
        Ok(r) => {
            let code = r.result["outcome"]
                .as_u64()
                .and_then(|c| usize::try_from(c).ok());
            let fallback = match r.status {
                nervros_ros::GoalStatus::Succeeded => Outcome::Success,
                nervros_ros::GoalStatus::Aborted => Outcome::Failure,
                nervros_ros::GoalStatus::Canceled => Outcome::Canceled,
                nervros_ros::GoalStatus::Unknown => Outcome::Error,
            };
            let outcome = code
                .and_then(|c| Outcome::BY_CODE.get(c))
                .copied()
                .unwrap_or(fallback);
            let text = |k: &str| r.result[k].as_str().unwrap_or_default().to_owned();
            // A rejected mission says why in its diagnostics, not in the failure reason.
            let mut reason = text("failure_reason");
            if reason.is_empty() {
                reason = text("diagnostics_json");
            }
            (outcome, text("failed_step_id"), reason)
        }
        Err(e) => (Outcome::Error, String::new(), e.to_string()),
    }
}

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
    /// Notes a step event: its first one starts the step, a success or failure ends it.
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
            // The mission's reason is the failed step's; an optional step that failed before it
            // has none of its own.
            reason: match outcome {
                "failure" if step.id == failed => reason.to_owned(),
                "failure" => "failed; the mission went on".to_owned(),
                _ => String::new(),
            },
        }
    }
}

/// The `s<N>` a node name or path belongs to: the step subtrees are named `s<N>_<Skill>`.
pub(super) fn step_of(text: &str) -> Option<String> {
    text.split(['/', ':'])
        .filter_map(|seg| {
            let rest = seg.strip_prefix('s')?;
            let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
            (!digits.is_empty() && rest[digits.len()..].starts_with('_'))
                .then(|| format!("s{digits}"))
        })
        .next_back()
}
