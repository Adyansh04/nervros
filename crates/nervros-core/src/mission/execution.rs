//! Running a mission: launching it on the executor, watching it, and keeping how it went.

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant, SystemTime};

use nervros_ros::{Goal, GoalResult, RosError};
use serde_json::{Value, json};
use tracing::Instrument as _;

use super::check::Observed;
use super::ledger::{MissionRecord, StepRecord};
use super::plan::{Author as By, Compiled, PlannedStep, arg};
use super::report::Evidence;
use super::{
    Claim, EXECUTE, Fault, LIVENESS, LOST_AFTER, Missions, Outcome, Running, SERVICE_TIMEOUT,
    STATE, STATE_FRESH, STATUSES, camera, check, goals, history, inbox, ledger,
};
use crate::lock;
use crate::session::{Command, Event};
use crate::tools::{Status, ToolOutcome};

impl Missions {
    /// Runs an approved plan, unless a mission runs or this one just failed the same way.
    pub(super) async fn run(self: &Arc<Self>, args: &Value) -> ToolOutcome {
        let mut compiled = match self.find(args["hash"].as_str().unwrap_or_default()) {
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
        let goal = match args["goal"].as_u64() {
            None => None,
            Some(n) => match u32::try_from(n)
                .ok()
                .filter(|g| self.goals.get().is_some_and(|goals| goals.active(*g)))
            {
                Some(g) => Some(g),
                None => {
                    return ToolOutcome::refused(format!(
                        "no active goal {n}; list the goals, or run it without `goal`"
                    ));
                }
            },
        };
        let message = if by == By::Operator {
            // An edit in the window keeps the plan's name, and the model told what it had planned:
            // the steps that run name the mission, here and in its report.
            let steps: Vec<String> = compiled
                .steps
                .iter()
                .map(|s| format!("{} {}", s.id, s.summary))
                .collect();
            compiled.plan.intent = format!("the operator's steps: {}", steps.join("; "));
            format!(
                "the operator set these steps themselves, in place of any you planned: {}. The \
                 mission is running; a report follows when it ends. Tell the operator it started, \
                 saying what these steps do, not what was first asked for.",
                steps.join("; ")
            )
        } else {
            "the mission is running; a report follows when it ends. Tell the operator it started."
                .to_owned()
        };
        match self.launch(compiled, Reported::ToModel).await {
            Ok(id) => {
                if let Some(g) = goal {
                    *lock(&self.for_goal) = Some((id.clone(), g));
                }
                ToolOutcome {
                    status: Status::Accepted,
                    data: json!({"mission_id": id}),
                    message,
                    images: Vec::new(),
                }
            }
            Err(e) => ToolOutcome::failed(e),
        }
    }

    /// Sends a checked plan to the executor and watches it; its id. The slot is claimed before the
    /// goal goes out, so two launches at once cannot both pass.
    pub(super) async fn launch(
        self: &Arc<Self>,
        compiled: Compiled,
        reported: Reported,
    ) -> Result<String, String> {
        // Run by its hash or on a schedule, a plan was checked when it was made: a zone may have
        // been added since, and a straight walk starts from where the robot stands now.
        self.still_clear(&compiled).await?;
        let id = uuid::Uuid::now_v7().to_string();
        {
            let mut running = lock(&self.running);
            if let Some(r) = running.as_ref() {
                return Err(format!(
                    "mission {} is still running; wait for its report or stop it",
                    r.id
                ));
            }
            *running = Some(Running {
                id: id.clone(),
                cancel: None,
            });
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
        if let Some(r) = lock(&self.running).as_mut().filter(|r| r.id == id) {
            r.cancel = Some(goal.canceller());
        }
        self.emit(Event::MissionStarted {
            id: id.clone(),
            hash: compiled.sha256.clone(),
        });
        let before = self
            .vision
            .get()
            .map(camera::Vision::frames)
            .unwrap_or_default();
        let span = crate::telemetry::job("mission", &id, &compiled.plan.intent);
        // What the operator asked for this mission: they may say more while it runs.
        let request = self.request();
        let started_s = crate::now_s();
        // Kept as running before it is watched, so a NervROS that stops now leaves a record the
        // next one settles, and the end's record cannot be overtaken by this one.
        let running = MissionRecord {
            id: id.clone(),
            hash: compiled.sha256.clone(),
            intent: compiled.plan.intent.clone(),
            request: request.clone(),
            started: started_s,
            ended: started_s,
            outcome: "running".to_owned(),
            plan: json!(compiled.plan.steps),
            ..MissionRecord::default()
        };
        if let Some(write) = self.keep(running) {
            let _ = write.await;
        }
        tokio::spawn(
            Arc::clone(self)
                .watch(
                    id.clone(),
                    compiled,
                    goal,
                    (before, held),
                    (request, reported),
                    started_s,
                )
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
        self.launch(compiled, Reported::ToModel).await
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
        (before, held): (camera::Frames, Option<crate::guard::ResourceLock>),
        (request, reported): (String, Reported),
        started_s: f64,
    ) {
        let started = Instant::now();
        let mut times = StepTimes::default();
        let result = self.outcome_of(&id, goal, &mut times).await;
        let outputs = outputs_of(&result);
        let ended = summarise(result, started.elapsed().as_secs_f64());
        drop(held);
        self.release(&id);
        tracing::Span::current().record("nervros.outcome", ended.outcome.as_str());
        // The guard against rerunning a plan unchanged is the model's: a job's look that saw
        // nothing does not stand against the model's own plan.
        if reported == Reported::ToModel {
            *lock(&self.last) = Some((compiled.sha256.clone(), ended.how()));
        }
        self.emit(Event::MissionFinished {
            id: id.clone(),
            outcome: ended.outcome,
            failed_step: ended.step.clone(),
            reason: ended.reason.clone(),
            elapsed_s: ended.elapsed_s,
            outputs,
            by_job: reported == Reported::ToJob,
            fault: ended.fault,
        });
        let cut = ended.outcome == Outcome::Canceled;
        let steps: Vec<StepRecord> = compiled
            .steps
            .iter()
            .map(|s| times.record(s, &ended.step, &ended.reason, cut))
            .collect();
        let passed_over = passed_over(&compiled, &steps, &ended.step);
        let kept = self.keep(MissionRecord {
            id: id.clone(),
            hash: compiled.sha256.clone(),
            intent: compiled.plan.intent.clone(),
            request,
            started: started_s,
            ended: crate::now_s(),
            outcome: ended.outcome.as_str().to_owned(),
            failed_step: ended.step.clone(),
            reason: ended.reason.clone(),
            steps,
            plan: json!(compiled.plan.steps),
            fault: ended
                .fault
                .map_or_else(String::new, |f| f.as_str().to_owned()),
        });
        // A taught skill is judged on its runs, this one included.
        if let Some(kept) = kept {
            let _ = kept.await;
        }
        if ended.fault.is_none() {
            self.settle_taught(&compiled, ended.outcome, &ended.step)
                .await;
        }
        // A job hears its mission end from the event; the report, and the failures it counts
        // against the model's request, are the model's.
        if reported == Reported::ToModel {
            let evidence = (before, passed_over, started_s);
            self.tell_model(&id, &compiled, &ended, evidence).await;
        }
    }

    /// Finds out what the model needs to know of a mission that ended, and sends it the report.
    async fn tell_model(
        self: &Arc<Self>,
        id: &str,
        compiled: &Compiled,
        ended: &Ended,
        (before, passed_over, started_s): (camera::Frames, Vec<String>, f64),
    ) {
        let seen = self.observe().await;
        // What happened to what the failed step was about: often it was moved, not missed. The
        // robot's own missions with it say so where the world model cannot tell which it is.
        let failed = compiled
            .steps
            .iter()
            .find(|s| s.id == ended.step && ended.outcome != Outcome::Success);
        let object = failed
            .and_then(|s| check::step_object(s, &seen))
            .and_then(|o| o["id"].as_str());
        let story = async {
            match object {
                Some(id) => self.object_story(id).await,
                None => Vec::new(),
            }
        };
        let moves = async {
            match failed.and_then(|s| arg(&s.args, "object_id")) {
                Some(id) => history::moves_of(self, id).await,
                None => Vec::new(),
            }
        };
        let (story, moves, changes) =
            tokio::join!(story, moves, self.world_changes(compiled, started_s));
        let camera = match self
            .vision
            .get()
            .filter(|_| ended.outcome == Outcome::Success)
        {
            Some(vision) => {
                let verdicts = check::check(&compiled.goal, &seen);
                vision.check(&verdicts, &seen, &before).await
            }
            None => camera::Checked::default(),
        };
        if let Some(image) = &camera.image {
            self.emit(image.into());
        }
        let evidence = Evidence {
            seen: &seen,
            story: &story,
            moves: &moves,
            camera: &camera.lines,
            passed_over: &passed_over,
            changes: &changes,
        };
        let mut report = self.report(id, compiled, ended, &evidence);
        report.push_str(&self.settle_goals(id, ended, &seen));
        if let Some(s) = self.session.get() {
            s.send(Command::Report(report));
        }
        self.propose_fix(id, compiled, ended, &seen);
    }

    /// After a step failed again on a fair run, asks the inbox for a fix, off the report's way:
    /// one failure may be bad luck, and a fault is no failure of the step.
    fn propose_fix(
        self: &Arc<Self>,
        id: &str,
        compiled: &Compiled,
        ended: &Ended,
        seen: &Observed,
    ) {
        let (Some(inbox), Some(ledger)) = (self.inbox.get().cloned(), self.ledger.get().cloned())
        else {
            return;
        };
        if ended.fault.is_some() || !matches!(ended.outcome, Outcome::Failure | Outcome::Timeout) {
            return;
        }
        let Some(step) = compiled.steps.iter().find(|s| s.id == ended.step) else {
            return;
        };
        let target = ledger::target_of(&step.args);
        if !inbox.wanted(&step.skill, &target) {
            return;
        }
        let failure = inbox::Failure {
            mission: id.to_owned(),
            intent: compiled.plan.intent.clone(),
            skill: step.skill.clone(),
            target,
            step: format!("{} {}", step.id, step.summary),
            reason: ended.reason.clone(),
            track: ledger::Track {
                runs: 0,
                succeeded: 0,
                typical_s: None,
                last_failure: None,
            },
            world: check::last_seen(step, seen, crate::now_s()),
        };
        let me = Arc::clone(self);
        tokio::spawn(async move {
            let key = (failure.skill.clone(), failure.target.clone());
            let track = tokio::task::spawn_blocking(move || ledger.track(&key.0, &key.1))
                .await
                .ok()
                .and_then(Result::ok)
                .flatten();
            // Twice, and a third of its runs at least: a pattern, not bad luck.
            let failing = |t: &ledger::Track| t.runs - t.succeeded;
            let Some(track) = track.filter(|t| failing(t) >= 2 && failing(t) * 3 >= t.runs) else {
                return;
            };
            let Some(proposal) = inbox.propose(&inbox::Failure { track, ..failure }).await else {
                return;
            };
            let text = format!(
                "the inbox has a proposed fix, for the operator to accept or dismiss: {}",
                proposal.line()
            );
            if let Some(s) = me.session.get() {
                s.send(Command::Note(text.clone()));
            }
            me.emit(Event::Notice { text });
        });
    }

    /// Counts mission `id` for the goal it worked toward, judges every goal, and says where the
    /// open ones and the ones that just ended stand, as the report's closing lines.
    fn settle_goals(&self, id: &str, ended: &Ended, seen: &Observed) -> String {
        let Some(goals) = self.goals.get() else {
            return String::new();
        };
        let ran = lock(&self.for_goal).take_if(|(m, _)| m == id);
        if let Some((_, goal)) = ran {
            goals.ran(goal, id, ended.fault.is_none());
        }
        // After a fault, or once the request failed too often, the report says not to run again:
        // its goals wait for the operator too.
        let may_go_on = ended.fault.is_none()
            && self.run_failures.load(Ordering::SeqCst) <= self.config.max_replans;
        goals::report_lines(&goals.settle(seen, SystemTime::now()), may_go_on)
    }

    /// Judges the goals as the robot stands at start, and tells the operator of the open ones and
    /// of any that ended while NervROS was not running.
    pub(super) async fn settle_goals_at_start(&self) {
        let Some(goals) = self.goals.get() else {
            return;
        };
        if goals.all().is_empty() {
            return;
        }
        let seen = self.observe().await;
        let lines = goals::report_lines(&goals.settle(&seen, SystemTime::now()), false);
        if !lines.is_empty() {
            self.emit(Event::Notice {
                text: lines.trim().to_owned(),
            });
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
        let mut paused = false;
        // The executor's run as it ran the mission: a new one says it restarted, and the mission
        // with it.
        let mut boot = None;
        let result = loop {
            tokio::select! {
                Some(fb) = feedback.recv() => {
                    if fb["paused"].as_bool().unwrap_or(false) != paused {
                        paused = !paused;
                        times.pause(paused, fb["elapsed_s"].as_f64().unwrap_or(0.0));
                        self.paused.store(paused, Ordering::SeqCst);
                        self.emit(Event::MissionPaused { id: id.to_owned(), paused });
                    }
                    self.progress(id, &fb, times);
                }
                r = &mut result => {
                    break r
                        .map_err(|_| RosError::Middleware("the goal was dropped".into()))
                        .and_then(|r| r);
                }
                _ = alive.tick() => {
                    let state = self.executor_state().await;
                    let runs = state.as_ref().is_some_and(|s| s["mission_id"].as_str() == Some(id));
                    let now = state.as_ref().and_then(boot_of);
                    if boot.as_ref().zip(now.as_ref()).is_some_and(|(was, is)| was != is) {
                        break Err(RosError::Middleware(
                            "the executor restarted, and the mission with it".into(),
                        ));
                    }
                    if runs {
                        boot = boot.or(now);
                    }
                    unheard = if runs { 0 } else { unheard + 1 };
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
        self.paused.store(false, Ordering::SeqCst);
        // The executor's last word on the steps can come just behind its result.
        while let Ok(Some(fb)) =
            tokio::time::timeout(Duration::from_millis(50), feedback.recv()).await
        {
            self.progress(id, &fb, times);
        }
        result
    }

    /// Writes a finished mission to the ledger, off the async threads; the write, to wait for.
    fn keep(&self, record: MissionRecord) -> Option<tokio::task::JoinHandle<()>> {
        let ledger = self.ledger.get().cloned()?;
        Some(tokio::task::spawn_blocking(move || {
            if let Err(e) = ledger.record(&record) {
                tracing::warn!(error = %e, mission = %record.id, "the mission was not recorded");
            }
        }))
    }

    /// Settles the missions an earlier NervROS left running: the executor's deadman stopped each,
    /// so they are kept as interrupted, and none is followed again. One the executor still runs,
    /// as with no deadman, is left running; `say_orphan` tells the operator of it.
    pub(super) async fn settle_interrupted(&self) {
        let Some(ledger) = self.ledger.get().cloned() else {
            return;
        };
        let still = self.executor_state().await.and_then(|s| {
            s["mission_id"]
                .as_str()
                .filter(|m| !m.is_empty())
                .map(str::to_owned)
        });
        let marked =
            match tokio::task::spawn_blocking(move || ledger.interrupted(still.as_deref())).await {
                Ok(Ok(marked)) => marked,
                Ok(Err(e)) => {
                    tracing::warn!(error = %e, "the missions left running were not settled");
                    return;
                }
                Err(_) => return,
            };
        for m in marked {
            let text = format!(
                "the mission \"{}\" ({}) was cut short when NervROS stopped",
                m.intent, m.id
            );
            // The model hears it with the operator's next words, or it answers from nothing.
            if let Some(s) = self.session.get() {
                s.send(Command::Note(text.clone()));
            }
            self.emit(Event::Notice { text });
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

    /// The executor's state, if it sent one lately.
    pub(super) async fn executor_state(&self) -> Option<Value> {
        self.robot
            .latest_fresh(
                &self.config.state,
                STATE,
                Duration::from_secs(1),
                STATE_FRESH,
            )
            .await
            .ok()
    }
}

/// The executor run a state comes from; none from an executor that does not say.
fn boot_of(state: &Value) -> Option<String> {
    state["boot_id"]
        .as_str()
        .filter(|b| !b.is_empty())
        .map(str::to_owned)
}

/// Who hears how a mission ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Reported {
    /// The model, which answers the report.
    ToModel,
    /// The job that ran it, which reports once it ends: a model answering each of a search's
    /// missions would spend its calls on what the job's own report says.
    ToJob,
}

/// What a mission's steps handed on, from the executor's `outputs_json`; none from an executor
/// that does not send it.
fn outputs_of(result: &Result<GoalResult, RosError>) -> serde_json::Map<String, Value> {
    result
        .as_ref()
        .ok()
        .and_then(|r| r.result["outputs_json"].as_str())
        .and_then(|text| serde_json::from_str::<Value>(text).ok())
        .and_then(|v| match v {
            Value::Object(map) => Some(map),
            _ => None,
        })
        .unwrap_or_default()
}

/// Steps that failed while the mission went on, such as a look that saw nothing: its success
/// alone would hide what they did not do.
fn passed_over(compiled: &Compiled, steps: &[StepRecord], failed: &str) -> Vec<String> {
    compiled
        .steps
        .iter()
        .zip(steps)
        .filter(|(s, r)| r.outcome == "failure" && s.id != failed)
        .map(|(s, _)| format!("{} {}", s.id, s.summary))
        .collect()
}

/// How a mission ended, from the executor's result.
pub(super) struct Ended {
    pub(super) outcome: Outcome,
    /// The step it failed or stopped at, or empty.
    pub(super) step: String,
    /// Why, in the executor's words.
    pub(super) reason: String,
    pub(super) fault: Option<Fault>,
    /// How long it ran.
    pub(super) elapsed_s: f64,
}

impl Ended {
    /// How it went, in a few words, for the guard against an unchanged rerun.
    fn how(&self) -> String {
        match (self.outcome, self.fault) {
            (_, Some(_)) => format!("was cut short: {}", self.reason),
            (Outcome::Success, None) => "succeeded".to_owned(),
            (Outcome::Canceled, None) => "was stopped".to_owned(),
            _ => format!("failed at {}: {}", self.step, self.reason),
        }
    }
}

fn summarise(result: Result<GoalResult, RosError>, elapsed_s: f64) -> Ended {
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
            Ended {
                outcome,
                step: text("failed_step_id"),
                reason,
                fault: r.result["cause"].as_u64().and_then(Fault::of_cause),
                elapsed_s,
            }
        }
        Err(e) => Ended {
            outcome: Outcome::Error,
            step: String::new(),
            reason: e.to_string(),
            fault: Some(Fault::ExecutorLost),
            elapsed_s,
        },
    }
}

/// When a step began, in seconds into the mission, and when and how it ended.
#[derive(Debug, Clone, Copy)]
struct StepTime {
    start: f64,
    end: Option<(f64, &'static str)>,
}

/// Each step's [`StepTime`], from the executor's events, for the ledger; time spent paused is
/// no step's.
#[derive(Debug, Default)]
struct StepTimes {
    steps: std::collections::BTreeMap<String, StepTime>,
    /// Seconds paused so far, and since when it is paused now.
    paused: f64,
    paused_at: Option<f64>,
}

impl StepTimes {
    /// Notes that the mission paused, or went on, `elapsed` seconds in.
    fn pause(&mut self, on: bool, elapsed: f64) {
        match (on, self.paused_at.take()) {
            (true, at) => self.paused_at = at.or(Some(elapsed)),
            (false, Some(at)) => self.paused += (elapsed - at).max(0.0),
            (false, None) => {}
        }
    }

    /// Notes a step event: its first one starts the step, a success or failure ends it.
    fn note(&mut self, step: &str, status: &'static str, elapsed: f64) {
        let elapsed = elapsed - self.paused;
        let time = self.steps.entry(step.to_owned()).or_insert(StepTime {
            start: elapsed,
            end: None,
        });
        if matches!(status, "success" | "failure") {
            time.end = Some((elapsed, status));
        }
    }

    /// A planned step as it went; `failed` is the step the mission failed at, with `reason`, or
    /// with `cut` the one a stop ended.
    fn record(&self, step: &PlannedStep, failed: &str, reason: &str, cut: bool) -> StepRecord {
        let (seconds, outcome) = match self.steps.get(&step.id) {
            Some(&StepTime {
                start,
                end: Some((end, status)),
            }) => (Some(end - start), status),
            _ if step.id == failed && cut => (None, "stopped"),
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
                "failure" | "stopped" if step.id == failed => reason.to_owned(),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_spent_paused_is_no_steps() {
        let mut times = StepTimes::default();
        times.note("s1", "running", 1.0);
        times.pause(true, 3.0);
        times.pause(false, 303.0);
        times.note("s1", "success", 304.0);
        let s1 = &times.steps["s1"];
        let end = s1.end.map_or(f64::NAN, |(at, _)| at);
        assert!((end - s1.start - 3.0).abs() < 1e-9, "{s1:?}");
    }
}
