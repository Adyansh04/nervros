//! Missions a job runs: checked as the model's plans are, run without asking again because the
//! job's envelope was the approval, and followed to their end.

use std::sync::Arc;
use std::time::Duration;

use serde_json::{Map, Value, json};
use tokio::sync::broadcast::error::RecvError;

use super::execution::Reported;
use super::plan::{Author as By, Compiled};
use super::{Missions, Outcome};
use crate::guard::Autonomy;
use crate::job::Context;
use crate::session::Event;
use crate::tools::Status;

/// How a job's mission ended.
#[derive(Debug, Clone, PartialEq)]
pub struct MissionEnd {
    /// The mission's id.
    pub id: String,
    /// How it ended.
    pub outcome: Outcome,
    /// The step it failed at, or empty.
    pub failed_step: String,
    /// The skill's own words on why.
    pub reason: String,
    /// What its steps handed on, by key (`s2_object_id`), when the executor says.
    pub outputs: Map<String, Value>,
    /// What made the run no test of its plan, when something did: worth no retry.
    pub fault: Option<super::Fault>,
}

/// Why a job's mission did not run, or did not end where the job could hear it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NotRun {
    /// This plan was refused: its checks, the job's approval or the profile's rules stand against
    /// it. Another plan may still run, such as a walk to the next place of a round.
    Refused(String),
    /// The robot cannot run missions now, the job was stopped, or its mission was lost track of.
    Cannot(String),
}

impl std::fmt::Display for NotRun {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Refused(why) | Self::Cannot(why) => f.write_str(why),
        }
    }
}

impl From<NotRun> for String {
    fn from(not: NotRun) -> Self {
        not.to_string()
    }
}

/// What a job hears when it was stopped, or ran out of time, before its mission ended.
const STOPPED: &str = "the job was stopped";
/// How long a stopped job waits for its mission to take the cancel.
const JOB_CANCEL_WAIT: Duration = Duration::from_secs(8);

impl Missions {
    /// Runs `steps` as one mission for the job `ctx` and waits for its end. Every step, the
    /// walks the planner adds included, must be a skill the job's envelope names, and no rule of
    /// the profile may stand against it: a job cannot stop to ask. It waits for a mission that
    /// runs already, and stopping the job cancels its mission.
    ///
    /// # Errors
    ///
    /// The plan failed its checks, a step is outside the envelope, a rule refuses it, the robot
    /// is not armed, the executor did not start it, or the job was stopped.
    pub async fn run_for_job(
        self: &Arc<Self>,
        ctx: &Context,
        intent: &str,
        steps: Value,
    ) -> Result<MissionEnd, NotRun> {
        let checked = self
            .plan(json!({"intent": intent, "steps": steps}), By::Job)
            .await;
        if checked.status != Status::Succeeded {
            return Err(NotRun::Refused(checked.message));
        }
        let hash = checked.data["hash"].as_str().unwrap_or_default();
        let mut compiled = self.find(hash).map_err(NotRun::Cannot)?;
        self.may_run_for(ctx, hash, &compiled)?;
        let session = ctx
            .session()
            .ok_or_else(|| NotRun::Cannot("no session to run the mission in".to_owned()))?
            .clone();
        // A mission the model, a schedule or another job runs ends before this one starts.
        tokio::select! {
            () = ctx.until_stopped() => return Err(NotRun::Cannot(STOPPED.to_owned())),
            () = self.idle() => {}
        }
        if ctx.stopped() {
            return Err(NotRun::Cannot(STOPPED.to_owned()));
        }
        // Subscribed before the launch, so even a mission refused at once is heard ending.
        let mut events = session.subscribe();
        compiled.plan.intent = format!("{intent} (job {})", ctx.id());
        let id = self
            .launch(compiled, Reported::ToJob, None)
            .await
            .map_err(NotRun::Cannot)?;
        // Dropped with a stopped job, or let go here, it cancels the mission if it still runs.
        let mut cancel = CancelOnDrop {
            missions: Arc::clone(self),
            id: id.clone(),
        };
        loop {
            let event = tokio::select! {
                biased;
                () = ctx.until_stopped() => {
                    self.cancel_for_stopped_job(&std::mem::take(&mut cancel.id), ctx).await;
                    return Err(NotRun::Cannot(STOPPED.to_owned()));
                }
                event = events.recv() => event,
            };
            match event {
                Ok(Event::MissionFinished {
                    id: ended,
                    outcome,
                    failed_step,
                    reason,
                    outputs,
                    fault,
                    ..
                }) if ended == id => {
                    return Ok(MissionEnd {
                        id,
                        outcome,
                        failed_step,
                        reason,
                        outputs,
                        fault,
                    });
                }
                // Behind on events, its end may have gone by unheard.
                Err(RecvError::Lagged(_)) if !self.runs(&id) => {
                    return Err(NotRun::Cannot(format!(
                        "lost track of mission {id}; its report says how it ended"
                    )));
                }
                Ok(_) | Err(RecvError::Lagged(_)) => {}
                Err(RecvError::Closed) => {
                    return Err(NotRun::Cannot("the session ended".to_owned()));
                }
            }
        }
    }
}

impl Missions {
    /// Whether job `ctx` may run the plan `hash` compiled to: every step within its approval,
    /// no rule of the profile against it, and the robot armed to act.
    fn may_run_for(&self, ctx: &Context, hash: &str, compiled: &Compiled) -> Result<(), NotRun> {
        if let Some(step) = compiled
            .steps
            .iter()
            .find(|s| !ctx.envelope().allows(&s.skill))
        {
            let added = if step.added {
                " (a walk the planner adds before the next step: approve GoToPlace too)"
            } else {
                ""
            };
            return Err(NotRun::Refused(format!(
                "{} is not among the skills this job was approved for{added}",
                step.skill
            )));
        }
        let guard = self
            .guard
            .get()
            .ok_or_else(|| NotRun::Cannot("the robot is not armed".to_owned()))?;
        if guard.policy().autonomy == Autonomy::Observe {
            return Err(NotRun::Cannot(
                "this profile only observes the robot".to_owned(),
            ));
        }
        if let Some(rule) = self
            .rule_view(hash)
            .and_then(|view| guard.rule("run_mission", &view))
        {
            return Err(NotRun::Refused(format!(
                "the profile's rules refuse it: {}",
                rule.reason
            )));
        }
        if !guard.armed() {
            return Err(NotRun::Cannot("the robot is not armed".to_owned()));
        }
        Ok(())
    }

    /// Cancels a stopped job's mission and waits for the answer: one not taken is noted on the
    /// job, whose report then says the robot may still be moving.
    async fn cancel_for_stopped_job(&self, id: &str, ctx: &Context) {
        let Some(cancel) = self.canceller(id) else {
            return;
        };
        let why = match tokio::time::timeout(JOB_CANCEL_WAIT, cancel()).await {
            Ok(Ok(())) => return,
            Ok(Err(e)) => format!("mission {id} did not take its cancel ({e}) and may still move"),
            Err(_) => format!(
                "mission {id} did not answer its cancel in {} s and may still move",
                JOB_CANCEL_WAIT.as_secs()
            ),
        };
        tracing::warn!(mission = %id, %why, "a stopped job's mission may still run");
        ctx.unsettled(why);
    }
}

/// Cancels a job's mission that still runs when the job lets go of it: only its own mission, so
/// the operator's turn and every other job go on.
struct CancelOnDrop {
    missions: Arc<Missions>,
    id: String,
}

impl Drop for CancelOnDrop {
    fn drop(&mut self) {
        let (Some(cancel), Ok(runtime)) = (
            self.missions.canceller(&self.id),
            tokio::runtime::Handle::try_current(),
        ) else {
            return;
        };
        let id = std::mem::take(&mut self.id);
        runtime.spawn(async move {
            if let Err(e) = cancel().await {
                tracing::warn!(error = %e, mission = %id, "the job's mission was not cancelled");
            }
        });
    }
}
