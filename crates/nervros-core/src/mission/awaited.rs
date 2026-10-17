//! Missions a job runs: checked as the model's plans are, run without asking again because the
//! job's envelope was the approval, and followed to their end.

use std::sync::Arc;
use std::time::Duration;

use serde_json::{Map, Value, json};
use tokio::sync::broadcast::error::RecvError;

use super::execution::Reported;
use super::plan::Author as By;
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

/// What a job hears when it was stopped, or ran out of time, before its mission ended.
const STOPPED: &str = "the job was stopped";
/// How often a job waiting for the mission slot looks again.
const SLOT_POLL: Duration = Duration::from_millis(250);

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
    ) -> Result<MissionEnd, String> {
        let checked = self
            .plan(json!({"intent": intent, "steps": steps}), By::Job)
            .await;
        if checked.status != Status::Succeeded {
            return Err(checked.message);
        }
        let hash = checked.data["hash"].as_str().unwrap_or_default();
        let mut compiled = self.find(hash)?;
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
            return Err(format!(
                "{} is not among the skills this job was approved for{added}",
                step.skill
            ));
        }
        let guard = self.guard.get().ok_or("the robot is not armed")?;
        if guard.policy().autonomy == Autonomy::Observe {
            return Err("this profile only observes the robot".to_owned());
        }
        if let Some(rule) = self
            .rule_view(hash)
            .and_then(|view| guard.rule("run_mission", &view))
        {
            return Err(format!("the profile's rules refuse it: {}", rule.reason));
        }
        if !guard.armed() {
            return Err("the robot is not armed".to_owned());
        }
        let session = ctx
            .session()
            .ok_or("no session to run the mission in")?
            .clone();
        // A mission the model, a schedule or another job runs ends before this one starts.
        while self.busy() {
            tokio::select! {
                () = ctx.until_stopped() => return Err(STOPPED.to_owned()),
                () = tokio::time::sleep(SLOT_POLL) => {}
            }
        }
        if ctx.stopped() {
            return Err(STOPPED.to_owned());
        }
        // Subscribed before the launch, so even a mission refused at once is heard ending.
        let mut events = session.subscribe();
        compiled.plan.intent = format!("{intent} (job {})", ctx.id());
        let id = self.launch(compiled, Reported::ToJob).await?;
        // Dropped with a stopped job, or let go here, it cancels the mission if it still runs.
        let _cancel = CancelOnDrop {
            missions: Arc::clone(self),
            id: id.clone(),
        };
        loop {
            let event = tokio::select! {
                biased;
                () = ctx.until_stopped() => return Err(STOPPED.to_owned()),
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
                    return Err(format!(
                        "lost track of mission {id}; its report says how it ended"
                    ));
                }
                Ok(_) | Err(RecvError::Lagged(_)) => {}
                Err(RecvError::Closed) => return Err("the session ended".to_owned()),
            }
        }
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
