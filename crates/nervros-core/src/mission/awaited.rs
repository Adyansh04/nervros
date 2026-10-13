//! Missions a job runs: checked as the model's plans are, run without asking again because the
//! job's envelope was the approval, and followed to their end.

use std::sync::Arc;

use serde_json::{Map, Value, json};
use tokio::sync::broadcast::error::RecvError;

use super::execution::Reported;
use super::plan::Author as By;
use super::{Missions, Outcome};
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
}

impl Missions {
    /// Runs `steps` as one mission for the job `ctx` and waits for its end. Every step, the
    /// walks the planner adds included, must be a skill the job's envelope names.
    ///
    /// # Errors
    ///
    /// The plan failed its checks, a step is outside the envelope, the robot is not armed or
    /// runs a mission already, or the executor did not start it.
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
        let compiled = self.find(checked.data["hash"].as_str().unwrap_or_default())?;
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
        if !self.guard.get().is_some_and(|g| g.armed()) {
            return Err("the robot is not armed".to_owned());
        }
        let session = ctx
            .session()
            .ok_or("no session to run the mission in")?
            .clone();
        // Subscribed before the launch, so even a mission refused at once is heard ending.
        let mut events = session.subscribe();
        let mut compiled = compiled;
        compiled.plan.intent = format!("{intent} (job {})", ctx.id());
        let id = self.launch(compiled, Reported::ToJob).await?;
        ctx.running(Some(id.clone()));
        let end = loop {
            match events.recv().await {
                Ok(Event::MissionFinished {
                    id: ended,
                    outcome,
                    failed_step,
                    reason,
                    outputs,
                    ..
                }) if ended == id => {
                    break MissionEnd {
                        id,
                        outcome,
                        failed_step,
                        reason,
                        outputs,
                    };
                }
                // Behind on events, its end may have gone by unheard.
                Err(RecvError::Lagged(_)) if !self.runs(&id) => {
                    ctx.running(None);
                    return Err(format!(
                        "lost track of mission {id}; its report says how it ended"
                    ));
                }
                Ok(_) | Err(RecvError::Lagged(_)) => {}
                Err(RecvError::Closed) => return Err("the session ended".to_owned()),
            }
        };
        ctx.running(None);
        Ok(end)
    }
}
