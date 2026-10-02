//! Sending or cancelling action goals, like `ros2 action send_goal`.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::GoalStatus;
use serde_json::{Value, json};

use super::{
    Ctx, MAX_GOAL_WAIT, Prepared, failed, name_arg, object, payload, seconds, spec, summarize,
};
use crate::guard::glob_match;
use crate::tools::{Assessment, Resource, Risk, SchemaPart, Tool, ToolOutcome, ToolSpec};

/// `action_goal`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(ActionGoal {
        spec: spec(
            "action_goal",
            "Send a goal to any ROS action and wait for its result, like ros2 action send_goal, \
             or cancel its goals. A goal still running after wait_s is cancelled. Needs the \
             robot armed and the operator's approval. Moving the robot is for its skills and \
             missions: use this for other actions.",
            object(
                json!({
                    "op": {"type": "string", "enum": ["send", "cancel"]},
                    "action": {"type": "string", "description": "Absolute action name"},
                    "type": {"type": "string", "description": "pkg/action/Name, if known"},
                    "goal": {"type": "object", "description": "The goal fields as JSON"},
                    "wait_s": {"type": "number", "description": "How long to wait for the result, up to 120 (30)"},
                }),
                &["action"],
            ),
            Risk::Motion,
        ),
        ctx,
    })
}

struct ActionGoal {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

/// Cancels a goal if the call that sent it is dropped, as when the operator stops the turn.
struct CancelOnDrop(Option<nervros_ros::CancelFn>);

impl Drop for CancelOnDrop {
    fn drop(&mut self) {
        if let Some(cancel) = self.0.take()
            && let Ok(runtime) = tokio::runtime::Handle::try_current()
        {
            runtime.spawn(async move {
                let _ = cancel().await;
            });
        }
    }
}

impl ActionGoal {
    fn cancelling(args: &Value) -> bool {
        args["op"].as_str() == Some("cancel")
    }

    async fn prepare(&self, args: &Value) -> Result<Prepared, ToolOutcome> {
        let action = name_arg(args, "action").map_err(ToolOutcome::refused)?;
        if !self
            .ctx
            .config
            .action_send
            .iter()
            .any(|p| glob_match(p, &action))
        {
            return Err(ToolOutcome::refused(format!(
                "the profile does not let this agent send goals to `{action}`"
            )));
        }
        let ty = match args["type"].as_str().filter(|t| !t.is_empty()) {
            Some(t) => t.to_owned(),
            None => self.ctx.action_type(&action).await?,
        };
        if let Some(out) = self.ctx.deny_act(&action, &ty) {
            return Err(out);
        }
        let goal = payload(args, "goal");
        if !Self::cancelling(args) {
            self.ctx
                .schemas
                .validate(&ty, SchemaPart::Goal, &goal)
                .map_err(|why| ToolOutcome::failed(format!("the goal does not fit {ty}: {why}")))?;
        }
        Ok(Prepared {
            name: action,
            ty,
            payload: goal,
        })
    }

    async fn cancel_all(&self, action: &str) -> ToolOutcome {
        // A zero goal id and stamp cancel every goal the server has.
        let request = json!({"goal_info": {"goal_id": {"uuid": vec![0_u8; 16]},
                                           "stamp": {"sec": 0, "nanosec": 0}}});
        match self
            .ctx
            .robot
            .call(
                &format!("{action}/_action/cancel_goal"),
                "action_msgs/srv/CancelGoal",
                request,
                Duration::from_secs(5),
            )
            .await
        {
            // action_msgs/srv/CancelGoal: 0 is done, then rejected, unknown goal, terminated.
            Ok(out) => match out["return_code"].as_i64().unwrap_or(0) {
                0 | 3 => ToolOutcome::ok(json!({
                    "action": action,
                    "cancelling": out["goals_canceling"].as_array().map_or(0, Vec::len),
                })),
                1 => ToolOutcome::failed(format!("{action} refused to cancel its goals")),
                2 => ToolOutcome::failed(format!("{action} has no goal to cancel")),
                code => {
                    ToolOutcome::failed(format!("{action} answered the cancel with code {code}"))
                }
            },
            Err(e) => failed(&e),
        }
    }
}

#[async_trait]
impl Tool for ActionGoal {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        Some(self.prepare(args).await.map(|p| Assessment {
            risk: Risk::Motion,
            resources: Resource::ALL.to_vec(),
            args: None,
            reason: if Self::cancelling(args) {
                format!("cancels every goal of {} ({})", p.name, p.ty)
            } else {
                format!("sends {} ({}) the goal {}", p.name, p.ty, p.payload)
            },
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let p = match self.prepare(&args).await {
            Ok(p) => p,
            Err(out) => return out,
        };
        if Self::cancelling(&args) {
            return self.cancel_all(&p.name).await;
        }
        let wait = seconds(&args, "wait_s", 30.0, MAX_GOAL_WAIT);
        let mut handle = match self
            .ctx
            .robot
            .send_goal(&p.name, &p.ty, p.payload, Duration::from_secs(5))
            .await
        {
            Ok(h) => h,
            Err(e) => return failed(&e),
        };
        let mut guard = CancelOnDrop(Some(handle.canceller()));
        let mut feedback = Vec::new();
        let deadline = tokio::time::Instant::now() + wait;
        let result = loop {
            tokio::select! {
                r = &mut handle.result => break Some(r),
                Some(f) = handle.feedback.recv() => {
                    feedback.push(summarize(&f));
                    if feedback.len() > 3 {
                        feedback.remove(0);
                    }
                }
                () = tokio::time::sleep_until(deadline) => break None,
            }
        };
        if result.is_some() {
            guard.0 = None;
        }
        match result {
            Some(Ok(Ok(done))) => {
                let ended = match done.status {
                    GoalStatus::Succeeded => "succeeded",
                    GoalStatus::Aborted => "aborted",
                    GoalStatus::Canceled => "canceled",
                    GoalStatus::Unknown => "ended in an unknown state",
                };
                let data = json!({
                    "action": p.name,
                    "status": ended,
                    "result": summarize(&done.result),
                    "last_feedback": feedback,
                });
                // A goal the server gave up on is not a success, whatever the call did.
                if done.status == GoalStatus::Succeeded {
                    ToolOutcome::ok(data)
                } else {
                    ToolOutcome {
                        data,
                        ..ToolOutcome::failed(format!("the goal {ended}"))
                    }
                }
            }
            Some(Ok(Err(e))) => failed(&e),
            Some(Err(_)) => ToolOutcome::failed("the action server went away before it answered"),
            // Cancelled here and waited for, so the answer says whether it really stopped.
            None => {
                let cancel = guard.0.take();
                let stopped = match cancel {
                    Some(cancel) => tokio::time::timeout(Duration::from_secs(5), cancel()).await,
                    None => Ok(Ok(())),
                };
                let how = match stopped {
                    Ok(Ok(())) => "so the goal was cancelled".to_owned(),
                    Ok(Err(e)) => format!(
                        "and cancelling it failed ({e}): it may still be running; stop the robot \
                         if it moves"
                    ),
                    Err(_) => "and the server did not confirm the cancel: it may still be \
                               running; stop the robot if it moves"
                        .to_owned(),
                };
                ToolOutcome::failed(format!(
                    "no result within {:.0} s, {how}; last feedback: {}",
                    wait.as_secs_f64(),
                    json!(feedback)
                ))
            }
        }
    }
}
