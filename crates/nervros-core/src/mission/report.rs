//! What the model reads when a mission ends.

use std::fmt::Write as _;
use std::sync::atomic::Ordering;

use serde_json::{Value, json};

use super::check::Observed;
use super::execution::Ended;
use super::plan::Compiled;
use super::plan::PlannedStep;
use super::{Missions, Outcome, SERVICE_TIMEOUT, check, held_by};

/// What was found out about a mission once it ended, for its report.
#[derive(Clone, Copy)]
pub(super) struct Evidence<'a> {
    /// The executor's state and the world model at the end.
    pub(super) seen: &'a Observed,
    /// What happened to the failed step's object.
    pub(super) story: &'a [String],
    /// The robot's earlier missions with it.
    pub(super) moves: &'a [String],
    /// The cameras' check of the goals.
    pub(super) camera: &'a [String],
    /// Steps that failed while the mission went on.
    pub(super) passed_over: &'a [String],
    /// Objects the plan did not name that moved or went missing.
    pub(super) changes: &'a [String],
}

/// The most objects a report names as changed by the way.
const MOST_CHANGES: usize = 5;

/// The goal checks as one line, each unmet one with the step that should have met it; none when
/// the plan names no end state.
fn goal_checks(compiled: &Compiled, seen: &Observed) -> Option<String> {
    let verdicts = check::check(&compiled.goal, seen);
    if verdicts.is_empty() {
        return None;
    }
    let lines: Vec<String> = verdicts
        .iter()
        .map(|v| {
            let mark = match v.ok {
                Some(true) => "holds",
                Some(false) => "DOES NOT hold",
                None => "unchecked",
            };
            let mut line = format!("{} {mark} ({})", v.predicate, v.detail);
            // The executor called every step a success: the cause is the step that should have
            // left this so.
            if v.ok == Some(false)
                && let Some(s) = should_have_left(compiled, &v.predicate)
            {
                let _ = write!(line, ", which {} {} should have left", s.id, s.summary);
            }
            line
        })
        .collect();
    Some(lines.join("; "))
}

/// What each hand holds, when either holds anything.
fn hands_now(seen: &Observed) -> Option<String> {
    if seen.state.is_null() {
        return None;
    }
    let hands: Vec<String> = ["left", "right"]
        .iter()
        .filter_map(|h| {
            let held = held_by(&seen.state, h);
            (!held.is_empty()).then(|| format!("the {h} hand holds {held}"))
        })
        .collect();
    (!hands.is_empty()).then(|| hands.join(" and "))
}

/// The last step that should have left `predicate` holding.
pub(super) fn should_have_left<'c>(
    compiled: &'c Compiled,
    predicate: &str,
) -> Option<&'c PlannedStep> {
    compiled
        .steps
        .iter()
        .rev()
        .find(|s| s.expect.iter().any(|e| e == predicate))
}

impl Missions {
    /// What the model reads when a mission ends: how it went, the goal checks or why it failed,
    /// where its object was last seen and which earlier missions took it, what else changed, and
    /// what the hands hold now.
    pub(super) fn report(
        &self,
        id: &str,
        compiled: &Compiled,
        ended: &Ended,
        evidence: &Evidence<'_>,
    ) -> String {
        let (step, reason) = (ended.step.as_str(), ended.reason.as_str());
        let mut report = format!(
            "Mission {id} ({}) ended: {} after {:.0} s.",
            compiled.plan.intent, ended.outcome, ended.elapsed_s
        );
        let mut next = "";
        let at = compiled
            .steps
            .iter()
            .find(|s| s.id == step)
            .map_or_else(String::new, |s| format!(" during {} {}", s.id, s.summary));
        let why = if reason.is_empty() {
            String::new()
        } else {
            format!(" ({reason})")
        };
        // Failures count per request, not in a row: a detour that works between two blocked
        // walks does not start the count again, and the operator's next message does. A fault
        // is no failure of the plan, so it counts for nothing.
        if let Some(fault) = ended.fault {
            if ended.outcome == Outcome::Rejected {
                let _ = write!(report, " It could not start{why}.");
            } else {
                let _ = write!(report, " It was cut short{at}{why}.");
            }
            next = fault.next();
        } else if ended.outcome == Outcome::Canceled {
            // A stop the operator or the agent asked for: not a failure to fix.
            let _ = write!(report, " It was stopped{at}{why}.");
            next = "Do not run it again unless the operator asks. Tell them where things stand.";
        } else if ended.outcome == Outcome::Success {
            if let Some(checks) = goal_checks(compiled, evidence.seen) {
                let _ = write!(report, " Goal checks: {checks}.");
            }
            if !evidence.camera.is_empty() {
                let _ = write!(report, " Camera check: {}.", evidence.camera.join("; "));
            }
            if !evidence.passed_over.is_empty() {
                let _ = write!(
                    report,
                    " It went on past steps that failed: {}; say what they did not do.",
                    evidence.passed_over.join("; ")
                );
            }
        } else {
            let n = self.run_failures.fetch_add(1, Ordering::SeqCst) + 1;
            let failed = compiled.steps.iter().find(|s| s.id == step);
            let what = failed.map_or_else(String::new, |s| format!(" {}", s.summary));
            let _ = write!(report, " Failed at {step}{what}: {reason}.");
            if let Some(line) =
                failed.and_then(|s| check::last_seen(s, evidence.seen, crate::now_s()))
            {
                let _ = write!(report, " The world model: {line}.");
            }
            if !evidence.story.is_empty() {
                let _ = write!(
                    report,
                    " What happened to it: {}.",
                    evidence.story.join(". ")
                );
            }
            if !evidence.moves.is_empty() {
                let _ = write!(
                    report,
                    " Earlier missions with it: {}.",
                    evidence.moves.join("; ")
                );
            }
            next = if n > self.config.max_replans {
                "This request has failed too often: do not retry. Tell the operator what went \
                 wrong and what would help."
            } else {
                "Find out why (robot_state, look, log_tail) and say it in one sentence. If a \
                 changed plan can work, run it now: the operator approves it. If not, say what is \
                 needed."
            };
        }
        if !evidence.changes.is_empty() {
            let _ = write!(
                report,
                " Elsewhere in the world model meanwhile: {}.",
                evidence.changes.join("; ")
            );
        }
        if let Some(hands) = hands_now(evidence.seen) {
            let _ = write!(report, " Now {hands}.");
        }
        // What the model should do next goes on its own line, after what happened.
        if !next.is_empty() {
            report.push('\n');
            report.push_str(next);
        }
        report
    }

    /// What happened to the objects `query` names, a line each, from the world model's history;
    /// none when the profile names no history service or it does not answer.
    pub(crate) async fn object_story(&self, query: &str) -> Vec<String> {
        check::story(&self.object_events(query, 0.0).await, crate::now_s())
    }

    /// Objects the plan did not name that moved or went missing since `since_s`, a line each and
    /// at most [`MOST_CHANGES`]: what a mission disturbs is in no goal check.
    pub(super) async fn world_changes(&self, compiled: &Compiled, since_s: f64) -> Vec<String> {
        let named: Vec<&str> = compiled
            .steps
            .iter()
            .flat_map(|s| s.args.iter().map(|a| a.value.as_str()))
            .collect();
        // Evidence beside the report's own: a history that does not answer leaves it out.
        self.changes(since_s, &named).await.unwrap_or_default()
    }

    /// Every object that moved or went missing since `since_s`, a line each.
    ///
    /// # Errors
    ///
    /// The world model's history did not say, so what changed is unknown, not nothing.
    pub(crate) async fn changes_since(&self, since_s: f64) -> Result<Vec<String>, String> {
        self.changes(since_s, &[]).await
    }

    /// Objects but `leave_out` that moved or went missing since `since_s`, at most
    /// [`MOST_CHANGES`] lines and the rest counted.
    async fn changes(&self, since_s: f64, leave_out: &[&str]) -> Result<Vec<String>, String> {
        let events: Vec<Value> = self
            .try_object_events("", since_s)
            .await?
            .into_iter()
            .filter(|e| matches!(e["kind"].as_str(), Some("moved" | "missing")))
            .filter(|e| !leave_out.contains(&e["id"].as_str().unwrap_or_default()))
            .collect();
        let mut lines = check::story(&events, crate::now_s());
        if lines.len() > MOST_CHANGES {
            let more = lines.len() - MOST_CHANGES;
            lines.truncate(MOST_CHANGES);
            lines.push(format!("{more} more"));
        }
        Ok(lines)
    }

    /// What happened to the objects `query` names since `since_s` (zero for all that is kept),
    /// oldest first, as the world model's `ObjectEvent`s; none when it keeps no history or does
    /// not answer.
    pub(crate) async fn object_events(&self, query: &str, since_s: f64) -> Vec<Value> {
        self.try_object_events(query, since_s)
            .await
            .unwrap_or_else(|e| {
                tracing::info!(error = %e, "no object history");
                Vec::new()
            })
    }

    /// As [`Self::object_events`], or why the world model did not say.
    ///
    /// # Errors
    ///
    /// The profile names no history service, or it did not answer.
    async fn try_object_events(&self, query: &str, since_s: f64) -> Result<Vec<Value>, String> {
        let service = self
            .profile
            .world
            .as_ref()
            .and_then(|w| w.history.as_ref())
            .ok_or("the world model keeps no history")?;
        // `builtin_interfaces/Time` takes whole seconds as an int32, not a float.
        let sec = i32::try_from(crate::vision::millis(since_s) / 1000).unwrap_or(i32::MAX);
        let request = json!({
            "query": query,
            "since": {"sec": sec, "nanosec": 0},
            "max_events": 30
        });
        let reply = self
            .robot
            .call(
                service,
                "canopy_msgs/srv/ObjectHistory",
                request,
                SERVICE_TIMEOUT,
            )
            .await
            .map_err(|e| e.to_string())?;
        Ok(reply["events"].as_array().cloned().unwrap_or_default())
    }
}
