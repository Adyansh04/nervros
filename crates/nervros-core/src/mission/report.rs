//! What the model reads when a mission ends.

use std::fmt::Write as _;
use std::sync::atomic::Ordering;

use super::check;
use super::check::Observed;
use super::plan::Compiled;
use serde_json::json;

use super::Missions;
use super::Outcome;
use super::SERVICE_TIMEOUT;
use super::held_by;

impl Missions {
    /// What the model reads when a mission ends: how it went, the goal checks or why it failed
    /// and where its object was last seen, and what the hands hold now.
    pub(super) fn report(
        &self,
        id: &str,
        compiled: &Compiled,
        (outcome, step, reason): (Outcome, &str, &str),
        elapsed: f64,
        (seen, story, camera): (&Observed, &[String], &[String]),
    ) -> String {
        let mut report = format!(
            "Mission {id} ({}) ended: {outcome} after {elapsed:.0} s.",
            compiled.plan.intent
        );
        let mut next = "";
        // Failures count per request, not in a row: a detour that works between two blocked
        // walks does not start the count again, and the operator's next message does.
        if outcome == Outcome::Canceled {
            // A stop is the operator's, the deadman's or the robot's own: not a failure to fix.
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
            let _ = write!(report, " It was stopped{at}{why}.");
            next = "Do not run it again unless the operator asks. Tell them where things stand.";
        } else if outcome == Outcome::Success {
            let verdicts = check::check(&compiled.goal, seen);
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
            if !camera.is_empty() {
                let _ = write!(
                    report,
                    " Camera check, before and after: {}.",
                    camera.join("; ")
                );
            }
        } else {
            let n = self.run_failures.fetch_add(1, Ordering::SeqCst) + 1;
            let failed = compiled.steps.iter().find(|s| s.id == step);
            let what = failed.map_or_else(String::new, |s| format!(" {}", s.summary));
            let _ = write!(report, " Failed at {step}{what}: {reason}.");
            if let Some(line) = failed.and_then(|s| check::last_seen(s, seen, crate::now_s())) {
                let _ = write!(report, " The world model: {line}.");
            }
            if !story.is_empty() {
                let _ = write!(report, " What happened to it: {}.", story.join(". "));
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
        if !seen.state.is_null() {
            let hands: Vec<String> = ["left", "right"]
                .iter()
                .filter_map(|h| {
                    let held = held_by(&seen.state, h);
                    (!held.is_empty()).then(|| format!("the {h} hand holds {held}"))
                })
                .collect();
            if !hands.is_empty() {
                let _ = write!(report, " Now {}.", hands.join(" and "));
            }
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
        let Some(service) = self.profile.world.as_ref().and_then(|w| w.history.as_ref()) else {
            return Vec::new();
        };
        let request = json!({"query": query, "since": {"sec": 0, "nanosec": 0}, "max_events": 30});
        match self
            .robot
            .call(
                service,
                "canopy_msgs/srv/ObjectHistory",
                request,
                SERVICE_TIMEOUT,
            )
            .await
        {
            Ok(reply) => check::story(
                reply["events"].as_array().map_or(&[][..], Vec::as_slice),
                crate::now_s(),
            ),
            Err(e) => {
                tracing::info!(error = %e, "no object history");
                Vec::new()
            }
        }
    }
}
