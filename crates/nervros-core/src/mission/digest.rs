//! The digest: what the robot did over the last hours, in sections code writes from the ledger,
//! the world model's history, the incidents and the goals: missions by how they ended, with the
//! runs that were no test of their plans apart, each skill's record on fair runs, what changed,
//! what went wrong and where the goals stand. The model phrases it for the operator;
//! `nervros-cli digest` prints it as it is.

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::Missions;
use super::goals::Goal;
use super::ledger::MissionRecord;
use crate::incident::{Incident, Investigator};
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// The longest period a digest covers, h.
const MOST_HOURS: f64 = 168.0;
/// The most failures, changes and incidents a section names; the rest are counted.
const MOST_LINES: usize = 5;

/// What a digest is made from.
#[derive(Debug, Default)]
pub struct Period<'a> {
    /// How many hours it covers.
    pub hours: f64,
    /// The missions that started in it, oldest first.
    pub missions: &'a [MissionRecord],
    /// What the world model saw change in it, a line per object; none without the robot.
    pub world: &'a [String],
    /// The incidents opened in it.
    pub incidents: &'a [Incident],
    /// Every goal.
    pub goals: &'a [Goal],
}

/// The digest's text, a section a paragraph.
#[must_use]
pub fn compose(p: &Period<'_>) -> String {
    let mut out = format!("The last {}:", crate::how_long(p.hours * 3600.0));
    let (faulted, fair): (Vec<&MissionRecord>, Vec<&MissionRecord>) =
        p.missions.iter().partition(|m| !m.fault.is_empty());
    let mut by_outcome: BTreeMap<&str, usize> = BTreeMap::new();
    for m in &fair {
        *by_outcome.entry(m.outcome.as_str()).or_default() += 1;
    }
    let counts: Vec<String> = by_outcome
        .iter()
        .map(|(o, n)| format!("{n} {}", ended_as(o)))
        .collect();
    let _ = write!(
        out,
        "\nMissions: {}{}.",
        crate::counted(p.missions.len(), "mission"),
        if counts.is_empty() {
            String::new()
        } else {
            format!(": {}", counts.join(", "))
        }
    );
    if !faulted.is_empty() {
        let mut faults: BTreeMap<&str, usize> = BTreeMap::new();
        for m in &faulted {
            *faults.entry(m.fault.as_str()).or_default() += 1;
        }
        let list: Vec<String> = faults.iter().map(|(f, n)| format!("{f} {n}")).collect();
        let _ = write!(
            out,
            " Cut short by the robot or NervROS, so no test of their plans: {} ({}).",
            faulted.len(),
            list.join(", ")
        );
    }
    let failed: Vec<String> = fair
        .iter()
        .filter(|m| matches!(m.outcome.as_str(), "failure" | "timeout"))
        .map(|m| format!("{} at {}: {}", m.intent, m.failed_step, m.reason))
        .collect();
    push_lines(&mut out, "Failed", &failed);
    push_lines(&mut out, "Skills on fair runs", &skill_records(&fair));
    push_lines(&mut out, "Changed in the world model", p.world);
    let incidents: Vec<String> = p
        .incidents
        .iter()
        .map(|i| format!("{} {}", i.id, i.headline()))
        .collect();
    push_lines(&mut out, "Incidents", &incidents);
    let goals: Vec<String> = p.goals.iter().map(Goal::line).collect();
    push_lines(&mut out, "Goals", &goals);
    out
}

/// How missions with the ledger's `outcome` ended, in a word.
fn ended_as(outcome: &str) -> &str {
    match outcome {
        "success" => "succeeded",
        "failure" => "failed",
        "canceled" => "stopped",
        "timeout" => "timed out",
        "rejected" => "refused",
        "error" => "lost",
        other => other,
    }
}

/// Each skill's successes of its fair runs, as `GoToPlace 14 of 16`, with its last failure.
fn skill_records(fair: &[&MissionRecord]) -> Vec<String> {
    let mut by_skill: BTreeMap<&str, (usize, usize, &str)> = BTreeMap::new();
    for s in fair.iter().flat_map(|m| &m.steps) {
        let entry = by_skill.entry(s.skill.as_str()).or_insert((0, 0, ""));
        match s.outcome.as_str() {
            "success" => {
                entry.0 += 1;
                entry.1 += 1;
            }
            "failure" => {
                entry.1 += 1;
                entry.2 = s.reason.as_str();
            }
            _ => {}
        }
    }
    by_skill
        .into_iter()
        .filter(|(_, (_, runs, _))| *runs > 0)
        .map(|(skill, (ok, runs, last))| {
            let failing = if last.is_empty() {
                String::new()
            } else {
                format!(", last failing: {last}")
            };
            format!("{skill} {ok} of {runs}{failing}")
        })
        .collect()
}

/// `lines` as a section, at most [`MOST_LINES`] of them and the rest counted; nothing when none.
fn push_lines(out: &mut String, title: &str, lines: &[String]) {
    if lines.is_empty() {
        return;
    }
    let _ = write!(out, "\n{title}: ");
    let shown: Vec<&str> = lines.iter().take(MOST_LINES).map(String::as_str).collect();
    out.push_str(&shown.join("; "));
    if lines.len() > MOST_LINES {
        let _ = write!(out, "; and {} more", lines.len() - MOST_LINES);
    }
    out.push('.');
}

/// The `digest` tool.
pub struct DigestTool {
    spec: ToolSpec,
    missions: Arc<Missions>,
    incidents: Arc<Investigator>,
}

impl DigestTool {
    /// The digest of `missions`' ledger and goals, the world model's history and `incidents`.
    #[must_use]
    pub fn new(missions: Arc<Missions>, incidents: Arc<Investigator>) -> Self {
        let spec = ToolSpec::new(
            "digest",
            "What the robot did over the last hours, for when the operator asks how it went: the \
             missions by how they ended, the runs the robot itself cut short apart, each skill's \
             record, what changed in the world model, the incidents and the goals. Phrase it for \
             the operator in a few sentences; do not list every line.",
            json!({"type": "object", "properties": {
                "hours": {"type": "number", "description": "How far back; 24 when left out, at most a week."}
            }, "additionalProperties": false}),
            Risk::Observe,
        );
        Self {
            spec,
            missions,
            incidents,
        }
    }
}

#[async_trait]
impl Tool for DigestTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let hours = args["hours"]
            .as_f64()
            .filter(|h| h.is_finite() && *h > 0.0)
            .unwrap_or(24.0)
            .min(MOST_HOURS);
        let since_s = crate::now_s() - hours * 3600.0;
        let missions = match self.missions.ledger().cloned() {
            Some(ledger) => tokio::task::spawn_blocking(move || ledger.since(since_s))
                .await
                .map_err(|e| e.to_string())
                .and_then(|r| r.map_err(|e| e.to_string())),
            None => Ok(Vec::new()),
        };
        let missions = match missions {
            Ok(m) => m,
            Err(e) => return ToolOutcome::failed(format!("the mission ledger: {e}")),
        };
        // Unknown is not nothing: the section says the history did not answer.
        let world = self
            .missions
            .changes_since(since_s)
            .await
            .unwrap_or_else(|e| vec![format!("unknown: {e}")]);
        let incidents: Vec<Incident> = self
            .incidents
            .incidents()
            .iter()
            .filter(|i| i.opened_s >= since_s)
            .cloned()
            .collect();
        let goals = self.missions.goals().map(|g| g.all()).unwrap_or_default();
        let text = compose(&Period {
            hours,
            missions: &missions,
            world: &world,
            incidents: &incidents,
            goals: &goals,
        });
        let mut out = ToolOutcome::ok(json!({ "digest": text }));
        out.message = format!(
            "{} over {}",
            crate::counted(missions.len(), "mission"),
            crate::how_long(hours * 3600.0)
        );
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::ledger::StepRecord;

    fn mission(outcome: &str, fault: &str, steps: &[(&str, &str, &str)]) -> MissionRecord {
        MissionRecord {
            id: "m".to_owned(),
            intent: "fetch the mug".to_owned(),
            outcome: outcome.to_owned(),
            fault: fault.to_owned(),
            failed_step: "s2".to_owned(),
            reason: "no usable grasp".to_owned(),
            steps: steps
                .iter()
                .map(|(skill, outcome, reason)| StepRecord {
                    skill: (*skill).to_owned(),
                    outcome: (*outcome).to_owned(),
                    reason: (*reason).to_owned(),
                    ..StepRecord::default()
                })
                .collect(),
            ..MissionRecord::default()
        }
    }

    #[test]
    fn faulted_runs_are_counted_apart_and_out_of_the_skills_records() {
        let missions = [
            mission(
                "success",
                "",
                &[("GoToPlace", "success", ""), ("PickObject", "success", "")],
            ),
            mission(
                "failure",
                "",
                &[
                    ("GoToPlace", "success", ""),
                    ("PickObject", "failure", "no usable grasp"),
                ],
            ),
            mission("canceled", "robot", &[("GoToPlace", "failure", "fallen")]),
        ];
        let text = compose(&Period {
            hours: 24.0,
            missions: &missions,
            ..Period::default()
        });
        assert!(
            text.contains("Missions: 3 missions: 1 failed, 1 succeeded."),
            "{text}"
        );
        assert!(
            text.contains("no test of their plans: 1 (robot 1)"),
            "{text}"
        );
        assert!(
            text.contains("Failed: fetch the mug at s2: no usable grasp."),
            "{text}"
        );
        assert!(
            text.contains("GoToPlace 2 of 2; PickObject 1 of 2, last failing: no usable grasp"),
            "the fall counts against no skill: {text}"
        );
        assert!(!text.contains("Incidents"), "no empty sections: {text}");
    }
}
