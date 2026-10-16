//! The `why` and `remember_finding` tools.

use std::borrow::Cow;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::Investigator;
use super::playbook::{Kind, Match, Playbook};
use super::report::{Incident, Verdict};
use super::trigger::Trigger;
use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};

/// An incident this recent is the one `why` explains when asked nothing more.
const RECENT_S: f64 = 600.0;
/// Timeline lines `why` returns, the newest.
const TIMELINE: usize = 15;
/// The forms a playbook's check takes, told when one cannot be read.
const CHECK_FORMS: &str = "each check has one test: {topic, type, condition: rate_below, hz}, \
     {topic, type, condition: value, field, op, value}, {topic, type, condition: text, contains}, \
     {log: words, node?, level?, at_least?}, {node_missing: /node}, {node_duplicated: /node}, \
     {tf_missing: [target, source]}, {pose_gap: [target, source], topic, type, field, metres?, \
     degrees?} or {failure_has: words}, and may have `name` and `not: true`";

/// The `why` tool.
pub(crate) struct Why(pub(crate) Arc<Investigator>);

/// The `remember_finding` tool.
pub(crate) struct Remember(pub(crate) Arc<Investigator>);

fn why_spec() -> ToolSpec {
    ToolSpec::new(
        "why",
        "Finds out what went wrong, or what is wrong now, from the robot's own evidence: its \
         last minutes of logs, nodes and missions, checked against what is known to go wrong \
         on it. Alone it explains the newest incident (a failed mission, a camera going quiet, \
         a node stopping) from the last ten minutes, else checks the robot now; `incident` \
         takes an incident's or mission's id, `now` checks now, `list` lists them. Call it \
         before guessing at a failure or retrying one, and tell the operator the cause, what \
         it rests on and the fix; when nothing matched, say so.",
        json!({"type": "object", "properties": {
            "incident": {"type": "string", "description": "An incident's id (i3) or a mission's (m7)"},
            "now": {"type": "boolean", "description": "Check the robot now"},
            "list": {"type": "boolean", "description": "List the incidents"}
        }, "additionalProperties": false}),
        Risk::Observe,
    )
}

fn remember_spec() -> ToolSpec {
    ToolSpec::new(
        "remember_finding",
        "Keeps what a debugging session found as a playbook, so the next incident like it is \
         named at once: `id` (kebab-case), `title`, `verdict`, `fix`, the incidents it \
         `explains` (none for all), `match` (all or any), and its read-only checks, written as \
         the checks a `why` answer shows. Only what the evidence showed, with the operator's \
         agreement; a check it cannot read is answered with the forms a check may take.",
        json!({"type": "object", "properties": {
            "id": {"type": "string"},
            "title": {"type": "string"},
            "verdict": {"type": "string"},
            "fix": {"type": "string"},
            "explains": {"type": "array", "items": {"type": "string",
                "enum": ["mission_failed", "topic_silent", "node_gone", "error_burst", "asked"]}},
            "match": {"type": "string", "enum": ["all", "any"]},
            "procedure": {"type": "string"},
            "check": {"type": "array", "items": {"type": "object"}, "minItems": 1}
        }, "required": ["id", "title", "verdict", "fix", "check"], "additionalProperties": false}),
        Risk::Annotate,
    )
}

fn cause(v: &Verdict) -> Value {
    let mut out = json!({"title": v.title, "verdict": v.verdict, "fix": v.fix,
                         "evidence": v.evidence});
    if !v.procedure.is_empty() {
        out["procedure"] = json!(v.procedure);
    }
    out
}

fn line(v: &Verdict) -> String {
    format!("{}: {}", v.title, v.evidence.join("; "))
}

/// What the model reads of an incident: the causes in full, the rest a line each.
fn told(incident: &Incident, now_s: f64) -> ToolOutcome {
    let skip = incident.timeline.len().saturating_sub(TIMELINE);
    let mut data = json!({
        "incident": incident.id,
        "opened": crate::mission::check::how_long(now_s - incident.opened_s) + " ago",
        "trigger": incident.trigger,
        "likely_causes": incident.causes.iter().map(cause).collect::<Vec<_>>(),
        "ruled_out": incident.ruled_out.iter().map(line).collect::<Vec<_>>(),
        "timeline": incident.timeline[skip..],
    });
    if !incident.unchecked.is_empty() {
        data["could_not_check"] = json!(incident.unchecked.iter().map(line).collect::<Vec<_>>());
    }
    let mut out = ToolOutcome::ok(data);
    out.message = match incident.causes.first() {
        Some(c) => format!("{}: likely {}", incident.id, c.title),
        None => format!(
            "{}: no playbook matched; {} ruled out",
            incident.id,
            incident.ruled_out.len()
        ),
    };
    out
}

#[async_trait]
impl Tool for Why {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Owned(why_spec())
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        self.0.settled().await;
        let now = crate::now_s();
        let incidents = self.0.incidents();
        if args["list"].as_bool() == Some(true) {
            let list: Vec<String> = incidents
                .iter()
                .rev()
                .take(20)
                .map(|i| {
                    format!(
                        "{} ({} ago): {}",
                        i.id,
                        crate::mission::check::how_long(now - i.opened_s),
                        i.headline()
                    )
                })
                .collect();
            let mut out = ToolOutcome::ok(json!({"incidents": list}));
            out.message = format!("{} incidents", list.len());
            return out;
        }
        if let Some(id) = args["incident"]
            .as_str()
            .map(str::trim)
            .filter(|s| !s.is_empty())
        {
            let found = incidents
                .iter()
                .rev()
                .find(|i| i.id.eq_ignore_ascii_case(id) || i.mission.eq_ignore_ascii_case(id));
            return match found {
                Some(i) => told(i, now),
                None => ToolOutcome::failed(format!(
                    "no incident {id}; `list` lists them, and `now` checks the robot as it is"
                )),
            };
        }
        let recent = incidents
            .last()
            .filter(|i| now - i.opened_s <= RECENT_S && args["now"].as_bool() != Some(true));
        if let Some(i) = recent {
            told(i, now)
        } else {
            let asked = Trigger::new(Kind::Asked, "asked what is wrong now");
            told(&self.0.investigate(asked).await, now)
        }
    }
}

impl Remember {
    fn playbook(args: &Value, read_deny: &[String]) -> Result<Playbook, String> {
        let book: Playbook = serde_json::from_value(args.clone())
            .map_err(|e| format!("not a playbook ({e}); {CHECK_FORMS}"))?;
        let loaded = book
            .clone()
            .check("remembered")
            .map_err(|e| format!("{e}; {CHECK_FORMS}"))?;
        loaded
            .tests
            .iter()
            .try_for_each(|t| t.readable(read_deny))?;
        Ok(book)
    }
}

#[async_trait]
impl Tool for Remember {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Owned(remember_spec())
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        Some(
            Self::playbook(args, self.0.read_deny())
                .map_err(ToolOutcome::failed)
                .map(|book| {
                    let when = match (book.checks.len(), book.mode) {
                        (1, _) => "its check holds".to_owned(),
                        (n, Match::All) => format!("its {n} checks hold"),
                        (n, Match::Any) => format!("one of its {n} checks holds"),
                    };
                    Assessment {
                        risk: Risk::Annotate,
                        resources: Vec::new(),
                        reason: format!(
                            "remembers \"{}\": when {when}, {}",
                            book.title, book.verdict
                        ),
                        args: None,
                    }
                }),
        )
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let book = match Self::playbook(&args, self.0.read_deny()) {
            Ok(b) => b,
            Err(e) => return ToolOutcome::failed(e),
        };
        match self.0.remember(&book) {
            Ok(path) => {
                let mut out = ToolOutcome::ok(json!({"kept": path.display().to_string()}));
                "kept; the next incident like it will name it".clone_into(&mut out.message);
                out
            }
            Err(e) => ToolOutcome::failed(e),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::incident::Kind;

    fn incident(id: &str, opened_s: f64, mission: &str) -> Incident {
        Incident {
            id: id.into(),
            opened_s,
            kind: Kind::MissionFailed,
            trigger: format!("mission {mission} failed"),
            mission: mission.into(),
            causes: vec![Verdict {
                playbook: "lost".into(),
                title: "The robot is lost".into(),
                verdict: "v".into(),
                fix: "f".into(),
                procedure: "lost-on-the-map".into(),
                evidence: vec!["map to base_link: 1.2 m off".into()],
            }],
            ruled_out: Vec::new(),
            unchecked: Vec::new(),
            timeline: (0..20).map(|i| format!("{i} s before: line")).collect(),
        }
    }

    #[test]
    fn an_incident_is_told_with_its_causes_in_full_and_the_newest_timeline() {
        let out = told(&incident("i2", 100.0, "m7"), 220.0);
        assert_eq!(out.message, "i2: likely The robot is lost");
        assert_eq!(out.data["opened"], "2 min ago");
        assert_eq!(out.data["likely_causes"][0]["procedure"], "lost-on-the-map");
        assert_eq!(out.data["timeline"].as_array().unwrap().len(), TIMELINE);
        assert_eq!(out.data["timeline"][0], "5 s before: line");
    }

    #[test]
    fn a_remembered_playbook_is_checked_before_it_is_kept() {
        let bad = Remember::playbook(
            &json!({"id": "x", "title": "t", "verdict": "v", "fix": "f",
                    "check": [{"log": "a", "tf_missing": ["m", "b"]}]}),
            &[],
        );
        assert!(bad.unwrap_err().contains("one test"));
        let camera = |condition: Value| {
            let mut check = json!({"topic": "/cam", "type": "sensor_msgs/msg/Image"});
            check
                .as_object_mut()
                .unwrap()
                .extend(condition.as_object().unwrap().clone());
            json!({"id": "camera-quiet", "title": "t", "verdict": "v", "fix": "f",
                   "explains": ["topic_silent"], "check": [check]})
        };
        let rate = camera(json!({"condition": "rate_below", "hz": 2}));
        let good = Remember::playbook(&rate, &[]).unwrap();
        assert_eq!(good.explains, [Kind::TopicSilent]);
        let denied = Remember::playbook(&rate, &["/ca*".to_owned()]).unwrap_err();
        assert!(denied.contains("keeps /cam from being read"), "{denied}");
        let whole = camera(json!({"condition": "value", "field": "width", "op": ">", "value": 0}));
        let large = Remember::playbook(&whole, &[]).unwrap_err();
        assert!(large.contains("too large"), "{large}");
    }
}
