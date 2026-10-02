//! Eval cases: what an operator asks and what the agent should do about it. The suite format
//! `nervros-cli eval` runs, how a conversation is watched and judged against a case, and how a
//! real conversation becomes one.

use std::collections::BTreeSet;
use std::time::Instant;

use serde::{Deserialize, Serialize};
use serde_json::Value;
use tokio::sync::broadcast;

use crate::session::{Command, Event, Session};

/// A suite: cases run in order, on one robot whose state carries from case to case.
#[derive(Debug, Default, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Suite {
    /// Where a simulator publishes the base's true pose.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub truth: Option<Truth>,
    /// The cases.
    #[serde(rename = "case", default)]
    pub cases: Vec<Case>,
}

/// A simulator's `nav_msgs/msg/Odometry` of the base's true pose: moves and turns are judged on
/// it rather than on the robot's own estimate, which can be off by a metre or a quarter turn.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Truth {
    /// Such as `/g1_sensor_relay/base_state`.
    pub topic: String,
}

/// One case.
#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Case {
    /// Unique in its suite.
    pub id: String,
    /// Sent first and not judged, such as walking back to the start.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub setup: Vec<String>,
    /// The operator's messages, in order, in one session.
    pub say: Vec<String>,
    /// How long the case may take.
    #[serde(default = "default_max_s")]
    pub max_s: u64,
    /// What should have happened.
    #[serde(default)]
    pub expect: Expect,
}

fn default_max_s() -> u64 {
    240
}

/// What a case expects.
#[derive(Debug, Clone, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Expect {
    /// Each called at least once, and it succeeded or started.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tools: Vec<String>,
    /// Never called.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub not_tools: Vec<String>,
    /// Each in some tool result's message, case aside, such as what a refusal told the model.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tool_said: Vec<String>,
    /// Each in some checked plan's steps, such as `WalkStraight`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub skills: Vec<String>,
    /// How the last mission ended: `success`, `failure`, or `none` for no mission at all.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mission: Option<String>,
    /// At most this many approvals asked.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub approvals_max: Option<usize>,
    /// The last reply holds one of these, case aside.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub reply_has: Vec<String>,
    /// The last reply holds none of these, case aside.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub reply_lacks: Vec<String>,
    /// How far the robot's base moved on the map, metres, `[min, max]`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub moved_m: Option<[f64; 2]>,
    /// How far it turned, degrees, `[min, max]`: positive to the left (counter-clockwise), so a
    /// turn the wrong way fails.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub turned_deg: Option<[f64; 2]>,
    /// Whether the conversation had to be condensed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compacted: Option<bool>,
}

/// Words a reply should not use to ask for what the approval card asks.
const ASKS_FIRST: [&str; 5] = [
    "would you like me to",
    "shall i ",
    "do you want me to",
    "should i proceed",
    "want me to execute",
];

/// What happened in a case: the tools called and how they ended, the plans, missions, replies
/// and what the model calls cost.
#[derive(Debug, Default, Serialize)]
pub struct Seen {
    /// Each tool call's tool and how it ended: `succeeded`, `failed`, `refused` or `accepted`.
    pub tools: Vec<(String, String)>,
    /// What each tool call's result said.
    pub said: Vec<String>,
    /// Each checked plan's steps, as summarised.
    pub skills: Vec<String>,
    /// How each mission ended.
    pub missions: Vec<String>,
    /// Approvals asked.
    pub approvals: usize,
    /// The model's replies, in order.
    pub replies: Vec<String>,
    /// Errors the session reported.
    pub errors: Vec<String>,
    /// Notices the session gave.
    pub notices: Vec<String>,
    /// Times the conversation was condensed.
    pub compactions: usize,
    /// How far the base moved, metres, by the robot's own estimate of its pose on the map.
    pub moved_m: Option<f64>,
    /// How far it turned, degrees, left positive, by the same estimate.
    pub turned_deg: Option<f64>,
    /// How far it moved by the simulator, when the suite names its truth.
    pub truth_moved_m: Option<f64>,
    /// How far it turned by the simulator.
    pub truth_turned_deg: Option<f64>,
    /// From the first message to the end of the last.
    pub seconds: f64,
    /// The case ran out of time.
    pub timed_out: bool,
    /// Model calls.
    pub calls: u64,
    /// Prompt tokens over every call.
    pub input_tokens: u64,
    /// Of those, read from the provider's prompt cache.
    pub cached_tokens: u64,
    /// Reply tokens over every call.
    pub output_tokens: u64,
    /// Time waiting on the model over every call.
    pub model_ms: u64,
}

/// Sends one message and waits for its turn, the missions it starts and their reports, granting
/// each approval asked when `approve`, else denying it; `true` when `deadline` passed first.
pub async fn say(
    session: &Session,
    events: &mut broadcast::Receiver<Event>,
    text: &str,
    deadline: Instant,
    approve: bool,
    seen: &mut Seen,
) -> bool {
    session.send(Command::User(text.to_owned()));
    let (mut mine, mut open, mut awaiting_report) = (None, 0u32, false);
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        let e = match tokio::time::timeout(left, events.recv()).await {
            Err(_) => return true,
            Ok(Err(broadcast::error::RecvError::Lagged(_))) => continue,
            Ok(Err(broadcast::error::RecvError::Closed)) => return false,
            Ok(Ok(e)) => e,
        };
        match e {
            Event::ApprovalRequested { id, .. } => {
                seen.approvals += 1;
                session.send(if approve {
                    Command::Approve(id)
                } else {
                    Command::Deny(id)
                });
            }
            Event::ToolFinished {
                tool,
                status,
                message,
                ..
            } => {
                seen.tools.push((tool, status.to_string()));
                seen.said.push(message);
            }
            Event::MissionPlanned { steps, .. } => {
                seen.skills.extend(steps.into_iter().map(|s| s.summary));
            }
            Event::MissionStarted { .. } => open += 1,
            Event::MissionFinished { outcome, .. } => {
                seen.missions.push(outcome);
                open = open.saturating_sub(1);
                awaiting_report = true;
            }
            Event::Report { .. } => awaiting_report = false,
            Event::Reply { text, .. } => seen.replies.push(text),
            Event::Error { text, .. } => seen.errors.push(text),
            Event::Notice { text } => seen.notices.push(text),
            Event::Compacted { .. } => seen.compactions += 1,
            Event::ModelCall {
                input_tokens,
                cached_tokens,
                output_tokens,
                ms,
                ..
            } => {
                seen.calls += 1;
                seen.input_tokens += input_tokens;
                seen.cached_tokens += cached_tokens;
                seen.output_tokens += output_tokens;
                seen.model_ms += ms;
            }
            Event::User { turn, text: sent } if sent == text => mine = Some(turn),
            Event::TurnFinished { turn }
                if mine.is_some_and(|m| turn >= m) && open == 0 && !awaiting_report =>
            {
                return false;
            }
            _ => {}
        }
    }
}

/// What `seen` did not do of what `expect` says, a line each; none when the case passed. Travel
/// is judged on the simulator's truth when there is one.
#[must_use]
pub fn judge(expect: &Expect, seen: &Seen) -> Vec<String> {
    let mut problems = Vec::new();
    if seen.timed_out {
        problems.push("timed out".to_owned());
    }
    for e in &seen.errors {
        problems.push(format!("error: {e}"));
    }
    let ok = |t: &str| {
        seen.tools
            .iter()
            .any(|(n, s)| n == t && (s == "succeeded" || s == "accepted"))
    };
    for t in expect.tools.iter().filter(|t| !ok(t)) {
        problems.push(format!("{t} not called successfully"));
    }
    for want in &expect.tool_said {
        let want_lower = want.to_lowercase();
        if !seen
            .said
            .iter()
            .any(|s| s.to_lowercase().contains(&want_lower))
        {
            problems.push(format!("no tool result says \"{want}\""));
        }
    }
    for t in &expect.not_tools {
        if seen.tools.iter().any(|(n, _)| n == t) {
            problems.push(format!("{t} called"));
        }
    }
    for s in &expect.skills {
        if !seen.skills.iter().any(|k| k.starts_with(s.as_str())) {
            problems.push(format!(
                "no {s} in any plan (plans: {})",
                seen.skills.join(", ")
            ));
        }
    }
    match (expect.mission.as_deref(), seen.missions.last()) {
        (Some("none"), Some(m)) => problems.push(format!("a mission ran ({m})")),
        (Some(want), None) if want != "none" => {
            problems.push(format!("no mission ({want} wanted)"));
        }
        (Some(want), Some(got)) if want != "none" && want != got => {
            problems.push(format!("mission {got}, {want} wanted"));
        }
        _ => {}
    }
    if let Some(max) = expect.approvals_max
        && seen.approvals > max
    {
        problems.push(format!(
            "{} approvals, at most {max} wanted",
            seen.approvals
        ));
    }
    if let Some(want) = expect.compacted
        && want != (seen.compactions > 0)
    {
        problems.push(format!("compacted {} times", seen.compactions));
    }
    judge_reply(expect, seen, &mut problems);
    judge_travel(expect, seen, &mut problems);
    problems
}

/// What the last reply says that it should not, or lacks.
fn judge_reply(expect: &Expect, seen: &Seen, problems: &mut Vec<String>) {
    let last = seen
        .replies
        .last()
        .map(|r| r.to_lowercase())
        .unwrap_or_default();
    if !expect.reply_has.is_empty()
        && !expect
            .reply_has
            .iter()
            .any(|w| last.contains(&w.to_lowercase()))
    {
        problems.push(format!("the reply has none of {:?}", expect.reply_has));
    }
    // Offering a follow-up is fine; asking leave for a plan the approval card asks about is not.
    let planned = !seen.skills.is_empty() || !seen.missions.is_empty();
    let asks = ASKS_FIRST.iter().copied().filter(|_| planned);
    for w in expect.reply_lacks.iter().map(String::as_str).chain(asks) {
        if last.contains(&w.to_lowercase()) {
            problems.push(format!("the reply says \"{w}\""));
        }
    }
}

/// How far the base went against what the case wants: on the simulator's truth when there is
/// one, since the robot's own estimate is what may be wrong.
fn judge_travel(expect: &Expect, seen: &Seen, problems: &mut Vec<String>) {
    let within =
        |v: Option<f64>, [lo, hi]: [f64; 2], what: &str, problems: &mut Vec<String>| match v {
            Some(v) if v < lo || v > hi => {
                problems.push(format!("{what} {v:.2}, wanted {lo}..{hi}"));
            }
            None => problems.push(format!("{what} unknown")),
            _ => {}
        };
    if let Some(range) = expect.moved_m {
        within(
            seen.truth_moved_m.or(seen.moved_m),
            range,
            "moved m",
            problems,
        );
    }
    if let Some(range) = expect.turned_deg {
        within(
            seen.truth_turned_deg.or(seen.turned_deg),
            range,
            "turned deg",
            problems,
        );
    }
}

/// A case from a session log (NDJSON, one `{"event": ...}` per line): the operator's messages,
/// the tools that worked, the skills its plans used, how its last mission ended and how many
/// approvals it asked. A starting point to trim: what the agent did is not always what it should.
#[must_use]
pub fn from_log(log: &str, id: &str) -> Case {
    let mut say = Vec::new();
    let mut tools = BTreeSet::new();
    let mut skills = BTreeSet::new();
    let mut mission = None;
    let mut approvals = 0;
    for event in log
        .lines()
        .filter_map(|l| serde_json::from_str::<Value>(l).ok())
        .map(|l| l["event"].clone())
    {
        let text = |k: &str| event[k].as_str().unwrap_or_default().to_owned();
        match event["kind"].as_str().unwrap_or_default() {
            "user" => say.push(text("text")),
            "tool_finished"
                if matches!(event["status"].as_str(), Some("succeeded" | "accepted")) =>
            {
                tools.insert(text("tool"));
            }
            "mission_planned" => skills.extend(
                event["steps"]
                    .as_array()
                    .map_or(&[][..], Vec::as_slice)
                    .iter()
                    .filter_map(|s| s["skill"].as_str())
                    .filter(|s| *s != "GoToPlace")
                    .map(str::to_owned),
            ),
            "mission_finished" => mission = Some(text("outcome")),
            "approval_requested" => approvals += 1,
            _ => {}
        }
    }
    Case {
        id: id.to_owned(),
        setup: Vec::new(),
        say,
        max_s: default_max_s(),
        expect: Expect {
            tools: tools.into_iter().collect(),
            skills: skills.into_iter().collect(),
            mission: Some(mission.unwrap_or_else(|| "none".to_owned())),
            approvals_max: Some(approvals),
            ..Expect::default()
        },
    }
}

/// `case` as a suite file's `[[case]]` table, to append to one.
///
/// # Errors
///
/// It cannot be written as TOML (it always can).
pub fn to_toml(case: &Case) -> Result<String, toml::ser::Error> {
    toml::to_string(&Suite {
        truth: None,
        cases: vec![case.clone()],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_case_is_judged_on_what_happened() {
        let expect: Expect = toml::from_str(
            "tools = [\"run_mission\"]\nskills = [\"WalkStraight\"]\nmission = \"success\"\n\
             approvals_max = 1\nmoved_m = [0.7, 1.3]\n",
        )
        .unwrap();
        let mut seen = Seen {
            tools: vec![("run_mission".into(), "accepted".into())],
            skills: vec!["WalkStraight(direction=forward, distance_m=1)".into()],
            missions: vec!["success".into()],
            approvals: 1,
            replies: vec!["It walked 1 m forward.".into()],
            moved_m: Some(0.95),
            turned_deg: Some(2.0),
            ..Seen::default()
        };
        assert!(
            judge(&expect, &seen).is_empty(),
            "{:?}",
            judge(&expect, &seen)
        );
        seen.replies
            .push("I planned it. Would you like me to run it?".into());
        seen.moved_m = Some(0.0);
        seen.approvals = 2;
        let problems = judge(&expect, &seen).join("; ");
        assert!(
            problems.contains("would you like me to")
                && problems.contains("moved m 0.00")
                && problems.contains("2 approvals"),
            "{problems}"
        );
    }

    #[test]
    fn a_session_becomes_a_case_that_reads_back() {
        let log = [
            r#"{"seq":1,"ts":1.0,"event":{"kind":"user","turn":1,"text":"Turn left 90 degrees."}}"#,
            r#"{"seq":2,"ts":2.0,"event":{"kind":"tool_finished","turn":1,"call":1,"tool":"run_mission","status":"failed","message":"","ms":3}}"#,
            r#"{"seq":3,"ts":3.0,"event":{"kind":"mission_planned","hash":"h","intent":"turn","worst_case_s":30.0,"steps":[{"id":"s1","skill":"TurnInPlace","summary":"","args":[],"timeout_s":30.0}]}}"#,
            r#"{"seq":4,"ts":4.0,"event":{"kind":"approval_requested","id":1,"tool":"run_mission","args":{},"reason":""}}"#,
            r#"{"seq":5,"ts":5.0,"event":{"kind":"tool_finished","turn":1,"call":2,"tool":"run_mission","status":"accepted","message":"","ms":3}}"#,
            r#"{"seq":6,"ts":9.0,"event":{"kind":"mission_finished","id":"m","outcome":"success","failed_step":"","reason":"","elapsed_s":4.0}}"#,
            "not json",
        ]
        .join("\n");

        let case = from_log(&log, "turn-left");

        assert_eq!(case.say, ["Turn left 90 degrees."]);
        assert_eq!(case.expect.tools, ["run_mission"]);
        assert_eq!(case.expect.skills, ["TurnInPlace"]);
        assert_eq!(case.expect.mission.as_deref(), Some("success"));
        assert_eq!(case.expect.approvals_max, Some(1));
        let text = to_toml(&case).unwrap();
        assert!(text.starts_with("[[case]]"), "{text}");
        let back: Suite = toml::from_str(&text).unwrap();
        assert_eq!(back.cases, [case]);
    }
}
