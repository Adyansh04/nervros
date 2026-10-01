//! Eval cases: what an operator asks and what the agent should do about it. The suite format
//! `nervros-cli eval` runs, and how a real conversation becomes a case.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};
use serde_json::Value;

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
