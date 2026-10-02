//! The scenarios in `scenarios/` at the repository's root: a scripted model against a scripted
//! robot, through the whole agent as the profile wires it, judged as `nervros-cli eval` judges a
//! case. No model and no ROS, so CI runs them with the other tests.
//!
//! A scenario is a TOML file:
//!
//! ```toml
//! say = ["Turn left 90 degrees."]   # the operator's messages, in order
//! approve = true                     # grant every approval asked (the default), or deny each
//! armed = true                       # arm before the first message (the default)
//!
//! [[model]]                          # the model's turns, in order: a tool call...
//! call = "run_mission"
//! args = { intent = "turn left", steps = [] }
//! [[model]]                          # ...or a reply
//! reply = "Turned left."
//!
//! [robot.actions."/x/execute_mission"]   # over `robot/robot.toml`, key by key
//! status = "aborted"
//!
//! [expect]                           # as an eval case's
//! mission = "failure"
//! ```
//!
//! A scenario passes when it meets its expectations and the model's script is used up exactly.
//! `NERVROS_SCENARIO=<text>` runs only the scenarios whose file names hold it.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use nervros_core::evalcase::{Expect, Seen, judge, say};
use nervros_core::llm::{AgentBuilder, AgentSource, LlmError};
use nervros_core::providers::Role;
use nervros_core::providers::router::Need;
use nervros_core::session::Command;
use nervros_ros::fake::{FakeRobot, ScriptedRun};
use nervros_ros::{GoalResult, GoalStatus, RobotPort, Transform};
use rig::test_utils::{MockCompletionModel, MockTurn};
use serde::Deserialize;
use serde_json::{Value, json};

/// A scenario that has not ended by then is stuck: everything in it is scripted and quick.
const DEADLINE: Duration = Duration::from_secs(20);

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    say: Vec<String>,
    #[serde(default = "yes")]
    approve: bool,
    #[serde(default = "yes")]
    armed: bool,
    #[serde(default)]
    model: Vec<Turn>,
    #[serde(default)]
    robot: Robot,
    #[serde(default)]
    expect: Expect,
}

fn yes() -> bool {
    true
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Turn {
    call: Option<String>,
    args: Option<Value>,
    reply: Option<String>,
}

/// Services by name with their response, actions with how they run, topics with their message.
#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
struct Robot {
    #[serde(default)]
    services: BTreeMap<String, Value>,
    #[serde(default)]
    actions: BTreeMap<String, Run>,
    #[serde(default)]
    topics: BTreeMap<String, Value>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Run {
    #[serde(default)]
    feedback: Vec<Value>,
    /// `succeeded`, `aborted` or `canceled`.
    status: String,
    #[serde(default)]
    result: Value,
}

impl Robot {
    /// This robot with `other`'s services, actions and topics in place of its own.
    fn with(mut self, other: Self) -> Self {
        self.services.extend(other.services);
        self.actions.extend(other.actions);
        self.topics.extend(other.topics);
        self
    }

    fn build(self) -> Result<FakeRobot, String> {
        let mut robot =
            FakeRobot::new().with_transform("map", "base_footprint", Transform::IDENTITY);
        for (name, response) in self.services {
            robot = robot.with_service(&name, move |_| Ok(response.clone()));
        }
        for (name, run) in self.actions {
            let status = match run.status.as_str() {
                "succeeded" => GoalStatus::Succeeded,
                "aborted" => GoalStatus::Aborted,
                "canceled" => GoalStatus::Canceled,
                other => return Err(format!("{name}: no status {other}")),
            };
            robot = robot.with_action(&name, move |_| ScriptedRun {
                feedback: run.feedback.clone(),
                result: Ok(GoalResult {
                    status,
                    result: run.result.clone(),
                }),
                ..ScriptedRun::default()
            });
        }
        for (name, message) in self.topics {
            robot = robot.with_topic(&name, message);
        }
        Ok(robot)
    }
}

/// The scenario's model: every role asks it, and it answers from its script.
struct Scripted(MockCompletionModel);

impl AgentSource for Scripted {
    fn candidates(&self, _role: Role, _need: Need) -> Vec<String> {
        vec!["scripted".to_owned()]
    }
    fn builder(&self, _id: &str) -> Result<AgentBuilder, LlmError> {
        Ok(AgentBuilder::new(self.0.clone()))
    }
    fn take_request(&self, _id: &str) -> Result<(), String> {
        Ok(())
    }
    fn park(&self, _id: &str, _for: Duration) {}
}

fn script(turns: &[Turn]) -> Result<Vec<MockTurn>, String> {
    turns
        .iter()
        .enumerate()
        .map(|(i, t)| match (&t.call, &t.reply) {
            (Some(tool), None) => Ok(MockTurn::tool_call(
                format!("call{i}"),
                tool,
                t.args.clone().unwrap_or_else(|| json!({})),
            )),
            (None, Some(text)) => Ok(MockTurn::text(text)),
            _ => Err(format!("model turn {}: a call or a reply, not both", i + 1)),
        })
        .collect()
}

/// What `path`'s scenario did not do of what it expects; nothing when it passed.
async fn run(path: &Path) -> Result<Vec<String>, String> {
    let read = |p: &Path| std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()));
    let scenario: Scenario = toml::from_str(&read(path)?).map_err(|e| e.to_string())?;
    let shared: Robot =
        toml::from_str(&read(&root().join("robot/robot.toml"))?).map_err(|e| e.to_string())?;
    let robot: Arc<dyn RobotPort> = Arc::new(shared.with(scenario.robot).build()?);
    let model = MockCompletionModel::new(script(&scenario.model)?);
    let state = tempfile::tempdir().map_err(|e| e.to_string())?;
    let options = nervros_core::app::StartOptions {
        source: Some(Arc::new(Scripted(model.clone()))),
        ..Default::default()
    };
    let agent = nervros_core::app::start_with(
        &root().join("robot/profile.toml"),
        robot,
        &state.path().join("quota.json"),
        options,
    )
    .map_err(|e| e.to_string())?;
    let mut events = agent.session.subscribe();
    if scenario.armed {
        agent.session.send(Command::Arm);
    }
    let mut seen = Seen::default();
    let deadline = Instant::now() + DEADLINE;
    for text in &scenario.say {
        if say(
            &agent.session,
            &mut events,
            text,
            deadline,
            scenario.approve,
            &mut seen,
        )
        .await
        {
            seen.timed_out = true;
            break;
        }
    }
    let mut problems = judge(&scenario.expect, &seen);
    if model.request_count() != scenario.model.len() {
        problems.push(format!(
            "the model was asked {} time(s); its script has {} turn(s)",
            model.request_count(),
            scenario.model.len()
        ));
    }
    if !problems.is_empty() {
        // What the tools told the model is most of what a failing scenario needs explained.
        let told: Vec<String> = seen
            .tools
            .iter()
            .zip(&seen.said)
            .map(|((tool, status), said)| format!("{tool} {status}: {said}"))
            .collect();
        problems.push(format!("tools said [{}]", told.join(" | ")));
    }
    Ok(problems)
}

#[tokio::test]
async fn every_scenario_does_what_it_expects() {
    let only = std::env::var("NERVROS_SCENARIO").ok();
    let mut paths: Vec<PathBuf> = std::fs::read_dir(root())
        .into_iter()
        .flatten()
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "toml"))
        .filter(|p| {
            only.as_deref().is_none_or(|o| {
                p.file_name()
                    .is_some_and(|n| n.to_string_lossy().contains(o))
            })
        })
        .collect();
    paths.sort();
    assert!(!paths.is_empty(), "no scenarios in {}", root().display());
    let mut failed = Vec::new();
    for path in &paths {
        let name = path
            .file_stem()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        match run(path).await {
            Ok(problems) if problems.is_empty() => println!("pass {name}"),
            Ok(problems) => failed.push(format!("{name}: {}", problems.join("; "))),
            Err(e) => failed.push(format!("{name}: {e}")),
        }
    }
    assert!(
        failed.is_empty(),
        "{} of {} failed:\n{}",
        failed.len(),
        paths.len(),
        failed.join("\n")
    );
}
