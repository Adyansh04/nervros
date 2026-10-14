//! A playbook's checks, run: each test against the robot as it is now and the recording of the
//! minutes before, read-only. A check that cannot run leaves its playbook undecided rather than
//! ruled out.

use std::time::Duration;

use nervros_ros::{GraphDetail, RobotPort, RosError, Transform};

use super::playbook::{Check, Loaded, Match, Test};
use super::recorder::Recording;
use super::trigger::names_node;
use crate::watch;

/// The longest a check may take: a rate needs its window, a topic its first message.
const CHECK_TIMEOUT: Duration = Duration::from_secs(6);
/// How long a pose's topic is waited on.
const POSE_WAIT: Duration = Duration::from_secs(2);
/// Less log than this, and a line not found in it rules nothing out.
const LEAST_LOG_S: f64 = 30.0;

/// What the checks look at.
pub(crate) struct Probe<'a> {
    pub robot: &'a dyn RobotPort,
    pub recording: &'a Recording,
    /// The graph as it is now, read once for every check; `None` when it could not be read.
    pub graph: Option<&'a GraphDetail>,
    /// The failed mission's reason, or empty.
    pub failure: &'a str,
    /// Topics the profile keeps from being read.
    pub read_deny: &'a [String],
}

/// What one check saw: whether its test held (before `not`), and what it saw.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Seen {
    pub held: bool,
    pub saw: String,
}

fn seen(held: bool, saw: impl Into<String>) -> Seen {
    Seen {
        held,
        saw: saw.into(),
    }
}

/// Runs one test.
///
/// # Errors
///
/// What kept it from running: a topic of a type this build does not know, an unreadable graph.
pub(crate) async fn run(test: &Test, probe: &Probe<'_>) -> Result<Seen, String> {
    test.readable(probe.read_deny)?;
    match test {
        Test::Topic {
            topic,
            ty,
            condition,
        } => Ok(watch::observe(probe.robot, topic, ty, condition)
            .await?
            .map_or_else(
                || seen(false, format!("{topic} has said nothing")),
                |(held, saw)| seen(held, saw),
            )),
        Test::Log {
            pattern,
            node,
            level,
            at_least,
        } => {
            let seen = logs(probe.recording, pattern, node, *level, *at_least);
            let recorded = probe.recording.at_s - probe.recording.since_s;
            if !seen.held && recorded < LEAST_LOG_S {
                // A line not seen in a moment's log may yet have been said.
                return Err(format!("only {recorded:.0} s of the log was recorded"));
            }
            Ok(seen)
        }
        Test::NodeMissing(name) => {
            let graph = probe.graph.ok_or("the graph could not be read")?;
            let running = graph.nodes.iter().any(|n| names_node(n, name));
            Ok(if running {
                seen(false, format!("{name} runs"))
            } else {
                seen(true, format!("{name} is not running"))
            })
        }
        Test::NodeDuplicated(name) => {
            let graph = probe.graph.ok_or("the graph could not be read")?;
            Ok(duplicated(&graph.nodes, name))
        }
        Test::TfMissing { target, source } => Ok(match probe.robot.transform(target, source) {
            Ok(t) => seen(
                false,
                format!(
                    "{target} to {source} is at ({:.2}, {:.2})",
                    t.translation[0], t.translation[1]
                ),
            ),
            Err(_) => seen(true, format!("nothing links {target} to {source}")),
        }),
        Test::PoseGap {
            target,
            source,
            topic,
            ty,
            field,
            metres,
            degrees,
        } => {
            pose_gap(
                probe.robot,
                (target, source),
                (topic, ty, field),
                *metres,
                *degrees,
            )
            .await
        }
        Test::Failure(pattern) => Ok(if probe.failure.is_empty() {
            seen(false, "no mission failed")
        } else {
            let said = crate::tools::clip(probe.failure, 160);
            seen(probe.failure.to_lowercase().contains(pattern), said)
        }),
    }
}

/// The recording's lines at `level` or above, from nodes holding `node`, that hold `pattern`.
fn logs(recording: &Recording, pattern: &str, node: &str, level: u8, at_least: usize) -> Seen {
    let found: Vec<_> = recording
        .logs
        .iter()
        .filter(|l| l.level >= level)
        .filter(|l| node.is_empty() || l.node.contains(node))
        .filter(|l| l.text.to_lowercase().contains(pattern))
        .collect();
    let Some(last) = found.last() else {
        let window = (recording.at_s - recording.since_s).max(0.0);
        return seen(
            false,
            format!("no line says \"{pattern}\" in the last {window:.0} s"),
        );
    };
    let times = if found.len() > 1 {
        format!(" ({} times)", found.len())
    } else {
        String::new()
    };
    seen(
        found.len() >= at_least,
        format!(
            "{}: {}{times}",
            last.node,
            crate::tools::clip(&last.text, 160)
        ),
    )
}

fn duplicated(nodes: &[String], name: &str) -> Seen {
    let mut counts: std::collections::BTreeMap<&str, usize> = std::collections::BTreeMap::new();
    for n in nodes.iter().filter(|n| name == "*" || names_node(n, name)) {
        *counts.entry(n.as_str()).or_default() += 1;
    }
    match counts.into_iter().find(|(_, c)| *c > 1) {
        Some((node, times)) => seen(true, format!("{node} runs {times} times")),
        None if name == "*" => seen(false, "no node runs twice"),
        None => seen(false, format!("{name} runs once at most")),
    }
}

async fn pose_gap(
    robot: &dyn RobotPort,
    (target, source): (&str, &str),
    (topic, ty, field): (&str, &str, &str),
    metres: f64,
    degrees: f64,
) -> Result<Seen, String> {
    let Ok(tf) = robot.transform(target, source) else {
        return Ok(seen(
            false,
            format!("nothing links {target} to {source} yet"),
        ));
    };
    let msg = match robot.latest(topic, ty, POSE_WAIT).await {
        Ok(m) => m,
        Err(RosError::NoData(_)) => return Ok(seen(false, format!("{topic} has said nothing"))),
        Err(e) => return Err(e.to_string()),
    };
    let pose = if field.is_empty() {
        Some(&msg)
    } else {
        watch::field(&msg, field)
    };
    let number = |v: &serde_json::Value, k: &str| v[k].as_f64();
    let other = pose
        .and_then(|p| {
            let (pos, q) = (&p["position"], &p["orientation"]);
            Some(Transform {
                translation: [number(pos, "x")?, number(pos, "y")?, 0.0],
                rotation: [
                    number(q, "x")?,
                    number(q, "y")?,
                    number(q, "z")?,
                    number(q, "w")?,
                ],
            })
        })
        .ok_or_else(|| format!("{topic} has no pose at `{field}`"))?;
    let (x, y, yaw) = tf.planar();
    let (ox, oy, oyaw) = other.planar();
    let gap = (x - ox).hypot(y - oy);
    let turned = (yaw - oyaw)
        .sin()
        .atan2((yaw - oyaw).cos())
        .abs()
        .to_degrees();
    Ok(seen(
        gap > metres || turned > degrees,
        format!("{target} to {source} is {gap:.2} m and {turned:.0} degrees from {topic}"),
    ))
}

/// A playbook's verdict on the moment: whether it holds (`None` when a check it needed could not
/// run), and what each check saw.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Judged {
    pub holds: Option<bool>,
    pub evidence: Vec<String>,
}

/// A check's name for the report: its own, or what it tests.
fn label(check: &Check, test: &Test) -> String {
    if !check.name.is_empty() {
        return check.name.clone();
    }
    match test {
        Test::Topic { topic, .. } | Test::PoseGap { topic, .. } => topic.clone(),
        Test::Log { node, .. } if !node.is_empty() => format!("{node}'s log"),
        Test::Log { .. } => "the log".to_owned(),
        Test::NodeMissing(n) | Test::NodeDuplicated(n) => n.clone(),
        Test::TfMissing { target, source } => format!("{target} to {source}"),
        Test::Failure(_) => "the failure".to_owned(),
    }
}

/// Runs every check of `book`, at once.
pub(crate) async fn judge(book: &Loaded, probe: &Probe<'_>) -> Judged {
    let results = futures::future::join_all(
        book.tests
            .iter()
            .map(|t| tokio::time::timeout(CHECK_TIMEOUT, run(t, probe))),
    )
    .await;
    let mut held = Vec::new();
    let mut evidence = Vec::new();
    for ((check, test), result) in book.book.checks.iter().zip(&book.tests).zip(results) {
        let name = label(check, test);
        match result {
            Ok(Ok(s)) => {
                held.push(Some(s.held != check.not));
                evidence.push(format!("{name}: {}", s.saw));
            }
            Ok(Err(e)) => {
                held.push(None);
                evidence.push(format!("{name}: could not check ({e})"));
            }
            Err(_) => {
                held.push(None);
                evidence.push(format!("{name}: took too long to check"));
            }
        }
    }
    let holds = match book.book.mode {
        Match::All if held.contains(&Some(false)) => Some(false),
        Match::Any if held.contains(&Some(true)) => Some(true),
        _ if held.contains(&None) => None,
        Match::All => Some(true),
        Match::Any => Some(false),
    };
    Judged { holds, evidence }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_1_SQRT_2;

    use nervros_ros::fake::FakeRobot;
    use serde_json::json;

    use super::*;
    use crate::incident::playbook::Playbook;
    use crate::incident::recorder::LogLine;

    fn book(toml_text: &str) -> Loaded {
        toml::from_str::<Playbook>(toml_text)
            .unwrap()
            .check("test")
            .unwrap()
    }

    fn recording(lines: &[(&str, u8, &str)]) -> Recording {
        Recording {
            at_s: 100.0,
            since_s: 0.0,
            logs: lines
                .iter()
                .map(|(node, level, text)| LogLine {
                    t: 90.0,
                    level: *level,
                    node: (*node).into(),
                    text: (*text).into(),
                })
                .collect(),
            marks: Vec::new(),
        }
    }

    #[tokio::test]
    async fn a_playbook_holds_when_its_checks_do_and_says_what_each_saw() {
        let robot = FakeRobot::new().with_topic("/state", json!({"tilt_deg": 70.0}));
        let graph = GraphDetail {
            nodes: vec!["/robot/detector".into(), "/nav".into(), "/nav".into()],
            ..GraphDetail::default()
        };
        let recording = recording(&[
            ("lifecycle_manager_navigation", 40, "Aborting bringup"),
            ("other", 40, "aborting bringup"),
        ]);
        let probe = Probe {
            robot: &robot,
            recording: &recording,
            graph: Some(&graph),
            failure: "Nav2 aborted the goal",
            read_deny: &[],
        };
        let fell = book(
            r#"
            id = "fell"
            title = "t"
            verdict = "v"
            fix = "f"
            [[check]]
            name = "the robot's tilt"
            topic = "/state"
            type = "nervros_interfaces/msg/RobotState"
            condition = "value"
            field = "tilt_deg"
            op = ">"
            value = 45
            [[check]]
            log = "aborting BRINGUP"
            node = "lifecycle"
            level = "error"
            [[check]]
            failure_has = "aborted"
            [[check]]
            node_missing = "detector"
            not = true
            [[check]]
            node_duplicated = "*"
            "#,
        );
        let judged = judge(&fell, &probe).await;
        assert_eq!(judged.holds, Some(true), "{:?}", judged.evidence);
        assert_eq!(
            judged.evidence,
            [
                "the robot's tilt: tilt_deg is 70.0",
                "lifecycle's log: lifecycle_manager_navigation: Aborting bringup",
                "the failure: Nav2 aborted the goal",
                "detector: detector runs",
                "*: /nav runs 2 times",
            ]
        );
    }

    #[tokio::test]
    async fn one_false_check_rules_out_all_and_one_true_one_holds_any() {
        let robot = FakeRobot::new();
        let recording = recording(&[("a", 30, "x")]);
        let probe = Probe {
            robot: &robot,
            recording: &recording,
            graph: None,
            failure: "",
            read_deny: &[],
        };
        let text = |mode: &str| {
            format!(
                "id = \"b\"\ntitle = \"t\"\nverdict = \"v\"\nfix = \"f\"\nmatch = \"{mode}\"\n\
                 [[check]]\nlog = \"x\"\n[[check]]\nlog = \"y\"\n[[check]]\nnode_missing = \"/n\"\n"
            )
        };
        let all = judge(&book(&text("all")), &probe).await;
        assert_eq!(all.holds, Some(false), "y was never said");
        let any = judge(&book(&text("any")), &probe).await;
        assert_eq!(any.holds, Some(true));
        assert!(
            any.evidence[2].contains("could not check (the graph could not be read)"),
            "{:?}",
            any.evidence
        );
        let only_graph = book(
            "id = \"c\"\ntitle = \"t\"\nverdict = \"v\"\nfix = \"f\"\n[[check]]\nnode_missing = \"/n\"\n",
        );
        assert_eq!(
            judge(&only_graph, &probe).await.holds,
            None,
            "undecided, not ruled out"
        );
    }

    #[tokio::test]
    async fn a_line_missing_from_a_moment_of_log_rules_nothing_out() {
        let robot = FakeRobot::new();
        let mut recording = recording(&[("a", 30, "x")]);
        recording.since_s = 95.0;
        let probe = Probe {
            robot: &robot,
            recording: &recording,
            graph: None,
            failure: "",
            read_deny: &[],
        };
        let log = |pattern: &str| Test::Log {
            pattern: pattern.into(),
            node: String::new(),
            level: 30,
            at_least: 1,
        };
        assert!(
            run(&log("x"), &probe).await.unwrap().held,
            "a line seen holds"
        );
        assert_eq!(
            run(&log("y"), &probe).await,
            Err("only 5 s of the log was recorded".to_owned())
        );
    }

    #[tokio::test]
    async fn a_pose_gap_compares_the_transform_with_the_topic_s_pose() {
        let truth = json!({"pose": {"pose": {"position": {"x": 1.0, "y": 0.0, "z": 0.8},
            "orientation": {"x": 0.0, "y": 0.0, "z": FRAC_1_SQRT_2, "w": FRAC_1_SQRT_2}}}});
        let gap = Test::PoseGap {
            target: "map".into(),
            source: "base_link".into(),
            topic: "/truth".into(),
            ty: "nav_msgs/msg/Odometry".into(),
            field: "pose.pose".into(),
            metres: 0.5,
            degrees: 20.0,
        };
        let recording = Recording::default();
        let check = |robot: FakeRobot| {
            let gap = gap.clone();
            let recording = recording.clone();
            async move {
                let probe = Probe {
                    robot: &robot,
                    recording: &recording,
                    graph: None,
                    failure: "",
                    read_deny: &[],
                };
                run(&gap, &probe).await.unwrap()
            }
        };
        let s = check(FakeRobot::new().with_topic("/truth", truth.clone())).await;
        assert!(
            !s.held && s.saw.contains("yet"),
            "{s:?}: no transform is not a gap"
        );
        let s = check(FakeRobot::new().with_topic("/truth", truth).with_transform(
            "map",
            "base_link",
            Transform::IDENTITY,
        ))
        .await;
        assert!(s.held, "{s:?}");
        assert_eq!(
            s.saw,
            "map to base_link is 1.00 m and 90 degrees from /truth"
        );
    }
}
