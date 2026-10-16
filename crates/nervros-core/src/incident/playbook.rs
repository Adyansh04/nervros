//! Playbooks: what an incident may be, written down by someone who debugged the robot before. A
//! TOML file each: the incidents it may explain, read-only checks, and when they hold, the verdict
//! and the fix. NervROS carries a few that fit any robot, a profile adds its robot's own, and the
//! agent adds what the operator asks it to remember; a later one replaces an earlier of its id.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::ros_tools::logs::level_number;
use crate::watch::Condition;

/// Playbooks that fit any robot, with `{map_frame}` and `{base_frame}` filled from the profile.
const BUILT_IN: [(&str, &str); 2] = [
    (
        "no-localization",
        include_str!("playbooks/no-localization.toml"),
    ),
    (
        "duplicate-nodes",
        include_str!("playbooks/duplicate-nodes.toml"),
    ),
];

/// What opened an incident.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    /// A mission ended in failure.
    MissionFailed,
    /// A topic the profile watches went quiet.
    TopicSilent,
    /// A node the profile watches stopped running.
    NodeGone,
    /// A node logged errors in a burst.
    ErrorBurst,
    /// The operator or the agent asked.
    Asked,
}

/// How a playbook's checks combine.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Match {
    /// Every check holds.
    #[default]
    All,
    /// One check holds.
    Any,
}

/// A playbook as written.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Playbook {
    /// Its name, kebab-case, such as `stale-shared-memory`.
    pub id: String,
    /// What it is, in a few words.
    pub title: String,
    /// The incidents it may explain; none for every one.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub explains: Vec<Kind>,
    /// Whether every check must hold, or one.
    #[serde(default, rename = "match")]
    pub mode: Match,
    /// What it means when the checks hold.
    pub verdict: String,
    /// What to do about it.
    pub fix: String,
    /// The agent skill that has the long form, if any.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub procedure: String,
    /// What it checks.
    #[serde(rename = "check")]
    pub checks: Vec<Check>,
}

/// One read-only check, with exactly one test: a `topic` with a watch `condition`, `log`,
/// `node_missing`, `node_duplicated`, `tf_missing`, `pose_gap` or `failure_has`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Check {
    /// What it looks at, for the report, such as "the head camera".
    #[serde(skip_serializing_if = "String::is_empty")]
    pub name: String,
    /// Holds when its test does not.
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub not: bool,
    /// A topic, read live; with `condition` (`rate_below` and `hz`, `value` and `field`, `op`,
    /// `value`, or `text` and `contains`), or the pose `pose_gap` compares.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub topic: String,
    /// The topic's type.
    #[serde(rename = "type", skip_serializing_if = "String::is_empty")]
    pub msg_type: String,
    /// The watch condition on `topic`.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub condition: String,
    /// For `rate_below`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hz: Option<f64>,
    /// For `value`, and for `pose_gap` the pose's path in the message, such as `pose.pose`.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub field: String,
    /// For `value`: `<`, `<=`, `>`, `>=`, `==` or `!=`.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub op: String,
    /// For `value`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value: Option<Value>,
    /// For `text`.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub contains: String,
    /// A log line in the recording holding this, case aside.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub log: String,
    /// For `log`: only from nodes whose name holds this.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub node: String,
    /// For `log`: the least level, `debug`, `info`, `warn` (unless given), `error` or `fatal`.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub level: String,
    /// For `log`: how many lines at least, 1 unless given.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub at_least: Option<u32>,
    /// A node, `/namespace/name`, that is not running.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub node_missing: String,
    /// A node that runs twice under one name, or `*` for any.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub node_duplicated: String,
    /// `[target, source]`: no transform links them.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub tf_missing: Vec<String>,
    /// `[target, source]`: the transform differs from the pose on `topic`, at `field`, by more
    /// than `metres` or `degrees`.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub pose_gap: Vec<String>,
    /// For `pose_gap`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metres: Option<f64>,
    /// For `pose_gap`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub degrees: Option<f64>,
    /// The failed mission's reason holds this, case aside.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub failure_has: String,
}

/// A check's test, made sure of when its playbook loads.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Test {
    Topic {
        topic: String,
        ty: String,
        condition: Condition,
    },
    Log {
        pattern: String,
        node: String,
        level: u8,
        at_least: usize,
    },
    NodeMissing(String),
    NodeDuplicated(String),
    TfMissing {
        target: String,
        source: String,
    },
    PoseGap {
        target: String,
        source: String,
        topic: String,
        ty: String,
        field: String,
        metres: f64,
        degrees: f64,
    },
    Failure(String),
}

impl Test {
    /// Refuses a test that reads what the profile keeps from being read, or decodes a camera's
    /// or a map's messages one by one.
    pub(crate) fn readable(&self, read_deny: &[String]) -> Result<(), String> {
        match self {
            Self::Topic {
                topic,
                ty,
                condition,
            } => {
                crate::watch::readable(topic, read_deny)?;
                if condition.reads_messages() {
                    crate::watch::small(topic, ty)?;
                }
                Ok(())
            }
            Self::PoseGap { topic, ty, .. } => {
                crate::watch::readable(topic, read_deny)?;
                crate::watch::small(topic, ty)
            }
            Self::Log { .. }
            | Self::NodeMissing(_)
            | Self::NodeDuplicated(_)
            | Self::TfMissing { .. }
            | Self::Failure(_) => Ok(()),
        }
    }
}

impl Check {
    /// Its test, or what is wrong with it.
    pub(crate) fn test(&self) -> Result<Test, String> {
        let pair = |v: &[String], what: &str| match v {
            [target, source] => Ok((target.clone(), source.clone())),
            _ => Err(format!("{what} is [target, source], two frames")),
        };
        let set = [
            !self.condition.is_empty(),
            !self.log.is_empty(),
            !self.node_missing.is_empty(),
            !self.node_duplicated.is_empty(),
            !self.tf_missing.is_empty(),
            !self.pose_gap.is_empty(),
            !self.failure_has.is_empty(),
        ];
        if set.iter().filter(|s| **s).count() != 1 {
            return Err(
                "a check has one test: a topic's condition, log, node_missing, \
                        node_duplicated, tf_missing, pose_gap or failure_has"
                    .to_owned(),
            );
        }
        let topic = || {
            (!self.topic.is_empty() && !self.msg_type.is_empty())
                .then(|| (self.topic.clone(), self.msg_type.clone()))
                .ok_or("it needs `topic` and its `type`")
        };
        if !self.condition.is_empty() {
            let (topic, ty) = topic()?;
            let condition = Condition::parse(&json!({
                "condition": self.condition, "hz": self.hz, "field": self.field,
                "op": if self.op.is_empty() { "==" } else { &self.op },
                "value": self.value, "contains": self.contains,
            }))?;
            return Ok(Test::Topic {
                topic,
                ty,
                condition,
            });
        }
        if !self.log.is_empty() {
            return Ok(Test::Log {
                pattern: self.log.to_lowercase(),
                node: self.node.trim_start_matches('/').to_owned(),
                level: level_number(&self.level).ok_or_else(|| {
                    format!(
                        "level is debug, info, warn, error or fatal, not {}",
                        self.level
                    )
                })?,
                at_least: self.at_least.map_or(1, |n| n.max(1) as usize),
            });
        }
        if !self.node_missing.is_empty() {
            return Ok(Test::NodeMissing(self.node_missing.clone()));
        }
        if !self.node_duplicated.is_empty() {
            return Ok(Test::NodeDuplicated(self.node_duplicated.clone()));
        }
        if !self.tf_missing.is_empty() {
            let (target, source) = pair(&self.tf_missing, "tf_missing")?;
            return Ok(Test::TfMissing { target, source });
        }
        if !self.pose_gap.is_empty() {
            let (target, source) = pair(&self.pose_gap, "pose_gap")?;
            let (topic, ty) = topic()?;
            let (metres, degrees) = (
                self.metres.unwrap_or(f64::INFINITY),
                self.degrees.unwrap_or(f64::INFINITY),
            );
            if !(metres.is_finite() || degrees.is_finite()) || metres <= 0.0 || degrees <= 0.0 {
                return Err("pose_gap needs `metres` or `degrees` above 0".to_owned());
            }
            return Ok(Test::PoseGap {
                target,
                source,
                topic,
                ty,
                field: self.field.clone(),
                metres,
                degrees,
            });
        }
        Ok(Test::Failure(self.failure_has.to_lowercase()))
    }
}

/// A playbook ready to run: its checks' tests made sure of.
#[derive(Debug, Clone)]
pub struct Loaded {
    /// As written.
    pub book: Playbook,
    /// Each check's test, in order.
    pub(crate) tests: Vec<Test>,
    /// Where it came from: `built in`, or its file.
    pub source: String,
}

impl Loaded {
    /// Whether it may explain an incident of this kind.
    #[must_use]
    pub fn explains(&self, kind: Kind) -> bool {
        self.book.explains.is_empty() || self.book.explains.contains(&kind)
    }
}

impl Playbook {
    /// Its checks made sure of, or every problem with it.
    ///
    /// # Errors
    ///
    /// A bad id, an empty field, no checks, or a check that cannot run.
    pub fn check(self, source: &str) -> Result<Loaded, String> {
        let mut problems = Vec::new();
        let kebab = |c: char| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-';
        if self.id.is_empty() || !self.id.chars().all(kebab) {
            problems.push(format!(
                "id `{}` is kebab-case, such as stale-shared-memory",
                self.id
            ));
        }
        for (what, text) in [
            ("title", &self.title),
            ("verdict", &self.verdict),
            ("fix", &self.fix),
        ] {
            if text.trim().is_empty() {
                problems.push(format!("it needs a {what}"));
            }
        }
        if self.checks.is_empty() {
            problems.push("it needs at least one check".to_owned());
        }
        let mut tests = Vec::new();
        for (i, check) in self.checks.iter().enumerate() {
            match check.test() {
                Ok(t) => tests.push(t),
                Err(e) => problems.push(format!("check {}: {e}", i + 1)),
            }
        }
        if problems.is_empty() {
            Ok(Loaded {
                book: self,
                tests,
                source: source.to_owned(),
            })
        } else {
            Err(problems.join("; "))
        }
    }
}

/// `text` with each `{name}` of `vars` filled in.
fn fill(text: &str, vars: &[(&str, &str)]) -> String {
    vars.iter().fold(text.to_owned(), |t, (name, value)| {
        t.replace(&format!("{{{name}}}"), value)
    })
}

fn parse(text: &str, vars: &[(&str, &str)], source: &str) -> Result<Loaded, String> {
    toml::from_str::<Playbook>(&fill(text, vars))
        .map_err(|e| e.message().to_owned())?
        .check(source)
}

/// NervROS's playbooks, then each folder's `*.toml`, a later one replacing an earlier of its id;
/// and what could not be read. `vars` fills `{map_frame}` and the like.
#[must_use]
pub fn load(dirs: &[PathBuf], vars: &[(&str, &str)]) -> (Vec<Loaded>, Vec<String>) {
    let mut books: Vec<Loaded> = Vec::new();
    let mut problems = Vec::new();
    let mut add = |loaded: Loaded| match books.iter_mut().find(|b| b.book.id == loaded.book.id) {
        Some(old) => *old = loaded,
        None => books.push(loaded),
    };
    for (name, text) in BUILT_IN {
        match parse(text, vars, "built in") {
            Ok(b) => add(b),
            Err(e) => problems.push(format!("built-in playbook {name}: {e}")),
        }
    }
    for dir in dirs {
        let Ok(entries) = std::fs::read_dir(dir) else {
            // A folder for remembered playbooks is made by the first one.
            continue;
        };
        let mut files: Vec<PathBuf> = entries
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x == "toml"))
            .collect();
        files.sort();
        for file in files {
            match std::fs::read_to_string(&file)
                .map_err(|e| e.to_string())
                .and_then(|text| parse(&text, vars, &file.display().to_string()))
            {
                Ok(b) => add(b),
                Err(e) => problems.push(format!("{}: {e}", file.display())),
            }
        }
    }
    (books, problems)
}

/// Writes `book` as `<dir>/<id>.toml`.
///
/// # Errors
///
/// It could not be written as TOML, or the file could not be.
pub fn save(dir: &Path, book: &Playbook) -> Result<PathBuf, String> {
    let text = toml::to_string_pretty(book).map_err(|e| e.to_string())?;
    let path = dir.join(format!("{}.toml", book.id));
    crate::persist::write_atomic(&path, text.as_bytes()).map_err(|e| e.to_string())?;
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    const VARS: [(&str, &str); 2] = [("map_frame", "map"), ("base_frame", "base_link")];

    #[test]
    fn the_built_in_playbooks_load_with_the_profile_s_frames() {
        let (books, problems) = load(&[], &VARS);
        assert!(problems.is_empty(), "{problems:?}");
        let lost = books
            .iter()
            .find(|b| b.book.id == "no-localization")
            .unwrap();
        assert_eq!(
            lost.tests,
            [Test::TfMissing {
                target: "map".into(),
                source: "base_link".into()
            }]
        );
        assert!(
            lost.book.verdict.contains("map to base_link"),
            "{}",
            lost.book.verdict
        );
    }

    #[test]
    fn a_folder_s_playbook_replaces_a_built_in_one_and_a_bad_one_is_named() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(
            dir.path().join("a.toml"),
            r#"
            id = "no-localization"
            title = "AMCL is off"
            explains = ["mission_failed"]
            verdict = "v"
            fix = "f"
            [[check]]
            topic = "/scan"
            type = "sensor_msgs/msg/LaserScan"
            condition = "rate_below"
            hz = 5
            "#,
        )
        .unwrap();
        std::fs::write(
            dir.path().join("b.toml"),
            r#"
            id = "Bad Id"
            title = "t"
            verdict = "v"
            fix = "f"
            [[check]]
            log = "x"
            node_missing = "/y"
            [[check]]
            pose_gap = ["map"]
            "#,
        )
        .unwrap();
        let (books, problems) = load(&[dir.path().to_path_buf()], &VARS);
        let lost = books
            .iter()
            .find(|b| b.book.id == "no-localization")
            .unwrap();
        assert_eq!(lost.book.title, "AMCL is off");
        assert!(lost.explains(Kind::MissionFailed) && !lost.explains(Kind::Asked));
        let bad = problems.join("\n");
        assert!(
            bad.contains("b.toml") && bad.contains("kebab-case"),
            "{bad}"
        );
        assert!(bad.contains("check 1: a check has one test"), "{bad}");
        assert!(
            bad.contains("check 2: pose_gap is [target, source]"),
            "{bad}"
        );
    }

    #[test]
    fn checks_become_their_tests() {
        let check = |toml: &str| toml::from_str::<Check>(toml).unwrap().test();
        assert_eq!(
            check(
                "log = \"Aborting Bringup\"\nnode = \"/lifecycle\"\nlevel = \"error\"\nat_least = 3"
            ),
            Ok(Test::Log {
                pattern: "aborting bringup".into(),
                node: "lifecycle".into(),
                level: 40,
                at_least: 3
            })
        );
        assert!(
            check("log = \"x\"\nlevel = \"loud\"")
                .unwrap_err()
                .contains("level is")
        );
        assert!(
            check("condition = \"rate_below\"\nhz = 2")
                .unwrap_err()
                .contains("`topic` and its `type`")
        );
        assert!(matches!(
            check("pose_gap = [\"map\", \"base_link\"]\ntopic = \"/t\"\ntype = \"x/msg/Y\"\nmetres = 0.5"),
            Ok(Test::PoseGap { metres, degrees, .. })
                if (metres - 0.5).abs() < 1e-12 && degrees.is_infinite()
        ));
        assert_eq!(
            check("failure_has = \"Aborted\""),
            Ok(Test::Failure("aborted".into()))
        );
    }

    #[test]
    fn a_saved_playbook_reads_back_the_same() {
        let dir = tempfile::tempdir().unwrap();
        let book = Playbook {
            id: "detector-budget".into(),
            title: "t".into(),
            explains: vec![Kind::Asked],
            mode: Match::Any,
            verdict: "v".into(),
            fix: "f".into(),
            procedure: String::new(),
            checks: vec![Check {
                topic: "/objects".into(),
                msg_type: "canopy_msgs/msg/WorldObjectArray".into(),
                condition: "value".into(),
                field: "objects.0.label".into(),
                value: Some(json!("mug")),
                ..Check::default()
            }],
        };
        let path = save(dir.path(), &book).unwrap();
        let text = std::fs::read_to_string(path).unwrap();
        assert_eq!(toml::from_str::<Playbook>(&text).unwrap(), book);
    }
}
