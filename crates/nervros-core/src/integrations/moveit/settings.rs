//! `arm_settings`: how the arm moves NervROS runs itself go, their speed and planner, and arm
//! poses remembered by name for those moves and for `arm_check`.

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use super::Arms;
use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};

const QUERY_WAIT: Duration = Duration::from_secs(3);

/// What NervROS's own arm moves use, kept per robot.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub(crate) struct Settings {
    /// The share of the arms' top speed and acceleration, above 0 and at most 1.
    pub(crate) scaling: f64,
    /// The planning pipeline, such as `ompl`; empty for MoveIt's default.
    pub(crate) pipeline: String,
    /// The planner in it; empty for the pipeline's default.
    pub(crate) planner: String,
    /// How long a plan may take, s.
    pub(crate) planning_s: f64,
    /// Arm poses remembered by name, by group: each joint's position.
    pub(crate) poses: BTreeMap<String, BTreeMap<String, BTreeMap<String, f64>>>,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            scaling: 0.2,
            pipeline: String::new(),
            planner: String::new(),
            planning_s: 5.0,
            poses: BTreeMap::new(),
        }
    }
}

/// The tool.
pub(super) struct ArmSettings {
    spec: ToolSpec,
    arms: Arc<Arms>,
}

impl ArmSettings {
    pub(super) fn new(arms: Arc<Arms>) -> Self {
        let spec = ToolSpec::new(
            "arm_settings",
            "How the arm moves NervROS runs itself go, and arm poses kept by name. show: the \
             speed, the planner and the remembered poses, with the planners MoveIt has. speed: \
             scaling, the share of the arms' top speed (slower is free, faster asks). planner: \
             pipeline, planner and planning_s (asks). remember: the pose group is in now, as \
             name, for ArmTo and arm_check. forget: a remembered pose.",
            json!({
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["show", "speed", "planner", "remember", "forget"]},
                    "scaling": {"type": "number", "description": "For speed: above 0, at most 1"},
                    "pipeline": {"type": "string"},
                    "planner": {"type": "string"},
                    "planning_s": {"type": "number"},
                    "group": {"type": "string", "description": "The arm: right_arm, or right"},
                    "name": {"type": "string", "description": "The pose's name"},
                },
                "required": ["action"],
            }),
            Risk::Observe,
        );
        Self { spec, arms }
    }

    async fn show(&self) -> ToolOutcome {
        let now = self.arms.settings();
        let service = self.arms.config.name("query_planner_interface");
        let planners: Vec<Value> = self
            .arms
            .robot
            .call(
                &service,
                "moveit_msgs/srv/QueryPlannerInterfaces",
                json!({}),
                QUERY_WAIT,
            )
            .await
            .map(|r| {
                r["planner_interfaces"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .map(|p| json!({"pipeline": p["pipeline_id"], "planners": p["planner_ids"]}))
                    .collect()
            })
            .unwrap_or_default();
        let remembered: BTreeMap<&String, Vec<&String>> = now
            .poses
            .iter()
            .map(|(group, poses)| (group, poses.keys().collect()))
            .collect();
        ToolOutcome::ok(json!({
            "scaling": now.scaling,
            "max_scaling": self.arms.config.max_scaling,
            "pipeline": now.pipeline,
            "planner": now.planner,
            "planning_s": now.planning_s,
            "remembered": remembered,
            "available": planners,
        }))
    }

    /// The scaling `args` ask for, and the one now.
    fn scalings(&self, args: &Value) -> Result<(f64, f64), String> {
        let max = self.arms.config.max_scaling;
        let next = args["scaling"]
            .as_f64()
            .filter(|s| s.is_finite() && *s > 0.0 && *s <= max)
            .ok_or_else(|| format!("scaling must be above 0 and at most {max}"))?;
        Ok((self.arms.settings().scaling, next))
    }

    async fn remember(&self, args: &Value) -> ToolOutcome {
        let name = args["name"].as_str().map(str::trim).unwrap_or_default();
        if name.is_empty() {
            return ToolOutcome::failed("give the pose a name");
        }
        let srdf = match self.arms.srdf().await {
            Ok(s) => s,
            Err(why) => return ToolOutcome::failed(why),
        };
        let group = match Arms::group_named(&srdf, args["group"].as_str().unwrap_or_default()) {
            Ok(g) => g.name.clone(),
            Err(why) => return ToolOutcome::failed(why),
        };
        let joints = match self.arms.joints().await {
            Ok(j) => j,
            Err(why) => return ToolOutcome::failed(why),
        };
        // A chain group lists no joints; its named poses name them all.
        let mine: Vec<String> = srdf
            .poses_of(&group)
            .flat_map(|p| p.joints.keys().cloned())
            .chain(
                srdf.group(&group)
                    .map(|g| g.joints.clone())
                    .unwrap_or_default(),
            )
            .collect();
        let pose: BTreeMap<String, f64> = joints
            .into_iter()
            .filter(|(j, _)| mine.contains(j))
            .collect();
        if pose.is_empty() {
            return ToolOutcome::failed(format!("the joint states name none of {group}'s joints"));
        }
        self.arms.change(|s| {
            s.poses
                .entry(group.clone())
                .or_default()
                .insert(name.to_owned(), pose.clone());
        });
        ToolOutcome::ok(json!({"remembered": name, "group": group, "joints": pose.len()}))
    }
}

#[async_trait]
impl Tool for ArmSettings {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let (risk, reason) = match args["action"].as_str().unwrap_or_default() {
            "speed" => {
                let (now, next) = match self.scalings(args) {
                    Ok(s) => s,
                    Err(why) => return Some(Err(ToolOutcome::failed(why))),
                };
                // Slower only makes the arms more careful: free.
                if next <= now {
                    return None;
                }
                (
                    Risk::WorldEdit,
                    format!(
                        "lets NervROS's own arm moves go faster: {:.0}% of top speed instead of {:.0}%",
                        next * 100.0,
                        now * 100.0
                    ),
                )
            }
            "planner" => (
                Risk::WorldEdit,
                "changes the planner NervROS's own arm moves plan with".to_owned(),
            ),
            "remember" | "forget" => (
                Risk::Annotate,
                format!(
                    "{}s the arm pose `{}`",
                    args["action"].as_str().unwrap_or_default(),
                    args["name"].as_str().unwrap_or("?")
                ),
            ),
            _ => return None,
        };
        Some(Ok(Assessment {
            risk,
            resources: Vec::new(),
            reason,
            args: None,
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str().unwrap_or_default() {
            "show" => self.show().await,
            "speed" => match self.scalings(&args) {
                Ok((_, next)) => {
                    self.arms.change(|s| s.scaling = next);
                    ToolOutcome::ok(json!({"scaling": next}))
                }
                Err(why) => ToolOutcome::failed(why),
            },
            "planner" => {
                let time = args["planning_s"].as_f64();
                if time.is_some_and(|t| !t.is_finite() || t <= 0.0 || t > 60.0) {
                    return ToolOutcome::failed("planning_s must be above 0 and at most 60");
                }
                let text = |k: &str| args[k].as_str().map(|v| v.trim().to_owned());
                self.arms.change(|s| {
                    if let Some(p) = text("pipeline") {
                        s.pipeline = p;
                    }
                    if let Some(p) = text("planner") {
                        s.planner = p;
                    }
                    if let Some(t) = time {
                        s.planning_s = t;
                    }
                });
                let now = self.arms.settings();
                ToolOutcome::ok(
                    json!({"pipeline": now.pipeline, "planner": now.planner, "planning_s": now.planning_s}),
                )
            }
            "remember" => self.remember(&args).await,
            "forget" => {
                let name = args["name"].as_str().unwrap_or_default();
                let mut gone = false;
                self.arms.change(|s| {
                    for poses in s.poses.values_mut() {
                        gone |= poses.remove(name).is_some();
                    }
                    s.poses.retain(|_, p| !p.is_empty());
                });
                if gone {
                    ToolOutcome::ok(json!({"forgot": name}))
                } else {
                    ToolOutcome::failed(format!("no arm pose `{name}` is remembered"))
                }
            }
            other => ToolOutcome::failed(format!(
                "action `{other}` is not one of show, speed, planner, remember, forget"
            )),
        }
    }
}
