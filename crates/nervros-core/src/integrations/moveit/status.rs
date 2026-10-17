//! What NervROS last saw of MoveIt: where each arm is among its named poses, and what is in the
//! planning scene.

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::Arms;
use super::srdf::Srdf;
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// How often the watcher looks.
const PERIOD: Duration = Duration::from_secs(5);
const SCENE_WAIT: Duration = Duration::from_secs(3);
/// How near a joint must be to a named pose's value to count as at it, radians.
pub(super) const AT_POSE: f64 = 0.05;
/// `PlanningSceneComponents`: the world's object names and the objects the arms hold.
const NAMES_AND_HELD: u64 = 8 | 4;

/// What the planning scene holds.
#[derive(Debug, Clone, Default, PartialEq)]
pub(super) struct SceneSummary {
    /// The obstacles' ids.
    pub(super) objects: Vec<String>,
    /// Each held object's id, and the link it hangs from.
    pub(super) held: Vec<(String, String)>,
}

/// Reads the scene's object names and the held objects.
pub(super) async fn scene_summary(arms: &Arms) -> Result<SceneSummary, String> {
    let service = arms.config.name("get_planning_scene");
    let scene = arms
        .robot
        .call(
            &service,
            "moveit_msgs/srv/GetPlanningScene",
            json!({"components": {"components": NAMES_AND_HELD}}),
            SCENE_WAIT,
        )
        .await
        .map_err(|e| format!("MoveIt's planning scene did not answer: {e}"))?;
    let scene = &scene["scene"];
    let objects = scene["world"]["collision_objects"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|o| o["id"].as_str().map(str::to_owned))
        .collect();
    let held = scene["robot_state"]["attached_collision_objects"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|a| {
            Some((
                a["object"]["id"].as_str()?.to_owned(),
                a["link_name"].as_str().unwrap_or_default().to_owned(),
            ))
        })
        .collect();
    Ok(SceneSummary { objects, held })
}

/// Where each arm is: the named pose it is at, or none between poses.
pub(super) fn arms_at(
    srdf: &Srdf,
    joints: &BTreeMap<String, f64>,
) -> Vec<(String, Option<String>)> {
    srdf.arms()
        .iter()
        .map(|g| {
            let at = srdf
                .pose_at(&g.name, joints, AT_POSE)
                .map(|p| p.name.clone());
            (g.name.clone(), at)
        })
        .collect()
}

/// Keeps a line on MoveIt fresh in the background while the session lives.
#[derive(Debug)]
pub(super) struct Watcher {
    parts: Mutex<Vec<String>>,
}

impl Watcher {
    /// Starts watching.
    pub(super) fn start(arms: Arc<Arms>) -> Arc<Self> {
        let watcher = Arc::new(Self {
            parts: Mutex::new(Vec::new()),
        });
        let weak = Arc::downgrade(&watcher);
        tokio::spawn(async move {
            loop {
                let Some(watcher) = weak.upgrade() else {
                    return;
                };
                let parts = look(&arms).await;
                *crate::lock(&watcher.parts) = parts;
                drop(watcher);
                tokio::time::sleep(PERIOD).await;
            }
        });
        watcher
    }

    /// What it last saw, a headline first.
    pub(super) fn parts(&self) -> Vec<String> {
        crate::lock(&self.parts).clone()
    }
}

async fn look(arms: &Arms) -> Vec<String> {
    let (srdf, joints, scene) = tokio::join!(arms.srdf(), arms.joints(), scene_summary(arms));
    let Ok(srdf) = srdf else {
        return vec!["MoveIt: not heard from".to_owned()];
    };
    let mut parts = Vec::new();
    if let Ok(joints) = joints {
        parts.push("MoveIt: up".to_owned());
        parts.extend(
            arms_at(&srdf, &joints)
                .into_iter()
                .map(|(arm, pose)| match pose {
                    Some(p) => format!("{arm} at `{p}`"),
                    None => format!("{arm} between named poses"),
                }),
        );
    } else {
        parts.push("MoveIt: up, no joint states".to_owned());
    }
    if let Ok(scene) = scene {
        if !scene.objects.is_empty() {
            parts.push(format!(
                "{} in its planning scene",
                crate::counted(scene.objects.len(), "obstacle")
            ));
        }
        for (object, link) in scene.held {
            parts.push(format!("`{object}` attached to {link}"));
        }
    }
    parts
}

/// `arm_status`: MoveIt's groups, poses, planners and scene, read fresh.
pub(super) struct ArmStatus {
    spec: ToolSpec,
    arms: Arc<Arms>,
}

impl ArmStatus {
    pub(super) fn new(arms: Arc<Arms>) -> Self {
        let spec = ToolSpec::new(
            "arm_status",
            "MoveIt now: each arm group, the named poses it can move to and the one it is at, \
             each hand and its named poses, the planners MoveIt has, and the obstacles in its \
             planning scene and what each hand holds.",
            json!({"type": "object", "properties": {}, "additionalProperties": false}),
            Risk::Observe,
        );
        Self { spec, arms }
    }

    async fn planners(&self) -> Value {
        let service = self.arms.config.name("query_planner_interface");
        match self
            .arms
            .robot
            .call(
                &service,
                "moveit_msgs/srv/QueryPlannerInterfaces",
                json!({}),
                SCENE_WAIT,
            )
            .await
        {
            Ok(r) => Value::Array(
                r["planner_interfaces"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .map(|p| json!({"pipeline": p["pipeline_id"], "planners": p["planner_ids"]}))
                    .collect(),
            ),
            Err(_) => Value::Null,
        }
    }
}

#[async_trait]
impl Tool for ArmStatus {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, _args: Value) -> ToolOutcome {
        let srdf = match self.arms.srdf().await {
            Ok(s) => s,
            Err(why) => return ToolOutcome::failed(why),
        };
        let (joints, scene, planners) = tokio::join!(
            self.arms.joints(),
            scene_summary(&self.arms),
            self.planners()
        );
        let joints = joints.unwrap_or_default();
        let at: BTreeMap<String, Option<String>> = arms_at(&srdf, &joints).into_iter().collect();
        let arms: Vec<Value> = srdf
            .arms()
            .iter()
            .map(|g| {
                json!({
                    "group": g.name,
                    "tip": srdf.tip(&g.name),
                    "poses": srdf.poses_of(&g.name).map(|p| p.name.clone()).collect::<Vec<_>>(),
                    "at": at.get(&g.name).cloned().flatten(),
                })
            })
            .collect();
        let hands: Vec<Value> = srdf
            .end_effectors
            .iter()
            .map(|e| {
                let at = srdf
                    .pose_at(&e.group, &joints, AT_POSE)
                    .map(|p| p.name.clone());
                json!({
                    "group": e.group,
                    "on": e.parent_group,
                    "poses": srdf.poses_of(&e.group).map(|p| p.name.clone()).collect::<Vec<_>>(),
                    "at": at,
                })
            })
            .collect();
        let scene = match scene {
            Ok(s) => json!({
                "obstacles": s.objects,
                "held": s.held.iter().map(|(o, l)| json!({"object": o, "link": l})).collect::<Vec<_>>(),
            }),
            Err(why) => json!(why),
        };
        ToolOutcome::ok(json!({
            "arms": arms,
            "hands": hands,
            "planners": planners,
            "scene": scene,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_arm_is_at_a_named_pose_or_between_them() {
        let srdf = Srdf::parse(
            r#"<robot name="r"><group name="arm"><chain base_link="b" tip_link="t"/></group>
               <group_state name="tucked" group="arm"><joint name="j" value="1.0"/></group_state></robot>"#,
        )
        .unwrap();
        let near = BTreeMap::from([("j".to_owned(), 1.02)]);
        assert_eq!(
            arms_at(&srdf, &near),
            vec![("arm".to_owned(), Some("tucked".to_owned()))]
        );
        let far = BTreeMap::from([("j".to_owned(), 0.5)]);
        assert_eq!(arms_at(&srdf, &far), vec![("arm".to_owned(), None)]);
    }
}
