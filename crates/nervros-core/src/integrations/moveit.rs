//! MoveIt: how the robot's arms plan around what is near them.
//!
//! NervROS reads the SRDF from `move_group` for the groups, end effectors and named poses; asks
//! MoveIt's services whether an arm reaches something and what its path would be, without moving
//! it; and keeps MoveIt's planning scene, the obstacles it plans around, in step with what the
//! operator says. Previews go through the plan service, never the move action, so a preview cannot
//! disturb a motion the robot's executor runs.

mod check;
mod scene;
pub(crate) mod srdf;
mod status;

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::RobotPort;
use serde::Deserialize;
use serde_json::{Value, json};

use super::{Integration, Parts, get_params, resolve};
use crate::doctor::Check;
use crate::profile::Profile;
use crate::tools::Tool;
use srdf::Srdf;

const READ_WAIT: Duration = Duration::from_secs(3);

/// `[moveit]`: where MoveIt runs.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MoveItConfig {
    /// The `move_group` node, whose services and parameters the tools use; its services are
    /// beside it, as MoveIt names them.
    #[serde(default = "default_move_group")]
    pub move_group: String,
    /// The frame MoveIt plans in, its robot model's root; the SRDF's virtual joint gives it when
    /// there is one.
    pub planning_frame: Option<String>,
    /// The `sensor_msgs/msg/JointState` topic with the arms' joints.
    #[serde(default = "default_joint_states")]
    pub joint_states: String,
}

impl Default for MoveItConfig {
    fn default() -> Self {
        Self {
            move_group: default_move_group(),
            planning_frame: None,
            joint_states: default_joint_states(),
        }
    }
}

fn default_move_group() -> String {
    "/move_group".to_owned()
}

fn default_joint_states() -> String {
    "/joint_states".to_owned()
}

impl MoveItConfig {
    /// One of `move_group`'s services or topics, which sit in its namespace: `get_planning_scene`
    /// beside `/move_group` is `/get_planning_scene`.
    #[must_use]
    pub fn name(&self, name: &str) -> String {
        let namespace = self.move_group.rsplit_once('/').map_or("", |(ns, _)| ns);
        resolve(namespace, name)
    }
}

/// What MoveIt's tools share: the robot, where MoveIt is, and its SRDF once read.
pub(crate) struct Arms {
    config: MoveItConfig,
    profile: Profile,
    robot: Arc<dyn RobotPort>,
    srdf: Mutex<Option<Arc<Srdf>>>,
    session: OnceLock<crate::session::SessionHandle>,
}

impl std::fmt::Debug for Arms {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Arms")
            .field("move_group", &self.config.move_group)
            .finish_non_exhaustive()
    }
}

impl Arms {
    /// The SRDF, read from `move_group` the first time it is asked for.
    pub(crate) async fn srdf(&self) -> Result<Arc<Srdf>, String> {
        if let Some(s) = crate::lock(&self.srdf).clone() {
            return Ok(s);
        }
        let names = ["robot_description_semantic".to_owned()];
        let values = get_params(self.robot.as_ref(), &self.config.move_group, &names).await?;
        let text = values
            .first()
            .and_then(|(v, _)| v.as_str())
            .filter(|t| !t.is_empty())
            .ok_or_else(|| {
                format!(
                    "{} has no robot_description_semantic: is MoveIt running?",
                    self.config.move_group
                )
            })?;
        let srdf = Arc::new(Srdf::parse(text)?);
        *crate::lock(&self.srdf) = Some(Arc::clone(&srdf));
        Ok(srdf)
    }

    /// Each joint's position now.
    pub(crate) async fn joints(&self) -> Result<BTreeMap<String, f64>, String> {
        let topic = &self.config.joint_states;
        let state = self
            .robot
            .latest(topic, "sensor_msgs/msg/JointState", READ_WAIT)
            .await
            .map_err(|e| format!("no joint states on {topic}: {e}"))?;
        let names = state["name"].as_array().cloned().unwrap_or_default();
        let positions = state["position"].as_array().cloned().unwrap_or_default();
        Ok(names
            .iter()
            .zip(positions)
            .filter_map(|(n, p)| Some((n.as_str()?.to_owned(), p.as_f64()?)))
            .collect())
    }

    /// The frame MoveIt plans in.
    pub(crate) fn planning_frame(&self, srdf: &Srdf) -> Result<String, String> {
        self.config
            .planning_frame
            .clone()
            .or_else(|| srdf.root_frame.clone())
            .ok_or_else(|| {
                "the profile gives no [moveit] planning_frame and the SRDF has no virtual joint"
                    .to_owned()
            })
    }

    /// A `moveit_msgs/msg/RobotState` of these joints, over the state MoveIt keeps.
    pub(crate) fn robot_state(joints: &BTreeMap<String, f64>) -> Value {
        json!({
            "joint_state": {
                "name": joints.keys().collect::<Vec<_>>(),
                "position": joints.values().collect::<Vec<_>>(),
            },
            "is_diff": true,
        })
    }

    /// The group an operator's name means: `right_arm`, or `right` for the one arm group with it.
    pub(crate) fn group_named<'s>(srdf: &'s Srdf, wanted: &str) -> Result<&'s srdf::Group, String> {
        let wanted = wanted.trim();
        let arms = srdf.arms();
        srdf.group(wanted)
            .or_else(|| {
                let mut hits = arms.iter().filter(|g| g.name.contains(wanted));
                match (hits.next(), hits.next()) {
                    (Some(one), None) => Some(*one),
                    _ => None,
                }
            })
            .ok_or_else(|| {
                let names: Vec<&str> = arms.iter().map(|g| g.name.as_str()).collect();
                format!(
                    "MoveIt has no group `{wanted}`; its arms are {}",
                    names.join(", ")
                )
            })
    }
}

/// MoveIt's error codes in words.
pub(crate) fn error_words(code: i64) -> &'static str {
    match code {
        1 => "done",
        -1 => "no plan was found",
        -2 => "the plan was invalid",
        -3 => "something in the way moved in after planning",
        -4 => "the controllers did not follow the plan",
        -6 => "it ran out of time",
        -7 => "it was preempted",
        -10 => "the arm starts in a collision",
        -12 => "the goal is in a collision",
        -14 | -16 => "the goal cannot be met as asked",
        -15 => "no such planning group",
        -17 | -26 => "the arm's state now is not one it can plan from",
        -18 => "no such link",
        -19 => "no such object",
        -21 => "a frame could not be placed (TF)",
        -31 => "no arm pose reaches it",
        _ => "MoveIt gave up",
    }
}

/// The MoveIt integration.
pub struct MoveIt {
    arms: Arc<Arms>,
    watcher: Arc<status::Watcher>,
    tools: Vec<Arc<dyn Tool>>,
}

impl std::fmt::Debug for MoveIt {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MoveIt")
            .field("move_group", &self.arms.config.move_group)
            .finish_non_exhaustive()
    }
}

impl MoveIt {
    /// The integration over the robot's `move_group`.
    pub(crate) fn start(config: &MoveItConfig, parts: &Parts<'_>) -> Arc<Self> {
        let arms = Arc::new(Arms {
            config: config.clone(),
            profile: parts.profile.clone(),
            robot: Arc::clone(parts.robot),
            srdf: Mutex::new(None),
            session: OnceLock::new(),
        });
        let watcher = status::Watcher::start(Arc::clone(&arms));
        let tools: Vec<Arc<dyn Tool>> = vec![
            Arc::new(status::ArmStatus::new(Arc::clone(&arms))),
            Arc::new(check::ArmCheck::new(Arc::clone(&arms))),
            scene::Scene::new(Arc::clone(&arms), parts.file("planning_scene")),
        ];
        Arc::new(Self {
            arms,
            watcher,
            tools,
        })
    }
}

#[async_trait]
impl Integration for MoveIt {
    fn id(&self) -> &'static str {
        "moveit"
    }

    fn tools(&self) -> Vec<Arc<dyn Tool>> {
        self.tools.clone()
    }

    fn status(&self) -> String {
        self.watcher.line()
    }

    fn attach(&self, session: &crate::session::SessionHandle) {
        let _ = self.arms.session.set(session.clone());
    }

    async fn stop(&self) {
        // MoveIt stops any trajectory it executes, whoever sent it; the hands keep their grip.
        let topic = self.arms.config.name("trajectory_execution_event");
        let stop = json!({"data": "stop"});
        if let Err(e) = self
            .arms
            .robot
            .publish(&topic, "std_msgs/msg/String", stop, 1, Duration::ZERO)
            .await
        {
            tracing::warn!(%topic, error = %e, "MoveIt was not told to stop");
        }
    }
}

/// Whether `move_group` runs.
pub(crate) async fn discover(config: &MoveItConfig, robot: &dyn RobotPort) -> Vec<Check> {
    let found = robot
        .graph_detail()
        .await
        .map(|g| g.nodes.contains(&config.move_group));
    vec![match found {
        Ok(found) => Check {
            ok: found,
            what: format!("MoveIt {}", config.move_group),
        },
        Err(e) => Check {
            ok: false,
            what: format!("MoveIt: reading the ROS graph: {e}"),
        },
    }]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn move_groups_services_sit_beside_it() {
        let root = MoveItConfig::default();
        assert_eq!(root.name("get_planning_scene"), "/get_planning_scene");
        let spaced = MoveItConfig {
            move_group: "/arm/move_group".into(),
            ..MoveItConfig::default()
        };
        assert_eq!(spaced.name("compute_ik"), "/arm/compute_ik");
    }

    #[test]
    fn an_arm_is_named_by_its_group_or_its_side() {
        let srdf = Srdf::parse(
            r#"<robot name="r"><group name="left_arm"><chain base_link="t" tip_link="l"/></group>
               <group name="right_arm"><chain base_link="t" tip_link="r"/></group></robot>"#,
        )
        .unwrap();
        assert_eq!(Arms::group_named(&srdf, "right").unwrap().name, "right_arm");
        assert_eq!(
            Arms::group_named(&srdf, "left_arm").unwrap().name,
            "left_arm"
        );
        let none = Arms::group_named(&srdf, "arm").unwrap_err();
        assert!(none.contains("left_arm, right_arm"), "{none}");
    }
}
