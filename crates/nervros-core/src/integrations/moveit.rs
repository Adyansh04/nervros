//! MoveIt: how the robot's arms plan around what is near them.
//!
//! NervROS reads the SRDF from `move_group` for the groups, end effectors and named poses; asks
//! MoveIt's services whether an arm reaches something and what its path would be, without moving
//! it; and keeps MoveIt's planning scene, the obstacles it plans around, in step with what the
//! operator says. Previews go through the plan service, never the move action, so a preview cannot
//! disturb a motion the robot's executor runs.

mod check;
mod motion;
mod scene;
mod settings;
pub(crate) mod srdf;
mod status;

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::RobotPort;
use serde::Deserialize;
use serde_json::{Value, json};

use super::{Goals, Integration, NativeSkill, Parts, get_params, resolve};
use crate::doctor::Check;
use crate::profile::Profile;
use crate::tools::Tool;
use srdf::Srdf;

const READ_WAIT: Duration = Duration::from_secs(3);
/// How far above an object's top a hand goes to it, m.
const ABOVE_M: f64 = 0.10;
/// How much of a straight move must be clear for it to run.
const CLEAR_SHARE: f64 = 0.98;
/// How finely a straight move is checked, m.
const LINE_STEP_M: f64 = 0.01;
/// The most a joint may move between two of a straight line's waypoints, rad and m.
const LINE_JUMP_RAD: f64 = 0.5;
const LINE_JUMP_M: f64 = 0.05;
/// How long the straight-line planner may take.
const LINE_WAIT: Duration = Duration::from_secs(10);

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
    /// The share of the arms' top speed NervROS's own arm moves start at.
    #[serde(default = "default_scaling")]
    pub scaling: f64,
    /// The most the operator may raise that share to.
    #[serde(default = "default_max_scaling")]
    pub max_scaling: f64,
}

fn default_scaling() -> f64 {
    0.2
}

fn default_max_scaling() -> f64 {
    0.5
}

impl Default for MoveItConfig {
    fn default() -> Self {
        Self {
            move_group: default_move_group(),
            planning_frame: None,
            joint_states: default_joint_states(),
            scaling: default_scaling(),
            max_scaling: default_max_scaling(),
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
    settings: Mutex<settings::Settings>,
    /// Where the settings are kept, if anywhere.
    settings_file: Option<std::path::PathBuf>,
    /// Held from a change to its write, so the file is written in the order of the changes.
    writing: tokio::sync::Mutex<()>,
}

impl std::fmt::Debug for Arms {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Arms")
            .field("move_group", &self.config.move_group)
            .finish_non_exhaustive()
    }
}

impl Arms {
    /// The settings NervROS's own arm moves use now.
    pub(crate) fn settings(&self) -> settings::Settings {
        crate::lock(&self.settings).clone()
    }

    /// Changes the settings and keeps them, written off the async threads.
    pub(crate) async fn change(&self, f: impl FnOnce(&mut settings::Settings)) {
        let _writing = self.writing.lock().await;
        let now = {
            let mut s = crate::lock(&self.settings);
            f(&mut s);
            s.clone()
        };
        let Some(file) = self.settings_file.clone() else {
            return;
        };
        let written =
            tokio::task::spawn_blocking(move || crate::persist::write_json(&file, &now)).await;
        if let Some(e) = written.map_or_else(|e| Some(e.to_string()), Result::err) {
            tracing::warn!(error = %e, "the arm settings were not kept");
        }
    }

    /// The SRDF, read from `move_group` the first time it is asked for.
    pub(crate) async fn srdf(&self) -> Result<Arc<Srdf>, String> {
        if let Some(s) = crate::lock(&self.srdf).clone() {
            return Ok(s);
        }
        let names = [
            "robot_description_semantic".to_owned(),
            "robot_description".to_owned(),
        ];
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
        let urdf = values
            .get(1)
            .and_then(|(v, _)| v.as_str())
            .unwrap_or_default();
        let srdf = Arc::new(Srdf::parse(text)?.with_urdf(urdf));
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

    /// What the hand of `group` holds by the executor's state: an object id,
    /// [`crate::mission::UNKNOWN_HELD`], or empty for nothing. Empty where no executor holds the
    /// arms.
    ///
    /// # Errors
    ///
    /// The executor's state is not fresh.
    pub(crate) async fn holding(&self, group: &str) -> Result<String, String> {
        let Some(mission) = &self.profile.mission else {
            return Ok(String::new());
        };
        let side = if group.contains("left") {
            "left"
        } else {
            "right"
        };
        // Fresh only: a stale "nothing held" from before a pick would let a full hand open.
        let state = self
            .robot
            .latest_fresh(
                &mission.state,
                "nervros_interfaces/msg/RobotState",
                Duration::from_secs(1),
                crate::mission::STATE_FRESH,
            )
            .await
            .map_err(|e| format!("what the {side} hand holds is not known: {e}"))?;
        Ok(crate::mission::held_by(&state, side))
    }

    /// The path constraint a hand of `group` moves under, turned as `turned`: what it holds stays
    /// as level as it is now the whole way, as a full mug would spill. Unknown counts as held:
    /// the hand is kept level unless it is known to be empty.
    pub(crate) async fn hand_path(
        &self,
        group: &str,
        planning: &str,
        tip: &str,
        turned: &Value,
    ) -> Option<Value> {
        let held = !self.holding(group).await.is_ok_and(|h| h.is_empty());
        held.then(|| motion::held_level(planning, tip, turned))
    }

    /// A motion plan request for `group` to `constraints`, under `path` when given, from `start`:
    /// the planner, scaling and time the arm moves with, so a check asks what the move will do.
    pub(crate) fn plan_request(
        &self,
        group: &str,
        constraints: &Value,
        path: Option<Value>,
        start: &Value,
    ) -> Value {
        let s = self.settings();
        let planning_s = if path.is_some() {
            s.planning_s.max(motion::HELD_PLANNING_S)
        } else {
            s.planning_s
        };
        json!({
            "group_name": group,
            "start_state": start,
            "goal_constraints": [constraints],
            "path_constraints": path.unwrap_or_else(|| json!({})),
            "pipeline_id": s.pipeline,
            "planner_id": s.planner,
            "num_planning_attempts": 1,
            "allowed_planning_time": planning_s,
            "max_velocity_scaling_factor": s.scaling,
            "max_acceleration_scaling_factor": s.scaling,
        })
    }

    /// The point `args` name, in the planning frame. For an object it is where a hand goes to
    /// it: [`ABOVE_M`] over its top, as the object itself is in the way.
    pub(crate) async fn point(
        &self,
        args: &Value,
        planning: &str,
    ) -> Result<([f64; 3], String), String> {
        let map = &self.profile.ros.map_frame;
        let (at, frame, what) = if let Some(id) = args["object"].as_str().filter(|s| !s.is_empty())
        {
            let topic = self
                .profile
                .world
                .as_ref()
                .and_then(|w| w.objects.clone())
                .ok_or("this robot has no world model with objects")?;
            let objects = self
                .robot
                .latest(&topic.topic, &topic.msg_type, READ_WAIT)
                .await
                .map_err(|e| format!("the world model did not answer: {e}"))?;
            let found = objects["objects"]
                .as_array()
                .into_iter()
                .flatten()
                .find(|o| o["id"].as_str() == Some(id))
                .ok_or_else(|| format!("the world model has no object `{id}`"))?;
            let p = &found["pose"]["position"];
            let label = found["label"].as_str().unwrap_or(id).to_owned();
            // Its centre is inside it: a tall bottle's would put the hand in the bottle.
            let top = num(&p["z"]) + found["size"]["z"].as_f64().unwrap_or(0.0).max(0.0) / 2.0;
            (
                [num(&p["x"]), num(&p["y"]), top + ABOVE_M],
                map.clone(),
                format!("above {label}"),
            )
        } else {
            let (Some(x), Some(y), Some(z)) =
                (args["x"].as_f64(), args["y"].as_f64(), args["z"].as_f64())
            else {
                return Err("say where: an object, or x, y and z".to_owned());
            };
            let frame = args["frame"].as_str().unwrap_or(map).to_owned();
            ([x, y, z], frame, format!("({x:.2}, {y:.2}, {z:.2})"))
        };
        let t = self
            .robot
            .transform(planning, &frame)
            .map_err(|e| format!("cannot place {frame} in {planning}: {e}"))?;
        Ok((t.apply(at), what))
    }

    /// The goal `args` name for the group.
    pub(crate) async fn goal(
        &self,
        args: &Value,
        srdf: &Srdf,
        group: &str,
        planning: &str,
    ) -> Result<Goal, String> {
        if let Some(name) = args["pose"].as_str().filter(|s| !s.is_empty()) {
            if let Some(pose) = srdf.poses_of(group).find(|p| p.name == name) {
                return Ok(Goal::Pose(pose.name.clone(), pose.joints.clone()));
            }
            let kept = self
                .settings()
                .poses
                .get(group)
                .cloned()
                .unwrap_or_default();
            if let Some(joints) = kept.get(name) {
                return Ok(Goal::Pose(name.to_owned(), joints.clone()));
            }
            let names: Vec<&str> = srdf
                .poses_of(group)
                .map(|p| p.name.as_str())
                .chain(kept.keys().map(String::as_str))
                .collect();
            return Err(format!(
                "{group} has no pose `{name}`; it has {}",
                names.join(", ")
            ));
        }
        let (at, what) = self.point(args, planning).await?;
        Ok(Goal::Point(at, what))
    }

    /// The hand's orientation now, in the planning frame.
    pub(crate) fn tip_orientation(&self, planning: &str, tip: &str) -> Result<Value, String> {
        // Unknown is not upright: a made-up turn would swing the hand, and what it holds.
        let tip_now = self.robot.transform(planning, tip).map_err(|e| {
            format!("how the hand {tip} is turned is not known ({e}), so it is not moved")
        })?;
        let [qx, qy, qz, qw] = tip_now.rotation;
        Ok(json!({"x": qx, "y": qy, "z": qz, "w": qw}))
    }

    /// MoveIt's answer for the hand of `group` going straight by `along` in the planning frame:
    /// how much of the way is clear (`fraction`), and the trajectory for it (`solution`).
    pub(crate) async fn straight(&self, group: &str, along: [f64; 3]) -> Result<Value, String> {
        let srdf = self.srdf().await?;
        let planning = self.planning_frame(&srdf)?;
        let tip = srdf
            .tip(group)
            .ok_or_else(|| format!("the SRDF gives {group} no tip link"))?;
        let now = self
            .robot
            .transform(&planning, &tip)
            .map_err(|e| format!("where the hand is: {e}"))?;
        let [x, y, z] = now.translation;
        let [qx, qy, qz, qw] = now.rotation;
        let s = self.settings();
        let request = json!({
            "header": {"frame_id": planning},
            "start_state": {"is_diff": true},
            "group_name": group,
            "link_name": tip,
            "waypoints": [{
                "position": {"x": x + along[0], "y": y + along[1], "z": z + along[2]},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            }],
            "max_step": LINE_STEP_M,
            "jump_threshold": 0.0,
            // A joint that swings this far between two waypoints a centimetre apart is a flip
            // of the arm's solution, not a straight line: the path ends there.
            "revolute_jump_threshold": LINE_JUMP_RAD,
            "prismatic_jump_threshold": LINE_JUMP_M,
            "avoid_collisions": true,
            "max_velocity_scaling_factor": s.scaling,
            "max_acceleration_scaling_factor": s.scaling,
        });
        let service = self.config.name("compute_cartesian_path");
        self.robot
            .call(
                &service,
                "moveit_msgs/srv/GetCartesianPath",
                request,
                LINE_WAIT,
            )
            .await
            .map_err(|e| format!("MoveIt's straight-line planner did not answer: {e}"))
    }

    /// The SRDF, the group `args` name, the planning frame and the group's tip link.
    pub(crate) async fn parts(
        &self,
        args: &Value,
    ) -> Result<(Arc<Srdf>, String, String, String), String> {
        let srdf = self.srdf().await?;
        let group = Arms::group_named(&srdf, args["group"].as_str().unwrap_or_default())?
            .name
            .clone();
        let planning = self.planning_frame(&srdf)?;
        let tip = srdf
            .tip(&group)
            .ok_or_else(|| format!("the SRDF gives {group} no tip link"))?;
        Ok((srdf, group, planning, tip))
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

/// Where a move goes: a named pose, or a point for the arm's tip.
pub(crate) enum Goal {
    Pose(String, BTreeMap<String, f64>),
    Point([f64; 3], String),
}

/// How far a reached point may be from the goal, m, and the hand's turn from now, rad.
const POSITION_TOLERANCE: f64 = 0.01;
const ORIENTATION_TOLERANCE: f64 = 0.1;

/// Constraints that put `tip` at `at` in `planning`, turned as `orientation`.
pub(crate) fn point_constraints(
    planning: &str,
    tip: &str,
    at: [f64; 3],
    orientation: &Value,
) -> Value {
    json!({
        "position_constraints": [{
            "header": {"frame_id": planning},
            "link_name": tip,
            "constraint_region": {
                "primitives": [{"type": 2, "dimensions": [POSITION_TOLERANCE]}],
                "primitive_poses": [{"position": {"x": at[0], "y": at[1], "z": at[2]}, "orientation": {"w": 1.0}}],
            },
            "weight": 1.0,
        }],
        "orientation_constraints": [{
            "header": {"frame_id": planning},
            "link_name": tip,
            "orientation": orientation,
            "absolute_x_axis_tolerance": ORIENTATION_TOLERANCE,
            "absolute_y_axis_tolerance": ORIENTATION_TOLERANCE,
            "absolute_z_axis_tolerance": ORIENTATION_TOLERANCE,
            "weight": 1.0,
        }],
    })
}

pub(crate) fn num(v: &Value) -> f64 {
    v.as_f64().unwrap_or(0.0)
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

/// The procedures MoveIt's faults call for.
pub(crate) const PROCEDURES: [&str; 1] = [include_str!("moveit/procedures/arm-cannot-reach.md")];

/// The MoveIt integration.
pub struct MoveIt {
    arms: Arc<Arms>,
    watcher: Arc<status::Watcher>,
    tools: Vec<Arc<dyn Tool>>,
    natives: Vec<Arc<dyn NativeSkill>>,
    /// The stop's publisher, made at start so it is matched long before a stop needs it.
    stop_event: Arc<tokio::sync::OnceCell<nervros_ros::Publisher>>,
}

/// MoveIt's execution events, where "stop" ends whatever trajectory it executes.
const EXECUTION_EVENT: &str = "std_msgs/msg/String";

impl std::fmt::Debug for MoveIt {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MoveIt")
            .field("move_group", &self.arms.config.move_group)
            .finish_non_exhaustive()
    }
}

impl MoveIt {
    /// The integration over the robot's `move_group`.
    pub(crate) fn start(config: &MoveItConfig, parts: &Parts<'_>, goals: &Arc<Goals>) -> Arc<Self> {
        let settings_file = parts.file("arm_settings");
        let mut kept: settings::Settings = settings_file
            .as_deref()
            .map(crate::persist::read_or_default)
            .unwrap_or_default();
        if !settings_file
            .as_deref()
            .is_some_and(std::path::Path::exists)
        {
            kept.scaling = config.scaling;
        }
        // A hand-edited file's scaling outside (0, max] is MoveIt's default speed, not a choice.
        if !(kept.scaling.is_finite() && kept.scaling > 0.0) {
            kept.scaling = config.scaling;
        }
        kept.scaling = kept.scaling.min(config.max_scaling);
        let arms = Arc::new(Arms {
            config: config.clone(),
            profile: parts.profile.clone(),
            robot: Arc::clone(parts.robot),
            srdf: Mutex::new(None),
            session: OnceLock::new(),
            settings: Mutex::new(kept),
            settings_file,
            writing: tokio::sync::Mutex::new(()),
        });
        let watcher = status::Watcher::start(Arc::clone(&arms));
        let tools: Vec<Arc<dyn Tool>> = vec![
            Arc::new(status::ArmStatus::new(Arc::clone(&arms))),
            Arc::new(check::ArmCheck::new(Arc::clone(&arms))),
            scene::Scene::new(Arc::clone(&arms), parts.file("planning_scene")),
            Arc::new(settings::ArmSettings::new(Arc::clone(&arms))),
        ];
        let natives = motion::Motions::natives(&arms, goals);
        let stop_event = Arc::new(tokio::sync::OnceCell::new());
        if let Ok(runtime) = tokio::runtime::Handle::try_current() {
            let (cell, arms) = (Arc::clone(&stop_event), Arc::clone(&arms));
            // Detached: it only warms the publisher up, and a stop makes one itself otherwise.
            runtime.spawn(async move {
                let topic = arms.config.name("trajectory_execution_event");
                if let Err(e) = cell
                    .get_or_try_init(|| arms.robot.publisher(&topic, EXECUTION_EVENT))
                    .await
                {
                    tracing::warn!(%topic, error = %e, "no kept publisher for MoveIt's stop");
                }
            });
        }
        Arc::new(Self {
            arms,
            watcher,
            tools,
            natives,
            stop_event,
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

    fn status(&self) -> Vec<String> {
        self.watcher.parts()
    }

    fn attach(&self, session: &crate::session::SessionHandle) {
        let _ = self.arms.session.set(session.clone());
    }

    fn natives(&self) -> Vec<Arc<dyn NativeSkill>> {
        self.natives.clone()
    }

    async fn arm_poses(&self) -> BTreeMap<String, String> {
        let (Ok(srdf), Ok(joints)) = (self.arms.srdf().await, self.arms.joints().await) else {
            return BTreeMap::new();
        };
        status::arms_at(&srdf, &joints)
            .into_iter()
            .filter_map(|(arm, pose)| Some((arm, pose?)))
            .collect()
    }

    async fn stop(&self) -> Vec<String> {
        // MoveIt stops any trajectory it executes, whoever sent it; the hands keep their grip.
        let topic = self.arms.config.name("trajectory_execution_event");
        let stop = json!({"data": "stop"});
        if let Some(publisher) = self.stop_event.get() {
            publisher.send(stop);
            return Vec::new();
        }
        // No kept publisher: a one-shot one, which a new subscriber may not hear in time.
        match self
            .arms
            .robot
            .publish(&topic, EXECUTION_EVENT, stop, 1, Duration::ZERO)
            .await
        {
            Ok(0) => vec![format!(
                "MoveIt: nothing subscribes to {topic}, so it may not have heard the stop"
            )],
            Ok(_) => Vec::new(),
            Err(e) => vec![format!(
                "MoveIt: the stop could not be sent on {topic}: {e}"
            )],
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
