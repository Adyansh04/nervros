//! `arm_check`: asks MoveIt about an arm without moving it: whether it reaches something, whether
//! a named pose is clear, and the path a move would take, drawn in the 3D view.

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::srdf::Srdf;
use super::{Arms, error_words};
use crate::session::Event;
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

const IK_WAIT: Duration = Duration::from_secs(5);
/// Planning may take seconds on a cluttered scene.
const PLAN_WAIT: Duration = Duration::from_secs(15);
/// How much of a preview's trajectory is drawn: this many points along it.
const DRAWN_POINTS: usize = 12;
/// A previewed move's speed, as the robot would move it: slow.
const PREVIEW_SCALING: f64 = 0.2;
/// How far a reached point may be from the goal, m, and the hand's turn from now, rad.
const POSITION_TOLERANCE: f64 = 0.01;
const ORIENTATION_TOLERANCE: f64 = 0.1;

/// The tool.
pub(super) struct ArmCheck {
    spec: ToolSpec,
    arms: Arc<Arms>,
}

/// Where a move goes: a named pose, or a point for the arm's tip.
enum Goal {
    Pose(String, BTreeMap<String, f64>),
    Point([f64; 3], String),
}

impl ArmCheck {
    pub(super) fn new(arms: Arc<Arms>) -> Self {
        let spec = ToolSpec::new(
            "arm_check",
            "Asks MoveIt about an arm without moving it. reach: whether the arm's hand reaches an \
             object (its id) or a point (x, y, z in frame, the map by default), held as it is now. \
             valid: whether a named pose of the arm is clear of collisions now. preview: the path \
             the arm would take to a named pose, an object or a point, how long it takes, drawn \
             in the 3D view. group is the arm, as arm_status names it, or its side.",
            json!({
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["reach", "valid", "preview"]},
                    "group": {"type": "string", "description": "The arm: right_arm, or right"},
                    "pose": {"type": "string", "description": "A named pose of the arm, from arm_status"},
                    "object": {"type": "string", "description": "An object's id from find_objects"},
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                    "frame": {"type": "string", "description": "The frame x, y and z are in; the map's by default"},
                },
                "required": ["action", "group"],
            }),
            Risk::Observe,
        );
        Self { spec, arms }
    }

    /// The point `args` name, in the planning frame.
    async fn point(&self, args: &Value, planning: &str) -> Result<([f64; 3], String), String> {
        let map = &self.arms.profile.ros.map_frame;
        let (at, frame, what) = if let Some(id) = args["object"].as_str().filter(|s| !s.is_empty())
        {
            let topic = self
                .arms
                .profile
                .world
                .as_ref()
                .and_then(|w| w.objects.clone())
                .ok_or("this robot has no world model with objects")?;
            let objects = self
                .arms
                .robot
                .latest(&topic.topic, &topic.msg_type, IK_WAIT)
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
            (
                [num(&p["x"]), num(&p["y"]), num(&p["z"])],
                map.clone(),
                label,
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
            .arms
            .robot
            .transform(planning, &frame)
            .map_err(|e| format!("cannot place {frame} in {planning}: {e}"))?;
        Ok((t.apply(at), what))
    }

    /// The goal `args` name for the group.
    async fn goal(
        &self,
        args: &Value,
        srdf: &Srdf,
        group: &str,
        planning: &str,
    ) -> Result<Goal, String> {
        if let Some(name) = args["pose"].as_str().filter(|s| !s.is_empty()) {
            let pose = srdf
                .poses_of(group)
                .find(|p| p.name == name)
                .ok_or_else(|| {
                    let names: Vec<&str> = srdf.poses_of(group).map(|p| p.name.as_str()).collect();
                    format!("{group} has no pose `{name}`; it has {}", names.join(", "))
                })?;
            return Ok(Goal::Pose(pose.name.clone(), pose.joints.clone()));
        }
        let (at, what) = self.point(args, planning).await?;
        Ok(Goal::Point(at, what))
    }

    /// The hand's orientation now, in the planning frame.
    fn tip_orientation(&self, planning: &str, tip: &str) -> Value {
        match self.arms.robot.transform(planning, tip) {
            Ok(tip_now) => {
                let [qx, qy, qz, qw] = tip_now.rotation;
                json!({"x": qx, "y": qy, "z": qz, "w": qw})
            }
            Err(_) => json!({"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}),
        }
    }

    async fn reach(&self, args: &Value) -> ToolOutcome {
        let (_, group, planning, tip) = match self.parts(args).await {
            Ok(p) => p,
            Err(why) => return ToolOutcome::failed(why),
        };
        let (at, what) = match self.point(args, &planning).await {
            Ok(p) => p,
            Err(why) => return ToolOutcome::failed(why),
        };
        let joints = self.arms.joints().await.unwrap_or_default();
        let request = json!({"ik_request": {
            "group_name": group,
            "robot_state": Arms::robot_state(&joints),
            "avoid_collisions": true,
            "ik_link_name": tip,
            "pose_stamped": {
                "header": {"frame_id": planning},
                "pose": {
                    "position": {"x": at[0], "y": at[1], "z": at[2]},
                    "orientation": self.tip_orientation(&planning, &tip),
                },
            },
            "timeout": {"sec": 1, "nanosec": 0},
        }});
        let service = self.arms.config.name("compute_ik");
        let answer = match self
            .arms
            .robot
            .call(&service, "moveit_msgs/srv/GetPositionIK", request, IK_WAIT)
            .await
        {
            Ok(a) => a,
            Err(e) => return ToolOutcome::failed(format!("MoveIt's IK did not answer: {e}")),
        };
        let code = answer["error_code"]["val"].as_i64().unwrap_or(0);
        if code != 1 {
            return ToolOutcome::ok(json!({
                "reaches": false,
                "group": group,
                "to": what,
                "why": error_words(code),
            }));
        }
        let solved = &answer["solution"]["joint_state"];
        let travel = travel(&joints, solved);
        ToolOutcome::ok(json!({
            "reaches": true,
            "group": group,
            "to": what,
            "joint_travel_rad": (travel * 100.0).round() / 100.0,
        }))
    }

    async fn valid(&self, args: &Value) -> ToolOutcome {
        let (srdf, group, planning, _) = match self.parts(args).await {
            Ok(p) => p,
            Err(why) => return ToolOutcome::failed(why),
        };
        let Goal::Pose(name, at) = (match self.goal(args, &srdf, &group, &planning).await {
            Ok(g) => g,
            Err(why) => return ToolOutcome::failed(why),
        }) else {
            return ToolOutcome::failed("valid checks a named pose: give pose");
        };
        let mut joints = self.arms.joints().await.unwrap_or_default();
        joints.extend(at);
        let request = json!({
            "robot_state": Arms::robot_state(&joints),
            "group_name": group,
        });
        let service = self.arms.config.name("check_state_validity");
        match self
            .arms
            .robot
            .call(
                &service,
                "moveit_msgs/srv/GetStateValidity",
                request,
                IK_WAIT,
            )
            .await
        {
            Ok(r) => {
                let contacts: Vec<String> = r["contacts"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .map(|c| {
                        format!(
                            "{} and {}",
                            c["contact_body_1"].as_str().unwrap_or("?"),
                            c["contact_body_2"].as_str().unwrap_or("?")
                        )
                    })
                    .collect();
                ToolOutcome::ok(json!({
                    "group": group,
                    "pose": name,
                    "clear": r["valid"].as_bool() == Some(true),
                    "touching": contacts,
                }))
            }
            Err(e) => ToolOutcome::failed(format!("MoveIt's state check did not answer: {e}")),
        }
    }

    async fn preview(&self, args: &Value) -> ToolOutcome {
        let (srdf, group, planning, tip) = match self.parts(args).await {
            Ok(p) => p,
            Err(why) => return ToolOutcome::failed(why),
        };
        let goal = match self.goal(args, &srdf, &group, &planning).await {
            Ok(g) => g,
            Err(why) => return ToolOutcome::failed(why),
        };
        let joints = self.arms.joints().await.unwrap_or_default();
        let (constraints, to) = match &goal {
            Goal::Pose(name, at) => (
                json!({"joint_constraints": at.iter().map(|(j, v)| json!({
                    "joint_name": j, "position": v,
                    "tolerance_above": 0.01, "tolerance_below": 0.01, "weight": 1.0,
                })).collect::<Vec<_>>()}),
                format!("`{name}`"),
            ),
            Goal::Point(at, what) => (
                point_constraints(&planning, &tip, *at, &self.tip_orientation(&planning, &tip)),
                what.clone(),
            ),
        };
        let request = json!({"motion_plan_request": {
            "group_name": group,
            "start_state": Arms::robot_state(&joints),
            "goal_constraints": [constraints],
            "num_planning_attempts": 1,
            "allowed_planning_time": 5.0,
            "max_velocity_scaling_factor": PREVIEW_SCALING,
            "max_acceleration_scaling_factor": PREVIEW_SCALING,
        }});
        let service = self.arms.config.name("plan_kinematic_path");
        let planned = match self
            .arms
            .robot
            .call(
                &service,
                "moveit_msgs/srv/GetMotionPlan",
                request,
                PLAN_WAIT,
            )
            .await
        {
            Ok(p) => p,
            Err(e) => return ToolOutcome::failed(format!("MoveIt's planner did not answer: {e}")),
        };
        let response = &planned["motion_plan_response"];
        let code = response["error_code"]["val"].as_i64().unwrap_or(0);
        if code != 1 {
            return ToolOutcome::ok(json!({
                "planned": false,
                "group": group,
                "to": to,
                "why": error_words(code),
            }));
        }
        let trajectory = &response["trajectory"]["joint_trajectory"];
        let names: Vec<String> = trajectory["joint_names"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|n| n.as_str().map(str::to_owned))
            .collect();
        let points = trajectory["points"].as_array().cloned().unwrap_or_default();
        let duration = points.last().map_or(0.0, |p| secs(&p["time_from_start"]));
        let drawn = self
            .draw(
                &planning,
                &tip,
                &joints,
                &names,
                &points,
                &format!("{group} to {to}"),
            )
            .await;
        ToolOutcome::ok(json!({
            "planned": true,
            "group": group,
            "to": to,
            "takes_s": (duration * 10.0).round() / 10.0,
            "planning_s": (response["planning_time"].as_f64().unwrap_or(0.0) * 100.0).round() / 100.0,
            "drawn": drawn,
        }))
    }

    /// Draws the hand's path along a planned trajectory, from MoveIt's own kinematics at a few of
    /// its points; whether it could.
    async fn draw(
        &self,
        planning: &str,
        tip: &str,
        joints: &BTreeMap<String, f64>,
        names: &[String],
        points: &[Value],
        label: &str,
    ) -> bool {
        let Some(session) = self.arms.session.get() else {
            return false;
        };
        let step = points.len().div_ceil(DRAWN_POINTS).max(1);
        let service = self.arms.config.name("compute_fk");
        let mut path = Vec::new();
        for p in points.iter().step_by(step).chain(points.last()) {
            let mut at = joints.clone();
            for (name, v) in names
                .iter()
                .zip(p["positions"].as_array().into_iter().flatten())
            {
                if let Some(v) = v.as_f64() {
                    at.insert(name.clone(), v);
                }
            }
            let request = json!({
                "header": {"frame_id": planning},
                "fk_link_names": [tip],
                "robot_state": Arms::robot_state(&at),
            });
            let Ok(r) = self
                .arms
                .robot
                .call(&service, "moveit_msgs/srv/GetPositionFK", request, IK_WAIT)
                .await
            else {
                return false;
            };
            let q = &r["pose_stamped"][0]["pose"]["position"];
            path.push([num(&q["x"]), num(&q["y"]), num(&q["z"])]);
        }
        session.emit(Event::Drawn {
            layer: "arm_preview".to_owned(),
            frame: planning.to_owned(),
            points: path,
            label: label.to_owned(),
        });
        true
    }

    /// The SRDF, the group `args` name, the planning frame and the group's tip link.
    async fn parts(&self, args: &Value) -> Result<(Arc<Srdf>, String, String, String), String> {
        let srdf = self.arms.srdf().await?;
        let group = Arms::group_named(&srdf, args["group"].as_str().unwrap_or_default())?
            .name
            .clone();
        let planning = self.arms.planning_frame(&srdf)?;
        let tip = srdf
            .tip(&group)
            .ok_or_else(|| format!("the SRDF gives {group} no tip link"))?;
        Ok((srdf, group, planning, tip))
    }
}

#[async_trait]
impl Tool for ArmCheck {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str().unwrap_or_default() {
            "reach" => self.reach(&args).await,
            "valid" => self.valid(&args).await,
            "preview" => self.preview(&args).await,
            other => ToolOutcome::failed(format!(
                "action `{other}` is not one of reach, valid, preview"
            )),
        }
    }
}

/// Goal constraints for the tip at a point, held as it is now: a small sphere for the position,
/// and the orientation within a tolerance.
fn point_constraints(planning: &str, tip: &str, at: [f64; 3], orientation: &Value) -> Value {
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

/// How far the joints a solution names would move from where they are now, summed, rad.
fn travel(now: &BTreeMap<String, f64>, solved: &Value) -> f64 {
    let names = solved["name"].as_array().cloned().unwrap_or_default();
    let positions = solved["position"].as_array().cloned().unwrap_or_default();
    names
        .iter()
        .zip(positions)
        .filter_map(|(n, p)| {
            let (n, p) = (n.as_str()?, p.as_f64()?);
            now.get(n).map(|q| (p - q).abs())
        })
        .sum()
}

fn secs(d: &Value) -> f64 {
    num(&d["sec"]) + num(&d["nanosec"]) / 1e9
}

fn num(v: &Value) -> f64 {
    v.as_f64().unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_solutions_travel_counts_only_the_joints_it_moves() {
        let now = BTreeMap::from([("a".to_owned(), 0.0), ("b".to_owned(), 1.0)]);
        let solved = json!({"name": ["a", "b", "c"], "position": [0.5, 0.75, 9.0]});
        assert!((travel(&now, &solved) - 0.75).abs() < 1e-9);
        assert!((secs(&json!({"sec": 2, "nanosec": 500_000_000})) - 2.5).abs() < 1e-9);
    }
}
