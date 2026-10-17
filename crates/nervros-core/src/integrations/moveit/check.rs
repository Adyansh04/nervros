//! `arm_check`: asks MoveIt about an arm without moving it: whether it reaches something, whether
//! a named pose is clear, and the path a move would take, drawn in the 3D view.

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::{Arms, Goal, error_words, num, point_constraints};
use crate::session::Event;
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

const IK_WAIT: Duration = Duration::from_secs(5);
/// Planning may take seconds on a cluttered scene.
const PLAN_WAIT: Duration = Duration::from_secs(15);
/// How much of a preview's trajectory is drawn: this many points along it.
const DRAWN_POINTS: usize = 12;
/// A previewed move's speed, as the robot would move it: slow.
const PREVIEW_SCALING: f64 = 0.2;

/// The tool.
pub(super) struct ArmCheck {
    spec: ToolSpec,
    arms: Arc<Arms>,
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

    async fn reach(&self, args: &Value) -> ToolOutcome {
        let (_, group, planning, tip) = match self.arms.parts(args).await {
            Ok(p) => p,
            Err(why) => return ToolOutcome::failed(why),
        };
        let (at, what) = match self.arms.point(args, &planning).await {
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
                    "orientation": self.arms.tip_orientation(&planning, &tip),
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
        let (srdf, group, planning, _) = match self.arms.parts(args).await {
            Ok(p) => p,
            Err(why) => return ToolOutcome::failed(why),
        };
        let Goal::Pose(name, at) = (match self.arms.goal(args, &srdf, &group, &planning).await {
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
        let (srdf, group, planning, tip) = match self.arms.parts(args).await {
            Ok(p) => p,
            Err(why) => return ToolOutcome::failed(why),
        };
        let goal = match self.arms.goal(args, &srdf, &group, &planning).await {
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
                point_constraints(
                    &planning,
                    &tip,
                    *at,
                    &self.arms.tip_orientation(&planning, &tip),
                ),
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
