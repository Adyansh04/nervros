//! MoveIt's arm motions as mission steps NervROS runs itself, for a robot whose executor has no
//! skill of the name: an arm to a named pose, the hand to an object or a point, a straight hand
//! move and a grip. An arm move runs to its end through a pause, as a pick does. Where the
//! robot's executor holds the arms, it lends them for the step.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::{GoalResult, GoalStatus};
use serde_json::{Value, json};

use super::{Arms, CLEAR_SHARE, Goal, error_words, point_constraints};
use crate::builtins::triple;
use crate::integrations::{Control, Goals, NativeSkill, run_goal};

const ARM_S: f64 = 60.0;
const CALL_WAIT: Duration = Duration::from_secs(10);
/// How far a held thing may tilt on the way, radians.
const HELD_TILT: f64 = 0.2;
/// Constrained planning needs longer than a free move, s.
pub(super) const HELD_PLANNING_S: f64 = 10.0;

/// The motions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    ArmTo,
    HandTo,
    HandLine,
    Grip,
}

/// What the motions share.
pub(super) struct Motions {
    arms: Arc<Arms>,
    goals: Arc<Goals>,
    /// The executor's `LendResources`, when it holds the arms.
    lend: Option<String>,
    /// Held while the arms go back, so a retry does not borrow them before a late give-back.
    returning: Arc<tokio::sync::Mutex<()>>,
}

impl Motions {
    pub(super) fn natives(arms: &Arc<Arms>, goals: &Arc<Goals>) -> Vec<Arc<dyn NativeSkill>> {
        let mission = arms.profile.mission.as_ref();
        let shared = Arc::new(Self {
            arms: Arc::clone(arms),
            goals: Arc::clone(goals),
            lend: mission.and_then(|m| m.lend.clone()),
            returning: Arc::default(),
        });
        [Kind::ArmTo, Kind::HandTo, Kind::HandLine, Kind::Grip]
            .into_iter()
            .map(|kind| {
                Arc::new(Native {
                    kind,
                    shared: Arc::clone(&shared),
                }) as Arc<dyn NativeSkill>
            })
            .collect()
    }

    /// Plans and moves `group` to `constraints` through `move_group`, within `path` all the way
    /// when given.
    async fn move_group(
        &self,
        group: &str,
        (constraints, path): (Value, Option<Value>),
        ctl: &Control,
    ) -> Result<(), String> {
        let goal = json!({
            "request": self.arms.plan_request(group, &constraints, path, &json!({"is_diff": true})),
            "planning_options": {
                "planning_scene_diff": {"is_diff": true, "robot_state": {"is_diff": true}},
                "plan_only": false,
            },
        });
        let action = self.arms.config.name("move_action");
        let types = (action.as_str(), "moveit_msgs/action/MoveGroup");
        ended(
            &run_goal(
                self.arms.robot.as_ref(),
                &self.goals,
                types,
                goal,
                ctl,
                None,
            )
            .await?,
        )
    }

    /// Moves the hand of `group` straight by `along` in the planning frame.
    async fn line(&self, group: &str, along: [f64; 3], ctl: &Control) -> Result<(), String> {
        let path = self.arms.straight(group, along).await?;
        let share = path["fraction"].as_f64().unwrap_or(0.0);
        if share < CLEAR_SHARE {
            if share <= 0.0
                && let Some(touching) = self.starts_in_collision(group).await
            {
                return Err(format!(
                    "the arm starts in a collision ({touching}): move it clear first, as to one \
                     of its named poses"
                ));
            }
            return Err(format!(
                "only {:.0}% of the way is clear of collisions or within reach",
                share * 100.0
            ));
        }
        let action = self.arms.config.name("execute_trajectory");
        let goal = json!({"trajectory": path["solution"], "controller_names": []});
        let types = (action.as_str(), "moveit_msgs/action/ExecuteTrajectory");
        ended(
            &run_goal(
                self.arms.robot.as_ref(),
                &self.goals,
                types,
                goal,
                ctl,
                None,
            )
            .await?,
        )
    }

    /// What `group` touches where it stands now, if it is in a collision.
    async fn starts_in_collision(&self, group: &str) -> Option<String> {
        let joints = self.arms.joints().await.ok()?;
        let request = json!({"robot_state": Arms::robot_state(&joints), "group_name": group});
        let service = self.arms.config.name("check_state_validity");
        let r = self
            .arms
            .robot
            .call(
                &service,
                "moveit_msgs/srv/GetStateValidity",
                request,
                CALL_WAIT,
            )
            .await
            .ok()?;
        if r["valid"].as_bool() != Some(false) {
            return None;
        }
        let pairs: Vec<String> = r["contacts"]
            .as_array()
            .into_iter()
            .flatten()
            .take(2)
            .map(|c| {
                format!(
                    "{} and {}",
                    c["contact_body_1"].as_str().unwrap_or("?"),
                    c["contact_body_2"].as_str().unwrap_or("?")
                )
            })
            .collect();
        Some(if pairs.is_empty() {
            "it touches something".to_owned()
        } else {
            pairs.join("; ")
        })
    }

    /// Runs `motion` with the arms the executor lends, given back however it ends.
    async fn lent<F>(&self, max_s: f64, motion: F) -> Result<(), String>
    where
        F: Future<Output = Result<(), String>> + Send,
    {
        let Some(service) = &self.lend else {
            // An executor that lends nothing would not know NervROS moves its arms.
            if self.arms.profile.mission.is_some() {
                return Err(
                    "this robot's executor has no `lend` service in `[mission]`, so NervROS may \
                     not move the arms it holds"
                        .to_owned(),
                );
            }
            return motion.await;
        };
        let ask = |lend: bool| {
            json!({"client": crate::mission::client_id(), "resources": ["left_arm", "right_arm"],
                   "lend": lend, "max_s": max_s})
        };
        drop(self.returning.lock().await);
        let robot = &self.arms.robot;
        let lent = robot
            .call(
                service,
                "nervros_interfaces/srv/LendResources",
                ask(true),
                CALL_WAIT,
            )
            .await
            .map_err(|e| format!("the executor did not lend the arms: {e}"))?;
        if lent["granted"].as_bool() != Some(true) {
            return Err(format!(
                "the executor kept the arms: {}",
                lent["reason"].as_str().unwrap_or("no reason given")
            ));
        }
        // Given back even when the step is dropped part way, as a timeout does; otherwise given
        // back before the next step, which may be the executor's. Either way only once MoveIt's
        // goals have ended: the executor takes the arms at once.
        let mut back = GiveBack(
            Some(Back {
                robot: Arc::clone(robot),
                service: service.clone(),
                request: ask(false),
                goals: Arc::clone(&self.goals),
                actions: [
                    self.arms.config.name("move_action"),
                    self.arms.config.name("execute_trajectory"),
                ],
            }),
            Arc::clone(&self.returning),
        );
        let out = motion.await;
        if let Some(b) = back.0.take() {
            b.give().await;
        }
        out
    }
}

async fn give_back(robot: &dyn nervros_ros::RobotPort, service: &str, request: Value) {
    let back = robot
        .call(
            service,
            "nervros_interfaces/srv/LendResources",
            request,
            CALL_WAIT,
        )
        .await;
    if let Err(e) = back {
        tracing::warn!(error = %e, "the arms were not given back; the executor takes them back when NervROS's heartbeat or its limit runs out");
    }
}

/// How long the arms wait for MoveIt's goals to end before they go back regardless: a little
/// longer than a cancel's retries.
const GIVE_BACK_WAIT: Duration = Duration::from_secs(12);

/// What giving the arms back takes.
struct Back {
    robot: Arc<dyn nervros_ros::RobotPort>,
    service: String,
    request: Value,
    goals: Arc<Goals>,
    actions: [String; 2],
}

impl Back {
    /// Gives the arms back once no goal of MoveIt's that NervROS sent is open.
    async fn give(self) {
        let deadline = tokio::time::Instant::now() + GIVE_BACK_WAIT;
        while self
            .goals
            .open()
            .iter()
            .any(|(a, _)| self.actions.contains(a))
            && tokio::time::Instant::now() < deadline
        {
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        give_back(self.robot.as_ref(), &self.service, self.request).await;
    }
}

/// Gives the arms back to the executor when dropped, holding `returning` until they are back.
struct GiveBack(Option<Back>, Arc<tokio::sync::Mutex<()>>);

impl Drop for GiveBack {
    fn drop(&mut self) {
        if let Some(back) = self.0.take() {
            let returning = Arc::clone(&self.1);
            // Taken here, before the next attempt can ask for the arms.
            let held = Arc::clone(&returning).try_lock_owned();
            // Detached on purpose: a drop cannot wait, and the arms must go back after the move.
            tokio::spawn(async move {
                let _held = match held {
                    Ok(held) => held,
                    Err(_) => returning.lock_owned().await,
                };
                back.give().await;
            });
        }
    }
}

fn full_hand(held: &str) -> String {
    format!("the hand holds {held}, and moving its fingers could drop it: a place puts it down")
}

/// A finished MoveIt goal, or what went wrong in words.
fn ended(r: &GoalResult) -> Result<(), String> {
    let code = r.result["error_code"]["val"].as_i64().unwrap_or(0);
    match r.status {
        GoalStatus::Succeeded if code == 1 => Ok(()),
        GoalStatus::Canceled => Err("MoveIt's move was cancelled".to_owned()),
        _ => Err(format!("MoveIt: {}", error_words(code))),
    }
}

/// Path constraints that keep `tip` turned as `orientation`, within a small tilt.
pub(super) fn held_level(planning: &str, tip: &str, orientation: &Value) -> Value {
    json!({"orientation_constraints": [{
        "header": {"frame_id": planning},
        "link_name": tip,
        "orientation": orientation,
        "absolute_x_axis_tolerance": HELD_TILT,
        "absolute_y_axis_tolerance": HELD_TILT,
        "absolute_z_axis_tolerance": HELD_TILT,
        "weight": 1.0,
    }]})
}

/// A hand move along `direction` by `metres`, in the planning frame: x forward, y left, z up.
pub(super) fn along(direction: &str, metres: f64) -> Result<[f64; 3], String> {
    let unit = match direction {
        "forward" => [1.0, 0.0, 0.0],
        "back" | "backward" => [-1.0, 0.0, 0.0],
        "left" => [0.0, 1.0, 0.0],
        "right" => [0.0, -1.0, 0.0],
        "up" => [0.0, 0.0, 1.0],
        "down" => [0.0, 0.0, -1.0],
        other => {
            return Err(format!(
                "`{other}` is not up, down, forward, back, left or right"
            ));
        }
    };
    Ok(unit.map(|u| u * metres))
}

/// The arguments `Arms::point` takes for a target: `x;y;z` on the map, else an object's id.
fn target_args(target: &str) -> Value {
    match triple(target) {
        Some([x, y, z]) => json!({"x": x, "y": y, "z": z}),
        None => json!({"object": target}),
    }
}

/// Joint constraints that hold each joint of `at` at its value.
fn joint_constraints(at: &BTreeMap<String, f64>) -> Value {
    json!({"joint_constraints": at.iter().map(|(j, v)| json!({
        "joint_name": j, "position": v,
        "tolerance_above": 0.01, "tolerance_below": 0.01, "weight": 1.0,
    })).collect::<Vec<_>>()})
}

/// One motion.
struct Native {
    kind: Kind,
    shared: Arc<Motions>,
}

impl Native {
    async fn arm_to(&self, arm: &str, pose: &str, ctl: &Control) -> Result<(), String> {
        let m = &self.shared;
        let (srdf, group, planning, _) = m.arms.parts(&json!({"group": arm})).await?;
        let Goal::Pose(_, at) = m
            .arms
            .goal(&json!({"pose": pose}), &srdf, &group, &planning)
            .await?
        else {
            return Err(format!("`{pose}` is not a named pose of {group}"));
        };
        m.lent(
            ARM_S,
            m.move_group(&group, (joint_constraints(&at), None), ctl),
        )
        .await
    }

    async fn hand_to(&self, arm: &str, target: &str, ctl: &Control) -> Result<(), String> {
        let m = &self.shared;
        let (_, group, planning, tip) = m.arms.parts(&json!({"group": arm})).await?;
        let (at, _) = m.arms.point(&target_args(target), &planning).await?;
        let turned = m.arms.tip_orientation(&planning, &tip)?;
        let constraints = point_constraints(&planning, &tip, at, &turned);
        let path = m.arms.hand_path(&group, &planning, &tip, &turned).await;
        m.lent(ARM_S, m.move_group(&group, (constraints, path), ctl))
            .await
    }

    async fn grip(&self, arm: &str, state: &str, ctl: &Control) -> Result<(), String> {
        let m = &self.shared;
        let srdf = m.arms.srdf().await?;
        let group = Arms::group_named(&srdf, arm)?.name.clone();
        let hand = srdf
            .end_effectors
            .iter()
            .find(|e| e.parent_group == group)
            .map(|e| e.group.clone())
            .ok_or_else(|| format!("the SRDF hangs no hand from {group}"))?;
        let pose = srdf
            .poses_of(&hand)
            .find(|p| p.name == state)
            .ok_or_else(|| {
                let names: Vec<&str> = srdf.poses_of(&hand).map(|p| p.name.as_str()).collect();
                format!("{hand} has no state `{state}`; it has {}", names.join(", "))
            })?;
        // Any state can loosen a grip, pinch_ready as much as open: putting down is a place's.
        let held = m.arms.holding(&group).await?;
        if !held.is_empty() {
            return Err(full_hand(&held));
        }
        m.lent(
            ARM_S,
            m.move_group(&hand, (joint_constraints(&pose.joints), None), ctl),
        )
        .await
    }
}

#[async_trait]
impl NativeSkill for Native {
    fn skill(&self) -> Value {
        let arm = json!({"name": "arm", "type": "string",
                         "description": "Which arm, as arm_status names it, such as right_arm."});
        let (name, description, args) = match self.kind {
            Kind::ArmTo => (
                "ArmTo",
                "Move an arm to a named pose: one of the arm's own (arm_status lists them) or one \
                 remembered with arm_settings.",
                json!([arm, {"name": "pose", "type": "string", "format": "id",
                             "description": "The pose's name, such as home or tucked."}]),
            ),
            Kind::HandTo => (
                "HandTo",
                "Move a hand to just above an object, or to a point, keeping it turned as it is, \
                 and level all the way when it holds something. It stops above an object, not on it, and only within the arm's reach. To show \
                 the operator where something is, use point instead.",
                json!([arm, {"name": "target", "type": "string",
                             "description": "An object's id, such as O17, or a point x;y;z on the map in metres."}]),
            ),
            Kind::HandLine => (
                "HandLine",
                "Move a hand straight by a short distance, keeping it turned as it is: up, down, \
                 forward, back, left or right from the robot's point of view. It fails before \
                 moving when the way is not clear.",
                json!([arm,
                       {"name": "direction", "type": "string", "enum": ["up", "down", "forward", "back", "left", "right"],
                        "description": "Which way, from the robot's point of view."},
                       {"name": "distance_m", "type": "string", "format": "number", "min": 0.01, "max": 0.5,
                        "description": "How far, in metres."}]),
            ),
            Kind::Grip => (
                "Grip",
                "Set a hand to one of its named states, such as open or closed. It never moves a \
                 hand that holds something, or may: a place puts it down.",
                json!([arm, {"name": "state", "type": "string", "format": "id",
                             "description": "The hand's state, as arm_status lists them."}]),
            ),
        };
        let effects = if self.kind == Kind::ArmTo {
            json!(["arm_at(arm, pose)"])
        } else {
            json!([])
        };
        json!({
            "name": name, "description": description, "args": args, "outputs": [],
            "requires": [], "effects": effects, "resources": ["left_arm", "right_arm"],
            "risk": "motion", "idempotent": true, "max_duration_s": ARM_S, "template": name,
            "native": "MoveIt",
        })
    }

    async fn check(&self, args: &BTreeMap<String, String>) -> Option<String> {
        // Every arm move with an object in hand risks it; carrying is what such an arm moves for,
        // and HandTo keeps a held object level instead.
        let guarded = match self.kind {
            Kind::Grip | Kind::HandLine => true,
            Kind::ArmTo => args.get("pose").is_none_or(|pose| pose.trim() != "carry"),
            Kind::HandTo => false,
        };
        if !guarded {
            return None;
        }
        match self.shared.arms.holding(args.get("arm")?).await {
            Ok(held) if held.is_empty() => None,
            Ok(held) => Some(full_hand(&held)),
            Err(e) => Some(e),
        }
    }

    fn needs(&self) -> Vec<String> {
        let arms = &self.shared.arms.config;
        match self.kind {
            Kind::HandLine => vec![arms.name("execute_trajectory")],
            _ => vec![arms.name("move_action")],
        }
    }

    async fn run(
        &self,
        args: &BTreeMap<String, String>,
        ctl: &Control,
    ) -> Result<BTreeMap<String, String>, String> {
        let arg = |name: &str| args.get(name).map_or("", |v| v.trim());
        match self.kind {
            Kind::ArmTo => self.arm_to(arg("arm"), arg("pose"), ctl).await,
            Kind::HandTo => self.hand_to(arg("arm"), arg("target"), ctl).await,
            Kind::HandLine => {
                // Refused, not clamped: a line of no length would succeed without moving.
                let metres = arg("distance_m")
                    .parse::<f64>()
                    .ok()
                    .filter(|v| (super::check::MIN_LINE_M..=super::check::MAX_LINE_M).contains(v))
                    .ok_or("`distance_m` must be a number from 0.01 to 0.5")?;
                let m = &self.shared;
                let srdf = m.arms.srdf().await?;
                let group = Arms::group_named(&srdf, arg("arm"))?.name.clone();
                let way = along(arg("direction"), metres)?;
                m.lent(ARM_S, m.line(&group, way, ctl)).await
            }
            Kind::Grip => self.grip(arg("arm"), arg("state"), ctl).await,
        }
        .map(|()| BTreeMap::new())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_held_thing_stays_turned_as_it_is() {
        let turned = json!({"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0});
        let level = held_level("pelvis", "hand", &turned);
        let kept = &level["orientation_constraints"][0];
        assert_eq!(kept["link_name"], "hand");
        assert_eq!(kept["orientation"], turned);
        assert_eq!(kept["absolute_x_axis_tolerance"], json!(HELD_TILT));
    }

    #[test]
    fn a_target_is_a_point_or_an_object() {
        assert_eq!(
            target_args("1; 2;0.5"),
            json!({"x": 1.0, "y": 2.0, "z": 0.5})
        );
        assert_eq!(target_args("O17"), json!({"object": "O17"}));
        assert_eq!(target_args("1;2"), json!({"object": "1;2"}));
    }

    #[test]
    fn a_straight_move_goes_the_way_asked_from_the_robots_view() {
        assert_eq!(along("up", 0.1), Ok([0.0, 0.0, 0.1]));
        assert_eq!(along("right", 0.2), Ok([0.0, -0.2, 0.0]));
        assert!(along("sideways", 0.1).is_err());
    }

    #[test]
    fn moveit_failures_read_as_words() {
        let aborted = GoalResult {
            status: GoalStatus::Aborted,
            result: json!({"error_code": {"val": -1}}),
        };
        assert_eq!(ended(&aborted), Err("MoveIt: no plan was found".to_owned()));
        let done = GoalResult {
            status: GoalStatus::Succeeded,
            result: json!({"error_code": {"val": 1}}),
        };
        assert_eq!(ended(&done), Ok(()));
    }
}
