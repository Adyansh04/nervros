//! Nav2's motions as mission steps NervROS runs itself: walks, routes, turns, straight walks, waits
//! and docking, for a robot whose executor has no skill of the name or that has no executor. They
//! take the names an executor's skills would, so plans read the same either way.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::{GoalResult, GoalStatus, RobotPort};
use serde_json::{Value, json};

use super::Nav2Config;
use crate::integrations::{
    Again, Control, Goals, NativeSkill, Spot, beside, locate, run_goal, stamped, turns,
};
use crate::places::Places;
use crate::profile::Profile;

/// How far from an object's box a walk to it stops.
const BESIDE_M: f64 = 0.5;
const READ_WAIT: Duration = Duration::from_secs(3);
/// The longest a single walk or route may take.
const WALK_S: f64 = 240.0;
const TURN_S: f64 = 60.0;
const STRAIGHT_S: f64 = 60.0;
/// The longest straight walk one step takes, as executors' walks keep it.
const STRAIGHT_MAX_M: f64 = 2.0;

/// The motions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    GoToPose,
    GoToTarget,
    WalkRoute,
    TurnInPlace,
    WalkStraight,
    Wait,
    Dock,
    Undock,
}

/// What the motions share.
pub(super) struct Motions {
    config: Nav2Config,
    profile: Profile,
    robot: Arc<dyn RobotPort>,
    places: Arc<Places>,
    goals: Arc<Goals>,
}

impl Motions {
    /// The motions this robot's Nav2 can serve: walks to targets need a world model, docking a
    /// docking server.
    pub(super) fn natives(
        config: &Nav2Config,
        profile: &Profile,
        (robot, places, goals): (Arc<dyn RobotPort>, Arc<Places>, &Arc<Goals>),
    ) -> Vec<Arc<dyn NativeSkill>> {
        let world = profile
            .world
            .as_ref()
            .is_some_and(|w| w.rooms.is_some() || w.objects.is_some());
        let shared = Arc::new(Self {
            config: config.clone(),
            profile: profile.clone(),
            robot,
            places,
            goals: Arc::clone(goals),
        });
        let mut kinds = vec![
            Kind::GoToPose,
            Kind::WalkRoute,
            Kind::TurnInPlace,
            Kind::WalkStraight,
            Kind::Wait,
        ];
        if world {
            kinds.push(Kind::GoToTarget);
        }
        if config.docking {
            kinds.extend([Kind::Dock, Kind::Undock]);
        }
        kinds
            .into_iter()
            .map(|kind| {
                Arc::new(Native {
                    kind,
                    shared: Arc::clone(&shared),
                }) as Arc<dyn NativeSkill>
            })
            .collect()
    }

    fn here(&self) -> Option<(f64, f64, f64)> {
        let (map, base) = (&self.profile.ros.map_frame, &self.profile.ros.base_frame);
        self.robot.transform(map, base).ok().map(|t| t.planar())
    }

    fn pose(&self, x: f64, y: f64, yaw: f64) -> Value {
        stamped(&self.profile.ros.map_frame, x, y, yaw)
    }

    async fn walk(&self, pose: Value, ctl: &Control) -> Result<(), String> {
        let goal = json!({"pose": pose, "behavior_tree": ""});
        // Paused, the walk stops; resumed, it sets out for the same pose again.
        let again = |goal: &Value, _: &Value| goal.clone();
        let types = ("navigate_to_pose", "nav2_msgs/action/NavigateToPose");
        self.send(types, goal, ctl, Some(&again)).await.map(|_| ())
    }

    /// Sends `goal` to Nav2's action `name` and follows it to its end, as [`run_goal`] does.
    async fn send(
        &self,
        (name, action_type): (&str, &str),
        goal: Value,
        ctl: &Control,
        again: Option<Again<'_>>,
    ) -> Result<Value, String> {
        let action = self.config.name(name);
        let types = (action.as_str(), action_type);
        ended(run_goal(self.robot.as_ref(), &self.goals, types, goal, ctl, again).await?)
    }

    /// Where `target` is: a place, a room by id or name, an object by id, or the nearest object
    /// it names; and its id.
    async fn target(&self, target: &str) -> Result<(Spot, String), String> {
        let here = self.here().map(|(x, y, _)| (x, y));
        let ctx = (&self.profile, self.places.as_ref(), self.robot.as_ref());
        if let Ok(spot) = locate(&json!({"place": target}), ctx, here, BESIDE_M).await {
            return Ok((spot, target.to_owned()));
        }
        let world = self.profile.world.as_ref();
        if let Some(topic) = world.and_then(|w| w.rooms.as_ref()) {
            let rooms = self
                .robot
                .latest(&topic.topic, &topic.msg_type, READ_WAIT)
                .await
                .unwrap_or(Value::Null);
            if let Some(id) = named(&rooms["rooms"], target, &["id", "name", "type"]) {
                let spot = locate(&json!({"room": id}), ctx, here, BESIDE_M).await?;
                return Ok((spot, id));
            }
        }
        if let Some(topic) = world.and_then(|w| w.objects.as_ref()) {
            let objects = self
                .robot
                .latest(&topic.topic, &topic.msg_type, READ_WAIT)
                .await
                .unwrap_or(Value::Null);
            let id = named(&objects["objects"], target, &["id"])
                .or_else(|| nearest(&objects["objects"], target, here));
            if let Some(id) = id {
                return Ok((beside(&objects, &id, here, BESIDE_M)?, id));
            }
        }
        Err(format!(
            "`{target}` is no place, room or object the robot knows; find_objects and list_places name them"
        ))
    }

    /// The poses a route passes through: each a place or a room, in order, facing on along it.
    async fn route(&self, names: &str) -> Result<Vec<(f64, f64, f64)>, String> {
        let ctx = (&self.profile, self.places.as_ref(), self.robot.as_ref());
        let mut from = self.here().map(|(x, y, _)| (x, y));
        let mut out = Vec::new();
        for name in names
            .split([',', ';'])
            .map(str::trim)
            .filter(|n| !n.is_empty())
        {
            let mut spot = Err(String::new());
            for key in ["place", "room"] {
                spot = locate(&json!({key: name}), ctx, from, BESIDE_M).await;
                if spot.is_ok() {
                    break;
                }
            }
            let spot = spot.map_err(|_| format!("`{name}` is not a known place or room"))?;
            let yaw = spot
                .yaw
                .unwrap_or_else(|| from.map_or(0.0, |(fx, fy)| (spot.y - fy).atan2(spot.x - fx)));
            out.push((spot.x, spot.y, yaw));
            from = Some((spot.x, spot.y));
        }
        if out.is_empty() {
            return Err("the route names no places".to_owned());
        }
        Ok(out)
    }
}

/// The id of the thing in `things` one of `keys` names as `wanted`.
fn named(things: &Value, wanted: &str, keys: &[&str]) -> Option<String> {
    things.as_array()?.iter().find_map(|t| {
        keys.iter()
            .any(|k| {
                t[k].as_str()
                    .is_some_and(|v| crate::places::same_words(v, wanted))
            })
            .then(|| t["id"].as_str().map(str::to_owned))
            .flatten()
    })
}

/// The id of the object labelled `label` nearest `from`.
fn nearest(objects: &Value, label: &str, from: Option<(f64, f64)>) -> Option<String> {
    let (fx, fy) = from.unwrap_or_default();
    objects
        .as_array()?
        .iter()
        .filter(|o| {
            o["label"]
                .as_str()
                .is_some_and(|l| crate::places::same_words(l, label))
        })
        .filter_map(|o| {
            let p = &o["pose"]["position"];
            let d = (p["x"].as_f64()? - fx).hypot(p["y"].as_f64()? - fy);
            Some((d, o["id"].as_str()?.to_owned()))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, id)| id)
}

/// A finished Nav2 goal: its result, or what went wrong in words.
fn ended(r: GoalResult) -> Result<Value, String> {
    let msg = r.result["error_msg"].as_str().unwrap_or_default().trim();
    let code = r.result["error_code"].as_u64().unwrap_or(0);
    match r.status {
        GoalStatus::Succeeded if r.result["success"].as_bool() != Some(false) => Ok(r.result),
        GoalStatus::Canceled => Err("Nav2 cancelled it".to_owned()),
        _ if !msg.is_empty() => Err(format!("Nav2: {msg}")),
        _ => Err(format!("Nav2: {}", error_words(code))),
    }
}

/// Nav2's error codes in words: the navigators pass on their planner's and controller's.
fn error_words(code: u64) -> &'static str {
    match code {
        102 | 202 | 702 | 722 => "a frame could not be looked up (TF)",
        103 => "the path was invalid",
        104 | 105 => "the robot made no progress",
        106 => "no velocity it could send was safe",
        107 | 207 | 701 | 721 => "it ran out of time",
        203 => "the robot is off the map",
        204 => "the goal is off the map",
        205 => "the robot stands in an obstacle",
        206 => "the goal is in an obstacle",
        208 => "no path reaches it",
        703 | 723 => "something is in the way",
        724 => "it was asked for something it cannot do",
        901 => "no such dock",
        902 => "the dock is not valid",
        903 => "it could not reach the dock's staging pose",
        904 => "it could not see the dock",
        905 => "it could not drive onto the dock",
        906 => "it is docked but not charging",
        _ => "it failed",
    }
}

fn number(args: &BTreeMap<String, String>, name: &str) -> Result<f64, String> {
    args.get(name)
        .and_then(|v| v.trim().parse::<f64>().ok())
        .filter(|v| v.is_finite())
        .ok_or_else(|| format!("`{name}` must be a number"))
}

/// `x;y;yaw` on the map.
fn station(text: &str) -> Result<(f64, f64, f64), String> {
    let v: Option<Vec<f64>> = text
        .split(';')
        .map(|p| p.trim().parse().ok().filter(|v: &f64| v.is_finite()))
        .collect();
    match v.as_deref() {
        Some(&[x, y, yaw]) => Ok((x, y, yaw)),
        _ => Err(format!("`{text}` is not x;y;yaw")),
    }
}

/// One motion.
struct Native {
    kind: Kind,
    shared: Arc<Motions>,
}

#[async_trait]
impl NativeSkill for Native {
    fn skill(&self) -> Value {
        let base = json!(["base"]);
        let (name, description, args, outputs, effects, resources, max) = match self.kind {
            Kind::GoToPose => (
                "GoToPose",
                "Walk to a fixed place given as coordinates in the map frame, then turn to face \
                 the given heading. Use it for a named place whose coordinates are known.",
                json!([{"name": "station", "type": "string", "format": "station",
                        "description": "Where to stand and face, as x;y;yaw in the map frame, metres and radians, such as 4.30;-5.60;1.5708."}]),
                json!([]),
                json!(["at(station)"]),
                base,
                WALK_S,
            ),
            Kind::GoToTarget => (
                "GoToTarget",
                "Walk to a room or an object in the world model and stop facing it. Given what an \
                 object is rather than its id, it walks to the nearest one, and target_id says \
                 which it was.",
                json!([{"name": "target", "type": "world_id", "format": "phrase",
                        "description": "The id of a room or an object in the world model, such as R2 or O17; or a room's name, or what an object is in plain words, for the nearest one."}]),
                json!([{"name": "target_id", "type": "world_id",
                        "description": "The id of the room or object it walked to."}]),
                json!(["near(target)"]),
                base,
                WALK_S,
            ),
            Kind::WalkRoute => (
                "WalkRoute",
                "Walk through named places or rooms in order without stopping at each, and stop \
                 at the last one. Use it for \"go through the hallway to the kitchen\" or a round \
                 of several places; to stop and do something at a place, walk to it on its own.",
                json!([{"name": "places", "type": "string",
                        "description": "The places or rooms to pass through, in order, separated by commas, such as hallway, kitchen."}]),
                json!([]),
                json!([]),
                base,
                WALK_S * 2.0,
            ),
            Kind::TurnInPlace => (
                "TurnInPlace",
                "Turn on the spot by an angle from where the robot faces now, and stop: positive \
                 turns left, negative right, up to half a turn either way.",
                json!([{"name": "degrees", "type": "string", "format": "number", "min": -180, "max": 180,
                        "description": "How far to turn, in degrees; positive is left."}]),
                json!([]),
                json!([]),
                base,
                TURN_S,
            ),
            Kind::WalkStraight => (
                "WalkStraight",
                "Walk straight forward or backward by a set distance and stop, without turning. \
                 Use it for small exact moves such as \"back up half a metre\"; to get to a place \
                 walk to it. It stops short and fails when something is in the way.",
                json!([
                    {"name": "direction", "type": "string", "enum": ["forward", "backward"],
                     "description": "Which way to walk, relative to where the robot faces."},
                    {"name": "distance_m", "type": "string", "format": "number", "min": 0.1, "max": STRAIGHT_MAX_M,
                     "description": "How far to walk, in metres; always positive."},
                ]),
                json!([]),
                json!([]),
                base,
                STRAIGHT_S,
            ),
            Kind::Wait => (
                "Wait",
                "Stand still for a while, as in \"wait a minute by the door, then come back\".",
                json!([{"name": "seconds", "type": "string", "format": "number", "min": 1, "max": 600,
                        "description": "How long to wait, in seconds."}]),
                json!([]),
                json!([]),
                json!([]),
                600.0,
            ),
            Kind::Dock => (
                "Dock",
                "Walk to a dock and dock onto it, to charge.",
                json!([{"name": "dock", "type": "string", "format": "id",
                        "description": "The dock's id in the robot's docking database."}]),
                json!([]),
                json!([]),
                base,
                600.0,
            ),
            Kind::Undock => (
                "Undock",
                "Drive off the dock the robot is on.",
                json!([]),
                json!([]),
                json!([]),
                base,
                60.0,
            ),
        };
        json!({
            "name": name, "description": description, "args": args, "outputs": outputs,
            "requires": [], "effects": effects, "resources": resources, "risk": "motion",
            "idempotent": false, "max_duration_s": max, "template": name,
        })
    }

    fn needs(&self) -> Vec<String> {
        let name = |n: &str| self.shared.config.name(n);
        match self.kind {
            Kind::GoToPose | Kind::GoToTarget => vec![name("navigate_to_pose")],
            Kind::WalkRoute => vec![name("navigate_through_poses")],
            Kind::TurnInPlace => vec![name("spin")],
            Kind::WalkStraight => vec![name("drive_on_heading")],
            Kind::Wait => Vec::new(),
            Kind::Dock => vec![name("dock_robot")],
            Kind::Undock => vec![name("undock_robot")],
        }
    }

    async fn run(
        &self,
        args: &BTreeMap<String, String>,
        ctl: &Control,
    ) -> Result<BTreeMap<String, String>, String> {
        let m = &self.shared;
        let arg = |name: &str| args.get(name).map_or("", |v| v.trim());
        let none = BTreeMap::new();
        match self.kind {
            Kind::GoToPose => {
                let (x, y, yaw) = station(arg("station"))?;
                m.walk(m.pose(x, y, yaw), ctl).await.map(|()| none)
            }
            Kind::GoToTarget => {
                let (spot, id) = m.target(arg("target")).await?;
                let yaw = spot.yaw.unwrap_or_else(|| {
                    m.here()
                        .map_or(0.0, |(x, y, _)| (spot.y - y).atan2(spot.x - x))
                });
                m.walk(m.pose(spot.x, spot.y, yaw), ctl).await?;
                Ok(BTreeMap::from([("target_id".to_owned(), id)]))
            }
            Kind::WalkRoute => {
                let poses: Vec<Value> = m
                    .route(arg("places"))
                    .await?
                    .into_iter()
                    .map(|(x, y, yaw)| m.pose(x, y, yaw))
                    .collect();
                let goal = json!({"poses": poses, "behavior_tree": ""});
                let types = (
                    "navigate_through_poses",
                    "nav2_msgs/action/NavigateThroughPoses",
                );
                m.send(types, goal, ctl, Some(&route_left))
                    .await
                    .map(|_| none)
            }
            Kind::TurnInPlace => {
                let degrees = number(args, "degrees")?.clamp(-180.0, 180.0);
                let types = ("spin", "nav2_msgs/action/Spin");
                m.send(types, spin(degrees.to_radians()), ctl, Some(&spin_left))
                    .await
                    .map(|_| none)
            }
            Kind::WalkStraight => {
                let metres = number(args, "distance_m")?.clamp(0.0, STRAIGHT_MAX_M);
                let sign = if arg("direction") == "backward" {
                    -1.0
                } else {
                    1.0
                };
                let goal = drive(sign * metres, sign * m.config.drive_speed_mps.abs());
                let types = ("drive_on_heading", "nav2_msgs/action/DriveOnHeading");
                m.send(types, goal, ctl, Some(&drive_left))
                    .await
                    .map(|_| none)
            }
            Kind::Wait => {
                let seconds = number(args, "seconds")?.clamp(0.0, 600.0);
                wait(Duration::from_secs_f64(seconds), ctl)
                    .await
                    .map(|()| none)
            }
            Kind::Dock => {
                let goal = json!({"use_dock_id": true, "dock_id": arg("dock"),
                                  "navigate_to_staging_pose": true, "max_staging_time": 1000.0});
                let types = ("dock_robot", "nav2_msgs/action/DockRobot");
                m.send(types, goal, ctl, None).await.map(|_| none)
            }
            Kind::Undock => {
                let goal = json!({"dock_type": "", "max_undocking_time": 30.0});
                let types = ("undock_robot", "nav2_msgs/action/UndockRobot");
                m.send(types, goal, ctl, None).await.map(|_| none)
            }
        }
    }
}

/// Resumed, a route walks on through the places it had not reached.
fn route_left(goal: &Value, fb: &Value) -> Value {
    let all = goal["poses"].as_array().cloned().unwrap_or_default();
    let left = fb["number_of_poses_remaining"]
        .as_u64()
        .and_then(|n| usize::try_from(n).ok())
        .unwrap_or(all.len())
        .clamp(1, all.len().max(1));
    json!({"poses": all[all.len().saturating_sub(left)..], "behavior_tree": ""})
}

/// A `Spin` goal of `radians`, positive left.
fn spin(radians: f64) -> Value {
    json!({"target_yaw": radians, "time_allowance": {"sec": 50, "nanosec": 0}})
}

/// Resumed, a turn turns what was left.
fn spin_left(goal: &Value, fb: &Value) -> Value {
    let target = goal["target_yaw"].as_f64().unwrap_or(0.0);
    let done = fb["angular_distance_traveled"]
        .as_f64()
        .unwrap_or(0.0)
        .abs();
    spin(target.signum() * (target.abs() - done).max(0.0))
}

/// Resumed, a straight walk walks what was left.
fn drive_left(goal: &Value, fb: &Value) -> Value {
    let target = goal["target"]["x"].as_f64().unwrap_or(0.0);
    let done = fb["distance_traveled"].as_f64().unwrap_or(0.0).abs();
    let left = target.signum() * (target.abs() - done).max(0.0);
    drive(left, goal["speed"].as_f64().unwrap_or(0.0))
}

/// A `DriveOnHeading` goal: `metres` along the heading, negative backward, at `speed` of the same
/// sign, with time to spare.
fn drive(metres: f64, speed: f64) -> Value {
    let allowance = (metres / speed.abs().max(0.05)).abs() * 2.0 + 10.0;
    let secs = Duration::from_secs_f64(allowance.min(STRAIGHT_S - 5.0)).as_secs() + 1;
    json!({
        "target": {"x": metres, "y": 0.0, "z": 0.0},
        "speed": speed,
        "time_allowance": {"sec": secs, "nanosec": 0},
    })
}

/// Stands still for `left`, which runs only while the mission is not paused.
async fn wait(mut left: Duration, ctl: &Control) -> Result<(), String> {
    let mut paused = ctl.paused.clone();
    while !left.is_zero() {
        if *paused.borrow() {
            tokio::select! {
                true = turns(&mut paused, false) => {}
                () = ctl.until_stopped() => return Err("stopped".to_owned()),
            }
        }
        let started = tokio::time::Instant::now();
        tokio::select! {
            () = tokio::time::sleep(left) => left = Duration::ZERO,
            () = ctl.until_stopped() => return Err("stopped".to_owned()),
            true = turns(&mut paused, true) => left = left.saturating_sub(started.elapsed()),
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_station_is_three_numbers() {
        assert_eq!(station("1;-2.5; 0.5"), Ok((1.0, -2.5, 0.5)));
        assert!(station("1;2").is_err());
        assert!(station("1;NaN;0").is_err());
    }

    #[test]
    fn a_straight_walk_back_drives_backward_and_resumes_with_what_is_left() {
        let back = drive(-0.8, -0.25);
        assert_eq!(back["target"]["x"], json!(-0.8));
        assert_eq!(back["speed"], json!(-0.25));
        // A Duration's seconds are an integer on the wire.
        assert!(back["time_allowance"]["sec"].as_u64().unwrap() <= 56);
        let rest = drive_left(&back, &json!({"distance_traveled": 0.3}));
        assert!((rest["target"]["x"].as_f64().unwrap() + 0.5).abs() < 1e-9);
        assert_eq!(rest["speed"], json!(-0.25));
        let right = spin(-1.5);
        let rest = spin_left(&right, &json!({"angular_distance_traveled": 1.0}));
        assert!((rest["target_yaw"].as_f64().unwrap() + 0.5).abs() < 1e-9);
    }

    #[test]
    fn a_resumed_route_walks_on_through_the_places_it_had_not_reached() {
        let route = json!({"poses": [1, 2, 3], "behavior_tree": ""});
        let rest = route_left(&route, &json!({"number_of_poses_remaining": 2}));
        assert_eq!(rest["poses"], json!([2, 3]));
        assert_eq!(route_left(&route, &json!({}))["poses"], json!([1, 2, 3]));
        assert_eq!(
            route_left(&route, &json!({"number_of_poses_remaining": 0}))["poses"],
            json!([3])
        );
    }

    #[test]
    fn nav2_failures_read_as_words() {
        let aborted = |result: Value| GoalResult {
            status: GoalStatus::Aborted,
            result,
        };
        assert_eq!(
            ended(aborted(json!({"error_code": 208, "error_msg": ""}))),
            Err("Nav2: no path reaches it".to_owned())
        );
        assert_eq!(
            ended(aborted(
                json!({"error_code": 208, "error_msg": "Goal occupied"})
            )),
            Err("Nav2: Goal occupied".to_owned())
        );
        let docked_not_charging = GoalResult {
            status: GoalStatus::Succeeded,
            result: json!({"success": false, "error_code": 906, "error_msg": ""}),
        };
        assert_eq!(
            ended(docked_not_charging),
            Err("Nav2: it is docked but not charging".to_owned())
        );
    }

    #[test]
    fn the_nearest_object_of_a_label_is_taken() {
        let objects = json!([
            {"id": "O1", "label": "mug", "pose": {"position": {"x": 5.0, "y": 0.0}}},
            {"id": "O2", "label": "mug", "pose": {"position": {"x": 1.0, "y": 0.0}}},
            {"id": "O3", "label": "book", "pose": {"position": {"x": 0.5, "y": 0.0}}},
        ]);
        assert_eq!(
            nearest(&objects, "mug", Some((0.0, 0.0))).as_deref(),
            Some("O2")
        );
        assert_eq!(named(&objects, "o3", &["id"]).as_deref(), Some("O3"));
        assert!(nearest(&objects, "plate", None).is_none());
    }

    #[tokio::test]
    async fn a_wait_ends_early_when_stopped_and_holds_while_paused() {
        let (stop_tx, stop) = tokio::sync::watch::channel(false);
        let (pause_tx, paused) = tokio::sync::watch::channel(true);
        let ctl = Control { stop, paused };
        let waiting = tokio::spawn(async move { wait(Duration::from_millis(30), &ctl).await });
        tokio::time::sleep(Duration::from_millis(60)).await;
        assert!(!waiting.is_finished(), "a paused wait does not count down");
        pause_tx.send_replace(false);
        tokio::time::sleep(Duration::from_millis(80)).await;
        assert!(waiting.is_finished());
        assert_eq!(waiting.await.unwrap(), Ok(()));

        let (_pause, paused) = tokio::sync::watch::channel(false);
        let ctl = Control {
            stop: stop_tx.subscribe(),
            paused,
        };
        let long = tokio::spawn(async move { wait(Duration::from_mins(1), &ctl).await });
        stop_tx.send_replace(true);
        assert_eq!(long.await.unwrap(), Err("stopped".to_owned()));
    }
}
