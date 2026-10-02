//! Builtin tools written once for every robot: `list_places`, `robot_state` and `stop`.
//!
//! `look` lives in [`crate::look`]. `stop` is always allowed, armed or not: it only ever makes the
//! robot do less.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::RobotPort;
use serde_json::{Value, json};

use crate::guard::Guard;
use crate::mission::held_by;
use crate::profile::{MissionConfig, Profile, TopicRef};
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

const WORLD_WAIT: Duration = Duration::from_secs(2);

/// A room of the world model.
struct Room {
    pub(super) id: String,
    pub(super) name: String,
    pub(super) kind: String,
    pub(super) outline: Vec<(f64, f64)>,
    /// What the camera has seen of it, when the world model says (canopy does).
    seen: Value,
}

/// How much of a room's floor and walls the camera has seen and how many objects it holds, from
/// canopy's `Room` fields, rounded, as flat keys: nested ones read as "unseen" to small models.
/// Null when the message has none of them.
fn seen_of(room: &Value) -> Value {
    let round = |k: &str| room[k].as_f64().map(|v| (v * 100.0).round() / 100.0);
    let (floor, faces) = (round("floor_coverage"), round("face_coverage"));
    if floor.is_none() && faces.is_none() {
        return Value::Null;
    }
    json!({"floor_seen": floor, "walls_seen": faces, "objects": room["object_count"]})
}

/// The world model's rooms, or none when it does not run.
async fn rooms(robot: &dyn RobotPort, topic: Option<&TopicRef>) -> Vec<Room> {
    let Some(t) = topic else { return Vec::new() };
    let Ok(msg) = robot.latest(&t.topic, &t.msg_type, WORLD_WAIT).await else {
        return Vec::new();
    };
    msg["rooms"]
        .as_array()
        .map(|list| {
            list.iter()
                .map(|r| {
                    let outline = r["outline"]["points"]
                        .as_array()
                        .map(|pts| {
                            pts.iter()
                                .map(|p| {
                                    (
                                        p["x"].as_f64().unwrap_or(0.0),
                                        p["y"].as_f64().unwrap_or(0.0),
                                    )
                                })
                                .collect()
                        })
                        .unwrap_or_default();
                    let text = |k: &str| r[k].as_str().unwrap_or_default().to_owned();
                    Room {
                        id: text("id"),
                        name: text("name"),
                        kind: text("type"),
                        outline,
                        seen: seen_of(r),
                    }
                })
                .collect()
        })
        .unwrap_or_default()
}

/// Ray casting: whether a point lies inside a polygon.
#[must_use]
pub fn inside(point: (f64, f64), polygon: &[(f64, f64)]) -> bool {
    let (x, y) = point;
    let mut hit = false;
    let mut j = polygon.len().wrapping_sub(1);
    for (i, &(xi, yi)) in polygon.iter().enumerate() {
        let (xj, yj) = polygon[j];
        if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
            hit = !hit;
        }
        j = i;
    }
    hit
}

/// `list_places`: named places and the world model's rooms.
pub struct ListPlaces {
    spec: ToolSpec,
    places: Arc<crate::places::Places>,
    rooms: Option<TopicRef>,
    robot: Arc<dyn RobotPort>,
}

impl ListPlaces {
    /// The profile's places and the remembered ones.
    #[must_use]
    pub fn new(
        profile: &Profile,
        places: Arc<crate::places::Places>,
        robot: Arc<dyn RobotPort>,
    ) -> Self {
        let spec = ToolSpec::new(
            "list_places",
            "Lists the places the robot can go to: named places with their aliases, and the rooms the \
             world model knows, with how much of each room's floor and walls the camera has seen (0 to \
             1) and how many objects it holds. Use the returned ids in plans. A room seen little may \
             hold objects nobody has found yet.",
            json!({"type": "object", "properties": {}, "additionalProperties": false}),
            Risk::Observe,
        );
        let rooms = profile.world.as_ref().and_then(|w| w.rooms.clone());
        Self {
            spec,
            places,
            rooms,
            robot,
        }
    }
}

#[async_trait]
impl Tool for ListPlaces {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, _args: Value) -> ToolOutcome {
        let places: Vec<Value> = self
            .places
            .all()
            .iter()
            .map(|p| {
                let kind = if self.places.is_tagged(&p.name) {
                    "remembered place"
                } else {
                    "place"
                };
                json!({"id": p.name, "aliases": p.aliases, "kind": kind})
            })
            .collect();
        let rooms: Vec<Value> = rooms(self.robot.as_ref(), self.rooms.as_ref())
            .await
            .into_iter()
            .map(|r| {
                let mut room = json!({"id": r.id, "name": r.name, "type": r.kind, "kind": "room"});
                if let (Some(room), Some(seen)) = (room.as_object_mut(), r.seen.as_object()) {
                    room.extend(seen.clone());
                }
                room
            })
            .collect();
        ToolOutcome::ok(json!({"places": places, "rooms": rooms}))
    }
}

/// `robot_state`: where the robot is, what it holds, and whether it may act.
pub struct RobotState {
    spec: ToolSpec,
    robot: Arc<dyn RobotPort>,
    frames: (String, String),
    rooms: Option<TopicRef>,
    executor_state: Option<String>,
    guard: Arc<Guard>,
}

impl RobotState {
    /// From the profile.
    #[must_use]
    pub fn new(profile: &Profile, robot: Arc<dyn RobotPort>, guard: Arc<Guard>) -> Self {
        let spec = ToolSpec::new(
            "robot_state",
            "The robot's pose on the map, the room it is in, what each hand holds, the running mission, \
             and whether it is armed.",
            json!({"type": "object", "properties": {}, "additionalProperties": false}),
            Risk::Observe,
        );
        Self {
            spec,
            robot,
            frames: (
                profile.ros.map_frame.clone(),
                profile.ros.base_frame.clone(),
            ),
            rooms: profile.world.as_ref().and_then(|w| w.rooms.clone()),
            executor_state: profile.mission.as_ref().map(|m| m.state.clone()),
            guard,
        }
    }
}

#[async_trait]
impl Tool for RobotState {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, _args: Value) -> ToolOutcome {
        let mut out = json!({"armed": self.guard.armed()});
        // The rooms and the executor's state are read together: each can wait its full time.
        let state = async {
            match &self.executor_state {
                Some(topic) => Some(
                    self.robot
                        .latest_fresh(
                            topic,
                            "nervros_interfaces/msg/RobotState",
                            Duration::from_secs(1),
                            crate::mission::STATE_FRESH,
                        )
                        .await,
                ),
                None => None,
            }
        };
        let (rooms, state) = tokio::join!(rooms(self.robot.as_ref(), self.rooms.as_ref()), state);
        match self.robot.transform(&self.frames.0, &self.frames.1) {
            Ok(t) => {
                let [x, y, _] = t.translation;
                let round = |v: f64| (v * 100.0).round() / 100.0;
                out["pose"] = json!({"frame": self.frames.0, "x": round(x), "y": round(y), "yaw": round(t.yaw())});
                if let Some(r) = rooms.into_iter().find(|r| inside((x, y), &r.outline)) {
                    out["room"] = json!({"id": r.id, "name": r.name, "type": r.kind});
                }
            }
            Err(e) => out["pose_error"] = Value::String(e.to_string()),
        }
        match state {
            Some(Ok(state)) => {
                let field = |k: &str| state[k].clone();
                out["executor"] = json!({
                    "mission": field("mission_id"),
                    "step": field("mission_step"),
                    "holding_left": held_by(&state, "left"),
                    "holding_right": held_by(&state, "right"),
                    "resources_held": field("resources_held"),
                    "stopped": field("stopped"),
                    "can_move": field("can_move"),
                    "cannot_move_reason": field("cannot_move_reason"),
                    "tilt_deg": field("tilt_deg"),
                    "teleop": field("teleop"),
                });
            }
            Some(Err(_)) => {
                out["executor"] = json!("not heard from lately: its state is unknown");
            }
            None => {}
        }
        ToolOutcome::ok(out)
    }
}

/// `stop`: stops every motion through the robot's `StopAll`; always allowed.
pub struct Stop {
    spec: ToolSpec,
    robot: Arc<dyn RobotPort>,
    mission: Option<MissionConfig>,
}

impl Stop {
    /// From the profile.
    #[must_use]
    pub fn new(profile: &Profile, robot: Arc<dyn RobotPort>) -> Self {
        let spec = ToolSpec::new(
            "stop",
            "Stops the robot now: halts the running mission and cancels its goals. Hands keep their grip.",
            json!({
                "type": "object",
                "properties": {"reason": {"type": "string", "description": "Why, in a few words"}},
                "additionalProperties": false
            }),
            Risk::Observe,
        );
        Self {
            spec,
            robot,
            mission: profile.mission.clone(),
        }
    }
}

#[async_trait]
impl Tool for Stop {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let Some(m) = &self.mission else {
            return ToolOutcome::ok(
                json!({"stopped": true, "note": "this robot has no mission executor; nothing was running"}),
            );
        };
        let reason = args["reason"].as_str().unwrap_or("stop tool").to_owned();
        match self
            .robot
            .call(
                &m.stop,
                "nervros_interfaces/srv/StopAll",
                json!({"reason": reason}),
                Duration::from_secs(5),
            )
            .await
        {
            Ok(r) if r["ok"].as_bool() == Some(true) => {
                ToolOutcome::ok(json!({"stopped": true, "state": r["state_after"]}))
            }
            Ok(r) => ToolOutcome::failed(format!("the robot refused to stop: {}", r["message"])),
            Err(e) => ToolOutcome::failed(format!("stop failed: {e}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nervros_ros::Transform;
    use nervros_ros::fake::FakeRobot;
    use std::path::Path;

    fn profile() -> Profile {
        let text = r#"
            [robot]
            name = "t"
            [world]
            rooms = { topic = "/rooms", type = "canopy_msgs/msg/RoomArray" }
            [mission]
            execute = "/x/execute"
            validate = "/x/validate"
            catalog = "/x/catalog"
            stop = "/x/stop"
            state = "/x/state"
            [[place]]
            name = "zone_a"
            aliases = ["zone A"]
            pose = { x = 1.0, y = 2.0 }
            [models]
            file = "m.toml"
        "#;
        Profile::from_toml(text, Path::new("nervros.toml")).unwrap()
    }

    fn rooms_msg() -> Value {
        json!({"rooms": [{"id": "R1", "name": "kitchen", "type": "kitchen", "outline": {"points": [
            {"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 4.0, "y": 0.0, "z": 0.0}, {"x": 4.0, "y": 4.0, "z": 0.0}, {"x": 0.0, "y": 4.0, "z": 0.0}
        ]}}]})
    }

    #[test]
    fn ray_casting() {
        let square = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)];
        assert!(inside((1.0, 1.0), &square));
        assert!(!inside((5.0, 1.0), &square));
    }

    #[tokio::test]
    async fn places_and_rooms_are_listed() {
        let robot: Arc<dyn RobotPort> =
            Arc::new(FakeRobot::new().with_topic("/rooms", rooms_msg()));
        let out = ListPlaces::new(
            &profile(),
            crate::places::Places::new(&profile(), None),
            robot,
        )
        .call(json!({}))
        .await;
        assert_eq!(out.data["places"][0]["id"], "zone_a");
        assert_eq!(out.data["rooms"][0]["name"], "kitchen");
        assert!(
            out.data["rooms"][0].get("floor_seen").is_none(),
            "no coverage fields, nothing claimed"
        );
    }

    #[tokio::test]
    async fn rooms_say_how_much_the_camera_has_seen_of_them() {
        let mut msg = rooms_msg();
        msg["rooms"][0]["floor_coverage"] = json!(0.724);
        msg["rooms"][0]["face_coverage"] = json!(0.31);
        msg["rooms"][0]["object_count"] = json!(5);
        let robot: Arc<dyn RobotPort> = Arc::new(FakeRobot::new().with_topic("/rooms", msg));
        let out = ListPlaces::new(
            &profile(),
            crate::places::Places::new(&profile(), None),
            robot,
        )
        .call(json!({}))
        .await;
        let room = &out.data["rooms"][0];
        assert_eq!(room["floor_seen"], 0.72);
        assert_eq!(room["walls_seen"], 0.31);
        assert_eq!(room["objects"], 5);
    }

    #[tokio::test]
    async fn state_names_the_room_and_arming() {
        let robot: Arc<dyn RobotPort> = Arc::new(
            FakeRobot::new()
                .with_topic("/rooms", rooms_msg())
                .with_transform(
                    "map",
                    "base_footprint",
                    Transform {
                        translation: [1.0, 1.0, 0.0],
                        ..Transform::IDENTITY
                    },
                )
                .with_topic(
                    "/x/state",
                    json!({"mission_id": "", "holding_right": "O17"}),
                ),
        );
        let guard = Arc::new(Guard::new(crate::guard::Policy::default()));
        let out = RobotState::new(&profile(), robot, guard)
            .call(json!({}))
            .await;
        assert_eq!(out.data["room"]["id"], "R1");
        assert_eq!(out.data["armed"], false);
        assert_eq!(out.data["executor"]["holding_right"], "O17");
    }

    #[tokio::test]
    async fn stop_calls_stop_all() {
        let robot = Arc::new(FakeRobot::new().with_service("/x/stop", |_| {
            Ok(json!({"ok": true, "state_after": "holding posture"}))
        }));
        let port: Arc<dyn RobotPort> = Arc::clone(&robot) as Arc<dyn RobotPort>;
        let out = Stop::new(&profile(), port)
            .call(json!({"reason": "user"}))
            .await;
        assert_eq!(out.data["stopped"], true);
        assert_eq!(robot.calls()[0].1, json!({"reason": "user"}));
    }
}
