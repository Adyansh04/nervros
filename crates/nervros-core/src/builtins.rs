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
use crate::profile::{MissionConfig, PlaceConfig, Profile, TopicRef};
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

const WORLD_WAIT: Duration = Duration::from_secs(2);

/// Rooms from the world model as `(id, name, type, outline)`, or none when it does not run.
async fn rooms(
    robot: &dyn RobotPort,
    topic: Option<&TopicRef>,
) -> Vec<(String, String, String, Vec<(f64, f64)>)> {
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
                    (text("id"), text("name"), text("type"), outline)
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
    places: Vec<PlaceConfig>,
    rooms: Option<TopicRef>,
    robot: Arc<dyn RobotPort>,
}

impl ListPlaces {
    /// From the profile.
    #[must_use]
    pub fn new(profile: &Profile, robot: Arc<dyn RobotPort>) -> Self {
        let spec = ToolSpec::new(
            "list_places",
            "Lists the places the robot can go to: named places with their aliases, and the rooms the \
             world model knows. Use the returned ids in plans.",
            json!({"type": "object", "properties": {}, "additionalProperties": false}),
            Risk::Observe,
        );
        let rooms = profile.world.as_ref().and_then(|w| w.rooms.clone());
        Self {
            spec,
            places: profile.places.clone(),
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
            .iter()
            .map(|p| json!({"id": p.name, "aliases": p.aliases, "kind": "place"}))
            .collect();
        let rooms: Vec<Value> = rooms(self.robot.as_ref(), self.rooms.as_ref())
            .await
            .into_iter()
            .map(
                |(id, name, kind, _)| json!({"id": id, "name": name, "type": kind, "kind": "room"}),
            )
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
        match self.robot.transform(&self.frames.0, &self.frames.1) {
            Ok(t) => {
                let [x, y, _] = t.translation;
                let round = |v: f64| (v * 100.0).round() / 100.0;
                out["pose"] = json!({"frame": self.frames.0, "x": round(x), "y": round(y), "yaw": round(t.yaw())});
                let room = rooms(self.robot.as_ref(), self.rooms.as_ref())
                    .await
                    .into_iter()
                    .find(|(_, _, _, outline)| inside((x, y), outline));
                if let Some((id, name, kind, _)) = room {
                    out["room"] = json!({"id": id, "name": name, "type": kind});
                }
            }
            Err(e) => out["pose_error"] = Value::String(e.to_string()),
        }
        if let Some(topic) = &self.executor_state
            && let Ok(state) = self
                .robot
                .latest(
                    topic,
                    "nervros_interfaces/msg/RobotState",
                    Duration::from_secs(1),
                )
                .await
        {
            let field = |k: &str| state[k].clone();
            out["executor"] = json!({
                "mission": field("mission_id"),
                "step": field("mission_step"),
                "holding_left": field("holding_left"),
                "holding_right": field("holding_right"),
                "resources_held": field("resources_held"),
                "stopped": field("stopped"),
            });
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
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("nervros.toml");
        std::fs::write(&path, text).unwrap();
        Profile::load(Path::new(&path)).unwrap()
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
        let out = ListPlaces::new(&profile(), robot).call(json!({})).await;
        assert_eq!(out.data["places"][0]["id"], "zone_a");
        assert_eq!(out.data["rooms"][0]["name"], "kitchen");
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
