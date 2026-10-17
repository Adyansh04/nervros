//! `scene`: the obstacles MoveIt plans the arms around, as the operator says them, and the objects
//! a hand holds.
//!
//! NervROS keeps the obstacles it added on the map and sends them again in MoveIt's planning frame
//! whenever that frame moves on the map: a robot whose model is rooted at its body, with no virtual
//! joint to the world, would otherwise carry them along as it walks. Adding an obstacle only makes
//! the arms more careful, so it is free; removing one, attaching, detaching and clearing the octomap
//! ask the operator.

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::Transform;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use super::Arms;
use crate::tools::{Assessment, Resource, Risk, Tool, ToolOutcome, ToolSpec};

const CALL_WAIT: Duration = Duration::from_secs(5);
/// How often the planning frame's place on the map is checked.
const FOLLOW_PERIOD: Duration = Duration::from_secs(1);
/// How far the planning frame moves before the obstacles are sent again, m and rad.
const MOVED_M: f64 = 0.01;
const TURNED_RAD: f64 = 0.02;
/// `PlanningSceneComponents`: object names and geometry, and what the arms hold.
const OBJECTS_AND_HELD: u64 = 8 | 16 | 4;
/// `CollisionObject` operations.
const ADD: u8 = 0;
const REMOVE: u8 = 1;

/// An obstacle's shape, metres.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Shape {
    /// Its size along x, y and z.
    Box([f64; 3]),
    /// Its height and radius.
    Cylinder(f64, f64),
    /// Its radius.
    Sphere(f64),
}

impl Shape {
    /// As a `shape_msgs/msg/SolidPrimitive`.
    fn primitive(&self) -> Value {
        match self {
            Self::Box(s) => json!({"type": 1, "dimensions": s}),
            Self::Sphere(r) => json!({"type": 2, "dimensions": [r]}),
            Self::Cylinder(h, r) => json!({"type": 3, "dimensions": [h, r]}),
        }
    }

    fn words(&self) -> String {
        match self {
            Self::Box([x, y, z]) => format!("a {x:.2} × {y:.2} × {z:.2} m box"),
            Self::Cylinder(h, r) => format!("a cylinder {h:.2} m tall, {r:.2} m in radius"),
            Self::Sphere(r) => format!("a ball {r:.2} m in radius"),
        }
    }
}

/// An obstacle NervROS added, on the map.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct Obstacle {
    shape: Shape,
    /// Its centre on the map, metres.
    at: [f64; 3],
    /// Its turn about the map's z axis, radians.
    yaw: f64,
}

/// The tool.
pub(super) struct Scene {
    spec: ToolSpec,
    arms: Arc<Arms>,
    file: Option<PathBuf>,
    added: Arc<Mutex<BTreeMap<String, Obstacle>>>,
}

impl Scene {
    /// The tool; the obstacles it added in earlier sessions go back into MoveIt, and follow the
    /// planning frame.
    pub(super) fn new(arms: Arc<Arms>, file: Option<PathBuf>) -> Arc<Self> {
        let spec = ToolSpec::new(
            "scene",
            "MoveIt's planning scene: the obstacles the arms plan around, and what a hand holds. \
             list them. add an obstacle: a box (size [x, y, z]), a cylinder (height, radius) or a \
             sphere (radius) at x, y, z on the map, or an object from the world model by its id, \
             its box as the world model has it. remove one NervROS added. attach an object to an \
             arm's hand, so plans carry it, and detach it. clear_octomap forgets what the sensors \
             filled in, as when the arm's own past positions block it. Adding is free; the rest \
             need the operator's approval.",
            json!({
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["list", "add", "remove", "attach", "detach", "clear_octomap"]},
                    "id": {"type": "string", "description": "The obstacle's name in the scene"},
                    "object": {"type": "string", "description": "For add: an object's id from find_objects"},
                    "shape": {"type": "string", "enum": ["box", "cylinder", "sphere"]},
                    "size": {"type": "array", "items": {"type": "number"}, "description": "A box's size along x, y and z, m"},
                    "height": {"type": "number"},
                    "radius": {"type": "number"},
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                    "yaw": {"type": "number"},
                    "group": {"type": "string", "description": "For attach: the arm whose hand holds it"},
                },
                "required": ["action"],
            }),
            Risk::Observe,
        );
        let kept: BTreeMap<String, Obstacle> = file
            .as_deref()
            .map(crate::persist::read_or_default)
            .unwrap_or_default();
        let scene = Arc::new(Self {
            spec,
            arms,
            file,
            added: Arc::new(Mutex::new(kept)),
        });
        scene.follow();
        scene
    }

    /// Sends NervROS's obstacles again whenever the planning frame moves on the map.
    fn follow(self: &Arc<Self>) {
        let weak = Arc::downgrade(self);
        tokio::spawn(async move {
            let mut sent_at: Option<Transform> = None;
            loop {
                tokio::time::sleep(FOLLOW_PERIOD).await;
                let Some(scene) = weak.upgrade() else {
                    return;
                };
                if crate::lock(&scene.added).is_empty() {
                    sent_at = None;
                    continue;
                }
                let Ok(t) = scene.map_to_planning().await else {
                    continue;
                };
                if sent_at.is_some_and(|s| !moved(&s, &t)) {
                    continue;
                }
                let all = crate::lock(&scene.added).clone();
                let objects: Vec<Value> = all
                    .iter()
                    .filter_map(|(id, o)| scene.object(id, o, &t).ok())
                    .collect();
                if scene.apply_objects(objects).await.is_ok() {
                    sent_at = Some(t);
                }
            }
        });
    }

    /// The transform from the map into MoveIt's planning frame, and that frame's name.
    async fn map_to_planning(&self) -> Result<Transform, String> {
        let srdf = self.arms.srdf().await?;
        let planning = self.arms.planning_frame(&srdf)?;
        let map = &self.arms.profile.ros.map_frame;
        self.arms
            .robot
            .transform(&planning, map)
            .map_err(|e| format!("cannot place the map in {planning}: {e}"))
    }

    /// An obstacle as a `moveit_msgs/msg/CollisionObject` to add, in the planning frame.
    fn object(&self, id: &str, o: &Obstacle, map_to_planning: &Transform) -> Result<Value, String> {
        let planning = crate::lock(&self.arms.srdf)
            .as_ref()
            .map(|s| self.arms.planning_frame(s))
            .ok_or("MoveIt's SRDF is not read yet")??;
        let half = o.yaw / 2.0;
        let on_map = Transform {
            translation: o.at,
            rotation: [0.0, 0.0, half.sin(), half.cos()],
        };
        let pose = map_to_planning.compose(&on_map);
        let [x, y, z] = pose.translation;
        let [qx, qy, qz, qw] = pose.rotation;
        Ok(json!({
            "header": {"frame_id": planning},
            "id": id,
            "primitives": [o.shape.primitive()],
            "primitive_poses": [{"position": {"x": x, "y": y, "z": z}, "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}}],
            "operation": ADD,
        }))
    }

    async fn apply(&self, scene: Value) -> Result<(), String> {
        let service = self.arms.config.name("apply_planning_scene");
        let answer = self
            .arms
            .robot
            .call(
                &service,
                "moveit_msgs/srv/ApplyPlanningScene",
                json!({"scene": scene}),
                CALL_WAIT,
            )
            .await
            .map_err(|e| format!("MoveIt's planning scene did not answer: {e}"))?;
        if answer["success"].as_bool() == Some(true) {
            Ok(())
        } else {
            Err("MoveIt did not take the change to its planning scene".to_owned())
        }
    }

    async fn apply_objects(&self, objects: Vec<Value>) -> Result<(), String> {
        self.apply(json!({"is_diff": true, "world": {"collision_objects": objects}, "robot_state": {"is_diff": true}}))
            .await
    }

    fn save(&self) {
        let Some(file) = &self.file else {
            return;
        };
        let all = crate::lock(&self.added).clone();
        if let Err(e) = serde_json::to_vec_pretty(&all)
            .map_err(|e| e.to_string())
            .and_then(|text| crate::persist::write_atomic(file, &text).map_err(|e| e.to_string()))
        {
            tracing::warn!(error = %e, "the planning scene's obstacles were not kept");
        }
    }

    /// The obstacle `args` ask for, and its name.
    async fn obstacle_for(&self, args: &Value) -> Result<(String, Obstacle), String> {
        if let Some(id) = args["object"].as_str().filter(|s| !s.is_empty()) {
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
                .latest(&topic.topic, &topic.msg_type, CALL_WAIT)
                .await
                .map_err(|e| format!("the world model did not answer: {e}"))?;
            let found = objects["objects"]
                .as_array()
                .into_iter()
                .flatten()
                .find(|o| o["id"].as_str() == Some(id))
                .ok_or_else(|| format!("the world model has no object `{id}`"))?;
            let p = &found["pose"]["position"];
            let s = &found["size"];
            let size = [num(&s["x"]), num(&s["y"]), num(&s["z"])];
            if size.iter().any(|v| *v <= 0.0) {
                return Err(format!("the world model gives `{id}` no size"));
            }
            let yaw = Transform::from_pose(&found["pose"]).map_or(0.0, |t| t.yaw());
            let name = args["id"].as_str().filter(|s| !s.is_empty()).unwrap_or(id);
            return Ok((
                name.to_owned(),
                Obstacle {
                    shape: Shape::Box(size),
                    at: [num(&p["x"]), num(&p["y"]), num(&p["z"])],
                    yaw,
                },
            ));
        }
        let id = args["id"]
            .as_str()
            .map(str::trim)
            .filter(|s| !s.is_empty())
            .ok_or("add needs an id for the obstacle, or an object from the world model")?;
        let positive = |v: f64| v.is_finite() && v > 0.0;
        let shape = match args["shape"].as_str().unwrap_or("box") {
            "box" => {
                let size: Option<Vec<f64>> = args["size"]
                    .as_array()
                    .map(|a| a.iter().filter_map(Value::as_f64).collect());
                match size.as_deref() {
                    Some(&[x, y, z]) if positive(x) && positive(y) && positive(z) => {
                        Shape::Box([x, y, z])
                    }
                    _ => return Err("a box needs size [x, y, z], each above 0 m".to_owned()),
                }
            }
            "cylinder" => match (args["height"].as_f64(), args["radius"].as_f64()) {
                (Some(h), Some(r)) if positive(h) && positive(r) => Shape::Cylinder(h, r),
                _ => return Err("a cylinder needs height and radius above 0 m".to_owned()),
            },
            "sphere" => match args["radius"].as_f64() {
                Some(r) if positive(r) => Shape::Sphere(r),
                _ => return Err("a sphere needs a radius above 0 m".to_owned()),
            },
            other => return Err(format!("`{other}` is not box, cylinder or sphere")),
        };
        let (Some(x), Some(y), Some(z)) =
            (args["x"].as_f64(), args["y"].as_f64(), args["z"].as_f64())
        else {
            return Err("say where on the map: x, y and z of its centre".to_owned());
        };
        Ok((
            id.to_owned(),
            Obstacle {
                shape,
                at: [x, y, z],
                yaw: args["yaw"].as_f64().unwrap_or(0.0),
            },
        ))
    }

    async fn list(&self) -> ToolOutcome {
        let service = self.arms.config.name("get_planning_scene");
        let scene = match self
            .arms
            .robot
            .call(
                &service,
                "moveit_msgs/srv/GetPlanningScene",
                json!({"components": {"components": OBJECTS_AND_HELD}}),
                CALL_WAIT,
            )
            .await
        {
            Ok(s) => s,
            Err(e) => {
                return ToolOutcome::failed(format!("MoveIt's planning scene did not answer: {e}"));
            }
        };
        let added = crate::lock(&self.added).clone();
        let obstacles: Vec<Value> = scene["scene"]["world"]["collision_objects"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|o| {
                let id = o["id"].as_str().unwrap_or_default();
                json!({
                    "id": id,
                    "by": if added.contains_key(id) { "NervROS" } else { "the robot" },
                    "shape": added.get(id).map(|a| a.shape.words()),
                    "on_map": added.get(id).map(|a| a.at),
                })
            })
            .collect();
        let held: Vec<Value> = scene["scene"]["robot_state"]["attached_collision_objects"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|a| json!({"object": a["object"]["id"], "link": a["link_name"]}))
            .collect();
        ToolOutcome::ok(json!({"obstacles": obstacles, "held": held}))
    }

    async fn add(&self, args: &Value) -> ToolOutcome {
        let (id, obstacle) = match self.obstacle_for(args).await {
            Ok(o) => o,
            Err(why) => return ToolOutcome::failed(why),
        };
        let t = match self.map_to_planning().await {
            Ok(t) => t,
            Err(why) => return ToolOutcome::failed(why),
        };
        let object = match self.object(&id, &obstacle, &t) {
            Ok(o) => o,
            Err(why) => return ToolOutcome::failed(why),
        };
        if let Err(why) = self.apply_objects(vec![object]).await {
            return ToolOutcome::failed(why);
        }
        let words = obstacle.shape.words();
        crate::lock(&self.added).insert(id.clone(), obstacle);
        self.save();
        ToolOutcome {
            message: format!("MoveIt plans the arms around `{id}`, {words}"),
            ..ToolOutcome::ok(json!({"added": id}))
        }
    }

    async fn remove(&self, args: &Value) -> ToolOutcome {
        let id = args["id"].as_str().unwrap_or_default();
        if !crate::lock(&self.added).contains_key(id) {
            return ToolOutcome::failed(format!(
                "NervROS added no obstacle `{id}`; the robot's own stay as it set them"
            ));
        }
        let gone = json!({"id": id, "operation": REMOVE});
        if let Err(why) = self.apply_objects(vec![gone]).await {
            return ToolOutcome::failed(why);
        }
        crate::lock(&self.added).remove(id);
        self.save();
        ToolOutcome::ok(json!({"removed": id}))
    }

    async fn attach(&self, args: &Value, on: bool) -> ToolOutcome {
        let id = args["id"].as_str().unwrap_or_default();
        if id.is_empty() {
            return ToolOutcome::failed("say which object: id");
        }
        let srdf = match self.arms.srdf().await {
            Ok(s) => s,
            Err(why) => return ToolOutcome::failed(why),
        };
        let group = match Arms::group_named(&srdf, args["group"].as_str().unwrap_or_default()) {
            Ok(g) => g.name.clone(),
            Err(why) => return ToolOutcome::failed(why),
        };
        let Some(tip) = srdf.tip(&group) else {
            return ToolOutcome::failed(format!("the SRDF gives {group} no hand link"));
        };
        let held = json!({
            "link_name": tip,
            "object": {"id": id, "operation": if on { ADD } else { REMOVE }},
            "touch_links": srdf.hand_links(&group),
        });
        let scene = json!({"is_diff": true, "robot_state": {"is_diff": true, "attached_collision_objects": [held]}});
        match self.apply(scene).await {
            Ok(()) => {
                // An attached obstacle is the hand's now, not the map's.
                if on {
                    crate::lock(&self.added).remove(id);
                    self.save();
                }
                ToolOutcome::ok(json!({if on { "attached" } else { "detached" }: id, "link": tip}))
            }
            Err(why) => ToolOutcome::failed(why),
        }
    }

    async fn clear_octomap(&self) -> ToolOutcome {
        let service = self.arms.config.name("clear_octomap");
        match self
            .arms
            .robot
            .call(&service, "std_srvs/srv/Empty", json!({}), CALL_WAIT)
            .await
        {
            Ok(_) => ToolOutcome {
                message: "MoveIt's octomap is cleared; the sensors fill it again as they see"
                    .to_owned(),
                ..ToolOutcome::ok(json!({"cleared": "octomap"}))
            },
            Err(e) => ToolOutcome::failed(format!("MoveIt did not clear its octomap: {e}")),
        }
    }
}

#[async_trait]
impl Tool for Scene {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let id = args["id"].as_str().unwrap_or("?");
        let arms = vec![Resource::LeftArm, Resource::RightArm];
        let (resources, reason) = match args["action"].as_str().unwrap_or_default() {
            "remove" => (
                Vec::new(),
                format!(
                    "takes `{id}` out of MoveIt's planning scene: the arms may move through it"
                ),
            ),
            "attach" => (
                Vec::new(),
                format!("attaches `{id}` to a hand in MoveIt's scene: plans carry it there"),
            ),
            "detach" => (
                Vec::new(),
                format!("detaches `{id}` from the hand in MoveIt's scene"),
            ),
            "clear_octomap" => (
                arms,
                "clears MoveIt's octomap: until the sensors see them again, the arms plan as if \
                 nothing they saw were there"
                    .to_owned(),
            ),
            _ => return None,
        };
        Some(Ok(Assessment {
            risk: Risk::WorldEdit,
            resources,
            reason,
            args: None,
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str().unwrap_or("list") {
            "list" => self.list().await,
            "add" => self.add(&args).await,
            "remove" => self.remove(&args).await,
            "attach" => self.attach(&args, true).await,
            "detach" => self.attach(&args, false).await,
            "clear_octomap" => self.clear_octomap().await,
            other => ToolOutcome::failed(format!(
                "action `{other}` is not one of list, add, remove, attach, detach, clear_octomap"
            )),
        }
    }
}

/// Whether the planning frame moved on the map enough to send the obstacles again.
fn moved(before: &Transform, now: &Transform) -> bool {
    let d: f64 = before
        .translation
        .iter()
        .zip(now.translation)
        .map(|(a, b)| (a - b) * (a - b))
        .sum::<f64>()
        .sqrt();
    let dot: f64 = before
        .rotation
        .iter()
        .zip(now.rotation)
        .map(|(a, b)| a * b)
        .sum::<f64>()
        .abs()
        .min(1.0);
    d > MOVED_M || 2.0 * dot.acos() > TURNED_RAD
}

fn num(v: &Value) -> f64 {
    v.as_f64().unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_obstacles_follow_the_planning_frame_only_when_it_moves() {
        let a = Transform::IDENTITY;
        let nudged = Transform {
            translation: [0.005, 0.0, 0.0],
            ..Transform::IDENTITY
        };
        let walked = Transform {
            translation: [0.2, 0.0, 0.0],
            ..Transform::IDENTITY
        };
        let turned = Transform {
            rotation: [0.0, 0.0, (0.05_f64).sin(), (0.05_f64).cos()],
            ..Transform::IDENTITY
        };
        assert!(!moved(&a, &nudged));
        assert!(moved(&a, &walked));
        assert!(moved(&a, &turned));
        assert_eq!(
            Shape::Cylinder(0.3, 0.05).primitive(),
            json!({"type": 3, "dimensions": [0.3, 0.05]})
        );
    }
}
