//! Integrations: what NervROS knows of a ROS stack the robot runs, such as Nav2 or MoveIt.
//!
//! Each is switched on by its profile section. It finds its stack on the graph and says what is
//! missing, adds the tools its stack can serve and a line on its state to each turn, and cancels
//! what it sent when the robot is stopped. Nothing outside this module names a stack.

pub mod moveit;
pub mod nav2;

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::RobotPort;
use serde_json::{Value, json};

use crate::doctor::Check;
use crate::places::Places;
use crate::profile::Profile;
use crate::tools::Tool;
use crate::zones::Zones;

/// How long a parameter or a world-model read may take.
const READ_WAIT: Duration = Duration::from_secs(3);

/// A ROS stack NervROS works with.
#[async_trait]
pub trait Integration: Send + Sync + std::fmt::Debug {
    /// Its name, as its profile section has it: `nav2`.
    fn id(&self) -> &'static str;

    /// The tools it adds.
    fn tools(&self) -> Vec<Arc<dyn Tool>>;

    /// A line on what it last saw of its stack, for each turn; empty when it has nothing to say.
    fn status(&self) -> String;

    /// Cancels every goal it sent that still runs.
    async fn stop(&self);

    /// Where it reports what it draws; once the session has started.
    fn attach(&self, _session: &crate::session::SessionHandle) {}
}

/// What the integrations start from.
pub(crate) struct Parts<'a> {
    pub(crate) profile: &'a Profile,
    pub(crate) robot: &'a Arc<dyn RobotPort>,
    pub(crate) zones: &'a Arc<Zones>,
    pub(crate) places: &'a Arc<Places>,
    /// The quota ledger, beside which the robot's files are kept.
    pub(crate) ledger: &'a std::path::Path,
}

impl Parts<'_> {
    /// Where an integration keeps what it must remember across sessions, such as the shapes it
    /// gave the collision monitor.
    pub(crate) fn file(&self, kind: &str) -> Option<std::path::PathBuf> {
        crate::app::robot_file(self.profile, self.ledger, kind)
    }
}

/// The integrations the profile switches on.
#[derive(Debug, Default)]
pub struct Integrations {
    all: Vec<Arc<dyn Integration>>,
}

impl Integrations {
    /// Starts the integrations the profile's sections name. Must run inside a tokio runtime.
    pub(crate) fn start(parts: &Parts<'_>) -> Arc<Self> {
        let mut all: Vec<Arc<dyn Integration>> = Vec::new();
        if let Some(config) = &parts.profile.nav2 {
            all.push(nav2::Nav2::start(config, parts));
        }
        if let Some(config) = &parts.profile.moveit {
            all.push(moveit::MoveIt::start(config, parts));
        }
        Arc::new(Self { all })
    }

    /// Each, in a fixed order.
    pub fn iter(&self) -> impl Iterator<Item = &Arc<dyn Integration>> {
        self.all.iter()
    }

    /// Every tool they add.
    #[must_use]
    pub fn tools(&self) -> Vec<Arc<dyn Tool>> {
        self.all.iter().flat_map(|i| i.tools()).collect()
    }

    /// Gives each the session to report to.
    pub fn attach(&self, session: &crate::session::SessionHandle) {
        for i in &self.all {
            i.attach(session);
        }
    }

    /// Cancels what each sent; they stop together.
    pub async fn stop(&self) {
        futures::future::join_all(self.all.iter().map(|i| i.stop())).await;
    }
}

impl crate::memory::Notes for Integrations {
    fn text(&self) -> String {
        let lines: Vec<String> = self
            .all
            .iter()
            .map(|i| i.status())
            .filter(|s| !s.is_empty())
            .collect();
        if lines.is_empty() {
            return String::new();
        }
        format!("\n\nThe robot's stacks now:\n- {}", lines.join("\n- "))
    }
}

/// What the profile's integrations find of their stacks on the graph, for the doctor.
pub async fn discover(profile: &Profile, robot: &dyn RobotPort) -> Vec<Check> {
    let mut out = Vec::new();
    if let Some(config) = &profile.nav2 {
        out.extend(nav2::discover(config, robot).await);
    }
    if let Some(config) = &profile.moveit {
        out.extend(moveit::discover(config, robot).await);
    }
    out
}

/// `name` under `namespace` when it is relative, as a node in that namespace resolves it.
pub(crate) fn resolve(namespace: &str, name: &str) -> String {
    if name.starts_with('/') {
        return name.to_owned();
    }
    let ns = namespace.trim_end_matches('/');
    if ns.is_empty() {
        format!("/{name}")
    } else if ns.starts_with('/') {
        format!("{ns}/{name}")
    } else {
        format!("/{ns}/{name}")
    }
}

/// A node's parameters by name, each as plain JSON with its type id; one the node does not have is
/// null with type 0.
///
/// # Errors
///
/// The node did not answer.
pub(crate) async fn get_params(
    robot: &dyn RobotPort,
    node: &str,
    names: &[String],
) -> Result<Vec<(Value, u64)>, String> {
    let values = get_values(robot, node, names).await?;
    // rclcpp answers no values at all when one name is not declared: then each is asked alone.
    if values.len() == names.len() {
        return Ok(values.iter().map(typed).collect());
    }
    let mut out = Vec::with_capacity(names.len());
    for name in names {
        let one = get_values(robot, node, std::slice::from_ref(name)).await?;
        out.push(one.first().map_or((Value::Null, 0), typed));
    }
    Ok(out)
}

async fn get_values(
    robot: &dyn RobotPort,
    node: &str,
    names: &[String],
) -> Result<Vec<Value>, String> {
    let out = robot
        .call(
            &format!("{node}/get_parameters"),
            "rcl_interfaces/srv/GetParameters",
            json!({"names": names}),
            READ_WAIT,
        )
        .await
        .map_err(|e| format!("{node} did not give its parameters: {e}"))?;
    Ok(out["values"].as_array().cloned().unwrap_or_default())
}

/// A `ParameterValue` as plain JSON, with its type id.
fn typed(v: &Value) -> (Value, u64) {
    (
        crate::ros_tools::params::param_value(v),
        v["type"].as_u64().unwrap_or(0),
    )
}

/// Sets one of a node's parameters to `value`, a `rcl_interfaces/msg/ParameterValue` as JSON.
///
/// # Errors
///
/// The node did not answer or refused, in its words.
pub(crate) async fn set_param(
    robot: &dyn RobotPort,
    node: &str,
    name: &str,
    value: Value,
) -> Result<(), String> {
    let out = robot
        .call(
            &format!("{node}/set_parameters"),
            "rcl_interfaces/srv/SetParameters",
            json!({"parameters": [{"name": name, "value": value}]}),
            READ_WAIT,
        )
        .await
        .map_err(|e| format!("{node} did not answer: {e}"))?;
    let result = &out["results"][0];
    if result["successful"].as_bool() == Some(true) {
        Ok(())
    } else {
        Err(format!(
            "{node} refused {name}: {}",
            result["reason"].as_str().unwrap_or("no reason given")
        ))
    }
}

/// A point on the map a tool was asked about, and what it is.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Spot {
    /// In the map frame, metres.
    pub(crate) x: f64,
    /// In the map frame, metres.
    pub(crate) y: f64,
    /// Which way to face there, when it matters.
    pub(crate) yaw: Option<f64>,
    /// What it is, for the operator: a place's or room's name, an object's label.
    pub(crate) what: String,
}

/// Where `args` point: `place` (a named place), `room` (an id or name), `object` (an id; the spot
/// is beside it, toward `from`, about `clear_m` off its box) or `x` and `y` (and `yaw`) on the map.
///
/// # Errors
///
/// None of them, or one that names nothing known, in words for the model.
pub(crate) async fn locate(
    args: &Value,
    (profile, places, robot): (&Profile, &Places, &dyn RobotPort),
    from: Option<(f64, f64)>,
    clear_m: f64,
) -> Result<Spot, String> {
    let text = |key: &str| args[key].as_str().map(str::trim).filter(|v| !v.is_empty());
    if let Some(place) = text("place") {
        let same = |a: &str| crate::places::same_words(a, place);
        let found = places
            .all()
            .into_iter()
            .find(|p| same(&p.name) || p.aliases.iter().any(|a| same(a)));
        return found
            .map(|p| Spot {
                x: p.pose.x,
                y: p.pose.y,
                yaw: Some(p.pose.yaw),
                what: p.name,
            })
            .ok_or_else(|| format!("`{place}` is not a known place; list_places names them"));
    }
    let world = profile.world.as_ref();
    if let Some(room) = text("room") {
        let Some(topic) = world.and_then(|w| w.rooms.as_ref()) else {
            return Err("this robot has no world model with rooms".to_owned());
        };
        let rooms = robot
            .latest(&topic.topic, &topic.msg_type, READ_WAIT)
            .await
            .unwrap_or(Value::Null);
        let found = rooms["rooms"].as_array().into_iter().flatten().find(|r| {
            ["id", "name", "type"].iter().any(|k| {
                r[k].as_str()
                    .is_some_and(|v| crate::places::same_words(v, room))
            })
        });
        let Some(found) = found else {
            return Err(format!(
                "the world model has no room `{room}`; list_places names them"
            ));
        };
        let c = &found["centroid"];
        return match (c["x"].as_f64(), c["y"].as_f64()) {
            (Some(x), Some(y)) => Ok(Spot {
                x,
                y,
                yaw: None,
                what: crate::places::room_label(found).unwrap_or(room).to_owned(),
            }),
            _ => Err(format!("the world model gives `{room}` no centre")),
        };
    }
    if let Some(id) = text("object") {
        let Some(topic) = world.and_then(|w| w.objects.as_ref()) else {
            return Err("this robot has no world model with objects".to_owned());
        };
        let objects = robot
            .latest(&topic.topic, &topic.msg_type, READ_WAIT)
            .await
            .unwrap_or(Value::Null);
        return beside(&objects, id, from, clear_m);
    }
    match (args["x"].as_f64(), args["y"].as_f64()) {
        (Some(x), Some(y)) if x.is_finite() && y.is_finite() => Ok(Spot {
            x,
            y,
            yaw: args["yaw"].as_f64().filter(|v| v.is_finite()),
            what: format!("({x:.2}, {y:.2})"),
        }),
        _ => Err("say where: a place, a room, an object, or x and y on the map".to_owned()),
    }
}

/// The spot beside object `id` of the world model's `objects`, on the side toward `from`, about
/// `clear_m` off its box: its own centre is occupied.
fn beside(
    objects: &Value,
    id: &str,
    from: Option<(f64, f64)>,
    clear_m: f64,
) -> Result<Spot, String> {
    let found = objects["objects"]
        .as_array()
        .into_iter()
        .flatten()
        .find(|o| o["id"].as_str() == Some(id));
    let Some(found) = found else {
        return Err(format!(
            "the world model has no object `{id}`; find_objects gives ids"
        ));
    };
    let p = &found["pose"]["position"];
    let (Some(ox), Some(oy)) = (p["x"].as_f64(), p["y"].as_f64()) else {
        return Err(format!("`{id}` has no position in the world model"));
    };
    let size = &found["size"];
    let half = size["x"]
        .as_f64()
        .unwrap_or(0.0)
        .hypot(size["y"].as_f64().unwrap_or(0.0))
        / 2.0;
    // Beside it on the side it is seen from: its own centre is occupied.
    let (fx, fy) = from.unwrap_or((ox - 1.0, oy));
    let (dx, dy) = (fx - ox, fy - oy);
    let d = dx.hypot(dy).max(1e-6);
    let off = half + clear_m;
    let label = found["label"]
        .as_str()
        .filter(|l| !l.is_empty())
        .unwrap_or(id);
    Ok(Spot {
        x: ox + dx / d * off,
        y: oy + dy / d * off,
        yaw: Some((-dy).atan2(-dx)),
        what: label.to_owned(),
    })
}

/// A pose in the map frame for a goal: `geometry_msgs/msg/PoseStamped` as JSON.
pub(crate) fn stamped(frame: &str, x: f64, y: f64, yaw: f64) -> Value {
    json!({
        "header": {"frame_id": frame},
        "pose": {
            "position": {"x": x, "y": y, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": (yaw / 2.0).sin(), "w": (yaw / 2.0).cos()},
        },
    })
}

/// Whether the graph's topics show the action `name`: its feedback topic is there.
pub(crate) fn has_action(topics: &nervros_ros::NamesAndTypes, name: &str) -> bool {
    let feedback = format!("{name}/_action/feedback");
    topics.iter().any(|(t, _)| *t == feedback)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::Notes as _;

    #[derive(Debug)]
    struct Says(&'static str);

    #[async_trait]
    impl Integration for Says {
        fn id(&self) -> &'static str {
            "says"
        }
        fn tools(&self) -> Vec<Arc<dyn Tool>> {
            Vec::new()
        }
        fn status(&self) -> String {
            self.0.to_owned()
        }
        async fn stop(&self) {}
    }

    #[tokio::test]
    async fn parameters_are_read_one_by_one_when_one_is_not_declared() {
        // As rclcpp does: no values at all when any name asked for is not declared.
        let robot = nervros_ros::fake::FakeRobot::new().with_service("/n/get_parameters", |req| {
            let names = req["names"].as_array().cloned().unwrap_or_default();
            let declared = |n: &Value| n == "a" || n == "b";
            if !names.iter().all(declared) {
                return Ok(json!({"values": []}));
            }
            let values: Vec<Value> = names
                .iter()
                .map(|n| json!({"type": 4, "string_value": n}))
                .collect();
            Ok(json!({"values": values}))
        });
        let names = ["a".to_owned(), "nope".to_owned(), "b".to_owned()];
        let got = get_params(&robot, "/n", &names).await.unwrap();
        assert_eq!(
            got,
            vec![(json!("a"), 4), (Value::Null, 0), (json!("b"), 4)]
        );
    }

    #[test]
    fn a_name_is_taken_under_the_namespace_unless_it_is_absolute() {
        assert_eq!(resolve("", "navigate_to_pose"), "/navigate_to_pose");
        assert_eq!(
            resolve("/robot1", "navigate_to_pose"),
            "/robot1/navigate_to_pose"
        );
        assert_eq!(resolve("robot1/", "spin"), "/robot1/spin");
        assert_eq!(resolve("/robot1", "/move_group"), "/move_group");
    }

    #[test]
    fn each_turn_hears_only_the_stacks_with_something_to_say() {
        let quiet = Integrations::default();
        assert_eq!(quiet.text(), "");
        let some = Integrations {
            all: vec![Arc::new(Says("Nav2: idle")), Arc::new(Says(""))],
        };
        assert_eq!(some.text(), "\n\nThe robot's stacks now:\n- Nav2: idle");
    }
}
