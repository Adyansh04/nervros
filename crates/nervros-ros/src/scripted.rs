//! A robot from a file: what the scenario tests talk to, and what `nervros-cli eval --robot` runs a
//! suite against, so models can be compared in seconds, without a simulator.
//!
//! Services answer with a fixed response, or one chosen by a word in the request; actions run to a
//! fixed end and topics hold one message. A camera gives the one image its file holds, and a
//! mission that succeeds moves the robot as its plan says, so a suite can judge looking, walking
//! and turning. The graph, what each node publishes and subscribes to, the nodes each component
//! container hosts, parameters, topic rates and frames make it look to the ROS tools like a live
//! graph; a service the graph lists without a response is not reachable.
//! Callers parse the file (TOML in this repository) into [`ScriptedRobot`] and
//! [`ScriptedRobot::build`] it.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde::Deserialize;
use serde_json::{Value, json};

use crate::fake::{FakeRobot, ScriptedRun};
use crate::image::Frame;
use crate::lock;
use crate::tf::TfBuffer;
use crate::{
    Endpoint, GoalResult, GoalStatus, GraphDetail, NamesAndTypes, NodeEntities, QosInfo,
    TopicEndpoints, Transform,
};

/// The service every rclcpp component container offers to list what it runs.
const LIST_NODES: &str = "composition_interfaces/srv/ListNodes";

/// A robot as a file says it is.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScriptedRobot {
    /// Services by name, each with the response it always gives.
    #[serde(default)]
    pub services: BTreeMap<String, Value>,
    /// Services by name, each answering by a word in the request.
    #[serde(default)]
    pub replies: BTreeMap<String, Reply>,
    /// Actions by name, each with how a goal runs.
    #[serde(default)]
    pub actions: BTreeMap<String, Run>,
    /// Topics by name, each with the message it holds.
    #[serde(default)]
    pub topics: BTreeMap<String, Value>,
    /// The graph's topic and service types, and its nodes.
    #[serde(default)]
    pub graph: GraphFile,
    /// Component containers by name, each with the nodes it hosts.
    #[serde(default)]
    pub containers: BTreeMap<String, Vec<String>>,
    /// What a node publishes, subscribes to, serves and calls.
    #[serde(default)]
    pub nodes: BTreeMap<String, NodeFile>,
    /// Parameters by node, then by name.
    #[serde(default)]
    pub params: BTreeMap<String, BTreeMap<String, Param>>,
    /// Topics that arrive at a rate.
    #[serde(default)]
    pub rates: BTreeMap<String, Rate>,
    /// Frames, each a child's pose in its parent.
    #[serde(default)]
    pub transforms: Vec<FrameFile>,
    /// Camera topics, each with the image file it gives. A relative path is the caller's to resolve
    /// against the robot file's folder.
    #[serde(default)]
    pub frames: BTreeMap<String, PathBuf>,
    /// How a mission moves the robot; without it, nothing does.
    pub motion: Option<Motion>,
}

/// A service that answers by what a request's text field says.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Reply {
    /// The request's field to read.
    pub field: String,
    /// Answers by word: the longest one the field holds, in lower case, answers.
    pub by: BTreeMap<String, Value>,
    /// The answer when none does.
    pub otherwise: Value,
}

impl Reply {
    fn answer(&self, request: &Value) -> Value {
        let said = request[&self.field]
            .as_str()
            .unwrap_or_default()
            .to_lowercase();
        self.by
            .iter()
            .filter(|(word, _)| said.contains(word.as_str()))
            .max_by_key(|(word, _)| word.len())
            .map_or(&self.otherwise, |(_, answer)| answer)
            .clone()
    }
}

/// How a successful mission moves the robot: its plan, read from the goal's behaviour tree, step by
/// step, nothing in the way.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Motion {
    /// The mission action.
    pub action: String,
    /// A path planner's action, answered with a straight line to the goal.
    pub planner: Option<String>,
    /// The map's frame.
    pub parent: String,
    /// The base's frame.
    pub child: String,
    /// The skill that walks a distance.
    pub walk: Option<Walk>,
    /// The skill that turns on the spot.
    pub turn: Option<Turn>,
    /// Where any step's argument naming one of these sends the robot.
    #[serde(default)]
    pub places: BTreeMap<String, Spot>,
    /// How long a mission takes, seconds.
    #[serde(default = "mission_seconds")]
    pub seconds: f64,
}

fn mission_seconds() -> f64 {
    1.0
}

/// The skill that walks a distance, and its arguments' names.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Walk {
    /// The skill.
    pub skill: String,
    /// Its metres.
    pub metres: String,
    /// Its direction: `forward`, `backward`, `left` or `right`.
    pub direction: Option<String>,
}

/// The skill that turns on the spot, and its argument's name.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Turn {
    /// The skill.
    pub skill: String,
    /// Its degrees.
    pub degrees: String,
    /// Whether positive degrees turn left.
    #[serde(default = "turns_left")]
    pub positive_left: bool,
}

fn turns_left() -> bool {
    true
}

/// A pose on the map.
#[derive(Debug, Clone, Copy, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Spot {
    /// Metres.
    pub x: f64,
    /// Metres.
    pub y: f64,
    /// Radians, or none to keep the robot's heading.
    pub yaw: Option<f64>,
}

impl Motion {
    /// Where the plan in `tree` leaves a robot at `(x, y, yaw)`; an unreadable tree moves nothing.
    fn after(&self, tree: &str, (mut x, mut y, mut yaw): (f64, f64, f64)) -> (f64, f64, f64) {
        let Ok(doc) = roxmltree::Document::parse(tree) else {
            return (x, y, yaw);
        };
        let number = |node: &roxmltree::Node, name: &str| -> Option<f64> {
            node.attribute(name)?.parse().ok()
        };
        for node in doc.descendants().filter(roxmltree::Node::is_element) {
            // NervROS writes each step as a `SubTree` that names its skill.
            let skill = node
                .attribute("ID")
                .unwrap_or_else(|| node.tag_name().name());
            if let Some(walk) = self.walk.as_ref().filter(|w| w.skill == skill) {
                let metres = number(&node, &walk.metres).unwrap_or(0.0);
                let side = walk.direction.as_deref().and_then(|d| node.attribute(d));
                let (along, across) = match side {
                    Some("backward" | "back") => (-metres, 0.0),
                    Some("left") => (0.0, metres),
                    Some("right") => (0.0, -metres),
                    _ => (metres, 0.0),
                };
                x += along * yaw.cos() - across * yaw.sin();
                y += along * yaw.sin() + across * yaw.cos();
            } else if let Some(turn) = self.turn.as_ref().filter(|t| t.skill == skill) {
                let degrees = number(&node, &turn.degrees).unwrap_or(0.0);
                let sign = if turn.positive_left { 1.0 } else { -1.0 };
                yaw += sign * degrees.to_radians();
            } else if let Some(spot) = node.attributes().find_map(|a| {
                self.places
                    .get(a.value())
                    .copied()
                    .or_else(|| station(a.value()))
            }) {
                (x, y) = (spot.x, spot.y);
                yaw = spot.yaw.unwrap_or(yaw);
            }
        }
        (x, y, yaw)
    }

    /// Moves the robot in `tf` as `goal`'s plan says.
    fn apply(&self, tf: &Mutex<TfBuffer>, goal: &Value) {
        let mut tf = lock(tf);
        let Ok(now) = tf.lookup(&self.parent, &self.child) else {
            return;
        };
        let tree = goal["tree_xml"].as_str().unwrap_or_default();
        let (x, y, yaw) = self.after(tree, now.planar());
        tf.insert(
            &self.parent,
            &self.child,
            pose_at(x, y, now.translation[2], yaw),
        );
    }

    /// A straight path from the robot, or the goal's start, to its target.
    fn path(&self, tf: &Mutex<TfBuffer>, goal: &Value) -> Value {
        let at = |p: &Value| json!({"pose": {"position": {"x": p["x"], "y": p["y"]}}});
        let target = &goal["goal"]["pose"]["position"];
        let start = if goal["use_start"].as_bool().unwrap_or(false) {
            goal["start"]["pose"]["position"].clone()
        } else {
            let (x, y, _) = lock(tf)
                .lookup(&self.parent, &self.child)
                .map_or((0.0, 0.0, 0.0), |t| t.planar());
            json!({"x": x, "y": y})
        };
        json!({"error_code": 0, "path": {"poses": [at(&start), at(target)]}})
    }
}

/// A `station` argument, `x;y;yaw`, as the executor contract writes a pose.
fn station(value: &str) -> Option<Spot> {
    let numbers: Vec<f64> = value
        .split(';')
        .map(|n| n.trim().parse())
        .collect::<Result<_, _>>()
        .ok()?;
    match numbers[..] {
        [x, y, yaw] => Some(Spot {
            x,
            y,
            yaw: Some(yaw),
        }),
        _ => None,
    }
}

fn pose_at(x: f64, y: f64, z: f64, yaw: f64) -> Transform {
    let half = yaw / 2.0;
    Transform {
        translation: [x, y, z],
        rotation: [0.0, 0.0, half.sin(), half.cos()],
    }
}

/// How often a scripted camera sends its frame again.
const FRAME_PERIOD: Duration = Duration::from_millis(500);

/// Seconds since the epoch.
pub(crate) fn now_s() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// An image file as a camera's one frame.
fn frame_of(topic: &str, file: &std::path::Path) -> Result<Frame, String> {
    let image = image::open(file)
        .map_err(|e| format!("{topic}: {}: {e}", file.display()))?
        .into_rgb8();
    let (width, height) = image.dimensions();
    Ok(Frame {
        stamp_s: now_s(),
        frame_id: "scripted".to_owned(),
        width,
        height,
        encoding: "rgb8".to_owned(),
        step: width * 3,
        is_bigendian: false,
        data: image.into_raw().into(),
    })
}

/// How a goal runs.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Run {
    /// Feedback messages, in order.
    #[serde(default)]
    pub feedback: Vec<Value>,
    /// `succeeded`, `aborted` or `canceled`.
    pub status: String,
    /// The result message.
    #[serde(default)]
    pub result: Value,
}

/// Types by name, and nodes.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GraphFile {
    /// Topics and their types.
    #[serde(default)]
    pub topics: BTreeMap<String, String>,
    /// Services and their types.
    #[serde(default)]
    pub services: BTreeMap<String, String>,
    /// Full node names.
    #[serde(default)]
    pub nodes: Vec<String>,
}

/// A node's topics and services, each name with its type.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeFile {
    /// Topics it publishes.
    #[serde(default)]
    pub publishers: BTreeMap<String, String>,
    /// Topics it subscribes to.
    #[serde(default)]
    pub subscribers: BTreeMap<String, String>,
    /// Services it offers.
    #[serde(default)]
    pub services: BTreeMap<String, String>,
    /// Services it calls.
    #[serde(default)]
    pub clients: BTreeMap<String, String>,
}

/// A parameter: its value, or its value with a description.
#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum Param {
    /// Value and description.
    Described {
        /// The value.
        value: Value,
        /// What it is for.
        #[serde(default)]
        description: String,
        /// Whether it may not be set.
        #[serde(default)]
        read_only: bool,
    },
    /// The value alone.
    Plain(Value),
}

impl Param {
    fn value(&self) -> &Value {
        match self {
            Self::Described { value, .. } | Self::Plain(value) => value,
        }
    }
}

/// A topic's rate and message size.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Rate {
    /// Messages a second.
    pub hz: f64,
    /// Bytes a message.
    #[serde(default)]
    pub bytes: usize,
}

/// A child frame's planar pose in its parent.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FrameFile {
    /// The parent frame.
    pub parent: String,
    /// The child frame.
    pub child: String,
    /// Metres.
    #[serde(default)]
    pub x: f64,
    /// Metres.
    #[serde(default)]
    pub y: f64,
    /// Metres.
    #[serde(default)]
    pub z: f64,
    /// Radians about z.
    #[serde(default)]
    pub yaw: f64,
}

/// A JSON value as a `rcl_interfaces/msg/ParameterValue`.
fn parameter_value(value: &Value) -> Value {
    let all = |items: &[Value], f: fn(&Value) -> bool| !items.is_empty() && items.iter().all(f);
    match value {
        Value::Bool(b) => json!({"type": 1, "bool_value": b}),
        Value::Number(n) if n.is_i64() || n.is_u64() => json!({"type": 2, "integer_value": n}),
        Value::Number(n) => json!({"type": 3, "double_value": n}),
        Value::String(s) => json!({"type": 4, "string_value": s}),
        Value::Array(items) if all(items, Value::is_boolean) => {
            json!({"type": 6, "bool_array_value": items})
        }
        Value::Array(items) if all(items, |v| v.is_i64() || v.is_u64()) => {
            json!({"type": 7, "integer_array_value": items})
        }
        Value::Array(items) if all(items, Value::is_number) => {
            json!({"type": 8, "double_array_value": items})
        }
        Value::Array(items) if all(items, Value::is_string) => {
            json!({"type": 9, "string_array_value": items})
        }
        _ => json!({"type": 0}),
    }
}

/// Each topic's publishers and subscribers, from what the nodes say they do; their `QoS` is
/// the ROS default, as a file does not say.
fn endpoints(nodes: &BTreeMap<String, NodeFile>) -> BTreeMap<String, TopicEndpoints> {
    let mut out: BTreeMap<String, TopicEndpoints> = BTreeMap::new();
    for (node, n) in nodes {
        let endpoint = |ty: &String| Endpoint {
            node: node.clone(),
            topic_type: ty.clone(),
            qos: QosInfo {
                reliability: "reliable".into(),
                durability: "volatile".into(),
                history: "keep_last".into(),
                depth: 10,
            },
        };
        for (topic, ty) in &n.publishers {
            out.entry(topic.clone())
                .or_default()
                .publishers
                .push(endpoint(ty));
        }
        for (topic, ty) in &n.subscribers {
            out.entry(topic.clone())
                .or_default()
                .subscribers
                .push(endpoint(ty));
        }
    }
    out
}

fn names_and_types(map: &BTreeMap<String, String>) -> NamesAndTypes {
    map.iter()
        .map(|(n, t)| (n.clone(), vec![t.clone()]))
        .collect()
}

impl ScriptedRobot {
    /// This robot with `other`'s entries in place of its own, key by key.
    #[must_use]
    pub fn with(mut self, other: Self) -> Self {
        self.services.extend(other.services);
        self.replies.extend(other.replies);
        self.frames.extend(other.frames);
        self.motion = other.motion.or(self.motion);
        self.actions.extend(other.actions);
        self.topics.extend(other.topics);
        self.graph.topics.extend(other.graph.topics);
        self.graph.services.extend(other.graph.services);
        self.graph.nodes.extend(other.graph.nodes);
        self.containers.extend(other.containers);
        self.nodes.extend(other.nodes);
        self.params.extend(other.params);
        self.rates.extend(other.rates);
        self.transforms.extend(other.transforms);
        self
    }

    /// The robot, answering as the file says.
    ///
    /// # Errors
    ///
    /// An action's status is not one of succeeded, aborted or canceled, a rate is out of reach, or
    /// a camera's image cannot be read.
    pub fn build(self) -> Result<FakeRobot, String> {
        let mut robot = FakeRobot::new().with_idle_cameras();
        let mut graph = GraphDetail {
            topics: names_and_types(&self.graph.topics),
            services: names_and_types(&self.graph.services),
            nodes: self.graph.nodes.clone(),
        };
        for (name, response) in self.services {
            robot = robot.with_service(&name, move |_| Ok(response.clone()));
        }
        for (name, reply) in self.replies {
            robot = robot.with_service(&name, move |request| Ok(reply.answer(request)));
        }
        for (topic, file) in &self.frames {
            robot = robot.with_camera(topic, frame_of(topic, file)?, FRAME_PERIOD);
        }
        robot = with_actions(robot, self.actions, self.motion.as_ref())?;
        for (name, message) in self.topics {
            robot = robot.with_topic(&name, message);
        }
        for (container, hosted) in self.containers {
            let service = format!("{container}/_container/list_nodes");
            graph
                .services
                .push((service.clone(), vec![LIST_NODES.to_owned()]));
            graph.nodes.push(container);
            graph.nodes.extend(hosted.iter().cloned());
            let ids: Vec<usize> = (1..=hosted.len()).collect();
            robot = robot.with_service(&service, move |_| {
                Ok(json!({"full_node_names": hosted, "unique_ids": ids}))
            });
        }
        for (node, params) in self.params {
            robot = with_params(robot, &mut graph, &node, params);
        }
        for (topic, ends) in endpoints(&self.nodes) {
            robot = robot.with_endpoints(&topic, ends);
        }
        for (name, n) in self.nodes {
            robot = robot.with_node(
                &name,
                NodeEntities {
                    publishers: names_and_types(&n.publishers),
                    subscribers: names_and_types(&n.subscribers),
                    services: names_and_types(&n.services),
                    clients: names_and_types(&n.clients),
                },
            );
        }
        for (topic, rate) in self.rates {
            // A rate makes the sampled period; zero, below or past these is not one.
            if !(0.001..=10_000.0).contains(&rate.hz) {
                return Err(format!("{topic}: hz must be from 0.001 to 10000"));
            }
            robot = robot.with_rate(&topic, rate.hz, rate.bytes);
        }
        for f in self.transforms {
            robot = robot.with_transform(&f.parent, &f.child, pose_at(f.x, f.y, f.z, f.yaw));
        }
        if graph != GraphDetail::default() {
            graph.topics.sort();
            graph.topics.dedup();
            graph.services.sort();
            graph.services.dedup();
            graph.nodes.sort();
            graph.nodes.dedup();
            robot = robot.with_graph(graph);
        }
        Ok(robot)
    }
}

/// `robot` with the file's actions, and the planner and mission that move it as `motion` says.
fn with_actions(
    mut robot: FakeRobot,
    actions: BTreeMap<String, Run>,
    motion: Option<&Motion>,
) -> Result<FakeRobot, String> {
    let tf = robot.tf();
    if let Some(motion) = motion.cloned().filter(|m| m.planner.is_some()) {
        let planner = motion.planner.clone().unwrap_or_default();
        let tf = Arc::clone(&tf);
        robot = robot.with_action(&planner, move |goal| ScriptedRun {
            result: Ok(GoalResult {
                status: GoalStatus::Succeeded,
                result: motion.path(&tf, goal),
            }),
            ..ScriptedRun::default()
        });
    }
    for (name, run) in actions {
        let status = match run.status.as_str() {
            "succeeded" => GoalStatus::Succeeded,
            "aborted" => GoalStatus::Aborted,
            "canceled" => GoalStatus::Canceled,
            other => return Err(format!("{name}: no status {other}")),
        };
        let motion = motion.cloned().filter(|m| m.action == name);
        let tf = Arc::clone(&tf);
        robot = robot.with_action(&name, move |goal| {
            let mut step = ScriptedRun::default().step;
            if let Some(motion) = motion.as_ref().filter(|_| status == GoalStatus::Succeeded) {
                motion.apply(&tf, goal);
                step = Duration::from_secs_f64(motion.seconds.max(0.0));
            }
            ScriptedRun {
                feedback: run.feedback.clone(),
                result: Ok(GoalResult {
                    status,
                    result: run.result.clone(),
                }),
                step,
            }
        });
    }
    Ok(robot)
}

/// A node's parameter services, answering from `params`; a set is refused, since the file is the
/// robot.
fn with_params(
    robot: FakeRobot,
    graph: &mut GraphDetail,
    node: &str,
    params: BTreeMap<String, Param>,
) -> FakeRobot {
    let services = [
        ("list_parameters", "rcl_interfaces/srv/ListParameters"),
        ("get_parameters", "rcl_interfaces/srv/GetParameters"),
        (
            "describe_parameters",
            "rcl_interfaces/srv/DescribeParameters",
        ),
        ("set_parameters", "rcl_interfaces/srv/SetParameters"),
    ];
    for (service, ty) in services {
        graph
            .services
            .push((format!("{node}/{service}"), vec![ty.to_owned()]));
    }
    graph.nodes.push(node.to_owned());
    let (listed, got, described) = (params.clone(), params.clone(), params);
    let requested = |request: &Value| -> Vec<String> {
        request["names"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|n| n.as_str().map(str::to_owned))
            .collect()
    };
    robot
        .with_service(&format!("{node}/list_parameters"), move |request| {
            let prefixes: Vec<&str> = request["prefixes"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(Value::as_str)
                .collect();
            let names: Vec<&String> = listed
                .keys()
                .filter(|n| prefixes.is_empty() || prefixes.iter().any(|p| n.starts_with(p)))
                .collect();
            Ok(json!({"result": {"names": names, "prefixes": []}}))
        })
        .with_service(&format!("{node}/get_parameters"), move |request| {
            let values: Vec<Value> = requested(request)
                .iter()
                .map(|n| got.get(n).map_or(json!({"type": 0}), |p| parameter_value(p.value())))
                .collect();
            Ok(json!({"values": values}))
        })
        .with_service(&format!("{node}/describe_parameters"), move |request| {
            let descriptors: Vec<Value> = requested(request)
                .iter()
                .map(|n| match described.get(n) {
                    Some(Param::Described {
                        value,
                        description,
                        read_only,
                    }) => json!({
                        "name": n,
                        "type": parameter_value(value)["type"],
                        "description": description,
                        "read_only": read_only,
                    }),
                    Some(Param::Plain(value)) => {
                        json!({"name": n, "type": parameter_value(value)["type"], "description": ""})
                    }
                    None => json!({"name": n, "type": 0, "description": ""}),
                })
                .collect();
            Ok(json!({"descriptors": descriptors}))
        })
        .with_service(&format!("{node}/set_parameters"), |request| {
            let n = request["parameters"].as_array().map_or(0, Vec::len);
            Ok(json!({"results": vec![
                json!({"successful": false, "reason": "a scripted robot keeps its parameters"});
                n
            ]}))
        })
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;
    use crate::RobotPort;

    fn robot(text: &serde_json::Value) -> FakeRobot {
        serde_json::from_value::<ScriptedRobot>(text.clone())
            .unwrap()
            .build()
            .unwrap()
    }

    #[tokio::test]
    async fn a_file_scripts_the_graph_containers_and_parameters() {
        let robot = robot(&json!({
            "graph": {"topics": {"/scan": "sensor_msgs/msg/LaserScan"}},
            "containers": {"/nav2_container": ["/amcl", "/controller_server"]},
            "params": {"/controller_server": {
                "FollowPath.desired_linear_vel": {"value": 0.6, "description": "Cruise speed, m/s."},
                "use_sim_time": true,
            }},
            "rates": {"/scan": {"hz": 10.0, "bytes": 7000}},
            "transforms": [{"parent": "map", "child": "base_footprint", "x": 1.0, "yaw": 0.5}],
        }));
        let graph = robot.graph_detail().await.unwrap();
        assert!(graph.nodes.contains(&"/amcl".to_owned()), "{graph:?}");
        assert!(
            graph
                .services
                .iter()
                .any(|(n, _)| n == "/controller_server/describe_parameters")
        );
        let hosted = robot
            .call(
                "/nav2_container/_container/list_nodes",
                LIST_NODES,
                json!({}),
                Duration::from_secs(1),
            )
            .await
            .unwrap();
        assert_eq!(
            hosted["full_node_names"],
            json!(["/amcl", "/controller_server"])
        );
        let values = robot
            .call(
                "/controller_server/get_parameters",
                "rcl_interfaces/srv/GetParameters",
                json!({"names": ["FollowPath.desired_linear_vel", "use_sim_time", "nope"]}),
                Duration::from_secs(1),
            )
            .await
            .unwrap();
        assert_eq!(
            values["values"],
            json!([{"type": 3, "double_value": 0.6}, {"type": 1, "bool_value": true}, {"type": 0}])
        );
        let described = robot
            .call(
                "/controller_server/describe_parameters",
                "rcl_interfaces/srv/DescribeParameters",
                json!({"names": ["FollowPath.desired_linear_vel"]}),
                Duration::from_secs(1),
            )
            .await
            .unwrap();
        assert_eq!(
            described["descriptors"][0]["description"],
            "Cruise speed, m/s."
        );
        let pose = robot.transform("map", "base_footprint").unwrap();
        assert!((pose.translation[0] - 1.0).abs() < 1e-9);
        assert!((pose.yaw() - 0.5).abs() < 1e-9);
    }

    #[tokio::test]
    async fn the_nodes_say_who_publishes_a_camera_waits_and_a_rate_must_be_one() {
        let robot = robot(&json!({
            "nodes": {"/laser": {"publishers": {"/scan": "sensor_msgs/msg/LaserScan"}},
                      "/amcl": {"subscribers": {"/scan": "sensor_msgs/msg/LaserScan"}}},
        }));
        let ends = robot.endpoints("/scan").await.unwrap();
        assert_eq!(ends.publishers[0].node, "/laser");
        assert_eq!(ends.subscribers[0].node, "/amcl");
        assert!(robot.frames("/camera/image").unwrap().borrow().is_none());
        for hz in [0.0, -1.0, f64::NAN] {
            let file = ScriptedRobot {
                rates: BTreeMap::from([("/scan".to_owned(), Rate { hz, bytes: 0 })]),
                ..ScriptedRobot::default()
            };
            assert!(file.build().is_err(), "{hz}");
        }
    }

    #[tokio::test]
    async fn a_mission_moves_the_robot_as_its_plan_says() {
        let robot = robot(&json!({
            "actions": {"/execute": {"status": "succeeded"}},
            "transforms": [{"parent": "map", "child": "base"}],
            "motion": {
                "action": "/execute", "planner": "/plan", "parent": "map", "child": "base",
                "seconds": 0.0,
                "walk": {"skill": "Walk", "metres": "m", "direction": "way"},
                "turn": {"skill": "Turn", "degrees": "deg"},
                "places": {"kitchen": {"x": 3.0, "y": 4.0}},
            },
        }));
        let run = |tree: &'static str| {
            let robot = &robot;
            async move {
                let goal = robot
                    .send_goal(
                        "/execute",
                        "t",
                        json!({"tree_xml": tree}),
                        Duration::from_secs(1),
                    )
                    .await
                    .unwrap();
                goal.result.await.unwrap().unwrap();
                robot.transform("map", "base").unwrap().planar()
            }
        };
        let near = |a: f64, b: f64| (a - b).abs() < 1e-6;
        let (x, y, _) = run(
            r#"<root><Timeout msec="9"><SubTree ID="Walk" m="2" way="forward"/></Timeout></root>"#,
        )
        .await;
        assert!(near(x, 2.0) && near(y, 0.0));
        let (_, _, yaw) = run(r#"<root><Turn deg="90"/></root>"#).await;
        assert!(near(yaw, std::f64::consts::FRAC_PI_2));
        let (x, y, _) = run(r#"<root><Walk m="1" way="backward"/></root>"#).await;
        assert!(
            near(x, 2.0) && near(y, -1.0),
            "along its new heading: {x} {y}"
        );
        let (x, y, yaw) = run(r#"<root><Go place="kitchen"/></root>"#).await;
        assert!(near(x, 3.0) && near(y, 4.0) && near(yaw, std::f64::consts::FRAC_PI_2));
        let (x, _, yaw) = run(r#"<root><SubTree ID="Pose" station="1.5;-2;0.5"/></root>"#).await;
        assert!(near(x, 1.5) && near(yaw, 0.5));
        let (x, y, _) = run(r#"<root><Go place="kitchen"/></root>"#).await;
        assert!(near(x, 3.0) && near(y, 4.0));
        let path = robot
            .send_goal(
                "/plan",
                "t",
                json!({"goal": {"pose": {"position": {"x": 3.0, "y": 8.0}}}}),
                Duration::from_secs(1),
            )
            .await
            .unwrap()
            .result
            .await
            .unwrap()
            .unwrap()
            .result;
        assert_eq!(path["path"]["poses"][0]["pose"]["position"]["y"], 4.0);
        assert_eq!(path["path"]["poses"][1]["pose"]["position"]["y"], 8.0);
    }

    #[tokio::test]
    async fn a_reply_goes_by_the_longest_word_the_request_holds() {
        let robot = robot(&json!({"replies": {"/find": {
            "field": "query",
            "by": {"mug": {"n": 1}, "red mug": {"n": 2}},
            "otherwise": {"n": 0},
        }}}));
        let ask = |query: &'static str| {
            let robot = &robot;
            async move {
                robot
                    .call(
                        "/find",
                        "t",
                        json!({"query": query}),
                        Duration::from_secs(1),
                    )
                    .await
                    .unwrap()["n"]
                    .as_u64()
            }
        };
        assert_eq!(ask("the small Red Mug").await, Some(2));
        assert_eq!(ask("a mug").await, Some(1));
        assert_eq!(ask("sofa").await, Some(0));
    }

    #[tokio::test]
    async fn a_camera_gives_its_file() {
        let dir = std::env::temp_dir().join(format!("scripted-frame-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("a.png");
        image::RgbImage::new(4, 2).save(&file).unwrap();
        let robot = robot(&json!({"frames": {"/cam": file}}));
        let mut frames = robot.frames("/cam").unwrap();
        let frame = frames.borrow_and_update().clone().unwrap();
        assert_eq!((frame.width, frame.height, frame.step), (4, 2, 12));
        tokio::time::timeout(FRAME_PERIOD * 4, frames.changed())
            .await
            .unwrap()
            .unwrap();
        assert!(frames.borrow().as_ref().unwrap().stamp_s > frame.stamp_s);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_later_file_replaces_an_earlier_one_key_by_key() {
        let base: ScriptedRobot = serde_json::from_value(json!({
            "services": {"/a": {"x": 1}, "/b": {"x": 1}},
        }))
        .unwrap();
        let over: ScriptedRobot =
            serde_json::from_value(json!({"services": {"/b": {"x": 2}}})).unwrap();
        let both = base.with(over);
        assert_eq!(both.services["/a"], json!({"x": 1}));
        assert_eq!(both.services["/b"], json!({"x": 2}));
    }
}
