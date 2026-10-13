//! A robot from a file: what the scenario tests talk to, and what `nervros-cli eval --robot` runs a
//! suite against, so models can be compared in seconds, without a simulator.
//!
//! Services answer with a fixed response, actions run to a fixed end and topics hold one message.
//! The graph, what each node publishes and subscribes to, the nodes each component container
//! hosts, parameters, topic rates and frames make it look to the ROS tools like a live graph; a
//! camera gives no image yet, and a service the graph lists without a response is not reachable.
//! Callers parse the file (TOML in this repository) into [`ScriptedRobot`] and
//! [`ScriptedRobot::build`] it.

use std::collections::BTreeMap;

use serde::Deserialize;
use serde_json::{Value, json};

use crate::fake::{FakeRobot, ScriptedRun};
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
    /// An action's status is not one of succeeded, aborted or canceled, or a rate is out of reach.
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
        for (name, run) in self.actions {
            let status = match run.status.as_str() {
                "succeeded" => GoalStatus::Succeeded,
                "aborted" => GoalStatus::Aborted,
                "canceled" => GoalStatus::Canceled,
                other => return Err(format!("{name}: no status {other}")),
            };
            robot = robot.with_action(&name, move |_| ScriptedRun {
                feedback: run.feedback.clone(),
                result: Ok(GoalResult {
                    status,
                    result: run.result.clone(),
                }),
                ..ScriptedRun::default()
            });
        }
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
            let half = f.yaw / 2.0;
            let pose = Transform {
                translation: [f.x, f.y, f.z],
                rotation: [0.0, 0.0, half.sin(), half.cos()],
            };
            robot = robot.with_transform(&f.parent, &f.child, pose);
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
