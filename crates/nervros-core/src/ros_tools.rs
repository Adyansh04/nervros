//! Generic ROS tools: look at any part of the graph the way `ros2` does, natively, and where the
//! profile allows it call services, send action goals, set parameters and publish.
//!
//! Reads are free. Every act goes through the guard like any other: armed, approved when
//! supervised, with the exact target and payload in the approval, after the hard deny lists. Every
//! name from the model must be absolute, and every payload is checked against its interface
//! before it reaches r2r, which would panic on a wrong array length.

use std::sync::Arc;
use std::time::Duration;

use nervros_ros::{Endpoint, GraphDetail, RobotPort, RosError};
use serde_json::{Map, Value, json};

use crate::guard::{Guard, canonical_ros_name, glob_match};
use crate::tools::{Risk, SchemaSource, Tool, ToolOutcome, ToolSpec};

mod action;
mod config;
mod graph;
mod interface;
mod logs;
mod params;
mod publish;
mod sample;
mod service;
#[cfg(test)]
mod tests;
mod tf;

pub use config::RosToolsConfig;

// Limits that keep a sample, an echo or a burst of messages small and short.
const MAX_SAMPLE: Duration = Duration::from_secs(10);
const MAX_ECHO: usize = 10;
const MAX_PUBLISH: usize = 10;
const MAX_PUBLISH_HZ: f64 = 10.0;
const MAX_CALL: Duration = Duration::from_secs(30);
const MAX_GOAL_WAIT: Duration = Duration::from_mins(2);
const LIST_LIMIT: usize = 60;
// A node's parameters shown at once when none are named.
const PARAMS_MAX: usize = 80;
// An echo shows this much of each array and string.
const ARRAY_ITEMS: usize = 8;
const TEXT_CHARS: usize = 120;
/// Message types too large to read message by message: echoed, watched or plotted.
pub(crate) const BULK_TYPES: [&str; 5] = [
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/PointCloud2",
    "nav_msgs/msg/OccupancyGrid",
    "octomap_msgs/msg/Octomap",
];

struct Ctx {
    robot: Arc<dyn RobotPort>,
    schemas: Arc<dyn SchemaSource>,
    guard: Arc<Guard>,
    config: RosToolsConfig,
}

impl Ctx {
    /// Whether a parameter's value is hidden, whatever the case of its name: `apiKey` too.
    fn masked(&self, name: &str) -> bool {
        let name = name.to_lowercase();
        self.config
            .param_read_deny
            .iter()
            .any(|p| glob_match(&p.to_lowercase(), &name))
    }

    fn hidden(&self, name: &str) -> bool {
        self.config.hidden.iter().any(|p| glob_match(p, name))
    }

    async fn graph(&self) -> Result<GraphDetail, ToolOutcome> {
        self.robot.graph_detail().await.map_err(|e| failed(&e))
    }

    /// The type a topic carries now, or why there is none.
    async fn topic_type(&self, topic: &str) -> Result<String, ToolOutcome> {
        let graph = self.graph().await?;
        find_type(&graph.topics, topic).ok_or_else(|| {
            ToolOutcome::failed(missing(
                "topic",
                topic,
                graph.topics.iter().map(|(n, _)| n.as_str()),
            ))
        })
    }

    /// The type a service has now, or why there is none.
    async fn service_type(&self, service: &str) -> Result<String, ToolOutcome> {
        let graph = self.graph().await?;
        find_type(&graph.services, service).ok_or_else(|| {
            ToolOutcome::failed(missing(
                "service",
                service,
                graph.services.iter().map(|(n, _)| n.as_str()),
            ))
        })
    }

    /// The type of an action, from the type of its feedback topic.
    async fn action_type(&self, action: &str) -> Result<String, ToolOutcome> {
        let graph = self.graph().await?;
        let all = actions(&graph);
        all.iter()
            .find(|(name, _)| name == action)
            .map(|(_, ty)| ty.clone())
            .ok_or_else(|| {
                ToolOutcome::failed(missing(
                    "action",
                    action,
                    all.iter().map(|(n, _)| n.as_str()),
                ))
            })
    }

    /// Why a model-given name or type may not be acted on, if it may not.
    fn deny_act(&self, name: &str, ty: &str) -> Option<ToolOutcome> {
        if name.contains("/_action/") {
            return Some(ToolOutcome::refused(
                "that is part of an action; use action_goal on the action itself",
            ));
        }
        if self.guard.hard_denied(name) {
            return Some(ToolOutcome::refused(format!(
                "`{name}` is on the robot's hard deny list; nothing may write to it"
            )));
        }
        if self.guard.hard_denied_type(ty) {
            return Some(ToolOutcome::refused(format!(
                "`{ty}` commands the robot directly (velocities, joints, controllers or \
                 lifecycles), which no generic tool may send; the robot's own skills do that"
            )));
        }
        None
    }
}

fn failed(e: &RosError) -> ToolOutcome {
    ToolOutcome::failed(e.to_string())
}

/// Why `name` is not one of `kind` in the graph, naming those like it: a small model guesses
/// `/odom` where the robot has `/Odometry_loc`, and needs the real name more than the advice to
/// list everything.
pub(crate) fn missing<'a>(kind: &str, name: &str, known: impl Iterator<Item = &'a str>) -> String {
    let word = name.rsplit('/').next().unwrap_or(name).to_lowercase();
    let like: Vec<&str> = if word.len() < 3 {
        Vec::new()
    } else {
        known
            .filter(|n| n.to_lowercase().contains(&word))
            .take(5)
            .collect()
    };
    if like.is_empty() {
        format!("no {kind} `{name}` in the graph; ros_graph lists them")
    } else {
        format!(
            "no {kind} `{name}` in the graph; ones like it: {}",
            like.join(", ")
        )
    }
}

fn find_type(list: &[(String, Vec<String>)], name: &str) -> Option<String> {
    list.iter()
        .find(|(n, _)| n == name)
        .and_then(|(_, types)| types.first().cloned())
}

/// Actions and their types, derived from `<action>/_action/feedback` topics.
fn actions(graph: &GraphDetail) -> Vec<(String, String)> {
    graph
        .topics
        .iter()
        .filter_map(|(topic, types)| {
            let name = topic.strip_suffix("/_action/feedback")?;
            let ty = types.first()?.strip_suffix("_FeedbackMessage")?;
            Some((name.to_owned(), ty.to_owned()))
        })
        .collect()
}

/// The ROS name under `key`, or the refusal to answer with.
fn name_arg(args: &Value, key: &str) -> Result<String, String> {
    canonical_ros_name(args[key].as_str().unwrap_or_default())
}

fn seconds(args: &Value, key: &str, default: f64, max: Duration) -> Duration {
    let s = args[key].as_f64().unwrap_or(default);
    Duration::from_secs_f64(s.clamp(0.1, max.as_secs_f64()))
}

fn count(args: &Value, key: &str, default: usize, max: usize) -> usize {
    args[key]
        .as_u64()
        .and_then(|n| usize::try_from(n).ok())
        .unwrap_or(default)
        .clamp(1, max)
}

fn spec(name: &str, description: &str, parameters: Value, risk: Risk) -> ToolSpec {
    let mut spec = ToolSpec::new(name, description, parameters, risk);
    spec.timeout = MAX_GOAL_WAIT + Duration::from_secs(10);
    spec
}

/// A message cut down to what a model can read: long arrays and strings shortened with their size.
fn summarize(value: &Value) -> Value {
    match value {
        Value::Array(items) if items.len() > ARRAY_ITEMS => {
            let mut head: Vec<Value> = items.iter().take(ARRAY_ITEMS).map(summarize).collect();
            head.push(json!(format!(
                "... {} more, {} in all",
                items.len() - ARRAY_ITEMS,
                items.len()
            )));
            Value::Array(head)
        }
        Value::Array(items) => Value::Array(items.iter().map(summarize).collect()),
        Value::Object(fields) => Value::Object(
            fields
                .iter()
                .map(|(k, v)| (k.clone(), summarize(v)))
                .collect(),
        ),
        Value::String(s) if s.chars().count() > TEXT_CHARS => {
            let head: String = s.chars().take(TEXT_CHARS).collect();
            json!(format!("{head}... ({} characters)", s.chars().count()))
        }
        other => other.clone(),
    }
}

/// Keeps only the listed top-level or dotted fields of a message.
fn pick(value: &Value, fields: &[String]) -> Value {
    let mut out = Map::new();
    for f in fields {
        // The same paths `watch` and `plot` take, list indices included.
        let found = crate::watch::field(value, f)
            .cloned()
            .unwrap_or(Value::Null);
        out.insert(f.clone(), found);
    }
    Value::Object(out)
}

fn endpoint_json(e: &Endpoint) -> Value {
    json!({"node": e.node, "type": e.topic_type, "reliability": e.qos.reliability,
           "durability": e.qos.durability, "depth": e.qos.depth})
}

/// Whether a subscriber with `sub`'s `QoS` receives from a publisher with `publisher`'s: reliable
/// asks more than best effort gives, and transient local more than volatile.
fn qos_matches(publisher: &Endpoint, sub: &Endpoint) -> bool {
    let reliable_ok =
        !(sub.qos.reliability == "reliable" && publisher.qos.reliability == "best_effort");
    let durable_ok =
        !(sub.qos.durability == "transient_local" && publisher.qos.durability == "volatile");
    reliable_ok && durable_ok
}

/// A generic call, resolved and checked: what the operator approves is what goes out.
struct Prepared {
    name: String,
    ty: String,
    payload: Value,
}

fn payload(args: &Value, key: &str) -> Value {
    match &args[key] {
        Value::Null => json!({}),
        other => other.clone(),
    }
}

/// The ROS tools a profile's `[ros_tools]` table enables: the six read tools, and each act tool
/// whose allow list is not empty.
#[must_use]
pub fn tools(
    config: &RosToolsConfig,
    robot: &Arc<dyn RobotPort>,
    schemas: &Arc<dyn SchemaSource>,
    guard: &Arc<Guard>,
) -> Vec<Arc<dyn Tool>> {
    let ctx = Arc::new(Ctx {
        robot: Arc::clone(robot),
        schemas: Arc::clone(schemas),
        guard: Arc::clone(guard),
        config: config.clone(),
    });
    let mut out = vec![
        graph::tool(Arc::clone(&ctx)),
        sample::tool(Arc::clone(&ctx)),
        interface::tool(Arc::clone(&ctx)),
        tf::tool(Arc::clone(&ctx)),
        params::tool(Arc::clone(&ctx)),
        logs::tool(Arc::clone(&ctx)),
    ];
    if !config.service_call.is_empty() {
        out.push(service::tool(Arc::clone(&ctx)));
    }
    if !config.action_send.is_empty() {
        out.push(action::tool(Arc::clone(&ctx)));
    }
    if !config.param_set.is_empty() {
        out.push(params::set_tool(Arc::clone(&ctx)));
    }
    if !config.publish.is_empty() {
        out.push(publish::tool(ctx));
    }
    out
}

/// A JSON schema object with these properties, `required` naming the ones a call must give.
fn object(properties: Value, required: &[&str]) -> Value {
    let mut schema = json!({"type": "object", "required": required});
    schema["properties"] = properties;
    schema
}
