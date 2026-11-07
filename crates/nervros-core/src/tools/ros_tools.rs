//! Generic ROS tools: look at any part of the graph the way `ros2` does, natively, and where the
//! profile allows it call services, send action goals, set parameters and publish.
//!
//! Reads are free. Every act goes through the guard like any other: armed, approved when
//! supervised, with the exact target and payload in the approval, after the hard deny lists. Every
//! name an act takes from the model must be absolute (a read takes `odom` as `/odom`), and every
//! payload is checked against its interface before it reaches r2r, which would panic on a wrong
//! array length.

use std::sync::Arc;
use std::time::Duration;

use nervros_ros::{
    Durability, Endpoint, GraphDetail, Liveliness, Reliability, RobotPort, RosError,
};
use serde_json::{Map, Value, json};

use crate::guard::{Guard, canonical_ros_name, glob_match};
use crate::tools::{Risk, SchemaPart, SchemaSource, Tool, ToolOutcome, ToolSpec};

mod action;
mod config;
mod graph;
mod interface;
pub(crate) mod logs;
pub(crate) mod params;
mod publish;
mod sample;
mod service;
#[cfg(test)]
mod tests;
mod tf;

pub use config::RosToolsConfig;
pub use params::param_value;

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
// An echo asked for in full stops here, so that one big array does not fill the conversation.
const FULL_ITEMS: usize = 2000;
const FULL_CHARS: usize = 8000;
/// Message types too large to read message by message: echoed, watched or plotted.
pub const BULK_TYPES: [&str; 5] = [
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/PointCloud2",
    "nav_msgs/msg/OccupancyGrid",
    "octomap_msgs/msg/Octomap",
];

/// Whether `name` matches one of the `patterns`, whatever the case.
#[must_use]
pub fn masked(patterns: &[String], name: &str) -> bool {
    let name = name.to_lowercase();
    patterns
        .iter()
        .any(|p| glob_match(&p.to_lowercase(), &name))
}

struct Ctx {
    robot: Arc<dyn RobotPort>,
    schemas: Arc<dyn SchemaSource>,
    guard: Arc<Guard>,
    config: RosToolsConfig,
    /// Where a goal sent here is kept while it runs, so a stop and the watchdog see it.
    goals: Option<Arc<crate::integrations::Goals>>,
    /// The graph as last read, and when: an act's check runs at its approval and again at its
    /// call, a moment apart.
    seen: std::sync::Mutex<Option<(std::time::Instant, GraphDetail)>>,
}

/// How long a graph read stands for the next check.
const GRAPH_FRESH: Duration = Duration::from_secs(2);

impl Ctx {
    /// Whether a parameter's value is hidden, whatever the case of its name: `apiKey` too.
    fn masked(&self, name: &str) -> bool {
        masked(&self.config.param_read_deny, name)
    }

    fn hidden(&self, name: &str) -> bool {
        self.config.hidden.iter().any(|p| glob_match(p, name))
    }

    async fn graph(&self) -> Result<GraphDetail, ToolOutcome> {
        if let Some((at, graph)) = crate::lock(&self.seen).as_ref()
            && at.elapsed() < GRAPH_FRESH
        {
            return Ok(graph.clone());
        }
        let graph = self.robot.graph_detail().await.map_err(|e| failed(&e))?;
        *crate::lock(&self.seen) = Some((std::time::Instant::now(), graph.clone()));
        Ok(graph)
    }

    /// The steps every act shares once the profile lets it: the type it takes, the deny rules on
    /// its name and that type, and its payload under `key` checked against the type's `part`
    /// (none for an act that sends no payload, as a cancel). A refusal is boxed: it is the rare
    /// path, and the outcome is large.
    fn checked(
        &self,
        (name, graph): (String, Result<String, ToolOutcome>),
        args: &Value,
        (key, part): (&str, Option<SchemaPart>),
    ) -> Result<Prepared, Box<ToolOutcome>> {
        let ty = Self::resolved(&name, args["type"].as_str(), graph)?;
        if let Some(out) = self.deny_act(&name, &ty) {
            return Err(Box::new(out));
        }
        let payload = payload(args, key);
        if let Some(part) = part {
            self.schemas.validate(&ty, part, &payload).map_err(|why| {
                Box::new(ToolOutcome::failed(format!(
                    "the {key} does not fit {ty}: {why}"
                )))
            })?;
        }
        Ok(Prepared { name, ty, payload })
    }

    /// The type to use for `name`: the model's only when the graph has none to compare it with.
    /// One the graph contradicts is refused, as the deny rules read the type.
    fn resolved(
        name: &str,
        given: Option<&str>,
        graph: Result<String, ToolOutcome>,
    ) -> Result<String, Box<ToolOutcome>> {
        match (given.filter(|t| !t.is_empty()), graph) {
            (Some(given), Ok(known)) if given != known => Err(Box::new(ToolOutcome::refused(
                format!("`{name}` is {known} in the graph, not {given}: give its own type or none"),
            ))),
            (Some(given), _) => Ok(given.to_owned()),
            (None, graph) => graph.map_err(Box::new),
        }
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

    /// A goal or a message that would send the robot into a keep-out zone, refused with the
    /// zone's name: Nav2's pose goals and paths, and a pose published for Nav2 to go to. A move
    /// relative to where the robot stands is checked only as a mission step, so while there are
    /// keep-out zones Nav2's own is refused.
    fn zone_refusal(&self, ty: &str, payload: &Value) -> Option<ToolOutcome> {
        use crate::integrations::nav2::goals::{Destinations, destinations};
        let zones = self.guard.zones()?;
        let poses = match destinations(ty, payload) {
            Destinations::Poses(poses) => poses,
            // A mission step is checked for entering a keep-out zone, which a move with no pose
            // on the map cannot be here; a slow zone refuses no move, there or here.
            Destinations::Unchecked if !zones.keep_out().is_empty() => {
                return Some(ToolOutcome::refused(
                    "with keep-out zones set, a move with no pose on the map is checked against \
                     them only as a mission step: walk with run_mission",
                ));
            }
            Destinations::Unchecked | Destinations::Elsewhere => return None,
        };
        // A path's poses may leave their frame to the path's own header.
        let path_frame = payload["path"]["header"]["frame_id"]
            .as_str()
            .unwrap_or_default();
        let mut points = Vec::with_capacity(poses.len());
        for stamped in poses {
            let frame = match stamped["header"]["frame_id"].as_str().unwrap_or_default() {
                "" => path_frame,
                own => own,
            };
            let p = &stamped["pose"]["position"];
            // A field left out goes as 0 on the wire, so it is checked as 0.
            let at = [
                p["x"].as_f64().unwrap_or(0.0),
                p["y"].as_f64().unwrap_or(0.0),
                p["z"].as_f64().unwrap_or(0.0),
            ];
            let at = if frame.is_empty() || frame == zones.frame() {
                at
            } else {
                match self.robot.transform(zones.frame(), frame) {
                    Ok(t) => t.apply(at),
                    Err(_) => {
                        return Some(ToolOutcome::refused(format!(
                            "a pose in `{frame}` cannot be checked against the keep-out zones: \
                             give it in `{}`",
                            zones.frame()
                        )));
                    }
                }
            };
            if let Some(zone) = zones.at(at[0], at[1]) {
                return Some(ToolOutcome::refused(format!(
                    "({:.2}, {:.2}) is inside the keep-out zone `{}`: the robot may not go \
                     there; geofence lists the zones, and only the operator removes one",
                    at[0], at[1], zone.name
                )));
            }
            points.push((at[0], at[1]));
        }
        // A followed path is driven pose to pose as given: no planner goes round a zone between.
        if ty == "nav2_msgs/action/FollowPath"
            && let Some((zone, _)) = points.windows(2).find_map(|w| zones.crossed(w[0], w[1]))
        {
            return Some(ToolOutcome::refused(format!(
                "the path crosses the keep-out zone `{}`: the robot may not go through it",
                zone.name
            )));
        }
        None
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

/// A name to read, where `controller_server` means `/controller_server`: only an act must be
/// given absolute names, since its deny lists are written for them.
fn read_name_arg(args: &Value, key: &str) -> Result<String, String> {
    read_name(args[key].as_str().unwrap_or_default())
}

/// The arguments with the name under `key` as a read resolves it, for the profile's rules: a rule
/// on `/camera/*` holds for `camera/image` too. `None` when they already say it.
fn resolved_view(args: &Value, key: &str) -> Option<Value> {
    let name = read_name_arg(args, key).ok()?;
    (args[key].as_str() != Some(name.as_str())).then(|| {
        let mut view = args.clone();
        view[key] = json!(name);
        view
    })
}

fn read_name(raw: &str) -> Result<String, String> {
    let raw = raw.trim();
    if raw.is_empty() || raw.starts_with('/') {
        canonical_ros_name(raw)
    } else {
        canonical_ros_name(&format!("/{raw}"))
    }
}

/// `args` with the type an act resolved: the approved call, and the rules that read it, see the
/// type the call is made with.
fn with_type(args: &Value, ty: &str) -> Value {
    let mut args = args.clone();
    args["type"] = json!(ty);
    args
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

/// Fields that carry free text a node may have taken from the world, such as a sign read aloud
/// or a speech result: fenced as the world's wherever a message holds them.
const FREE_TEXT: [&str; 8] = [
    "data",
    "msg",
    "text",
    "message",
    "description",
    "label",
    "caption",
    "transcript",
];

/// A message cut down to what a model can read: long arrays and strings shortened with their size,
/// and free text fenced as the world's.
fn summarize(value: &Value) -> Value {
    let mut out = shortened(value, ARRAY_ITEMS, TEXT_CHARS);
    let keys: Vec<String> = FREE_TEXT.iter().map(|k| (*k).to_owned()).collect();
    crate::tools::mark_world_text(&mut out, &keys);
    out
}

/// A message with its arrays cut at `items` and its strings at `chars`, saying how long they were.
fn shortened(value: &Value, items: usize, chars: usize) -> Value {
    match value {
        Value::Array(all) if all.len() > items => {
            let mut head: Vec<Value> = all
                .iter()
                .take(items)
                .map(|v| shortened(v, items, chars))
                .collect();
            head.push(json!(format!(
                "... {} more, {} in all",
                all.len() - items,
                all.len()
            )));
            Value::Array(head)
        }
        Value::Array(all) => Value::Array(all.iter().map(|v| shortened(v, items, chars)).collect()),
        Value::Object(fields) => Value::Object(
            fields
                .iter()
                .map(|(k, v)| (k.clone(), shortened(v, items, chars)))
                .collect(),
        ),
        Value::String(s) if s.chars().count() > chars => {
            let head: String = s.chars().take(chars).collect();
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
    let mut out = json!({"node": e.node, "type": e.topic_type, "reliability": e.qos.reliability,
                         "durability": e.qos.durability, "history": e.qos.history,
                         "depth": e.qos.depth});
    if e.qos.liveliness != nervros_ros::Liveliness::SystemDefault {
        out["liveliness"] = json!(e.qos.liveliness);
    }
    if !e.gid.is_empty() {
        out["gid"] = json!(e.gid);
    }
    let limits = [
        ("deadline_s", e.qos.deadline),
        ("lifespan_s", e.qos.lifespan),
        ("lease_s", e.qos.lease),
    ];
    for (key, limit) in limits {
        if let Some(limit) = limit {
            out[key] = json!(limit.as_secs_f64());
        }
    }
    out
}

/// Whether a subscriber with `sub`'s `QoS` receives from a publisher with `publisher`'s: reliable
/// asks more than best effort gives, transient local more than volatile, a deadline more than a
/// slower one or none, and manual liveliness more than automatic.
fn qos_matches(publisher: &Endpoint, sub: &Endpoint) -> bool {
    let reliable_ok = !(sub.qos.reliability == Reliability::Reliable
        && publisher.qos.reliability == Reliability::BestEffort);
    let durable_ok = !(sub.qos.durability == Durability::TransientLocal
        && publisher.qos.durability == Durability::Volatile);
    let deadline_ok = match (sub.qos.deadline, publisher.qos.deadline) {
        (Some(asked), Some(given)) => asked >= given,
        (Some(_), None) => false,
        (None, _) => true,
    };
    let lively_ok = !(sub.qos.liveliness == Liveliness::ManualByTopic
        && publisher.qos.liveliness == Liveliness::Automatic);
    reliable_ok && durable_ok && deadline_ok && lively_ok
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
    (schemas, guard): (&Arc<dyn SchemaSource>, &Arc<Guard>),
    goals: Option<Arc<crate::integrations::Goals>>,
) -> Vec<Arc<dyn Tool>> {
    let ctx = Arc::new(Ctx {
        robot: Arc::clone(robot),
        schemas: Arc::clone(schemas),
        guard: Arc::clone(guard),
        config: config.clone(),
        goals,
        seen: std::sync::Mutex::new(None),
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
