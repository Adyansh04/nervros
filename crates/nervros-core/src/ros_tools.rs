//! Generic ROS tools: look at any part of the graph the way `ros2` does, natively, and where the
//! profile allows it call services, send action goals, set parameters and publish.
//!
//! Reads are free. Every act goes through the guard like any other: armed, approved when
//! supervised, with the exact target and payload in the approval, after the hard deny lists. Every
//! name from the model must be absolute, and every payload is checked against its interface
//! before it reaches r2r, which would panic on a wrong array length.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::{Endpoint, GraphDetail, RobotPort, RosError};
use serde::Deserialize;
use serde_json::{Map, Value, json};

use crate::guard::{Guard, canonical_ros_name, glob_match};
use crate::tools::{
    Assessment, Resource, Risk, SchemaPart, SchemaSource, Tool, ToolOutcome, ToolSpec,
};

/// The `[ros_tools]` table.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RosToolsConfig {
    /// Names left out of listings unless asked for; a convenience, not a boundary.
    #[serde(default = "d_hidden")]
    pub hidden: Vec<String>,
    /// Topics never sampled or echoed.
    #[serde(default)]
    pub read_deny: Vec<String>,
    /// Parameter names whose values are masked.
    #[serde(default = "d_param_read_deny")]
    pub param_read_deny: Vec<String>,
    /// Services that only read: `service_call` runs them without arming or approval.
    #[serde(default = "d_service_observe")]
    pub service_observe: Vec<String>,
    /// Services `service_call` may call; empty leaves the tool out.
    #[serde(default)]
    pub service_call: Vec<String>,
    /// Actions `action_goal` may send goals to; empty leaves the tool out.
    #[serde(default)]
    pub action_send: Vec<String>,
    /// Topics `topic_publish` may publish on; empty leaves the tool out.
    #[serde(default)]
    pub publish: Vec<String>,
    /// `node:parameter` globs `param_set` may change; empty leaves the tool out.
    #[serde(default)]
    pub param_set: Vec<String>,
}

fn d_hidden() -> Vec<String> {
    [
        "*/_action/*",
        "/rosout",
        "/parameter_events",
        "*/describe_parameters",
        "*/get_parameters",
        "*/get_parameter_types",
        "*/list_parameters",
        "*/set_parameters",
        "*/set_parameters_atomically",
        "*/get_type_description",
    ]
    .map(str::to_owned)
    .to_vec()
}

fn d_param_read_deny() -> Vec<String> {
    ["*key*", "*token*", "*secret*", "*password*"]
        .map(str::to_owned)
        .to_vec()
}

fn d_service_observe() -> Vec<String> {
    ["*/get_*", "*/list_*", "*/describe_*"]
        .map(str::to_owned)
        .to_vec()
}

impl Default for RosToolsConfig {
    fn default() -> Self {
        Self {
            hidden: d_hidden(),
            read_deny: Vec::new(),
            param_read_deny: d_param_read_deny(),
            service_observe: d_service_observe(),
            service_call: Vec::new(),
            action_send: Vec::new(),
            publish: Vec::new(),
            param_set: Vec::new(),
        }
    }
}

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
// Messages whose JSON is megabytes: counted, never echoed.
const BULK_TYPES: [&str; 5] = [
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/PointCloud2",
    "nav_msgs/msg/OccupancyGrid",
    "octomap_msgs/msg/Octomap",
];
const ALL: [Resource; 3] = [Resource::Base, Resource::LeftArm, Resource::RightArm];

struct Ctx {
    robot: Arc<dyn RobotPort>,
    schemas: Arc<dyn SchemaSource>,
    guard: Arc<Guard>,
    config: RosToolsConfig,
}

impl Ctx {
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
            ToolOutcome::failed(format!(
                "no topic `{topic}` in the graph; ros_graph lists the topics"
            ))
        })
    }

    /// The type a service has now, or why there is none.
    async fn service_type(&self, service: &str) -> Result<String, ToolOutcome> {
        let graph = self.graph().await?;
        find_type(&graph.services, service).ok_or_else(|| {
            ToolOutcome::failed(format!(
                "no service `{service}` in the graph; ros_graph lists the services"
            ))
        })
    }

    /// The type of an action, from the type of its feedback topic.
    async fn action_type(&self, action: &str) -> Result<String, ToolOutcome> {
        let graph = self.graph().await?;
        actions(&graph)
            .into_iter()
            .find(|(name, _)| name == action)
            .map(|(_, ty)| ty)
            .ok_or_else(|| {
                ToolOutcome::failed(format!(
                    "no action `{action}` in the graph; ros_graph lists the actions"
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

#[expect(
    clippy::result_large_err,
    reason = "the error is the tool's answer, returned once per call"
)]
fn name_arg(args: &Value, key: &str) -> Result<String, ToolOutcome> {
    let raw = args[key].as_str().unwrap_or_default();
    canonical_ros_name(raw).map_err(ToolOutcome::refused)
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
        let mut v = value;
        for part in f.split('.') {
            v = &v[part];
        }
        out.insert(f.clone(), v.clone());
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

// --- ros_graph -------------------------------------------------------------------------------

struct RosGraph {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

impl RosGraph {
    fn list(&self, graph: &GraphDetail, args: &Value) -> ToolOutcome {
        let kind = args["kind"].as_str().unwrap_or("topics");
        let filter = args["filter"].as_str().unwrap_or_default();
        let all = args["all"].as_bool().unwrap_or(false);
        let items: Vec<(String, String)> = match kind {
            "topics" => graph
                .topics
                .iter()
                .map(|(n, t)| (n.clone(), t.join(", ")))
                .collect(),
            "services" => graph
                .services
                .iter()
                .map(|(n, t)| (n.clone(), t.join(", ")))
                .collect(),
            "actions" => actions(graph),
            "nodes" => graph
                .nodes
                .iter()
                .map(|n| (n.clone(), String::new()))
                .collect(),
            other => {
                return ToolOutcome::failed(format!(
                    "kind `{other}` is not one of topics, services, actions, nodes"
                ));
            }
        };
        let matching: Vec<&(String, String)> = items
            .iter()
            .filter(|(n, _)| all || !self.ctx.hidden(n))
            .filter(|(n, _)| {
                filter.is_empty()
                    || if filter.contains('*') {
                        glob_match(filter, n)
                    } else {
                        n.contains(filter)
                    }
            })
            .collect();
        let shown: Vec<Value> = matching
            .iter()
            .take(LIST_LIMIT)
            .map(|(n, t)| {
                if t.is_empty() {
                    json!(n)
                } else {
                    json!({"name": n, "type": t})
                }
            })
            .collect();
        ToolOutcome::ok(json!({
            "kind": kind,
            "total": matching.len(),
            "shown": shown.len(),
            "items": shown,
        }))
    }

    async fn topic(&self, graph: &GraphDetail, name: &str) -> ToolOutcome {
        let types = graph
            .topics
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, t)| t.clone())
            .unwrap_or_default();
        let ends = match self.ctx.robot.endpoints(name).await {
            Ok(e) => e,
            Err(e) => return failed(&e),
        };
        // The classic silent failure: a subscriber that asks for more than a publisher gives.
        let mismatched: Vec<Value> = ends
            .subscribers
            .iter()
            .flat_map(|s| {
                ends.publishers
                    .iter()
                    .filter(move |p| !qos_matches(p, s))
                    .map(move |p| json!({"publisher": p.node, "subscriber": s.node}))
            })
            .collect();
        let mut data = json!({
            "topic": name,
            "types": types,
            "publishers": ends.publishers.iter().map(endpoint_json).collect::<Vec<_>>(),
            "subscribers": ends.subscribers.iter().map(endpoint_json).collect::<Vec<_>>(),
        });
        if !mismatched.is_empty() {
            data["qos_mismatch"] = json!(mismatched);
            data["note"] = json!(
                "these subscribers ask for more (reliable or transient local) than the publishers \
                 give, so they receive nothing from them"
            );
        }
        ToolOutcome::ok(data)
    }

    async fn node(&self, name: &str) -> ToolOutcome {
        match self.ctx.robot.node_entities(name).await {
            Ok(n) => {
                let list = |items: &[(String, Vec<String>)]| {
                    items
                        .iter()
                        .filter(|(n, _)| !self.ctx.hidden(n))
                        .map(|(n, t)| json!({"name": n, "type": t.join(", ")}))
                        .collect::<Vec<_>>()
                };
                ToolOutcome::ok(json!({
                    "node": name,
                    "publishers": list(&n.publishers),
                    "subscribers": list(&n.subscribers),
                    "services": list(&n.services),
                    "clients": list(&n.clients),
                }))
            }
            Err(e) => failed(&e),
        }
    }

    async fn service(&self, name: &str, ty: &str) -> ToolOutcome {
        let available = self
            .ctx
            .robot
            .service_available(name, ty, Duration::from_secs(1))
            .await;
        ToolOutcome::ok(json!({
            "service": name,
            "type": ty,
            "reachable": available,
            "definition": self.ctx.schemas.show(ty),
        }))
    }
}

#[async_trait]
impl Tool for RosGraph {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let graph = match self.ctx.graph().await {
            Ok(g) => g,
            Err(out) => return out,
        };
        let Some(raw) = args["name"].as_str().filter(|n| !n.is_empty()) else {
            return self.list(&graph, &args);
        };
        let name = match canonical_ros_name(raw) {
            Ok(n) => n,
            Err(e) => return ToolOutcome::refused(e),
        };
        if graph.topics.iter().any(|(n, _)| *n == name) {
            return self.topic(&graph, &name).await;
        }
        if graph.nodes.contains(&name) {
            return self.node(&name).await;
        }
        if let Some(ty) = find_type(&graph.services, &name) {
            return self.service(&name, &ty).await;
        }
        if let Some((_, ty)) = actions(&graph).into_iter().find(|(n, _)| *n == name) {
            return ToolOutcome::ok(json!({
                "action": name,
                "type": ty,
                "definition": self.ctx.schemas.show(&ty),
            }));
        }
        ToolOutcome::failed(format!(
            "`{name}` is not a topic, node, service or action in the graph"
        ))
    }
}

// --- topic_sample ----------------------------------------------------------------------------

struct TopicSample {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

fn rate_stats(arrivals: &[(Duration, usize)], window: Duration) -> Value {
    let n = arrivals.len();
    let bytes: usize = arrivals.iter().map(|(_, b)| b).sum();
    #[expect(
        clippy::cast_precision_loss,
        reason = "counts and sizes far below 2^52"
    )]
    let (n_f, bytes_f) = (n as f64, bytes as f64);
    let mut out = json!({
        "messages": n,
        "window_s": window.as_secs_f64(),
        "bytes_per_s": (bytes_f / window.as_secs_f64()).round(),
        "mean_size_bytes": if n > 0 { (bytes_f / n_f).round() } else { 0.0 },
    });
    if n >= 2 {
        let gaps: Vec<f64> = arrivals
            .windows(2)
            .map(|w| w[1].0.saturating_sub(w[0].0).as_secs_f64())
            .collect();
        #[expect(clippy::cast_precision_loss, reason = "a few thousand gaps at most")]
        let mean = gaps.iter().sum::<f64>() / gaps.len() as f64;
        #[expect(clippy::cast_precision_loss, reason = "a few thousand gaps at most")]
        let var = gaps.iter().map(|g| (g - mean).powi(2)).sum::<f64>() / gaps.len() as f64;
        let round = |x: f64| (x * 1000.0).round() / 1000.0;
        out["rate_hz"] = json!(round(1.0 / mean));
        out["gap_min_s"] = json!(round(gaps.iter().copied().fold(f64::INFINITY, f64::min)));
        out["gap_max_s"] = json!(round(gaps.iter().copied().fold(0.0, f64::max)));
        out["gap_std_s"] = json!(round(var.sqrt()));
    }
    out
}

#[async_trait]
impl Tool for TopicSample {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let topic = match name_arg(&args, "topic") {
            Ok(t) => t,
            Err(out) => return out,
        };
        if self
            .ctx
            .config
            .read_deny
            .iter()
            .any(|p| glob_match(p, &topic))
        {
            return ToolOutcome::refused(format!("`{topic}` may not be read by this agent"));
        }
        let ty = match self.ctx.topic_type(&topic).await {
            Ok(t) => t,
            Err(out) => return out,
        };
        let mode = args["mode"].as_str().unwrap_or("echo");
        let window = seconds(&args, "seconds", 3.0, MAX_SAMPLE);
        match mode {
            "hz" | "bw" => {
                let arrivals = match self
                    .ctx
                    .robot
                    .sample_sizes(&topic, &ty, window, 10_000)
                    .await
                {
                    Ok(a) => a,
                    Err(e) => return failed(&e),
                };
                let mut data = rate_stats(&arrivals, window);
                data["topic"] = json!(topic);
                data["type"] = json!(ty);
                if arrivals.len() < 2
                    && let Ok(ends) = self.ctx.robot.endpoints(&topic).await
                {
                    data["publishers"] = json!(
                        ends.publishers
                            .iter()
                            .map(endpoint_json)
                            .collect::<Vec<_>>()
                    );
                    data["note"] = json!(if ends.publishers.is_empty() {
                        "nobody publishes on this topic"
                    } else {
                        "the publishers sent next to nothing in the window"
                    });
                }
                ToolOutcome::ok(data)
            }
            "echo" => {
                if BULK_TYPES.contains(&ty.as_str()) {
                    return ToolOutcome::refused(format!(
                        "`{ty}` is too large to echo; use mode hz or bw, or look for images"
                    ));
                }
                let n = count(&args, "count", 1, MAX_ECHO);
                let messages = match self.ctx.robot.sample_messages(&topic, &ty, n, window).await {
                    Ok(m) => m,
                    Err(e) => return failed(&e),
                };
                let fields: Vec<String> = args["fields"]
                    .as_array()
                    .map(|f| {
                        f.iter()
                            .filter_map(|v| v.as_str().map(str::to_owned))
                            .collect()
                    })
                    .unwrap_or_default();
                let shown: Vec<Value> = messages
                    .iter()
                    .map(|m| {
                        if fields.is_empty() {
                            summarize(m)
                        } else {
                            summarize(&pick(m, &fields))
                        }
                    })
                    .collect();
                if shown.is_empty() {
                    return ToolOutcome::failed(format!(
                        "no message on `{topic}` within {:.1} s",
                        window.as_secs_f64()
                    ));
                }
                ToolOutcome::ok(json!({"topic": topic, "type": ty, "messages": shown}))
            }
            other => ToolOutcome::failed(format!("mode `{other}` is not one of echo, hz, bw")),
        }
    }
}

// --- interface_show --------------------------------------------------------------------------

struct InterfaceShow {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

#[async_trait]
impl Tool for InterfaceShow {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let ty = args["type"].as_str().unwrap_or_default();
        match self.ctx.schemas.show(ty) {
            Some(text) => ToolOutcome::ok(json!({"type": ty, "definition": text})),
            None => ToolOutcome::failed(format!(
                "`{ty}` is not an interface this agent knows; give pkg/msg/Name, pkg/srv/Name or \
                 pkg/action/Name"
            )),
        }
    }
}

// --- tf --------------------------------------------------------------------------------------

struct Tf {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

#[async_trait]
impl Tool for Tf {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let round = |x: f64| (x * 1000.0).round() / 1000.0;
        if args["op"].as_str() == Some("tree") {
            let links: Vec<Value> = self
                .ctx
                .robot
                .tf_links()
                .iter()
                .map(|l| {
                    json!({"parent": l.parent, "child": l.child, "static": l.is_static,
                           "age_s": round(l.age.as_secs_f64())})
                })
                .collect();
            return ToolOutcome::ok(json!({"frames": links.len(), "links": links}));
        }
        let target = args["target"].as_str().unwrap_or("map");
        let Some(source) = args["source"].as_str() else {
            return ToolOutcome::failed("a lookup needs source, the frame to locate");
        };
        match self.ctx.robot.transform(target, source) {
            Ok(t) => ToolOutcome::ok(json!({
                "target": target,
                "source": source,
                "translation_m": t.translation.map(round),
                "rotation_xyzw": t.rotation.map(round),
                "yaw_deg": round(t.yaw().to_degrees()),
            })),
            Err(e) => failed(&e),
        }
    }
}

// --- params ----------------------------------------------------------------------------------

struct Params {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

/// A `rcl_interfaces/msg/ParameterValue` as plain JSON.
fn param_value(v: &Value) -> Value {
    match v["type"].as_u64() {
        Some(1) => v["bool_value"].clone(),
        Some(2) => v["integer_value"].clone(),
        Some(3) => v["double_value"].clone(),
        Some(4) => v["string_value"].clone(),
        Some(5) => v["byte_array_value"].clone(),
        Some(6) => v["bool_array_value"].clone(),
        Some(7) => v["integer_array_value"].clone(),
        Some(8) => v["double_array_value"].clone(),
        Some(9) => v["string_array_value"].clone(),
        _ => Value::Null,
    }
}

/// Plain JSON as a `ParameterValue` of the parameter's current type, so `1` for a double is `1.0`.
fn to_param_value(type_id: u64, value: &Value) -> Result<Value, String> {
    let bad = |what: &str| format!("the parameter holds {what}, which `{value}` is not");
    Ok(match type_id {
        1 => json!({"type": 1, "bool_value": value.as_bool().ok_or_else(|| bad("true or false"))?}),
        2 => {
            json!({"type": 2, "integer_value": value.as_i64().ok_or_else(|| bad("a whole number"))?})
        }
        3 => json!({"type": 3, "double_value": value.as_f64().ok_or_else(|| bad("a number"))?}),
        4 => json!({"type": 4, "string_value": value.as_str().ok_or_else(|| bad("text"))?}),
        7 => {
            json!({"type": 7, "integer_array_value": value.as_array().ok_or_else(|| bad("a list of whole numbers"))?})
        }
        8 => {
            json!({"type": 8, "double_array_value": value.as_array().ok_or_else(|| bad("a list of numbers"))?})
        }
        9 => {
            json!({"type": 9, "string_array_value": value.as_array().ok_or_else(|| bad("a list of text"))?})
        }
        6 => {
            json!({"type": 6, "bool_array_value": value.as_array().ok_or_else(|| bad("a list of true or false"))?})
        }
        _ => return Err("the parameter is not set or of a type this tool cannot write".to_owned()),
    })
}

impl Ctx {
    async fn param_call(
        &self,
        node: &str,
        service: &str,
        ty: &str,
        request: Value,
    ) -> Result<Value, ToolOutcome> {
        self.robot
            .call(
                &format!("{node}/{service}"),
                ty,
                request,
                Duration::from_secs(5),
            )
            .await
            .map_err(|e| failed(&e))
    }

    async fn param_values(
        &self,
        node: &str,
        names: &[String],
    ) -> Result<Vec<(String, Value, u64)>, ToolOutcome> {
        let out = self
            .param_call(
                node,
                "get_parameters",
                "rcl_interfaces/srv/GetParameters",
                json!({"names": names}),
            )
            .await?;
        let values = out["values"].as_array().cloned().unwrap_or_default();
        Ok(names
            .iter()
            .zip(values)
            .map(|(n, v)| (n.clone(), param_value(&v), v["type"].as_u64().unwrap_or(0)))
            .collect())
    }

    async fn param_names(&self, node: &str) -> Result<Vec<String>, ToolOutcome> {
        let out = self
            .param_call(
                node,
                "list_parameters",
                "rcl_interfaces/srv/ListParameters",
                json!({"prefixes": [], "depth": 0}),
            )
            .await?;
        Ok(out["result"]["names"]
            .as_array()
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str().map(str::to_owned))
                    .collect()
            })
            .unwrap_or_default())
    }

    fn masked(&self, name: &str) -> bool {
        self.config
            .param_read_deny
            .iter()
            .any(|p| glob_match(p, name))
    }
}

#[async_trait]
impl Tool for Params {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let node = match name_arg(&args, "node") {
            Ok(n) => n,
            Err(out) => return out,
        };
        let mut names: Vec<String> = args["names"]
            .as_array()
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str().map(str::to_owned))
                    .collect()
            })
            .unwrap_or_default();
        if let Some(one) = args["name"].as_str() {
            names.push(one.to_owned());
        }
        match args["op"].as_str().unwrap_or("list") {
            "list" => {
                let prefix = args["prefix"].as_str().unwrap_or_default();
                let prefixes: Vec<&str> = if prefix.is_empty() {
                    vec![]
                } else {
                    vec![prefix]
                };
                match self
                    .ctx
                    .param_call(
                        &node,
                        "list_parameters",
                        "rcl_interfaces/srv/ListParameters",
                        json!({"prefixes": prefixes, "depth": 0}),
                    )
                    .await
                {
                    Ok(out) => {
                        ToolOutcome::ok(json!({"node": node, "names": out["result"]["names"]}))
                    }
                    Err(out) => out,
                }
            }
            "get" => {
                // No names asks for them all, which is what a model means by it.
                if names.is_empty() {
                    match self.ctx.param_names(&node).await {
                        Ok(all) => names = all.into_iter().take(PARAMS_MAX).collect(),
                        Err(out) => return out,
                    }
                }
                match self.ctx.param_values(&node, &names).await {
                    Ok(values) => {
                        let mut out = Map::new();
                        for (n, v, _) in values {
                            let shown = if self.ctx.masked(&n) {
                                json!("(hidden)")
                            } else {
                                v
                            };
                            out.insert(n, shown);
                        }
                        ToolOutcome::ok(json!({"node": node, "values": out}))
                    }
                    Err(out) => out,
                }
            }
            "describe" => {
                if names.is_empty() {
                    return ToolOutcome::failed("describe needs names; list them first");
                }
                match self
                    .ctx
                    .param_call(
                        &node,
                        "describe_parameters",
                        "rcl_interfaces/srv/DescribeParameters",
                        json!({"names": names}),
                    )
                    .await
                {
                    Ok(out) => ToolOutcome::ok(
                        json!({"node": node, "descriptors": summarize(&out["descriptors"])}),
                    ),
                    Err(out) => out,
                }
            }
            other => ToolOutcome::failed(format!("op `{other}` is not one of list, get, describe")),
        }
    }
}

// --- log_tail --------------------------------------------------------------------------------

struct LogTail {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

fn level_number(word: &str) -> u64 {
    match word {
        "debug" => 10,
        "info" => 20,
        "error" => 40,
        "fatal" => 50,
        _ => 30,
    }
}

fn level_word(n: u64) -> &'static str {
    match n {
        0..=10 => "DEBUG",
        11..=20 => "INFO",
        21..=30 => "WARN",
        31..=40 => "ERROR",
        _ => "FATAL",
    }
}

#[async_trait]
impl Tool for LogTail {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let min = level_number(args["min_level"].as_str().unwrap_or("warn"));
        let node = args["node"]
            .as_str()
            .unwrap_or_default()
            .trim_start_matches('/')
            .to_owned();
        let window = seconds(&args, "seconds", 2.0, MAX_SAMPLE);
        let max = count(&args, "max", 30, 100);
        // /rosout keeps recent history for late subscribers, so this sees the last few seconds too.
        let logs = match self
            .ctx
            .robot
            .sample_messages("/rosout", "rcl_interfaces/msg/Log", 500, window)
            .await
        {
            Ok(l) => l,
            Err(e) => return failed(&e),
        };
        let lines: Vec<String> = logs
            .iter()
            .filter(|l| l["level"].as_u64().unwrap_or(0) >= min)
            .filter(|l| node.is_empty() || l["name"].as_str().unwrap_or_default().contains(&node))
            .map(|l| {
                let text: String = l["msg"]
                    .as_str()
                    .unwrap_or_default()
                    .chars()
                    .take(300)
                    .collect();
                format!(
                    "[{}] {}: {text}",
                    level_word(l["level"].as_u64().unwrap_or(0)),
                    l["name"].as_str().unwrap_or("?")
                )
            })
            .collect();
        let skip = lines.len().saturating_sub(max);
        ToolOutcome::ok(json!({"lines": lines[skip..], "matched": lines.len()}))
    }
}

// --- service_call ----------------------------------------------------------------------------

struct ServiceCall {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
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

impl ServiceCall {
    fn observes(&self, service: &str) -> bool {
        self.ctx
            .config
            .service_observe
            .iter()
            .any(|p| glob_match(p, service))
    }

    async fn prepare(&self, args: &Value) -> Result<Prepared, ToolOutcome> {
        let service = name_arg(args, "service")?;
        let listed = self
            .ctx
            .config
            .service_call
            .iter()
            .any(|p| glob_match(p, &service));
        if !listed && !self.observes(&service) {
            return Err(ToolOutcome::refused(format!(
                "the profile does not let this agent call `{service}`"
            )));
        }
        let ty = match args["type"].as_str().filter(|t| !t.is_empty()) {
            Some(t) => t.to_owned(),
            None => self.ctx.service_type(&service).await?,
        };
        if let Some(out) = self.ctx.deny_act(&service, &ty) {
            return Err(out);
        }
        let request = payload(args, "request");
        self.ctx
            .schemas
            .validate(&ty, SchemaPart::Request, &request)
            .map_err(|why| ToolOutcome::failed(format!("the request does not fit {ty}: {why}")))?;
        Ok(Prepared {
            name: service,
            ty,
            payload: request,
        })
    }
}

#[async_trait]
impl Tool for ServiceCall {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        Some(self.prepare(args).await.map(|p| {
            if self.observes(&p.name) {
                Assessment {
                    risk: Risk::Observe,
                    resources: Vec::new(),
                    reason: format!("reads {} ({})", p.name, p.ty),
                }
            } else {
                Assessment {
                    risk: Risk::Motion,
                    resources: ALL.to_vec(),
                    reason: format!("calls {} ({}) with {}", p.name, p.ty, p.payload),
                }
            }
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let p = match self.prepare(&args).await {
            Ok(p) => p,
            Err(out) => return out,
        };
        let timeout = seconds(&args, "timeout_s", 5.0, MAX_CALL);
        match self
            .ctx
            .robot
            .call(&p.name, &p.ty, p.payload, timeout)
            .await
        {
            Ok(response) => ToolOutcome::ok(
                json!({"service": p.name, "type": p.ty, "response": summarize(&response)}),
            ),
            Err(e) => failed(&e),
        }
    }
}

// --- action_goal -----------------------------------------------------------------------------

struct ActionGoal {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

/// Cancels a goal if the call that sent it is dropped, as when the operator stops the turn.
struct CancelOnDrop(Option<nervros_ros::CancelFn>);

impl Drop for CancelOnDrop {
    fn drop(&mut self) {
        if let Some(cancel) = self.0.take()
            && let Ok(runtime) = tokio::runtime::Handle::try_current()
        {
            runtime.spawn(async move {
                let _ = cancel().await;
            });
        }
    }
}

impl ActionGoal {
    fn cancelling(args: &Value) -> bool {
        args["op"].as_str() == Some("cancel")
    }

    async fn prepare(&self, args: &Value) -> Result<Prepared, ToolOutcome> {
        let action = name_arg(args, "action")?;
        if !self
            .ctx
            .config
            .action_send
            .iter()
            .any(|p| glob_match(p, &action))
        {
            return Err(ToolOutcome::refused(format!(
                "the profile does not let this agent send goals to `{action}`"
            )));
        }
        let ty = match args["type"].as_str().filter(|t| !t.is_empty()) {
            Some(t) => t.to_owned(),
            None => self.ctx.action_type(&action).await?,
        };
        if let Some(out) = self.ctx.deny_act(&action, &ty) {
            return Err(out);
        }
        let goal = payload(args, "goal");
        if !Self::cancelling(args) {
            self.ctx
                .schemas
                .validate(&ty, SchemaPart::Goal, &goal)
                .map_err(|why| ToolOutcome::failed(format!("the goal does not fit {ty}: {why}")))?;
        }
        Ok(Prepared {
            name: action,
            ty,
            payload: goal,
        })
    }

    async fn cancel_all(&self, action: &str) -> ToolOutcome {
        // A zero goal id and stamp cancel every goal the server has.
        let request = json!({"goal_info": {"goal_id": {"uuid": vec![0_u8; 16]},
                                           "stamp": {"sec": 0, "nanosec": 0}}});
        match self
            .ctx
            .robot
            .call(
                &format!("{action}/_action/cancel_goal"),
                "action_msgs/srv/CancelGoal",
                request,
                Duration::from_secs(5),
            )
            .await
        {
            Ok(out) => ToolOutcome::ok(json!({
                "action": action,
                "cancelling": out["goals_canceling"].as_array().map_or(0, Vec::len),
            })),
            Err(e) => failed(&e),
        }
    }
}

#[async_trait]
impl Tool for ActionGoal {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        Some(self.prepare(args).await.map(|p| Assessment {
            risk: Risk::Motion,
            resources: ALL.to_vec(),
            reason: if Self::cancelling(args) {
                format!("cancels every goal of {} ({})", p.name, p.ty)
            } else {
                format!("sends {} ({}) the goal {}", p.name, p.ty, p.payload)
            },
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let p = match self.prepare(&args).await {
            Ok(p) => p,
            Err(out) => return out,
        };
        if Self::cancelling(&args) {
            return self.cancel_all(&p.name).await;
        }
        let wait = seconds(&args, "wait_s", 30.0, MAX_GOAL_WAIT);
        let mut handle = match self
            .ctx
            .robot
            .send_goal(&p.name, &p.ty, p.payload, Duration::from_secs(5))
            .await
        {
            Ok(h) => h,
            Err(e) => return failed(&e),
        };
        let mut guard = CancelOnDrop(Some(handle.canceller()));
        let mut feedback = Vec::new();
        let deadline = tokio::time::Instant::now() + wait;
        let result = loop {
            tokio::select! {
                r = &mut handle.result => break Some(r),
                Some(f) = handle.feedback.recv() => {
                    feedback.push(summarize(&f));
                    if feedback.len() > 3 {
                        feedback.remove(0);
                    }
                }
                () = tokio::time::sleep_until(deadline) => break None,
            }
        };
        if result.is_some() {
            guard.0 = None;
        }
        match result {
            Some(Ok(Ok(done))) => ToolOutcome::ok(json!({
                "action": p.name,
                "status": format!("{:?}", done.status).to_lowercase(),
                "result": summarize(&done.result),
                "last_feedback": feedback,
            })),
            Some(Ok(Err(e))) => failed(&e),
            Some(Err(_)) => ToolOutcome::failed("the action server went away before it answered"),
            // Dropping the guard cancels the goal: nothing runs on after the tool returns.
            None => ToolOutcome::failed(format!(
                "no result within {:.0} s, so the goal was cancelled; last feedback: {}",
                wait.as_secs_f64(),
                json!(feedback)
            )),
        }
    }
}

// --- param_set -------------------------------------------------------------------------------

struct ParamSet {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

/// A parameter change, resolved: the node, the name, the value in the parameter's own type and
/// the value it has now.
struct ParamChange {
    node: String,
    name: String,
    value: Value,
    was: Value,
}

impl ParamSet {
    async fn prepare(&self, args: &Value) -> Result<ParamChange, ToolOutcome> {
        let node = name_arg(args, "node")?;
        let name = args["name"].as_str().unwrap_or_default().to_owned();
        let key = format!("{node}:{name}");
        if name.is_empty()
            || !self
                .ctx
                .config
                .param_set
                .iter()
                .any(|p| glob_match(p, &key))
        {
            return Err(ToolOutcome::refused(format!(
                "the profile does not let this agent set `{key}`"
            )));
        }
        // A node under a denied namespace, such as /controller_manager/*, keeps its parameters.
        if self.ctx.guard.hard_denied(&format!("{node}/")) {
            return Err(ToolOutcome::refused(format!(
                "`{node}` is on the robot's hard deny list"
            )));
        }
        let before = self
            .ctx
            .param_values(&node, std::slice::from_ref(&name))
            .await?;
        let (_, was, type_id) = before.into_iter().next().unwrap_or_default();
        let value = to_param_value(type_id, &args["value"]).map_err(ToolOutcome::failed)?;
        Ok(ParamChange {
            node,
            name,
            value,
            was,
        })
    }
}

#[async_trait]
impl Tool for ParamSet {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        Some(self.prepare(args).await.map(|c| Assessment {
            risk: Risk::WorldEdit,
            resources: Vec::new(),
            reason: format!(
                "sets {} on {} from {} to {}",
                c.name,
                c.node,
                c.was,
                param_value(&c.value)
            ),
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let c = match self.prepare(&args).await {
            Ok(c) => c,
            Err(out) => return out,
        };
        let request = json!({"parameters": [{"name": c.name, "value": c.value}]});
        let out = match self
            .ctx
            .param_call(
                &c.node,
                "set_parameters",
                "rcl_interfaces/srv/SetParameters",
                request,
            )
            .await
        {
            Ok(o) => o,
            Err(out) => return out,
        };
        let result = &out["results"][0];
        if result["successful"].as_bool() != Some(true) {
            return ToolOutcome::failed(format!(
                "{} refused: {}",
                c.node,
                result["reason"].as_str().unwrap_or("no reason given")
            ));
        }
        let after = self
            .ctx
            .param_values(&c.node, std::slice::from_ref(&c.name))
            .await
            .ok();
        ToolOutcome::ok(json!({
            "node": c.node,
            "name": c.name,
            "was": c.was,
            "now": after.and_then(|a| a.first().map(|(_, v, _)| v.clone())),
        }))
    }
}

// --- topic_publish ---------------------------------------------------------------------------

struct TopicPublish {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

impl TopicPublish {
    async fn prepare(&self, args: &Value) -> Result<Prepared, ToolOutcome> {
        let topic = name_arg(args, "topic")?;
        if !self
            .ctx
            .config
            .publish
            .iter()
            .any(|p| glob_match(p, &topic))
        {
            return Err(ToolOutcome::refused(format!(
                "the profile does not let this agent publish on `{topic}`"
            )));
        }
        let ty = match args["type"].as_str().filter(|t| !t.is_empty()) {
            Some(t) => t.to_owned(),
            None => self.ctx.topic_type(&topic).await.map_err(|_| {
                ToolOutcome::failed(format!(
                    "`{topic}` is not in the graph yet; give its type, such as std_msgs/msg/String"
                ))
            })?,
        };
        if let Some(out) = self.ctx.deny_act(&topic, &ty) {
            return Err(out);
        }
        let message = payload(args, "message");
        self.ctx
            .schemas
            .validate(&ty, SchemaPart::Message, &message)
            .map_err(|why| ToolOutcome::failed(format!("the message does not fit {ty}: {why}")))?;
        Ok(Prepared {
            name: topic,
            ty,
            payload: message,
        })
    }
}

#[async_trait]
impl Tool for TopicPublish {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let times = count(args, "count", 1, MAX_PUBLISH);
        Some(self.prepare(args).await.map(|p| Assessment {
            risk: Risk::Motion,
            resources: ALL.to_vec(),
            reason: format!(
                "publishes {} on {} ({}) {times} time(s)",
                p.payload, p.name, p.ty
            ),
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let p = match self.prepare(&args).await {
            Ok(p) => p,
            Err(out) => return out,
        };
        let times = count(&args, "count", 1, MAX_PUBLISH);
        let hz = args["rate_hz"]
            .as_f64()
            .unwrap_or(1.0)
            .clamp(0.1, MAX_PUBLISH_HZ);
        match self
            .ctx
            .robot
            .publish(
                &p.name,
                &p.ty,
                p.payload,
                times,
                Duration::from_secs_f64(1.0 / hz),
            )
            .await
        {
            Ok(matched) => ToolOutcome::ok(json!({
                "topic": p.name,
                "type": p.ty,
                "published": times,
                "subscribers": matched,
            })),
            Err(e) => failed(&e),
        }
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
    let obj = |props: Value, required: &[&str]| json!({"type": "object", "properties": props, "required": required});
    let mut out: Vec<Arc<dyn Tool>> = vec![
        Arc::new(RosGraph {
            spec: spec(
                "ros_graph",
                "Look at the ROS graph, like ros2 topic/service/action/node list and info. Without \
                 name: lists topics, services, actions or nodes, optionally filtered by a substring \
                 or glob. With name: details of that topic (types, publishers, subscribers, QoS and \
                 mismatches), node (what it publishes, subscribes to, serves, calls), service (type, \
                 definition, reachable) or action (type, definition).",
                obj(
                    json!({
                        "kind": {"type": "string", "enum": ["topics", "services", "actions", "nodes"]},
                        "filter": {"type": "string", "description": "Substring or glob such as /canopy/*"},
                        "name": {"type": "string", "description": "An absolute name to describe"},
                        "all": {"type": "boolean", "description": "Include hidden names such as parameter services"},
                    }),
                    &[],
                ),
                Risk::Observe,
            ),
            ctx: Arc::clone(&ctx),
        }),
        Arc::new(TopicSample {
            spec: spec(
                "topic_sample",
                "Sample a topic, like ros2 topic echo/hz/bw: echo shows up to 10 messages (long \
                 arrays and strings shortened; fields picks dotted fields); hz measures the rate and \
                 its jitter; bw the bytes per second. Images and point clouds are too large to echo.",
                obj(
                    json!({
                        "topic": {"type": "string", "description": "Absolute topic name"},
                        "mode": {"type": "string", "enum": ["echo", "hz", "bw"]},
                        "seconds": {"type": "number", "description": "How long to listen, up to 10 (3)"},
                        "count": {"type": "integer", "description": "Messages to echo, up to 10 (1)"},
                        "fields": {"type": "array", "items": {"type": "string"}, "description": "Only these fields, e.g. header.stamp"},
                    }),
                    &["topic"],
                ),
                Risk::Observe,
            ),
            ctx: Arc::clone(&ctx),
        }),
        Arc::new(InterfaceShow {
            spec: spec(
                "interface_show",
                "Show a message, service or action definition, like ros2 interface show.",
                obj(
                    json!({"type": {"type": "string", "description": "pkg/msg/Name, pkg/srv/Name or pkg/action/Name"}}),
                    &["type"],
                ),
                Risk::Observe,
            ),
            ctx: Arc::clone(&ctx),
        }),
        Arc::new(Tf {
            spec: spec(
                "tf",
                "TF: lookup gives where the source frame is in the target frame (metres, degrees); \
                 tree lists every frame link with its parent, whether it is static and how old it is.",
                obj(
                    json!({
                        "op": {"type": "string", "enum": ["lookup", "tree"]},
                        "target": {"type": "string", "description": "Frame to express the pose in (map)"},
                        "source": {"type": "string", "description": "Frame to locate"},
                    }),
                    &[],
                ),
                Risk::Observe,
            ),
            ctx: Arc::clone(&ctx),
        }),
        Arc::new(Params {
            spec: spec(
                "params",
                "A node's parameters, like ros2 param list/get/describe.",
                obj(
                    json!({
                        "op": {"type": "string", "enum": ["list", "get", "describe"]},
                        "node": {"type": "string", "description": "Absolute node name"},
                        "names": {"type": "array", "items": {"type": "string"}},
                        "prefix": {"type": "string"},
                    }),
                    &["node"],
                ),
                Risk::Observe,
            ),
            ctx: Arc::clone(&ctx),
        }),
        Arc::new(LogTail {
            spec: spec(
                "log_tail",
                "Recent ROS log lines from /rosout, newest last: warnings and errors by default.",
                obj(
                    json!({
                        "min_level": {"type": "string", "enum": ["debug", "info", "warn", "error", "fatal"]},
                        "node": {"type": "string", "description": "Only nodes whose name contains this"},
                        "seconds": {"type": "number", "description": "How long to listen, up to 10 (2)"},
                        "max": {"type": "integer", "description": "Most lines to return (30)"},
                    }),
                    &[],
                ),
                Risk::Observe,
            ),
            ctx: Arc::clone(&ctx),
        }),
    ];
    if !config.service_call.is_empty() {
        out.push(Arc::new(ServiceCall {
            spec: spec(
                "service_call",
                "Call any ROS service, like ros2 service call. The type is looked up when omitted; \
                 ros_graph with the name shows the request fields. Services that only read run at \
                 once; the rest need the robot armed and the operator's approval.",
                obj(
                    json!({
                        "service": {"type": "string", "description": "Absolute service name"},
                        "type": {"type": "string", "description": "pkg/srv/Name, if known"},
                        "request": {"type": "object", "description": "The request fields as JSON"},
                        "timeout_s": {"type": "number", "description": "Up to 30 (5)"},
                    }),
                    &["service"],
                ),
                Risk::Motion,
            ),
            ctx: Arc::clone(&ctx),
        }));
    }
    if !config.action_send.is_empty() {
        out.push(Arc::new(ActionGoal {
            spec: spec(
                "action_goal",
                "Send a goal to any ROS action and wait for its result, like ros2 action send_goal, \
                 or cancel its goals. A goal still running after wait_s is cancelled. Needs the \
                 robot armed and the operator's approval. Moving the robot is for its skills and \
                 missions: use this for other actions.",
                obj(json!({
                    "op": {"type": "string", "enum": ["send", "cancel"]},
                    "action": {"type": "string", "description": "Absolute action name"},
                    "type": {"type": "string", "description": "pkg/action/Name, if known"},
                    "goal": {"type": "object", "description": "The goal fields as JSON"},
                    "wait_s": {"type": "number", "description": "How long to wait for the result, up to 120 (30)"},
                }), &["action"]),
                Risk::Motion,
            ),
            ctx: Arc::clone(&ctx),
        }));
    }
    if !config.param_set.is_empty() {
        out.push(Arc::new(ParamSet {
            spec: spec(
                "param_set",
                "Set one parameter of a node, like ros2 param set, and read it back. The value is \
                 converted to the parameter's current type. Needs the operator's approval.",
                obj(
                    json!({
                        "node": {"type": "string", "description": "Absolute node name"},
                        "name": {"type": "string"},
                        "value": {"description": "The new value"},
                    }),
                    &["node", "name", "value"],
                ),
                Risk::WorldEdit,
            ),
            ctx: Arc::clone(&ctx),
        }));
    }
    if !config.publish.is_empty() {
        out.push(Arc::new(TopicPublish {
            spec: spec(
                "topic_publish",
                "Publish a message on a topic a few times, like ros2 topic pub --times. Velocity, \
                 joint and low-level command topics are refused: the robot moves through its skills. \
                 Needs the robot armed and the operator's approval.",
                obj(json!({
                    "topic": {"type": "string", "description": "Absolute topic name"},
                    "type": {"type": "string", "description": "pkg/msg/Name; needed for a topic not in the graph"},
                    "message": {"type": "object", "description": "The message fields as JSON"},
                    "count": {"type": "integer", "description": "How many times, up to 10 (1)"},
                    "rate_hz": {"type": "number", "description": "Up to 10 (1)"},
                }), &["topic", "message"]),
                Risk::Motion,
            ),
            ctx,
        }));
    }
    out
}

#[cfg(test)]
mod tests {
    use nervros_ros::fake::FakeRobot;
    use nervros_ros::{NodeEntities, QosInfo, TopicEndpoints};

    use super::*;
    use crate::guard::Policy;

    /// Checks array lengths like the real registry for one fixed-array request.
    struct Schemas;

    impl SchemaSource for Schemas {
        fn schema(&self, _t: &str, _p: SchemaPart, _h: &[String]) -> Result<Value, String> {
            Ok(json!({"type": "object"}))
        }
        fn validate(&self, ros_type: &str, _p: SchemaPart, value: &Value) -> Result<(), String> {
            if ros_type == "demo/srv/Fixed"
                && value["uuid"].as_array().is_some_and(|a| a.len() != 16)
            {
                return Err("`uuid` must have exactly 16 items".to_owned());
            }
            Ok(())
        }
        fn show(&self, ros_type: &str) -> Option<String> {
            (ros_type == "std_srvs/srv/Trigger")
                .then(|| "---\nbool success\nstring message\n".to_owned())
        }
    }

    fn qos(reliability: &str, durability: &str) -> QosInfo {
        QosInfo {
            reliability: reliability.into(),
            durability: durability.into(),
            history: "keep_last".into(),
            depth: 10,
        }
    }

    fn robot() -> FakeRobot {
        let graph = GraphDetail {
            topics: vec![
                ("/odom".into(), vec!["nav_msgs/msg/Odometry".into()]),
                ("/map".into(), vec!["nav_msgs/msg/OccupancyGrid".into()]),
                (
                    "/spin/_action/feedback".into(),
                    vec!["nav2_msgs/action/Spin_FeedbackMessage".into()],
                ),
                ("/status".into(), vec!["std_msgs/msg/String".into()]),
                ("/cmd_vel".into(), vec!["geometry_msgs/msg/Twist".into()]),
            ],
            services: vec![
                ("/reset".into(), vec!["std_srvs/srv/Trigger".into()]),
                ("/get_state".into(), vec!["std_srvs/srv/Trigger".into()]),
                ("/fixed".into(), vec!["demo/srv/Fixed".into()]),
                (
                    "/detector/get_parameters".into(),
                    vec!["rcl_interfaces/srv/GetParameters".into()],
                ),
            ],
            nodes: vec!["/detector".into(), "/nav".into()],
        };
        let reliable_sub = Endpoint {
            node: "/viewer".into(),
            topic_type: "std_msgs/msg/String".into(),
            qos: qos("reliable", "volatile"),
        };
        let lossy_pub = Endpoint {
            node: "/talker".into(),
            topic_type: "std_msgs/msg/String".into(),
            qos: qos("best_effort", "volatile"),
        };
        FakeRobot::new()
            .with_graph(graph)
            .with_endpoints(
                "/status",
                TopicEndpoints {
                    publishers: vec![lossy_pub],
                    subscribers: vec![reliable_sub],
                },
            )
            .with_node(
                "/nav",
                NodeEntities {
                    publishers: vec![("/plan".into(), vec!["nav_msgs/msg/Path".into()])],
                    ..NodeEntities::default()
                },
            )
            .with_rate("/odom", 50.0, 700)
            .with_topic(
                "/status",
                json!({"data": "x".repeat(300), "list": (0..20).collect::<Vec<_>>()}),
            )
            .with_topic(
                "/rosout",
                json!({"level": 40, "name": "nav", "msg": "planner failed"}),
            )
            .with_service("/reset", |_| Ok(json!({"success": true, "message": ""})))
            .with_service("/get_state", |_| {
                Ok(json!({"success": true, "message": "ok"}))
            })
            .with_service("/detector/get_parameters", |req| {
                let n = req["names"].as_array().map_or(0, Vec::len);
                Ok(json!({"values": vec![json!({"type": 3, "double_value": 0.5}); n]}))
            })
            .with_service("/detector/set_parameters", |_| {
                Ok(json!({"results": [{"successful": true, "reason": ""}]}))
            })
    }

    fn all_tools(
        config: &RosToolsConfig,
        armed: bool,
    ) -> (Arc<FakeRobot>, Vec<Arc<dyn Tool>>, Arc<Guard>) {
        let fake = Arc::new(robot());
        let robot: Arc<dyn RobotPort> = Arc::clone(&fake) as Arc<dyn RobotPort>;
        let schemas: Arc<dyn SchemaSource> = Arc::new(Schemas);
        let guard = Arc::new(Guard::new(Policy::default()));
        guard.set_armed(armed);
        let tools = tools(config, &robot, &schemas, &guard);
        (fake, tools, guard)
    }

    fn open() -> RosToolsConfig {
        RosToolsConfig {
            service_call: vec!["*".into()],
            action_send: vec!["*".into()],
            publish: vec!["*".into()],
            param_set: vec!["/detector:*".into()],
            ..RosToolsConfig::default()
        }
    }

    fn tool<'a>(tools: &'a [Arc<dyn Tool>], name: &str) -> &'a Arc<dyn Tool> {
        tools.iter().find(|t| t.spec().name == name).unwrap()
    }

    #[test]
    fn act_tools_exist_only_when_the_profile_lists_them() {
        let (_, closed, _) = all_tools(&RosToolsConfig::default(), false);
        let names: Vec<String> = closed.iter().map(|t| t.spec().name.clone()).collect();
        assert_eq!(
            names,
            [
                "ros_graph",
                "topic_sample",
                "interface_show",
                "tf",
                "params",
                "log_tail"
            ]
        );
        let (_, all, _) = all_tools(&open(), false);
        assert_eq!(all.len(), 10);
    }

    #[tokio::test]
    async fn the_graph_lists_hides_internals_and_derives_actions() {
        let (_, tools, _) = all_tools(&RosToolsConfig::default(), false);
        let graph = tool(&tools, "ros_graph");
        let topics = graph.call(json!({"kind": "topics"})).await;
        let names: Vec<&str> = topics.data["items"]
            .as_array()
            .unwrap()
            .iter()
            .map(|i| i["name"].as_str().unwrap())
            .collect();
        assert!(!names.iter().any(|n| n.contains("_action")), "{names:?}");
        let actions = graph.call(json!({"kind": "actions"})).await;
        assert_eq!(
            actions.data["items"][0],
            json!({"name": "/spin", "type": "nav2_msgs/action/Spin"})
        );
        let services = graph
            .call(json!({"kind": "services", "filter": "get"}))
            .await;
        assert_eq!(
            services.data["total"], 1,
            "parameter services are hidden: {}",
            services.data
        );
    }

    #[tokio::test]
    async fn topic_info_finds_the_qos_mismatch_that_drops_every_message() {
        let (_, tools, _) = all_tools(&RosToolsConfig::default(), false);
        let info = tool(&tools, "ros_graph")
            .call(json!({"name": "/status"}))
            .await;
        assert_eq!(
            info.data["qos_mismatch"][0],
            json!({"publisher": "/talker", "subscriber": "/viewer"})
        );
        let node = tool(&tools, "ros_graph")
            .call(json!({"name": "/nav"}))
            .await;
        assert_eq!(node.data["publishers"][0]["name"], "/plan");
    }

    #[tokio::test]
    async fn hz_and_echo_measure_and_summarise() {
        let (_, tools, _) = all_tools(&RosToolsConfig::default(), false);
        let sample = tool(&tools, "topic_sample");
        let hz = sample
            .call(json!({"topic": "/odom", "mode": "hz", "seconds": 2}))
            .await;
        assert_eq!(hz.data["rate_hz"], 50.0, "{}", hz.data);
        assert_eq!(
            hz.data["bytes_per_s"], 35000.0,
            "100 messages of 700 bytes in 2 s"
        );
        let echo = sample.call(json!({"topic": "/status"})).await;
        let msg = &echo.data["messages"][0];
        assert!(msg["data"].as_str().unwrap().ends_with("(300 characters)"));
        assert_eq!(
            msg["list"].as_array().unwrap().len(),
            9,
            "8 items and a note"
        );
        let map = sample.call(json!({"topic": "/map"})).await;
        assert!(map.message.contains("too large"), "{}", map.message);
        let relative = sample.call(json!({"topic": "odom"})).await;
        assert!(
            relative.message.contains("absolute"),
            "{}",
            relative.message
        );
    }

    #[tokio::test]
    async fn logs_filter_by_level() {
        let (_, tools, _) = all_tools(&RosToolsConfig::default(), false);
        let logs = tool(&tools, "log_tail")
            .call(json!({"min_level": "error"}))
            .await;
        assert_eq!(logs.data["lines"][0], "[ERROR] nav: planner failed");
        let quiet = tool(&tools, "log_tail")
            .call(json!({"min_level": "fatal"}))
            .await;
        assert_eq!(quiet.data["matched"], 0);
    }

    #[tokio::test]
    async fn a_call_is_assessed_denied_or_checked_before_it_reaches_the_robot() {
        let (fake, tools, _) = all_tools(&open(), true);
        let call = tool(&tools, "service_call");
        let read = call.assess(&json!({"service": "/get_state"})).await;
        assert_eq!(read.unwrap().unwrap().risk, Risk::Observe);
        let act = call
            .assess(&json!({"service": "/reset", "request": {}}))
            .await
            .unwrap()
            .unwrap();
        assert_eq!((act.risk, act.resources.len()), (Risk::Motion, 3));
        assert!(act.reason.contains("/reset"), "{}", act.reason);
        let bad = call
            .call(json!({"service": "/fixed", "request": {"uuid": [1, 2]}}))
            .await;
        assert!(bad.message.contains("exactly 16"), "{}", bad.message);
        let ok = call.call(json!({"service": "/reset"})).await;
        assert_eq!(ok.data["response"]["success"], true);
        assert_eq!(
            fake.calls().iter().filter(|(n, _)| n == "/fixed").count(),
            0,
            "never sent"
        );
    }

    #[tokio::test]
    async fn publishing_refuses_command_topics_and_types() {
        let (fake, tools, _) = all_tools(&open(), true);
        let publish = tool(&tools, "topic_publish");
        let cmd = publish
            .call(json!({"topic": "/cmd_vel", "message": {}}))
            .await;
        assert!(cmd.message.contains("hard deny"), "{}", cmd.message);
        let twist = publish
            .call(json!({"topic": "/teleop", "type": "geometry_msgs/msg/Twist", "message": {}}))
            .await;
        assert!(
            twist.message.contains("commands the robot directly"),
            "{}",
            twist.message
        );
        let ok = publish
            .call(json!({"topic": "/status", "message": {"data": "hi"}, "count": 3}))
            .await;
        assert_eq!(ok.data["published"], 3, "{}", ok.message);
        assert_eq!(fake.published().len(), 3);
    }

    #[tokio::test]
    async fn a_parameter_is_set_in_its_own_type_and_read_back() {
        let (fake, tools, _) = all_tools(&open(), true);
        let set = tool(&tools, "param_set");
        let out = set
            .call(json!({"node": "/detector", "name": "confidence", "value": 1}))
            .await;
        assert_eq!(out.data["now"], 0.5, "{}", out.message);
        let request = fake
            .calls()
            .into_iter()
            .find(|(n, _)| n == "/detector/set_parameters")
            .unwrap()
            .1;
        assert_eq!(
            request["parameters"][0]["value"],
            json!({"type": 3, "double_value": 1.0})
        );
        let other = set
            .call(json!({"node": "/nav", "name": "x", "value": 1}))
            .await;
        assert!(other.message.contains("does not let"), "{}", other.message);
    }
}
