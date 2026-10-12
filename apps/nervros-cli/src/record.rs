//! `nervros-cli record`: the live robot written as a scripted robot file for `eval --robot`, so a
//! suite runs against what this robot's graph was, without the robot.
//!
//! It keeps the graph, the nodes each component container hosts, every node's topics, services
//! and parameters, the rates and messages of the topics the profile names and of the sensors', the
//! robot's place on the map, and the mission executor's catalog, with answers that accept every
//! plan and finish every mission: nothing moves when a recording runs one.

use std::collections::BTreeSet;
use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context as _, Result};
use futures::StreamExt as _;
use nervros_core::profile::Profile;
use nervros_ros::{GraphDetail, RobotPort};
use serde_json::{Map, Value, json};

/// Types too large to keep a message of: images, clouds, grids and masks.
const BULK: [&str; 7] = [
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/PointCloud2",
    "nav_msgs/msg/OccupancyGrid",
    "octomap_msgs/msg/Octomap",
    "canopy_msgs/msg/InstanceMaskArray",
    "tf2_msgs/msg/TFMessage",
];
/// Sensor types whose rates a suite asks about.
const SENSORS: [&str; 6] = [
    "sensor_msgs/msg/LaserScan",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/PointCloud2",
    "nav_msgs/msg/Odometry",
    "sensor_msgs/msg/CameraInfo",
];
/// A kept message larger than this is left out; a recording is read by people too.
const MESSAGE_MAX: usize = 64 * 1024;
const RATE_WINDOW: Duration = Duration::from_secs(2);
const CALL: Duration = Duration::from_secs(3);
/// Nodes asked at once for their parameters.
const AT_ONCE: usize = 8;

/// Records the robot `profile_path` names into `out`.
///
/// # Errors
///
/// The profile cannot be loaded, the robot cannot be reached, or the file cannot be written.
pub(crate) async fn record(profile_path: &Path, out: &Path) -> Result<()> {
    let profile = Profile::load(profile_path).context("loading the profile")?;
    let robot = nervros_core::app::connect(&profile)?;
    // Discovery needs a moment after the node starts.
    tokio::time::sleep(Duration::from_secs(3)).await;
    let graph = robot.graph_detail().await.context("reading the graph")?;
    let mut doc = Map::new();
    let first = |items: &[(String, Vec<String>)]| -> Map<String, Value> {
        items
            .iter()
            .filter_map(|(n, t)| t.first().map(|t| (n.clone(), json!(t))))
            .collect()
    };
    doc.insert(
        "graph".into(),
        json!({"topics": first(&graph.topics), "services": first(&graph.services), "nodes": graph.nodes}),
    );
    eprintln!(
        "{} topics, {} services, {} nodes",
        graph.topics.len(),
        graph.services.len(),
        graph.nodes.len()
    );
    doc.insert("containers".into(), containers(&robot, &graph).await);
    doc.insert("nodes".into(), nodes(&robot, &graph).await);
    doc.insert("params".into(), params(&robot, &graph).await);
    let named = profile_topics(&profile);
    doc.insert("rates".into(), rates(&robot, &graph, &named).await);
    doc.insert("topics".into(), messages(&robot, &graph, &named).await);
    let (services, actions) = executor(&robot, &profile).await;
    doc.insert("services".into(), services);
    doc.insert("actions".into(), actions);
    if let Ok(pose) = robot.transform(&profile.ros.map_frame, &profile.ros.base_frame) {
        let (x, y, yaw) = pose.planar();
        doc.insert(
            "transforms".into(),
            json!([{"parent": profile.ros.map_frame, "child": profile.ros.base_frame, "x": x, "y": y, "yaw": yaw}]),
        );
    }
    let text = toml::to_string_pretty(&without_nulls(Value::Object(doc)))
        .context("writing the recording as TOML")?;
    let header = format!(
        "# Recorded by `nervros-cli record` from {} at {}: run a suite on it with\n\
         # `nervros-cli eval <suite> --robot <this file>`. Nothing moves; every plan is accepted.\n\n",
        profile_path.display(),
        humantime::format_rfc3339_seconds(std::time::SystemTime::now())
    );
    std::fs::write(out, header + &text).with_context(|| format!("writing {}", out.display()))?;
    eprintln!("recorded into {}", out.display());
    Ok(())
}

/// TOML has no null: a field left out reads as its default.
fn without_nulls(value: Value) -> Value {
    match value {
        Value::Object(fields) => Value::Object(
            fields
                .into_iter()
                .filter(|(_, v)| !v.is_null())
                .map(|(k, v)| (k, without_nulls(v)))
                .collect(),
        ),
        Value::Array(items) => Value::Array(
            items
                .into_iter()
                .filter(|v| !v.is_null())
                .map(without_nulls)
                .collect(),
        ),
        other => other,
    }
}

fn has_service(graph: &GraphDetail, name: &str) -> bool {
    graph.services.iter().any(|(n, _)| n == name)
}

async fn containers(robot: &Arc<dyn RobotPort>, graph: &GraphDetail) -> Value {
    let mut out = Map::new();
    for node in &graph.nodes {
        let service = format!("{node}/_container/list_nodes");
        if !has_service(graph, &service) {
            continue;
        }
        let ty = "composition_interfaces/srv/ListNodes";
        if let Ok(hosted) = robot.call(&service, ty, json!({}), CALL).await {
            out.insert(node.clone(), hosted["full_node_names"].clone());
        }
    }
    Value::Object(out)
}

async fn nodes(robot: &Arc<dyn RobotPort>, graph: &GraphDetail) -> Value {
    let mut out = Map::new();
    let pairs = |items: &[(String, Vec<String>)]| -> Value {
        Value::Object(
            items
                .iter()
                .filter_map(|(n, t)| t.first().map(|t| (n.clone(), json!(t))))
                .collect(),
        )
    };
    for node in &graph.nodes {
        if let Ok(e) = robot.node_entities(node).await {
            out.insert(
                node.clone(),
                json!({
                    "publishers": pairs(&e.publishers),
                    "subscribers": pairs(&e.subscribers),
                    "services": pairs(&e.services),
                    "clients": pairs(&e.clients),
                }),
            );
        }
    }
    Value::Object(out)
}

/// A `rcl_interfaces/msg/ParameterValue` as plain JSON.
fn plain(v: &Value) -> Value {
    let field = match v["type"].as_u64() {
        Some(1) => "bool_value",
        Some(2) => "integer_value",
        Some(3) => "double_value",
        Some(4) => "string_value",
        Some(5) => "byte_array_value",
        Some(6) => "bool_array_value",
        Some(7) => "integer_array_value",
        Some(8) => "double_array_value",
        Some(9) => "string_array_value",
        _ => return Value::Null,
    };
    v[field].clone()
}

/// One node's parameters with their descriptions; none when it does not answer.
async fn node_params(robot: Arc<dyn RobotPort>, node: String) -> Option<(String, Value)> {
    let call = |service: &str, ty: &str, request: Value| {
        let robot = Arc::clone(&robot);
        let name = format!("{node}/{service}");
        let ty = ty.to_owned();
        async move { robot.call(&name, &ty, request, CALL).await.ok() }
    };
    let listed = call(
        "list_parameters",
        "rcl_interfaces/srv/ListParameters",
        json!({"prefixes": [], "depth": 0}),
    )
    .await?;
    let names: Vec<String> = listed["result"]["names"]
        .as_array()?
        .iter()
        .filter_map(|n| n.as_str().map(str::to_owned))
        .collect();
    let values = call(
        "get_parameters",
        "rcl_interfaces/srv/GetParameters",
        json!({"names": names}),
    )
    .await?;
    let described = call(
        "describe_parameters",
        "rcl_interfaces/srv/DescribeParameters",
        json!({"names": names}),
    )
    .await;
    let values = values["values"].as_array().cloned().unwrap_or_default();
    let descriptors = described
        .and_then(|d| d["descriptors"].as_array().cloned())
        .unwrap_or_default();
    let mut out = Map::new();
    for (i, name) in names.iter().enumerate() {
        let value = values.get(i).map(plain).unwrap_or_default();
        if value.is_null() {
            continue;
        }
        let d = descriptors.get(i).unwrap_or(&Value::Null);
        let description = d["description"].as_str().unwrap_or_default();
        let read_only = d["read_only"].as_bool().unwrap_or(false);
        out.insert(
            name.clone(),
            if description.is_empty() && !read_only {
                value
            } else {
                json!({"value": value, "description": description, "read_only": read_only})
            },
        );
    }
    Some((node, Value::Object(out)))
}

async fn params(robot: &Arc<dyn RobotPort>, graph: &GraphDetail) -> Value {
    let askable: Vec<String> = graph
        .nodes
        .iter()
        .filter(|n| has_service(graph, &format!("{n}/list_parameters")))
        .cloned()
        .collect();
    eprintln!("asking {} nodes for their parameters", askable.len());
    let answered: Vec<Option<(String, Value)>> = futures::stream::iter(askable)
        .map(|n| node_params(Arc::clone(robot), n))
        .buffer_unordered(AT_ONCE)
        .collect()
        .await;
    Value::Object(answered.into_iter().flatten().collect())
}

/// Every topic the profile names.
fn profile_topics(profile: &Profile) -> BTreeSet<String> {
    let mut named = BTreeSet::new();
    if let Some(look) = &profile.look {
        named.insert(look.image.clone());
        named.insert(look.detections.topic.clone());
        for camera in look.cameras.values() {
            named.insert(camera.image.clone());
            if let Some(d) = &camera.detections {
                named.insert(d.topic.clone());
            }
        }
    }
    if let Some(world) = &profile.world {
        for t in [
            &world.map,
            &world.rooms,
            &world.objects,
            &world.coverage,
            &world.trail,
        ]
        .into_iter()
        .flatten()
        {
            named.insert(t.topic.clone());
        }
    }
    if let Some(mission) = &profile.mission {
        named.insert(mission.state.clone());
    }
    for tool in &profile.tools {
        if matches!(tool.kind, nervros_core::tools::ToolKind::Topic) {
            named.insert(tool.ros_name.clone());
        }
    }
    named
}

fn type_of<'a>(graph: &'a GraphDetail, topic: &str) -> Option<&'a str> {
    graph
        .topics
        .iter()
        .find(|(n, _)| n == topic)
        .and_then(|(_, t)| t.first())
        .map(String::as_str)
}

async fn rates(robot: &Arc<dyn RobotPort>, graph: &GraphDetail, named: &BTreeSet<String>) -> Value {
    let wanted: Vec<(String, String)> = graph
        .topics
        .iter()
        .filter_map(|(n, t)| t.first().map(|t| (n.clone(), t.clone())))
        .filter(|(n, t)| named.contains(n) || SENSORS.contains(&t.as_str()))
        .collect();
    eprintln!(
        "timing {} topics for {} s",
        wanted.len(),
        RATE_WINDOW.as_secs()
    );
    let timed = futures::future::join_all(wanted.iter().map(|(n, t)| async move {
        let sizes = robot.sample_sizes(n, t, RATE_WINDOW, 10_000).await.ok()?;
        if sizes.is_empty() {
            return None;
        }
        let bytes = sizes.iter().map(|(_, b)| b).sum::<usize>() / sizes.len();
        #[expect(clippy::cast_precision_loss, reason = "a message count per window")]
        let hz = sizes.len() as f64 / RATE_WINDOW.as_secs_f64();
        Some((
            n.clone(),
            json!({"hz": (hz * 10.0).round() / 10.0, "bytes": bytes}),
        ))
    }))
    .await;
    Value::Object(timed.into_iter().flatten().collect())
}

async fn messages(
    robot: &Arc<dyn RobotPort>,
    graph: &GraphDetail,
    named: &BTreeSet<String>,
) -> Value {
    let mut out = Map::new();
    for topic in named {
        let Some(ty) = type_of(graph, topic) else {
            continue;
        };
        if BULK.contains(&ty) {
            continue;
        }
        if let Ok(message) = robot.latest(topic, ty, Duration::from_secs(2)).await
            && serde_json::to_string(&message).is_ok_and(|s| s.len() <= MESSAGE_MAX)
        {
            out.insert(topic.clone(), message);
        }
    }
    Value::Object(out)
}

/// The mission executor's services and action: its catalog as it is, and answers that accept every
/// plan and finish every mission.
async fn executor(robot: &Arc<dyn RobotPort>, profile: &Profile) -> (Value, Value) {
    let mut services = Map::new();
    let mut actions = Map::new();
    let Some(m) = &profile.mission else {
        return (Value::Object(services), Value::Object(actions));
    };
    if let Ok(catalog) = robot
        .call(
            &m.catalog,
            "nervros_interfaces/srv/GetCatalog",
            json!({}),
            CALL,
        )
        .await
    {
        services.insert(m.catalog.clone(), catalog);
    }
    services.insert(
        m.validate.clone(),
        json!({"ok": true, "diagnostics_json": "[]", "worst_case_duration_s": 60.0}),
    );
    services.insert(
        m.stop.clone(),
        json!({"ok": true, "message": "", "state_after": "holding posture"}),
    );
    if let Some(preview) = &m.preview {
        services.insert(
            preview.clone(),
            json!({"ok": true, "message": "", "steps": []}),
        );
    }
    if let Some(pause) = &m.pause {
        services.insert(
            pause.clone(),
            json!({"ok": false, "message": "no mission is running"}),
        );
    }
    if let Some(teleop) = &m.teleop {
        services.insert(teleop.clone(), json!({"ok": true, "message": ""}));
    }
    actions.insert(
        m.execute.clone(),
        json!({
            "feedback": [{"elapsed_s": 1.0, "running_nodes": [], "events": []}],
            "status": "succeeded",
            "result": {"outcome": 0, "failed_node": "", "failed_step_id": "", "failure_reason": "", "diagnostics_json": "[]", "elapsed_s": 1.0},
        }),
    );
    (Value::Object(services), Value::Object(actions))
}
