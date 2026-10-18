//! `collision_monitor`: Nav2's last guard before an obstacle, as NervROS may change it while it
//! runs: on or off, one polygon on or off, and the shape of a polygon the robot lets it set.
//!
//! What the monitor allows at runtime is all this is: in Nav2 1.3 a polygon's action, slowdown
//! ratio, limits and timing are read when the node starts, and a shape changes only for a polygon
//! that takes it from a topic instead of fixed points. Such a polygon detects nothing until it has
//! a shape, so a robot leaves it off until NervROS gives it one, and giving a shape turns it on.
//! Turning anything off or shrinking a shape asks the operator; turning on or growing is free.

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::{Publisher, RobotPort};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use super::Nav2Config;
use crate::integrations::{get_params, set_param};
use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};

/// Each polygon's parameters NervROS reads, in the order `Monitor::polygons` takes them.
const FIELDS: [&str; 12] = [
    "type",
    "action_type",
    "enabled",
    "points",
    "radius",
    "polygon_sub_topic",
    "slowdown_ratio",
    "linear_limit",
    "angular_limit",
    "time_before_collision",
    "visualize",
    "polygon_pub_topic",
];

const TOGGLE_WAIT: Duration = Duration::from_secs(5);
const SHOWN_WAIT: Duration = Duration::from_secs(1);

/// A shape NervROS gave a polygon.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Shape {
    /// Corners in the robot's frame, metres.
    Points(Vec<[f64; 2]>),
    /// A circle's radius, metres.
    Radius(f64),
}

impl Shape {
    /// Its area, m².
    fn area(&self) -> f64 {
        match self {
            Self::Points(p) => shoelace(p),
            Self::Radius(r) => std::f64::consts::PI * r * r,
        }
    }

    /// Scaled about the robot's centre.
    fn scaled(&self, by: f64) -> Self {
        match self {
            Self::Points(p) => Self::Points(p.iter().map(|[x, y]| [x * by, y * by]).collect()),
            Self::Radius(r) => Self::Radius(r * by),
        }
    }
}

/// A shape kept across sessions, with where it goes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct Kept {
    topic: String,
    frame: String,
    shape: Shape,
    /// Whether the polygon was on when it was last set, so it is on again after a restart.
    #[serde(default = "on")]
    on: bool,
}

fn on() -> bool {
    true
}

/// One of the monitor's polygons, as its parameters say.
#[derive(Debug, Clone, Default, PartialEq)]
struct Polygon {
    name: String,
    kind: String,
    action: String,
    enabled: Option<bool>,
    points: Option<Vec<[f64; 2]>>,
    radius: Option<f64>,
    /// Where it takes its shape from, when it takes it from a topic.
    topic: Option<String>,
    slowdown_ratio: Option<f64>,
    linear_limit: Option<f64>,
    angular_limit: Option<f64>,
    time_before_collision: Option<f64>,
    /// Where it shows its shape, when it does.
    shown: Option<String>,
}

impl Polygon {
    /// What it does, in words.
    fn does(&self) -> String {
        match self.action.as_str() {
            "stop" => "stops the robot while an obstacle is inside".to_owned(),
            "slowdown" => format!(
                "slows the robot to {:.0}% while an obstacle is inside",
                self.slowdown_ratio.map_or(50.0, |r| r * 100.0)
            ),
            "limit" => format!(
                "limits the robot to {:.2} m/s and {:.2} rad/s while an obstacle is inside",
                self.linear_limit.unwrap_or(0.5),
                self.angular_limit.unwrap_or(0.5)
            ),
            "approach" => format!(
                "slows the robot to keep {:.1} s from a collision",
                self.time_before_collision.unwrap_or(2.0)
            ),
            other => format!("does `{other}`"),
        }
    }

    /// Whether NervROS may set its shape.
    fn shapeable(&self) -> bool {
        self.topic.is_some() && self.points.is_none()
    }
}

/// The tool.
pub(super) struct Monitor {
    spec: ToolSpec,
    node: String,
    robot: Arc<dyn RobotPort>,
    file: Option<PathBuf>,
    kept: Mutex<BTreeMap<String, Kept>>,
    publishers: tokio::sync::Mutex<BTreeMap<String, Publisher>>,
    /// On or off, as NervROS last set it; none until it does.
    on: Mutex<Option<bool>>,
}

impl Monitor {
    /// The tool; the shapes it gave in earlier sessions go out again, latched.
    pub(super) fn new(
        config: &Nav2Config,
        robot: Arc<dyn RobotPort>,
        file: Option<PathBuf>,
    ) -> Arc<Self> {
        let spec = ToolSpec::new(
            "collision_monitor",
            "Nav2's collision monitor, which stops or slows the robot when an obstacle comes \
             inside one of its polygons. status: each polygon, what it does and whether it is \
             on. enable, disable: one polygon, by name (\"turn the slow polygon off\"). on, \
             off: the whole monitor, every polygon at once. shape: a polygon's \
             corners in the robot's frame ([[x, y], ...], metres), a circle's radius, or `scale` \
             of its shape, for the polygons it lets NervROS shape. Turning on and growing are \
             free; turning off and shrinking need the operator's approval.",
            json!({
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["status", "on", "off", "enable", "disable", "shape"]},
                    "polygon": {"type": "string", "description": "For enable, disable and shape: the polygon's name from status"},
                    "points": {"type": "array", "items": {"type": "array", "items": {"type": "number"}}},
                    "radius": {"type": "number", "description": "For shape: a circle's radius, m"},
                    "scale": {"type": "number", "description": "For shape: grow (above 1) or shrink (below 1) its shape"},
                },
                "required": ["action"],
            }),
            Risk::Observe,
        );
        let kept: BTreeMap<String, Kept> = file
            .as_deref()
            .map(crate::persist::read_or_default)
            .unwrap_or_default();
        let monitor = Arc::new(Self {
            spec,
            node: config.monitor_node(),
            robot,
            file,
            kept: Mutex::new(kept.clone()),
            publishers: tokio::sync::Mutex::new(BTreeMap::new()),
            on: Mutex::new(None),
        });
        if !kept.is_empty() {
            let weak = Arc::downgrade(&monitor);
            tokio::spawn(async move {
                for (name, k) in kept {
                    let Some(monitor) = weak.upgrade() else {
                        return;
                    };
                    let restored = match monitor.publish(&name, &k).await {
                        Ok(()) => monitor.set_on(&name, k.on).await,
                        Err(why) => Err(why),
                    };
                    if let Err(why) = restored {
                        tracing::warn!(polygon = %name, %why, "a kept collision polygon did not go out");
                    }
                }
            });
        }
        monitor
    }

    /// The monitor's polygons and the frame they are drawn in.
    async fn polygons(&self) -> Result<(Vec<Polygon>, String), String> {
        let robot = self.robot.as_ref();
        let top = get_params(
            robot,
            &self.node,
            &["polygons".into(), "base_frame_id".into()],
        )
        .await?;
        let names: Vec<String> = top
            .first()
            .and_then(|(v, _)| v.as_array())
            .into_iter()
            .flatten()
            .filter_map(|n| n.as_str().map(str::to_owned))
            .collect();
        let frame = top
            .get(1)
            .and_then(|(v, _)| v.as_str())
            .unwrap_or("base_footprint")
            .to_owned();
        let mut out = Vec::new();
        for name in names {
            let asked: Vec<String> = FIELDS.iter().map(|f| format!("{name}.{f}")).collect();
            let v = get_params(robot, &self.node, &asked).await?;
            let get = |i: usize| v.get(i).map_or(&Value::Null, |(value, _)| value);
            let text = |i: usize| get(i).as_str().filter(|s| !s.is_empty()).map(str::to_owned);
            let visualize = get(10).as_bool() == Some(true);
            out.push(Polygon {
                kind: text(0).unwrap_or_else(|| "polygon".into()),
                action: text(1).unwrap_or_default(),
                enabled: get(2).as_bool(),
                points: text(3).and_then(|p| parse_points(&p)),
                radius: get(4).as_f64(),
                topic: text(5).map(|t| crate::integrations::resolve(self.namespace(), &t)),
                slowdown_ratio: get(6).as_f64(),
                linear_limit: get(7).as_f64(),
                angular_limit: get(8).as_f64(),
                time_before_collision: get(9).as_f64(),
                shown: text(11)
                    .filter(|_| visualize)
                    .map(|t| crate::integrations::resolve(self.namespace(), &t)),
                name,
            });
        }
        Ok((out, frame))
    }

    fn namespace(&self) -> &str {
        self.node.rsplit_once('/').map_or("", |(ns, _)| ns)
    }

    async fn polygon(&self, args: &Value) -> Result<(Polygon, String), String> {
        let wanted = args["polygon"].as_str().unwrap_or_default();
        let (all, frame) = self.polygons().await?;
        let names: Vec<String> = all.iter().map(|p| p.name.clone()).collect();
        all.into_iter()
            .find(|p| p.name.eq_ignore_ascii_case(wanted))
            .map(|p| (p, frame))
            .ok_or_else(|| {
                format!(
                    "the collision monitor has no polygon `{wanted}`; it has {}",
                    names.join(", ")
                )
            })
    }

    /// The shape a polygon has now: what NervROS gave it, else what it shows, else its own.
    async fn shape_now(&self, p: &Polygon) -> Option<Shape> {
        if let Some(k) = crate::lock(&self.kept).get(&p.name) {
            return Some(k.shape.clone());
        }
        if let Some(shown) = &p.shown
            && let Ok(m) = self
                .robot
                .latest(shown, "geometry_msgs/msg/PolygonStamped", SHOWN_WAIT)
                .await
        {
            let points: Vec<[f64; 2]> = m["polygon"]["points"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(|q| Some([q["x"].as_f64()?, q["y"].as_f64()?]))
                .collect();
            if points.len() >= 3 {
                return Some(Shape::Points(points));
            }
        }
        p.points
            .clone()
            .map(Shape::Points)
            .or_else(|| p.radius.map(Shape::Radius))
    }

    /// The shape `args` ask for a polygon.
    async fn shape_for(&self, p: &Polygon, args: &Value) -> Result<Shape, String> {
        if !p.shapeable() {
            return Err(format!(
                "`{}` has fixed corners in the robot's configuration; only a polygon that takes its \
                 shape from a topic can change while Nav2 runs",
                p.name
            ));
        }
        if let Some(points) = args["points"].as_array() {
            let points: Option<Vec<[f64; 2]>> = points
                .iter()
                .map(|q| Some([q.get(0)?.as_f64()?, q.get(1)?.as_f64()?]))
                .collect();
            return match points {
                Some(p) if p.len() >= 3 && p.iter().flatten().all(|v| v.is_finite()) => {
                    Ok(Shape::Points(p))
                }
                _ => Err("points are three or more [x, y] corners in metres".to_owned()),
            };
        }
        if let Some(r) = args["radius"].as_f64() {
            return if r.is_finite() && r > 0.0 {
                Ok(Shape::Radius(r))
            } else {
                Err("a radius is above 0 metres".to_owned())
            };
        }
        if let Some(by) = args["scale"].as_f64() {
            if !(by.is_finite() && by > 0.0) {
                return Err("scale is above 0".to_owned());
            }
            return self
                .shape_now(p)
                .await
                .map(|s| s.scaled(by))
                .ok_or_else(|| format!("`{}` has no shape yet to scale: give its points", p.name));
        }
        Err("shape needs points, radius or scale".to_owned())
    }

    async fn publish(&self, name: &str, k: &Kept) -> Result<(), String> {
        let (ty, message) = match &k.shape {
            Shape::Points(points) => (
                "geometry_msgs/msg/PolygonStamped",
                json!({
                    "header": {"frame_id": k.frame},
                    "polygon": {"points": points.iter().map(|[x, y]| json!({"x": x, "y": y, "z": 0.0})).collect::<Vec<_>>()},
                }),
            ),
            Shape::Radius(r) => ("std_msgs/msg/Float32", json!({"data": r})),
        };
        let mut publishers = self.publishers.lock().await;
        if !publishers.contains_key(name) {
            let p = self
                .robot
                .latched_publisher(&k.topic, ty)
                .await
                .map_err(|e| format!("cannot publish on {}: {e}", k.topic))?;
            publishers.insert(name.to_owned(), p);
        }
        if let Some(p) = publishers.get(name) {
            p.send(message);
        }
        Ok(())
    }

    async fn status(&self) -> ToolOutcome {
        let (polygons, frame) = match self.polygons().await {
            Ok(p) => p,
            Err(why) => return ToolOutcome::failed(why),
        };
        let mut listed = Vec::new();
        for p in &polygons {
            let mut entry = json!({
                "name": p.name,
                "kind": p.kind,
                "does": p.does(),
                "on": p.enabled.unwrap_or(true),
                "shapeable": p.shapeable(),
            });
            if let Some(shape) = self.shape_now(p).await {
                entry["shape"] = match shape {
                    Shape::Points(points) => json!({"points": points}),
                    Shape::Radius(r) => json!({"radius_m": r}),
                };
            }
            listed.push(entry);
        }
        let on = *crate::lock(&self.on);
        ToolOutcome::ok(json!({
            "node": self.node,
            "frame": frame,
            "monitor": match on {
                Some(true) => "on",
                Some(false) => "off",
                None => "on unless turned off outside NervROS",
            },
            "polygons": listed,
        }))
    }

    async fn toggle(&self, on: bool) -> ToolOutcome {
        let service = format!("{}/toggle", self.node);
        match self
            .robot
            .call(
                &service,
                "nav2_msgs/srv/Toggle",
                json!({"enable": on}),
                TOGGLE_WAIT,
            )
            .await
        {
            Ok(r) if r["success"].as_bool() != Some(false) => {
                *crate::lock(&self.on) = Some(on);
                let message = if on {
                    "the collision monitor is on"
                } else {
                    "the collision monitor is off: nothing stops or slows the robot before an \
                     obstacle until it is on again"
                };
                ToolOutcome {
                    message: message.to_owned(),
                    ..ToolOutcome::ok(json!({"monitor": if on { "on" } else { "off" }}))
                }
            }
            Ok(r) => ToolOutcome::failed(format!(
                "the collision monitor refused: {}",
                r["message"].as_str().unwrap_or("no reason given")
            )),
            Err(e) => ToolOutcome::failed(format!(
                "the collision monitor did not answer on {service}: {e}"
            )),
        }
    }

    async fn set_on(&self, polygon: &str, on: bool) -> Result<(), String> {
        let value = json!({"type": 1, "bool_value": on});
        set_param(
            self.robot.as_ref(),
            &self.node,
            &format!("{polygon}.enabled"),
            value,
        )
        .await
    }

    fn keep(&self, all: &BTreeMap<String, Kept>) {
        let Some(file) = &self.file else {
            return;
        };
        if let Err(e) = serde_json::to_vec_pretty(all)
            .map_err(|e| e.to_string())
            .and_then(|text| crate::persist::write_atomic(file, &text).map_err(|e| e.to_string()))
        {
            tracing::warn!(error = %e, "the collision monitor's shapes were not kept");
        }
    }

    async fn enable(&self, args: &Value, on: bool) -> ToolOutcome {
        let (p, _) = match self.polygon(args).await {
            Ok(p) => p,
            Err(why) => return ToolOutcome::failed(why),
        };
        if on && p.shapeable() && self.shape_now(&p).await.is_none() {
            return ToolOutcome::failed(format!(
                "`{}` has no shape yet and would detect nothing: give it one with shape",
                p.name
            ));
        }
        if let Err(why) = self.set_on(&p.name, on).await {
            return ToolOutcome::failed(why);
        }
        let mut kept = crate::lock(&self.kept).clone();
        if let Some(k) = kept.get_mut(&p.name) {
            k.on = on;
            self.keep(&kept);
            *crate::lock(&self.kept) = kept;
        }
        ToolOutcome::ok(json!({"polygon": p.name, "on": on, "does": p.does()}))
    }

    async fn shape(&self, args: &Value) -> ToolOutcome {
        let (p, frame) = match self.polygon(args).await {
            Ok(p) => p,
            Err(why) => return ToolOutcome::failed(why),
        };
        let shape = match self.shape_for(&p, args).await {
            Ok(s) => s,
            Err(why) => return ToolOutcome::failed(why),
        };
        let Some(topic) = p.topic.clone() else {
            return ToolOutcome::failed(format!("`{}` takes no shape from a topic", p.name));
        };
        let kept = Kept {
            topic,
            frame,
            shape: shape.clone(),
            on: true,
        };
        if let Err(why) = self.publish(&p.name, &kept).await {
            return ToolOutcome::failed(why);
        }
        // A shape is given to be used: the polygon goes on with it.
        if p.enabled != Some(true)
            && let Err(why) = self.set_on(&p.name, true).await
        {
            return ToolOutcome::failed(why);
        }
        let mut all = crate::lock(&self.kept).clone();
        all.insert(p.name.clone(), kept);
        self.keep(&all);
        *crate::lock(&self.kept) = all;
        ToolOutcome::ok(json!({
            "polygon": p.name,
            "area_m2": (shape.area() * 100.0).round() / 100.0,
            "shape": shape,
            "on": true,
        }))
    }
}

#[async_trait]
impl Tool for Monitor {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let reason = match action(args) {
            Some("off") => "turns Nav2's collision monitor off: nothing stops or slows the robot \
                            before an obstacle until it is on again"
                .to_owned(),
            Some("disable") => {
                let (p, _) = match self.polygon(args).await {
                    Ok(p) => p,
                    Err(why) => return Some(Err(ToolOutcome::failed(why))),
                };
                format!(
                    "turns the collision monitor's `{}` off: it no longer {}",
                    p.name,
                    p.does().replacen("s ", " ", 1)
                )
            }
            Some("shape") => {
                let (p, _) = match self.polygon(args).await {
                    Ok(p) => p,
                    Err(why) => return Some(Err(ToolOutcome::failed(why))),
                };
                let next = match self.shape_for(&p, args).await {
                    Ok(s) => s,
                    Err(why) => return Some(Err(ToolOutcome::failed(why))),
                };
                let now = self.shape_now(&p).await;
                // Growing only makes it react sooner: free.
                // A polygon that takes its shape from a topic and has none yet detects nothing:
                // any shape it gets is a gain.
                let unshaped = p.shapeable() && now.is_none();
                if unshaped || now.as_ref().is_some_and(|n| next.area() >= n.area()) {
                    return None;
                }
                format!(
                    "makes the collision monitor's `{}` smaller ({} m² from {}): it reacts later \
                     to an obstacle",
                    p.name,
                    (next.area() * 100.0).round() / 100.0,
                    now.map_or("an unknown size".to_owned(), |n| format!(
                        "{} m²",
                        (n.area() * 100.0).round() / 100.0
                    ))
                )
            }
            _ => return None,
        };
        Some(Ok(Assessment {
            risk: Risk::WorldEdit,
            resources: Vec::new(),
            reason,
            args: None,
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match action(&args).unwrap_or("status") {
            "status" => self.status().await,
            "on" => self.toggle(true).await,
            "off" => self.toggle(false).await,
            "enable" => self.enable(&args, true).await,
            "disable" => self.enable(&args, false).await,
            "shape" => self.shape(&args).await,
            other => ToolOutcome::failed(format!(
                "action `{other}` is not one of status, on, off, enable, disable, shape"
            )),
        }
    }
}

/// The action `args` ask for: on or off with a polygon named is that polygon's enable or disable,
/// as "turn the slow polygon off" reads.
fn action(args: &Value) -> Option<&str> {
    let named = args["polygon"]
        .as_str()
        .is_some_and(|p| !p.trim().is_empty());
    match args["action"].as_str() {
        Some("on") if named => Some("enable"),
        Some("off") if named => Some("disable"),
        other => other,
    }
}

/// Nav2's `points` parameter, `[[x, y], ...]` written as text.
fn parse_points(text: &str) -> Option<Vec<[f64; 2]>> {
    let value: Value = serde_json::from_str(text).ok()?;
    let points: Option<Vec<[f64; 2]>> = value
        .as_array()?
        .iter()
        .map(|q| Some([q.get(0)?.as_f64()?, q.get(1)?.as_f64()?]))
        .collect();
    points.filter(|p| p.len() >= 3)
}

fn shoelace(p: &[[f64; 2]]) -> f64 {
    let n = p.len();
    let twice: f64 = (0..n)
        .map(|i| {
            let ([x1, y1], [x2, y2]) = (p[i], p[(i + 1) % n]);
            x1 * y2 - x2 * y1
        })
        .sum();
    twice.abs() / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use nervros_ros::fake::FakeRobot;

    fn robot() -> FakeRobot {
        let polygons = json!({"type": 9, "string_array_value": ["PolygonStop", "PolygonSlow"]});
        FakeRobot::new()
            .with_service("/collision_monitor/get_parameters", move |req| {
                let names = req["names"].as_array().cloned().unwrap_or_default();
                let values: Vec<Value> = names
                    .iter()
                    .map(|n| match n.as_str().unwrap_or_default() {
                        "polygons" => polygons.clone(),
                        "base_frame_id" => json!({"type": 4, "string_value": "base_footprint"}),
                        "PolygonStop.type" | "PolygonSlow.type" => json!({"type": 4, "string_value": "polygon"}),
                        "PolygonStop.action_type" => json!({"type": 4, "string_value": "stop"}),
                        "PolygonSlow.action_type" => json!({"type": 4, "string_value": "slowdown"}),
                        "PolygonSlow.slowdown_ratio" => json!({"type": 3, "double_value": 0.3}),
                        "PolygonStop.points" => json!({"type": 4, "string_value": "[[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]]"}),
                        "PolygonSlow.polygon_sub_topic" => json!({"type": 4, "string_value": "polygon_slow"}),
                        _ => json!({"type": 0}),
                    })
                    .collect();
                Ok(json!({"values": values}))
            })
            .with_service("/collision_monitor/set_parameters", |_| {
                Ok(json!({"results": [{"successful": true}]}))
            })
            .with_service("/collision_monitor/toggle", |_| Ok(json!({"success": true})))
            .with_service("/collision_monitor/list_parameters", |_| {
                Ok(json!({"result": {"names": [
                    "polygons", "base_frame_id", "PolygonStop.type", "PolygonSlow.type",
                    "PolygonStop.action_type", "PolygonSlow.action_type",
                    "PolygonSlow.slowdown_ratio", "PolygonStop.points",
                    "PolygonSlow.polygon_sub_topic"]}}))
            })
    }

    fn monitor(robot: FakeRobot) -> Arc<Monitor> {
        Monitor::new(&Nav2Config::default(), Arc::new(robot), None)
    }

    #[tokio::test]
    async fn status_says_what_each_polygon_does_and_which_nervros_may_shape() {
        let out = monitor(robot()).call(json!({"action": "status"})).await;
        let polygons = out.data["polygons"].as_array().unwrap();
        assert_eq!(
            polygons[0]["does"],
            "stops the robot while an obstacle is inside"
        );
        assert_eq!(polygons[0]["shapeable"], false);
        assert_eq!(
            polygons[1]["does"],
            "slows the robot to 30% while an obstacle is inside"
        );
        assert_eq!(polygons[1]["shapeable"], true);
    }

    #[tokio::test]
    async fn turning_off_and_shrinking_ask_while_turning_on_and_growing_do_not() {
        let m = monitor(robot());
        assert!(m.assess(&json!({"action": "on"})).await.is_none());
        let off = m.assess(&json!({"action": "off"})).await.unwrap().unwrap();
        assert_eq!(off.risk, Risk::WorldEdit);
        let disable = m
            .assess(&json!({"action": "disable", "polygon": "PolygonStop"}))
            .await
            .unwrap()
            .unwrap();
        assert!(
            disable.reason.contains("no longer stop"),
            "{}",
            disable.reason
        );
        // "Turn the stop polygon off" is that polygon's, not the whole monitor's.
        let named = m
            .assess(&json!({"action": "off", "polygon": "PolygonStop"}))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(named.reason, disable.reason);
        // A fixed polygon cannot be shaped at all.
        let fixed = m
            .assess(&json!({"action": "shape", "polygon": "PolygonStop", "scale": 2.0}))
            .await
            .unwrap()
            .unwrap_err();
        assert!(fixed.message.contains("fixed corners"), "{}", fixed.message);
        let square = |h: f64| json!([[h, h], [h, -h], [-h, -h], [-h, h]]);
        let first = m
            .call(json!({"action": "shape", "polygon": "PolygonSlow", "points": square(0.5)}))
            .await;
        assert_eq!(first.data["area_m2"], 1.0, "{}", first.message);
        let grow = json!({"action": "shape", "polygon": "PolygonSlow", "points": square(0.6)});
        assert!(m.assess(&grow).await.is_none());
        let shrink = json!({"action": "shape", "polygon": "PolygonSlow", "scale": 0.5});
        let asked = m.assess(&shrink).await.unwrap().unwrap();
        assert!(
            asked.reason.contains("smaller (0.25 m² from 1 m²)"),
            "{}",
            asked.reason
        );
    }
}
