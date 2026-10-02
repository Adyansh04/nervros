//! Maps ROS 2 data and agent events to Rerun, for the viewer embedded in the GUI.
//!
//! Every Rerun logging call lives here, so an upgrade touches one crate. The bridge samples the
//! robot at fixed rates and logs only what changed, which bounds what the viewer stores and keeps
//! it idle while the robot is.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use nervros_core::mission::preview::PreviewStep;
use nervros_core::profile::{LayerConfig, LookConfig, Profile, TopicRef};
use nervros_core::session::Event;
use nervros_ros::image::encode_jpeg;
use nervros_ros::{RobotPort, Transform};
use rerun::external::re_log_channel::{self, LogReceiver, LogSource};
use rerun::sink::CallbackSink;
use rerun::{AsComponents, RecordingStream, RecordingStreamBuilder};
use serde_json::{Value, json};
use tokio::sync::broadcast;
use tokio::task::JoinSet;
use tokio::time::{MissedTickBehavior, interval};

mod layers;

pub use layers::Layers;

const POSE_PERIOD: Duration = Duration::from_millis(100);
const SCAN_PERIOD: Duration = Duration::from_millis(200);
const CAMERA_PERIOD: Duration = Duration::from_millis(500);
const OBJECTS_PERIOD: Duration = Duration::from_secs(1);
const SLOW_PERIOD: Duration = Duration::from_secs(5);
const TOPIC_WAIT: Duration = Duration::from_secs(1);
const CAMERA_QUALITY: u8 = 75;
/// Where the frame goes: on its own, or in the world with the camera's pose and lens, where 3D
/// views draw it as a frustum.
const CAMERA_PATH: [&str; 2] = ["/camera", "/world/robot/camera"];
/// Where the robot's model goes, under the robot's pose.
const MODEL_PATH: &str = "world/robot/model";
/// The plan awaiting approval or running: its walks and where each step ends.
const PLAN_PATH: &str = "world/plan";
/// The viewer's name for the frame of the `world/robot` entity, which the model hangs from.
const ROBOT_FRAME: &str = "tf#/world/robot";
const REMOVED: u64 = 2;
const STALE: u64 = 1;
/// canopy's coverage grid values: seen, still to see, written off (no pose can see it).
const SEEN: i64 = 0;
const TO_SEE: i64 = 90;
const WRITTEN_OFF: i64 = 99;
/// Colours objects keep by label, so a class looks the same everywhere.
const OBJECT_PALETTE: [(u8, u8, u8); 8] = [
    (120, 160, 255),
    (255, 170, 60),
    (110, 200, 120),
    (230, 100, 140),
    (170, 130, 240),
    (80, 200, 210),
    (230, 210, 80),
    (200, 140, 100),
];

/// A recording whose data goes straight to a viewer in this process, and that viewer's input.
///
/// # Errors
///
/// The recording could not be created.
pub fn in_process() -> rerun::RecordingStreamResult<(RecordingStream, LogReceiver)> {
    let (tx, rx) = re_log_channel::log_channel(LogSource::Sdk);
    let rec = RecordingStreamBuilder::new("nervros").buffered()?;
    rec.set_sink(Box::new(CallbackSink::new(move |msgs| {
        for msg in msgs {
            // Fails only once the viewer is gone, when the app is exiting.
            let _ = tx.send(msg.clone().into());
        }
    })));
    Ok((rec, rx))
}

/// The bridge's tasks and the switches for what it draws; the tasks end when it is dropped.
pub struct Bridge {
    /// Every layer the viewer draws, which the operator can hide.
    pub layers: Arc<Layers>,
    rec: RecordingStream,
    cameras: Vec<(String, String)>,
    _tasks: JoinSet<()>,
}

impl Bridge {
    /// Puts the viewer's panes back as NervROS lays them out.
    pub fn reset_layout(&self) {
        layout(&self.rec, &self.cameras);
    }
}

/// A layer as a task draws it: its name in [`Layers`] and the entity it draws under.
#[derive(Clone)]
struct Layer {
    name: String,
    path: String,
    layers: Arc<Layers>,
}

impl Layer {
    fn shown(&self) -> bool {
        self.layers.shown(&self.name)
    }

    /// Takes everything the layer drew out of the viewer.
    fn clear(&self, rec: &RecordingStream) {
        put_static(rec, &self.path, &rerun::Clear::recursive());
        put(rec, &self.path, &rerun::Clear::recursive());
    }
}

/// Starts the bridge: robot pose, cameras, map, rooms and objects from ROS, the profile's layers,
/// and the agent's events.
pub fn spawn(
    rec: &RecordingStream,
    robot: &Arc<dyn RobotPort>,
    profile: &Profile,
    events: broadcast::Receiver<Event>,
) -> Bridge {
    put_static(rec, "world", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP());
    let mut bridge = Spawner {
        rec: rec.clone(),
        robot: Arc::clone(robot),
        layers: Arc::default(),
        tasks: JoinSet::new(),
    };
    bridge.tasks.spawn(pose(
        rec.clone(),
        Arc::clone(robot),
        profile.ros.map_frame.clone(),
        profile.ros.base_frame.clone(),
    ));
    bridge.robot_model(profile);
    let cameras = bridge.cameras(profile);
    layout(rec, &cameras);
    bridge.world(profile);
    bridge.profile_layers(profile);
    bridge
        .tasks
        .spawn(agent(rec.clone(), Arc::clone(robot), events));
    Bridge {
        layers: bridge.layers,
        rec: rec.clone(),
        cameras,
        _tasks: bridge.tasks,
    }
}

/// What `spawn` starts the tasks with.
struct Spawner {
    rec: RecordingStream,
    robot: Arc<dyn RobotPort>,
    layers: Arc<Layers>,
    tasks: JoinSet<()>,
}

impl Spawner {
    fn layer(&self, name: &str, path: &str, shown: bool) -> Layer {
        self.layers.add(name, shown);
        Layer {
            name: name.to_owned(),
            path: path.to_owned(),
            layers: Arc::clone(&self.layers),
        }
    }

    fn watch(&mut self, topic: Option<TopicRef>, period: Duration, layer: Layer, draw: Drawer) {
        if let Some(topic) = topic {
            let (rec, robot) = (self.rec.clone(), Arc::clone(&self.robot));
            self.tasks
                .spawn(sample(rec, robot, topic, period, layer, draw));
        }
    }

    fn robot_model(&mut self, profile: &Profile) {
        if let Some(urdf) = profile.viz.as_ref().and_then(|v| v.urdf.as_ref()) {
            let layer = self.layer("Robot model", MODEL_PATH, true);
            self.tasks.spawn(robot_model(
                self.rec.clone(),
                Arc::clone(&self.robot),
                profile.resolve(urdf),
                profile.ros.base_frame.clone(),
                layer,
            ));
        }
    }

    /// Every camera the profile names, each in its own view, the first in the world when the
    /// profile gives its lens; with the detector's boxes on the frames.
    /// Draws every camera and its detections; returns each camera's name and entity path.
    fn cameras(&mut self, profile: &Profile) -> Vec<(String, String)> {
        let lens = profile
            .viz
            .as_ref()
            .and_then(|v| v.camera_info.clone())
            .map(|info| (info, profile.ros.base_frame.clone()));
        let cameras = profile
            .look
            .as_ref()
            .map(LookConfig::all_cameras)
            .unwrap_or_default();
        let mut views = Vec::new();
        for (i, (name, config)) in cameras.into_iter().enumerate() {
            let path = if i == 0 {
                CAMERA_PATH[usize::from(lens.is_some())].to_owned()
            } else {
                format!("/cameras/{}", layers::slug(&name))
            };
            self.tasks.spawn(camera(
                self.rec.clone(),
                Arc::clone(&self.robot),
                config.image,
                path.clone(),
                if i == 0 { lens.clone() } else { None },
            ));
            let boxes = self.layer("Detections", &format!("{path}/detections"), true);
            self.watch(
                config.detections,
                CAMERA_PERIOD,
                boxes,
                Box::new(draw_detections),
            );
            views.push((name, path));
        }
        views
    }

    fn world(&mut self, profile: &Profile) {
        if let Some(world) = &profile.world {
            let map = self.layer("Map", "world/map", true);
            self.watch(world.map.clone(), SLOW_PERIOD, map, Box::new(draw_map));
            let coverage = self.layer("Camera coverage", "world/coverage", true);
            self.watch(
                world.coverage.clone(),
                OBJECTS_PERIOD,
                coverage,
                Box::new(draw_coverage),
            );
            let (rooms, mut drawn) = (self.layer("Rooms", "world/rooms", true), BTreeSet::new());
            self.watch(
                world.rooms.clone(),
                SLOW_PERIOD,
                rooms,
                Box::new(move |rec, path, msg| draw_rooms(rec, path, msg, &mut drawn)),
            );
            let trail = self.layer("Trail", "world/trail", true);
            self.watch(
                world.trail.clone(),
                SLOW_PERIOD,
                trail,
                Box::new(draw_trail),
            );
            let (objects, mut drawn) = (
                self.layer("Objects", "world/objects", true),
                BTreeMap::new(),
            );
            self.watch(
                world.objects.clone(),
                OBJECTS_PERIOD,
                objects,
                Box::new(move |rec, path, msg| draw_objects(rec, path, msg, &mut drawn)),
            );
            // Off by default: forty names bury the map, and hovering a box names it.
            let names = self.layer("Object names", "world/object_names", false);
            self.watch(
                world.objects.clone(),
                OBJECTS_PERIOD,
                names,
                Box::new(draw_object_names),
            );
        }
        let plan = self.layer("Plan", "world/plan", true);
        let topic = profile.viz.as_ref().and_then(|v| v.plan.clone());
        self.watch(topic, POSE_PERIOD * 5, plan, Box::new(draw_plan));
    }

    fn profile_layers(&mut self, profile: &Profile) {
        for (i, config) in profile.viz.iter().flat_map(|v| &v.layers).enumerate() {
            let path = format!("world/layers/{}", layers::slug(&config.name));
            let layer = self.layer(&config.name, &path, !config.hidden);
            if config.msg_type == "sensor_msgs/msg/LaserScan" {
                self.tasks.spawn(scan(
                    self.rec.clone(),
                    Arc::clone(&self.robot),
                    config.topic.clone(),
                    profile.ros.map_frame.clone(),
                    layer,
                    OBJECT_PALETTE[i % OBJECT_PALETTE.len()],
                ));
                continue;
            }
            let topic = TopicRef {
                topic: config.topic.clone(),
                msg_type: config.msg_type.clone(),
            };
            let draw = profile_layer(config, i, &self.robot, &profile.ros.map_frame);
            self.watch(Some(topic), OBJECTS_PERIOD, layer, draw);
        }
    }
}

/// How a `[[viz.layer]]` grid or marker array is drawn; the scan has its own task.
fn profile_layer(
    config: &LayerConfig,
    index: usize,
    robot: &Arc<dyn RobotPort>,
    map: &str,
) -> Drawer {
    if config.msg_type == "visualization_msgs/msg/MarkerArray" {
        let (robot, map, namespaces) =
            (Arc::clone(robot), map.to_owned(), config.namespaces.clone());
        let mut drawn = BTreeSet::new();
        return Box::new(move |rec, path, msg| {
            let to_map = |frame: &str| {
                if frame.is_empty() || frame == map {
                    Some(Transform::IDENTITY)
                } else {
                    robot.transform(&map, frame).ok()
                }
            };
            layers::draw_markers(rec, path, msg, &namespaces, &to_map, &mut drawn);
        });
    }
    let (r, g, b) = OBJECT_PALETTE[index % OBJECT_PALETTE.len()];
    // Above the map and coverage, one layer over the next.
    #[expect(clippy::cast_precision_loss, reason = "a handful of layers")]
    let lift = 0.02 + 0.005 * index as f32;
    Box::new(move |rec, path, msg| layers::draw_occupied(rec, path, msg, [r, g, b], lift))
}

/// A laser scan as points in the map, redrawn as scans arrive.
async fn scan(
    rec: RecordingStream,
    robot: Arc<dyn RobotPort>,
    topic: String,
    map: String,
    layer: Layer,
    (r, g, b): (u8, u8, u8),
) {
    let mut tick = interval(SCAN_PERIOD);
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let (mut last, mut drawn) = (Value::Null, false);
    loop {
        tick.tick().await;
        if !layer.shown() {
            if drawn {
                layer.clear(&rec);
                drawn = false;
            }
            continue;
        }
        let Ok(msg) = robot
            .latest_shared(&topic, "sensor_msgs/msg/LaserScan", TOPIC_WAIT)
            .await
        else {
            continue;
        };
        let stamp = msg["header"]["stamp"].clone();
        if drawn && stamp == last {
            continue;
        }
        let frame = msg["header"]["frame_id"].as_str().unwrap_or_default();
        let Ok(to_map) = robot.transform(&map, frame) else {
            continue;
        };
        put_static(
            &rec,
            &layer.path,
            &rerun::Points3D::new(layers::scan_points(&msg, &to_map))
                .with_radii([0.025])
                .with_colors([rerun::Color::from_rgb(r, g, b)]),
        );
        (last, drawn) = (stamp, true);
    }
}

/// The detector's boxes on a camera frame, labelled and coloured by label.
fn draw_detections(rec: &RecordingStream, path: &str, msg: &Value) {
    let found = nervros_core::vision::parse_detections(msg).instances;
    let (mins, sizes): (Vec<[f32; 2]>, Vec<[f32; 2]>) = found
        .iter()
        .map(|d| {
            #[expect(clippy::cast_precision_loss, reason = "pixel coordinates")]
            let (x, y, w, h) = (
                d.bbox.0 as f32,
                d.bbox.1 as f32,
                d.bbox.2 as f32,
                d.bbox.3 as f32,
            );
            ([x, y], [w, h])
        })
        .unzip();
    put(
        rec,
        path,
        &rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
            .with_labels(found.iter().map(|d| d.label.clone()))
            .with_colors(found.iter().map(|d| object_colour(&d.label, 255))),
    );
}

/// The default layout: the world large; beside it the first camera over the others and the agent's
/// last marked image, in tabs as the chat shows that image too; and the agent's log and the
/// mission's steps in a strip below, as wide as the viewer so their columns read. It is made
/// active, not only the default: the viewer would otherwise restore the last session's layout,
/// closed panes and an older version's layout included.
fn layout(rec: &RecordingStream, cameras: &[(String, String)]) {
    use rerun::blueprint::{
        Blueprint, BlueprintActivation, Horizontal, Spatial2DView, Spatial3DView,
        StateTimelineView, Tabs, TextLogView, TimeSeriesView, Vertical,
    };
    let view = |(name, path): &(String, String)| -> rerun::blueprint::ContainerLike {
        let mut title: String = name.clone();
        if let Some(first) = title.get_mut(0..1) {
            first.make_ascii_uppercase();
        }
        Spatial2DView::new(title).with_origin(path.as_str()).into()
    };
    let mut views: Vec<rerun::blueprint::ContainerLike> = match cameras {
        [] => vec![
            Spatial2DView::new("Camera")
                .with_origin(CAMERA_PATH[0])
                .into(),
        ],
        [(_, path)] => vec![
            Spatial2DView::new("Camera")
                .with_origin(path.as_str())
                .into(),
        ],
        many => many.iter().map(view).collect(),
    };
    let first = views.remove(0);
    views.push(
        Spatial2DView::new("Last look")
            .with_origin("/agent/look")
            .into(),
    );
    let top = Horizontal::new([
        Spatial3DView::new("World").with_origin("/world").into(),
        Vertical::new([first, Tabs::new(views).into()]).into(),
    ])
    .with_column_shares([3.0, 2.0]);
    let strip = Tabs::new([
        TextLogView::new("Agent").with_origin("/agent/log").into(),
        StateTimelineView::new("Mission")
            .with_origin("/mission")
            .into(),
        TimeSeriesView::new("Exploring")
            .with_origin("/mapping")
            .into(),
        TimeSeriesView::new("Plots").with_origin("/plots").into(),
    ]);
    let root = Vertical::new([top.into(), strip.into()]).with_row_shares([3.0, 1.0]);
    let activation = BlueprintActivation {
        make_active: true,
        make_default: true,
    };
    if let Err(e) = Blueprint::new(root)
        .with_auto_views(false)
        .send(rec, activation)
    {
        tracing::debug!(error = %e, "the viewer layout was not sent");
    }
}

fn put(rec: &RecordingStream, path: &str, what: &impl AsComponents) {
    if let Err(e) = rec.log(path, what) {
        tracing::debug!(error = %e, path, "rerun log failed");
    }
}

fn put_static(rec: &RecordingStream, path: &str, what: &impl AsComponents) {
    if let Err(e) = rec.log_static(path, what) {
        tracing::debug!(error = %e, path, "rerun log failed");
    }
}

#[expect(clippy::cast_possible_truncation, reason = "the viewer draws in f32")]
fn f32s<const N: usize>(v: [f64; N]) -> [f32; N] {
    v.map(|x| x as f32)
}

fn moved(a: &Transform, b: &Transform) -> bool {
    let near = |x: &[f64], y: &[f64], eps: f64| x.iter().zip(y).all(|(p, q)| (p - q).abs() < eps);
    !near(&a.translation, &b.translation, 0.005) || !near(&a.rotation, &b.rotation, 0.001)
}

async fn pose(rec: RecordingStream, robot: Arc<dyn RobotPort>, map: String, base: String) {
    put_static(
        &rec,
        "world/robot/heading",
        &rerun::Arrows3D::from_vectors([[0.5, 0.0, 0.0]]).with_radii([0.02]),
    );
    let mut tick = interval(POSE_PERIOD);
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut last: Option<Transform> = None;
    loop {
        tick.tick().await;
        let Ok(t) = robot.transform(&map, &base) else {
            continue;
        };
        if last.as_ref().is_some_and(|l| !moved(l, &t)) {
            continue;
        }
        put(
            &rec,
            "world/robot",
            &rerun::Transform3D::from_translation_rotation(
                f32s(t.translation),
                rerun::Quaternion::from_xyzw(f32s(t.rotation)),
            ),
        );
        last = Some(t);
    }
}

/// A frame of the robot's model that TF moves: the model's root under the robot's pose, or a joint
/// that is not fixed.
#[derive(Debug, PartialEq, Eq)]
struct ModelFrame {
    /// The entity its transform is logged on.
    entity: String,
    /// TF's parent and child frames.
    tf: (String, String),
    /// The viewer's parent frame; the child keeps TF's name, which is the URDF link's.
    parent: String,
}

/// Sends the robot's model from its URDF, geometry and joints at rest, all static, and returns the
/// frames TF moves.
fn load_model(rec: &RecordingStream, urdf: &Path, base: &str) -> Result<Vec<ModelFrame>, String> {
    use rerun::external::re_importer::UrdfTree;
    use rerun::external::urdf_rs::JointType;
    let tree = UrdfTree::from_file_path(urdf, Some(MODEL_PATH.into()))
        .map_err(|e| e.to_string())?
        .with_static_transform_entity(format!("{MODEL_PATH}/rest"));
    tree.emit(
        &mut |chunk| rec.send_chunk(chunk),
        &rerun::TimePoint::default(),
        true,
    )
    .map_err(|e| e.to_string())?;
    let root = ModelFrame {
        entity: format!("{MODEL_PATH}/joints/root"),
        tf: (base.to_owned(), tree.root().name.clone()),
        parent: ROBOT_FRAME.to_owned(),
    };
    let joints = tree
        .joints()
        .filter(|j| !matches!(j.joint_type, JointType::Fixed))
        .map(|j| ModelFrame {
            entity: format!("{MODEL_PATH}/joints/{}", j.name),
            tf: (j.parent.link.clone(), j.child.link.clone()),
            parent: j.parent.link.clone(),
        });
    Ok(std::iter::once(root).chain(joints).collect())
}

/// The robot's model, posed from TF as often as the robot's pose is drawn.
/// The robot's model, posed by TF; hidden, it is cleared, and loaded again when shown.
async fn robot_model(
    rec: RecordingStream,
    robot: Arc<dyn RobotPort>,
    urdf: PathBuf,
    base: String,
    layer: Layer,
) {
    let mut frames: Option<Vec<ModelFrame>> = None;
    let mut last = Vec::new();
    let mut tick = interval(POSE_PERIOD);
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    loop {
        tick.tick().await;
        if !layer.shown() {
            if frames.take().is_some() {
                layer.clear(&rec);
            }
            continue;
        }
        let model = if let Some(model) = &frames {
            model
        } else {
            let (sender, urdf, base) = (rec.clone(), urdf.clone(), base.clone());
            match tokio::task::spawn_blocking(move || load_model(&sender, &urdf, &base)).await {
                Ok(Ok(model)) => {
                    last = vec![None; model.len()];
                    frames.insert(model)
                }
                Ok(Err(e)) => {
                    tracing::warn!(error = %e, "the viewer shows no robot model");
                    return;
                }
                Err(_) => return,
            }
        };
        pose_model(&rec, robot.as_ref(), model, &mut last);
    }
}

/// Logs each model frame TF has and that moved since it was last logged.
fn pose_model(
    rec: &RecordingStream,
    robot: &dyn RobotPort,
    frames: &[ModelFrame],
    last: &mut [Option<Transform>],
) {
    for (frame, last) in frames.iter().zip(last) {
        let Ok(t) = robot.transform(&frame.tf.0, &frame.tf.1) else {
            continue;
        };
        if last.as_ref().is_some_and(|l| !moved(l, &t)) {
            continue;
        }
        // The frames' names never change: logged once, not with every pose.
        if last.is_none() {
            put_static(
                rec,
                &frame.entity,
                &rerun::Transform3D::update_fields()
                    .with_parent_frame(frame.parent.as_str())
                    .with_child_frame(frame.tf.1.as_str()),
            );
        }
        put(
            rec,
            &frame.entity,
            &rerun::Transform3D::update_fields()
                .with_translation(f32s(t.translation))
                .with_rotation(rerun::Quaternion::from_xyzw(f32s(t.rotation))),
        );
        *last = Some(t);
    }
}

async fn camera(
    rec: RecordingStream,
    robot: Arc<dyn RobotPort>,
    topic: String,
    path: String,
    lens: Option<(TopicRef, String)>,
) {
    let rx = match robot.frames(&topic) {
        Ok(rx) => rx,
        Err(e) => {
            tracing::warn!(error = %e, topic, "the viewer shows no camera");
            return;
        }
    };
    let path = path.as_str();
    let mut tick = interval(CAMERA_PERIOD);
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut last_stamp = f64::NAN;
    let mut pinhole_drawn = false;
    loop {
        tick.tick().await;
        let Some(frame) = rx.borrow().clone() else {
            continue;
        };
        if frame.stamp_s.total_cmp(&last_stamp).is_eq() {
            continue;
        }
        last_stamp = frame.stamp_s;
        if let Some((info, base)) = &lens {
            // Where the camera is on the robot, which a waist or head joint can change.
            if let Ok(t) = robot.transform(base, &frame.frame_id) {
                put(
                    &rec,
                    path,
                    &rerun::Transform3D::from_translation_rotation(
                        f32s(t.translation),
                        rerun::Quaternion::from_xyzw(f32s(t.rotation)),
                    ),
                );
            }
            if !pinhole_drawn
                && let Ok(msg) = robot.latest(&info.topic, &info.msg_type, TOPIC_WAIT).await
                && let Some(pinhole) = pinhole(&msg)
            {
                put_static(&rec, path, &pinhole);
                pinhole_drawn = true;
            }
        }
        let jpeg = tokio::task::spawn_blocking(move || {
            frame
                .to_rgb()
                .ok()
                .and_then(|img| encode_jpeg(&img, CAMERA_QUALITY).ok())
        })
        .await;
        if let Ok(Some(bytes)) = jpeg {
            put(&rec, path, &rerun::EncodedImage::from_file_contents(bytes));
        }
    }
}

/// The lens of a `sensor_msgs/msg/CameraInfo`: focal lengths and principal point from `k`.
fn pinhole(msg: &Value) -> Option<rerun::Pinhole> {
    let k: Vec<f64> = msg["k"]
        .as_array()?
        .iter()
        .filter_map(Value::as_f64)
        .collect();
    let (w, h) = (msg["width"].as_u64()?, msg["height"].as_u64()?);
    if k.len() != 9 || k[0] <= 0.0 || w == 0 || h == 0 {
        return None;
    }
    #[expect(clippy::cast_precision_loss, reason = "image sizes are far below 2^24")]
    let size = [w as f32, h as f32];
    let [fx, fy, cx, cy] = f32s([k[0], k[4], k[2], k[5]]);
    Some(
        rerun::Pinhole::from_focal_length_and_resolution([fx, fy], size)
            .with_principal_point([cx, cy])
            .with_image_plane_distance(0.4),
    )
}

/// Draws one topic's message under the layer's entity path; objects keep state between calls.
type Drawer = Box<dyn FnMut(&RecordingStream, &str, &Value) + Send>;

/// Polls a topic and redraws when its message changed, or clears the layer while it is hidden.
async fn sample(
    rec: RecordingStream,
    robot: Arc<dyn RobotPort>,
    topic: TopicRef,
    period: Duration,
    layer: Layer,
    mut draw: Drawer,
) {
    let mut tick = interval(period);
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut last: Option<Arc<Value>> = None;
    loop {
        tick.tick().await;
        if !layer.shown() {
            if last.take().is_some() {
                layer.clear(&rec);
            }
            continue;
        }
        let Ok(msg) = robot
            .latest_shared(&topic.topic, &topic.msg_type, TOPIC_WAIT)
            .await
        else {
            continue;
        };
        // The same message is the same pointer, a map included: compared whole only when a new
        // one arrived, and drawn only when it differs.
        let same = last
            .as_ref()
            .is_some_and(|l| Arc::ptr_eq(l, &msg) || **l == *msg);
        if !same {
            draw(&rec, &layer.path, &msg);
        }
        last = Some(msg);
    }
}

fn num(v: &Value) -> f64 {
    v.as_f64().unwrap_or(0.0)
}

fn xyz(v: &Value) -> [f32; 3] {
    f32s([num(&v["x"]), num(&v["y"]), num(&v["z"])])
}

/// An occupancy grid as image bytes, top row first as images are, with `N` bytes per cell.
struct Grid {
    bytes: Vec<u8>,
    size: [u32; 2],
    cell_m: f32,
    /// The lower-left corner in the map frame.
    corner: [f32; 3],
}

/// A `nav_msgs/msg/OccupancyGrid` as a [`Grid`], each cell's value mapped by `cell`.
// ponytail: ignores the origin's rotation, which map servers leave at zero; rotate the grid if a
// robot publishes a rotated one.
fn grid<const N: usize>(msg: &Value, cell: impl Fn(i64) -> [u8; N]) -> Option<Grid> {
    let info = &msg["info"];
    let res = num(&info["resolution"]);
    let width = usize::try_from(info["width"].as_u64()?).ok()?;
    let height = usize::try_from(info["height"].as_u64()?).ok()?;
    let data = msg["data"].as_array()?;
    if res <= 0.0 || width == 0 || data.len() != width * height {
        return None;
    }
    let mut bytes = Vec::with_capacity(data.len() * N);
    // ROS rows start at the origin, the bottom of the map; image rows start at the top.
    for row in data.chunks(width).rev() {
        for v in row {
            bytes.extend_from_slice(&cell(v.as_i64().unwrap_or(-1)));
        }
    }
    let origin = &info["origin"]["position"];
    let [cell_m, x, y] = f32s([res, num(&origin["x"]), num(&origin["y"])]);
    Some(Grid {
        bytes,
        size: [u32::try_from(width).ok()?, u32::try_from(height).ok()?],
        cell_m,
        corner: [x, y, 0.0],
    })
}

fn put_grid(
    rec: &RecordingStream,
    path: &str,
    grid: Grid,
    model: rerun::ColorModel,
    lift: f32,
    map: bool,
) {
    let format = rerun::components::ImageFormat::from_color_model(
        grid.size,
        model,
        rerun::ChannelDatatype::U8,
    );
    let [x, y, _] = grid.corner;
    let mut layer =
        rerun::GridMap::new(grid.bytes, format, grid.cell_m).with_translation([x, y, lift]);
    if map {
        layer = layer.with_colormap(rerun::components::Colormap::RvizMap);
    }
    // Static: only the newest grid is kept, where a new one a second would pile up.
    put_static(rec, path, &layer);
}

/// ROS occupancy (-1 unknown, 0 free, 100 occupied) in the byte values `Colormap::RvizMap` reads.
fn occupancy(v: i64) -> [u8; 1] {
    if v < 0 {
        [255]
    } else {
        [u8::try_from(v.min(100)).unwrap_or(100)]
    }
}

/// canopy's coverage values as colours over the map: seen green, still to see amber, written off
/// grey, the rest clear.
fn coverage_colour(v: i64) -> [u8; 4] {
    match v {
        SEEN => [70, 180, 90, 110],
        TO_SEE => [240, 160, 40, 150],
        WRITTEN_OFF => [130, 130, 130, 90],
        _ => [0, 0, 0, 0],
    }
}

fn draw_map(rec: &RecordingStream, path: &str, msg: &Value) {
    if let Some(g) = grid(msg, occupancy) {
        put_grid(rec, path, g, rerun::ColorModel::L, 0.0, true);
    }
}

fn draw_coverage(rec: &RecordingStream, path: &str, msg: &Value) {
    if let Some(g) = grid(msg, coverage_colour) {
        put_grid(rec, path, g, rerun::ColorModel::RGBA, 0.01, false);
    }
}

/// How much of a room the camera has seen, the mean of its floor and walls (canopy's fields).
fn seen(room: &Value) -> Option<f64> {
    let floor = room["floor_coverage"].as_f64()?;
    let faces = room["face_coverage"].as_f64().unwrap_or(floor);
    Some(f64::midpoint(floor, faces).clamp(0.0, 1.0))
}

/// Red for unseen through amber to green for seen.
fn seen_colour(seen: f64) -> rerun::Color {
    let lerp = |a: u8, b: u8, t: f64| {
        let v = f64::from(a) + (f64::from(b) - f64::from(a)) * t;
        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "v is within 0..=255"
        )]
        let byte = v.round().clamp(0.0, 255.0) as u8;
        byte
    };
    let (from, to, t) = if seen < 0.5 {
        ((220, 80, 60), (235, 170, 40), seen * 2.0)
    } else {
        ((235, 170, 40), (70, 180, 90), (seen - 0.5) * 2.0)
    };
    rerun::Color::from_rgb(
        lerp(from.0, to.0, t),
        lerp(from.1, to.1, t),
        lerp(from.2, to.2, t),
    )
}

/// The area of a polygon, by the shoelace formula.
fn area(points: &[[f32; 3]]) -> f64 {
    let n = points.len();
    let twice: f64 = (0..n)
        .map(|i| {
            let (a, b) = (points[i], points[(i + 1) % n]);
            f64::from(a[0]) * f64::from(b[1]) - f64::from(b[0]) * f64::from(a[1])
        })
        .sum();
    twice.abs() / 2.0
}

/// Room outlines from a `canopy_msgs/msg/RoomArray`, one entity each so a click in the viewer
/// names the room, labelled and coloured by how much the camera has seen of it; rooms that went
/// away are cleared. The share seen overall, by area, goes on the "Exploring" plot.
fn draw_rooms(rec: &RecordingStream, path: &str, msg: &Value, drawn: &mut BTreeSet<String>) {
    let list = msg["rooms"].as_array().map_or(&[][..], Vec::as_slice);
    let mut now = BTreeSet::new();
    let (mut seen_area, mut total_area) = (0.0, 0.0);
    for room in list {
        let (Some(id), Some(points)) = (
            room["id"].as_str().filter(|i| !i.is_empty()),
            room["outline"]["points"].as_array(),
        ) else {
            continue;
        };
        let mut strip: Vec<[f32; 3]> = points
            .iter()
            .map(|p| [xyz(p)[0], xyz(p)[1], 0.02])
            .collect();
        let room_seen = seen(room);
        if let Some(fraction) = room_seen {
            let a = area(&strip);
            seen_area += fraction * a;
            total_area += a;
        }
        if let Some(first) = strip.first().copied() {
            strip.push(first);
        }
        // canopy's names are "room C" until someone names the room; its type says more.
        let name = room["type"]
            .as_str()
            .filter(|t| !t.is_empty())
            .or_else(|| room["name"].as_str())
            .unwrap_or_default();
        let label = match room_seen {
            Some(f) => format!("{id} {name} {:.0}%", f * 100.0),
            None => format!("{id} {name}"),
        };
        put_static(
            rec,
            &format!("{path}/{id}"),
            &rerun::LineStrips3D::new([strip])
                .with_labels([label])
                .with_colors([room_seen.map_or(rerun::Color::from_rgb(200, 200, 210), seen_colour)])
                .with_radii([0.03]),
        );
        now.insert(id.to_owned());
    }
    for gone in drawn.difference(&now) {
        put_static(rec, &format!("{path}/{gone}"), &rerun::Clear::flat());
    }
    #[expect(clippy::cast_precision_loss, reason = "a handful of rooms")]
    put(
        rec,
        "mapping/rooms",
        &rerun::Scalars::single(now.len() as f64),
    );
    if total_area > 0.0 {
        put(
            rec,
            "mapping/seen %",
            &rerun::Scalars::single(100.0 * seen_area / total_area),
        );
    }
    *drawn = now;
}

/// The path the navigation stack is following, from a `nav_msgs/msg/Path`.
fn draw_plan(rec: &RecordingStream, path: &str, msg: &Value) {
    let points: Vec<[f32; 3]> = msg["poses"]
        .as_array()
        .map_or(&[][..], Vec::as_slice)
        .iter()
        .map(|p| {
            let [x, y, _] = xyz(&p["pose"]["position"]);
            [x, y, 0.05]
        })
        .collect();
    put_static(
        rec,
        path,
        &rerun::LineStrips3D::new([points])
            .with_radii([0.02])
            .with_colors([rerun::Color::from_rgb(255, 120, 200)]),
    );
}

/// Where a plan would take the robot: each walk's path, and an arrow where each step ends,
/// labelled with its step.
fn draw_preview(rec: &RecordingStream, steps: &[PreviewStep]) {
    const LIFT: f32 = 0.06;
    let colour = rerun::Color::from_rgb(255, 196, 64);
    put(rec, PLAN_PATH, &rerun::Clear::recursive());
    let paths: Vec<Vec<[f32; 3]>> = steps
        .iter()
        .filter(|s| s.path.len() > 1)
        .map(|s| {
            s.path
                .iter()
                .map(|&(x, y)| {
                    let [x, y] = f32s([x, y]);
                    [x, y, LIFT]
                })
                .collect()
        })
        .collect();
    if !paths.is_empty() {
        put(
            rec,
            &format!("{PLAN_PATH}/paths"),
            &rerun::LineStrips3D::new(paths)
                .with_radii([0.025])
                .with_colors([colour]),
        );
    }
    let ends: Vec<(&str, [f32; 3])> = steps
        .iter()
        .filter_map(|s| s.goal.map(|g| (s.id.as_str(), f32s([g.0, g.1, g.2]))))
        .collect();
    if !ends.is_empty() {
        put(
            rec,
            &format!("{PLAN_PATH}/ends"),
            &rerun::Arrows3D::from_vectors(
                ends.iter()
                    .map(|(_, [_, _, yaw])| [0.4 * yaw.cos(), 0.4 * yaw.sin(), 0.0]),
            )
            .with_origins(ends.iter().map(|(_, [x, y, _])| [*x, *y, LIFT]))
            .with_labels(ends.iter().map(|(id, _)| *id))
            .with_radii([0.03])
            .with_colors([colour]),
        );
    }
}

/// Where the robot has been, from a `nav_msgs/msg/Path`.
fn draw_trail(rec: &RecordingStream, path: &str, msg: &Value) {
    let points: Vec<[f32; 3]> = msg["poses"]
        .as_array()
        .map_or(&[][..], Vec::as_slice)
        .iter()
        .map(|p| {
            let [x, y, _] = xyz(&p["pose"]["position"]);
            [x, y, 0.03]
        })
        .collect();
    put_static(
        rec,
        path,
        &rerun::LineStrips3D::new([points])
            .with_radii([0.015])
            .with_colors([rerun::Color::from_rgb(90, 170, 255)]),
    );
}

/// A colour by label: FNV-1a over its bytes into the palette.
fn object_colour(label: &str, alpha: u8) -> rerun::Color {
    let hash = label.bytes().fold(0xcbf2_9ce4_8422_2325_u64, |h, b| {
        (h ^ u64::from(b)).wrapping_mul(0x100_0000_01b3)
    });
    let (r, g, b) = OBJECT_PALETTE[usize::try_from(hash % 8).unwrap_or(0)];
    rerun::Color::from_unmultiplied_rgba(r, g, b, alpha)
}

/// What an object is called: its name, or its label until it has one.
fn object_name(o: &Value) -> &str {
    o["name"]
        .as_str()
        .filter(|n| !n.is_empty())
        .or_else(|| o["label"].as_str())
        .unwrap_or_default()
}

/// Every object's name above it, as one batch redrawn whole.
fn draw_object_names(rec: &RecordingStream, path: &str, msg: &Value) {
    let list = msg["objects"].as_array().map_or(&[][..], Vec::as_slice);
    let (mut at, mut names) = (Vec::new(), Vec::new());
    for o in list.iter().filter(|o| o["state"].as_u64() != Some(REMOVED)) {
        let [x, y, z] = xyz(&o["pose"]["position"]);
        at.push([x, y, z + xyz(&o["size"])[2] / 2.0 + 0.05]);
        names.push(format!(
            "{} {}",
            o["id"].as_str().unwrap_or_default(),
            object_name(o)
        ));
    }
    put_static(
        rec,
        path,
        &rerun::Points3D::new(at)
            .with_labels(names)
            .with_show_labels(true)
            .with_radii([0.01]),
    );
}

/// Objects from a `canopy_msgs/msg/WorldObjectArray`, one entity each so a click in the viewer
/// names the object; stale ones are faded, and ones that went away are cleared.
fn draw_objects(
    rec: &RecordingStream,
    path: &str,
    msg: &Value,
    drawn: &mut BTreeMap<String, Value>,
) {
    let list = msg["objects"].as_array().map_or(&[][..], Vec::as_slice);
    let mut now = BTreeMap::new();
    for o in list.iter().filter(|o| o["state"].as_u64() != Some(REMOVED)) {
        let Some(id) = o["id"].as_str().filter(|i| !i.is_empty()) else {
            continue;
        };
        // Only what changes how it is drawn; one object's change redraws that object alone.
        let looks = json!([o["pose"], o["size"], o["state"], o["label"], o["name"]]);
        let unchanged = drawn.get(id) == Some(&looks);
        now.insert(id.to_owned(), looks);
        if unchanged {
            continue;
        }
        let pose = layers::pose(&o["pose"]);
        let label = o["label"].as_str().unwrap_or_default();
        let name = object_name(o);
        let alpha = if o["state"].as_u64() == Some(STALE) {
            90
        } else {
            255
        };
        put_static(
            rec,
            &format!("{path}/{id}"),
            &rerun::Boxes3D::from_centers_and_half_sizes(
                [f32s(pose.translation)],
                [xyz(&o["size"]).map(|v| v / 2.0)],
            )
            .with_quaternions([rerun::Quaternion::from_xyzw(f32s(pose.rotation))])
            .with_labels([format!("{id} {name}")])
            .with_show_labels(false)
            .with_colors([object_colour(label, alpha)]),
        );
    }
    for gone in drawn.keys().filter(|id| !now.contains_key(*id)) {
        put_static(rec, &format!("{path}/{gone}"), &rerun::Clear::flat());
    }
    #[expect(clippy::cast_precision_loss, reason = "far fewer objects than 2^52")]
    put(
        rec,
        "mapping/objects",
        &rerun::Scalars::single(now.len() as f64),
    );
    *drawn = now;
}

/// How a mission step's states look on the "Mission" timeline.
fn step_states() -> rerun::StateConfiguration {
    rerun::StateConfiguration::new()
        .with_values(["running", "success", "failure", "skipped", "idle"])
        .with_colors([
            rerun::Color::from_rgb(235, 170, 40),
            rerun::Color::from_rgb(70, 180, 90),
            rerun::Color::from_rgb(220, 80, 60),
            rerun::Color::from_rgb(140, 140, 150),
            rerun::Color::from_rgb(90, 90, 100),
        ])
}

/// The agent's replies and tool calls as a text log, and its marked images.
/// Draws one number from a topic's messages over `for_s` seconds; `source` is the topic, its
/// type and the field.
fn plot(
    rec: &RecordingStream,
    robot: &Arc<dyn RobotPort>,
    name: &str,
    (topic, ty, field): (&str, &str, &str),
    for_s: u64,
) -> impl Future<Output = ()> + Send + 'static {
    let path = format!("plots/{}", layers::slug(name));
    put_static(rec, &path, &rerun::SeriesLines::new().with_names([name]));
    let (rec, robot) = (rec.clone(), Arc::clone(robot));
    let (topic, ty, field) = (topic.to_owned(), ty.to_owned(), field.to_owned());
    let until = tokio::time::Instant::now() + Duration::from_secs(for_s);
    async move {
        let mut tick = interval(POSE_PERIOD);
        tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let mut last: Option<Arc<Value>> = None;
        while tokio::time::Instant::now() < until {
            tick.tick().await;
            let Ok(msg) = robot.latest_shared(&topic, &ty, TOPIC_WAIT).await else {
                continue;
            };
            // The newest message stays until the next arrives: draw each once.
            if !last.as_ref().is_some_and(|l| Arc::ptr_eq(l, &msg)) {
                let value = nervros_core::watch::field(&msg, &field).and_then(Value::as_f64);
                if let Some(v) = value {
                    put(&rec, &path, &rerun::Scalars::single(v));
                }
                last = Some(msg);
            }
        }
    }
}

async fn agent(
    rec: RecordingStream,
    robot: Arc<dyn RobotPort>,
    mut events: broadcast::Receiver<Event>,
) {
    use rerun::TextLogLevel as L;
    // Plots end with the bridge, like every other task it draws with.
    let mut plots = JoinSet::new();
    loop {
        let event = match events.recv().await {
            Ok(e) => e,
            Err(broadcast::error::RecvError::Lagged(_)) => continue,
            Err(broadcast::error::RecvError::Closed) => return,
        };
        let (level, text) = match &event {
            Event::Plot {
                name,
                topic,
                msg_type,
                field,
                for_s,
            } => {
                plots.spawn(plot(&rec, &robot, name, (topic, msg_type, field), *for_s));
                continue;
            }
            Event::Snapshot { jpeg, .. } => {
                let image = rerun::EncodedImage::from_file_contents(jpeg.to_vec());
                put(&rec, "agent/look", &image);
                continue;
            }
            Event::MissionStarted { .. } => {
                // A new mission's steps start from empty lanes.
                put(&rec, "mission", &rerun::Clear::recursive());
                continue;
            }
            Event::MissionPreview { steps, .. } => {
                draw_preview(&rec, steps);
                continue;
            }
            // A preview lasts while its plan is pending or running.
            Event::MissionPlanned { .. }
            | Event::MissionFinished { .. }
            | Event::ApprovalResolved {
                approved: false, ..
            } => {
                put(&rec, PLAN_PATH, &rerun::Clear::recursive());
                continue;
            }
            Event::MissionProgress {
                step, node, status, ..
            } if node.is_empty() && !step.is_empty() => {
                let path = format!("mission/{step}");
                put_static(&rec, &path, &step_states());
                put(&rec, &path, &rerun::StateChange::single(status.as_str()));
                continue;
            }
            Event::Reply { text, model, .. } => (L::INFO, format!("{model}: {text}")),
            Event::ToolStarted { tool, args, .. } => (L::DEBUG, format!("{tool} {args}")),
            Event::ToolFinished {
                tool,
                status,
                message,
                ms,
                ..
            } => {
                let level = if status.ok() { L::INFO } else { L::WARN };
                (level, format!("{tool} {status} in {ms} ms: {message}"))
            }
            Event::ApprovalRequested { tool, reason, .. } => {
                (L::WARN, format!("approval needed for {tool}: {reason}"))
            }
            Event::Halted { reason } => (L::WARN, format!("halted: {reason}")),
            Event::Stopped { ok, detail } => (
                if *ok { L::INFO } else { L::ERROR },
                format!(
                    "stop {}: {detail}",
                    if *ok { "confirmed" } else { "failed" }
                ),
            ),
            Event::Notice { text } => (L::WARN, text.clone()),
            Event::Error { text, .. } => (L::ERROR, text.clone()),
            _ => continue,
        };
        put(
            &rec,
            "agent/log",
            &rerun::TextLog::new(text).with_level(level),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_urdf_gives_the_frames_tf_moves() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("arm.urdf");
        std::fs::write(
            &urdf,
            r#"<robot name="arm">
                 <link name="base"><visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual></link>
                 <link name="upper"><visual><geometry><box size="0.1 0.1 0.4"/></geometry></visual></link>
                 <link name="tip"/>
                 <joint name="shoulder" type="revolute">
                   <parent link="base"/><child link="upper"/><axis xyz="0 1 0"/>
                   <limit lower="-1" upper="1" effort="1" velocity="1"/>
                 </joint>
                 <joint name="tool" type="fixed"><parent link="upper"/><child link="tip"/></joint>
               </robot>"#,
        )
        .unwrap();
        let (rec, _storage) = RecordingStreamBuilder::new("test").memory().unwrap();
        let frames = load_model(&rec, &urdf, "base_footprint").unwrap();
        let root = ModelFrame {
            entity: "world/robot/model/joints/root".into(),
            tf: ("base_footprint".into(), "base".into()),
            parent: ROBOT_FRAME.into(),
        };
        let shoulder = ModelFrame {
            entity: "world/robot/model/joints/shoulder".into(),
            tf: ("base".into(), "upper".into()),
            parent: "base".into(),
        };
        assert_eq!(frames, [root, shoulder], "the fixed joint stays at rest");
        assert!(load_model(&rec, &dir.path().join("missing.urdf"), "base_footprint").is_err());
    }

    #[test]
    fn small_motions_are_not_logged() {
        let a = Transform::IDENTITY;
        let mut b = a;
        b.translation[0] = 0.001;
        assert!(!moved(&a, &b));
        b.translation[0] = 0.01;
        assert!(moved(&a, &b));
    }

    #[test]
    fn grids_come_out_top_row_first() {
        let msg = serde_json::json!({
            "info": {"resolution": 0.5, "width": 2, "height": 2,
                     "origin": {"position": {"x": 1.0, "y": -1.0, "z": 0.0}}},
            "data": [0, 100, -1, 65]
        });
        let g = grid(&msg, occupancy).unwrap();
        assert_eq!(
            g.bytes,
            vec![255, 65, 0, 100],
            "the second ROS row is the image's first"
        );
        assert_eq!(g.size, [2, 2]);
        assert!(
            g.corner
                .iter()
                .zip([1.0, -1.0, 0.0])
                .all(|(a, b)| (a - b).abs() < f32::EPSILON)
        );
        let short =
            serde_json::json!({"info": {"resolution": 0.5, "width": 2, "height": 2}, "data": [0]});
        assert!(
            grid(&short, occupancy).is_none(),
            "a grid with missing cells is not drawn"
        );
    }

    #[test]
    fn coverage_is_coloured_by_what_the_camera_saw() {
        assert_eq!(coverage_colour(SEEN)[1], 180);
        assert_eq!(coverage_colour(TO_SEE)[0], 240);
        assert_eq!(coverage_colour(-1)[3], 0, "outside the rooms stays clear");
    }

    #[test]
    fn a_room_is_as_seen_as_the_mean_of_floor_and_walls() {
        let room = serde_json::json!({"floor_coverage": 0.9, "face_coverage": 0.5});
        assert!((seen(&room).unwrap() - 0.7).abs() < 1e-9);
        assert!(seen(&serde_json::json!({})).is_none());
        assert_eq!(seen_colour(0.0), rerun::Color::from_rgb(220, 80, 60));
        assert_eq!(seen_colour(1.0), rerun::Color::from_rgb(70, 180, 90));
        let square = [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 3.0, 0.0],
            [0.0, 3.0, 0.0],
        ];
        assert!((area(&square) - 6.0).abs() < 1e-9);
    }

    #[test]
    fn a_camera_info_gives_the_lens() {
        let info = serde_json::json!({"width": 848, "height": 480,
            "k": [600.0, 0.0, 424.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]});
        assert!(pinhole(&info).is_some());
        let blank = serde_json::json!({"width": 848, "height": 480, "k": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]});
        assert!(
            pinhole(&blank).is_none(),
            "an uncalibrated camera draws no frustum"
        );
    }

    #[test]
    fn objects_keep_their_colour_by_label() {
        assert_eq!(object_colour("mug", 255), object_colour("mug", 255));
    }
}
