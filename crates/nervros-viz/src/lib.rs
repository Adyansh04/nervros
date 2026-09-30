//! Maps ROS 2 data and agent events to Rerun, for the viewer embedded in the GUI.
//!
//! Every Rerun logging call lives here, so an upgrade touches one crate. The bridge samples the
//! robot at fixed rates and logs only what changed, which bounds what the viewer stores and keeps
//! it idle while the robot is.

use std::collections::BTreeSet;
use std::sync::Arc;
use std::time::Duration;

use nervros_core::profile::{Profile, TopicRef};
use nervros_core::session::Event;
use nervros_ros::image::encode_jpeg;
use nervros_ros::{RobotPort, Transform};
use rerun::external::re_log_channel::{self, LogReceiver, LogSource};
use rerun::sink::CallbackSink;
use rerun::{AsComponents, RecordingStream, RecordingStreamBuilder};
use serde_json::Value;
use tokio::sync::broadcast;
use tokio::task::JoinSet;
use tokio::time::{MissedTickBehavior, interval};

const POSE_PERIOD: Duration = Duration::from_millis(100);
const CAMERA_PERIOD: Duration = Duration::from_millis(500);
const OBJECTS_PERIOD: Duration = Duration::from_secs(1);
const SLOW_PERIOD: Duration = Duration::from_secs(5);
const TOPIC_WAIT: Duration = Duration::from_secs(1);
const CAMERA_QUALITY: u8 = 75;
/// Where the frame goes: on its own, or in the world with the camera's pose and lens, where 3D
/// views draw it as a frustum.
const CAMERA_PATH: [&str; 2] = ["/camera", "/world/robot/camera"];
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

/// Starts the bridge: robot pose, camera, map, rooms and objects from ROS, and the agent's events.
/// The tasks end when the set is dropped.
pub fn spawn(
    rec: &RecordingStream,
    robot: &Arc<dyn RobotPort>,
    profile: &Profile,
    events: broadcast::Receiver<Event>,
) -> JoinSet<()> {
    put_static(rec, "world", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP());
    layout(
        rec,
        profile
            .viz
            .as_ref()
            .is_some_and(|v| v.camera_info.is_some()),
    );
    let mut tasks = JoinSet::new();
    tasks.spawn(pose(
        rec.clone(),
        Arc::clone(robot),
        profile.ros.map_frame.clone(),
        profile.ros.base_frame.clone(),
    ));
    let camera_info = profile.viz.as_ref().and_then(|v| v.camera_info.clone());
    if let Some(look) = &profile.look {
        tasks.spawn(camera(
            rec.clone(),
            Arc::clone(robot),
            look.image.clone(),
            camera_info
                .clone()
                .map(|info| (info, profile.ros.base_frame.clone())),
        ));
    }
    if let Some(world) = &profile.world {
        let mut watch = |topic: &Option<TopicRef>, period, draw: Drawer| {
            if let Some(topic) = topic {
                tasks.spawn(sample(
                    rec.clone(),
                    Arc::clone(robot),
                    topic.clone(),
                    period,
                    draw,
                ));
            }
        };
        watch(&world.map, SLOW_PERIOD, Box::new(draw_map));
        watch(&world.coverage, OBJECTS_PERIOD, Box::new(draw_coverage));
        let mut rooms_drawn = BTreeSet::new();
        watch(
            &world.rooms,
            SLOW_PERIOD,
            Box::new(move |rec, msg| draw_rooms(rec, msg, &mut rooms_drawn)),
        );
        watch(&world.trail, SLOW_PERIOD, Box::new(draw_trail));
        let mut drawn = BTreeSet::new();
        watch(
            &world.objects,
            OBJECTS_PERIOD,
            Box::new(move |rec, msg| draw_objects(rec, msg, &mut drawn)),
        );
    }
    if let Some(plan) = profile.viz.as_ref().and_then(|v| v.plan.clone()) {
        tasks.spawn(sample(
            rec.clone(),
            Arc::clone(robot),
            plan,
            POSE_PERIOD * 5,
            Box::new(draw_plan),
        ));
    }
    tasks.spawn(agent(rec.clone(), events));
    tasks
}

/// The default layout: the world large on the left; the camera, the agent's last marked image and
/// its log on the right. Only a default, so a layout the operator arranges is kept.
fn layout(rec: &RecordingStream, camera_in_world: bool) {
    use rerun::blueprint::{
        Blueprint, BlueprintActivation, Horizontal, Spatial2DView, Spatial3DView,
        StateTimelineView, Tabs, TextLogView, TimeSeriesView, Vertical,
    };
    let side = Vertical::new([
        Spatial2DView::new("Camera")
            .with_origin(CAMERA_PATH[usize::from(camera_in_world)])
            .into(),
        Spatial2DView::new("Last look")
            .with_origin("/agent/look")
            .into(),
        Tabs::new([
            TextLogView::new("Agent").with_origin("/agent/log").into(),
            StateTimelineView::new("Mission")
                .with_origin("/mission")
                .into(),
            TimeSeriesView::new("Exploring")
                .with_origin("/mapping")
                .into(),
        ])
        .into(),
    ]);
    let root = Horizontal::new([
        Spatial3DView::new("World").with_origin("/world").into(),
        side.into(),
    ])
    .with_column_shares([2.0, 1.0]);
    let activation = BlueprintActivation {
        make_active: false,
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

async fn camera(
    rec: RecordingStream,
    robot: Arc<dyn RobotPort>,
    topic: String,
    lens: Option<(TopicRef, String)>,
) {
    let rx = match robot.frames(&topic) {
        Ok(rx) => rx,
        Err(e) => {
            tracing::warn!(error = %e, topic, "the viewer shows no camera");
            return;
        }
    };
    let path = CAMERA_PATH[usize::from(lens.is_some())];
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

/// Draws one topic's message; objects keep state between calls.
type Drawer = Box<dyn FnMut(&RecordingStream, &Value) + Send>;

/// Polls a topic and redraws when its message changed.
async fn sample(
    rec: RecordingStream,
    robot: Arc<dyn RobotPort>,
    topic: TopicRef,
    period: Duration,
    mut draw: Drawer,
) {
    let mut tick = interval(period);
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut last = Value::Null;
    loop {
        tick.tick().await;
        let Ok(msg) = robot
            .latest(&topic.topic, &topic.msg_type, TOPIC_WAIT)
            .await
        else {
            continue;
        };
        if msg != last {
            draw(&rec, &msg);
            last = msg;
        }
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

fn draw_map(rec: &RecordingStream, msg: &Value) {
    if let Some(g) = grid(msg, occupancy) {
        put_grid(rec, "world/map", g, rerun::ColorModel::L, 0.0, true);
    }
}

fn draw_coverage(rec: &RecordingStream, msg: &Value) {
    if let Some(g) = grid(msg, coverage_colour) {
        put_grid(
            rec,
            "world/coverage",
            g,
            rerun::ColorModel::RGBA,
            0.01,
            false,
        );
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
fn draw_rooms(rec: &RecordingStream, msg: &Value, drawn: &mut BTreeSet<String>) {
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
        let name = room["name"].as_str().unwrap_or_default();
        let label = match room_seen {
            Some(f) => format!("{id} {name} {:.0}%", f * 100.0),
            None => format!("{id} {name}"),
        };
        put_static(
            rec,
            &format!("world/rooms/{id}"),
            &rerun::LineStrips3D::new([strip])
                .with_labels([label])
                .with_colors([room_seen.map_or(rerun::Color::from_rgb(200, 200, 210), seen_colour)])
                .with_radii([0.03]),
        );
        now.insert(id.to_owned());
    }
    for gone in drawn.difference(&now) {
        put_static(rec, &format!("world/rooms/{gone}"), &rerun::Clear::flat());
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
fn draw_plan(rec: &RecordingStream, msg: &Value) {
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
        "world/plan",
        &rerun::LineStrips3D::new([points])
            .with_radii([0.02])
            .with_colors([rerun::Color::from_rgb(255, 120, 200)]),
    );
}

/// Where the robot has been, from a `nav_msgs/msg/Path`.
fn draw_trail(rec: &RecordingStream, msg: &Value) {
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
        "world/trail",
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

/// Objects from a `canopy_msgs/msg/WorldObjectArray`, one entity each so a click in the viewer
/// names the object; stale ones are faded, and ones that went away are cleared.
fn draw_objects(rec: &RecordingStream, msg: &Value, drawn: &mut BTreeSet<String>) {
    let list = msg["objects"].as_array().map_or(&[][..], Vec::as_slice);
    let mut now = BTreeSet::new();
    for o in list.iter().filter(|o| o["state"].as_u64() != Some(REMOVED)) {
        let Some(id) = o["id"].as_str().filter(|i| !i.is_empty()) else {
            continue;
        };
        let q = &o["pose"]["orientation"];
        let w = q["w"].as_f64().unwrap_or(1.0);
        let label = o["label"].as_str().unwrap_or_default();
        let name = o["name"]
            .as_str()
            .filter(|n| !n.is_empty())
            .unwrap_or(label);
        let alpha = if o["state"].as_u64() == Some(STALE) {
            90
        } else {
            255
        };
        put_static(
            rec,
            &format!("world/objects/{id}"),
            &rerun::Boxes3D::from_centers_and_half_sizes(
                [xyz(&o["pose"]["position"])],
                [xyz(&o["size"]).map(|v| v / 2.0)],
            )
            .with_quaternions([rerun::Quaternion::from_xyzw(f32s([
                num(&q["x"]),
                num(&q["y"]),
                num(&q["z"]),
                w,
            ]))])
            .with_labels([format!("{id} {name}")])
            .with_colors([object_colour(label, alpha)]),
        );
        now.insert(id.to_owned());
    }
    for gone in drawn.difference(&now) {
        put_static(rec, &format!("world/objects/{gone}"), &rerun::Clear::flat());
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
async fn agent(rec: RecordingStream, mut events: broadcast::Receiver<Event>) {
    use rerun::TextLogLevel as L;
    loop {
        let event = match events.recv().await {
            Ok(e) => e,
            Err(broadcast::error::RecvError::Lagged(_)) => continue,
            Err(broadcast::error::RecvError::Closed) => return,
        };
        let (level, text) = match &event {
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
                let level = if *status == "succeeded" {
                    L::INFO
                } else {
                    L::WARN
                };
                (level, format!("{tool} {status} in {ms} ms: {message}"))
            }
            Event::ApprovalRequested { tool, reason, .. } => {
                (L::WARN, format!("approval needed for {tool}: {reason}"))
            }
            Event::Halted { reason } => (L::WARN, format!("halted: {reason}")),
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
