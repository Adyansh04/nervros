//! Maps ROS 2 data and agent events to Rerun, for the viewer embedded in the GUI.
//!
//! Every Rerun logging call lives here, so an upgrade touches one crate. The bridge samples the
//! robot at fixed rates and logs only what changed, which bounds what the viewer stores and keeps
//! it idle while the robot is.

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
/// Occupancy at or above this is a wall, as in `nav2_map_server`'s default.
const OCCUPIED: i64 = 65;
const REMOVED: u64 = 2;
const STALE: u64 = 1;

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
    let mut tasks = JoinSet::new();
    tasks.spawn(pose(
        rec.clone(),
        Arc::clone(robot),
        profile.ros.map_frame.clone(),
        profile.ros.base_frame.clone(),
    ));
    if let Some(look) = &profile.look {
        tasks.spawn(camera(rec.clone(), Arc::clone(robot), look.image.clone()));
    }
    if let Some(world) = &profile.world {
        let topics: [(Option<TopicRef>, Duration, Drawer); 3] = [
            (world.map.clone(), SLOW_PERIOD, draw_map),
            (world.rooms.clone(), SLOW_PERIOD, draw_rooms),
            (world.objects.clone(), OBJECTS_PERIOD, draw_objects),
        ];
        for (topic, period, draw) in topics {
            if let Some(topic) = topic {
                tasks.spawn(sample(rec.clone(), Arc::clone(robot), topic, period, draw));
            }
        }
    }
    tasks.spawn(agent(rec.clone(), events));
    tasks
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

async fn camera(rec: RecordingStream, robot: Arc<dyn RobotPort>, topic: String) {
    let rx = match robot.frames(&topic) {
        Ok(rx) => rx,
        Err(e) => {
            tracing::warn!(error = %e, topic, "the viewer shows no camera");
            return;
        }
    };
    let mut tick = interval(CAMERA_PERIOD);
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut last_stamp = f64::NAN;
    loop {
        tick.tick().await;
        let Some(frame) = rx.borrow().clone() else {
            continue;
        };
        if frame.stamp_s.total_cmp(&last_stamp).is_eq() {
            continue;
        }
        last_stamp = frame.stamp_s;
        let jpeg = tokio::task::spawn_blocking(move || {
            frame
                .to_rgb()
                .ok()
                .and_then(|img| encode_jpeg(&img, CAMERA_QUALITY).ok())
        })
        .await;
        if let Ok(Some(bytes)) = jpeg {
            put(
                &rec,
                "camera",
                &rerun::EncodedImage::from_file_contents(bytes),
            );
        }
    }
}

type Drawer = fn(&RecordingStream, &Value);

/// Polls a topic and redraws when its message changed.
async fn sample(
    rec: RecordingStream,
    robot: Arc<dyn RobotPort>,
    topic: TopicRef,
    period: Duration,
    draw: Drawer,
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

/// The centres of a `nav_msgs/msg/OccupancyGrid`'s occupied cells, and the cell size.
// ponytail: ignores the origin's rotation, which map servers leave at zero; rotate the cells if
// a robot publishes a rotated grid.
fn walls(msg: &Value) -> (Vec<[f32; 3]>, f32) {
    let info = &msg["info"];
    let res = num(&info["resolution"]);
    let width = info["width"].as_u64().unwrap_or(0);
    let cells = msg["data"].as_array().map_or(&[][..], Vec::as_slice);
    if res <= 0.0 || width == 0 {
        return (Vec::new(), 0.0);
    }
    let origin = &info["origin"]["position"];
    let (ox, oy) = (num(&origin["x"]), num(&origin["y"]));
    #[expect(
        clippy::cast_precision_loss,
        reason = "grid indices are far below 2^52"
    )]
    let points = cells
        .iter()
        .zip(0u64..)
        .filter(|(v, _)| v.as_i64().is_some_and(|o| o >= OCCUPIED))
        .map(|(_, i)| {
            let (col, row) = ((i % width) as f64, (i / width) as f64);
            f32s([ox + (col + 0.5) * res, oy + (row + 0.5) * res, 0.0])
        })
        .collect();
    let [size] = f32s([res]);
    (points, size)
}

fn draw_map(rec: &RecordingStream, msg: &Value) {
    let (points, size) = walls(msg);
    put(
        rec,
        "world/map",
        &rerun::Points3D::new(points)
            .with_radii([size / 2.0])
            .with_colors([rerun::Color::from_rgb(140, 140, 150)]),
    );
}

/// Room outlines from a `canopy_msgs/msg/RoomArray`, labelled with their names.
fn draw_rooms(rec: &RecordingStream, msg: &Value) {
    let list = msg["rooms"].as_array().map_or(&[][..], Vec::as_slice);
    let mut strips = Vec::new();
    let mut labels = Vec::new();
    for room in list {
        let Some(points) = room["outline"]["points"].as_array() else {
            continue;
        };
        let mut strip: Vec<[f32; 3]> = points
            .iter()
            .map(|p| [xyz(p)[0], xyz(p)[1], 0.02])
            .collect();
        if let Some(first) = strip.first().copied() {
            strip.push(first);
        }
        strips.push(strip);
        let text = |k: &str| room[k].as_str().unwrap_or_default().to_owned();
        labels.push(format!("{} {}", text("id"), text("name")));
    }
    put(
        rec,
        "world/rooms",
        &rerun::LineStrips3D::new(strips)
            .with_labels(labels)
            .with_radii([0.02]),
    );
}

/// Objects from a `canopy_msgs/msg/WorldObjectArray` as labelled boxes; stale ones are faded.
fn draw_objects(rec: &RecordingStream, msg: &Value) {
    let list = msg["objects"].as_array().map_or(&[][..], Vec::as_slice);
    let live: Vec<&Value> = list
        .iter()
        .filter(|o| o["state"].as_u64() != Some(REMOVED))
        .collect();
    let centers = live.iter().map(|o| xyz(&o["pose"]["position"]));
    let halves = live.iter().map(|o| xyz(&o["size"]).map(|s| s / 2.0));
    let rotations = live.iter().map(|o| {
        let q = &o["pose"]["orientation"];
        let w = q["w"].as_f64().unwrap_or(1.0);
        rerun::Quaternion::from_xyzw(f32s([num(&q["x"]), num(&q["y"]), num(&q["z"]), w]))
    });
    let labels = live.iter().map(|o| {
        let name = o["name"]
            .as_str()
            .filter(|n| !n.is_empty())
            .or_else(|| o["label"].as_str())
            .unwrap_or_default();
        format!("{} {name}", o["id"].as_str().unwrap_or_default())
    });
    let colors = live.iter().map(|o| {
        let alpha = if o["state"].as_u64() == Some(STALE) {
            90
        } else {
            255
        };
        rerun::Color::from_unmultiplied_rgba(120, 160, 255, alpha)
    });
    put(
        rec,
        "world/objects",
        &rerun::Boxes3D::from_centers_and_half_sizes(centers, halves)
            .with_quaternions(rotations)
            .with_labels(labels)
            .with_colors(colors),
    );
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
    fn walls_are_occupied_cell_centres() {
        let msg = serde_json::json!({
            "info": {"resolution": 0.5, "width": 2, "height": 2,
                     "origin": {"position": {"x": 1.0, "y": -1.0, "z": 0.0}}},
            "data": [0, 100, -1, 65]
        });
        let (points, size) = walls(&msg);
        assert_eq!(points, vec![[1.75, -0.75, 0.0], [1.75, -0.25, 0.0]]);
        assert!((size - 0.5).abs() < f32::EPSILON);
    }
}
