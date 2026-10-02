//! The sampling tasks: the robot and its model, the scan, the cameras and detections, and plots.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use nervros_core::profile::TopicRef;
use nervros_ros::image::encode_jpeg;
use nervros_ros::{RobotPort, Transform};
use rerun::RecordingStream;
use serde_json::Value;
use tokio::time::{MissedTickBehavior, interval};

use crate::objects::object_colour;
use crate::spawner::Layer;
use crate::{
    CAMERA_PERIOD, CAMERA_QUALITY, MODEL_PATH, POSE_PERIOD, ROBOT_FRAME, SCAN_PERIOD, TOPIC_WAIT,
    f32s, layers, put, put_static,
};

/// A laser scan as points in the map, redrawn as scans arrive.
pub(super) async fn scan(
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
pub(super) fn draw_detections(rec: &RecordingStream, path: &str, msg: &Value) {
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

pub(super) fn moved(a: &Transform, b: &Transform) -> bool {
    let near = |x: &[f64], y: &[f64], eps: f64| x.iter().zip(y).all(|(p, q)| (p - q).abs() < eps);
    !near(&a.translation, &b.translation, 0.005) || !near(&a.rotation, &b.rotation, 0.001)
}

pub(super) async fn pose(
    rec: RecordingStream,
    robot: Arc<dyn RobotPort>,
    map: String,
    base: String,
) {
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
pub(super) struct ModelFrame {
    /// The entity its transform is logged on.
    pub(super) entity: String,
    /// TF's parent and child frames.
    pub(super) tf: (String, String),
    /// The viewer's parent frame; the child keeps TF's name, which is the URDF link's.
    pub(super) parent: String,
}

/// Sends the robot's model from its URDF, geometry and joints at rest, all static, and returns the
/// frames TF moves.
pub(super) fn load_model(
    rec: &RecordingStream,
    urdf: &Path,
    base: &str,
) -> Result<Vec<ModelFrame>, String> {
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
pub(super) async fn robot_model(
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

pub(super) async fn camera(
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
pub(super) fn pinhole(msg: &Value) -> Option<rerun::Pinhole> {
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
pub(super) type Drawer = Box<dyn FnMut(&RecordingStream, &str, &Value) + Send>;

/// Polls a topic and redraws when its message changed, or clears the layer while it is hidden.
pub(super) async fn sample(
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

/// The agent's replies and tool calls as a text log, and its marked images.
/// Draws one number from a topic's messages over `for_s` seconds; `source` is the topic, its
/// type and the field.
pub(super) fn plot(
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
