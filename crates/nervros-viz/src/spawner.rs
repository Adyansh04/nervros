//! Which tasks a profile starts, and the viewer's layout for the cameras and layers it has.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;
use std::time::Duration;

use nervros_core::profile::{LayerConfig, LookConfig, Profile, TopicRef};
use nervros_ros::{RobotPort, Transform};
use rerun::RecordingStream;
use tokio::task::JoinSet;

use crate::blueprint::Layout;
use crate::grid::{draw_coverage, draw_map, known_extent};
use crate::objects::{draw_object_names, draw_objects};
use crate::plan::{draw_plan, draw_trail};
use crate::rooms::draw_rooms;
use crate::tasks::{Drawer, camera, draw_detections, robot_model, sample, scan};
use crate::{
    CAMERA_PATH, CAMERA_PERIOD, Layers, MODEL_PATH, OBJECT_PALETTE, OBJECTS_PERIOD, POSE_PERIOD,
    SLOW_PERIOD, layers, put, put_static,
};

/// A layer as a task draws it: its name in [`Layers`] and the entity it draws under.
#[derive(Clone)]
pub(super) struct Layer {
    pub(super) name: String,
    pub(super) path: String,
    pub(super) layers: Arc<Layers>,
}

impl Layer {
    pub(super) fn shown(&self) -> bool {
        self.layers.shown(&self.name)
    }

    /// Takes everything the layer drew out of the viewer.
    pub(super) fn clear(&self, rec: &RecordingStream) {
        put_static(rec, &self.path, &rerun::Clear::recursive());
        put(rec, &self.path, &rerun::Clear::recursive());
    }
}

/// What `spawn` starts the tasks with.
pub(super) struct Spawner {
    pub(super) rec: RecordingStream,
    pub(super) robot: Arc<dyn RobotPort>,
    pub(super) layers: Arc<Layers>,
    pub(super) tasks: JoinSet<()>,
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

    pub(super) fn robot_model(&mut self, profile: &Profile) {
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
    pub(super) fn cameras(&mut self, profile: &Profile) -> Vec<(String, String)> {
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

    /// The world model's layers; the first map frames the 3D view.
    pub(super) fn world(&mut self, profile: &Profile, layout: &Arc<Layout>) {
        if let Some(world) = &profile.world {
            let map = self.layer("Map", "world/map", true);
            let layout = Arc::clone(layout);
            self.watch(
                world.map.clone(),
                SLOW_PERIOD,
                map,
                Box::new(move |rec, path, msg| {
                    draw_map(rec, path, msg);
                    if let Some(extent) = known_extent(msg) {
                        layout.map(extent);
                    }
                }),
            );
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

    pub(super) fn profile_layers(&mut self, profile: &Profile) {
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
