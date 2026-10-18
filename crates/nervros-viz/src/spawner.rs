//! Which tasks a profile starts, and the viewer's layout for the cameras and layers it has.

use std::collections::BTreeSet;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use nervros_core::profile::{LayerConfig, LookConfig, Profile, TopicRef};
use nervros_core::zones::Zones;
use nervros_ros::{RobotPort, Transform};
use rerun::RecordingStream;
use tokio::task::JoinSet;

use crate::blueprint::Layout;
use crate::grid::{draw_coverage, draw_map, known_extent};
use crate::objects::{Drawn, draw_object_names, draw_objects};
use crate::plan::{draw_plan, draw_trail};
use crate::rooms::draw_rooms;
use crate::tasks::{Drawer, arm_scene, camera, draw_detections, robot_model, sample, scan};
use crate::zones::zones;
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
    /// Clears so far, for a drawer that leaves out what it drew before.
    clears: Arc<AtomicU64>,
}

impl Layer {
    pub(super) fn shown(&self) -> bool {
        self.layers.shown(&self.name)
    }

    /// Takes everything the layer drew out of the viewer.
    pub(super) fn clear(&self, rec: &RecordingStream) {
        put_static(rec, &self.path, &rerun::Clear::recursive());
        put(rec, &self.path, &rerun::Clear::recursive());
        self.clears.fetch_add(1, Ordering::Relaxed);
    }

    /// Counts its clears: shown again, the layer is empty, whatever a drawer kept.
    pub(super) fn clears(&self) -> Arc<AtomicU64> {
        Arc::clone(&self.clears)
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
            clears: Arc::default(),
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
            let objects = self.layer("Objects", "world/objects", true);
            let mut drawn = Drawn::new(objects.clears());
            self.watch(
                world.objects.clone(),
                OBJECTS_PERIOD,
                objects,
                Box::new(move |rec, path, msg| draw_objects(rec, path, msg, drawn.looks())),
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

    /// The keep-out zones, redrawn as the operator adds or removes them.
    pub(super) fn zones(&mut self, all: &Arc<Zones>) {
        let layer = self.layer("Zones", "world/zones", true);
        self.tasks
            .spawn(zones(self.rec.clone(), Arc::clone(all), layer));
    }

    /// What MoveIt plans the arms around, on a robot with MoveIt.
    pub(super) fn arm_scene(&mut self, profile: &Profile) {
        if let Some(moveit) = &profile.moveit {
            let layer = self.layer("Arm obstacles", "world/arm_scene", true);
            self.tasks.spawn(arm_scene(
                self.rec.clone(),
                Arc::clone(&self.robot),
                (
                    moveit.name("get_planning_scene"),
                    profile.ros.map_frame.clone(),
                ),
                layer,
            ));
        }
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
    let colour = config.color.unwrap_or([r, g, b]);
    // Above the map and coverage, one layer over the next.
    #[expect(clippy::cast_precision_loss, reason = "a handful of layers")]
    let lift = 0.02 + 0.005 * index as f32;
    if config.msg_type == "geometry_msgs/msg/PolygonStamped" {
        let (robot, map) = (Arc::clone(robot), map.to_owned());
        return Box::new(move |rec, path, msg| {
            let frame = msg["header"]["frame_id"].as_str().unwrap_or_default();
            let to_map = if frame.is_empty() || frame == map {
                Ok(Transform::IDENTITY)
            } else {
                robot.transform(&map, frame)
            };
            if let Ok(t) = to_map {
                layers::draw_polygon(rec, path, msg, &t, (colour, lift));
            }
        });
    }
    let (robot, map) = (Arc::clone(robot), map.to_owned());
    let costmap = config.style.as_deref() == Some("costmap");
    Box::new(move |rec, path, msg| {
        // A grid in another frame, as a local costmap in odom, is placed on the map by TF.
        let frame = msg["header"]["frame_id"].as_str().unwrap_or_default();
        let to_map = if frame.is_empty() || frame == map {
            Ok(Transform::IDENTITY)
        } else {
            robot.transform(&map, frame)
        };
        match to_map {
            Ok(t) if costmap => layers::draw_costmap(rec, path, msg, &t, lift),
            Ok(t) => layers::draw_occupied(rec, path, msg, &t, (colour, lift)),
            Err(_) => {}
        }
    })
}
