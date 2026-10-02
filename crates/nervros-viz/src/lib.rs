//! Maps ROS 2 data and agent events to Rerun, for the viewer embedded in the GUI.
//!
//! Every Rerun logging call lives here, so an upgrade touches one crate. The bridge samples the
//! robot at fixed rates and logs only what changed, which bounds what the viewer stores and keeps
//! it idle while the robot is.

use std::sync::Arc;
use std::time::Duration;

use nervros_core::profile::Profile;
use nervros_core::session::Event;
use nervros_ros::RobotPort;
use rerun::external::re_log_channel::{self, LogReceiver, LogSource};
use rerun::sink::CallbackSink;
use rerun::{AsComponents, RecordingStream, RecordingStreamBuilder};
use serde_json::Value;
use tokio::sync::broadcast;
use tokio::task::JoinSet;

mod agent;
mod blueprint;
mod grid;
mod layers;
mod objects;
mod plan;
mod rooms;
mod spawner;
mod tasks;

pub use layers::Layers;

use agent::agent;
use blueprint::Layout;
use spawner::Spawner;
use tasks::pose;

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

/// The robot's ring and heading arrow.
const ROBOT_MARK: [u8; 3] = [80, 170, 255];

/// What the 3D view tracks to follow the robot: two unseen points around it, whose span sets how
/// far back the viewer orbits.
const FOLLOW_PATH: &str = "world/robot/follow";

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
    layout: Arc<Layout>,
    _tasks: JoinSet<()>,
}

impl Bridge {
    /// Puts the viewer's panes back as NervROS lays them out, the 3D view on the latest map.
    pub fn reset_layout(&self) {
        self.layout.send();
    }

    /// Keeps the 3D view on the robot from the angle it has, or frames the whole map again; the
    /// layout is sent anew, so its panes are put back too.
    pub fn follow(&self, on: bool) {
        self.layout.follow(on);
    }

    /// Whether the 3D view keeps to the robot.
    #[must_use]
    pub fn following(&self) -> bool {
        self.layout.following()
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
    let layout = Arc::new(Layout::new(rec.clone(), bridge.cameras(profile)));
    layout.send();
    bridge.world(profile, &layout);
    bridge.profile_layers(profile);
    bridge
        .tasks
        .spawn(agent(rec.clone(), Arc::clone(robot), events));
    Bridge {
        layers: bridge.layers,
        layout,
        _tasks: bridge.tasks,
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

fn num(v: &Value) -> f64 {
    v.as_f64().unwrap_or(0.0)
}

fn xyz(v: &Value) -> [f32; 3] {
    f32s([num(&v["x"]), num(&v["y"]), num(&v["z"])])
}

#[cfg(test)]
mod tests {
    use nervros_ros::Transform;

    use super::*;
    use crate::grid::{coverage_colour, grid, known_extent, map_colour, seen, seen_colour};
    use crate::objects::object_colour;
    use crate::rooms::area;
    use crate::tasks::{ModelFrame, load_model, moved, pinhole, pose_model};

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
    fn each_pose_names_the_frames_it_moves() {
        // A row without frame names moves the entity's own frame: the model then kept its rest
        // pose with the pelvis on the floor.
        use rerun::external::re_chunk::Chunk;
        use rerun::external::re_log_types::LogMsg;
        let lift = Transform {
            translation: [0.0, 0.0, 0.73],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        let robot =
            nervros_ros::fake::FakeRobot::new().with_transform("base_footprint", "pelvis", lift);
        let pelvis = ModelFrame {
            entity: "world/robot/model/joints/root".into(),
            tf: ("base_footprint".into(), "pelvis".into()),
            parent: ROBOT_FRAME.into(),
        };
        let (rec, storage) = RecordingStreamBuilder::new("test").memory().unwrap();
        pose_model(&rec, &robot, &[pelvis], &mut [None]);
        rec.flush_blocking().unwrap();
        let rows: Vec<Chunk> = storage
            .take()
            .into_iter()
            .filter_map(|m| match m {
                LogMsg::ArrowMsg(_, arrow) => Chunk::from_arrow_msg(&arrow).ok(),
                _ => None,
            })
            .filter(|c| c.entity_path().to_string() == "/world/robot/model/joints/root")
            .collect();
        assert!(!rows.is_empty(), "the root was posed");
        for chunk in rows {
            let has =
                |d: rerun::ComponentDescriptor| chunk.components().get_array(d.component).is_some();
            assert!(has(rerun::Transform3D::descriptor_translation()));
            assert!(
                has(rerun::Transform3D::descriptor_child_frame()),
                "names the pelvis"
            );
            assert!(
                has(rerun::Transform3D::descriptor_parent_frame()),
                "and the robot it hangs from"
            );
        }
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
        let g = grid(&msg, map_colour).unwrap();
        let [clear, wall, floor] = [-1, 100, 0].map(map_colour);
        assert_eq!(
            g.bytes,
            [clear, wall, floor, wall].concat(),
            "the second ROS row is the image's first"
        );
        assert_eq!(clear[3], 0, "the unknown stays clear");
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
            grid(&short, map_colour).is_none(),
            "a grid with missing cells is not drawn"
        );
    }

    #[test]
    fn a_map_is_framed_on_its_known_cells_not_its_unknown_margin() {
        // 4 x 3 cells of 0.5 m from (1, -1); only the middle two columns of the top two rows
        // are known.
        let msg = serde_json::json!({
            "info": {"resolution": 0.5, "width": 4, "height": 3,
                     "origin": {"position": {"x": 1.0, "y": -1.0, "z": 0.0}}},
            "data": [-1, -1, -1, -1,  -1, 0, 100, -1,  -1, 0, -1, -1]
        });
        assert_eq!(known_extent(&msg), Some([1.5, -0.5, 2.5, 0.5]));
        let unknown = serde_json::json!({
            "info": {"resolution": 0.5, "width": 2, "height": 1,
                     "origin": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}}},
            "data": [-1, -1]
        });
        assert_eq!(known_extent(&unknown), None, "nothing known frames nothing");
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
