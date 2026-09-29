//! Spike S1: the r2r port against grove-g1's simulator (apartment world, canopy with the mock
//! detector, Nav2 in mapping mode). Ignored by default; run with the stack up:
//!
//! ```text
//! export NERVROS_EXTRA_IDL_PACKAGES="g1_msgs;canopy_msgs"
//! source scripts/ros-env.sh && ROS_DOMAIN_ID=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
//!   cargo test -p nervros-ros --test sim_s1 -- --ignored --nocapture
//! ```

#![expect(
    clippy::unwrap_used,
    reason = "a live spike: any failure should stop it loudly"
)]

use std::time::{Duration, Instant};

use nervros_ros::{GoalStatus, R2rConfig, R2rPort, RobotPort, Transform};
use serde_json::json;

async fn wait_for<T>(what: &str, limit: Duration, mut f: impl FnMut() -> Option<T>) -> T {
    let start = Instant::now();
    loop {
        if let Some(v) = f() {
            println!("{what}: {:.1} s", start.elapsed().as_secs_f64());
            return v;
        }
        assert!(start.elapsed() < limit, "{what} not within {limit:?}");
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

fn goal_ahead(pose: &Transform, metres: f64) -> serde_json::Value {
    let (x, y, yaw) = (pose.translation[0], pose.translation[1], pose.yaw());
    json!({
        "pose": {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
            "pose": {
                "position": {"x": x + metres * yaw.cos(), "y": y + metres * yaw.sin(), "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": (yaw / 2.0).sin(), "w": (yaw / 2.0).cos()}
            }
        },
        "behavior_tree": ""
    })
}

async fn robot_pose(port: &R2rPort) -> Transform {
    let pose = wait_for("TF map -> base_footprint", Duration::from_secs(30), || {
        port.transform("map", "base_footprint").ok()
    })
    .await;
    let [x, y, _] = pose.translation;
    println!("robot at ({x:.2}, {y:.2}), yaw {:.2}", pose.yaw());
    pose
}

async fn camera_rate(port: &R2rPort) {
    let mut frames = port.frames("/camera/color/image_raw").unwrap();
    let first = wait_for("first camera frame", Duration::from_secs(30), || {
        frames.borrow().clone()
    })
    .await;
    let (w, h, bytes) = (first.width, first.height, first.data.len());
    println!("frame {w}x{h} {} ({bytes} bytes)", first.encoding);
    let start = Instant::now();
    let mut count = 0u32;
    while start.elapsed() < Duration::from_secs(5) {
        let next = tokio::time::timeout(Duration::from_secs(1), frames.changed()).await;
        count += u32::from(next.is_ok());
    }
    println!(
        "camera: {count} frames in 5 s ({:.1} Hz)",
        f64::from(count) / 5.0
    );
    assert!(count > 0);
}

async fn find_objects(port: &R2rPort) {
    let t = Instant::now();
    let request = json!({"query": "dustbin"});
    let found = port
        .call(
            "/canopy/find_objects",
            "canopy_msgs/srv/FindObjects",
            request,
            Duration::from_secs(30),
        )
        .await
        .unwrap();
    let secs = t.elapsed().as_secs_f64();
    println!(
        "find_objects in {secs:.2} s: found={} coverage={}",
        found["found"], found["searched_coverage"]
    );
}

async fn navigate_and_cancel(port: &R2rPort, pose: &Transform) {
    let action = ("/navigate_to_pose", "nav2_msgs/action/NavigateToPose");
    let t = Instant::now();
    let mut goal = port
        .send_goal(
            action.0,
            action.1,
            goal_ahead(pose, 0.6),
            Duration::from_mins(1),
        )
        .await
        .unwrap();
    println!("goal accepted in {:.2} s", t.elapsed().as_secs_f64());
    if let Ok(Some(fb)) = tokio::time::timeout(Duration::from_secs(20), goal.feedback.recv()).await
    {
        println!("feedback: distance_remaining={}", fb["distance_remaining"]);
    }
    let done = tokio::time::timeout(Duration::from_mins(2), goal.result)
        .await
        .unwrap()
        .unwrap();
    println!(
        "short goal: {:?} after {:.1} s",
        done.unwrap().status,
        t.elapsed().as_secs_f64()
    );

    let pose = port.transform("map", "base_footprint").unwrap();
    let goal = port
        .send_goal(
            action.0,
            action.1,
            goal_ahead(&pose, 3.0),
            Duration::from_secs(30),
        )
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_secs(2)).await;
    let t = Instant::now();
    let cancel = goal.cancel().await;
    let ended = tokio::time::timeout(Duration::from_secs(30), goal.result)
        .await
        .unwrap()
        .unwrap();
    let ended = ended.unwrap();
    println!(
        "cancel {cancel:?}: ended {:?} {:.2} s later",
        ended.status,
        t.elapsed().as_secs_f64()
    );
    if cancel.is_ok() {
        assert_eq!(ended.status, GoalStatus::Canceled);
    }
    let [x, y, _] = port.transform("map", "base_footprint").unwrap().translation;
    println!("robot now at ({x:.2}, {y:.2})");
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "needs the grove-g1 simulator"]
async fn s1_against_the_simulator() {
    let port = R2rPort::start(R2rConfig::default()).unwrap();
    let pose = robot_pose(&port).await;
    camera_rate(&port).await;
    find_objects(&port).await;
    navigate_and_cancel(&port, &pose).await;
}
