//! Plans: the path the robot follows, a plan's preview, the robot's trail, and the steps' states.

use nervros_core::mission::preview::PreviewStep;
use rerun::RecordingStream;
use serde_json::Value;

use crate::{PLAN_PATH, f32s, put, put_static, xyz};

/// The path the navigation stack is following, from a `nav_msgs/msg/Path`.
pub(super) fn draw_plan(rec: &RecordingStream, path: &str, msg: &Value) {
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
pub(super) fn draw_preview(rec: &RecordingStream, steps: &[PreviewStep]) {
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
pub(super) fn draw_trail(rec: &RecordingStream, path: &str, msg: &Value) {
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

/// How a mission step's states look on the "Mission" timeline.
pub(super) fn step_states() -> rerun::StateConfiguration {
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
