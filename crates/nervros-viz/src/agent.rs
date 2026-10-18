//! The agent's events in the viewer: its log, snapshots, a mission's steps, its plan, and what the
//! robot's stacks draw.

use std::sync::Arc;

use nervros_core::mission::Outcome;
use nervros_core::session::Event;
use nervros_ros::RobotPort;
use rerun::RecordingStream;
use tokio::sync::broadcast;
use tokio::task::JoinSet;

use crate::blueprint::Layout;
use crate::plan::{draw_preview, step_states};
use crate::tasks::plot;
use crate::{PLAN_PATH, f32s, put, put_static};

/// The robot's own colour, the window's teal: what a stack draws of the robot's next move.
const DRAWN_TEAL: [u8; 3] = [76, 201, 196];

pub(super) async fn agent(
    rec: RecordingStream,
    (robot, map_frame): (Arc<dyn RobotPort>, String),
    layout: Arc<Layout>,
    mut events: broadcast::Receiver<Event>,
) {
    // Plots end with the bridge, like every other task it draws with.
    let mut plots = JoinSet::new();
    loop {
        let event = match events.recv().await {
            Ok(e) => e,
            Err(broadcast::error::RecvError::Lagged(_)) => continue,
            Err(broadcast::error::RecvError::Closed) => return,
        };
        match &event {
            Event::Plot {
                name,
                topic,
                msg_type,
                field,
                for_s,
            } => {
                plots.spawn(plot(&rec, &robot, name, (topic, msg_type, field), *for_s));
                // Asked for, so shown: the agent's reply says where to look.
                layout.show_plots(*for_s);
                continue;
            }
            Event::Drawn {
                layer,
                frame,
                points,
                label,
            } => {
                draw_line(
                    &rec,
                    (robot.as_ref(), &map_frame),
                    layer,
                    (frame, points),
                    label,
                );
                continue;
            }
            Event::Snapshot { jpeg, .. } => {
                let image = rerun::EncodedImage::from_file_contents(jpeg.to_vec());
                put(&rec, "agent/look", &image);
                continue;
            }
            Event::MissionStarted { .. } => {
                // A new mission's steps start from empty lanes, on show; the stacks' drawings
                // from before it, as an arm preview, would read as part of it.
                put(&rec, "mission", &rerun::Clear::recursive());
                put_static(&rec, "world/drawn", &rerun::Clear::recursive());
                layout.show_mission();
                continue;
            }
            Event::MissionPreview { steps, .. } => {
                draw_preview(&rec, steps);
                continue;
            }
            // A preview lasts while its plan is pending or running.
            Event::MissionPlanned { .. }
            | Event::ApprovalResolved {
                approved: false, ..
            } => {
                clear_plan(&rec);
                continue;
            }
            Event::MissionFinished {
                outcome,
                failed_step,
                ..
            } => {
                clear_plan(&rec);
                end_lane(&rec, *outcome, failed_step);
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
            _ => {}
        }
        if let Some((level, text)) = log_line(&event) {
            put(
                &rec,
                "agent/log",
                &rerun::TextLog::new(text).with_level(level),
            );
        }
    }
}

/// The agent log's line for an event, if it writes one.
fn log_line(event: &Event) -> Option<(&'static str, String)> {
    use rerun::TextLogLevel as L;
    Some(match event {
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
        _ => return None,
    })
}

/// A stack's line on the map, placed by TF from its frame now; no points clear it.
fn draw_line(
    rec: &RecordingStream,
    (robot, map): (&dyn RobotPort, &str),
    layer: &str,
    (frame, points): (&str, &[[f64; 3]]),
    label: &str,
) {
    let path = format!("world/drawn/{layer}");
    if points.is_empty() {
        put_static(rec, &path, &rerun::Clear::recursive());
        return;
    }
    let Ok(t) = robot.transform(map, frame) else {
        tracing::debug!(%frame, "a drawing's frame is not on the map");
        return;
    };
    let on_map: Vec<[f32; 3]> = points.iter().map(|p| f32s(t.apply(*p))).collect();
    let [r, g, b] = DRAWN_TEAL;
    let colour = rerun::Color::from_rgb(r, g, b);
    put_static(
        rec,
        &path,
        &rerun::LineStrips3D::new([on_map.clone()])
            .with_colors([colour])
            .with_radii([0.02]),
    );
    if let (Some(start), Some(end)) = (on_map.first(), on_map.last()) {
        let mut dot = rerun::Points3D::new([*end])
            .with_colors([colour])
            .with_radii([0.03]);
        // A short drawing, as an arm's path, lies on the robot, where a label would hide it.
        let reach: f32 = start.iter().zip(end).map(|(a, b)| (a - b).powi(2)).sum();
        if reach.sqrt() > 1.0 {
            dot = dot.with_labels([label]);
        }
        put_static(rec, &format!("{path}/end"), &dot);
    }
}

/// Takes the preview and the navigation stack's last path off the map. The path is drawn as
/// static data, which a clear on the timeline leaves standing: a denied plan's path stayed drawn.
fn clear_plan(rec: &RecordingStream) {
    put_static(rec, PLAN_PATH, &rerun::Clear::recursive());
    put(rec, PLAN_PATH, &rerun::Clear::recursive());
}

/// The executor sends no last state for a step it stopped: its lane would stay "running" for
/// good.
fn end_lane(rec: &RecordingStream, outcome: Outcome, failed_step: &str) {
    if failed_step.is_empty() {
        return;
    }
    let state = if outcome == Outcome::Canceled {
        "stopped"
    } else {
        "failure"
    };
    put(
        rec,
        &format!("mission/{failed_step}"),
        &rerun::StateChange::single(state),
    );
}
