//! The agent's events in the viewer: its log, snapshots, a mission's steps and its plan.

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
use crate::{PLAN_PATH, put, put_static};

pub(super) async fn agent(
    rec: RecordingStream,
    robot: Arc<dyn RobotPort>,
    layout: Arc<Layout>,
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
                // Asked for, so shown: the agent's reply says where to look.
                layout.show_plots();
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
            | Event::ApprovalResolved {
                approved: false, ..
            } => {
                put(&rec, PLAN_PATH, &rerun::Clear::recursive());
                continue;
            }
            Event::MissionFinished {
                outcome,
                failed_step,
                ..
            } => {
                put(&rec, PLAN_PATH, &rerun::Clear::recursive());
                // The executor sends no last state for a step it stopped: the lane would stay
                // "running" for good.
                if !failed_step.is_empty() {
                    let state = if *outcome == Outcome::Canceled {
                        "stopped"
                    } else {
                        "failure"
                    };
                    let path = format!("mission/{failed_step}");
                    put(&rec, &path, &rerun::StateChange::single(state));
                }
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
