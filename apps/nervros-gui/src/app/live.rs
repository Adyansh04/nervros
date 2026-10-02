//! What the window keeps up to date in the background: the robot, the graph and the checks.

use std::collections::HashMap;
use std::sync::{Arc, PoisonError};
use std::time::Duration;

use nervros_core::app::Agent;
use rerun::external::egui;
use tokio::sync::broadcast;

use super::SharedLive;

/// The executor's state and the robot's pose, once a second: the Robot tab and the stop
/// button's label follow them.
fn watch_robot(agent: &Agent, live: &SharedLive, ctx: &egui::Context) {
    let (robot, profile) = (Arc::clone(&agent.robot), agent.profile.clone());
    let (live, wake) = (Arc::clone(live), ctx.clone());
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_secs(1));
        let state = profile.mission.as_ref().map(|m| m.state.clone());
        let (map, base) = (
            profile.ros.map_frame.clone(),
            profile.ros.base_frame.clone(),
        );
        loop {
            tick.tick().await;
            // An executor that has gone quiet shows as gone, not as its last word.
            let executor = match &state {
                Some(topic) => robot
                    .latest_fresh(
                        topic,
                        "nervros_interfaces/msg/RobotState",
                        Duration::from_secs(1),
                        nervros_core::mission::STATE_FRESH,
                    )
                    .await
                    .ok(),
                None => None,
            };
            let pose = robot.transform(&map, &base).ok().map(|t| t.planar());
            let changed = {
                let mut l = live.lock().unwrap_or_else(PoisonError::into_inner);
                let changed = l.executor != executor || l.pose != pose;
                l.executor = executor;
                l.pose = pose;
                changed
            };
            // An idle window sleeps.
            if changed {
                wake.request_repaint();
            }
        }
    });
}

/// Keeps [`Live`] current and wakes the window when the agent or the robot changes.
pub(super) fn watch(
    agent: &Agent,
    live: &SharedLive,
    ctx: &egui::Context,
) -> tokio::sync::mpsc::UnboundedSender<()> {
    let mut events = agent.session.subscribe();
    let wake = ctx.clone();
    tokio::spawn(async move {
        while let Ok(_) | Err(broadcast::error::RecvError::Lagged(_)) = events.recv().await {
            wake.request_repaint();
        }
    });
    watch_robot(agent, live, ctx);
    let (robot, profile) = (Arc::clone(&agent.robot), agent.profile.clone());
    let (live_graph, wake) = (Arc::clone(live), ctx.clone());
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_secs(5));
        let world = profile.world.clone();
        loop {
            tick.tick().await;
            let topics = robot.graph().await.ok().map(|g| g.topics.len());
            let latest = |t: Option<nervros_core::profile::TopicRef>| {
                let robot = Arc::clone(&robot);
                async move {
                    let t = t?;
                    robot
                        .latest(&t.topic, &t.msg_type, Duration::from_secs(1))
                        .await
                        .ok()
                }
            };
            let rooms = latest(world.as_ref().and_then(|w| w.rooms.clone())).await;
            let objects_msg = latest(world.as_ref().and_then(|w| w.objects.clone())).await;
            let objects = objects_msg
                .as_ref()
                .and_then(|m| m["objects"].as_array().map(Vec::len));
            let object_names: HashMap<String, String> = objects_msg
                .as_ref()
                .and_then(|m| m["objects"].as_array())
                .map(|list| {
                    list.iter()
                        .filter_map(|o| {
                            let id = o["id"].as_str()?;
                            let name = o["name"]
                                .as_str()
                                .filter(|n| !n.is_empty())
                                .or_else(|| o["label"].as_str())?;
                            Some((id.to_owned(), name.to_owned()))
                        })
                        .collect()
                })
                .unwrap_or_default();
            let read = |topic: Option<String>, msg_type: &'static str| {
                let robot = Arc::clone(&robot);
                async move {
                    robot
                        .latest(&topic?, msg_type, Duration::from_secs(1))
                        .await
                        .ok()
                }
            };
            let battery = read(
                profile.robot.battery.clone(),
                "sensor_msgs/msg/BatteryState",
            )
            .await;
            let motors = read(
                profile.robot.diagnostics.clone(),
                "diagnostic_msgs/msg/DiagnosticArray",
            )
            .await;
            {
                let mut l = live_graph.lock().unwrap_or_else(PoisonError::into_inner);
                l.topics = topics;
                l.rooms = rooms;
                l.objects = objects;
                l.object_names = object_names;
                l.battery = battery;
                l.motors = motors;
            }
            wake.request_repaint();
        }
    });
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<()>();
    let (robot, profile) = (Arc::clone(&agent.robot), agent.profile.clone());
    let (live_checks, wake) = (Arc::clone(live), ctx.clone());
    tokio::spawn(async move {
        // Discovery needs a moment before the first check means anything.
        tokio::time::sleep(Duration::from_secs(3)).await;
        loop {
            let checks = nervros_core::doctor::run(&profile, robot.as_ref()).await;
            live_checks
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .checks = Some(checks);
            wake.request_repaint();
            if rx.recv().await.is_none() {
                return;
            }
        }
    });
    tx
}
