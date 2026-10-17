//! What NervROS last saw of Nav2: whether it is up, how sure it is of where the robot is, what it
//! is doing, and what slows it down.

use std::fmt::Write as _;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use nervros_ros::RobotPort;
use serde::Serialize;
use serde_json::{Value, json};

use super::Nav2Config;
use crate::integrations::has_action;
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};
use crate::zones::Zones;

/// How often the watcher looks.
const PERIOD: Duration = Duration::from_secs(3);
/// How long one read may wait.
const READ_WAIT: Duration = Duration::from_millis(400);
/// Feedback older than this is from a walk that ended.
const FEEDBACK_FRESH: Duration = Duration::from_secs(3);
/// How often the graph is read again for the actions Nav2 serves.
const GRAPH_EVERY: u32 = 10;
/// Nav2's actions that move the base, by the name each runs under.
pub(super) const MOVING: [&str; 9] = [
    "navigate_to_pose",
    "navigate_through_poses",
    "follow_waypoints",
    "follow_path",
    "spin",
    "drive_on_heading",
    "backup",
    "dock_robot",
    "undock_robot",
];

/// A walk's progress, from its feedback.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub(super) struct Walk {
    /// Metres still to go.
    pub(super) remaining_m: f64,
    /// Nav2's estimate of the time it still takes, s.
    pub(super) eta_s: f64,
    /// How often it recovered (cleared costmaps, spun, waited) on the way.
    pub(super) recoveries: u64,
}

/// What NervROS last saw of Nav2.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub(super) struct Seen {
    /// Whether the navigation lifecycle manager says its nodes are active; none when it did not
    /// answer.
    pub(super) active: Option<bool>,
    /// How sure localization is: the position's spread (one standard deviation), m.
    pub(super) spread_m: Option<f64>,
    /// The actions moving the base now.
    pub(super) running: Vec<String>,
    /// The walk under way, from its feedback.
    pub(super) walk: Option<Walk>,
    /// What the collision monitor does now and for which polygon, while it does anything.
    pub(super) monitor: Option<(String, String)>,
    /// The speed limit Nav2's controller is under, as said; none when there is none.
    pub(super) limit: Option<String>,
    /// Whether it was ever heard from.
    pub(super) heard: bool,
}

impl Seen {
    /// Whether the base is being moved by Nav2.
    pub(super) fn moving(&self) -> bool {
        !self.running.is_empty()
    }

    /// One line for each turn.
    pub(super) fn parts(&self) -> Vec<String> {
        if !self.heard {
            return vec!["Nav2: not heard from".to_owned()];
        }
        let mut parts = Vec::new();
        parts.push(match self.active {
            Some(true) => "Nav2: up".to_owned(),
            Some(false) => "Nav2: its nodes are not active".to_owned(),
            None => "Nav2".to_owned(),
        });
        if let Some(s) = self.spread_m {
            parts.push(if s < 0.01 {
                "localized to within a centimetre".to_owned()
            } else {
                format!("localized to ±{s:.2} m")
            });
        }
        if self.running.is_empty() {
            parts.push("idle".to_owned());
        } else {
            let mut doing = format!("moving the base ({})", self.running.join(", "));
            if let Some(w) = &self.walk {
                let _ = write!(
                    doing,
                    ": {:.1} m to go, about {:.0} s",
                    w.remaining_m, w.eta_s
                );
                if w.recoveries > 0 {
                    let _ = write!(doing, ", {}", crate::counted(w.recoveries, "recovery"));
                }
            }
            parts.push(doing);
        }
        if let Some((action, polygon)) = &self.monitor {
            parts.push(format!("collision monitor {action} ({polygon})"));
        }
        if let Some(limit) = &self.limit {
            parts.push(format!("speed limit {limit}"));
        }
        parts
    }

    /// The same, for `nav_status`.
    pub(super) fn json(&self) -> Value {
        json!({
            "active": self.active,
            "localization_spread_m": self.spread_m.map(|s| (s * 1000.0).round() / 1000.0),
            "moving": self.running,
            "walk": self.walk,
            "collision_monitor": self.monitor.as_ref().map_or_else(
                || json!("quiet: nothing is inside its polygons, or it does not say"),
                |(a, p)| json!({"doing": a, "polygon": p}),
            ),
            "speed_limit": self.limit,
        })
    }
}

/// Keeps [`Seen`] fresh in the background while the session lives.
#[derive(Debug)]
pub(super) struct Watcher {
    seen: Mutex<Seen>,
    config: Nav2Config,
    /// The collision monitor's state topic, when it publishes one.
    state_topic: Mutex<Option<String>>,
}

impl Watcher {
    /// Starts watching.
    pub(super) fn start(config: &Nav2Config, robot: Arc<dyn RobotPort>) -> Arc<Self> {
        let watcher = Arc::new(Self {
            seen: Mutex::new(Seen::default()),
            config: config.clone(),
            state_topic: Mutex::new(None),
        });
        let weak = Arc::downgrade(&watcher);
        tokio::spawn(async move {
            let mut served: Vec<String> = Vec::new();
            let mut rounds = 0_u32;
            loop {
                let Some(watcher) = weak.upgrade() else {
                    return;
                };
                if rounds.is_multiple_of(GRAPH_EVERY) {
                    served = watcher.served(robot.as_ref()).await;
                }
                rounds = rounds.wrapping_add(1);
                let seen = watcher.look(robot.as_ref(), &served).await;
                *crate::lock(&watcher.seen) = seen;
                drop(watcher);
                tokio::time::sleep(PERIOD).await;
            }
        });
        watcher
    }

    /// What it last saw.
    pub(super) fn seen(&self) -> Seen {
        crate::lock(&self.seen).clone()
    }

    /// Looks now, rather than at its next round.
    pub(super) async fn now(&self, robot: &dyn RobotPort) -> Seen {
        let served = self.served(robot).await;
        let seen = self.look(robot, &served).await;
        *crate::lock(&self.seen) = seen.clone();
        seen
    }

    /// The moving actions Nav2 serves, and the collision monitor's state topic.
    async fn served(&self, robot: &dyn RobotPort) -> Vec<String> {
        let Ok(graph) = robot.graph_detail().await else {
            return Vec::new();
        };
        let monitor = self.config.monitor_node();
        if let Ok(values) =
            crate::integrations::get_params(robot, &monitor, &["state_topic".to_owned()]).await
            && let Some(topic) = values
                .first()
                .and_then(|(v, _)| v.as_str())
                .filter(|t| !t.is_empty())
        {
            *crate::lock(&self.state_topic) =
                Some(crate::integrations::resolve(&self.config.namespace, topic));
        }
        MOVING
            .iter()
            .map(|a| self.config.name(a))
            .filter(|a| has_action(&graph.topics, a))
            .collect()
    }

    async fn look(&self, robot: &dyn RobotPort, served: &[String]) -> Seen {
        let c = &self.config;
        let names = (
            c.name(&format!("{}/is_active", c.lifecycle_manager)),
            c.name("amcl_pose"),
            format!("{}/_action/feedback", c.name("navigate_to_pose")),
            c.name("speed_limit"),
        );
        let lifecycle = robot.call(&names.0, "std_srvs/srv/Trigger", json!({}), READ_WAIT);
        let pose = robot.latest(
            &names.1,
            "geometry_msgs/msg/PoseWithCovarianceStamped",
            READ_WAIT,
        );
        let statuses = futures::future::join_all(served.iter().map(|a| async move {
            let status = robot
                .latest(
                    &format!("{a}/_action/status"),
                    "action_msgs/msg/GoalStatusArray",
                    READ_WAIT,
                )
                .await;
            (a.clone(), status.is_ok_and(|s| running(&s)))
        }));
        let feedback = robot.latest_fresh(
            &names.2,
            "nav2_msgs/action/NavigateToPose_FeedbackMessage",
            READ_WAIT,
            FEEDBACK_FRESH,
        );
        let limit = robot.latest(&names.3, "nav2_msgs/msg/SpeedLimit", READ_WAIT);
        let state_topic = crate::lock(&self.state_topic).clone();
        let monitor = async {
            match state_topic {
                Some(t) => robot
                    .latest(&t, "nav2_msgs/msg/CollisionMonitorState", READ_WAIT)
                    .await
                    .ok(),
                None => None,
            }
        };
        let (lifecycle, pose, statuses, feedback, limit, monitor) =
            tokio::join!(lifecycle, pose, statuses, feedback, limit, monitor);
        let running: Vec<String> = statuses
            .into_iter()
            .filter(|(_, r)| *r)
            .map(|(a, _)| a.rsplit('/').next().unwrap_or(&a).to_owned())
            .collect();
        let heard = lifecycle.is_ok() || pose.is_ok() || !served.is_empty();
        Seen {
            active: lifecycle.ok().and_then(|r| r["success"].as_bool()),
            spread_m: pose.ok().and_then(|p| spread(&p)),
            walk: feedback
                .ok()
                .filter(|_| running.iter().any(|r| r == "navigate_to_pose"))
                .and_then(|f| walk(&f["feedback"])),
            running,
            monitor: monitor.and_then(|m| monitoring(&m)),
            limit: limit.ok().and_then(|l| limit_words(&l)),
            heard,
        }
    }
}

/// `nav_status`: what Nav2 is doing now, read fresh.
pub(super) struct NavStatus {
    spec: ToolSpec,
    watcher: Arc<Watcher>,
    robot: Arc<dyn RobotPort>,
    zones: Arc<Zones>,
}

impl NavStatus {
    pub(super) fn new(watcher: Arc<Watcher>, robot: Arc<dyn RobotPort>, zones: Arc<Zones>) -> Self {
        let spec = ToolSpec::new(
            "nav_status",
            "Nav2 now: whether its servers are up, how sure localization is of where the robot \
             stands, what moves the base and how far it has to go, what the collision monitor \
             is doing, the speed limit in force, and the zones.",
            json!({"type": "object", "properties": {}, "additionalProperties": false}),
            Risk::Observe,
        );
        Self {
            spec,
            watcher,
            robot,
            zones,
        }
    }
}

#[async_trait::async_trait]
impl Tool for NavStatus {
    fn spec(&self) -> std::borrow::Cow<'_, ToolSpec> {
        std::borrow::Cow::Borrowed(&self.spec)
    }

    async fn call(&self, _args: Value) -> ToolOutcome {
        let seen = self.watcher.now(self.robot.as_ref()).await;
        if !seen.heard {
            return ToolOutcome::failed(
                "Nav2 did not answer: its lifecycle manager, localization and actions are all \
                 silent",
            );
        }
        let mut out = seen.json();
        let zones = self.zones.all();
        let slow = zones.iter().filter(|z| !z.keeps_out()).count();
        out["zones"] = json!({"keep_out": zones.len() - slow, "slow": slow});
        ToolOutcome::ok(out)
    }
}

/// Whether an action's status array has a goal accepted, running or being cancelled.
fn running(status: &Value) -> bool {
    status["status_list"]
        .as_array()
        .into_iter()
        .flatten()
        .any(|s| matches!(s["status"].as_u64(), Some(1..=3)))
}

/// The position's spread from a pose's covariance: the larger of x's and y's.
fn spread(pose: &Value) -> Option<f64> {
    let cov = pose["pose"]["covariance"].as_array()?;
    let (xx, yy) = (cov.first()?.as_f64()?, cov.get(7)?.as_f64()?);
    Some(xx.max(yy).max(0.0).sqrt())
}

fn walk(feedback: &Value) -> Option<Walk> {
    let secs = |d: &Value| {
        let s = d["sec"].as_f64()?;
        Some(s + d["nanosec"].as_f64().unwrap_or(0.0) / 1e9)
    };
    Some(Walk {
        remaining_m: feedback["distance_remaining"].as_f64()?,
        eta_s: secs(&feedback["estimated_time_remaining"]).unwrap_or(0.0),
        recoveries: feedback["number_of_recoveries"].as_u64().unwrap_or(0),
    })
}

/// What the collision monitor does, in words, unless it does nothing.
fn monitoring(state: &Value) -> Option<(String, String)> {
    let doing = match state["action_type"].as_u64()? {
        1 => "stopping the robot",
        2 => "slowing the robot",
        3 => "keeping its distance",
        4 => "limiting its speed",
        _ => return None,
    };
    let polygon = state["polygon_name"].as_str().unwrap_or_default();
    Some((doing.to_owned(), polygon.to_owned()))
}

/// The limit a `SpeedLimit` sets, in words; none for 0 or 100 per cent, which set none.
fn limit_words(limit: &Value) -> Option<String> {
    let v = limit["speed_limit"].as_f64().filter(|v| *v > 0.0)?;
    if limit["percentage"].as_bool() == Some(true) {
        (v < 100.0).then(|| format!("{v:.0}%"))
    } else {
        Some(format!("{v:.2} m/s"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_line_says_what_nav2_does_and_what_slows_it() {
        let idle = Seen {
            active: Some(true),
            spread_m: Some(0.05),
            heard: true,
            ..Seen::default()
        };
        assert_eq!(
            idle.parts().join("; "),
            "Nav2: up; localized to ±0.05 m; idle"
        );
        let walking = Seen {
            running: vec!["navigate_to_pose".into()],
            walk: Some(Walk {
                remaining_m: 3.21,
                eta_s: 8.6,
                recoveries: 1,
            }),
            monitor: Some(("slowing the robot".into(), "PolygonSlow".into())),
            limit: Some("30%".into()),
            ..idle
        };
        assert_eq!(
            walking.parts().join("; "),
            "Nav2: up; localized to ±0.05 m; moving the base (navigate_to_pose): 3.2 m to go, \
             about 9 s, 1 recovery; collision monitor slowing the robot (PolygonSlow); speed \
             limit 30%"
        );
        assert_eq!(Seen::default().parts().join("; "), "Nav2: not heard from");
    }

    #[test]
    fn statuses_spreads_and_limits_read_as_nav2_means_them() {
        assert!(running(
            &json!({"status_list": [{"status": 4}, {"status": 2}]})
        ));
        assert!(!running(
            &json!({"status_list": [{"status": 4}, {"status": 6}]})
        ));
        let mut cov = vec![0.0; 36];
        cov[0] = 0.0025;
        cov[7] = 0.0016;
        let s = spread(&json!({"pose": {"covariance": cov}})).unwrap();
        assert!((s - 0.05).abs() < 1e-9);
        assert_eq!(
            limit_words(&json!({"percentage": true, "speed_limit": 30.0})),
            Some("30%".into())
        );
        assert_eq!(
            limit_words(&json!({"percentage": true, "speed_limit": 0.0})),
            None
        );
        assert_eq!(
            limit_words(&json!({"percentage": false, "speed_limit": 0.2})),
            Some("0.20 m/s".into())
        );
        assert_eq!(
            monitoring(&json!({"action_type": 0, "polygon_name": ""})),
            None
        );
    }
}
