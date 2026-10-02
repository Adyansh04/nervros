//! The connection check: whether the robot offers everything the profile names. The CLI prints
//! it and the GUI shows it on its Doctor tab; `health_check` gives the agent the same, with the
//! cameras' rates, the robot's place on the map and the executor's state.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::RobotPort;
use serde_json::{Value, json};

use crate::profile::Profile;
use crate::tools::{Risk, Tool, ToolKind, ToolOutcome, ToolSpec};

const SERVICE_WAIT: Duration = Duration::from_secs(5);

/// One line of the report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Check {
    /// Whether it passed.
    pub ok: bool,
    /// What was checked, with the names involved.
    pub what: String,
}

/// Checks the interface files, every tool, the camera and the mission executor. Call it after
/// discovery has had a moment.
pub async fn run(profile: &Profile, robot: &dyn RobotPort) -> Vec<Check> {
    let mut out = Vec::new();
    let mut report = |ok: bool, what: String| out.push(Check { ok, what });
    let graph = match robot.graph().await {
        Ok(g) => g,
        Err(e) => {
            report(false, format!("reading the ROS graph: {e}"));
            return out;
        }
    };
    let has_topic = |t: &str| graph.topics.iter().any(|(name, _)| name == t);
    // Hundreds of files to parse: off the threads that serve the stop.
    let owned = profile.clone();
    let loaded = tokio::task::spawn_blocking(move || {
        crate::schemas::RosidlSchemas::load(&owned).map(|s| s.len())
    })
    .await;
    match loaded {
        Ok(Ok(n)) => report(
            n > 0 || profile.tools.is_empty(),
            format!("{n} interface definitions loaded"),
        ),
        Ok(Err(e)) => report(false, format!("interface files: {e}")),
        Err(e) => report(false, format!("interface files: {e}")),
    }
    // A robot that is down answers no service: waited for together, that is one wait, not one
    // per service.
    let tools = futures::future::join_all(profile.tools.iter().map(|t| async move {
        match t.kind {
            ToolKind::Service => {
                robot
                    .service_available(&t.ros_name, &t.ros_type, SERVICE_WAIT)
                    .await
            }
            ToolKind::Topic => has_topic(&t.ros_name),
        }
    }))
    .await;
    for (t, ok) in profile.tools.iter().zip(tools) {
        report(
            ok,
            format!("tool {} -> {} ({})", t.name, t.ros_name, t.ros_type),
        );
    }
    if let Some(look) = &profile.look {
        report(has_topic(&look.image), format!("look image {}", look.image));
        report(
            has_topic(&look.detections.topic),
            format!("look detections {}", look.detections.topic),
        );
    }
    if let Some(m) = &profile.mission {
        let services = [
            (
                "validate",
                &m.validate,
                "nervros_interfaces/srv/ValidateMission",
            ),
            ("catalog", &m.catalog, "nervros_interfaces/srv/GetCatalog"),
            ("stop", &m.stop, "nervros_interfaces/srv/StopAll"),
        ];
        let answered = futures::future::join_all(
            services
                .iter()
                .map(|(_, name, ty)| robot.service_available(name, ty, SERVICE_WAIT)),
        )
        .await;
        for ((what, name, _), ok) in services.iter().zip(answered) {
            report(ok, format!("mission {what} {name}"));
        }
    }
    out
}

/// Well under any camera's rate, so only a stalled or starved one fails.
const MIN_CAMERA_HZ: f64 = 2.0;
const RATE_WINDOW: Duration = Duration::from_secs(1);

/// The `health_check` tool.
pub struct HealthCheck {
    spec: ToolSpec,
    profile: Arc<Profile>,
    robot: Arc<dyn RobotPort>,
}

impl HealthCheck {
    /// The check over a profile's robot.
    #[must_use]
    pub fn new(profile: &Profile, robot: Arc<dyn RobotPort>) -> Self {
        let spec = ToolSpec::new(
            "health_check",
            "Checks the robot end to end, as a person would before blaming the model: every \
             service and topic the profile uses, each camera's frame rate, whether the robot \
             knows where it is on the map, and what the mission executor is doing. Returns the \
             problems first.",
            json!({"type": "object", "properties": {}, "additionalProperties": false}),
            Risk::Observe,
        );
        Self {
            spec,
            profile: Arc::new(profile.clone()),
            robot,
        }
    }

    async fn checks(&self) -> Vec<Check> {
        let mut out = run(&self.profile, self.robot.as_ref()).await;
        let cameras: Vec<_> = self
            .profile
            .look
            .iter()
            .flat_map(crate::profile::LookConfig::all_cameras)
            .collect();
        // Each camera's window at once.
        let rates = futures::future::join_all(cameras.iter().map(|(_, camera)| {
            self.robot
                .sample_sizes(&camera.image, "sensor_msgs/msg/Image", RATE_WINDOW, 1000)
        }))
        .await;
        for ((name, camera), frames) in cameras.iter().zip(rates) {
            #[expect(clippy::cast_precision_loss, reason = "a count of frames")]
            let hz = frames.unwrap_or_default().len() as f64 / RATE_WINDOW.as_secs_f64();
            out.push(Check {
                ok: hz >= MIN_CAMERA_HZ,
                what: format!("camera {name} ({}) at {hz:.0} Hz", camera.image),
            });
        }
        let (map, base) = (&self.profile.ros.map_frame, &self.profile.ros.base_frame);
        out.push(match self.robot.transform(map, base) {
            Ok(t) => Check {
                ok: true,
                what: format!(
                    "the robot is at ({:.2}, {:.2}) in {map}",
                    t.translation[0], t.translation[1]
                ),
            },
            Err(e) => Check {
                ok: false,
                what: format!("no {map} to {base} transform: localization is not running ({e})"),
            },
        });
        if let Some(m) = &self.profile.mission {
            let state = self
                .robot
                .latest_fresh(
                    &m.state,
                    "nervros_interfaces/msg/RobotState",
                    RATE_WINDOW,
                    crate::mission::STATE_FRESH,
                )
                .await;
            out.push(match state {
                Ok(s) => {
                    let said = s["message"]
                        .as_str()
                        .filter(|m| !m.is_empty())
                        .map(|m| format!(": {m}"))
                        .unwrap_or_default();
                    let what = match s["mission_id"].as_str().filter(|id| !id.is_empty()) {
                        Some(id) => format!("the executor runs mission {id}{said}"),
                        None => format!("the executor is idle{said}"),
                    };
                    Check { ok: true, what }
                }
                Err(e) => Check {
                    ok: false,
                    what: format!("no executor state on {} ({e})", m.state),
                },
            });
        }
        out
    }
}

#[async_trait]
impl Tool for HealthCheck {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, _args: Value) -> ToolOutcome {
        let checks = self.checks().await;
        let problems: Vec<&str> = checks
            .iter()
            .filter(|c| !c.ok)
            .map(|c| c.what.as_str())
            .collect();
        let fine: Vec<&str> = checks
            .iter()
            .filter(|c| c.ok)
            .map(|c| c.what.as_str())
            .collect();
        let mut out = ToolOutcome::ok(
            json!({"healthy": problems.is_empty(), "problems": problems, "fine": fine}),
        );
        out.message = if problems.is_empty() {
            format!("all {} checks pass", checks.len())
        } else {
            format!("{} of {} checks fail", problems.len(), checks.len())
        };
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nervros_ros::fake::FakeRobot;

    #[tokio::test]
    async fn a_missing_camera_fails_its_line() {
        let profile = Profile::from_toml(
            r#"
            [robot]
            name = "t"
            [look]
            image = "/cam"
            detections = { topic = "/det", type = "canopy_msgs/msg/InstanceMaskArray" }
            [models]
            file = "m.toml"
            "#,
            std::path::Path::new("nervros.toml"),
        )
        .unwrap();
        let robot = FakeRobot::new().with_topic("/det", serde_json::json!({}));
        let checks = run(&profile, &robot).await;
        let line = |w: &str| checks.iter().find(|c| c.what.contains(w)).unwrap().ok;
        assert!(!line("/cam"));
        assert!(line("/det"));
    }
}
