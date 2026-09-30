//! The connection check: whether the robot offers everything the profile names. The CLI prints
//! it and the GUI shows it on its Doctor tab.

use std::time::Duration;

use nervros_ros::RobotPort;

use crate::profile::Profile;
use crate::tools::ToolKind;

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
    match crate::schemas::RosidlSchemas::load(profile) {
        Ok(s) => report(
            !s.is_empty() || profile.tools.is_empty(),
            format!("{} interface definitions loaded", s.len()),
        ),
        Err(e) => report(false, format!("interface files: {e}")),
    }
    for t in &profile.tools {
        let ok = match t.kind {
            ToolKind::Service => {
                robot
                    .service_available(&t.ros_name, &t.ros_type, SERVICE_WAIT)
                    .await
            }
            ToolKind::Topic => has_topic(&t.ros_name),
        };
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
        for (what, name, ty) in [
            (
                "validate",
                &m.validate,
                "nervros_interfaces/srv/ValidateMission",
            ),
            ("catalog", &m.catalog, "nervros_interfaces/srv/GetCatalog"),
            ("stop", &m.stop, "nervros_interfaces/srv/StopAll"),
        ] {
            let ok = robot.service_available(name, ty, SERVICE_WAIT).await;
            report(ok, format!("mission {what} {name}"));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use nervros_ros::fake::FakeRobot;

    #[tokio::test]
    async fn a_missing_camera_fails_its_line() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("nervros.toml");
        std::fs::write(
            &path,
            r#"
            [robot]
            name = "t"
            [look]
            image = "/cam"
            detections = { topic = "/det", type = "canopy_msgs/msg/InstanceMaskArray" }
            [models]
            file = "m.toml"
            "#,
        )
        .unwrap();
        let profile = Profile::load(&path).unwrap();
        let robot = FakeRobot::new().with_topic("/det", serde_json::json!({}));
        let checks = run(&profile, &robot).await;
        let line = |w: &str| checks.iter().find(|c| c.what.contains(w)).unwrap().ok;
        assert!(!line("/cam"));
        assert!(line("/det"));
    }
}
