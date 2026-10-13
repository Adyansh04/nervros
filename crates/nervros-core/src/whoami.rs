//! The robot's self-description: facts gathered from the profile, the live graph, the mission
//! executor's catalog, the URDF and the world model, and a persona drafted from them by the model
//! for the operator to read, edit and keep. A draft is written beside the persona and never over
//! it: what the robot is told about itself stays the operator's choice.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::Duration;

use nervros_ros::RobotPort;
use serde_json::{Value, json};

use crate::llm::{Ask, Llm, LlmError};
use crate::profile::Profile;
use crate::providers::Role;

/// What a sensor type is called in the facts.
const SENSORS: [(&str, &str); 8] = [
    ("sensor_msgs/msg/LaserScan", "laser scans"),
    ("sensor_msgs/msg/PointCloud2", "point clouds"),
    ("sensor_msgs/msg/Image", "camera images"),
    ("sensor_msgs/msg/Imu", "IMU"),
    ("nav_msgs/msg/Odometry", "odometry"),
    ("sensor_msgs/msg/NavSatFix", "GPS"),
    ("sensor_msgs/msg/JointState", "joint states"),
    ("sensor_msgs/msg/BatteryState", "battery"),
];
/// Stacks known by a node they always run.
const STACKS: [(&str, &str); 6] = [
    ("/bt_navigator", "Nav2 navigation"),
    ("/amcl", "AMCL localisation on a saved map"),
    ("/slam_toolbox", "SLAM Toolbox mapping"),
    ("/move_group", "MoveIt arm planning"),
    ("/controller_manager", "ros2_control"),
    (
        "/robot_state_publisher",
        "a robot description (URDF) on /robot_description",
    ),
];
/// At most this many names of one kind, so a big graph stays a short prompt.
const NAMES_MAX: usize = 12;
const CALL: Duration = Duration::from_secs(3);

const PREAMBLE: &str = "You write a robot's self-description: the part of its assistant's \
    system prompt that says what the robot is, what it senses, what it can do and where. Use only \
    the facts given and leave out what they do not say. Write as the example does when there is \
    one: a first line addressed to the assistant ('You are the assistant of ...'), then short \
    bullets of plain facts the assistant needs, the topic and skill names exact. No headings, no \
    filler, under 30 lines.";

/// A URDF's body in short: its links, its joints by type and the names of those that move.
#[must_use]
pub(crate) fn body(urdf: &str) -> Value {
    let mut by_type: BTreeMap<String, usize> = BTreeMap::new();
    let mut moving = Vec::new();
    for tag in urdf.split("<joint").skip(1) {
        let head = tag.split('>').next().unwrap_or_default();
        let (Some(name), Some(kind)) = (attribute(head, "name"), attribute(head, "type")) else {
            continue;
        };
        *by_type.entry(kind.to_owned()).or_default() += 1;
        if kind != "fixed" {
            moving.push(name.to_owned());
        }
    }
    let links = urdf.matches("<link").count();
    json!({"links": links, "joints": by_type, "moving_joints": moving})
}

/// The value of `name="…"` in a tag's head.
fn attribute<'a>(head: &'a str, name: &str) -> Option<&'a str> {
    let at = head.find(&format!(" {name}=\""))? + name.len() + 3;
    head[at..].split('"').next()
}

/// Everything the robot can tell about itself now. A part it cannot read is left out.
pub async fn facts(profile: &Profile, robot: &dyn RobotPort) -> Value {
    let mut facts = json!({"robot": profile.robot.name});
    if let Some(look) = &profile.look {
        let mut cameras =
            vec![json!({"name": look.name, "image": look.image, "about": look.about})];
        for (name, c) in &look.cameras {
            cameras.push(json!({"name": name, "image": c.image, "about": c.about}));
        }
        facts["cameras"] = json!(cameras);
    }
    if !profile.places.is_empty() {
        facts["places"] = json!(profile.places.iter().map(|p| &p.name).collect::<Vec<_>>());
    }
    if !profile.tools.is_empty() {
        facts["robot_tools"] = json!(
            profile
                .tools
                .iter()
                .map(|t| json!({"name": t.name, "description": t.description}))
                .collect::<Vec<_>>()
        );
    }
    if let Some(urdf) = profile.viz.as_ref().and_then(|v| v.urdf.as_ref())
        && let Ok(text) = std::fs::read_to_string(profile.resolve(urdf))
    {
        facts["body"] = body(&text);
    }
    if let Ok(graph) = robot.graph_detail().await {
        let mut sensors: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
        for (topic, types) in &graph.topics {
            for (ty, kind) in SENSORS {
                if types.iter().any(|t| t == ty) {
                    let list = sensors.entry(kind).or_default();
                    if list.len() < NAMES_MAX {
                        list.push(topic);
                    }
                }
            }
        }
        facts["sensors"] = json!(sensors);
        let stacks: Vec<&str> = STACKS
            .iter()
            .filter(|(node, _)| graph.nodes.iter().any(|n| n == node))
            .map(|(_, what)| *what)
            .collect();
        facts["software"] = json!(stacks);
        facts["graph"] = json!({"topics": graph.topics.len(), "nodes": graph.nodes.len()});
    }
    if let Some(m) = &profile.mission
        && let Ok(out) = robot
            .call(
                &m.catalog,
                "nervros_interfaces/srv/GetCatalog",
                json!({}),
                CALL,
            )
            .await
        && let Some(catalog) = out["catalog_json"]
            .as_str()
            .and_then(|t| crate::mission::catalog::Catalog::parse(t).ok())
    {
        // As the planner shows them to the model: the skills it may name, with what it gives.
        facts["skills"] = json!(catalog.describe());
    }
    if let Some(rooms) = profile.world.as_ref().and_then(|w| w.rooms.as_ref())
        && let Ok(message) = robot.latest(&rooms.topic, &rooms.msg_type, CALL).await
    {
        facts["rooms"] = json!(
            message["rooms"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(crate::places::room_label)
                .take(NAMES_MAX * 2)
                .collect::<Vec<_>>()
        );
    }
    facts
}

/// Where the draft goes: beside the persona as `<persona>.draft.md`, or beside the profile.
#[must_use]
pub(crate) fn draft_path(profile: &Profile) -> PathBuf {
    match &profile.robot.persona {
        Some(persona) => profile.resolve(persona).with_extension("draft.md"),
        None => profile.resolve(std::path::Path::new("persona.draft.md")),
    }
}

/// A persona drafted from `facts`, in the style of the profile's own persona when it has one.
///
/// # Errors
///
/// No model of the routine role could answer.
pub async fn draft(llm: &Llm, profile: &Profile, facts: &Value) -> Result<String, LlmError> {
    let example = profile
        .robot
        .persona
        .as_ref()
        .and_then(|p| std::fs::read_to_string(profile.resolve(p)).ok());
    let style = example.map_or_else(String::new, |e| {
        format!(
            "\nThe robot's current self-description, for its style; keep what it says that the \
             facts cannot tell, such as the operator's own rules:\n{e}\n"
        )
    });
    let prompt = format!(
        "Facts about the robot, as JSON:\n{}\n{style}",
        serde_json::to_string_pretty(facts).unwrap_or_default()
    );
    let answer = llm
        .ask(Ask {
            role: Role::Routine,
            preamble: PREAMBLE,
            prompt: &prompt,
            image: None,
        })
        .await?;
    Ok(answer.text)
}

/// Writes a draft to [`draft_path`]; returns where.
///
/// # Errors
///
/// The file could not be written.
pub fn save(profile: &Profile, text: &str) -> Result<PathBuf, String> {
    let path = draft_path(profile);
    let text = text.trim().to_owned() + "\n";
    crate::persist::write_atomic(&path, text.as_bytes())
        .map_err(|e| format!("writing {}: {e}", path.display()))?;
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn the_skills_are_those_the_planner_lets_the_model_name() {
        let profile = Profile::from_toml(
            "[robot]\nname = \"t\"\n[models]\nfile = \"m.toml\"\n\
             [mission]\nexecute = \"/x/execute\"\nvalidate = \"/x/validate\"\n\
             catalog = \"/x/catalog\"\nstop = \"/x/stop\"\nstate = \"/x/state\"\n",
            std::path::Path::new("nervros.toml"),
        )
        .unwrap();
        let robot = nervros_ros::fake::FakeRobot::new().with_service("/x/catalog", |_| {
            Ok(json!({"catalog_json": crate::mission::catalog::tests::CATALOG}))
        });
        let facts = facts(&profile, &robot).await;
        let skills = facts["skills"].as_str().unwrap_or_default();
        assert!(skills.contains("GoToPlace(place)"), "{skills}");
        assert!(!skills.contains("GoToPose"), "{skills}");
    }

    #[test]
    fn a_urdf_comes_down_to_its_links_and_moving_joints() {
        let urdf = r#"<robot name="g1">
            <link name="pelvis"/><link name="left_hip"/><link name="head"/>
            <joint name="left_hip_pitch_joint" type="revolute"><parent link="pelvis"/></joint>
            <joint name="head_joint" type="fixed"><parent link="pelvis"/></joint>
            <joint name="wheel" type="continuous"></joint>
        </robot>"#;
        let summary = body(urdf);
        assert_eq!(summary["links"], 3);
        assert_eq!(
            summary["joints"],
            json!({"continuous": 1, "fixed": 1, "revolute": 1})
        );
        assert_eq!(
            summary["moving_joints"],
            json!(["left_hip_pitch_joint", "wheel"])
        );
    }

    #[test]
    fn a_draft_goes_beside_the_persona_never_over_it() {
        let mut profile: Profile =
            toml::from_str("[robot]\nname = \"x\"\n[models]\nfile = \"m.toml\"\n").unwrap();
        profile.robot.persona = Some("apartment.md".into());
        let path = draft_path(&profile);
        assert!(path.ends_with("apartment.draft.md"), "{}", path.display());
    }
}
