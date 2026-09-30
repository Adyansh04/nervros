//! The robot profile: one `nervros.toml` per robot, kept in the robot's own repository.
//!
//! It names the robot, how to reach its ROS graph, the policy, the places, the camera and detection
//! topics `look` reads, the mission executor's interfaces, the tools and the models file. Paths are
//! relative to the profile's directory.

use std::path::{Path, PathBuf};
use std::time::Duration;

use serde::{Deserialize, Deserializer};

use crate::guard::Policy;
use crate::tools::ToolConfig;

/// A whole profile.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Profile {
    /// Who the robot is.
    pub robot: RobotConfig,
    /// How to reach its graph.
    #[serde(default)]
    pub ros: RosConfig,
    /// Where camera images may go.
    #[serde(default)]
    pub privacy: PrivacyConfig,
    /// What the agent may do.
    #[serde(default)]
    pub policy: Policy,
    /// Inputs of the `look` builtin.
    pub look: Option<LookConfig>,
    /// The robot's mission executor.
    pub mission: Option<MissionConfig>,
    /// Named places beyond the world model's rooms.
    #[serde(rename = "place", default)]
    pub places: Vec<PlaceConfig>,
    /// Tools declared by ROS name and type.
    #[serde(rename = "tool", default)]
    pub tools: Vec<ToolConfig>,
    /// The world model's topics, when one runs.
    pub world: Option<WorldConfig>,
    /// What the viewer draws beyond the world model.
    pub viz: Option<VizConfig>,
    /// The models file, relative to the profile.
    pub models: ModelsRef,
    #[serde(skip)]
    dir: PathBuf,
}

/// Who the robot is.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotConfig {
    /// Display name.
    pub name: String,
    /// A markdown file describing the robot, its abilities and limits, for the system prompt.
    pub persona: Option<PathBuf>,
}

/// How the agent joins the ROS graph.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RosConfig {
    /// `ROS_DOMAIN_ID`.
    #[serde(default)]
    pub domain_id: u32,
    /// `udp` turns off Fast DDS shared memory, for a graph run by another user.
    #[serde(default)]
    pub transport: Transport,
    /// The agent's node name.
    #[serde(default = "default_node_name")]
    pub node_name: String,
    /// How long start-up waits for the graph.
    #[serde(default = "default_discovery", deserialize_with = "duration")]
    pub discovery_timeout: Duration,
    /// Directories holding interface packages, for tool schemas.
    #[serde(default)]
    pub interfaces: Vec<PathBuf>,
    /// Path prefixes to rewrite when install symlinks point at another machine's paths.
    #[serde(default)]
    pub remap: std::collections::BTreeMap<PathBuf, PathBuf>,
    /// The fixed frame places and poses are in.
    #[serde(default = "default_map_frame")]
    pub map_frame: String,
    /// The robot's base frame.
    #[serde(default = "default_base_frame")]
    pub base_frame: String,
}

fn default_map_frame() -> String {
    "map".to_owned()
}

fn default_base_frame() -> String {
    "base_footprint".to_owned()
}

impl Default for RosConfig {
    fn default() -> Self {
        Self {
            domain_id: 0,
            transport: Transport::default(),
            node_name: default_node_name(),
            discovery_timeout: default_discovery(),
            interfaces: Vec::new(),
            remap: std::collections::BTreeMap::new(),
            map_frame: default_map_frame(),
            base_frame: default_base_frame(),
        }
    }
}

fn default_node_name() -> String {
    "nervros".to_owned()
}

fn default_discovery() -> Duration {
    Duration::from_secs(20)
}

/// DDS transports.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Transport {
    /// The middleware's defaults.
    #[default]
    Default,
    /// UDP only.
    Udp,
}

/// The privacy mode, see [`crate::providers::router::PrivacyMode`].
#[derive(Debug, Clone, Copy, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PrivacyConfig {
    /// `sim` or `home`.
    #[serde(default)]
    pub mode: PrivacyModeConfig,
}

/// Serialised form of the privacy mode.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PrivacyModeConfig {
    /// Simulated frames.
    #[default]
    Sim,
    /// Real frames stay local.
    Home,
}

/// Where `look` reads from.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LookConfig {
    /// The colour image topic.
    pub image: String,
    /// The detection topic and its type.
    pub detections: TopicRef,
    /// At most this many marks.
    #[serde(default = "default_marks")]
    pub max_marks: usize,
    /// Detections older than this are ignored.
    #[serde(default = "default_look_age", deserialize_with = "duration")]
    pub max_age: Duration,
    /// What the vision model should know about this camera, such as where it points and how far
    /// it sees, so it does not take a view of the floor for an empty room.
    #[serde(default)]
    pub about: Option<String>,
}

fn default_marks() -> usize {
    12
}

fn default_look_age() -> Duration {
    Duration::from_secs(5)
}

/// A topic and its message type.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TopicRef {
    /// Topic name.
    pub topic: String,
    /// `pkg/msg/Name`.
    #[serde(rename = "type")]
    pub msg_type: String,
}

/// The robot's mission executor (see `nervros_interfaces`).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MissionConfig {
    /// `ExecuteMission` action.
    pub execute: String,
    /// `ValidateMission` service.
    pub validate: String,
    /// `GetCatalog` service.
    pub catalog: String,
    /// `StopAll` service.
    pub stop: String,
    /// `RobotState` topic.
    pub state: String,
    /// Replans allowed after a failed mission.
    #[serde(default = "default_replans")]
    pub max_replans: u32,
}

fn default_replans() -> u32 {
    2
}

/// The world model's topics.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorldConfig {
    /// The occupancy grid (`nav_msgs/msg/OccupancyGrid`), drawn in the viewer.
    pub map: Option<TopicRef>,
    /// Rooms (`canopy_msgs/msg/RoomArray` or compatible).
    pub rooms: Option<TopicRef>,
    /// Objects (`canopy_msgs/msg/WorldObjectArray` or compatible).
    pub objects: Option<TopicRef>,
    /// What the camera has seen, as an occupancy grid (canopy's: 0 seen, 90 still to see, 99
    /// written off), drawn over the map while the building is explored.
    pub coverage: Option<TopicRef>,
    /// Where the robot has been (`nav_msgs/msg/Path`), drawn as a line.
    pub trail: Option<TopicRef>,
}

/// What the viewer draws beyond the world model.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VizConfig {
    /// The path the navigation stack follows (`nav_msgs/msg/Path`, Nav2's `/plan`).
    pub plan: Option<TopicRef>,
    /// The `[look]` camera's `sensor_msgs/msg/CameraInfo`: with it, the frame is drawn in the world
    /// where the camera is, as a frustum, placed by TF from the base frame to the image's frame.
    pub camera_info: Option<TopicRef>,
    /// The robot's URDF, relative to the profile: the viewer draws the robot with it, posed by TF.
    /// `package://` meshes are found through `ROS_PACKAGE_PATH` or `AMENT_PREFIX_PATH`.
    pub urdf: Option<PathBuf>,
}

/// A named place.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlaceConfig {
    /// The id plans use, such as `zone_a`.
    pub name: String,
    /// Other names people use for it.
    #[serde(default)]
    pub aliases: Vec<String>,
    /// The pose's frame.
    #[serde(default = "default_frame")]
    pub frame: String,
    /// Where the robot stands.
    pub pose: PlacePose,
}

fn default_frame() -> String {
    "map".to_owned()
}

/// A planar pose.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlacePose {
    /// Metres.
    pub x: f64,
    /// Metres.
    pub y: f64,
    /// Radians.
    #[serde(default)]
    pub yaw: f64,
}

/// The `[models]` table.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelsRef {
    /// Path to `models.toml`.
    pub file: PathBuf,
}

/// A profile that cannot be used.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ProfileError {
    /// The file could not be read.
    #[error("cannot read {path}: {source}")]
    Read {
        /// The file.
        path: PathBuf,
        /// The underlying error.
        source: std::io::Error,
    },
    /// The TOML does not match the schema.
    #[error("invalid profile {path}: {source}")]
    Parse {
        /// The file.
        path: PathBuf,
        /// The parse error.
        source: toml::de::Error,
    },
    /// Two tools or two places share a name.
    #[error("duplicate {kind} `{name}`")]
    Duplicate {
        /// `tool` or `place`.
        kind: &'static str,
        /// The repeated name.
        name: String,
    },
}

impl Profile {
    /// Loads and checks a profile.
    ///
    /// # Errors
    ///
    /// Unreadable files, schema errors and duplicate names.
    pub fn load(path: &Path) -> Result<Self, ProfileError> {
        let text = std::fs::read_to_string(path).map_err(|source| ProfileError::Read {
            path: path.to_path_buf(),
            source,
        })?;
        let mut profile: Self = toml::from_str(&text).map_err(|source| ProfileError::Parse {
            path: path.to_path_buf(),
            source,
        })?;
        profile.dir = path.parent().map(Path::to_path_buf).unwrap_or_default();
        profile.check()?;
        Ok(profile)
    }

    fn check(&self) -> Result<(), ProfileError> {
        let mut seen = std::collections::BTreeSet::new();
        for t in &self.tools {
            if !seen.insert(t.name.as_str()) {
                return Err(ProfileError::Duplicate {
                    kind: "tool",
                    name: t.name.clone(),
                });
            }
        }
        let mut seen = std::collections::BTreeSet::new();
        for p in &self.places {
            if !seen.insert(p.name.as_str()) {
                return Err(ProfileError::Duplicate {
                    kind: "place",
                    name: p.name.clone(),
                });
            }
        }
        Ok(())
    }

    /// A path from the profile, resolved against its directory; `~/` means home.
    #[must_use]
    pub fn resolve(&self, path: &Path) -> PathBuf {
        let path = crate::secret::expand_home(path);
        if path.is_absolute() {
            path
        } else {
            self.dir.join(path)
        }
    }
}

/// Reads `"5s"`, `"250ms"`, `"2m"` or a bare number of seconds.
///
/// # Errors
///
/// Any other text.
pub fn duration<'de, D: Deserializer<'de>>(de: D) -> Result<Duration, D::Error> {
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum Raw {
        Secs(f64),
        Text(String),
    }
    let secs = match Raw::deserialize(de)? {
        Raw::Secs(s) => s,
        Raw::Text(t) => parse_duration(&t)
            .ok_or_else(|| serde::de::Error::custom(format!("bad duration `{t}`")))?,
    };
    Duration::try_from_secs_f64(secs).map_err(serde::de::Error::custom)
}

fn parse_duration(text: &str) -> Option<f64> {
    let text = text.trim();
    let (number, scale) = if let Some(n) = text.strip_suffix("ms") {
        (n, 0.001)
    } else if let Some(n) = text.strip_suffix('s') {
        (n, 1.0)
    } else if let Some(n) = text.strip_suffix('m') {
        (n, 60.0)
    } else {
        (text, 1.0)
    };
    number
        .trim()
        .parse::<f64>()
        .ok()
        .filter(|v| v.is_finite() && *v >= 0.0)
        .map(|v| v * scale)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_durations() {
        assert_eq!(parse_duration("5s"), Some(5.0));
        assert_eq!(parse_duration("250ms"), Some(0.25));
        assert_eq!(parse_duration("2m"), Some(120.0));
        assert_eq!(parse_duration("1.5"), Some(1.5));
        assert_eq!(parse_duration("-1s"), None);
        assert_eq!(parse_duration("soon"), None);
    }

    #[test]
    fn the_example_profile_loads() {
        let path =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../profiles/example/nervros.toml");
        let profile = Profile::load(&path).unwrap();
        assert_eq!(profile.robot.name, "Example robot");
        assert!(
            profile
                .resolve(&profile.models.file)
                .ends_with("profiles/example/models.toml")
        );
        assert!(!profile.tools.is_empty());
    }
}
