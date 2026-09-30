//! Assembles a running agent from a profile and a robot port: the model layer, schemas, guard,
//! tools and the session. The CLI and the GUI both start here.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use nervros_ros::RobotPort;

use crate::builtins::{ListPlaces, RobotState, Stop};
use crate::context::system_prompt;
use crate::guard::Guard;
use crate::llm::Llm;
use crate::look::{LookTool, SnapshotStore};
use crate::mission::Missions;
use crate::profile::{PrivacyModeConfig, Profile};
use crate::providers::ModelsConfig;
use crate::providers::router::{PrivacyMode, Router};
use crate::schemas::RosidlSchemas;
use crate::session::{Session, SessionConfig};
use crate::tools::{Registry, SchemaSource, Tool};

/// A running agent.
pub struct Agent {
    /// The conversation.
    pub session: Session,
    /// The profile it runs.
    pub profile: Profile,
    /// The guard, for arming state.
    pub guard: Arc<Guard>,
    /// The robot.
    pub robot: Arc<dyn RobotPort>,
    /// Marked images by snapshot id.
    pub snapshots: Arc<SnapshotStore>,
    /// The model layer, for quota display.
    pub llm: Arc<Llm>,
    /// Tool names, in order.
    pub tools: Vec<String>,
}

impl std::fmt::Debug for Agent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Agent")
            .field("robot", &self.profile.robot.name)
            .field("tools", &self.tools)
            .finish_non_exhaustive()
    }
}

/// Why an agent could not start.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum StartError {
    /// The profile.
    #[error(transparent)]
    Profile(#[from] crate::profile::ProfileError),
    /// The models file.
    #[error(transparent)]
    Models(#[from] crate::providers::ConfigError),
    /// The quota ledger.
    #[error("quota ledger: {0}")]
    Ledger(std::io::Error),
    /// Provider keys.
    #[error(transparent)]
    Llm(#[from] crate::llm::LlmError),
    /// Interface files.
    #[error("interface files: {0}")]
    Schemas(#[from] rosidl_schema::Error),
    /// A tool.
    #[error(transparent)]
    Tool(#[from] crate::tools::RegistryError),
    /// The camera subscription for `look`.
    #[error("look: {0}")]
    Look(nervros_ros::RosError),
    /// The ROS environment does not match the profile.
    #[error("{0}")]
    Environment(String),
    /// The ROS node.
    #[error("starting the ROS node: {0}")]
    Ros(nervros_ros::RosError),
}

/// Where the quota ledger and session logs live: `$XDG_STATE_HOME/nervros`, else
/// `~/.local/state/nervros`.
#[must_use]
pub fn state_dir() -> PathBuf {
    std::env::var_os("XDG_STATE_HOME").map_or_else(
        || crate::secret::expand_home(Path::new("~/.local/state/nervros")),
        |d| PathBuf::from(d).join("nervros"),
    )
}

/// Starts the ROS node after checking the environment matches the profile; the environment
/// decides, because ROS reads it when the node starts.
///
/// # Errors
///
/// [`StartError::Environment`] on a mismatch, [`StartError::Ros`] if the node cannot start.
#[cfg(feature = "rcl")]
pub fn connect(profile: &Profile) -> Result<Arc<dyn RobotPort>, StartError> {
    use crate::profile::Transport;
    let domain = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|d| d.parse::<u32>().ok())
        .unwrap_or(0);
    if domain != profile.ros.domain_id {
        return Err(StartError::Environment(format!(
            "ROS_DOMAIN_ID is {domain} but the profile says {}; export it before starting",
            profile.ros.domain_id
        )));
    }
    let udp = std::env::var("FASTDDS_BUILTIN_TRANSPORTS").is_ok_and(|t| t == "UDPv4");
    if profile.ros.transport == Transport::Udp && !udp {
        return Err(StartError::Environment(
            "the profile wants UDP only; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
             (NERVROS_UDP_ONLY=1 with scripts/ros-env.sh)"
                .to_owned(),
        ));
    }
    let config = nervros_ros::R2rConfig {
        node_name: profile.ros.node_name.clone(),
        ..nervros_ros::R2rConfig::default()
    };
    Ok(Arc::new(
        nervros_ros::R2rPort::start(config).map_err(StartError::Ros)?,
    ))
}

/// How long start-up waits for the executor's latched `RobotState`.
const ORPHAN_WAIT: std::time::Duration = std::time::Duration::from_secs(5);

/// What to tell the operator when the robot is running a mission this session did not start.
fn orphan_notice(state: &serde_json::Value) -> Option<String> {
    let id = state["mission_id"].as_str().filter(|id| !id.is_empty())?;
    let step = state["mission_step"]
        .as_str()
        .filter(|s| !s.is_empty())
        .map_or_else(String::new, |s| format!(", at {s}"));
    Some(format!(
        "The robot is running mission {id}{step}, started before this session. It carries on \
         unwatched until it ends; Stop ends it now."
    ))
}

/// Starts an agent. Must run inside a tokio runtime.
///
/// # Errors
///
/// Anything in [`StartError`].
pub fn start(
    profile_path: &Path,
    robot: Arc<dyn RobotPort>,
    ledger: &Path,
) -> Result<Agent, StartError> {
    let profile = Profile::load(profile_path)?;
    let models = ModelsConfig::load(&profile.resolve(&profile.models.file))?;
    let privacy = match profile.privacy.mode {
        PrivacyModeConfig::Sim => PrivacyMode::Sim,
        PrivacyModeConfig::Home => PrivacyMode::Home,
    };
    let router = Router::with_ledger_file(models, ledger, privacy).map_err(StartError::Ledger)?;
    let llm = Arc::new(Llm::new(router)?);
    let guard = Arc::new(Guard::new(profile.policy.clone()));
    let schemas: Arc<dyn SchemaSource> = Arc::new(RosidlSchemas::load(&profile)?);
    let mut registry = Registry::from_config(&profile.tools, &robot, &schemas, &guard)?;
    let snapshots = Arc::new(SnapshotStore::default());
    if let Some(look) = profile.look.clone() {
        let eyes: Arc<dyn crate::look::Eyes> = Arc::clone(&llm) as Arc<dyn crate::look::Eyes>;
        let tool = LookTool::start(look, Arc::clone(&robot), Arc::clone(&snapshots), Some(eyes))
            .map_err(StartError::Look)?;
        registry.add(Arc::new(tool))?;
    }
    registry.add(Arc::new(ListPlaces::new(&profile, Arc::clone(&robot))))?;
    registry.add(Arc::new(RobotState::new(
        &profile,
        Arc::clone(&robot),
        Arc::clone(&guard),
    )))?;
    if let Some(config) = &profile.ros_tools {
        for tool in crate::ros_tools::tools(config, &robot, &schemas, &guard) {
            registry.add(tool)?;
        }
    }
    let stop: Arc<dyn Tool> = Arc::new(Stop::new(&profile, Arc::clone(&robot)));
    registry.add(Arc::clone(&stop))?;
    let missions = Missions::new(&profile, Arc::clone(&robot));
    if let Some(m) = &missions {
        for tool in m.tools() {
            registry.add(tool)?;
        }
    }
    let tools = registry.iter().map(|t| t.spec().name.clone()).collect();
    let config = SessionConfig {
        preamble: system_prompt(&profile),
        max_model_calls: usize::try_from(profile.policy.budgets.model_calls).unwrap_or(6),
        approval_ttl: profile.policy.approval_ttl,
        turn_time: profile.policy.budgets.wall_time,
        ..SessionConfig::default()
    };
    let session = Session::start(
        Arc::clone(&llm) as Arc<dyn crate::llm::AgentSource>,
        Arc::new(registry),
        Arc::clone(&guard),
        Some(stop),
        config,
    );
    if let Some(m) = &missions {
        m.attach(session.handle());
    }
    if let Some(state) = profile.mission.as_ref().map(|m| m.state.clone()) {
        // A mission started before this session runs on unwatched: say so, once, at start.
        let (robot, handle) = (Arc::clone(&robot), session.handle());
        tokio::spawn(async move {
            let latched = robot
                .latest(&state, "nervros_interfaces/msg/RobotState", ORPHAN_WAIT)
                .await;
            if let Some(text) = latched.ok().as_ref().and_then(orphan_notice) {
                handle.emit(crate::session::Event::Notice { text });
            }
        });
    }
    Ok(Agent {
        session,
        profile,
        guard,
        robot,
        snapshots,
        llm,
        tools,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mission_left_running_is_told_and_an_idle_robot_is_not() {
        let running = serde_json::json!({"mission_id": "m7", "mission_step": "s2_GoToPlace"});
        let text = orphan_notice(&running).unwrap();
        assert!(text.contains("m7, at s2_GoToPlace"), "{text}");
        assert!(orphan_notice(&serde_json::json!({"mission_id": ""})).is_none());
    }
}
