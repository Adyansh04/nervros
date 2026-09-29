//! Assembles a running agent from a profile and a robot port: the model layer, schemas, guard,
//! tools and the session. The CLI and the GUI both start here.

use std::path::Path;
use std::sync::Arc;

use nervros_ros::RobotPort;

use crate::builtins::{ListPlaces, RobotState, Stop};
use crate::context::system_prompt;
use crate::guard::Guard;
use crate::llm::Llm;
use crate::look::{LookTool, SnapshotStore};
use crate::profile::{PrivacyModeConfig, Profile};
use crate::providers::ModelsConfig;
use crate::providers::router::{PrivacyMode, Router};
use crate::schemas::RosidlSchemas;
use crate::session::{Session, SessionConfig};
use crate::tools::{Registry, Tool};

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
    let schemas = RosidlSchemas::load(&profile)?;
    let mut registry = Registry::from_config(&profile.tools, &robot, &schemas, &guard)?;
    let snapshots = Arc::new(SnapshotStore::default());
    if let Some(look) = profile.look.clone() {
        let tool = LookTool::start(look, Arc::clone(&robot), Arc::clone(&snapshots))
            .map_err(StartError::Look)?;
        registry.add(Arc::new(tool))?;
    }
    registry.add(Arc::new(ListPlaces::new(&profile, Arc::clone(&robot))))?;
    registry.add(Arc::new(RobotState::new(
        &profile,
        Arc::clone(&robot),
        Arc::clone(&guard),
    )))?;
    let stop: Arc<dyn Tool> = Arc::new(Stop::new(&profile, Arc::clone(&robot)));
    registry.add(Arc::clone(&stop))?;
    let tools = registry.iter().map(|t| t.spec().name.clone()).collect();
    let config = SessionConfig {
        preamble: system_prompt(&profile),
        max_model_calls: usize::try_from(profile.policy.budgets.model_calls).unwrap_or(6),
        approval_ttl: profile.policy.approval_ttl,
        ..SessionConfig::default()
    };
    let session = Session::start(
        Arc::clone(&llm) as Arc<dyn crate::llm::AgentSource>,
        Arc::new(registry),
        Arc::clone(&guard),
        Some(stop),
        config,
    );
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
