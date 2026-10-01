//! Assembles a running agent from a profile and a robot port: the model layer, schemas, guard,
//! tools and the session. The CLI and the GUI both start here.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use nervros_ros::RobotPort;

use crate::builtins::{ListPlaces, RobotState, Stop};
use crate::context::system_prompt;
use crate::guard::Guard;
use crate::llm::Llm;
use crate::look::{Cameras, LookTool, SnapshotStore};
use crate::mission::Missions;
use crate::profile::{PrivacyModeConfig, Profile};
use crate::providers::ModelsConfig;
use crate::providers::router::{PrivacyMode, Router};
use crate::schemas::RosidlSchemas;
use crate::segment::SegmentTool;
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
    /// The world editor, when the profile names one.
    pub editor: Option<Arc<crate::editor::EditorClient>>,
    /// What the operator asked to remember.
    pub memory: Arc<crate::memory::Memory>,
    /// Missions run again and again, when the robot has an executor.
    pub schedules: Option<Arc<crate::schedule::Schedules>>,
    /// Missions, when the robot has an executor: their ledger is the window's history.
    pub missions: Option<Arc<Missions>>,
    /// What the MCP servers brought, for the doctor.
    pub mcp: Vec<crate::doctor::Check>,
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

/// What seeing set up beyond its tools: the world editor's client, for the app, and the camera
/// check of a mission's end, for the missions.
struct Seeing {
    editor: Option<Arc<crate::editor::EditorClient>>,
    vision: Option<crate::mission::camera::Vision>,
}

/// `look`, `point`, `segment` and the world editor's tools, as the profile has them.
fn seeing_tools(
    profile: &Profile,
    robot: &Arc<dyn RobotPort>,
    llm: &Arc<Llm>,
    snapshots: &Arc<SnapshotStore>,
    registry: &mut Registry,
) -> Result<Seeing, StartError> {
    let robot = Arc::clone(robot);
    let snapshots = Arc::clone(snapshots);
    let llm = Arc::clone(llm);
    let mut seen_by = None;
    if let Some(look) = profile.look.clone() {
        let cameras = Cameras::start(&look, &robot).map_err(StartError::Look)?;
        seen_by = Some(Arc::clone(&cameras));
        let eyes: Arc<dyn crate::look::Eyes> = Arc::clone(&llm) as Arc<dyn crate::look::Eyes>;
        registry.add(Arc::new(LookTool::new(
            look,
            Arc::clone(&cameras),
            Arc::clone(&robot),
            Arc::clone(&snapshots),
            Some(eyes),
        )))?;
        // Pointing asks the models that outline, a skill few models have.
        if !llm.router().config().roles.segment.is_empty() {
            let pointer = Arc::clone(&llm) as Arc<dyn crate::segment::Outliner>;
            registry.add(Arc::new(crate::point::PointTool::new(
                Arc::clone(&cameras),
                Arc::clone(&robot),
                Arc::clone(&snapshots),
                pointer,
            )))?;
        }
        if let Some(segment) = profile.segment.clone() {
            let outliner = Arc::clone(&llm) as Arc<dyn crate::segment::Outliner>;
            registry.add(Arc::new(SegmentTool::new(
                segment,
                cameras,
                Arc::clone(&robot),
                Arc::clone(&snapshots),
                Some(outliner),
            )))?;
        }
    }
    let editor = match &profile.editor {
        Some(config) => Some(Arc::new(
            crate::editor::EditorClient::new(config).map_err(StartError::Environment)?,
        )),
        None => None,
    };
    if let Some(editor) = &editor {
        let eyes: Arc<dyn crate::look::Eyes> = Arc::clone(&llm) as Arc<dyn crate::look::Eyes>;
        for tool in crate::editor::tools(editor, &robot, &snapshots, Some(eyes)) {
            registry.add(tool)?;
        }
    }
    let vision = seen_by.map(|cameras| crate::mission::camera::Vision {
        eyes: Arc::clone(&llm) as Arc<dyn crate::look::Eyes>,
        cameras,
        snapshots: Arc::clone(&snapshots),
    });
    Ok(Seeing { editor, vision })
}

/// `list_places`, `tag_place` and `forget_place`, over the profile's places and the ones remembered
/// beside the quota ledger, one file per robot.
fn place_tools(
    profile: &Profile,
    robot: &Arc<dyn RobotPort>,
    ledger: &Path,
    registry: &mut Registry,
) -> Result<Arc<crate::places::Places>, StartError> {
    let file = robot_file(profile, ledger, "places");
    let places = crate::places::Places::new(profile, file);
    registry.add(Arc::new(ListPlaces::new(
        profile,
        Arc::clone(&places),
        Arc::clone(robot),
    )))?;
    registry.add(Arc::new(crate::places::TagPlace::new(
        profile,
        Arc::clone(&places),
        Arc::clone(robot),
    )))?;
    registry.add(Arc::new(crate::places::ForgetPlace::new(Arc::clone(
        &places,
    ))))?;
    Ok(places)
}

/// `<kind>/<robot>.json` beside the quota ledger: what this robot keeps across sessions.
fn robot_file(profile: &Profile, ledger: &Path, kind: &str) -> Option<PathBuf> {
    ledger.parent().map(|dir| {
        let name = Some(crate::places::slug(&profile.robot.name))
            .filter(|n| !n.is_empty())
            .unwrap_or_else(|| "robot".into());
        dir.join(kind).join(format!("{name}.json"))
    })
}

/// Where a robot's mission ledger lives under the state folder `state`.
#[must_use]
pub fn mission_ledger_file(profile: &Profile, state: &Path) -> PathBuf {
    robot_file(profile, &state.join("quota.json"), "missions")
        .unwrap_or_else(|| state.join("missions").join("robot.json"))
        .with_extension("sqlite3")
}

/// The robot's mission ledger, beside its memory. Missions still run without one, unrecorded.
fn mission_ledger(profile: &Profile, ledger: &Path) -> Option<Arc<crate::mission::ledger::Ledger>> {
    let path = mission_ledger_file(profile, ledger.parent()?);
    crate::mission::ledger::Ledger::open(&path)
        .map_err(|e| tracing::warn!(error = %e, path = %path.display(), "no mission ledger"))
        .ok()
}

/// The watches and the health check: what a person runs before blaming the model.
fn debug_tools(
    profile: &Profile,
    robot: &Arc<dyn RobotPort>,
    registry: &mut Registry,
) -> Result<Arc<crate::watch::Watches>, StartError> {
    let watches = crate::watch::Watches::new(Arc::clone(robot));
    for tool in watches.tools() {
        registry.add(tool)?;
    }
    registry.add(Arc::new(crate::doctor::HealthCheck::new(
        profile,
        Arc::clone(robot),
    )))?;
    Ok(watches)
}

/// How a session starts: where it keeps its conversation, the conversation it carries on, and
/// the model it talks on.
#[derive(Default, Clone)]
pub struct StartOptions {
    /// Written after every turn, so the session can be resumed.
    pub history: Option<PathBuf>,
    /// An earlier conversation to carry on.
    pub resume: Option<crate::llm::History>,
    /// The only model for the routine role, by its `models.toml` id: an eval's arm.
    pub model: Option<String>,
    /// What the session talks to in place of `models.toml`'s models, such as a scripted model
    /// for a scenario; the profile's other roles (plan checks, vision) keep theirs.
    pub source: Option<Arc<dyn crate::llm::AgentSource>>,
}

/// Starts an agent with a fresh conversation that is not saved. Must run inside a tokio runtime.
///
/// # Errors
///
/// Anything in [`StartError`].
pub fn start(
    profile_path: &Path,
    robot: Arc<dyn RobotPort>,
    ledger: &Path,
) -> Result<Agent, StartError> {
    start_with(profile_path, robot, ledger, StartOptions::default())
}

/// Missions, when the profile has a `[mission]` section: kept in the robot's ledger, checked by
/// `critic` too when a model has the `plan_check` role, helped by `advisor` when the `plan` role
/// has models of its own, and their ends seen through the camera with `vision`.
fn missions(
    profile: &Profile,
    places: Arc<crate::places::Places>,
    robot: &Arc<dyn RobotPort>,
    ledger: &Path,
    critic: Option<Arc<dyn crate::mission::sanity::Critic>>,
    advisor: Option<Arc<dyn crate::mission::advice::Advisor>>,
    vision: Option<crate::mission::camera::Vision>,
) -> Option<Arc<Missions>> {
    let mut m = Missions::new(profile, places, Arc::clone(robot))?;
    if let Some(l) = mission_ledger(profile, ledger) {
        m = m.with_ledger(l);
    }
    if let Some(c) = critic {
        m = m.with_critic(c);
    }
    if let Some(a) = advisor {
        m = m.with_advisor(a);
    }
    if let Some(v) = vision {
        m = m.with_vision(v);
    }
    Some(m)
}

/// A mission started before this session runs on unwatched: says so, once, at start.
fn say_orphan(
    profile: &Profile,
    robot: &Arc<dyn RobotPort>,
    handle: crate::session::SessionHandle,
) {
    let Some(state) = profile.mission.as_ref().map(|m| m.state.clone()) else {
        return;
    };
    let robot = Arc::clone(robot);
    tokio::spawn(async move {
        let latched = robot
            .latest(&state, "nervros_interfaces/msg/RobotState", ORPHAN_WAIT)
            .await;
        if let Some(text) = latched.ok().as_ref().and_then(orphan_notice) {
            handle.emit(crate::session::Event::Notice { text });
        }
    });
}

/// The model layer and the second opinions it gives on plans.
struct ModelLayer {
    llm: Arc<Llm>,
    privacy: PrivacyMode,
    critic: Option<Arc<dyn crate::mission::sanity::Critic>>,
    advisor: Option<Arc<dyn crate::mission::advice::Advisor>>,
}

/// The profile's models behind one router, the routine role on `model` alone when one is given.
fn model_layer(
    profile: &Profile,
    ledger: &Path,
    model: Option<String>,
) -> Result<ModelLayer, StartError> {
    let mut models = ModelsConfig::load(&profile.resolve(&profile.models.file))?;
    if let Some(id) = model {
        if models.model(&id).is_none() {
            return Err(crate::providers::ConfigError::Unknown {
                from: "the model asked for".to_owned(),
                kind: "model",
                id,
            }
            .into());
        }
        models.roles.routine = vec![id];
    }
    let privacy = match profile.privacy.mode {
        PrivacyModeConfig::Sim => PrivacyMode::Sim,
        PrivacyModeConfig::Home => PrivacyMode::Home,
    };
    let checks_plans = !models.roles.plan_check.is_empty();
    // An advisor that is the planner itself would only repeat it.
    let advises = !models.roles.plan.is_empty() && models.roles.plan != models.roles.routine;
    let router = Router::with_ledger_file(models, ledger, privacy).map_err(StartError::Ledger)?;
    let llm = Arc::new(Llm::new(router));
    Ok(ModelLayer {
        critic: checks_plans.then(|| Arc::clone(&llm) as Arc<dyn crate::mission::sanity::Critic>),
        advisor: advises.then(|| Arc::clone(&llm) as Arc<dyn crate::mission::advice::Advisor>),
        llm,
        privacy,
    })
}

/// Starts an agent as `options` say. Must run inside a tokio runtime.
///
/// # Errors
///
/// Anything in [`StartError`].
pub fn start_with(
    profile_path: &Path,
    robot: Arc<dyn RobotPort>,
    ledger: &Path,
    options: StartOptions,
) -> Result<Agent, StartError> {
    let profile = Profile::load(profile_path)?;
    let models = model_layer(&profile, ledger, options.model)?;
    let llm = Arc::clone(&models.llm);
    let guard = Arc::new(Guard::new(profile.policy.clone()));
    let schemas: Arc<dyn SchemaSource> = Arc::new(RosidlSchemas::load(&profile)?);
    let mut registry = Registry::from_config(&profile.tools, &robot, &schemas, &guard)?;
    let snapshots = Arc::new(SnapshotStore::default());
    let seeing = seeing_tools(&profile, &robot, &llm, &snapshots, &mut registry)?;
    let places = place_tools(&profile, &robot, ledger, &mut registry)?;
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
    let watches = debug_tools(&profile, &robot, &mut registry)?;
    let mcp = mcp_tools(&profile, models.privacy, &mut registry)?;
    let skills = skill_tools(&profile, &mut registry)?;
    let memory = crate::memory::Memory::new(robot_file(&profile, ledger, "memory"));
    registry.add(Arc::new(crate::memory::MemoryTool::new(Arc::clone(
        &memory,
    ))))?;
    let stop: Arc<dyn Tool> = Arc::new(Stop::new(&profile, Arc::clone(&robot)));
    registry.add(Arc::clone(&stop))?;
    let missions = missions(
        &profile,
        places,
        &robot,
        ledger,
        models.critic,
        models.advisor,
        seeing.vision,
    );
    let schedules = missions
        .as_ref()
        .map(|m| crate::schedule::Schedules::new(Arc::clone(m), Arc::clone(&guard)));
    for tool in missions
        .iter()
        .flat_map(Missions::tools)
        .chain(missions.iter().flat_map(crate::mission::history::tools))
        .chain(schedules.iter().flat_map(crate::schedule::Schedules::tools))
    {
        registry.add(tool)?;
    }
    let tools = registry.iter().map(|t| t.spec().name.clone()).collect();
    let config = SessionConfig {
        preamble: system_prompt(&profile, &skills),
        max_model_calls: usize::try_from(profile.policy.budgets.model_calls).unwrap_or(6),
        approval_ttl: profile.policy.approval_ttl,
        turn_time: profile.policy.budgets.wall_time,
        history_file: options.history,
        resume: options.resume,
        notes: Some(Arc::clone(&memory) as Arc<dyn crate::memory::Notes>),
        pulse: missions
            .as_ref()
            .map(|m| Arc::clone(m) as Arc<dyn crate::session::Pulse>),
        ..SessionConfig::default()
    };
    let source = options
        .source
        .unwrap_or_else(|| Arc::clone(&llm) as Arc<dyn crate::llm::AgentSource>);
    let session = Session::start(
        source,
        Arc::new(registry),
        Arc::clone(&guard),
        Some(stop),
        config,
    );
    if let Some(m) = &missions {
        m.attach(session.handle());
    }
    if let Some(s) = &schedules {
        s.attach(session.handle());
    }
    watches.attach(session.handle());
    for check in mcp.iter().filter(|c| !c.ok) {
        session.handle().emit(crate::session::Event::Notice {
            text: check.what.clone(),
        });
    }
    say_orphan(&profile, &robot, session.handle());
    Ok(Agent {
        session,
        profile,
        guard,
        robot,
        snapshots,
        llm,
        tools,
        editor: seeing.editor,
        memory,
        schedules,
        missions,
        mcp,
    })
}

/// The profile's skills, and the `skill` tool to read them when there are any. A folder or file
/// that cannot be read is logged and left out.
fn skill_tools(
    profile: &Profile,
    registry: &mut Registry,
) -> Result<Vec<crate::skills::Skill>, StartError> {
    let dirs: Vec<PathBuf> = profile.skills.iter().map(|d| profile.resolve(d)).collect();
    let (skills, problems) = crate::skills::load(&dirs);
    for problem in problems {
        tracing::warn!(%problem, "a skill was left out");
    }
    if !skills.is_empty() {
        registry.add(Arc::new(crate::skills::SkillTool::new(skills.clone())))?;
    }
    Ok(skills)
}

/// The approved tools of the profile's MCP servers, connected once at start: the tool set stays
/// fixed for the session, which keeps the model server's prompt cache whole.
fn mcp_tools(
    profile: &Profile,
    privacy: PrivacyMode,
    registry: &mut Registry,
) -> Result<Vec<crate::doctor::Check>, StartError> {
    if profile.mcp_servers.is_empty() {
        return Ok(Vec::new());
    }
    let home = privacy == PrivacyMode::Home;
    // Start-up is not async; the runtime it runs in has threads to spare for this one wait.
    let (servers, checks) = tokio::task::block_in_place(|| {
        tokio::runtime::Handle::current().block_on(crate::mcp::connect_all(profile, home))
    });
    for server in &servers {
        for tool in server.tools() {
            registry.add(tool)?;
        }
    }
    Ok(checks)
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
