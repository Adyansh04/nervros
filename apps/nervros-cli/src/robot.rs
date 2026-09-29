//! Commands that talk to a live ROS graph.

use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context as _, Result, bail};
use nervros_core::look::{LookTool, SnapshotStore};
use nervros_core::profile::{Profile, Transport};
use nervros_core::tools::{Status, Tool as _};
use nervros_ros::{R2rConfig, R2rPort, RobotPort};

/// Starts the node after checking the environment matches the profile; the environment decides,
/// because ROS reads it when the node starts.
pub(crate) fn connect(profile: &Profile) -> Result<Arc<dyn RobotPort>> {
    let domain = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|d| d.parse::<u32>().ok())
        .unwrap_or(0);
    if domain != profile.ros.domain_id {
        bail!(
            "ROS_DOMAIN_ID is {domain} but the profile says {}; export it before starting",
            profile.ros.domain_id
        );
    }
    let udp = std::env::var("FASTDDS_BUILTIN_TRANSPORTS").is_ok_and(|t| t == "UDPv4");
    if profile.ros.transport == Transport::Udp && !udp {
        bail!(
            "the profile wants UDP only; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 (NERVROS_UDP_ONLY=1 with scripts/ros-env.sh)"
        );
    }
    let config = R2rConfig {
        node_name: profile.ros.node_name.clone(),
        ..R2rConfig::default()
    };
    Ok(Arc::new(
        R2rPort::start(config).context("starting the ROS node")?,
    ))
}

pub(crate) async fn look(profile_path: &Path, out: &Path) -> Result<()> {
    let profile = Profile::load(profile_path).context("loading the profile")?;
    let config = profile
        .look
        .clone()
        .context("the profile has no [look] section")?;
    let robot = connect(&profile)?;
    let tool = LookTool::start(config, robot, Arc::new(SnapshotStore::default()))
        .context("subscribing to the camera")?;
    // Frames and discovery need a moment after the node starts.
    tokio::time::sleep(Duration::from_secs(2)).await;
    let outcome = tool.call(serde_json::json!({})).await;
    if outcome.status != Status::Succeeded {
        bail!("look failed: {}", outcome.message);
    }
    println!("{}", serde_json::to_string_pretty(&outcome.data)?);
    if let Some(image) = outcome.images.first() {
        std::fs::write(out, image.jpeg.as_slice())
            .with_context(|| format!("writing {}", out.display()))?;
        eprintln!(
            "marked image: {} ({}x{})",
            out.display(),
            image.width,
            image.height
        );
    }
    Ok(())
}

use nervros_core::session::{Command as SessionCommand, Event};
use tokio::io::AsyncBufReadExt as _;

/// Options of `chat`.
pub(crate) struct ChatOptions {
    /// Messages to send in order instead of reading stdin.
    pub(crate) say: Vec<String>,
    /// Arm at start.
    pub(crate) arm: bool,
    /// Approve every request (scripted runs only).
    pub(crate) approve: bool,
}

fn print_event(e: &Event, logs: &Path) {
    match e {
        Event::Reply { text, model, .. } => println!("robot> {text}\n        [{model}]"),
        Event::ToolStarted { tool, args, .. } => println!("  > {tool} {args}"),
        Event::ToolFinished {
            tool,
            status,
            message,
            ms,
            ..
        } => {
            let note = if message.is_empty() {
                String::new()
            } else {
                format!(": {message}")
            };
            println!("  < {tool} {status} in {ms} ms{note}");
        }
        Event::Snapshot {
            id, width, height, ..
        } => {
            println!(
                "  [image {id} {width}x{height}: {}]",
                logs.join(format!("{id}.jpg")).display()
            );
        }
        Event::ApprovalRequested {
            id,
            tool,
            args,
            reason,
        } => {
            println!("  ? approve #{id}: {tool} {args} ({reason}); answer /yes {id} or /no {id}");
        }
        Event::ApprovalResolved { id, approved } => {
            println!("  approval #{id}: {}", if *approved { "yes" } else { "no" });
        }
        Event::Armed { armed } => println!("  [{}]", if *armed { "armed" } else { "disarmed" }),
        Event::Halted { reason } => println!("  [halted: {reason}]"),
        Event::Notice { text } => println!("  [{text}]"),
        Event::Error { text, .. } => println!("  [error: {text}]"),
        Event::TurnStarted { .. } | Event::TurnFinished { .. } => {}
    }
}

fn parse_line(line: &str) -> Option<SessionCommand> {
    let line = line.trim();
    let arg = |p: &str| {
        line.strip_prefix(p)
            .and_then(|r| r.trim().parse::<u64>().ok())
    };
    match line {
        "" => None,
        "/arm" => Some(SessionCommand::Arm),
        "/disarm" => Some(SessionCommand::Disarm),
        // An exact stop word stops the robot without asking the model.
        "/stop" | "stop" | "halt" | "freeze" => Some(SessionCommand::StopMission),
        _ => {
            if let Some(id) = arg("/yes") {
                Some(SessionCommand::Approve(id))
            } else if let Some(id) = arg("/no") {
                Some(SessionCommand::Deny(id))
            } else {
                Some(SessionCommand::User(line.to_owned()))
            }
        }
    }
}

pub(crate) async fn chat(profile_path: &Path, state: &Path, options: ChatOptions) -> Result<()> {
    let profile = Profile::load(profile_path).context("loading the profile")?;
    let robot = connect(&profile)?;
    let agent = nervros_core::app::start(profile_path, robot, &state.join("quota.json"))
        .context("starting the agent")?;
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let logs_dir = state.join("logs");
    let (log_path, _log) = nervros_core::log::spawn(
        &logs_dir,
        &format!("session-{stamp}"),
        agent.session.subscribe(),
    )
    .context("opening the session log")?;
    let blobs = logs_dir.join(format!("session-{stamp}"));
    eprintln!(
        "NervROS: {} with tools {}; log {}",
        agent.profile.robot.name,
        agent.tools.join(", "),
        log_path.display()
    );
    let mut events = agent.session.subscribe();
    if options.arm {
        agent.session.send(SessionCommand::Arm);
    }
    // Discovery and the camera need a moment after the node starts.
    tokio::time::sleep(Duration::from_secs(2)).await;
    if !options.say.is_empty() {
        for text in options.say {
            println!("you> {text}");
            agent.session.send(SessionCommand::User(text));
            loop {
                let e = events.recv().await.context("the session ended")?;
                if let Event::ApprovalRequested { id, .. } = &e
                    && options.approve
                {
                    agent.session.send(SessionCommand::Approve(*id));
                }
                print_event(&e, &blobs);
                if matches!(e, Event::TurnFinished { .. }) {
                    break;
                }
            }
        }
        return Ok(());
    }
    let printer = tokio::spawn(async move {
        while let Ok(e) = events.recv().await {
            print_event(&e, &blobs);
        }
    });
    let mut lines = tokio::io::BufReader::new(tokio::io::stdin()).lines();
    while let Some(line) = lines.next_line().await? {
        if line.trim() == "/quit" {
            break;
        }
        if let Some(cmd) = parse_line(&line) {
            agent.session.send(cmd);
        }
    }
    printer.abort();
    Ok(())
}

pub(crate) async fn doctor(profile_path: &Path) -> Result<()> {
    let profile = Profile::load(profile_path).context("loading the profile")?;
    let robot = connect(&profile)?;
    tokio::time::sleep(profile.ros.discovery_timeout.min(Duration::from_secs(5))).await;
    let graph = robot.graph().await.context("reading the ROS graph")?;
    let has_topic = |t: &str| graph.topics.iter().any(|(name, _)| name == t);
    let mut bad = 0u32;
    let mut report = |ok: bool, what: String| {
        println!("{} {what}", if ok { "ok     " } else { "MISSING" });
        bad += u32::from(!ok);
    };
    let schemas =
        nervros_core::schemas::RosidlSchemas::load(&profile).context("loading interface files")?;
    report(
        !schemas.is_empty() || profile.tools.is_empty(),
        format!("{} interface definitions loaded", schemas.len()),
    );
    for t in &profile.tools {
        let ok = match t.kind {
            nervros_core::tools::ToolKind::Service => {
                robot
                    .service_available(&t.ros_name, &t.ros_type, Duration::from_secs(5))
                    .await
            }
            nervros_core::tools::ToolKind::Topic => has_topic(&t.ros_name),
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
            report(
                robot
                    .service_available(name, ty, Duration::from_secs(5))
                    .await,
                format!("mission {what} {name}"),
            );
        }
    }
    if bad > 0 {
        bail!("{bad} checks failed");
    }
    Ok(())
}
