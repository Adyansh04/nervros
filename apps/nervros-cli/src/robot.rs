//! Commands that talk to a live ROS graph.

use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context as _, Result, bail};
use nervros_core::look::{LookTool, SnapshotStore};
use nervros_core::profile::Profile;
use nervros_core::tools::{Status, Tool as _};

pub(crate) async fn look(profile_path: &Path, out: &Path) -> Result<()> {
    let profile = Profile::load(profile_path).context("loading the profile")?;
    let config = profile
        .look
        .clone()
        .context("the profile has no [look] section")?;
    let robot = nervros_core::app::connect(&profile)?;
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
    let robot = nervros_core::app::connect(&profile)?;
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
    let robot = nervros_core::app::connect(&profile)?;
    tokio::time::sleep(profile.ros.discovery_timeout.min(Duration::from_secs(5))).await;
    let checks = nervros_core::doctor::run(&profile, robot.as_ref()).await;
    for c in &checks {
        println!("{} {}", if c.ok { "ok     " } else { "MISSING" }, c.what);
    }
    let bad = checks.iter().filter(|c| !c.ok).count();
    if bad > 0 {
        bail!("{bad} checks failed");
    }
    Ok(())
}
