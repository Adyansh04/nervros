//! Commands that talk to a live ROS graph.

use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context as _, Result, bail};
use nervros_core::look::{Cameras, LookTool, SnapshotStore};
use nervros_core::profile::Profile;
use nervros_core::segment::SegmentTool;
use nervros_core::tools::{Status, Tool};

pub(crate) async fn look(profile_path: &Path, camera: Option<String>, out: &Path) -> Result<()> {
    let profile = Profile::load(profile_path).context("loading the profile")?;
    let config = profile
        .look
        .clone()
        .context("the profile has no [look] section")?;
    let robot = nervros_core::app::connect(&profile)?;
    let cameras = Cameras::start(&config, &robot).context("subscribing to the cameras")?;
    let tool = LookTool::new(
        config,
        cameras,
        robot,
        Arc::new(SnapshotStore::default()),
        None,
    );
    run_once(&tool, serde_json::json!({ "camera": camera }), out).await
}

pub(crate) async fn segment(
    profile_path: &Path,
    args: serde_json::Value,
    out: &Path,
) -> Result<()> {
    use nervros_core::llm::Llm;
    use nervros_core::providers::ModelsConfig;
    use nervros_core::providers::router::{PrivacyMode, Router};

    let profile = Profile::load(profile_path).context("loading the profile")?;
    let look = profile
        .look
        .clone()
        .context("the profile has no [look] section")?;
    let config = profile
        .segment
        .clone()
        .context("the profile has no [segment] section")?;
    let models = ModelsConfig::load(&profile.resolve(&profile.models.file))
        .context("loading the models file")?;
    let ledger = nervros_core::app::state_dir().join("quota.json");
    let privacy = match profile.privacy.mode {
        nervros_core::profile::PrivacyModeConfig::Sim => PrivacyMode::Sim,
        nervros_core::profile::PrivacyModeConfig::Home => PrivacyMode::Home,
    };
    let router =
        Router::with_ledger_file(models, &ledger, privacy).context("loading the quota ledger")?;
    let llm = Arc::new(Llm::new(router));
    let robot = nervros_core::app::connect(&profile)?;
    let cameras = Cameras::start(&look, &robot).context("subscribing to the cameras")?;
    let tool = SegmentTool::new(
        config,
        cameras,
        robot,
        Arc::new(SnapshotStore::default()),
        Some(llm as Arc<dyn nervros_core::segment::Outliner>),
    );
    let args = match args {
        serde_json::Value::Object(map) => {
            serde_json::Value::Object(map.into_iter().filter(|(_, v)| !v.is_null()).collect())
        }
        other => other,
    };
    run_once(&tool, args, out).await
}

/// Calls an image tool once, prints its data and saves its image.
async fn run_once(tool: &dyn Tool, args: serde_json::Value, out: &Path) -> Result<()> {
    // Frames and discovery need a moment after the node starts.
    tokio::time::sleep(Duration::from_secs(2)).await;
    let name = tool.spec().name.clone();
    let outcome = tool.call(args).await;
    if outcome.status != Status::Succeeded {
        bail!("{name} failed: {}", outcome.message);
    }
    println!("{}", serde_json::to_string_pretty(&outcome.data)?);
    if let Some(image) = outcome.images.first() {
        std::fs::write(out, image.jpeg.as_slice())
            .with_context(|| format!("writing {}", out.display()))?;
        eprintln!(
            "image: {} ({}x{})",
            out.display(),
            image.width,
            image.height
        );
    }
    Ok(())
}

/// Runs one read-only ROS tool once and prints what it returns.
pub(crate) async fn ros(profile_path: &Path, name: &str, args: &str) -> Result<()> {
    use nervros_core::guard::Guard;
    use nervros_core::schemas::RosidlSchemas;
    use nervros_core::tools::{Risk, SchemaSource};

    let args: serde_json::Value =
        serde_json::from_str(args).context("the arguments are not JSON")?;
    let profile = Profile::load(profile_path).context("loading the profile")?;
    let robot = nervros_core::app::connect(&profile)?;
    let schemas: Arc<dyn SchemaSource> =
        Arc::new(RosidlSchemas::load(&profile).context("loading the interfaces")?);
    let guard = Arc::new(Guard::new(profile.policy.clone()));
    let config = profile.ros_tools.clone().unwrap_or_default();
    let tools = nervros_core::ros_tools::tools(&config, &robot, &schemas, &guard);
    let names: Vec<String> = tools.iter().map(|t| t.spec().name.clone()).collect();
    let tool = tools
        .iter()
        .find(|t| t.spec().name == name)
        .with_context(|| format!("no ROS tool `{name}`; there are {}", names.join(", ")))?;
    // Discovery needs a moment after the node starts.
    tokio::time::sleep(Duration::from_secs(2)).await;
    let risk = match tool.assess(&args).await {
        Some(Ok(a)) => a.risk,
        Some(Err(out)) => bail!("{name}: {}", out.message),
        None => tool.spec().risk,
    };
    if risk != Risk::Observe {
        bail!("`{name}` would act on the robot; ask for it in `chat`, where it is approved");
    }
    let outcome = tool.call(args).await;
    if !outcome.message.is_empty() {
        eprintln!("{}", outcome.message);
    }
    println!("{}", serde_json::to_string_pretty(&outcome.data)?);
    if outcome.status != Status::Succeeded {
        bail!("{name} did not succeed");
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
    /// A saved conversation to carry on: a path, or `last`.
    pub(crate) resume: Option<String>,
}

/// A checked plan, each step with how it has gone before, then what may not match the request.
fn print_plan(
    hash: &str,
    steps: &[nervros_core::mission::plan::PlannedStep],
    worst_case_s: f64,
    concerns: &[nervros_core::mission::sanity::Concern],
) {
    let short = hash.get(..8).unwrap_or(hash);
    println!("  plan {short} ({worst_case_s:.0} s at most):");
    for s in steps {
        let record = s.track.as_ref().map_or_else(String::new, |t| {
            let typical = t
                .typical_s
                .map_or_else(String::new, |s| format!(", ~{s:.0} s"));
            format!("  ({} of {} ok{typical})", t.succeeded, t.runs)
        });
        println!("    {} {}{record}", s.id, s.summary);
    }
    for c in concerns {
        let step = if c.step.is_empty() {
            String::new()
        } else {
            format!("{}: ", c.step)
        };
        println!("    ! {step}{}", c.message);
    }
}

/// A mission's events: its plan, preview, progress and end.
fn print_mission(e: &Event) {
    match e {
        Event::MissionPlanned {
            hash,
            steps,
            worst_case_s,
            concerns,
            ..
        } => print_plan(hash, steps, *worst_case_s, concerns),
        Event::MissionPreview { steps, .. } => {
            for s in steps.iter().filter(|s| !s.note.is_empty()) {
                println!("    {} {}", s.id, s.note);
            }
        }
        Event::MissionStarted { id, .. } => println!("  [mission {id} started]"),
        Event::MissionProgress {
            step, node, status, ..
        } => {
            if node.is_empty() {
                println!("  [{step} {status}]");
            } else {
                println!("  [{step} {node} {status}]");
            }
        }
        Event::MissionFinished {
            outcome,
            failed_step,
            reason,
            elapsed_s,
            ..
        } => {
            let why = if failed_step.is_empty() {
                String::new()
            } else {
                format!(" at {failed_step}: {reason}")
            };
            println!("  [mission {outcome} after {elapsed_s:.0} s{why}]");
        }
        _ => {}
    }
}

fn print_event(e: &Event, logs: &Path) {
    match e {
        Event::Reply { text, model, .. } => println!("robot> {text}\n        [{model}]"),
        Event::Steer { text, .. } => println!("you, while it works> {text}"),
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
            can_allow,
        } => {
            let allow = if *can_allow {
                format!(", /allow {id} for the session,")
            } else {
                String::new()
            };
            println!(
                "  ? approve #{id}: {tool} {args} ({reason}); answer /yes {id}{allow} or /no {id}"
            );
        }
        Event::ApprovalEdited { id, reason, .. } => println!("  approval #{id} edited: {reason}"),
        Event::EditRejected { id, message } => println!("  edit of #{id} refused: {message}"),
        Event::ApprovalResolved { id, approved } => {
            println!("  approval #{id}: {}", if *approved { "yes" } else { "no" });
        }
        Event::Armed { armed } => println!("  [{}]", if *armed { "armed" } else { "disarmed" }),
        Event::Halted { reason } => println!("  [halted: {reason}]"),
        Event::Notice { text } => println!("  [{text}]"),
        Event::Error { text, .. } => println!("  [error: {text}]"),
        Event::Report { text, .. } => println!("report> {text}"),
        Event::MissionPlanned { .. }
        | Event::MissionPreview { .. }
        | Event::MissionStarted { .. }
        | Event::MissionProgress { .. }
        | Event::MissionFinished { .. } => print_mission(e),
        // Plots are drawn by the app's viewer; the tool's reply already says what it plots.
        Event::User { .. }
        | Event::TurnStarted { .. }
        | Event::TurnFinished { .. }
        | Event::Plot { .. }
        | Event::Context { .. }
        | Event::ModelCall { .. }
        | Event::ReplyDelta { .. } => {}
        Event::Restored { exchanges } => {
            println!(
                "  [carrying on an earlier conversation: {} messages]",
                exchanges.len()
            );
        }
        Event::Compacted {
            before,
            after,
            summarised,
        } => {
            let how = if *summarised { "summarised" } else { "cut" };
            println!("  [conversation {how}: {before} -> {after} tokens]");
        }
    }
}

/// The newest conversation a session saved under `logs`.
fn newest_history(logs: &Path) -> Option<std::path::PathBuf> {
    std::fs::read_dir(logs)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.to_string_lossy().ends_with(".history.json"))
        .max_by_key(|p| std::fs::metadata(p).and_then(|m| m.modified()).ok())
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
        "/compact" => Some(SessionCommand::Compact),
        // An exact stop word stops the robot without asking the model.
        "/stop" | "stop" | "halt" | "freeze" => Some(SessionCommand::StopMission),
        _ => {
            if let Some(id) = arg("/yes") {
                Some(SessionCommand::Approve(id))
            } else if let Some(id) = arg("/allow") {
                Some(SessionCommand::AllowForSession(id))
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
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let logs_dir = state.join("logs");
    let resume = match options.resume.as_deref() {
        None => None,
        Some(which) => {
            let path = if which == "last" {
                newest_history(&logs_dir).context("no saved conversation to resume")?
            } else {
                std::path::PathBuf::from(which)
            };
            eprintln!("resuming {}", path.display());
            Some(
                nervros_core::llm::History::load(&path)
                    .with_context(|| format!("reading {}", path.display()))?,
            )
        }
    };
    let files = nervros_core::app::StartOptions {
        history: Some(logs_dir.join(format!("session-{stamp}.history.json"))),
        resume,
        ..Default::default()
    };
    let agent =
        nervros_core::app::start_with(profile_path, robot, &state.join("quota.json"), files)
            .context("starting the agent")?;
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
    for notice in &agent.notices {
        println!("  ! {notice}");
    }
    for (n, u) in agent.unanswered.iter().enumerate() {
        println!(
            "  ? waiting for your approval when the last session ended: {} (/again {})",
            u.reason,
            n + 1
        );
    }
    let mut events = agent.session.subscribe();
    if options.arm {
        agent.session.send(SessionCommand::Arm);
    }
    // Discovery and the camera need a moment after the node starts.
    tokio::time::sleep(Duration::from_secs(2)).await;
    if !options.say.is_empty() {
        return say_all(&agent, &mut events, options.say, options.approve, &blobs).await;
    }
    interactive(&agent, events, blobs).await
}

/// Sends each message in turn and waits for its turn, the missions it starts and their reports;
/// approves every request when `approve`.
async fn say_all(
    agent: &nervros_core::app::Agent,
    events: &mut tokio::sync::broadcast::Receiver<Event>,
    say: Vec<String>,
    approve: bool,
    blobs: &Path,
) -> Result<()> {
    // Missions outlive the turn that started them: wait for their reports too.
    let (mut open, mut awaiting_report) = (0u32, false);
    for text in say {
        println!("you> {text}");
        agent.session.send(SessionCommand::User(text.clone()));
        // A report's reply can run first: wait for the turn this message starts.
        let mut mine = None;
        loop {
            let e = events.recv().await.context("the session ended")?;
            if let Event::ApprovalRequested { id, .. } = &e
                && approve
            {
                agent.session.send(SessionCommand::Approve(*id));
            }
            print_event(&e, blobs);
            match e {
                Event::MissionStarted { .. } => open += 1,
                Event::MissionFinished { .. } => {
                    open = open.saturating_sub(1);
                    awaiting_report = true;
                }
                Event::Report { .. } => awaiting_report = false,
                Event::User { turn, text: sent } if sent == text => mine = Some(turn),
                Event::TurnFinished { turn }
                    if mine.is_some_and(|m| turn >= m) && open == 0 && !awaiting_report =>
                {
                    break;
                }
                _ => {}
            }
        }
    }
    Ok(())
}

/// Reads the operator's lines until `/quit`, printing what the session reports meanwhile.
async fn interactive(
    agent: &nervros_core::app::Agent,
    mut events: tokio::sync::broadcast::Receiver<Event>,
    blobs: std::path::PathBuf,
) -> Result<()> {
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
        let again = line
            .trim()
            .strip_prefix("/again")
            .and_then(|n| n.trim().parse::<usize>().ok())
            .and_then(|n| agent.unanswered.get(n.checked_sub(1)?));
        if let Some(u) = again {
            agent.session.send(SessionCommand::Run {
                tool: u.tool.clone(),
                args: u.args.clone(),
            });
            continue;
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
