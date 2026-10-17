//! What the agent has done, read back: a session log replayed, the missions and skill gaps the
//! ledger keeps, and a session turned into an eval case.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use anyhow::{Context as _, Result, bail};
use nervros_core::mission::ledger::Ledger;
use nervros_core::profile::Profile;
use serde_json::Value;

/// A session log by path, or `last` for the newest under the state folder.
fn session_log(which: &str) -> Result<PathBuf> {
    if which != "last" {
        return Ok(PathBuf::from(which));
    }
    let logs = nervros_core::app::state_dir().join("logs");
    std::fs::read_dir(&logs)
        .with_context(|| format!("reading {}", logs.display()))?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "ndjson"))
        .max_by_key(|p| p.metadata().and_then(|m| m.modified()).ok())
        .context("no session log yet")
}

/// Prints a session log as it happened, each line with its time into the session.
pub fn replay(which: &str) -> Result<()> {
    let path = session_log(which)?;
    let text =
        std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
    let mut start = None;
    for line in text
        .lines()
        .filter_map(|l| serde_json::from_str::<Value>(l).ok())
    {
        let ts = line["ts"].as_f64().unwrap_or(0.0);
        let at = ts - *start.get_or_insert(ts);
        if let Some(text) = describe(&line["event"]) {
            println!("{at:7.1}s  {text}");
        }
    }
    Ok(())
}

/// One logged event as a line, or nothing for the ones not worth a line.
fn describe(e: &Value) -> Option<String> {
    let s = |k: &str| e[k].as_str().unwrap_or_default();
    Some(match s("kind") {
        "user" => format!("you> {}", s("text")),
        "steer" => format!("you, while it worked> {}", s("text")),
        "reply" => format!("robot> {}", s("text").replace('\n', " ")),
        "report" => format!("report> {}", s("text")),
        "tool_started" => format!("  > {} {}", s("tool"), e["args"]),
        "tool_finished" => format!("  < {} {} {}", s("tool"), s("status"), s("message")),
        "approval_requested" => format!("  ? approve {}: {}", s("tool"), s("reason")),
        "approval_resolved" => format!(
            "  ? {}",
            if e["approved"].as_bool() == Some(true) {
                "approved"
            } else {
                "denied"
            }
        ),
        "mission_planned" => {
            let steps: Vec<&str> = e["steps"]
                .as_array()
                .map_or(&[][..], Vec::as_slice)
                .iter()
                .filter_map(|st| st["summary"].as_str())
                .collect();
            format!("  plan: {}", steps.join(" -> "))
        }
        "mission_progress" if s("node").is_empty() => format!("  [{} {}]", s("step"), s("status")),
        "mission_finished" => {
            let mut line = format!("  [mission {}]", s("outcome"));
            if !s("failed_step").is_empty() {
                let _ = write!(line, " at {}: {}", s("failed_step"), s("reason"));
            }
            line
        }
        "model_call" => {
            let n = |k: &str| e[k].as_u64().unwrap_or_default();
            format!(
                "  . {} {} ms: {} in ({} cached), {} out",
                s("model"),
                n("ms"),
                n("input_tokens"),
                n("cached_tokens"),
                n("output_tokens")
            )
        }
        "notice" => format!("  ! {}", s("text")),
        "error" => format!("  error: {}", s("text")),
        // What the live printer shows, so a replay leaves out no stop or arming.
        "halted" => format!("  [halted: {}]", s("reason")),
        "stopped" if e["ok"].as_bool() == Some(true) => {
            format!("  [the robot stopped: {}]", s("detail"))
        }
        "stopped" => format!("  ! the robot did not confirm the stop: {}", s("detail")),
        "armed" => format!(
            "  [{}]",
            if e["armed"].as_bool() == Some(true) {
                "armed"
            } else {
                "observe only"
            }
        ),
        "approval_edited" => format!("  ? edited: {}", s("reason")),
        "edit_rejected" => format!("  ? edit rejected: {}", s("message")),
        "compacted" => format!(
            "  [condensed from {} to {} tokens]",
            e["before"].as_u64().unwrap_or_default(),
            e["after"].as_u64().unwrap_or_default()
        ),
        _ => return None,
    })
}

/// The robot's mission ledger, or `None`, said, when it has run none yet: an empty record is not
/// an error.
fn open_ledger(profile: &Path) -> Result<Option<std::sync::Arc<Ledger>>> {
    let profile = Profile::load(profile).context("loading the profile")?;
    let path = nervros_core::app::mission_ledger_file(&profile, &nervros_core::app::state_dir());
    if !path.exists() {
        println!("no missions recorded yet ({})", path.display());
        return Ok(None);
    }
    Ledger::open(&path)
        .map(Some)
        .with_context(|| format!("opening {}", path.display()))
}

/// The latest missions, or one in full by its id.
pub fn missions(profile: &Path, id: Option<&str>, limit: usize) -> Result<()> {
    let Some(ledger) = open_ledger(profile)? else {
        return Ok(());
    };
    let now = nervros_core::now_s();
    let list = match id {
        Some(id) => ledger.mission(id)?,
        None => ledger.recent(limit)?,
    };
    for m in &list {
        // A faulted run says nothing about its plan; it is told apart from the plan's failures.
        let fault = if m.fault.is_empty() {
            String::new()
        } else {
            format!(", fault: {}", m.fault)
        };
        println!(
            "{} {} ago: {} ({}{fault}, {:.0} s)",
            // A version 7 id begins with its millisecond: this much tells missions apart.
            &m.id[..m.id.len().min(13)],
            nervros_core::how_long(now - m.started),
            m.intent,
            m.outcome,
            m.ended - m.started
        );
        if !m.request.is_empty() {
            println!("    asked: {}", m.request);
        }
        if id.is_some() || m.outcome != "success" {
            for st in &m.steps {
                let time = st
                    .seconds
                    .map_or_else(String::new, |s| format!(" {s:.0} s"));
                let why = if st.reason.is_empty() {
                    String::new()
                } else {
                    format!(": {}", st.reason)
                };
                println!(
                    "    {} {} {} {}{time}{why}",
                    st.id, st.skill, st.target, st.outcome
                );
            }
        }
    }
    if list.is_empty() {
        println!("no missions");
    }
    Ok(())
}

/// The requests no skill could do, newest first.
pub fn gaps(profile: &Path, limit: usize) -> Result<()> {
    let Some(ledger) = open_ledger(profile)? else {
        return Ok(());
    };
    let now = nervros_core::now_s();
    let gaps = ledger.gaps(limit)?;
    if gaps.is_empty() {
        println!("no skill gaps: every request so far had a skill for it");
    }
    for g in gaps {
        let nearest = if g.nearest.is_empty() {
            String::new()
        } else {
            format!(" (nearest: {})", g.nearest)
        };
        println!(
            "{} ago: \"{}\": {}{nearest}",
            nervros_core::how_long(now - g.at),
            g.request,
            g.reason
        );
    }
    Ok(())
}

/// What the robot did over the last `hours`, from its ledger, incidents and goals.
pub fn digest(profile: &Path, hours: f64) -> Result<()> {
    if !(hours.is_finite() && hours > 0.0) {
        bail!("--hours is more than 0");
    }
    let since = nervros_core::now_s() - hours * 3600.0;
    let missions = match open_ledger(profile)? {
        Some(ledger) => ledger.since(since)?,
        None => Vec::new(),
    };
    let profile = Profile::load(profile).context("loading the profile")?;
    let state = nervros_core::app::state_dir();
    let (file, _) = nervros_core::app::incident_files(&profile, &state);
    let incidents: Vec<_> = nervros_core::incident::read(&file)
        .into_iter()
        .filter(|i| i.opened_s >= since)
        .collect();
    let goals = nervros_core::mission::goals::Goals::new(Some(nervros_core::app::goals_file(
        &profile, &state,
    )))
    .all();
    println!(
        "{}",
        nervros_core::mission::digest::compose(&nervros_core::mission::digest::Period {
            hours,
            missions: &missions,
            world: &[],
            incidents: &incidents,
            goals: &goals,
        })
    );
    Ok(())
}

/// The robot's incidents, newest first, or one in full by its id.
pub fn incidents(profile: &Path, id: Option<&str>, limit: usize) -> Result<()> {
    let profile = Profile::load(profile).context("loading the profile")?;
    let (file, _) = nervros_core::app::incident_files(&profile, &nervros_core::app::state_dir());
    let all = nervros_core::incident::read(&file);
    let now = nervros_core::now_s();
    if let Some(id) = id {
        let Some(incident) = all.iter().find(|i| i.id.eq_ignore_ascii_case(id)) else {
            bail!("no incident {id} in {}", file.display());
        };
        print!("{}", incident.text(now));
        return Ok(());
    }
    if all.is_empty() {
        println!("no incidents kept yet ({})", file.display());
    }
    for i in all.iter().rev().take(limit) {
        println!(
            "{} {} ago: {}",
            i.id,
            nervros_core::how_long(now - i.opened_s),
            i.headline()
        );
    }
    Ok(())
}

/// A session as an eval case: printed, or appended to a suite file.
pub fn case(which: &str, id: &str, append: Option<&Path>) -> Result<()> {
    let path = session_log(which)?;
    let log =
        std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
    let case = nervros_core::evalcase::from_log(&log, id);
    if case.say.is_empty() {
        bail!("{} has no operator messages", path.display());
    }
    match append {
        Some(suite) => {
            let id = nervros_core::evalcase::append(suite, case, &path)
                .with_context(|| format!("appending to {}", suite.display()))?;
            println!("appended case {id} to {}", suite.display());
        }
        None => print!("{}", nervros_core::evalcase::to_toml(&case)?),
    }
    Ok(())
}
