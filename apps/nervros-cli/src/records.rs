//! What the agent has done, read back: a session log replayed, the missions and skill gaps the
//! ledger keeps, and a session turned into an eval case.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use anyhow::{Context as _, Result, bail};
use nervros_core::mission::ledger::{self, Ledger};
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
        "notice" => format!("  ! {}", s("text")),
        "error" => format!("  error: {}", s("text")),
        _ => return None,
    })
}

fn open_ledger(profile: &Path) -> Result<std::sync::Arc<Ledger>> {
    let profile = Profile::load(profile).context("loading the profile")?;
    let path = nervros_core::app::mission_ledger_file(&profile, &nervros_core::app::state_dir());
    if !path.exists() {
        bail!("no missions recorded yet ({})", path.display());
    }
    Ledger::open(&path).with_context(|| format!("opening {}", path.display()))
}

/// The latest missions, or one in full by its id.
pub fn missions(profile: &Path, id: Option<&str>, limit: usize) -> Result<()> {
    let ledger = open_ledger(profile)?;
    let now = ledger::now_s();
    let list = match id {
        Some(id) => ledger.mission(id)?.into_iter().collect(),
        None => ledger.recent(limit)?,
    };
    for m in &list {
        let ago = (now - m.started) / 60.0;
        println!(
            "{} {:.0} min ago: {} ({}, {:.0} s)",
            &m.id[..m.id.len().min(8)],
            ago,
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
    let ledger = open_ledger(profile)?;
    let now = ledger::now_s();
    for g in ledger.gaps(limit)? {
        let nearest = if g.nearest.is_empty() {
            String::new()
        } else {
            format!(" (nearest: {})", g.nearest)
        };
        println!(
            "{:.0} min ago: \"{}\": {}{nearest}",
            (now - g.at) / 60.0,
            g.request,
            g.reason
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
    let text = nervros_core::evalcase::to_toml(&case)?;
    match append {
        Some(suite) => {
            use std::io::Write as _;
            let mut file = std::fs::OpenOptions::new()
                .append(true)
                .open(suite)
                .with_context(|| format!("opening {}", suite.display()))?;
            write!(file, "\n# From {}\n{text}", path.display())?;
            println!("appended case {id} to {}", suite.display());
        }
        None => print!("{text}"),
    }
    Ok(())
}
