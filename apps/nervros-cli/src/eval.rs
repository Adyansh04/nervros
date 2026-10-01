//! `nervros-cli eval`: runs a suite of requests against the live robot, each in a fresh session,
//! approving every request, and checks what the agent did against what each case expects.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context as _, Result};
use nervros_core::profile::Profile;
use nervros_core::session::{Command, Event};
use nervros_ros::RobotPort;
use serde::{Deserialize, Serialize};
use tokio::sync::broadcast;

/// A suite: cases run in order, on one robot whose state carries from case to case.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Suite {
    #[serde(rename = "case")]
    cases: Vec<Case>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Case {
    id: String,
    /// Sent first and not judged, such as walking back to the start.
    #[serde(default)]
    setup: Vec<String>,
    /// The operator's messages, in order, in one session.
    say: Vec<String>,
    #[serde(default = "d_max_s")]
    max_s: u64,
    #[serde(default)]
    expect: Expect,
}

fn d_max_s() -> u64 {
    240
}

#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
struct Expect {
    /// Each called at least once, and it succeeded or started.
    #[serde(default)]
    tools: Vec<String>,
    /// Never called.
    #[serde(default)]
    not_tools: Vec<String>,
    /// Each in some checked plan's steps, such as `WalkStraight`.
    #[serde(default)]
    skills: Vec<String>,
    /// How the last mission ended: `success`, `failure`, or `none` for no mission at all.
    mission: Option<String>,
    /// At most this many approvals asked.
    approvals_max: Option<usize>,
    /// The last reply holds one of these, case aside.
    #[serde(default)]
    reply_has: Vec<String>,
    /// The last reply holds none of these, case aside.
    #[serde(default)]
    reply_lacks: Vec<String>,
    /// How far the robot's base moved on the map, metres, `[min, max]`.
    moved_m: Option<[f64; 2]>,
    /// How far it turned, degrees, `[min, max]`: positive to the left (counter-clockwise), so a
    /// turn the wrong way fails.
    turned_deg: Option<[f64; 2]>,
    /// Whether the conversation had to be condensed.
    compacted: Option<bool>,
}

/// Words a reply should not use to ask for what the approval card asks.
const ASKS_FIRST: [&str; 5] = [
    "would you like me to",
    "shall i ",
    "do you want me to",
    "should i proceed",
    "want me to execute",
];

/// What happened in one case.
#[derive(Debug, Default, Serialize)]
struct Seen {
    tools: Vec<(String, String)>,
    skills: Vec<String>,
    missions: Vec<String>,
    approvals: usize,
    replies: Vec<String>,
    errors: Vec<String>,
    notices: Vec<String>,
    compactions: usize,
    moved_m: Option<f64>,
    turned_deg: Option<f64>,
    seconds: f64,
    timed_out: bool,
}

#[derive(Debug, Serialize)]
struct Verdict {
    id: String,
    passed: bool,
    problems: Vec<String>,
    seen: Seen,
}

/// Runs `suite` (only the cases whose ids hold `only`, when given) and writes `report.md` and
/// `results.json` into `out`. Returns how many cases failed.
///
/// # Errors
///
/// The profile, the suite or the robot cannot be loaded, or the report cannot be written.
pub(crate) async fn run(
    profile_path: &Path,
    suite_path: &Path,
    only: Option<&str>,
    out: &Path,
) -> Result<usize> {
    let suite: Suite = toml::from_str(
        &std::fs::read_to_string(suite_path)
            .with_context(|| format!("reading {}", suite_path.display()))?,
    )
    .with_context(|| format!("parsing {}", suite_path.display()))?;
    let profile = Profile::load(profile_path).context("loading the profile")?;
    let robot = nervros_core::app::connect(&profile)?;
    std::fs::create_dir_all(out).with_context(|| format!("creating {}", out.display()))?;
    let ledger = nervros_core::app::state_dir().join("quota.json");
    // Discovery and the cameras need a moment after the node starts.
    tokio::time::sleep(Duration::from_secs(3)).await;
    let mut verdicts = Vec::new();
    for case in suite
        .cases
        .iter()
        .filter(|c| only.is_none_or(|o| c.id.contains(o)))
    {
        eprintln!("== {}", case.id);
        let seen = run_case(profile_path, &profile, &robot, &ledger, case, out).await?;
        let problems = judge(&case.expect, &seen);
        let passed = problems.is_empty();
        eprintln!(
            "   {} in {:.0} s{}",
            if passed { "PASS" } else { "FAIL" },
            seen.seconds,
            if passed {
                String::new()
            } else {
                format!(": {}", problems.join("; "))
            }
        );
        verdicts.push(Verdict {
            id: case.id.clone(),
            passed,
            problems,
            seen,
        });
    }
    std::fs::write(
        out.join("results.json"),
        serde_json::to_string_pretty(&verdicts)?,
    )?;
    std::fs::write(out.join("report.md"), report(&verdicts))?;
    let failed = verdicts.iter().filter(|v| !v.passed).count();
    eprintln!(
        "{} of {} passed; report {}",
        verdicts.len() - failed,
        verdicts.len(),
        out.join("report.md").display()
    );
    Ok(failed)
}

/// Where the base is on the map: x, y and yaw.
fn pose(robot: &Arc<dyn RobotPort>, profile: &Profile) -> Option<(f64, f64, f64)> {
    let t = robot
        .transform(&profile.ros.map_frame, &profile.ros.base_frame)
        .ok()?;
    Some((t.translation[0], t.translation[1], t.yaw()))
}

async fn run_case(
    profile_path: &Path,
    profile: &Profile,
    robot: &Arc<dyn RobotPort>,
    ledger: &Path,
    case: &Case,
    out: &Path,
) -> Result<Seen> {
    let agent = nervros_core::app::start(profile_path, Arc::clone(robot), ledger)
        .context("starting the agent")?;
    let (_path, _log) =
        nervros_core::log::spawn(out, &format!("case-{}", case.id), agent.session.subscribe())
            .context("opening the case log")?;
    let mut events = agent.session.subscribe();
    agent.session.send(Command::Arm);
    let mut seen = Seen::default();
    let deadline = Instant::now() + Duration::from_secs(case.max_s);
    for text in &case.setup {
        let mut ignored = Seen::default();
        say(&agent.session, &mut events, text, deadline, &mut ignored).await;
    }
    let start = pose(robot, profile);
    let began = Instant::now();
    for text in &case.say {
        if say(&agent.session, &mut events, text, deadline, &mut seen).await {
            seen.timed_out = true;
            // A case over its time stops the robot rather than leave it running into the next.
            agent.session.send(Command::StopMission);
            tokio::time::sleep(Duration::from_secs(2)).await;
            break;
        }
    }
    seen.seconds = began.elapsed().as_secs_f64();
    if let (Some(a), Some(b)) = (start, pose(robot, profile)) {
        seen.moved_m = Some((b.0 - a.0).hypot(b.1 - a.1));
        let turn = (b.2 - a.2 + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        seen.turned_deg = Some(turn.to_degrees());
    }
    Ok(seen)
}

/// Sends one message and waits for its turn, the missions it starts and their reports;
/// `true` when the deadline passed first.
async fn say(
    session: &nervros_core::session::Session,
    events: &mut broadcast::Receiver<Event>,
    text: &str,
    deadline: Instant,
    seen: &mut Seen,
) -> bool {
    session.send(Command::User(text.to_owned()));
    let (mut mine, mut open, mut awaiting_report) = (None, 0u32, false);
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        let e = match tokio::time::timeout(left, events.recv()).await {
            Err(_) => return true,
            Ok(Err(broadcast::error::RecvError::Lagged(_))) => continue,
            Ok(Err(broadcast::error::RecvError::Closed)) => return false,
            Ok(Ok(e)) => e,
        };
        match e {
            Event::ApprovalRequested { id, .. } => {
                seen.approvals += 1;
                session.send(Command::Approve(id));
            }
            Event::ToolFinished { tool, status, .. } => seen.tools.push((tool, status.to_owned())),
            Event::MissionPlanned { steps, .. } => {
                seen.skills.extend(steps.into_iter().map(|s| s.summary));
            }
            Event::MissionStarted { .. } => open += 1,
            Event::MissionFinished { outcome, .. } => {
                seen.missions.push(outcome);
                open = open.saturating_sub(1);
                awaiting_report = true;
            }
            Event::Report { .. } => awaiting_report = false,
            Event::Reply { text, .. } => seen.replies.push(text),
            Event::Error { text, .. } => seen.errors.push(text),
            Event::Notice { text } => seen.notices.push(text),
            Event::Compacted { .. } => seen.compactions += 1,
            Event::User { turn, text: sent } if sent == text => mine = Some(turn),
            Event::TurnFinished { turn }
                if mine.is_some_and(|m| turn >= m) && open == 0 && !awaiting_report =>
            {
                return false;
            }
            _ => {}
        }
    }
}

fn judge(expect: &Expect, seen: &Seen) -> Vec<String> {
    let mut problems = Vec::new();
    if seen.timed_out {
        problems.push("timed out".to_owned());
    }
    for e in &seen.errors {
        problems.push(format!("error: {e}"));
    }
    let ok = |t: &str| {
        seen.tools
            .iter()
            .any(|(n, s)| n == t && (s == "succeeded" || s == "accepted"))
    };
    for t in expect.tools.iter().filter(|t| !ok(t)) {
        problems.push(format!("{t} not called successfully"));
    }
    for t in &expect.not_tools {
        if seen.tools.iter().any(|(n, _)| n == t) {
            problems.push(format!("{t} called"));
        }
    }
    for s in &expect.skills {
        if !seen.skills.iter().any(|k| k.starts_with(s.as_str())) {
            problems.push(format!(
                "no {s} in any plan (plans: {})",
                seen.skills.join(", ")
            ));
        }
    }
    match (expect.mission.as_deref(), seen.missions.last()) {
        (Some("none"), Some(m)) => problems.push(format!("a mission ran ({m})")),
        (Some(want), None) if want != "none" => {
            problems.push(format!("no mission ({want} wanted)"));
        }
        (Some(want), Some(got)) if want != "none" && want != got => {
            problems.push(format!("mission {got}, {want} wanted"));
        }
        _ => {}
    }
    if let Some(max) = expect.approvals_max
        && seen.approvals > max
    {
        problems.push(format!(
            "{} approvals, at most {max} wanted",
            seen.approvals
        ));
    }
    let last = seen
        .replies
        .last()
        .map(|r| r.to_lowercase())
        .unwrap_or_default();
    if !expect.reply_has.is_empty()
        && !expect
            .reply_has
            .iter()
            .any(|w| last.contains(&w.to_lowercase()))
    {
        problems.push(format!("the reply has none of {:?}", expect.reply_has));
    }
    // Offering a follow-up is fine; asking leave for a plan the approval card asks about is not.
    let planned = !seen.skills.is_empty() || !seen.missions.is_empty();
    let asks = ASKS_FIRST.iter().copied().filter(|_| planned);
    for w in expect.reply_lacks.iter().map(String::as_str).chain(asks) {
        if last.contains(&w.to_lowercase()) {
            problems.push(format!("the reply says \"{w}\""));
        }
    }
    let within =
        |v: Option<f64>, [lo, hi]: [f64; 2], what: &str, problems: &mut Vec<String>| match v {
            Some(v) if v < lo || v > hi => {
                problems.push(format!("{what} {v:.2}, wanted {lo}..{hi}"));
            }
            None => problems.push(format!("{what} unknown")),
            _ => {}
        };
    if let Some(range) = expect.moved_m {
        within(seen.moved_m, range, "moved m", &mut problems);
    }
    if let Some(want) = expect.compacted
        && want != (seen.compactions > 0)
    {
        problems.push(format!("compacted {} times", seen.compactions));
    }
    if let Some(range) = expect.turned_deg {
        within(seen.turned_deg, range, "turned deg", &mut problems);
    }
    problems
}

fn report(verdicts: &[Verdict]) -> String {
    let passed = verdicts.iter().filter(|v| v.passed).count();
    let mut out = format!("# Eval: {passed} of {} passed\n\n", verdicts.len());
    out.push_str("| Case | Result | s | Approvals | Tools | Missions | Problems |\n|---|---|---|---|---|---|---|\n");
    for v in verdicts {
        let tools: Vec<String> = v
            .seen
            .tools
            .iter()
            .map(|(t, s)| format!("{t}:{s}"))
            .collect();
        let _ = writeln!(
            out,
            "| {} | {} | {:.0} | {} | {} | {} | {} |",
            v.id,
            if v.passed { "pass" } else { "FAIL" },
            v.seen.seconds,
            v.seen.approvals,
            tools.join(" "),
            v.seen.missions.join(" "),
            v.problems.join("; ").replace('|', "/")
        );
    }
    out.push_str("\n## Replies\n");
    for v in verdicts {
        let last = v.seen.replies.last().map_or("(none)", String::as_str);
        let _ = writeln!(out, "\n**{}**: {}", v.id, last.replace('\n', " "));
    }
    out
}

/// Where a run's report goes when `--out` is not given.
pub(crate) fn default_out() -> PathBuf {
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    nervros_core::app::state_dir()
        .join("evals")
        .join(format!("eval-{stamp}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_case_is_judged_on_what_happened() {
        let expect: Expect = toml::from_str(
            "tools = [\"run_mission\"]\nskills = [\"WalkStraight\"]\nmission = \"success\"\n\
             approvals_max = 1\nmoved_m = [0.7, 1.3]\n",
        )
        .unwrap();
        let mut seen = Seen {
            tools: vec![("run_mission".into(), "accepted".into())],
            skills: vec!["WalkStraight(direction=forward, distance_m=1)".into()],
            missions: vec!["success".into()],
            approvals: 1,
            replies: vec!["It walked 1 m forward.".into()],
            moved_m: Some(0.95),
            turned_deg: Some(2.0),
            ..Seen::default()
        };
        assert!(
            judge(&expect, &seen).is_empty(),
            "{:?}",
            judge(&expect, &seen)
        );
        seen.replies
            .push("I planned it. Would you like me to run it?".into());
        seen.moved_m = Some(0.0);
        seen.approvals = 2;
        let problems = judge(&expect, &seen).join("; ");
        assert!(
            problems.contains("would you like me to")
                && problems.contains("moved m 0.00")
                && problems.contains("2 approvals"),
            "{problems}"
        );
    }
}
