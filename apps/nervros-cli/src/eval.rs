//! `nervros-cli eval`: runs a suite of requests against the live robot, each in a fresh session,
//! approving every request, and checks what the agent did against what each case expects.
//!
//! Each case runs `repeat` times on each model asked for, the models taking turns so a change in
//! the world falls on all of them alike. The report gives pass^k, what the calls cost, and where
//! the run came from: the checkout, the suite's hash, the models and the server behind each.

use std::collections::BTreeSet;
use std::f64::consts::{PI, TAU};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context as _, Result};
use nervros_core::evalcase::{Case, Expect, Suite, Truth};
use nervros_core::profile::Profile;
use nervros_core::providers::{ModelsConfig, ProviderKind};
use nervros_core::session::{Command, Event};
use nervros_ros::RobotPort;
use nervros_ros::tf::Transform;
use serde::Serialize;
use serde_json::{Value, json};
use sha2::{Digest as _, Sha256};
use tokio::sync::broadcast;

/// Words a reply should not use to ask for what the approval card asks.
const ASKS_FIRST: [&str; 5] = [
    "would you like me to",
    "shall i ",
    "do you want me to",
    "should i proceed",
    "want me to execute",
];

/// The robot's estimate and the simulator's truth further apart than this are worth a note.
const DRIFT_M: f64 = 0.25;
const DRIFT_DEG: f64 = 15.0;

/// How a suite runs.
pub(crate) struct Options {
    /// Only the cases whose ids hold this.
    pub only: Option<String>,
    /// Trials per case and model.
    pub repeat: usize,
    /// The models the routine role runs on, one after another, by `models.toml` id; none for the
    /// profile's chain.
    pub models: Vec<String>,
}

/// What happened in one trial.
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
    /// By the robot's own estimate of its pose on the map.
    moved_m: Option<f64>,
    turned_deg: Option<f64>,
    /// By the simulator, when the suite names its truth.
    truth_moved_m: Option<f64>,
    truth_turned_deg: Option<f64>,
    seconds: f64,
    timed_out: bool,
    calls: u64,
    input_tokens: u64,
    cached_tokens: u64,
    output_tokens: u64,
    model_ms: u64,
}

#[derive(Debug, Serialize)]
struct Trial {
    case: String,
    /// The model the routine role ran on, or none for the profile's chain.
    model: Option<String>,
    attempt: usize,
    passed: bool,
    problems: Vec<String>,
    /// What does not fail the trial but is worth a look.
    notes: Vec<String>,
    seen: Seen,
}

/// Where a run came from, so two runs can be told apart.
#[derive(Debug, Serialize)]
struct Provenance {
    started: String,
    nervros: String,
    profile: String,
    suite: String,
    suite_sha256: String,
    repeat: usize,
    /// The routine role's models as run, each with its provider's settings and server.
    models: Vec<Value>,
}

/// Runs `suite` as `options` say and writes `report.md` and `results.json` into `out`, after
/// every trial so a run cut short still leaves its results. Returns how many trials failed.
///
/// # Errors
///
/// The profile, the suite or the robot cannot be loaded, or the report cannot be written.
pub(crate) async fn run(
    profile_path: &Path,
    suite_path: &Path,
    options: &Options,
    out: &Path,
) -> Result<usize> {
    let text = std::fs::read_to_string(suite_path)
        .with_context(|| format!("reading {}", suite_path.display()))?;
    let suite: Suite =
        toml::from_str(&text).with_context(|| format!("parsing {}", suite_path.display()))?;
    let profile = Profile::load(profile_path).context("loading the profile")?;
    let models =
        ModelsConfig::load(&profile.resolve(&profile.models.file)).context("loading the models")?;
    if let Some(m) = options.models.iter().find(|m| models.model(m).is_none()) {
        anyhow::bail!("models.toml has no model {m}");
    }
    let robot = nervros_core::app::connect(&profile)?;
    std::fs::create_dir_all(out).with_context(|| format!("creating {}", out.display()))?;
    let ledger = nervros_core::app::state_dir().join("quota.json");
    let arms: Vec<Option<&str>> = if options.models.is_empty() {
        vec![None]
    } else {
        options.models.iter().map(|m| Some(m.as_str())).collect()
    };
    let provenance = Provenance {
        started: humantime::format_rfc3339_seconds(std::time::SystemTime::now()).to_string(),
        nervros: checkout(),
        profile: profile_path.display().to_string(),
        suite: suite_path.display().to_string(),
        suite_sha256: hex(&Sha256::digest(text.as_bytes()))[..12].to_owned(),
        repeat: options.repeat,
        models: describe_models(&models, &options.models).await,
    };
    // Discovery and the cameras need a moment after the node starts.
    tokio::time::sleep(Duration::from_secs(3)).await;
    let run = Run {
        profile_path,
        profile: &profile,
        robot: &robot,
        ledger: &ledger,
        truth: suite.truth.as_ref(),
        out,
    };
    let mut trials = Vec::new();
    let cases = suite
        .cases
        .iter()
        .filter(|c| options.only.as_deref().is_none_or(|o| c.id.contains(o)));
    for case in cases {
        for trial in 1..=options.repeat {
            for model in &arms {
                eprintln!(
                    "== {} #{trial}{}",
                    case.id,
                    model.map_or(String::new(), |m| format!(" on {m}"))
                );
                let seen = run_case(&run, case, *model, trial).await?;
                let problems = judge(&case.expect, &seen);
                let passed = problems.is_empty();
                eprintln!(
                    "   {} in {:.0} s, {} calls{}",
                    if passed { "PASS" } else { "FAIL" },
                    seen.seconds,
                    seen.calls,
                    if passed {
                        String::new()
                    } else {
                        format!(": {}", problems.join("; "))
                    }
                );
                trials.push(Trial {
                    case: case.id.clone(),
                    model: model.map(str::to_owned),
                    attempt: trial,
                    passed,
                    problems,
                    notes: notes(&seen),
                    seen,
                });
                save(out, &provenance, &trials)?;
            }
        }
    }
    let failed = trials.iter().filter(|t| !t.passed).count();
    eprintln!(
        "{} of {} trials passed; report {}",
        trials.len() - failed,
        trials.len(),
        out.join("report.md").display()
    );
    Ok(failed)
}

fn save(out: &Path, provenance: &Provenance, trials: &[Trial]) -> Result<()> {
    let results = json!({"provenance": provenance, "trials": trials});
    std::fs::write(
        out.join("results.json"),
        serde_json::to_string_pretty(&results)?,
    )?;
    std::fs::write(out.join("report.md"), report(provenance, trials))?;
    Ok(())
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

/// The NervROS checkout this binary was built in, as it is now: its version, commit, and whether
/// it has changes not committed.
fn checkout() -> String {
    let git = |args: &[&str]| {
        std::process::Command::new("git")
            .arg("-C")
            .arg(env!("CARGO_MANIFEST_DIR"))
            .args(args)
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_owned())
    };
    let commit = git(&["rev-parse", "--short", "HEAD"]).unwrap_or_else(|| "unknown".to_owned());
    let changed =
        git(&["status", "--porcelain", "--untracked-files=no"]).is_some_and(|s| !s.is_empty());
    format!(
        "{} at {commit}{}",
        env!("CARGO_PKG_VERSION"),
        if changed { " with changes" } else { "" }
    )
}

/// Each model the routine role can run on here: its provider's settings, the request fields it
/// sends (sampling, seed), and for a local llama.cpp server its build, model file and template.
async fn describe_models(models: &ModelsConfig, pinned: &[String]) -> Vec<Value> {
    let ids = if pinned.is_empty() {
        models.roles.routine.as_slice()
    } else {
        pinned
    };
    let mut described = Vec::new();
    for id in ids {
        let Some(m) = models.model(id) else { continue };
        let provider = models.providers.iter().find(|p| p.id == m.provider);
        let base_url = provider.and_then(|p| p.base_url.clone());
        let mut entry = json!({"id": id, "provider": m.provider, "model": m.model,
            "base_url": base_url, "params": m.params});
        let local = base_url.as_deref().filter(|u| {
            provider.is_some_and(|p| p.kind == ProviderKind::OpenaiCompat)
                && (u.contains("://127.0.0.1") || u.contains("://localhost"))
        });
        if let Some(url) = local
            && let Some(props) = server_props(url).await
        {
            entry["server"] = props;
        }
        described.push(entry);
    }
    described
}

/// What a llama.cpp server says of itself at `/props`.
async fn server_props(base_url: &str) -> Option<Value> {
    let root = base_url.trim_end_matches('/').trim_end_matches("/v1");
    let props: Value = reqwest::Client::new()
        .get(format!("{root}/props"))
        .timeout(Duration::from_secs(2))
        .send()
        .await
        .ok()?
        .json()
        .await
        .ok()?;
    let template = props["chat_template"].as_str().unwrap_or_default();
    Some(json!({
        "build": props["build_info"],
        "model_path": props["model_path"],
        "context": props["default_generation_settings"]["n_ctx"],
        "template_sha256": hex(&Sha256::digest(template.as_bytes()))[..12],
    }))
}

/// Where the base is on the map: x, y and yaw.
fn pose(robot: &Arc<dyn RobotPort>, profile: &Profile) -> Option<(f64, f64, f64)> {
    let t = robot
        .transform(&profile.ros.map_frame, &profile.ros.base_frame)
        .ok()?;
    Some((t.translation[0], t.translation[1], t.yaw()))
}

/// What every trial shares.
struct Run<'a> {
    profile_path: &'a Path,
    profile: &'a Profile,
    robot: &'a Arc<dyn RobotPort>,
    ledger: &'a Path,
    truth: Option<&'a Truth>,
    out: &'a Path,
}

/// Where the simulator says the base is: x, y and yaw.
async fn truth_pose(robot: &Arc<dyn RobotPort>, truth: &Truth) -> Option<(f64, f64, f64)> {
    let odom = robot
        .latest(
            &truth.topic,
            "nav_msgs/msg/Odometry",
            Duration::from_secs(2),
        )
        .await
        .ok()?;
    let (p, q) = (
        &odom["pose"]["pose"]["position"],
        &odom["pose"]["pose"]["orientation"],
    );
    let n = |v: &Value| v.as_f64();
    let t = Transform {
        translation: [n(&p["x"])?, n(&p["y"])?, 0.0],
        rotation: [n(&q["x"])?, n(&q["y"])?, n(&q["z"])?, n(&q["w"])?],
    };
    Some((t.translation[0], t.translation[1], t.yaw()))
}

/// How far the base went from `a` to `b`, metres, and how far it turned, degrees, left positive.
fn travel(a: (f64, f64, f64), b: (f64, f64, f64)) -> (f64, f64) {
    let turn = (b.2 - a.2 + PI).rem_euclid(TAU) - PI;
    ((b.0 - a.0).hypot(b.1 - a.1), turn.to_degrees())
}

async fn run_case(run: &Run<'_>, case: &Case, model: Option<&str>, trial: usize) -> Result<Seen> {
    let options = nervros_core::app::StartOptions {
        model: model.map(str::to_owned),
        ..Default::default()
    };
    let agent =
        nervros_core::app::start_with(run.profile_path, Arc::clone(run.robot), run.ledger, options)
            .context("starting the agent")?;
    let name = format!("case-{}-{}-{trial}", case.id, model.unwrap_or("chain"));
    let (_path, _log) = nervros_core::log::spawn(run.out, &name, agent.session.subscribe())
        .context("opening the case log")?;
    let mut events = agent.session.subscribe();
    agent.session.send(Command::Arm);
    let mut seen = Seen::default();
    let deadline = Instant::now() + Duration::from_secs(case.max_s);
    for text in &case.setup {
        let mut ignored = Seen::default();
        say(&agent.session, &mut events, text, deadline, &mut ignored).await;
    }
    let start = pose(run.robot, run.profile);
    let truth_start = match run.truth {
        Some(t) => truth_pose(run.robot, t).await,
        None => None,
    };
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
    if let (Some(a), Some(b)) = (start, pose(run.robot, run.profile)) {
        let (moved, turned) = travel(a, b);
        (seen.moved_m, seen.turned_deg) = (Some(moved), Some(turned));
    }
    if let (Some(t), Some(a)) = (run.truth, truth_start)
        && let Some(b) = truth_pose(run.robot, t).await
    {
        let (moved, turned) = travel(a, b);
        (seen.truth_moved_m, seen.truth_turned_deg) = (Some(moved), Some(turned));
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
            Event::ModelCall {
                input_tokens,
                cached_tokens,
                output_tokens,
                ms,
                ..
            } => {
                seen.calls += 1;
                seen.input_tokens += input_tokens;
                seen.cached_tokens += cached_tokens;
                seen.output_tokens += output_tokens;
                seen.model_ms += ms;
            }
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
    // The simulator's truth when there is one: the robot's estimate is what may be wrong.
    if let Some(range) = expect.moved_m {
        within(
            seen.truth_moved_m.or(seen.moved_m),
            range,
            "moved m",
            &mut problems,
        );
    }
    if let Some(want) = expect.compacted
        && want != (seen.compactions > 0)
    {
        problems.push(format!("compacted {} times", seen.compactions));
    }
    if let Some(range) = expect.turned_deg {
        within(
            seen.truth_turned_deg.or(seen.turned_deg),
            range,
            "turned deg",
            &mut problems,
        );
    }
    problems
}

/// Where the robot's estimate of its travel disagrees with the simulator's.
fn notes(seen: &Seen) -> Vec<String> {
    let mut notes = Vec::new();
    if let (Some(map), Some(truth)) = (seen.moved_m, seen.truth_moved_m)
        && (map - truth).abs() > DRIFT_M
    {
        notes.push(format!(
            "the map says it moved {map:.2} m, the simulator {truth:.2} m"
        ));
    }
    if let (Some(map), Some(truth)) = (seen.turned_deg, seen.truth_turned_deg)
        && (map - truth).abs() > DRIFT_DEG
    {
        notes.push(format!(
            "the map says it turned {map:.0} deg, the simulator {truth:.0} deg"
        ));
    }
    notes
}

/// The chance that `k` trials drawn from `n`, `c` of which passed, all pass: C(c,k)/C(n,k), an
/// unbiased estimate of pass^k.
fn pass_hat(c: usize, n: usize, k: usize) -> f64 {
    (0..k.min(n))
        .map(|i| real(c.saturating_sub(i)) / real(n - i))
        .product()
}

/// The 95 % Wilson interval of `c` passes in `n`.
fn wilson(c: usize, n: usize) -> (f64, f64) {
    const Z: f64 = 1.96;
    let (c, n) = (real(c), real(n.max(1)));
    let p = c / n;
    let d = 1.0 + Z * Z / n;
    let centre = (p + Z * Z / (2.0 * n)) / d;
    let half = Z * (p * (1.0 - p) / n + Z * Z / (4.0 * n * n)).sqrt() / d;
    ((centre - half).max(0.0), (centre + half).min(1.0))
}

fn real(n: impl TryInto<u32>) -> f64 {
    f64::from(n.try_into().unwrap_or(u32::MAX))
}

/// The ids in the order trials first ran them.
fn in_order<T: PartialEq>(ids: impl Iterator<Item = T>) -> Vec<T> {
    ids.fold(Vec::new(), |mut seen, id| {
        if !seen.contains(&id) {
            seen.push(id);
        }
        seen
    })
}

fn arm(model: Option<&String>) -> &str {
    model.map_or("chain", String::as_str)
}

fn report(provenance: &Provenance, trials: &[Trial]) -> String {
    let arms = in_order(trials.iter().map(|t| t.model.as_ref()));
    let cases = in_order(trials.iter().map(|t| t.case.as_str()));
    let mut out = format!("# Eval: {}\n\n", provenance.suite);
    let _ = writeln!(
        out,
        "Started {}, NervROS {}, profile {}, suite sha256 {}, {} trial(s) per case and model.\n",
        provenance.started,
        provenance.nervros,
        provenance.profile,
        provenance.suite_sha256,
        provenance.repeat
    );
    for m in &provenance.models {
        let _ = writeln!(out, "- `{m}`");
    }
    let k = provenance.repeat;
    let _ = write!(
        out,
        "\n## Summary\n\n| Model | Trials | pass^1 (95 % CI) | pass^{k} | Calls | Model s | Tokens in | Cached | Tokens out |\n|---|---|---|---|---|---|---|---|---|\n"
    );
    for a in &arms {
        let mine: Vec<&Trial> = trials.iter().filter(|t| t.model.as_ref() == *a).collect();
        summary_row(&mut out, arm(*a), &mine, &cases, k);
    }
    out.push_str("\nCalls, model seconds and tokens are means per trial.\n\n## Cases\n\n| Case |");
    for a in &arms {
        let _ = write!(out, " {} |", arm(*a));
    }
    out.push_str(" s | Problems and notes |\n|---|");
    out.push_str(&"---|".repeat(arms.len() + 2));
    out.push('\n');
    for id in &cases {
        case_row(&mut out, id, &arms, trials);
    }
    out.push_str("\n## Last replies\n");
    for id in &cases {
        for a in &arms {
            let last = trials
                .iter()
                .rev()
                .find(|t| t.case == *id && t.model.as_ref() == *a);
            if let Some(t) = last {
                let reply = t.seen.replies.last().map_or("(none)", String::as_str);
                let _ = writeln!(
                    out,
                    "\n**{id}** ({}): {}",
                    arm(*a),
                    reply.replace('\n', " ")
                );
            }
        }
    }
    out
}

/// One model's line of the summary: its pass rate with an interval, pass^k over the cases, and
/// what a trial cost on average.
fn summary_row(out: &mut String, model: &str, mine: &[&Trial], cases: &[&str], k: usize) {
    let n = mine.len();
    let c = mine.iter().filter(|t| t.passed).count();
    let (lo, hi) = wilson(c, n);
    let per_case: Vec<f64> = cases
        .iter()
        .filter_map(|id| {
            let runs: Vec<&&Trial> = mine.iter().filter(|t| t.case == *id).collect();
            let passed = runs.iter().filter(|t| t.passed).count();
            (!runs.is_empty()).then(|| pass_hat(passed, runs.len(), k))
        })
        .collect();
    let mean = |f: &dyn Fn(&Seen) -> u64| {
        real(mine.iter().map(|t| f(&t.seen)).sum::<u64>()) / real(n.max(1))
    };
    let (input, cached) = (mean(&|s| s.input_tokens), mean(&|s| s.cached_tokens));
    let _ = writeln!(
        out,
        "| {model} | {n} | {:.2} ({lo:.2}-{hi:.2}) | {:.2} | {:.1} | {:.0} | {input:.0} | {:.0} % | {:.0} |",
        real(c) / real(n.max(1)),
        per_case.iter().sum::<f64>() / real(per_case.len().max(1)),
        mean(&|s| s.calls),
        mean(&|s| s.model_ms) / 1000.0,
        if input > 0.0 {
            cached / input * 100.0
        } else {
            0.0
        },
        mean(&|s| s.output_tokens),
    );
}

/// One case's line: passes per model, mean seconds, and what went wrong or looked off.
fn case_row(out: &mut String, id: &str, arms: &[Option<&String>], trials: &[Trial]) {
    let runs: Vec<&Trial> = trials.iter().filter(|t| t.case == id).collect();
    let _ = write!(out, "| {id} |");
    for a in arms {
        let mine: Vec<&&Trial> = runs.iter().filter(|t| t.model.as_ref() == *a).collect();
        let c = mine.iter().filter(|t| t.passed).count();
        let _ = write!(out, " {c}/{} |", mine.len());
    }
    let seconds = runs.iter().map(|t| t.seen.seconds).sum::<f64>() / real(runs.len().max(1));
    let said: BTreeSet<String> = runs
        .iter()
        .flat_map(|t| {
            let who = if arms.len() > 1 {
                format!("{}: ", arm(t.model.as_ref()))
            } else {
                String::new()
            };
            t.problems
                .iter()
                .chain(&t.notes)
                .map(move |p| format!("{who}{p}"))
        })
        .collect();
    let said: Vec<String> = said.into_iter().collect();
    let _ = writeln!(
        out,
        " {seconds:.0} | {} |",
        said.join("; ").replace('|', "/")
    );
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

    #[test]
    fn the_simulator_judges_travel_and_a_drifting_estimate_is_noted() {
        let expect: Expect = toml::from_str("moved_m = [0.7, 1.3]\n").unwrap();
        let seen = Seen {
            moved_m: Some(1.0),
            truth_moved_m: Some(0.1),
            ..Seen::default()
        };
        assert_eq!(judge(&expect, &seen), ["moved m 0.10, wanted 0.7..1.3"]);
        assert_eq!(
            notes(&seen),
            ["the map says it moved 1.00 m, the simulator 0.10 m"]
        );
        let (moved, turned) = travel((0.0, 0.0, 3.0), (3.0, 4.0, -3.0));
        assert!(
            (moved - 5.0).abs() < 1e-9 && (turned - 16.2).abs() < 0.1,
            "{turned}"
        );
    }

    #[test]
    fn pass_hat_k_is_unbiased_and_the_interval_holds_the_rate() {
        assert!((pass_hat(3, 3, 3) - 1.0).abs() < 1e-12);
        assert!(
            (pass_hat(2, 4, 2) - 1.0 / 6.0).abs() < 1e-12,
            "C(2,2)/C(4,2)"
        );
        assert!(pass_hat(1, 3, 2).abs() < 1e-12);
        assert!((pass_hat(2, 4, 1) - 0.5).abs() < 1e-12);
        let (lo, hi) = wilson(8, 10);
        assert!(
            (lo - 0.490).abs() < 0.005 && (hi - 0.943).abs() < 0.005,
            "{lo} {hi}"
        );
        let (lo, hi) = wilson(0, 5);
        assert!(lo.abs() < 1e-12 && hi < 0.5, "{hi}");
    }

    #[test]
    fn the_report_shows_each_model_side_by_side() {
        let provenance = Provenance {
            started: "2026-10-01T20:00:00Z".into(),
            nervros: "0.1.0 at abc1234".into(),
            profile: "g1.toml".into(),
            suite: "apartment.toml".into(),
            suite_sha256: "0123456789ab".into(),
            repeat: 2,
            models: vec![json!({"id": "qwen"})],
        };
        let trial = |model: &str, n, passed, cached| Trial {
            case: "walk".into(),
            model: Some(model.into()),
            attempt: n,
            passed,
            problems: if passed {
                vec![]
            } else {
                vec!["timed out".into()]
            },
            notes: vec![],
            seen: Seen {
                calls: 2,
                input_tokens: 1000,
                cached_tokens: cached,
                ..Seen::default()
            },
        };
        let trials = [
            trial("qwen", 1, true, 800),
            trial("gemma", 1, false, 0),
            trial("qwen", 2, true, 800),
            trial("gemma", 2, true, 0),
        ];
        let text = report(&provenance, &trials);
        assert!(
            text.contains("| qwen | 2 | 1.00 (0.34-1.00) | 1.00 | 2.0 |"),
            "{text}"
        );
        assert!(
            text.contains("| gemma | 2 | 0.50 (0.09-0.91) | 0.00 |"),
            "{text}"
        );
        assert!(text.contains("| 80 % |"), "{text}");
        assert!(text.contains("| walk | 2/2 | 1/2 |"), "{text}");
        assert!(text.contains("gemma: timed out"), "{text}");
    }
}
