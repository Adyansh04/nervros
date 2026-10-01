//! The real window against the live robot and model, driven as an operator drives it: each
//! prompt goes in the composer and out with Send, each approval card is approved, and the window
//! is saved as a PNG at every stage for a person to look over, with the dock's tabs at the end.
//!
//! Ignored by default: it needs the robot's stack, the model and a profile.
//!
//! ```bash
//! NERVROS_PROFILE=<nervros.toml> NERVROS_GUI_EVAL_OUT=<dir> \
//!   cargo test -p nervros-gui live_eval -- --ignored --nocapture
//! ```
//!
//! `NERVROS_GUI_EVAL_PROMPTS` gives the prompts, one per line, in place of the defaults.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use egui_kittest::Harness;
use egui_kittest::kittest::Queryable as _;
use rerun::external::{egui, re_memory, re_viewer};

use super::{Gui, Tab, ViewerFeed};

const PROMPTS: [&str; 4] = [
    "What can you see right now?",
    "Where is the red mug?",
    "Turn left 90 degrees.",
    "Walk forward half a metre.",
];
/// A prompt not settled by then is a finding of its own.
const PER_PROMPT: Duration = Duration::from_mins(4);
/// Settled means quiet this long: a mission's report starts a turn of its own a moment after.
const QUIET: Duration = Duration::from_secs(4);

/// Frames while the agent's tasks run on the runtime's own threads.
fn wait(harness: &mut Harness<'_, Gui>, for_: Duration) {
    let until = Instant::now() + for_;
    while Instant::now() < until {
        harness.run_steps(1);
        std::thread::sleep(Duration::from_millis(50));
    }
}

fn save(harness: &mut Harness<'_, Gui>, out: &Path, name: &str) {
    match harness.render() {
        Ok(image) => {
            let path = out.join(format!("{name}.png"));
            image
                .save(&path)
                .unwrap_or_else(|e| eprintln!("{}: {e}", path.display()));
            eprintln!("saved {}", path.display());
        }
        Err(e) => eprintln!("{name}: not rendered: {e}"),
    }
}

/// Approves what waits, saving the card first; true when something was approved.
fn approve(harness: &mut Harness<'_, Gui>, out: &Path, n: usize) -> bool {
    if harness.state().chat.pending().count() == 0 {
        return false;
    }
    save(harness, out, &format!("{n}-approval"));
    let button = harness.query_all_by_label("Approve").next();
    match button {
        Some(b) => {
            b.click();
            harness.run_steps(2);
            true
        }
        None => false,
    }
}

/// Sends `prompt` and steps until the session is quiet: no turn, no approval waiting and no
/// mission running.
fn converse(harness: &mut Harness<'_, Gui>, out: &Path, n: usize, prompt: &str) {
    eprintln!("== {n}: {prompt}");
    harness.state_mut().input = prompt.to_owned();
    harness.run_steps(1);
    harness.get_by_label("Send").click();
    harness.run_steps(2);
    wait(harness, Duration::from_secs(2));
    save(harness, out, &format!("{n}-sent"));
    let (began, mut quiet_since, mut saw_mission) = (Instant::now(), None, false);
    while began.elapsed() < PER_PROMPT {
        wait(harness, Duration::from_millis(250));
        approve(harness, out, n);
        let running = harness.state().mission_running();
        if running && !saw_mission {
            saw_mission = true;
            wait(harness, Duration::from_secs(3));
            save(harness, out, &format!("{n}-running"));
        }
        let gui = harness.state();
        let busy = gui.chat.turn.is_some() || gui.chat.pending().count() > 0 || running;
        match (busy, quiet_since) {
            (true, _) => quiet_since = None,
            (false, None) => quiet_since = Some(Instant::now()),
            (false, Some(t)) if t.elapsed() >= QUIET => break,
            (false, Some(_)) => {}
        }
    }
    if began.elapsed() >= PER_PROMPT {
        eprintln!("   not settled in {} s", PER_PROMPT.as_secs());
    }
    save(harness, out, &format!("{n}-done"));
}

#[test]
#[ignore = "needs the robot's stack, the model and NERVROS_PROFILE"]
fn live_eval() {
    let profile_path = PathBuf::from(std::env::var("NERVROS_PROFILE").expect("NERVROS_PROFILE"));
    let out = PathBuf::from(
        std::env::var("NERVROS_GUI_EVAL_OUT").unwrap_or_else(|_| "gui-eval".to_owned()),
    );
    std::fs::create_dir_all(&out).expect("the output folder");
    let prompts: Vec<String> = std::env::var("NERVROS_GUI_EVAL_PROMPTS").map_or_else(
        |_| PROMPTS.map(str::to_owned).to_vec(),
        |p| {
            p.lines()
                .filter(|l| !l.trim().is_empty())
                .map(str::to_owned)
                .collect()
        },
    );
    let runtime = tokio::runtime::Runtime::new().expect("tokio");
    let _entered = runtime.enter();
    let profile = nervros_core::profile::Profile::load(&profile_path).expect("the profile");
    let robot = nervros_core::app::connect(&profile).expect("the ROS node");
    let options = nervros_core::app::StartOptions {
        history: Some(out.join("session.history.json")),
        ..Default::default()
    };
    let state = nervros_core::app::state_dir();
    let agent =
        nervros_core::app::start_with(&profile_path, robot, &state.join("quota.json"), options)
            .expect("the agent");
    let (log, _log) =
        nervros_core::log::spawn(&out, "session", agent.session.subscribe()).expect("the log");
    let (rec, input) = nervros_viz::in_process().expect("the recording");
    let bridge = nervros_viz::spawn(
        &rec,
        &agent.robot,
        &agent.profile,
        agent.session.subscribe(),
    );
    agent.session.send(nervros_core::session::Command::Arm);
    let handle = runtime.handle().clone();
    let mut harness = Harness::builder()
        .wgpu()
        .with_size(egui::vec2(1600.0, 960.0))
        .build_eframe(|cc| {
            let feed = ViewerFeed {
                input,
                bridge,
                memory_limit: re_memory::MemoryLimit::from_bytes(4 << 30),
            };
            let token = re_viewer::MainThreadToken::i_promise_i_am_only_using_this_for_a_test();
            Gui::start(token, cc, agent, feed, handle, log).expect("the window")
        });
    // The map, the camera and the world model need a moment to arrive.
    wait(&mut harness, Duration::from_secs(10));
    save(&mut harness, &out, "0-start");
    for (i, prompt) in prompts.iter().enumerate() {
        converse(&mut harness, &out, i + 1, prompt);
    }
    for (tab, name) in Tab::ALL {
        harness.state_mut().tab = tab;
        wait(&mut harness, Duration::from_secs(1));
        save(&mut harness, &out, &format!("tab-{}", name.to_lowercase()));
    }
    drop(rec);
}
