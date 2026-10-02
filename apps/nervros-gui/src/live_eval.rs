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
//! `NERVROS_GUI_EVAL_VIDEO=1` also records each prompt as an MP4 at 20 frames a second, through
//! `ffmpeg`, rendered offscreen as the PNGs are.

use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
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
/// Frames a second, of the window's steps and of the videos.
const FPS: u32 = 20;

/// A video of the window through `ffmpeg`, kept in step with the clock: a frame that renders late
/// is written again rather than slowing the video down.
struct Video {
    ffmpeg: Child,
    started: Instant,
    frames: u64,
}

impl Video {
    fn start(path: &Path, (width, height): (u32, u32)) -> Option<Self> {
        let ffmpeg = Command::new("ffmpeg")
            .args([
                "-loglevel",
                "error",
                "-y",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgba",
            ])
            .args([
                "-s",
                &format!("{width}x{height}"),
                "-r",
                &FPS.to_string(),
                "-i",
                "-",
            ])
            // Small files: the window scaled to 1280 wide, H.264 that browsers and GitHub play.
            .args([
                "-vf",
                "scale=1280:-2:flags=lanczos",
                "-c:v",
                "libx264",
                "-preset",
                "slow",
            ])
            .args([
                "-crf",
                "26",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
            ])
            .arg(path)
            .stdin(Stdio::piped())
            .spawn();
        match ffmpeg {
            Ok(ffmpeg) => Some(Self {
                ffmpeg,
                started: Instant::now(),
                frames: 0,
            }),
            Err(e) => {
                eprintln!("no video: ffmpeg did not start: {e}");
                None
            }
        }
    }

    fn frame(&mut self, image: &image::RgbaImage) {
        let ms = u64::try_from(self.started.elapsed().as_millis()).unwrap_or(u64::MAX);
        let due = ms.saturating_mul(u64::from(FPS)) / 1000 + 1;
        let stdin = self.ffmpeg.stdin.as_mut().expect("ffmpeg's input");
        while self.frames < due {
            stdin
                .write_all(image.as_raw())
                .expect("ffmpeg takes the frame");
            self.frames += 1;
        }
    }

    fn finish(mut self) {
        drop(self.ffmpeg.stdin.take());
        let _ = self.ffmpeg.wait();
    }
}

/// The window being driven, where its pictures go, and the video being recorded, if any.
struct Run<'a> {
    harness: Harness<'a, Gui>,
    out: PathBuf,
    record: bool,
    video: Option<Video>,
}

impl Run<'_> {
    /// Frames while the agent's tasks run on the runtime's own threads, each one recorded when a
    /// video runs.
    fn wait(&mut self, for_: Duration) {
        let period = Duration::from_secs(1) / FPS;
        let until = Instant::now() + for_;
        while Instant::now() < until {
            let tick = Instant::now();
            self.harness.run_steps(1);
            if let Some(video) = &mut self.video
                && let Ok(image) = self.harness.render()
            {
                video.frame(&image);
            }
            std::thread::sleep(period.saturating_sub(tick.elapsed()));
        }
    }

    fn save(&mut self, name: &str) {
        match self.harness.render() {
            Ok(image) => {
                let path = self.out.join(format!("{name}.png"));
                image
                    .save(&path)
                    .unwrap_or_else(|e| eprintln!("{}: {e}", path.display()));
                eprintln!("saved {}", path.display());
            }
            Err(e) => eprintln!("{name}: not rendered: {e}"),
        }
    }

    /// Approves what waits, saving the card first; true when something was approved.
    fn approve(&mut self, n: usize) -> bool {
        if self.harness.state().chat.pending().count() == 0 {
            return false;
        }
        // The card stays a moment in the video, as an operator would read it.
        if self.video.is_some() {
            self.wait(Duration::from_secs(2));
        }
        self.save(&format!("{n}-approval"));
        let button = self.harness.query_all_by_label("Approve").next();
        match button {
            Some(b) => {
                b.click();
                self.harness.run_steps(2);
                true
            }
            None => false,
        }
    }

    /// Sends `prompt` and steps until the session is quiet: no turn, no approval waiting and no
    /// mission running.
    fn converse(&mut self, n: usize, prompt: &str) {
        eprintln!("== {n}: {prompt}");
        if self.record {
            let size = self
                .harness
                .render()
                .map_or((1600, 960), |i| i.dimensions());
            self.video = Video::start(&self.out.join(format!("{n}.mp4")), size);
        }
        self.harness.state_mut().input = prompt.to_owned();
        self.wait(Duration::from_secs(1));
        self.harness.get_by_label("Send").click();
        self.harness.run_steps(2);
        self.wait(Duration::from_secs(2));
        self.save(&format!("{n}-sent"));
        let (began, mut quiet_since, mut saw_mission) = (Instant::now(), None, false);
        while began.elapsed() < PER_PROMPT {
            self.wait(Duration::from_millis(250));
            self.approve(n);
            let running = self.harness.state().mission_running();
            if running && !saw_mission {
                saw_mission = true;
                self.wait(Duration::from_secs(3));
                self.save(&format!("{n}-running"));
            }
            let gui = self.harness.state();
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
        self.save(&format!("{n}-done"));
        if let Some(video) = self.video.take() {
            video.finish();
        }
    }
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
    let record = std::env::var_os("NERVROS_GUI_EVAL_VIDEO").is_some_and(|v| v == "1");
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
    let events = agent.session.subscribe();
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
    let harness = Harness::builder()
        .wgpu()
        .with_size(egui::vec2(1600.0, 960.0))
        .build_eframe(|cc| {
            let feed = ViewerFeed {
                input,
                bridge,
                memory_limit: re_memory::MemoryLimit::from_bytes(4 << 30),
            };
            let token = re_viewer::MainThreadToken::i_promise_i_am_only_using_this_for_a_test();
            Gui::start(token, cc, agent, events, feed, handle, log).expect("the window")
        });
    let mut run = Run {
        harness,
        out,
        record,
        video: None,
    };
    // The map, the camera and the world model need a moment to arrive.
    run.wait(Duration::from_secs(10));
    run.save("0-start");
    for (i, prompt) in prompts.iter().enumerate() {
        run.converse(i + 1, prompt);
    }
    for (tab, name) in Tab::ALL {
        run.harness.state_mut().tab = tab;
        run.wait(Duration::from_secs(1));
        run.save(&format!("tab-{}", name.to_lowercase()));
    }
    // The 3D view close on the robot, as Follow robot shows it.
    run.harness.state().bridge.follow(true);
    run.wait(Duration::from_secs(3));
    run.save("follow");
    drop(rec);
}
