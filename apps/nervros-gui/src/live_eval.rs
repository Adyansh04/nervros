//! The real window against the live robot and model, driven by a script as an operator drives
//! it: prompts typed into the composer and sent, approval cards approved, dock tabs, the 3D view's
//! follow mode and clicks on widgets, with PNGs where the script asks and, when asked, every frame
//! recorded for a clip.
//!
//! Ignored by default: it needs the robot's stack, the model and a profile.
//!
//! ```bash
//! NERVROS_PROFILE=<nervros.toml> NERVROS_GUI_EVAL_OUT=<dir> NERVROS_GUI_EVAL_SCRIPT=<file> \
//!   cargo test -p nervros-gui live_eval -- --ignored --nocapture
//! ```
//!
//! `NERVROS_GUI_EVAL_MODEL=<id>` talks on that model of `models.toml` alone. The run keeps the
//! robot's files (missions, zones, programs, incidents) in `<dir>/state`.
//!
//! A script has one step per line: a prompt, typed and sent, then waited on until the session is
//! quiet, each approval approved; or a `:command` (see [`Step`]). Settings at its top
//! (`:follow`, `:tab`, `:edit`, `:approve`, `:speed`, `:missions`) apply before recording starts. Without a
//! script the window answers four prompts and saves each dock tab.
//!
//! `NERVROS_GUI_EVAL_VIDEO=<name>` records the script as `<name>.mkv`, every frame the window
//! renders stamped with when it rendered, `<name>.speed.json`, the playback speed the script
//! asked for each stretch, and `<name>.moments.json`, when the recording started and the stretches
//! where the pointer travels and clicks, which play at normal speed. An encoder makes the clip from
//! them: a slow frame then lasts longer instead of being written again.
//!
//! The pointer stays in the window as an operator's would: it waits in the message field, travels
//! to what a step clicks, and comes back.

use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::time::{Duration, Instant};

use egui_kittest::Harness;
use egui_kittest::kittest::Queryable as _;
use rerun::external::eframe::{egui_wgpu, wgpu};
use rerun::external::egui::{Key, Modifiers};
use rerun::external::re_viewer::SystemCommandSender as _;
use rerun::external::{egui, re_entity_db, re_log_types, re_memory, re_viewer, re_viewer_context};

use super::{COMPOSER, Gui, Picked, Tab, ViewerFeed};

const PROMPTS: [&str; 4] = [
    "What can you see right now?",
    "Where is the red mug?",
    "Turn left 90 degrees.",
    "Walk forward half a metre.",
];
/// A prompt not settled by then is a finding of its own, unless `:limit` gives longer.
const PER_PROMPT: Duration = Duration::from_mins(6);
/// Settled means quiet this long: no turn, no mission, nothing new in the chat. A mission's
/// report starts a turn of its own a moment after the mission ends.
const QUIET: Duration = Duration::from_secs(5);
/// How fast a prompt is typed, in characters a second.
const TYPING: f64 = 24.0;
/// How long an approval card shows before it is approved, as an operator would read it.
const READING: Duration = Duration::from_millis(2500);
/// How fast the pointer travels to what it clicks, in points a second, within the shortest and
/// longest move a hand makes.
const GLIDE: f32 = 1400.0;
const GLIDE_SECONDS: (f32, f32) = (0.35, 0.95);

/// The machine's GPU: kittest's own setup prefers a software rasterizer, for snapshots that match
/// on every machine.
fn gpu() -> egui_wgpu::WgpuSetup {
    let mut setup = egui_wgpu::WgpuSetupCreateNew::without_display_handle();
    setup.native_adapter_selector = Some(Arc::new(|adapters, _surface| {
        adapters
            .iter()
            .max_by_key(|a| match a.get_info().device_type {
                wgpu::DeviceType::DiscreteGpu => 3,
                wgpu::DeviceType::IntegratedGpu => 2,
                wgpu::DeviceType::Other | wgpu::DeviceType::VirtualGpu => 1,
                wgpu::DeviceType::Cpu => 0,
            })
            .cloned()
            .ok_or_else(|| "no graphics adapter".to_owned())
    }));
    egui_wgpu::WgpuSetup::CreateNew(setup)
}

/// One line of a script.
#[derive(Debug, Clone, PartialEq)]
enum Step {
    /// Typed into the composer and sent; the run waits until the session is quiet.
    Say(String),
    /// `:send text`: typed and sent, and the run goes on at once, as to stop what it starts.
    Send(String),
    /// `:wait 3`: seconds of window.
    Wait(Duration),
    /// `:follow on|off`: the 3D view keeps to the robot, or frames the map.
    Follow(bool),
    /// `:tab world`: a dock tab.
    Tab(Tab),
    /// `:click Go there`: the widget of that label, else the first whose label holds the text.
    Click(String),
    /// `:type text`: typed into the focused field.
    Type(String),
    /// `:key ctrl+k`: a key press, with modifiers.
    Key(Modifiers, Key),
    /// `:pick object O12`, `:pick room R3`, `:pick point 1.5 -2`: as a click in the 3D view.
    Pick(Picked),
    /// `:speed 4`: how much faster the clip plays from here.
    Speed(f64),
    /// `:shot name`: the window as `name.png`.
    Shot(String),
    /// `:edit on|off`: the world editor in place of the viewer.
    Edit(bool),
    /// `:approve on|off`: approve what the agent asks, or leave it to a `:click`.
    Approve(bool),
    /// `:missions 3`: how much faster the clip plays while a mission runs.
    Missions(f64),
    /// `:settle`: wait until the session is quiet, as after a prompt.
    Settle,
    /// `:hold W 2`: a key held down for seconds, as when driving by hand.
    Hold(Key, Duration),
    /// `:focus Name`: the text field of that name gets the keyboard; `:focus 2`, the second.
    Focus(String),
    /// `:limit 40`: minutes a prompt may take to settle, for a long mission such as exploring.
    Limit(Duration),
    /// `:record`: the clip starts here; the steps before it set the scene, unrecorded.
    Record,
    /// `:scroll 600`: the dock scrolled down by that many points (up when negative), with the
    /// wheel, as to reach a button below its fold.
    Scroll(f32),
}

impl Step {
    fn parse(line: &str) -> Result<Self, String> {
        let Some(command) = line.strip_prefix(':') else {
            return Ok(Self::Say(line.to_owned()));
        };
        let (name, rest) = command.split_once(' ').unwrap_or((command, ""));
        let rest = rest.trim();
        let bad = |what: &str| format!("`{line}`: {what}");
        let on = || match rest {
            "on" => Ok(true),
            "off" => Ok(false),
            _ => Err(bad("on or off")),
        };
        match name {
            "wait" => rest
                .parse::<f64>()
                .map(|s| Self::Wait(Duration::from_secs_f64(s)))
                .map_err(|_| bad("seconds")),
            "follow" => on().map(Self::Follow),
            "edit" => on().map(Self::Edit),
            "approve" => on().map(Self::Approve),
            "tab" => Tab::ALL
                .iter()
                .find(|(_, n)| n.eq_ignore_ascii_case(rest))
                .map(|(t, _)| Self::Tab(*t))
                .ok_or_else(|| bad("no such tab")),
            "click" => Ok(Self::Click(rest.to_owned())),
            "type" => Ok(Self::Type(rest.to_owned())),
            "send" if !rest.is_empty() => Ok(Self::Send(rest.to_owned())),
            "key" => key(rest)
                .map(|(m, k)| Self::Key(m, k))
                .ok_or_else(|| bad("a key, as ctrl+k or Enter")),
            "pick" => pick(rest)
                .map(Self::Pick)
                .ok_or_else(|| bad("object <id>, room <id> or point <x> <y>")),
            "speed" => rest
                .parse::<f64>()
                .ok()
                .filter(|s| *s > 0.0)
                .map(Self::Speed)
                .ok_or_else(|| bad("a speed above 0")),
            "shot" if !rest.is_empty() => Ok(Self::Shot(rest.to_owned())),
            "missions" => rest
                .parse::<f64>()
                .ok()
                .filter(|s| *s > 0.0)
                .map(Self::Missions)
                .ok_or_else(|| bad("a speed above 0")),
            "settle" => Ok(Self::Settle),
            "limit" => rest
                .parse::<f64>()
                .ok()
                .filter(|m| *m > 0.0)
                .map(|m| Self::Limit(Duration::from_secs_f64(m * 60.0)))
                .ok_or_else(|| bad("minutes above 0")),
            "hold" => rest
                .split_once(' ')
                .and_then(|(k, secs)| Some((key(k)?.1, secs.trim().parse::<f64>().ok()?)))
                .map(|(k, secs)| Self::Hold(k, Duration::from_secs_f64(secs)))
                .ok_or_else(|| bad("a key and seconds")),
            "focus" if !rest.is_empty() && rest != "0" => Ok(Self::Focus(rest.to_owned())),
            "record" => Ok(Self::Record),
            "scroll" => rest
                .parse::<f32>()
                .ok()
                .filter(|d| d.is_finite())
                .map(Self::Scroll)
                .ok_or_else(|| bad("points to scroll")),
            _ => Err(bad("no such command")),
        }
    }

    /// A setting, which at the top of a script applies before recording starts.
    fn is_setting(&self) -> bool {
        matches!(
            self,
            Self::Follow(_)
                | Self::Tab(_)
                | Self::Edit(_)
                | Self::Approve(_)
                | Self::Speed(_)
                | Self::Missions(_)
                | Self::Limit(_)
        )
    }
}

/// `ctrl+k`, `shift+Enter`, `Escape`.
fn key(text: &str) -> Option<(Modifiers, Key)> {
    let mut modifiers = Modifiers::NONE;
    let mut parts: Vec<&str> = text.split('+').collect();
    let name = parts.pop()?;
    for m in parts {
        modifiers |= match m.to_ascii_lowercase().as_str() {
            "ctrl" => Modifiers::COMMAND | Modifiers::CTRL,
            "shift" => Modifiers::SHIFT,
            "alt" => Modifiers::ALT,
            _ => return None,
        };
    }
    let key = Key::from_name(name).or_else(|| Key::from_name(&name.to_ascii_uppercase()))?;
    Some((modifiers, key))
}

fn pick(text: &str) -> Option<Picked> {
    let words: Vec<&str> = text.split_whitespace().collect();
    match words.as_slice() {
        ["object", id] => Some(Picked::Object((*id).to_owned())),
        ["room", id] => Some(Picked::Room((*id).to_owned())),
        ["point", x, y] => Some(Picked::Point([x.parse().ok()?, y.parse().ok()?])),
        _ => None,
    }
}

/// Every frame the window renders, to `ffmpeg` stamped with when it rendered, and the playback
/// speed of each stretch.
struct Capture {
    ffmpeg: Child,
    base: PathBuf,
    started: Instant,
    /// When it started, in seconds since the Unix epoch, to line the session log up with it.
    epoch: f64,
    /// Seconds into the recording, and the speed from then on.
    speeds: Vec<(f64, f64)>,
    /// Seconds into the recording from and to which the pointer travels and clicks.
    moments: Vec<(f64, f64)>,
    frames: u64,
    last: Instant,
    slowest: Duration,
    /// Time in the window's own frames and in rendering them, over all frames.
    ui: Duration,
    render: Duration,
}

impl Capture {
    fn start(base: &Path, (width, height): (u32, u32), speed: f64) -> Option<Self> {
        let ffmpeg = Command::new("ffmpeg")
            .args([
                "-loglevel",
                "error",
                "-y",
                "-use_wallclock_as_timestamps",
                "1",
            ])
            .args(["-f", "rawvideo", "-pix_fmt", "rgba"])
            .args(["-s", &format!("{width}x{height}"), "-i", "-"])
            // Fast and close to lossless: the clip is encoded from this once the run is over.
            .args(["-c:v", "libx264", "-preset", "ultrafast", "-crf", "12"])
            .args(["-pix_fmt", "yuv444p", "-fps_mode", "passthrough"])
            .arg(base.with_extension("mkv"))
            .stdin(Stdio::piped())
            .spawn();
        match ffmpeg {
            Ok(ffmpeg) => {
                let now = Instant::now();
                let epoch = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0.0, |d| d.as_secs_f64());
                Some(Self {
                    ffmpeg,
                    base: base.to_owned(),
                    started: now,
                    epoch,
                    speeds: vec![(0.0, speed)],
                    moments: Vec::new(),
                    frames: 0,
                    last: now,
                    slowest: Duration::ZERO,
                    ui: Duration::ZERO,
                    render: Duration::ZERO,
                })
            }
            Err(e) => {
                eprintln!("no video: ffmpeg did not start: {e}");
                None
            }
        }
    }

    fn frame(&mut self, image: &image::RgbaImage) {
        let stdin = self.ffmpeg.stdin.as_mut().expect("ffmpeg's input");
        stdin
            .write_all(image.as_raw())
            .expect("ffmpeg takes the frame");
        if self.frames > 0 {
            self.slowest = self.slowest.max(self.last.elapsed());
        }
        self.last = Instant::now();
        self.frames += 1;
    }

    fn speed(&mut self, speed: f64) {
        self.speeds
            .push((self.started.elapsed().as_secs_f64(), speed));
    }

    fn finish(mut self) {
        drop(self.ffmpeg.stdin.take());
        let _ = self.ffmpeg.wait();
        let seconds = self.started.elapsed().as_secs_f64();
        let speeds: Vec<_> = self
            .speeds
            .iter()
            .map(|(at, speed)| serde_json::json!({"at": at, "speed": speed}))
            .collect();
        let moments: Vec<_> = self
            .moments
            .iter()
            .map(|(from, to)| serde_json::json!({"from": from, "to": to}))
            .collect();
        let moments = serde_json::json!({"started": self.epoch, "moments": moments});
        for (extension, value) in [
            ("speed.json", serde_json::json!(speeds)),
            ("moments.json", moments),
        ] {
            let path = self.base.with_extension(extension);
            let json = serde_json::to_string_pretty(&value).unwrap_or_default();
            if let Err(e) = std::fs::write(&path, json) {
                eprintln!("{}: {e}", path.display());
            }
        }
        #[expect(clippy::cast_precision_loss, reason = "a frame count")]
        let fps = self.frames as f64 / seconds.max(1e-3);
        let per = |d: Duration| d.as_millis() / u128::from(self.frames.max(1));
        eprintln!(
            "recorded {}: {} frames in {seconds:.0} s, {fps:.1} a second, the slowest {} ms; \
             per frame {} ms in the window, {} ms rendering",
            self.base.display(),
            self.frames,
            self.slowest.as_millis(),
            per(self.ui),
            per(self.render),
        );
    }
}

/// The window being driven, where its pictures go, and the clip being recorded, if any.
struct Run<'a> {
    harness: Harness<'a, Gui>,
    out: PathBuf,
    capture: Option<Capture>,
    approve: bool,
    /// The clip's speed, and its speed while a mission runs.
    speed: f64,
    mission_speed: Option<f64>,
    /// A mission ran at the last frame.
    in_mission: bool,
    /// How long a prompt may take to settle.
    limit: Duration,
    /// Where the pointer is.
    pointer: egui::Pos2,
    /// Steps move the pointer and click, as an operator would; the settings at the top of a
    /// script, before the window is up, act directly.
    moving: bool,
}

impl Run<'_> {
    /// One frame of the window: recorded while a clip records, else paced to some 20 a second
    /// while the agent's tasks run on the runtime's own threads.
    fn frame(&mut self) {
        let began = Instant::now();
        self.harness.run_steps(1);
        let ui = began.elapsed();
        let running = self.harness.state().mission_running();
        if running != self.in_mission {
            self.in_mission = running;
            let speed = if running {
                self.mission_speed.unwrap_or(self.speed)
            } else {
                self.speed
            };
            if let Some(capture) = &mut self.capture {
                capture.speed(speed);
            }
        }
        match &mut self.capture {
            Some(capture) => {
                let began = Instant::now();
                if let Ok(image) = self.harness.render() {
                    capture.render += began.elapsed();
                    capture.ui += ui;
                    capture.frame(&image);
                }
            }
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    }

    fn wait(&mut self, for_: Duration) {
        let until = Instant::now() + for_;
        while Instant::now() < until {
            self.frame();
        }
    }

    /// The window as a PNG, without the pointer, as a screenshot leaves the cursor out. The steps
    /// that hide and bring back the pointer are not recorded, so the clip does not flicker.
    fn save(&mut self, name: &str) {
        self.harness.event(egui::Event::PointerGone);
        self.harness.run_steps(1);
        let image = self.harness.render();
        self.harness.event(egui::Event::PointerMoved(self.pointer));
        self.harness.run_steps(1);
        match image {
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

    fn step(&mut self, step: &Step) {
        match step {
            Step::Say(text) => {
                self.send(text);
                self.settle();
            }
            Step::Send(text) => self.send(text),
            Step::Wait(d) => self.wait(*d),
            Step::Follow(on) => self.harness.state().bridge.follow(*on),
            Step::Tab(tab) => {
                self.harness.state_mut().dock_open = true;
                if self.moving {
                    self.frame();
                    if let Some(rect) = self.tab_button(*tab) {
                        self.click_rect(rect);
                    }
                }
                // Not clicked, or the click missed: the script still gets its tab.
                self.harness.state_mut().tab = *tab;
            }
            Step::Edit(on) => {
                let gui = self.harness.state_mut();
                if gui.editor.is_some() {
                    gui.editing = *on;
                    gui.tab = Tab::World;
                    gui.dock_open = true;
                } else {
                    eprintln!("no world editor in this profile");
                }
            }
            Step::Approve(on) => self.approve = *on,
            Step::Click(label) => {
                self.click_label(label);
            }
            Step::Type(text) => self.type_text(text),
            Step::Key(modifiers, key) => self.harness.key_press_modifiers(*modifiers, *key),
            Step::Pick(picked) => match picked {
                // As a click: the viewer selects it, highlights it in 3D, and its selection
                // brings up what the agent can do with it.
                Picked::Object(id) | Picked::Room(id) => {
                    let layer = if matches!(picked, Picked::Object(_)) {
                        "objects"
                    } else {
                        "rooms"
                    };
                    let path = re_log_types::EntityPath::from(format!("world/{layer}/{id}"));
                    let item = re_viewer_context::Item::InstancePath(
                        re_entity_db::InstancePath::entity_all(path),
                    );
                    self.harness
                        .state()
                        .viewer_commands
                        .send_system(re_viewer::SystemCommand::set_selection(item));
                }
                // A point on the floor has no entity to select: the click lands where it says.
                Picked::Point(_) => {
                    self.harness.state().picked.replace(Some(picked.clone()));
                }
            },
            Step::Speed(speed) => {
                self.speed = *speed;
                if let Some(capture) = &mut self.capture {
                    capture.speed(*speed);
                }
            }
            Step::Missions(speed) => self.mission_speed = Some(*speed),
            Step::Settle => self.settle(),
            Step::Limit(limit) => self.limit = *limit,
            Step::Hold(key, for_) => {
                self.harness.key_down(*key);
                self.wait(*for_);
                self.harness.key_up(*key);
            }
            Step::Focus(which) => {
                let role = egui::accesskit::Role::TextInput;
                let field = match which.parse::<usize>() {
                    Ok(n) => self.harness.query_all_by_role(role).nth(n - 1),
                    Err(_) => self.harness.query_all_by_role_and_label(role, which).next(),
                };
                match field.map(|f| f.rect()) {
                    // A click focuses a field, and shows the operator doing it.
                    Some(rect) => self.click_rect(rect),
                    None => eprintln!("no text field {which}"),
                }
            }
            Step::Shot(name) => self.save(name),
            Step::Record => {}
            Step::Scroll(points) => self.scroll_dock(*points),
        }
        self.frame();
    }

    /// Types into the focused field at [`TYPING`], as text events.
    fn type_text(&mut self, text: &str) {
        let began = Instant::now();
        for (typed, c) in text.chars().enumerate() {
            #[expect(clippy::cast_precision_loss, reason = "a count of characters")]
            let due = typed as f64 / TYPING;
            while began.elapsed().as_secs_f64() < due {
                self.frame();
            }
            self.harness.event(egui::Event::Text(c.to_string()));
        }
    }

    /// Types `text` into the composer a key at a time and sends it.
    fn send(&mut self, text: &str) {
        eprintln!("== {text}");
        self.harness
            .ctx
            .memory_mut(|m| m.request_focus(egui::Id::new(COMPOSER)));
        let chars: Vec<char> = text.chars().collect();
        let began = Instant::now();
        loop {
            #[expect(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "a count of characters, from a time that is never negative"
            )]
            let shown = ((began.elapsed().as_secs_f64() * TYPING) as usize).min(chars.len());
            self.harness.state_mut().input = chars[..shown].iter().collect();
            self.frame();
            if shown == chars.len() {
                break;
            }
        }
        self.wait(Duration::from_millis(400));
        self.click_label("Send");
    }

    /// Where the pointer waits between clicks: in the message field, where no tooltip opens.
    fn rest_point(&self) -> egui::Pos2 {
        self.harness
            .ctx
            .read_response(egui::Id::new(COMPOSER))
            .map_or(egui::pos2(220.0, 900.0), |r| {
                egui::pos2(r.rect.left() + r.rect.width() * 0.45, r.rect.center().y)
            })
    }

    fn point(&mut self, at: egui::Pos2) {
        self.pointer = at;
        self.harness.event(egui::Event::PointerMoved(at));
        self.frame();
    }

    /// Moves the pointer to `to` as a hand does: easing in and out, on a slight bow.
    fn glide(&mut self, to: egui::Pos2) {
        let (from, path) = (self.pointer, to - self.pointer);
        let length = path.length();
        if length < 1.0 {
            return;
        }
        let seconds = (length / GLIDE).clamp(GLIDE_SECONDS.0, GLIDE_SECONDS.1);
        let bow = path.rot90().normalized() * (length * 0.06).min(28.0);
        let began = Instant::now();
        loop {
            let t = (began.elapsed().as_secs_f32() / seconds).min(1.0);
            let eased = t * t * (3.0 - 2.0 * t);
            self.point(from + path * eased + bow * (std::f32::consts::PI * eased).sin());
            if t >= 1.0 {
                break;
            }
        }
    }

    /// The widget called `label`, else the first whose label holds it: a card's title
    /// "Approve geofence?" comes before its Approve button.
    fn rect_of(&self, label: &str) -> Option<egui::Rect> {
        self.harness
            .query_all_by_label(label)
            .next()
            .or_else(|| self.harness.query_all_by_label_contains(label).next())
            .map(|node| node.rect())
    }

    /// A dock tab's button: the topmost widget of its name, as the viewer's own tabs below share
    /// some names.
    fn tab_button(&self, tab: Tab) -> Option<egui::Rect> {
        let name = Tab::ALL.iter().find(|(t, _)| *t == tab)?.1;
        self.harness
            .query_all_by_label(name)
            .map(|node| node.rect())
            .min_by(|a, b| a.top().total_cmp(&b.top()))
    }

    fn click_label(&mut self, label: &str) {
        if let Some(rect) = self.rect_of(label) {
            self.click_rect(rect);
        } else {
            eprintln!("nothing to click labelled {label:?}");
        }
    }

    /// Travels to `rect`, clicks it and comes back to rest. The moment plays at normal speed in
    /// the clip, however fast the stretch around it.
    fn click_rect(&mut self, rect: egui::Rect) {
        let began = self.moment();
        self.glide(rect.center());
        self.wait(Duration::from_millis(120));
        let pos = self.pointer;
        for pressed in [true, false] {
            self.harness.event(egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed,
                modifiers: Modifiers::NONE,
            });
            self.frame();
        }
        self.wait(Duration::from_millis(350));
        self.glide(self.rest_point());
        self.end_moment(began);
    }

    /// Scrolls the dock with the wheel, a few notches over a few frames, the pointer over it.
    fn scroll_dock(&mut self, points: f32) {
        let tab = self.harness.state().tab;
        let Some(bar) = self.tab_button(tab) else {
            eprintln!("no dock to scroll");
            return;
        };
        let began = self.moment();
        self.glide(bar.center() + egui::vec2(0.0, 320.0));
        for _ in 0..6 {
            self.harness.event(egui::Event::MouseWheel {
                unit: egui::MouseWheelUnit::Point,
                delta: egui::vec2(0.0, -points / 6.0),
                phase: egui::TouchPhase::Move,
                modifiers: Modifiers::NONE,
            });
            self.wait(Duration::from_millis(60));
        }
        self.wait(Duration::from_millis(300));
        self.end_moment(began);
    }

    /// Seconds into the recording now, while one records.
    fn moment(&self) -> Option<f64> {
        self.capture
            .as_ref()
            .map(|c| c.started.elapsed().as_secs_f64())
    }

    fn end_moment(&mut self, began: Option<f64>) {
        if let (Some(began), Some(capture)) = (began, &mut self.capture) {
            let ended = capture.started.elapsed().as_secs_f64();
            capture.moments.push((began, ended));
        }
    }

    /// Steps until no turn, mission or job runs, no approval waits (unless approving is left to
    /// the script) and the chat has stayed the same for [`QUIET`].
    fn settle(&mut self) {
        let began = Instant::now();
        let (mut quiet_since, mut seen) = (None::<Instant>, usize::MAX);
        while began.elapsed() < self.limit {
            self.frame();
            if self.approve && self.harness.state().chat.pending().count() > 0 {
                // The card is read at normal speed too, before the pointer goes to it.
                let began = self.moment();
                self.wait(READING);
                let button = self
                    .harness
                    .query_all_by_label("Approve")
                    .next()
                    .map(|b| b.rect());
                if let Some(rect) = button {
                    self.click_rect(rect);
                }
                self.end_moment(began);
                continue;
            }
            let gui = self.harness.state();
            // Left to the script, a turn waiting on an approval waits on the script's next step.
            let asking = gui.chat.pending().count() > 0;
            let busy =
                (gui.chat.turn.is_some() && !asking) || gui.mission_running() || gui.job_running();
            let items = gui.chat.items.len();
            if busy || items != seen {
                (quiet_since, seen) = (None, items);
                continue;
            }
            match quiet_since {
                None => quiet_since = Some(Instant::now()),
                Some(t) if t.elapsed() >= QUIET => return,
                Some(_) => {}
            }
        }
        eprintln!("   not settled in {} s", self.limit.as_secs());
    }
}

/// The script from `NERVROS_GUI_EVAL_SCRIPT`, a file or the lines themselves; by default the
/// four prompts and each dock tab.
fn script() -> Vec<Step> {
    let text = match std::env::var("NERVROS_GUI_EVAL_SCRIPT") {
        Ok(s) if Path::new(&s).is_file() => std::fs::read_to_string(&s).expect("the script"),
        Ok(s) => s,
        Err(_) => {
            let tabs = Tab::ALL.iter().map(|(_, name)| {
                format!(":tab {name}\n:wait 1\n:shot tab-{}", name.to_lowercase())
            });
            PROMPTS
                .iter()
                .map(|p| (*p).to_owned())
                .chain(tabs)
                .collect::<Vec<_>>()
                .join("\n")
        }
    };
    text.lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .map(|l| Step::parse(l).unwrap_or_else(|e| panic!("{e}")))
        .collect()
}

#[test]
fn a_script_reads_prompts_commands_and_settings() {
    assert_eq!(
        Step::parse("Go to the bedroom."),
        Ok(Step::Say("Go to the bedroom.".into()))
    );
    assert_eq!(Step::parse(":follow on"), Ok(Step::Follow(true)));
    assert_eq!(Step::parse(":tab world"), Ok(Step::Tab(Tab::World)));
    assert_eq!(
        Step::parse(":key ctrl+k"),
        Ok(Step::Key(Modifiers::COMMAND | Modifiers::CTRL, Key::K))
    );
    assert_eq!(
        Step::parse(":pick point 1.5 -2"),
        Ok(Step::Pick(Picked::Point([1.5, -2.0])))
    );
    assert_eq!(Step::parse(":speed 4"), Ok(Step::Speed(4.0)));
    assert!(Step::parse(":speed 0").is_err());
    assert!(Step::parse(":follow maybe").is_err());
    assert!(Step::parse(":fly").is_err());
    assert!(Step::Speed(4.0).is_setting() && !Step::Wait(Duration::ZERO).is_setting());
    assert_eq!(
        Step::parse(":hold w 1.5"),
        Ok(Step::Hold(Key::W, Duration::from_millis(1500)))
    );
    assert_eq!(Step::parse(":focus 2"), Ok(Step::Focus("2".into())));
    assert_eq!(
        Step::parse(":send Find the mug."),
        Ok(Step::Send("Find the mug.".into()))
    );
    assert_eq!(Step::parse(":focus Name"), Ok(Step::Focus("Name".into())));
    assert!(Step::parse(":focus 0").is_err());
    assert_eq!(
        Step::parse(":limit 40"),
        Ok(Step::Limit(Duration::from_mins(40)))
    );
    assert_eq!(Step::parse(":record"), Ok(Step::Record));
    assert_eq!(Step::parse(":scroll 600"), Ok(Step::Scroll(600.0)));
    assert!(Step::parse(":scroll far").is_err());
}

#[test]
#[ignore = "needs the robot's stack, the model and NERVROS_PROFILE"]
fn live_eval() {
    let profile_path = PathBuf::from(std::env::var("NERVROS_PROFILE").expect("NERVROS_PROFILE"));
    let out = PathBuf::from(
        std::env::var("NERVROS_GUI_EVAL_OUT").unwrap_or_else(|_| "gui-eval".to_owned()),
    );
    std::fs::create_dir_all(&out).expect("the output folder");
    let steps = script();
    let clip = std::env::var("NERVROS_GUI_EVAL_VIDEO")
        .ok()
        .filter(|n| !n.is_empty());
    let runtime = tokio::runtime::Runtime::new().expect("tokio");
    let _entered = runtime.enter();
    let profile = nervros_core::profile::Profile::load(&profile_path).expect("the profile");
    let robot = nervros_core::app::connect(&profile).expect("the ROS node");
    // Its own robot files, so a run neither finds an earlier one's missions and zones nor leaves
    // its own; and the model the run is for, when one is named.
    let options = nervros_core::app::StartOptions {
        history: Some(out.join("session.history.json")),
        state: Some(out.join("state")),
        model: std::env::var("NERVROS_GUI_EVAL_MODEL")
            .ok()
            .filter(|m| !m.is_empty()),
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
        &agent.zones,
        agent.session.subscribe(),
    );
    agent.session.send(nervros_core::session::Command::Arm);
    let handle = runtime.handle().clone();
    let harness = Harness::builder()
        .wgpu_setup(gpu())
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
        capture: None,
        approve: true,
        speed: 1.0,
        mission_speed: None,
        in_mission: false,
        limit: PER_PROMPT,
        pointer: egui::Pos2::ZERO,
        moving: false,
    };
    let settings = steps.iter().take_while(|s| s.is_setting()).count();
    for step in &steps[..settings] {
        run.step(step);
    }
    // The map, the camera and the world model need a moment to arrive.
    run.wait(Duration::from_secs(10));
    run.moving = true;
    let steps = &steps[settings..];
    let scene = steps
        .iter()
        .position(|s| *s == Step::Record)
        .map_or(0, |at| at + 1);
    for step in &steps[..scene] {
        run.step(step);
    }
    // From the first frame the pointer is there, waiting in the message field.
    let rest = run.rest_point();
    run.point(rest);
    if let Some(name) = &clip {
        let size = run.harness.render().map_or((1600, 960), |i| i.dimensions());
        run.capture = Capture::start(&run.out.join(name), size, run.speed);
    }
    for step in &steps[scene..] {
        run.step(step);
    }
    // The end holds a moment, so the last frame is the settled window.
    run.wait(Duration::from_secs(2));
    if let Some(capture) = run.capture.take() {
        capture.finish();
    }
    drop(rec);
}
