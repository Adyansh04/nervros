//! The window: a top bar, the chat on the left, the embedded viewer in the centre, a dock on the
//! right and a status bar. The viewer draws last, into whatever space the panels leave.

use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use nervros_core::app::Agent;
use nervros_core::doctor::Check;
use nervros_core::mission::{Outcome, held_by};
use nervros_core::session::{Command, Event};
use rerun::external::egui::{self, Frame, Key, Margin, Modifiers, RichText};
use rerun::external::re_log_channel::LogReceiver;
use rerun::external::re_sdk_types::blueprint::components::PanelState;
use rerun::external::re_ui::{CommandPalette, UiExt as _};
use rerun::external::re_viewer::SystemCommandSender as _;
use rerun::external::re_viewer::external::re_log_types::{TimeReal, TimelineName};
use rerun::external::re_viewer::external::re_viewer_context::TimeControlCommand;
use rerun::external::{eframe, re_memory, re_viewer};
use serde_json::Value;
use tokio::sync::broadcast;

use crate::chat::{Action, Chat};
use crate::palette::{self, Cmd};
use crate::toasts::{Kind, Toasts};

mod agent_tab;
mod bars;
mod composer;
mod dock;
mod live;
mod world;

use live::watch;

const EVENT_LOG: usize = 300;

const COMPOSER: &str = "nervros_composer";

const STOP_HINT: &str = "Halts the mission and cancels every goal; the hands keep their grip. \
                         The e-stop on the robot's remote is the real emergency stop.";

/// What the World tab's Explore button says to the agent.
pub(crate) const EXPLORE_REQUEST: &str = "Explore the building to fill in the map.";

/// What background tasks learn about the robot.
#[derive(Debug, Default)]
struct Live {
    /// Topics in the graph, once read.
    pub topics: Option<usize>,
    /// The executor's last `RobotState`.
    pub executor: Option<Value>,
    /// The last connection check.
    pub checks: Option<Vec<Check>>,
    /// The world model's last rooms message.
    pub rooms: Option<Value>,
    /// How many objects the world model holds.
    pub objects: Option<usize>,
    /// Each object's name, or its label, by id.
    pub object_names: HashMap<String, String>,
    /// Where the robot stands on the map.
    pub pose: Option<crate::robot::Pose>,
    /// The robot's battery, when the profile names its topic.
    pub battery: Option<Value>,
    /// Its motor diagnostics, when the profile names their topic.
    pub motors: Option<Value>,
}

/// What the operator clicked in the viewer, when it is something the agent can act on.
#[derive(Debug, Clone, PartialEq)]
enum Picked {
    Object(String),
    Room(String),
    /// A point on the floor, in the map frame.
    Point([f32; 2]),
}

impl Picked {
    /// From the entity paths the viewer bridge draws: `world/objects/<id>`, `world/rooms/<id>`,
    /// and the map or coverage grid, where the clicked point counts.
    fn from_click(path: &str, position: Option<[f32; 3]>) -> Option<Self> {
        let path = path.trim_start_matches('/');
        if let Some(id) = path.strip_prefix("world/objects/") {
            return Some(Self::Object(id.to_owned()));
        }
        if let Some(id) = path.strip_prefix("world/rooms/") {
            return Some(Self::Room(id.to_owned()));
        }
        let [x, y, _] = position.filter(|_| matches!(path, "world/map" | "world/coverage"))?;
        Some(Self::Point([x, y]))
    }
}

/// Shared between the window and the tasks that fill it.
type SharedLive = Arc<Mutex<Live>>;

/// The side panel's tabs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Tab {
    Mission,
    World,
    Layers,
    Approvals,
    Events,
    Agent,
    Doctor,
    Robot,
}

impl Tab {
    const ALL: [(Self, &'static str); 8] = [
        (Self::Mission, "Mission"),
        (Self::World, "World"),
        (Self::Layers, "Layers"),
        (Self::Approvals, "Approvals"),
        (Self::Events, "Events"),
        (Self::Agent, "Agent"),
        (Self::Doctor, "Doctor"),
        (Self::Robot, "Robot"),
    ];
}

/// What the embedded viewer shows: the recording's data, the bridge drawing it, whose layers the
/// Layers tab switches, and how much of it the viewer keeps.
pub struct ViewerFeed {
    /// The recording's data.
    pub input: LogReceiver,
    /// The bridge filling the recording.
    pub bridge: nervros_viz::Bridge,
    /// The viewer drops its oldest data past this.
    pub memory_limit: re_memory::MemoryLimit,
}

/// Where closing the window stands while a mission runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Closing {
    Open,
    /// Asking the operator what to do with the running mission.
    Asking,
    /// Stop sent; the window closes once the robot says it stopped.
    Stopping(std::time::Instant),
    /// The robot did not confirm the stop: the operator decides again, having read why.
    StopFailed,
    Allowed,
}

/// The operator's answer when they close the window with a mission running.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CloseChoice {
    StopAndClose,
    LeaveRunning,
    Cancel,
}

/// What a request to close the window does: whether it is cancelled, and the state after it.
fn on_close(closing: Closing, mission_running: bool) -> (bool, Closing) {
    match closing {
        Closing::Open if mission_running => (true, Closing::Asking),
        Closing::Open | Closing::Allowed => (false, closing),
        Closing::Asking | Closing::Stopping(_) | Closing::StopFailed => (true, closing),
    }
}

/// How long closing waits for the robot to say it stopped, past the stop's own timeout.
const STOP_BEFORE_CLOSE: Duration = Duration::from_secs(8);

/// The question asked when the window is closed with a mission running, or after a stop the robot
/// did not confirm.
fn close_dialog(ctx: &egui::Context, failed: bool) -> Option<CloseChoice> {
    let mut choice = None;
    let modal = egui::Modal::new(egui::Id::new("nervros_close")).show(ctx, |ui| {
        ui.set_max_width(380.0);
        if failed {
            ui.label(
                RichText::new("The robot did not confirm the stop")
                    .strong()
                    .color(ui.tokens().error_fg_color),
            );
            ui.add_space(6.0);
            ui.label(
                "The mission may still be running. Try the stop again, or close and leave it \
                 running with nobody watching; the chat says what the robot answered.",
            );
        } else {
            ui.label(RichText::new("A mission is running").strong());
            ui.add_space(6.0);
            ui.label(
                "Closing leaves it running with nobody watching. Stop the robot first, or leave \
                 it to finish.",
            );
        }
        ui.add_space(10.0);
        ui.horizontal(|ui| {
            if ui.button("Stop it and close").clicked() {
                choice = Some(CloseChoice::StopAndClose);
            }
            if ui.button("Leave it running").clicked() {
                choice = Some(CloseChoice::LeaveRunning);
            }
            if ui.button("Cancel").clicked() {
                choice = Some(CloseChoice::Cancel);
            }
        });
    });
    if modal.should_close() && choice.is_none() {
        choice = Some(CloseChoice::Cancel);
    }
    choice
}

/// The app.
pub struct Gui {
    viewer: re_viewer::App,
    agent: Agent,
    events: broadcast::Receiver<Event>,
    live: SharedLive,
    log_path: PathBuf,
    /// Earlier sessions for the Agent tab, and when they were listed.
    sessions: Option<(Instant, Vec<crate::sessions::SessionInfo>)>,
    /// What the listing has read of them.
    listing: crate::sessions::Listing,
    chat: Chat,
    event_log: VecDeque<String>,
    input: String,
    history: Vec<String>,
    history_pos: Option<usize>,
    tab: Tab,
    dock_open: bool,
    recheck: tokio::sync::mpsc::UnboundedSender<()>,
    /// Set by the viewer on the UI thread when the selection changes.
    picked: Rc<RefCell<Option<Picked>>>,
    closing: Closing,
    bridge: nervros_viz::Bridge,
    /// The world editor, when the profile names one; `editing` shows it in place of the viewer.
    editor: Option<crate::editor::WorldEditor>,
    editing: bool,
    /// Ctrl+K, and the commands it lists.
    palette: CommandPalette,
    commands: Vec<palette::Entry>,
    toasts: Toasts,
    /// Moves the viewer's time cursor.
    viewer_commands: re_viewer::CommandSender,
    /// The earlier moment the 3D view is paused at, if it is not following the newest data.
    viewing: Option<SystemTime>,
    /// Driving by hand, when the profile names the executor's teleop.
    drive: Option<crate::robot::Drive>,
    /// The mission ledger's lists, for the Mission tab.
    records: crate::history::History,
    /// Images waiting to go with the next message.
    attachments: crate::attach::Attachments,
}

impl Gui {
    /// Sets up the viewer and the tasks that keep the window current, around a running agent.
    ///
    /// # Errors
    ///
    /// The renderer could not be set up.
    pub fn start(
        main_thread: re_viewer::MainThreadToken,
        cc: &eframe::CreationContext<'_>,
        agent: Agent,
        events: tokio::sync::broadcast::Receiver<Event>,
        feed: ViewerFeed,
        runtime: tokio::runtime::Handle,
        log_path: PathBuf,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        re_viewer::customize_eframe_and_setup_renderer(cc)?;
        let mut startup = re_viewer::StartupOptions {
            hide_welcome_screen: true,
            expect_data_soon: Some(true),
            // Tests must neither load nor overwrite the layout a user saved.
            persist_state: !cfg!(test),
            ..re_viewer::StartupOptions::default()
        };
        // A click on an object or a room offers what the agent can do with it.
        let picked = Rc::new(RefCell::new(None));
        let sink = Rc::clone(&picked);
        startup.on_event = Some(Rc::new(move |event: re_viewer::ViewerEvent| {
            if let re_viewer::ViewerEventKind::SelectionChange { items } = &event.kind {
                *sink.borrow_mut() = items.iter().find_map(|item| match item {
                    re_viewer::SelectionChangeItem::Entity {
                        entity_path,
                        position,
                        ..
                    } => Picked::from_click(&entity_path.to_string(), position.map(Into::into)),
                    _ => None,
                });
            }
        }));
        // Our own top bar replaces the viewer's, and its side panels stay shut: the app is the
        // interface, and Reset layout brings back any view closed by mistake.
        let panels = &mut startup.panel_state_overrides;
        panels.top = Some(PanelState::Hidden);
        panels.blueprint = Some(PanelState::Collapsed);
        panels.selection = Some(PanelState::Collapsed);
        panels.time = Some(PanelState::Collapsed);
        let editor = agent
            .editor
            .clone()
            .map(|client| crate::editor::WorldEditor::new(client, runtime.clone()));
        let drive =
            crate::robot::Drive::new(&agent.profile, Arc::clone(&agent.robot), runtime.clone());
        let (viewer_commands, receiver) = re_viewer::command_channel();
        let mut viewer = re_viewer::App::with_commands(
            main_thread,
            re_viewer::build_info(),
            if cfg!(test) {
                re_viewer::AppEnvironment::Test
            } else {
                re_viewer::AppEnvironment::Custom("NervROS".to_owned())
            },
            startup,
            cc,
            None,
            re_viewer::AsyncRuntimeHandle::new_native(runtime),
            re_viewer::register_text_log_receiver(),
            (viewer_commands.clone(), receiver),
        );
        viewer.app_options_mut().memory_limit = feed.memory_limit;
        viewer.add_log_receiver(feed.input);
        let live = Arc::new(Mutex::new(Live::default()));
        let recheck = watch(&agent, &live, &cc.egui_ctx);
        let mut chat = Chat::default();
        chat.items
            .extend(agent.notices.iter().cloned().map(crate::chat::Item::Notice));
        chat.items.extend(
            agent
                .unanswered
                .iter()
                .cloned()
                .map(|u| crate::chat::Item::Unanswered(u, std::cell::Cell::new(false))),
        );
        let agent_snapshots = Arc::clone(&agent.snapshots);
        Ok(Self {
            viewer,
            agent,
            events,
            live,
            log_path,
            sessions: None,
            listing: crate::sessions::Listing::default(),
            chat,
            event_log: VecDeque::new(),
            input: String::new(),
            history: Vec::new(),
            history_pos: None,
            tab: Tab::Approvals,
            dock_open: true,
            recheck,
            picked,
            closing: Closing::Open,
            bridge: feed.bridge,
            editor,
            editing: false,
            palette: CommandPalette::default(),
            commands: palette::entries(),
            toasts: Toasts::default(),
            viewer_commands,
            viewing: None,
            drive,
            records: crate::history::History::default(),
            attachments: crate::attach::Attachments::new(Arc::clone(&agent_snapshots)),
        })
    }

    /// Whether the executor says a mission runs.
    fn mission_running(&self) -> bool {
        self.live()
            .executor
            .as_ref()
            .and_then(|s| s["mission_id"].as_str())
            .is_some_and(|id| !id.is_empty())
    }

    /// Closing the window with a mission running asks first: the mission would go on unwatched.
    fn guard_close(&mut self, ctx: &egui::Context) {
        if ctx.input(|i| i.viewport().close_requested()) {
            let cancel;
            (cancel, self.closing) = on_close(self.closing, self.mission_running());
            if cancel {
                ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            }
        }
        match self.closing {
            Closing::Asking | Closing::StopFailed => {
                match close_dialog(ctx, self.closing == Closing::StopFailed) {
                    Some(CloseChoice::StopAndClose) => {
                        self.agent.session.send(Command::StopMission);
                        self.closing = Closing::Stopping(std::time::Instant::now());
                    }
                    Some(CloseChoice::LeaveRunning) => {
                        self.closing = Closing::Allowed;
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                    Some(CloseChoice::Cancel) => self.closing = Closing::Open,
                    None => {}
                }
            }
            // No answer at all is no confirmation either.
            Closing::Stopping(since) if since.elapsed() >= STOP_BEFORE_CLOSE => {
                self.closing = Closing::StopFailed;
            }
            Closing::Stopping(_) => ctx.request_repaint_after(Duration::from_millis(200)),
            Closing::Allowed => ctx.send_viewport_cmd(egui::ViewportCommand::Close),
            Closing::Open => {}
        }
    }

    fn live(&self) -> std::sync::MutexGuard<'_, Live> {
        self.live.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn drain_events(&mut self) {
        loop {
            match self.events.try_recv() {
                Ok(e) => {
                    self.chat.apply(&e);
                    self.toast(&e);
                    // Closed only once the robot confirms the stop; a failed one asks again.
                    if let (Closing::Stopping(_), Event::Stopped { ok, .. }) = (self.closing, &e) {
                        self.closing = if *ok {
                            Closing::Allowed
                        } else {
                            Closing::StopFailed
                        };
                    }
                    if self.event_log.len() == EVENT_LOG {
                        self.event_log.pop_back();
                    }
                    let line = serde_json::to_string(&e).unwrap_or_default();
                    self.event_log.push_front(line);
                    if matches!(e, Event::ApprovalRequested { .. }) {
                        self.tab = Tab::Approvals;
                    }
                }
                Err(broadcast::error::TryRecvError::Lagged(_)) => {}
                Err(_) => break,
            }
        }
    }

    /// A finished mission gets a toast: the operator may be looking at the viewer or away.
    fn toast(&mut self, e: &Event) {
        let Event::MissionFinished {
            id,
            outcome,
            failed_step,
            reason,
            elapsed_s,
        } = e
        else {
            return;
        };
        self.records.stale();
        let what = self.chat.intent_of_mission(id).unwrap_or("the mission");
        let (kind, text) = match outcome {
            Outcome::Success => (Kind::Success, format!("Done: {what} in {elapsed_s:.0} s")),
            Outcome::Canceled => (Kind::Info, format!("Stopped: {what}")),
            _ if failed_step.is_empty() => (Kind::Failure, format!("{outcome}: {what}: {reason}")),
            _ => (
                Kind::Failure,
                format!("{outcome}: {what} at {failed_step}: {reason}"),
            ),
        };
        self.toasts.add(kind, text);
    }

    /// The bubble appears when the session starts the turn, so a message it turns away shows
    /// only as its notice. A plain order to stop stops the robot without a model, and a `/name`
    /// runs that command instead.
    fn say(&mut self, text: &str) {
        self.history.push(text.to_owned());
        self.history_pos = None;
        if nervros_core::session::is_stop_word(text) {
            self.agent.session.send(Command::StopMission);
            return;
        }
        if text.starts_with('/') {
            if let Some(cmd) = palette::slash(&self.commands, text).cloned() {
                self.run(cmd);
                return;
            }
            // A lone unknown `/word` is a mistyped command; a sentence such as "/scan has no
            // data" or a ROS name with parts is for the agent.
            if !text.trim().contains(char::is_whitespace) && text.matches('/').count() == 1 {
                self.chat.items.push(crate::chat::Item::Notice(format!(
                    "No command {text}; Ctrl+K lists them"
                )));
                return;
            }
        }
        let (text, shown) = self.attachments.send(text);
        let handle = self.agent.session.handle();
        for image in shown {
            handle.emit(image);
        }
        self.agent.session.send(Command::User(text));
    }

    /// Runs a palette or slash command.
    fn run(&mut self, cmd: Cmd) {
        match cmd {
            Cmd::Say(text) => self.say(text),
            Cmd::Send(c) => self.agent.session.send(c),
            Cmd::Tab(tab) => {
                self.tab = tab;
                self.dock_open = true;
            }
            Cmd::Doctor => {
                self.tab = Tab::Doctor;
                self.dock_open = true;
                self.live().checks = None;
                let _ = self.recheck.send(());
            }
            Cmd::Dock => self.dock_open = !self.dock_open,
            Cmd::ResetLayout => self.bridge.reset_layout(),
            Cmd::EditWorld => {
                if self.editor.is_some() {
                    self.editing = !self.editing;
                    self.tab = Tab::World;
                    self.dock_open = true;
                }
            }
            Cmd::Live => self.follow_live(),
        }
    }

    /// Pauses the 3D view at `at`, where the robot was and what the world model held then.
    fn show_at(&mut self, at: SystemTime) {
        let ns = at
            .duration_since(UNIX_EPOCH)
            .map_or(0, |d| i64::try_from(d.as_nanos()).unwrap_or(i64::MAX));
        self.time_commands(vec![
            TimeControlCommand::SetActiveTimeline(TimelineName::log_time()),
            TimeControlCommand::Pause,
            TimeControlCommand::SetTime(TimeReal::from(ns)),
        ]);
        self.viewing = Some(at);
    }

    fn follow_live(&mut self) {
        self.time_commands(vec![TimeControlCommand::MoveEndAndFollow]);
        self.viewing = None;
    }

    fn time_commands(&self, time_commands: Vec<TimeControlCommand>) {
        if let Some(store_id) = self.viewer.active_recording_id().cloned() {
            self.viewer_commands
                .send_system(re_viewer::SystemCommand::TimeControlCommands {
                    store_id,
                    time_commands,
                });
        }
    }

    fn act(&mut self, ctx: &egui::Context, actions: Vec<Action>) {
        for a in actions {
            match a {
                Action::Send(c) => {
                    if let Command::Edit { id, .. } = &c {
                        self.chat.edit_sent(*id);
                    }
                    self.agent.session.send(c);
                }
                Action::Say(text) => self.say(&text),
                Action::Prefill(text) => {
                    self.input = text;
                    ctx.memory_mut(|m| m.request_focus(egui::Id::new(COMPOSER)));
                }
                Action::ShowAt(at) => self.show_at(at),
            }
        }
    }

    fn keys(&mut self, ctx: &egui::Context) {
        let working = self.chat.turn.is_some();
        if ctx.input_mut(|i| i.consume_key(Modifiers::COMMAND, Key::K)) {
            self.palette.toggle();
        }
        // Esc in another text field, such as a plan's argument on its card, leaves the field: as
        // a stop it would also turn down the plan being edited.
        let elsewhere = ctx
            .memory(egui::Memory::focused)
            .is_some_and(|f| f != egui::Id::new(COMPOSER));
        let (esc, stop, tab) = ctx.input_mut(|i| {
            let esc = working && !elsewhere && i.consume_key(Modifiers::NONE, Key::Escape);
            let stop = i.consume_key(Modifiers::CTRL | Modifiers::SHIFT, Key::S);
            let keys = [
                Key::Num1,
                Key::Num2,
                Key::Num3,
                Key::Num4,
                Key::Num5,
                Key::Num6,
                Key::Num7,
                Key::Num8,
            ];
            let tab = keys.iter().position(|k| i.consume_key(Modifiers::CTRL, *k));
            (esc, stop, tab)
        });
        if esc {
            self.agent.session.send(Command::StopGeneration);
        }
        if stop {
            self.agent.session.send(Command::StopMission);
        }
        if let Some(n) = tab {
            self.tab = Tab::ALL[n].0;
            self.dock_open = true;
        }
    }

    fn stop_label(&self) -> String {
        let live = self.live();
        let held = live.executor.as_ref().and_then(|s| {
            ["left", "right"].into_iter().find_map(|hand| {
                let obj = held_by(s, hand);
                (!obj.is_empty()).then(|| format!("Stop · {hand} hand keeps {obj}"))
            })
        });
        held.unwrap_or_else(|| "Stop mission".to_owned())
    }
}

impl eframe::App for Gui {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        self.drain_events();
        if self.agent.tools.iter().any(|t| t == "look") {
            self.attachments.take_dropped(ui.ctx());
        }
        self.keys(ui.ctx());
        self.guard_close(ui.ctx());
        let t = ui.tokens();
        egui::Panel::top("nervros_top")
            .frame(
                Frame::new()
                    .fill(t.top_bar_color)
                    .inner_margin(Margin::symmetric(12, 8)),
            )
            .show(ui, |ui| self.top_bar(ui));
        egui::Panel::bottom("nervros_status")
            .frame(
                Frame::new()
                    .fill(t.bottom_bar_color)
                    .inner_margin(Margin::symmetric(12, 4)),
            )
            .show(ui, |ui| self.status_bar(ui, frame));
        egui::Panel::left("nervros_chat")
            .default_size(440.0)
            .size_range(320.0..=720.0)
            .frame(
                Frame::new()
                    .fill(t.panel_bg_color)
                    .inner_margin(Margin::symmetric(16, 8)),
            )
            .show(ui, |ui| self.chat_panel(ui));
        if self.dock_open {
            egui::Panel::right("nervros_dock")
                .default_size(340.0)
                .size_range(280.0..=640.0)
                .frame(
                    Frame::new()
                        .fill(t.panel_bg_color)
                        .inner_margin(Margin::symmetric(12, 4)),
                )
                .show(ui, |ui| self.dock(ui));
        }
        match (&mut self.editor, self.editing) {
            (Some(editor), true) => {
                egui::CentralPanel::no_frame().show(ui, |ui| editor.canvas(ui));
            }
            _ => self.viewer.ui(ui, frame),
        }
        let (editor, viewing) = (self.editor.is_some(), self.viewing.is_some());
        let available = move |cmd: &Cmd| match cmd {
            Cmd::EditWorld => editor,
            Cmd::Live => viewing,
            _ => true,
        };
        let mut provider = palette::Provider {
            entries: &self.commands,
            available: &available,
        };
        if let Some(cmd) = self.palette.show(ui.ctx(), &mut provider) {
            self.run(cmd);
        }
        self.toasts.show(ui.ctx());
    }

    fn logic(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        self.viewer.logic(ctx, frame);
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        self.viewer.save(storage);
    }
}

#[cfg(test)]
#[path = "live_eval.rs"]
mod live_eval;

#[cfg(test)]
mod tests {
    use super::composer::{walk_to_point, walk_to_target};
    use super::world::{layers_view, world_view};
    use super::*;
    use egui_kittest::kittest::Queryable as _;
    use egui_kittest::{Harness, SnapshotOptions};
    use nervros_ros::fake::FakeRobot;

    const PROFILE: &str = r#"
        [robot]
        name = "Unitree G1 humanoid (simulated)"
        [look]
        image = "/camera"
        detections = { topic = "/detections", type = "canopy_msgs/msg/InstanceMaskArray" }
        [world]
        rooms = { topic = "/rooms", type = "canopy_msgs/msg/RoomArray" }
        objects = { topic = "/objects", type = "canopy_msgs/msg/WorldObjectArray" }
        [models]
        file = "models.toml"
        # Long, so the approval's countdown reads the same however slowly the test runs.
        [policy]
        approval_ttl = 600
    "#;

    fn robot() -> FakeRobot {
        let (w, h) = (160u32, 90u32);
        let data: Vec<u8> = (0..w * h)
            .flat_map(|i| {
                let [x, y] = [i % w, i / w].map(|v| u8::try_from(v).unwrap_or(u8::MAX));
                [x, y, 160]
            })
            .collect();
        let frame = nervros_ros::image::Frame {
            stamp_s: 1.0,
            frame_id: "camera".to_owned(),
            width: w,
            height: h,
            encoding: "rgb8".to_owned(),
            step: w * 3,
            is_bigendian: false,
            data: data.into(),
        };
        let square = |x0: f64, y0: f64, s: f64| {
            serde_json::json!({"points": [
                {"x": x0, "y": y0, "z": 0.0}, {"x": x0 + s, "y": y0, "z": 0.0},
                {"x": x0 + s, "y": y0 + s, "z": 0.0}, {"x": x0, "y": y0 + s, "z": 0.0}]})
        };
        let object = |id: &str, name: &str, x: f64, y: f64| {
            serde_json::json!({"id": id, "label": name, "name": name, "state": 0,
                "pose": {"position": {"x": x, "y": y, "z": 0.4},
                         "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
                "size": {"x": 0.4, "y": 0.3, "z": 0.8}})
        };
        FakeRobot::new()
            .with_frame("/camera", frame)
            .with_transform("map", "base_footprint", nervros_ros::Transform::IDENTITY)
            .with_topic(
                "/rooms",
                serde_json::json!({"rooms": [
                    {"id": "R1", "name": "office", "outline": square(-3.0, -3.0, 6.0)},
                    {"id": "R2", "name": "kitchen", "outline": square(3.0, -3.0, 5.0)}]}),
            )
            .with_topic(
                "/objects",
                serde_json::json!({"objects": [object("O1", "shelf", 1.5, 1.0),
                                               object("O2", "cardboard box", 5.0, 0.5)]}),
            )
    }

    // Nothing listens on port 9; the window never calls the model here.
    const MODELS: &str = r#"
        [[provider]]
        id = "local"
        kind = "openai_compat"
        base_url = "http://127.0.0.1:9/v1"
        [[model]]
        id = "qwen3.5-9b-local"
        provider = "local"
        model = "qwen3.5-9b"
        vision = true
        tools = true
        privacy = { local = true }
        [roles]
        routine = ["qwen3.5-9b-local"]
    "#;

    #[test]
    fn a_click_names_the_object_room_or_floor_point_it_hit() {
        let at = Some([1.5, -2.0, 0.0]);
        assert_eq!(
            Picked::from_click("/world/objects/O17", at),
            Some(Picked::Object("O17".into()))
        );
        assert_eq!(
            Picked::from_click("world/rooms/R3", None),
            Some(Picked::Room("R3".into()))
        );
        assert_eq!(
            Picked::from_click("/world/map", at),
            Some(Picked::Point([1.5, -2.0]))
        );
        assert_eq!(Picked::from_click("/world/map", None), None);
        assert_eq!(Picked::from_click("/camera", at), None);
    }

    #[test]
    fn a_click_on_the_floor_walks_there_facing_the_way_it_walked() {
        let plan = walk_to_point(2.0, 1.0, Some((0.0, 1.0, 1.0)));
        assert_eq!(plan["steps"][0]["skill"], "GoToPose");
        assert_eq!(plan["steps"][0]["args"][0]["value"], "2.00;1.00;0.000");
        let here = walk_to_point(0.0, 1.0, Some((0.0, 1.0, 1.0)));
        assert_eq!(here["steps"][0]["args"][0]["value"], "0.00;1.00;1.000");
        assert_eq!(
            walk_to_target("O17", "O17 (red mug)")["steps"][0]["args"][0]["value"],
            "O17"
        );
    }

    #[test]
    fn snapshot_world_tab() {
        let rooms = serde_json::json!({"rooms": [
            {"id": "R1", "name": "kitchen", "type": "kitchen", "floor_coverage": 0.96, "face_coverage": 0.88, "object_count": 9},
            {"id": "R2", "name": "", "type": "living room", "floor_coverage": 0.61, "face_coverage": 0.34, "object_count": 4},
            {"id": "R3", "name": "bedroom", "type": "bedroom", "floor_coverage": 0.05, "face_coverage": 0.0, "object_count": 0}
        ]});
        let mut harness = Harness::builder()
            .wgpu()
            .with_size(egui::vec2(420.0, 300.0))
            .build_ui(move |ui| {
                Frame::new()
                    .fill(ui.tokens().panel_bg_color)
                    .inner_margin(Margin::same(12))
                    .show(ui, |ui| {
                        let _ = world_view(ui, &rooms, Some(13));
                    });
            });
        crate::chat::style_for_tests(&harness.ctx);
        harness.run();
        harness.fit_contents();
        crate::chat::compare(&mut harness, "world", &SnapshotOptions::new());
    }

    #[test]
    fn snapshot_layers_tab() {
        let layers = nervros_viz::Layers::default();
        for (name, shown) in [
            ("Robot model", true),
            ("Detections", true),
            ("Map", true),
            ("Camera coverage", false),
            ("Rooms", true),
            ("Scan", true),
        ] {
            layers.add(name, shown);
        }
        let mut harness = Harness::builder()
            .wgpu()
            .with_size(egui::vec2(340.0, 300.0))
            .build_ui(move |ui| {
                Frame::new()
                    .fill(ui.tokens().panel_bg_color)
                    .inner_margin(Margin::same(12))
                    .show(ui, |ui| layers_view(ui, &layers));
            });
        crate::chat::style_for_tests(&harness.ctx);
        harness.run();
        harness.fit_contents();
        crate::chat::compare(&mut harness, "layers", &SnapshotOptions::new());
    }

    #[test]
    fn closing_with_a_mission_running_asks_first() {
        assert_eq!(on_close(Closing::Open, false), (false, Closing::Open));
        assert_eq!(on_close(Closing::Open, true), (true, Closing::Asking));
        assert_eq!(on_close(Closing::Asking, true), (true, Closing::Asking));
        let stopping = Closing::Stopping(std::time::Instant::now());
        assert_eq!(on_close(stopping, true), (true, stopping));
        assert_eq!(on_close(Closing::Allowed, true), (false, Closing::Allowed));
    }

    #[test]
    fn snapshot_window() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("nervros.toml"), PROFILE).unwrap();
        std::fs::write(dir.path().join("models.toml"), MODELS).unwrap();
        let runtime = tokio::runtime::Runtime::new().unwrap();
        let _entered = runtime.enter();
        let robot: Arc<dyn nervros_ros::RobotPort> = Arc::new(robot());
        let profile = dir.path().join("nervros.toml");
        let agent =
            nervros_core::app::start(&profile, robot, &dir.path().join("quota.json")).unwrap();
        agent.memory.remember("The kitchen door sticks.").unwrap();
        let (rec, input) = nervros_viz::in_process().unwrap();
        let bridge = nervros_viz::spawn(
            &rec,
            &agent.robot,
            &agent.profile,
            agent.session.subscribe(),
        );
        let handle = runtime.handle().clone();
        let mut harness = Harness::builder()
            .wgpu()
            .with_size(egui::vec2(1280.0, 800.0))
            .build_eframe(|cc| {
                let feed = ViewerFeed {
                    input,
                    bridge,
                    memory_limit: re_memory::MemoryLimit::from_bytes(1 << 30),
                };
                let log = PathBuf::from("session.ndjson");
                let token = re_viewer::MainThreadToken::i_promise_i_am_only_using_this_for_a_test();
                let events = agent.session.subscribe();
                let mut gui = Gui::start(token, cc, agent, events, feed, handle, log).unwrap();
                gui.chat = crate::chat::tests::sample();
                gui
            });
        harness.run_steps(4);
        // The bridge's first samples reach the viewer on its own tasks.
        std::thread::sleep(Duration::from_millis(1500));
        harness.run_steps(8);
        // The status bar's memory and frame-time figures differ on every run.
        let options = SnapshotOptions::new().max_failed_pixels(1500);
        crate::chat::compare(&mut harness, "window", &options);
        // Reset layout lays the panes out the same way again.
        harness.get_by_label("Reset layout").click();
        harness.run_steps(2);
        harness.input_mut().events.push(egui::Event::PointerGone);
        harness.run_steps(8);
        crate::chat::compare(&mut harness, "window", &options);
        // The Agent tab: sessions to resume, what it remembers, the models.
        harness.state_mut().tab = Tab::Agent;
        harness.run_steps(4);
        crate::chat::compare(&mut harness, "window_agent", &options);
    }
}
