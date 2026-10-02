//! The window: a top bar, the chat on the left, the embedded viewer in the centre, a dock on the
//! right and a status bar. The viewer draws last, into whatever space the panels leave.

use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use nervros_core::app::Agent;
use nervros_core::doctor::Check;
use nervros_core::mission::held_by;
use nervros_core::providers::router::{Need, Router};
use nervros_core::providers::{ModelConfig, Role};
use nervros_core::session::{Command, Event};
use rerun::external::egui::{
    self, Align, Color32, CornerRadius, Frame, Key, Layout, Margin, Modifiers, RichText,
};
use rerun::external::re_log_channel::LogReceiver;
use rerun::external::re_sdk_types::blueprint::components::PanelState;
use rerun::external::re_ui::{CommandPalette, ReButton, UiExt as _, icons};
use rerun::external::re_viewer::SystemCommandSender as _;
use rerun::external::re_viewer::external::re_log_types::{TimeReal, TimelineName};
use rerun::external::re_viewer::external::re_viewer_context::TimeControlCommand;
use rerun::external::{eframe, re_memory, re_viewer};
use serde_json::Value;
use tokio::sync::broadcast;

use crate::chat::{Action, Chat};
use crate::palette::{self, Cmd};
use crate::toasts::{Kind, Toasts};

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
        let (kind, text) = match outcome.as_str() {
            "success" => (Kind::Success, format!("Done: {what} in {elapsed_s:.0} s")),
            "canceled" => (Kind::Info, format!("Stopped: {what}")),
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

    fn top_bar(&mut self, ui: &mut egui::Ui) {
        let t = ui.tokens();
        // With the viewer drawing the window's decorations, its hidden top bar took the window
        // buttons and the drag handle with it; ours has them instead.
        let window_chrome = self.viewer.app_options().custom_window_decorations;
        if window_chrome {
            let bar = ui.interact(ui.max_rect(), ui.id().with("drag"), egui::Sense::click());
            if bar.double_clicked() {
                let maximized = ui.input(|i| i.viewport().maximized.unwrap_or(false));
                ui.send_viewport_cmd(egui::ViewportCommand::Maximized(!maximized));
            } else if bar.is_pointer_button_down_on() {
                ui.send_viewport_cmd(egui::ViewportCommand::StartDrag);
            }
        }
        ui.horizontal_centered(|ui| {
            ui.spacing_mut().item_spacing.x = 8.0;
            ui.label(RichText::new("NervROS").strong().size(15.0));
            ui.label(RichText::new(&self.agent.profile.robot.name).color(t.text_subdued));
            ui.add_space(8.0);
            self.status_chips(ui);
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if window_chrome {
                    ui.native_window_buttons_ui();
                    ui.add_space(4.0);
                }
                let stop = egui::Button::new(
                    RichText::new(self.stop_label())
                        .strong()
                        .color(Color32::WHITE),
                )
                .fill(t.error_fg_color)
                .corner_radius(CornerRadius::same(8))
                .min_size(egui::vec2(0.0, 28.0));
                if ui.add(stop).on_hover_text(STOP_HINT).clicked() {
                    self.agent.session.send(Command::StopMission);
                }
                let toggle = ReButton::new("Dock")
                    .small()
                    .secondary()
                    .selected(self.dock_open);
                if ui.add(toggle).clicked() {
                    self.dock_open = !self.dock_open;
                }
                if self.editor.is_some() {
                    let edit = ReButton::new("Edit world")
                        .small()
                        .secondary()
                        .selected(self.editing);
                    if ui
                        .add(edit)
                        .on_hover_text("The saved world on its floor plan, to fix by hand")
                        .clicked()
                    {
                        self.editing = !self.editing;
                        if self.editing {
                            self.tab = Tab::World;
                            self.dock_open = true;
                        }
                    }
                }
                let reset = ReButton::new("Reset layout").small().secondary();
                if ui
                    .add(reset)
                    .on_hover_text("Put the viewer's panes back the way NervROS lays them out")
                    .clicked()
                {
                    self.bridge.reset_layout();
                }
                let mut armed = self.agent.guard.armed();
                let label = if armed { "Armed" } else { "Observe only" };
                ui.label(RichText::new(label).color(if armed {
                    t.warn_fg_color
                } else {
                    t.text_subdued
                }));
                if ui.toggle_switch(14.0, &mut armed).changed() {
                    self.agent
                        .session
                        .send(if armed { Command::Arm } else { Command::Disarm });
                }
            });
        });
    }

    /// How the robot, the executor and the model are, and a way back to the live 3D view.
    fn status_chips(&mut self, ui: &mut egui::Ui) {
        let t = ui.tokens();
        let (topics, executor) = {
            let live = self.live();
            (live.topics, live.executor.is_some())
        };
        match topics {
            Some(n) if n > 2 => chip(ui, t.success_text_color, format!("ROS · {n} topics")),
            Some(_) => chip(ui, t.error_fg_color, "ROS · empty graph"),
            None => chip(ui, t.warn_fg_color, "ROS · connecting"),
        }
        if self.agent.profile.mission.is_some() {
            if executor {
                chip(ui, t.success_text_color, "executor");
            } else {
                chip(ui, t.warn_fg_color, "executor offline");
            }
        }
        let (model, quota) = self.model_status();
        chip(ui, t.info_text_color, format!("{model} · {quota}"));
        if let Some(at) = self.viewing {
            let secs = at.duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs());
            let back = ReButton::new(format!(
                "Viewing {} · back to live",
                crate::sessions::ago(secs)
            ))
            .small()
            .secondary();
            if ui.add(back).clicked() {
                self.follow_live();
            }
        }
    }

    /// The model answering now, and its quota for today.
    fn model_status(&self) -> (String, String) {
        let router = self.agent.llm.router();
        let now = SystemTime::now();
        let need = Need {
            tools: true,
            ..Need::default()
        };
        let id = self.chat.model.clone().or_else(|| {
            let (take, _) = router.candidates(Role::Routine, need, now);
            take.first().map(|m| m.id.clone())
        });
        let Some(id) = id else {
            return ("no model".to_owned(), "none usable".to_owned());
        };
        let quota = router
            .config()
            .model(&id)
            .map(|m| quota(router, m, now))
            .unwrap_or_default();
        (id, quota)
    }

    fn chat_panel(&mut self, ui: &mut egui::Ui) {
        let mut actions = Vec::new();
        egui::Panel::bottom("nervros_composer_panel")
            .frame(Frame::new().inner_margin(Margin::symmetric(0, 8)))
            .show(ui, |ui| {
                self.picked_bar(ui, &mut actions);
                self.composer(ui, &mut actions);
            });
        egui::ScrollArea::vertical()
            .stick_to_bottom(true)
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.add_space(8.0);
                self.chat
                    .show(ui, self.agent.profile.policy.approval_ttl, &mut actions);
            });
        self.act(ui.ctx(), actions);
    }

    /// What was clicked in the viewer: a walk there to approve at once, and messages about it
    /// filled in for the operator to read and send.
    fn picked_bar(&mut self, ui: &mut egui::Ui, actions: &mut Vec<Action>) {
        let Some(picked) = self.picked.borrow().clone() else {
            return;
        };
        let (what, go, prompts) = {
            let live = self.live();
            match &picked {
                Picked::Object(id) => {
                    let what = live
                        .object_names
                        .get(id)
                        .map_or_else(|| id.clone(), |n| format!("{id} ({n})"));
                    let prompts = vec![
                        ("What is it?", format!("Tell me about {what}.")),
                        ("Pick it up", format!("Pick up {what}.")),
                    ];
                    (what.clone(), walk_to_target(id, &what), prompts)
                }
                Picked::Room(id) => {
                    let name = live.rooms.as_ref().and_then(|m| {
                        m["rooms"]
                            .as_array()?
                            .iter()
                            .find(|r| r["id"] == id.as_str())
                            .and_then(|r| {
                                r["type"]
                                    .as_str()
                                    .filter(|t| !t.is_empty())
                                    .or_else(|| r["name"].as_str())
                                    .filter(|n| !n.is_empty())
                                    .map(str::to_owned)
                            })
                    });
                    let what = name.map_or_else(|| id.clone(), |n| format!("{id} ({n})"));
                    let prompts = vec![("What is in it?", format!("What objects are in {what}?"))];
                    (what.clone(), walk_to_target(id, &what), prompts)
                }
                Picked::Point([x, y]) => {
                    let what = format!("x {x:.2}, y {y:.2} on the map");
                    (what, walk_to_point(*x, *y, live.pose), Vec::new())
                }
            }
        };
        ui.horizontal(|ui| {
            ui.label(
                RichText::new(format!("Selected {what}"))
                    .small()
                    .color(ui.tokens().text_subdued),
            );
            if ui
                .small_button("Go there")
                .on_hover_text("Plan the walk now; you approve it before the robot moves")
                .clicked()
            {
                actions.push(Action::Send(Command::Run {
                    tool: "run_mission".to_owned(),
                    args: go,
                }));
            }
            for (label, text) in prompts {
                if ui.small_button(label).clicked() {
                    actions.push(Action::Prefill(text));
                }
            }
            if ui
                .small_button("✕")
                .on_hover_text("Clear the selection")
                .clicked()
            {
                self.picked.replace(None);
            }
        });
        ui.add_space(4.0);
    }

    fn composer(&mut self, ui: &mut egui::Ui, actions: &mut Vec<Action>) {
        let id = egui::Id::new(COMPOSER);
        let focused = ui.memory(|m| m.has_focus(id));
        let browsing = self.input.is_empty() || self.history_pos.is_some();
        let (enter, up, down) = ui.input_mut(|i| {
            if !focused {
                return (false, false, false);
            }
            // consume_key matches an extra Shift too: Shift+Enter must reach the field as a
            // new line, and Shift+arrows select.
            let plain = i.modifiers.is_none();
            let enter = plain && i.consume_key(Modifiers::NONE, Key::Enter);
            let up = plain && browsing && i.consume_key(Modifiers::NONE, Key::ArrowUp);
            let down = plain && browsing && i.consume_key(Modifiers::NONE, Key::ArrowDown);
            (enter, up, down)
        });
        if up {
            let pos = self
                .history_pos
                .unwrap_or(self.history.len())
                .saturating_sub(1);
            if let Some(h) = self.history.get(pos) {
                self.input.clone_from(h);
                self.history_pos = Some(pos);
            }
        }
        if down && let Some(pos) = self.history_pos {
            if let Some(h) = self.history.get(pos + 1) {
                self.input.clone_from(h);
                self.history_pos = Some(pos + 1);
            } else {
                self.input.clear();
                self.history_pos = None;
            }
        }
        let working = self.chat.turn.is_some();
        let sees = self.agent.tools.iter().any(|t| t == "look");
        if sees {
            self.attachments.show(ui);
        }
        ui.horizontal(|ui| {
            if sees
                && ui
                    .small_icon_button(&icons::ADD, "Paste an image")
                    .on_hover_text(
                        "Paste an image from the clipboard for the agent to look at; or drop \
                         one on the window",
                    )
                    .clicked()
            {
                self.attachments.paste(ui.ctx());
            }
            let hint = if working {
                "Say something while it works: it reads it at its next step · Esc stops it"
            } else {
                "Message the robot · Enter sends, Shift+Enter adds a line"
            };
            let edit = egui::TextEdit::multiline(&mut self.input)
                .id(id)
                .hint_text(hint)
                .desired_rows(2)
                .desired_width(ui.available_width() - 72.0)
                .margin(Margin::symmetric(8, 6));
            ui.add(edit);
            let has_text = !self.input.trim().is_empty();
            if working && !has_text {
                if ui
                    .add(ReButton::new("Stop").secondary())
                    .on_hover_text("Stops the reply (Esc)")
                    .clicked()
                {
                    actions.push(Action::Send(Command::StopGeneration));
                }
            } else {
                let send = ui.add_enabled(has_text, ReButton::new("Send").primary());
                if has_text && (enter || send.clicked()) {
                    actions.push(Action::Say(
                        std::mem::take(&mut self.input).trim().to_owned(),
                    ));
                }
            }
        });
    }

    fn dock(&mut self, ui: &mut egui::Ui) {
        ui.add_space(8.0);
        // Wrapped, so the tabs never widen the dock past what the operator gave it.
        ui.horizontal_wrapped(|ui| {
            for (tab, name) in Tab::ALL {
                let count = if tab == Tab::Approvals {
                    self.chat.pending().count()
                } else {
                    0
                };
                let label = if count > 0 {
                    format!("{name} ({count})")
                } else {
                    name.to_owned()
                };
                if ui
                    .add(
                        ReButton::new(label)
                            .small()
                            .ghost()
                            .selected(self.tab == tab),
                    )
                    .clicked()
                {
                    self.tab = tab;
                }
            }
        });
        ui.full_span_separator();
        ui.add_space(8.0);
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| match self.tab {
                Tab::Mission => self.mission_tab(ui),
                Tab::World => self.world_tab(ui),
                Tab::Layers => self.layers_tab(ui),
                Tab::Approvals => self.approvals_tab(ui),
                Tab::Events => self.events_tab(ui),
                Tab::Agent => self.agent_tab(ui),
                Tab::Doctor => self.doctor_tab(ui),
                Tab::Robot => self.robot_tab(ui),
            });
    }

    fn mission_tab(&mut self, ui: &mut egui::Ui) {
        let mut actions = Vec::new();
        match self.chat.latest_plan() {
            Some(p) => {
                crate::chat::plan_card(ui, p, &mut actions);
                crate::chat::tree_panel(ui, p);
            }
            None => empty(
                ui,
                "No plan yet. Ask the robot to do something; its plan appears here.",
            ),
        }
        self.schedules_list(ui);
        let ledger = self.agent.missions.as_ref().and_then(|m| m.ledger());
        crate::history::show(ui, &mut self.records, ledger, &mut actions);
        self.act(ui.ctx(), actions);
    }

    /// Missions that run again and again, each with a way to end it.
    fn schedules_list(&self, ui: &mut egui::Ui) {
        let Some(schedules) = &self.agent.schedules else {
            return;
        };
        let list = schedules.list();
        if list.is_empty() {
            return;
        }
        ui.add_space(12.0);
        ui.label(RichText::new("Schedules").strong());
        let mut cancel = None;
        for s in &list {
            ui.horizontal(|ui| {
                ui.label(RichText::new(&s.id).monospace().small());
                ui.label(RichText::new(&s.intent).small());
                let when = if s.when.is_empty() {
                    format!(
                        "every {} min, {} of {} runs left",
                        s.every_min, s.left, s.times
                    )
                } else {
                    format!("when {}, {} of {} runs left", s.when, s.left, s.times)
                };
                ui.label(RichText::new(when).small().color(ui.tokens().text_subdued));
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui
                        .add(ReButton::new("Cancel").small().secondary())
                        .clicked()
                    {
                        cancel = Some(s.id.clone());
                    }
                });
            });
        }
        if let Some(id) = cancel {
            schedules.cancel(&id);
        }
    }

    fn world_tab(&mut self, ui: &mut egui::Ui) {
        if let (true, Some(editor)) = (self.editing, &mut self.editor) {
            editor.panel(ui);
            return;
        }
        if self.editor.is_some()
            && ui
                .add(ReButton::new("Edit the world").small())
                .on_hover_text("Fix labels, boxes and rooms on the saved floor plan")
                .clicked()
        {
            self.editing = true;
        }
        let (rooms, objects) = {
            let l = self.live();
            (l.rooms.clone(), l.objects)
        };
        let Some(rooms) = rooms else {
            empty(
                ui,
                "No world model yet. With one running, its rooms and how much of each the camera \
                 has seen appear here.",
            );
            return;
        };
        let actions = world_view(ui, &rooms, objects);
        self.act(ui.ctx(), actions);
    }

    fn robot_tab(&mut self, ui: &mut egui::Ui) {
        let status = {
            let l = self.live();
            crate::robot::Status {
                state: l.executor.clone(),
                pose: l.pose,
                rooms: l.rooms.clone(),
                names: l.object_names.clone(),
                battery: l.battery.clone(),
                motors: l.motors.clone(),
                armed: self.agent.guard.armed(),
            }
        };
        crate::robot::tab(ui, &status, self.drive.as_mut());
    }

    fn layers_tab(&self, ui: &mut egui::Ui) {
        layers_view(ui, &self.bridge.layers);
    }

    fn approvals_tab(&mut self, ui: &mut egui::Ui) {
        let pending: Vec<(u64, String, String)> = self
            .chat
            .pending()
            .map(|a| (a.id(), a.tool().to_owned(), a.reason().to_owned()))
            .collect();
        if pending.is_empty() {
            empty(
                ui,
                "Nothing to approve. Requests to act appear here and in the chat.",
            );
            return;
        }
        for (id, tool, reason) in pending {
            ui.horizontal(|ui| {
                ui.label(RichText::new(tool).monospace());
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.add(ReButton::new("Approve").small().primary()).clicked() {
                        self.agent.session.send(Command::Approve(id));
                    }
                    if ui.add(ReButton::new("Deny").small().secondary()).clicked() {
                        self.agent.session.send(Command::Deny(id));
                    }
                });
            });
            // What it would do, so the dock alone is enough to decide.
            ui.add(
                egui::Label::new(
                    RichText::new(reason)
                        .small()
                        .color(ui.tokens().text_subdued),
                )
                .wrap(),
            );
            ui.add_space(6.0);
        }
    }

    fn events_tab(&self, ui: &mut egui::Ui) {
        if self.event_log.is_empty() {
            empty(ui, "Session events appear here, newest first.");
        }
        for line in &self.event_log {
            ui.label(
                RichText::new(line)
                    .monospace()
                    .size(11.0)
                    .color(ui.tokens().text_subdued),
            );
        }
    }

    /// Saves a session as an eval case, and says where.
    fn save_as_case(&mut self, log: &Path) {
        match crate::sessions::save_as_case(log) {
            Ok((id, suite)) => self.toasts.add(
                Kind::Success,
                format!("Saved as test case {id} in {}", suite.display()),
            ),
            Err(e) => self.toasts.add(Kind::Failure, format!("Not saved: {e}")),
        }
    }

    /// Earlier sessions to carry on or keep as tests, then memory, then the models and quotas.
    fn agent_tab(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("Sessions").strong());
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui
                    .add(ReButton::new("Save this one as a test").small().secondary())
                    .on_hover_text(
                        "Keep this conversation as an eval case, expecting what happened; run \
                         it with nervros-cli eval",
                    )
                    .clicked()
                {
                    let log = self.log_path.clone();
                    self.save_as_case(&log);
                }
            });
        });
        let fresh = self
            .sessions
            .as_ref()
            .is_some_and(|(at, _)| at.elapsed() < Duration::from_secs(5));
        if !fresh {
            let logs = self
                .log_path
                .parent()
                .map(Path::to_path_buf)
                .unwrap_or_default();
            self.sessions = Some((Instant::now(), self.listing.list(&logs, &self.log_path)));
        }
        let (mut resume, mut keep) = (None, None);
        let listed = self
            .sessions
            .as_ref()
            .map_or(&[][..], |(_, l)| l.as_slice());
        if listed.is_empty() {
            ui.label(
                RichText::new("No earlier session saved its conversation yet.")
                    .small()
                    .color(ui.tokens().text_subdued),
            );
        }
        for s in listed {
            ui.horizontal(|ui| {
                ui.label(RichText::new(&s.when).monospace().small());
                ui.label(RichText::new(&s.first).small())
                    .on_hover_text(format!("{} messages", s.messages));
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui
                        .add(ReButton::new("Resume").small().secondary())
                        .on_hover_text("Carry on this conversation")
                        .clicked()
                    {
                        resume = Some(s.path.clone());
                    }
                    if ui
                        .add(ReButton::new("Save as test").small().secondary())
                        .on_hover_text("Keep it as an eval case, expecting what happened")
                        .clicked()
                    {
                        keep = Some(crate::sessions::log_of(&s.path));
                    }
                });
            });
        }
        if let Some(log) = keep {
            self.save_as_case(&log);
        }
        if let Some(path) = resume {
            match nervros_core::llm::History::load(&path) {
                Ok(history) => self.agent.session.send(Command::Restore(history)),
                Err(e) => self
                    .chat
                    .items
                    .push(crate::chat::Item::error(format!("{}: {e}", path.display()))),
            }
        }
        ui.add_space(12.0);
        self.memory_list(ui);
        ui.add_space(12.0);
        self.models_list(ui);
    }

    /// What the operator asked the agent to remember, each with a way to forget it.
    fn memory_list(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Memory").strong());
        let notes = self.agent.memory.all();
        if notes.is_empty() {
            ui.label(
                RichText::new("Nothing yet: say \"remember that…\" in the chat.")
                    .small()
                    .color(ui.tokens().text_subdued),
            );
        }
        let mut forget = None;
        for n in &notes {
            ui.horizontal(|ui| {
                ui.label(RichText::new(n.id.to_string()).monospace().small());
                ui.label(RichText::new(&n.text).small());
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui
                        .add(ReButton::new("Forget").small().secondary())
                        .clicked()
                    {
                        forget = Some(n.id);
                    }
                });
            });
        }
        if let Some(id) = forget
            && let Err(e) = self.agent.memory.forget(id)
        {
            self.chat.items.push(crate::chat::Item::error(e));
        }
    }

    fn models_list(&self, ui: &mut egui::Ui) {
        let router = self.agent.llm.router();
        let now = SystemTime::now();
        for (role, name) in [
            (Role::Routine, "Routine"),
            (Role::Plan, "Plan advice"),
            (Role::VisionCheck, "Vision check"),
            (Role::Summarise, "Summarise"),
            (Role::Segment, "Segment"),
            (Role::PlanCheck, "Plan check"),
        ] {
            ui.label(RichText::new(name).strong());
            let (take, skipped) = router.candidates(role, Need::default(), now);
            if take.is_empty() && skipped.is_empty() {
                ui.label(
                    RichText::new("off: models.toml lists no model for it")
                        .small()
                        .color(ui.tokens().text_subdued),
                );
            }
            for m in take {
                ui.horizontal(|ui| {
                    ui.bullet(ui.tokens().success_text_color);
                    ui.label(RichText::new(&m.id).monospace());
                    let q = quota(router, m, now);
                    ui.label(RichText::new(q).small().color(ui.tokens().text_subdued));
                });
            }
            for (id, why) in skipped {
                ui.horizontal(|ui| {
                    ui.bullet(ui.tokens().text_subdued);
                    ui.label(
                        RichText::new(id)
                            .monospace()
                            .color(ui.tokens().text_subdued),
                    );
                    ui.label(
                        RichText::new(format!("{why:?}"))
                            .small()
                            .color(ui.tokens().text_subdued),
                    );
                });
            }
            ui.add_space(8.0);
        }
    }

    fn doctor_tab(&self, ui: &mut egui::Ui) {
        // What the MCP servers brought at start goes with what the robot answers now.
        let checks = self.live().checks.clone().map(|mut c| {
            c.extend(self.agent.mcp.iter().cloned());
            c
        });
        match checks {
            None => {
                ui.horizontal(|ui| {
                    crate::chat::spinner(ui);
                    ui.label("Checking the robot…");
                });
            }
            Some(checks) => {
                let bad = checks.iter().filter(|c| !c.ok).count();
                if bad == 0 {
                    ui.success_label("Everything the profile names is there.");
                } else {
                    ui.warning_label(format!("{bad} of {} checks failed.", checks.len()));
                }
                ui.add_space(8.0);
                for c in &checks {
                    ui.horizontal(|ui| {
                        let t = ui.tokens();
                        ui.bullet(if c.ok {
                            t.success_text_color
                        } else {
                            t.error_fg_color
                        });
                        ui.label(RichText::new(&c.what).size(12.0));
                    });
                }
                ui.add_space(8.0);
                if ui
                    .add(ReButton::new("Check again").small().secondary())
                    .clicked()
                {
                    self.live().checks = None;
                    let _ = self.recheck.send(());
                }
            }
        }
    }

    fn status_bar(&self, ui: &mut egui::Ui, frame: &eframe::Frame) {
        let t = ui.tokens();
        let subdued = |s: String| RichText::new(s).small().color(t.text_subdued);
        ui.horizontal_centered(|ui| {
            ui.spacing_mut().item_spacing.x = 16.0;
            let state = self.chat.turn.map_or_else(
                || "idle".to_owned(),
                |since| format!("working {} s", since.elapsed().as_secs()),
            );
            ui.label(subdued(state));
            ui.label(subdued(format!("{} tools", self.agent.tools.len())));
            if let Some((used, window)) = self.chat.context {
                let text = format!(
                    "context {} / {}",
                    crate::chat::thousands(used),
                    crate::chat::thousands(window)
                );
                // Past three quarters a compaction is near; past the window, a request fails.
                let full = used.saturating_mul(4) >= window.saturating_mul(3);
                let colour = if full {
                    t.warn_fg_color
                } else {
                    t.text_subdued
                };
                ui.label(RichText::new(text).small().color(colour))
                    .on_hover_text(
                        "Tokens the latest request took; /compact condenses the conversation",
                    );
            }
            let spent = self.chat.spent;
            if spent.calls > 0 {
                let cached = spent.cached_tokens * 100 / spent.input_tokens.max(1);
                ui.label(subdued(format!("{} calls, {cached}% cached", spent.calls)))
                    .on_hover_text(format!(
                        "Model calls this session: {} tokens in, {} of them read from the model's \
                         prompt cache, {} out, {:.0} s waiting",
                        crate::chat::thousands(spent.input_tokens),
                        crate::chat::thousands(spent.cached_tokens),
                        crate::chat::thousands(spent.output_tokens),
                        Duration::from_millis(spent.ms).as_secs_f64(),
                    ));
            }
            if let Some(bytes) = re_memory::MemoryUse::capture().counted {
                #[expect(clippy::cast_precision_loss, reason = "shown to one decimal")]
                let gb = bytes as f64 / 1e9;
                ui.label(subdued(format!("memory {gb:.1} GB")));
            }
            if let Some(cpu) = frame.info().cpu_usage {
                ui.label(subdued(format!("UI {:.1} ms", cpu * 1000.0)));
            }
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                ui.label(subdued(format!("log {}", self.log_path.display())));
            });
        });
    }
}

/// A plan that walks to a room or an object of the world model.
fn walk_to_target(id: &str, what: &str) -> Value {
    serde_json::json!({
        "intent": format!("walk to {what}"),
        "steps": [{"skill": "GoToPlace", "args": [{"name": "place", "value": id}]}]
    })
}

/// A plan that walks to a point on the map, facing the way it walked from where it stands.
fn walk_to_point(x: f32, y: f32, from: Option<crate::robot::Pose>) -> Value {
    let (x, y) = (f64::from(x), f64::from(y));
    let yaw = from.map_or(0.0, |(fx, fy, heading)| {
        if (x - fx).hypot(y - fy) < 0.05 {
            heading
        } else {
            (y - fy).atan2(x - fx)
        }
    });
    serde_json::json!({
        "intent": format!("walk to x {x:.2}, y {y:.2}"),
        "steps": [{"skill": "GoToPose", "args": [
            {"name": "station", "value": format!("{x:.2};{y:.2};{yaw:.3}")}
        ]}]
    })
}

/// Today's use against the tightest daily limit: the shared pool's if the model is in one.
fn quota(router: &Router, m: &ModelConfig, now: SystemTime) -> String {
    if m.privacy.local {
        return "local".to_owned();
    }
    if let Some(name) = m.limits.pool.as_deref()
        && let Some(pool) = router.config().pools.get(name)
    {
        return format!("{}/{} today", router.used_today(name, now), pool.rpd);
    }
    let used = router.used_today(&m.id, now);
    m.limits.rpd.map_or_else(
        || format!("{used} today"),
        |rpd| format!("{used}/{rpd} today"),
    )
}

fn chip(ui: &mut egui::Ui, color: Color32, text: impl Into<String>) {
    Frame::new()
        .fill(ui.tokens().faint_bg_color)
        .corner_radius(CornerRadius::same(10))
        .inner_margin(Margin::symmetric(8, 3))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                let (rect, _) = ui.allocate_exact_size(egui::vec2(8.0, 8.0), egui::Sense::hover());
                ui.painter().circle_filled(rect.center(), 4.0, color);
                ui.label(RichText::new(text.into()).small());
            });
        });
}

/// The World tab: the world model's rooms, how much of each the camera has seen, and a button that
/// asks the agent to explore.
fn world_view(ui: &mut egui::Ui, rooms: &Value, objects: Option<usize>) -> Vec<Action> {
    let list = rooms["rooms"].as_array().cloned().unwrap_or_default();
    let fraction = |r: &Value, k: &str| r[k].as_f64().map(|v| v.clamp(0.0, 1.0));
    let seen: Vec<f64> = list
        .iter()
        .filter_map(|r| {
            let floor = fraction(r, "floor_coverage")?;
            Some(f64::midpoint(
                floor,
                fraction(r, "face_coverage").unwrap_or(floor),
            ))
        })
        .collect();
    let mut actions = Vec::new();
    ui.horizontal(|ui| {
        let mut head = format!("{} rooms · {} objects", list.len(), objects.unwrap_or(0));
        if !seen.is_empty() {
            #[expect(clippy::cast_precision_loss, reason = "a handful of rooms")]
            let mean = seen.iter().sum::<f64>() / seen.len() as f64;
            let _ = write!(head, " · {:.0}% seen", mean * 100.0);
        }
        ui.label(RichText::new(head).strong());
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            if ui
                .button("Explore")
                .on_hover_text(
                    "Ask the robot to explore the building. You approve the mission it plans.",
                )
                .clicked()
            {
                actions.push(Action::Say(EXPLORE_REQUEST.to_owned()));
            }
        });
    });
    ui.add_space(4.0);
    egui::ScrollArea::vertical().show(ui, |ui| {
        egui::Grid::new("world_rooms")
            .num_columns(4)
            .striped(true)
            .spacing([10.0, 6.0])
            .show(ui, |ui| {
                for head in ["Room", "Floor seen", "Walls seen", "Objects"] {
                    ui.label(RichText::new(head).small().color(ui.tokens().text_subdued));
                }
                ui.end_row();
                for r in &list {
                    let text = |k: &str| r[k].as_str().unwrap_or_default().to_owned();
                    let (id, name, kind) = (text("id"), text("name"), text("type"));
                    // canopy's names are often its types; say it once.
                    let name = [
                        id,
                        name.clone(),
                        if kind == name { String::new() } else { kind },
                    ]
                    .into_iter()
                    .filter(|t| !t.is_empty())
                    .collect::<Vec<_>>()
                    .join(" ");
                    ui.label(name);
                    for k in ["floor_coverage", "face_coverage"] {
                        match fraction(r, k) {
                            #[expect(clippy::cast_possible_truncation, reason = "a fraction")]
                            Some(f) => {
                                ui.add(
                                    egui::ProgressBar::new(f as f32)
                                        .desired_width(90.0)
                                        .text(format!("{:.0}%", f * 100.0)),
                                );
                            }
                            None => {
                                ui.label("·");
                            }
                        }
                    }
                    ui.label(
                        r["object_count"]
                            .as_u64()
                            .map_or("·".to_owned(), |n| n.to_string()),
                    );
                    ui.end_row();
                }
            });
    });
    actions
}

/// A switch per layer the viewer draws, as RViz's displays list has.
fn layers_view(ui: &mut egui::Ui, layers: &nervros_viz::Layers) {
    let list = layers.list();
    if list.is_empty() {
        empty(ui, "The profile gives the viewer nothing to draw.");
        return;
    }
    ui.label(
        RichText::new("What the world and camera views draw. A hidden layer is cleared.")
            .small()
            .color(ui.tokens().text_subdued),
    );
    ui.add_space(6.0);
    for (name, mut shown) in list {
        ui.horizontal(|ui| {
            if ui.toggle_switch(12.0, &mut shown).changed() {
                layers.set(&name, shown);
            }
            let text = RichText::new(&name);
            ui.label(if shown {
                text
            } else {
                text.color(ui.tokens().text_subdued)
            });
        });
        ui.add_space(2.0);
    }
}

fn empty(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).color(ui.tokens().text_subdued));
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

/// The executor's state and the robot's pose, once a second: the Robot tab and the stop
/// button's label follow them.
fn watch_robot(agent: &Agent, live: &SharedLive, ctx: &egui::Context) {
    let (robot, profile) = (Arc::clone(&agent.robot), agent.profile.clone());
    let (live, wake) = (Arc::clone(live), ctx.clone());
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_secs(1));
        let state = profile.mission.as_ref().map(|m| m.state.clone());
        let (map, base) = (
            profile.ros.map_frame.clone(),
            profile.ros.base_frame.clone(),
        );
        loop {
            tick.tick().await;
            // An executor that has gone quiet shows as gone, not as its last word.
            let executor = match &state {
                Some(topic) => robot
                    .latest_fresh(
                        topic,
                        "nervros_interfaces/msg/RobotState",
                        Duration::from_secs(1),
                        nervros_core::mission::STATE_FRESH,
                    )
                    .await
                    .ok(),
                None => None,
            };
            let pose = robot
                .transform(&map, &base)
                .ok()
                .map(|t| (t.translation[0], t.translation[1], t.yaw()));
            let changed = {
                let mut l = live.lock().unwrap_or_else(PoisonError::into_inner);
                let changed = l.executor != executor || l.pose != pose;
                l.executor = executor;
                l.pose = pose;
                changed
            };
            // An idle window sleeps.
            if changed {
                wake.request_repaint();
            }
        }
    });
}

/// Keeps [`Live`] current and wakes the window when the agent or the robot changes.
fn watch(
    agent: &Agent,
    live: &SharedLive,
    ctx: &egui::Context,
) -> tokio::sync::mpsc::UnboundedSender<()> {
    let mut events = agent.session.subscribe();
    let wake = ctx.clone();
    tokio::spawn(async move {
        while let Ok(_) | Err(broadcast::error::RecvError::Lagged(_)) = events.recv().await {
            wake.request_repaint();
        }
    });
    watch_robot(agent, live, ctx);
    let (robot, profile) = (Arc::clone(&agent.robot), agent.profile.clone());
    let (live_graph, wake) = (Arc::clone(live), ctx.clone());
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_secs(5));
        let world = profile.world.clone();
        loop {
            tick.tick().await;
            let topics = robot.graph().await.ok().map(|g| g.topics.len());
            let latest = |t: Option<nervros_core::profile::TopicRef>| {
                let robot = Arc::clone(&robot);
                async move {
                    let t = t?;
                    robot
                        .latest(&t.topic, &t.msg_type, Duration::from_secs(1))
                        .await
                        .ok()
                }
            };
            let rooms = latest(world.as_ref().and_then(|w| w.rooms.clone())).await;
            let objects_msg = latest(world.as_ref().and_then(|w| w.objects.clone())).await;
            let objects = objects_msg
                .as_ref()
                .and_then(|m| m["objects"].as_array().map(Vec::len));
            let object_names: HashMap<String, String> = objects_msg
                .as_ref()
                .and_then(|m| m["objects"].as_array())
                .map(|list| {
                    list.iter()
                        .filter_map(|o| {
                            let id = o["id"].as_str()?;
                            let name = o["name"]
                                .as_str()
                                .filter(|n| !n.is_empty())
                                .or_else(|| o["label"].as_str())?;
                            Some((id.to_owned(), name.to_owned()))
                        })
                        .collect()
                })
                .unwrap_or_default();
            let read = |topic: Option<String>, msg_type: &'static str| {
                let robot = Arc::clone(&robot);
                async move {
                    robot
                        .latest(&topic?, msg_type, Duration::from_secs(1))
                        .await
                        .ok()
                }
            };
            let battery = read(
                profile.robot.battery.clone(),
                "sensor_msgs/msg/BatteryState",
            )
            .await;
            let motors = read(
                profile.robot.diagnostics.clone(),
                "diagnostic_msgs/msg/DiagnosticArray",
            )
            .await;
            {
                let mut l = live_graph.lock().unwrap_or_else(PoisonError::into_inner);
                l.topics = topics;
                l.rooms = rooms;
                l.objects = objects;
                l.object_names = object_names;
                l.battery = battery;
                l.motors = motors;
            }
            wake.request_repaint();
        }
    });
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<()>();
    let (robot, profile) = (Arc::clone(&agent.robot), agent.profile.clone());
    let (live_checks, wake) = (Arc::clone(live), ctx.clone());
    tokio::spawn(async move {
        // Discovery needs a moment before the first check means anything.
        tokio::time::sleep(Duration::from_secs(3)).await;
        loop {
            let checks = nervros_core::doctor::run(&profile, robot.as_ref()).await;
            live_checks
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .checks = Some(checks);
            wake.request_repaint();
            if rx.recv().await.is_none() {
                return;
            }
        }
    });
    tx
}

#[cfg(test)]
#[path = "live_eval.rs"]
mod live_eval;

#[cfg(test)]
mod tests {
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
