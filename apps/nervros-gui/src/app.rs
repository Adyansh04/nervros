//! The window: a top bar, the chat on the left, the embedded viewer in the centre, a dock on the
//! right and a status bar. The viewer draws last, into whatever space the panels leave.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, SystemTime};

use nervros_core::app::Agent;
use nervros_core::doctor::Check;
use nervros_core::providers::router::{Need, Router};
use nervros_core::providers::{ModelConfig, Role};
use nervros_core::session::{Command, Event};
use rerun::external::egui::{
    self, Align, Color32, CornerRadius, Frame, Key, Layout, Margin, Modifiers, RichText,
};
use rerun::external::re_log_channel::LogReceiver;
use rerun::external::re_sdk_types::blueprint::components::PanelState;
use rerun::external::re_ui::{ReButton, UiExt as _};
use rerun::external::{eframe, re_memory, re_viewer};
use serde_json::Value;
use tokio::sync::broadcast;

use crate::chat::{Action, Chat};

const EVENT_LOG: usize = 300;
const COMPOSER: &str = "nervros_composer";
const STOP_HINT: &str = "Halts the mission and cancels every goal; the hands keep their grip. \
                         The e-stop on the robot's remote is the real emergency stop.";

/// What background tasks learn about the robot.
#[derive(Debug, Default)]
struct Live {
    /// Topics in the graph, once read.
    pub topics: Option<usize>,
    /// The executor's last `RobotState`.
    pub executor: Option<Value>,
    /// The last connection check.
    pub checks: Option<Vec<Check>>,
}

/// Shared between the window and the tasks that fill it.
type SharedLive = Arc<Mutex<Live>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tab {
    Mission,
    Approvals,
    Events,
    Models,
    Doctor,
}

impl Tab {
    const ALL: [(Self, &'static str); 5] = [
        (Self::Mission, "Mission"),
        (Self::Approvals, "Approvals"),
        (Self::Events, "Events"),
        (Self::Models, "Models"),
        (Self::Doctor, "Doctor"),
    ];
}

/// The app.
pub struct Gui {
    viewer: re_viewer::App,
    agent: Agent,
    events: broadcast::Receiver<Event>,
    live: SharedLive,
    log_path: PathBuf,
    chat: Chat,
    event_log: VecDeque<String>,
    input: String,
    history: Vec<String>,
    history_pos: Option<usize>,
    tab: Tab,
    dock_open: bool,
    recheck: tokio::sync::mpsc::UnboundedSender<()>,
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
        input: LogReceiver,
        memory_limit: re_memory::MemoryLimit,
        runtime: tokio::runtime::Handle,
        log_path: PathBuf,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        re_viewer::customize_eframe_and_setup_renderer(cc)?;
        let mut startup = re_viewer::StartupOptions {
            hide_welcome_screen: true,
            expect_data_soon: Some(true),
            ..re_viewer::StartupOptions::default()
        };
        // Our own top bar replaces the viewer's; its side panels start collapsed.
        let panels = &mut startup.panel_state_overrides;
        panels.top = Some(PanelState::Hidden);
        panels.blueprint = Some(PanelState::Collapsed);
        panels.selection = Some(PanelState::Collapsed);
        panels.time = Some(PanelState::Collapsed);
        let mut viewer = re_viewer::App::new(
            main_thread,
            re_viewer::build_info(),
            re_viewer::AppEnvironment::Custom("NervROS".to_owned()),
            startup,
            cc,
            None,
            re_viewer::AsyncRuntimeHandle::new_native(runtime),
        );
        viewer.app_options_mut().memory_limit = memory_limit;
        viewer.add_log_receiver(input);
        let live = Arc::new(Mutex::new(Live::default()));
        let recheck = watch(&agent, &live, &cc.egui_ctx);
        let events = agent.session.subscribe();
        Ok(Self {
            viewer,
            agent,
            events,
            live,
            log_path,
            chat: Chat::default(),
            event_log: VecDeque::new(),
            input: String::new(),
            history: Vec::new(),
            history_pos: None,
            tab: Tab::Approvals,
            dock_open: true,
            recheck,
        })
    }

    fn live(&self) -> std::sync::MutexGuard<'_, Live> {
        self.live.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn drain_events(&mut self) {
        loop {
            match self.events.try_recv() {
                Ok(e) => {
                    self.chat.apply(&e);
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

    /// The bubble appears when the session starts the turn, so a message it turns away shows
    /// only as its notice.
    fn say(&mut self, text: String) {
        self.history.push(text.clone());
        self.history_pos = None;
        self.agent.session.send(Command::User(text));
    }

    fn act(&mut self, ctx: &egui::Context, actions: Vec<Action>) {
        for a in actions {
            match a {
                Action::Send(c) => self.agent.session.send(c),
                Action::Say(text) => self.say(text),
                Action::Prefill(text) => {
                    self.input = text;
                    ctx.memory_mut(|m| m.request_focus(egui::Id::new(COMPOSER)));
                }
            }
        }
    }

    fn keys(&mut self, ctx: &egui::Context) {
        let working = self.chat.turn.is_some();
        let (esc, stop, tab) = ctx.input_mut(|i| {
            let esc = working && i.consume_key(Modifiers::NONE, Key::Escape);
            let stop = i.consume_key(Modifiers::CTRL | Modifiers::SHIFT, Key::S);
            let keys = [Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5];
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
                let obj = s[format!("holding_{hand}")]
                    .as_str()
                    .filter(|o| !o.is_empty())?;
                Some(format!("Stop · {hand} hand keeps {obj}"))
            })
        });
        held.unwrap_or_else(|| "Stop mission".to_owned())
    }

    fn top_bar(&mut self, ui: &mut egui::Ui) {
        let t = ui.tokens();
        ui.horizontal_centered(|ui| {
            ui.spacing_mut().item_spacing.x = 8.0;
            ui.label(RichText::new("NervROS").strong().size(15.0));
            ui.label(RichText::new(&self.agent.profile.robot.name).color(t.text_subdued));
            ui.add_space(8.0);
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
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
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
            .show(ui, |ui| self.composer(ui, &mut actions));
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

    fn composer(&mut self, ui: &mut egui::Ui, actions: &mut Vec<Action>) {
        let id = egui::Id::new(COMPOSER);
        let focused = ui.memory(|m| m.has_focus(id));
        let browsing = self.input.is_empty() || self.history_pos.is_some();
        let (enter, up, down) = ui.input_mut(|i| {
            if !focused {
                return (false, false, false);
            }
            let enter = i.consume_key(Modifiers::NONE, Key::Enter);
            let up = browsing && i.consume_key(Modifiers::NONE, Key::ArrowUp);
            let down = browsing && i.consume_key(Modifiers::NONE, Key::ArrowDown);
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
        ui.horizontal(|ui| {
            let edit = egui::TextEdit::multiline(&mut self.input)
                .id(id)
                .hint_text("Message the robot · Enter sends, Shift+Enter adds a line")
                .desired_rows(2)
                .desired_width(ui.available_width() - 72.0)
                .margin(Margin::symmetric(8, 6));
            ui.add(edit);
            if working {
                if ui
                    .add(ReButton::new("Stop").secondary())
                    .on_hover_text("Stops the reply (Esc)")
                    .clicked()
                {
                    actions.push(Action::Send(Command::StopGeneration));
                }
            } else {
                let can_send = !self.input.trim().is_empty();
                let send = ui.add_enabled(can_send, ReButton::new("Send").primary());
                if can_send && (enter || send.clicked()) {
                    actions.push(Action::Say(
                        std::mem::take(&mut self.input).trim().to_owned(),
                    ));
                }
            }
        });
    }

    fn dock(&mut self, ui: &mut egui::Ui) {
        ui.add_space(8.0);
        ui.horizontal(|ui| {
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
                Tab::Approvals => self.approvals_tab(ui),
                Tab::Events => self.events_tab(ui),
                Tab::Models => self.models_tab(ui),
                Tab::Doctor => self.doctor_tab(ui),
            });
    }

    fn mission_tab(&mut self, ui: &mut egui::Ui) {
        let mut actions = Vec::new();
        match self.chat.latest_plan() {
            Some(p) => crate::chat::plan_card(ui, p, &mut actions),
            None => empty(
                ui,
                "No plan yet. Ask the robot to do something; its plan appears here.",
            ),
        }
        self.act(ui.ctx(), actions);
    }

    fn approvals_tab(&mut self, ui: &mut egui::Ui) {
        let pending: Vec<(u64, String)> = self
            .chat
            .pending()
            .map(|a| (a.id(), a.tool().to_owned()))
            .collect();
        if pending.is_empty() {
            empty(
                ui,
                "Nothing to approve. Requests to act appear here and in the chat.",
            );
            return;
        }
        for (id, tool) in pending {
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

    fn models_tab(&self, ui: &mut egui::Ui) {
        let router = self.agent.llm.router();
        let now = SystemTime::now();
        for (role, name) in [
            (Role::Routine, "Routine"),
            (Role::Plan, "Plan"),
            (Role::VisionCheck, "Vision check"),
            (Role::Summarise, "Summarise"),
        ] {
            ui.label(RichText::new(name).strong());
            let (take, skipped) = router.candidates(role, Need::default(), now);
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
        let checks = self.live().checks.clone();
        match checks {
            None => {
                ui.horizontal(|ui| {
                    ui.spinner();
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

fn empty(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).color(ui.tokens().text_subdued));
}

impl eframe::App for Gui {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        self.drain_events();
        self.keys(ui.ctx());
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
                .default_size(320.0)
                .frame(
                    Frame::new()
                        .fill(t.panel_bg_color)
                        .inner_margin(Margin::symmetric(12, 4)),
                )
                .show(ui, |ui| self.dock(ui));
        }
        self.viewer.ui(ui, frame);
    }

    fn logic(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        self.viewer.logic(ctx, frame);
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        self.viewer.save(storage);
    }
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
    let (robot, profile) = (Arc::clone(&agent.robot), agent.profile.clone());
    let (live_graph, wake) = (Arc::clone(live), ctx.clone());
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_secs(5));
        let state = profile.mission.as_ref().map(|m| m.state.clone());
        loop {
            tick.tick().await;
            let topics = robot.graph().await.ok().map(|g| g.topics.len());
            let executor = match &state {
                Some(topic) => robot
                    .latest(
                        topic,
                        "nervros_interfaces/msg/RobotState",
                        Duration::from_secs(1),
                    )
                    .await
                    .ok(),
                None => None,
            };
            {
                let mut l = live_graph.lock().unwrap_or_else(PoisonError::into_inner);
                l.topics = topics;
                l.executor = executor;
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
mod tests {
    use super::*;
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
        let (rec, input) = nervros_viz::in_process().unwrap();
        let _bridge = nervros_viz::spawn(
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
                let limit = re_memory::MemoryLimit::from_bytes(1 << 30);
                let log = PathBuf::from("session.ndjson");
                let token = re_viewer::MainThreadToken::i_promise_i_am_only_using_this_for_a_test();
                let mut gui = Gui::start(token, cc, agent, input, limit, handle, log).unwrap();
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
    }
}
