//! The conversation as the operator sees it: items built from session events, and how each one
//! is drawn. Colours and sizes come from `re_ui`'s tokens so chat and viewer read as one app.

use std::cell::OnceCell;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use nervros_core::mission::plan::PlannedStep;
use nervros_core::session::{Command, Event};
use rerun::external::egui::{self, Align, Color32, CornerRadius, Frame, Layout, Margin, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _, icons};
use serde_json::Value;

/// Text never runs wider than this, for readable line lengths.
const MAX_TEXT_WIDTH: f32 = 720.0;
const SUGGESTIONS: [&str; 3] = [
    "What do you see?",
    "Where are you?",
    "Which places can you go to?",
];

/// What a click in the chat asks the app to do.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Action {
    /// Send a command to the session.
    Send(Command),
    /// Send text as if typed.
    Say(String),
    /// Put text in the composer.
    Prefill(String),
}

/// One tool call, from its start to its end.
#[derive(Debug, Clone)]
pub struct ToolCall {
    turn: u64,
    call: u64,
    tool: String,
    args: Value,
    status: Option<&'static str>,
    message: String,
    ms: u64,
}

/// A request the operator must answer.
#[derive(Debug, Clone)]
pub struct Approval {
    id: u64,
    tool: String,
    args: Value,
    reason: String,
    asked: Instant,
    answer: Option<bool>,
}

impl Approval {
    /// Answer with this id.
    pub fn id(&self) -> u64 {
        self.id
    }

    /// The tool that asks.
    pub fn tool(&self) -> &str {
        &self.tool
    }
}

/// One entry in the conversation.
#[derive(Clone)]
pub enum Item {
    /// The operator's message.
    User(String),
    /// The agent's text.
    Reply {
        /// Markdown.
        text: String,
        /// The model that wrote it.
        model: String,
    },
    /// A tool call.
    Tool(ToolCall),
    /// A marked image.
    Image {
        /// The snapshot id marks refer to.
        id: String,
        /// Decoded once, uploaded on first draw.
        pixels: Arc<egui::ColorImage>,
        texture: OnceCell<egui::TextureHandle>,
    },
    /// An approval request.
    Approval(Approval),
    /// Something the operator should know.
    Notice(String),
    /// A failed turn.
    Error(String),
    /// A report from the robot, which the agent then answers.
    Report(String),
    /// A plan, and the mission that runs it.
    Plan(PlanCard),
}

/// A plan from `plan_mission`, updated as its mission runs.
#[derive(Clone)]
pub struct PlanCard {
    hash: String,
    intent: String,
    steps: Vec<PlannedStep>,
    worst_case_s: f64,
    mission: Option<String>,
    started: Option<Instant>,
    /// Per step: its status and the node running inside it.
    progress: HashMap<String, (String, String)>,
    /// `(outcome, failed step, reason, seconds)` once it ended.
    finished: Option<(String, String, String, f64)>,
}

/// The conversation.
#[derive(Default)]
pub struct Chat {
    /// Items, oldest first.
    pub items: Vec<Item>,
    /// When the running turn started.
    pub turn: Option<Instant>,
    /// The model of the last reply.
    pub model: Option<String>,
    last_user: Option<String>,
}

impl Chat {
    /// Adds the operator's message.
    fn push_user(&mut self, text: String) {
        self.last_user = Some(text.clone());
        self.items.push(Item::User(text));
    }

    /// The pending approvals, oldest first.
    pub fn pending(&self) -> impl Iterator<Item = &Approval> {
        self.items.iter().filter_map(|i| match i {
            Item::Approval(a) if a.answer.is_none() => Some(a),
            _ => None,
        })
    }

    /// Folds a session event into the conversation.
    pub fn apply(&mut self, event: &Event) {
        match event {
            Event::TurnStarted { .. } => self.turn = Some(Instant::now()),
            Event::TurnFinished { .. } => self.turn = None,
            Event::Reply { text, model, .. } => {
                self.model = Some(model.clone());
                self.items.push(Item::Reply {
                    text: text.clone(),
                    model: model.clone(),
                });
            }
            Event::ToolStarted {
                turn,
                call,
                tool,
                args,
            } => self.items.push(Item::Tool(ToolCall {
                turn: *turn,
                call: *call,
                tool: tool.clone(),
                args: args.clone(),
                status: None,
                message: String::new(),
                ms: 0,
            })),
            Event::ToolFinished {
                turn,
                call,
                status,
                message,
                ms,
                ..
            } => {
                let found = self.items.iter_mut().rev().find_map(|i| match i {
                    Item::Tool(t) if t.turn == *turn && t.call == *call => Some(t),
                    _ => None,
                });
                if let Some(t) = found {
                    t.status = Some(status);
                    t.message.clone_from(message);
                    t.ms = *ms;
                }
            }
            Event::Snapshot {
                id,
                jpeg,
                width,
                height,
            } => match decode(jpeg) {
                Some(pixels) => self.items.push(Item::Image {
                    id: id.clone(),
                    pixels: Arc::new(pixels),
                    texture: OnceCell::new(),
                }),
                None => self.items.push(Item::Notice(format!(
                    "Snapshot {id} ({width}×{height}) could not be decoded"
                ))),
            },
            Event::ApprovalRequested {
                id,
                tool,
                args,
                reason,
            } => self.items.push(Item::Approval(Approval {
                id: *id,
                tool: tool.clone(),
                args: args.clone(),
                reason: reason.clone(),
                asked: Instant::now(),
                answer: None,
            })),
            Event::ApprovalResolved { id, approved } => {
                for item in &mut self.items {
                    if let Item::Approval(a) = item
                        && a.id == *id
                    {
                        a.answer = Some(*approved);
                    }
                }
            }
            Event::Armed { armed } => self.items.push(Item::Notice(if *armed {
                "Armed: the agent may now act".to_owned()
            } else {
                "Observe only: the agent cannot act".to_owned()
            })),
            Event::Halted { reason } => self.items.push(Item::Notice(format!("Stopped: {reason}"))),
            Event::Notice { text } => self.items.push(Item::Notice(text.clone())),
            Event::Error { text, .. } => self.items.push(Item::Error(text.clone())),
            Event::User { text, .. } => self.push_user(text.clone()),
            Event::Report { text, .. } => self.items.push(Item::Report(text.clone())),
            Event::MissionPlanned { .. }
            | Event::MissionStarted { .. }
            | Event::MissionProgress { .. }
            | Event::MissionFinished { .. } => self.apply_mission(event),
        }
    }

    fn apply_mission(&mut self, event: &Event) {
        match event {
            Event::MissionPlanned {
                hash,
                intent,
                steps,
                worst_case_s,
            } => self.items.push(Item::Plan(PlanCard {
                hash: hash.clone(),
                intent: intent.clone(),
                steps: steps.clone(),
                worst_case_s: *worst_case_s,
                mission: None,
                started: None,
                progress: HashMap::new(),
                finished: None,
            })),
            Event::MissionStarted { id, hash } => {
                if let Some(p) = self.plan_mut(|p| p.hash == *hash) {
                    p.mission = Some(id.clone());
                    p.started = Some(Instant::now());
                }
            }
            Event::MissionProgress {
                id,
                step,
                node,
                status,
                ..
            } => {
                if let Some(p) = self.plan_mut(|p| p.mission.as_ref() == Some(id)) {
                    let entry = p.progress.entry(step.clone()).or_default();
                    if node.is_empty() {
                        entry.0.clone_from(status);
                    } else if status == "running" {
                        entry.1.clone_from(node);
                    }
                }
            }
            Event::MissionFinished {
                id,
                outcome,
                failed_step,
                reason,
                elapsed_s,
            } => {
                if let Some(p) = self.plan_mut(|p| p.mission.as_ref() == Some(id)) {
                    p.finished = Some((
                        outcome.clone(),
                        failed_step.clone(),
                        reason.clone(),
                        *elapsed_s,
                    ));
                }
            }
            _ => {}
        }
    }

    /// The newest plan matching `which`.
    fn plan_mut(&mut self, which: impl Fn(&PlanCard) -> bool) -> Option<&mut PlanCard> {
        self.items.iter_mut().rev().find_map(|i| match i {
            Item::Plan(p) if which(p) => Some(p),
            _ => None,
        })
    }

    /// The plan an approval of `run_mission` refers to.
    fn plan_for(&self, a: &Approval) -> Option<&PlanCard> {
        let hash = a.args["hash"].as_str()?;
        self.items.iter().rev().find_map(|i| match i {
            Item::Plan(p) if !hash.is_empty() && p.hash.starts_with(hash) => Some(p),
            _ => None,
        })
    }

    /// The newest plan, for the dock.
    pub fn latest_plan(&self) -> Option<&PlanCard> {
        self.items.iter().rev().find_map(|i| match i {
            Item::Plan(p) => Some(p),
            _ => None,
        })
    }

    /// Draws the conversation; clicks are appended to `actions`.
    pub fn show(&self, ui: &mut egui::Ui, approval_ttl: Duration, actions: &mut Vec<Action>) {
        if self.items.is_empty() && self.turn.is_none() {
            empty_state(ui, actions);
            return;
        }
        let width = ui.available_width().min(MAX_TEXT_WIDTH);
        ui.spacing_mut().item_spacing.y = 8.0;
        for item in &self.items {
            ui.scope(|ui| {
                ui.set_max_width(width);
                match item {
                    Item::User(text) => user_bubble(ui, text),
                    Item::Reply { text, model } => reply(ui, text, model),
                    Item::Tool(t) => tool_chip(ui, t),
                    Item::Image {
                        id,
                        pixels,
                        texture,
                    } => {
                        let texture = texture.get_or_init(|| {
                            let options = egui::TextureOptions::LINEAR;
                            ui.ctx().load_texture(id, Arc::clone(pixels), options)
                        });
                        image_card(ui, id, texture, actions);
                    }
                    Item::Approval(a) => {
                        approval_card(ui, a, self.plan_for(a), approval_ttl, actions);
                    }
                    Item::Notice(text) => notice(ui, text),
                    Item::Error(text) => error_card(ui, text, self.last_user.as_deref(), actions),
                    Item::Report(text) => report(ui, text),
                    Item::Plan(p) => plan_card(ui, p, actions),
                }
            });
        }
        if let Some(since) = self.turn {
            ui.horizontal(|ui| {
                ui.spinner();
                let secs = since.elapsed().as_secs();
                ui.label(
                    RichText::new(format!("Working… {secs} s")).color(ui.tokens().text_subdued),
                );
            });
            // Only while a turn runs, so an idle app does not redraw.
            ui.ctx().request_repaint_after(Duration::from_millis(250));
        }
    }
}

/// Running is blue, success green, failure red and waiting amber, everywhere in the app.
pub fn status_color(ui: &egui::Ui, status: Option<&str>) -> Color32 {
    let t = ui.tokens();
    match status {
        None => t.info_text_color,
        Some("succeeded" | "accepted") => t.success_text_color,
        Some("refused") => t.warn_fg_color,
        Some(_) => t.error_fg_color,
    }
}

fn card(ui: &egui::Ui, stroke: Color32) -> Frame {
    Frame::new()
        .fill(ui.tokens().panel_bg_color)
        .stroke(egui::Stroke::new(1.0, stroke))
        .corner_radius(CornerRadius::same(10))
        .inner_margin(Margin::same(12))
}

fn empty_state(ui: &mut egui::Ui, actions: &mut Vec<Action>) {
    ui.add_space(ui.available_height() * 0.3);
    ui.vertical_centered(|ui| {
        ui.label(
            RichText::new("Ask about the robot and its surroundings")
                .strong()
                .size(16.0),
        );
        ui.add_space(8.0);
        ui.label(
            RichText::new("The agent can look, find objects and check the robot's state.")
                .color(ui.tokens().text_subdued),
        );
        ui.add_space(16.0);
        for s in SUGGESTIONS {
            if ui.add(ReButton::new(s).secondary()).clicked() {
                actions.push(Action::Say(s.to_owned()));
            }
            ui.add_space(4.0);
        }
    });
}

fn user_bubble(ui: &mut egui::Ui, text: &str) {
    ui.with_layout(Layout::right_to_left(Align::Min), |ui| {
        Frame::new()
            .fill(ui.tokens().selection_bg_fill)
            .corner_radius(CornerRadius::same(12))
            .inner_margin(Margin::symmetric(12, 8))
            .show(ui, |ui| {
                ui.set_max_width(ui.available_width() * 0.8);
                ui.label(RichText::new(text).color(ui.tokens().text_strong));
            });
    });
}

fn reply(ui: &mut egui::Ui, text: &str, model: &str) {
    ui.markdown_ui(text);
    ui.label(RichText::new(model).small().color(ui.tokens().text_subdued));
}

fn tool_chip(ui: &mut egui::Ui, t: &ToolCall) {
    let color = status_color(ui, t.status);
    let summary = t.message.lines().next().unwrap_or_default();
    let header = format!(
        "{}  {}{}",
        t.tool,
        summary,
        if t.status.is_some() {
            format!("  · {} ms", t.ms)
        } else {
            String::new()
        }
    );
    Frame::new()
        .fill(ui.tokens().faint_bg_color)
        .corner_radius(CornerRadius::same(8))
        .inner_margin(Margin::symmetric(8, 4))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                if t.status.is_none() {
                    ui.spinner();
                } else {
                    let icon = match t.status {
                        Some("succeeded" | "accepted") => &icons::SUCCESS,
                        Some("refused") => &icons::WARNING,
                        _ => &icons::ERROR,
                    };
                    ui.small_icon(icon, Some(color));
                }
                egui::CollapsingHeader::new(RichText::new(header).monospace().size(12.0))
                    .id_salt(("tool", t.turn, t.call))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("arguments")
                                .small()
                                .color(ui.tokens().text_subdued),
                        );
                        code(ui, &pretty(&t.args));
                        if !t.message.is_empty() {
                            ui.label(
                                RichText::new("result")
                                    .small()
                                    .color(ui.tokens().text_subdued),
                            );
                            code(ui, &t.message);
                        }
                    });
            });
        });
}

fn image_card(
    ui: &mut egui::Ui,
    id: &str,
    texture: &egui::TextureHandle,
    actions: &mut Vec<Action>,
) {
    let stroke = ui.tokens().widget_noninteractive_bg_stroke;
    card(ui, stroke).show(ui, |ui| {
        let [w, h] = texture.size();
        ui.add(
            egui::Image::new(texture)
                .max_width(ui.available_width())
                .corner_radius(CornerRadius::same(6)),
        );
        ui.horizontal(|ui| {
            let caption = RichText::new(format!("Snapshot {id} · {w}×{h}"));
            ui.label(caption.small().color(ui.tokens().text_subdued));
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui
                    .add(ReButton::new("Ask about it").small().primary())
                    .clicked()
                {
                    actions.push(Action::Prefill(format!("In snapshot {id}, ")));
                }
            });
        });
    });
}

/// A JPEG as egui pixels, or `None` if it does not decode.
fn decode(jpeg: &[u8]) -> Option<egui::ColorImage> {
    let img = image::load_from_memory_with_format(jpeg, image::ImageFormat::Jpeg).ok()?;
    let rgba = img.to_rgba8();
    let size = [rgba.width(), rgba.height()].map(|v| usize::try_from(v).unwrap_or(0));
    Some(egui::ColorImage::from_rgba_unmultiplied(
        size,
        rgba.as_raw(),
    ))
}

fn approval_card(
    ui: &mut egui::Ui,
    a: &Approval,
    plan: Option<&PlanCard>,
    ttl: Duration,
    actions: &mut Vec<Action>,
) {
    let t = ui.tokens();
    let stroke = if a.answer.is_none() {
        t.warn_fg_color
    } else {
        t.widget_noninteractive_bg_stroke
    };
    card(ui, stroke).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.small_icon(&icons::WARNING, Some(t.warn_fg_color));
            let title = match plan {
                Some(p) => format!("Run \"{}\"?", p.intent),
                None => format!("Approve {}?", a.tool),
            };
            ui.label(RichText::new(title).strong());
        });
        if let Some(p) = plan {
            steps_table(ui, p, "approval");
            let short = p.hash.get(..8).unwrap_or(&p.hash);
            ui.label(
                RichText::new(format!(
                    "plan {short} · at most {}",
                    minutes(p.worst_case_s)
                ))
                .small()
                .color(t.text_subdued),
            );
        } else {
            ui.label(&a.reason);
            code(ui, &pretty(&a.args));
        }
        ui.horizontal(|ui| match a.answer {
            Some(true) => {
                ui.label(RichText::new("Approved").color(t.success_text_color));
            }
            Some(false) => {
                ui.label(RichText::new("Denied").color(t.text_subdued));
            }
            None => {
                let left = ttl.saturating_sub(a.asked.elapsed()).as_secs_f32().ceil();
                ui.label(RichText::new(format!("{left} s left")).color(t.text_subdued));
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.add(ReButton::new("Approve").primary()).clicked() {
                        actions.push(Action::Send(Command::Approve(a.id)));
                    }
                    if ui.add(ReButton::new("Deny").secondary()).clicked() {
                        actions.push(Action::Send(Command::Deny(a.id)));
                    }
                });
                ui.ctx().request_repaint_after(Duration::from_secs(1));
            }
        });
    });
}

/// The plan's steps with each one's state; `salt` tells apart two tables of one plan.
fn steps_table(ui: &mut egui::Ui, p: &PlanCard, salt: &str) {
    let t = ui.tokens();
    egui::Grid::new(("plan_steps", &p.hash, salt))
        .num_columns(3)
        .spacing([8.0, 4.0])
        .show(ui, |ui| {
            for s in &p.steps {
                let (status, node) = p.progress.get(&s.id).cloned().unwrap_or_default();
                let failed = p.finished.as_ref().is_some_and(|f| f.1 == s.id);
                let (color, mark) = match (status.as_str(), failed) {
                    (_, true) | ("failure", _) => (t.error_fg_color, "✗"),
                    ("success", _) => (t.success_text_color, "✓"),
                    ("running", _) => (t.info_text_color, "●"),
                    ("skipped", _) => (t.text_subdued, "–"),
                    _ => (t.text_subdued, "○"),
                };
                ui.label(RichText::new(mark).color(color));
                ui.label(
                    RichText::new(&s.id)
                        .monospace()
                        .size(12.0)
                        .color(t.text_subdued),
                );
                let text = if status == "running" && !node.is_empty() {
                    format!("{} · {node}", s.summary)
                } else {
                    s.summary.clone()
                };
                ui.label(RichText::new(text).monospace().size(12.0));
                ui.end_row();
            }
        });
}

fn minutes(seconds: f64) -> String {
    if seconds < 90.0 {
        format!("{seconds:.0} s")
    } else {
        format!("{:.0} min", seconds / 60.0)
    }
}

/// A plan and its mission: steps with live state, and one action while it runs.
pub fn plan_card(ui: &mut egui::Ui, p: &PlanCard, actions: &mut Vec<Action>) {
    let t = ui.tokens();
    let stroke = match &p.finished {
        Some((outcome, ..)) if outcome == "success" => t.success_text_color,
        Some(_) => t.error_fg_color,
        None if p.mission.is_some() => t.info_text_color,
        None => t.widget_noninteractive_bg_stroke,
    };
    card(ui, stroke).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.small_icon(&icons::PLAN_PENDING, Some(t.text_subdued));
            ui.label(RichText::new(format!("Plan · {}", p.intent)).strong());
        });
        steps_table(ui, p, "plan");
        ui.horizontal(|ui| {
            let state = match (&p.finished, p.started) {
                (Some((outcome, .., secs)), _) if outcome == "success" => {
                    RichText::new(format!("Done in {}", minutes(*secs))).color(t.success_text_color)
                }
                (Some((outcome, step, reason, _)), _) => {
                    let at = if step.is_empty() {
                        String::new()
                    } else {
                        format!(" at {step}")
                    };
                    RichText::new(format!("{}{at}: {reason}", capitalise(outcome)))
                        .color(t.error_fg_color)
                }
                (None, Some(since)) => {
                    ui.ctx().request_repaint_after(Duration::from_secs(1));
                    RichText::new(format!(
                        "Running {}",
                        minutes(since.elapsed().as_secs_f64())
                    ))
                    .color(t.info_text_color)
                }
                (None, None) => RichText::new(format!(
                    "Checked · at most {} · waiting to run",
                    minutes(p.worst_case_s)
                ))
                .color(t.text_subdued),
            };
            ui.label(state.small());
            if p.mission.is_some() && p.finished.is_none() {
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.add(ReButton::new("Stop").small().secondary()).clicked() {
                        actions.push(Action::Send(Command::StopMission));
                    }
                });
            }
        });
    });
}

fn capitalise(word: &str) -> String {
    let mut c = word.chars();
    c.next()
        .map_or_else(String::new, |f| f.to_uppercase().chain(c).collect())
}

fn report(ui: &mut egui::Ui, text: &str) {
    Frame::new()
        .fill(ui.tokens().faint_bg_color)
        .corner_radius(CornerRadius::same(8))
        .inner_margin(Margin::symmetric(10, 6))
        .show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.small_icon(&icons::AGENT, Some(ui.tokens().text_subdued));
                ui.label(RichText::new("Robot report").small().strong());
                ui.label(RichText::new(text).small().color(ui.tokens().text_subdued));
            });
        });
}

fn notice(ui: &mut egui::Ui, text: &str) {
    ui.horizontal(|ui| {
        ui.small_icon(&icons::INFO, Some(ui.tokens().text_subdued));
        // The viewer's style extends labels, so a long notice would widen the panel instead.
        ui.add(egui::Label::new(RichText::new(text).color(ui.tokens().text_subdued)).wrap());
    });
}

fn error_card(ui: &mut egui::Ui, text: &str, last_user: Option<&str>, actions: &mut Vec<Action>) {
    let t = ui.tokens();
    card(ui, t.error_fg_color).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.small_icon(&icons::ERROR, Some(t.error_fg_color));
            ui.add(egui::Label::new(RichText::new(text).color(t.error_fg_color)).wrap());
        });
        if let Some(last) = last_user
            && ui.add(ReButton::new("Retry").small().secondary()).clicked()
        {
            actions.push(Action::Say(last.to_owned()));
        }
    });
}

fn pretty(v: &Value) -> String {
    serde_json::to_string_pretty(v).unwrap_or_else(|_| v.to_string())
}

fn code(ui: &mut egui::Ui, text: &str) {
    Frame::new()
        .fill(ui.tokens().extreme_bg_color)
        .corner_radius(CornerRadius::same(6))
        .inner_margin(Margin::same(8))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(text).monospace().size(12.0));
        });
}

/// Applies `re_ui`'s style for tests that draw widgets without the viewer.
#[cfg(test)]
pub fn style_for_tests(ctx: &egui::Context) {
    rerun::external::re_ui::apply_style_and_install_loaders(ctx);
}

/// Compares a render with its stored snapshot. CI draws with the lavapipe software rasterizer,
/// which anti-aliases differently from the GPU the snapshots come from, so there a difference is
/// reported and the test only proves the screen renders.
#[cfg(test)]
pub fn compare<S>(
    harness: &mut egui_kittest::Harness<'_, S>,
    name: &str,
    options: &egui_kittest::SnapshotOptions,
) {
    let result = harness.try_snapshot_options(name, options);
    if std::env::var_os("CI").is_some() {
        if let Err(e) = result {
            eprintln!("snapshot {name} differs on this renderer: {e}");
        }
    } else {
        result.unwrap();
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use egui_kittest::{Harness, SnapshotOptions};

    pub(crate) fn sample() -> Chat {
        let mut chat = Chat::default();
        chat.push_user("What do you see right now?".to_owned());
        chat.apply(&Event::TurnStarted { turn: 1 });
        chat.apply(&Event::ToolStarted {
            turn: 1,
            call: 1,
            tool: "look".to_owned(),
            args: serde_json::json!({}),
        });
        chat.apply(&Event::ToolFinished {
            turn: 1,
            call: 1,
            tool: "look".to_owned(),
            status: "succeeded",
            message: "2 marks: 1 shelf, 2 cardboard box".to_owned(),
            ms: 427,
        });
        chat.apply(&Event::Reply {
            turn: 1,
            text: "I see a **shelf** (mark 1) and a **cardboard box** (mark 2).".to_owned(),
            model: "qwen3.5-9b-local".to_owned(),
        });
        chat.apply(&Event::ApprovalRequested {
            id: 7,
            tool: "navigate".to_owned(),
            args: serde_json::json!({"place": "kitchen"}),
            reason: "moves the base".to_owned(),
        });
        chat.apply(&Event::TurnFinished { turn: 1 });
        chat
    }

    #[test]
    fn events_fold_into_items() {
        let chat = sample();
        assert_eq!(chat.items.len(), 4);
        assert!(
            matches!(&chat.items[1], Item::Tool(t) if t.status == Some("succeeded") && t.ms == 427)
        );
        assert_eq!(chat.pending().count(), 1);
        assert_eq!(chat.model.as_deref(), Some("qwen3.5-9b-local"));
        assert!(chat.turn.is_none());
    }

    #[test]
    fn a_resolved_approval_is_no_longer_pending() {
        let mut chat = sample();
        chat.apply(&Event::ApprovalResolved {
            id: 7,
            approved: false,
        });
        assert_eq!(chat.pending().count(), 0);
    }

    /// A harness drawing `chat` on the panel background, cropped to what it draws.
    fn render(chat: Chat, name: &str) {
        let mut harness = Harness::builder()
            .wgpu()
            .with_size(egui::vec2(440.0, 800.0))
            .build_ui(move |ui| {
                Frame::new()
                    .fill(ui.tokens().panel_bg_color)
                    .inner_margin(Margin::same(16))
                    .show(ui, |ui| {
                        chat.show(ui, Duration::from_mins(1), &mut Vec::new());
                    });
            });
        style_for_tests(&harness.ctx);
        harness.run();
        harness.fit_contents();
        compare(&mut harness, name, &SnapshotOptions::new());
    }

    #[test]
    fn snapshot_chat() {
        render(sample(), "chat");
    }

    #[test]
    fn snapshot_empty_state() {
        render(Chat::default(), "chat_empty");
    }

    #[test]
    fn snapshot_mission() {
        let mut chat = Chat::default();
        chat.apply(&Event::User {
            turn: 1,
            text: "Put the red mug in the basket".to_owned(),
        });
        let step = |id: &str, summary: &str| PlannedStep {
            id: id.to_owned(),
            skill: summary.split('(').next().unwrap_or_default().to_owned(),
            summary: summary.to_owned(),
            timeout_s: 300.0,
        };
        chat.apply(&Event::MissionPlanned {
            hash: "a91f3c2e77d04b1e".to_owned(),
            intent: "put the red mug in the basket".to_owned(),
            steps: vec![
                step("s1", "GoToPlace(place=kitchen)"),
                step("s2", "PickObject(object_id=O17, phrase=red mug, arm=right)"),
                step(
                    "s3",
                    "PlaceInto(container_id=O31, phrase=basket, arm=right)",
                ),
            ],
            worst_case_s: 1140.0,
        });
        chat.apply(&Event::ApprovalRequested {
            id: 3,
            tool: "run_mission".to_owned(),
            args: serde_json::json!({"hash": "a91f3c2e"}),
            reason: "`run_mission` acts on the robot".to_owned(),
        });
        chat.apply(&Event::ApprovalResolved {
            id: 3,
            approved: true,
        });
        chat.apply(&Event::MissionStarted {
            id: "m1".to_owned(),
            hash: "a91f3c2e77d04b1e".to_owned(),
        });
        for (step, node, status) in [
            ("s1", "", "running"),
            ("s1", "", "success"),
            ("s2", "", "running"),
            ("s2", "Pick", "running"),
            ("s2", "", "failure"),
        ] {
            chat.apply(&Event::MissionProgress {
                id: "m1".to_owned(),
                step: step.to_owned(),
                node: node.to_owned(),
                status: status.to_owned(),
                elapsed_s: 0.0,
            });
        }
        chat.apply(&Event::MissionFinished {
            id: "m1".to_owned(),
            outcome: "failure".to_owned(),
            failed_step: "s2".to_owned(),
            reason: "the grasp slipped".to_owned(),
            elapsed_s: 94.0,
        });
        chat.apply(&Event::Report {
            turn: 2,
            text: "Mission m1 ended: failure after 94 s. Failed at s2: the grasp slipped."
                .to_owned(),
        });
        render(chat, "chat_mission");
    }

    #[test]
    fn snapshot_error_and_image() {
        let mut chat = Chat::default();
        chat.push_user("Look again".to_owned());
        let mut jpeg = Vec::new();
        let img = image::RgbImage::from_pixel(96, 54, image::Rgb([60, 90, 140]));
        image::codecs::jpeg::JpegEncoder::new(&mut jpeg)
            .encode_image(&img)
            .unwrap();
        chat.apply(&Event::Snapshot {
            id: "S3".to_owned(),
            jpeg: Arc::new(jpeg),
            width: 96,
            height: 54,
        });
        chat.apply(&Event::Notice {
            text: "The robot is running mission m7, at s2_GoToPlace, started before this session. \
                   It carries on unwatched until it ends; Stop ends it now."
                .to_owned(),
        });
        chat.apply(&Event::Error {
            turn: 2,
            text: "every model failed: qwen3.5-9b-local: connection refused, and \
                   qwen3.8-27b-or: the free tier's daily quota is spent"
                .to_owned(),
        });
        render(chat, "chat_error_image");
    }
}
