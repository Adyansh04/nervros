//! The conversation as the operator sees it: items built from session events, and how each one
//! is drawn. Colours and sizes come from `re_ui`'s tokens so chat and viewer read as one app.

use std::cell::OnceCell;
use std::sync::Arc;
use std::time::{Duration, Instant};

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
    pub fn push_user(&mut self, text: String) {
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
        }
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
                    Item::Approval(a) => approval_card(ui, a, approval_ttl, actions),
                    Item::Notice(text) => notice(ui, text),
                    Item::Error(text) => error_card(ui, text, self.last_user.as_deref(), actions),
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

fn approval_card(ui: &mut egui::Ui, a: &Approval, ttl: Duration, actions: &mut Vec<Action>) {
    let t = ui.tokens();
    let stroke = if a.answer.is_none() {
        t.warn_fg_color
    } else {
        t.widget_noninteractive_bg_stroke
    };
    card(ui, stroke).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.small_icon(&icons::WARNING, Some(t.warn_fg_color));
            ui.label(RichText::new(format!("Approve {}?", a.tool)).strong());
        });
        ui.label(&a.reason);
        code(ui, &pretty(&a.args));
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

fn notice(ui: &mut egui::Ui, text: &str) {
    ui.horizontal(|ui| {
        ui.small_icon(&icons::INFO, Some(ui.tokens().text_subdued));
        ui.label(RichText::new(text).color(ui.tokens().text_subdued));
    });
}

fn error_card(ui: &mut egui::Ui, text: &str, last_user: Option<&str>, actions: &mut Vec<Action>) {
    let t = ui.tokens();
    card(ui, t.error_fg_color).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.small_icon(&icons::ERROR, Some(t.error_fg_color));
            ui.label(RichText::new(text).color(t.error_fg_color));
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

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use egui_kittest::Harness;

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
        harness.snapshot(name);
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
        chat.apply(&Event::Error {
            turn: 2,
            text: "every model failed: qwen3.5-9b-local: connection refused".to_owned(),
        });
        render(chat, "chat_error_image");
    }
}
