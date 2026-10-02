//! The conversation as the operator sees it: items built from session events, and how each one
//! is drawn. Colours and sizes come from `re_ui`'s tokens so chat and viewer read as one app.

use std::cell::{Cell, OnceCell, RefCell};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use std::fmt::Write as _;

use nervros_core::mission::plan::PlannedStep;
use nervros_core::mission::preview::PreviewStep;
use nervros_core::mission::sanity::Concern;
use nervros_core::session::{Command, Event, Unanswered};
use nervros_core::tools::Status;
use rerun::external::egui::{self, Align, Color32, CornerRadius, Frame, Layout, Margin, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _, icons};
use serde_json::Value;

use crate::plan_edit::{self, EditStep};

/// Text never runs wider than this, for readable line lengths.
const MAX_TEXT_WIDTH: f32 = 720.0;
const SUGGESTIONS: [&str; 3] = [
    "What do you see?",
    "Where are you?",
    "Which places can you go to?",
];

/// What a click in the chat asks the app to do.
#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    /// Send a command to the session.
    Send(Command),
    /// Send text as if typed.
    Say(String),
    /// Put text in the composer.
    Prefill(String),
    /// Show the 3D view as it was at this time.
    ShowAt(SystemTime),
}

/// One tool call, from its start to its end.
#[derive(Debug, Clone)]
pub struct ToolCall {
    turn: u64,
    call: u64,
    tool: String,
    args: Value,
    status: Option<Status>,
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
    /// Approving can cover the rest of the session.
    can_allow: bool,
    asked: Instant,
    answer: Option<bool>,
    /// An edit is being checked.
    checking: bool,
    /// Why the last edit failed its checks.
    problem: Option<String>,
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

    /// Why it asks, such as what a plan runs.
    pub fn reason(&self) -> &str {
        &self.reason
    }
}

/// One entry in the conversation.
#[derive(Clone)]
pub enum Item {
    /// The operator's message.
    User {
        /// What they said.
        text: String,
        /// It started a turn of its own, so the conversation keeps it as the operator's: a
        /// message said while the agent worked goes into a tool result instead.
        turn: bool,
    },
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
        /// Decoded once, given up when uploaded on first draw: the texture holds it then.
        pixels: RefCell<Option<Arc<egui::ColorImage>>>,
        texture: OnceCell<egui::TextureHandle>,
        /// What each numbered mark is, mark 1 first.
        marks: Vec<String>,
        /// When it arrived, for showing the 3D view as it was then.
        at: SystemTime,
    },
    /// An approval request.
    Approval(Approval),
    /// Something the operator should know.
    Notice(String),
    /// A failure.
    Error {
        /// What went wrong.
        text: String,
        /// The message whose turn failed, to send again.
        retry: Option<String>,
    },
    /// A report from the robot, which the agent then answers.
    Report(String),
    /// A plan, and the mission that runs it.
    Plan(PlanCard),
    /// A request the last session ended waiting on, and whether it was taken up or put aside.
    Unanswered(Unanswered, Cell<bool>),
}

/// A plan `run_mission` checked, updated as its mission runs.
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
    /// Where each step would take the robot, once the executor has said.
    preview: Vec<PreviewStep>,
    /// Ways it may not do what the operator asked.
    concerns: Vec<Concern>,
    /// An edit replaced it before it ran.
    replaced: bool,
    /// Each tree node seen so far, by path, with its latest status, in the order they started.
    nodes: Vec<(String, String)>,
}

/// What a session's model calls cost, summed.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Spent {
    pub calls: u64,
    pub input_tokens: u64,
    pub cached_tokens: u64,
    pub output_tokens: u64,
    pub ms: u64,
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
    /// Tokens the latest request took of the model's window, when the window is known.
    pub context: Option<(u64, u64)>,
    /// What the session's model calls cost.
    pub spent: Spent,
    /// The reply as it streams in, until it arrives whole or a tool call takes over.
    pub draft: Option<String>,
    /// What the operator said to start each turn, for the Retry of a turn that failed.
    asked: HashMap<u64, String>,
    /// Each item's height when last drawn: one scrolled out of view takes its space undrawn.
    heights: RefCell<Vec<f32>>,
}

impl Item {
    /// A failure with nothing to retry.
    #[must_use]
    pub fn error(text: impl Into<String>) -> Self {
        Self::Error {
            text: text.into(),
            retry: None,
        }
    }
}

impl Chat {
    /// Adds the operator's message. A message said mid-turn that the session sends again as a
    /// turn of its own is the same bubble, now a turn's.
    fn push_user(&mut self, text: String, turn: bool) {
        if turn
            && let Some(Item::User { text: said, turn }) = self.items.last_mut()
            && !*turn
            && *said == text
        {
            *turn = true;
            return;
        }
        self.items.push(Item::User { text, turn });
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
        self.settle_draft(event);
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
            Event::Snapshot {
                id,
                jpeg,
                width,
                height,
                marks,
            } => match decode(jpeg) {
                Some(pixels) => self.items.push(Item::Image {
                    id: id.clone(),
                    pixels: RefCell::new(Some(Arc::new(pixels))),
                    texture: OnceCell::new(),
                    marks: marks.clone(),
                    at: SystemTime::now(),
                }),
                None => self.items.push(Item::Notice(format!(
                    "Snapshot {id} ({width}×{height}) could not be decoded"
                ))),
            },
            Event::Armed { armed } => self.items.push(Item::Notice(if *armed {
                "Armed: the agent may now act".to_owned()
            } else {
                "Observe only: the agent cannot act".to_owned()
            })),
            Event::Halted { reason } => self.items.push(Item::Notice(format!("Stopped: {reason}"))),
            Event::Stopped { ok: true, detail } => {
                self.items
                    .push(Item::Notice(format!("The robot stopped: {detail}")));
            }
            Event::Stopped { ok: false, detail } => {
                self.items.push(Item::error(format!(
                    "The robot did not confirm the stop: {detail}"
                )));
            }
            Event::Notice { text } => self.items.push(Item::Notice(text.clone())),
            Event::Error { turn, text } => self.items.push(Item::Error {
                text: text.clone(),
                retry: self.asked.get(turn).cloned(),
            }),
            // Said while the agent works: the model reads it with its next step.
            Event::User { turn, text } => {
                self.asked.insert(*turn, text.clone());
                self.push_user(text.clone(), true);
            }
            Event::Steer { text, .. } => self.push_user(text.clone(), false),
            Event::Report { text, .. } => self.items.push(Item::Report(text.clone())),
            Event::ToolStarted { .. }
            | Event::ToolFinished { .. }
            | Event::ApprovalRequested { .. }
            | Event::ApprovalEdited { .. }
            | Event::EditRejected { .. }
            | Event::ApprovalResolved { .. } => self.apply_tool(event),
            Event::MissionPlanned { .. }
            | Event::MissionPreview { .. }
            | Event::MissionStarted { .. }
            | Event::MissionProgress { .. }
            | Event::MissionFinished { .. } => self.apply_mission(event),
            // The viewer draws it; the tool's card already says what.
            Event::Plot { .. } => {}
            Event::ModelCall {
                input_tokens,
                cached_tokens,
                output_tokens,
                ms,
                ..
            } => {
                self.spent.calls += 1;
                self.spent.input_tokens += input_tokens;
                self.spent.cached_tokens += cached_tokens;
                self.spent.output_tokens += output_tokens;
                self.spent.ms += ms;
            }
            Event::Context { .. }
            | Event::Restored { .. }
            | Event::Compacted { .. }
            | Event::ReplyDelta { .. } => self.apply_conversation(event),
        }
    }

    /// Tool calls as they start and end, and the approvals they ask for.
    fn apply_tool(&mut self, event: &Event) {
        match event {
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
                    t.status = Some(*status);
                    t.message.clone_from(message);
                    t.ms = *ms;
                }
            }
            Event::ApprovalRequested {
                id,
                tool,
                args,
                reason,
                can_allow,
            } => self.items.push(Item::Approval(Approval {
                id: *id,
                tool: tool.clone(),
                args: args.clone(),
                reason: reason.clone(),
                can_allow: *can_allow,
                asked: Instant::now(),
                answer: None,
                checking: false,
                problem: None,
            })),
            Event::ApprovalEdited { id, args, reason } => self.edited(*id, args, reason),
            Event::EditRejected { id, message } => {
                if let Some(a) = self.approval_mut(*id) {
                    a.checking = false;
                    a.problem = Some(message.clone());
                    a.asked = Instant::now();
                }
            }
            Event::ApprovalResolved { id, approved } => {
                for item in &mut self.items {
                    if let Item::Approval(a) = item
                        && a.id == *id
                    {
                        a.answer = Some(*approved);
                    }
                }
            }
            _ => {}
        }
    }

    fn approval_mut(&mut self, id: u64) -> Option<&mut Approval> {
        self.items.iter_mut().rev().find_map(|i| match i {
            Item::Approval(a) if a.id == id => Some(a),
            _ => None,
        })
    }

    /// An edit passed: the approval now asks about the edited plan, so it moves below that
    /// plan's card, and the plan it replaced says so.
    fn edited(&mut self, id: u64, args: &Value, reason: &str) {
        let Some(at) = self
            .items
            .iter()
            .position(|i| matches!(i, Item::Approval(a) if a.id == id))
        else {
            return;
        };
        let Item::Approval(mut a) = self.items.remove(at) else {
            return;
        };
        let before = a.args["hash"].as_str().unwrap_or_default().to_owned();
        if let Some(p) = self.plan_mut(|p| !before.is_empty() && p.hash.starts_with(&before)) {
            p.replaced = true;
        }
        a.args = args.clone();
        reason.clone_into(&mut a.reason);
        a.asked = Instant::now();
        a.checking = false;
        a.problem = None;
        self.items.push(Item::Approval(a));
    }

    /// The operator sent an edit of approval `id` to be checked.
    pub fn edit_sent(&mut self, id: u64) {
        if let Some(a) = self.approval_mut(id) {
            a.checking = true;
            a.problem = None;
        }
    }

    /// A streamed reply ends whole, or gives way to a tool call.
    fn settle_draft(&mut self, event: &Event) {
        if matches!(
            event,
            Event::Reply { .. }
                | Event::ToolStarted { .. }
                | Event::TurnFinished { .. }
                | Event::Error { .. }
        ) {
            self.draft = None;
        }
    }

    /// The reply as it streams, how full the model's context is, and the conversation condensed
    /// or taken up again.
    fn apply_conversation(&mut self, event: &Event) {
        match event {
            Event::ReplyDelta { text, .. } => self.draft.get_or_insert_default().push_str(text),
            Event::Context { used, window } => self.context = Some((*used, *window)),
            Event::Restored { exchanges } => {
                self.items.push(Item::Notice(
                    "Carrying on an earlier conversation".to_owned(),
                ));
                for (operator, text) in exchanges {
                    if *operator {
                        self.push_user(text.clone(), true);
                    } else {
                        self.items.push(Item::Reply {
                            text: text.clone(),
                            model: "earlier".to_owned(),
                        });
                    }
                }
            }
            Event::Compacted { before, after, .. } => self.items.push(Item::Notice(format!(
                "Condensed the conversation to stay inside the model's context: {} to {} tokens",
                thousands(*before),
                thousands(*after)
            ))),
            _ => {}
        }
    }

    fn apply_mission(&mut self, event: &Event) {
        match event {
            Event::MissionPlanned {
                hash,
                intent,
                steps,
                worst_case_s,
                concerns,
            } => self.items.push(Item::Plan(PlanCard {
                hash: hash.clone(),
                intent: intent.clone(),
                steps: steps.clone(),
                worst_case_s: *worst_case_s,
                mission: None,
                started: None,
                progress: HashMap::new(),
                finished: None,
                preview: Vec::new(),
                concerns: concerns.clone(),
                replaced: false,
                nodes: Vec::new(),
            })),
            Event::MissionPreview { hash, steps } => {
                if let Some(p) = self.plan_mut(|p| p.hash == *hash) {
                    p.preview.clone_from(steps);
                }
            }
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
                path,
                status,
                ..
            } => {
                if let Some(p) = self.plan_mut(|p| p.mission.as_ref() == Some(id)) {
                    if !path.is_empty() {
                        match p.nodes.iter_mut().find(|(n, _)| n == path) {
                            Some((_, s)) => s.clone_from(status),
                            None => p.nodes.push((path.clone(), status.clone())),
                        }
                    }
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

    /// What a mission was for, by its id.
    pub fn intent_of_mission(&self, id: &str) -> Option<&str> {
        self.items.iter().rev().find_map(|i| match i {
            Item::Plan(p) if p.mission.as_deref() == Some(id) => Some(p.intent.as_str()),
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
        // "Condense up to here" counts what the conversation keeps as the operator's.
        let mut later = self
            .items
            .iter()
            .filter(|i| matches!(i, Item::User { turn: true, .. }))
            .count();
        let mut heights = self.heights.borrow_mut();
        heights.resize(self.items.len(), 0.0);
        let shown = ui.clip_rect();
        for (item, height) in self.items.iter().zip(heights.iter_mut()) {
            // A long conversation lays out only what is on screen; the rest keeps its place.
            let top = ui.cursor().top();
            if *height > 0.0 && (top + *height < shown.top() || top > shown.bottom()) {
                if matches!(item, Item::User { turn: true, .. }) {
                    later -= 1;
                }
                ui.allocate_space(egui::vec2(width, *height));
                continue;
            }
            *height = ui
                .scope(|ui| {
                    ui.set_max_width(width);
                    match item {
                        Item::User { text, turn } => {
                            let keep = turn.then(|| {
                                later -= 1;
                                later
                            });
                            user_bubble(ui, text, keep, actions);
                        }
                        Item::Reply { text, model } => reply(ui, text, model),
                        Item::Tool(t) => tool_chip(ui, t),
                        Item::Image {
                            id,
                            pixels,
                            texture,
                            marks,
                            at,
                        } => {
                            let texture = texture.get_or_init(|| {
                                let options = egui::TextureOptions::LINEAR;
                                let image = pixels.borrow_mut().take().unwrap_or_else(|| {
                                    Arc::new(egui::ColorImage::filled([1, 1], Color32::GRAY))
                                });
                                ui.ctx().load_texture(id, image, options)
                            });
                            image_card(ui, id, texture, marks, *at, actions);
                        }
                        Item::Approval(a) => {
                            approval_card(ui, a, self.plan_for(a), approval_ttl, actions);
                        }
                        Item::Notice(text) => notice(ui, text),
                        Item::Error { text, retry } => {
                            error_card(ui, text, retry.as_deref(), actions);
                        }
                        Item::Report(text) => report(ui, text),
                        Item::Plan(p) => plan_card(ui, p, actions),
                        Item::Unanswered(u, done) => unanswered_card(ui, u, done, actions),
                    }
                })
                .response
                .rect
                .height();
        }
        drop(heights);
        if let Some(draft) = self.draft.as_deref().filter(|d| !d.trim().is_empty()) {
            reply(ui, draft, "…");
        }
        if let Some(since) = self.turn {
            ui.horizontal(|ui| {
                spinner(ui);
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

/// A token count as people read it: 812, 9.8k.
#[must_use]
pub fn thousands(n: u64) -> String {
    if n < 1000 {
        n.to_string()
    } else {
        #[expect(clippy::cast_precision_loss, reason = "shown to one decimal")]
        let k = n as f64 / 1000.0;
        format!("{k:.1}k")
    }
}

/// Running is blue, success green, failure red and waiting amber, everywhere in the app; a
/// call cut short by a stop is grey.
pub fn status_color(ui: &egui::Ui, status: Option<Status>) -> Color32 {
    let t = ui.tokens();
    match status {
        None => t.info_text_color,
        Some(Status::Succeeded | Status::Accepted) => t.success_text_color,
        Some(Status::Refused) => t.warn_fg_color,
        Some(Status::Stopped) => t.text_subdued,
        Some(Status::Failed) => t.error_fg_color,
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

/// The operator's message, with "condense up to here" on a right click; `later` is how many of
/// their messages came after it.
/// A message of the operator's; `later` is how many turns of theirs came after it, for a message
/// that started one.
fn user_bubble(ui: &mut egui::Ui, text: &str, later: Option<usize>, actions: &mut Vec<Action>) {
    ui.with_layout(Layout::right_to_left(Align::Min), |ui| {
        let bubble = Frame::new()
            .fill(ui.tokens().selection_bg_fill)
            .corner_radius(CornerRadius::same(12))
            .inner_margin(Margin::symmetric(12, 8))
            .show(ui, |ui| {
                ui.set_max_width(ui.available_width() * 0.8);
                ui.label(RichText::new(text).color(ui.tokens().text_strong));
            })
            .response
            .interact(egui::Sense::click());
        let Some(later) = later else {
            return;
        };
        bubble.context_menu(|ui| {
            if ui
                .button("Condense up to here")
                .on_hover_text(
                    "Summarise the conversation up to this message; what came after stays as it is",
                )
                .clicked()
            {
                actions.push(Action::Send(Command::CompactUpTo { keep: later }));
                ui.close();
            }
        });
    });
}

fn reply(ui: &mut egui::Ui, text: &str, model: &str) {
    // An image in a reply would be fetched from wherever it points, a URL prompt injection can
    // write: shown as a link, it goes nowhere unless clicked.
    ui.markdown_ui(&text.replace("![", "["));
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
                    spinner(ui);
                } else {
                    let icon = match t.status {
                        Some(Status::Succeeded | Status::Accepted) => &icons::SUCCESS,
                        Some(Status::Refused) => &icons::WARNING,
                        Some(Status::Stopped) => &icons::PAUSE,
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

/// A marked image: its marks as a legend to ask about one by one, when it was taken, and the
/// 3D view as it was then.
fn image_card(
    ui: &mut egui::Ui,
    id: &str,
    texture: &egui::TextureHandle,
    marks: &[String],
    at: SystemTime,
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
        if !marks.is_empty() {
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().item_spacing.x = 4.0;
                for (i, label) in marks.iter().enumerate() {
                    let n = i + 1;
                    if ui
                        .add(ReButton::new(format!("{n} {label}")).small().secondary())
                        .on_hover_text(format!("Ask about mark {n}"))
                        .clicked()
                    {
                        actions.push(Action::Prefill(format!(
                            "In snapshot {id}, mark {n} ({label}): "
                        )));
                    }
                }
            });
        }
        ui.horizontal(|ui| {
            let taken = at.duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs());
            let caption = format!("Snapshot {id} · {} · {w}×{h}", crate::sessions::ago(taken));
            let caption = RichText::new(caption);
            ui.label(caption.small().color(ui.tokens().text_subdued));
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui
                    .add(ReButton::new("Ask about it").small().primary())
                    .clicked()
                {
                    actions.push(Action::Prefill(format!("In snapshot {id}, ")));
                }
                if ui
                    .add(ReButton::new("Show in 3D").small().secondary())
                    .on_hover_text("Pause the 3D view at the moment this was taken")
                    .clicked()
                {
                    actions.push(Action::ShowAt(at));
                }
            });
        });
    });
}

/// A JPEG as egui pixels, or `None` if it does not decode.
fn decode(jpeg: &[u8]) -> Option<egui::ColorImage> {
    // A JPEG has no alpha: straight to RGB, one pass fewer.
    let rgb = image::load_from_memory_with_format(jpeg, image::ImageFormat::Jpeg)
        .ok()?
        .into_rgb8();
    let size = [rgb.width(), rgb.height()].map(|v| usize::try_from(v).unwrap_or(0));
    Some(egui::ColorImage::from_rgb(size, rgb.as_raw()))
}

/// A spinner that moves eight times a second. egui's own redraws the whole window, viewer
/// included, at the screen's rate for as long as it shows.
pub fn spinner(ui: &mut egui::Ui) {
    let size = ui.spacing().interact_size.y * 0.6;
    let (rect, _) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::hover());
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "a step of eight, from a time that is never negative"
    )]
    let lit = (ui.input(|i| i.time) * 8.0).rem_euclid(8.0) as u8;
    let colour = ui.tokens().info_text_color;
    for k in 0u8..8 {
        let angle = f32::from(k) * std::f32::consts::TAU / 8.0;
        let at = rect.center() + size * 0.38 * egui::vec2(angle.cos(), angle.sin());
        let alpha = if k == lit { 1.0 } else { 0.3 };
        ui.painter()
            .circle_filled(at, size * 0.1, colour.gamma_multiply(alpha));
    }
    ui.ctx().request_repaint_after(Duration::from_millis(125));
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
    // An edit in progress lives in egui's memory under the plan it started from: once an edit
    // passes, the approval asks about a new plan and the editor closes by itself.
    let key = plan.map(|p| egui::Id::new(("plan_edit", a.id, p.hash.as_str())));
    let mut editing: Option<Vec<EditStep>> = key
        .filter(|_| a.answer.is_none())
        .and_then(|k| ui.data(|d| d.get_temp(k)));
    card(ui, stroke).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.small_icon(&icons::WARNING, Some(t.warn_fg_color));
            let title = match plan {
                Some(p) => format!("Run \"{}\"?", p.intent),
                None => format!("Approve {}?", a.tool),
            };
            ui.label(RichText::new(title).strong());
        });
        match (plan, editing.as_mut(), key) {
            (Some(_), Some(steps), Some(key)) => {
                ui.label(
                    RichText::new(
                        "Change the steps; the plan is checked again before you approve it",
                    )
                    .small()
                    .color(t.text_subdued),
                );
                plan_edit::editor(ui, key, steps);
            }
            (Some(p), ..) => {
                // The plan's card above lists the steps and follows them; this only asks.
                let short = p.hash.get(..8).unwrap_or(&p.hash);
                let steps = match p.steps.len() {
                    1 => "1 step".to_owned(),
                    n => format!("{n} steps"),
                };
                ui.label(
                    RichText::new(format!(
                        "plan {short} · {steps} · at most {}",
                        minutes(p.worst_case_s)
                    ))
                    .small()
                    .color(t.text_subdued),
                );
            }
            (None, ..) => {
                ui.label(&a.reason);
                code(ui, &pretty(&a.args));
            }
        }
        if let Some(problem) = a.problem.as_deref().filter(|_| a.answer.is_none()) {
            ui.add(egui::Label::new(RichText::new(problem).small().color(t.error_fg_color)).wrap());
        }
        ui.horizontal(|ui| match a.answer {
            Some(true) => {
                ui.label(RichText::new("Approved").color(t.success_text_color));
            }
            Some(false) => {
                ui.label(RichText::new("Denied").color(t.text_subdued));
            }
            None => {
                if a.checking {
                    spinner(ui);
                    ui.label(RichText::new("Checking the edit…").color(t.text_subdued));
                } else {
                    let left = time_left(ttl.saturating_sub(a.asked.elapsed()));
                    ui.label(RichText::new(left).color(t.text_subdued));
                }
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    approval_buttons(ui, a, plan, &mut editing, actions);
                });
                ui.ctx().request_repaint_after(Duration::from_secs(1));
            }
        });
    });
    if let Some(key) = key {
        ui.data_mut(|d| match editing {
            Some(steps) => {
                d.insert_temp(key, steps);
            }
            None => d.remove::<Vec<EditStep>>(key),
        });
    }
}

/// A request the last session ended waiting on: asked again only if the operator wants it, and
/// checked again then, since the robot and its world have moved on.
fn unanswered_card(
    ui: &mut egui::Ui,
    u: &Unanswered,
    done: &Cell<bool>,
    actions: &mut Vec<Action>,
) {
    let t = ui.tokens();
    card(ui, t.widget_noninteractive_bg_stroke).show(ui, |ui| {
        ui.label(RichText::new("Waiting for your approval when the app last closed").strong());
        ui.label(&u.reason);
        // A plan's reason names it already; anything else shows what it would send.
        if u.args.get("steps").is_none() {
            code(ui, &pretty(&u.args));
        }
        ui.horizontal(|ui| {
            if done.get() {
                ui.label(RichText::new("Done").color(t.text_subdued));
                return;
            }
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui
                    .add(ReButton::new("Ask again").primary())
                    .on_hover_text("Check it again now and ask for your approval")
                    .clicked()
                {
                    actions.push(Action::Send(Command::Run {
                        tool: u.tool.clone(),
                        args: u.args.clone(),
                    }));
                    done.set(true);
                }
                if ui.add(ReButton::new("Dismiss").secondary()).clicked() {
                    done.set(true);
                }
            });
        });
    });
}

/// How long an approval has left: whole minutes while there are more than 90 s, as a count of
/// seconds would only distract, then seconds.
fn time_left(left: Duration) -> String {
    let secs = left.as_secs_f32().ceil();
    if secs > 90.0 {
        format!("{} min left", (secs / 60.0).ceil())
    } else {
        format!("{secs} s left")
    }
}

/// Approve, deny, edit or apply the suggested fixes; while editing, check or cancel. Nothing
/// can be approved while an edit is checked: it would approve a plan about to change.
fn approval_buttons(
    ui: &mut egui::Ui,
    a: &Approval,
    plan: Option<&PlanCard>,
    editing: &mut Option<Vec<EditStep>>,
    actions: &mut Vec<Action>,
) {
    let idle = !a.checking;
    let edit = |args| Action::Send(Command::Edit { id: a.id, args });
    if let (Some(p), Some(steps)) = (plan, editing.as_ref()) {
        if ui
            .add_enabled(idle && !steps.is_empty(), ReButton::new("Check").primary())
            .clicked()
        {
            actions.push(edit(plan_edit::plan_args(&p.intent, steps)));
        }
        if ui.add(ReButton::new("Cancel").secondary()).clicked() {
            *editing = None;
        }
        return;
    }
    if ui
        .add_enabled(idle, ReButton::new("Approve").primary())
        .clicked()
    {
        actions.push(Action::Send(Command::Approve(a.id)));
    }
    if ui.add(ReButton::new("Deny").secondary()).clicked() {
        actions.push(Action::Send(Command::Deny(a.id)));
    }
    let Some(p) = plan else {
        if a.can_allow
            && ui
                .add_enabled(idle, ReButton::new("Allow for session").secondary())
                .on_hover_text(
                    "Approve, and let this tool run without asking for the rest of the session; \
                 what moves the robot still asks",
                )
                .clicked()
        {
            actions.push(Action::Send(Command::AllowForSession(a.id)));
        }
        return;
    };
    if ui
        .add_enabled(idle, ReButton::new("Edit").secondary())
        .on_hover_text("Change, move or drop steps before approving")
        .clicked()
    {
        *editing = Some(plan_edit::editable(&p.steps));
    }
    if let Some(fixed) = plan_edit::fixed(&p.steps, &p.concerns)
        && ui
            .add_enabled(idle, ReButton::new("Apply fixes").secondary())
            .on_hover_text(
                "Change the plan as its checks suggest; it is checked again before you approve it",
            )
            .clicked()
    {
        actions.push(edit(plan_edit::plan_args(&p.intent, &fixed)));
    }
}

/// The plan's steps with each one's state, how it has gone before, and what the preview warns of.
fn steps_table(ui: &mut egui::Ui, p: &PlanCard) {
    let t = ui.tokens();
    egui::Grid::new(("plan_steps", &p.hash))
        .num_columns(3)
        .spacing([8.0, 4.0])
        .show(ui, |ui| {
            for s in &p.steps {
                let (status, node) = p.progress.get(&s.id).cloned().unwrap_or_default();
                let failed = p.finished.as_ref().is_some_and(|f| f.1 == s.id);
                let (color, mark) = if failed {
                    (t.error_fg_color, "✗")
                } else {
                    node_look(ui, &status)
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
                let preview_note = p
                    .preview
                    .iter()
                    .find(|v| v.id == s.id)
                    .map(|v| v.note.as_str())
                    .filter(|n| !n.is_empty());
                ui.horizontal(|ui| {
                    let summary = ui.label(RichText::new(text).monospace().size(12.0));
                    if let Some(note) = preview_note {
                        summary.on_hover_text(note);
                    }
                    track_label(ui, s);
                });
                ui.end_row();
                // Only a warning earns a line of its own; the rest is in the hover.
                let warnings = preview_note
                    .filter(|n| n.starts_with("no path"))
                    .into_iter()
                    .chain(
                        p.concerns
                            .iter()
                            .filter(|c| c.step == s.id)
                            .map(|c| c.message.as_str()),
                    );
                for warning in warnings {
                    ui.label("");
                    ui.label("");
                    let text = RichText::new(warning).small().color(t.warn_fg_color);
                    ui.add(egui::Label::new(text).wrap());
                    ui.end_row();
                }
            }
        });
}

/// A step's or node's status as a coloured mark.
fn node_look(ui: &egui::Ui, status: &str) -> (Color32, &'static str) {
    let t = ui.tokens();
    match status {
        "failure" => (t.error_fg_color, "✗"),
        "success" => (t.success_text_color, "✓"),
        "running" => (t.info_text_color, "●"),
        "skipped" => (t.text_subdued, "–"),
        _ => (t.text_subdued, "○"),
    }
}

/// The mission's behaviour tree as it has run so far, as the executor reports its nodes: each
/// under the subtrees it is in, marked with its latest status.
pub fn tree_panel(ui: &mut egui::Ui, p: &PlanCard) {
    if p.nodes.is_empty() {
        return;
    }
    ui.add_space(8.0);
    egui::CollapsingHeader::new(RichText::new("Behaviour tree").strong())
        .id_salt(("tree", &p.hash))
        .default_open(true)
        .show(ui, |ui| {
            for (path, status) in &p.nodes {
                let depth = u16::try_from(path.matches('/').count()).unwrap_or(u16::MAX);
                let name = path.rsplit('/').next().unwrap_or(path);
                // BehaviorTree.CPP names an unnamed node by its type and uid, as "Sequence::3".
                let name = name.split_once("::").map_or(name, |(n, _)| n);
                let (colour, mark) = node_look(ui, status);
                ui.horizontal(|ui| {
                    ui.add_space(14.0 * f32::from(depth));
                    ui.label(RichText::new(mark).color(colour));
                    ui.label(RichText::new(name).monospace().size(12.0))
                        .on_hover_text(format!("{path}: {status}"));
                });
            }
        });
}

/// How a step has gone before, small and quiet, with the details in its hover.
fn track_label(ui: &mut egui::Ui, s: &PlannedStep) {
    let Some(track) = &s.track else {
        return;
    };
    let t = ui.tokens();
    let mut text = format!("{} of {}", track.succeeded, track.runs);
    if let Some(typical) = track.typical_s {
        let _ = write!(text, " · {}", minutes(typical));
    }
    // Under three in four is worth a second look before approving.
    let color = if track.succeeded * 4 < track.runs * 3 {
        t.warn_fg_color
    } else {
        t.text_subdued
    };
    let mut hover = format!(
        "{} succeeded {} of its last {} runs",
        s.summary, track.succeeded, track.runs
    );
    if let Some(why) = &track.last_failure {
        let _ = write!(hover, "; it last failed because {why}");
    }
    ui.label(RichText::new(text).small().color(color))
        .on_hover_text(hover);
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
        steps_table(ui, p);
        for c in p.concerns.iter().filter(|c| c.step.is_empty()) {
            ui.horizontal(|ui| {
                ui.small_icon(&icons::WARNING, Some(t.warn_fg_color));
                ui.add(
                    egui::Label::new(RichText::new(&c.message).small().color(t.warn_fg_color))
                        .wrap(),
                );
            });
        }
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
                (None, None) if p.replaced => {
                    RichText::new("Replaced by an edited plan").color(t.text_subdued)
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
                ui.label(
                    RichText::new(report_text(text))
                        .small()
                        .color(ui.tokens().text_subdued),
                );
            });
        });
}

/// A robot report as the operator reads it: what happened, with mission ids cut to their first
/// eight characters; the line after it tells the model what to do next.
fn report_text(text: &str) -> String {
    let first = text.lines().next().unwrap_or(text);
    first
        .split(' ')
        .map(|w| {
            let id = w.len() == 36 && w.chars().all(|c| c.is_ascii_hexdigit() || c == '-');
            match w.get(..8) {
                Some(short) if id => short,
                _ => w,
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn notice(ui: &mut egui::Ui, text: &str) {
    ui.horizontal(|ui| {
        ui.small_icon(&icons::INFO, Some(ui.tokens().text_subdued));
        // The viewer's style extends labels, so a long notice would widen the panel instead.
        ui.add(egui::Label::new(RichText::new(text).color(ui.tokens().text_subdued)).wrap());
    });
}

fn error_card(ui: &mut egui::Ui, text: &str, retry: Option<&str>, actions: &mut Vec<Action>) {
    let t = ui.tokens();
    card(ui, t.error_fg_color).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.small_icon(&icons::ERROR, Some(t.error_fg_color));
            ui.add(egui::Label::new(RichText::new(text).color(t.error_fg_color)).wrap());
        });
        if let Some(again) = retry
            && ui.add(ReButton::new("Retry").small().secondary()).clicked()
        {
            actions.push(Action::Say(again.to_owned()));
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
    use egui_kittest::kittest::Queryable as _;
    use egui_kittest::{Harness, SnapshotOptions};
    use nervros_core::mission::plan::StepArg;
    use std::cell::RefCell;
    use std::rc::Rc;

    pub(crate) fn sample() -> Chat {
        let mut chat = Chat::default();
        chat.push_user("What do you see right now?".to_owned(), true);
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
            status: Status::Succeeded,
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
            can_allow: false,
        });
        chat.apply(&Event::TurnFinished { turn: 1 });
        chat
    }

    #[test]
    fn events_fold_into_items() {
        let chat = sample();
        assert_eq!(chat.items.len(), 4);
        assert!(
            matches!(&chat.items[1], Item::Tool(t) if t.status == Some(Status::Succeeded) && t.ms == 427)
        );
        assert_eq!(chat.pending().count(), 1);
        assert_eq!(chat.model.as_deref(), Some("qwen3.5-9b-local"));
        assert!(chat.turn.is_none());
    }

    #[test]
    fn a_long_wait_counts_in_minutes_and_the_last_ninety_seconds_in_seconds() {
        assert_eq!(time_left(Duration::from_mins(10)), "10 min left");
        assert_eq!(time_left(Duration::from_secs(91)), "2 min left");
        assert_eq!(time_left(Duration::from_millis(59_200)), "60 s left");
        assert_eq!(time_left(Duration::ZERO), "0 s left");
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

    #[test]
    fn a_report_shows_what_happened_with_short_ids() {
        let text = "Mission 01a0f95c-2c09-7662-b4ee-222fda068459 (turn left) ended: failure after \
                    2 s. Failed at s1.\nFind out why and say it in one sentence.";
        assert_eq!(
            report_text(text),
            "Mission 01a0f95c (turn left) ended: failure after 2 s. Failed at s1."
        );
    }

    #[test]
    fn condensing_up_to_a_message_keeps_the_ones_after_it() {
        let mut chat = Chat::default();
        for (turn, text) in [(1, "Where is the mug?"), (2, "Bring it here.")] {
            chat.apply(&Event::User {
                turn,
                text: text.to_owned(),
            });
        }
        let sent = Rc::new(RefCell::new(Vec::new()));
        let seen = Rc::clone(&sent);
        let mut harness = Harness::builder()
            .with_size(egui::vec2(440.0, 400.0))
            .build_ui(move |ui| {
                let mut actions = Vec::new();
                chat.show(ui, Duration::from_mins(10), &mut actions);
                seen.borrow_mut().extend(actions);
            });
        harness.run();
        harness.get_by_label("Where is the mug?").click_secondary();
        harness.run();
        harness.get_by_label("Condense up to here").click();
        harness.run();
        assert!(
            sent.borrow()
                .iter()
                .any(|a| matches!(a, Action::Send(Command::CompactUpTo { keep: 1 }))),
            "{:?}",
            sent.borrow().len()
        );
    }

    /// A harness drawing `chat` on the panel background, cropped to what it draws.
    fn render(chat: Chat, name: &str) {
        render_with(chat, name, |_| {});
    }

    /// As [`render`], with `setup` run on the context first, such as to open an editor.
    fn render_with(chat: Chat, name: &str, setup: impl FnOnce(&egui::Context)) {
        let mut harness = Harness::builder()
            .wgpu()
            .with_size(egui::vec2(440.0, 800.0))
            .build_ui(move |ui| {
                Frame::new()
                    .fill(ui.tokens().panel_bg_color)
                    .inner_margin(Margin::same(16))
                    .show(ui, |ui| {
                        // Long, so the countdown reads the same however slowly the test runs.
                        chat.show(ui, Duration::from_mins(10), &mut Vec::new());
                    });
            });
        style_for_tests(&harness.ctx);
        setup(&harness.ctx);
        harness.run();
        harness.fit_contents();
        compare(&mut harness, name, &SnapshotOptions::new());
    }

    /// A plan for "turn left, then walk half a metre" that turns right and walks a metre.
    fn doubtful_plan() -> Chat {
        let mut chat = Chat::default();
        chat.apply(&Event::User {
            turn: 1,
            text: "Turn left 90 degrees, then walk forward half a metre".to_owned(),
        });
        let step = |id: &str, skill: &str, args: &[(&str, &str)]| {
            let args: Vec<StepArg> = args
                .iter()
                .map(|(n, v)| StepArg {
                    name: (*n).to_owned(),
                    value: (*v).to_owned(),
                })
                .collect();
            let shown: Vec<String> = args
                .iter()
                .map(|a| format!("{}={}", a.name, a.value))
                .collect();
            PlannedStep {
                id: id.to_owned(),
                skill: skill.to_owned(),
                summary: format!("{skill}({})", shown.join(", ")),
                args,
                timeout_s: 30.0,
                ..PlannedStep::default()
            }
        };
        let concern = |step: &str, message: &str, fix: Option<(&str, &str)>| Concern {
            step: step.to_owned(),
            message: message.to_owned(),
            fix: fix.map(|(name, value)| StepArg {
                name: name.to_owned(),
                value: value.to_owned(),
            }),
        };
        chat.apply(&Event::MissionPlanned {
            hash: "5be0c1d9f2a4".to_owned(),
            intent: "turn left and walk half a metre".to_owned(),
            steps: vec![
                step("s1", "TurnInPlace", &[("degrees", "-90")]),
                step(
                    "s2",
                    "WalkStraight",
                    &[("direction", "forward"), ("distance_m", "1.0")],
                ),
            ],
            worst_case_s: 60.0,
            concerns: vec![
                concern(
                    "s1",
                    "the operator said turn left, but degrees=-90 turns the other way \
                     (positive degrees turn left)",
                    Some(("degrees", "90")),
                ),
                concern(
                    "s2",
                    "the operator asked for 0.5 m, but the walks add up to 1 m",
                    Some(("distance_m", "0.5")),
                ),
                concern("", "the walk may end close to the table", None),
            ],
        });
        chat.apply(&Event::ApprovalRequested {
            id: 9,
            tool: "run_mission".to_owned(),
            args: serde_json::json!({"hash": "5be0c1d9f2a4"}),
            reason: "runs it".to_owned(),
            can_allow: false,
        });
        chat
    }

    #[test]
    fn snapshot_plan_concerns() {
        render(doubtful_plan(), "chat_plan_concerns");
    }

    #[test]
    fn snapshot_plan_edit() {
        let mut chat = doubtful_plan();
        chat.edit_sent(9);
        chat.apply(&Event::EditRejected {
            id: 9,
            message: "s2: distance_m must be from 0.1 to 2.0".to_owned(),
        });
        let Some(Item::Plan(p)) = chat.items.get(1).cloned() else {
            panic!("the plan comes second");
        };
        render_with(chat, "chat_plan_edit", move |ctx| {
            let key = egui::Id::new(("plan_edit", 9_u64, p.hash.as_str()));
            ctx.data_mut(|d| d.insert_temp(key, plan_edit::editable(&p.steps)));
        });
    }

    #[test]
    fn snapshot_tree_panel() {
        let mut chat = doubtful_plan();
        chat.apply(&Event::MissionStarted {
            id: "m2".to_owned(),
            hash: "5be0c1d9f2a4".to_owned(),
        });
        for (step, node, path, status) in [
            ("s1", "", "s1_TurnInPlace::2", "running"),
            ("s1", "Turn", "s1_TurnInPlace::2/Turn", "running"),
            ("s1", "Turn", "s1_TurnInPlace::2/Turn", "success"),
            ("s1", "", "s1_TurnInPlace::2", "success"),
            ("s2", "", "s2_WalkStraight::5", "running"),
            (
                "s2",
                "Sequence",
                "s2_WalkStraight::5/Sequence::6",
                "running",
            ),
            (
                "s2",
                "CheckClear",
                "s2_WalkStraight::5/Sequence::6/CheckClear",
                "success",
            ),
            (
                "s2",
                "Walk",
                "s2_WalkStraight::5/Sequence::6/Walk",
                "running",
            ),
        ] {
            chat.apply(&Event::MissionProgress {
                id: "m2".to_owned(),
                step: step.to_owned(),
                node: node.to_owned(),
                path: path.to_owned(),
                status: status.to_owned(),
                elapsed_s: 3.0,
            });
        }
        let plan = chat.latest_plan().cloned().unwrap();
        let mut harness = Harness::builder()
            .wgpu()
            .with_size(egui::vec2(440.0, 600.0))
            .build_ui(move |ui| {
                Frame::new()
                    .fill(ui.tokens().panel_bg_color)
                    .inner_margin(Margin::same(16))
                    .show(ui, |ui| tree_panel(ui, &plan));
            });
        style_for_tests(&harness.ctx);
        harness.run();
        harness.fit_contents();
        compare(&mut harness, "tree_panel", &SnapshotOptions::new());
    }

    #[test]
    fn a_passed_edit_moves_the_approval_below_its_new_plan() {
        let mut chat = doubtful_plan();
        chat.edit_sent(9);
        assert!(matches!(chat.items.last(), Some(Item::Approval(a)) if a.checking));
        chat.apply(&Event::MissionPlanned {
            hash: "e7f3aa01c2d4".to_owned(),
            intent: "turn left and walk half a metre".to_owned(),
            steps: Vec::new(),
            worst_case_s: 60.0,
            concerns: Vec::new(),
        });
        chat.apply(&Event::ApprovalEdited {
            id: 9,
            args: serde_json::json!({"hash": "e7f3aa01c2d4"}),
            reason: "runs the edited plan".to_owned(),
        });
        let Some(Item::Approval(a)) = chat.items.last() else {
            panic!("the approval comes last");
        };
        assert!(!a.checking);
        assert_eq!(
            chat.plan_for(a).map(|p| p.hash.as_str()),
            Some("e7f3aa01c2d4")
        );
        assert!(matches!(&chat.items[1], Item::Plan(p) if p.replaced));
    }

    #[test]
    fn snapshot_chat() {
        render(sample(), "chat");
    }

    #[test]
    fn snapshot_unanswered_and_allow() {
        let mut chat = Chat::default();
        chat.items.push(Item::Unanswered(
            Unanswered {
                tool: "run_mission".to_owned(),
                args: serde_json::json!({"intent": "bring the red mug to the sofa", "steps": []}),
                reason: "runs \"bring the red mug to the sofa\": 4 step(s), at most 6 min"
                    .to_owned(),
            },
            Cell::new(false),
        ));
        chat.apply(&Event::ApprovalRequested {
            id: 3,
            tool: "set_parameter".to_owned(),
            args: serde_json::json!({"node": "/controller_server", "name": "max_vel_x", "value": 0.3}),
            reason: "`set_parameter` acts on the robot".to_owned(),
            can_allow: true,
        });
        render(chat, "chat_unanswered");
    }

    #[test]
    fn snapshot_draft_and_compaction() {
        let mut chat = Chat::default();
        chat.apply(&Event::Compacted {
            before: 9800,
            after: 2100,
            summarised: true,
        });
        chat.apply(&Event::User {
            turn: 3,
            text: "Where is the mug?".to_owned(),
        });
        for piece in ["The small white mug ", "is on the dining table, "] {
            chat.apply(&Event::ReplyDelta {
                turn: 3,
                text: piece.to_owned(),
            });
        }
        render(chat, "chat_draft");
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
            ..PlannedStep::default()
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
            concerns: Vec::new(),
        });
        chat.apply(&Event::ApprovalRequested {
            id: 3,
            tool: "run_mission".to_owned(),
            args: serde_json::json!({"hash": "a91f3c2e"}),
            reason: "`run_mission` acts on the robot".to_owned(),
            can_allow: false,
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
                path: String::new(),
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
    fn snapshot_plan_with_records_and_a_preview_warning() {
        use nervros_core::mission::ledger::Track;
        let mut chat = Chat::default();
        chat.apply(&Event::User {
            turn: 1,
            text: "Bring the small white mug to the tray".to_owned(),
        });
        let step = |id: &str, summary: &str, track: Option<Track>| PlannedStep {
            id: id.to_owned(),
            skill: summary.split('(').next().unwrap_or_default().to_owned(),
            summary: summary.to_owned(),
            timeout_s: 300.0,
            track,
            ..PlannedStep::default()
        };
        let track = |succeeded, runs, typical_s, last: Option<&str>| Track {
            runs,
            succeeded,
            typical_s,
            last_failure: last.map(str::to_owned),
        };
        chat.apply(&Event::MissionPlanned {
            hash: "7c01d2aa9e3b".to_owned(),
            intent: "bring the mug to the tray".to_owned(),
            steps: vec![
                step(
                    "s1",
                    "GoToPlace(place=dining_table_side)",
                    Some(track(8, 8, Some(21.0), None)),
                ),
                step(
                    "s2",
                    "PickObject(object_id=mug_4, arm=left)",
                    Some(track(
                        1,
                        3,
                        Some(44.0),
                        Some("nothing called mug_4 on /objects"),
                    )),
                ),
                step("s3", "GoToPlace(place=office_desk_tray)", None),
                step("s4", "PlaceInto(container_id=tray_1, arm=left)", None),
            ],
            worst_case_s: 1500.0,
            concerns: Vec::new(),
        });
        let preview = |id: &str, note: &str| PreviewStep {
            id: id.to_owned(),
            goal: None,
            path: Vec::new(),
            note: note.to_owned(),
        };
        chat.apply(&Event::MissionPreview {
            hash: "7c01d2aa9e3b".to_owned(),
            steps: vec![
                preview("s1", "walks there along Nav2's path"),
                preview("s2", "closes in on what it sees; ends within reach of it"),
                preview("s3", "no path: the goal is inside an obstacle"),
            ],
        });
        chat.apply(&Event::ApprovalRequested {
            id: 4,
            tool: "run_mission".to_owned(),
            args: serde_json::json!({"hash": "7c01d2aa9e3b"}),
            reason: "`run_mission` acts on the robot".to_owned(),
            can_allow: false,
        });
        render(chat, "chat_plan_records");
    }

    #[test]
    fn snapshot_error_and_image() {
        let mut chat = Chat::default();
        chat.apply(&Event::User {
            turn: 2,
            text: "Look again".to_owned(),
        });
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
            marks: vec!["red mug".to_owned(), "fruit bowl".to_owned()],
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
