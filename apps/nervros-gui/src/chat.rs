//! The conversation as the operator sees it: items built from session events, and how each one
//! is drawn. Colours and sizes come from `re_ui`'s tokens so chat and viewer read as one app.

use std::cell::{Cell, OnceCell, RefCell};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use nervros_core::mission::Outcome;
use nervros_core::mission::plan::PlannedStep;
use nervros_core::mission::preview::PreviewStep;
use nervros_core::mission::sanity::Concern;
use nervros_core::session::{Command, Event, NotApproved, Unanswered};
use nervros_core::tools::Status;
use rerun::external::egui::{self, Color32, RichText};
use serde_json::Value;

use crate::theme;

mod approval;
mod job;
mod plan;
mod report;
#[cfg(test)]
pub(crate) mod tests;
mod tree;
mod view;

pub(crate) use job::readable;
pub(crate) use plan::plan_card;
pub(crate) use view::{decode, spinner, thousands};

use approval::approval_card;
use approval::unanswered_card;
use report::report;
pub use tree::tree_tab;
use view::agent_header;
use view::empty_state;
use view::error_card;
use view::image_card;
use view::notice;
use view::reply;
use view::tool_chip;
use view::user_bubble;

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
    /// Pause the running mission, or with false let it go on.
    Pause(bool),
    /// Stop a job, by its id.
    StopJob(String),
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
    /// Why it did not run, once answered no.
    not_approved: Option<NotApproved>,
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
    /// A program or a search the operator approved once.
    Job(job::JobCard),
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
    finished: Option<(Outcome, String, String, f64)>,
    /// Where each step would take the robot, once the executor has said.
    preview: Vec<PreviewStep>,
    /// Ways it may not do what the operator asked.
    concerns: Vec<Concern>,
    /// An edit replaced it before it ran.
    replaced: bool,
    /// Why it never ran: its approval was denied or ran out, or its call was refused.
    not_run: Option<String>,
    /// Each tree node seen so far, in the order they started: the steps and what ran in them.
    nodes: Vec<Node>,
    /// Paused, or waiting to pause once a pick or place ends.
    paused: bool,
}

impl PlanCard {
    /// Whether its mission runs now.
    #[must_use]
    pub fn running(&self) -> bool {
        self.mission.is_some() && self.finished.is_none()
    }

    /// Whether its mission is paused, or waits to pause once a pick or place ends.
    #[must_use]
    pub fn paused(&self) -> bool {
        self.running() && self.paused
    }
}

/// A node of a running mission's tree, as its events report it.
#[derive(Debug, Clone, PartialEq)]
pub struct Node {
    /// Its path in the tree, the step's name first: `s2_GoToPlace/walk_there`.
    pub path: String,
    /// Its latest status: `running`, `success`, `failure`, or `idle` once halted.
    pub status: String,
    /// Seconds into the mission it started, and ended once it has.
    pub started_s: f64,
    pub ended_s: Option<f64>,
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
    /// The robot's executor can pause a mission, so a running plan offers it.
    pub can_pause: bool,
    /// When the running turn started.
    pub turn: Option<Instant>,
    /// The model of the last reply.
    pub model: Option<String>,
    /// Tokens the latest request took of the model's window, when the window is known.
    pub context: Option<(u64, u64)>,
    /// Times the conversation was condensed on its own, as the window filled: the status bar
    /// counts them, the chat shows only what the operator asked for.
    pub condensed: u32,
    /// A job's newest mission, kept off the chat (the job's card tells it) for the Mission tab,
    /// with how many items the chat had then, to tell it from a newer plan of the model's.
    job_plan: Option<(usize, PlanCard)>,
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

    /// What the agent says or does, as against what the operator, the robot or the app says.
    fn is_agents(&self) -> bool {
        matches!(
            self,
            Self::Reply { .. }
                | Self::Tool(_)
                | Self::Image { .. }
                | Self::Approval(_)
                | Self::Plan(_)
                | Self::Job(_)
        )
    }
}

impl Chat {
    /// Adds the operator's message. A message said mid-turn that the session sends again as a
    /// turn of its own is the same bubble, now a turn's.
    fn push_user(&mut self, text: String, turn: bool) {
        // Said while the agent worked and unread, it starts the next turn: the bubble is the one
        // already shown, even with the reply that was running drawn after it.
        if turn {
            let said = self
                .items
                .iter_mut()
                .rev()
                .take_while(|i| !matches!(i, Item::User { turn: true, .. }))
                .find_map(|i| match i {
                    Item::User { text: said, turn } if !*turn && *said == text => Some(turn),
                    _ => None,
                });
            if let Some(turn) = said {
                *turn = true;
                return;
            }
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
                stamp_s,
            } => match decode(jpeg) {
                Some(pixels) => self.items.push(Item::Image {
                    id: id.clone(),
                    pixels: RefCell::new(Some(Arc::new(pixels))),
                    texture: OnceCell::new(),
                    marks: marks.clone(),
                    at: taken_at(*stamp_s),
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
            Event::Halted { reason } => self.items.push(Item::Notice(plan::capitalise(reason))),
            Event::Stopped { ok: true, detail } => {
                self.items
                    .push(Item::Notice(format!("The robot stopped: {detail}")));
            }
            Event::Stopped { ok: false, detail } => {
                self.items.push(Item::error(format!(
                    "The robot did not confirm the stop: {detail}"
                )));
            }
            // As every notice reads: the robot's answers come as phrases, "no mission is running".
            Event::Notice { text } => self
                .items
                .push(Item::Notice(plan::capitalise(&report::short_ids(text)))),
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
            | Event::MissionPaused { .. }
            | Event::MissionFinished { .. } => self.apply_mission(event),
            Event::JobStarted { job } => self.items.push(Item::Job(job::JobCard::new(job.clone()))),
            Event::JobLog { id, line } => {
                if let Some(card) = self.job_mut(id) {
                    card.log(line);
                }
            }
            Event::JobFinished { job } => {
                if let Some(card) = self.job_mut(&job.id) {
                    card.ended(job);
                }
            }
            // The viewer draws a plot, and the tool's card already says what; the Doctor tab
            // lists an incident, and a toast says it opened.
            Event::Plot { .. } | Event::Drawn { .. } | Event::Incident { .. } => {}
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
                let found = self.items.iter().rposition(
                    |i| matches!(i, Item::Tool(t) if t.turn == *turn && t.call == *call),
                );
                let Some(at) = found else { return };
                if let Item::Tool(t) = &mut self.items[at] {
                    t.status = Some(*status);
                    t.message.clone_from(message);
                    t.ms = *ms;
                }
                // A plan its call then refused, as run_mission when disarmed or schedule on a bad
                // count, never runs: the plans this call made come before the next call's row.
                if !status.ok() {
                    for item in &mut self.items[at + 1..] {
                        match item {
                            Item::Tool(_) => break,
                            Item::Plan(p) if p.mission.is_none() && !p.replaced => {
                                p.not_run.get_or_insert_with(|| message.clone());
                            }
                            _ => {}
                        }
                    }
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
                not_approved: None,
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
            Event::ApprovalResolved {
                id,
                approved,
                not_approved,
            } => {
                let mut asked_for = None;
                for item in &mut self.items {
                    if let Item::Approval(a) = item
                        && a.id == *id
                    {
                        a.answer = Some(*approved);
                        a.not_approved = *not_approved;
                        asked_for = a.args["hash"].as_str().map(str::to_owned);
                    }
                }
                // Else its card would say "waiting to run" for good.
                if !*approved
                    && let Some(hash) = asked_for.filter(|h| !h.is_empty())
                    && let Some(p) = self.plan_mut(|p| p.hash.starts_with(&hash))
                {
                    let why = match not_approved {
                        Some(NotApproved::Unanswered) => "nobody answered in time",
                        Some(NotApproved::Disarmed) => "the robot was disarmed",
                        _ => "not approved",
                    };
                    p.not_run.get_or_insert_with(|| why.to_owned());
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
        let after = args["hash"].as_str().unwrap_or_default();
        // An edit that changed nothing replaces nothing.
        if !after.starts_with(&before)
            && let Some(p) = self.plan_mut(|p| !before.is_empty() && p.hash.starts_with(&before))
        {
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
            Event::Compacted {
                before,
                after,
                asked: true,
                ..
            } => self.items.push(Item::Notice(format!(
                "Condensed the conversation to stay inside the model's context: {} to {} tokens",
                thousands(*before),
                thousands(*after)
            ))),
            Event::Compacted { .. } => self.condensed += 1,
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
                by_job,
            } => {
                let card = PlanCard {
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
                    not_run: None,
                    nodes: Vec::new(),
                    paused: false,
                };
                // A job's missions show on its own card in the chat, as the lines it logs.
                if *by_job {
                    self.job_plan = Some((self.items.len(), card));
                } else if !self.items.iter().any(|i| {
                    // Planned again unchanged, as by an edit checked as it was: one card.
                    matches!(i, Item::Plan(p) if p.hash == *hash && p.mission.is_none()
                        && p.finished.is_none() && !p.replaced && p.not_run.is_none())
                }) {
                    self.items.push(Item::Plan(card));
                }
            }
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
                elapsed_s,
            } => {
                if let Some(p) = self.plan_mut(|p| p.mission.as_ref() == Some(id)) {
                    if !path.is_empty() {
                        let done = (status != "running").then_some(*elapsed_s);
                        match p.nodes.iter_mut().find(|n| n.path == *path) {
                            Some(n) => {
                                // A retried node starts again.
                                if status == "running" && n.ended_s.is_some() {
                                    n.started_s = *elapsed_s;
                                }
                                n.status.clone_from(status);
                                n.ended_s = done;
                            }
                            None => p.nodes.push(Node {
                                path: path.clone(),
                                status: status.clone(),
                                started_s: *elapsed_s,
                                ended_s: done,
                            }),
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
            Event::MissionPaused { id, paused } => {
                if let Some(p) = self.plan_mut(|p| p.mission.as_ref() == Some(id)) {
                    p.paused = *paused;
                }
            }
            Event::MissionFinished {
                id,
                outcome,
                failed_step,
                reason,
                elapsed_s,
                ..
            } => {
                if let Some(p) = self.plan_mut(|p| p.mission.as_ref() == Some(id)) {
                    p.finished = Some((*outcome, failed_step.clone(), reason.clone(), *elapsed_s));
                    p.paused = false;
                }
            }
            _ => {}
        }
    }

    /// The card of job `id`.
    fn job_mut(&mut self, id: &str) -> Option<&mut job::JobCard> {
        self.items.iter_mut().rev().find_map(|i| match i {
            Item::Job(card) if card.job.id == id => Some(card),
            _ => None,
        })
    }

    /// The newest plan matching `which`.
    fn plan_mut(&mut self, which: impl Fn(&PlanCard) -> bool) -> Option<&mut PlanCard> {
        if self.job_plan.as_ref().is_some_and(|(_, p)| which(p)) {
            return self.job_plan.as_mut().map(|(_, p)| p);
        }
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
        let in_chat = self
            .items
            .iter()
            .enumerate()
            .rev()
            .find_map(|(at, i)| match i {
                Item::Plan(p) => Some((at, p)),
                _ => None,
            });
        match (in_chat, &self.job_plan) {
            (Some((at, p)), Some((job_at, job))) => Some(if *job_at > at { job } else { p }),
            (None, Some((_, job))) => Some(job),
            (in_chat, None) => in_chat.map(|(_, p)| p),
        }
    }

    /// Draws the conversation; clicks are appended to `actions`.
    pub fn show(&self, ui: &mut egui::Ui, approval_ttl: Duration, actions: &mut Vec<Action>) {
        if self.items.is_empty() && self.turn.is_none() {
            empty_state(ui, actions);
            return;
        }
        let width = ui.available_width().min(MAX_TEXT_WIDTH);
        ui.spacing_mut().item_spacing.y = 10.0;
        // "Condense up to here" counts what the conversation keeps as the operator's.
        let mut later = self
            .items
            .iter()
            .filter(|i| matches!(i, Item::User { turn: true, .. }))
            .count();
        let mut heights = self.heights.borrow_mut();
        heights.resize(self.items.len(), 0.0);
        let shown = ui.clip_rect();
        // The agent's name heads its first item after each message or robot report.
        let mut turn_starts = true;
        for (n, (item, height)) in self.items.iter().zip(heights.iter_mut()).enumerate() {
            let header = turn_starts && item.is_agents();
            if item.is_agents() {
                turn_starts = false;
            } else if matches!(item, Item::User { .. } | Item::Report(_)) {
                turn_starts = true;
            }
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
                    if header {
                        if n > 0 {
                            ui.add_space(4.0);
                        }
                        agent_header(ui, self.model_after(n));
                    }
                    match item {
                        Item::User { text, turn } => {
                            let keep = turn.then(|| {
                                later -= 1;
                                later
                            });
                            ui.add_space(4.0);
                            user_bubble(ui, text, keep, actions);
                        }
                        Item::Reply { text, .. } => reply(ui, text),
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
                        Item::Plan(p) => plan_card(ui, p, self.can_pause, actions),
                        Item::Job(card) => job::job_card(ui, card, actions),
                        Item::Unanswered(u, done) => unanswered_card(ui, u, done, actions),
                    }
                })
                .response
                .rect
                .height();
        }
        drop(heights);
        self.show_live(ui, turn_starts);
        // Notices alone, such as arming first, are no conversation yet: the suggestions stay.
        if self.turn.is_none() && self.items.iter().all(|i| matches!(i, Item::Notice(_))) {
            empty_state(ui, actions);
        }
    }

    /// The reply as it streams in, and how long the turn has run; `turn_starts` when the agent
    /// has not yet said anything since the last message.
    fn show_live(&self, ui: &mut egui::Ui, turn_starts: bool) {
        if let Some(draft) = self.draft.as_deref().filter(|d| !d.trim().is_empty()) {
            if turn_starts {
                agent_header(ui, self.model.as_deref());
            }
            reply(ui, draft);
        }
        if let Some(since) = self.turn {
            ui.horizontal(|ui| {
                spinner(ui);
                let secs = since.elapsed().as_secs();
                ui.label(
                    RichText::new(format!("Working… {secs} s"))
                        .size(13.0)
                        .color(theme::DIM),
                );
            });
            // Only while a turn runs, so an idle app does not redraw.
            ui.ctx().request_repaint_after(Duration::from_millis(250));
        }
    }

    /// The model that answered the turn holding item `n`: its first reply's.
    fn model_after(&self, n: usize) -> Option<&str> {
        self.items[n..]
            .iter()
            .take_while(|i| !matches!(i, Item::User { .. } | Item::Report(_)))
            .find_map(|i| match i {
                Item::Reply { model, .. } => Some(model.as_str()),
                _ => None,
            })
    }
}

/// When a snapshot's frame was taken, for its caption and Show in 3D; its arrival when the stamp
/// is not on the wall clock, as a clock that started with its process (before 2001) is not.
fn taken_at(stamp_s: f64) -> SystemTime {
    Duration::try_from_secs_f64(stamp_s)
        .ok()
        .filter(|d| d.as_secs() > 1_000_000_000)
        .and_then(|d| UNIX_EPOCH.checked_add(d))
        .unwrap_or_else(SystemTime::now)
}
