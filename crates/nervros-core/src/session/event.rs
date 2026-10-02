//! The session's protocol: the commands it takes and the events it reports.

use std::sync::Arc;

use serde::Serialize;
use serde_json::Value;

use crate::llm::History;
use crate::mission::plan::PlannedStep;
use crate::mission::sanity::Concern;
use crate::tools::Status;

/// What a UI or the CLI asks the session to do.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// A message from the operator.
    User(String),
    /// Stop the model's reply; the robot is not touched.
    StopGeneration,
    /// Stop the reply and every motion, through the robot's `StopAll`.
    StopMission,
    /// Approve a pending request.
    Approve(u64),
    /// Approve a pending request, and let its tool run without asking for the rest of the
    /// session, unless it moves the robot or the profile's rules ask for it.
    AllowForSession(u64),
    /// Refuse a pending request.
    Deny(u64),
    /// Check changed arguments for a pending request, such as an edited plan; it then waits on
    /// those, or keeps the old ones when they fail their checks.
    Edit {
        /// The request.
        id: u64,
        /// What to check instead.
        args: Value,
    },
    /// Call a tool for the operator, such as a plan made by a click on the map: checked and
    /// approved as a call of the model's is, outside any turn. The model hears of it only
    /// through what the tool reports, such as a mission's end.
    Run {
        /// The tool.
        tool: String,
        /// Its arguments.
        args: Value,
    },
    /// Enable act-lane tools.
    Arm,
    /// Disable act-lane tools.
    Disarm,
    /// Something the robot reports, such as a finished mission; the model answers it in a turn
    /// of its own, after any running turn.
    Report(String),
    /// Condense the conversation now, as it is condensed when it grows past half the model's
    /// window.
    Compact,
    /// Condense the conversation up to one of the operator's messages: everything before their
    /// last `keep` messages is summarised, and those stay as they are.
    CompactUpTo {
        /// The operator's newest messages to keep.
        keep: usize,
    },
    /// Carry on an earlier conversation instead of this one.
    Restore(History),
}

/// What the session reports.
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Event {
    /// A turn began.
    TurnStarted {
        /// Turn number.
        turn: u64,
    },
    /// A message the operator sent while a turn ran: the model reads it with its next tool
    /// result, or in a turn of its own when the turn ends first.
    Steer {
        /// The running turn.
        turn: u64,
        /// The text.
        text: String,
    },
    /// The operator's message that started a turn.
    User {
        /// Turn number.
        turn: u64,
        /// The text.
        text: String,
    },
    /// A robot report that started a turn.
    Report {
        /// Turn number.
        turn: u64,
        /// The text.
        text: String,
    },
    /// The model's reply.
    Reply {
        /// Turn number.
        turn: u64,
        /// The text.
        text: String,
        /// Which model answered.
        model: String,
    },
    /// A tool call began.
    ToolStarted {
        /// Turn number.
        turn: u64,
        /// Call number.
        call: u64,
        /// Tool name.
        tool: String,
        /// Arguments as the model sent them.
        args: Value,
    },
    /// A tool call ended.
    ToolFinished {
        /// Turn number.
        turn: u64,
        /// Call number.
        call: u64,
        /// Tool name.
        tool: String,
        /// How it ended.
        status: Status,
        /// The outcome's message.
        message: String,
        /// Duration in milliseconds.
        ms: u64,
    },
    /// A marked image to show.
    Snapshot {
        /// The snapshot id marks refer to.
        id: String,
        /// JPEG bytes; the log stores them as a file.
        #[serde(skip)]
        jpeg: Arc<Vec<u8>>,
        /// Width.
        width: u32,
        /// Height.
        height: u32,
        /// What each numbered mark on it is, mark 1 first.
        #[serde(skip_serializing_if = "Vec::is_empty")]
        marks: Vec<String>,
    },
    /// A number in a topic's messages to draw over time, in the viewer's Plots tab.
    Plot {
        /// The series' name.
        name: String,
        /// The topic.
        topic: String,
        /// Its message type.
        msg_type: String,
        /// A dotted path to the number in each message.
        field: String,
        /// How long to draw it, in seconds.
        for_s: u64,
    },
    /// A piece of the reply as the model writes it; the `Reply` that follows replaces them.
    ReplyDelta {
        /// Turn number.
        turn: u64,
        /// The new text.
        text: String,
    },
    /// A model call ended: what it cost, for the log and the evals.
    ModelCall {
        /// Turn number.
        turn: u64,
        /// The model id from `models.toml`.
        model: String,
        /// Prompt tokens, as the provider counted them.
        input_tokens: u64,
        /// Of those, read from the provider's prompt cache.
        cached_tokens: u64,
        /// Reply tokens.
        output_tokens: u64,
        /// From the request to the whole reply.
        ms: u64,
    },
    /// How full the model's context was on the turn's latest request.
    Context {
        /// Tokens the request took, as the provider counted them, or as estimated.
        used: u64,
        /// The model's window.
        window: u64,
    },
    /// An earlier conversation was taken up: its exchanges, oldest first, the operator's marked.
    Restored {
        /// Each message's text, and whether the operator wrote it.
        exchanges: Vec<(bool, String)>,
    },
    /// The conversation was condensed to stay inside the model's context.
    Compacted {
        /// Its estimated size before, in tokens.
        before: u64,
        /// And after.
        after: u64,
        /// A model summarised the older part, rather than its old results only being cut.
        summarised: bool,
    },
    /// The operator must approve a call.
    ApprovalRequested {
        /// Answer with `Approve(id)`, `Deny(id)` or `Edit`.
        id: u64,
        /// The tool.
        tool: String,
        /// Its arguments.
        args: Value,
        /// Why approval is needed.
        reason: String,
        /// Whether `AllowForSession` would spare asking again: never for what moves the robot.
        can_allow: bool,
    },
    /// An edit to a pending approval passed its checks: it waits on these arguments now.
    ApprovalEdited {
        /// The request.
        id: u64,
        /// What it runs with if approved.
        args: Value,
        /// What it does now.
        reason: String,
    },
    /// An edit to a pending approval failed its checks; it still waits on what it had.
    EditRejected {
        /// The request.
        id: u64,
        /// What is wrong with the edit.
        message: String,
    },
    /// An approval was answered or expired.
    ApprovalResolved {
        /// The request.
        id: u64,
        /// Whether it may run.
        approved: bool,
    },
    /// Arming changed.
    Armed {
        /// The new state.
        armed: bool,
    },
    /// Work was stopped.
    Halted {
        /// Why and what was stopped.
        reason: String,
    },
    /// What the robot answered a stop of the robot from the operator.
    Stopped {
        /// Whether it confirmed the stop.
        ok: bool,
        /// Its state after the stop, or why the stop failed.
        detail: String,
    },
    /// Something the operator should know.
    Notice {
        /// The text.
        text: String,
    },
    /// A turn failed.
    Error {
        /// Turn number.
        turn: u64,
        /// What went wrong.
        text: String,
    },
    /// A turn ended.
    TurnFinished {
        /// Turn number.
        turn: u64,
    },
    /// A plan passed its checks and can be run by its hash.
    MissionPlanned {
        /// SHA-256 of the tree.
        hash: String,
        /// What the operator asked for.
        intent: String,
        /// The steps.
        steps: Vec<PlannedStep>,
        /// The longest it can take.
        worst_case_s: f64,
        /// Ways it may not do what the operator asked.
        #[serde(skip_serializing_if = "Vec::is_empty")]
        concerns: Vec<Concern>,
    },
    /// Where a checked plan would take the robot, for the viewer; follows its `MissionPlanned`.
    MissionPreview {
        /// The plan's hash.
        hash: String,
        /// Each step's predicted end and path.
        steps: Vec<crate::mission::preview::PreviewStep>,
    },
    /// The executor started a mission.
    MissionStarted {
        /// Mission id.
        id: String,
        /// The plan's hash.
        hash: String,
    },
    /// A step, or a node inside it, changed state.
    MissionProgress {
        /// Mission id.
        id: String,
        /// `s1`, `s2`, ...
        step: String,
        /// The node inside the step, or empty for the step itself.
        node: String,
        /// The node's path in the tree, `/` between the subtrees it is in.
        #[serde(skip_serializing_if = "String::is_empty")]
        path: String,
        /// `running`, `success`, `failure` or `skipped`.
        status: String,
        /// Since the mission started.
        elapsed_s: f64,
    },
    /// A mission ended.
    MissionFinished {
        /// Mission id.
        id: String,
        /// How it ended.
        outcome: crate::mission::Outcome,
        /// The step that failed, or empty.
        failed_step: String,
        /// The skill's own text.
        reason: String,
        /// How long it ran.
        elapsed_s: f64,
    },
}

impl From<&crate::tools::ImageArtifact> for Event {
    fn from(image: &crate::tools::ImageArtifact) -> Self {
        Self::Snapshot {
            id: image.snapshot.clone(),
            jpeg: Arc::clone(&image.jpeg),
            width: image.width,
            height: image.height,
            marks: image.marks.clone(),
        }
    }
}

/// Whether the operator's message is a plain order to stop, which takes the stop path and never
/// waits on a model: "stop", "Stop!", "halt", "freeze", "please stop" and the like.
#[must_use]
pub fn is_stop_word(text: &str) -> bool {
    let text = text
        .trim()
        .trim_end_matches(['!', '.'])
        .trim()
        .to_lowercase();
    let text = text.strip_prefix("please ").unwrap_or(&text);
    matches!(
        text,
        "stop"
            | "/stop"
            | "halt"
            | "freeze"
            | "stop it"
            | "stop now"
            | "stop moving"
            | "stop the robot"
    )
}
