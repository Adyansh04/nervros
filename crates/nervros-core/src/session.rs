//! The session: one conversation with one robot.
//!
//! An actor owns the conversation. User text starts a turn, which runs as its own task so that stop
//! and approval commands are always handled at once: stop aborts the turn and calls the robot's
//! `StopAll` without asking any model. Each turn tries the routine role's models in order; a model
//! that fails before any act-lane tool ran is replaced by the next one, and after one did, the turn
//! ends instead of repeating an action. Every tool call passes the guard.

use std::borrow::Cow;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use tokio::sync::{broadcast, mpsc, oneshot};
use tokio::task::JoinHandle;
use tracing::Instrument as _;

use crate::guard::{Decision, Guard, Refusal, RuleAction};
use crate::llm::{self, AgentSource, History, LoopTool};
use crate::mission::plan::PlannedStep;
use crate::mission::sanity::Concern;
use crate::providers::Role;
use crate::providers::router::Need;
use crate::tools::{
    Assessment, Lane, Registry, Resource, Risk, Status, Tool, ToolOutcome, ToolSpec,
};

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
        /// `success`, `failure`, `canceled`, `timeout`, `rejected` or `error`.
        outcome: String,
        /// The step that failed, or empty.
        failed_step: String,
        /// The skill's own text.
        reason: String,
        /// How long it ran.
        elapsed_s: f64,
    },
}

/// Session settings.
#[derive(Debug, Clone)]
pub struct SessionConfig {
    /// The system prompt.
    pub preamble: String,
    /// Model calls per turn.
    pub max_model_calls: usize,
    /// Messages kept in the history.
    pub history_max: usize,
    /// Characters of tool result the model sees.
    pub result_chars: usize,
    /// How long approval requests wait.
    pub approval_ttl: Duration,
    /// How long a turn may take, not counting the operator's time on approvals.
    pub turn_time: Duration,
    /// Where the conversation is written after every turn, so it can be resumed.
    pub history_file: Option<std::path::PathBuf>,
    /// A conversation to carry on.
    pub resume: Option<History>,
    /// Added to the system prompt on every turn, such as what the operator asked to remember.
    pub notes: Option<Arc<dyn crate::memory::Notes>>,
    /// Called from the session's own loop, the one that serves Stop, every [`PULSE_PERIOD`]:
    /// a heartbeat fed from it stops when that loop does.
    pub pulse: Option<Arc<dyn Pulse>>,
    /// Where requests waiting on the operator are kept while they wait, so that a session
    /// that ends first leaves them for the next to ask again ([`take_unanswered`]).
    pub pending_file: Option<std::path::PathBuf>,
}

/// A request the operator had not answered when its session ended.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Unanswered {
    /// The tool.
    pub tool: String,
    /// Its arguments, as they would be sent anew.
    pub args: Value,
    /// Why it asked.
    pub reason: String,
}

/// The requests an earlier session left unanswered in `file`; the file goes, so each is offered
/// once.
#[must_use]
pub fn take_unanswered(file: &std::path::Path) -> Vec<Unanswered> {
    let Ok(text) = std::fs::read(file) else {
        return Vec::new();
    };
    if let Err(e) = std::fs::remove_file(file) {
        tracing::warn!(error = %e, "the unanswered requests stay");
    }
    serde_json::from_slice(&text).unwrap_or_default()
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

/// Events a subscriber may fall behind by before it misses some.
pub const EVENT_BACKLOG: usize = 4096;

/// How often the session's loop calls [`SessionConfig::pulse`].
pub const PULSE_PERIOD: Duration = Duration::from_millis(200);

/// Something the session's loop proves alive by calling it.
pub trait Pulse: Send + Sync + std::fmt::Debug {
    /// One beat.
    fn pulse(&self);
}

impl Default for SessionConfig {
    fn default() -> Self {
        Self {
            preamble: String::new(),
            max_model_calls: 6,
            history_max: 60,
            result_chars: 6000,
            approval_ttl: Duration::from_mins(1),
            turn_time: Duration::from_secs(90),
            history_file: None,
            resume: None,
            notes: None,
            pulse: None,
            pending_file: None,
        }
    }
}

/// Who asked for a tool call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Caller {
    /// The model, in a turn.
    Model,
    /// The operator, from the window.
    Operator,
}

/// The operator's answer to an approval.
#[derive(Debug)]
enum Answer {
    Approve,
    /// Approve, and stop asking about this tool for the session.
    AllowForSession,
    Deny,
    /// Check these arguments instead, and wait again.
    Edit(Value),
}

/// What an approval settled: the arguments to run with and what they occupy.
struct Approved {
    args: Value,
    resources: Vec<Resource>,
}

/// Denials in a row after which the model is told to stop asking and ask the operator instead.
const DENIAL_BREAK: u32 = 3;
/// The same failure this many times in a row in a turn is stuck, not unlucky.
const STUCK_AFTER: u32 = 3;

/// An approval waiting on the operator.
struct Asked {
    answer: oneshot::Sender<Answer>,
    /// It would act on the robot, so disarming turns it down.
    acts: bool,
}

/// Closes an approval however its wait ends: answered, or dropped by a stop or the end of the
/// session, so the window never keeps a card nobody can answer.
struct Asking<'a> {
    shared: &'a Shared,
    id: u64,
    approved: bool,
}

impl Drop for Asking<'_> {
    fn drop(&mut self) {
        lock(&self.shared.approvals).remove(&self.id);
        self.shared.emit(Event::ApprovalResolved {
            id: self.id,
            approved: self.approved,
        });
    }
}

/// Ends a tool call's chip however the call ends: a stop or the turn's time limit drops the
/// call before it returns.
struct Calling<'a> {
    shared: &'a Shared,
    turn: u64,
    call: u64,
    tool: String,
    started: Instant,
    done: bool,
}

impl Drop for Calling<'_> {
    fn drop(&mut self) {
        if !self.done {
            self.shared.emit(Event::ToolFinished {
                turn: self.turn,
                call: self.call,
                tool: std::mem::take(&mut self.tool),
                status: Status::Stopped,
                message: "stopped before it finished".to_owned(),
                ms: millis(self.started.elapsed()),
            });
        }
    }
}

fn millis(d: Duration) -> u64 {
    u64::try_from(d.as_millis()).unwrap_or(u64::MAX)
}

struct Shared {
    events: broadcast::Sender<Event>,
    approvals: Mutex<HashMap<u64, Asked>>,
    /// Approvals the operator turned down in a row since they last approved one or spoke.
    denials: AtomicU32,
    /// This turn's last failed result, as tool and message, and how often it came in a row.
    repeated: Mutex<(String, u32)>,
    /// What the operator said during the running turn, for its next tool result.
    steer: Mutex<Vec<String>>,
    calls: AtomicU64,
    next_approval: AtomicU64,
    /// Milliseconds this turn spent waiting for the operator.
    waited_ms: AtomicU64,
    /// Tools the operator let run without asking for the rest of the session.
    allowed: Mutex<HashSet<String>>,
    /// Requests waiting on the operator, by approval id, as `pending_file` keeps them.
    waiting: Mutex<BTreeMap<u64, Unanswered>>,
    guard: Arc<Guard>,
    /// The robot's stop: always allowed, whoever asks and however busy the turn.
    stop: Option<Arc<dyn Tool>>,
    config: SessionConfig,
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

impl Shared {
    fn emit(&self, e: Event) {
        // No subscriber is fine: a headless run may not watch events.
        let _ = self.events.send(e);
    }

    /// Asks the operator, who may approve, deny or edit; an edit that passes its checks replaces
    /// what is asked and the wait starts again. `None` when denied or expired.
    #[tracing::instrument(
        name = "nervros.approval",
        skip_all,
        fields(nervros.tool = name, nervros.outcome = tracing::field::Empty)
    )]
    async fn ask_approval(
        &self,
        tool: &Arc<dyn Tool>,
        name: &str,
        reason: String,
        risk: Risk,
        mut approved: Approved,
    ) -> Option<Approved> {
        let can_allow = !matches!(risk, Risk::Motion | Risk::Manipulation);
        let acts = risk.lane() == Lane::Act;
        let id = self.next_approval.fetch_add(1, Ordering::Relaxed) + 1;
        let mut asking = Asking {
            shared: self,
            id,
            approved: false,
        };
        let mut rx = self.listen(id, acts);
        self.keep_waiting(id, tool, name, &reason, &approved.args);
        self.emit(Event::ApprovalRequested {
            id,
            tool: name.to_owned(),
            args: approved.args.clone(),
            reason: reason.clone(),
            can_allow,
        });
        let asked = Instant::now();
        let mut early = None;
        let yes = loop {
            let answer = match early.take() {
                Some(answer) => Ok(Ok(answer)),
                None => tokio::time::timeout(self.config.approval_ttl, &mut rx).await,
            };
            match answer {
                Ok(Ok(Answer::Approve)) => break true,
                Ok(Ok(Answer::AllowForSession)) => {
                    let text = if can_allow {
                        lock(&self.allowed).insert(name.to_owned());
                        format!(
                            "`{name}` runs without asking for the rest of this session, except \
                             what moves the robot"
                        )
                    } else {
                        format!("`{name}` moves the robot, so it still asks every time")
                    };
                    self.emit(Event::Notice { text });
                    break true;
                }
                Ok(Ok(Answer::Edit(edited))) => {
                    // Waiting again before the check, so a stop during it still lands.
                    rx = self.listen(id, acts);
                    let passed = match tool.assess_operator(edited).await {
                        Some(Ok(a)) => {
                            if let Some(args) = a.args {
                                approved.args = args;
                            }
                            approved.resources = a.resources;
                            self.keep_waiting(id, tool, name, &reason, &approved.args);
                            self.emit(Event::ApprovalEdited {
                                id,
                                args: approved.args.clone(),
                                reason: a.reason,
                            });
                            true
                        }
                        Some(Err(out)) => {
                            self.emit(Event::EditRejected {
                                id,
                                message: out.message,
                            });
                            false
                        }
                        None => {
                            self.emit(Event::EditRejected {
                                id,
                                message: format!("{name} cannot be edited"),
                            });
                            false
                        }
                    };
                    // An approval sent during the check was for the plan before it, so it does
                    // not approve an edit nobody has seen; a denial or a newer edit stands.
                    match rx.try_recv() {
                        Ok(Answer::Approve) if passed => rx = self.listen(id, acts),
                        Ok(answer) => early = Some(answer),
                        Err(_) => {}
                    }
                }
                Ok(Ok(Answer::Deny) | Err(_)) | Err(_) => break false,
            }
        };
        self.waited_ms
            .fetch_add(millis(asked.elapsed()), Ordering::Relaxed);
        self.done_waiting(id);
        tracing::Span::current().record("nervros.outcome", if yes { "approved" } else { "denied" });
        asking.approved = yes;
        drop(asking);
        yes.then_some(approved)
    }

    /// Keeps approval `id` where the next session finds it if this one ends first.
    fn keep_waiting(&self, id: u64, tool: &Arc<dyn Tool>, name: &str, reason: &str, args: &Value) {
        if self.config.pending_file.is_none() {
            return;
        }
        if let Some(args) = tool.ask_again(args) {
            let entry = Unanswered {
                tool: name.to_owned(),
                args,
                reason: reason.to_owned(),
            };
            lock(&self.waiting).insert(id, entry);
            self.write_waiting();
        }
    }

    fn done_waiting(&self, id: u64) {
        if lock(&self.waiting).remove(&id).is_some() {
            self.write_waiting();
        }
    }

    fn write_waiting(&self) {
        let Some(file) = &self.config.pending_file else {
            return;
        };
        let waiting = lock(&self.waiting);
        let written = if waiting.is_empty() {
            match std::fs::remove_file(file) {
                Err(e) if e.kind() != std::io::ErrorKind::NotFound => Err(e),
                _ => Ok(()),
            }
        } else {
            serde_json::to_vec(&waiting.values().collect::<Vec<_>>())
                .map_err(std::io::Error::other)
                .and_then(|json| std::fs::write(file, json))
        };
        if let Err(e) = written {
            tracing::warn!(error = %e, "the requests waiting on the operator were not kept");
        }
    }

    /// A fresh channel for the next answer to approval `id`.
    fn listen(&self, id: u64, acts: bool) -> oneshot::Receiver<Answer> {
        let (answer, rx) = oneshot::channel();
        lock(&self.approvals).insert(id, Asked { answer, acts });
        rx
    }

    fn answer(&self, id: u64, answer: Answer) {
        if let Some(asked) = lock(&self.approvals).remove(&id) {
            let _ = asked.answer.send(answer);
        }
    }

    /// Turns down every request that would act on the robot, as disarming does.
    fn deny_acts(&self) {
        let acting: Vec<u64> = lock(&self.approvals)
            .iter()
            .filter(|(_, a)| a.acts)
            .map(|(id, _)| *id)
            .collect();
        for id in acting {
            self.answer(id, Answer::Deny);
        }
    }

    async fn run(&self, tool: &Arc<dyn Tool>, args: Value, resources: &[Resource]) -> ToolOutcome {
        let _held = if resources.is_empty() {
            None
        } else {
            match self.guard.lock(resources) {
                Ok(l) => Some(l),
                Err(r) => return ToolOutcome::refused(r.message),
            }
        };
        tool.call(args).await
    }

    /// What a call would do, for a tool whose risk depends on its arguments, and the answer when
    /// it cannot go out at all, before anyone is asked to approve it. The model's arguments are
    /// checked against the schema first: a skill name it made up ends here.
    async fn assess(
        tool: &Arc<dyn Tool>,
        args: &Value,
        caller: Caller,
    ) -> Option<Result<Assessment, ToolOutcome>> {
        match caller {
            Caller::Model => {
                let invalid = crate::argcheck::check(&tool.spec().parameters, args);
                if invalid.is_empty() {
                    tool.assess(args).await
                } else {
                    Some(Err(ToolOutcome::failed(format!(
                        "{}; call it again with arguments that fit",
                        invalid.join("; ")
                    ))))
                }
            }
            Caller::Operator => match tool.assess_operator(args.clone()).await {
                Some(checked) => Some(checked),
                None => tool.assess(args).await,
            },
        }
    }

    /// Asks the operator, then runs it if they approve. After `DENIAL_BREAK` denials in a row the
    /// model is told to stop asking, and asks no more until the operator approves or speaks.
    async fn approved_run(
        &self,
        tool: &Arc<dyn Tool>,
        spec: &ToolSpec,
        args: Value,
        reason: String,
        caller: Caller,
    ) -> ToolOutcome {
        if caller == Caller::Model && self.denials.load(Ordering::SeqCst) >= DENIAL_BREAK {
            return ToolOutcome::refused(format!(
                "the operator turned down the last {DENIAL_BREAK} requests: stop asking, and ask \
                 them what they want instead"
            ));
        }
        let asked = Approved {
            args,
            resources: spec.resources.clone(),
        };
        if let Some(a) = self
            .ask_approval(tool, &spec.name, reason, spec.risk, asked)
            .await
        {
            self.denials.store(0, Ordering::SeqCst);
            // Disarming during the wait outranks an approval given before it.
            if spec.lane() == Lane::Act && !self.guard.armed() {
                return ToolOutcome::refused(
                    "the robot was disarmed while this waited for approval; nothing ran",
                );
            }
            return self.run(tool, a.args, &a.resources).await;
        }
        let n = self.denials.fetch_add(1, Ordering::SeqCst) + 1;
        ToolOutcome::refused(if n >= DENIAL_BREAK {
            format!(
                "the operator did not approve this, {n} times in a row: stop acting and ask them \
                 what they want"
            )
        } else {
            "the operator did not approve this".to_owned()
        })
    }

    /// The guard's decision, tightened by the profile's rule for this call: its refusal stands
    /// over everything, and its question makes an allowed call ask.
    fn decide(
        &self,
        spec: &ToolSpec,
        args: &Value,
        caller: Caller,
        rule: Option<crate::guard::ArgRule>,
    ) -> Decision {
        if let Some(r) = rule.as_ref().filter(|r| r.then == RuleAction::Deny) {
            return Decision::Deny(Refusal::new("rule", r.reason.clone()));
        }
        let decision = match caller {
            Caller::Model => self.guard.decide(spec, args),
            Caller::Operator => self.guard.decide_operator(spec),
        };
        let decision = match (decision, rule) {
            (Decision::Allow, Some(r)) => Decision::NeedApproval { reason: r.reason },
            // Allowed for the session, unless this call moves the robot.
            (Decision::NeedApproval { .. }, None)
                if caller == Caller::Model
                    && !matches!(spec.risk, Risk::Motion | Risk::Manipulation)
                    && lock(&self.allowed).contains(&spec.name) =>
            {
                Decision::Allow
            }
            (decision, _) => decision,
        };
        let (verdict, why) = match &decision {
            Decision::Allow => ("allow", ""),
            Decision::NeedApproval { reason } => ("ask", reason.as_str()),
            Decision::Deny(r) => ("deny", r.message.as_str()),
        };
        tracing::info!(nervros.tool = %spec.name, nervros.guard = verdict, why, "guard");
        decision
    }

    async fn invoke(
        &self,
        tool: &Arc<dyn Tool>,
        turn: u64,
        args: Value,
        flags: &TurnFlags,
        caller: Caller,
    ) -> Value {
        let call = self.calls.fetch_add(1, Ordering::Relaxed) + 1;
        // The stop is never held up: no budget, loop breaker, rule or argument check applies.
        let stopping = self.stop.as_ref().is_some_and(|s| Arc::ptr_eq(s, tool));
        let (assessed, rule) = if stopping {
            (None, None)
        } else {
            // The profile's rules read the arguments as they were sent, before checking settled
            // them.
            let rule = self.guard.rule(&tool.spec().name, &args).cloned();
            (Self::assess(tool, &args, caller).await, rule)
        };
        let (mut assessment, early) = match assessed {
            Some(Ok(a)) => (Some(a), None),
            Some(Err(out)) => (None, Some(out)),
            None => (None, None),
        };
        let args = assessment
            .as_mut()
            .and_then(|a| a.args.take())
            .unwrap_or(args);
        let spec = match &assessment {
            Some(a) => {
                let mut spec = tool.spec().into_owned();
                spec.risk = a.risk;
                spec.resources.clone_from(&a.resources);
                Cow::Owned(spec)
            }
            None => tool.spec(),
        };
        self.emit(Event::ToolStarted {
            turn,
            call,
            tool: spec.name.clone(),
            args: args.clone(),
        });
        let mut calling = Calling {
            shared: self,
            turn,
            call,
            tool: spec.name.clone(),
            started: Instant::now(),
            done: false,
        };
        let outcome = match early {
            Some(out) => out,
            None if stopping => {
                self.guard.note_stop();
                tool.call(args).await
            }
            None => match self.decide(&spec, &args, caller, rule) {
                Decision::Deny(r) => ToolOutcome::refused(r.message),
                Decision::NeedApproval { reason } => {
                    let reason = assessment.map_or(reason, |a| a.reason);
                    self.approved_run(tool, &spec, args, reason, caller).await
                }
                Decision::Allow => self.run(tool, args, &spec.resources).await,
            },
        };
        // An edit is not repeated by the next model either.
        if spec.lane() != Lane::Observe && outcome.status.ok() {
            flags.acted.store(true, Ordering::SeqCst);
            lock(&flags.done).push(format!(
                "{} {}: {}",
                spec.name,
                outcome.status,
                crate::tools::clip(&outcome.message, 200)
            ));
        }
        // Something now runs that will report back, such as a mission: the rest is the answer.
        if outcome.status == Status::Accepted {
            flags.started.store(true, Ordering::SeqCst);
        }
        for image in &outcome.images {
            self.emit(Event::Snapshot {
                id: image.snapshot.clone(),
                jpeg: Arc::clone(&image.jpeg),
                width: image.width,
                height: image.height,
                marks: image.marks.clone(),
            });
        }
        calling.done = true;
        self.emit(Event::ToolFinished {
            turn,
            call,
            tool: spec.name.clone(),
            status: outcome.status,
            message: crate::tools::clip(&outcome.message, 2000),
            ms: millis(calling.started.elapsed()),
        });
        self.for_model(&spec.name, &outcome, caller)
    }

    /// What the model reads of an outcome: what the operator said meanwhile, and whether the turn
    /// is stuck, go in fields of their own, past the cut of the message, so they always reach it.
    fn for_model(&self, tool: &str, outcome: &ToolOutcome, caller: Caller) -> Value {
        let mut out = outcome.for_model(self.config.result_chars);
        if caller == Caller::Model {
            let said = std::mem::take(&mut *lock(&self.steer));
            if !said.is_empty() {
                out["operator"] = json!(format!(
                    "said while you worked: \"{}\"; take it into account now",
                    said.join(" ")
                ));
            }
        }
        if let Some(n) = self.stuck(tool, outcome) {
            out["stuck"] = json!(format!(
                "came back the same {n} times in a row: do not try it again; tell the operator \
                 what is stuck"
            ));
        }
        out
    }

    /// How often this turn's failures have come back the same in a row, once that is stuck.
    fn stuck(&self, tool: &str, outcome: &ToolOutcome) -> Option<u32> {
        let mut last = lock(&self.repeated);
        if outcome.status.ok() {
            *last = (String::new(), 0);
            return None;
        }
        let said = format!("{tool}: {}", outcome.message);
        if last.0 == said {
            last.1 += 1;
        } else {
            *last = (said, 1);
        }
        (last.1 >= STUCK_AFTER).then_some(last.1)
    }
}

/// Lets background work, such as a running mission, report into a session without keeping it
/// alive.
#[derive(Debug, Clone)]
pub struct SessionHandle {
    tx: mpsc::WeakUnboundedSender<Command>,
    events: broadcast::Sender<Event>,
}

impl SessionHandle {
    /// Sends a command; ignored once the session has ended.
    pub fn send(&self, command: Command) {
        if let Some(tx) = self.tx.upgrade() {
            let _ = tx.send(command);
        }
    }

    /// Publishes an event to the session's subscribers.
    pub fn emit(&self, event: Event) {
        let _ = self.events.send(event);
    }

    /// A new event stream.
    #[must_use]
    pub fn subscribe(&self) -> broadcast::Receiver<Event> {
        self.events.subscribe()
    }

    /// A handle onto bare channels, for tests of background work.
    #[cfg(test)]
    pub(crate) fn for_tests(
        tx: &mpsc::UnboundedSender<Command>,
        events: broadcast::Sender<Event>,
    ) -> Self {
        Self {
            tx: tx.downgrade(),
            events,
        }
    }
}

/// A running session. Dropping it ends the actor.
pub struct Session {
    tx: mpsc::UnboundedSender<Command>,
    events: broadcast::Sender<Event>,
}

impl std::fmt::Debug for Session {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Session").finish_non_exhaustive()
    }
}

impl Session {
    /// Starts the actor. `stop` is the robot's stop tool, called directly on [`Command::StopMission`].
    #[must_use]
    pub fn start(
        source: Arc<dyn AgentSource>,
        registry: Arc<Registry>,
        guard: Arc<Guard>,
        stop: Option<Arc<dyn Tool>>,
        config: SessionConfig,
    ) -> Self {
        let (tx, rx) = mpsc::unbounded_channel();
        // A streamed reply sends an event per piece; a slow reader falls behind only past this.
        let (events, _) = broadcast::channel(EVENT_BACKLOG);
        let shared = Arc::new(Shared {
            events: events.clone(),
            approvals: Mutex::default(),
            calls: AtomicU64::new(0),
            next_approval: AtomicU64::new(0),
            denials: AtomicU32::new(0),
            repeated: Mutex::default(),
            steer: Mutex::default(),
            waited_ms: AtomicU64::new(0),
            allowed: Mutex::default(),
            waiting: Mutex::default(),
            guard,
            stop,
            config,
        });
        tokio::spawn(actor(rx, shared, source, registry));
        Self { tx, events }
    }

    /// Sends a command; ignored once the actor has ended.
    pub fn send(&self, command: Command) {
        let _ = self.tx.send(command);
    }

    /// A new event stream.
    #[must_use]
    pub fn subscribe(&self) -> broadcast::Receiver<Event> {
        self.events.subscribe()
    }

    /// A handle for background work.
    #[must_use]
    pub fn handle(&self) -> SessionHandle {
        SessionHandle {
            tx: self.tx.downgrade(),
            events: self.events.clone(),
        }
    }
}

#[expect(
    clippy::too_many_lines,
    reason = "one arm per command, each a few lines"
)]
async fn actor(
    mut rx: mpsc::UnboundedReceiver<Command>,
    shared: Arc<Shared>,
    source: Arc<dyn AgentSource>,
    registry: Arc<Registry>,
) {
    let mut history = shared.config.resume.clone().unwrap_or_default();
    let mut turns = 0u64;
    let mut pulse = tokio::time::interval(PULSE_PERIOD);
    pulse.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut running: Option<(u64, JoinHandle<Option<History>>)> = None;
    // Calls the operator made from the window, which a stop of the robot ends too.
    let mut operator_runs: Vec<JoinHandle<()>> = Vec::new();
    let mut reports: Vec<String> = Vec::new();
    // Nobody asked for a report's reply, so a message sent during one waits for it, not refused.
    let (mut answering_report, mut queued) = (false, None::<String>);
    let start = |turn: u64, origin: Origin, history: &History| {
        let span = turn_span(turn, &origin);
        let task = run_turn(
            turn,
            origin,
            history.clone(),
            Arc::clone(&shared),
            Arc::clone(&source),
            Arc::clone(&registry),
        );
        let task = limited(turn, Arc::clone(&shared), task).instrument(span);
        (turn, tokio::spawn(task))
    };
    loop {
        let finished = async {
            match running.as_mut() {
                Some((_, handle)) => handle.await,
                None => std::future::pending().await,
            }
        };
        tokio::select! {
            biased;
            _ = pulse.tick(), if shared.config.pulse.is_some() => {
                if let Some(p) = &shared.config.pulse {
                    p.pulse();
                }
            }
            cmd = rx.recv() => {
                let Some(cmd) = cmd else { break };
                match cmd {
                    Command::User(text) => {
                        // The operator speaking is a new say on what they want.
                        shared.denials.store(0, Ordering::SeqCst);
                        if let Some((turn, _)) = &running {
                            if answering_report {
                                if queued.is_none() {
                                    queued = Some(text);
                                    shared.emit(Event::Notice { text: "queued until the reply to the report ends".into() });
                                } else {
                                    shared.emit(Event::Notice { text: "one message is queued already; wait or stop it".into() });
                                }
                            } else {
                                // Steering: the model reads it with its next tool result.
                                lock(&shared.steer).push(text.clone());
                                shared.emit(Event::Steer { turn: *turn, text });
                            }
                            continue;
                        }
                        turns += 1;
                        running = Some(start(turns, Origin::User(text), &history));
                        answering_report = false;
                    }
                    Command::Report(text) => {
                        reports.push(text);
                        if running.is_none() {
                            turns += 1;
                            running = Some(start(turns, Origin::Report(reports.split_off(0).join("\n\n")), &history));
                            answering_report = true;
                        }
                    }
                    Command::StopGeneration | Command::StopMission => {
                        let mission = cmd == Command::StopMission;
                        if queued.take().is_some() {
                            shared.emit(Event::Notice { text: "the queued message was dropped".into() });
                        }
                        if let Some((turn, handle)) = running.take() {
                            handle.abort();
                            shared.emit(Event::TurnFinished { turn });
                        }
                        if mission {
                            operator_runs.drain(..).for_each(|h| h.abort());
                        }
                        halt(&shared, mission);
                        // Esc stops the reply to say something else: what was said meanwhile goes
                        // now. A stop of the robot drops it.
                        let said = std::mem::take(&mut *lock(&shared.steer));
                        if !said.is_empty() && !mission {
                            turns += 1;
                            running = Some(start(turns, Origin::User(said.join(" ")), &history));
                            answering_report = false;
                        }
                    }
                    Command::Compact | Command::CompactUpTo { .. } => {
                        if running.is_some() {
                            shared.emit(Event::Notice { text: "still working on the last message; compact after it".into() });
                            continue;
                        }
                        let how = match cmd {
                            Command::CompactUpTo { keep } => Condense::UpTo(keep),
                            _ => Condense::Now,
                        };
                        turns += 1;
                        running = Some((turns, compaction(turns, &shared, &source, &registry, history.clone(), how)));
                        answering_report = false;
                    }
                    Command::Restore(earlier) => {
                        if running.is_some() {
                            shared.emit(Event::Notice { text: "still working on the last message; resume after it".into() });
                            continue;
                        }
                        shared.emit(Event::Restored { exchanges: earlier.exchanges() });
                        history = earlier;
                    }
                    Command::Approve(id) => shared.answer(id, Answer::Approve),
                    Command::AllowForSession(id) => shared.answer(id, Answer::AllowForSession),
                    Command::Deny(id) => shared.answer(id, Answer::Deny),
                    Command::Edit { id, args } => shared.answer(id, Answer::Edit(args)),
                    Command::Run { tool, args } => match registry.get(&tool).cloned() {
                        Some(tool) => {
                            let shared = Arc::clone(&shared);
                            operator_runs.retain(|h| !h.is_finished());
                            operator_runs.push(tokio::spawn(async move {
                                let flags = TurnFlags::default();
                                shared.invoke(&tool, 0, args, &flags, Caller::Operator).await;
                            }));
                        }
                        None => shared.emit(Event::Notice { text: format!("there is no tool {tool}") }),
                    },
                    Command::Arm | Command::Disarm => {
                        let armed = cmd == Command::Arm;
                        shared.guard.set_armed(armed);
                        if !armed {
                            shared.deny_acts();
                        }
                        shared.emit(Event::Armed { armed });
                    }
                }
            }
            done = finished => {
                if let Some((turn, _)) = running.take() {
                    if let Ok(Some(updated)) = done {
                        history = updated;
                        save(&shared, &history);
                    }
                    shared.emit(Event::TurnFinished { turn });
                }
                // What the operator said that no tool result carried goes in a turn of its own.
                let unread = std::mem::take(&mut *lock(&shared.steer));
                if let Some(text) = queued.take().or_else(|| (!unread.is_empty()).then(|| unread.join(" "))) {
                    turns += 1;
                    running = Some(start(turns, Origin::User(text), &history));
                    answering_report = false;
                } else if !reports.is_empty() {
                    turns += 1;
                    running = Some(start(turns, Origin::Report(reports.split_off(0).join("\n\n")), &history));
                    answering_report = true;
                }
            }
        }
    }
}

/// Condenses the conversation as a turn of its own, so a stop is never held up by a summary.
fn compaction(
    turn: u64,
    shared: &Arc<Shared>,
    source: &Arc<dyn AgentSource>,
    registry: &Arc<Registry>,
    mut history: History,
    how: Condense,
) -> JoinHandle<Option<History>> {
    let (shared, source, registry) = (Arc::clone(shared), Arc::clone(source), Arc::clone(registry));
    tokio::spawn(async move {
        shared.emit(Event::TurnStarted { turn });
        let room = room_for(&shared, &source, &registry).unwrap_or(COMPACT_ROOM);
        compact(&shared, &source, &mut history, room, how).await;
        Some(history)
    })
}

/// Writes the conversation where the session keeps it, if it keeps it anywhere.
fn save(shared: &Shared, history: &History) {
    if let Some(file) = &shared.config.history_file
        && let Err(e) = history.save(file)
    {
        tracing::warn!(error = %e, "the conversation was not saved");
    }
}

/// Before a turn, condenses a history past half the first candidate's window.
async fn fit_window(
    shared: &Shared,
    source: &Arc<dyn AgentSource>,
    candidates: &[String],
    tools: &[LoopTool],
    history: &mut History,
) {
    let Some(window) = candidates.first().and_then(|m| source.context(m)) else {
        return;
    };
    let room =
        window.saturating_sub(llm::fixed_cost(&preamble(shared), tools) + llm::RESERVE_TOKENS);
    if history.size() > room / 2 {
        compact(shared, source, history, room, Condense::ToFit).await;
    }
}

/// Each model call's cost, to the log and to `used`, the context gauge's count.
fn costs(shared: &Arc<Shared>, turn: u64, model: &str, used: &Arc<AtomicU64>) -> llm::OnCall {
    let (shared, model, used) = (Arc::clone(shared), model.to_owned(), Arc::clone(used));
    Arc::new(move |cost: llm::CallCost| {
        if cost.input_tokens > 0 {
            used.store(cost.input_tokens, Ordering::Relaxed);
        }
        shared.emit(Event::ModelCall {
            turn,
            model: model.clone(),
            input_tokens: cost.input_tokens,
            cached_tokens: cost.cached_tokens,
            output_tokens: cost.output_tokens,
            ms: cost.ms,
        });
    })
}

/// Passes a streamed reply's pieces to the UI.
fn deltas(shared: &Arc<Shared>, turn: u64) -> llm::OnDelta {
    let shared = Arc::clone(shared);
    Arc::new(move |text: &str| {
        shared.emit(Event::ReplyDelta {
            turn,
            text: text.to_owned(),
        });
    })
}

/// Tells the UI how full the model's window was: as the provider counted, or as estimated.
fn report_context(
    shared: &Shared,
    tools: &[LoopTool],
    history: &History,
    counted: u64,
    window: usize,
) {
    let estimated = llm::fixed_cost(&preamble(shared), tools) + history.size();
    shared.emit(Event::Context {
        used: if counted > 0 {
            counted
        } else {
            u64::try_from(estimated).unwrap_or(u64::MAX)
        },
        window: u64::try_from(window).unwrap_or(u64::MAX),
    });
}

/// Denies what waits for approval and, for a stop of the robot, calls its stop too.
fn halt(shared: &Arc<Shared>, robot: bool) {
    let pending: Vec<u64> = lock(&shared.approvals).keys().copied().collect();
    for id in pending {
        shared.answer(id, Answer::Deny);
    }
    // Turned down, so not left for the next session to ask again.
    let waited = !std::mem::take(&mut *lock(&shared.waiting)).is_empty();
    if waited {
        shared.write_waiting();
    }
    let stop = shared.stop.as_ref().filter(|_| robot);
    if robot {
        shared.guard.note_stop();
    }
    if let Some(stop) = stop.cloned() {
        let shared = Arc::clone(shared);
        tokio::spawn(async move {
            let out = stop.call(json!({"reason": "operator"})).await;
            let text = if out.status == Status::Succeeded {
                format!("robot stopped: {}", out.data)
            } else {
                format!("stop failed: {}", out.message)
            };
            shared.emit(Event::Notice { text });
        });
    }
    let reason = if stop.is_some() {
        "stopped by the operator: reply and robot"
    } else {
        "reply stopped by the operator"
    };
    shared.emit(Event::Halted {
        reason: reason.into(),
    });
}

/// Runs a turn under its time limit. A turn over it ends with an error, as a stopped one does;
/// the operator's time on approvals is added back, and never runs out while one is open.
/// A turn's span, named and shaped as OpenTelemetry's `GenAI` conventions name an agent's run, so
/// rig records the turn's token use on it and nests its model and tool calls inside.
fn turn_span(turn: u64, origin: &Origin) -> tracing::Span {
    let empty = tracing::field::Empty;
    tracing::info_span!(
        "invoke_agent",
        otel.name = "invoke_agent nervros",
        gen_ai.operation.name = "invoke_agent",
        gen_ai.agent.name = "nervros",
        nervros.turn = turn,
        nervros.origin = match origin {
            Origin::User(_) => "operator",
            Origin::Report(_) => "report",
        },
        gen_ai.prompt = empty,
        gen_ai.completion = empty,
        gen_ai.usage.input_tokens = empty,
        gen_ai.usage.output_tokens = empty,
        gen_ai.usage.cache_read.input_tokens = empty,
        gen_ai.usage.cache_creation.input_tokens = empty,
        gen_ai.usage.tool_use_prompt_tokens = empty,
        gen_ai.usage.reasoning_tokens = empty,
    )
}

async fn limited(
    turn: u64,
    shared: Arc<Shared>,
    task: impl Future<Output = Option<History>>,
) -> Option<History> {
    tokio::pin!(task);
    shared.waited_ms.store(0, Ordering::Relaxed);
    let started = Instant::now();
    let limit = shared.config.turn_time;
    let mut deadline = started + limit;
    loop {
        tokio::select! {
            out = &mut task => return out,
            () = tokio::time::sleep_until(deadline.into()) => {
                if !lock(&shared.approvals).is_empty() {
                    deadline = Instant::now() + Duration::from_secs(1);
                    continue;
                }
                let credited = started + limit + Duration::from_millis(shared.waited_ms.load(Ordering::Relaxed));
                if credited > Instant::now() {
                    deadline = credited;
                    continue;
                }
                shared.emit(Event::Error {
                    turn,
                    text: format!("the turn took longer than {} s and was stopped", limit.as_secs()),
                });
                return None;
            }
        }
    }
}

/// The system prompt for this turn: the fixed one and the notes as they are now.
fn preamble(shared: &Shared) -> String {
    let notes = shared
        .config
        .notes
        .as_ref()
        .map(|n| n.text())
        .unwrap_or_default();
    format!("{}{notes}", shared.config.preamble)
}

/// Room for the history when the model's window is unknown, for an operator's compaction.
const COMPACT_ROOM: usize = 12_000;

/// Room left for the history in the first routine model's window, if the models file gives it.
fn room_for(shared: &Shared, source: &Arc<dyn AgentSource>, registry: &Registry) -> Option<usize> {
    let need = Need {
        tools: true,
        ..Need::default()
    };
    let window = source
        .candidates(Role::Routine, need)
        .first()
        .and_then(|m| source.context(m))?;
    let schemas: usize = registry
        .iter()
        .map(|t| {
            let spec = t.spec();
            spec.name.len() + spec.description.len() + spec.parameters.to_string().len()
        })
        .sum();
    let fixed = llm::tokens_of(preamble(shared).len() + schemas);
    Some(window.saturating_sub(fixed + llm::RESERVE_TOKENS))
}

/// Condenses the history to about a quarter of `room`: the older part summarised by a model,
/// or, when none answers, old results cut and the oldest exchanges dropped.
/// How far a compaction goes.
#[derive(Debug, Clone, Copy)]
enum Condense {
    /// Past half the window: old results cut first, and a summary only when that is not enough.
    ToFit,
    /// The operator's `/compact`: old results cut, and the older part summarised.
    Now,
    /// "Condense up to here": all but the operator's last `n` messages summarised.
    UpTo(usize),
}

async fn compact(
    shared: &Shared,
    source: &Arc<dyn AgentSource>,
    history: &mut History,
    room: usize,
    how: Condense,
) {
    let before = history.size();
    history.mask();
    let older = match how {
        Condense::ToFit if history.size() <= room / 2 => None,
        Condense::ToFit | Condense::Now => history.older(room / 4),
        Condense::UpTo(keep) => history.before_last(keep),
    };
    let mut summarised = false;
    if let Some((n, text)) = older
        && let Some(summary) = llm::summarise(source, &text).await
    {
        history.summarised(n, &summary);
        summarised = true;
    }
    if history.size() > room / 2 {
        history.squeeze(room / 2);
    }
    shared.emit(Event::Compacted {
        before: u64::try_from(before).unwrap_or(u64::MAX),
        after: u64::try_from(history.size()).unwrap_or(u64::MAX),
        summarised,
    });
}

/// Who started a turn.
enum Origin {
    User(String),
    Report(String),
}

/// What the tools of one turn have done so far.
#[derive(Debug, Default)]
struct TurnFlags {
    /// An act ran, so the turn is not retried on another model.
    acted: AtomicBool,
    /// What each act did, for the history when the reply after it is lost.
    done: Mutex<Vec<String>>,
    /// Something started that reports back later, so no more tools are offered.
    started: Arc<AtomicBool>,
}

/// This turn's tools, each call going through the guard and recorded in `flags`.
fn loop_tools(
    registry: &Registry,
    shared: &Arc<Shared>,
    turn: u64,
    flags: &Arc<TurnFlags>,
) -> Vec<LoopTool> {
    registry
        .iter()
        .map(|tool| {
            let (tool, shared, flags) = (Arc::clone(tool), Arc::clone(shared), Arc::clone(flags));
            let spec = tool.spec().into_owned();
            LoopTool {
                name: spec.name,
                description: spec.description,
                parameters: spec.parameters,
                invoke: Arc::new(move |args| {
                    let (tool, shared, flags) =
                        (Arc::clone(&tool), Arc::clone(&shared), Arc::clone(&flags));
                    Box::pin(async move {
                        let caller = Caller::Model;
                        shared.invoke(&tool, turn, args, &flags, caller).await
                    })
                }),
            }
        })
        .collect()
}

async fn run_turn(
    turn: u64,
    origin: Origin,
    history: History,
    shared: Arc<Shared>,
    source: Arc<dyn AgentSource>,
    registry: Arc<Registry>,
) -> Option<History> {
    shared.guard.begin_turn();
    *lock(&shared.repeated) = (String::new(), 0);
    shared.emit(Event::TurnStarted { turn });
    let text = match origin {
        Origin::User(text) => {
            shared.emit(Event::User {
                turn,
                text: text.clone(),
            });
            text
        }
        Origin::Report(text) => {
            shared.emit(Event::Report {
                turn,
                text: text.clone(),
            });
            format!("{}\n{text}", llm::REPORT_MARK)
        }
    };
    let flags = Arc::new(TurnFlags::default());
    for tool in registry.iter() {
        tool.ready().await;
    }
    let tools = loop_tools(&registry, &shared, turn, &flags);
    let need = Need {
        tools: !tools.is_empty(),
        ..Need::default()
    };
    let candidates = source.candidates(Role::Routine, need);
    let mut history = history;
    fit_window(&shared, &source, &candidates, &tools, &mut history).await;
    let mut failures = Vec::new();
    for model in candidates {
        let mut updated = history.clone();
        let used = Arc::new(AtomicU64::new(0));
        let window = source.context(&model);
        let system = preamble(&shared);
        let setup = llm::TurnSetup {
            preamble: &system,
            max_turns: shared.config.max_model_calls,
            tools: &tools,
            started: Arc::clone(&flags.started),
            window,
            on_call: Some(costs(&shared, turn, &model, &used)),
            delta: Some(deltas(&shared, turn)),
        };
        let result = llm::chat(&model, Arc::clone(&source), setup, &mut updated, &text).await;
        match result {
            Ok(reply) => {
                shared.emit(Event::Reply {
                    turn,
                    text: reply,
                    model,
                });
                if let Some(window) = window {
                    report_context(
                        &shared,
                        &tools,
                        &updated,
                        used.load(Ordering::Relaxed),
                        window,
                    );
                }
                updated.trim(shared.config.history_max);
                return Some(updated);
            }
            Err(e) => {
                if matches!(
                    e,
                    llm::LlmError::Turn {
                        rate_limited: true,
                        ..
                    }
                ) {
                    source.park(&model);
                }
                if !flags.acted.load(Ordering::SeqCst) {
                    shared.emit(Event::Notice {
                        text: format!("{e}; trying the next model"),
                    });
                    failures.push(e.to_string());
                    continue;
                }
                // What the robot was asked to do goes on, and its report will follow; only the
                // reply is lost. The history keeps the request and the act, or the next turn
                // would not know of either and might ask for it again.
                shared.emit(Event::Notice {
                    text: format!("{e}; the robot already acted, so the turn is not retried"),
                });
                history.lost_reply(&text, &lock(&flags.done));
                return Some(history);
            }
        }
    }
    let text = if failures.is_empty() {
        "no model is available".to_owned()
    } else {
        failures.join("; ")
    };
    shared.emit(Event::Error { turn, text });
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::guard::Policy;
    use crate::llm::{AgentBuilder, LlmError};
    use crate::tools::{Risk, ToolSpec};
    use async_trait::async_trait;
    use rig::test_utils::{MockCompletionModel, MockTurn};
    use std::borrow::Cow;

    struct Scripted(MockCompletionModel);

    impl AgentSource for Scripted {
        fn candidates(&self, _role: Role, _need: Need) -> Vec<String> {
            vec!["mock".into()]
        }
        fn builder(&self, _id: &str) -> Result<AgentBuilder, LlmError> {
            Ok(AgentBuilder::new(self.0.clone()))
        }
        fn take_request(&self, _id: &str) -> Result<(), String> {
            Ok(())
        }
        fn park(&self, _id: &str) {}
    }

    /// A scripted model whose quota grants `allowed` requests, counting those it grants.
    struct Metered {
        inner: Scripted,
        allowed: usize,
        taken: std::sync::atomic::AtomicUsize,
    }

    impl AgentSource for Metered {
        fn candidates(&self, role: Role, need: Need) -> Vec<String> {
            self.inner.candidates(role, need)
        }
        fn builder(&self, id: &str) -> Result<AgentBuilder, LlmError> {
            self.inner.builder(id)
        }
        fn take_request(&self, _id: &str) -> Result<(), String> {
            self.taken
                .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |n| {
                    (n < self.allowed).then_some(n + 1)
                })
                .map(|_| ())
                .map_err(|_| "its daily limit is used up".to_owned())
        }
        fn park(&self, _id: &str) {}
    }

    fn metered(allowed: usize) -> Arc<Metered> {
        Arc::new(Metered {
            inner: Scripted(MockCompletionModel::new([
                MockTurn::tool_call("c1", "find_objects", json!({"query": "cup"})),
                MockTurn::text("The cup is in the kitchen."),
            ])),
            allowed,
            taken: std::sync::atomic::AtomicUsize::new(0),
        })
    }

    async fn ask_where_the_cup_is(source: &Arc<Metered>) -> Vec<Event> {
        let session = Session::start(
            Arc::clone(source) as Arc<dyn AgentSource>,
            registry(Risk::Observe),
            Arc::new(Guard::new(Policy::default())),
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("Where is the cup?".into()));
        collect_until_finished(&mut rx, |_| None, &session).await
    }

    #[tokio::test]
    async fn every_request_of_a_turn_counts_against_the_quota() {
        let source = metered(10);
        let events = ask_where_the_cup_is(&source).await;
        assert!(events.iter().any(|e| matches!(e, Event::Reply { .. })));
        assert_eq!(
            source.taken.load(Ordering::SeqCst),
            2,
            "the tool call and the reply are two requests"
        );
        let costed = events
            .iter()
            .filter(|e| matches!(e, Event::ModelCall { model, turn: 1, .. } if model == "mock"))
            .count();
        assert_eq!(costed, 2, "each request's cost is logged: {events:?}");
    }

    #[tokio::test]
    async fn a_spent_quota_ends_the_turn() {
        let source = metered(1);
        let events = ask_where_the_cup_is(&source).await;
        assert!(!events.iter().any(|e| matches!(e, Event::Reply { .. })));
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::Error { text, .. } if text.contains("daily limit"))),
            "{events:?}"
        );
        assert_eq!(source.taken.load(Ordering::SeqCst), 1);
    }

    #[tokio::test]
    async fn a_synchronous_act_leaves_the_tools_offered_to_check_its_effect() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "find_objects", json!({})),
            MockTurn::text("Done."),
        ]);
        let guard = Arc::new(Guard::new(Policy::default()));
        guard.set_armed(true);
        let session = Session::start(
            Arc::new(Scripted(model.clone())),
            registry(Risk::Motion),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("reset it".into()));
        let approve = |e: &Event| match e {
            Event::ApprovalRequested { id, .. } => Some(Command::Approve(*id)),
            _ => None,
        };
        collect_until_finished(&mut rx, approve, &session).await;
        let offered: Vec<usize> = model.requests().iter().map(|r| r.tools.len()).collect();
        assert_eq!(offered, [1, 1]);
    }

    #[tokio::test]
    async fn once_a_mission_has_started_the_model_is_offered_no_tools() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "find_objects", json!({})),
            MockTurn::text("Started."),
        ]);
        let guard = Arc::new(Guard::new(Policy::default()));
        guard.set_armed(true);
        let mut r = Registry::default();
        let spec = ToolSpec::new(
            "find_objects",
            "Starts.",
            json!({"type": "object"}),
            Risk::Motion,
        );
        r.add(Arc::new(Starter(spec))).unwrap();
        let session = Session::start(
            Arc::new(Scripted(model.clone())),
            Arc::new(r),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let approve = |e: &Event| match e {
            Event::ApprovalRequested { id, .. } => Some(Command::Approve(*id)),
            _ => None,
        };
        let events = collect_until_finished(&mut rx, approve, &session).await;
        assert!(events.iter().any(|e| matches!(e, Event::Reply { .. })));
        let offered: Vec<usize> = model.requests().iter().map(|r| r.tools.len()).collect();
        assert_eq!(
            offered,
            [1, 0],
            "the reply after the act is asked for without tools"
        );
    }

    #[tokio::test]
    async fn an_invented_tool_name_gets_the_real_ones_back() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "default_api", json!({})),
            MockTurn::text("The cup is in the kitchen."),
        ]);
        let session = Session::start(
            Arc::new(Scripted(model.clone())),
            registry(Risk::Observe),
            Arc::new(Guard::new(Policy::default())),
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("Where is the cup?".into()));
        let events = collect_until_finished(&mut rx, |_| None, &session).await;
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::Reply { text, .. } if text.contains("kitchen"))),
            "{events:?}"
        );
        let feedback = format!("{:?}", model.requests()[1].chat_history);
        assert!(feedback.contains("find_objects"), "{feedback}");
    }

    struct Echo(ToolSpec);

    #[async_trait]
    impl Tool for Echo {
        fn spec(&self) -> Cow<'_, ToolSpec> {
            Cow::Borrowed(&self.0)
        }
        async fn call(&self, args: Value) -> ToolOutcome {
            ToolOutcome::ok(json!({"echo": args}))
        }
    }

    /// Starts something in the background, as `run_mission` does.
    struct Starter(ToolSpec);

    #[async_trait]
    impl Tool for Starter {
        fn spec(&self) -> Cow<'_, ToolSpec> {
            Cow::Borrowed(&self.0)
        }
        async fn call(&self, _args: Value) -> ToolOutcome {
            ToolOutcome {
                status: Status::Accepted,
                ..ToolOutcome::ok(json!({"mission_id": "m1"}))
            }
        }
    }

    fn registry(risk: Risk) -> Arc<Registry> {
        let mut r = Registry::default();
        let spec = ToolSpec::new(
            "find_objects",
            "Finds things.",
            json!({"type": "object"}),
            risk,
        );
        r.add(Arc::new(Echo(spec))).unwrap();
        Arc::new(r)
    }

    async fn collect_until_finished(
        rx: &mut broadcast::Receiver<Event>,
        on: impl Fn(&Event) -> Option<Command>,
        session: &Session,
    ) -> Vec<Event> {
        let mut out = Vec::new();
        loop {
            let e = tokio::time::timeout(Duration::from_secs(5), rx.recv())
                .await
                .unwrap()
                .unwrap();
            if let Some(cmd) = on(&e) {
                session.send(cmd);
            }
            let done = matches!(e, Event::TurnFinished { .. });
            out.push(e);
            if done {
                return out;
            }
        }
    }

    #[tokio::test]
    async fn a_turn_calls_a_tool_and_replies() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "find_objects", json!({"query": "cup"})),
            MockTurn::text("The cup is in the kitchen."),
        ]);
        let guard = Arc::new(Guard::new(Policy::default()));
        let session = Session::start(
            Arc::new(Scripted(model)),
            registry(Risk::Observe),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("Where is the cup?".into()));
        let events = collect_until_finished(&mut rx, |_| None, &session).await;
        assert!(events.iter().any(|e| matches!(e, Event::ToolFinished { tool, status: Status::Succeeded, .. } if tool == "find_objects")));
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::Reply { text, .. } if text.contains("kitchen")))
        );
    }

    /// A source whose model streams.
    struct Streaming(MockCompletionModel);

    impl AgentSource for Streaming {
        fn candidates(&self, _role: Role, _need: Need) -> Vec<String> {
            vec!["mock".into()]
        }
        fn builder(&self, _id: &str) -> Result<AgentBuilder, LlmError> {
            Ok(AgentBuilder::new(self.0.clone()))
        }
        fn take_request(&self, _id: &str) -> Result<(), String> {
            Ok(())
        }
        fn park(&self, _id: &str) {}
        fn streams(&self, _id: &str) -> bool {
            true
        }
    }

    #[tokio::test]
    async fn a_streamed_reply_arrives_in_pieces_then_whole() {
        use rig::test_utils::MockStreamEvent;
        let model = MockCompletionModel::from_stream_turns([vec![
            MockStreamEvent::text("The cup "),
            MockStreamEvent::text("is in the kitchen."),
            MockStreamEvent::final_response_with_default_usage(),
        ]]);
        let guard = Arc::new(Guard::new(Policy::default()));
        let session = Session::start(
            Arc::new(Streaming(model)),
            registry(Risk::Observe),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("Where is the cup?".into()));
        let events = collect_until_finished(&mut rx, |_| None, &session).await;
        let pieces: Vec<&str> = events
            .iter()
            .filter_map(|e| match e {
                Event::ReplyDelta { text, .. } => Some(text.as_str()),
                _ => None,
            })
            .collect();
        assert_eq!(pieces, ["The cup ", "is in the kitchen."]);
        assert!(events.iter().any(
            |e| matches!(e, Event::Reply { text, .. } if text == "The cup is in the kitchen.")
        ));
    }

    #[tokio::test]
    async fn the_operator_compacts_and_resumes_a_conversation() {
        let model = MockCompletionModel::new([MockTurn::text("They asked for ten things.")]);
        let config = SessionConfig {
            resume: Some(History::sample(10, 3000)),
            ..SessionConfig::default()
        };
        let guard = Arc::new(Guard::new(Policy::default()));
        let session = Session::start(
            Arc::new(Scripted(model)),
            registry(Risk::Observe),
            guard,
            None,
            config,
        );
        let mut rx = session.subscribe();
        session.send(Command::Compact);
        let events = collect_until_finished(&mut rx, |_| None, &session).await;
        assert!(
            events.iter().any(|e| matches!(e, Event::Compacted { summarised: true, before, after } if after < before)),
            "{events:?}"
        );
        session.send(Command::CompactUpTo { keep: 1 });
        let events = collect_until_finished(&mut rx, |_| None, &session).await;
        assert!(
            events.iter().any(|e| matches!(
                e,
                Event::Compacted {
                    summarised: false,
                    ..
                }
            )),
            "a second summary needs a model turn the script lacks, so it only cuts: {events:?}"
        );
        session.send(Command::Restore(History::sample(2, 10)));
        let restored = tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .unwrap()
            .unwrap();
        assert!(matches!(restored, Event::Restored { exchanges } if exchanges.len() == 4));
    }

    struct Slow(ToolSpec, Duration);

    #[async_trait]
    impl Tool for Slow {
        fn spec(&self) -> Cow<'_, ToolSpec> {
            Cow::Borrowed(&self.0)
        }
        async fn call(&self, _args: Value) -> ToolOutcome {
            tokio::time::sleep(self.1).await;
            ToolOutcome::ok(json!({}))
        }
    }

    #[tokio::test]
    async fn a_message_sent_while_a_report_is_answered_runs_after_it() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "find_objects", json!({})),
            MockTurn::text("noted"),
            MockTurn::text("hello"),
        ]);
        let mut r = Registry::default();
        let spec = ToolSpec::new(
            "find_objects",
            "Slow.",
            json!({"type": "object"}),
            Risk::Observe,
        );
        r.add(Arc::new(Slow(spec, Duration::from_millis(300))))
            .unwrap();
        let guard = Arc::new(Guard::new(Policy::default()));
        let session = Session::start(
            Arc::new(Scripted(model)),
            Arc::new(r),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::Report("the camera slowed".into()));
        let mut events = Vec::new();
        let mut finished = 0;
        while finished < 2 {
            let e = tokio::time::timeout(Duration::from_secs(5), rx.recv())
                .await
                .unwrap()
                .unwrap();
            if matches!(e, Event::ToolStarted { .. }) {
                session.send(Command::User("hi".into()));
            }
            finished += usize::from(matches!(e, Event::TurnFinished { .. }));
            events.push(e);
        }
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::Notice { text } if text.starts_with("queued")))
        );
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::User { turn: 2, text } if text == "hi"))
        );
        assert!(
            matches!(events.iter().rev().nth(1), Some(Event::Reply { text, .. }) if text == "hello")
        );
    }

    #[tokio::test]
    async fn a_turn_over_its_time_limit_is_stopped() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "find_objects", json!({})),
            MockTurn::text("done"),
        ]);
        let mut r = Registry::default();
        let spec = ToolSpec::new(
            "find_objects",
            "Slow.",
            json!({"type": "object"}),
            Risk::Observe,
        );
        r.add(Arc::new(Slow(spec, Duration::from_secs(5)))).unwrap();
        let config = SessionConfig {
            turn_time: Duration::from_millis(200),
            ..SessionConfig::default()
        };
        let guard = Arc::new(Guard::new(Policy::default()));
        let session = Session::start(Arc::new(Scripted(model)), Arc::new(r), guard, None, config);
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let events = collect_until_finished(&mut rx, |_| None, &session).await;
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::Error { text, .. } if text.contains("longer than")))
        );
        assert!(!events.iter().any(|e| matches!(e, Event::Reply { .. })));
    }

    /// Two calls of the one tool in one turn, the second with other arguments.
    fn twice() -> MockCompletionModel {
        MockCompletionModel::new([
            MockTurn::tool_call("c1", "find_objects", json!({"n": 1})),
            MockTurn::tool_call("c2", "find_objects", json!({"n": 2})),
            MockTurn::text("done"),
        ])
    }

    #[tokio::test]
    async fn a_tool_allowed_for_the_session_stops_asking_unless_it_moves_the_robot() {
        for (risk, asked) in [(Risk::WorldEdit, 1), (Risk::Motion, 2)] {
            let guard = Arc::new(Guard::new(Policy::default()));
            guard.set_armed(true);
            let session = Session::start(
                Arc::new(Scripted(twice())),
                registry(risk),
                guard,
                None,
                SessionConfig::default(),
            );
            let mut rx = session.subscribe();
            session.send(Command::User("go".into()));
            let allow = |e: &Event| match e {
                Event::ApprovalRequested { id, .. } => Some(Command::AllowForSession(*id)),
                _ => None,
            };
            let events = collect_until_finished(&mut rx, allow, &session).await;
            let n = events
                .iter()
                .filter(|e| matches!(e, Event::ApprovalRequested { .. }))
                .count();
            assert_eq!(n, asked, "{risk:?}: {events:?}");
            let ran = events
                .iter()
                .filter(|e| {
                    matches!(
                        e,
                        Event::ToolFinished {
                            status: Status::Succeeded,
                            ..
                        }
                    )
                })
                .count();
            assert_eq!(ran, 2, "{risk:?}");
        }
    }

    #[tokio::test]
    async fn a_request_left_waiting_is_kept_for_the_next_session_and_dropped_once_answered() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("pending.json");
        let guard = Arc::new(Guard::new(Policy::default()));
        guard.set_armed(true);
        let config = SessionConfig {
            pending_file: Some(file.clone()),
            ..SessionConfig::default()
        };
        let session = Session::start(
            Arc::new(Scripted(twice())),
            registry(Risk::WorldEdit),
            guard,
            None,
            config,
        );
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let id = loop {
            if let Event::ApprovalRequested { id, .. } = rx.recv().await.unwrap() {
                break id;
            }
        };
        let kept: Vec<Unanswered> = serde_json::from_slice(&std::fs::read(&file).unwrap()).unwrap();
        assert_eq!(kept.len(), 1);
        assert_eq!(
            (kept[0].tool.as_str(), &kept[0].args),
            ("find_objects", &json!({"n": 1}))
        );
        session.send(Command::Deny(id));
        let deny = |e: &Event| match e {
            Event::ApprovalRequested { id, .. } => Some(Command::Deny(*id)),
            _ => None,
        };
        collect_until_finished(&mut rx, deny, &session).await;
        assert!(!file.exists(), "answered, nothing waits");

        std::fs::write(&file, serde_json::to_vec(&kept).unwrap()).unwrap();
        assert_eq!(take_unanswered(&file), kept);
        assert!(take_unanswered(&file).is_empty(), "offered once");
    }

    #[tokio::test]
    async fn act_tools_are_refused_while_disarmed_and_run_after_approval() {
        let script = || {
            MockCompletionModel::new([
                MockTurn::tool_call("c1", "find_objects", json!({})),
                MockTurn::text("done"),
            ])
        };
        let guard = Arc::new(Guard::new(Policy::default()));
        let session = Session::start(
            Arc::new(Scripted(script())),
            registry(Risk::Motion),
            Arc::clone(&guard),
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let events = collect_until_finished(&mut rx, |_| None, &session).await;
        assert!(events.iter().any(|e| matches!(
            e,
            Event::ToolFinished {
                status: Status::Refused,
                ..
            }
        )));

        guard.set_armed(true);
        let session = Session::start(
            Arc::new(Scripted(script())),
            registry(Risk::Motion),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let approve = |e: &Event| match e {
            Event::ApprovalRequested { id, .. } => Some(Command::Approve(*id)),
            _ => None,
        };
        let events = collect_until_finished(&mut rx, approve, &session).await;
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::ApprovalResolved { approved: true, .. }))
        );
        assert!(events.iter().any(|e| matches!(
            e,
            Event::ToolFinished {
                status: Status::Succeeded,
                ..
            }
        )));
    }

    #[tokio::test]
    async fn stop_ends_the_turn_and_denies_pending_approvals() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "find_objects", json!({})),
            MockTurn::text("done"),
        ]);
        let guard = Arc::new(Guard::new(Policy {
            start_armed: true,
            ..Policy::default()
        }));
        let session = Session::start(
            Arc::new(Scripted(model)),
            registry(Risk::Motion),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let stop = |e: &Event| {
            matches!(e, Event::ApprovalRequested { .. }).then_some(Command::StopMission)
        };
        let events = collect_until_finished(&mut rx, stop, &session).await;
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::TurnFinished { .. }))
        );
        let halted = tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                if let Ok(Event::Halted { .. }) = rx.recv().await {
                    return;
                }
            }
        });
        assert!(halted.await.is_ok() || events.iter().any(|e| matches!(e, Event::Halted { .. })));
    }

    /// Events after the ones already read, until `done` holds for one or a second passes.
    async fn more_until(
        rx: &mut broadcast::Receiver<Event>,
        done: impl Fn(&[Event]) -> bool,
    ) -> Vec<Event> {
        let mut out = Vec::new();
        let _ = tokio::time::timeout(Duration::from_secs(1), async {
            while let Ok(e) = rx.recv().await {
                out.push(e);
                if done(&out) {
                    return;
                }
            }
        })
        .await;
        out
    }

    #[test]
    fn a_plain_order_to_stop_is_known_and_a_sentence_about_stopping_is_not() {
        for word in [
            "stop",
            "STOP",
            "Stop!",
            "halt.",
            " freeze ",
            "please stop",
            "stop the robot!",
        ] {
            assert!(is_stop_word(word), "{word}");
        }
        for text in ["stop at the kitchen", "don't stop", "bus stop", "stopwatch"] {
            assert!(!is_stop_word(text), "{text}");
        }
    }

    #[tokio::test]
    async fn a_stop_closes_what_it_cut_short_and_forgets_the_request() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "find_objects", json!({})),
            MockTurn::text("done"),
        ]);
        let guard = Arc::new(Guard::new(Policy {
            start_armed: true,
            ..Policy::default()
        }));
        let dir = tempfile::tempdir().unwrap();
        let pending = dir.path().join("pending.json");
        let session = Session::start(
            Arc::new(Scripted(model)),
            registry(Risk::Motion),
            guard,
            None,
            SessionConfig {
                pending_file: Some(pending.clone()),
                ..SessionConfig::default()
            },
        );
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let stop = |e: &Event| {
            matches!(e, Event::ApprovalRequested { .. }).then_some(Command::StopMission)
        };
        let mut events = collect_until_finished(&mut rx, stop, &session).await;
        let closed = |events: &[Event]| {
            events.iter().any(|e| {
                matches!(
                    e,
                    Event::ApprovalResolved {
                        approved: false,
                        ..
                    }
                )
            }) && events.iter().any(|e| {
                matches!(
                    e,
                    Event::ToolFinished {
                        status: Status::Stopped,
                        ..
                    }
                )
            })
        };
        if !closed(&events) {
            events.extend(more_until(&mut rx, closed).await);
        }
        assert!(closed(&events), "{events:?}");
        assert!(!pending.exists(), "a stopped request is not asked again");
    }

    #[tokio::test]
    async fn disarming_turns_down_a_waiting_approval_however_it_is_answered() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "find_objects", json!({})),
            MockTurn::text("not moving"),
        ]);
        let guard = Arc::new(Guard::new(Policy {
            start_armed: true,
            ..Policy::default()
        }));
        let session = Session::start(
            Arc::new(Scripted(model)),
            registry(Risk::Motion),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let mut events = Vec::new();
        loop {
            let e = tokio::time::timeout(Duration::from_secs(5), rx.recv())
                .await
                .unwrap()
                .unwrap();
            if let Event::ApprovalRequested { id, .. } = &e {
                // Off, then approved: the approval comes too late to move anything.
                session.send(Command::Disarm);
                session.send(Command::Approve(*id));
            }
            let done = matches!(e, Event::TurnFinished { .. });
            events.push(e);
            if done {
                break;
            }
        }
        let status = events.iter().find_map(|e| match e {
            Event::ToolFinished { status, .. } => Some(*status),
            _ => None,
        });
        assert_eq!(status, Some(Status::Refused), "{events:?}");
    }

    #[tokio::test]
    async fn the_model_can_always_stop_the_robot() {
        let stop_spec = ToolSpec::new("stop", "Stops.", json!({"type": "object"}), Risk::Observe);
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "find_objects", json!({})),
            MockTurn::tool_call("c2", "stop", json!({"reason": null})),
            MockTurn::tool_call("c3", "stop", json!({"reason": null})),
            MockTurn::text("stopped"),
        ]);
        let guard = Arc::new(Guard::new(Policy {
            budgets: crate::guard::Budgets {
                tool_calls: 1,
                repeat_break: 2,
                ..crate::guard::Budgets::default()
            },
            ..Policy::default()
        }));
        let stop: Arc<dyn Tool> = Arc::new(Echo(stop_spec));
        let mut r = Registry::default();
        r.add(Arc::new(Echo(ToolSpec::new(
            "find_objects",
            "Finds things.",
            json!({"type": "object"}),
            Risk::Observe,
        ))))
        .unwrap();
        r.add(Arc::clone(&stop)).unwrap();
        let session = Session::start(
            Arc::new(Scripted(model)),
            Arc::new(r),
            Arc::clone(&guard),
            Some(stop),
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("look, then stop".into()));
        let events = collect_until_finished(&mut rx, |_| None, &session).await;
        let stops: Vec<Status> = events
            .iter()
            .filter_map(|e| match e {
                Event::ToolFinished { tool, status, .. } if tool == "stop" => Some(*status),
                _ => None,
            })
            .collect();
        assert_eq!(
            stops,
            [Status::Succeeded, Status::Succeeded],
            "past the budget, twice the same"
        );
        assert_eq!(guard.stops(), 2);
    }

    #[tokio::test]
    async fn a_reply_lost_after_acting_keeps_the_request_in_the_history() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "run_mission", json!({})),
            MockTurn::error("the provider is down"),
            MockTurn::text("It is still running."),
        ]);
        let spec = ToolSpec::new(
            "run_mission",
            "Runs.",
            json!({"type": "object"}),
            Risk::Motion,
        );
        let guard = Arc::new(Guard::new(Policy {
            start_armed: true,
            autonomy: crate::guard::Autonomy::Autonomous,
            ..Policy::default()
        }));
        let session = Session::start(
            Arc::new(Scripted(model.clone())),
            one_tool(Arc::new(Starter(spec))),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("bring me the cup".into()));
        collect_until_finished(&mut rx, |_| None, &session).await;
        session.send(Command::User("is it done?".into()));
        collect_until_finished(&mut rx, |_| None, &session).await;
        let seen = format!("{:?}", model.requests().last().unwrap().chat_history);
        assert!(seen.contains("bring me the cup"), "{seen}");
        assert!(seen.contains("run_mission accepted"), "{seen}");
    }

    /// Checks an edit as `run_mission` checks an edited plan: `{"ok": true}` passes as a new hash.
    struct Editable(ToolSpec);

    #[async_trait]
    impl Tool for Editable {
        fn spec(&self) -> Cow<'_, ToolSpec> {
            Cow::Borrowed(&self.0)
        }
        async fn assess_operator(&self, edited: Value) -> Option<Result<Assessment, ToolOutcome>> {
            if edited["slow"] == true {
                tokio::time::sleep(Duration::from_millis(200)).await;
            }
            Some(if edited["ok"] == true {
                Ok(Assessment {
                    risk: Risk::Motion,
                    resources: Vec::new(),
                    reason: "runs the edited plan".into(),
                    args: Some(json!({"hash": "edited"})),
                })
            } else {
                Err(ToolOutcome::failed("s1: no such place"))
            })
        }
        async fn call(&self, args: Value) -> ToolOutcome {
            ToolOutcome::ok(json!({"ran": args}))
        }
    }

    #[tokio::test]
    async fn an_edit_that_fails_its_checks_keeps_the_request_and_one_that_passes_replaces_it() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "run_mission", json!({"hash": "planned"})),
            MockTurn::text("Done."),
        ]);
        let guard = Arc::new(Guard::new(Policy::default()));
        guard.set_armed(true);
        let mut r = Registry::default();
        let spec = ToolSpec::new(
            "run_mission",
            "Runs.",
            json!({"type": "object"}),
            Risk::Motion,
        );
        r.add(Arc::new(Editable(spec))).unwrap();
        let session = Session::start(
            Arc::new(Scripted(model.clone())),
            Arc::new(r),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let operator = |e: &Event| match e {
            Event::ApprovalRequested { id, .. } => Some(Command::Edit {
                id: *id,
                args: json!({"ok": false}),
            }),
            Event::EditRejected { id, .. } => Some(Command::Edit {
                id: *id,
                args: json!({"ok": true}),
            }),
            Event::ApprovalEdited { id, .. } => Some(Command::Approve(*id)),
            _ => None,
        };
        let events = collect_until_finished(&mut rx, operator, &session).await;
        assert!(
            events.iter().any(|e| matches!(e, Event::EditRejected { message, .. } if message == "s1: no such place")),
            "{events:?}"
        );
        assert!(
            events.iter().any(
                |e| matches!(e, Event::ApprovalEdited { args, .. } if args["hash"] == "edited")
            ),
            "{events:?}"
        );
        let resolved: Vec<bool> = events
            .iter()
            .filter_map(|e| match e {
                Event::ApprovalResolved { approved, .. } => Some(*approved),
                _ => None,
            })
            .collect();
        assert_eq!(resolved, [true], "one request, answered once");
        let ran = format!("{:?}", model.requests()[1].chat_history);
        assert!(ran.contains("edited"), "it ran with the edit: {ran}");
    }

    #[tokio::test]
    async fn an_approval_sent_while_an_edit_is_checked_does_not_approve_the_edit() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "run_mission", json!({"hash": "planned"})),
            MockTurn::text("Not run."),
        ]);
        let guard = Arc::new(Guard::new(Policy::default()));
        guard.set_armed(true);
        let mut r = Registry::default();
        let spec = ToolSpec::new(
            "run_mission",
            "Runs.",
            json!({"type": "object"}),
            Risk::Motion,
        );
        r.add(Arc::new(Editable(spec))).unwrap();
        let session = Session::start(
            Arc::new(Scripted(model)),
            Arc::new(r),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let mut resolved = Vec::new();
        loop {
            let e = tokio::time::timeout(Duration::from_secs(5), rx.recv())
                .await
                .unwrap()
                .unwrap();
            match e {
                Event::ApprovalRequested { id, .. } => {
                    let args = json!({"ok": true, "slow": true});
                    session.send(Command::Edit { id, args });
                    session.send(Command::Approve(id));
                }
                // Not approved after the edit: nothing may run.
                Event::ApprovalEdited { id, .. } => session.send(Command::Deny(id)),
                Event::ApprovalResolved { approved, .. } => resolved.push(approved),
                Event::TurnFinished { .. } => break,
                _ => {}
            }
        }
        assert_eq!(resolved, [false]);
    }

    #[tokio::test]
    async fn the_operator_runs_a_tool_checked_as_their_own_and_approves_it() {
        let guard = Arc::new(Guard::new(Policy::default()));
        let mut r = Registry::default();
        let spec = ToolSpec::new(
            "run_mission",
            "Runs.",
            json!({"type": "object"}),
            Risk::Motion,
        );
        r.add(Arc::new(Editable(spec))).unwrap();
        let session = Session::start(
            Arc::new(Scripted(MockCompletionModel::new([]))),
            Arc::new(r),
            Arc::clone(&guard),
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        let run = || Command::Run {
            tool: "run_mission".into(),
            args: json!({"ok": true}),
        };
        let mut next = async || {
            tokio::time::timeout(Duration::from_secs(5), rx.recv())
                .await
                .unwrap()
                .unwrap()
        };

        session.send(run());
        let refused = loop {
            if let Event::ToolFinished {
                status, message, ..
            } = next().await
            {
                break (status, message);
            }
        };
        assert_eq!(refused.0, Status::Refused);
        assert!(refused.1.contains("arm it"), "{}", refused.1);

        guard.set_armed(true);
        session.send(run());
        loop {
            match next().await {
                Event::ToolStarted { args, .. } => {
                    assert_eq!(args["hash"], "edited", "run as checked");
                }
                Event::ApprovalRequested { id, .. } => session.send(Command::Approve(id)),
                Event::ToolFinished { status, .. } => {
                    assert_eq!(status, Status::Succeeded);
                    break;
                }
                _ => {}
            }
        }
    }

    /// Fails the same way every time, whatever it is asked.
    struct Broken(ToolSpec);

    #[async_trait]
    impl Tool for Broken {
        fn spec(&self) -> Cow<'_, ToolSpec> {
            Cow::Borrowed(&self.0)
        }
        async fn call(&self, _args: Value) -> ToolOutcome {
            ToolOutcome::failed("the camera is not publishing")
        }
    }

    fn one_tool(tool: Arc<dyn Tool>) -> Arc<Registry> {
        let mut r = Registry::default();
        r.add(tool).unwrap();
        Arc::new(r)
    }

    #[tokio::test]
    async fn after_three_denials_in_a_row_the_model_is_told_to_stop_asking() {
        let calls: Vec<MockTurn> = (1..=4)
            .map(|n| MockTurn::tool_call(format!("c{n}"), "find_objects", json!({"n": n})))
            .chain([MockTurn::text("I will ask what you want.")])
            .collect();
        let model = MockCompletionModel::new(calls);
        let guard = Arc::new(Guard::new(Policy::default()));
        guard.set_armed(true);
        let session = Session::start(
            Arc::new(Scripted(model.clone())),
            registry(Risk::Motion),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let deny = |e: &Event| match e {
            Event::ApprovalRequested { id, .. } => Some(Command::Deny(*id)),
            _ => None,
        };
        let events = collect_until_finished(&mut rx, deny, &session).await;
        let asked = events
            .iter()
            .filter(|e| matches!(e, Event::ApprovalRequested { .. }))
            .count();
        assert_eq!(asked, 3, "the fourth is not asked");
        let seen = format!("{:?}", model.requests().last().unwrap().chat_history);
        assert!(seen.contains("3 times in a row"), "{seen}");
        assert!(seen.contains("turned down the last 3 requests"), "{seen}");
    }

    #[tokio::test]
    async fn the_same_failure_three_times_in_a_row_is_called_stuck() {
        let calls: Vec<MockTurn> = (1..=3)
            .map(|n| MockTurn::tool_call(format!("c{n}"), "look", json!({"n": n})))
            .chain([MockTurn::text("The camera is stuck.")])
            .collect();
        let model = MockCompletionModel::new(calls);
        let spec = ToolSpec::new("look", "Looks.", json!({"type": "object"}), Risk::Observe);
        let session = Session::start(
            Arc::new(Scripted(model.clone())),
            one_tool(Arc::new(Broken(spec))),
            Arc::new(Guard::new(Policy::default())),
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("look".into()));
        collect_until_finished(&mut rx, |_| None, &session).await;
        let requests = model.requests();
        let second = format!("{:?}", requests[2].chat_history);
        assert!(!second.contains("came back the same"), "twice is not stuck");
        let third = format!("{:?}", requests[3].chat_history);
        assert!(
            third.contains("came back the same 3 times in a row"),
            "{third}"
        );
    }

    #[tokio::test]
    async fn a_message_sent_mid_turn_reaches_the_model_with_its_next_tool_result() {
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "look", json!({})),
            MockTurn::text("Looking at the kitchen too."),
        ]);
        let spec = ToolSpec::new("look", "Looks.", json!({"type": "object"}), Risk::Observe);
        let slow = Slow(spec, Duration::from_millis(300));
        let session = Session::start(
            Arc::new(Scripted(model.clone())),
            one_tool(Arc::new(slow)),
            Arc::new(Guard::new(Policy::default())),
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("look around".into()));
        let mut steered = false;
        loop {
            let e = tokio::time::timeout(Duration::from_secs(5), rx.recv())
                .await
                .unwrap()
                .unwrap();
            match e {
                Event::ToolStarted { .. } => session.send(Command::User("and the kitchen".into())),
                Event::Steer { text, .. } => steered = text == "and the kitchen",
                Event::TurnFinished { .. } => break,
                _ => {}
            }
        }
        assert!(steered, "shown as said during the turn");
        let read = format!("{:?}", model.requests()[1].chat_history);
        assert!(
            read.contains("said while you worked: \\\"and the kitchen\\\""),
            "{read}"
        );
        assert_eq!(model.requests().len(), 2, "no turn of its own: it was read");
    }

    #[tokio::test]
    async fn a_profile_rule_refuses_one_value_and_asks_before_another() {
        let policy: crate::guard::Policy = toml::from_str(
            r#"
            [[rule]]
            tool = "find_objects"
            arg = "query"
            matches = "*knife*"
            then = "deny"
            reason = "do not look for knives"
            [[rule]]
            tool = "find_objects"
            arg = "query"
            matches = "*bedroom*"
            then = "ask"
            reason = "the bedroom is private"
            "#,
        )
        .unwrap();
        let model = MockCompletionModel::new([
            MockTurn::tool_call("c1", "find_objects", json!({"query": "the knife"})),
            MockTurn::tool_call("c2", "find_objects", json!({"query": "the bedroom lamp"})),
            MockTurn::text("Done."),
        ]);
        let session = Session::start(
            Arc::new(Scripted(model.clone())),
            registry(Risk::Observe),
            Arc::new(Guard::new(policy)),
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("find things".into()));
        let approve = |e: &Event| match e {
            Event::ApprovalRequested { id, .. } => Some(Command::Approve(*id)),
            _ => None,
        };
        let events = collect_until_finished(&mut rx, approve, &session).await;
        let finished: Vec<(Status, &str)> = events
            .iter()
            .filter_map(|e| match e {
                Event::ToolFinished {
                    status, message, ..
                } => Some((*status, message.as_str())),
                _ => None,
            })
            .collect();
        assert_eq!(finished[0], (Status::Refused, "do not look for knives"));
        assert_eq!(finished[1].0, Status::Succeeded, "asked, then run");
        assert!(events.iter().any(
            |e| matches!(e, Event::ApprovalRequested { reason, .. } if reason == "the bedroom is private")
        ));
    }
}
