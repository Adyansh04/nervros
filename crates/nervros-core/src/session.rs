//! The session: one conversation with one robot.
//!
//! An actor owns the conversation. User text starts a turn, which runs as its own task so that stop
//! and approval commands are always handled at once: stop aborts the turn and calls the robot's
//! `StopAll` without asking any model. Each turn tries the routine role's models in order; a model
//! that fails before any act-lane tool ran is replaced by the next one, and after one did, the turn
//! ends instead of repeating an action. Every tool call passes the guard.

use std::borrow::Cow;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use serde::Serialize;
use serde_json::{Value, json};
use tokio::sync::{broadcast, mpsc, oneshot};
use tokio::task::JoinHandle;

use crate::guard::{Decision, Guard};
use crate::llm::{self, AgentSource, History, LoopTool};
use crate::mission::plan::PlannedStep;
use crate::providers::Role;
use crate::providers::router::Need;
use crate::tools::{Lane, Registry, Resource, Status, Tool, ToolOutcome};

/// What a UI or the CLI asks the session to do.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    /// A message from the operator.
    User(String),
    /// Stop the model's reply; the robot is not touched.
    StopGeneration,
    /// Stop the reply and every motion, through the robot's `StopAll`.
    StopMission,
    /// Approve a pending request.
    Approve(u64),
    /// Refuse a pending request.
    Deny(u64),
    /// Enable act-lane tools.
    Arm,
    /// Disable act-lane tools.
    Disarm,
    /// Something the robot reports, such as a finished mission; the model answers it in a turn
    /// of its own, after any running turn.
    Report(String),
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
        /// `succeeded`, `failed`, `refused` or `accepted`.
        status: &'static str,
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
    },
    /// The operator must approve a call.
    ApprovalRequested {
        /// Answer with `Approve(id)` or `Deny(id)`.
        id: u64,
        /// The tool.
        tool: String,
        /// Its arguments.
        args: Value,
        /// Why approval is needed.
        reason: String,
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
        }
    }
}

struct Shared {
    events: broadcast::Sender<Event>,
    approvals: Mutex<HashMap<u64, oneshot::Sender<bool>>>,
    calls: AtomicU64,
    next_approval: AtomicU64,
    /// Milliseconds this turn spent waiting for the operator.
    waited_ms: AtomicU64,
    guard: Arc<Guard>,
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

    async fn ask_approval(&self, tool: &str, args: &Value, reason: String) -> bool {
        let id = self.next_approval.fetch_add(1, Ordering::Relaxed) + 1;
        let (tx, rx) = oneshot::channel();
        lock(&self.approvals).insert(id, tx);
        self.emit(Event::ApprovalRequested {
            id,
            tool: tool.to_owned(),
            args: args.clone(),
            reason,
        });
        let asked = Instant::now();
        let approved = matches!(
            tokio::time::timeout(self.config.approval_ttl, rx).await,
            Ok(Ok(true))
        );
        let waited = u64::try_from(asked.elapsed().as_millis()).unwrap_or(u64::MAX);
        self.waited_ms.fetch_add(waited, Ordering::Relaxed);
        lock(&self.approvals).remove(&id);
        self.emit(Event::ApprovalResolved { id, approved });
        approved
    }

    fn resolve(&self, id: u64, approved: bool) {
        if let Some(tx) = lock(&self.approvals).remove(&id) {
            let _ = tx.send(approved);
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

    async fn invoke(
        &self,
        tool: &Arc<dyn Tool>,
        turn: u64,
        args: Value,
        flags: &TurnFlags,
    ) -> Value {
        let call = self.calls.fetch_add(1, Ordering::Relaxed) + 1;
        // A tool whose risk depends on its arguments says what this call would do, and a call
        // that cannot go out fails here, before anyone is asked to approve it.
        let (assessment, early) = match tool.assess(&args).await {
            Some(Ok(a)) => (Some(a), None),
            Some(Err(out)) => (None, Some(out)),
            None => (None, None),
        };
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
        let started = Instant::now();
        let outcome = match early {
            Some(out) => out,
            None => match self.guard.decide(&spec, &args) {
                Decision::Deny(r) => ToolOutcome::refused(r.message),
                Decision::NeedApproval { reason } => {
                    let reason = assessment.map_or(reason, |a| a.reason);
                    if self.ask_approval(&spec.name, &args, reason).await {
                        self.run(tool, args, &spec.resources).await
                    } else {
                        ToolOutcome::refused("the operator did not approve this")
                    }
                }
                Decision::Allow => self.run(tool, args, &spec.resources).await,
            },
        };
        // An edit is not repeated by the next model either.
        if spec.lane() != Lane::Observe
            && matches!(outcome.status, Status::Succeeded | Status::Accepted)
        {
            flags.acted.store(true, Ordering::SeqCst);
        }
        // Something now runs that will report back, such as a mission: the rest is the answer.
        if outcome.status == Status::Accepted {
            flags.started.store(true, Ordering::SeqCst);
        }
        let status = match outcome.status {
            Status::Succeeded => "succeeded",
            Status::Failed => "failed",
            Status::Refused => "refused",
            Status::Accepted => "accepted",
        };
        for image in &outcome.images {
            self.emit(Event::Snapshot {
                id: image.snapshot.clone(),
                jpeg: Arc::clone(&image.jpeg),
                width: image.width,
                height: image.height,
            });
        }
        let ms = u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX);
        self.emit(Event::ToolFinished {
            turn,
            call,
            tool: spec.name.clone(),
            status,
            message: outcome.message.clone(),
            ms,
        });
        outcome.for_model(self.config.result_chars)
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
        let (events, _) = broadcast::channel(512);
        let shared = Arc::new(Shared {
            events: events.clone(),
            approvals: Mutex::default(),
            calls: AtomicU64::new(0),
            next_approval: AtomicU64::new(0),
            waited_ms: AtomicU64::new(0),
            guard,
            config,
        });
        tokio::spawn(actor(rx, shared, source, registry, stop));
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

async fn actor(
    mut rx: mpsc::UnboundedReceiver<Command>,
    shared: Arc<Shared>,
    source: Arc<dyn AgentSource>,
    registry: Arc<Registry>,
    stop: Option<Arc<dyn Tool>>,
) {
    let mut history = History::default();
    let mut turns = 0u64;
    let mut running: Option<(u64, JoinHandle<Option<History>>)> = None;
    let mut reports: Vec<String> = Vec::new();
    // Nobody asked for a report's reply, so a message sent during one waits for it, not refused.
    let (mut answering_report, mut queued) = (false, None::<String>);
    let start = |turn: u64, origin: Origin, history: &History| {
        let task = run_turn(
            turn,
            origin,
            history.clone(),
            Arc::clone(&shared),
            Arc::clone(&source),
            Arc::clone(&registry),
        );
        (turn, tokio::spawn(limited(turn, Arc::clone(&shared), task)))
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
            cmd = rx.recv() => {
                let Some(cmd) = cmd else { break };
                match cmd {
                    Command::User(text) => {
                        if running.is_some() {
                            let note = if answering_report && queued.is_none() {
                                queued = Some(text);
                                "queued until the reply to the report ends"
                            } else {
                                "still working on the last message; wait or stop it"
                            };
                            shared.emit(Event::Notice { text: note.into() });
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
                        halt(&shared, stop.as_ref().filter(|_| mission));
                    }
                    Command::Approve(id) => shared.resolve(id, true),
                    Command::Deny(id) => shared.resolve(id, false),
                    Command::Arm | Command::Disarm => {
                        let armed = cmd == Command::Arm;
                        shared.guard.set_armed(armed);
                        shared.emit(Event::Armed { armed });
                    }
                }
            }
            done = finished => {
                if let Some((turn, _)) = running.take() {
                    if let Ok(Some(updated)) = done {
                        history = updated;
                    }
                    shared.emit(Event::TurnFinished { turn });
                }
                if let Some(text) = queued.take() {
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

/// Denies what waits for approval and, given the stop tool, stops the robot too.
fn halt(shared: &Arc<Shared>, stop: Option<&Arc<dyn Tool>>) {
    let pending: Vec<u64> = lock(&shared.approvals).keys().copied().collect();
    for id in pending {
        shared.resolve(id, false);
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
                    Box::pin(async move { shared.invoke(&tool, turn, args, &flags).await })
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
            format!("[Report from the robot, not the operator]\n{text}")
        }
    };
    let flags = Arc::new(TurnFlags::default());
    let tools = loop_tools(&registry, &shared, turn, &flags);
    let need = Need {
        tools: !tools.is_empty(),
        ..Need::default()
    };
    let candidates = source.candidates(Role::Routine, need);
    let mut failures = Vec::new();
    for model in candidates {
        let mut updated = history.clone();
        let setup = llm::TurnSetup {
            preamble: &shared.config.preamble,
            max_turns: shared.config.max_model_calls,
            tools: &tools,
            started: Arc::clone(&flags.started),
        };
        let result = llm::chat(&model, Arc::clone(&source), setup, &mut updated, &text).await;
        match result {
            Ok(reply) => {
                shared.emit(Event::Reply {
                    turn,
                    text: reply,
                    model,
                });
                updated.trim(shared.config.history_max);
                return Some(updated);
            }
            Err(e) if !flags.acted.load(Ordering::SeqCst) => {
                if matches!(
                    e,
                    llm::LlmError::Turn {
                        rate_limited: true,
                        ..
                    }
                ) {
                    source.park(&model);
                }
                shared.emit(Event::Notice {
                    text: format!("{e}; trying the next model"),
                });
                failures.push(e.to_string());
            }
            // What the robot was asked to do goes on, and its report will follow; only the reply
            // is lost, so this is not the operator's error to deal with.
            Err(e) => {
                shared.emit(Event::Notice {
                    text: format!("{e}; the robot already acted, so the turn is not retried"),
                });
                return None;
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
        assert!(events.iter().any(|e| matches!(e, Event::ToolFinished { tool, status: "succeeded", .. } if tool == "find_objects")));
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::Reply { text, .. } if text.contains("kitchen")))
        );
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
                status: "refused",
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
                status: "succeeded",
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
}
