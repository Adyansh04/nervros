//! The session: one conversation with one robot.
//!
//! An actor owns the conversation. User text starts a turn, which runs as its own task so that stop
//! and approval commands are always handled at once: stop aborts the turn and calls the robot's
//! `StopAll` without asking any model. Each turn tries the routine role's models in order; a model
//! that fails before any act-lane tool ran is replaced by the next one, and after one did, the turn
//! ends instead of repeating an action. Every tool call passes the guard.

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
use crate::providers::Role;
use crate::providers::router::Need;
use crate::tools::{Lane, Registry, Status, Tool, ToolOutcome};

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
}

impl Default for SessionConfig {
    fn default() -> Self {
        Self {
            preamble: String::new(),
            max_model_calls: 6,
            history_max: 60,
            result_chars: 6000,
            approval_ttl: Duration::from_mins(1),
        }
    }
}

struct Shared {
    events: broadcast::Sender<Event>,
    approvals: Mutex<HashMap<u64, oneshot::Sender<bool>>>,
    calls: AtomicU64,
    next_approval: AtomicU64,
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
        let approved = matches!(
            tokio::time::timeout(self.config.approval_ttl, rx).await,
            Ok(Ok(true))
        );
        lock(&self.approvals).remove(&id);
        self.emit(Event::ApprovalResolved { id, approved });
        approved
    }

    fn resolve(&self, id: u64, approved: bool) {
        if let Some(tx) = lock(&self.approvals).remove(&id) {
            let _ = tx.send(approved);
        }
    }

    async fn run(&self, tool: &Arc<dyn Tool>, args: Value) -> ToolOutcome {
        let spec = tool.spec();
        let _held = if spec.resources.is_empty() {
            None
        } else {
            match self.guard.lock(&spec.resources) {
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
        acted: &AtomicBool,
    ) -> Value {
        let call = self.calls.fetch_add(1, Ordering::Relaxed) + 1;
        let spec = tool.spec();
        self.emit(Event::ToolStarted {
            turn,
            call,
            tool: spec.name.clone(),
            args: args.clone(),
        });
        let started = Instant::now();
        let outcome = match self.guard.decide(&spec, &args) {
            Decision::Deny(r) => ToolOutcome::refused(r.message),
            Decision::NeedApproval { reason } => {
                if self.ask_approval(&spec.name, &args, reason).await {
                    self.run(tool, args).await
                } else {
                    ToolOutcome::refused("the operator did not approve this")
                }
            }
            Decision::Allow => self.run(tool, args).await,
        };
        if spec.lane() == Lane::Act
            && matches!(outcome.status, Status::Succeeded | Status::Accepted)
        {
            acted.store(true, Ordering::SeqCst);
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
                            shared.emit(Event::Notice { text: "still working on the last message; wait or stop it".into() });
                            continue;
                        }
                        turns += 1;
                        let task = run_turn(turns, text, history.clone(), Arc::clone(&shared), Arc::clone(&source), Arc::clone(&registry));
                        running = Some((turns, tokio::spawn(task)));
                    }
                    Command::StopGeneration | Command::StopMission => {
                        let mission = cmd == Command::StopMission;
                        if let Some((turn, handle)) = running.take() {
                            handle.abort();
                            shared.emit(Event::TurnFinished { turn });
                        }
                        let pending: Vec<u64> = lock(&shared.approvals).keys().copied().collect();
                        for id in pending {
                            shared.resolve(id, false);
                        }
                        if mission && let Some(stop) = stop.clone() {
                            let shared = Arc::clone(&shared);
                            tokio::spawn(async move {
                                let out = stop.call(json!({"reason": "operator"})).await;
                                let text = if out.status == Status::Succeeded { format!("robot stopped: {}", out.data) } else { format!("stop failed: {}", out.message) };
                                shared.emit(Event::Notice { text });
                            });
                        }
                        let reason = if mission { "stopped by the operator: reply and robot" } else { "reply stopped by the operator" };
                        shared.emit(Event::Halted { reason: reason.into() });
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
            }
        }
    }
}

async fn run_turn(
    turn: u64,
    text: String,
    history: History,
    shared: Arc<Shared>,
    source: Arc<dyn AgentSource>,
    registry: Arc<Registry>,
) -> Option<History> {
    shared.guard.begin_turn();
    shared.emit(Event::TurnStarted { turn });
    let acted = Arc::new(AtomicBool::new(false));
    let tools: Vec<LoopTool> = registry
        .iter()
        .map(|tool| {
            let (tool, shared, acted) = (Arc::clone(tool), Arc::clone(&shared), Arc::clone(&acted));
            let spec = tool.spec().into_owned();
            LoopTool {
                name: spec.name,
                description: spec.description,
                parameters: spec.parameters,
                invoke: Arc::new(move |args| {
                    let (tool, shared, acted) =
                        (Arc::clone(&tool), Arc::clone(&shared), Arc::clone(&acted));
                    Box::pin(async move { shared.invoke(&tool, turn, args, &acted).await })
                }),
            }
        })
        .collect();
    let need = Need {
        tools: !tools.is_empty(),
        ..Need::default()
    };
    let candidates = source.candidates(Role::Routine, need);
    let mut failures = Vec::new();
    for model in candidates {
        let builder = match source.builder(&model) {
            Ok(b) => b,
            Err(e) => {
                failures.push(format!("{model}: {e}"));
                continue;
            }
        };
        let mut updated = history.clone();
        source.record_use(&model);
        let result = llm::chat(
            &model,
            builder,
            &shared.config.preamble,
            shared.config.max_model_calls,
            &tools,
            &mut updated,
            &text,
        )
        .await;
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
            Err(e) if !acted.load(Ordering::SeqCst) => {
                shared.emit(Event::Notice {
                    text: format!("{e}; trying the next model"),
                });
                failures.push(e.to_string());
            }
            Err(e) => {
                shared.emit(Event::Error {
                    turn,
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
    use std::borrow::Cow;
    use crate::guard::Policy;
    use crate::llm::{AgentBuilder, LlmError};
    use crate::tools::{Risk, ToolSpec};
    use async_trait::async_trait;
    use rig::test_utils::{MockCompletionModel, MockTurn};

    struct Scripted(MockCompletionModel);

    impl AgentSource for Scripted {
        fn candidates(&self, _role: Role, _need: Need) -> Vec<String> {
            vec!["mock".into()]
        }
        fn builder(&self, _id: &str) -> Result<AgentBuilder, LlmError> {
            Ok(AgentBuilder::new(self.0.clone()))
        }
        fn record_use(&self, _id: &str) {}
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
