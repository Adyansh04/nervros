//! The session: one conversation with one robot.
//!
//! An actor owns the conversation. User text starts a turn, which runs as its own task so that stop
//! and approval commands are always handled at once: stop aborts the turn and calls the robot's
//! `StopAll` without asking any model. Each turn tries the routine role's models in order; a model
//! that fails before any act-lane tool ran is replaced by the next one, and after one did, the turn
//! ends instead of repeating an action. Every tool call passes the guard.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::sync::atomic::{AtomicU32, AtomicU64};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use tokio::sync::{broadcast, mpsc};

use crate::guard::Guard;
use crate::llm::{AgentSource, History};
use crate::tools::{Registry, Tool};

mod actor;
mod approval;
mod calls;
mod config;
mod event;
#[cfg(test)]
mod tests;
mod turn;
mod window;

use actor::actor;
use approval::Asked;
pub use approval::{Unanswered, take_unanswered};
pub use config::{EVENT_BACKLOG, PULSE_PERIOD, Pulse, SessionConfig};
pub use event::{Command, Event, is_stop_word};

fn millis(d: Duration) -> u64 {
    u64::try_from(d.as_millis()).unwrap_or(u64::MAX)
}

/// What the actor, its turns and their tool calls share.
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
    /// The conversation to write next, for the writer that keeps the disk off this loop.
    to_save: Option<tokio::sync::watch::Sender<Option<History>>>,
    config: SessionConfig,
}

impl Shared {
    fn emit(&self, e: Event) {
        // No subscriber is fine: a headless run may not watch events.
        let _ = self.events.send(e);
    }
}

/// Hands the conversation to its writer, if the session keeps it anywhere.
fn save(shared: &Shared, history: &History) {
    if let Some(to_save) = &shared.to_save {
        to_save.send_replace(Some(history.clone()));
    }
}

/// Writes each conversation handed to it to `file`, in order, newest only when several wait: a
/// slow disk holds up neither a stop nor the next turn.
fn writer(file: std::path::PathBuf) -> tokio::sync::watch::Sender<Option<History>> {
    let (tx, mut rx) = tokio::sync::watch::channel(None::<History>);
    tokio::spawn(async move {
        while rx.changed().await.is_ok() {
            let Some(history) = rx.borrow_and_update().clone() else {
                continue;
            };
            let file = file.clone();
            let written = tokio::task::spawn_blocking(move || history.save(&file)).await;
            if let Ok(Err(e)) = written {
                tracing::warn!(error = %e, "the conversation was not saved");
            }
        }
    });
    tx
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
        let to_save = config.history_file.clone().map(writer);
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
            to_save,
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
