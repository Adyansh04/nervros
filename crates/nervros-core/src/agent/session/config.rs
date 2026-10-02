//! A session's settings.

use std::sync::Arc;
use std::time::Duration;

use crate::llm::History;

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
