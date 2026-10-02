//! The agent loop: the session that owns the conversation, the guard every tool call passes, the
//! argument checks and context the model gets, and the session's log and evals.

pub(crate) mod argcheck;
pub(crate) mod context;
pub mod evalcase;
pub mod guard;
pub mod log;
pub mod session;
