//! The NervROS agent core: configuration, model providers, tools, guard, missions and the session.
//!
//! This crate holds everything that is not GUI. The only module that imports `rig` is [`llm`], so
//! a rig upgrade touches one module.

pub mod agent;
pub mod app;
pub mod config;
pub mod doctor;
pub mod mission;
pub mod model;
mod persist;
pub mod telemetry;
pub mod tools;

pub(crate) use agent::{argcheck, context};
pub use agent::{evalcase, guard, log, session};
pub(crate) use config::secret;
pub use config::{profile, schemas};
pub(crate) use mission::schedule;
pub use model::{llm, providers};
pub use tools::{builtins, editor, look, mcp, ros_tools, segment, skills, vision, watch};
pub(crate) use tools::{memory, places, point};

/// Whole seconds since the Unix epoch; 0 for a clock set before it.
#[must_use]
pub fn unix_secs(at: std::time::SystemTime) -> u64 {
    at.duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

/// Now, in seconds since the Unix epoch.
#[must_use]
pub fn now_s() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// A mutex's guard, poisoned or not: a panic while it was held leaves data still worth reading.
pub(crate) fn lock<T>(m: &std::sync::Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}
