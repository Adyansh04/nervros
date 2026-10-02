//! The NervROS agent core: configuration, model providers, tools, guard, missions and the session.
//!
//! This crate holds everything that is not GUI. The only module that imports `rig` is [`llm`], so
//! a rig upgrade touches one module.

pub mod app;
pub mod argcheck;
pub mod builtins;
pub mod context;
pub mod doctor;
pub mod editor;
pub mod evalcase;
pub mod guard;
pub mod llm;
pub mod log;
pub mod look;
pub mod mcp;
pub mod memory;
pub mod mission;
mod persist;
pub mod places;
pub mod point;
pub mod profile;
pub mod providers;
pub mod ros_tools;
pub mod schedule;
pub mod schemas;
pub mod secret;
pub mod segment;
pub mod session;
pub mod skills;
pub mod telemetry;
pub mod tools;
pub mod watch;

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
