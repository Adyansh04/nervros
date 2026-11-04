//! The NervROS agent core: configuration, model providers, tools, guard, missions and the session.
//!
//! This crate holds everything that is not GUI. The only module that imports `rig` is [`llm`], so
//! a rig upgrade touches one module.

pub mod agent;
pub mod app;
pub mod config;
pub mod doctor;
pub mod incident;
pub mod integrations;
pub mod job;
pub mod mission;
pub mod model;
mod persist;
pub mod program;
pub mod search;
pub mod telemetry;
pub mod tools;
pub mod whoami;

pub(crate) use agent::{argcheck, context};
pub use agent::{evalcase, guard, log, session};
pub(crate) use config::secret;
pub use config::{profile, schemas};
pub use mission::check::how_long;
pub(crate) use mission::schedule;
pub use model::{llm, providers};
pub use tools::places::room_label;
pub use tools::{
    background, builtins, editor, look, mcp, ros_tools, segment, skills, vision, watch, zones,
};
pub(crate) use tools::{memory, places, point};

/// Whole seconds since the Unix epoch; 0 for a clock set before it.
#[must_use]
pub fn unix_secs(at: std::time::SystemTime) -> u64 {
    at.duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

/// The string at `key` of a JSON object, empty when there is none.
pub(crate) fn text<'v>(v: &'v serde_json::Value, key: &str) -> &'v str {
    v[key].as_str().unwrap_or_default()
}

/// A ROS `builtin_interfaces` Time or Duration as seconds, a missing field counting as zero.
#[must_use]
pub(crate) fn stamp_s(stamp: &serde_json::Value) -> f64 {
    stamp["sec"].as_f64().unwrap_or(0.0) + stamp["nanosec"].as_f64().unwrap_or(0.0) * 1e-9
}

/// Now, in seconds since the Unix epoch.
#[must_use]
pub fn now_s() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// `n` and its noun, which takes an "s" unless `n` is 1: "1 step", "3 steps".
pub(crate) fn counted<N: std::fmt::Display + PartialEq + From<u8> + Copy>(
    n: N,
    noun: &str,
) -> String {
    let s = if n == N::from(1) { "" } else { "s" };
    format!("{n} {noun}{s}")
}

/// A duration as the window words it: seconds under a minute and a half, else minutes, so an
/// approval's "at most" reads as its plan card's.
pub(crate) fn duration_words(seconds: f64) -> String {
    if seconds < 90.0 {
        format!("{seconds:.0} s")
    } else {
        format!("{:.0} min", seconds / 60.0)
    }
}

pub(crate) use nervros_ros::lock;

#[cfg(test)]
mod tests {
    #[test]
    fn counts_read_as_words() {
        assert_eq!(super::counted(1usize, "step"), "1 step");
        assert_eq!(super::counted(4u64, "step"), "4 steps");
        assert_eq!(super::counted(0u32, "time"), "0 times");
        assert_eq!(super::duration_words(30.0), "30 s");
        assert_eq!(super::duration_words(240.0), "4 min");
    }
}
