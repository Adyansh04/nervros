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
pub mod memory;
pub mod mission;
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
pub mod tools;
pub mod watch;
