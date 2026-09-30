//! The NervROS agent core: configuration, model providers, tools, guard, missions and the session.
//!
//! This crate holds everything that is not GUI. The only module that imports `rig` is [`llm`], so
//! a rig upgrade touches one module.

pub mod app;
pub mod builtins;
pub mod context;
pub mod doctor;
pub mod editor;
pub mod guard;
pub mod llm;
pub mod log;
pub mod look;
pub mod mission;
pub mod profile;
pub mod providers;
pub mod ros_tools;
pub mod schemas;
pub mod secret;
pub mod segment;
pub mod session;
pub mod tools;
pub mod watch;
