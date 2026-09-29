//! The NervROS agent core: configuration, model providers, tools, guard, missions and the session.
//!
//! This crate holds everything that is not GUI. The only module that imports `rig` is [`llm`], so
//! a rig upgrade touches one module.

pub mod builtins;
pub mod guard;
pub mod llm;
pub mod look;
pub mod profile;
pub mod providers;
pub mod schemas;
pub mod secret;
pub mod tools;
