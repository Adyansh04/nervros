//! Skills the agent teaches the robot: a small tree of the leaves the executor lets a taught skill
//! use, sent to its `DefineSkill` as a draft. The first mission that runs a draft, which the
//! operator approves like any other, keeps it when it succeeds; a kept skill that keeps failing is
//! forgotten again. The executor checks each one in full, so nothing here is the safety boundary.

mod body;
mod keep;
mod tool;

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use serde_json::Value;

use super::Missions;
use crate::lock;
use crate::tools::Tool;

/// The drafts taught in this session, by name: the request that keeps each one sends again.
#[derive(Debug, Default)]
pub(crate) struct Drafts(Mutex<BTreeMap<String, Value>>);

impl Drafts {
    fn put(&self, name: &str, request: Value) {
        lock(&self.0).insert(name.to_owned(), request);
    }

    fn take(&self, name: &str) -> Option<Value> {
        lock(&self.0).remove(name)
    }
}

/// The `teach_skill` tool, when the profile names the executor's `DefineSkill` and
/// `ForgetSkill`.
#[must_use]
pub fn tools(missions: &Arc<Missions>) -> Vec<Arc<dyn Tool>> {
    if missions.config.define_skill.is_none() || missions.config.forget_skill.is_none() {
        return Vec::new();
    }
    vec![Arc::new(tool::TeachSkill(Arc::clone(missions)))]
}
