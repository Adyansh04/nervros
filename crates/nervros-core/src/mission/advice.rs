//! Advice from a stronger model when the planner's own plans keep failing their checks: the
//! models of the `plan` role, asked once a request's plans have failed twice, when that role is
//! not the model that plans. The small model keeps the conversation and the tools; the strong one
//! sees only the request, the skills and what went wrong, and answers in a few lines. A role whose
//! models are out of quota answers nothing, and the planner carries on without.

use async_trait::async_trait;
use serde_json::Value;

/// Plans failed this many times in a row for one request before the advisor is asked.
pub(crate) const ADVISE_AFTER: u32 = 2;
/// Advice past this many characters is cut: a hint, not a plan of its own.
pub(crate) const ADVICE_CHARS: usize = 600;

/// How the advisor is told to answer.
pub const ADVISOR_PREAMBLE: &str = "A small model plans missions for a robot from a list of \
    skills, and its plans keep failing their checks. You see the operator's request, the skills, \
    its last plan and why it failed. Say in at most five short lines how to fix the plan: which \
    skills, in what order, with which arguments. Use only the skills listed; if they cannot do \
    the request, say so.";

/// A stronger model to ask.
#[async_trait]
pub trait Advisor: Send + Sync {
    /// Its advice, or `None` when no model of the role could answer.
    async fn advise(&self, prompt: &str) -> Option<String>;
}

/// What the advisor is asked.
pub(crate) fn prompt(request: &str, skills: &str, plan: &Value, problems: &Value) -> String {
    format!(
        "The operator asked: {request}\n\nThe skills:\n{skills}\n\nThe last plan:\n{plan}\n\n\
         Why it failed:\n{problems}"
    )
}
