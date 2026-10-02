//! The model's context window: what fits, how full it is, and condensing to fit.

use std::sync::Arc;

use super::Shared;
use super::event::Event;
use crate::llm;
use crate::llm::AgentSource;
use crate::llm::History;
use crate::llm::LoopTool;
use crate::providers::Role;
use crate::providers::router::Need;
use crate::tools::Registry;

/// Before a turn, condenses a history past half the first candidate's window.
pub(super) async fn fit_window(
    shared: &Shared,
    source: &Arc<dyn AgentSource>,
    candidates: &[String],
    tools: &[LoopTool],
    history: &mut History,
) {
    let Some(window) = candidates.first().and_then(|m| source.context(m)) else {
        return;
    };
    let room =
        window.saturating_sub(llm::fixed_cost(&preamble(shared), tools) + llm::RESERVE_TOKENS);
    if history.size() > room / 2 {
        compact(shared, source, history, room, Condense::ToFit).await;
    }
}

/// Tells the UI how full the model's window was: as the provider counted, or as estimated.
pub(super) fn report_context(
    shared: &Shared,
    tools: &[LoopTool],
    history: &History,
    counted: u64,
    window: usize,
) {
    let estimated = llm::fixed_cost(&preamble(shared), tools) + history.size();
    shared.emit(Event::Context {
        used: if counted > 0 {
            counted
        } else {
            u64::try_from(estimated).unwrap_or(u64::MAX)
        },
        window: u64::try_from(window).unwrap_or(u64::MAX),
    });
}

/// The system prompt for this turn: the fixed one and the notes as they are now.
pub(super) fn preamble(shared: &Shared) -> String {
    let notes = shared
        .config
        .notes
        .as_ref()
        .map(|n| n.text())
        .unwrap_or_default();
    format!("{}{notes}", shared.config.preamble)
}

/// Room for the history when the model's window is unknown, for an operator's compaction.
pub(super) const COMPACT_ROOM: usize = 12_000;

/// Room left for the history in the first routine model's window, if the models file gives it.
pub(super) fn room_for(
    shared: &Shared,
    source: &Arc<dyn AgentSource>,
    registry: &Registry,
) -> Option<usize> {
    let need = Need {
        tools: true,
        ..Need::default()
    };
    let window = source
        .candidates(Role::Routine, need)
        .first()
        .and_then(|m| source.context(m))?;
    let schemas: usize = registry
        .iter()
        .map(|t| {
            let spec = t.spec();
            spec.name.len() + spec.description.len() + spec.parameters.to_string().len()
        })
        .sum();
    let fixed = llm::tokens_of(preamble(shared).len() + schemas);
    Some(window.saturating_sub(fixed + llm::RESERVE_TOKENS))
}

/// How far a compaction goes.
#[derive(Debug, Clone, Copy)]
pub(super) enum Condense {
    /// Past half the window: old results cut first, and a summary only when that is not enough.
    ToFit,
    /// The operator's `/compact`: old results cut, and the older part summarised.
    Now,
    /// "Condense up to here": all but the operator's last `n` messages summarised.
    UpTo(usize),
}

/// Condenses the history to about a quarter of `room`: the older part summarised by a model,
/// or, when none answers, old results cut and the oldest exchanges dropped.
pub(super) async fn compact(
    shared: &Shared,
    source: &Arc<dyn AgentSource>,
    history: &mut History,
    room: usize,
    how: Condense,
) {
    let before = history.size();
    history.mask();
    let older = match how {
        Condense::ToFit if history.size() <= room / 2 => None,
        Condense::ToFit | Condense::Now => history.older(room / 4),
        Condense::UpTo(keep) => history.before_last(keep),
    };
    let mut summarised = false;
    if let Some((n, text)) = older
        && let Some(summary) = llm::summarise(source, &text).await
    {
        history.summarised(n, &summary);
        summarised = true;
    }
    if history.size() > room / 2 {
        history.squeeze(room / 2);
    }
    shared.emit(Event::Compacted {
        before: u64::try_from(before).unwrap_or(u64::MAX),
        after: u64::try_from(history.size()).unwrap_or(u64::MAX),
        summarised,
    });
}
