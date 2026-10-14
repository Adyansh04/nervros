//! What a mission's end means for the skills taught: a draft that just worked is kept, and a kept
//! one that keeps failing is forgotten.

use std::collections::BTreeSet;
use std::sync::Arc;

use serde_json::Value;

use crate::mission::plan::Compiled;
use crate::mission::{Missions, Outcome};
use crate::session::Event;

/// Runs of a taught skill counted before it may be forgotten for failing.
const JUDGED_AFTER: u32 = 4;

impl Missions {
    /// After a mission ends: keeps each draft it ran when it succeeded, and forgets a kept taught
    /// skill it failed at that now fails more often than not.
    pub(crate) async fn settle_taught(
        self: &Arc<Self>,
        compiled: &Compiled,
        outcome: Outcome,
        failed_step: &str,
    ) {
        // A stop is the operator's, not the skill's failure.
        if self.config.define_skill.is_none() || outcome == Outcome::Canceled {
            return;
        }
        let Ok(catalog) = self.catalog().await else {
            return;
        };
        let taught: BTreeSet<&str> = compiled
            .steps
            .iter()
            .filter_map(|s| catalog.skill(&s.skill))
            .filter(|s| !s.learned.is_empty())
            .map(|s| s.name.as_str())
            .collect();
        for name in taught {
            let draft = catalog.skill(name).is_some_and(|s| s.learned == "draft");
            if draft && outcome == Outcome::Success {
                self.keep_draft(name).await;
            } else if !draft
                && compiled
                    .steps
                    .iter()
                    .any(|s| s.id == failed_step && s.skill == name)
            {
                self.maybe_forget(name).await;
            }
        }
    }

    /// Keeps a draft for good: the executor gets the same definition again, to keep on disk.
    async fn keep_draft(&self, name: &str) {
        let Some(mut request) = self.drafts.take(name) else {
            // Taught before this session began: nothing here knows its definition to keep.
            return;
        };
        request["keep"] = Value::Bool(true);
        let text = match self.define_skill(&request).await {
            Ok(()) => format!("{name} worked, and is one of the robot's skills now"),
            Err(why) => {
                self.drafts.put(name, request);
                format!("{name} worked but could not be kept: {why}")
            }
        };
        self.emit(Event::Notice { text });
    }

    /// Forgets a kept taught skill once it has failed more often than not in its latest runs.
    async fn maybe_forget(&self, name: &str) {
        let Some(ledger) = self.ledger().cloned() else {
            return;
        };
        let skill = name.to_owned();
        let runs = tokio::task::spawn_blocking(move || ledger.skill_runs(&skill))
            .await
            .ok()
            .and_then(Result::ok);
        let Some((runs, succeeded)) = runs.filter(|(runs, _)| *runs >= JUDGED_AFTER) else {
            return;
        };
        if succeeded * 2 >= runs {
            return;
        }
        let text = match self.forget_skill(name).await {
            Ok(()) => format!(
                "{name} failed {} of its last {runs} runs and is forgotten; teach it again if it is \
                 still wanted",
                runs - succeeded
            ),
            Err(why) => format!("{name} keeps failing, but could not be forgotten: {why}"),
        };
        self.emit(Event::Notice { text });
    }
}
