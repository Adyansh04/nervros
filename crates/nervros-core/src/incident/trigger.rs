//! When the recorder opens an incident: a mission failed, a watched topic went quiet, a watched
//! node went, or a node logged errors in a burst. Each opens one incident until it recovers, so a
//! dead camera is one incident and not one a second.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use super::playbook::Kind;

/// Seconds a watched topic stays under its rate before it counts as quiet: a camera can stall.
const QUIET_S: f64 = 3.0;
/// Error lines from one node within `BURST_S` that make a burst.
const BURST_LINES: usize = 10;
const BURST_S: f64 = 10.0;
/// After a burst, that node's errors open nothing more for this long.
const BURST_QUIET_S: f64 = 60.0;

/// Why an incident opens.
#[derive(Debug, Clone, PartialEq)]
pub struct Trigger {
    /// What kind of thing happened.
    pub kind: Kind,
    /// What happened, in a sentence.
    pub text: String,
    /// The mission that failed, or empty.
    pub mission: String,
    /// Its failure as the executor said it, or empty.
    pub failure: String,
}

impl Trigger {
    /// A trigger with no mission to it.
    #[must_use]
    pub fn new(kind: Kind, text: impl Into<String>) -> Self {
        Self {
            kind,
            text: text.into(),
            mission: String::new(),
            failure: String::new(),
        }
    }
}

/// How a watched topic's rate changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Change {
    Same,
    WentQuiet,
    CameBack,
}

/// One watched topic's state: quiet only after it kept its rate once, so a stack still starting
/// opens nothing.
#[derive(Debug, Default)]
pub(crate) struct QuietTopic {
    kept_once: bool,
    under_since: Option<f64>,
    quiet: bool,
}

impl QuietTopic {
    /// The topic's rate at `now`.
    pub(crate) fn rate(&mut self, hz: f64, min_hz: f64, now: f64) -> Change {
        if hz >= min_hz {
            self.kept_once = true;
            self.under_since = None;
            return if std::mem::take(&mut self.quiet) {
                Change::CameBack
            } else {
                Change::Same
            };
        }
        if !self.kept_once || self.quiet {
            return Change::Same;
        }
        let since = *self.under_since.get_or_insert(now);
        if now - since >= QUIET_S {
            self.quiet = true;
            Change::WentQuiet
        } else {
            Change::Same
        }
    }
}

/// The watched nodes that were running before and are not now.
pub(crate) fn gone<'a>(
    before: &BTreeSet<String>,
    now: &BTreeSet<String>,
    watched: &'a [String],
) -> Vec<&'a str> {
    let running = |set: &BTreeSet<String>, w: &str| set.iter().any(|n| names_node(n, w));
    watched
        .iter()
        .filter(|w| running(before, w) && !running(now, w))
        .map(String::as_str)
        .collect()
}

/// Whether the full node name `full` (`/ns/name`) is the node `wanted` names: in full, or by its
/// last part.
#[must_use]
pub(crate) fn names_node(full: &str, wanted: &str) -> bool {
    let wanted = wanted.trim_start_matches('/');
    full.trim_start_matches('/') == wanted || full.ends_with(&format!("/{wanted}"))
}

/// Error lines as they come, telling a burst from one node.
#[derive(Debug, Default)]
pub(crate) struct ErrorBursts {
    recent: VecDeque<(f64, String)>,
    quiet_until: BTreeMap<String, f64>,
}

impl ErrorBursts {
    /// An error line from `node` at `now`: whether it makes a burst that opens an incident.
    pub(crate) fn error(&mut self, node: &str, now: f64) -> bool {
        while self.recent.front().is_some_and(|(t, _)| now - t > BURST_S) {
            self.recent.pop_front();
        }
        self.recent.push_back((now, node.to_owned()));
        if self.quiet_until.get(node).is_some_and(|until| now < *until) {
            return false;
        }
        let lines = self.recent.iter().filter(|(_, n)| n == node).count();
        if lines < BURST_LINES {
            return false;
        }
        self.quiet_until
            .insert(node.to_owned(), now + BURST_QUIET_S);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_topic_is_quiet_once_it_kept_its_rate_and_then_stayed_under_it() {
        let mut camera = QuietTopic::default();
        assert_eq!(camera.rate(0.0, 5.0, 0.0), Change::Same, "not up yet");
        assert_eq!(camera.rate(15.0, 5.0, 1.0), Change::Same);
        assert_eq!(camera.rate(0.0, 5.0, 2.0), Change::Same, "a stall");
        assert_eq!(camera.rate(0.0, 5.0, 4.0), Change::Same);
        assert_eq!(camera.rate(0.0, 5.0, 5.0), Change::WentQuiet);
        assert_eq!(camera.rate(0.0, 5.0, 9.0), Change::Same, "once");
        assert_eq!(camera.rate(14.0, 5.0, 10.0), Change::CameBack);
        assert_eq!(camera.rate(1.0, 5.0, 11.0), Change::Same);
    }

    #[test]
    fn a_watched_node_that_went_is_named_by_its_last_part_or_in_full() {
        let set = |names: &[&str]| names.iter().map(|n| (*n).to_owned()).collect();
        let before = set(&["/robot/detector", "/controller_server", "/other"]);
        let now = set(&["/controller_server"]);
        let watched = vec!["detector".to_owned(), "/controller_server".to_owned()];
        assert_eq!(gone(&before, &now, &watched), ["detector"]);
        assert!(names_node("/ns/a_node", "/a_node") && !names_node("/ns/an_a_node", "a_node"));
    }

    #[test]
    fn ten_errors_from_one_node_in_ten_seconds_are_a_burst_once_a_minute() {
        let mut bursts = ErrorBursts::default();
        for i in 0..9 {
            assert!(!bursts.error("/nav", f64::from(i)));
            assert!(!bursts.error("/other", f64::from(i)));
        }
        assert!(bursts.error("/nav", 9.0));
        assert!(!bursts.error("/nav", 9.5), "the burst opened one incident");
        for i in 0..12 {
            assert!(
                !bursts.error("/slow", 20.0 + f64::from(i) * 2.0),
                "five in ten seconds"
            );
        }
        let later: Vec<bool> = (0..10)
            .map(|i| bursts.error("/nav", 90.0 + f64::from(i)))
            .collect();
        assert_eq!(
            later.iter().filter(|b| **b).count(),
            1,
            "a minute on, it may again"
        );
    }
}
