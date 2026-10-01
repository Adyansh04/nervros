//! Earlier sessions whose conversations were saved, for the Agent tab to resume.

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use nervros_core::llm::History;

/// How many earlier sessions the tab lists.
const LISTED: usize = 12;

/// One saved conversation.
#[derive(Debug, Clone)]
pub struct SessionInfo {
    /// Its `.history.json`.
    pub path: PathBuf,
    /// How long ago it started, as people say it.
    pub when: String,
    /// The operator's first message, shortened.
    pub first: String,
    /// Messages in it.
    pub messages: usize,
}

/// The saved conversations under `logs`, newest first, without the running session's own.
#[must_use]
pub fn list(logs: &Path, current_log: &Path) -> Vec<SessionInfo> {
    let own = current_log.with_extension("history.json");
    let mut found: Vec<(u64, PathBuf)> = std::fs::read_dir(logs)
        .map(|d| {
            d.filter_map(Result::ok)
                .map(|e| e.path())
                .filter(|p| *p != own)
                .filter_map(|p| {
                    let name = p.file_name()?.to_str()?;
                    let stamp = name
                        .strip_prefix("session-")?
                        .strip_suffix(".history.json")?;
                    Some((stamp.parse().ok()?, p))
                })
                .collect()
        })
        .unwrap_or_default();
    found.sort_by_key(|(stamp, _)| std::cmp::Reverse(*stamp));
    found
        .into_iter()
        .take(LISTED)
        .filter_map(|(stamp, path)| {
            let history = History::load(&path).ok()?;
            let exchanges = history.exchanges();
            let first = exchanges.iter().find(|(operator, _)| *operator)?.1.clone();
            Some(SessionInfo {
                when: ago(stamp),
                first: shorten(&first, 48),
                messages: exchanges.len(),
                path,
            })
        })
        .collect()
}

fn shorten(text: &str, max: usize) -> String {
    let line = text.lines().next().unwrap_or_default();
    if line.chars().count() <= max {
        line.to_owned()
    } else {
        format!("{}…", line.chars().take(max).collect::<String>())
    }
}

/// "5 min ago", "3 h ago", "2 days ago", from seconds since the Unix epoch.
pub fn ago(stamp: u64) -> String {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or(Duration::ZERO)
        .as_secs();
    let mins = now.saturating_sub(stamp) / 60;
    match mins {
        0 => "just now".to_owned(),
        1..60 => format!("{mins} min ago"),
        60..1440 => format!("{} h ago", mins / 60),
        _ => format!("{} days ago", mins / 1440),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ages_and_long_messages_read_short() {
        assert_eq!(
            shorten("Go to the bedroom.\nthen more", 48),
            "Go to the bedroom."
        );
        assert_eq!(shorten(&"x".repeat(60), 10), format!("{}…", "x".repeat(10)));
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        assert_eq!(ago(now - 7200), "2 h ago");
        assert_eq!(ago(now), "just now");
    }
}
