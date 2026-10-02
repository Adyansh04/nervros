//! Earlier sessions whose conversations were saved, for the Agent tab to resume.

use std::collections::HashMap;
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

/// The saved conversations seen so far, each with the time its file was written: a file is read
/// again only once it changes, so listing them every few seconds costs a folder listing.
#[derive(Default)]
pub struct Listing {
    read: HashMap<PathBuf, (SystemTime, Option<SessionInfo>)>,
}

impl Listing {
    /// The saved conversations under `logs`, newest first, without the running session's own.
    pub fn list(&mut self, logs: &Path, current_log: &Path) -> Vec<SessionInfo> {
        let mut out = Vec::new();
        for (stamp, path) in newest(logs, current_log) {
            let written = std::fs::metadata(&path)
                .and_then(|m| m.modified())
                .unwrap_or(UNIX_EPOCH);
            let fresh = self.read.get(&path).filter(|(at, _)| *at == written);
            let info = if let Some((_, info)) = fresh {
                info.clone()
            } else {
                let info = read(stamp, &path);
                self.read.insert(path, (written, info.clone()));
                info
            };
            out.extend(info.map(|i| SessionInfo {
                when: ago(stamp),
                ..i
            }));
        }
        out
    }
}

/// One saved conversation, read whole.
fn read(stamp: u64, path: &Path) -> Option<SessionInfo> {
    let history = History::load(path).ok()?;
    let exchanges = history.exchanges();
    let first = exchanges.iter().find(|(operator, _)| *operator)?.1.clone();
    Some(SessionInfo {
        when: ago(stamp),
        first: shorten(&first, 48),
        messages: exchanges.len(),
        path: path.to_path_buf(),
    })
}

/// The newest saved conversations' files and stamps, without the running session's own.
fn newest(logs: &Path, current_log: &Path) -> Vec<(u64, PathBuf)> {
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
    found.truncate(LISTED);
    found
}

fn shorten(text: &str, max: usize) -> String {
    let line = text.lines().next().unwrap_or_default();
    if line.chars().count() <= max {
        line.to_owned()
    } else {
        format!("{}…", line.chars().take(max).collect::<String>())
    }
}

/// A session's event log, beside its saved conversation.
pub fn log_of(history: &Path) -> PathBuf {
    let name = history
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or_default();
    history.with_file_name(name.replace(".history.json", ".ndjson"))
}

/// A session as an eval case, appended to the suite of saved cases in the state folder: its id
/// and the suite's path. The case expects what happened, to trim before relying on it.
///
/// # Errors
///
/// The log cannot be read, holds no operator message, or the suite cannot be written.
pub fn save_as_case(log: &Path) -> Result<(String, PathBuf), String> {
    use std::io::Write as _;
    let text = std::fs::read_to_string(log).map_err(|e| format!("{}: {e}", log.display()))?;
    let mut case = nervros_core::evalcase::from_log(&text, "");
    let first = case
        .say
        .first()
        .ok_or("the session has no operator message")?;
    let suite = nervros_core::app::state_dir()
        .join("evals")
        .join("saved.toml");
    // Saved twice, or two sessions that began alike, still make two cases.
    let saved = std::fs::read_to_string(&suite).unwrap_or_default();
    case.id = nervros_core::evalcase::unique_id(&saved, &slug(first));
    let toml = nervros_core::evalcase::to_toml(&case).map_err(|e| e.to_string())?;
    if let Some(dir) = suite.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&suite)
        .map_err(|e| format!("{}: {e}", suite.display()))?;
    write!(file, "\n# From {}\n{toml}", log.display()).map_err(|e| e.to_string())?;
    Ok((case.id, suite))
}

/// A case id from the operator's words: "turn-left-90-degrees".
fn slug(text: &str) -> String {
    let words: Vec<String> = text
        .split(|c: char| !c.is_alphanumeric())
        .filter(|w| !w.is_empty())
        .take(6)
        .map(str::to_lowercase)
        .collect();
    words.join("-")
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

    #[test]
    fn a_case_is_named_by_the_operators_words_and_found_beside_its_conversation() {
        assert_eq!(
            slug("Turn left 90 degrees, then stop."),
            "turn-left-90-degrees-then-stop"
        );
        assert_eq!(
            log_of(Path::new("/l/session-17.history.json")),
            Path::new("/l/session-17.ndjson")
        );
    }
}
