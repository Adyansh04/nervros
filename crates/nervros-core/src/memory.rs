//! What the operator asks the agent to remember across sessions ("the kitchen door sticks",
//! "always use the left arm"): kept beside the quota ledger, one file per robot, and put in front
//! of the model on every turn, so a compacted conversation does not lose it.

use std::borrow::Cow;
use std::fmt::Write as _;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{SystemTime, UNIX_EPOCH};

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};

/// A note is a sentence or two, not a document.
const NOTE_CHARS: usize = 300;
/// Enough for what one operator says to keep, little enough to stay in every prompt.
const MAX_NOTES: usize = 40;

/// One thing to remember.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Note {
    /// Its number, for forgetting it.
    pub id: u32,
    /// What to remember.
    pub text: String,
    /// When it was said, seconds since 1970.
    pub at: u64,
}

/// The notes, and the file they live in.
#[derive(Debug)]
pub struct Memory {
    notes: Mutex<Vec<Note>>,
    file: Option<PathBuf>,
}

impl Memory {
    /// The notes in `file`, when it exists.
    #[must_use]
    pub fn new(file: Option<PathBuf>) -> Arc<Self> {
        let notes = file
            .as_deref()
            .map(crate::persist::read_or_default)
            .unwrap_or_default();
        Arc::new(Self {
            notes: Mutex::new(notes),
            file,
        })
    }

    fn lock(&self) -> MutexGuard<'_, Vec<Note>> {
        self.notes.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Every note, oldest first.
    #[must_use]
    pub fn all(&self) -> Vec<Note> {
        self.lock().clone()
    }

    fn save(&self, notes: &[Note]) -> Result<(), String> {
        let Some(file) = &self.file else {
            return Ok(());
        };
        let text = serde_json::to_vec_pretty(notes).map_err(|e| e.to_string())?;
        crate::persist::write_atomic(file, &text)
            .map_err(|e| format!("saving {}: {e}", file.display()))
    }

    /// Keeps a note; the same words twice are kept once.
    ///
    /// # Errors
    ///
    /// The note is empty or too long, there are too many, or the file cannot be written.
    pub fn remember(&self, text: &str) -> Result<Note, String> {
        let text = text.trim();
        if text.is_empty() {
            return Err("`text` is what to remember, in a sentence".to_owned());
        }
        if text.chars().count() > NOTE_CHARS {
            return Err(format!(
                "a note is at most {NOTE_CHARS} characters; say it shorter"
            ));
        }
        let mut notes = self.lock();
        if let Some(same) = notes.iter().find(|n| n.text.eq_ignore_ascii_case(text)) {
            return Ok(same.clone());
        }
        if notes.len() >= MAX_NOTES {
            return Err(format!(
                "{MAX_NOTES} notes are kept already; forget one the operator no longer needs"
            ));
        }
        let note = Note {
            id: notes.iter().map(|n| n.id).max().unwrap_or(0) + 1,
            text: text.to_owned(),
            at: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_or(0, |d| d.as_secs()),
        };
        notes.push(note.clone());
        self.save(&notes)?;
        Ok(note)
    }

    /// Forgets a note by number; `false` when there was none.
    ///
    /// # Errors
    ///
    /// The file cannot be written.
    pub fn forget(&self, id: u32) -> Result<bool, String> {
        let mut notes = self.lock();
        let before = notes.len();
        notes.retain(|n| n.id != id);
        if notes.len() == before {
            return Ok(false);
        }
        self.save(&notes).map(|()| true)
    }
}

/// What the system prompt gains on every turn.
pub trait Notes: Send + Sync + std::fmt::Debug {
    /// Lines to add, or nothing.
    fn text(&self) -> String;
}

impl Notes for Memory {
    fn text(&self) -> String {
        let notes = self.lock();
        if notes.is_empty() {
            return String::new();
        }
        let mut out =
            "\n\nWhat the operator asked you to remember (the `memory` tool keeps it):".to_owned();
        for n in notes.iter() {
            let _ = write!(out, "\n- [{}] {}", n.id, n.text);
        }
        out
    }
}

/// The `memory` tool.
pub struct MemoryTool {
    spec: ToolSpec,
    memory: Arc<Memory>,
}

impl MemoryTool {
    /// Remembering and forgetting change what the agent knows, so the operator approves them
    /// when supervised; listing is a read.
    #[must_use]
    pub fn new(memory: Arc<Memory>) -> Self {
        let spec = ToolSpec::new(
            "memory",
            "Keeps what the operator asks you to remember across sessions, such as \"the kitchen \
             door sticks\" or \"always use the left arm\", and forgets it when asked. Kept notes \
             are listed in your instructions on every turn. Save only what the operator asks you \
             to remember.",
            json!({"type": "object", "properties": {
                "action": {"type": "string", "enum": ["remember", "forget", "list"]},
                "text": {"type": "string", "description": "For remember: the fact, in one sentence."},
                "id": {"type": "integer", "description": "For forget: the note's number."}
            }, "required": ["action"], "additionalProperties": false}),
            Risk::Annotate,
        );
        Self { spec, memory }
    }
}

#[async_trait]
impl Tool for MemoryTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let (risk, reason) = match args["action"].as_str() {
            Some("list") => (Risk::Observe, "lists the notes".to_owned()),
            Some("remember") => (
                Risk::Annotate,
                format!(
                    "remembers \"{}\"",
                    args["text"].as_str().unwrap_or_default()
                ),
            ),
            Some("forget") => (Risk::Annotate, format!("forgets note {}", args["id"])),
            _ => {
                return Some(Err(ToolOutcome::failed(
                    "`action` is remember, forget or list",
                )));
            }
        };
        Some(Ok(Assessment {
            risk,
            resources: Vec::new(),
            reason,
            args: None,
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str() {
            Some("remember") => match self
                .memory
                .remember(args["text"].as_str().unwrap_or_default())
            {
                Ok(note) => {
                    let mut out = ToolOutcome::ok(json!({"id": note.id, "text": note.text}));
                    out.message = format!("remembered as note {}", note.id);
                    out
                }
                Err(why) => ToolOutcome::failed(why),
            },
            Some("forget") => {
                let id = args["id"]
                    .as_u64()
                    .and_then(|i| u32::try_from(i).ok())
                    .unwrap_or(0);
                match self.memory.forget(id) {
                    Ok(true) => ToolOutcome::ok(json!({"forgot": id})),
                    Ok(false) => ToolOutcome::failed(format!("no note {id}; list them first")),
                    Err(why) => ToolOutcome::failed(why),
                }
            }
            _ => ToolOutcome::ok(json!({"notes": self.memory.all()})),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn notes_are_kept_across_sessions_and_put_in_the_prompt() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("memory").join("r.json");
        let memory = Memory::new(Some(file.clone()));
        assert_eq!(memory.text(), "");
        let door = memory.remember("The kitchen door sticks.").unwrap();
        assert_eq!(
            memory.remember("the kitchen door sticks.").unwrap().id,
            door.id
        );
        memory.remember("Always use the left arm.").unwrap();
        let again = Memory::new(Some(file));
        assert!(
            again.text().contains("[1] The kitchen door sticks.")
                && again.text().contains("[2] Always use")
        );
        assert!(again.forget(1).unwrap() && !again.forget(1).unwrap());
        assert_eq!(again.all().len(), 1);
        assert!(memory.remember(&"x".repeat(301)).is_err());
    }
}
