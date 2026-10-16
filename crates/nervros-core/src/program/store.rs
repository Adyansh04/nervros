//! Programs kept by name, per robot, in the state folder, to run again.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use serde::{Deserialize, Serialize};

use crate::lock;
use crate::persist::{read_or_default, write_atomic};

/// Programs kept at most, per robot.
const MAX_SAVED: usize = 32;

/// A program kept by name.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Saved {
    /// Its name, such as "count the mugs".
    pub name: String,
    /// The program.
    pub source: String,
    /// Its time limit.
    pub max_minutes: u64,
    /// Unix seconds.
    pub saved_s: f64,
    /// How often it has run since.
    #[serde(default)]
    pub runs: u32,
}

#[derive(Debug, Default, Serialize, Deserialize)]
struct File {
    programs: Vec<Saved>,
}

/// The kept programs, written through to their file.
#[derive(Debug)]
pub struct Store {
    path: Option<PathBuf>,
    list: Mutex<Vec<Saved>>,
}

impl Store {
    /// The programs kept at `path`, or none kept anywhere.
    #[must_use]
    pub fn open(path: Option<PathBuf>) -> Arc<Self> {
        let list = path
            .as_deref()
            .map(|p| read_or_default::<File>(p).programs)
            .unwrap_or_default();
        Arc::new(Self {
            path,
            list: Mutex::new(list),
        })
    }

    /// The kept programs, newest last.
    #[must_use]
    pub fn list(&self) -> Vec<Saved> {
        lock(&self.list).clone()
    }

    /// The program kept as `name`, matched without regard to case.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<Saved> {
        lock(&self.list)
            .iter()
            .find(|p| p.name.eq_ignore_ascii_case(name.trim()))
            .cloned()
    }

    /// Keeps `saved`, replacing one of its name.
    ///
    /// # Errors
    ///
    /// The store is full, or the file could not be written.
    pub fn save(&self, saved: Saved) -> Result<(), String> {
        let mut list = lock(&self.list);
        let mut next = list.clone();
        next.retain(|p| !p.name.eq_ignore_ascii_case(&saved.name));
        if next.len() >= MAX_SAVED {
            return Err(format!(
                "{MAX_SAVED} programs are kept already; forget one first"
            ));
        }
        next.push(saved);
        self.write(&next)?;
        *list = next;
        Ok(())
    }

    /// Forgets the program kept as `name`; whether there was one.
    ///
    /// # Errors
    ///
    /// The file could not be written.
    pub fn forget(&self, name: &str) -> Result<bool, String> {
        let mut list = lock(&self.list);
        let mut next = list.clone();
        next.retain(|p| !p.name.eq_ignore_ascii_case(name.trim()));
        if next.len() == list.len() {
            return Ok(false);
        }
        self.write(&next)?;
        *list = next;
        Ok(true)
    }

    /// Counts a run of the program kept as `name`; a failed write only loses the count.
    pub fn ran(&self, name: &str) {
        let mut list = lock(&self.list);
        let mut next = list.clone();
        if let Some(p) = next.iter_mut().find(|p| p.name.eq_ignore_ascii_case(name)) {
            p.runs += 1;
            if self.write(&next).is_ok() {
                *list = next;
            }
        }
    }

    /// Written first and kept in memory only once on disk, so the two never disagree.
    fn write(&self, list: &[Saved]) -> Result<(), String> {
        let Some(path) = &self.path else {
            return Ok(());
        };
        let text = serde_json::to_vec_pretty(&serde_json::json!({"programs": list}))
            .map_err(|e| e.to_string())?;
        write_atomic(path, &text).map_err(|e| format!("could not keep the program: {e}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn saved(name: &str) -> Saved {
        Saved {
            name: name.to_owned(),
            source: "1".to_owned(),
            max_minutes: 5,
            saved_s: 0.0,
            runs: 0,
        }
    }

    #[test]
    fn programs_are_kept_by_name_across_restarts() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("programs").join("t.json");
        let store = Store::open(Some(path.clone()));
        store.save(saved("count mugs")).unwrap();
        store.save(saved("Count Mugs")).unwrap();
        store.ran("count mugs");
        assert_eq!(store.list().len(), 1, "same name, replaced");

        let again = Store::open(Some(path));
        assert_eq!(again.get("COUNT MUGS").unwrap().runs, 1);
        assert!(again.forget("count mugs").unwrap());
        assert!(!again.forget("count mugs").unwrap());
        assert!(again.get("count mugs").is_none());
    }
}
