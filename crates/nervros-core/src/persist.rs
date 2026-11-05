//! The small JSON files the agent keeps between runs: notes, places, quota counts, a conversation,
//! the requests left waiting. A write goes to a temporary file that replaces the old one only once
//! complete, so a crash mid-write leaves the old file whole. A file that cannot be read is moved
//! aside, never overwritten, and the store starts empty.

use std::io::Write as _;
use std::path::{Path, PathBuf};

use serde::Serialize;
use serde::de::DeserializeOwned;

/// Writes `bytes` to `path` whole or not at all, making its folder first.
///
/// # Errors
///
/// The folder, the temporary file or the rename failed.
pub fn write_atomic(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir)?;
    }
    let tmp = beside(path, &format!("{}.tmp", std::process::id()));
    let written = std::fs::File::create(&tmp).and_then(|mut f| {
        f.write_all(bytes)?;
        f.sync_all()
    });
    match written.and_then(|()| std::fs::rename(&tmp, path)) {
        Ok(()) => Ok(()),
        Err(e) => {
            let _ = std::fs::remove_file(&tmp);
            Err(e)
        }
    }
}

/// `value` as pretty JSON in `path`, whole or not at all.
///
/// # Errors
///
/// `value` cannot be serialised, or the file cannot be written; the text names the file.
pub fn write_json<T: Serialize + ?Sized>(path: &Path, value: &T) -> Result<(), String> {
    let text = serde_json::to_vec_pretty(value).map_err(|e| e.to_string())?;
    write_atomic(path, &text).map_err(|e| format!("saving {}: {e}", path.display()))
}

/// `path` read as JSON, or the default when there is no such file. One that cannot be read as a
/// `T` is moved aside as `<name>.corrupt`, with a warning, so that the next save does not destroy
/// what it held.
#[must_use]
pub fn read_or_default<T: DeserializeOwned + Default>(path: &Path) -> T {
    let text = match std::fs::read(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return T::default(),
        Err(e) => {
            tracing::warn!(file = %path.display(), error = %e, "unreadable; starting empty");
            return T::default();
        }
    };
    match serde_json::from_slice(&text) {
        Ok(value) => value,
        Err(e) => {
            let aside = beside(path, "corrupt");
            let moved = std::fs::rename(path, &aside);
            tracing::warn!(
                file = %path.display(),
                error = %e,
                kept_as = %aside.display(),
                moved = moved.is_ok(),
                "not readable as it should be; starting empty"
            );
            T::default()
        }
    }
}

/// `path` with `.suffix` added to its file name.
fn beside(path: &Path, suffix: &str) -> PathBuf {
    let mut name = path.as_os_str().to_owned();
    name.push(".");
    name.push(suffix);
    PathBuf::from(name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_write_replaces_the_file_whole_and_a_corrupt_file_is_kept_aside() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("deep").join("notes.json");
        write_atomic(&path, b"[1, 2]").unwrap();
        assert_eq!(read_or_default::<Vec<u32>>(&path), [1, 2]);
        assert!(!beside(&path, &format!("{}.tmp", std::process::id())).exists());
        std::fs::write(&path, b"[1, 2").unwrap();
        assert!(read_or_default::<Vec<u32>>(&path).is_empty());
        assert_eq!(std::fs::read(beside(&path, "corrupt")).unwrap(), b"[1, 2");
        assert!(read_or_default::<Vec<u32>>(&dir.path().join("none.json")).is_empty());
    }
}
