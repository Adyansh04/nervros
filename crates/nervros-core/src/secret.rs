//! Loading API keys without ever printing them.
//!
//! A key is read once from a file or an environment variable into a [`SecretString`]. Errors name
//! the source, never the value.

use std::path::{Path, PathBuf};

use secrecy::SecretString;
use serde::Deserialize;

/// Where a provider's API key comes from.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(untagged)]
pub enum KeySource {
    /// A file holding only the key, such as `~/.config/openrouter.key` (mode 0600).
    File {
        /// Path to the file; a leading `~/` means the home directory.
        file: PathBuf,
    },
    /// An environment variable holding the key.
    Env {
        /// Name of the variable.
        env: String,
    },
}

/// Why a key could not be loaded. Never carries the key itself.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum SecretError {
    /// The key file could not be read.
    #[error("cannot read the key file {path}: {source}")]
    File {
        /// The file that was tried.
        path: PathBuf,
        /// The underlying error.
        source: std::io::Error,
    },
    /// The environment variable is not set or not valid Unicode.
    #[error("environment variable {0} is not set")]
    Env(String),
    /// The file or variable is empty.
    #[error("the key from {0} is empty")]
    Empty(String),
}

impl KeySource {
    /// Reads the key.
    ///
    /// # Errors
    ///
    /// [`SecretError`] when the file or variable is missing or empty.
    pub fn load(&self) -> Result<SecretString, SecretError> {
        let (raw, origin) = match self {
            Self::File { file } => {
                let path = expand_home(file);
                let text = std::fs::read_to_string(&path).map_err(|source| SecretError::File {
                    path: path.clone(),
                    source,
                })?;
                (text, path.display().to_string())
            }
            Self::Env { env } => {
                let value = std::env::var(env).map_err(|_| SecretError::Env(env.clone()))?;
                (value, format!("${env}"))
            }
        };
        let key = raw.trim();
        if key.is_empty() {
            return Err(SecretError::Empty(origin));
        }
        Ok(SecretString::from(key.to_owned()))
    }
}

/// Replaces a leading `~/` with the home directory; other paths are returned unchanged.
#[must_use]
pub fn expand_home(path: &Path) -> PathBuf {
    match (path.strip_prefix("~"), std::env::var_os("HOME")) {
        (Ok(rest), Some(home)) => PathBuf::from(home).join(rest),
        _ => path.to_path_buf(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use secrecy::ExposeSecret;

    #[test]
    fn reads_and_trims_a_key_file() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("key");
        std::fs::write(&path, "  sk-test-123\n").unwrap();
        let key = KeySource::File { file: path }.load().unwrap();
        assert_eq!(key.expose_secret(), "sk-test-123");
    }

    #[test]
    fn errors_never_contain_the_key() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("key");
        std::fs::write(&path, "\n").unwrap();
        let err = KeySource::File { file: path }.load().unwrap_err();
        assert!(matches!(err, SecretError::Empty(_)));
        let missing = KeySource::Env {
            env: "NERVROS_TEST_UNSET_VAR".into(),
        }
        .load()
        .unwrap_err();
        assert_eq!(
            missing.to_string(),
            "environment variable NERVROS_TEST_UNSET_VAR is not set"
        );
    }

    #[test]
    fn expands_a_leading_tilde_only() {
        let home = std::env::var_os("HOME").map(PathBuf::from).unwrap();
        assert_eq!(expand_home(Path::new("~/a/b")), home.join("a/b"));
        assert_eq!(expand_home(Path::new("/x/~/y")), PathBuf::from("/x/~/y"));
    }
}
