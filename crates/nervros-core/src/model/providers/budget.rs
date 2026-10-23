//! Dollar budgets for paid providers: each call's tokens priced and added to one file that every
//! NervROS process on the machine shares, so no run starts again from nothing, and no call starts
//! once the budget is spent.

use std::fs::{File, OpenOptions};
use std::io::{Read as _, Seek as _, SeekFrom, Write as _};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// US dollars per million tokens.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Price {
    /// Prompt tokens sent fresh.
    pub input: f64,
    /// Reply tokens, thinking included.
    pub output: f64,
    /// Prompt tokens read from the provider's cache.
    pub cache_read: f64,
    /// Prompt tokens written to the provider's cache.
    pub cache_write: f64,
}

impl Price {
    /// Whether every figure is a finite, non-negative price.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        [self.input, self.output, self.cache_read, self.cache_write]
            .iter()
            .all(|p| p.is_finite() && *p >= 0.0)
    }

    /// What a call's tokens cost, in US dollars.
    #[must_use]
    pub fn cost(&self, tokens: Tokens) -> f64 {
        let fresh = tokens
            .input
            .saturating_sub(tokens.cached.saturating_add(tokens.cache_write));
        let millions = |n: u64| f64::from(u32::try_from(n).unwrap_or(u32::MAX)) / 1e6;
        millions(fresh) * self.input
            + millions(tokens.cached) * self.cache_read
            + millions(tokens.cache_write) * self.cache_write
            + millions(tokens.output) * self.output
    }
}

/// One call's tokens as the provider counted them.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Tokens {
    /// Every prompt token: fresh, read from the cache and written to it.
    pub input: u64,
    /// Of those, read from the cache.
    pub cached: u64,
    /// Of those, written to the cache.
    pub cache_write: u64,
    /// Reply tokens.
    pub output: u64,
}

/// A provider's budget, as `models.toml` gives it.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BudgetConfig {
    /// US dollars the key may spend in all; no call starts once this much is spent.
    pub usd: f64,
    /// Where the spend is counted; a leading `~/` means the home directory.
    pub spent_file: PathBuf,
}

/// The spend file's contents.
#[derive(Debug, Default, Serialize, Deserialize)]
struct Spent {
    usd: f64,
}

/// A budget and its spend file.
#[derive(Debug, Clone)]
pub struct Budget {
    usd: f64,
    path: PathBuf,
}

impl Budget {
    /// The budget `config` describes, its file's `~/` resolved.
    #[must_use]
    pub fn new(config: &BudgetConfig) -> Self {
        Self {
            usd: config.usd,
            path: crate::secret::expand_home(&config.spent_file),
        }
    }

    /// The budget in US dollars.
    #[must_use]
    pub fn usd(&self) -> f64 {
        self.usd
    }

    /// US dollars spent so far. An unreadable file counts as the whole budget spent: a budget
    /// that cannot be read must not be spent past.
    #[must_use]
    pub fn spent(&self) -> f64 {
        match self.read() {
            Ok(usd) => usd,
            Err(e) => {
                tracing::warn!(file = %self.path.display(), error = %e, "the spend file is unreadable");
                self.usd
            }
        }
    }

    /// Whether another call may start.
    #[must_use]
    pub fn allows(&self) -> bool {
        self.spent() < self.usd
    }

    /// Adds `usd` to the spend, under an exclusive lock so two processes cannot lose a call.
    ///
    /// # Errors
    ///
    /// The file cannot be opened, locked, read or written.
    pub fn add(&self, usd: f64) -> std::io::Result<f64> {
        if let Some(dir) = self.path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&self.path)?;
        file.lock()?;
        let spent = read_spent(&mut file)? + usd.max(0.0);
        let text = serde_json::to_vec(&Spent { usd: spent }).map_err(std::io::Error::other)?;
        file.seek(SeekFrom::Start(0))?;
        file.set_len(0)?;
        file.write_all(&text)?;
        file.sync_all()?;
        file.unlock()?;
        Ok(spent)
    }

    fn read(&self) -> std::io::Result<f64> {
        let mut file = match File::open(&self.path) {
            Ok(file) => file,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(0.0),
            Err(e) => return Err(e),
        };
        file.lock_shared()?;
        let spent = read_spent(&mut file);
        file.unlock()?;
        spent
    }

    /// The spend file's path.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }
}

/// The spend a file holds; an empty file is a new budget.
fn read_spent(file: &mut File) -> std::io::Result<f64> {
    let mut text = String::new();
    file.read_to_string(&mut text)?;
    if text.trim().is_empty() {
        return Ok(0.0);
    }
    let spent: Spent = serde_json::from_str(&text).map_err(std::io::Error::other)?;
    if spent.usd.is_finite() && spent.usd >= 0.0 {
        Ok(spent.usd)
    } else {
        Err(std::io::Error::other(
            "the spend is not a finite, non-negative number",
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HAIKU: Price = Price {
        input: 0.10,
        output: 0.50,
        cache_read: 0.01,
        cache_write: 0.125,
    };

    #[test]
    fn each_kind_of_token_is_priced_on_its_own() {
        // 12k prompt of which 10k cached and 1k written, 2k reply.
        let tokens = Tokens {
            input: 12_000,
            cached: 10_000,
            cache_write: 1_000,
            output: 2_000,
        };
        let usd = HAIKU.cost(tokens);
        let expected = 0.001 * 0.10 + 0.010 * 0.01 + 0.001 * 0.125 + 0.002 * 0.50;
        assert!((usd - expected).abs() < 1e-12, "{usd}");
    }

    #[test]
    fn the_spend_adds_up_in_its_file_and_stops_calls_at_the_budget() {
        let dir = tempfile::tempdir().unwrap();
        let budget = Budget::new(&BudgetConfig {
            usd: 1.0,
            spent_file: dir.path().join("sub/spend.json"),
        });
        assert!((budget.spent() - 0.0).abs() < f64::EPSILON);
        assert!(budget.allows());
        budget.add(0.4).unwrap();
        // Another process's view of the same file.
        let other = Budget::new(&BudgetConfig {
            usd: 1.0,
            spent_file: dir.path().join("sub/spend.json"),
        });
        other.add(0.7).unwrap();
        assert!((budget.spent() - 1.1).abs() < 1e-9);
        assert!(!budget.allows());
    }

    #[test]
    fn a_spend_file_that_cannot_be_read_counts_as_spent() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("spend.json");
        std::fs::write(&path, "not json").unwrap();
        let budget = Budget::new(&BudgetConfig {
            usd: 5.0,
            spent_file: path,
        });
        assert!(!budget.allows());
        assert!(budget.add(0.1).is_err());
    }

    #[test]
    fn prices_must_be_finite_and_not_negative() {
        assert!(HAIKU.is_valid());
        assert!(
            !Price {
                input: -1.0,
                ..HAIKU
            }
            .is_valid()
        );
        assert!(
            !Price {
                output: f64::NAN,
                ..HAIKU
            }
            .is_valid()
        );
    }
}
