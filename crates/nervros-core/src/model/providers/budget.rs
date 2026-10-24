//! Dollar budgets for paid providers: each call's tokens priced and added to one file that every
//! NervROS process on the machine shares, so no run starts again from nothing, and no call starts
//! once the budget is spent.

use std::ffi::OsString;
use std::fs::{File, OpenOptions};
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

/// How long a reading of the spend file stands: the window asks every frame, and another
/// process's spend shows within this.
const REREAD: Duration = Duration::from_secs(2);

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

/// What this process knows of the spend.
#[derive(Debug, Default)]
struct Seen {
    /// The file's spend as last read or written, and when.
    usd: f64,
    at: Option<Instant>,
    /// Charged here and not yet in the file.
    pending: f64,
    /// A charge could not be written: the budget counts as spent from then on.
    lost: bool,
}

/// A budget and its spend file.
#[derive(Debug)]
pub struct Budget {
    usd: f64,
    path: PathBuf,
    seen: Arc<Mutex<Seen>>,
}

impl Budget {
    /// The budget `config` describes, its file's `~/` resolved.
    #[must_use]
    pub fn new(config: &BudgetConfig) -> Self {
        Self {
            usd: config.usd,
            path: crate::secret::expand_home(&config.spent_file),
            seen: Arc::default(),
        }
    }

    /// The budget in US dollars.
    #[must_use]
    pub fn usd(&self) -> f64 {
        self.usd
    }

    /// US dollars spent so far, this process's charges on their way to the file included. A file
    /// that cannot be read, or a charge that could not be written, counts as the whole budget
    /// spent: a budget that cannot be counted must not be spent past.
    #[must_use]
    pub fn spent(&self) -> f64 {
        let mut seen = crate::lock(&self.seen);
        if seen.lost {
            return self.usd;
        }
        if seen.at.is_none_or(|at| at.elapsed() >= REREAD) {
            seen.usd = read(&self.path).unwrap_or_else(|e| {
                tracing::warn!(file = %self.path.display(), error = %e, "the spend file is unreadable");
                self.usd
            });
            seen.at = Some(Instant::now());
        }
        seen.usd + seen.pending
    }

    /// Whether another call may start.
    #[must_use]
    pub fn allows(&self) -> bool {
        self.spent() < self.usd
    }

    /// Charges `usd` for a call to `model`: counted here at once, and added to the file off the
    /// async runtime when there is one, since the write waits on a lock and the disk.
    pub fn charge(&self, usd: f64, model: &str) {
        let usd = usd.max(0.0);
        crate::lock(&self.seen).pending += usd;
        let (path, seen, budget, model) = (
            self.path.clone(),
            Arc::clone(&self.seen),
            self.usd,
            model.to_owned(),
        );
        let write = move || {
            let added = add(&path, usd);
            let mut seen = crate::lock(&seen);
            seen.pending = (seen.pending - usd).max(0.0);
            match added {
                Ok(spent) => {
                    seen.usd = spent;
                    seen.at = Some(Instant::now());
                    tracing::info!(model = %model, call_usd = usd, spent_usd = spent, budget_usd = budget, "spend");
                }
                Err(e) => {
                    seen.lost = true;
                    tracing::warn!(model = %model, file = %path.display(), error = %e, "could not count the spend; the budget now reads as spent");
                }
            }
        };
        match tokio::runtime::Handle::try_current() {
            Ok(runtime) => drop(runtime.spawn_blocking(write)),
            Err(_) => write(),
        }
    }

    /// The spend file's path.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }
}

/// `path` with `suffix` added to its name.
fn beside(path: &Path, suffix: &str) -> PathBuf {
    let mut name = OsString::from(path.as_os_str());
    name.push(suffix);
    PathBuf::from(name)
}

/// Adds `usd` to the spend in `path` under an exclusive lock on a file beside it, so two processes
/// cannot lose a call, and replaces the file whole: a crash leaves the old spend or the new one.
fn add(path: &Path, usd: f64) -> std::io::Result<f64> {
    let dir = path.parent().unwrap_or_else(|| Path::new("."));
    std::fs::create_dir_all(dir)?;
    let lock = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(false)
        .open(beside(path, ".lock"))?;
    lock.lock()?;
    let spent = read(path)? + usd;
    let next = beside(path, ".next");
    let mut file = File::create(&next)?;
    file.write_all(&serde_json::to_vec(&Spent { usd: spent }).map_err(std::io::Error::other)?)?;
    file.sync_all()?;
    std::fs::rename(&next, path)?;
    // The rename itself is on disk only once the directory is.
    File::open(dir)?.sync_all()?;
    lock.unlock()?;
    Ok(spent)
}

/// The spend in `path`: none yet when there is no file. A file is only ever replaced whole, so an
/// empty one was damaged, not started.
fn read(path: &Path) -> std::io::Result<f64> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(0.0),
        Err(e) => return Err(e),
    };
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
        budget.charge(0.4, "m");
        // Other processes' views of the same file: one charges, one only reads.
        let process = || {
            Budget::new(&BudgetConfig {
                usd: 1.0,
                spent_file: dir.path().join("sub/spend.json"),
            })
        };
        process().charge(0.7, "m");
        let other = process();
        assert!((other.spent() - 1.1).abs() < 1e-9);
        assert!(!other.allows());
        assert!(
            (budget.spent() - 0.4).abs() < 1e-9,
            "read again only once its reading is old"
        );
    }

    #[test]
    fn a_spend_file_that_cannot_be_read_counts_as_spent() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("spend.json");
        std::fs::write(&path, "not json").unwrap();
        let budget = Budget::new(&BudgetConfig {
            usd: 5.0,
            spent_file: path.clone(),
        });
        assert!(!budget.allows());
        // Written whole, so an empty file was damaged rather than started.
        std::fs::write(&path, "").unwrap();
        let budget = Budget::new(&BudgetConfig {
            usd: 5.0,
            spent_file: path,
        });
        assert!(!budget.allows());
    }

    #[test]
    fn a_charge_that_cannot_be_written_leaves_the_budget_spent() {
        let dir = tempfile::tempdir().unwrap();
        // The lock file cannot be opened where a directory stands, whoever runs the test.
        std::fs::create_dir(dir.path().join("spend.json.lock")).unwrap();
        let budget = Budget::new(&BudgetConfig {
            usd: 5.0,
            spent_file: dir.path().join("spend.json"),
        });
        assert!(budget.allows());
        budget.charge(0.1, "m");
        assert!(!budget.allows());
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
