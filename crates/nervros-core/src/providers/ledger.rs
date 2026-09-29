//! Request counts per model and per pool, persisted so a restart keeps today's counts.
//!
//! Daily counts reset at the provider's own midnight: UTC for OpenRouter, Pacific time for Gemini.
//! Per-minute counts live in memory only. A model refused with 429 is parked until a given time.

use std::collections::{BTreeMap, HashMap, VecDeque};
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};

/// Which midnight a daily quota resets at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetZone {
    /// 00:00 UTC.
    Utc,
    /// 00:00 in California.
    Pacific,
}

/// Seconds since the Unix epoch, saturating at zero for clocks before 1970.
#[must_use]
pub fn unix_secs(now: SystemTime) -> u64 {
    now.duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs())
}

/// The day number a timestamp falls in, for a reset zone.
#[must_use]
pub fn day_index(now: SystemTime, zone: ResetZone) -> u64 {
    // ponytail: Pacific is taken as UTC-8 all year, so from March to November the day turns an
    // hour late. A 429 still parks the model at the real reset; use a tz database if that matters.
    let offset = match zone {
        ResetZone::Utc => 0,
        ResetZone::Pacific => 8 * 3600,
    };
    unix_secs(now).saturating_sub(offset) / 86_400
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
struct DayCount {
    day: u64,
    count: u32,
}

/// Counts, parked models and their persistence.
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Ledger {
    days: BTreeMap<String, DayCount>,
    parked_until: BTreeMap<String, u64>,
    #[serde(skip)]
    minute: HashMap<String, VecDeque<u64>>,
    #[serde(skip)]
    path: Option<PathBuf>,
}

/// Why a model cannot take a request now.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Refused {
    /// Parked after a 429 until this Unix time.
    Parked(u64),
    /// Its own daily limit is used up.
    Daily,
    /// Its shared pool's daily limit is used up.
    Pool(String),
    /// Its per-minute limit is used up.
    Minute,
}

impl Ledger {
    /// Loads the ledger from a JSON file, or starts empty when the file does not exist.
    ///
    /// # Errors
    ///
    /// An unreadable or corrupt file.
    pub fn load(path: &Path) -> std::io::Result<Self> {
        let mut ledger = match std::fs::read_to_string(path) {
            Ok(text) => serde_json::from_str(&text).map_err(std::io::Error::other)?,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Self::default(),
            Err(e) => return Err(e),
        };
        ledger.path = Some(path.to_path_buf());
        Ok(ledger)
    }

    /// Writes the ledger back to its file, if it has one.
    ///
    /// # Errors
    ///
    /// A write failure.
    pub fn save(&self) -> std::io::Result<()> {
        let Some(path) = &self.path else {
            return Ok(());
        };
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let text = serde_json::to_string_pretty(self).map_err(std::io::Error::other)?;
        std::fs::write(path, text)
    }

    /// Requests counted today under `key` (a model id or a pool name).
    #[must_use]
    pub fn used_today(&self, key: &str, now: SystemTime, zone: ResetZone) -> u32 {
        let today = day_index(now, zone);
        self.days
            .get(key)
            .filter(|d| d.day == today)
            .map_or(0, |d| d.count)
    }

    /// Checks whether a model may take one more request.
    ///
    /// # Errors
    ///
    /// [`Refused`] naming the limit that blocks it.
    pub fn check(
        &self,
        model: &str,
        limits: &super::Limits,
        pool_rpd: Option<(&str, u32)>,
        now: SystemTime,
        zone: ResetZone,
    ) -> Result<(), Refused> {
        let t = unix_secs(now);
        if let Some(&until) = self.parked_until.get(model)
            && until > t
        {
            return Err(Refused::Parked(until));
        }
        if limits
            .rpd
            .is_some_and(|rpd| self.used_today(model, now, zone) >= rpd)
        {
            return Err(Refused::Daily);
        }
        if let Some((pool, rpd)) = pool_rpd
            && self.used_today(pool, now, zone) >= rpd
        {
            return Err(Refused::Pool(pool.to_owned()));
        }
        if let Some(rpm) = limits.rpm {
            let recent = self
                .minute
                .get(model)
                .map_or(0, |q| q.iter().filter(|&&s| s + 60 > t).count());
            if recent >= usize::try_from(rpm).unwrap_or(usize::MAX) {
                return Err(Refused::Minute);
            }
        }
        Ok(())
    }

    /// Counts one request against a model and, if given, its pool.
    pub fn record(&mut self, model: &str, pool: Option<&str>, now: SystemTime, zone: ResetZone) {
        let today = day_index(now, zone);
        for key in std::iter::once(model).chain(pool) {
            let entry = self.days.entry(key.to_owned()).or_default();
            if entry.day != today {
                *entry = DayCount {
                    day: today,
                    count: 0,
                };
            }
            entry.count = entry.count.saturating_add(1);
        }
        let t = unix_secs(now);
        let q = self.minute.entry(model.to_owned()).or_default();
        q.push_back(t);
        while q.front().is_some_and(|&s| s + 60 <= t) {
            q.pop_front();
        }
    }

    /// Parks a model after a 429 for `for_how_long`.
    pub fn park(&mut self, model: &str, now: SystemTime, for_how_long: Duration) {
        self.parked_until
            .insert(model.to_owned(), unix_secs(now + for_how_long));
    }

    /// Replaces today's count for a pool with the provider's own figure, such as OpenRouter's
    /// `free_model_daily_requests.used`.
    pub fn set_pool_used(&mut self, pool: &str, used: u32, now: SystemTime, zone: ResetZone) {
        self.days.insert(
            pool.to_owned(),
            DayCount {
                day: day_index(now, zone),
                count: used,
            },
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::providers::Limits;

    fn at(secs: u64) -> SystemTime {
        UNIX_EPOCH + Duration::from_secs(secs)
    }

    #[test]
    fn daily_limits_reset_at_the_zones_midnight() {
        let mut ledger = Ledger::default();
        let limits = Limits {
            rpd: Some(2),
            ..Limits::default()
        };
        let noon = at(86_400 * 100 + 12 * 3600);
        ledger.record("m", None, noon, ResetZone::Utc);
        ledger.record("m", None, noon, ResetZone::Utc);
        assert_eq!(
            ledger.check("m", &limits, None, noon, ResetZone::Utc),
            Err(Refused::Daily)
        );
        let next_day = at(86_400 * 101 + 60);
        assert!(
            ledger
                .check("m", &limits, None, next_day, ResetZone::Utc)
                .is_ok()
        );
        // 07:00 UTC is still the previous day in Pacific time.
        let early = at(86_400 * 101 + 7 * 3600);
        assert_eq!(day_index(early, ResetZone::Pacific), 100);
    }

    #[test]
    fn a_pool_is_shared_between_models() {
        let mut ledger = Ledger::default();
        let t = at(86_400 * 5);
        ledger.record("a", Some("free"), t, ResetZone::Utc);
        ledger.record("b", Some("free"), t, ResetZone::Utc);
        let limits = Limits::default();
        assert_eq!(
            ledger.check("c", &limits, Some(("free", 2)), t, ResetZone::Utc),
            Err(Refused::Pool("free".into()))
        );
    }

    #[test]
    fn per_minute_limits_and_parking() {
        let mut ledger = Ledger::default();
        let limits = Limits {
            rpm: Some(1),
            ..Limits::default()
        };
        let t = at(1_000_000);
        ledger.record("m", None, t, ResetZone::Utc);
        assert_eq!(
            ledger.check("m", &limits, None, t, ResetZone::Utc),
            Err(Refused::Minute)
        );
        assert!(
            ledger
                .check("m", &limits, None, at(1_000_061), ResetZone::Utc)
                .is_ok()
        );
        ledger.park("m", t, Duration::from_secs(30));
        assert!(matches!(
            ledger.check("m", &Limits::default(), None, at(1_000_010), ResetZone::Utc),
            Err(Refused::Parked(_))
        ));
    }

    #[test]
    fn counts_survive_a_restart() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("quota.json");
        let t = at(86_400 * 7);
        let mut ledger = Ledger::load(&path).unwrap();
        ledger.record("m", Some("p"), t, ResetZone::Utc);
        ledger.save().unwrap();
        let again = Ledger::load(&path).unwrap();
        assert_eq!(again.used_today("m", t, ResetZone::Utc), 1);
        assert_eq!(again.used_today("p", t, ResetZone::Utc), 1);
    }
}
