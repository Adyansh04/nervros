//! Request counts per model and per pool, persisted so a restart keeps today's counts.
//!
//! Daily counts reset at the provider's own midnight: UTC for OpenRouter, Pacific time for Gemini.
//! Per-minute counts live in memory only. A model refused with 429 is parked until a given time.

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use serde::{Deserialize, Serialize};

/// Which midnight a daily quota resets at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetZone {
    /// 00:00 UTC.
    Utc,
    /// 00:00 in California.
    Pacific,
}

/// The day number a timestamp falls in, for a reset zone.
#[must_use]
pub fn day_index(now: SystemTime, zone: ResetZone) -> u64 {
    let secs = crate::unix_secs(now);
    let offset = match zone {
        ResetZone::Utc => 0,
        ResetZone::Pacific => pacific_offset(secs),
    };
    secs.saturating_sub(offset) / 86_400
}

/// How long from `now` until the zone's next midnight, when a daily quota comes back.
#[must_use]
pub fn until_next_day(now: SystemTime, zone: ResetZone) -> Duration {
    let secs = crate::unix_secs(now);
    let offset = match zone {
        ResetZone::Utc => 0,
        ResetZone::Pacific => pacific_offset(secs),
    };
    let next = (day_index(now, zone) + 1) * 86_400 + offset;
    Duration::from_secs(next.saturating_sub(secs))
}

/// Pacific time's lag behind UTC at `secs`: daylight time (UTC-7) from 02:00 on the second Sunday
/// in March to 02:00 on the first Sunday in November, standard time (UTC-8) otherwise.
fn pacific_offset(secs: u64) -> u64 {
    const HOUR: u64 = 3600;
    let year = civil_year(secs / 86_400);
    let starts = nth_sunday(year, 3, 2) * 86_400 + 10 * HOUR;
    let ends = nth_sunday(year, 11, 1) * 86_400 + 9 * HOUR;
    if (starts..ends).contains(&secs) {
        7 * HOUR
    } else {
        8 * HOUR
    }
}

/// The year a day since 1970-01-01 falls in: Howard Hinnant's `civil_from_days`.
fn civil_year(days: u64) -> u64 {
    let z = days + 719_468;
    let (era, doe) = (z / 146_097, z % 146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let march_based_month = (5 * doy + 2) / 153;
    era * 400 + yoe + u64::from(march_based_month >= 10)
}

/// The day since 1970-01-01 of a month's `n`th Sunday: Howard Hinnant's `days_from_civil`.
fn nth_sunday(year: u64, month: u64, n: u64) -> u64 {
    let y = if month <= 2 { year - 1 } else { year };
    let (era, yoe) = (y / 400, y % 400);
    let doy = (153 * ((month + 9) % 12) + 2) / 5;
    let first = era * 146_097 + yoe * 365 + yoe / 4 - yoe / 100 + doy - 719_468;
    // 1970-01-01 was a Thursday: day 3 is the first Sunday.
    let to_sunday = (7 - (first + 4) % 7) % 7;
    first + to_sunday + 7 * (n - 1)
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
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
    /// Models whose server did not answer, set aside by this process alone: that one process
    /// cannot reach a server says nothing of the model's quota.
    #[serde(skip)]
    silent_until: HashMap<String, u64>,
    #[serde(skip)]
    path: Option<PathBuf>,
    /// What this process counted since it last saved: a save adds it to what the file holds by
    /// then, so a window and a CLI on one robot count into the same day.
    #[serde(skip)]
    unsaved: BTreeMap<String, DayCount>,
    /// Counts set from a provider's own figure, which replace the file's.
    #[serde(skip)]
    set: BTreeSet<String>,
}

/// Why a model cannot take a request now.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Refused {
    /// Parked after a 429 until this Unix time.
    Parked(u64),
    /// Set aside until this Unix time after its server did not answer.
    Silent(u64),
    /// Its own daily limit is used up.
    Daily,
    /// Its shared pool's daily limit is used up.
    Pool(String),
    /// Its per-minute limit is used up.
    Minute,
}

impl std::fmt::Display for Refused {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Parked(_) => f.write_str("set aside after a 429"),
            Self::Silent(_) => f.write_str("set aside: its server did not answer"),
            Self::Daily => f.write_str("its daily limit is used up"),
            Self::Pool(pool) => write!(f, "the {pool} pool's daily limit is used up"),
            Self::Minute => f.write_str("its per-minute limit is used up"),
        }
    }
}

impl Ledger {
    /// Loads the ledger from a JSON file, or starts empty when there is none. A file that cannot
    /// be read is moved aside and the counts start again: better a 429 later than no agent.
    #[must_use]
    pub fn load(path: &Path) -> Self {
        let mut ledger: Self = crate::persist::read_or_default(path);
        ledger.path = Some(path.to_path_buf());
        ledger
    }

    /// Writes the ledger back to its file, if it has one, taking in first what another process
    /// saved there since: its counts and ours add up, and the later of two parks stands.
    ///
    /// # Errors
    ///
    /// A write failure.
    pub fn save(&mut self) -> std::io::Result<()> {
        let Some(path) = self.path.clone() else {
            return Ok(());
        };
        let disk: Self = crate::persist::read_or_default(&path);
        let mut days = disk.days;
        for (key, mine) in std::mem::take(&mut self.unsaved) {
            let entry = days.entry(key).or_default();
            if entry.day == mine.day {
                entry.count = entry.count.saturating_add(mine.count);
            } else if entry.day < mine.day {
                *entry = mine;
            }
        }
        for key in std::mem::take(&mut self.set) {
            if let Some(mine) = self.days.get(&key) {
                days.insert(key, mine.clone());
            }
        }
        let mut parked = disk.parked_until;
        for (model, until) in &self.parked_until {
            let entry = parked.entry(model.clone()).or_default();
            *entry = (*entry).max(*until);
        }
        self.days = days;
        self.parked_until = parked;
        let text = serde_json::to_vec_pretty(self).map_err(std::io::Error::other)?;
        crate::persist::write_atomic(&path, &text)
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
        let t = crate::unix_secs(now);
        if let Some(&until) = self.parked_until.get(model)
            && until > t
        {
            return Err(Refused::Parked(until));
        }
        if let Some(&until) = self.silent_until.get(model)
            && until > t
        {
            return Err(Refused::Silent(until));
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
            for counts in [&mut self.days, &mut self.unsaved] {
                let entry = counts.entry(key.to_owned()).or_default();
                if entry.day != today {
                    *entry = DayCount {
                        day: today,
                        count: 0,
                    };
                }
                entry.count = entry.count.saturating_add(1);
            }
        }
        let t = crate::unix_secs(now);
        let q = self.minute.entry(model.to_owned()).or_default();
        q.push_back(t);
        while q.front().is_some_and(|&s| s + 60 <= t) {
            q.pop_front();
        }
    }

    /// Parks a model after a 429 for `for_how_long`.
    pub fn park(&mut self, model: &str, now: SystemTime, for_how_long: Duration) {
        self.parked_until
            .insert(model.to_owned(), crate::unix_secs(now + for_how_long));
    }

    /// Sets a model whose server did not answer aside for `for_how_long`, in this process only.
    pub fn set_aside(&mut self, model: &str, now: SystemTime, for_how_long: Duration) {
        self.silent_until
            .insert(model.to_owned(), crate::unix_secs(now + for_how_long));
    }

    /// When a model's park, and each request of its last minute, stop counting against it, as
    /// offsets from `now`, soonest first: the moments a refusal may lift.
    #[must_use]
    pub fn lifts(&self, model: &str, now: SystemTime) -> Vec<Duration> {
        let t = crate::unix_secs(now);
        let mut at: Vec<u64> = self
            .parked_until
            .get(model)
            .copied()
            .into_iter()
            .chain(self.minute.get(model).into_iter().flatten().map(|s| s + 60))
            .filter(|&s| s > t)
            .collect();
        at.sort_unstable();
        at.into_iter().map(|s| Duration::from_secs(s - t)).collect()
    }

    /// Replaces today's count for a pool with the provider's own figure, such as OpenRouter's
    /// `free_model_daily_requests.used`.
    pub fn set_pool_used(&mut self, pool: &str, used: u32, now: SystemTime, zone: ResetZone) {
        self.unsaved.remove(pool);
        self.set.insert(pool.to_owned());
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
        std::time::UNIX_EPOCH + Duration::from_secs(secs)
    }

    #[test]
    fn the_next_day_starts_at_the_zones_midnight() {
        let noon = at(86_400 * 100 + 12 * 3600);
        assert_eq!(
            until_next_day(noon, ResetZone::Utc),
            Duration::from_hours(12)
        );
        // 2026-10-07 06:00 UTC is 23:00 the day before in California, on daylight time.
        let late = at(1_791_352_800);
        assert_eq!(
            until_next_day(late, ResetZone::Pacific),
            Duration::from_hours(1)
        );
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
        let early = at(86_400 * 10 + 7 * 3600);
        assert_eq!(
            day_index(early, ResetZone::Pacific),
            9,
            "07:00 UTC in winter"
        );
    }

    #[test]
    fn pacific_days_turn_at_midnight_in_summer_and_in_winter() {
        let day = |secs| day_index(at(secs), ResetZone::Pacific);
        assert_eq!(day(1_791_010_799), 20_728, "2026-10-03 06:59:59 UTC");
        assert_eq!(day(1_791_010_801), 20_729, "2026-10-03 07:00:01 UTC");
        assert_eq!(day(1_768_460_400), 20_467, "2026-01-15 07:00 UTC");
        assert_eq!(day(1_768_464_000), 20_468, "2026-01-15 08:00 UTC");
        assert_eq!(pacific_offset(1_772_963_999), 8 * 3600);
        assert_eq!(pacific_offset(1_772_964_000), 7 * 3600);
        assert_eq!(pacific_offset(1_793_523_599), 7 * 3600);
        assert_eq!(pacific_offset(1_793_523_600), 8 * 3600);
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
    fn a_park_and_the_last_minutes_requests_lift_in_turn_and_a_silent_server_stays_out() {
        let mut ledger = Ledger::default();
        let t = at(1_000_000);
        ledger.record("m", None, at(999_950), ResetZone::Utc);
        ledger.record("m", None, at(999_980), ResetZone::Utc);
        ledger.park("m", t, Duration::from_secs(25));
        assert_eq!(ledger.lifts("m", t), [10, 25, 40].map(Duration::from_secs));
        ledger.set_aside("s", t, Duration::from_secs(5));
        assert_eq!(
            ledger.check("s", &Limits::default(), None, t, ResetZone::Utc),
            Err(Refused::Silent(1_000_005))
        );
        assert!(ledger.lifts("s", t).is_empty());
        assert_eq!(
            serde_json::to_value(&ledger).unwrap()["parked_until"],
            serde_json::json!({"m": 1_000_025}),
            "a server one process cannot reach is not the others' business"
        );
    }

    #[test]
    fn counts_survive_a_restart() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("quota.json");
        let t = at(86_400 * 7);
        let mut ledger = Ledger::load(&path);
        ledger.record("m", Some("p"), t, ResetZone::Utc);
        ledger.save().unwrap();
        let again = Ledger::load(&path);
        assert_eq!(again.used_today("m", t, ResetZone::Utc), 1);
        assert_eq!(again.used_today("p", t, ResetZone::Utc), 1);
    }

    #[test]
    fn two_processes_count_into_one_day_and_a_torn_file_is_no_stop() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("quota.json");
        let t = at(86_400 * 7);
        let (mut window, mut cli) = (Ledger::load(&path), Ledger::load(&path));
        window.record("m", Some("p"), t, ResetZone::Utc);
        window.save().unwrap();
        cli.record("m", Some("p"), t, ResetZone::Utc);
        cli.record("m", Some("p"), t, ResetZone::Utc);
        cli.save().unwrap();
        window.record("m", Some("p"), t, ResetZone::Utc);
        window.save().unwrap();
        assert_eq!(Ledger::load(&path).used_today("p", t, ResetZone::Utc), 4);
        assert_eq!(
            window.used_today("p", t, ResetZone::Utc),
            4,
            "it read the CLI's too"
        );
        std::fs::write(&path, "{\"days\": {").unwrap();
        assert_eq!(Ledger::load(&path).used_today("p", t, ResetZone::Utc), 0);
    }
}
