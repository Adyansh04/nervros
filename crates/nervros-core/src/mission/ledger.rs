//! The mission ledger: every mission the agent ran, step by step, kept per robot in SQLite.
//!
//! The plan card's track records, `recall`, replay, plan templates and the skill-gap log read and
//! write it. Its calls block for a few milliseconds; async callers run them on the blocking pool.

use std::path::Path;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use rusqlite::{Connection, OptionalExtension as _, Row, params};
use serde::Serialize;
use serde_json::Value;

use super::plan::StepArg;

/// How many of a step's latest runs its track record counts.
const TRACK_RUNS: u32 = 20;

const SCHEMA: &str = "
    PRAGMA journal_mode = WAL;
    PRAGMA synchronous = NORMAL;
    PRAGMA foreign_keys = ON;
    CREATE TABLE IF NOT EXISTS missions (
        id TEXT PRIMARY KEY,
        hash TEXT NOT NULL,
        intent TEXT NOT NULL,
        request TEXT NOT NULL,
        started REAL NOT NULL,
        ended REAL NOT NULL,
        outcome TEXT NOT NULL,
        failed_step TEXT NOT NULL,
        reason TEXT NOT NULL
    );
    CREATE INDEX IF NOT EXISTS missions_by_time ON missions (started);
    CREATE TABLE IF NOT EXISTS steps (
        mission TEXT NOT NULL REFERENCES missions (id) ON DELETE CASCADE,
        id TEXT NOT NULL,
        skill TEXT NOT NULL,
        target TEXT NOT NULL,
        args TEXT NOT NULL,
        seconds REAL,
        outcome TEXT NOT NULL,
        reason TEXT NOT NULL,
        PRIMARY KEY (mission, id)
    );
    CREATE INDEX IF NOT EXISTS steps_by_skill ON steps (skill, target);
    CREATE TABLE IF NOT EXISTS templates (
        name TEXT PRIMARY KEY,
        intent TEXT NOT NULL,
        steps TEXT NOT NULL,
        saved REAL NOT NULL,
        runs INTEGER NOT NULL DEFAULT 0
    );
    CREATE TABLE IF NOT EXISTS gaps (
        at REAL NOT NULL,
        request TEXT NOT NULL,
        reason TEXT NOT NULL,
        nearest TEXT NOT NULL
    );
";

/// One mission as it ended.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct MissionRecord {
    /// The mission id.
    pub id: String,
    /// The plan's hash.
    pub hash: String,
    /// What the plan was for.
    pub intent: String,
    /// What the operator said that led to it.
    pub request: String,
    /// Unix seconds.
    pub started: f64,
    /// Unix seconds.
    pub ended: f64,
    /// `success`, `failure`, `canceled`, `timeout`, `rejected` or `error`.
    pub outcome: String,
    /// The step it failed at, if it did.
    pub failed_step: String,
    /// Why, in the executor's words.
    pub reason: String,
    /// In order.
    pub steps: Vec<StepRecord>,
}

/// One step of a mission as it ended.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct StepRecord {
    /// `s1`, `s2`, ...
    pub id: String,
    /// The skill as planned.
    pub skill: String,
    /// What it acted on or went to, from its arguments: the key of its track record.
    pub target: String,
    /// Its arguments, as a JSON object.
    pub args: Value,
    /// How long it ran, when it ran.
    pub seconds: Option<f64>,
    /// `success`, `failure`, or `skipped` when the mission never got to it.
    pub outcome: String,
    /// Why it failed.
    pub reason: String,
}

/// How a step has gone before, from its latest runs.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Track {
    /// Runs counted.
    pub runs: u32,
    /// Of them, successes.
    pub succeeded: u32,
    /// The median time of the successes.
    pub typical_s: Option<f64>,
    /// Why it failed most recently, if it did.
    pub last_failure: Option<String>,
}

/// A plan the operator saved by name, to run again.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Template {
    /// Its name, such as "evening check".
    pub name: String,
    /// What it is for.
    pub intent: String,
    /// The plan's steps, as `run_mission` takes them.
    pub steps: Value,
    /// How often it has been used.
    pub runs: u32,
}

/// A request the robot's skills could not do.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Gap {
    /// Unix seconds.
    pub at: f64,
    /// What was asked.
    pub request: String,
    /// What is missing.
    pub reason: String,
    /// The skills that came closest.
    pub nearest: String,
}

/// The ledger of one robot.
pub struct Ledger {
    db: Mutex<Connection>,
}

impl std::fmt::Debug for Ledger {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Ledger").finish_non_exhaustive()
    }
}

/// Seconds since the Unix epoch, as the ledger stores times.
#[must_use]
pub fn now_s() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// What a step acts on or goes to, for its track record: `PickObject` on `mug_4` is one record,
/// whichever mission it was part of.
#[must_use]
pub fn target_of(args: &[StepArg]) -> String {
    ["place", "object_id", "container_id", "target", "direction"]
        .iter()
        .find_map(|key| args.iter().find(|a| a.name == *key))
        .map(|a| a.value.clone())
        .unwrap_or_default()
}

impl Ledger {
    /// Opens the ledger at `path`, creating it and its folder when missing.
    ///
    /// # Errors
    ///
    /// The file cannot be opened or its tables made.
    pub fn open(path: &Path) -> rusqlite::Result<Arc<Self>> {
        if let Some(dir) = path.parent() {
            // A failure here shows as the open's own error.
            let _ = std::fs::create_dir_all(dir);
        }
        Self::with(Connection::open(path)?)
    }

    /// A ledger kept in memory and gone when dropped: tests, and a robot with no state folder.
    ///
    /// # Errors
    ///
    /// SQLite could not start.
    pub fn in_memory() -> rusqlite::Result<Arc<Self>> {
        Self::with(Connection::open_in_memory()?)
    }

    fn with(db: Connection) -> rusqlite::Result<Arc<Self>> {
        db.execute_batch(SCHEMA)?;
        Ok(Arc::new(Self { db: Mutex::new(db) }))
    }

    fn db(&self) -> MutexGuard<'_, Connection> {
        self.db.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Records a mission and its steps, replacing any record with its id.
    ///
    /// # Errors
    ///
    /// The write failed.
    pub fn record(&self, m: &MissionRecord) -> rusqlite::Result<()> {
        let mut db = self.db();
        let tx = db.transaction()?;
        tx.execute(
            "INSERT OR REPLACE INTO missions VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9)",
            params![
                m.id,
                m.hash,
                m.intent,
                m.request,
                m.started,
                m.ended,
                m.outcome,
                m.failed_step,
                m.reason
            ],
        )?;
        {
            let mut step = tx.prepare_cached(
                "INSERT OR REPLACE INTO steps VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8)",
            )?;
            for s in &m.steps {
                step.execute(params![
                    m.id,
                    s.id,
                    s.skill,
                    s.target,
                    s.args.to_string(),
                    s.seconds,
                    s.outcome,
                    s.reason
                ])?;
            }
        }
        tx.commit()
    }

    /// How `skill` on `target` has gone in its latest runs; none before its first.
    ///
    /// # Errors
    ///
    /// The read failed.
    pub fn track(&self, skill: &str, target: &str) -> rusqlite::Result<Option<Track>> {
        let db = self.db();
        let mut runs = db.prepare_cached(
            "SELECT steps.outcome, steps.seconds, steps.reason FROM steps
             JOIN missions ON missions.id = steps.mission
             WHERE steps.skill = ?1 AND steps.target = ?2
               AND steps.outcome IN ('success', 'failure')
             ORDER BY missions.started DESC LIMIT ?3",
        )?;
        let mut track = Track {
            runs: 0,
            succeeded: 0,
            typical_s: None,
            last_failure: None,
        };
        let mut times = Vec::new();
        let rows = runs.query_map(params![skill, target, TRACK_RUNS], |r| {
            Ok((
                r.get::<_, String>(0)?,
                r.get::<_, Option<f64>>(1)?,
                r.get::<_, String>(2)?,
            ))
        })?;
        for row in rows {
            let (outcome, seconds, reason) = row?;
            track.runs += 1;
            if outcome == "success" {
                track.succeeded += 1;
                times.extend(seconds);
            } else if track.last_failure.is_none() {
                track.last_failure = Some(reason);
            }
        }
        if track.runs == 0 {
            return Ok(None);
        }
        times.sort_by(f64::total_cmp);
        track.typical_s = times.get(times.len() / 2).copied();
        Ok(Some(track))
    }

    /// The latest missions, newest first, with their steps.
    ///
    /// # Errors
    ///
    /// The read failed.
    pub fn recent(&self, limit: usize) -> rusqlite::Result<Vec<MissionRecord>> {
        let db = self.db();
        let mut missions =
            db.prepare_cached("SELECT * FROM missions ORDER BY started DESC LIMIT ?1")?;
        let mut out: Vec<MissionRecord> = missions
            .query_map([i64::try_from(limit).unwrap_or(i64::MAX)], mission_of)?
            .collect::<rusqlite::Result<_>>()?;
        for m in &mut out {
            m.steps = steps_of(&db, &m.id)?;
        }
        Ok(out)
    }

    /// The mission whose id starts with `id`.
    ///
    /// # Errors
    ///
    /// The read failed.
    pub fn mission(&self, id: &str) -> rusqlite::Result<Option<MissionRecord>> {
        let db = self.db();
        let found = db
            .query_row(
                "SELECT * FROM missions WHERE id LIKE ?1 || '%' ORDER BY started DESC LIMIT 1",
                [id],
                mission_of,
            )
            .optional()?;
        let Some(mut found) = found else {
            return Ok(None);
        };
        found.steps = steps_of(&db, &found.id)?;
        Ok(Some(found))
    }

    /// Saves a plan under `name`, replacing one of that name.
    ///
    /// # Errors
    ///
    /// The write failed.
    pub fn save_template(&self, name: &str, intent: &str, steps: &Value) -> rusqlite::Result<()> {
        self.db().execute(
            "INSERT OR REPLACE INTO templates VALUES (?1, ?2, ?3, ?4, 0)",
            params![name, intent, steps.to_string(), now_s()],
        )?;
        Ok(())
    }

    /// The saved plans, most used first.
    ///
    /// # Errors
    ///
    /// The read failed.
    pub fn templates(&self) -> rusqlite::Result<Vec<Template>> {
        let db = self.db();
        let mut all = db.prepare_cached(
            "SELECT name, intent, steps, runs FROM templates ORDER BY runs DESC, name",
        )?;
        all.query_map([], template_of)?.collect()
    }

    /// The saved plan called `name`, counted as used.
    ///
    /// # Errors
    ///
    /// The read or the count failed.
    pub fn use_template(&self, name: &str) -> rusqlite::Result<Option<Template>> {
        let db = self.db();
        let found = db
            .query_row(
                "SELECT name, intent, steps, runs FROM templates WHERE name = ?1",
                [name],
                template_of,
            )
            .optional()?;
        if found.is_some() {
            db.execute(
                "UPDATE templates SET runs = runs + 1 WHERE name = ?1",
                [name],
            )?;
        }
        Ok(found)
    }

    /// Forgets the saved plan called `name`; whether there was one.
    ///
    /// # Errors
    ///
    /// The write failed.
    pub fn forget_template(&self, name: &str) -> rusqlite::Result<bool> {
        Ok(self
            .db()
            .execute("DELETE FROM templates WHERE name = ?1", [name])?
            > 0)
    }

    /// Notes a request the skills could not do.
    ///
    /// # Errors
    ///
    /// The write failed.
    pub fn note_gap(&self, request: &str, reason: &str, nearest: &str) -> rusqlite::Result<()> {
        self.db().execute(
            "INSERT INTO gaps VALUES (?1, ?2, ?3, ?4)",
            params![now_s(), request, reason, nearest],
        )?;
        Ok(())
    }

    /// The latest gaps, newest first.
    ///
    /// # Errors
    ///
    /// The read failed.
    pub fn gaps(&self, limit: usize) -> rusqlite::Result<Vec<Gap>> {
        let db = self.db();
        let mut latest = db.prepare_cached("SELECT * FROM gaps ORDER BY at DESC LIMIT ?1")?;
        latest
            .query_map([i64::try_from(limit).unwrap_or(i64::MAX)], |r| {
                Ok(Gap {
                    at: r.get(0)?,
                    request: r.get(1)?,
                    reason: r.get(2)?,
                    nearest: r.get(3)?,
                })
            })?
            .collect()
    }
}

fn steps_of(db: &Connection, mission: &str) -> rusqlite::Result<Vec<StepRecord>> {
    let mut steps = db.prepare_cached(
        "SELECT id, skill, target, args, seconds, outcome, reason FROM steps
         WHERE mission = ?1 ORDER BY CAST(SUBSTR(id, 2) AS INTEGER)",
    )?;
    steps.query_map([mission], step_of)?.collect()
}

fn mission_of(r: &Row<'_>) -> rusqlite::Result<MissionRecord> {
    Ok(MissionRecord {
        id: r.get(0)?,
        hash: r.get(1)?,
        intent: r.get(2)?,
        request: r.get(3)?,
        started: r.get(4)?,
        ended: r.get(5)?,
        outcome: r.get(6)?,
        failed_step: r.get(7)?,
        reason: r.get(8)?,
        steps: Vec::new(),
    })
}

fn step_of(r: &Row<'_>) -> rusqlite::Result<StepRecord> {
    let args: String = r.get(3)?;
    Ok(StepRecord {
        id: r.get(0)?,
        skill: r.get(1)?,
        target: r.get(2)?,
        args: serde_json::from_str(&args).unwrap_or(Value::Null),
        seconds: r.get(4)?,
        outcome: r.get(5)?,
        reason: r.get(6)?,
    })
}

fn template_of(r: &Row<'_>) -> rusqlite::Result<Template> {
    let steps: String = r.get(2)?;
    Ok(Template {
        name: r.get(0)?,
        intent: r.get(1)?,
        steps: serde_json::from_str(&steps).unwrap_or(Value::Null),
        runs: r.get(3)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn mission(id: &str, started: f64, pick: (&str, f64, &str)) -> MissionRecord {
        let (outcome, seconds, reason) = pick;
        MissionRecord {
            id: id.to_owned(),
            hash: "h".to_owned(),
            intent: "bring the mug".to_owned(),
            request: "bring me the mug".to_owned(),
            started,
            ended: started + 60.0,
            outcome: outcome.to_owned(),
            failed_step: if outcome == "success" {
                String::new()
            } else {
                "s2".to_owned()
            },
            reason: reason.to_owned(),
            steps: vec![
                StepRecord {
                    id: "s1".to_owned(),
                    skill: "GoToPlace".to_owned(),
                    target: "dining_table_side".to_owned(),
                    args: json!({"place": "dining_table_side"}),
                    seconds: Some(20.0),
                    outcome: "success".to_owned(),
                    reason: String::new(),
                },
                StepRecord {
                    id: "s2".to_owned(),
                    skill: "PickObject".to_owned(),
                    target: "mug_4".to_owned(),
                    args: json!({"object_id": "mug_4"}),
                    seconds: Some(seconds),
                    outcome: outcome.to_owned(),
                    reason: reason.to_owned(),
                },
            ],
        }
    }

    #[test]
    fn a_track_record_counts_the_latest_runs_and_names_the_last_failure() {
        let ledger = Ledger::in_memory().unwrap();
        assert_eq!(ledger.track("PickObject", "mug_4").unwrap(), None);
        ledger
            .record(&mission("m1", 100.0, ("success", 40.0, "")))
            .unwrap();
        ledger
            .record(&mission(
                "m2",
                200.0,
                ("failure", 25.0, "nothing called mug_4 on /objects"),
            ))
            .unwrap();
        ledger
            .record(&mission("m3", 300.0, ("success", 50.0, "")))
            .unwrap();

        let track = ledger.track("PickObject", "mug_4").unwrap().unwrap();

        assert_eq!((track.runs, track.succeeded), (3, 2));
        assert_eq!(track.typical_s, Some(50.0), "the upper median of 40 and 50");
        assert_eq!(
            track.last_failure.as_deref(),
            Some("nothing called mug_4 on /objects")
        );
        assert_eq!(
            ledger
                .track("GoToPlace", "dining_table_side")
                .unwrap()
                .unwrap()
                .runs,
            3
        );
    }

    #[test]
    fn missions_read_back_newest_first_with_their_steps_in_order() {
        let ledger = Ledger::in_memory().unwrap();
        ledger
            .record(&mission("0190a", 100.0, ("success", 40.0, "")))
            .unwrap();
        ledger
            .record(&mission("0190b", 200.0, ("failure", 25.0, "slipped")))
            .unwrap();

        let recent = ledger.recent(10).unwrap();
        assert_eq!(
            recent.iter().map(|m| m.id.as_str()).collect::<Vec<_>>(),
            ["0190b", "0190a"]
        );
        assert_eq!(recent[0].steps.len(), 2);
        assert_eq!(recent[0].steps[1].args, json!({"object_id": "mug_4"}));
        let started = ledger.mission("0190a").unwrap().unwrap().started;
        assert!((started - 100.0).abs() < 1e-9);
        assert_eq!(ledger.mission("zzz").unwrap(), None);
    }

    #[test]
    fn templates_save_count_their_uses_and_can_be_forgotten() {
        let ledger = Ledger::in_memory().unwrap();
        let steps =
            json!([{"skill": "GoToPlace", "args": [{"name": "place", "value": "kitchen"}]}]);
        ledger
            .save_template("evening check", "walk the rooms", &steps)
            .unwrap();

        let used = ledger.use_template("evening check").unwrap().unwrap();
        assert_eq!(used.steps, steps);
        assert_eq!(ledger.templates().unwrap()[0].runs, 1);
        assert!(ledger.forget_template("evening check").unwrap());
        assert!(!ledger.forget_template("evening check").unwrap());
    }

    #[test]
    fn gaps_are_kept_newest_first() {
        let ledger = Ledger::in_memory().unwrap();
        ledger
            .note_gap(
                "put the mug on the sofa",
                "PlaceInto takes only tray_1",
                "PlaceInto",
            )
            .unwrap();
        assert_eq!(ledger.gaps(5).unwrap()[0].nearest, "PlaceInto");
    }

    #[test]
    fn the_target_is_the_first_argument_that_names_a_place_or_thing() {
        let arg = |name: &str, value: &str| StepArg {
            name: name.to_owned(),
            value: value.to_owned(),
        };
        assert_eq!(
            target_of(&[arg("arm", "left"), arg("object_id", "mug_4")]),
            "mug_4"
        );
        assert_eq!(target_of(&[arg("degrees", "90")]), "");
    }
}
