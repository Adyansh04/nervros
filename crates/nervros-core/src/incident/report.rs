//! An incident as it is kept and read: what opened it, the playbooks that hold and those ruled
//! out with what each saw, and the minutes before it as a timeline. Kept per robot in the state
//! folder, the newest few.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use serde::{Deserialize, Serialize};

use super::checks::Judged;
use super::playbook::{Kind, Loaded};
use super::recorder::Recording;
use super::trigger::Trigger;
use crate::lock;
use crate::persist::{read_or_default, write_atomic};

/// Incidents kept per robot.
const KEPT: usize = 50;
/// Timeline lines at most, the newest kept.
const TIMELINE: usize = 30;
/// How far back the timeline reaches, s.
const TIMELINE_S: f64 = 180.0;

/// A playbook's verdict on an incident.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Verdict {
    /// Its id.
    pub playbook: String,
    /// What it is.
    pub title: String,
    /// What it means.
    pub verdict: String,
    /// What to do.
    pub fix: String,
    /// The agent skill with the long form, if any.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub procedure: String,
    /// What each check saw.
    pub evidence: Vec<String>,
}

/// One incident.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Incident {
    /// `i1`, `i2`, ... per robot.
    pub id: String,
    /// Unix seconds.
    pub opened_s: f64,
    /// What kind of thing opened it.
    pub kind: Kind,
    /// What opened it, in a sentence.
    pub trigger: String,
    /// The mission that failed, or empty.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub mission: String,
    /// The playbooks whose checks hold: the likely causes.
    pub causes: Vec<Verdict>,
    /// The playbooks whose checks do not.
    pub ruled_out: Vec<Verdict>,
    /// The playbooks a check of which could not run.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub unchecked: Vec<Verdict>,
    /// The minutes before it, oldest first: `-42 s WARN controller_server: ...`.
    pub timeline: Vec<String>,
}

impl Incident {
    /// It in a line: what opened it and its likeliest cause.
    #[must_use]
    pub fn headline(&self) -> String {
        match self.causes.first() {
            Some(c) => format!("{}: {}", self.trigger, c.title),
            None => format!("{}: no known cause matched", self.trigger),
        }
    }

    /// It as text for a terminal: the causes with their evidence and fix, then the rest.
    #[must_use]
    pub fn text(&self, now_s: f64) -> String {
        let ago = crate::mission::check::how_long(now_s - self.opened_s);
        let mut out = format!("{}, {ago} ago: {}\n", self.id, self.trigger);
        if self.causes.is_empty() {
            out.push_str("No known cause matched.\n");
        }
        for c in &self.causes {
            let _ = writeln!(out, "Likely: {}. {}", c.title, c.verdict);
            for e in &c.evidence {
                let _ = writeln!(out, "  {e}");
            }
            let _ = writeln!(out, "  Fix: {}", c.fix);
        }
        for (head, list) in [
            ("Ruled out", &self.ruled_out),
            ("Could not check", &self.unchecked),
        ] {
            if !list.is_empty() {
                let _ = writeln!(out, "{head}:");
                for v in list {
                    let _ = writeln!(out, "  {}: {}", v.title, v.evidence.join("; "));
                }
            }
        }
        if !self.timeline.is_empty() {
            out.push_str("Before it:\n");
            for line in &self.timeline {
                let _ = writeln!(out, "  {line}");
            }
        }
        out
    }
}

/// The incidents kept in `path`, newest last; none when it cannot be read.
#[must_use]
pub fn read(path: &std::path::Path) -> Vec<Incident> {
    read_or_default::<File>(path).incidents
}

/// Builds the incident from the playbooks' verdicts and the recording.
pub(crate) fn build(
    id: String,
    trigger: &Trigger,
    recording: &Recording,
    judged: Vec<(&Loaded, Judged)>,
) -> Incident {
    let mut incident = Incident {
        id,
        opened_s: recording.at_s,
        kind: trigger.kind,
        trigger: trigger.text.clone(),
        mission: trigger.mission.clone(),
        causes: Vec::new(),
        ruled_out: Vec::new(),
        unchecked: Vec::new(),
        timeline: timeline(recording),
    };
    for (book, j) in judged {
        let verdict = Verdict {
            playbook: book.book.id.clone(),
            title: book.book.title.clone(),
            verdict: book.book.verdict.clone(),
            fix: book.book.fix.clone(),
            procedure: book.book.procedure.clone(),
            evidence: j.evidence,
        };
        match j.holds {
            Some(true) => incident.causes.push(verdict),
            Some(false) => incident.ruled_out.push(verdict),
            None => incident.unchecked.push(verdict),
        }
    }
    incident
}

/// The recording's lines and marks as one timeline, in seconds before it was frozen.
fn timeline(recording: &Recording) -> Vec<String> {
    let level = |l: u8| crate::ros_tools::logs::level_word(u64::from(l));
    let lines: Vec<(f64, String)> = recording
        .logs
        .iter()
        .map(|l| (l.t, format!("{} {}: {}", level(l.level), l.node, l.text)))
        .chain(recording.marks.iter().map(|m| (m.t, m.text.clone())))
        .filter(|(t, _)| recording.at_s - t <= TIMELINE_S)
        .collect();
    // One line said again and again is one line, at its latest, with how often.
    let mut folded: std::collections::HashMap<String, (f64, usize)> =
        std::collections::HashMap::new();
    for (t, text) in lines {
        let (latest, n) = folded.entry(text).or_insert((t, 0));
        *latest = latest.max(t);
        *n += 1;
    }
    let mut out: Vec<(f64, String, usize)> = folded
        .into_iter()
        .map(|(text, (t, n))| (t, text, n))
        .collect();
    out.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
    let skip = out.len().saturating_sub(TIMELINE);
    out.into_iter()
        .skip(skip)
        .map(|(t, text, n)| {
            let again = if n > 1 {
                format!(" (x{n})")
            } else {
                String::new()
            };
            // In words: a model read "-2 s detector_head stopped" as "-2 seconds ago".
            format!(
                "{:.0} s before: {text}{again}",
                (recording.at_s - t).max(0.0)
            )
        })
        .collect()
}

#[derive(Debug, Default, Serialize, Deserialize)]
struct File {
    next: u64,
    incidents: Vec<Incident>,
}

/// The robot's incidents, written through to their file.
#[derive(Debug)]
pub(crate) struct Store {
    path: Option<PathBuf>,
    file: Mutex<File>,
    /// The incidents as readers see them, swapped whole on each add: a window that draws them
    /// every frame copies nothing and never waits on the file's write.
    shown: Mutex<Arc<[Incident]>>,
    /// Held across a write, so the file always ends with the newest incidents.
    writing: Mutex<()>,
}

impl Store {
    pub(crate) fn open(path: Option<PathBuf>) -> Self {
        let file: File = path.as_deref().map(read_or_default).unwrap_or_default();
        Self {
            path,
            shown: Mutex::new(file.incidents.clone().into()),
            file: Mutex::new(file),
            writing: Mutex::default(),
        }
    }

    /// The next incident's id.
    pub(crate) fn next_id(&self) -> String {
        let mut f = lock(&self.file);
        f.next += 1;
        format!("i{}", f.next)
    }

    pub(crate) fn add(&self, incident: Incident) {
        let _writing = lock(&self.writing);
        let bytes = {
            let mut f = lock(&self.file);
            f.incidents.push(incident);
            let over = f.incidents.len().saturating_sub(KEPT);
            f.incidents.drain(..over);
            *lock(&self.shown) = f.incidents.clone().into();
            self.path
                .as_ref()
                .map(|_| serde_json::to_vec_pretty(&*f).map_err(std::io::Error::other))
        };
        if let (Some(path), Some(bytes)) = (&self.path, bytes)
            && let Err(e) = bytes.and_then(|b| write_atomic(path, &b))
        {
            tracing::warn!(error = %e, file = %path.display(), "incidents not saved");
        }
    }

    /// Newest last.
    pub(crate) fn list(&self) -> Arc<[Incident]> {
        Arc::clone(&lock(&self.shown))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::incident::playbook::Playbook;
    use crate::incident::recorder::{LogLine, Mark};

    #[test]
    fn verdicts_sort_into_causes_ruled_out_and_unchecked_with_a_timeline() {
        let book = |id: &str| {
            toml::from_str::<Playbook>(&format!(
                "id = \"{id}\"\ntitle = \"{id} title\"\nverdict = \"v\"\nfix = \"f\"\n\
                 [[check]]\nlog = \"x\"\n"
            ))
            .unwrap()
            .check("test")
            .unwrap()
        };
        let (a, b, c) = (book("a"), book("b"), book("c"));
        let judged = |holds| Judged {
            holds,
            evidence: vec!["the log: x".into()],
        };
        let line = |t: f64, text: &str| LogLine {
            t,
            level: 40,
            node: "nav".into(),
            text: text.into(),
        };
        let recording = Recording {
            at_s: 1000.0,
            since_s: 0.0,
            logs: vec![
                line(500.0, "too old"),
                line(950.0, "No path"),
                line(955.0, "No path"),
                line(990.0, "Aborted"),
            ],
            marks: vec![Mark {
                t: 940.0,
                text: "mission m1 started".into(),
            }],
        };
        let trigger = Trigger::new(Kind::Asked, "asked why");
        let incident = build(
            "i4".into(),
            &trigger,
            &recording,
            vec![
                (&a, judged(Some(true))),
                (&b, judged(Some(false))),
                (&c, judged(None)),
            ],
        );
        assert_eq!(incident.causes[0].playbook, "a");
        assert_eq!(incident.ruled_out[0].playbook, "b");
        assert_eq!(incident.unchecked[0].playbook, "c");
        assert_eq!(incident.headline(), "asked why: a title");
        assert_eq!(
            incident.timeline,
            [
                "60 s before: mission m1 started",
                "45 s before: ERROR nav: No path (x2)",
                "10 s before: ERROR nav: Aborted"
            ]
        );
    }

    #[test]
    fn the_store_numbers_keeps_and_reads_back_its_incidents() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("incidents").join("g1.json");
        let store = Store::open(Some(path.clone()));
        let incident = |id: String| Incident {
            id,
            opened_s: 1.0,
            kind: Kind::Asked,
            trigger: "t".into(),
            mission: String::new(),
            causes: Vec::new(),
            ruled_out: Vec::new(),
            unchecked: Vec::new(),
            timeline: Vec::new(),
        };
        for _ in 0..(KEPT + 2) {
            store.add(incident(store.next_id()));
        }
        let again = Store::open(Some(path));
        assert_eq!(again.list().len(), KEPT);
        assert_eq!(again.next_id(), format!("i{}", KEPT + 3));
        let ids: Vec<String> = again.list().iter().map(|i| i.id.clone()).collect();
        assert_eq!(ids.first().map(String::as_str), Some("i3"));
        assert_eq!(ids.last().map(String::as_str), Some("i52"));
    }
}
