//! The flight recorder: the minutes before an incident, kept as they pass, so an investigation
//! sees what led up to it and not only what is left. Warnings and errors from `/rosout`, the nodes
//! that came and went, the watched topics going quiet and coming back, and the session's missions.

use std::collections::{BTreeSet, VecDeque};
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use futures::StreamExt as _;
use nervros_ros::RobotPort;
use serde::{Deserialize, Serialize};
use tokio::sync::{broadcast, mpsc};
use tokio::task::JoinHandle;

use super::playbook::Kind;
use super::trigger::{self, Change, ErrorBursts, QuietTopic, Trigger};
use crate::lock;
use crate::mission::Outcome;
use crate::profile::IncidentConfig;
use crate::session::Event;

/// Lines and marks kept at most, whatever the window: a node that floods `/rosout` cannot grow it.
const MAX_LOGS: usize = 2000;
const MAX_MARKS: usize = 500;
/// A log line's text past this is cut.
const LINE_CHARS: usize = 400;
/// How often the graph is read for nodes that came or went.
const NODES_EVERY: Duration = Duration::from_secs(5);
/// How often a watched topic's rate is taken.
const RATE_EVERY: Duration = Duration::from_secs(1);
/// More nodes than this coming or going at once is one mark, not one each.
const NODES_NAMED: usize = 4;

/// A `/rosout` line at warning or above.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LogLine {
    /// Unix seconds, as it arrived.
    pub t: f64,
    /// As `rcl_interfaces/msg/Log` numbers it: 30 a warning, 40 an error, 50 fatal.
    pub level: u8,
    /// The node, as it names itself.
    pub node: String,
    /// What it said.
    pub text: String,
}

/// Something that happened: a node came or went, a topic went quiet, a mission ended.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Mark {
    /// Unix seconds.
    pub t: f64,
    /// What happened.
    pub text: String,
}

/// The recorder's window, frozen for an investigation.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Recording {
    /// When it was frozen, Unix seconds.
    pub at_s: f64,
    /// Since when it has recorded, Unix seconds: its first line, or when the recorder started.
    #[serde(default)]
    pub since_s: f64,
    /// Oldest first.
    pub logs: Vec<LogLine>,
    /// Oldest first.
    pub marks: Vec<Mark>,
}

#[derive(Debug, Default)]
struct Buffers {
    logs: VecDeque<LogLine>,
    marks: VecDeque<Mark>,
}

/// Keeps the window and opens incidents; its tasks end when it is dropped.
#[derive(Debug)]
pub struct Recorder {
    window_s: f64,
    started_s: f64,
    buffers: Mutex<Buffers>,
    opened: mpsc::UnboundedSender<Trigger>,
    tasks: Mutex<Vec<JoinHandle<()>>>,
}

impl Recorder {
    /// Starts recording the robot; what should open an incident goes to `opened`.
    #[must_use]
    pub fn start(
        robot: &Arc<dyn RobotPort>,
        config: &IncidentConfig,
        opened: mpsc::UnboundedSender<Trigger>,
    ) -> Arc<Self> {
        let me = Arc::new(Self {
            window_s: config.window_s,
            started_s: crate::now_s(),
            buffers: Mutex::default(),
            opened,
            tasks: Mutex::default(),
        });
        let weak = Arc::downgrade(&me);
        let mut tasks = vec![
            tokio::spawn(follow_logs(Arc::clone(robot), weak.clone())),
            tokio::spawn(follow_nodes(
                Arc::clone(robot),
                weak.clone(),
                config.nodes.clone(),
            )),
        ];
        for watched in &config.watch {
            tasks.push(tokio::spawn(follow_rate(
                Arc::clone(robot),
                weak.clone(),
                watched.clone(),
            )));
        }
        lock(&me.tasks).extend(tasks);
        me
    }

    /// Follows a session's events: its missions are marked, and a failed one opens an incident.
    pub fn follow(self: &Arc<Self>, events: broadcast::Receiver<Event>) {
        let task = tokio::spawn(follow_events(Arc::downgrade(self), events));
        lock(&self.tasks).push(task);
    }

    /// The window as it is now.
    #[must_use]
    pub fn snapshot(&self) -> Recording {
        let now = crate::now_s();
        let mut b = lock(&self.buffers);
        self.prune(&mut b, now);
        let first = b.logs.front().map_or(self.started_s, |l| l.t);
        Recording {
            at_s: now,
            since_s: first.min(self.started_s).max(now - self.window_s),
            logs: b.logs.iter().cloned().collect(),
            marks: b.marks.iter().cloned().collect(),
        }
    }

    /// Adds log lines as if they had come in, such as `/rosout`'s history when the recorder has
    /// only just started.
    pub fn add_logs(&self, lines: impl IntoIterator<Item = LogLine>) {
        let mut b = lock(&self.buffers);
        for line in lines {
            if !b.logs.contains(&line) {
                b.logs.push_back(line);
            }
        }
        b.logs.make_contiguous().sort_by(|a, b| a.t.total_cmp(&b.t));
        while b.logs.len() > MAX_LOGS {
            b.logs.pop_front();
        }
    }

    fn mark(&self, text: String) {
        let now = crate::now_s();
        let mut b = lock(&self.buffers);
        b.marks.push_back(Mark { t: now, text });
        while b.marks.len() > MAX_MARKS {
            b.marks.pop_front();
        }
        self.prune(&mut b, now);
    }

    fn log(&self, line: LogLine) {
        // A line's stamp is its host's clock, up to an hour off: pruning by it could empty all.
        let now = crate::now_s();
        let mut b = lock(&self.buffers);
        b.logs.push_back(line);
        while b.logs.len() > MAX_LOGS {
            b.logs.pop_front();
        }
        self.prune(&mut b, now);
    }

    pub(super) fn open(&self, trigger: Trigger) {
        let _ = self.opened.send(trigger);
    }

    fn prune(&self, b: &mut Buffers, now: f64) {
        let old = |t: f64| now - t > self.window_s;
        while b.logs.front().is_some_and(|l| old(l.t)) {
            b.logs.pop_front();
        }
        while b.marks.front().is_some_and(|m| old(m.t)) {
            b.marks.pop_front();
        }
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        for task in lock(&self.tasks).drain(..) {
            task.abort();
        }
    }
}

/// A `rcl_interfaces/msg/Log` as a line, when it is a warning or worse.
pub(crate) fn line_of(msg: &serde_json::Value, arrived: f64) -> Option<LogLine> {
    let level = u8::try_from(msg["level"].as_u64()?).ok()?;
    // Stamped by the wall clock where it was logged; one far from now is a clock not to trust.
    #[expect(
        clippy::cast_precision_loss,
        reason = "seconds since 1970 fit a double"
    )]
    let stamped = msg["stamp"]["sec"]
        .as_i64()
        .map(|s| s as f64)
        .zip(msg["stamp"]["nanosec"].as_f64())
        .map(|(s, ns)| s + ns * 1e-9)
        .filter(|t| (arrived - t).abs() < 3600.0);
    (level >= 30).then(|| LogLine {
        t: stamped.unwrap_or(arrived),
        level,
        node: msg["name"].as_str().unwrap_or("?").to_owned(),
        text: msg["msg"]
            .as_str()
            .unwrap_or_default()
            .chars()
            .take(LINE_CHARS)
            .collect(),
    })
}

async fn follow_logs(robot: Arc<dyn RobotPort>, me: Weak<Recorder>) {
    let mut stream = match robot.messages("/rosout", "rcl_interfaces/msg/Log").await {
        Ok(s) => s,
        Err(e) => {
            tracing::warn!(error = %e, "the flight recorder cannot follow /rosout");
            return;
        }
    };
    let mut bursts = ErrorBursts::default();
    while let Some(msg) = stream.next().await {
        let Ok(msg) = msg else { continue };
        let now = crate::now_s();
        let Some(line) = line_of(&msg, now) else {
            continue;
        };
        let Some(me) = me.upgrade() else { return };
        let node = line.node.clone();
        let error = line.level >= 40;
        me.log(line);
        if error && bursts.error(&node, now) {
            me.open(Trigger::new(
                Kind::ErrorBurst,
                format!("{node} logged errors in a burst"),
            ));
        }
    }
}

async fn follow_nodes(robot: Arc<dyn RobotPort>, me: Weak<Recorder>, watched: Vec<String>) {
    let mut before: Option<BTreeSet<String>> = None;
    let mut tick = tokio::time::interval(NODES_EVERY);
    loop {
        tick.tick().await;
        let Ok(graph) = robot.graph_detail().await else {
            continue;
        };
        let now: BTreeSet<String> = graph.nodes.into_iter().collect();
        let Some(me) = me.upgrade() else { return };
        if let Some(before) = &before {
            let went: Vec<&String> = before.difference(&now).collect();
            let came: Vec<&String> = now.difference(before).collect();
            for (names, verb) in [(&went, "stopped"), (&came, "started")] {
                if let Some(text) = nodes_text(names, verb) {
                    me.mark(text);
                }
            }
            for node in trigger::gone(before, &now, &watched) {
                me.open(Trigger::new(
                    Kind::NodeGone,
                    format!("{node} stopped running"),
                ));
            }
        }
        before = Some(now);
    }
}

/// "3 nodes stopped: /a, /b, /c", or nothing for none.
fn nodes_text(names: &[&String], verb: &str) -> Option<String> {
    match names {
        [] => None,
        [one] => Some(format!("{one} {verb}")),
        many => {
            let shown: Vec<&str> = many.iter().take(NODES_NAMED).map(|n| n.as_str()).collect();
            let more = many.len().saturating_sub(NODES_NAMED);
            let tail = if more > 0 {
                format!(" and {more} more")
            } else {
                String::new()
            };
            Some(format!(
                "{} nodes {verb}: {}{tail}",
                many.len(),
                shown.join(", ")
            ))
        }
    }
}

async fn follow_rate(
    robot: Arc<dyn RobotPort>,
    me: Weak<Recorder>,
    watched: crate::profile::WatchedTopic,
) {
    let mut arrivals = match robot.arrivals(&watched.topic, &watched.msg_type).await {
        Ok(s) => s,
        Err(e) => {
            tracing::warn!(topic = %watched.topic, error = %e, "the flight recorder cannot follow it");
            return;
        }
    };
    let name = if watched.name.is_empty() {
        watched.topic.clone()
    } else {
        format!("{} ({})", watched.name, watched.topic)
    };
    let mut state = QuietTopic::default();
    let mut tick = tokio::time::interval(RATE_EVERY);
    let mut count = 0_u32;
    let mut last_hz = 0.0;
    let mut since = tokio::time::Instant::now();
    loop {
        tokio::select! {
            got = arrivals.next() => match got {
                Some(_) => count += 1,
                None => return,
            },
            _ = tick.tick() => {
                let hz = f64::from(count) / since.elapsed().as_secs_f64().max(1e-3);
                count = 0;
                since = tokio::time::Instant::now();
                let Some(me) = me.upgrade() else { return };
                match state.rate(hz, watched.min_hz, crate::now_s()) {
                    Change::WentQuiet => {
                        let text = format!(
                            "{name} went quiet: {hz:.1} Hz, from {last_hz:.1} Hz, under its {} Hz",
                            watched.min_hz
                        );
                        me.mark(text.clone());
                        me.open(Trigger::new(Kind::TopicSilent, text));
                    }
                    Change::CameBack => me.mark(format!("{name} is back at {hz:.1} Hz")),
                    Change::Same => {}
                }
                if hz >= watched.min_hz {
                    last_hz = hz;
                }
            }
        }
    }
}

async fn follow_events(me: Weak<Recorder>, mut events: broadcast::Receiver<Event>) {
    loop {
        let event = match events.recv().await {
            Ok(e) => e,
            Err(broadcast::error::RecvError::Lagged(_)) => continue,
            Err(broadcast::error::RecvError::Closed) => return,
        };
        let Some(me) = me.upgrade() else { return };
        match event {
            Event::MissionStarted { id, .. } => me.mark(format!("mission {id} started")),
            Event::MissionFinished {
                id,
                outcome,
                failed_step,
                reason,
                by_job,
                ..
            } => {
                let at = if failed_step.is_empty() {
                    String::new()
                } else {
                    format!(" at {failed_step}")
                };
                let said = if reason.is_empty() {
                    String::new()
                } else {
                    format!(": {reason}")
                };
                let text = format!("mission {id} ended, {}{at}{said}", outcome.as_str());
                me.mark(text.clone());
                // A job's failed step is the job's answer, such as a look that saw nothing.
                let failed = match outcome {
                    Outcome::Failure => !by_job,
                    Outcome::Timeout | Outcome::Error => true,
                    Outcome::Success | Outcome::Canceled | Outcome::Rejected => false,
                };
                if failed {
                    me.open(Trigger {
                        kind: Kind::MissionFailed,
                        text,
                        mission: id,
                        failure: reason,
                    });
                }
            }
            Event::Halted { reason } => me.mark(format!("halted: {reason}")),
            Event::Stopped { ok, detail } => me.mark(if ok {
                format!("the robot was stopped: {detail}")
            } else {
                format!("a stop was not confirmed: {detail}")
            }),
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;
    use crate::profile::IncidentConfig;

    fn config() -> IncidentConfig {
        IncidentConfig {
            window_s: 60.0,
            ..IncidentConfig::default()
        }
    }

    #[tokio::test]
    async fn a_failed_mission_is_marked_and_opens_an_incident() {
        let robot: Arc<dyn RobotPort> = Arc::new(nervros_ros::fake::FakeRobot::new());
        let (tx, mut opened) = mpsc::unbounded_channel();
        let recorder = Recorder::start(&robot, &config(), tx);
        let (events, rx) = broadcast::channel(16);
        recorder.follow(rx);
        events
            .send(Event::MissionStarted {
                id: "m3".into(),
                hash: String::new(),
            })
            .unwrap();
        let finished = |id: &str, reason: &str, by_job| Event::MissionFinished {
            id: id.into(),
            outcome: Outcome::Failure,
            failed_step: "s2".into(),
            reason: reason.into(),
            elapsed_s: 30.0,
            outputs: serde_json::Map::new(),
            by_job,
            fault: None,
        };
        events.send(finished("m9", "nothing seen", true)).unwrap();
        events
            .send(finished("m3", "Nav2 aborted the goal", false))
            .unwrap();
        let trigger = tokio::time::timeout(Duration::from_secs(2), opened.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(trigger.kind, Kind::MissionFailed);
        assert_eq!(
            trigger.mission, "m3",
            "a job's look that saw nothing opens none"
        );
        assert_eq!(trigger.failure, "Nav2 aborted the goal");
        let marks: Vec<String> = recorder
            .snapshot()
            .marks
            .into_iter()
            .map(|m| m.text)
            .collect();
        assert_eq!(
            marks,
            [
                "mission m3 started",
                "mission m9 ended, failure at s2: nothing seen",
                "mission m3 ended, failure at s2: Nav2 aborted the goal"
            ]
        );
    }

    #[test]
    fn warnings_and_worse_become_lines_and_old_ones_leave_the_window() {
        let warn = json!({"level": 30, "name": "controller_server", "msg": "No path"});
        assert_eq!(
            line_of(&warn, 5.0),
            Some(LogLine {
                t: 5.0,
                level: 30,
                node: "controller_server".into(),
                text: "No path".into()
            })
        );
        assert_eq!(line_of(&json!({"level": 20, "msg": "fine"}), 5.0), None);
        let stamped = json!({"level": 40, "name": "n", "msg": "m",
                             "stamp": {"sec": 1000, "nanosec": 500_000_000}});
        assert!((line_of(&stamped, 1002.0).unwrap().t - 1000.5).abs() < 1e-9);
        assert!((line_of(&stamped, 99_999.0).unwrap().t - 99_999.0).abs() < 1e-9);
        let (tx, _rx) = mpsc::unbounded_channel();
        let recorder = Recorder {
            window_s: 60.0,
            started_s: crate::now_s(),
            buffers: Mutex::default(),
            opened: tx,
            tasks: Mutex::default(),
        };
        let now = crate::now_s();
        recorder.add_logs([
            line_of(&warn, now - 10.0).unwrap(),
            line_of(&warn, now - 120.0).unwrap(),
        ]);
        recorder.add_logs([line_of(&warn, now - 10.0).unwrap()]);
        assert_eq!(
            recorder.snapshot().logs.len(),
            1,
            "one in the window, kept once"
        );
        recorder.log(LogLine {
            t: now + 600.0,
            ..line_of(&warn, now).unwrap()
        });
        assert_eq!(
            recorder.snapshot().logs.len(),
            2,
            "a line from a clock ten minutes ahead empties nothing"
        );
    }

    #[test]
    fn many_nodes_at_once_are_one_mark() {
        let names: Vec<String> = (0..6).map(|i| format!("/n{i}")).collect();
        let refs: Vec<&String> = names.iter().collect();
        assert_eq!(
            nodes_text(&refs, "stopped").unwrap(),
            "6 nodes stopped: /n0, /n1, /n2, /n3 and 2 more"
        );
        assert_eq!(nodes_text(&refs[..1], "started").unwrap(), "/n0 started");
        assert_eq!(nodes_text(&[], "started"), None);
    }
}
