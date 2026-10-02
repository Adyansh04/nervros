//! The `watch`, `watches` and `plot` tools: a condition on a topic kept in view in the
//! background, as someone debugging a robot leaves a monitor running. When it holds, the watch
//! reports into the session, and the agent tells the operator: the camera's rate falling, a value
//! crossing a line, a log line matching. A plot draws a number over time in the viewer instead.

use std::borrow::Cow;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use nervros_ros::{RobotPort, RosError};
use serde_json::{Value, json};
use tokio::task::JoinHandle;
use tracing::Instrument as _;

use crate::session::{Command, Event, SessionHandle};
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// How often a value or a message is looked at.
const PERIOD: Duration = Duration::from_millis(500);
/// The shortest window a rate is measured over; a slow rate gets the two periods it needs.
const RATE_WINDOW: Duration = Duration::from_secs(2);
const MAX_RATE_WINDOW: Duration = Duration::from_secs(30);
const DEFAULT_FOR_S: u64 = 1800;
const MAX_FOR_S: u64 = 7200;
const MAX_WATCHES: usize = 8;
const DEFAULT_PLOT_S: u64 = 120;
const MAX_PLOT_S: u64 = 1800;

/// What a watch waits for.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Condition {
    /// The topic's rate under this many hertz, silence included.
    RateBelow(f64),
    /// A field of the newest message compared with a value.
    Value {
        field: String,
        op: String,
        value: Value,
    },
    /// The newest message's text holding this, case aside.
    Text(String),
}

impl Condition {
    /// Whether a look reads whole messages, which a camera or a map makes too large, rather
    /// than only counting them.
    pub(crate) fn reads_messages(&self) -> bool {
        !matches!(self, Self::RateBelow(_))
    }

    pub(crate) fn parse(args: &Value) -> Result<Self, String> {
        match args["condition"].as_str().unwrap_or_default() {
            "rate_below" => args["hz"]
                .as_f64()
                .filter(|hz| *hz > 0.0)
                .map(Self::RateBelow)
                .ok_or_else(|| "rate_below needs `hz`, above 0".to_owned()),
            "value" => {
                let field = args["field"]
                    .as_str()
                    .filter(|f| !f.trim().is_empty())
                    .ok_or("value needs `field`, such as pose.pose.position.x")?;
                let op = args["op"].as_str().unwrap_or("==");
                if !["<", "<=", ">", ">=", "==", "!="].contains(&op) {
                    return Err(format!("`op` is one of < <= > >= == !=, not `{op}`"));
                }
                if args["value"].is_null() {
                    return Err("value needs `value` to compare with".to_owned());
                }
                if !matches!(op, "==" | "!=") && number(&args["value"]).is_none() {
                    return Err(format!("`{op}` compares numbers: give `value` as a number"));
                }
                Ok(Self::Value {
                    field: field.trim().to_owned(),
                    op: op.to_owned(),
                    value: args["value"].clone(),
                })
            }
            "text" => args["contains"]
                .as_str()
                .filter(|t| !t.is_empty())
                .map(|t| Self::Text(t.to_lowercase()))
                .ok_or_else(|| "text needs `contains`".to_owned()),
            other => Err(format!(
                "`condition` is rate_below, value or text, not `{other}`"
            )),
        }
    }

    pub(crate) fn describe(&self, topic: &str) -> String {
        match self {
            Self::RateBelow(hz) => format!("{topic} under {hz} Hz"),
            Self::Value { field, op, value } => format!("{topic} {field} {op} {value}"),
            Self::Text(t) => format!("{topic} saying \"{t}\""),
        }
    }
}

/// A field by dotted path (`pose.pose.position.x`, `status.0.level`).
#[must_use]
pub fn field<'a>(msg: &'a Value, path: &str) -> Option<&'a Value> {
    path.split('.')
        .try_fold(msg, |v, part| match part.parse::<usize>() {
            Ok(i) if v.is_array() => v.get(i),
            _ => v.get(part),
        })
}

/// What a value is, in a few words: a list is never quoted whole.
fn kind(v: &Value, path: &str) -> String {
    match v {
        Value::Array(a) => format!("a list of {} values; plot one, such as {path}.0", a.len()),
        Value::Object(o) => {
            let fields: Vec<&str> = o.keys().map(String::as_str).take(8).collect();
            format!(
                "a message with {}; plot one of its fields",
                fields.join(", ")
            )
        }
        other => crate::tools::clip(&other.to_string(), 80),
    }
}

/// A number, or text that reads as one: a model often quotes the value it compares with.
fn number(v: &Value) -> Option<f64> {
    v.as_f64().or_else(|| v.as_str()?.trim().parse().ok())
}

/// Whether `found op wanted` holds: numbers as numbers, anything else by equality.
fn compare(found: &Value, op: &str, wanted: &Value) -> bool {
    if let (Some(a), Some(b)) = (found.as_f64(), number(wanted)) {
        // A float32 field arrives as the nearest double, 0.2 as 0.20000000298: equal within a
        // millionth of the larger.
        let same = (a - b).abs() <= 1e-6 * a.abs().max(b.abs()).max(1.0);
        return match op {
            "<" => a < b && !same,
            "<=" => a < b || same,
            ">" => a > b && !same,
            ">=" => a > b || same,
            "!=" => !same,
            _ => same,
        };
    }
    let (a, b) = (
        found
            .as_str()
            .map_or_else(|| found.to_string(), str::to_owned),
        wanted
            .as_str()
            .map_or_else(|| wanted.to_string(), str::to_owned),
    );
    match op {
        "!=" => a != b,
        "==" => a == b,
        _ => false,
    }
}

/// One look: `Some(what was seen)` when the condition holds.
///
/// # Errors
///
/// The topic cannot be read at all, such as a type this build does not know: a look that could
/// not subscribe says so, rather than reading as silence.
pub(crate) async fn holds(
    robot: &dyn RobotPort,
    topic: &str,
    ty: &str,
    condition: &Condition,
) -> Result<Option<String>, String> {
    match condition {
        Condition::RateBelow(hz) => {
            let window = RATE_WINDOW.max(Duration::from_secs_f64(2.0 / hz).min(MAX_RATE_WINDOW));
            let arrivals = robot
                .sample_sizes(topic, ty, window, 100_000)
                .await
                .map_err(|e| e.to_string())?;
            #[expect(clippy::cast_precision_loss, reason = "a count of messages")]
            let rate = arrivals.len() as f64 / window.as_secs_f64();
            Ok((rate < *hz).then(|| format!("{rate:.1} Hz")))
        }
        Condition::Value {
            field: path,
            op,
            value,
        } => {
            let msg = match robot.latest(topic, ty, PERIOD).await {
                Ok(msg) => msg,
                Err(RosError::NoData(_)) => return Ok(None),
                Err(e) => return Err(e.to_string()),
            };
            Ok(field(&msg, path)
                .filter(|found| compare(found, op, value))
                .map(|found| format!("{path} is {found}")))
        }
        Condition::Text(text) => {
            // Every message in the window, not the newest: a log line is gone by the next look.
            let msgs = robot
                .sample_messages(topic, ty, 1000, RATE_WINDOW)
                .await
                .map_err(|e| e.to_string())?;
            Ok(msgs
                .iter()
                .map(Value::to_string)
                .find(|body| body.to_lowercase().contains(text))
                .map(|body| {
                    let excerpt: String = body.chars().take(200).collect();
                    format!("it said {excerpt}")
                }))
        }
    }
}

/// The topic the arguments name, and its type from the graph: one the profile lets the agent
/// read, and small enough to read message by message when `whole` messages are read.
pub(crate) async fn topic_of(
    robot: &dyn RobotPort,
    args: &Value,
    read_deny: &[String],
    whole: bool,
) -> Result<(String, String), String> {
    let topic = crate::guard::canonical_ros_name(args["topic"].as_str().unwrap_or_default())
        .map_err(|e| format!("`topic`: {e}, such as /scan"))?;
    if read_deny
        .iter()
        .any(|p| crate::guard::glob_match(p, &topic))
    {
        return Err(format!("the profile keeps {topic} from being read"));
    }
    let graph = robot.graph().await.map_err(|e| e.to_string())?;
    let ty = graph
        .topics
        .iter()
        .find(|(name, _)| *name == topic)
        .map(|(_, types)| types.first().cloned().unwrap_or_default())
        // The local model took a one-off reading instead of setting the watch again.
        .ok_or_else(|| {
            let names = graph.topics.iter().map(|(n, _)| n.as_str());
            format!(
                "{}; call this tool again with the right one",
                crate::ros_tools::missing("topic", &topic, names)
            )
        })?;
    if whole && crate::ros_tools::BULK_TYPES.contains(&ty.as_str()) {
        return Err(format!(
            "{topic} carries {ty}, too large to read message by message; watch its rate, or look"
        ));
    }
    Ok((topic, ty))
}

struct Entry {
    id: String,
    what: String,
    started: Instant,
    task: JoinHandle<()>,
}

/// The running watches, and the session they report into.
pub struct Watches {
    robot: Arc<dyn RobotPort>,
    /// Topics the profile keeps from being read.
    read_deny: Vec<String>,
    session: OnceLock<SessionHandle>,
    running: Mutex<Vec<Entry>>,
    next: AtomicU64,
}

impl Watches {
    /// No watches yet; [`Self::attach`] connects them to the session. `read_deny` is the
    /// profile's, as `topic_sample` keeps it.
    #[must_use]
    pub fn new(robot: Arc<dyn RobotPort>, read_deny: Vec<String>) -> Arc<Self> {
        Arc::new(Self {
            robot,
            read_deny,
            session: OnceLock::new(),
            running: Mutex::new(Vec::new()),
            next: AtomicU64::new(1),
        })
    }

    /// Where the watches report.
    pub fn attach(&self, session: SessionHandle) {
        let _ = self.session.set(session);
    }

    fn lock(&self) -> MutexGuard<'_, Vec<Entry>> {
        crate::lock(&self.running)
    }

    /// The `watch` and `watches` tools.
    #[must_use]
    pub fn tools(self: &Arc<Self>) -> Vec<Arc<dyn Tool>> {
        vec![
            Arc::new(WatchTool {
                spec: watch_spec(),
                watches: Arc::clone(self),
            }),
            Arc::new(WatchesTool {
                spec: watches_spec(),
                watches: Arc::clone(self),
            }),
            Arc::new(PlotTool {
                spec: plot_spec(),
                watches: Arc::clone(self),
            }),
        ]
    }

    async fn topic(&self, args: &Value, whole: bool) -> Result<(String, String), String> {
        topic_of(self.robot.as_ref(), args, &self.read_deny, whole).await
    }

    async fn start(self: &Arc<Self>, args: &Value) -> Result<ToolOutcome, String> {
        let condition = Condition::parse(args)?;
        let (topic, ty) = self.topic(args, condition.reads_messages()).await?;
        let for_s = args["for_s"]
            .as_u64()
            .unwrap_or(DEFAULT_FOR_S)
            .clamp(1, MAX_FOR_S);
        let repeat = args["repeat"].as_bool().unwrap_or(false);
        let say = args["say"].as_str().unwrap_or_default().trim().to_owned();
        let mut running = self.lock();
        running.retain(|e| !e.task.is_finished());
        if running.len() >= MAX_WATCHES {
            return Err(format!(
                "{MAX_WATCHES} watches already run; cancel one with watches first"
            ));
        }
        let id = format!("w{}", self.next.fetch_add(1, Ordering::Relaxed));
        let what = condition.describe(&topic);
        let (robot, session) = (Arc::clone(&self.robot), self.session.get().cloned());
        let (task_id, task_what) = (id.clone(), what.clone());
        let span = crate::telemetry::job("watch", &id, &what);
        let task = tokio::spawn(
            async move {
                let deadline = Instant::now() + Duration::from_secs(for_s);
                let mut was = false;
                while Instant::now() < deadline {
                    let looked = Instant::now();
                    let seen = match holds(robot.as_ref(), &topic, &ty, &condition).await {
                        Ok(seen) => seen,
                        // Watching what cannot be read would report silence for hours.
                        Err(e) => {
                            if let Some(s) = &session {
                                s.send(Command::Report(format!(
                                    "Watch {task_id} ({task_what}) stopped: it cannot read {topic} ({e})."
                                )));
                            }
                            return;
                        }
                    };
                    if let Some(seen) = &seen
                        && !was
                    {
                        let note = if say.is_empty() {
                            String::new()
                        } else {
                            format!(" {say}")
                        };
                        let report = format!("Watch {task_id} ({task_what}) fired: {seen}.{note}");
                        if let Some(s) = &session {
                            s.send(Command::Report(report));
                        }
                        if !repeat {
                            return;
                        }
                    }
                    was = seen.is_some();
                    // Rate and text looks span a window already; a value look, or a failed one, waits.
                    tokio::time::sleep(PERIOD.saturating_sub(looked.elapsed())).await;
                }
            }
            .instrument(span),
        );
        running.push(Entry {
            id: id.clone(),
            what: what.clone(),
            started: Instant::now(),
            task,
        });
        let mut out =
            ToolOutcome::ok(json!({"watch": id, "what": what, "for_s": for_s, "repeat": repeat}));
        out.message = format!("watching {what}; a report comes when it holds");
        Ok(out)
    }

    /// Checks the field is a number in the topic's newest message, then asks the viewer to plot
    /// it.
    async fn plot(&self, args: &Value) -> Result<ToolOutcome, String> {
        let path = args["field"]
            .as_str()
            .map(str::trim)
            .filter(|f| !f.is_empty())
            .ok_or("`field` is a dotted path to a number, such as twist.twist.linear.x")?
            .to_owned();
        let (topic, ty) = self.topic(args, true).await?;
        let msg = self
            .robot
            .latest(&topic, &ty, Duration::from_secs(2))
            .await
            .map_err(|e| format!("nothing arrived on {topic} to plot ({e})"))?;
        let now = match field(&msg, &path) {
            Some(v) => v.as_f64().ok_or_else(|| {
                format!("`{path}` in {topic} is {}, not a number", kind(v, &path))
            })?,
            None => {
                return Err(format!(
                    "{topic} has no `{path}`; topic_sample shows its fields"
                ));
            }
        };
        let for_s = args["for_s"]
            .as_u64()
            .unwrap_or(DEFAULT_PLOT_S)
            .clamp(1, MAX_PLOT_S);
        let name = format!("{topic} {path}");
        if let Some(s) = self.session.get() {
            s.emit(Event::Plot {
                name: name.clone(),
                topic,
                msg_type: ty,
                field: path,
                for_s,
            });
        }
        let mut out = ToolOutcome::ok(json!({"plot": name, "now": now, "for_s": for_s}));
        out.message = format!("plotting {name} in the viewer's Plots tab for {for_s} s");
        Ok(out)
    }

    fn list(&self) -> Value {
        let mut running = self.lock();
        running.retain(|e| !e.task.is_finished());
        Value::Array(
            running
                .iter()
                .map(|e| json!({"watch": e.id, "what": e.what, "running_s": e.started.elapsed().as_secs()}))
                .collect(),
        )
    }

    fn cancel(&self, id: &str) -> bool {
        let mut running = self.lock();
        let before = running.len();
        running.retain(|e| {
            if e.id == id || id == "all" {
                e.task.abort();
                false
            } else {
                true
            }
        });
        running.len() < before
    }
}

impl Drop for Watches {
    fn drop(&mut self) {
        // A watch reports into its session, so it ends with it.
        for entry in self.lock().iter() {
            entry.task.abort();
        }
    }
}

fn watch_spec() -> ToolSpec {
    ToolSpec::new(
        "watch",
        "Keeps an eye on a topic in the background and reports when a condition holds, once or \
         each time it comes back: rate_below (hz: the camera or the scan slowing or going \
         silent), value (field, op, value: a reading crossing a line, such as \
         pose.pose.position.x > 5), or text (contains: a log line, such as /rosout saying \
         \"error\"). Returns at once; the report follows as a message.",
        json!({"type": "object", "properties": {
            "topic": {"type": "string", "description": "The topic, such as /scan."},
            "condition": {"type": "string", "enum": ["rate_below", "value", "text"]},
            "hz": {"type": "number", "description": "For rate_below."},
            "field": {"type": "string", "description": "For value: a dotted path into the message."},
            "op": {"type": "string", "enum": ["<", "<=", ">", ">=", "==", "!="]},
            "value": {"description": "For value: what to compare with."},
            "contains": {"type": "string", "description": "For text."},
            "say": {"type": "string", "description": "What the report should add, such as what to do then."},
            "repeat": {"type": "boolean", "description": "true for \"each time\", \"every time\" or \"whenever\": report each time it comes back. Otherwise it reports once and ends."},
            "for_s": {"type": "integer", "minimum": 1, "maximum": MAX_FOR_S, "description": "How long to watch (1800)."}
        }, "required": ["topic", "condition"], "additionalProperties": false}),
        Risk::Observe,
    )
}

fn plot_spec() -> ToolSpec {
    ToolSpec::new(
        "plot",
        "Draws a number from a topic's messages over time in the app's Plots tab, as rqt_plot \
         does: a speed, a joint's position, a battery's charge. Use it to show the operator how \
         something changes; watch reports a condition instead.",
        json!({"type": "object", "properties": {
            "topic": {"type": "string", "description": "The topic, such as /odom."},
            "field": {"type": "string", "description": "A dotted path to a number, such as twist.twist.linear.x or position.3."},
            "for_s": {"type": "integer", "minimum": 1, "maximum": MAX_PLOT_S, "description": "How long to draw it (120)."}
        }, "required": ["topic", "field"], "additionalProperties": false}),
        Risk::Observe,
    )
}

fn watches_spec() -> ToolSpec {
    ToolSpec::new(
        "watches",
        "Lists the running watches, or cancels one by id (or all).",
        json!({"type": "object", "properties": {
            "action": {"type": "string", "enum": ["list", "cancel"]},
            "watch": {"type": "string", "description": "For cancel: the id, such as w2, or all."}
        }, "required": ["action"], "additionalProperties": false}),
        Risk::Observe,
    )
}

struct WatchTool {
    spec: ToolSpec,
    watches: Arc<Watches>,
}

#[async_trait]
impl Tool for WatchTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        self.watches
            .start(&args)
            .await
            .unwrap_or_else(ToolOutcome::failed)
    }
}

struct WatchesTool {
    spec: ToolSpec,
    watches: Arc<Watches>,
}

struct PlotTool {
    spec: ToolSpec,
    watches: Arc<Watches>,
}

#[async_trait]
impl Tool for PlotTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        self.watches
            .plot(&args)
            .await
            .unwrap_or_else(ToolOutcome::failed)
    }
}

#[async_trait]
impl Tool for WatchesTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str() {
            Some("list") => ToolOutcome::ok(json!({"watches": self.watches.list()})),
            Some("cancel") => {
                let id = args["watch"].as_str().unwrap_or_default();
                // Cancelling all of none is done, not a mistake.
                if self.watches.cancel(id) || id == "all" {
                    ToolOutcome::ok(json!({"cancelled": id}))
                } else {
                    ToolOutcome::failed(format!("no watch `{id}`; watches lists them"))
                }
            }
            _ => ToolOutcome::failed("`action` is list or cancel"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nervros_ros::fake::FakeRobot;

    #[test]
    fn fields_are_found_by_path_and_compared_as_numbers_or_text() {
        let msg = json!({"pose": {"pose": {"position": {"x": 5.5}}}, "status": [{"level": 2, "name": "lidar"}]});
        assert_eq!(field(&msg, "pose.pose.position.x"), Some(&json!(5.5)));
        assert_eq!(field(&msg, "status.0.name"), Some(&json!("lidar")));
        assert!(field(&msg, "status.3.name").is_none());
        assert!(compare(&json!(5.5), ">", &json!(5)));
        assert!(!compare(&json!(5.5), "<=", &json!(5)));
        assert!(compare(&json!("lidar"), "==", &json!("lidar")));
        assert!(!compare(&json!("lidar"), "<", &json!("z")));
    }

    #[test]
    fn a_condition_says_what_it_lacks() {
        let err = |v: Value| Condition::parse(&v).unwrap_err();
        assert!(err(json!({"condition": "rate_below"})).contains("hz"));
        assert!(err(json!({"condition": "value", "field": "x"})).contains("value"));
        assert!(
            err(json!({"condition": "value", "field": "x", "op": "~", "value": 1})).contains("op")
        );
        assert!(err(json!({"condition": "soon"})).contains("rate_below"));
        assert_eq!(
            Condition::parse(&json!({"condition": "text", "contains": "ERROR"})).unwrap(),
            Condition::Text("error".into())
        );
    }

    #[tokio::test]
    async fn a_value_watch_reports_once_when_it_holds() {
        let robot: Arc<dyn RobotPort> = Arc::new(
            FakeRobot::new()
                .with_topic("/odom", json!({"pose": {"pose": {"position": {"x": 6.0}}}})),
        );
        let watches = Watches::new(robot, vec!["/secret*".to_owned()]);
        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
        let (events, _) = tokio::sync::broadcast::channel(8);
        watches.attach(SessionHandle::for_tests(&tx, events));
        let out = watches
            .start(
                &json!({"topic": "/odom", "condition": "value", "field": "pose.pose.position.x",
                           "op": ">", "value": 5, "say": "it left the room"}),
            )
            .await
            .unwrap();
        assert_eq!(out.data["watch"], "w1");
        let report = tokio::time::timeout(Duration::from_secs(3), rx.recv())
            .await
            .unwrap()
            .unwrap();
        match report {
            Command::Report(text) => {
                assert!(
                    text.contains("w1")
                        && text.contains("is 6.0")
                        && text.contains("left the room"),
                    "{text}"
                );
            }
            other => panic!("{other:?}"),
        }
        assert!(
            watches.list().as_array().unwrap().is_empty(),
            "a once watch ends"
        );
        assert!(!watches.cancel("w9"));
        let missing = watches
            .start(&json!({"topic": "/nothing", "condition": "text", "contains": "x"}))
            .await;
        assert!(missing.unwrap_err().contains("no topic"));
    }

    #[tokio::test]
    async fn a_plot_needs_a_number_and_asks_the_viewer_for_it() {
        let robot: Arc<dyn RobotPort> = Arc::new(FakeRobot::new().with_topic(
            "/odom",
            json!({"twist": {"twist": {"linear": {"x": 0.4}}}, "child_frame_id": "pelvis",
                   "data": vec![0; 40_000]}),
        ));
        let watches = Watches::new(robot, vec!["/secret*".to_owned()]);
        let (tx, _rx) = tokio::sync::mpsc::unbounded_channel();
        let (events, mut seen) = tokio::sync::broadcast::channel(8);
        watches.attach(SessionHandle::for_tests(&tx, events));
        let out = watches
            .plot(&json!({"topic": "/odom", "field": "twist.twist.linear.x"}))
            .await
            .unwrap();
        assert_eq!(out.data["now"], 0.4);
        assert!(
            matches!(seen.try_recv(), Ok(Event::Plot { field, for_s: 120, .. }) if field == "twist.twist.linear.x")
        );
        let text = watches
            .plot(&json!({"topic": "/odom", "field": "child_frame_id"}))
            .await;
        assert!(text.unwrap_err().contains("not a number"));
        // A grid's cells are named, not quoted: 40 000 of them once filled the model's context.
        let grid = watches
            .plot(&json!({"topic": "/odom", "field": "data"}))
            .await
            .unwrap_err();
        assert!(
            grid.contains("a list of 40000 values") && grid.len() < 200,
            "{grid}"
        );
        let missing = watches
            .plot(&json!({"topic": "/odom", "field": "pose.x"}))
            .await;
        assert!(missing.unwrap_err().contains("no `pose.x`"));
    }

    #[tokio::test]
    async fn a_text_watch_finds_a_log_line() {
        let robot = FakeRobot::new().with_topic("/rosout", json!({"msg": "Lidar ERROR: timeout"}));
        let seen = holds(
            &robot,
            "/rosout",
            "rcl_interfaces/msg/Log",
            &Condition::Text("error".into()),
        )
        .await;
        assert!(seen.unwrap().unwrap().contains("Lidar ERROR"));
    }

    #[test]
    fn a_float32_equals_its_decimal_and_a_quoted_number_compares() {
        let f32_tenth = json!(f64::from(0.2_f32));
        assert!(compare(&f32_tenth, "==", &json!(0.2)));
        assert!(!compare(&f32_tenth, "!=", &json!(0.2)));
        assert!(compare(&json!(6), ">", &json!("5")));
        assert!(!compare(&json!(5.0), ">", &json!(5)));
        assert!(compare(&json!(5.0), ">=", &json!(5)));
        assert!(
            Condition::parse(&json!({"condition": "value", "field": "x", "op": ">", "value": "a"}))
                .is_err()
        );
    }

    #[tokio::test]
    async fn a_topic_too_large_or_kept_from_reading_is_refused() {
        let typed = |name: &str, ty: &str| (name.to_owned(), vec![ty.to_owned()]);
        let robot: Arc<dyn RobotPort> =
            Arc::new(FakeRobot::new().with_graph(nervros_ros::GraphDetail {
                topics: vec![
                    typed("/camera", "sensor_msgs/msg/Image"),
                    typed("/secret_stuff", "std_msgs/msg/String"),
                ],
                ..nervros_ros::GraphDetail::default()
            }));
        let watches = Watches::new(Arc::clone(&robot), vec!["/secret*".to_owned()]);
        let image = |condition: &str| {
            json!({"topic": "/camera", "condition": condition, "contains": "a",
                   "field": "width", "op": ">", "value": 0, "hz": 5})
        };
        let text = watches.start(&image("text")).await;
        assert!(text.unwrap_err().contains("too large"));
        let secret = watches
            .start(&json!({"topic": "/secret_stuff", "condition": "text", "contains": "a"}))
            .await;
        assert!(secret.unwrap_err().contains("keeps /secret_stuff"));
        let rate = watches.start(&image("rate_below")).await;
        assert!(rate.is_ok(), "a camera's rate is cheap to watch: {rate:?}");
    }
}
