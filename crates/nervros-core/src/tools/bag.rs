//! The `bag` tool: records topics as `ros2 bag record` does, in-process, into an MCAP file that
//! `ros2 bag info` and `ros2 bag play` read. Messages go to disk as they came off the wire, never
//! decoded, so a camera costs no more than its bytes. There is no playback: it would publish on
//! the robot's topics.

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError, TrySendError};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use futures::StreamExt as _;
use mcap::records::MessageHeader;
use nervros_ros::RobotPort;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use tokio::sync::watch;
use tokio::task::JoinHandle;

use crate::background::Background;
use crate::session::{Command, SessionHandle};
use crate::tools::{Risk, SchemaSource, Tool, ToolOutcome, ToolSpec};
use crate::watch::topic_of;

const DEFAULT_S: u64 = 60;
const MAX_S: u64 = 3600;
const MAX_BYTES: u64 = 2 << 30;
const MAX_TOPICS: usize = 32;
const MAX_RECORDINGS: usize = 2;
/// Messages waiting for the disk. A full queue drops and counts rather than falling behind.
const QUEUE: usize = 4096;
/// How often a recording looks at its stop and its limits when nothing arrives.
const LOOK: Duration = Duration::from_millis(200);

/// One message on its way to the file: its topic's place, when it came, and its bytes.
type Arrived = (usize, u64, Vec<u8>);

/// A finished recording, kept beside its file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct Summary {
    file: PathBuf,
    started_s: f64,
    seconds: f64,
    bytes: u64,
    /// By topic.
    topics: BTreeMap<String, TopicSummary>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct TopicSummary {
    #[serde(rename = "type")]
    ty: String,
    messages: u64,
    dropped: u64,
}

/// One topic of a recording.
struct Recorded {
    topic: String,
    ty: String,
    definition: String,
    messages: u64,
    dropped: Arc<AtomicU64>,
}

struct Entry {
    id: String,
    what: String,
    started: Instant,
    stop: watch::Sender<bool>,
    task: JoinHandle<()>,
}

/// The recordings, and where they go.
pub struct Bags {
    robot: Arc<dyn RobotPort>,
    schemas: Arc<dyn SchemaSource>,
    read_deny: Vec<String>,
    dir: PathBuf,
    session: OnceLock<SessionHandle>,
    running: Mutex<Vec<Entry>>,
    next: AtomicU64,
}

impl Bags {
    /// No recordings yet; they are kept in `dir`, and `read_deny` is the profile's, as
    /// `topic_sample` keeps it.
    #[must_use]
    pub fn new(
        robot: Arc<dyn RobotPort>,
        schemas: Arc<dyn SchemaSource>,
        read_deny: Vec<String>,
        dir: PathBuf,
    ) -> Arc<Self> {
        Arc::new(Self {
            robot,
            schemas,
            read_deny,
            dir,
            session: OnceLock::new(),
            running: Mutex::default(),
            next: AtomicU64::new(1),
        })
    }

    /// Where a recording reports when it ends.
    pub fn attach(&self, session: SessionHandle) {
        let _ = self.session.set(session);
    }

    fn lock(&self) -> MutexGuard<'_, Vec<Entry>> {
        crate::lock(&self.running)
    }

    /// The `bag` tool.
    #[must_use]
    pub fn tool(self: &Arc<Self>) -> Arc<dyn Tool> {
        Arc::new(BagTool {
            spec: spec(),
            bags: Arc::clone(self),
        })
    }

    async fn record(self: &Arc<Self>, args: &Value) -> Result<ToolOutcome, String> {
        let name = bag_name(args)?;
        let file = self.dir.join(format!("{name}.mcap"));
        if file.exists() {
            return Err(format!(
                "a recording `{name}` exists already; give another name"
            ));
        }
        let mut topics = self.topics(args).await?;
        let seconds = args["seconds"]
            .as_u64()
            .unwrap_or(DEFAULT_S)
            .clamp(1, MAX_S);
        let what = format!(
            "{} to {name}.mcap",
            topics
                .iter()
                .map(|t| t.topic.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        );
        let (tx, rx) = mpsc::sync_channel(QUEUE);
        let mut readers = Vec::with_capacity(topics.len());
        for (at, t) in topics.iter().enumerate() {
            let mut messages = self
                .robot
                .raw(&t.topic, &t.ty)
                .await
                .map_err(|e| format!("cannot read {}: {e}", t.topic))?;
            let (tx, dropped) = (tx.clone(), Arc::clone(&t.dropped));
            readers.push(tokio::spawn(async move {
                while let Some(bytes) = messages.next().await {
                    let now = crate::now_s();
                    #[expect(
                        clippy::cast_possible_truncation,
                        clippy::cast_sign_loss,
                        reason = "nanoseconds since the epoch, for centuries yet"
                    )]
                    let at_ns = (now * 1e9) as u64;
                    match tx.try_send((at, at_ns, bytes)) {
                        Ok(()) => {}
                        Err(TrySendError::Full(_)) => {
                            dropped.fetch_add(1, Ordering::Relaxed);
                        }
                        Err(TrySendError::Disconnected(_)) => return,
                    }
                }
            }));
        }
        drop(tx);
        let (mut running, id) = self.room()?;
        let (stop, stopped) = watch::channel(false);
        let session = self.session.get().cloned();
        let (task_id, dir) = (id.clone(), self.dir.clone());
        let task = tokio::spawn(async move {
            let until = Duration::from_secs(seconds);
            let ended = tokio::task::spawn_blocking(move || {
                std::fs::create_dir_all(&dir)
                    .map_err(|e| format!("making {}: {e}", dir.display()))?;
                write(&file, &mut topics, &rx, &stopped, until)
            })
            .await
            .map_err(|e| e.to_string())
            .and_then(|written| written);
            for reader in readers {
                reader.abort();
            }
            let report = match ended {
                Ok(s) => format!(
                    "Recording {task_id} ended: {} in {:.0} s, {} MB, in {}.{}",
                    crate::counted(
                        s.topics.values().map(|t| t.messages).sum::<u64>(),
                        "message"
                    ),
                    s.seconds,
                    s.bytes >> 20,
                    s.file.display(),
                    lost(&s)
                ),
                Err(e) => format!("Recording {task_id} failed: {e}."),
            };
            if let Some(s) = session {
                s.send(Command::Report(report));
            }
        });
        running.push(Entry {
            id: id.clone(),
            what: what.clone(),
            started: Instant::now(),
            stop,
            task,
        });
        let mut out = ToolOutcome::ok(json!({"recording": id, "what": what, "seconds": seconds}));
        out.message =
            format!("recording {what} for up to {seconds} s; a report comes when it ends");
        Ok(out)
    }

    /// The topics named, each one the agent may read and with its definition.
    async fn topics(&self, args: &Value) -> Result<Vec<Recorded>, String> {
        let named = args["topics"]
            .as_array()
            .filter(|t| !t.is_empty())
            .ok_or("`topics` lists what to record, such as [\"/scan\"]")?;
        if named.len() > MAX_TOPICS {
            return Err(format!("at most {MAX_TOPICS} topics at once"));
        }
        let mut topics = Vec::with_capacity(named.len());
        for topic in named {
            let asked = json!({"topic": topic});
            // Whole messages are never decoded here, so a camera is as good as any.
            let (topic, ty) = topic_of(self.robot.as_ref(), &asked, &self.read_deny, false).await?;
            let definition = self.schemas.definition(&ty).unwrap_or_default();
            topics.push(Recorded {
                topic,
                ty,
                definition,
                messages: 0,
                dropped: Arc::default(),
            });
        }
        Ok(topics)
    }

    /// The running recordings, with room for one more, and its id.
    fn room(&self) -> Result<(MutexGuard<'_, Vec<Entry>>, String), String> {
        let mut running = self.lock();
        running.retain(|e| !e.task.is_finished());
        if running.len() >= MAX_RECORDINGS {
            return Err(format!(
                "{MAX_RECORDINGS} recordings run already; cancel one with background first"
            ));
        }
        let id = format!("b{}", self.next.fetch_add(1, Ordering::Relaxed));
        Ok((running, id))
    }

    /// What was recorded, by name, or all of it.
    fn info(&self, args: &Value) -> Result<ToolOutcome, String> {
        if args["name"].is_string() {
            let name = bag_name(args)?;
            let summary = summary_of(&self.dir.join(format!("{name}.json")))
                .ok_or_else(|| format!("no recording `{name}`; info without a name lists them"))?;
            return Ok(ToolOutcome::ok(json!({"name": name, "recording": summary})));
        }
        let mut names: Vec<String> = std::fs::read_dir(&self.dir)
            .into_iter()
            .flatten()
            .flatten()
            .filter_map(|e| {
                let path = e.path();
                let name = path.file_stem()?.to_str()?.to_owned();
                (path.extension()? == "json").then_some(name)
            })
            .collect();
        names.sort();
        Ok(ToolOutcome::ok(
            json!({"folder": self.dir, "recordings": names}),
        ))
    }
}

fn summary_of(path: &Path) -> Option<Summary> {
    serde_json::from_slice(&std::fs::read(path).ok()?).ok()
}

/// What a report says about messages the disk could not keep up with.
fn lost(summary: &Summary) -> String {
    let dropped: u64 = summary.topics.values().map(|t| t.dropped).sum();
    if dropped == 0 {
        String::new()
    } else {
        format!(" It dropped {dropped}: the disk fell behind.")
    }
}

/// The recording's name: letters, digits, `-` and `_` only, as it names files. By the clock when
/// none is given.
fn bag_name(args: &Value) -> Result<String, String> {
    let Some(name) = args["name"]
        .as_str()
        .map(str::trim)
        .filter(|n| !n.is_empty())
    else {
        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "seconds since the epoch"
        )]
        return Ok(format!("bag-{}", crate::now_s() as u64));
    };
    if name
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
    {
        Ok(name.to_owned())
    } else {
        Err(format!(
            "`{name}`: a name has only letters, digits, - and _"
        ))
    }
}

/// Writes what arrives into `file` until the stop, the time limit, the size limit or the end of
/// every topic, then closes it, which is what makes it readable, and keeps its summary beside it.
fn write(
    file: &Path,
    topics: &mut [Recorded],
    arrivals: &mpsc::Receiver<Arrived>,
    stopped: &watch::Receiver<bool>,
    until: Duration,
) -> Result<Summary, String> {
    let failed = |e: mcap::McapError| format!("writing {}: {e}", file.display());
    let started = (Instant::now(), crate::now_s());
    let out = File::create_new(file).map_err(|e| format!("creating {}: {e}", file.display()))?;
    let mut bag = mcap::WriteOptions::new()
        .compression(None)
        .profile("ros2")
        .library("nervros")
        .create(BufWriter::new(out))
        .map_err(failed)?;
    let mut channels = Vec::with_capacity(topics.len());
    for t in topics.iter() {
        let schema = bag
            .add_schema(&t.ty, "ros2msg", t.definition.as_bytes())
            .map_err(failed)?;
        channels.push(
            bag.add_channel(schema, &t.topic, "cdr", &BTreeMap::new())
                .map_err(failed)?,
        );
    }
    let mut bytes = 0u64;
    while !*stopped.borrow() && started.0.elapsed() < until && bytes < MAX_BYTES {
        match arrivals.recv_timeout(LOOK) {
            Ok((at, time, data)) => {
                let t = &mut topics[at];
                let sequence = u32::try_from(t.messages).unwrap_or(u32::MAX);
                let header = MessageHeader {
                    channel_id: channels[at],
                    sequence,
                    log_time: time,
                    publish_time: time,
                };
                bag.write_to_known_channel(&header, &data).map_err(failed)?;
                t.messages += 1;
                bytes += data.len() as u64;
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }
    }
    bag.finish().map_err(failed)?;
    drop(bag);
    let summary = Summary {
        file: file.to_owned(),
        started_s: started.1,
        seconds: started.0.elapsed().as_secs_f64(),
        bytes,
        topics: topics
            .iter()
            .map(|t| {
                let counts = TopicSummary {
                    ty: t.ty.clone(),
                    messages: t.messages,
                    dropped: t.dropped.load(Ordering::Relaxed),
                };
                (t.topic.clone(), counts)
            })
            .collect(),
    };
    crate::persist::write_json(&file.with_extension("json"), &summary)?;
    Ok(summary)
}

impl Background for Bags {
    fn list(&self) -> Vec<Value> {
        let mut running = self.lock();
        running.retain(|e| !e.task.is_finished());
        running
            .iter()
            .map(|e| {
                json!({"id": e.id, "kind": "recording", "what": e.what,
                       "running_s": e.started.elapsed().as_secs()})
            })
            .collect()
    }

    fn end(&self, id: &str) -> bool {
        let running = self.lock();
        let mut any = false;
        // Told, not aborted: only a recording that closes its file leaves one that can be read.
        for e in running
            .iter()
            .filter(|e| !e.task.is_finished() && (id == "all" || e.id == id))
        {
            e.stop.send_replace(true);
            any = true;
        }
        any
    }
}

impl Drop for Bags {
    fn drop(&mut self) {
        self.end("all");
    }
}

fn spec() -> ToolSpec {
    ToolSpec::new(
        "bag",
        "Records topics to a file, like ros2 bag record, for looking at later with ros2 bag info \
         and ros2 bag play outside this agent; it never plays one back. record takes topics, \
         seconds (up to 3600, 60 if left out) and a name; it returns at once and reports when it \
         ends, at the time limit or at 2 GB; background lists and ends it. info shows a \
         recording by name, or lists them.",
        json!({"type": "object", "properties": {
            "action": {"type": "string", "enum": ["record", "info"]},
            "topics": {"type": "array", "items": {"type": "string"}, "description": "For record: topic names."},
            "seconds": {"type": "integer", "minimum": 1, "maximum": MAX_S},
            "name": {"type": "string", "description": "Letters, digits, - and _."}
        }, "required": ["action"], "additionalProperties": false}),
        Risk::Observe,
    )
}

struct BagTool {
    spec: ToolSpec,
    bags: Arc<Bags>,
}

#[async_trait]
impl Tool for BagTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str() {
            Some("record") => self.bags.record(&args).await,
            Some("info") => self.bags.info(&args),
            _ => Err("`action` is record or info".to_owned()),
        }
        .unwrap_or_else(ToolOutcome::failed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::Status;
    use nervros_ros::fake::FakeRobot;

    struct NoSchemas;

    impl SchemaSource for NoSchemas {
        fn schema(
            &self,
            _: &str,
            _: crate::tools::SchemaPart,
            _: &[String],
        ) -> Result<Value, String> {
            Ok(json!({}))
        }
    }

    fn bags(dir: &Path, deny: &[&str]) -> Arc<Bags> {
        let robot = FakeRobot::new()
            .with_topic("/scan", json!({}))
            .with_topic("/secret", json!({}))
            .with_raw("/scan", vec![vec![0, 1, 0, 0, 7], vec![0, 1, 0, 0, 8]]);
        Bags::new(
            Arc::new(robot),
            Arc::new(NoSchemas),
            deny.iter().map(ToString::to_string).collect(),
            dir.to_owned(),
        )
    }

    #[tokio::test]
    async fn a_recording_told_to_end_leaves_a_file_with_every_message() {
        let dir = tempfile::tempdir().unwrap();
        let bags = bags(dir.path(), &[]);
        let tool = bags.tool();
        let out = tool
            .call(json!({"action": "record", "topics": ["/scan"], "name": "first"}))
            .await;
        assert_eq!(out.status, Status::Succeeded, "{}", out.message);
        assert_eq!(bags.list()[0]["id"], "b1");
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(bags.end("b1"));
        while !bags.list().is_empty() {
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let file = std::fs::read(dir.path().join("first.mcap")).unwrap();
        let messages: Vec<_> = mcap::MessageStream::new(&file)
            .unwrap()
            .collect::<Result<_, _>>()
            .unwrap();
        assert_eq!(messages.len(), 2);
        assert_eq!(messages[0].channel.topic, "/scan");
        assert_eq!(messages[0].channel.message_encoding, "cdr");
        assert_eq!(*messages[1].data, [0, 1, 0, 0, 8]);
        let info = tool.call(json!({"action": "info", "name": "first"})).await;
        assert_eq!(info.data["recording"]["topics"]["/scan"]["messages"], 2);
        let all = tool.call(json!({"action": "info"})).await;
        assert_eq!(all.data["recordings"], json!(["first"]));
    }

    #[tokio::test]
    async fn it_records_only_what_may_be_read_and_names_only_files() {
        let dir = tempfile::tempdir().unwrap();
        let tool = bags(dir.path(), &["/secret*"]).tool();
        let denied = tool
            .call(json!({"action": "record", "topics": ["/secret"]}))
            .await;
        assert_eq!(denied.status, Status::Failed);
        let escape = tool
            .call(json!({"action": "record", "topics": ["/scan"], "name": "../up"}))
            .await;
        assert_eq!(escape.status, Status::Failed);
        assert!(escape.message.contains("letters"), "{}", escape.message);
        let none = tool.call(json!({"action": "record", "topics": []})).await;
        assert_eq!(none.status, Status::Failed);
    }
}
