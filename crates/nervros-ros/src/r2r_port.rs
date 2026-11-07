//! [`RobotPort`] over r2r.
//!
//! One node lives on its own thread (`nervros-ros-spin`) and spins every few milliseconds. r2r
//! needs `&mut Node` to create clients and subscriptions, so the async side asks for them over a
//! command channel; requests, goals and streams then run on the tokio runtime and are resolved by
//! the spinning thread.

use std::collections::HashMap;
use std::fmt::Write as _;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc as std_mpsc;
use std::sync::{Arc, Mutex, OnceLock};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use async_trait::async_trait;
use bytes::Bytes;
use futures::StreamExt as _;
use futures::stream::BoxStream;
use r2r::QosProfile;
use serde_json::Value;
use tokio::sync::{mpsc, oneshot, watch};

use crate::lock;
use crate::tf::TfBuffer;
use crate::{
    Endpoint, Frame, Goal, GoalResult, GoalStatus, Graph, GraphDetail, NamesAndTypes, NodeEntities,
    Publisher, QosInfo, RobotPort, RosError, TfLink, TopicEndpoints, Transform,
};

/// How the node is set up.
#[derive(Debug, Clone)]
pub struct R2rConfig {
    /// Node name; grove-g1's `clean-stack.sh` knows `nervros`.
    pub node_name: String,
    /// Node namespace.
    pub namespace: String,
    /// Time between spins.
    pub spin_period: Duration,
}

impl Default for R2rConfig {
    fn default() -> Self {
        Self {
            node_name: "nervros".to_owned(),
            namespace: String::new(),
            spin_period: Duration::from_millis(5),
        }
    }
}

enum Cmd {
    Client {
        name: String,
        ty: String,
        reply: oneshot::Sender<r2r::Result<r2r::ClientUntyped>>,
    },
    ActionClient {
        name: String,
        ty: String,
        reply: oneshot::Sender<r2r::Result<r2r::ActionClientUntyped>>,
    },
    SubscribeJson {
        topic: String,
        ty: String,
        tx: watch::Sender<Option<Sample>>,
        /// Why its messages cannot be read, once one could not.
        undecodable: Arc<Mutex<Option<String>>>,
        /// Whether the topic had no publisher, so that the `QoS` may miss a latched message, and
        /// the publishers it was made for.
        reply: oneshot::Sender<Result<(bool, Vec<String>), RosError>>,
    },
    SubscribeImage {
        topic: String,
        tx: watch::Sender<Option<Arc<Frame>>>,
    },
    Graph {
        reply: oneshot::Sender<r2r::Result<HashMap<String, Vec<String>>>>,
    },
    GraphDetail {
        reply: oneshot::Sender<Result<GraphDetail, RosError>>,
    },
    Endpoints {
        topic: String,
        reply: oneshot::Sender<Result<TopicEndpoints, RosError>>,
    },
    NodeEntities {
        name: String,
        namespace: String,
        reply: oneshot::Sender<Result<NodeEntities, RosError>>,
    },
    SubscribeRaw {
        topic: String,
        ty: String,
        hearing: Hearing,
        reply: oneshot::Sender<Result<BoxStream<'static, Vec<u8>>, RosError>>,
    },
    SubscribeSample {
        topic: String,
        ty: String,
        reply: oneshot::Sender<Result<BoxStream<'static, r2r::Result<Value>>, RosError>>,
    },
    Publisher {
        topic: String,
        ty: String,
        keeps: Keeps,
        reply: oneshot::Sender<r2r::Result<r2r::PublisherUntyped>>,
    },
    DestroyPublisher {
        publisher: r2r::PublisherUntyped,
    },
}

/// How a raw subscription asks for messages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Hearing {
    /// Best effort, as `ros2 topic hz` subscribes: it matches every publisher.
    Any,
    /// As its publishers offer, so a latched topic gives what it holds already.
    AsOffered,
}

/// What a publisher keeps for subscribers that join later.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Keeps {
    /// Nothing: volatile, depth 10.
    Nothing,
    /// Its last message: transient local, depth 1.
    Last,
}

/// A message that could not be converted, to or from `name`'s type.
fn conversion(name: &str, e: impl std::fmt::Display) -> RosError {
    RosError::Conversion {
        name: name.to_owned(),
        message: e.to_string(),
    }
}

type Key = (String, String);

/// A message as it arrived.
#[derive(Debug, Clone)]
struct Sample {
    at: Instant,
    value: Arc<Value>,
}

/// A topic read as JSON, kept subscribed for the next read.
struct Topic {
    ty: String,
    rx: watch::Receiver<Option<Sample>>,
    /// Subscribed while nothing published it, with a `QoS` that misses a latched message.
    blind: bool,
    since: Instant,
    /// The publishers it was made for, by gid.
    publishers: Vec<String>,
    /// When it was last read.
    read: Instant,
    /// When its publishers were last compared with those it was made for.
    checked: Instant,
    /// Why its messages cannot be read, once one could not.
    undecodable: Arc<Mutex<Option<String>>>,
}

/// What a read finds kept for a topic.
enum Kept {
    /// A subscription to read.
    Ready(watch::Receiver<Option<Sample>>),
    /// One that has heard nothing for a while, and the publishers it was made for.
    Quiet(watch::Receiver<Option<Sample>>, Vec<String>),
    /// None to use: one is made.
    Gone,
}

/// A blind subscription that has had nothing this long is made again, with the `QoS` of the
/// publishers that may have come since.
const RESUBSCRIBE: Duration = Duration::from_secs(2);
/// A subscription nobody read this long is let go: it ends with its last reader, and stops
/// turning every message, a whole cloud or map, into JSON.
const IDLE: Duration = Duration::from_mins(1);
/// A kept subscription that has heard nothing this long is checked against the publishers there
/// are now: one that restarted may offer a `QoS` it no longer matches.
const QUIET: Duration = Duration::from_secs(5);
/// How many messages of a latched topic a recording asks for: `/tf_static` holds many.
const LATCHED_DEPTH: usize = 100;

/// A running r2r node.
pub struct R2rPort {
    cmd: std_mpsc::Sender<Cmd>,
    clients: Mutex<HashMap<Key, Arc<r2r::ClientUntyped>>>,
    actions: Mutex<HashMap<Key, r2r::ActionClientUntyped>>,
    json: Mutex<HashMap<String, Topic>>,
    images: Mutex<HashMap<String, watch::Receiver<Option<Arc<Frame>>>>>,
    tf: Arc<Mutex<TfBuffer>>,
    /// Set to end the spin loop: kept publishers hold the command channel open.
    stopping: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    /// When the node started: until discovery has had its time, the graph may not yet list a
    /// publisher that exists.
    started: Instant,
    /// The node's ROS clock, which follows `/clock` under `use_sim_time`.
    clock: Arc<Mutex<r2r::Clock>>,
}

/// How long discovery takes to list the publishers already running.
const DISCOVERY: Duration = Duration::from_secs(3);

/// The oldest a `/tf` link may be for a transform built on it. Odometry and localisation publish
/// at 10 Hz or more, so a link this old means its publisher stopped.
const TF_MAX_AGE: Duration = Duration::from_secs(5);

/// How long a goal's cancel may take to be answered. A stop awaits it, so a wedged or dead server
/// must not hold the stop for ever.
const CANCEL_TIMEOUT: Duration = Duration::from_secs(5);

/// How long a publisher outlives its last message: reliable delivery needs it a moment longer.
const PUBLISH_GRACE: Duration = Duration::from_millis(200);

/// How long a goal its caller gave up on may still be accepted, and so need cancelling.
const LATE_ACCEPT: Duration = Duration::from_mins(1);

/// The node's logger, once the node is up.
static LOGGER: OnceLock<String> = OnceLock::new();

/// A warning the robot's people should see too: through the node's logger, to `/rosout`.
fn to_rosout(text: &str) {
    if let Some(logger) = LOGGER.get() {
        // rcutils takes a C string: a NUL in the text would end it, and r2r would panic.
        r2r::log_warn!(logger, "{}", text.replace('\0', " "));
    }
}

/// Cancels a goal nobody watches any more: one accepted after its caller gave up, or handed to a
/// caller that was gone before it took it. A failure is an error, since the goal may still run.
async fn cancel_unwatched(goal: r2r::ActionClientGoalUntyped, action: &str) {
    tracing::warn!(action, goal = %goal.uuid, "a goal nobody watches was accepted; cancelling");
    to_rosout(&format!(
        "{action}: a goal nobody watches was accepted; cancelling it"
    ));
    let outcome = match goal.cancel() {
        Ok(pending) => match tokio::time::timeout(CANCEL_TIMEOUT, pending).await {
            Ok(Ok(())) => return,
            Ok(Err(e)) => e.to_string(),
            Err(_) => format!("no answer in {} s", CANCEL_TIMEOUT.as_secs()),
        },
        Err(e) => e.to_string(),
    };
    tracing::error!(action, goal = %goal.uuid, %outcome, "could not cancel a goal nobody watches: it may still run");
    to_rosout(&format!(
        "{action}: could not cancel a goal nobody watches ({outcome}): it may still run"
    ));
}

impl std::fmt::Debug for R2rPort {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("R2rPort").finish_non_exhaustive()
    }
}

/// The most service or action clients kept, each name and type once: r2r cannot destroy one, so a
/// model trying names without end must not grow them without end.
const MAX_CLIENTS: usize = 256;

fn too_many_clients(name: &str) -> RosError {
    RosError::Middleware(format!(
        "{name}: NervROS has made {MAX_CLIENTS} clients and makes no more until it restarts"
    ))
}

fn mw(e: impl std::fmt::Display) -> RosError {
    RosError::Middleware(e.to_string())
}

impl R2rPort {
    /// Starts the node and its TF listener. Must be called inside a tokio runtime.
    ///
    /// # Errors
    ///
    /// The ROS context or node could not be created.
    pub fn start(config: R2rConfig) -> Result<Self, RosError> {
        let runtime = tokio::runtime::Handle::try_current().map_err(mw)?;
        let threaded = runtime.runtime_flavor() == tokio::runtime::RuntimeFlavor::MultiThread;
        let tf = Arc::new(Mutex::new(TfBuffer::default()));
        // Unbounded on purpose: its senders are async tasks that must never block on a send, and
        // the spin thread takes each command as it comes.
        let (cmd_tx, cmd_rx) = std_mpsc::channel();
        let (ready_tx, ready_rx) = std_mpsc::channel();
        let tf_thread = Arc::clone(&tf);
        let stopping = Arc::new(AtomicBool::new(false));
        let stop_thread = Arc::clone(&stopping);
        let thread = std::thread::Builder::new()
            .name("nervros-ros-spin".to_owned())
            .spawn(move || {
                spin(
                    &config,
                    &runtime,
                    &tf_thread,
                    &cmd_rx,
                    &ready_tx,
                    &stop_thread,
                );
            })
            .map_err(mw)?;
        // Waiting for rcl to start must not hold a runtime worker another task needs.
        let ready = if threaded {
            tokio::task::block_in_place(|| ready_rx.recv())
        } else {
            ready_rx.recv()
        };
        let clock = ready.map_err(mw)??;
        Ok(Self {
            cmd: cmd_tx,
            clients: Mutex::default(),
            actions: Mutex::default(),
            json: Mutex::default(),
            images: Mutex::default(),
            tf,
            stopping,
            thread: Some(thread),
            started: Instant::now(),
            clock,
        })
    }

    fn send(&self, cmd: Cmd) -> Result<(), RosError> {
        self.cmd
            .send(cmd)
            .map_err(|_| RosError::Middleware("the ROS node thread has stopped".to_owned()))
    }

    /// Sends the command `cmd` builds round its reply channel, and waits for the answer.
    async fn ask<T>(&self, cmd: impl FnOnce(oneshot::Sender<T>) -> Cmd) -> Result<T, RosError> {
        let (reply, rx) = oneshot::channel();
        self.send(cmd(reply))?;
        rx.await.map_err(mw)
    }

    /// A publisher kept until its handle drops, each message sent as it is set.
    async fn kept_publisher(
        &self,
        topic: &str,
        ty: &str,
        keeps: Keeps,
    ) -> Result<Publisher, RosError> {
        let publisher = self
            .ask(|reply| Cmd::Publisher {
                topic: topic.to_owned(),
                ty: ty.to_owned(),
                keeps,
                reply,
            })
            .await?
            .map_err(|e| type_error(ty, &e))?;
        let (handle, mut messages) = Publisher::channel();
        let cmd = self.cmd.clone();
        let topic = topic.to_owned();
        tokio::spawn(async move {
            let mut warned = false;
            // Ends when the handle is dropped.
            while messages.changed().await.is_ok() {
                let Some(message) = messages.borrow_and_update().clone() else {
                    continue;
                };
                if let Err(e) = publisher.publish(message)
                    && !warned
                {
                    tracing::warn!(%topic, error = %e, "a kept publisher could not publish");
                    warned = true;
                }
            }
            // As for a one-off publish: reliable delivery of the last message needs a moment.
            tokio::time::sleep(PUBLISH_GRACE).await;
            let _ = cmd.send(Cmd::DestroyPublisher { publisher });
        });
        Ok(handle)
    }

    async fn client(&self, name: &str, ty: &str) -> Result<Arc<r2r::ClientUntyped>, RosError> {
        let key = (name.to_owned(), ty.to_owned());
        if let Some(c) = lock(&self.clients).get(&key) {
            return Ok(Arc::clone(c));
        }
        // rcl keeps a client for the node's life: names tried past this many are refused.
        if lock(&self.clients).len() >= MAX_CLIENTS {
            return Err(too_many_clients(name));
        }
        let client = self
            .ask(|reply| Cmd::Client {
                name: name.to_owned(),
                ty: ty.to_owned(),
                reply,
            })
            .await?
            .map_err(|e| type_error(ty, &e))?;
        let client = Arc::new(client);
        lock(&self.clients).insert(key, Arc::clone(&client));
        Ok(client)
    }

    /// What is kept for reading `topic` as JSON; the topics nobody read for a while let go.
    fn kept(&self, topic: &str, ty: &str) -> Kept {
        let mut json = lock(&self.json);
        json.retain(|_, t| t.read.elapsed() < IDLE);
        let Some(t) = json.get_mut(topic).filter(|t| t.ty == ty) else {
            return Kept::Gone;
        };
        let newest = t.rx.borrow().as_ref().map(|s| s.at);
        if t.blind && newest.is_none() && t.since.elapsed() >= RESUBSCRIBE {
            return Kept::Gone;
        }
        t.read = Instant::now();
        if newest.is_none_or(|at| at.elapsed() >= QUIET) && t.checked.elapsed() >= RESUBSCRIBE {
            t.checked = Instant::now();
            return Kept::Quiet(t.rx.clone(), t.publishers.clone());
        }
        Kept::Ready(t.rx.clone())
    }

    /// The subscription for reading `topic` as JSON: kept from an earlier read of the same type,
    /// or made now. It is made again when it began before anything published and still has
    /// nothing, and when it has heard nothing for a while and its publishers changed; its last
    /// message is kept then.
    async fn subscription(
        &self,
        topic: &str,
        ty: &str,
    ) -> Result<watch::Receiver<Option<Sample>>, RosError> {
        let carried = match self.kept(topic, ty) {
            Kept::Ready(rx) => return Ok(rx),
            Kept::Quiet(rx, made_for) => match self.endpoints(topic).await {
                Ok(now) if gids(&now.publishers) != made_for => rx.borrow().clone(),
                _ => return Ok(rx),
            },
            Kept::Gone => None,
        };
        let (tx, rx) = watch::channel(carried);
        let undecodable: Arc<Mutex<Option<String>>> = Arc::default();
        let (blind, publishers) = self
            .ask(|reply| Cmd::SubscribeJson {
                topic: topic.to_owned(),
                ty: ty.to_owned(),
                tx,
                undecodable: Arc::clone(&undecodable),
                reply,
            })
            .await??;
        let now = Instant::now();
        lock(&self.json).insert(
            topic.to_owned(),
            Topic {
                ty: ty.to_owned(),
                rx: rx.clone(),
                blind,
                since: now,
                publishers,
                read: now,
                checked: now,
                undecodable,
            },
        );
        Ok(rx)
    }

    /// The newest message on `topic`, waiting up to `wait` for one, and for one no older than
    /// `max_age` when given.
    async fn newest(
        &self,
        topic: &str,
        ty: &str,
        wait: Duration,
        max_age: Option<Duration>,
    ) -> Result<Arc<Value>, RosError> {
        let mut rx = self.subscription(topic, ty).await?;
        let nothing_yet = rx.borrow().is_none();
        // Nothing yet and nobody publishing: there is nothing to wait for.
        if nothing_yet
            && self.started.elapsed() > DISCOVERY
            && self
                .endpoints(topic)
                .await
                .is_ok_and(|e| e.publishers.is_empty())
        {
            return Err(RosError::NoData(topic.to_owned()));
        }
        let fresh = |s: &Option<Sample>| {
            s.as_ref()
                .is_some_and(|s| max_age.is_none_or(|max| s.at.elapsed() <= max))
        };
        let answer = match tokio::time::timeout(wait, rx.wait_for(fresh)).await {
            Ok(Ok(s)) => s.as_ref().map(|s| Arc::clone(&s.value)),
            _ => None,
        };
        // Nothing came because nothing could be read: say why rather than that nothing came.
        answer.ok_or_else(|| {
            let why = lock(&self.json)
                .get(topic)
                .and_then(|t| lock(&t.undecodable).clone());
            why.map_or_else(
                || RosError::NoData(topic.to_owned()),
                |message| conversion(topic, message),
            )
        })
    }

    async fn action_client(
        &self,
        name: &str,
        ty: &str,
    ) -> Result<r2r::ActionClientUntyped, RosError> {
        let key = (name.to_owned(), ty.to_owned());
        if let Some(c) = lock(&self.actions).get(&key) {
            return Ok(c.clone());
        }
        if lock(&self.actions).len() >= MAX_CLIENTS {
            return Err(too_many_clients(name));
        }
        let client = self
            .ask(|reply| Cmd::ActionClient {
                name: name.to_owned(),
                ty: ty.to_owned(),
                reply,
            })
            .await?
            .map_err(|e| type_error(ty, &e))?;
        lock(&self.actions).insert(key, client.clone());
        Ok(client)
    }
}

impl Drop for R2rPort {
    fn drop(&mut self) {
        // The thread exits within one spin period, whoever still holds the command channel.
        self.stopping.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take()
            && t.join().is_err()
        {
            tracing::warn!("the ROS node thread panicked");
        }
    }
}

/// Destroys a publisher when dropped.
struct Destroy<'a> {
    cmd: &'a std_mpsc::Sender<Cmd>,
    publisher: Option<r2r::PublisherUntyped>,
}

impl Drop for Destroy<'_> {
    fn drop(&mut self) {
        if let Some(publisher) = self.publisher.take() {
            let _ = self.cmd.send(Cmd::DestroyPublisher { publisher });
        }
    }
}

fn type_error(ty: &str, e: &r2r::Error) -> RosError {
    match e {
        r2r::Error::InvalidMessageType { .. } => RosError::UnknownType(ty.to_owned()),
        other => mw(other),
    }
}

fn spin(
    config: &R2rConfig,
    runtime: &tokio::runtime::Handle,
    tf: &Arc<Mutex<TfBuffer>>,
    cmds: &std_mpsc::Receiver<Cmd>,
    ready: &std_mpsc::Sender<Result<Arc<Mutex<r2r::Clock>>, RosError>>,
    stopping: &AtomicBool,
) {
    let node = r2r::Context::create()
        .and_then(|ctx| r2r::Node::create(ctx, &config.node_name, &config.namespace));
    let mut node = match node {
        Ok(node) => node,
        Err(e) => {
            let _ = ready.send(Err(mw(e)));
            return;
        }
    };
    let _ = LOGGER.set(node.logger().to_owned());
    if let Err(e) = listen_tf(&mut node, runtime, tf) {
        tracing::warn!(error = %e, "TF listener not started");
        to_rosout(&format!("TF listener not started: {e}"));
    }
    // Launched with `use_sim_time:=true`, the clock follows `/clock` as an rclcpp node's does.
    let sim_time = matches!(
        lock(&node.params).get("use_sim_time").map(|p| &p.value),
        Some(r2r::ParameterValue::Bool(true))
    );
    if sim_time && let Err(e) = node.get_time_source().enable_sim_time(&mut node) {
        tracing::warn!(error = %e, "simulated time not followed");
        to_rosout(&format!("simulated time not followed: {e}"));
    }
    if ready.send(Ok(node.get_ros_clock())).is_err() {
        return;
    }
    while !stopping.load(Ordering::SeqCst) {
        loop {
            match cmds.try_recv() {
                Ok(cmd) => handle(&mut node, runtime, cmd),
                Err(std_mpsc::TryRecvError::Empty) => break,
                Err(std_mpsc::TryRecvError::Disconnected) => return,
            }
        }
        node.spin_once(config.spin_period);
    }
}

fn listen_tf(
    node: &mut r2r::Node,
    runtime: &tokio::runtime::Handle,
    tf: &Arc<Mutex<TfBuffer>>,
) -> r2r::Result<()> {
    // Best effort hears every publisher, a best-effort one as well as a reliable one.
    let dynamic = node.subscribe::<r2r::tf2_msgs::msg::TFMessage>(
        "/tf",
        QosProfile::default().keep_last(100).best_effort(),
    )?;
    let statics = node.subscribe::<r2r::tf2_msgs::msg::TFMessage>(
        "/tf_static",
        QosProfile::default()
            .keep_last(100)
            .reliable()
            .transient_local(),
    )?;
    for (mut stream, is_static) in [(dynamic.boxed(), false), (statics.boxed(), true)] {
        let tf = Arc::clone(tf);
        runtime.spawn(async move {
            while let Some(msg) = stream.next().await {
                let mut buf = lock(&tf);
                for t in msg.transforms {
                    let (p, q) = (t.transform.translation, t.transform.rotation);
                    buf.insert_from(
                        &t.header.frame_id,
                        &t.child_frame_id,
                        Transform {
                            translation: [p.x, p.y, p.z],
                            rotation: [q.x, q.y, q.z, q.w],
                        },
                        is_static,
                    );
                }
            }
        });
    }
    Ok(())
}

/// `QoS` that matches the topic's current publishers, as `ros2 topic echo` chooses it: reliable only
/// if all are reliable, transient local only if all are, and then `latched_depth` deep.
fn auto_qos(node: &r2r::Node, topic: &str, depth: usize, latched_depth: usize) -> QosProfile {
    let base = QosProfile::default().keep_last(depth);
    let Ok(pubs) = node.get_publishers_info_by_topic(topic, false) else {
        return base.best_effort();
    };
    if pubs.is_empty() {
        return base.best_effort();
    }
    let reliable = pubs
        .iter()
        .all(|p| p.qos_profile.reliability == r2r::qos::ReliabilityPolicy::Reliable);
    let latched = pubs
        .iter()
        .all(|p| p.qos_profile.durability == r2r::qos::DurabilityPolicy::TransientLocal);
    let q = if reliable {
        base.reliable()
    } else {
        base.best_effort()
    };
    if latched {
        q.transient_local().keep_last(latched_depth)
    } else {
        q.volatile()
    }
}

/// The gids of `publishers`, in order.
fn gids(publishers: &[Endpoint]) -> Vec<String> {
    let mut out: Vec<String> = publishers.iter().map(|p| p.gid.clone()).collect();
    out.sort();
    out
}

fn sorted(map: HashMap<String, Vec<String>>) -> NamesAndTypes {
    let mut v: NamesAndTypes = map.into_iter().collect();
    v.sort();
    v
}

fn full_name(namespace: &str, name: &str) -> String {
    if namespace.is_empty() || namespace == "/" {
        format!("/{name}")
    } else {
        format!("{}/{name}", namespace.trim_end_matches('/'))
    }
}

fn qos_info(q: &QosProfile) -> QosInfo {
    use r2r::qos::{
        DurabilityPolicy as D, HistoryPolicy as H, LivelinessPolicy as L, ReliabilityPolicy as R,
    };
    // The middleware says "none" with zero or with its very long infinity.
    let limit = |d: Duration| (!d.is_zero() && d.as_secs() < 1 << 31).then_some(d);
    QosInfo {
        reliability: match q.reliability {
            R::Reliable => crate::Reliability::Reliable,
            R::BestEffort => crate::Reliability::BestEffort,
            _ => crate::Reliability::SystemDefault,
        },
        durability: match q.durability {
            D::TransientLocal => crate::Durability::TransientLocal,
            D::Volatile => crate::Durability::Volatile,
            _ => crate::Durability::SystemDefault,
        },
        history: match q.history {
            H::KeepLast => crate::History::KeepLast,
            H::KeepAll => crate::History::KeepAll,
            _ => crate::History::SystemDefault,
        },
        depth: q.depth,
        deadline: limit(q.deadline),
        lifespan: limit(q.lifespan),
        liveliness: match q.liveliness {
            L::Automatic => crate::Liveliness::Automatic,
            L::ManualByTopic => crate::Liveliness::ManualByTopic,
            _ => crate::Liveliness::SystemDefault,
        },
        lease: limit(q.liveliness_lease_duration),
    }
}

fn endpoints(infos: Vec<r2r::TopicEndpointInfo>) -> Vec<Endpoint> {
    let mut out: Vec<Endpoint> = infos
        .into_iter()
        .map(|i| Endpoint {
            node: full_name(&i.node_namespace, &i.node_name),
            topic_type: i.topic_type,
            qos: qos_info(&i.qos_profile),
            gid: i.endpoint_gid[..16].iter().fold(String::new(), |mut s, b| {
                let _ = write!(s, "{b:02x}");
                s
            }),
        })
        .collect();
    out.sort_by(|a, b| a.node.cmp(&b.node));
    out
}

fn graph_detail(node: &r2r::Node) -> Result<GraphDetail, RosError> {
    let topics = node.get_topic_names_and_types().map_err(mw)?;
    let services = node.get_service_names_and_types().map_err(mw)?;
    let mut nodes: Vec<String> = node
        .get_node_names()
        .map_err(mw)?
        .into_iter()
        .map(|(name, ns)| full_name(&ns, &name))
        .collect();
    // Kept twice when two nodes share a name, as `ros2 node list` shows them: two stacks on one
    // graph look like that.
    nodes.sort();
    Ok(GraphDetail {
        topics: sorted(topics),
        services: sorted(services),
        nodes,
    })
}

fn node_entities(node: &r2r::Node, name: &str, ns: &str) -> Result<NodeEntities, RosError> {
    Ok(NodeEntities {
        publishers: sorted(
            node.get_publisher_names_and_types_by_node(name, ns)
                .map_err(mw)?,
        ),
        subscribers: sorted(
            node.get_subscriber_names_and_types_by_node(name, ns)
                .map_err(mw)?,
        ),
        services: sorted(
            node.get_service_names_and_types_by_node(name, ns)
                .map_err(mw)?,
        ),
        clients: sorted(
            node.get_client_names_and_types_by_node(name, ns)
                .map_err(mw)?,
        ),
    })
}

fn handle(node: &mut r2r::Node, runtime: &tokio::runtime::Handle, cmd: Cmd) {
    match cmd {
        Cmd::GraphDetail { reply } => {
            let _ = reply.send(graph_detail(node));
        }
        Cmd::Endpoints { topic, reply } => {
            let both = node
                .get_publishers_info_by_topic(&topic, false)
                .and_then(|p| Ok((p, node.get_subscriptions_info_by_topic(&topic, false)?)))
                .map(|(p, s)| TopicEndpoints {
                    publishers: endpoints(p),
                    subscribers: endpoints(s),
                })
                .map_err(mw);
            let _ = reply.send(both);
        }
        Cmd::NodeEntities {
            name,
            namespace,
            reply,
        } => {
            let _ = reply.send(node_entities(node, &name, &namespace));
        }
        Cmd::SubscribeRaw {
            topic,
            ty,
            hearing,
            reply,
        } => {
            let qos = match hearing {
                Hearing::AsOffered => auto_qos(node, &topic, 10, LATCHED_DEPTH),
                Hearing::Any => QosProfile::sensor_data(),
            };
            let stream = node
                .subscribe_raw(&topic, &ty, qos)
                .map(futures::StreamExt::boxed)
                .map_err(|e| type_error(&ty, &e));
            let _ = reply.send(stream);
        }
        Cmd::SubscribeSample { topic, ty, reply } => {
            let qos = auto_qos(node, &topic, 10, 10);
            let stream = node
                .subscribe_untyped(&topic, &ty, qos)
                .map(futures::StreamExt::boxed)
                .map_err(|e| type_error(&ty, &e));
            let _ = reply.send(stream);
        }
        Cmd::Publisher {
            topic,
            ty,
            keeps,
            reply,
        } => {
            // Our own QoS, never the caller's: a raw QoS could reach a DDS topic such as rt/lowcmd.
            let qos = match keeps {
                Keeps::Last => QosProfile::default()
                    .keep_last(1)
                    .reliable()
                    .transient_local(),
                Keeps::Nothing => QosProfile::default().keep_last(10).reliable().volatile(),
            };
            let _ = reply.send(node.create_publisher_untyped(&topic, &ty, qos));
        }
        Cmd::DestroyPublisher { publisher } => node.destroy_publisher_untyped(publisher),
        Cmd::Client { name, ty, reply } => {
            let _ =
                reply.send(node.create_client_untyped(&name, &ty, QosProfile::services_default()));
        }
        Cmd::ActionClient { name, ty, reply } => {
            let _ = reply.send(node.create_action_client_untyped(&name, &ty));
        }
        Cmd::Graph { reply } => {
            let _ = reply.send(node.get_topic_names_and_types());
        }
        Cmd::SubscribeJson {
            topic,
            ty,
            tx,
            undecodable,
            reply,
        } => {
            let _ = reply.send(subscribe_json(node, runtime, topic, &ty, (tx, undecodable)));
        }
        Cmd::SubscribeImage { topic, tx } => subscribe_image(node, runtime, &topic, tx),
    }
}

/// Subscribes to `topic` as JSON for [`R2rPort::latest`]; whether nothing published it yet, and
/// the gids of what did.
fn subscribe_json(
    node: &mut r2r::Node,
    runtime: &tokio::runtime::Handle,
    topic: String,
    ty: &str,
    (tx, undecodable): (watch::Sender<Option<Sample>>, Arc<Mutex<Option<String>>>),
) -> Result<(bool, Vec<String>), RosError> {
    let publishers = node
        .get_publishers_info_by_topic(&topic, false)
        .map(endpoints)
        .unwrap_or_default();
    let blind = publishers.is_empty();
    let qos = auto_qos(node, &topic, 1, 1);
    let mut stream = node
        .subscribe_untyped(&topic, ty, qos)
        .map_err(|e| type_error(ty, &e))?;
    // Ends, and so frees the subscription, once nobody reads the topic.
    runtime.spawn(async move {
        loop {
            tokio::select! {
                () = tx.closed() => break,
                msg = stream.next() => match msg {
                    Some(Ok(v)) => {
                        tx.send_replace(Some(Sample {
                            at: Instant::now(),
                            value: Arc::new(v),
                        }));
                    }
                    Some(Err(e)) => {
                        // Once per subscription, not at the topic's rate; a read says why.
                        let mut kept = lock(&undecodable);
                        if kept.is_none() {
                            tracing::warn!(%topic, error = %e, "could not decode a message");
                        }
                        *kept = Some(e.to_string());
                    }
                    None => break,
                },
            }
        }
    });
    Ok((blind, gids(&publishers)))
}

/// Subscribes to an image topic for [`R2rPort::frames`]; a failure drops `tx`, which the next
/// call notices.
fn subscribe_image(
    node: &mut r2r::Node,
    runtime: &tokio::runtime::Handle,
    topic: &str,
    tx: watch::Sender<Option<Arc<Frame>>>,
) {
    let qos = auto_qos(node, topic, 1, 1);
    match node.subscribe::<r2r::sensor_msgs::msg::Image>(topic, qos) {
        Ok(mut stream) => {
            runtime.spawn(async move {
                while let Some(img) = stream.next().await {
                    if tx.is_closed() {
                        break;
                    }
                    let stamp = f64::from(img.header.stamp.sec)
                        + f64::from(img.header.stamp.nanosec) * 1e-9;
                    tx.send_replace(Some(Arc::new(Frame {
                        stamp_s: stamp,
                        frame_id: img.header.frame_id,
                        width: img.width,
                        height: img.height,
                        encoding: img.encoding,
                        step: img.step,
                        is_bigendian: img.is_bigendian != 0,
                        data: Bytes::from(img.data),
                    })));
                }
            });
        }
        Err(e) => {
            tracing::warn!(%topic, error = %e, "image subscription failed");
            to_rosout(&format!("{topic}: image subscription failed: {e}"));
        }
    }
}

fn status(s: r2r::GoalStatus) -> GoalStatus {
    match s {
        r2r::GoalStatus::Succeeded => GoalStatus::Succeeded,
        r2r::GoalStatus::Aborted => GoalStatus::Aborted,
        r2r::GoalStatus::Canceled => GoalStatus::Canceled,
        _ => GoalStatus::Unknown,
    }
}

/// Waits for a server; `ready` is `r2r::Node::is_available(&client)`, whose trait r2r keeps private.
async fn available(
    ready: r2r::Result<impl std::future::Future<Output = r2r::Result<()>>>,
    name: &str,
    wait: Duration,
) -> Result<(), RosError> {
    let ready = ready.map_err(mw)?;
    match tokio::time::timeout(wait, ready).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(e)) => Err(mw(e)),
        Err(_) => Err(RosError::Unavailable(name.to_owned())),
    }
}

#[async_trait]
impl RobotPort for R2rPort {
    async fn call(
        &self,
        service: &str,
        ty: &str,
        request: Value,
        timeout: Duration,
    ) -> Result<Value, RosError> {
        // One budget for finding the server and its answer: a stop sized by it must not overrun.
        let deadline = tokio::time::Instant::now() + timeout;
        let client = self.client(service, ty).await?;
        available(r2r::Node::is_available(&*client), service, timeout).await?;
        let pending = client
            .request(request)
            .map_err(|e| conversion(service, e))?;
        match tokio::time::timeout_at(deadline, pending).await {
            Err(_) => Err(RosError::Timeout(service.to_owned())),
            Ok(Err(e)) => Err(mw(e)),
            Ok(Ok(Err(e))) => Err(conversion(service, e)),
            Ok(Ok(Ok(v))) => Ok(v),
        }
    }

    async fn send_goal(
        &self,
        action: &str,
        ty: &str,
        goal: Value,
        timeout: Duration,
    ) -> Result<Goal, RosError> {
        let deadline = tokio::time::Instant::now() + timeout;
        let client = self.action_client(action, ty).await?;
        available(r2r::Node::is_available(&client), action, timeout).await?;
        let pending = client
            .send_goal_request(goal)
            .map_err(|e| conversion(action, e))?;
        // The goal is out. Should the server accept it after the caller gave up (a timeout, a
        // stop, a dropped future), nobody would watch or cancel it: the wait is a task of its own,
        // which cancels such a goal, also one handed over but never taken.
        let (accepted_tx, accepted) = oneshot::channel();
        let name = action.to_owned();
        // Detached on purpose: it outlives a caller that gave up, which is what it is for.
        tokio::spawn(async move {
            let Ok(answer) = tokio::time::timeout(LATE_ACCEPT, pending).await else {
                tracing::error!(action = %name, "no answer to a goal after {} s: if it was accepted, it runs unwatched", LATE_ACCEPT.as_secs());
                to_rosout(&format!(
                    "{name}: no answer to a goal after {} s: if it was accepted, it runs unwatched",
                    LATE_ACCEPT.as_secs()
                ));
                return;
            };
            let goal = answer.as_ref().ok().map(|(goal, _, _)| goal.clone());
            let (taken_tx, taken) = oneshot::channel::<()>();
            match accepted_tx.send((answer, taken_tx)) {
                Err((Ok((late, _, _)), _)) => cancel_unwatched(late, &name).await,
                Err(_) => {}
                Ok(()) => {
                    if taken.await.is_err()
                        && let Some(goal) = goal
                    {
                        cancel_unwatched(goal, &name).await;
                    }
                }
            }
        });
        let (answer, taken) = match tokio::time::timeout_at(deadline, accepted).await {
            Err(_) => return Err(RosError::Timeout(action.to_owned())),
            Ok(Err(_)) => return Err(mw("the goal's answer was lost")),
            Ok(Ok(handoff)) => handoff,
        };
        let (handle, result, mut feedback) = match answer {
            Err(r2r::Error::RCL_RET_ACTION_GOAL_REJECTED) => {
                return Err(RosError::Rejected(action.to_owned()));
            }
            Err(e) => return Err(mw(e)),
            Ok(parts) => parts,
        };
        // Taken: from here the returned Goal is what watches and cancels it.
        let _ = taken.send(());
        let (fb_tx, fb_rx) = mpsc::channel(64);
        // r2r never ends a goal's feedback stream, so this ends when the goal's reader goes.
        tokio::spawn(async move {
            loop {
                tokio::select! {
                    () = fb_tx.closed() => break,
                    item = feedback.next() => match item {
                        Some(Ok(v)) => {
                            if fb_tx.send(v).await.is_err() {
                                break;
                            }
                        }
                        Some(Err(_)) => {}
                        None => break,
                    },
                }
            }
        });
        let (res_tx, res_rx) = oneshot::channel();
        let name = action.to_owned();
        tokio::spawn(async move {
            let out = match result.await {
                Ok((s, Ok(v))) => Ok(GoalResult {
                    status: status(s),
                    result: v,
                }),
                Ok((_, Err(e))) => Err(conversion(&name, e)),
                Err(e) => Err(mw(e)),
            };
            let _ = res_tx.send(out);
        });
        let id = handle.uuid;
        let name = action.to_owned();
        let cancel = Arc::new(move || {
            let pending = handle.cancel();
            let what = format!("cancel of {name} goal {id}");
            Box::pin(async move {
                match tokio::time::timeout(CANCEL_TIMEOUT, pending.map_err(mw)?).await {
                    Ok(answer) => answer.map_err(mw),
                    Err(_) => Err(RosError::Timeout(what)),
                }
            }) as futures::future::BoxFuture<'static, Result<(), RosError>>
        });
        Ok(Goal::new(id, fb_rx, res_rx, cancel))
    }

    async fn latest(&self, topic: &str, ty: &str, wait: Duration) -> Result<Value, RosError> {
        self.newest(topic, ty, wait, None)
            .await
            .map(|v| Value::clone(&v))
    }

    async fn latest_shared(
        &self,
        topic: &str,
        ty: &str,
        wait: Duration,
    ) -> Result<Arc<Value>, RosError> {
        self.newest(topic, ty, wait, None).await
    }

    async fn latest_fresh(
        &self,
        topic: &str,
        ty: &str,
        wait: Duration,
        max_age: Duration,
    ) -> Result<Value, RosError> {
        self.newest(topic, ty, wait, Some(max_age))
            .await
            .map(|v| Value::clone(&v))
    }

    async fn latest_fresh_shared(
        &self,
        topic: &str,
        ty: &str,
        wait: Duration,
        max_age: Duration,
    ) -> Result<Arc<Value>, RosError> {
        self.newest(topic, ty, wait, Some(max_age)).await
    }

    fn frames(&self, topic: &str) -> Result<watch::Receiver<Option<Arc<Frame>>>, RosError> {
        // A subscription that could not be made has dropped its sender: try it again.
        if let Some(rx) = lock(&self.images)
            .get(topic)
            .filter(|rx| rx.has_changed().is_ok())
        {
            return Ok(rx.clone());
        }
        let (tx, rx) = watch::channel(None);
        self.send(Cmd::SubscribeImage {
            topic: topic.to_owned(),
            tx,
        })?;
        lock(&self.images).insert(topic.to_owned(), rx.clone());
        Ok(rx)
    }

    fn ros_now_s(&self) -> f64 {
        // A clock rcl cannot read is the wall's, which is the ROS clock without simulated time.
        lock(&self.clock)
            .get_now()
            .map_or_else(|_| crate::now_s(), |d| d.as_secs_f64())
    }

    fn transform(&self, target: &str, source: &str) -> Result<Transform, RosError> {
        lock(&self.tf).lookup_within(target, source, TF_MAX_AGE)
    }

    async fn graph(&self) -> Result<Graph, RosError> {
        let map = self.ask(|reply| Cmd::Graph { reply }).await?.map_err(mw)?;
        let mut topics: Vec<(String, Vec<String>)> = map.into_iter().collect();
        topics.sort();
        Ok(Graph { topics })
    }

    async fn service_available(&self, service: &str, ty: &str, wait: Duration) -> bool {
        match self.client(service, ty).await {
            Ok(client) => available(r2r::Node::is_available(&*client), service, wait)
                .await
                .is_ok(),
            Err(_) => false,
        }
    }

    async fn graph_detail(&self) -> Result<GraphDetail, RosError> {
        self.ask(|reply| Cmd::GraphDetail { reply }).await?
    }

    async fn endpoints(&self, topic: &str) -> Result<TopicEndpoints, RosError> {
        self.ask(|reply| Cmd::Endpoints {
            topic: topic.to_owned(),
            reply,
        })
        .await?
    }

    async fn node_entities(&self, node: &str) -> Result<NodeEntities, RosError> {
        let (namespace, name) = match node.rsplit_once('/') {
            Some(("", name)) => ("/".to_owned(), name.to_owned()),
            Some((ns, name)) => (ns.to_owned(), name.to_owned()),
            None => ("/".to_owned(), node.to_owned()),
        };
        self.ask(|reply| Cmd::NodeEntities {
            name,
            namespace,
            reply,
        })
        .await?
    }

    async fn arrivals(&self, topic: &str, ty: &str) -> Result<BoxStream<'static, usize>, RosError> {
        let bytes = self
            .ask(|reply| Cmd::SubscribeRaw {
                topic: topic.to_owned(),
                ty: ty.to_owned(),
                hearing: Hearing::Any,
                reply,
            })
            .await??;
        Ok(bytes.map(|bytes| bytes.len()).boxed())
    }

    async fn raw(&self, topic: &str, ty: &str) -> Result<BoxStream<'static, Vec<u8>>, RosError> {
        // Before discovery the graph may not list a latched topic's publishers yet, and a
        // recording would ask it for no latched message.
        tokio::time::sleep(DISCOVERY.saturating_sub(self.started.elapsed())).await;
        let bytes = self
            .ask(|reply| Cmd::SubscribeRaw {
                topic: topic.to_owned(),
                ty: ty.to_owned(),
                hearing: Hearing::AsOffered,
                reply,
            })
            .await??;
        Ok(bytes.boxed())
    }

    async fn messages(
        &self,
        topic: &str,
        ty: &str,
    ) -> Result<BoxStream<'static, Result<Value, RosError>>, RosError> {
        let messages = self
            .ask(|reply| Cmd::SubscribeSample {
                topic: topic.to_owned(),
                ty: ty.to_owned(),
                reply,
            })
            .await??;
        let name = topic.to_owned();
        Ok(messages
            .map(move |m| m.map_err(|e| conversion(&name, e)))
            .boxed())
    }

    async fn publisher(&self, topic: &str, ty: &str) -> Result<Publisher, RosError> {
        self.kept_publisher(topic, ty, Keeps::Nothing).await
    }

    async fn latched_publisher(&self, topic: &str, ty: &str) -> Result<Publisher, RosError> {
        self.kept_publisher(topic, ty, Keeps::Last).await
    }

    async fn publish(
        &self,
        topic: &str,
        ty: &str,
        message: Value,
        count: usize,
        period: Duration,
    ) -> Result<usize, RosError> {
        let publisher = self
            .ask(|reply| Cmd::Publisher {
                topic: topic.to_owned(),
                ty: ty.to_owned(),
                keeps: Keeps::Nothing,
                reply,
            })
            .await?
            .map_err(|e| type_error(ty, &e))?;
        // Destroyed however the call ends, a stop of the turn included.
        let held = Destroy {
            cmd: &self.cmd,
            publisher: Some(publisher),
        };
        // Borrowed a statement at a time: a publisher may not be shared across threads, and the
        // waits below may move this call to another.
        // A new publisher is not matched at once; waiting a moment keeps the first message.
        let wait = held
            .publisher
            .as_ref()
            .and_then(|p| p.wait_for_inter_process_subscribers().ok());
        if let Some(wait) = wait {
            let _ = tokio::time::timeout(Duration::from_secs(2), wait).await;
        }
        let matched = held
            .publisher
            .as_ref()
            .and_then(|p| p.get_inter_process_subscription_count().ok())
            .unwrap_or(0);
        let mut result = Ok(matched);
        for i in 0..count {
            if i > 0 {
                tokio::time::sleep(period).await;
            }
            let sent = held.publisher.as_ref().map(|p| p.publish(message.clone()));
            if let Some(Err(e)) = sent {
                result = Err(conversion(topic, e));
                break;
            }
        }
        // Reliable delivery needs the publisher a moment longer.
        tokio::time::sleep(PUBLISH_GRACE).await;
        drop(held);
        result
    }

    fn tf_links(&self) -> Vec<TfLink> {
        lock(&self.tf).links()
    }
}
