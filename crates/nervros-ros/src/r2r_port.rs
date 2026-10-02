//! [`RobotPort`] over r2r.
//!
//! One node lives on its own thread (`nervros-ros-spin`) and spins every few milliseconds. r2r
//! needs `&mut Node` to create clients and subscriptions, so the async side asks for them over a
//! command channel; requests, goals and streams then run on the tokio runtime and are resolved by
//! the spinning thread.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc as std_mpsc;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use async_trait::async_trait;
use bytes::Bytes;
use futures::StreamExt as _;
use futures::stream::BoxStream;
use r2r::QosProfile;
use serde_json::Value;
use tokio::sync::{mpsc, oneshot, watch};

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
        /// Whether the topic had no publisher, so that the `QoS` may miss a latched message.
        reply: oneshot::Sender<Result<bool, RosError>>,
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
        reply: oneshot::Sender<r2r::Result<r2r::PublisherUntyped>>,
    },
    DestroyPublisher {
        publisher: r2r::PublisherUntyped,
    },
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
}

/// A blind subscription that has had nothing this long is made again, with the `QoS` of the
/// publishers that may have come since.
const RESUBSCRIBE: Duration = Duration::from_secs(2);

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
}

impl std::fmt::Debug for R2rPort {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("R2rPort").finish_non_exhaustive()
    }
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
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
        let tf = Arc::new(Mutex::new(TfBuffer::default()));
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
        ready_rx.recv().map_err(mw)??;
        Ok(Self {
            cmd: cmd_tx,
            clients: Mutex::default(),
            actions: Mutex::default(),
            json: Mutex::default(),
            images: Mutex::default(),
            tf,
            stopping,
            thread: Some(thread),
        })
    }

    fn send(&self, cmd: Cmd) -> Result<(), RosError> {
        self.cmd
            .send(cmd)
            .map_err(|_| RosError::Middleware("the ROS node thread has stopped".to_owned()))
    }

    async fn client(&self, name: &str, ty: &str) -> Result<Arc<r2r::ClientUntyped>, RosError> {
        let key = (name.to_owned(), ty.to_owned());
        if let Some(c) = lock(&self.clients).get(&key) {
            return Ok(Arc::clone(c));
        }
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::Client {
            name: name.to_owned(),
            ty: ty.to_owned(),
            reply,
        })?;
        let client = Arc::new(rx.await.map_err(mw)?.map_err(|e| type_error(ty, &e))?);
        lock(&self.clients).insert(key, Arc::clone(&client));
        Ok(client)
    }

    /// The subscription for reading `topic` as JSON: kept from an earlier read of the same type,
    /// or made now, and made again when it began before anything published and still has nothing.
    async fn subscription(
        &self,
        topic: &str,
        ty: &str,
    ) -> Result<watch::Receiver<Option<Sample>>, RosError> {
        let kept = lock(&self.json)
            .get(topic)
            .filter(|t| {
                t.ty == ty
                    && !(t.blind && t.rx.borrow().is_none() && t.since.elapsed() >= RESUBSCRIBE)
            })
            .map(|t| t.rx.clone());
        if let Some(rx) = kept {
            return Ok(rx);
        }
        let (tx, rx) = watch::channel(None);
        let (reply, made) = oneshot::channel();
        self.send(Cmd::SubscribeJson {
            topic: topic.to_owned(),
            ty: ty.to_owned(),
            tx,
            reply,
        })?;
        let blind = made.await.map_err(mw)??;
        lock(&self.json).insert(
            topic.to_owned(),
            Topic {
                ty: ty.to_owned(),
                rx: rx.clone(),
                blind,
                since: Instant::now(),
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
    ) -> Result<Value, RosError> {
        let mut rx = self.subscription(topic, ty).await?;
        let fresh = |s: &Option<Sample>| {
            s.as_ref()
                .is_some_and(|s| max_age.is_none_or(|max| s.at.elapsed() <= max))
        };
        match tokio::time::timeout(wait, rx.wait_for(fresh)).await {
            Ok(Ok(s)) => s
                .as_ref()
                .map(|s| Value::clone(&s.value))
                .ok_or_else(|| RosError::NoData(topic.to_owned())),
            _ => Err(RosError::NoData(topic.to_owned())),
        }
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
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::ActionClient {
            name: name.to_owned(),
            ty: ty.to_owned(),
            reply,
        })?;
        let client = rx.await.map_err(mw)?.map_err(|e| type_error(ty, &e))?;
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
    ready: &std_mpsc::Sender<Result<(), RosError>>,
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
    if let Err(e) = listen_tf(&mut node, runtime, tf) {
        tracing::warn!(error = %e, "TF listener not started");
    }
    if ready.send(Ok(())).is_err() {
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
    let dynamic = node
        .subscribe::<r2r::tf2_msgs::msg::TFMessage>("/tf", QosProfile::default().keep_last(100))?;
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
/// if all are reliable, transient local only if all are.
fn auto_qos(node: &r2r::Node, topic: &str, depth: usize) -> QosProfile {
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
        q.transient_local()
    } else {
        q.volatile()
    }
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
    use r2r::qos::{DurabilityPolicy as D, HistoryPolicy as H, ReliabilityPolicy as R};
    let word = |s: &str| s.to_owned();
    QosInfo {
        reliability: word(match q.reliability {
            R::Reliable => "reliable",
            R::BestEffort => "best_effort",
            _ => "system_default",
        }),
        durability: word(match q.durability {
            D::TransientLocal => "transient_local",
            D::Volatile => "volatile",
            _ => "system_default",
        }),
        history: word(match q.history {
            H::KeepLast => "keep_last",
            H::KeepAll => "keep_all",
            _ => "system_default",
        }),
        depth: q.depth,
    }
}

fn endpoints(infos: Vec<r2r::TopicEndpointInfo>) -> Vec<Endpoint> {
    let mut out: Vec<Endpoint> = infos
        .into_iter()
        .map(|i| Endpoint {
            node: full_name(&i.node_namespace, &i.node_name),
            topic_type: i.topic_type,
            qos: qos_info(&i.qos_profile),
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
    nodes.sort();
    nodes.dedup();
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
        Cmd::SubscribeRaw { topic, ty, reply } => {
            // Best effort, as `ros2 topic hz` subscribes: it matches every publisher.
            let stream = node
                .subscribe_raw(&topic, &ty, QosProfile::sensor_data())
                .map(futures::StreamExt::boxed)
                .map_err(|e| type_error(&ty, &e));
            let _ = reply.send(stream);
        }
        Cmd::SubscribeSample { topic, ty, reply } => {
            let qos = auto_qos(node, &topic, 10);
            let stream = node
                .subscribe_untyped(&topic, &ty, qos)
                .map(futures::StreamExt::boxed)
                .map_err(|e| type_error(&ty, &e));
            let _ = reply.send(stream);
        }
        Cmd::Publisher { topic, ty, reply } => {
            // Our own QoS, never the caller's: a raw QoS could reach a DDS topic such as rt/lowcmd.
            let qos = QosProfile::default().keep_last(10).reliable().volatile();
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
            reply,
        } => {
            let _ = reply.send(subscribe_json(node, runtime, topic, &ty, tx));
        }
        Cmd::SubscribeImage { topic, tx } => subscribe_image(node, runtime, &topic, tx),
    }
}

/// Subscribes to `topic` as JSON for [`R2rPort::latest`]; whether nothing published it yet.
fn subscribe_json(
    node: &mut r2r::Node,
    runtime: &tokio::runtime::Handle,
    topic: String,
    ty: &str,
    tx: watch::Sender<Option<Sample>>,
) -> Result<bool, RosError> {
    let blind = node
        .get_publishers_info_by_topic(&topic, false)
        .map_or(true, |p| p.is_empty());
    let qos = auto_qos(node, &topic, 1);
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
                        tracing::warn!(%topic, error = %e, "could not decode a message");
                    }
                    None => break,
                },
            }
        }
    });
    Ok(blind)
}

/// Subscribes to an image topic for [`R2rPort::frames`]; a failure drops `tx`, which the next
/// call notices.
fn subscribe_image(
    node: &mut r2r::Node,
    runtime: &tokio::runtime::Handle,
    topic: &str,
    tx: watch::Sender<Option<Arc<Frame>>>,
) {
    let qos = auto_qos(node, topic, 1);
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
        Err(e) => tracing::warn!(%topic, error = %e, "image subscription failed"),
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
        let client = self.client(service, ty).await?;
        available(r2r::Node::is_available(&*client), service, timeout).await?;
        let pending = client.request(request).map_err(|e| RosError::Conversion {
            name: service.to_owned(),
            message: e.to_string(),
        })?;
        match tokio::time::timeout(timeout, pending).await {
            Err(_) => Err(RosError::Timeout(service.to_owned())),
            Ok(Err(e)) => Err(mw(e)),
            Ok(Ok(Err(e))) => Err(RosError::Conversion {
                name: service.to_owned(),
                message: e.to_string(),
            }),
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
        let client = self.action_client(action, ty).await?;
        available(r2r::Node::is_available(&client), action, timeout).await?;
        let pending = client
            .send_goal_request(goal)
            .map_err(|e| RosError::Conversion {
                name: action.to_owned(),
                message: e.to_string(),
            })?;
        // The goal is out. Should the server accept it after the caller gave up, on a timeout or
        // a stop, nobody would watch or cancel it: the wait is a task of its own, which cancels
        // such a goal.
        let (accepted_tx, accepted) = oneshot::channel();
        let name = action.to_owned();
        tokio::spawn(async move {
            if let Err(Ok((late, _, _))) = accepted_tx.send(pending.await) {
                tracing::warn!(action = %name, goal = %late.uuid, "accepted after its caller gave up; cancelling");
                if let Ok(cancel) = late.cancel() {
                    let _ = cancel.await;
                }
            }
        });
        let (handle, result, mut feedback) = match tokio::time::timeout(timeout, accepted).await {
            Err(_) => return Err(RosError::Timeout(action.to_owned())),
            Ok(Err(_)) => return Err(mw("the goal's answer was lost")),
            Ok(Ok(Err(r2r::Error::RCL_RET_ACTION_GOAL_REJECTED))) => {
                return Err(RosError::Rejected(action.to_owned()));
            }
            Ok(Ok(Err(e))) => return Err(mw(e)),
            Ok(Ok(Ok(parts))) => parts,
        };
        let (fb_tx, fb_rx) = mpsc::channel(64);
        tokio::spawn(async move {
            while let Some(item) = feedback.next().await {
                if let Ok(v) = item
                    && fb_tx.send(v).await.is_err()
                {
                    break;
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
                Ok((_, Err(e))) => Err(RosError::Conversion {
                    name,
                    message: e.to_string(),
                }),
                Err(e) => Err(mw(e)),
            };
            let _ = res_tx.send(out);
        });
        let id = handle.uuid;
        let cancel = Arc::new(move || {
            let pending = handle.cancel();
            Box::pin(async move { pending.map_err(mw)?.await.map_err(mw) })
                as futures::future::BoxFuture<'static, Result<(), RosError>>
        });
        Ok(Goal::new(id, fb_rx, res_rx, cancel))
    }

    async fn latest(&self, topic: &str, ty: &str, wait: Duration) -> Result<Value, RosError> {
        self.newest(topic, ty, wait, None).await
    }

    async fn latest_fresh(
        &self,
        topic: &str,
        ty: &str,
        wait: Duration,
        max_age: Duration,
    ) -> Result<Value, RosError> {
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

    fn transform(&self, target: &str, source: &str) -> Result<Transform, RosError> {
        lock(&self.tf).lookup(target, source)
    }

    async fn graph(&self) -> Result<Graph, RosError> {
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::Graph { reply })?;
        let map = rx.await.map_err(mw)?.map_err(mw)?;
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
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::GraphDetail { reply })?;
        rx.await.map_err(mw)?
    }

    async fn endpoints(&self, topic: &str) -> Result<TopicEndpoints, RosError> {
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::Endpoints {
            topic: topic.to_owned(),
            reply,
        })?;
        rx.await.map_err(mw)?
    }

    async fn node_entities(&self, node: &str) -> Result<NodeEntities, RosError> {
        let (namespace, name) = match node.rsplit_once('/') {
            Some(("", name)) => ("/".to_owned(), name.to_owned()),
            Some((ns, name)) => (ns.to_owned(), name.to_owned()),
            None => ("/".to_owned(), node.to_owned()),
        };
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::NodeEntities {
            name,
            namespace,
            reply,
        })?;
        rx.await.map_err(mw)?
    }

    async fn sample_sizes(
        &self,
        topic: &str,
        ty: &str,
        window: Duration,
        max: usize,
    ) -> Result<Vec<(Duration, usize)>, RosError> {
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::SubscribeRaw {
            topic: topic.to_owned(),
            ty: ty.to_owned(),
            reply,
        })?;
        let mut stream = rx.await.map_err(mw)??;
        let start = tokio::time::Instant::now();
        let mut out = Vec::new();
        while out.len() < max {
            match tokio::time::timeout_at(start + window, stream.next()).await {
                Ok(Some(bytes)) => out.push((start.elapsed(), bytes.len())),
                _ => break,
            }
        }
        Ok(out)
    }

    async fn sample_messages(
        &self,
        topic: &str,
        ty: &str,
        count: usize,
        timeout: Duration,
    ) -> Result<Vec<Value>, RosError> {
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::SubscribeSample {
            topic: topic.to_owned(),
            ty: ty.to_owned(),
            reply,
        })?;
        let mut stream = rx.await.map_err(mw)??;
        let deadline = tokio::time::Instant::now() + timeout;
        let mut out = Vec::new();
        while out.len() < count {
            match tokio::time::timeout_at(deadline, stream.next()).await {
                Ok(Some(Ok(v))) => out.push(v),
                Ok(Some(Err(e))) => {
                    return Err(RosError::Conversion {
                        name: topic.to_owned(),
                        message: e.to_string(),
                    });
                }
                _ => break,
            }
        }
        Ok(out)
    }

    async fn publisher(&self, topic: &str, ty: &str) -> Result<Publisher, RosError> {
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::Publisher {
            topic: topic.to_owned(),
            ty: ty.to_owned(),
            reply,
        })?;
        let publisher = rx.await.map_err(mw)?.map_err(|e| type_error(ty, &e))?;
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
            let _ = cmd.send(Cmd::DestroyPublisher { publisher });
        });
        Ok(handle)
    }

    async fn publish(
        &self,
        topic: &str,
        ty: &str,
        message: Value,
        count: usize,
        period: Duration,
    ) -> Result<usize, RosError> {
        let (reply, rx) = oneshot::channel();
        self.send(Cmd::Publisher {
            topic: topic.to_owned(),
            ty: ty.to_owned(),
            reply,
        })?;
        let publisher = rx.await.map_err(mw)?.map_err(|e| type_error(ty, &e))?;
        // A new publisher is not matched at once; waiting a moment keeps the first message.
        if let Ok(wait) = publisher.wait_for_inter_process_subscribers() {
            let _ = tokio::time::timeout(Duration::from_secs(2), wait).await;
        }
        let matched = publisher
            .get_inter_process_subscription_count()
            .unwrap_or(0);
        let mut result = Ok(matched);
        for i in 0..count {
            if i > 0 {
                tokio::time::sleep(period).await;
            }
            if let Err(e) = publisher.publish(message.clone()) {
                result = Err(RosError::Conversion {
                    name: topic.to_owned(),
                    message: e.to_string(),
                });
                break;
            }
        }
        // Reliable delivery needs the publisher a moment longer.
        tokio::time::sleep(Duration::from_millis(200)).await;
        self.send(Cmd::DestroyPublisher { publisher })?;
        result
    }

    fn tf_links(&self) -> Vec<TfLink> {
        lock(&self.tf).links()
    }
}
