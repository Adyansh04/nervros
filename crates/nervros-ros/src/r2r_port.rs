//! [`RobotPort`] over r2r.
//!
//! One node lives on its own thread (`nervros-ros-spin`) and spins every few milliseconds. r2r
//! needs `&mut Node` to create clients and subscriptions, so the async side asks for them over a
//! command channel; requests, goals and streams then run on the tokio runtime and are resolved by
//! the spinning thread.

use std::collections::HashMap;
use std::sync::mpsc as std_mpsc;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::Duration;

use async_trait::async_trait;
use bytes::Bytes;
use futures::StreamExt as _;
use r2r::QosProfile;
use serde_json::Value;
use tokio::sync::{mpsc, oneshot, watch};

use crate::tf::TfBuffer;
use crate::{Frame, Goal, GoalResult, GoalStatus, Graph, RobotPort, RosError, Transform};

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
        tx: watch::Sender<Option<Value>>,
    },
    SubscribeImage {
        topic: String,
        tx: watch::Sender<Option<Arc<Frame>>>,
    },
    Graph {
        reply: oneshot::Sender<r2r::Result<HashMap<String, Vec<String>>>>,
    },
}

type Key = (String, String);

/// A running r2r node.
pub struct R2rPort {
    cmd: std_mpsc::Sender<Cmd>,
    clients: Mutex<HashMap<Key, Arc<r2r::ClientUntyped>>>,
    actions: Mutex<HashMap<Key, r2r::ActionClientUntyped>>,
    json: Mutex<HashMap<String, watch::Receiver<Option<Value>>>>,
    images: Mutex<HashMap<String, watch::Receiver<Option<Arc<Frame>>>>>,
    tf: Arc<Mutex<TfBuffer>>,
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
        let thread = std::thread::Builder::new()
            .name("nervros-ros-spin".to_owned())
            .spawn(move || spin(&config, &runtime, &tf_thread, &cmd_rx, &ready_tx))
            .map_err(mw)?;
        ready_rx.recv().map_err(mw)??;
        Ok(Self {
            cmd: cmd_tx,
            clients: Mutex::default(),
            actions: Mutex::default(),
            json: Mutex::default(),
            images: Mutex::default(),
            tf,
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
        // Dropping the sender ends the spin loop; the thread exits within one spin period.
        let (tx, _) = std_mpsc::channel();
        drop(std::mem::replace(&mut self.cmd, tx));
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
    loop {
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
    for mut stream in [dynamic.boxed(), statics.boxed()] {
        let tf = Arc::clone(tf);
        runtime.spawn(async move {
            while let Some(msg) = stream.next().await {
                let mut buf = lock(&tf);
                for t in msg.transforms {
                    let (p, q) = (t.transform.translation, t.transform.rotation);
                    buf.insert(
                        &t.header.frame_id,
                        &t.child_frame_id,
                        Transform {
                            translation: [p.x, p.y, p.z],
                            rotation: [q.x, q.y, q.z, q.w],
                        },
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

fn handle(node: &mut r2r::Node, runtime: &tokio::runtime::Handle, cmd: Cmd) {
    match cmd {
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
        Cmd::SubscribeJson { topic, ty, tx } => {
            let qos = auto_qos(node, &topic, 1);
            match node.subscribe_untyped(&topic, &ty, qos) {
                Ok(mut stream) => {
                    runtime.spawn(async move {
                        while let Some(msg) = stream.next().await {
                            match msg {
                                Ok(v) => {
                                    tx.send_replace(Some(v));
                                }
                                Err(e) => {
                                    tracing::warn!(%topic, error = %e, "could not decode a message");
                                }
                            }
                        }
                    });
                }
                Err(e) => tracing::warn!(%topic, %ty, error = %e, "subscription failed"),
            }
        }
        Cmd::SubscribeImage { topic, tx } => {
            let qos = auto_qos(node, &topic, 1);
            match node.subscribe::<r2r::sensor_msgs::msg::Image>(&topic, qos) {
                Ok(mut stream) => {
                    runtime.spawn(async move {
                        while let Some(img) = stream.next().await {
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
        let (handle, result, mut feedback) = match tokio::time::timeout(timeout, pending).await {
            Err(_) => return Err(RosError::Timeout(action.to_owned())),
            Ok(Err(r2r::Error::RCL_RET_ACTION_GOAL_REJECTED)) => {
                return Err(RosError::Rejected(action.to_owned()));
            }
            Ok(Err(e)) => return Err(mw(e)),
            Ok(Ok(parts)) => parts,
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
        let existing = lock(&self.json).get(topic).cloned();
        let mut rx = if let Some(rx) = existing {
            rx
        } else {
            let (tx, rx) = watch::channel(None);
            self.send(Cmd::SubscribeJson {
                topic: topic.to_owned(),
                ty: ty.to_owned(),
                tx,
            })?;
            lock(&self.json).insert(topic.to_owned(), rx.clone());
            rx
        };
        match tokio::time::timeout(wait, rx.wait_for(Option::is_some)).await {
            Ok(Ok(v)) => v.clone().ok_or_else(|| RosError::NoData(topic.to_owned())),
            _ => Err(RosError::NoData(topic.to_owned())),
        }
    }

    fn frames(&self, topic: &str) -> Result<watch::Receiver<Option<Arc<Frame>>>, RosError> {
        if let Some(rx) = lock(&self.images).get(topic) {
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
}
