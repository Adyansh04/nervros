//! ROS 2 access for NervROS: calls by type string, action jobs, snapshots, images and TF behind one
//! trait.
//!
//! [`RobotPort`] is what the agent core talks to. [`R2rPort`] implements it over r2r (feature
//! `rcl`, which needs a sourced ROS environment), and [`fake::FakeRobot`] implements it from a
//! script for tests that run without ROS.

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use futures::StreamExt as _;
use futures::future::BoxFuture;
use futures::stream::BoxStream;
use serde_json::Value;
use tokio::sync::{mpsc, oneshot, watch};

pub mod fake;
pub mod image;
#[cfg(feature = "rcl")]
mod r2r_port;
pub mod tf;

#[cfg(feature = "rcl")]
pub use r2r_port::{R2rConfig, R2rPort};

pub use crate::image::Frame;
pub use crate::tf::Transform;

/// A ROS operation that failed.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum RosError {
    /// No server answered within the timeout.
    #[error("`{0}` is not available")]
    Unavailable(String),
    /// The call or goal did not finish within its timeout.
    #[error("`{0}` timed out")]
    Timeout(String),
    /// The type is not known to this build (r2r compiles types in; see `scripts/ros-env.sh`).
    #[error("unknown interface type `{0}`")]
    UnknownType(String),
    /// The server rejected the goal.
    #[error("`{0}` rejected the goal")]
    Rejected(String),
    /// A message could not be converted to or from JSON.
    #[error("`{name}`: {message}")]
    Conversion {
        /// The service, action or topic.
        name: String,
        /// What went wrong.
        message: String,
    },
    /// No data on a topic yet, or all of it too old.
    #[error("no fresh data on `{0}`")]
    NoData(String),
    /// A transform chain does not exist.
    #[error("no transform from `{source_frame}` to `{target}`")]
    NoTransform {
        /// The target frame.
        target: String,
        /// The source frame.
        source_frame: String,
    },
    /// The node is gone or another middleware error.
    #[error("ROS: {0}")]
    Middleware(String),
    /// This robot connection cannot do that.
    #[error("not supported here: {0}")]
    Unsupported(String),
}

/// How an action ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GoalStatus {
    /// The server reported success.
    Succeeded,
    /// The server gave up.
    Aborted,
    /// The goal was cancelled.
    Canceled,
    /// Any other final state.
    Unknown,
}

/// The final result of an action goal.
#[derive(Debug, Clone, PartialEq)]
pub struct GoalResult {
    /// How it ended.
    pub status: GoalStatus,
    /// The result message as JSON.
    pub result: Value,
}

/// A publisher [`RobotPort::publisher`] keeps for a message sent again and again, such as a
/// heartbeat or a held command: each message sent goes out once, the newest wins when they come
/// faster than they go, and the publisher goes when this handle is dropped. Nothing repeats on its
/// own, so a sender that stops shows as silence.
#[derive(Debug)]
pub struct Publisher {
    message: watch::Sender<Option<Value>>,
}

impl Publisher {
    /// A handle, and the end the publishing task follows.
    #[must_use]
    pub fn channel() -> (Self, watch::Receiver<Option<Value>>) {
        let (message, follow) = watch::channel(None);
        (Self { message }, follow)
    }

    /// Publishes `message` once.
    pub fn send(&self, message: Value) {
        self.message.send_replace(Some(message));
    }
}

/// Cancels a running goal.
pub type CancelFn = Arc<dyn Fn() -> BoxFuture<'static, Result<(), RosError>> + Send + Sync>;

/// A goal the server accepted.
pub struct Goal {
    /// The goal id.
    pub id: uuid::Uuid,
    /// Feedback messages as JSON, in order.
    pub feedback: mpsc::Receiver<Value>,
    /// The final result.
    pub result: oneshot::Receiver<Result<GoalResult, RosError>>,
    cancel: CancelFn,
}

impl std::fmt::Debug for Goal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Goal")
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}

impl Goal {
    /// A goal from its parts; used by [`RobotPort`] implementations.
    #[must_use]
    pub fn new(
        id: uuid::Uuid,
        feedback: mpsc::Receiver<Value>,
        result: oneshot::Receiver<Result<GoalResult, RosError>>,
        cancel: CancelFn,
    ) -> Self {
        Self {
            id,
            feedback,
            result,
            cancel,
        }
    }

    /// A handle that cancels this goal; it stays usable after the goal is moved.
    #[must_use]
    pub fn canceller(&self) -> CancelFn {
        Arc::clone(&self.cancel)
    }

    /// Asks the server to cancel the goal.
    ///
    /// # Errors
    ///
    /// The server refused or the node is gone.
    pub async fn cancel(&self) -> Result<(), RosError> {
        (self.cancel)().await
    }
}

/// Topics and their types, as the graph reports them.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Graph {
    /// Topic name to its types.
    pub topics: Vec<(String, Vec<String>)>,
}

/// Names and their types, sorted by name.
pub type NamesAndTypes = Vec<(String, Vec<String>)>;

/// The whole graph: topics, services and nodes.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct GraphDetail {
    /// Topics and their types.
    pub topics: NamesAndTypes,
    /// Services and their types.
    pub services: NamesAndTypes,
    /// Full node names, `/namespace/name`.
    pub nodes: Vec<String>,
}

/// The `QoS` an endpoint uses, as words.
#[derive(Debug, Clone, Default, PartialEq, Eq, serde::Serialize)]
pub struct QosInfo {
    /// `reliable` or `best_effort`.
    pub reliability: String,
    /// `volatile` or `transient_local`.
    pub durability: String,
    /// `keep_last` or `keep_all`.
    pub history: String,
    /// The queue depth for `keep_last`.
    pub depth: usize,
}

/// One publisher or subscriber of a topic.
#[derive(Debug, Clone, Default, PartialEq, Eq, serde::Serialize)]
pub struct Endpoint {
    /// Full node name.
    pub node: String,
    /// The type it uses.
    pub topic_type: String,
    /// Its `QoS`.
    pub qos: QosInfo,
}

/// Who publishes and who subscribes to a topic.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TopicEndpoints {
    /// Its publishers.
    pub publishers: Vec<Endpoint>,
    /// Its subscribers.
    pub subscribers: Vec<Endpoint>,
}

/// What a node publishes, subscribes to, serves and calls.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct NodeEntities {
    /// Topics it publishes.
    pub publishers: NamesAndTypes,
    /// Topics it subscribes to.
    pub subscribers: NamesAndTypes,
    /// Services it offers.
    pub services: NamesAndTypes,
    /// Services it calls.
    pub clients: NamesAndTypes,
}

/// One frame link in TF.
#[derive(Debug, Clone, PartialEq)]
pub struct TfLink {
    /// The parent frame.
    pub parent: String,
    /// The child frame.
    pub child: String,
    /// From `/tf_static`.
    pub is_static: bool,
    /// Since the newest message for this link arrived.
    pub age: Duration,
}

fn unsupported<T>(what: &str) -> Result<T, RosError> {
    Err(RosError::Unsupported(what.to_owned()))
}

/// Everything the agent core needs from a robot.
#[async_trait]
pub trait RobotPort: Send + Sync {
    /// Calls a service by name and type string (`pkg/srv/Name`) with a JSON request.
    async fn call(
        &self,
        service: &str,
        service_type: &str,
        request: Value,
        timeout: Duration,
    ) -> Result<Value, RosError>;

    /// Sends an action goal by name and type string (`pkg/action/Name`).
    async fn send_goal(
        &self,
        action: &str,
        action_type: &str,
        goal: Value,
        timeout: Duration,
    ) -> Result<Goal, RosError>;

    /// The newest message on a topic as JSON, waiting up to `wait` for the first one, however
    /// old it is: a latched map is as good as ever.
    async fn latest(&self, topic: &str, msg_type: &str, wait: Duration) -> Result<Value, RosError>;

    /// As [`RobotPort::latest`], shared rather than copied: a reader that polls a large message,
    /// such as a map, tells a new one by pointer and copies nothing.
    async fn latest_shared(
        &self,
        topic: &str,
        msg_type: &str,
        wait: Duration,
    ) -> Result<Arc<Value>, RosError> {
        self.latest(topic, msg_type, wait).await.map(Arc::new)
    }

    /// As [`RobotPort::latest`], but only a message that arrived within `max_age`, waiting up to
    /// `wait` for one: what a publisher said before it went quiet is not its state now.
    /// A port that keeps no arrival times answers as `latest` does.
    async fn latest_fresh(
        &self,
        topic: &str,
        msg_type: &str,
        wait: Duration,
        _max_age: Duration,
    ) -> Result<Value, RosError> {
        self.latest(topic, msg_type, wait).await
    }

    /// The newest frame of an image topic (`sensor_msgs/msg/Image`), kept up to date.
    ///
    /// # Errors
    ///
    /// The subscription could not be created.
    fn frames(&self, topic: &str) -> Result<watch::Receiver<Option<Arc<Frame>>>, RosError>;

    /// The transform that maps points in `source` into `target`, from the newest TF data.
    ///
    /// # Errors
    ///
    /// [`RosError::NoTransform`] when no chain links the frames.
    fn transform(&self, target: &str, source: &str) -> Result<Transform, RosError>;

    /// The topics in the graph.
    async fn graph(&self) -> Result<Graph, RosError>;

    /// Whether a service server is reachable within `wait`.
    async fn service_available(&self, service: &str, service_type: &str, wait: Duration) -> bool;

    /// Topics, services and nodes.
    async fn graph_detail(&self) -> Result<GraphDetail, RosError> {
        unsupported("listing services and nodes")
    }

    /// A topic's publishers and subscribers.
    async fn endpoints(&self, _topic: &str) -> Result<TopicEndpoints, RosError> {
        unsupported("listing a topic's endpoints")
    }

    /// What a node (`/namespace/name`) publishes, subscribes to, serves and calls.
    async fn node_entities(&self, _node: &str) -> Result<NodeEntities, RosError> {
        unsupported("listing a node's topics and services")
    }

    /// The serialized size of each message on a topic from now until the stream is dropped,
    /// subscribing best effort as `ros2 topic hz` does: one subscription for a reader that keeps
    /// counting.
    async fn arrivals(
        &self,
        _topic: &str,
        _msg_type: &str,
    ) -> Result<BoxStream<'static, usize>, RosError> {
        unsupported("following a topic")
    }

    /// Each message on a topic from now until the stream is dropped, as JSON.
    async fn messages(
        &self,
        _topic: &str,
        _msg_type: &str,
    ) -> Result<BoxStream<'static, Result<Value, RosError>>, RosError> {
        unsupported("following a topic")
    }

    /// Arrival times since the start and serialized sizes of a topic's messages over `window`,
    /// at most `max` of them.
    async fn sample_sizes(
        &self,
        topic: &str,
        msg_type: &str,
        window: Duration,
        max: usize,
    ) -> Result<Vec<(Duration, usize)>, RosError> {
        let mut sizes = self.arrivals(topic, msg_type).await?;
        let start = tokio::time::Instant::now();
        let mut out = Vec::new();
        while out.len() < max {
            match tokio::time::timeout_at(start + window, sizes.next()).await {
                Ok(Some(size)) => out.push((start.elapsed(), size)),
                _ => break,
            }
        }
        Ok(out)
    }

    /// Up to `count` messages from a new subscription, as JSON, waiting up to `timeout`.
    async fn sample_messages(
        &self,
        topic: &str,
        msg_type: &str,
        count: usize,
        timeout: Duration,
    ) -> Result<Vec<Value>, RosError> {
        let mut messages = self.messages(topic, msg_type).await?;
        let deadline = tokio::time::Instant::now() + timeout;
        let mut out = Vec::new();
        while out.len() < count {
            match tokio::time::timeout_at(deadline, messages.next()).await {
                Ok(Some(message)) => out.push(message?),
                _ => break,
            }
        }
        Ok(out)
    }

    /// Publishes `message` `count` times, `period` apart, and returns how many subscribers were
    /// matched when it started.
    async fn publish(
        &self,
        _topic: &str,
        _msg_type: &str,
        _message: Value,
        _count: usize,
        _period: Duration,
    ) -> Result<usize, RosError> {
        unsupported("publishing")
    }

    /// A publisher kept for messages sent again and again, where one per message would pay
    /// discovery every time.
    async fn publisher(&self, _topic: &str, _msg_type: &str) -> Result<Publisher, RosError> {
        unsupported("keeping a publisher")
    }

    /// Every frame link TF has seen.
    fn tf_links(&self) -> Vec<TfLink> {
        Vec::new()
    }
}
