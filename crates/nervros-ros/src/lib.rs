//! ROS 2 access for NervROS: calls by type string, action jobs, snapshots, images and TF behind one
//! trait.
//!
//! [`RobotPort`] is what the agent core talks to. [`R2rPort`] implements it over r2r (feature
//! `rcl`, which needs a sourced ROS environment), and [`fake::FakeRobot`] implements it from a
//! script for tests that run without ROS.

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use futures::future::BoxFuture;
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

    /// The newest message on a topic as JSON, waiting up to `wait` for the first one.
    async fn latest(&self, topic: &str, msg_type: &str, wait: Duration) -> Result<Value, RosError>;

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
}
