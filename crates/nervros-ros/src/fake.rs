//! A scripted robot for tests: services, actions, topics, frames and TF from closures and values.
//!
//! Every call and goal is recorded, so a test can assert what the agent asked the robot to do.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use async_trait::async_trait;
use serde_json::Value;
use tokio::sync::{mpsc, oneshot, watch};

use crate::tf::TfBuffer;
use crate::{Frame, Goal, GoalResult, GoalStatus, Graph, RobotPort, RosError, Transform};

/// A scripted service: request in, response out.
pub type ServiceFn = Arc<dyn Fn(&Value) -> Result<Value, RosError> + Send + Sync>;

/// A scripted action run.
#[derive(Debug, Clone)]
pub struct ScriptedRun {
    /// Feedback messages, sent one per `step`.
    pub feedback: Vec<Value>,
    /// The final result.
    pub result: Result<GoalResult, RosError>,
    /// Time between feedback messages and before the result.
    pub step: Duration,
}

/// A scripted action: goal in, run out.
pub type ActionFn = Arc<dyn Fn(&Value) -> ScriptedRun + Send + Sync>;

/// A robot made of scripts.
#[derive(Default)]
pub struct FakeRobot {
    services: HashMap<String, ServiceFn>,
    actions: HashMap<String, ActionFn>,
    topics: Mutex<HashMap<String, Value>>,
    frames: HashMap<String, watch::Sender<Option<Arc<Frame>>>>,
    tf: Mutex<TfBuffer>,
    latency: Duration,
    calls: Arc<Mutex<Vec<(String, Value)>>>,
}

impl std::fmt::Debug for FakeRobot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FakeRobot")
            .field("services", &self.services.keys().collect::<Vec<_>>())
            .field("actions", &self.actions.keys().collect::<Vec<_>>())
            .finish_non_exhaustive()
    }
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

impl FakeRobot {
    /// An empty robot.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a service.
    #[must_use]
    pub fn with_service(
        mut self,
        name: &str,
        f: impl Fn(&Value) -> Result<Value, RosError> + Send + Sync + 'static,
    ) -> Self {
        self.services.insert(name.to_owned(), Arc::new(f));
        self
    }

    /// Adds an action.
    #[must_use]
    pub fn with_action(
        mut self,
        name: &str,
        f: impl Fn(&Value) -> ScriptedRun + Send + Sync + 'static,
    ) -> Self {
        self.actions.insert(name.to_owned(), Arc::new(f));
        self
    }

    /// Sets a topic's newest message.
    #[must_use]
    pub fn with_topic(self, name: &str, value: Value) -> Self {
        self.set_topic(name, value);
        self
    }

    /// Adds an image topic with one frame.
    #[must_use]
    pub fn with_frame(mut self, topic: &str, frame: Frame) -> Self {
        let (tx, _) = watch::channel(Some(Arc::new(frame)));
        self.frames.insert(topic.to_owned(), tx);
        self
    }

    /// Adds a transform: `child`'s pose in `parent`.
    #[must_use]
    pub fn with_transform(self, parent: &str, child: &str, t: Transform) -> Self {
        lock(&self.tf).insert(parent, child, t);
        self
    }

    /// Delays every call and goal acceptance.
    #[must_use]
    pub fn with_latency(mut self, latency: Duration) -> Self {
        self.latency = latency;
        self
    }

    /// Replaces a topic's newest message.
    pub fn set_topic(&self, name: &str, value: Value) {
        lock(&self.topics).insert(name.to_owned(), value);
    }

    /// Every call and goal so far, as (name, request or goal).
    #[must_use]
    pub fn calls(&self) -> Vec<(String, Value)> {
        lock(&self.calls).clone()
    }
}

#[async_trait]
impl RobotPort for FakeRobot {
    async fn call(
        &self,
        service: &str,
        _ty: &str,
        request: Value,
        timeout: Duration,
    ) -> Result<Value, RosError> {
        let f = self
            .services
            .get(service)
            .cloned()
            .ok_or_else(|| RosError::Unavailable(service.to_owned()))?;
        lock(&self.calls).push((service.to_owned(), request.clone()));
        if self.latency > timeout {
            return Err(RosError::Timeout(service.to_owned()));
        }
        tokio::time::sleep(self.latency).await;
        f(&request)
    }

    async fn send_goal(
        &self,
        action: &str,
        _ty: &str,
        goal: Value,
        timeout: Duration,
    ) -> Result<Goal, RosError> {
        let f = self
            .actions
            .get(action)
            .cloned()
            .ok_or_else(|| RosError::Unavailable(action.to_owned()))?;
        lock(&self.calls).push((action.to_owned(), goal.clone()));
        if self.latency > timeout {
            return Err(RosError::Timeout(action.to_owned()));
        }
        tokio::time::sleep(self.latency).await;
        let run = f(&goal);
        let (fb_tx, fb_rx) = mpsc::channel(32);
        let (res_tx, res_rx) = oneshot::channel();
        let cancelled = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&cancelled);
        tokio::spawn(async move {
            for item in run.feedback {
                tokio::time::sleep(run.step).await;
                if flag.load(Ordering::SeqCst) {
                    break;
                }
                // A dropped receiver only means nobody watches the feedback.
                let _ = fb_tx.send(item).await;
            }
            tokio::time::sleep(run.step).await;
            let result = if flag.load(Ordering::SeqCst) {
                Ok(GoalResult {
                    status: GoalStatus::Canceled,
                    result: Value::Null,
                })
            } else {
                run.result
            };
            let _ = res_tx.send(result);
        });
        let cancel = Arc::new(move || {
            cancelled.store(true, Ordering::SeqCst);
            Box::pin(async { Ok(()) }) as futures::future::BoxFuture<'static, Result<(), RosError>>
        });
        Ok(Goal::new(uuid::Uuid::new_v4(), fb_rx, res_rx, cancel))
    }

    async fn latest(&self, topic: &str, _ty: &str, _wait: Duration) -> Result<Value, RosError> {
        lock(&self.topics)
            .get(topic)
            .cloned()
            .ok_or_else(|| RosError::NoData(topic.to_owned()))
    }

    fn frames(&self, topic: &str) -> Result<watch::Receiver<Option<Arc<Frame>>>, RosError> {
        self.frames
            .get(topic)
            .map(watch::Sender::subscribe)
            .ok_or_else(|| RosError::NoData(topic.to_owned()))
    }

    fn transform(&self, target: &str, source: &str) -> Result<Transform, RosError> {
        lock(&self.tf).lookup(target, source)
    }

    async fn graph(&self) -> Result<Graph, RosError> {
        let mut topics: Vec<(String, Vec<String>)> = lock(&self.topics)
            .keys()
            .map(|k| (k.clone(), Vec::new()))
            .collect();
        topics.sort();
        Ok(Graph { topics })
    }

    async fn service_available(&self, service: &str, _ty: &str, _wait: Duration) -> bool {
        self.services.contains_key(service)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[tokio::test]
    async fn services_record_calls_and_answer() {
        let robot = FakeRobot::new().with_service("/find", |req| Ok(json!({"echo": req["q"]})));
        let out = robot
            .call(
                "/find",
                "x/srv/Y",
                json!({"q": "cup"}),
                Duration::from_secs(1),
            )
            .await
            .unwrap();
        assert_eq!(out, json!({"echo": "cup"}));
        assert_eq!(robot.calls(), [("/find".to_owned(), json!({"q": "cup"}))]);
        assert!(matches!(
            robot
                .call("/nope", "x/srv/Y", json!({}), Duration::from_secs(1))
                .await,
            Err(RosError::Unavailable(_))
        ));
    }

    #[tokio::test]
    async fn actions_stream_feedback_and_can_be_cancelled() {
        let run = |_: &Value| ScriptedRun {
            feedback: vec![json!({"left": 2.0}), json!({"left": 1.0})],
            result: Ok(GoalResult {
                status: GoalStatus::Succeeded,
                result: json!({}),
            }),
            step: Duration::from_millis(5),
        };
        let robot = FakeRobot::new().with_action("/go", run);
        let mut goal = robot
            .send_goal("/go", "x/action/Go", json!({}), Duration::from_secs(1))
            .await
            .unwrap();
        assert_eq!(goal.feedback.recv().await, Some(json!({"left": 2.0})));
        let done = goal.result.await.unwrap().unwrap();
        assert_eq!(done.status, GoalStatus::Succeeded);

        let goal = robot
            .send_goal("/go", "x/action/Go", json!({}), Duration::from_secs(1))
            .await
            .unwrap();
        goal.cancel().await.unwrap();
        assert_eq!(
            goal.result.await.unwrap().unwrap().status,
            GoalStatus::Canceled
        );
    }
}
