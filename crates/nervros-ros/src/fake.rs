//! A scripted robot for tests: services, actions, topics, frames and TF from closures and values.
//!
//! Every call and goal is recorded, so a test can assert what the agent asked the robot to do.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use async_trait::async_trait;
use futures::StreamExt as _;
use futures::stream::BoxStream;
use serde_json::Value;
use tokio::sync::{mpsc, oneshot, watch};

use crate::tf::TfBuffer;
use crate::{
    Frame, Goal, GoalResult, GoalStatus, Graph, GraphDetail, NodeEntities, Publisher, RobotPort,
    RosError, TfLink, TopicEndpoints, Transform,
};

/// How often a followed topic repeats its value.
const MESSAGE_EVERY: Duration = Duration::from_millis(100);

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

impl Default for ScriptedRun {
    /// Succeeds at once with an empty result.
    fn default() -> Self {
        Self {
            feedback: Vec::new(),
            result: Ok(GoalResult {
                status: GoalStatus::Succeeded,
                result: Value::Object(serde_json::Map::new()),
            }),
            step: Duration::from_millis(10),
        }
    }
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
    graph: Option<GraphDetail>,
    endpoints: HashMap<String, TopicEndpoints>,
    nodes: HashMap<String, NodeEntities>,
    rates: HashMap<String, (f64, usize)>,
    published: Arc<Mutex<Vec<(String, Value)>>>,
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

    /// Scripts the graph: topics, services and nodes.
    #[must_use]
    pub fn with_graph(mut self, graph: GraphDetail) -> Self {
        self.graph = Some(graph);
        self
    }

    /// Scripts a topic's publishers and subscribers.
    #[must_use]
    pub fn with_endpoints(mut self, topic: &str, endpoints: TopicEndpoints) -> Self {
        self.endpoints.insert(topic.to_owned(), endpoints);
        self
    }

    /// Scripts what a node publishes, subscribes to, serves and calls.
    #[must_use]
    pub fn with_node(mut self, node: &str, entities: NodeEntities) -> Self {
        self.nodes.insert(node.to_owned(), entities);
        self
    }

    /// Makes a topic arrive at `hz`, each message `bytes` long.
    #[must_use]
    pub fn with_rate(mut self, topic: &str, hz: f64, bytes: usize) -> Self {
        self.rates.insert(topic.to_owned(), (hz, bytes));
        self
    }

    /// Every message published so far, as (topic, message).
    #[must_use]
    pub fn published(&self) -> Vec<(String, Value)> {
        lock(&self.published).clone()
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
        // A scripted graph knows the topics' types; otherwise every topic set has one, untyped.
        if let Some(graph) = &self.graph {
            return Ok(Graph {
                topics: graph.topics.clone(),
            });
        }
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

    async fn graph_detail(&self) -> Result<GraphDetail, RosError> {
        self.graph
            .clone()
            .ok_or_else(|| RosError::Unsupported("no scripted graph".to_owned()))
    }

    async fn endpoints(&self, topic: &str) -> Result<TopicEndpoints, RosError> {
        Ok(self.endpoints.get(topic).cloned().unwrap_or_default())
    }

    async fn node_entities(&self, node: &str) -> Result<NodeEntities, RosError> {
        Ok(self.nodes.get(node).cloned().unwrap_or_default())
    }

    async fn sample_sizes(
        &self,
        topic: &str,
        _ty: &str,
        window: Duration,
        max: usize,
    ) -> Result<Vec<(Duration, usize)>, RosError> {
        // Synthesised rather than waited for, so tests stay fast.
        let Some(&(hz, bytes)) = self.rates.get(topic) else {
            return Ok(Vec::new());
        };
        let step = Duration::from_secs_f64(1.0 / hz);
        Ok((1..)
            .map(|i| step * i)
            .take_while(|t| *t <= window)
            .take(max)
            .map(|t| (t, bytes))
            .collect())
    }

    async fn arrivals(
        &self,
        topic: &str,
        _ty: &str,
    ) -> Result<BoxStream<'static, usize>, RosError> {
        let Some(&(hz, bytes)) = self.rates.get(topic) else {
            return Ok(futures::stream::pending().boxed());
        };
        // Paced as the rate says, so a reader counts what it would on a robot.
        let step = Duration::from_secs_f64(1.0 / hz);
        Ok(futures::stream::repeat(bytes)
            .then(move |b| async move {
                tokio::time::sleep(step).await;
                b
            })
            .boxed())
    }

    async fn messages(
        &self,
        topic: &str,
        _ty: &str,
    ) -> Result<BoxStream<'static, Result<Value, RosError>>, RosError> {
        let Some(value) = lock(&self.topics).get(topic).cloned() else {
            return Ok(futures::stream::pending().boxed());
        };
        Ok(futures::stream::repeat(value)
            .then(|v| async move {
                tokio::time::sleep(MESSAGE_EVERY).await;
                Ok(v)
            })
            .boxed())
    }

    async fn sample_messages(
        &self,
        topic: &str,
        _ty: &str,
        count: usize,
        _timeout: Duration,
    ) -> Result<Vec<Value>, RosError> {
        Ok(lock(&self.topics)
            .get(topic)
            .map(|v| vec![v.clone(); count])
            .unwrap_or_default())
    }

    async fn publish(
        &self,
        topic: &str,
        _ty: &str,
        message: Value,
        count: usize,
        _period: Duration,
    ) -> Result<usize, RosError> {
        let mut log = lock(&self.published);
        for _ in 0..count {
            log.push((topic.to_owned(), message.clone()));
        }
        Ok(self.endpoints.get(topic).map_or(0, |e| e.subscribers.len()))
    }

    async fn publisher(&self, topic: &str, _ty: &str) -> Result<Publisher, RosError> {
        let (handle, mut messages) = Publisher::channel();
        let (log, topic) = (Arc::clone(&self.published), topic.to_owned());
        tokio::spawn(async move {
            while messages.changed().await.is_ok() {
                if let Some(m) = messages.borrow_and_update().clone() {
                    lock(&log).push((topic.clone(), m));
                }
            }
        });
        Ok(handle)
    }

    fn tf_links(&self) -> Vec<TfLink> {
        lock(&self.tf).links()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[tokio::test]
    async fn a_kept_publisher_sends_each_message_once_and_ends_with_its_handle() {
        let robot = FakeRobot::new();
        let publisher = robot.publisher("/beat", "x/msg/Y").await.unwrap();
        let settle = || tokio::time::sleep(Duration::from_millis(30));
        publisher.send(json!({"n": 1}));
        settle().await;
        publisher.send(json!({"n": 2}));
        settle().await;
        assert_eq!(
            robot.published(),
            [
                ("/beat".to_owned(), json!({"n": 1})),
                ("/beat".to_owned(), json!({"n": 2}))
            ]
        );
        settle().await;
        assert_eq!(
            robot.published().len(),
            2,
            "it repeated a message on its own"
        );
        drop(publisher);
    }

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
