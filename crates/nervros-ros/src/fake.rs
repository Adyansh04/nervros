//! A robot of scripts: services, actions, topics, frames and TF from closures and values, for
//! tests and for the robot a file describes (`scripted`).
//!
//! Every call and goal is recorded, so a test can assert what the agent asked the robot to do.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use futures::StreamExt as _;
use futures::stream::BoxStream;
use serde_json::Value;
use tokio::sync::{mpsc, oneshot, watch};

use crate::lock;
use crate::tf::TfBuffer;
use crate::{
    Frame, Goal, GoalResult, GoalStatus, Graph, GraphDetail, NodeEntities, Publisher, RobotPort,
    RosError, TfLink, TopicEndpoints, Transform,
};

/// How often a followed topic repeats its value.
const MESSAGE_EVERY: Duration = Duration::from_millis(100);

/// A scripted service: request in, response out.
pub type ServiceFn = Arc<dyn Fn(&Value) -> Result<Value, RosError> + Send + Sync>;

/// What a scripted goal does as it ends.
#[derive(Clone)]
pub struct OnEnd(pub Arc<dyn Fn() + Send + Sync>);

impl std::fmt::Debug for OnEnd {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("OnEnd")
    }
}

/// A scripted action run.
#[derive(Debug, Clone)]
pub struct ScriptedRun {
    /// Feedback messages, sent one per `step`.
    pub feedback: Vec<Value>,
    /// The final result.
    pub result: Result<GoalResult, RosError>,
    /// Time between feedback messages and before the result.
    pub step: Duration,
    /// Runs when the goal ends as scripted, never once it was cancelled: a walk arrives then.
    pub on_end: Option<OnEnd>,
    /// The server refuses to cancel it, as one that cannot stop.
    pub refuse_cancel: bool,
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
            on_end: None,
            refuse_cancel: false,
        }
    }
}

/// A scripted action: goal in, run out.
pub type ActionFn = Arc<dyn Fn(&Value) -> ScriptedRun + Send + Sync>;

/// Where a camera's frames come from.
type FrameSender = watch::Sender<Option<Arc<Frame>>>;

/// A robot made of scripts.
#[derive(Default)]
pub struct FakeRobot {
    services: HashMap<String, ServiceFn>,
    actions: HashMap<String, ActionFn>,
    topics: Mutex<HashMap<String, Value>>,
    /// Topics whose newest message is older than any freshness asked for.
    quiet: Mutex<std::collections::HashSet<String>>,
    /// When each topic's newest message was set.
    arrived: Mutex<HashMap<String, std::time::Instant>>,
    /// Goal cancels asked for.
    cancels: Arc<std::sync::atomic::AtomicUsize>,
    frames: HashMap<String, Arc<FrameSender>>,
    tf: Arc<Mutex<TfBuffer>>,
    latency: Duration,
    calls: Arc<Mutex<Vec<(String, Value)>>>,
    graph: Option<GraphDetail>,
    endpoints: HashMap<String, TopicEndpoints>,
    raw: HashMap<String, Vec<Vec<u8>>>,
    nodes: HashMap<String, NodeEntities>,
    rates: HashMap<String, (f64, usize)>,
    published: Arc<Mutex<Vec<(String, Value)>>>,
    /// Cameras it has no frames for, answered with none yet, as the real port does.
    idle_frames: Mutex<HashMap<String, FrameSender>>,
    /// The cameras' threads, joined when it drops.
    cameras: Vec<std::thread::JoinHandle<()>>,
    /// Header stamps set anew at each read, on the cameras' grid: see
    /// [`FakeRobot::with_live_stamps`].
    live_stamps: Option<Duration>,
}

/// `now` on a grid of `period`: cameras of one period stamp their frames alike, and a message
/// stamped on the grid at the same time is of the frame on screen.
fn on_grid(now: f64, period: Duration) -> f64 {
    let p = period.as_secs_f64();
    if p > 0.0 { (now / p).floor() * p } else { now }
}

impl Drop for FakeRobot {
    fn drop(&mut self) {
        // Each camera ends once its frames' sender is gone, within a period.
        self.frames.clear();
        for camera in self.cameras.drain(..) {
            let _ = camera.join();
        }
    }
}

impl std::fmt::Debug for FakeRobot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FakeRobot")
            .field("services", &self.services.keys().collect::<Vec<_>>())
            .field("actions", &self.actions.keys().collect::<Vec<_>>())
            .finish_non_exhaustive()
    }
}

impl FakeRobot {
    /// Refuses `ty` for `name` when its graph lists `name` with other types, as r2r refuses a type
    /// the name does not have.
    fn typed(&self, name: &str, ty: &str) -> Result<(), RosError> {
        let Some(graph) = &self.graph else {
            return Ok(());
        };
        // Actions are listed by their hidden topics, so an action's own name is never checked.
        let listed: Vec<&String> = graph
            .topics
            .iter()
            .chain(&graph.services)
            .filter(|(n, _)| n == name)
            .flat_map(|(_, types)| types)
            .collect();
        if listed.is_empty() || ty.is_empty() || listed.iter().any(|t| *t == ty) {
            Ok(())
        } else {
            Err(RosError::UnknownType(format!(
                "{name} is {}, not {ty}",
                listed
                    .iter()
                    .map(|t| t.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            )))
        }
    }

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
        self.frames.insert(topic.to_owned(), Arc::new(tx));
        self
    }

    /// Adds a transform: `child`'s pose in `parent`.
    #[must_use]
    pub fn with_transform(self, parent: &str, child: &str, t: Transform) -> Self {
        lock(&self.tf).insert(parent, child, t);
        self
    }

    /// Its transforms, for what moves it as it runs.
    pub(crate) fn tf(&self) -> Arc<Mutex<TfBuffer>> {
        Arc::clone(&self.tf)
    }

    /// Delays every call and goal acceptance.
    #[must_use]
    pub fn with_latency(mut self, latency: Duration) -> Self {
        self.latency = latency;
        self
    }

    /// Replaces a topic's newest message, just arrived.
    pub fn set_topic(&self, name: &str, value: Value) {
        lock(&self.quiet).remove(name);
        lock(&self.arrived).insert(name.to_owned(), std::time::Instant::now());
        lock(&self.topics).insert(name.to_owned(), value);
    }

    /// How many goal cancels were asked for.
    #[must_use]
    pub fn cancels(&self) -> usize {
        self.cancels.load(Ordering::SeqCst)
    }

    /// Ages a topic's newest message past any freshness asked for, as a publisher that died
    /// leaves it, until the topic is set again.
    pub fn go_quiet(&self, name: &str) {
        lock(&self.quiet).insert(name.to_owned());
    }

    /// Header stamps of the topics' messages set at each read to the newest time on the grid of
    /// `period`, the one cameras of that period stamp their frames on: a scripted message reads
    /// as one a live publisher keeps sending, and detections as of the frame on screen.
    #[must_use]
    pub fn with_live_stamps(mut self, period: Duration) -> Self {
        self.live_stamps = Some(period);
        self
    }

    /// Adds an image topic that sends `frame` every `period`, stamped anew on the grid of the
    /// period, as long as the robot lives: a camera that goes quiet reads as a stopped one.
    #[must_use]
    pub fn with_camera(mut self, topic: &str, mut frame: Frame, period: Duration) -> Self {
        frame.stamp_s = on_grid(crate::now_s(), period);
        let (tx, _) = watch::channel(Some(Arc::new(frame.clone())));
        let tx = Arc::new(tx);
        let sender = Arc::downgrade(&tx);
        let camera = std::thread::Builder::new()
            .name(format!("fake camera {topic}"))
            .spawn(move || {
                loop {
                    std::thread::sleep(period);
                    let Some(tx) = sender.upgrade() else { return };
                    frame.stamp_s = on_grid(crate::now_s(), period);
                    tx.send_replace(Some(Arc::new(frame.clone())));
                }
            });
        // Without its thread the camera keeps its first frame, which reads as a stopped one.
        if let Ok(camera) = camera {
            self.cameras.push(camera);
        }
        self.frames.insert(topic.to_owned(), tx);
        self
    }

    /// Sends a new frame on an image topic added with [`FakeRobot::with_frame`].
    pub fn set_frame(&self, topic: &str, frame: Frame) {
        if let Some(tx) = self.frames.get(topic) {
            tx.send_replace(Some(Arc::new(frame)));
        }
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

    /// What a raw subscription to a topic gets, then silence.
    #[must_use]
    pub fn with_raw(mut self, topic: &str, messages: Vec<Vec<u8>>) -> Self {
        self.raw.insert(topic.to_owned(), messages);
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
        ty: &str,
        request: Value,
        timeout: Duration,
    ) -> Result<Value, RosError> {
        self.typed(service, ty)?;
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
        ty: &str,
        goal: Value,
        timeout: Duration,
    ) -> Result<Goal, RosError> {
        self.typed(action, ty)?;
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
        // Set once the result is out: a cancel after it is refused, as a real server refuses one.
        let ended = Arc::new(AtomicBool::new(false));
        let done = Arc::clone(&ended);
        let refuse = run.refuse_cancel;
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
                if let Some(end) = &run.on_end {
                    (end.0)();
                }
                run.result
            };
            done.store(true, Ordering::SeqCst);
            let _ = res_tx.send(result);
        });
        let cancels = Arc::clone(&self.cancels);
        let cancel = Arc::new(move || {
            cancels.fetch_add(1, Ordering::SeqCst);
            let answer = if refuse {
                Err(RosError::Rejected(
                    "the server refused the cancel".to_owned(),
                ))
            } else if ended.load(Ordering::SeqCst) {
                Err(RosError::Rejected("the goal had ended".to_owned()))
            } else {
                cancelled.store(true, Ordering::SeqCst);
                Ok(())
            };
            Box::pin(async move { answer })
                as futures::future::BoxFuture<'static, Result<(), RosError>>
        });
        Ok(Goal::new(uuid::Uuid::new_v4(), fb_rx, res_rx, cancel))
    }

    async fn latest(&self, topic: &str, ty: &str, _wait: Duration) -> Result<Value, RosError> {
        self.typed(topic, ty)?;
        let mut msg = lock(&self.topics)
            .get(topic)
            .cloned()
            .ok_or_else(|| RosError::NoData(topic.to_owned()))?;
        if let Some(period) = self.live_stamps
            && msg["header"]["stamp"].is_object()
        {
            let at = on_grid(crate::now_s(), period);
            #[expect(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "seconds since 1970 and their fraction, both far in range"
            )]
            let (sec, nanosec) = (
                at.floor() as i64,
                ((at - at.floor()) * 1e9).round().min(999_999_999.0) as u32,
            );
            msg["header"]["stamp"] = serde_json::json!({"sec": sec, "nanosec": nanosec});
        }
        Ok(msg)
    }

    async fn latest_fresh(
        &self,
        topic: &str,
        ty: &str,
        wait: Duration,
        max_age: Duration,
    ) -> Result<Value, RosError> {
        // Answered at once, as `latest` is: what a script set is all there is to wait for.
        let stale = lock(&self.arrived)
            .get(topic)
            .is_some_and(|at| at.elapsed() > max_age);
        if stale || lock(&self.quiet).contains(topic) {
            return Err(RosError::NoData(topic.to_owned()));
        }
        self.latest(topic, ty, wait).await
    }

    fn frames(&self, topic: &str) -> Result<watch::Receiver<Option<Arc<Frame>>>, RosError> {
        if let Some(tx) = self.frames.get(topic) {
            return Ok(tx.subscribe());
        }
        Ok(lock(&self.idle_frames)
            .entry(topic.to_owned())
            .or_insert_with(|| watch::Sender::new(None))
            .subscribe())
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

    async fn raw(&self, topic: &str, _ty: &str) -> Result<BoxStream<'static, Vec<u8>>, RosError> {
        let messages = self.raw.get(topic).cloned().unwrap_or_default();
        Ok(futures::stream::iter(messages)
            .chain(futures::stream::pending())
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

    async fn latched_publisher(&self, topic: &str, ty: &str) -> Result<Publisher, RosError> {
        self.publisher(topic, ty).await
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
            ..ScriptedRun::default()
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
