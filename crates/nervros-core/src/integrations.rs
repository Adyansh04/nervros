//! Integrations: what NervROS knows of a ROS stack the robot runs, such as Nav2 or MoveIt.
//!
//! Each is switched on by its profile section. It finds its stack on the graph and says what is
//! missing, adds the tools its stack can serve and a line on its state to each turn, and cancels
//! what it sent when the robot is stopped. Nothing outside this module names a stack.

pub mod moveit;
pub mod nav2;

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::RobotPort;
use serde_json::{Value, json};

use crate::doctor::Check;
use crate::places::Places;
use crate::profile::Profile;
use crate::tools::Tool;
use crate::zones::Zones;

/// How long a parameter or a world-model read may take.
const READ_WAIT: Duration = Duration::from_secs(3);

/// A ROS stack NervROS works with.
#[async_trait]
pub trait Integration: Send + Sync + std::fmt::Debug {
    /// Its name, as its profile section has it: `nav2`.
    fn id(&self) -> &'static str;

    /// The tools it adds.
    fn tools(&self) -> Vec<Arc<dyn Tool>>;

    /// What it last saw of its stack, for each turn and the window: a headline such as
    /// "Nav2: up", then details; empty when it has nothing to say.
    fn status(&self) -> Vec<String>;

    /// Cancels every goal it sent that still runs.
    async fn stop(&self);

    /// Where it reports what it draws; once the session has started.
    fn attach(&self, _session: &crate::session::SessionHandle) {}

    /// The motions it runs itself as mission steps.
    fn natives(&self) -> Vec<Arc<dyn NativeSkill>> {
        Vec::new()
    }

    /// The named pose each arm group of its stack is at, for goal checks.
    async fn arm_poses(&self) -> std::collections::BTreeMap<String, String> {
        std::collections::BTreeMap::new()
    }
}

/// A motion a stack runs itself as a mission step: its catalog entry, and how it runs. A skill of
/// the same name in the robot's executor is used instead.
#[async_trait]
pub trait NativeSkill: Send + Sync {
    /// Its catalog entry, as an executor's `GetCatalog` writes one (name, description, args,
    /// outputs, resources, `max_duration_s`, template).
    fn skill(&self) -> Value;

    /// The actions it sends goals to, which must be on the graph for a plan with it to pass.
    fn needs(&self) -> Vec<String> {
        Vec::new()
    }

    /// What makes these arguments wrong now, before the operator is asked to approve them, such as
    /// opening a hand that holds something.
    async fn check(&self, _args: &std::collections::BTreeMap<String, String>) -> Option<String> {
        None
    }

    /// Where it would take the base, for the approval card, as a `StepPreview` without its ids;
    /// none when that is not known before it runs.
    async fn preview(&self, _args: &std::collections::BTreeMap<String, String>) -> Option<Value> {
        None
    }

    /// Runs it with its arguments by name; what it hands on by output name, or why it failed.
    /// It ends early when `ctl` says stop, and holds the robot still while `ctl` says paused.
    async fn run(
        &self,
        args: &std::collections::BTreeMap<String, String>,
        ctl: &Control,
    ) -> Result<std::collections::BTreeMap<String, String>, String>;
}

/// What a running native step listens to.
#[derive(Debug, Clone)]
pub struct Control {
    /// Turns true when the mission is stopped or cancelled.
    pub stop: tokio::sync::watch::Receiver<bool>,
    /// True while the mission is paused.
    pub paused: tokio::sync::watch::Receiver<bool>,
}

impl Control {
    /// Whether the mission was stopped.
    #[must_use]
    pub fn stopped(&self) -> bool {
        *self.stop.borrow()
    }

    /// Waits until the mission is stopped.
    pub async fn until_stopped(&self) {
        let mut stop = self.stop.clone();
        let _ = stop.wait_for(|s| *s).await;
    }
}

/// The goals the stacks' own motions have open: Stop cancels them all, and a watchdog on the robot
/// is told of them so it can cancel them if NervROS goes silent.
#[derive(Default)]
pub struct Goals {
    open: Mutex<std::collections::BTreeMap<uuid::Uuid, (String, nervros_ros::CancelFn)>>,
    changed: Arc<tokio::sync::Notify>,
}

/// How often the open goals are sent again while any is open.
const OPEN_GOALS_EVERY: Duration = Duration::from_millis(500);

impl std::fmt::Debug for Goals {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let open: Vec<String> = crate::lock(&self.open)
            .values()
            .map(|(a, _)| a.clone())
            .collect();
        f.debug_struct("Goals")
            .field("open", &open)
            .finish_non_exhaustive()
    }
}

impl Goals {
    /// Goals sent to a watchdog on `topic`, if any: at each change, and again and again while any
    /// is open, so their silence says NervROS has gone.
    pub(crate) fn published(robot: Arc<dyn RobotPort>, topic: Option<String>) -> Arc<Self> {
        let goals = Arc::new(Self::default());
        let Some(topic) = topic else {
            return goals;
        };
        let (weak, changed) = (Arc::downgrade(&goals), Arc::clone(&goals.changed));
        tokio::spawn(async move {
            let publisher = match robot
                .latched_publisher(&topic, "nervros_interfaces/msg/OpenGoals")
                .await
            {
                Ok(p) => p,
                Err(e) => {
                    tracing::warn!(%topic, error = %e, "no watchdog will hear of NervROS's goals");
                    return;
                }
            };
            while let Some(goals) = weak.upgrade() {
                let open = goals.open();
                drop(goals);
                publisher.send(open_goals(&open));
                if open.is_empty() {
                    changed.notified().await;
                } else {
                    let _ = tokio::time::timeout(OPEN_GOALS_EVERY, changed.notified()).await;
                }
            }
        });
        goals
    }

    /// Keeps `goal` of `action` until the returned guard drops.
    pub fn track(self: &Arc<Self>, action: &str, goal: &nervros_ros::Goal) -> Tracked {
        crate::lock(&self.open).insert(goal.id, (action.to_owned(), goal.canceller()));
        self.changed.notify_one();
        Tracked {
            goals: Arc::clone(self),
            id: goal.id,
        }
    }

    /// The open goals: action and id.
    #[must_use]
    pub fn open(&self) -> Vec<(String, uuid::Uuid)> {
        crate::lock(&self.open)
            .iter()
            .map(|(id, (action, _))| (action.clone(), *id))
            .collect()
    }

    /// Cancels every open goal.
    pub async fn cancel_all(&self) {
        let cancels: Vec<nervros_ros::CancelFn> = crate::lock(&self.open)
            .values()
            .map(|(_, c)| Arc::clone(c))
            .collect();
        futures::future::join_all(cancels.iter().map(|c| c())).await;
    }
}

/// A goal [`Goals`] keeps while it runs.
#[derive(Debug)]
pub struct Tracked {
    goals: Arc<Goals>,
    id: uuid::Uuid,
}

impl Drop for Tracked {
    fn drop(&mut self) {
        crate::lock(&self.goals.open).remove(&self.id);
        self.goals.changed.notify_one();
    }
}

/// An `OpenGoals` message of `open`.
fn open_goals(open: &[(String, uuid::Uuid)]) -> Value {
    let since = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default();
    json!({
        "stamp": {"sec": since.as_secs(), "nanosec": since.subsec_nanos()},
        "client": crate::mission::client_id(),
        "actions": open.iter().map(|(a, _)| a).collect::<Vec<_>>(),
        "goal_ids": open.iter().map(|(_, id)| json!({"uuid": id.as_bytes()})).collect::<Vec<_>>(),
    })
}

/// Cancels a goal that is dropped before it ends, as when its step runs out of time.
struct CancelOnDrop(Option<nervros_ros::CancelFn>);

impl Drop for CancelOnDrop {
    fn drop(&mut self) {
        if let Some(cancel) = self.0.take() {
            tokio::spawn(async move {
                let _ = cancel().await;
            });
        }
    }
}

/// Makes the goal that does the rest of a paused one, from it and its last feedback.
pub(crate) type Again<'a> = &'a (dyn Fn(&Value, &Value) -> Value + Sync);

/// How long an action server may take to accept a goal.
const SEND_WAIT: Duration = Duration::from_secs(5);

/// Runs one goal of `action` to its end and gives its result, kept in `goals` while it runs and
/// cancelled when `ctl` says stop. Paused, the goal is cancelled and, once resumed, `again` makes
/// the goal that does the rest from it and the last feedback; without `again` it runs on through a
/// pause, as an arm move finishes first.
///
/// # Errors
///
/// The server refused the goal or went away, or the mission was stopped.
pub(crate) async fn run_goal(
    robot: &dyn RobotPort,
    goals: &Arc<Goals>,
    (action, action_type): (&str, &str),
    mut goal: Value,
    ctl: &Control,
    again: Option<Again<'_>>,
) -> Result<nervros_ros::GoalResult, String> {
    let (mut stop, mut paused) = (ctl.stop.clone(), ctl.paused.clone());
    loop {
        if *paused.borrow() {
            tokio::select! {
                true = turns(&mut paused, false) => {}
                () = ctl.until_stopped() => {}
            }
        }
        if ctl.stopped() {
            return Err("stopped".to_owned());
        }
        let sent = robot
            .send_goal(action, action_type, goal.clone(), SEND_WAIT)
            .await
            .map_err(|e| format!("{action} did not take the goal: {e}"))?;
        let _open = goals.track(action, &sent);
        let cancel = sent.canceller();
        let mut unfinished = CancelOnDrop(Some(Arc::clone(&cancel)));
        let nervros_ros::Goal {
            mut feedback,
            result,
            ..
        } = sent;
        tokio::pin!(result);
        let mut last = Value::Null;
        let ended = loop {
            tokio::select! {
                Some(fb) = feedback.recv() => last = fb,
                r = &mut result => break Some(r),
                true = turns(&mut stop, true) => {
                    let _ = cancel().await;
                    return Err("stopped".to_owned());
                }
                true = turns(&mut paused, true), if again.is_some() => {
                    let _ = cancel().await;
                    break None;
                }
            }
        };
        unfinished.0 = None;
        match (ended, again) {
            (Some(Ok(r)), _) => return r.map_err(|e| format!("{action}: {e}")),
            (Some(Err(_)), _) => return Err(format!("{action} went away")),
            (None, Some(again)) => goal = again(&goal, &last),
            (None, None) => {}
        }
    }
}

/// Waits until `flag` is `to`; false when its sender has gone.
pub(crate) async fn turns(flag: &mut tokio::sync::watch::Receiver<bool>, to: bool) -> bool {
    flag.wait_for(|v| *v == to).await.is_ok()
}

/// What the integrations start from.
pub(crate) struct Parts<'a> {
    pub(crate) profile: &'a Profile,
    pub(crate) robot: &'a Arc<dyn RobotPort>,
    pub(crate) zones: &'a Arc<Zones>,
    pub(crate) places: &'a Arc<Places>,
    /// The quota ledger, beside which the robot's files are kept.
    pub(crate) ledger: &'a std::path::Path,
}

impl Parts<'_> {
    /// Where an integration keeps what it must remember across sessions, such as the shapes it
    /// gave the collision monitor.
    pub(crate) fn file(&self, kind: &str) -> Option<std::path::PathBuf> {
        crate::app::robot_file(self.profile, self.ledger, kind)
    }
}

/// The integrations the profile switches on.
#[derive(Debug, Default)]
pub struct Integrations {
    all: Vec<Arc<dyn Integration>>,
}

impl Integrations {
    /// Starts the integrations the profile's sections name. Must run inside a tokio runtime.
    pub(crate) fn start(parts: &Parts<'_>) -> Arc<Self> {
        let open = parts.profile.ros.open_goals.clone();
        let goals = Goals::published(Arc::clone(parts.robot), open);
        let mut all: Vec<Arc<dyn Integration>> = Vec::new();
        if let Some(config) = &parts.profile.nav2 {
            all.push(nav2::Nav2::start(config, parts, &goals));
        }
        if let Some(config) = &parts.profile.moveit {
            all.push(moveit::MoveIt::start(config, parts, &goals));
        }
        Arc::new(Self { all })
    }

    /// Each, in a fixed order.
    pub fn iter(&self) -> impl Iterator<Item = &Arc<dyn Integration>> {
        self.all.iter()
    }

    /// Every tool they add.
    #[must_use]
    pub fn tools(&self) -> Vec<Arc<dyn Tool>> {
        self.all.iter().flat_map(|i| i.tools()).collect()
    }

    /// The motions they run themselves, by name; the first of a name wins.
    #[must_use]
    pub fn natives(&self) -> Vec<Arc<dyn NativeSkill>> {
        let mut seen = std::collections::BTreeSet::new();
        self.all
            .iter()
            .flat_map(|i| i.natives())
            .filter(|n| seen.insert(n.skill()["name"].as_str().unwrap_or_default().to_owned()))
            .collect()
    }

    /// Gives each the session to report to.
    pub fn attach(&self, session: &crate::session::SessionHandle) {
        for i in &self.all {
            i.attach(session);
        }
    }

    /// Cancels what each sent; they stop together.
    pub async fn stop(&self) {
        futures::future::join_all(self.all.iter().map(|i| i.stop())).await;
    }
}

#[async_trait]
impl crate::mission::ArmPoses for Integrations {
    async fn arm_poses(&self) -> std::collections::BTreeMap<String, String> {
        let all = futures::future::join_all(self.all.iter().map(|i| i.arm_poses())).await;
        all.into_iter().flatten().collect()
    }
}

impl crate::memory::Notes for Integrations {
    fn text(&self) -> String {
        let lines: Vec<String> = self
            .all
            .iter()
            .map(|i| i.status().join("; "))
            .filter(|s| !s.is_empty())
            .collect();
        if lines.is_empty() {
            return String::new();
        }
        format!("\n\nThe robot's stacks now:\n- {}", lines.join("\n- "))
    }
}

/// The procedures (`SKILL.md` text) the profile's stacks bring for their faults.
#[must_use]
pub fn procedures(profile: &Profile) -> Vec<&'static str> {
    let mut out = Vec::new();
    if profile.nav2.is_some() {
        out.extend(nav2::PROCEDURES);
    }
    if profile.moveit.is_some() {
        out.extend(moveit::PROCEDURES);
    }
    out
}

/// What the profile's integrations find of their stacks on the graph, for the doctor.
pub async fn discover(profile: &Profile, robot: &dyn RobotPort) -> Vec<Check> {
    let mut out = Vec::new();
    if let Some(config) = &profile.nav2 {
        out.extend(nav2::discover(config, robot).await);
    }
    if let Some(config) = &profile.moveit {
        out.extend(moveit::discover(config, robot).await);
    }
    out
}

/// `name` under `namespace` when it is relative, as a node in that namespace resolves it.
pub(crate) fn resolve(namespace: &str, name: &str) -> String {
    if name.starts_with('/') {
        return name.to_owned();
    }
    let ns = namespace.trim_end_matches('/');
    if ns.is_empty() {
        format!("/{name}")
    } else if ns.starts_with('/') {
        format!("{ns}/{name}")
    } else {
        format!("/{ns}/{name}")
    }
}

/// A node's parameters by name, each as plain JSON with its type id; one the node does not have is
/// null with type 0.
///
/// # Errors
///
/// The node did not answer.
pub(crate) async fn get_params(
    robot: &dyn RobotPort,
    node: &str,
    names: &[String],
) -> Result<Vec<(Value, u64)>, String> {
    // rclcpp refuses a whole read when one name is not declared, and logs a warning each time:
    // the declared names are asked for, the rest come back unset.
    if let Ok(declared) = declared(robot, node).await {
        let asked: Vec<String> = names
            .iter()
            .filter(|n| declared.contains(*n))
            .cloned()
            .collect();
        let values = if asked.is_empty() {
            Vec::new()
        } else {
            get_values(robot, node, &asked).await?
        };
        let found: HashMap<&String, (Value, u64)> =
            asked.iter().zip(values.iter().map(typed)).collect();
        return Ok(names
            .iter()
            .map(|n| found.get(n).cloned().unwrap_or((Value::Null, 0)))
            .collect());
    }
    // A node that cannot list them is asked for them all, then one by one.
    let values = get_values(robot, node, names).await?;
    if values.len() == names.len() {
        return Ok(values.iter().map(typed).collect());
    }
    let mut out = Vec::with_capacity(names.len());
    for name in names {
        let one = get_values(robot, node, std::slice::from_ref(name)).await?;
        out.push(one.first().map_or((Value::Null, 0), typed));
    }
    Ok(out)
}

/// Every parameter `node` declares.
async fn declared(robot: &dyn RobotPort, node: &str) -> Result<HashSet<String>, String> {
    let out = robot
        .call(
            &format!("{node}/list_parameters"),
            "rcl_interfaces/srv/ListParameters",
            json!({"prefixes": [], "depth": 0}),
            READ_WAIT,
        )
        .await
        .map_err(|e| format!("{node} did not list its parameters: {e}"))?;
    Ok(out["result"]["names"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|n| n.as_str().map(str::to_owned))
        .collect())
}

async fn get_values(
    robot: &dyn RobotPort,
    node: &str,
    names: &[String],
) -> Result<Vec<Value>, String> {
    let out = robot
        .call(
            &format!("{node}/get_parameters"),
            "rcl_interfaces/srv/GetParameters",
            json!({"names": names}),
            READ_WAIT,
        )
        .await
        .map_err(|e| format!("{node} did not give its parameters: {e}"))?;
    Ok(out["values"].as_array().cloned().unwrap_or_default())
}

/// A `ParameterValue` as plain JSON, with its type id.
fn typed(v: &Value) -> (Value, u64) {
    (
        crate::ros_tools::params::param_value(v),
        v["type"].as_u64().unwrap_or(0),
    )
}

/// Sets one of a node's parameters to `value`, a `rcl_interfaces/msg/ParameterValue` as JSON.
///
/// # Errors
///
/// The node did not answer or refused, in its words.
pub(crate) async fn set_param(
    robot: &dyn RobotPort,
    node: &str,
    name: &str,
    value: Value,
) -> Result<(), String> {
    let out = robot
        .call(
            &format!("{node}/set_parameters"),
            "rcl_interfaces/srv/SetParameters",
            json!({"parameters": [{"name": name, "value": value}]}),
            READ_WAIT,
        )
        .await
        .map_err(|e| format!("{node} did not answer: {e}"))?;
    let result = &out["results"][0];
    if result["successful"].as_bool() == Some(true) {
        Ok(())
    } else {
        Err(format!(
            "{node} refused {name}: {}",
            result["reason"].as_str().unwrap_or("no reason given")
        ))
    }
}

/// A point on the map a tool was asked about, and what it is.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Spot {
    /// In the map frame, metres.
    pub(crate) x: f64,
    /// In the map frame, metres.
    pub(crate) y: f64,
    /// Which way to face there, when it matters.
    pub(crate) yaw: Option<f64>,
    /// What it is, for the operator: a place's or room's name, an object's label.
    pub(crate) what: String,
}

/// Where `args` point: `place` (a named place), `room` (an id or name), `object` (an id; the spot
/// is beside it, toward `from`, about `clear_m` off its box) or `x` and `y` (and `yaw`) on the map.
///
/// # Errors
///
/// None of them, or one that names nothing known, in words for the model.
pub(crate) async fn locate(
    args: &Value,
    (profile, places, robot): (&Profile, &Places, &dyn RobotPort),
    from: Option<(f64, f64)>,
    clear_m: f64,
) -> Result<Spot, String> {
    let text = |key: &str| args[key].as_str().map(str::trim).filter(|v| !v.is_empty());
    let known = |name: &str| {
        let same = |a: &str| crate::places::same_words(a, name);
        places
            .all()
            .into_iter()
            .find(|p| same(&p.name) || p.aliases.iter().any(|a| same(a)))
    };
    // A model gives a place's name as an object as often as not.
    let place = text("place").or_else(|| text("object").filter(|o| known(o).is_some()));
    if let Some(place) = place {
        return known(place)
            .map(|p| Spot {
                x: p.pose.x,
                y: p.pose.y,
                yaw: Some(p.pose.yaw),
                what: p.name,
            })
            .ok_or_else(|| format!("`{place}` is not a known place; list_places names them"));
    }
    let world = profile.world.as_ref();
    if let Some(room) = text("room") {
        let Some(topic) = world.and_then(|w| w.rooms.as_ref()) else {
            return Err("this robot has no world model with rooms".to_owned());
        };
        let rooms = robot
            .latest(&topic.topic, &topic.msg_type, READ_WAIT)
            .await
            .unwrap_or(Value::Null);
        let found = rooms["rooms"].as_array().into_iter().flatten().find(|r| {
            ["id", "name", "type"].iter().any(|k| {
                r[k].as_str()
                    .is_some_and(|v| crate::places::same_words(v, room))
            })
        });
        let Some(found) = found else {
            return Err(format!(
                "the world model has no room `{room}`; list_places names them"
            ));
        };
        let c = &found["centroid"];
        return match (c["x"].as_f64(), c["y"].as_f64()) {
            (Some(x), Some(y)) => Ok(Spot {
                x,
                y,
                yaw: None,
                what: crate::places::room_label(found).unwrap_or(room).to_owned(),
            }),
            _ => Err(format!("the world model gives `{room}` no centre")),
        };
    }
    if let Some(id) = text("object") {
        let Some(topic) = world.and_then(|w| w.objects.as_ref()) else {
            return Err("this robot has no world model with objects".to_owned());
        };
        let objects = robot
            .latest(&topic.topic, &topic.msg_type, READ_WAIT)
            .await
            .unwrap_or(Value::Null);
        return beside(&objects, id, from, clear_m);
    }
    match (args["x"].as_f64(), args["y"].as_f64()) {
        (Some(x), Some(y)) if x.is_finite() && y.is_finite() => Ok(Spot {
            x,
            y,
            yaw: args["yaw"].as_f64().filter(|v| v.is_finite()),
            what: format!("({x:.2}, {y:.2})"),
        }),
        _ => Err("say where: a place, a room, an object, or x and y on the map".to_owned()),
    }
}

/// The spot beside object `id` of the world model's `objects`, on the side toward `from`, about
/// `clear_m` off its box: its own centre is occupied.
pub(crate) fn beside(
    objects: &Value,
    id: &str,
    from: Option<(f64, f64)>,
    clear_m: f64,
) -> Result<Spot, String> {
    let found = objects["objects"]
        .as_array()
        .into_iter()
        .flatten()
        .find(|o| o["id"].as_str() == Some(id));
    let Some(found) = found else {
        return Err(format!(
            "the world model has no object `{id}`; find_objects gives ids"
        ));
    };
    let p = &found["pose"]["position"];
    let (Some(ox), Some(oy)) = (p["x"].as_f64(), p["y"].as_f64()) else {
        return Err(format!("`{id}` has no position in the world model"));
    };
    let size = &found["size"];
    let half = size["x"]
        .as_f64()
        .unwrap_or(0.0)
        .hypot(size["y"].as_f64().unwrap_or(0.0))
        / 2.0;
    // Beside it on the side it is seen from: its own centre is occupied.
    let (fx, fy) = from.unwrap_or((ox - 1.0, oy));
    let (dx, dy) = (fx - ox, fy - oy);
    let d = dx.hypot(dy).max(1e-6);
    let off = half + clear_m;
    let label = found["label"]
        .as_str()
        .filter(|l| !l.is_empty())
        .unwrap_or(id);
    Ok(Spot {
        x: ox + dx / d * off,
        y: oy + dy / d * off,
        yaw: Some((-dy).atan2(-dx)),
        what: label.to_owned(),
    })
}

/// A pose in the map frame for a goal: `geometry_msgs/msg/PoseStamped` as JSON.
pub(crate) fn stamped(frame: &str, x: f64, y: f64, yaw: f64) -> Value {
    json!({
        "header": {"frame_id": frame},
        "pose": {
            "position": {"x": x, "y": y, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": (yaw / 2.0).sin(), "w": (yaw / 2.0).cos()},
        },
    })
}

/// Whether the graph's topics show the action `name`: its feedback topic is there.
pub(crate) fn has_action(topics: &nervros_ros::NamesAndTypes, name: &str) -> bool {
    let feedback = format!("{name}/_action/feedback");
    topics.iter().any(|(t, _)| *t == feedback)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::Notes as _;

    #[derive(Debug)]
    struct Says(&'static str);

    #[async_trait]
    impl Integration for Says {
        fn id(&self) -> &'static str {
            "says"
        }
        fn tools(&self) -> Vec<Arc<dyn Tool>> {
            Vec::new()
        }
        fn status(&self) -> Vec<String> {
            vec![self.0.to_owned()]
        }
        async fn stop(&self) {}
    }

    #[tokio::test]
    async fn a_place_named_as_an_object_is_the_place() {
        let profile: Profile = toml::from_str(
            "[robot]\nname = \"x\"\n[models]\nfile = \"m.toml\"\n[[place]]\nname = \"office_desk_tray\"\npose = { x = 5.0, y = -4.0, yaw = 0.5 }\n",
        )
        .unwrap();
        let places = crate::places::Places::new(&profile, None);
        let robot = nervros_ros::fake::FakeRobot::new();
        let ctx = (&profile, places.as_ref(), &robot as &dyn RobotPort);
        let spot = locate(&json!({"object": "office_desk_tray"}), ctx, None, 0.3)
            .await
            .unwrap();
        assert_eq!(
            (spot.x, spot.y, spot.what.as_str()),
            (5.0, -4.0, "office_desk_tray")
        );
    }

    #[test]
    fn the_stacks_procedures_read_as_skills() {
        for text in [
            include_str!("integrations/nav2/procedures/robot-held-back.md"),
            include_str!("integrations/nav2/procedures/no-path-there.md"),
            include_str!("integrations/moveit/procedures/arm-cannot-reach.md"),
        ] {
            let skill = crate::skills::parse(text).unwrap();
            assert!(skill.body.contains("1. Call"), "{}", skill.name);
        }
    }

    #[tokio::test]
    async fn a_watchdog_hears_of_open_goals_until_they_close() {
        let robot = Arc::new(nervros_ros::fake::FakeRobot::new());
        let goals = Goals::published(robot.clone(), Some("/open_goals".to_owned()));
        let (_feed, feedback) = tokio::sync::mpsc::channel(1);
        let (_end, result) = tokio::sync::oneshot::channel();
        let cancel: nervros_ros::CancelFn = Arc::new(|| Box::pin(async { Ok(()) }));
        let goal = nervros_ros::Goal::new(uuid::Uuid::new_v4(), feedback, result, cancel);
        let last = || {
            robot
                .published()
                .into_iter()
                .rev()
                .find(|(t, _)| t == "/open_goals")
                .map(|(_, m)| m)
        };
        let kept = goals.track("/navigate_through_poses", &goal);
        tokio::time::sleep(Duration::from_millis(50)).await;
        let open = last().unwrap();
        assert_eq!(open["actions"], json!(["/navigate_through_poses"]));
        assert_eq!(open["goal_ids"][0]["uuid"], json!(goal.id.as_bytes()));
        drop(kept);
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(last().unwrap()["actions"], json!([]));
    }

    #[tokio::test]
    async fn parameters_are_read_one_by_one_when_one_is_not_declared() {
        // As rclcpp does: no values at all when any name asked for is not declared.
        let robot = nervros_ros::fake::FakeRobot::new().with_service("/n/get_parameters", |req| {
            let names = req["names"].as_array().cloned().unwrap_or_default();
            let declared = |n: &Value| n == "a" || n == "b";
            if !names.iter().all(declared) {
                return Ok(json!({"values": []}));
            }
            let values: Vec<Value> = names
                .iter()
                .map(|n| json!({"type": 4, "string_value": n}))
                .collect();
            Ok(json!({"values": values}))
        });
        let names = ["a".to_owned(), "nope".to_owned(), "b".to_owned()];
        let got = get_params(&robot, "/n", &names).await.unwrap();
        assert_eq!(
            got,
            vec![(json!("a"), 4), (Value::Null, 0), (json!("b"), 4)]
        );
    }

    #[test]
    fn a_name_is_taken_under_the_namespace_unless_it_is_absolute() {
        assert_eq!(resolve("", "navigate_to_pose"), "/navigate_to_pose");
        assert_eq!(
            resolve("/robot1", "navigate_to_pose"),
            "/robot1/navigate_to_pose"
        );
        assert_eq!(resolve("robot1/", "spin"), "/robot1/spin");
        assert_eq!(resolve("/robot1", "/move_group"), "/move_group");
    }

    #[test]
    fn each_turn_hears_only_the_stacks_with_something_to_say() {
        let quiet = Integrations::default();
        assert_eq!(quiet.text(), "");
        let some = Integrations {
            all: vec![Arc::new(Says("Nav2: idle")), Arc::new(Says(""))],
        };
        assert_eq!(some.text(), "\n\nThe robot's stacks now:\n- Nav2: idle");
    }
}
