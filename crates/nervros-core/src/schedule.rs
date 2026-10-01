//! Patrols and other repeated missions: a plan the operator approves once, run at once and then
//! every so many minutes, or each time something happens ("when a cup appears in the kitchen"),
//! a set number of times, each run reported like any mission. A run is skipped while the robot
//! is disarmed or busy; stopping the robot cancels every schedule.

use std::borrow::Cow;
use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use serde::Serialize;
use serde_json::{Value, json};
use tokio::task::JoinHandle;
use tracing::Instrument as _;

use crate::guard::Guard;
use crate::mission::Missions;
use crate::mission::check::named;
use crate::mission::plan::Compiled;
use crate::session::{Command, Event, SessionHandle};
use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};
use crate::watch::{Condition, holds, topic_of};

const MAX_TIMES: u64 = 48;
const MAX_EVERY_MIN: u64 = 24 * 60;
const MAX_SCHEDULES: usize = 4;
/// How often a run waits for the mission before it to end.
const POLL: Duration = Duration::from_secs(2);
/// How often a trigger on a topic looks; one on the world model looks at the POLL rate.
const TOPIC_LOOK: Duration = Duration::from_millis(500);
const DEFAULT_FOR_MIN: u64 = 60;
/// The canopy object state for an object it removed.
const REMOVED: u64 = 2;

/// A schedule as the operator sees it.
#[derive(Debug, Clone, Serialize)]
pub struct ScheduleInfo {
    /// Its id, such as `p1`.
    pub id: String,
    /// What it does.
    pub intent: String,
    /// Minutes between runs; 0 for a trigger.
    pub every_min: u64,
    /// For a trigger: what starts a run.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub when: String,
    /// Runs still to come.
    pub left: u32,
    /// Runs in all.
    pub times: u32,
}

struct Entry {
    info: ScheduleInfo,
    left: Arc<AtomicU32>,
    task: JoinHandle<()>,
}

/// The running schedules.
pub struct Schedules {
    missions: Arc<Missions>,
    guard: Arc<Guard>,
    list: Mutex<Vec<Entry>>,
    next: AtomicU64,
    session: OnceLock<SessionHandle>,
}

impl Schedules {
    /// No schedules yet; [`Self::attach`] connects them to the session.
    #[must_use]
    pub fn new(missions: Arc<Missions>, guard: Arc<Guard>) -> Arc<Self> {
        Arc::new(Self {
            missions,
            guard,
            list: Mutex::default(),
            next: AtomicU64::new(1),
            session: OnceLock::new(),
        })
    }

    fn lock(&self) -> MutexGuard<'_, Vec<Entry>> {
        self.list.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Where runs report; also cancels every schedule when the robot is stopped.
    pub fn attach(self: &Arc<Self>, session: SessionHandle) {
        let mut events = session.subscribe();
        if self.session.set(session).is_err() {
            return;
        }
        let me = Arc::downgrade(self);
        tokio::spawn(async move {
            loop {
                let stopped = match events.recv().await {
                    Ok(Event::Halted { reason }) => reason.contains("robot"),
                    Ok(Event::ToolFinished { tool, .. }) => tool == "stop",
                    Ok(_) | Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => false,
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => return,
                };
                let Some(me) = me.upgrade() else { return };
                if stopped && me.cancel("all") {
                    me.say(Event::Notice {
                        text: "the stop cancelled every schedule".to_owned(),
                    });
                }
            }
        });
    }

    fn say(&self, event: Event) {
        if let Some(s) = self.session.get() {
            s.emit(event);
        }
    }

    fn report(&self, text: String) {
        if let Some(s) = self.session.get() {
            s.send(Command::Report(text));
        }
    }

    /// The `schedule` tool.
    #[must_use]
    pub fn tools(self: &Arc<Self>) -> [Arc<dyn Tool>; 1] {
        [Arc::new(ScheduleTool {
            spec: spec(),
            schedules: Arc::clone(self),
        })]
    }

    /// The schedules, oldest first.
    #[must_use]
    pub fn list(&self) -> Vec<ScheduleInfo> {
        let mut list = self.lock();
        list.retain(|e| !e.task.is_finished());
        list.iter()
            .map(|e| ScheduleInfo {
                left: e.left.load(Ordering::Relaxed),
                ..e.info.clone()
            })
            .collect()
    }

    /// Cancels one schedule by id, or `all`; whether any was cancelled.
    pub fn cancel(&self, id: &str) -> bool {
        let mut list = self.lock();
        let before = list.len();
        list.retain(|e| {
            let hit = id == "all" || e.info.id == id;
            if hit {
                e.task.abort();
            }
            !hit
        });
        list.len() < before
    }

    fn add(
        self: &Arc<Self>,
        compiled: Compiled,
        intent: String,
        every_min: u64,
        times: u32,
    ) -> ScheduleInfo {
        let id = format!("p{}", self.next.fetch_add(1, Ordering::Relaxed));
        let left = Arc::new(AtomicU32::new(times));
        let info = ScheduleInfo {
            id: id.clone(),
            intent: intent.clone(),
            every_min,
            when: String::new(),
            left: times,
            times,
        };
        let (me, counter) = (Arc::clone(self), Arc::clone(&left));
        let span = crate::telemetry::job("schedule", &id, &intent);
        let task = tokio::spawn(async move {
            for run in 1..=times {
                if run > 1 {
                    tokio::time::sleep(Duration::from_secs(every_min * 60)).await;
                }
                // The interval counts from the end of the run before, never overlapping it.
                while me.missions.busy() {
                    tokio::time::sleep(POLL).await;
                }
                counter.fetch_sub(1, Ordering::Relaxed);
                let label = format!("{intent}, run {run} of {times} of schedule {id}");
                if !me.guard.armed() {
                    me.report(format!("Schedule {id} skipped run {run} of {times} ({intent}): the robot is not armed."));
                    continue;
                }
                if let Err(e) = me.missions.run_scheduled(compiled.clone(), &label).await {
                    me.report(format!(
                        "Schedule {id} skipped run {run} of {times} ({intent}): {e}."
                    ));
                }
            }
        }.instrument(span));
        self.lock().push(Entry {
            info: info.clone(),
            left,
            task,
        });
        info
    }

    /// Runs `compiled` each time `trigger` newly holds, up to `times` runs within `for_min`.
    fn add_trigger(
        self: &Arc<Self>,
        compiled: Compiled,
        intent: String,
        trigger: Trigger,
        times: u32,
        for_min: u64,
    ) -> ScheduleInfo {
        let id = format!("t{}", self.next.fetch_add(1, Ordering::Relaxed));
        let left = Arc::new(AtomicU32::new(times));
        let what = trigger.describe();
        let info = ScheduleInfo {
            id: id.clone(),
            intent: intent.clone(),
            every_min: 0,
            when: what.clone(),
            left: times,
            times,
        };
        let (me, counter) = (Arc::clone(self), Arc::clone(&left));
        let span = crate::telemetry::job("trigger", &id, &what);
        let task = tokio::spawn(async move {
            let deadline = Instant::now() + Duration::from_secs(for_min * 60);
            let mut looker = Looker::new(trigger, &me.missions).await;
            let mut run = 0;
            while run < times && Instant::now() < deadline {
                tokio::time::sleep(looker.period()).await;
                let Some(seen) = looker.look(&me.missions).await else {
                    continue;
                };
                run += 1;
                counter.fetch_sub(1, Ordering::Relaxed);
                let skipped = |why: &str| {
                    format!(
                        "Trigger {id} ({what}) fired: {seen}; it skipped run {run} of {times} ({intent}): {why}."
                    )
                };
                if me.missions.busy() {
                    me.report(skipped("a mission was running"));
                    continue;
                }
                if !me.guard.armed() {
                    me.report(skipped("the robot is not armed"));
                    continue;
                }
                me.say(Event::Notice {
                    text: format!("Trigger {id} fired: {seen}; running \"{intent}\""),
                });
                let label = format!("{intent}, run {run} of {times} of trigger {id} ({seen})");
                if let Err(e) = me.missions.run_scheduled(compiled.clone(), &label).await {
                    me.report(skipped(&e));
                }
                // What the run itself changes does not fire it again.
                while me.missions.busy() {
                    tokio::time::sleep(POLL).await;
                }
                looker.settle(&me.missions).await;
            }
        }.instrument(span));
        self.lock().push(Entry {
            info: info.clone(),
            left,
            task,
        });
        info
    }
}

/// What starts a triggered run.
#[derive(Debug, Clone, PartialEq)]
enum Trigger {
    /// A condition on a topic, as `watch` takes it, becoming true.
    Topic {
        topic: String,
        ty: String,
        condition: Condition,
    },
    /// An object like `phrase` appearing in the world model, in `room` when it names one.
    Object {
        phrase: String,
        room: Option<String>,
    },
}

impl Trigger {
    /// From the `when` argument: `{topic, condition, ...}` or `{object, room}`.
    async fn parse(when: &Value, missions: &Missions) -> Result<Self, String> {
        if let Some(phrase) = when["object"]
            .as_str()
            .map(str::trim)
            .filter(|p| !p.is_empty())
        {
            let room = when["room"]
                .as_str()
                .map(str::trim)
                .filter(|r| !r.is_empty())
                .map(str::to_owned);
            return Ok(Self::Object {
                phrase: phrase.to_owned(),
                room,
            });
        }
        if when.get("topic").is_some() {
            let condition = Condition::parse(when)?;
            let (topic, ty) = topic_of(missions.robot().as_ref(), when).await?;
            return Ok(Self::Topic {
                topic,
                ty,
                condition,
            });
        }
        Err("`when` names an object (and maybe a room), or a topic and a condition as watch takes them".to_owned())
    }

    fn describe(&self) -> String {
        match self {
            Self::Topic {
                topic, condition, ..
            } => condition.describe(topic),
            Self::Object { phrase, room } => match room {
                Some(r) => format!("a {phrase} appears in {r}"),
                None => format!("a {phrase} appears"),
            },
        }
    }
}

/// A trigger's view of the robot, to tell a new occurrence from one already there: a topic
/// condition fires when it turns true, an object trigger when an object it has not seen appears.
struct Looker {
    trigger: Trigger,
    was: bool,
    known: BTreeSet<String>,
}

impl Looker {
    /// What holds already does not fire.
    async fn new(trigger: Trigger, missions: &Missions) -> Self {
        let mut looker = Self {
            trigger,
            was: false,
            known: BTreeSet::new(),
        };
        looker.settle(missions).await;
        looker
    }

    fn period(&self) -> Duration {
        match self.trigger {
            Trigger::Topic { .. } => TOPIC_LOOK,
            Trigger::Object { .. } => POLL,
        }
    }

    /// Takes in what holds now without firing.
    async fn settle(&mut self, missions: &Missions) {
        match &self.trigger {
            Trigger::Topic {
                topic,
                ty,
                condition,
            } => {
                self.was = holds(missions.robot().as_ref(), topic, ty, condition)
                    .await
                    .is_some();
            }
            Trigger::Object { phrase, room } => {
                self.known.extend(objects(
                    &missions.observe().await.objects,
                    phrase,
                    room.as_deref(),
                ));
            }
        }
    }

    /// What newly holds, if anything.
    async fn look(&mut self, missions: &Missions) -> Option<String> {
        match &self.trigger {
            Trigger::Topic {
                topic,
                ty,
                condition,
            } => {
                let seen = holds(missions.robot().as_ref(), topic, ty, condition).await;
                let fired = !self.was && seen.is_some();
                self.was = seen.is_some();
                seen.filter(|_| fired)
            }
            Trigger::Object { phrase, room } => {
                let now = objects(&missions.observe().await.objects, phrase, room.as_deref());
                let new: Vec<String> = now.difference(&self.known).cloned().collect();
                self.known.extend(now);
                (!new.is_empty()).then(|| format!("{} appeared", new.join(" and ")))
            }
        }
    }
}

/// The ids of the objects like `phrase` the world model holds, in `room` when given.
fn objects(msg: &Value, phrase: &str, room: Option<&str>) -> BTreeSet<String> {
    msg["objects"]
        .as_array()
        .map_or(&[][..], Vec::as_slice)
        .iter()
        .filter(|o| o["state"].as_u64() != Some(REMOVED) && named(o, phrase))
        .filter(|o| room.is_none_or(|r| o["room_id"].as_str() == Some(r)))
        .filter_map(|o| o["id"].as_str().map(str::to_owned))
        .collect()
}

fn spec() -> ToolSpec {
    ToolSpec::new(
        "schedule",
        "Runs a mission again and again, such as a patrol: \"every 30 minutes walk through the \
         rooms\". add takes a plan's steps as run_mission does, every_min and times; it runs at \
         once and then every every_min minutes, times runs in all. With when instead of \
         every_min it runs each time something happens, up to times runs within for_min \
         minutes: when {object, room} for \"when a cup appears in the kitchen\", or a topic \
         condition as watch takes it. The operator approves it once for all its runs. list shows \
         the schedules, cancel ends one by id or all.",
        json!({"type": "object", "properties": {
            "action": {"type": "string", "enum": ["add", "list", "cancel"]},
            "intent": {"type": "string", "description": "For add: what it does, in a few words."},
            "steps": {"type": "array", "items": {"type": "object"}, "description": "For add: the plan's steps, as run_mission takes them."},
            "every_min": {"type": "integer", "minimum": 1, "maximum": MAX_EVERY_MIN},
            "when": {"type": "object", "description": "For add, instead of every_min: {\"object\": what, \"room\": an optional room id} runs when such an object appears; {\"topic\", \"condition\", ...} as watch takes it runs when that becomes true."},
            "for_min": {"type": "integer", "minimum": 1, "maximum": MAX_EVERY_MIN, "description": "With when: how long to wait for it; 60 by default."},
            "times": {"type": "integer", "minimum": 1, "maximum": MAX_TIMES},
            "id": {"type": "string", "description": "For cancel: such as p1, or all."}
        }, "required": ["action"], "additionalProperties": false}),
        Risk::Manipulation,
    )
}

struct ScheduleTool {
    spec: ToolSpec,
    schedules: Arc<Schedules>,
}

impl ScheduleTool {
    async fn trigger(
        &self,
        compiled: Compiled,
        intent: String,
        args: &Value,
        times: u32,
    ) -> ToolOutcome {
        let trigger = match Trigger::parse(&args["when"], &self.schedules.missions).await {
            Ok(t) => t,
            Err(e) => return ToolOutcome::failed(e),
        };
        let for_min = args["for_min"]
            .as_u64()
            .unwrap_or(DEFAULT_FOR_MIN)
            .clamp(1, MAX_EVERY_MIN);
        let info = self
            .schedules
            .add_trigger(compiled, intent, trigger, times, for_min);
        let mut out = ToolOutcome::ok(json!({"schedule": info}));
        out.message = format!(
            "trigger {} runs each time {}, up to {times} time(s) in the next {for_min} min; it \
             does not run for what holds already",
            info.id, info.when
        );
        out
    }
}

#[async_trait]
impl Tool for ScheduleTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    /// Listing and cancelling only read or do less; adding checks the plan first, so the one
    /// approval is asked for a sound plan, and settles it as its hash.
    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let observe = |reason: &str| {
            Some(Ok(Assessment {
                risk: Risk::Observe,
                resources: Vec::new(),
                reason: reason.to_owned(),
                args: None,
            }))
        };
        match args["action"].as_str() {
            Some("list") => return observe("lists the schedules"),
            Some("cancel") => return observe("cancels a schedule"),
            Some("add") => {}
            _ => return Some(Err(ToolOutcome::failed("`action` is add, list or cancel"))),
        }
        if args.get("hash").is_some() {
            return None;
        }
        let intent = args["intent"].as_str().unwrap_or("a schedule");
        let every = args["every_min"]
            .as_u64()
            .unwrap_or(0)
            .clamp(1, MAX_EVERY_MIN);
        let times = args["times"].as_u64().unwrap_or(0).clamp(1, MAX_TIMES);
        let trigger = match args.get("when").filter(|w| w.is_object()) {
            Some(when) => match Trigger::parse(when, &self.schedules.missions).await {
                Ok(t) => Some(t),
                Err(e) => return Some(Err(ToolOutcome::failed(e))),
            },
            None => None,
        };
        let (hash, steps, _) = match self.schedules.missions.check(intent, &args["steps"]).await {
            Ok(checked) => checked,
            Err(problems) => return Some(Err(problems)),
        };
        if let Some(trigger) = trigger {
            let for_min = args["for_min"]
                .as_u64()
                .unwrap_or(DEFAULT_FOR_MIN)
                .clamp(1, MAX_EVERY_MIN);
            return Some(Ok(Assessment {
                risk: Risk::Manipulation,
                resources: Vec::new(),
                reason: format!(
                    "runs \"{intent}\" ({steps} step(s)) each time {}, up to {times} time(s) in the next {for_min} min",
                    trigger.describe()
                ),
                args: Some(json!({"action": "add", "hash": hash, "intent": intent,
                                  "when": args["when"], "times": times, "for_min": for_min})),
            }));
        }
        Some(Ok(Assessment {
            risk: Risk::Manipulation,
            resources: Vec::new(),
            reason: format!(
                "runs \"{intent}\" ({steps} step(s)) now and every {every} min, {times} time(s) in all"
            ),
            args: Some(json!({"action": "add", "hash": hash, "intent": intent,
                              "every_min": every, "times": times})),
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str() {
            Some("list") => ToolOutcome::ok(json!({"schedules": self.schedules.list()})),
            Some("cancel") => {
                let id = args["id"].as_str().unwrap_or("all");
                if self.schedules.cancel(id) || id == "all" {
                    ToolOutcome::ok(json!({"cancelled": id}))
                } else {
                    ToolOutcome::failed(format!("no schedule {id}; list them first"))
                }
            }
            Some("add") => {
                if self.schedules.list().len() >= MAX_SCHEDULES {
                    return ToolOutcome::refused(format!(
                        "{MAX_SCHEDULES} schedules run already; cancel one first"
                    ));
                }
                let compiled = match self
                    .schedules
                    .missions
                    .find(args["hash"].as_str().unwrap_or_default())
                {
                    Ok(c) => c,
                    Err(e) => return ToolOutcome::refused(e),
                };
                let every = args["every_min"]
                    .as_u64()
                    .unwrap_or(1)
                    .clamp(1, MAX_EVERY_MIN);
                let times = u32::try_from(args["times"].as_u64().unwrap_or(1).clamp(1, MAX_TIMES))
                    .unwrap_or(1);
                let intent = args["intent"].as_str().unwrap_or("a schedule").to_owned();
                if args["when"].is_object() {
                    return self.trigger(compiled, intent, &args, times).await;
                }
                let info = self.schedules.add(compiled, intent, every, times);
                let mut out = ToolOutcome::ok(json!({"schedule": info}));
                out.message = format!(
                    "schedule {} runs now and every {every} min, {times} time(s); a report follows each run",
                    info.id
                );
                out
            }
            _ => ToolOutcome::failed("`action` is add, list or cancel"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::places::Places;
    use crate::profile::Profile;
    use nervros_ros::fake::FakeRobot;

    const PROFILE: &str = r#"
        [robot]
        name = "t"
        [world]
        objects = { topic = "/objects", type = "canopy_msgs/msg/WorldObjectArray" }
        [mission]
        execute = "/x/execute"
        validate = "/x/validate"
        catalog = "/x/catalog"
        stop = "/x/stop"
        state = "/x/state"
        [models]
        file = "m.toml"
    "#;

    fn missions(robot: &Arc<FakeRobot>) -> Arc<Missions> {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("nervros.toml");
        std::fs::write(&path, PROFILE).unwrap();
        let profile = Profile::load(&path).unwrap();
        let robot = Arc::clone(robot) as Arc<dyn nervros_ros::RobotPort>;
        Missions::new(&profile, Places::new(&profile, None), robot).unwrap()
    }

    fn object(id: &str, label: &str, room: &str, state: u64) -> Value {
        json!({"id": id, "label": label, "room_id": room, "state": state})
    }

    #[tokio::test]
    async fn a_topic_trigger_fires_when_its_condition_turns_true_not_while_it_stays() {
        let robot = Arc::new(FakeRobot::new().with_topic("/door", json!({"data": true})));
        let missions = missions(&robot);
        let condition = Condition::parse(
            &json!({"condition": "value", "field": "data", "op": "==", "value": true}),
        )
        .unwrap();
        let trigger = Trigger::Topic {
            topic: "/door".to_owned(),
            ty: "std_msgs/msg/Bool".to_owned(),
            condition,
        };
        let mut looker = Looker::new(trigger, &missions).await;
        assert_eq!(looker.look(&missions).await, None, "open already when set");
        robot.set_topic("/door", json!({"data": false}));
        assert_eq!(looker.look(&missions).await, None);
        robot.set_topic("/door", json!({"data": true}));
        assert_eq!(
            looker.look(&missions).await.as_deref(),
            Some("data is true")
        );
        assert_eq!(looker.look(&missions).await, None, "once per opening");
    }

    #[tokio::test]
    async fn an_object_trigger_fires_for_a_new_object_in_its_room() {
        let robot = Arc::new(FakeRobot::new().with_topic(
            "/objects",
            json!({"objects": [object("O1", "white cup", "R2", 0)]}),
        ));
        let missions = missions(&robot);
        let trigger = Trigger::Object {
            phrase: "cup".to_owned(),
            room: Some("R2".to_owned()),
        };
        assert_eq!(trigger.describe(), "a cup appears in R2");
        let mut looker = Looker::new(trigger, &missions).await;
        assert_eq!(looker.look(&missions).await, None, "O1 was there already");
        robot.set_topic(
            "/objects",
            json!({"objects": [
                object("O1", "white cup", "R2", 0),
                object("O2", "red cup", "R1", 0),
                object("O3", "plate", "R2", 0),
                object("O4", "blue cup", "R2", 2),
            ]}),
        );
        assert_eq!(
            looker.look(&missions).await,
            None,
            "another room, not a cup, removed"
        );
        robot.set_topic(
            "/objects",
            json!({"objects": [object("O1", "white cup", "R2", 0), object("O5", "cup", "R2", 0)]}),
        );
        assert_eq!(looker.look(&missions).await.as_deref(), Some("O5 appeared"));
        assert_eq!(looker.look(&missions).await, None);
    }

    #[tokio::test]
    async fn when_names_an_object_or_a_topic_condition() {
        let robot = Arc::new(FakeRobot::new());
        let missions = missions(&robot);
        let object = Trigger::parse(&json!({"object": " cup ", "room": ""}), &missions)
            .await
            .unwrap();
        assert_eq!(
            object,
            Trigger::Object {
                phrase: "cup".to_owned(),
                room: None
            }
        );
        let neither = Trigger::parse(&json!({"room": "R2"}), &missions).await;
        assert!(neither.unwrap_err().contains("names an object"));
    }
}
