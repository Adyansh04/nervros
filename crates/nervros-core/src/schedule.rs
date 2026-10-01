//! Patrols and other repeated missions: a plan the operator approves once, run at once and then
//! every so many minutes, a set number of times, each run reported like any mission. A run is
//! skipped while the robot is disarmed or busy; stopping the robot cancels every schedule.

use std::borrow::Cow;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::Duration;

use async_trait::async_trait;
use serde::Serialize;
use serde_json::{Value, json};
use tokio::task::JoinHandle;

use crate::guard::Guard;
use crate::mission::Missions;
use crate::mission::plan::Compiled;
use crate::session::{Command, Event, SessionHandle};
use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};

const MAX_TIMES: u64 = 48;
const MAX_EVERY_MIN: u64 = 24 * 60;
const MAX_SCHEDULES: usize = 4;
/// How often a run waits for the mission before it to end.
const POLL: Duration = Duration::from_secs(2);

/// A schedule as the operator sees it.
#[derive(Debug, Clone, Serialize)]
pub struct ScheduleInfo {
    /// Its id, such as `p1`.
    pub id: String,
    /// What it does.
    pub intent: String,
    /// Minutes between runs.
    pub every_min: u64,
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
            left: times,
            times,
        };
        let (me, counter) = (Arc::clone(self), Arc::clone(&left));
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
        });
        self.lock().push(Entry {
            info: info.clone(),
            left,
            task,
        });
        info
    }
}

fn spec() -> ToolSpec {
    ToolSpec::new(
        "schedule",
        "Runs a mission again and again, such as a patrol: \"every 30 minutes walk through the \
         rooms\". add takes a plan's steps as run_mission does, every_min and times; it runs at \
         once and then every every_min minutes, times runs in all. The operator approves it once \
         for all its runs. list shows the schedules, cancel ends one by id or all.",
        json!({"type": "object", "properties": {
            "action": {"type": "string", "enum": ["add", "list", "cancel"]},
            "intent": {"type": "string", "description": "For add: what it does, in a few words."},
            "steps": {"type": "array", "items": {"type": "object"}, "description": "For add: the plan's steps, as run_mission takes them."},
            "every_min": {"type": "integer", "minimum": 1, "maximum": MAX_EVERY_MIN},
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
        let (hash, steps, _) = match self.schedules.missions.check(intent, &args["steps"]).await {
            Ok(checked) => checked,
            Err(problems) => return Some(Err(problems)),
        };
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
                if self.schedules.cancel(id) {
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
