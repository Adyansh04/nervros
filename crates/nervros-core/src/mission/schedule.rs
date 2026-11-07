//! Patrols and other repeated missions: a plan the operator approves once, run at once and then
//! every so many minutes, or each time something happens ("when a cup appears in the kitchen"),
//! a set number of times, each run reported like any mission. A run is skipped while the robot
//! is disarmed or busy; stopping the robot cancels every schedule.

use std::borrow::Cow;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use serde::Serialize;
use serde_json::{Value, json};
use tokio::task::JoinHandle;
use tracing::Instrument as _;

use crate::background::Background;
use crate::guard::Guard;
use crate::mission::Missions;
use crate::mission::plan::Compiled;
use crate::session::{Command, Event, SessionHandle};
use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};
use crate::watch::{Detector, Trigger};

const MAX_TIMES: u64 = 48;
const MAX_EVERY_MIN: u64 = 24 * 60;
const MAX_SCHEDULES: usize = 4;
const DEFAULT_FOR_MIN: u64 = 60;

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
        crate::lock(&self.list)
    }

    /// Where runs report; also cancels every schedule as soon as the robot is stopped. Each run
    /// checks for a stop itself too.
    pub fn attach(self: &Arc<Self>, session: SessionHandle) {
        if self.session.set(session).is_err() {
            return;
        }
        let me = Arc::downgrade(self);
        let mut stops = self.guard.stopped();
        tokio::spawn(async move {
            while stops.changed().await.is_ok() {
                let Some(me) = me.upgrade() else { return };
                if me.cancel("all") {
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
    pub fn running(&self) -> Vec<ScheduleInfo> {
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
        // Held weakly between runs, so that the schedules end with the session.
        let (me, counter) = (Arc::downgrade(self), Arc::clone(&left));
        let stops = self.guard.stops();
        let span = crate::telemetry::job("schedule", &id, &intent);
        let task = tokio::spawn(async move {
            for run in 1..=times {
                if run > 1 {
                    tokio::time::sleep(Duration::from_secs(every_min * 60)).await;
                }
                let Some(me) = me.upgrade() else { return };
                // The interval counts from the launch of the run before; a run still going
                // delays the next.
                me.missions.idle().await;
                if me.guard.stops() != stops {
                    return;
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
        // Held weakly between looks, so that the triggers end with the session.
        let (me, counter) = (Arc::downgrade(self), Arc::clone(&left));
        let stops = self.guard.stops();
        let span = crate::telemetry::job("trigger", &id, &what);
        let task = tokio::spawn(async move {
            let deadline = Instant::now() + Duration::from_secs(for_min * 60);
            let Some(first) = me.upgrade() else { return };
            let (robot, objects) = (first.missions.robot(), first.missions.objects_topic());
            let mut detector = match Detector::open(trigger, robot, objects).await {
                Ok(detector) => detector,
                Err(e) => {
                    first.report(format!("Trigger {id} ({what}) stopped: {e}."));
                    return;
                }
            };
            drop(first);
            // What holds already does not fire; a topic it cannot read shows at the first look.
            let _ = detector.settle().await;
            let mut run = 0;
            while run < times && Instant::now() < deadline {
                tokio::time::sleep(detector.period()).await;
                let Some(me) = me.upgrade() else { return };
                if me.guard.stops() != stops {
                    return;
                }
                let newly = detector.newly().await;
                if let Some(quiet) = detector.went_quiet() {
                    me.report(format!("Trigger {id} ({what}): {quiet}."));
                }
                let seen = match newly {
                    Ok(Some(seen)) => seen,
                    Ok(None) => continue,
                    // A trigger on what cannot be read would wait out its hours for nothing.
                    Err(e) => {
                        me.report(format!("Trigger {id} ({what}) stopped: {e}."));
                        return;
                    }
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
                me.missions.idle().await;
                let _ = detector.settle().await;
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

impl Background for Schedules {
    fn list(&self) -> Vec<Value> {
        self.running()
            .into_iter()
            .map(|s| {
                json!({"id": s.id, "kind": "schedule", "what": s.intent, "every_min": s.every_min,
                       "when": s.when, "left": s.left, "times": s.times})
            })
            .collect()
    }

    fn end(&self, id: &str) -> bool {
        self.cancel(id)
    }
}

impl Drop for Schedules {
    fn drop(&mut self) {
        // A run reports into its session, so the schedules end with it.
        for entry in self.lock().iter() {
            entry.task.abort();
        }
    }
}

fn spec() -> ToolSpec {
    ToolSpec::new(
        "schedule",
        "Runs a mission again and again, such as a patrol: \"every 30 minutes walk through the \
         rooms\". It takes a plan's steps as run_mission does, every_min and times; it runs at \
         once and then every every_min minutes, times runs in all. With when instead of \
         every_min it runs each time something happens, up to times runs within for_min \
         minutes: when {object, room} for \"when a cup appears in the kitchen, go there\", or a \
         topic condition as watch takes it. To only be told when something happens, use watch. \
         The operator approves it once for all its runs. background lists and cancels them.",
        json!({"type": "object", "properties": {
            "intent": {"type": "string", "description": "What it does, in a few words."},
            "steps": {"type": "array", "items": {"type": "object"}, "description": "The plan's steps, as run_mission takes them."},
            "every_min": {"type": "integer", "minimum": 1, "maximum": MAX_EVERY_MIN},
            "when": {"type": "object", "description": "Instead of every_min: {\"object\": what, \"room\": id}, or a topic and a condition as watch takes them."},
            "for_min": {"type": "integer", "minimum": 1, "maximum": MAX_EVERY_MIN, "description": "With when: how long it stays on the lookout (60)."},
            "times": {"type": "integer", "minimum": 1, "maximum": MAX_TIMES}
        }, "required": ["steps"], "additionalProperties": false}),
        Risk::Manipulation,
    )
}

/// What a clock schedule's numbers say that the operator's words do not.
fn numbers_differ(request: &str, every_min: u64, times: u64) -> Option<String> {
    let asked = crate::mission::sanity::repeat(request)?;
    let real = |n: u64| f64::from(u32::try_from(n).unwrap_or(u32::MAX));
    let mut wrong = Vec::new();
    if (asked.every_min - real(every_min)).abs() > 0.01 && asked.every_min >= 1.0 {
        wrong.push(format!(
            "every {} min, not every_min={every_min}",
            asked.every_min
        ));
    }
    if let Some(n) = asked.times.filter(|n| (n - real(times)).abs() > 0.01) {
        wrong.push(format!("{n} times in all, not times={times}"));
    }
    (!wrong.is_empty()).then(|| {
        format!(
            "the operator asked for {}: make the schedule match their words",
            wrong.join(" and ")
        )
    })
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
        let missions = &self.schedules.missions;
        let trigger = match Trigger::parse(
            &args["when"],
            missions.robot().as_ref(),
            missions.read_deny(),
        )
        .await
        {
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
            "trigger {} runs each time {}, up to {} in the next {for_min} min; it does not count \
             what holds already",
            info.id,
            info.when,
            crate::counted(times, "time")
        );
        out
    }
}

impl ScheduleTool {
    /// A plan that does its one step as many times as the schedule runs it: "turn left 10
    /// degrees, three times in all" once became three turns a run, three runs.
    fn counted_twice(&self, hash: &str, times: u64) -> Option<ToolOutcome> {
        let c = self.schedules.missions.find(hash).ok()?;
        let same = c.steps.windows(2).all(|w| w[0].summary == w[1].summary);
        (times > 1 && c.steps.len() == usize::try_from(times).ok()? && same).then(|| {
            ToolOutcome::refused(format!(
                "the schedule runs it {times} times, and each run does `{}` {times} times over: \
                 give the plan that step once",
                c.steps[0].summary
            ))
        })
    }
}

#[async_trait]
impl Tool for ScheduleTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    /// Checks the plan first, so the one approval is asked for a sound plan, and settles it as
    /// its hash.
    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let intent = args["intent"].as_str().unwrap_or("a schedule");
        let every = args["every_min"]
            .as_u64()
            .unwrap_or(0)
            .clamp(1, MAX_EVERY_MIN);
        let times = args["times"].as_u64().unwrap_or(0).clamp(1, MAX_TIMES);
        if args.get("when").is_none()
            && let Some(wrong) = numbers_differ(&self.schedules.missions.request(), every, times)
        {
            return Some(Err(ToolOutcome::refused(wrong)));
        }
        let trigger = match args.get("when").filter(|w| w.is_object()) {
            Some(when) => {
                let missions = &self.schedules.missions;
                match Trigger::parse(when, missions.robot().as_ref(), missions.read_deny()).await {
                    Ok(t) => Some(t),
                    Err(e) => return Some(Err(ToolOutcome::failed(e))),
                }
            }
            None => None,
        };
        // A plan checked before goes by its hash, past none of the checks above.
        let (hash, steps) = match args["hash"].as_str() {
            Some(hash) => match self.schedules.missions.find(hash) {
                Ok(c) => (c.sha256, c.steps.len()),
                Err(e) => return Some(Err(ToolOutcome::refused(e))),
            },
            None => match self.schedules.missions.check(intent, &args["steps"]).await {
                Ok((hash, steps, _)) => (hash, steps),
                Err(problems) => return Some(Err(problems)),
            },
        };
        if let Some(twice) = self.counted_twice(&hash, times) {
            return Some(Err(twice));
        }
        if let Some(trigger) = trigger {
            let for_min = args["for_min"]
                .as_u64()
                .unwrap_or(DEFAULT_FOR_MIN)
                .clamp(1, MAX_EVERY_MIN);
            return Some(Ok(Assessment {
                risk: Risk::Manipulation,
                resources: Vec::new(),
                reason: format!(
                    "runs \"{intent}\" ({}) each time {}, up to {} in the next {for_min} min",
                    crate::counted(steps, "step"),
                    trigger.describe(),
                    crate::counted(times, "time")
                ),
                args: Some(json!({"hash": hash, "intent": intent, "when": args["when"],
                                  "times": times, "for_min": for_min})),
            }));
        }
        Some(Ok(Assessment {
            risk: Risk::Manipulation,
            resources: Vec::new(),
            reason: format!(
                "runs \"{intent}\" ({}) now and every {every} min, {} in all",
                crate::counted(steps, "step"),
                crate::counted(times, "time")
            ),
            args: Some(json!({"hash": hash, "intent": intent, "every_min": every, "times": times})),
        }))
    }

    /// What it adds, with the plan's steps as they will run.
    fn rule_view(&self, args: &Value) -> Option<Value> {
        let mut view = args.clone();
        let plan = self.schedules.missions.rule_view(args["hash"].as_str()?)?;
        view["steps"] = plan["steps"].clone();
        Some(view)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        if self.schedules.running().len() >= MAX_SCHEDULES {
            return ToolOutcome::refused(format!(
                "{MAX_SCHEDULES} schedules run already; cancel one with background first"
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
        let times =
            u32::try_from(args["times"].as_u64().unwrap_or(1).clamp(1, MAX_TIMES)).unwrap_or(1);
        let intent = args["intent"].as_str().unwrap_or("a schedule").to_owned();
        if args["when"].is_object() {
            return self.trigger(compiled, intent, &args, times).await;
        }
        let info = self.schedules.add(compiled, intent, every, times);
        let mut out = ToolOutcome::ok(json!({"schedule": info}));
        out.message = format!(
            "schedule {} runs now and every {every} min, {}; a report follows each run",
            info.id,
            crate::counted(times, "time")
        );
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_schedule_must_count_as_the_operator_did() {
        let asked = "Every minute, turn left 90 degrees, 2 times in all.";
        assert_eq!(numbers_differ(asked, 1, 2), None);
        let wrong = numbers_differ(asked, 1, 48).unwrap();
        assert!(wrong.contains("2 times in all, not times=48"), "{wrong}");
        assert!(
            numbers_differ("every 5 minutes check the door", 1, 3)
                .unwrap()
                .contains("every 5 min")
        );
        assert_eq!(
            numbers_differ("turn left twice", 1, 7),
            None,
            "no clock, no schedule to match"
        );
    }
}
