//! Goals: what the operator wants to hold, such as "keep the mug on the tray", kept across
//! sessions. Code judges each goal's predicates after every mission and at start; the model works
//! toward an open one a mission at a time, each approved like any other, and may only give one up
//! as impossible, with the reason. Budgets are counted here, never by the model.

use std::borrow::Cow;
use std::fmt::Write as _;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::SystemTime;

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use super::Missions;
use super::check::{self, Observed};
use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};

/// Enough open goals for one operator, few enough to stay in every prompt.
const MAX_OPEN: usize = 8;
/// The missions a goal may take when the model names no budget.
const DEFAULT_MISSIONS: u32 = 5;
/// The most missions one goal may take.
const MOST_MISSIONS: u32 = 20;
/// The most predicates one goal may name.
const MOST_PREDICATES: usize = 6;

/// Where a goal stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum State {
    /// Worked toward.
    Active,
    /// Kept, but no mission is run for it.
    Paused,
    /// Every predicate held.
    Met,
    /// The model gave it up, with the reason.
    Impossible,
    /// Its missions or its time ran out first.
    Exhausted,
}

impl State {
    fn name(self) -> &'static str {
        match self {
            Self::Active => "active",
            Self::Paused => "paused",
            Self::Met => "met",
            Self::Impossible => "impossible",
            Self::Exhausted => "exhausted",
        }
    }

    fn open(self) -> bool {
        matches!(self, Self::Active | Self::Paused)
    }
}

/// One goal.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Goal {
    /// Its number.
    pub id: u32,
    /// The operator's words.
    pub text: String,
    /// What holds once it is met, such as `inside(O17, tray_1)`: all of them.
    pub done_when: Vec<String>,
    /// The missions it may take. A run the robot or the executor cut short is not counted.
    pub max_missions: u32,
    /// The missions run for it that counted, by id.
    pub missions: Vec<String>,
    /// When it was set, seconds since 1970.
    pub set_at: u64,
    /// When it runs out, seconds since 1970; none for never.
    pub expires: Option<u64>,
    /// Where it stands.
    pub state: State,
    /// What did not hold when it was last judged, with why.
    pub unmet: Vec<String>,
    /// Why it ended, when it was given up or ran out.
    pub why: String,
}

impl Goal {
    /// One line for the model and the operator.
    #[must_use]
    pub fn line(&self) -> String {
        format!("goal {} ({}): {}", self.id, self.text, self.status())
    }

    /// Where it stands: its state, with what is unmet and the missions used, or why it ended.
    #[must_use]
    pub fn status(&self) -> String {
        let mut line = self.state.name().to_owned();
        match self.state {
            State::Active | State::Paused => {
                if !self.unmet.is_empty() {
                    let _ = write!(line, "; unmet: {}", self.unmet.join(", "));
                }
                let _ = write!(
                    line,
                    "; {} of {} missions used",
                    self.missions.len(),
                    self.max_missions
                );
            }
            State::Impossible | State::Exhausted => {
                let _ = write!(line, ": {}", self.why.trim_end_matches('.'));
            }
            State::Met => {}
        }
        line
    }
}

/// What a request to set a goal says.
#[derive(Debug, Clone, Default)]
pub struct NewGoal {
    /// The operator's words.
    pub text: String,
    /// Its predicates.
    pub done_when: Vec<String>,
    /// Its missions; none for the default.
    pub max_missions: Option<u32>,
    /// How many hours it may take; none for no limit.
    pub hours: Option<f64>,
}

/// The goals, and the file they live in.
#[derive(Debug)]
pub struct Goals {
    goals: Mutex<Vec<Goal>>,
    file: Option<PathBuf>,
}

impl Goals {
    /// The goals in `file`, when it exists.
    #[must_use]
    pub fn new(file: Option<PathBuf>) -> Arc<Self> {
        let goals = file
            .as_deref()
            .map(crate::persist::read_or_default)
            .unwrap_or_default();
        Arc::new(Self {
            goals: Mutex::new(goals),
            file,
        })
    }

    fn lock(&self) -> MutexGuard<'_, Vec<Goal>> {
        crate::lock(&self.goals)
    }

    fn save(&self, goals: &[Goal]) -> Result<(), String> {
        let Some(file) = &self.file else {
            return Ok(());
        };
        let text = serde_json::to_vec_pretty(goals).map_err(|e| e.to_string())?;
        crate::persist::write_atomic(file, &text)
            .map_err(|e| format!("saving {}: {e}", file.display()))
    }

    /// Every goal, oldest first.
    #[must_use]
    pub fn all(&self) -> Vec<Goal> {
        self.lock().clone()
    }

    /// Whether goal `id` is worked toward now.
    #[must_use]
    pub fn active(&self, id: u32) -> bool {
        self.lock()
            .iter()
            .any(|g| g.id == id && g.state == State::Active)
    }

    /// Sets a goal and judges it at once against `seen`: one that already holds is met.
    ///
    /// # Errors
    ///
    /// Its words or predicates are missing or unusable, there are too many open goals, or the
    /// file cannot be written.
    pub fn set(&self, new: &NewGoal, seen: &Observed, now: SystemTime) -> Result<Goal, String> {
        let text = new.text.trim();
        if text.is_empty() {
            return Err("`text` is the goal in the operator's words".to_owned());
        }
        let done_when: Vec<String> = new
            .done_when
            .iter()
            .map(|p| p.trim().to_owned())
            .filter(|p| !p.is_empty())
            .collect();
        if done_when.is_empty() || done_when.len() > MOST_PREDICATES {
            return Err(format!(
                "`done_when` takes 1 to {MOST_PREDICATES} predicates that all hold once the goal \
                 is met"
            ));
        }
        if let Some(bad) = done_when.iter().find(|p| !check::checkable(p)) {
            return Err(format!(
                "{bad} is not a predicate NervROS can check: use at(place), holding(left or \
                 right, object), inside(object, container) or on(object, support), with ids from \
                 find_objects and list_places"
            ));
        }
        // A goal on a name only a detector knows could never be seen met: the world model judges.
        if seen.objects.is_null() && done_when.iter().any(|p| !p.starts_with("holding")) {
            return Err(
                "the world model has not said what it knows, so the goal's names cannot be \
                 checked; set it once the world model runs"
                    .to_owned(),
            );
        }
        let mut unknown: Vec<&str> = Vec::new();
        for id in done_when.iter().flat_map(|p| check::unknown_in(p, seen)) {
            if !unknown.contains(&id) {
                unknown.push(id);
            }
        }
        if !unknown.is_empty() {
            return Err(unknown_names(&done_when, &unknown));
        }
        let max_missions = new.max_missions.unwrap_or(DEFAULT_MISSIONS);
        if !(1..=MOST_MISSIONS).contains(&max_missions) {
            return Err(format!("`max_missions` is 1 to {MOST_MISSIONS}"));
        }
        let set_at = crate::unix_secs(now);
        let expires = match new.hours {
            None => None,
            Some(h) if h.is_finite() && h > 0.0 && h <= 24.0 * 30.0 => {
                Some(set_at + std::time::Duration::from_secs_f64(h * 3600.0).as_secs())
            }
            Some(_) => return Err("`hours` is more than 0 and at most 720".to_owned()),
        };
        let mut goals = self.lock();
        if goals.iter().filter(|g| g.state.open()).count() >= MAX_OPEN {
            return Err(format!(
                "{MAX_OPEN} goals are open already; drop or give up one first"
            ));
        }
        let mut goal = Goal {
            id: goals.iter().map(|g| g.id).max().unwrap_or(0) + 1,
            text: text.to_owned(),
            done_when,
            max_missions,
            missions: Vec::new(),
            set_at,
            expires,
            state: State::Active,
            unmet: Vec::new(),
            why: String::new(),
        };
        judge(&mut goal, seen, set_at);
        goals.push(goal.clone());
        self.save(&goals)?;
        Ok(goal)
    }

    /// Changes goal `id` with `change`; the goal as it is now.
    fn change(
        &self,
        id: u32,
        change: impl FnOnce(&mut Goal) -> Result<(), String>,
    ) -> Result<Goal, String> {
        let mut goals = self.lock();
        let goal = goals
            .iter_mut()
            .find(|g| g.id == id)
            .ok_or_else(|| format!("no goal {id}; list them first"))?;
        change(goal)?;
        let changed = goal.clone();
        self.save(&goals)?;
        Ok(changed)
    }

    /// Pauses goal `id`, or with `on` false makes it active again.
    ///
    /// # Errors
    ///
    /// No open goal has that number, or the file cannot be written.
    pub fn pause(&self, id: u32, on: bool) -> Result<Goal, String> {
        self.change(id, |g| {
            if !g.state.open() {
                return Err(format!("goal {id} is {} already", g.state.name()));
            }
            g.state = if on { State::Paused } else { State::Active };
            Ok(())
        })
    }

    /// Gives up goal `id` as impossible, for `why`.
    ///
    /// # Errors
    ///
    /// No reason was given, no open goal has that number, or the file cannot be written.
    pub fn impossible(&self, id: u32, why: &str) -> Result<Goal, String> {
        let why = why.trim();
        if why.is_empty() {
            return Err("`why` says what makes the goal impossible".to_owned());
        }
        self.change(id, |g| {
            if !g.state.open() {
                return Err(format!("goal {id} is {} already", g.state.name()));
            }
            g.state = State::Impossible;
            why.clone_into(&mut g.why);
            Ok(())
        })
    }

    /// Forgets goal `id`; `false` when there was none.
    ///
    /// # Errors
    ///
    /// The file cannot be written.
    pub fn drop_goal(&self, id: u32) -> Result<bool, String> {
        let mut goals = self.lock();
        let before = goals.len();
        goals.retain(|g| g.id != id);
        if goals.len() == before {
            return Ok(false);
        }
        self.save(&goals).map(|()| true)
    }

    /// Counts mission `mission`, run for goal `id`, unless a fault made it no test of the plan.
    pub fn ran(&self, id: u32, mission: &str, counted: bool) {
        if !counted {
            return;
        }
        let changed = self.change(id, |g| {
            g.missions.push(mission.to_owned());
            Ok(())
        });
        if let Err(e) = changed {
            tracing::warn!(error = %e, goal = id, "the goal's mission was not counted");
        }
    }

    /// Judges every active goal against `seen`, as after a mission or at start; the open goals
    /// and those that just ended.
    pub fn settle(&self, seen: &Observed, now: SystemTime) -> Vec<Goal> {
        let at = crate::unix_secs(now);
        let mut goals = self.lock();
        let mut out = Vec::new();
        for goal in goals.iter_mut() {
            let was = goal.state;
            // An exhausted goal that holds after all is met: the world can catch up.
            if matches!(was, State::Active | State::Exhausted) {
                judge(goal, seen, at);
            }
            if goal.state.open() || goal.state != was {
                out.push(goal.clone());
            }
        }
        if let Err(e) = self.save(&goals) {
            tracing::warn!(error = %e, "the goals were not saved");
        }
        out
    }
}

/// `done_when` with each name the world model does not know, such as a detector's `mug_4`, taken
/// as the world model's likeliest object of its kind, and a line for each taken: the planner's
/// names are the detector's, and a model told to set the goal again in other ids ran the mission
/// first. A name with no likely object is left, for `set` to refuse.
fn in_world_ids(done_when: &[String], seen: &Observed) -> (Vec<String>, Vec<String>) {
    let now_s = crate::now_s();
    let mut taken: Vec<(&str, String)> = Vec::new();
    let mut lines = Vec::new();
    for id in done_when.iter().flat_map(|p| check::unknown_in(p, seen)) {
        if taken.iter().any(|(name, _)| *name == id) {
            continue;
        }
        if let Some((best, line)) = check::likeliest(seen, id, now_s) {
            lines.push(format!("{id} taken as {line}"));
            taken.push((id, best));
        }
    }
    let rewritten = done_when
        .iter()
        .map(|p| check::renamed(p, &taken))
        .collect();
    (rewritten, lines)
}

/// Why a goal naming `unknown` is refused: the world model could never see it met.
fn unknown_names(done_when: &[String], unknown: &[&str]) -> String {
    // "Not set" leads: a model that kept a refusal's first words reported one as set.
    format!(
        "not set: the world model does not know {}, so NervROS could never see {} met: set it \
         with ids find_objects and list_places return",
        unknown.join(" and "),
        done_when.join(", ")
    )
}

/// Judges one goal: met once every predicate holds, else exhausted once its missions or its time
/// ran out, but only on what the world model plainly says: one it cannot judge now, as while
/// perception catches up with a place, is not yet unmet for good.
fn judge(goal: &mut Goal, seen: &Observed, at: u64) {
    #[expect(clippy::cast_precision_loss, reason = "seconds since 1970")]
    let verdicts = check::check(&goal.done_when, seen, goal.set_at as f64);
    goal.unmet = verdicts
        .iter()
        .filter(|v| v.ok != Some(true))
        .map(|v| format!("{} ({})", v.predicate, v.detail))
        .collect();
    if goal.unmet.is_empty() {
        goal.state = State::Met;
        return;
    }
    if goal.state == State::Exhausted || verdicts.iter().any(|v| v.ok.is_none()) {
        return;
    }
    if goal.missions.len() >= usize::try_from(goal.max_missions).unwrap_or(usize::MAX) {
        goal.state = State::Exhausted;
        goal.why = format!("its {} missions were used up", goal.max_missions);
    } else if goal.expires.is_some_and(|e| at >= e) {
        goal.state = State::Exhausted;
        "its time ran out".clone_into(&mut goal.why);
    }
}

/// The goals as a report's closing lines, with `ask_next` a request for the next mission of each
/// active one.
#[must_use]
pub fn report_lines(goals: &[Goal], ask_next: bool) -> String {
    let mut out = String::new();
    for g in goals {
        let _ = write!(out, "\nGoal: {}.", g.line());
        if g.state == State::Active && ask_next {
            let _ = write!(
                out,
                " If a mission can bring it closer, plan it with run_mission's `goal` {}; if \
                 nothing can, give it up with `goal` impossible and the reason.",
                g.id
            );
        }
    }
    out
}

impl crate::memory::Notes for Goals {
    fn text(&self) -> String {
        let goals = self.lock();
        let open: Vec<String> = goals
            .iter()
            .filter(|g| g.state.open())
            .map(|g| format!("\n- {}", g.line()))
            .collect();
        if open.is_empty() {
            return String::new();
        }
        format!(
            "\n\nThe operator's open goals (the `goal` tool keeps them; a mission for one names \
             it in run_mission's `goal`):{}",
            open.concat()
        )
    }
}

/// The `goal` tool.
pub struct GoalTool {
    spec: ToolSpec,
    goals: Arc<Goals>,
    missions: Arc<Missions>,
}

impl GoalTool {
    /// Setting, pausing, giving up and dropping change what the robot works toward, so the
    /// operator approves them when supervised; listing is a read.
    #[must_use]
    pub fn new(goals: Arc<Goals>, missions: Arc<Missions>) -> Self {
        let spec = ToolSpec::new(
            "goal",
            "Keeps a goal the operator wants to hold across missions and sessions, such as \"keep \
             the mug on the tray\", with the predicates that hold once it is met. NervROS checks \
             them after every mission and says what is unmet; work toward an active goal with \
             missions that name it in run_mission's `goal`. Only the operator's goals: set one \
             when they ask for a lasting outcome, not for a single task. Give one up as \
             impossible, with the reason, only when no mission can bring it closer.",
            json!({"type": "object", "properties": {
                "action": {"type": "string", "enum": ["set", "list", "pause", "resume", "impossible", "drop"]},
                "text": {"type": "string", "description": "For set: the goal in the operator's words."},
                "done_when": {"type": "array", "items": {"type": "string"}, "description": "For set: predicates that all hold once it is met: at(place), holding(left or right, object), inside(object, container), on(object, support), with ids from find_objects and list_places."},
                "max_missions": {"type": "integer", "minimum": 1, "maximum": MOST_MISSIONS, "description": "For set: the most missions it may take; 5 when left out."},
                "hours": {"type": "number", "description": "For set: how long it may take; no limit when left out."},
                "id": {"type": "integer", "description": "For pause, resume, impossible and drop: the goal's number."},
                "why": {"type": "string", "description": "For impossible: what makes it impossible."}
            }, "required": ["action"], "additionalProperties": false}),
            Risk::Annotate,
        );
        Self {
            spec,
            goals,
            missions,
        }
    }
}

fn id_of(args: &Value) -> u32 {
    args["id"]
        .as_u64()
        .and_then(|i| u32::try_from(i).ok())
        .unwrap_or(0)
}

#[async_trait]
impl Tool for GoalTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let id = id_of(args);
        let action = args["action"].as_str();
        if id == 0 && matches!(action, Some("pause" | "resume" | "impossible" | "drop")) {
            return Some(Err(ToolOutcome::failed(
                "`id` is the goal's number; list the goals to find it",
            )));
        }
        let (risk, reason) = match action {
            Some("list") => (Risk::Observe, "lists the goals".to_owned()),
            Some("set") => (
                Risk::Annotate,
                format!(
                    "sets the goal \"{}\"",
                    args["text"].as_str().unwrap_or_default()
                ),
            ),
            Some("pause") => (Risk::Annotate, format!("pauses goal {id}")),
            Some("resume") => (Risk::Annotate, format!("resumes goal {id}")),
            Some("impossible") => (
                Risk::Annotate,
                format!(
                    "gives up goal {id}: {}",
                    args["why"].as_str().unwrap_or_default()
                ),
            ),
            Some("drop") => (Risk::Annotate, format!("drops goal {id}")),
            _ => {
                return Some(Err(ToolOutcome::failed(
                    "`action` is set, list, pause, resume, impossible or drop",
                )));
            }
        };
        Some(Ok(Assessment {
            risk,
            resources: Vec::new(),
            reason,
            args: None,
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let id = id_of(&args);
        let mut taken = Vec::new();
        let done = match args["action"].as_str() {
            Some("set") => {
                let mut new = NewGoal {
                    text: args["text"].as_str().unwrap_or_default().to_owned(),
                    done_when: args["done_when"]
                        .as_array()
                        .map(|a| {
                            a.iter()
                                .filter_map(Value::as_str)
                                .map(str::to_owned)
                                .collect()
                        })
                        .unwrap_or_default(),
                    max_missions: args["max_missions"]
                        .as_u64()
                        .and_then(|m| u32::try_from(m).ok()),
                    hours: args["hours"].as_f64(),
                };
                let seen = self.missions.observe().await;
                (new.done_when, taken) = in_world_ids(&new.done_when, &seen);
                self.goals.set(&new, &seen, SystemTime::now())
            }
            Some("pause") => self.goals.pause(id, true),
            Some("resume") => self.goals.pause(id, false),
            Some("impossible") => self
                .goals
                .impossible(id, args["why"].as_str().unwrap_or_default()),
            Some("drop") => {
                return match self.goals.drop_goal(id) {
                    Ok(true) => ToolOutcome::ok(json!({"dropped": id})),
                    Ok(false) => ToolOutcome::failed(format!("no goal {id}; list them first")),
                    Err(why) => ToolOutcome::failed(why),
                };
            }
            _ => {
                let lines: Vec<String> = self.goals.all().iter().map(Goal::line).collect();
                return ToolOutcome::ok(json!({"goals": lines}));
            }
        };
        match done {
            Ok(goal) => {
                let mut out = ToolOutcome::ok(json!({"goal": goal.line()}));
                // Worked toward at once, not from the next mission's report.
                out.message = if goal.state == State::Active {
                    format!(
                        "{}; plan a mission toward it now, naming it in run_mission's `goal` {}",
                        goal.line(),
                        goal.id
                    )
                } else {
                    goal.line()
                };
                if !taken.is_empty() {
                    let _ = write!(
                        out.message,
                        ". Set in the world model's ids: {}; if another object is meant, drop it \
                         and set it again with that one's id",
                        taken.join("; ")
                    );
                }
                out
            }
            Err(why) => ToolOutcome::failed(why),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn seen(right_hand: &str) -> Observed {
        Observed {
            state: json!({"holding_left": "", "holding_right": right_hand}),
            ..Observed::default()
        }
    }

    fn new(done_when: &[&str], max_missions: Option<u32>) -> NewGoal {
        NewGoal {
            text: "hold the mug".to_owned(),
            done_when: done_when.iter().map(|p| (*p).to_owned()).collect(),
            max_missions,
            hours: None,
        }
    }

    #[test]
    fn a_goal_is_met_by_code_counts_only_fair_missions_and_runs_out() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("goals").join("r.json");
        let goals = Goals::new(Some(file.clone()));
        let now = SystemTime::now();
        let goal = goals
            .set(&new(&["holding(right, O17)"], Some(2)), &seen(""), now)
            .unwrap();
        assert_eq!(goal.state, State::Active);
        assert_eq!(
            goal.unmet,
            ["holding(right, O17) (the right hand holds nothing)"]
        );

        goals.ran(goal.id, "m1", false);
        goals.ran(goal.id, "m2", true);
        assert_eq!(
            goals.settle(&seen(""), now)[0].state,
            State::Active,
            "a fault is not counted"
        );
        assert_eq!(goals.settle(&seen("O17"), now)[0].state, State::Met);

        let other = goals
            .set(&new(&["holding(right, O18)"], Some(1)), &seen(""), now)
            .unwrap();
        goals.ran(other.id, "m3", true);
        let ended = goals.settle(&seen(""), now);
        let other = ended.iter().find(|g| g.id == other.id).unwrap();
        assert_eq!(other.state, State::Exhausted);
        assert_eq!(other.why, "its 1 missions were used up");
        assert_eq!(
            Goals::new(Some(file)).all().len(),
            2,
            "kept across sessions"
        );
    }

    #[test]
    fn no_verdict_exhausts_nothing_and_an_exhausted_goal_may_be_met_late() {
        let goals = Goals::new(None);
        let now = SystemTime::now();
        let g = goals
            .set(&new(&["holding(right, O17)"], Some(1)), &seen(""), now)
            .unwrap();
        goals.ran(g.id, "m1", true);
        let silent = Observed::default();
        assert_eq!(
            goals.settle(&silent, now)[0].state,
            State::Active,
            "no verdict yet"
        );
        assert_eq!(goals.settle(&seen(""), now)[0].state, State::Exhausted);
        assert_eq!(
            goals.settle(&seen("O17"), now)[0].state,
            State::Met,
            "it held after all"
        );
    }

    #[test]
    fn a_detectors_names_are_taken_as_the_world_models_likeliest_objects() {
        let seen = Observed {
            objects: json!({"objects": [
                {"id": "O27", "label": "mug", "state": 0, "last_seen": {"sec": 100, "nanosec": 0}},
                {"id": "O783", "label": "mug", "state": 0, "last_seen": {"sec": 900, "nanosec": 0}},
                {"id": "O261", "label": "tray", "state": 0}]}),
            ..seen("")
        };
        let (done_when, taken) = in_world_ids(
            &["inside(mug_4, tray_1)".to_owned(), "at(O261)".to_owned()],
            &seen,
        );
        assert_eq!(done_when, ["inside(O783, O261)", "at(O261)"]);
        assert_eq!(taken.len(), 2, "{taken:?}");
        assert!(
            taken[0].starts_with("mug_4 taken as O783 (mug"),
            "{taken:?}"
        );
        let (kept, none) = in_world_ids(&["inside(kettle_2, O261)".to_owned()], &seen);
        assert_eq!(
            kept,
            ["inside(kettle_2, O261)"],
            "no likely object: left for set to refuse"
        );
        assert!(none.is_empty());
    }

    #[test]
    fn only_checkable_goals_are_set_and_only_open_ones_given_up() {
        let goals = Goals::new(None);
        let now = SystemTime::now();
        assert!(
            goals
                .set(&new(&["tidy(kitchen)"], None), &seen(""), now)
                .is_err()
        );
        assert!(goals.set(&new(&[], None), &seen(""), now).is_err());
        let with_mug = Observed {
            objects: json!({"objects": [{"id": "O27", "label": "red mug"}, {"id": "O31", "label": "tray"}]}),
            ..seen("")
        };
        let unknown = goals
            .set(&new(&["inside(mug_4, O31)"], None), &with_mug, now)
            .unwrap_err();
        assert!(
            unknown.starts_with("not set: the world model does not know mug_4"),
            "a detector's name the world model cannot judge: {unknown}"
        );
        let met = goals
            .set(&new(&["holding(right, O17)"], None), &seen("O17"), now)
            .unwrap();
        assert_eq!(met.state, State::Met, "it held already");
        assert!(goals.impossible(met.id, "gone").is_err());
        let open = goals
            .set(&new(&["holding(right, O18)"], None), &seen(""), now)
            .unwrap();
        assert!(
            goals.impossible(open.id, " ").is_err(),
            "a reason is needed"
        );
        assert_eq!(
            goals.impossible(open.id, "no O18 anywhere").unwrap().state,
            State::Impossible
        );
        assert!(goals.drop_goal(open.id).unwrap() && !goals.drop_goal(open.id).unwrap());
    }
}
