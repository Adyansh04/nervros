//! The `program` tool: runs, keeps and stops robot programs.

use std::borrow::Cow;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::api::SkillCall;
use super::store::{Saved, Store};
use super::{api, engine, host::RobotHost, skills};
use crate::job::envelope::{DEFAULT_MINUTES, MAX_MINUTES};
use crate::job::{Approval, Envelope, JobKind, Jobs};
use crate::mission::Missions;
use crate::mission::catalog::{Catalog, GO_TO_PLACE};
use crate::tools::{Assessment, Risk, Status, Tool, ToolOutcome, ToolSpec};

/// Runs, keeps and stops programs.
pub struct ProgramTool {
    spec: ToolSpec,
    missions: Arc<Missions>,
    jobs: Arc<Jobs>,
    store: Arc<Store>,
    /// The map frame and the robot's base frame.
    frames: (String, String),
}

impl ProgramTool {
    /// The tool for `missions`' robot, its jobs in `jobs` and its kept programs in `store`.
    #[must_use]
    pub fn new(
        missions: Arc<Missions>,
        jobs: Arc<Jobs>,
        store: Arc<Store>,
        frames: (String, String),
    ) -> Self {
        Self {
            spec: spec(),
            missions,
            jobs,
            store,
            frames,
        }
    }

    /// The program a `run` means, with its title: its text, or the kept one it names, with the
    /// minutes it was kept with.
    fn source(&self, args: &Value) -> Result<(String, String, Option<u64>), String> {
        let title = args["title"].as_str().unwrap_or_default().trim().to_owned();
        if let Some(source) = args["source"].as_str().filter(|s| !s.trim().is_empty()) {
            let title = if title.is_empty() {
                "a program".to_owned()
            } else {
                title
            };
            return Ok((title, source.to_owned(), None));
        }
        let name = args["name"].as_str().unwrap_or_default();
        match self.store.get(name) {
            Some(saved) => Ok((saved.name, saved.source, Some(saved.max_minutes))),
            None if name.is_empty() => {
                Err("`source` is the program to run, or `name` a kept one".to_owned())
            }
            None => Err(format!("no program is kept as \"{name}\"; list them")),
        }
    }

    /// The program a `run` means, checked as the card and the run both need it: parsed, its skills
    /// known to the robot, and its minutes those given, those it was kept with, or the default.
    async fn prepare(&self, args: &Value) -> Result<Prepared, String> {
        let (title, source, kept_minutes) = self.source(args)?;
        let ast = engine::compile(&source)?;
        let catalog = self.missions.catalog().await?;
        let skills = callable(&catalog);
        let named = skills::named_in(&source, &skills)?;
        let minutes = args["max_minutes"]
            .as_u64()
            .or(kept_minutes)
            .unwrap_or(DEFAULT_MINUTES);
        let envelope = Envelope::new(named, minutes);
        let unknown = envelope.unknown(&catalog);
        if !unknown.is_empty() {
            return Err(format!(
                "the robot has no skill {}; its skills are {}",
                unknown.join(", "),
                catalog.plan_skills().join(", ")
            ));
        }
        Ok(Prepared {
            title,
            source,
            ast,
            skills,
            envelope,
            catalog,
        })
    }

    /// Checks a program to run and settles what the operator approves.
    async fn assess_run(&self, args: &Value) -> Result<Assessment, ToolOutcome> {
        let p = self.prepare(args).await.map_err(ToolOutcome::failed)?;
        let approval = Approval::of(p.envelope.clone(), &p.catalog);
        let lines = crate::counted(
            p.source.lines().filter(|l| !l.trim().is_empty()).count(),
            "line",
        );
        Ok(Assessment {
            risk: p.envelope.risk(&p.catalog),
            resources: Vec::new(),
            reason: format!(
                "runs the program \"{}\" ({lines}) on its own: it {}",
                p.title, approval.text
            ),
            args: Some(
                json!({"action": "run", "title": p.title, "source": p.source,
                              "max_minutes": p.envelope.max_minutes()}),
            ),
        })
    }

    async fn run(&self, args: &Value) -> ToolOutcome {
        let p = match self.prepare(args).await {
            Ok(p) => p,
            Err(e) => return ToolOutcome::failed(e),
        };
        // A run of the kept program, not new source given its title.
        let kept = self
            .store
            .get(&p.title)
            .is_some_and(|k| k.source == p.source);
        let approval = Approval::of(p.envelope, &p.catalog);
        let moves = approval.moves;
        let (missions, frames, ast, skills) = (
            Arc::clone(&self.missions),
            self.frames.clone(),
            p.ast,
            p.skills,
        );
        let work = move |ctx: crate::job::Context| async move {
            let runtime = tokio::runtime::Handle::current();
            let robot = Arc::new(RobotHost::new(missions, ctx.clone(), runtime, frames));
            let host: Arc<dyn api::Host> = Arc::clone(&robot) as Arc<dyn api::Host>;
            tokio::task::spawn_blocking(move || {
                let engine = engine::engine(&host, &skills, move || ctx.stopped());
                let value = engine::run(&engine, &ast)?;
                let text = value.unwrap_or_else(|| engine::nothing(robot.last_said().as_deref()));
                // A loop over rooms() in an unmapped building walks nowhere, and a model then
                // told the operator it had walked through every room.
                Ok(if moves && robot.skills_run() == 0 {
                    format!("{text}; it ran no skill, so the robot did not move")
                } else {
                    text
                })
            })
            .await
            .map_err(|e| format!("the program's thread failed: {e}"))?
        };
        match self.jobs.spawn(JobKind::Program, &p.title, approval, work) {
            Ok(job) => {
                if kept {
                    let (store, title) = (Arc::clone(&self.store), p.title.clone());
                    // Not awaited: a count lost to a slow disk only loses the count.
                    drop(tokio::task::spawn_blocking(move || store.ran(&title)));
                }
                ToolOutcome {
                    status: Status::Accepted,
                    message: format!(
                        "program {} runs on its own; its lines show as it goes and its result comes \
                     as a report. Tell the operator it started.",
                        job.id
                    ),
                    data: json!({"job": job}),
                    images: Vec::new(),
                }
            }
            Err(e) => ToolOutcome::refused(e),
        }
    }

    async fn save(&self, args: &Value) -> ToolOutcome {
        let name = args["name"].as_str().unwrap_or_default().trim();
        let source = args["source"].as_str().unwrap_or_default();
        if name.is_empty() || source.trim().is_empty() {
            return ToolOutcome::failed(
                "save takes the program as `source` and a `name` to keep it under",
            );
        }
        if let Err(e) = engine::compile(source) {
            return ToolOutcome::failed(e);
        }
        let saved = Saved {
            name: name.to_owned(),
            source: source.to_owned(),
            max_minutes: args["max_minutes"].as_u64().unwrap_or(DEFAULT_MINUTES),
            saved_s: crate::now_s(),
            runs: 0,
        };
        let store = Arc::clone(&self.store);
        // The store writes and syncs its file: off the async threads.
        match tokio::task::spawn_blocking(move || store.save(saved)).await {
            Ok(Ok(())) => ToolOutcome::ok(json!({"saved": name})),
            Ok(Err(e)) => ToolOutcome::failed(e),
            Err(e) => ToolOutcome::failed(format!("the program was not kept: {e}")),
        }
    }

    fn list(&self) -> ToolOutcome {
        let kept: Vec<Value> = self
            .store
            .list()
            .iter()
            .map(|p| json!({"name": p.name, "runs": p.runs, "max_minutes": p.max_minutes}))
            .collect();
        ToolOutcome::ok(json!({"kept": kept}))
    }
}

/// A program checked to run.
struct Prepared {
    title: String,
    source: String,
    ast: rhai::AST,
    skills: Vec<SkillCall>,
    envelope: Envelope,
    catalog: Arc<Catalog>,
}

/// The skills a program may call by name, each with the arguments bare values fill.
fn callable(catalog: &Catalog) -> Vec<SkillCall> {
    catalog
        .plan_skills()
        .into_iter()
        .map(|name| {
            let args = if name == GO_TO_PLACE {
                vec!["place".to_owned()]
            } else {
                catalog
                    .skill(&name)
                    .map(|s| s.model_args().map(|a| a.name.clone()).collect())
                    .unwrap_or_default()
            };
            SkillCall { name, args }
        })
        .collect()
}

fn spec() -> ToolSpec {
    ToolSpec::new(
        "program",
        &format!(
            "Runs a short program on the robot, for what takes loops, counting or measuring: \
             \"count the mugs in every room\", \"which room has the most chairs?\", \"walk to each \
             room and look for a cup\", \"time five walks of 2 m\". Counting or listing what the \
             world model knows needs no walking: use objects() and rooms(); walk and find only \
             when the operator wants the camera to check. Not for exploring or mapping: rooms() \
             lists only the rooms already mapped, so plan a mission with the robot's own skill \
             for that. A mission ends at its first failed \
             step, so a look that finds nothing would end a walk through several rooms: to look \
             in each place whatever each look finds, write a program. Write Rhai, which is not JavaScript: end \
             every statement with `;`, an empty map is `#{{}}` (never `{{}}`), a map is \
             `#{{key: value}}`, there is no `?:` (write `if c {{ a }} else {{ b }}`), no `.length` \
             (write `.len()`), `()` is nothing, and the last line's value is the result. Counting \
             mugs per room: `let n = #{{}}; for r in rooms() {{ n[r.type] = objects(\"mug\", \
             r.id).len(); }} n`. Its functions: {}. run actions take `source` (or `name`, a kept \
             program), a short `title`, and `max_minutes` (1 to {MAX_MINUTES}, {DEFAULT_MINUTES} if left \
             out). The operator approves it once, with the skills its walk, find and run calls \
             name and its time limit; it then runs on its own, and its result comes as a report. \
             save keeps one as `name`; list shows the kept ones; forget drops one. background \
             lists the running ones and ends them.",
            api::REFERENCE
        ),
        json!({"type": "object", "properties": {
            "action": {"type": "string", "enum": ["run", "save", "list", "forget"]},
            "source": {"type": "string", "description": "For run or save: the program."},
            "title": {"type": "string", "description": "For run: what it is for, in a few words."},
            "name": {"type": "string", "description": "For save or forget, or run instead of source: the name it is kept as."},
            "max_minutes": {"type": "integer", "minimum": 1, "maximum": MAX_MINUTES}
        }, "required": ["action"], "additionalProperties": false}),
        Risk::Manipulation,
    )
}

#[async_trait]
impl Tool for ProgramTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    /// Listing and stopping only read or do less; keeping changes what the robot keeps; running
    /// is approved as what its skills can do, for its time limit.
    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let simple = |risk: Risk, reason: &str| {
            Some(Ok(Assessment {
                risk,
                resources: Vec::new(),
                reason: reason.to_owned(),
                args: None,
            }))
        };
        match args["action"].as_str() {
            Some("list") => simple(Risk::Observe, "lists the programs"),
            Some("save") => simple(Risk::Annotate, "keeps a program by name"),
            Some("forget") => simple(Risk::Annotate, "forgets a kept program"),
            Some("run") => Some(self.assess_run(args).await),
            _ => Some(Err(ToolOutcome::failed(
                "`action` is run, save, list or forget",
            ))),
        }
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str() {
            Some("run") => self.run(&args).await,
            Some("save") => self.save(&args).await,
            Some("list") => self.list(),
            Some("forget") => {
                let name = args["name"].as_str().unwrap_or_default().to_owned();
                let (store, which) = (Arc::clone(&self.store), name.clone());
                match tokio::task::spawn_blocking(move || store.forget(&which)).await {
                    Ok(Ok(true)) => ToolOutcome::ok(json!({"forgot": name})),
                    Ok(Ok(false)) => {
                        ToolOutcome::failed(format!("no program is kept as \"{name}\""))
                    }
                    Ok(Err(e)) => ToolOutcome::failed(e),
                    Err(e) => ToolOutcome::failed(format!("the program was not forgotten: {e}")),
                }
            }
            _ => ToolOutcome::failed("`action` is run, save, list or forget"),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;
    use std::time::Duration;

    use nervros_ros::RobotPort;
    use nervros_ros::fake::FakeRobot;

    use super::*;
    use crate::guard::{Guard, Policy};
    use crate::job::JobState;
    use crate::mission::catalog::tests::CATALOG;
    use crate::places::Places;
    use crate::profile::Profile;
    use crate::session::{Event, SessionHandle};

    fn tool() -> (ProgramTool, Arc<Jobs>) {
        let profile = Profile::from_toml(
            r#"
            [robot]
            name = "t"
            [world]
            rooms = { topic = "/rooms", type = "canopy_msgs/msg/RoomArray" }
            [mission]
            execute = "/x/execute"
            validate = "/x/validate"
            catalog = "/x/catalog"
            stop = "/x/stop"
            state = "/x/state"
            [models]
            file = "m.toml"
            "#,
            Path::new("nervros.toml"),
        )
        .unwrap();
        let robot: Arc<dyn RobotPort> = Arc::new(
            FakeRobot::new()
                .with_service("/x/catalog", |_| Ok(json!({"catalog_json": CATALOG})))
                .with_topic(
                    "/rooms",
                    json!({"rooms": [{"id": "R1", "type": "kitchen"},
                                                       {"id": "R2", "type": "office"}]}),
                ),
        );
        let missions = Missions::new(&profile, Places::new(&profile, None), robot).unwrap();
        let guard = Arc::new(Guard::new(Policy::default()));
        let jobs = Jobs::new(guard);
        let (tx, _commands) = tokio::sync::mpsc::unbounded_channel();
        let (events, _rx) = tokio::sync::broadcast::channel::<Event>(64);
        let handle = SessionHandle::for_tests(&tx, events);
        missions.attach(handle.clone());
        jobs.attach(handle);
        let tool = ProgramTool::new(
            missions,
            Arc::clone(&jobs),
            Store::open(None),
            ("map".to_owned(), "base_footprint".to_owned()),
        );
        (tool, jobs)
    }

    #[tokio::test]
    async fn a_program_is_approved_as_what_its_skills_can_do() {
        let (tool, _jobs) = tool();
        let source = r#"
            for r in rooms() {
                if walk(r.id) { run("PickObject", #{object_id: "O1", phrase: "mug", arm: "left"}); }
            }
        "#;
        let assessed = tool
            .assess(&json!({"action": "run", "title": "tidy", "source": source, "max_minutes": 15}))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(assessed.risk, Risk::Manipulation);
        assert!(
            assessed.reason.contains(
                "may move the arms (PickObject) and move the base (GoToPlace), for at most 15 min"
            ),
            "{}",
            assessed.reason
        );
        let dancing = tool
            .assess(&json!({"action": "run", "source": "run(\"Dance\", #{})"}))
            .await
            .unwrap()
            .unwrap_err();
        assert!(
            dancing.message.contains("no skill Dance"),
            "{}",
            dancing.message
        );
    }

    #[tokio::test]
    async fn a_program_that_only_reads_runs_and_reports_its_result() {
        let (tool, jobs) = tool();
        let source = "let types = []; for r in rooms() { types.push(r.type); } types";
        let assessed = tool
            .assess(&json!({"action": "run", "title": "rooms", "source": source}))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(assessed.risk, Risk::Observe, "nothing in it moves");
        let started = tool.call(assessed.args.unwrap()).await;
        assert_eq!(started.status, Status::Accepted, "{}", started.message);
        let id = started.data["job"]["id"].as_str().unwrap().to_owned();
        let mut ended = None;
        for _ in 0..250 {
            ended = jobs
                .list()
                .into_iter()
                .find(|j| j.id == id && j.state != JobState::Running);
            if ended.is_some() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let ended = ended.expect("the program never ended");
        assert_eq!(ended.state, JobState::Done, "{}", ended.summary);
        assert_eq!(ended.summary, r#"["kitchen","office"]"#);
    }
}
