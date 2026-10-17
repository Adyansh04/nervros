//! The `search` tool: where to look for something, and a search that walks to each place and
//! looks until the detector sees it.

use std::borrow::Cow;
use std::fmt::Write as _;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::rank::{Lookout, Seen, bare, label_word, rank};
use crate::job::envelope::{DEFAULT_MINUTES, MAX_MINUTES};
use crate::job::{Approval, Context, Envelope, JobKind, Jobs};
use crate::mission::catalog::{Catalog, FIND_IN_VIEW, GO_TO_PLACE};
use crate::mission::{Missions, Outcome};
use crate::tools::{Assessment, Risk, Status, Tool, ToolOutcome, ToolSpec};

const DEFAULT_PLACES: u64 = 4;
const MAX_PLACES: u64 = 8;
/// Lookouts the approval names before "and N more".
const NAMED: usize = 3;

/// Says where to look, and searches.
pub struct SearchTool {
    spec: ToolSpec,
    missions: Arc<Missions>,
    jobs: Arc<Jobs>,
    /// The map frame and the robot's base frame.
    frames: (String, String),
}

/// A search, as checked: what to look for, the detector's word for it, and where.
struct Plan {
    what: String,
    /// What the detector looks for, or why this robot cannot look for it: a plan that moves
    /// nothing still says where it would be.
    phrase: Result<String, String>,
    lookouts: Vec<Lookout>,
    minutes: u64,
}

impl SearchTool {
    /// The tool for `missions`' robot, its searches run in `jobs`.
    #[must_use]
    pub fn new(missions: Arc<Missions>, jobs: Arc<Jobs>, frames: (String, String)) -> Self {
        Self {
            spec: spec(),
            missions,
            jobs,
            frames,
        }
    }

    /// Where to look for what `args` names, best first, and the detector's word for it.
    async fn plan(&self, args: &Value) -> Result<Plan, String> {
        let what = args["what"].as_str().unwrap_or_default().trim().to_owned();
        if what.is_empty() {
            return Err(
                "`what` is the thing to look for, such as \"keys\" or \"red mug\"".to_owned(),
            );
        }
        let catalog = self.missions.catalog().await?;
        let phrase = phrase_for(&catalog, &what);
        // The world model's labels are the detector's words, not the operator's "my keys".
        let named = phrase.clone().unwrap_or_else(|_| bare(&what));
        let (seen, events) = tokio::join!(
            self.missions.observe(),
            self.missions.object_events(label_word(&named), 0.0)
        );
        if seen.rooms.is_null() && seen.objects.is_null() {
            return Err(
                "the world model has not said what it knows; it may not be running".to_owned(),
            );
        }
        let (map, base) = &self.frames;
        let robot = self.missions.robot().transform(map, base).ok().map(|t| {
            let (x, y, _) = t.planar();
            (x, y)
        });
        let ranked = rank(&Seen {
            words: &what,
            objects: &seen.objects,
            rooms: &seen.rooms,
            events: &events,
            robot,
            now_s: crate::now_s(),
        });
        if ranked.is_empty() {
            return Err(
                "the world model knows no rooms to look in; map the building first".to_owned(),
            );
        }
        let places = args["max_places"]
            .as_u64()
            .unwrap_or(DEFAULT_PLACES)
            .clamp(1, MAX_PLACES);
        let lookouts = ranked
            .into_iter()
            .take(usize::try_from(places).unwrap_or(usize::MAX))
            .collect();
        Ok(Plan {
            what,
            phrase,
            lookouts,
            minutes: args["max_minutes"].as_u64().unwrap_or(DEFAULT_MINUTES),
        })
    }

    async fn go(&self, args: &Value) -> ToolOutcome {
        let plan = match self.plan(args).await {
            Ok(p) => p,
            Err(e) => return ToolOutcome::failed(e),
        };
        if let Err(e) = &plan.phrase {
            return ToolOutcome::failed(e.clone());
        }
        let catalog = match self.missions.catalog().await {
            Ok(c) => c,
            Err(e) => return ToolOutcome::failed(e),
        };
        let envelope = Envelope::new(
            [GO_TO_PLACE.to_owned(), FIND_IN_VIEW.to_owned()],
            plan.minutes,
        );
        let approval = Approval::of(envelope, &catalog);
        let missions = Arc::clone(&self.missions);
        let name = plan.what.clone();
        let work = move |ctx: Context| async move { search(&missions, &ctx, plan).await };
        match self.jobs.spawn(JobKind::Search, &name, approval, work) {
            Ok(job) => ToolOutcome {
                status: Status::Accepted,
                message: format!(
                    "search {} runs on its own; its lines show where it looks, and a report \
                     follows. Tell the operator it started.",
                    job.id
                ),
                data: json!({"job": job}),
                images: Vec::new(),
            },
            Err(e) => ToolOutcome::refused(e),
        }
    }
}

/// The detector's word for `what`: itself, or the one word of a narrowed list it holds.
fn phrase_for(catalog: &Catalog, what: &str) -> Result<String, String> {
    let skill = catalog
        .skill(FIND_IN_VIEW)
        .ok_or("this robot has no FindInView skill to look with")?;
    if !catalog.can_travel() {
        return Err("this robot cannot walk anywhere to look".to_owned());
    }
    let choices = skill
        .args
        .iter()
        .find(|a| a.name == "phrase")
        .map(|a| a.choices.as_slice())
        .unwrap_or_default();
    if choices.is_empty() {
        return Ok(bare(what));
    }
    let lower = what.to_lowercase();
    choices
        .iter()
        .find(|c| lower.contains(&c.to_lowercase()))
        .cloned()
        .ok_or_else(|| {
            format!(
                "the detector here can look only for {}; ask for one of those",
                choices.join(", ")
            )
        })
}

/// Walks to each lookout in turn and looks, until the detector sees the thing.
async fn search(missions: &Arc<Missions>, ctx: &Context, plan: Plan) -> Result<String, String> {
    let mut looked = Vec::new();
    for (n, l) in plan.lookouts.iter().enumerate() {
        ctx.log(format!("{}. {}: {}", n + 1, l.target, l.why));
        let steps = json!([
            {"skill": GO_TO_PLACE, "args": {"place": l.target}},
            {"skill": FIND_IN_VIEW, "args": {"phrase": plan.phrase.as_deref().unwrap_or_default()}}
        ]);
        let intent = format!("look for {} at {}", plan.what, l.target);
        let end = missions
            .run_for_job(ctx, &intent, steps)
            .await
            // The robot cannot run missions now: looking further would fail the same way.
            .map_err(|e| format!("the search could not go on: {e}"))?;
        match (end.outcome, end.failed_step.as_str()) {
            (Outcome::Success, _) => {
                let id = end
                    .outputs
                    .iter()
                    .find(|(k, _)| k.ends_with("_object_id"))
                    .and_then(|(_, v)| v.as_str())
                    .map(|id| format!("; the detector calls it {id}"))
                    .unwrap_or_default();
                return Ok(format!("found {} at {}{id}", plan.what, l.target));
            }
            (Outcome::Failure | Outcome::Timeout, "s1") => {
                ctx.log(format!("could not get to {}: {}", l.target, end.reason));
                looked.push(format!("{} (not reached)", l.target));
            }
            (Outcome::Failure | Outcome::Timeout, "s2") => {
                ctx.log(format!("not seen at {}", l.target));
                looked.push(l.target.clone());
            }
            // A cancel names the step it stopped too: wherever it came, it ends the search.
            (Outcome::Canceled, _) => return Err("the search was stopped".to_owned()),
            (other, _) => {
                return Err(format!(
                    "the search could not go on: its mission ended {other}: {}",
                    end.reason
                ));
            }
        }
    }
    let mut text = format!(
        "did not find {}; looked at {}",
        plan.what,
        looked.join(", ")
    );
    let _ = write!(text, " ({} places)", looked.len());
    Err(text)
}

/// Lookouts as the model reads them: why look there names what the world model calls things,
/// which is world text.
fn fenced(lookouts: Vec<Lookout>) -> Vec<Lookout> {
    lookouts
        .into_iter()
        .map(|l| Lookout {
            why: crate::tools::from_world(&l.why),
            ..l
        })
        .collect()
}

/// The places a search would look, as the operator reads them.
fn describe(plan: &Plan) -> String {
    let mut text = plan
        .lookouts
        .iter()
        .take(NAMED)
        .map(|l| format!("{} ({})", l.target, l.why))
        .collect::<Vec<_>>()
        .join(", then ");
    if plan.lookouts.len() > NAMED {
        let _ = write!(text, ", and {} more", plan.lookouts.len() - NAMED);
    }
    text
}

fn spec() -> ToolSpec {
    ToolSpec::new(
        "search",
        &format!(
            "Finds something the operator lost or wants: \"find my keys\", \"look for the \
             tray\", \"go and find the white mug\" are action=go, which walks to each likely place \
             and looks with the detector, up to max_places ({DEFAULT_PLACES} by default) within \
             max_minutes ({DEFAULT_MINUTES}), until it sees it, and reports where. Places come best \
             first: where the world model saw it, where it went since, then the rooms such things \
             are usually in, the least seen first. The operator approves a search once. Only when \
             the operator asks where it could be without moving is it action=plan, which lists the \
             places and moves nothing. For where something was last seen, recall answers at once."
        ),
        json!({"type": "object", "properties": {
            "action": {"type": "string", "enum": ["plan", "go"]},
            "what": {"type": "string", "description": "The thing, in the operator's words."},
            "max_places": {"type": "integer", "minimum": 1, "maximum": MAX_PLACES},
            "max_minutes": {"type": "integer", "minimum": 1, "maximum": MAX_MINUTES}
        }, "required": ["action", "what"], "additionalProperties": false}),
        Risk::Motion,
    )
}

#[async_trait]
impl Tool for SearchTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    /// Planning only reads; going is approved with the places it will look, in order.
    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        match args["action"].as_str() {
            Some("plan") => Some(Ok(Assessment {
                risk: Risk::Observe,
                resources: Vec::new(),
                reason: "says where it would look".to_owned(),
                args: None,
            })),
            Some("go") => Some(match self.plan(args).await {
                Ok(Plan { phrase: Err(e), .. }) | Err(e) => Err(ToolOutcome::failed(e)),
                Ok(plan) => Ok(Assessment {
                    risk: Risk::Motion,
                    resources: Vec::new(),
                    reason: format!(
                        "searches for \"{}\" on its own, for at most {} min, looking at each \
                         place it walks to: {}",
                        plan.what,
                        plan.minutes.clamp(1, MAX_MINUTES),
                        describe(&plan)
                    ),
                    args: Some(json!({"action": "go", "what": plan.what,
                                      "max_places": plan.lookouts.len(),
                                      "max_minutes": plan.minutes})),
                }),
            }),
            _ => Some(Err(ToolOutcome::failed("`action` is plan or go"))),
        }
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str() {
            Some("plan") => match self.plan(&args).await {
                Ok(plan) => ToolOutcome {
                    status: Status::Succeeded,
                    message: match &plan.phrase {
                        Ok(_) => "where it would look, best first; nothing moved. To look, call \
                                  search with action go."
                            .to_owned(),
                        // Where it would be still helps the operator look themselves.
                        Err(e) => format!(
                            "where it would be, best first; nothing moved, and the robot cannot \
                             look for it itself: {e}"
                        ),
                    },
                    data: json!({"what": plan.what, "look_at": fenced(plan.lookouts)}),
                    images: Vec::new(),
                },
                Err(e) => ToolOutcome::failed(e),
            },
            Some("go") => self.go(&args).await,
            _ => ToolOutcome::failed("`action` is plan or go"),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use nervros_ros::RobotPort;
    use nervros_ros::fake::FakeRobot;

    use super::*;
    use crate::guard::{Guard, Policy};
    use crate::places::Places;
    use crate::profile::Profile;

    const SKILLS: &str = r#"{"catalog_version": "v", "skills": [
        {"name": "GoToTarget", "resources": ["base"], "max_duration_s": 60, "template": "GoToTarget",
         "args": [{"name": "target", "type": "world_id", "format": "phrase"}]},
        {"name": "FindInView", "resources": [], "max_duration_s": 10, "template": "FindInView",
         "args": [{"name": "phrase", "type": "string", "enum": ["mug", "tray"]}],
         "outputs": [{"name": "object_id", "type": "string"}]}]}"#;

    fn tool() -> SearchTool {
        let profile = Profile::from_toml(
            r#"
            [robot]
            name = "t"
            [world]
            rooms = { topic = "/rooms", type = "canopy_msgs/msg/RoomArray" }
            objects = { topic = "/objects", type = "canopy_msgs/msg/WorldObjectArray" }
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
        let now = crate::now_s();
        let robot: Arc<dyn RobotPort> = Arc::new(
            FakeRobot::new()
                .with_service("/x/catalog", |_| Ok(json!({"catalog_json": SKILLS})))
                .with_topic("/rooms", json!({"rooms": [
                    {"id": "R1", "type": "kitchen", "centroid": {"x": 1.0, "y": 0.0}, "floor_coverage": 0.9},
                    {"id": "R2", "type": "office", "centroid": {"x": 6.0, "y": 0.0}, "floor_coverage": 0.3}]}))
                .with_topic("/objects", json!({"objects": [
                    {"id": "O7", "label": "mug", "room_id": "R2", "state": 0,
                     "pose": {"position": {"x": 6.5, "y": 0.5}}, "last_seen": {"sec": now - 120.0}}]})),
        );
        let missions = Missions::new(&profile, Places::new(&profile, None), robot).unwrap();
        let jobs = Jobs::new(Arc::new(Guard::new(Policy::default())));
        SearchTool::new(
            missions,
            jobs,
            ("map".to_owned(), "base_footprint".to_owned()),
        )
    }

    #[tokio::test]
    async fn a_search_is_approved_with_where_it_looks_in_order() {
        let tool = tool();
        let assessed = tool
            .assess(&json!({"action": "go", "what": "the red mug", "max_places": 3}))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(assessed.risk, Risk::Motion);
        assert!(
            assessed.reason.starts_with(
                "searches for \"the red mug\" on its own, for at most 10 min, looking at each place it walks to: O7 (a mug was seen there 2 min ago), then R2"
            ),
            "{}",
            assessed.reason
        );
        let planned = tool.call(json!({"action": "plan", "what": "mug"})).await;
        assert_eq!(planned.status, Status::Succeeded, "{}", planned.message);
        let targets: Vec<&str> = planned.data["look_at"]
            .as_array()
            .unwrap()
            .iter()
            .filter_map(|l| l["target"].as_str())
            .collect();
        // The office, little of it seen, before the kitchen, nearly all of it seen.
        assert_eq!(targets, ["O7", "R2", "R1"]);
        let refused = tool
            .assess(&json!({"action": "go", "what": "keys"}))
            .await
            .unwrap()
            .unwrap_err();
        assert!(
            refused.message.contains("only for mug, tray"),
            "{}",
            refused.message
        );
        // Where keys might be still helps, though the detector here cannot look for them.
        let keys = tool
            .call(json!({"action": "plan", "what": "my keys"}))
            .await;
        assert_eq!(keys.status, Status::Succeeded, "{}", keys.message);
        assert!(
            keys.message.contains("cannot look for it itself"),
            "{}",
            keys.message
        );
        assert!(!keys.data["look_at"].as_array().unwrap().is_empty());
    }

    fn catalog(phrase_choices: &str) -> Catalog {
        Catalog::parse(&format!(
            r#"{{"catalog_version": "v", "skills": [
                {{"name": "GoToTarget", "resources": ["base"], "max_duration_s": 60, "template": "GoToTarget"}},
                {{"name": "FindInView", "resources": [], "max_duration_s": 10, "template": "FindInView",
                  "args": [{{"name": "phrase", "type": "string", "enum": [{phrase_choices}]}}]}}
            ]}}"#
        ))
        .unwrap()
    }

    #[test]
    fn the_detector_word_is_the_one_its_list_holds() {
        assert_eq!(phrase_for(&catalog(""), "my red mug").unwrap(), "red mug");
        let narrowed = catalog(r#""mug", "tray""#);
        assert_eq!(phrase_for(&narrowed, "the Red MUG").unwrap(), "mug");
        let refused = phrase_for(&narrowed, "keys").unwrap_err();
        assert!(refused.contains("only for mug, tray"), "{refused}");
    }
}
