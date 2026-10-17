//! The `patrol` tool: a round of places the robot walks as a job, looking around at each (a full
//! turn on the spot, with the profile's turn skill) so the cameras sweep the room, then what it
//! saw in each room and what changed in the world model since the round began: the objects that
//! moved or went missing, which no mission's goal check would name.

use std::borrow::Cow;
use std::fmt::Write as _;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use serde_json::{Value, json};

use crate::job::envelope::{DEFAULT_MINUTES, MAX_MINUTES};
use crate::job::{Approval, Context, Envelope, JobKind, Jobs};
use crate::mission::catalog::GO_TO_PLACE;
use crate::mission::check::{self, Observed};
use crate::mission::{Missions, Outcome};
use crate::profile::TurnSkill;
use crate::tools::{Assessment, Risk, Status, Tool, ToolOutcome, ToolSpec};

/// The most places one round walks.
const MAX_PLACES: usize = 8;
/// How long the robot holds each view: a world model takes frames from a still base only (canopy
/// a second after it stops, then one a second), and its detector takes its own time.
const HOLD: Duration = Duration::from_secs(3);
/// A look around: the views of a full turn on the spot, a quarter turn apart.
const LOOK_VIEWS: usize = 4;
/// The turn between two views.
const LOOK_STEP_DEG: u32 = 90;
/// The most kinds of thing a room's line names; the rest are counted.
const MOST_KINDS: usize = 10;

/// The `patrol` tool.
pub struct PatrolTool {
    spec: ToolSpec,
    missions: Arc<Missions>,
    jobs: Arc<Jobs>,
}

impl PatrolTool {
    /// Rounds of `missions`' robot, run in `jobs`.
    #[must_use]
    pub fn new(missions: Arc<Missions>, jobs: Arc<Jobs>) -> Self {
        let spec = ToolSpec::new(
            "patrol",
            "Inspects a round of places on its own: walks to each, looks around it with a full \
             turn on the spot, then reports what it saw in each room and what moved or went \
             missing meanwhile: for \"check on the apartment\" or \"is everything where it \
             should be?\". The places are names from list_places, or room or object ids. Runs \
             as a job; a report follows.",
            json!({"type": "object", "properties": {
                "places": {"type": "array", "minItems": 1, "maxItems": MAX_PLACES, "items": {"type": "string"}, "description": "The round, in order."},
                "max_minutes": {"type": "integer", "minimum": 1, "maximum": MAX_MINUTES, "description": "How long the round may take."}
            }, "required": ["places"], "additionalProperties": false}),
            Risk::Motion,
        );
        Self {
            spec,
            missions,
            jobs,
        }
    }
}

/// The round's places and its time limit, from the arguments.
fn round(args: &Value) -> Result<(Vec<String>, u64), String> {
    let places: Vec<String> = args["places"]
        .as_array()
        .map(|a| {
            a.iter()
                .filter_map(Value::as_str)
                .map(str::trim)
                .filter(|p| !p.is_empty())
                .map(str::to_owned)
                .collect()
        })
        .unwrap_or_default();
    if places.is_empty() || places.len() > MAX_PLACES {
        return Err(format!(
            "`places` is the round, 1 to {MAX_PLACES} names from list_places or room or object \
             ids"
        ));
    }
    let minutes = args["max_minutes"]
        .as_u64()
        .unwrap_or(DEFAULT_MINUTES)
        .clamp(1, MAX_MINUTES);
    Ok((places, minutes))
}

/// Walks the round, looking around at each place, then says what it saw in each room and what
/// changed since it began.
async fn patrol(
    missions: &Arc<Missions>,
    ctx: &Context,
    places: &[String],
    turn: Option<&TurnSkill>,
) -> Result<String, String> {
    let began = crate::now_s();
    let (mut visited, mut missed, mut saw) = (Vec::new(), Vec::new(), Vec::new());
    for (n, place) in places.iter().enumerate() {
        ctx.log(format!("{}. {place}", n + 1));
        let steps = json!([{"skill": GO_TO_PLACE, "args": {"place": place}}]);
        let end = missions
            .run_for_job(ctx, &format!("patrol to {place}"), steps)
            .await
            .map_err(|e| format!("the patrol could not go on: {e}"))?;
        if let Some(fault) = end.fault {
            return Err(format!(
                "the patrol stopped at {place}, cut short by the robot ({}): {}",
                fault.as_str(),
                end.reason
            ));
        }
        match end.outcome {
            Outcome::Success => visited.push(place.clone()),
            Outcome::Canceled => return Err("the patrol was stopped".to_owned()),
            _ => {
                ctx.log(format!("could not get to {place}: {}", end.reason));
                missed.push(place.clone());
                continue;
            }
        }
        let arrived = crate::now_s();
        match turn {
            Some(turn) => look_around(missions, ctx, place, turn).await?,
            None => hold(ctx).await?,
        }
        let seen = missions.observe().await;
        if let Some(line) = seen_from_here(&seen, arrived) {
            ctx.log(line.clone());
            saw.push(line);
        }
    }
    let mut text = format!("visited {}", list(&visited));
    if turn.is_none() {
        text.push_str(" without looking around: the profile names no turn on the spot");
    }
    if !missed.is_empty() {
        let _ = write!(text, "; could not get to {}", list(&missed));
    }
    if !saw.is_empty() {
        let _ = write!(text, "; saw {}", saw.join("; "));
    }
    match missions.changes_since(began).await {
        Ok(changes) if changes.is_empty() => {
            text.push_str("; nothing moved or went missing in the world model meanwhile");
        }
        Ok(changes) => {
            let _ = write!(
                text,
                "; changed in the world model meanwhile: {}",
                changes.join("; ")
            );
        }
        Err(e) => {
            let _ = write!(text, "; what changed is unknown: {e}");
        }
    }
    Ok(text)
}

/// The views of a full turn on the spot at `place`, each held still: a turn mid-way is no view to
/// the world model. A turn that fails ends the look and the round goes on: the place was reached,
/// and the views taken still count.
async fn look_around(
    missions: &Arc<Missions>,
    ctx: &Context,
    place: &str,
    turn: &TurnSkill,
) -> Result<(), String> {
    let quarter = json!([{"skill": turn.skill, "args": {turn.degrees.as_str(): LOOK_STEP_DEG}}]);
    for view in 0..LOOK_VIEWS {
        if view > 0 {
            let end = missions
                .run_for_job(ctx, &format!("look around at {place}"), quarter.clone())
                .await
                .map_err(|e| format!("the patrol could not go on: {e}"))?;
            if let Some(fault) = end.fault {
                return Err(format!(
                    "the patrol stopped at {place}, cut short by the robot ({}): {}",
                    fault.as_str(),
                    end.reason
                ));
            }
            match end.outcome {
                Outcome::Success => {}
                Outcome::Canceled => return Err("the patrol was stopped".to_owned()),
                _ => {
                    ctx.log(format!("could not look around at {place}: {}", end.reason));
                    return Ok(());
                }
            }
        }
        hold(ctx).await?;
    }
    Ok(())
}

/// Stands still for [`HOLD`], unless the patrol is stopped.
async fn hold(ctx: &Context) -> Result<(), String> {
    tokio::select! {
        () = ctx.until_stopped() => Err("the patrol was stopped".to_owned()),
        () = tokio::time::sleep(HOLD) => Ok(()),
    }
}

/// What the cameras showed the world model since `since_s`, room by room, the robot's own first:
/// "the bedroom (R3): a bed and 2 nightstands, and in view the study (R7): a wardrobe". None when
/// nothing was seen.
fn seen_from_here(seen: &Observed, since_s: f64) -> Option<String> {
    let mut rooms: Vec<(&str, Vec<(&str, usize)>)> = Vec::new();
    for o in seen.objects["objects"].as_array()? {
        let label = o["label"].as_str().unwrap_or_default();
        let at = o["last_seen"]["sec"].as_f64().unwrap_or(0.0)
            + o["last_seen"]["nanosec"].as_f64().unwrap_or(0.0) * 1e-9;
        if label.is_empty() || o["state"].as_u64() != Some(0) || at < since_s {
            continue;
        }
        let room = o["room_id"].as_str().unwrap_or_default();
        let at_room = rooms
            .iter()
            .position(|(r, _)| *r == room)
            .unwrap_or_else(|| {
                rooms.push((room, Vec::new()));
                rooms.len() - 1
            });
        let kinds = &mut rooms[at_room].1;
        match kinds.iter_mut().find(|(k, _)| *k == label) {
            Some((_, n)) => *n += 1,
            None => kinds.push((label, 1)),
        }
    }
    let here = check::room_of_robot(seen).unwrap_or_default();
    // The robot's room first, then the busiest others.
    rooms.sort_by_key(|(room, kinds)| {
        (
            *room != here,
            std::cmp::Reverse(kinds.iter().map(|(_, n)| n).sum::<usize>()),
        )
    });
    let lines: Vec<String> = rooms
        .iter()
        .map(|(room, kinds)| {
            let name = if room.is_empty() {
                "outside the rooms".to_owned()
            } else {
                check::room_said(seen, room)
            };
            format!("{name}: {}", things(kinds))
        })
        .collect();
    let (first, rest) = lines.split_first()?;
    Some(if rest.is_empty() {
        first.clone()
    } else {
        format!("{first}, and in view {}", rest.join(", "))
    })
}

/// Kinds of thing with how many of each, the commonest first, as a person names a room's furniture
/// before its cups: "2 chairs, an armchair and a bed".
fn things(kinds: &[(&str, usize)]) -> String {
    let mut kinds = kinds.to_vec();
    kinds.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(b.0)));
    let mut said: Vec<String> = kinds
        .iter()
        .take(MOST_KINDS)
        .map(|(label, n)| match n {
            1 if label.starts_with(['a', 'e', 'i', 'o', 'u']) => format!("an {label}"),
            1 => format!("a {label}"),
            _ => crate::counted(*n, label),
        })
        .collect();
    if kinds.len() > MOST_KINDS {
        said.push(format!("{} more kinds", kinds.len() - MOST_KINDS));
    }
    match said.pop() {
        Some(last) if said.is_empty() => last,
        Some(last) => format!("{} and {last}", said.join(", ")),
        None => String::new(),
    }
}

fn list(places: &[String]) -> String {
    if places.is_empty() {
        "nothing".to_owned()
    } else {
        places.join(", ")
    }
}

#[async_trait]
impl Tool for PatrolTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    /// Approved once with its round and its time limit.
    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        Some(match round(args) {
            Ok((places, minutes)) => Ok(Assessment {
                risk: Risk::Motion,
                resources: Vec::new(),
                reason: format!(
                    "patrols {} on its own, looking around at each, for at most {minutes} min, \
                     then says what it saw and what changed",
                    places.join(", ")
                ),
                args: Some(json!({"places": places, "max_minutes": minutes})),
            }),
            Err(e) => Err(ToolOutcome::failed(e)),
        })
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let (places, minutes) = match round(&args) {
            Ok(r) => r,
            Err(e) => return ToolOutcome::failed(e),
        };
        let catalog = match self.missions.catalog().await {
            Ok(c) => c,
            Err(e) => return ToolOutcome::failed(e),
        };
        // GoToPlace is the planner's, folded over the navigation skills the catalog has.
        if !catalog.can_travel() {
            return ToolOutcome::failed(
                "this robot cannot walk a round: it has no navigation skill",
            );
        }
        // A look around needs the profile's turn on the spot, approved with the walks.
        let turn = self
            .missions
            .turn_skill()
            .filter(|t| catalog.skill(&t.skill).is_some())
            .cloned();
        let skills =
            std::iter::once(GO_TO_PLACE.to_owned()).chain(turn.iter().map(|t| t.skill.clone()));
        let approval = Approval::of(Envelope::new(skills, minutes), &catalog);
        let missions = Arc::clone(&self.missions);
        let name = places.join(", ");
        let work = move |ctx: Context| async move {
            patrol(&missions, &ctx, &places, turn.as_ref()).await
        };
        match self.jobs.spawn(JobKind::Patrol, &name, approval, work) {
            Ok(job) => ToolOutcome {
                status: Status::Accepted,
                message: format!(
                    "patrol {} runs on its own; its lines show where it is, and a report follows. \
                     Tell the operator it started.",
                    job.id
                ),
                data: json!({"job": job}),
                images: Vec::new(),
            },
            Err(e) => ToolOutcome::refused(e),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_place_reads_as_what_was_seen_from_it_room_by_room_its_own_first() {
        let seen_at = |sec: f64| json!({"sec": sec, "nanosec": 0});
        let thing = |id: &str, label: &str, room: &str, sec: f64| json!({"id": id, "label": label, "room_id": room, "state": 0, "last_seen": seen_at(sec)});
        let square = |x0: f64| {
            json!({"points": [{"x": x0, "y": 0.0}, {"x": x0 + 4.0, "y": 0.0},
                              {"x": x0 + 4.0, "y": 4.0}, {"x": x0, "y": 4.0}]})
        };
        let mut seen = Observed {
            pose: Some((1.0, 1.0)),
            rooms: json!({"rooms": [
                {"id": "R7", "name": "room G", "type": "study", "outline": square(4.0)},
                {"id": "R3", "name": "room C", "type": "bedroom", "outline": square(0.0)}]}),
            objects: json!({"objects": [
                thing("O1", "chair", "R3", 120.0),
                thing("O2", "chair", "R3", 130.0),
                thing("O3", "bed", "R3", 125.0),
                thing("O4", "armchair", "R3", 121.0),
                thing("O5", "nightstand", "R3", 50.0),
                thing("O6", "wardrobe", "R7", 130.0)]}),
            ..Observed::default()
        };
        assert_eq!(
            seen_from_here(&seen, 100.0).as_deref(),
            Some(
                "the bedroom (R3): 2 chairs, an armchair and a bed, and in view the study (R7): \
                 a wardrobe"
            ),
            "not the nightstand, seen before the robot came"
        );
        assert_eq!(seen_from_here(&seen, 200.0), None, "nothing seen since");
        seen.pose = Some((5.0, 1.0));
        assert!(
            seen_from_here(&seen, 100.0)
                .unwrap()
                .starts_with("the study (R7): a wardrobe, and in view the bedroom (R3)"),
            "the robot's own room first"
        );
    }

    #[test]
    fn a_round_takes_one_to_eight_places_and_a_bounded_time() {
        let (places, minutes) = round(&json!({"places": [" R1 ", "", "R3"]})).unwrap();
        assert_eq!(places, ["R1", "R3"]);
        assert_eq!(minutes, DEFAULT_MINUTES);
        assert!(round(&json!({"places": []})).is_err());
        assert!(round(&json!({ "places": vec!["a"; 9] })).is_err());
        assert_eq!(
            round(&json!({"places": ["R1"], "max_minutes": 100_000}))
                .unwrap()
                .1,
            MAX_MINUTES
        );
    }
}
