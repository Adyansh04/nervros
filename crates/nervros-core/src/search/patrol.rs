//! The `patrol` tool: a round of places the robot walks as a job, standing a moment at each so the
//! world model sees it, then what changed in the world model since the round began: the objects
//! that moved or went missing, which no mission's goal check would name.

use std::borrow::Cow;
use std::fmt::Write as _;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use serde_json::{Value, json};

use crate::job::envelope::{DEFAULT_MINUTES, MAX_MINUTES};
use crate::job::{Approval, Context, Envelope, JobKind, Jobs};
use crate::mission::catalog::GO_TO_PLACE;
use crate::mission::{Missions, Outcome};
use crate::tools::{Assessment, Risk, Status, Tool, ToolOutcome, ToolSpec};

/// The most places one round walks.
const MAX_PLACES: usize = 8;
/// How long the robot stands at each place, so the world model sees it.
const DWELL: Duration = Duration::from_secs(4);

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
            "Walks a round of places on its own, standing a moment at each so the world model \
             sees it, then reports what moved or went missing meanwhile: for \"check on the \
             apartment\" or \"is everything where it should be?\". The places are names from \
             list_places, or room or object ids. Runs as a job; a report follows.",
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

/// Walks the round, then says what changed since it began.
async fn patrol(
    missions: &Arc<Missions>,
    ctx: &Context,
    places: &[String],
) -> Result<String, String> {
    let began = crate::now_s();
    let (mut visited, mut missed) = (Vec::new(), Vec::new());
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
            Outcome::Success => {
                visited.push(place.clone());
                tokio::select! {
                    () = ctx.until_stopped() => return Err("the patrol was stopped".to_owned()),
                    () = tokio::time::sleep(DWELL) => {}
                }
            }
            Outcome::Canceled => return Err("the patrol was stopped".to_owned()),
            _ => {
                ctx.log(format!("could not get to {place}: {}", end.reason));
                missed.push(place.clone());
            }
        }
    }
    let changes = missions.changes_since(began).await;
    let mut text = format!("visited {}", list(&visited));
    if !missed.is_empty() {
        let _ = write!(text, "; could not get to {}", list(&missed));
    }
    if changes.is_empty() {
        text.push_str("; nothing moved or went missing in the world model meanwhile");
    } else {
        let _ = write!(
            text,
            "; changed in the world model meanwhile: {}",
            changes.join("; ")
        );
    }
    Ok(text)
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
                    "patrols {} on its own, for at most {minutes} min, then says what changed",
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
        if catalog.skill(GO_TO_PLACE).is_none() {
            return ToolOutcome::failed("this robot has no GoToPlace skill to walk a round with");
        }
        let approval = Approval::of(Envelope::new([GO_TO_PLACE.to_owned()], minutes), &catalog);
        let missions = Arc::clone(&self.missions);
        let name = places.join(", ");
        let work = move |ctx: Context| async move { patrol(&missions, &ctx, &places).await };
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
