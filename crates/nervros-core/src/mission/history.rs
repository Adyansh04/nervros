//! What the robot has done and could not do, from the mission ledger and the world model:
//! `recall` for past missions, where things were, and plans that worked before; `plans` for
//! plans saved by name; `skill_gap` for requests no skill covers.

use std::borrow::Cow;
use std::fmt::Write as _;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::Missions;
use super::check::{describe_seen, find_object, how_long};
use super::ledger::{Ledger, MissionRecord};
use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};

/// How many missions `recall` lists.
const RECENT: usize = 8;
/// How many earlier missions it searches for plans that worked.
const SEARCHED: usize = 200;
/// How many such plans it gives.
const PROVEN: usize = 3;
/// How many of the robot's missions with an object it gives, as a failure report does.
const MOVES: usize = 2;

/// The three tools.
#[must_use]
pub fn tools(missions: &Arc<Missions>) -> [Arc<dyn Tool>; 3] {
    [
        Arc::new(Recall {
            spec: recall_spec(),
            missions: Arc::clone(missions),
        }),
        Arc::new(Plans {
            spec: plans_spec(),
            missions: Arc::clone(missions),
        }),
        Arc::new(SkillGap {
            spec: gap_spec(),
            missions: Arc::clone(missions),
        }),
    ]
}

/// Runs a ledger read off the async threads.
async fn read<T: Send + 'static>(
    missions: &Missions,
    f: impl FnOnce(&Ledger) -> rusqlite::Result<T> + Send + 'static,
) -> Result<T, String> {
    let ledger = missions
        .ledger()
        .cloned()
        .ok_or("nothing is kept on this robot: it has no mission ledger")?;
    tokio::task::spawn_blocking(move || f(&ledger))
        .await
        .map_err(|e| e.to_string())?
        .map_err(|e| e.to_string())
}

/// The words of a request that say what it is about, for matching it against earlier ones.
fn words(text: &str) -> Vec<String> {
    const SMALL: [&str; 12] = [
        "the", "and", "please", "from", "into", "onto", "with", "then", "that", "this", "can",
        "you",
    ];
    text.split(|c: char| !c.is_alphanumeric())
        .map(str::to_lowercase)
        .filter(|w| w.len() > 2 && !SMALL.contains(&w.as_str()))
        .collect()
}

/// A step of a recorded mission as a planner writes it: `PickObject(object_id=mug_4)`.
fn step_text(skill: &str, args: &Value) -> String {
    let args: Vec<String> = args
        .as_object()
        .map(|a| {
            a.iter()
                .map(|(k, v)| format!("{k}={}", v.as_str().unwrap_or_default()))
                .collect()
        })
        .unwrap_or_default();
    format!("{skill}({})", args.join(", "))
}

fn mission_line(m: &MissionRecord, now_s: f64) -> String {
    let mut line = format!(
        "{} ago: {} ({})",
        how_long(now_s - m.started),
        m.intent,
        m.outcome
    );
    if !m.failed_step.is_empty() {
        let _ = write!(line, " at {}: {}", m.failed_step, m.reason);
    }
    line
}

/// The robot's own successful missions that named `what` (an id such as `mug_4`), newest first:
/// a line each, with its steps from the first that named it. They say where the robot took it
/// when the world model cannot tell which of its mugs `mug_4` is.
pub(super) async fn moves_of(missions: &Missions, what: &str) -> Vec<String> {
    let now_s = crate::now_s();
    let Ok(recent) = read(missions, |l| l.recent(SEARCHED)).await else {
        return Vec::new();
    };
    recent
        .iter()
        .filter(|m| m.outcome == "success")
        .filter_map(|m| {
            let from = m.steps.iter().position(|s| {
                s.args.as_object().is_some_and(|a| {
                    a.values()
                        .any(|v| v.as_str().is_some_and(|v| v.eq_ignore_ascii_case(what)))
                })
            })?;
            let steps: Vec<String> = m.steps[from..]
                .iter()
                .map(|s| step_text(&s.skill, &s.args))
                .collect();
            Some(format!("{}: {}", mission_line(m, now_s), steps.join(", ")))
        })
        .take(MOVES)
        .collect()
}

fn recall_spec() -> ToolSpec {
    ToolSpec::new(
        "recall",
        "What the robot remembers: about=missions lists what it did lately, and how each \
         ended; about=object says where and when the world model last saw something, and \
         which of the robot's missions took it (query: its name or id); about=plans gives \
         plans that worked before for a request like the query, to plan from.",
        json!({"type": "object", "properties": {
            "about": {"type": "string", "enum": ["missions", "object", "plans"]},
            "query": {"type": "string", "description": "For object: the thing; for plans: the request; for missions: optional words to filter by."}
        }, "required": ["about"], "additionalProperties": false}),
        Risk::Observe,
    )
}

/// `recall`.
struct Recall {
    spec: ToolSpec,
    missions: Arc<Missions>,
}

#[async_trait]
impl Tool for Recall {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let query = args["query"].as_str().unwrap_or_default().trim().to_owned();
        let now_s = crate::now_s();
        match args["about"].as_str() {
            Some("object") => {
                if query.is_empty() {
                    return ToolOutcome::failed("`query` names the object");
                }
                let (seen, story, moves) = tokio::join!(
                    self.missions.observe(),
                    self.missions.object_story(&query),
                    moves_of(&self.missions, &query)
                );
                let found = find_object(&seen, &[query.as_str()], Some(&query));
                if found.is_none() && story.is_empty() && moves.is_empty() {
                    return ToolOutcome::failed(format!(
                        "the world model knows nothing called \"{query}\"; find_objects searches it"
                    ));
                }
                let mut out = json!({});
                if let Some(found) = found {
                    out["seen"] = json!(describe_seen(found, &seen, now_s));
                }
                if !story.is_empty() {
                    out["history"] = json!(story);
                }
                if !moves.is_empty() {
                    out["missions"] = json!(moves);
                }
                ToolOutcome::ok(out)
            }
            Some("plans") => {
                let wanted = words(&query);
                match read(&self.missions, |l| l.recent(SEARCHED)).await {
                    Ok(missions) => {
                        let mut scored: Vec<(usize, &MissionRecord)> = missions
                            .iter()
                            .filter(|m| m.outcome == "success")
                            .map(|m| {
                                let said = words(&format!("{} {}", m.request, m.intent));
                                (wanted.iter().filter(|w| said.contains(w)).count(), m)
                            })
                            .filter(|(score, _)| *score > 0)
                            .collect();
                        // Best match first; among equals, the most recent (the list is newest first).
                        scored.sort_by_key(|s| std::cmp::Reverse(s.0));
                        let plans: Vec<String> = scored
                            .iter()
                            .take(PROVEN)
                            .map(|(_, m)| {
                                let steps: Vec<String> = m
                                    .steps
                                    .iter()
                                    .map(|s| step_text(&s.skill, &s.args))
                                    .collect();
                                format!(
                                    "{} ago, \"{}\" worked: {}",
                                    how_long(now_s - m.started),
                                    m.intent,
                                    steps.join(" -> ")
                                )
                            })
                            .collect();
                        if plans.is_empty() {
                            ToolOutcome::ok(
                                json!({"plans": [], "note": "no earlier plan for this worked"}),
                            )
                        } else {
                            ToolOutcome::ok(json!({"plans": plans}))
                        }
                    }
                    Err(e) => ToolOutcome::failed(e),
                }
            }
            Some("missions") => match read(&self.missions, |l| l.recent(SEARCHED)).await {
                Ok(missions) => {
                    let wanted = words(&query);
                    let lines: Vec<String> = missions
                        .iter()
                        .filter(|m| {
                            wanted.is_empty()
                                || words(&format!("{} {}", m.request, m.intent))
                                    .iter()
                                    .any(|w| wanted.contains(w))
                        })
                        .take(RECENT)
                        .map(|m| mission_line(m, now_s))
                        .collect();
                    ToolOutcome::ok(json!({"missions": lines}))
                }
                Err(e) => ToolOutcome::failed(e),
            },
            _ => ToolOutcome::failed("`about` is missions, object or plans"),
        }
    }
}

fn plans_spec() -> ToolSpec {
    ToolSpec::new(
        "plans",
        "Plans saved by name, such as \"evening check\": action=save keeps a plan checked \
         before (its hash) under a name; list shows them; forget removes one. To run one, \
         call run_mission with template set to its name; the operator approves it then.",
        json!({"type": "object", "properties": {
            "action": {"type": "string", "enum": ["save", "list", "forget"]},
            "name": {"type": "string", "description": "For save and forget."},
            "hash": {"type": "string", "description": "For save: a plan run_mission checked."}
        }, "required": ["action"], "additionalProperties": false}),
        Risk::Annotate,
    )
}

/// `plans`: plans saved by name. Running one is `run_mission` with `template`.
struct Plans {
    spec: ToolSpec,
    missions: Arc<Missions>,
}

#[async_trait]
impl Tool for Plans {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let observe = args["action"].as_str() == Some("list");
        Some(Ok(Assessment {
            risk: if observe {
                Risk::Observe
            } else {
                Risk::Annotate
            },
            resources: Vec::new(),
            reason: if observe {
                "lists the saved plans".to_owned()
            } else {
                format!(
                    "{}s the plan \"{}\"",
                    args["action"].as_str().unwrap_or("change"),
                    args["name"].as_str().unwrap_or_default()
                )
            },
            args: None,
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let name = args["name"].as_str().unwrap_or_default().trim().to_owned();
        match args["action"].as_str() {
            Some("save") => {
                if name.is_empty() {
                    return ToolOutcome::failed("`name` is what to call the plan");
                }
                let compiled = match self
                    .missions
                    .find(args["hash"].as_str().unwrap_or_default())
                {
                    Ok(c) => c,
                    Err(e) => return ToolOutcome::failed(e),
                };
                let steps = json!(compiled.plan.steps);
                let intent = compiled.plan.intent.clone();
                let saved = name.clone();
                match read(&self.missions, move |l| {
                    l.save_template(&saved, &intent, &steps)
                })
                .await
                {
                    Ok(()) => ToolOutcome::ok(json!({"saved": name})),
                    Err(e) => ToolOutcome::failed(e),
                }
            }
            Some("forget") => match read(&self.missions, move |l| l.forget_template(&name)).await {
                Ok(true) => ToolOutcome::ok(json!({"forgot": args["name"]})),
                Ok(false) => ToolOutcome::failed("no plan is saved under that name; list them"),
                Err(e) => ToolOutcome::failed(e),
            },
            Some("list") => match read(&self.missions, Ledger::templates).await {
                Ok(all) => ToolOutcome::ok(json!({"plans": all
                    .iter()
                    .map(|t| json!({"name": t.name, "intent": t.intent, "runs": t.runs}))
                    .collect::<Vec<_>>()})),
                Err(e) => ToolOutcome::failed(e),
            },
            _ => ToolOutcome::failed("`action` is save, list or forget"),
        }
    }
}

fn gap_spec() -> ToolSpec {
    ToolSpec::new(
        "skill_gap",
        "When the robot's skills cannot do what the operator asked, log it here (what is \
         missing, and the skills that came closest) before you tell the operator: the log \
         is the list of skills to build next.",
        json!({"type": "object", "properties": {
            "missing": {"type": "string", "description": "What no skill can do, in one sentence."},
            "nearest": {"type": "string", "description": "The skills that came closest, if any."}
        }, "required": ["missing"], "additionalProperties": false}),
        Risk::Observe,
    )
}

/// `skill_gap`.
struct SkillGap {
    spec: ToolSpec,
    missions: Arc<Missions>,
}

#[async_trait]
impl Tool for SkillGap {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let missing = args["missing"].as_str().unwrap_or_default().trim();
        if missing.is_empty() {
            return ToolOutcome::failed("`missing` says what no skill can do");
        }
        self.missions
            .note_gap(missing, args["nearest"].as_str().unwrap_or_default());
        ToolOutcome::ok(json!({"logged": missing}))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_request_keeps_the_words_that_say_what_it_is_about() {
        assert_eq!(
            words("Please bring the small white mug to the tray!"),
            ["bring", "small", "white", "mug", "tray"]
        );
    }

    #[test]
    fn a_recorded_step_reads_as_a_planner_writes_it() {
        assert_eq!(
            step_text("PickObject", &json!({"arm": "left", "object_id": "mug_4"})),
            "PickObject(arm=left, object_id=mug_4)"
        );
    }
}
