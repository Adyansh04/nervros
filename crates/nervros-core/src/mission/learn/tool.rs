//! The `teach_skill` tool.

use std::borrow::Cow;
use std::fmt::Write as _;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::body;
use crate::lock;
use crate::mission::catalog::{Catalog, Learnable, Skill};
use crate::mission::{Missions, SERVICE_TIMEOUT};
use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};

const DEFINE: &str = "nervros_interfaces/srv/DefineSkill";
const FORGET: &str = "nervros_interfaces/srv/ForgetSkill";
/// A taught skill's time when the model names none.
const DEFAULT_S: f64 = 60.0;

/// Teaches, lists and forgets skills.
pub(crate) struct TeachSkill(pub(crate) Arc<Missions>);

impl TeachSkill {
    /// The spec, its leaves read off the catalog as it is now.
    fn teach_spec(&self) -> ToolSpec {
        let catalog = lock(&self.0.catalog).clone();
        let leaves = catalog
            .as_deref()
            .and_then(|c| c.learnable.as_ref())
            .map_or_else(
                || "(the leaves load with the skill catalog)".to_owned(),
                |l| leaf_lines(l, false).join("; "),
            );
        let description = format!(
            "Teaches the robot a skill no skill does yet, such as pointing at something, waving, \
             or looking around a room, from the leaves below: action define with a PascalCase \
             `name`, a `description`, its `args` ({{name, type: string or world_id, \
             description, enum?}}), its `body` as a tree, and `max_s`. Nodes: {{\"sequence\": \
             [..]}}, {{\"fallback\": [..]}}, {{\"retry\": 1-3, \"do\": node}}, {{\"timeout_s\": n, \
             \"do\": node}}, {{\"force_success\": node}}, and {{\"leaf\": Name, input: value}} with \
             each value a literal or {{arg}}, one of the skill's args. Pointing at an object: \
             name PointAt, args object (world_id) and arm, body {{\"sequence\": [{{\"leaf\": \
             \"MoveHand\", \"arm\": \"{{arm}}\", \"target\": \"{{object}}\", \"mode\": \"point\"}}, \
             {{\"leaf\": \"Wait\", \"seconds\": 2}}]}}. A wave is a hand raised above the \
             shoulder (a reach to a point offset from a frame of the robot's own), then to the \
             side and back, with short waits; list says what each input is for. The executor checks it; it is a draft until \
             a mission with it, which the operator approves as usual, succeeds, and then it is \
             kept. Teach one only when no skill fits, then plan a mission with it at once: do not \
             ask first, as the operator approves both. list shows the taught skills and what each \
             leaf's inputs are for; forget drops one by name. Leaves and their inputs: {leaves}"
        );
        ToolSpec::new(
            "teach_skill",
            &description,
            json!({"type": "object", "properties": {
                "action": {"type": "string", "enum": ["define", "list", "forget"]},
                "name": {"type": "string"},
                "description": {"type": "string"},
                "args": {"type": "array", "items": {"type": "object"}},
                "body": {"type": "object"},
                "max_s": {"type": "number", "minimum": 1}
            }, "required": ["action"], "additionalProperties": false}),
            Risk::WorldEdit,
        )
    }

    /// The `DefineSkill` request a define asks for, checked as far as the catalog says.
    async fn request(&self, args: &Value, keep: bool) -> Result<Value, ToolOutcome> {
        let catalog = self.0.catalog().await.map_err(ToolOutcome::failed)?;
        let learnable = catalog
            .learnable
            .as_ref()
            .ok_or_else(|| ToolOutcome::refused("this robot's executor takes no taught skills"))?;
        let name = args["name"].as_str().unwrap_or_default().trim();
        if name.is_empty() {
            return Err(ToolOutcome::failed(
                "`name` is the skill's, such as PointAt",
            ));
        }
        let world_args: Vec<&str> = args["args"]
            .as_array()
            .map_or(&[][..], Vec::as_slice)
            .iter()
            .filter(|a| a["type"] == "world_id")
            .filter_map(|a| a["name"].as_str())
            .collect();
        let xml = body::to_xml(&args["body"], learnable, &world_args)
            .map_err(|problems| ToolOutcome::failed(problems.join("; ")))?;
        // "torso_link" as MoveHand's target failed only once it ran: a literal where a world-model
        // id goes must name something the world model knows.
        let literals = body::world_literals(&args["body"], learnable);
        if !literals.is_empty() {
            let seen = self.0.observe().await;
            let unknown: Vec<String> = literals
                .iter()
                .filter(|l| !knows(&seen.objects, &l.value))
                .map(|l| {
                    let instead = if l.instead.is_empty() {
                        String::new()
                    } else {
                        format!(", or give it as {}", l.instead.join(" or "))
                    };
                    format!(
                        "`{}` for {}'s {} names nothing in the world model: give an object it \
                         knows, or one of the skill's arguments{instead}",
                        l.value, l.leaf, l.input
                    )
                })
                .collect();
            if !unknown.is_empty() {
                return Err(ToolOutcome::failed(unknown.join("; ")));
            }
        }
        let args_list: Vec<Value> = args["args"]
            .as_array()
            .map_or(&[][..], Vec::as_slice)
            .iter()
            .map(|a| {
                json!({
                    "name": a["name"], "type": a["type"].as_str().unwrap_or("string"),
                    "format": a["format"].as_str().unwrap_or_default(),
                    "description": a["description"].as_str().unwrap_or_default(),
                    "enum": a["enum"].as_array().cloned().unwrap_or_default(),
                })
            })
            .collect();
        Ok(json!({
            "name": name,
            "description": args["description"].as_str().unwrap_or_default(),
            "args_json": Value::Array(args_list).to_string(),
            "body_xml": xml,
            "max_duration_s": time_for(args, learnable),
            "taught_for": self.0.request(),
            "keep": keep,
        }))
    }

    async fn define(&self, args: &Value) -> ToolOutcome {
        let request = match self.request(args, false).await {
            Ok(r) => r,
            Err(out) => return out,
        };
        match self.0.define_skill(&request).await {
            Ok(()) => {
                let name = request["name"].as_str().unwrap_or_default();
                self.0.drafts.put(name, request.clone());
                let mut out = ToolOutcome::ok(json!({"draft": name, "body": request["body_xml"],
                                                     "max_s": request["max_duration_s"]}));
                // The skill list this turn's plans read was built before the skill was.
                let only = (self.0.catalog().await.ok())
                    .and_then(|c| c.skill(name).map(pinned))
                    .unwrap_or_default();
                out.message = format!(
                    "{name} is a draft skill now, with up to {:.0} s to run{only}. Plan a \
                     mission with it to try it: the operator approves that as usual, and it is \
                     kept once it succeeds",
                    request["max_duration_s"].as_f64().unwrap_or(DEFAULT_S)
                );
                out
            }
            Err(why) => ToolOutcome::failed(why),
        }
    }

    async fn forget(&self, args: &Value) -> ToolOutcome {
        let name = args["name"].as_str().unwrap_or_default().trim().to_owned();
        match self.0.forget_skill(&name).await {
            Ok(()) => {
                self.0.drafts.take(&name);
                ToolOutcome::ok(json!({"forgot": name}))
            }
            Err(why) => ToolOutcome::failed(why),
        }
    }
}

/// The values the executor pins on a skill's arguments, as `; its arm takes only left or right`.
fn pinned(skill: &Skill) -> String {
    skill
        .model_args()
        .filter(|a| !a.choices.is_empty())
        .fold(String::new(), |mut text, a| {
            let _ = write!(
                text,
                "; its {} takes only {}",
                a.name,
                a.choices.join(" or ")
            );
            text
        })
}

/// Whether the world model's `objects` (a `WorldObjectArray`) holds something `value` names: an
/// id, or a label one of its words is ("the red mug"). With no objects at all, nothing is known
/// either way, and it passes.
fn knows(objects: &Value, value: &str) -> bool {
    let Some(list) = objects["objects"].as_array().filter(|l| !l.is_empty()) else {
        return true;
    };
    let words = |text: &str| -> Vec<String> {
        text.split(|c: char| !c.is_ascii_alphabetic())
            .filter(|w| w.len() > 2)
            .map(str::to_ascii_lowercase)
            .collect()
    };
    let asked = words(value);
    list.iter().any(|o| {
        o["id"].as_str() == Some(value)
            || o["label"]
                .as_str()
                .is_some_and(|l| words(l).iter().any(|w| asked.contains(w)))
    })
}

/// The skill's time: what the model asked, raised to what its leaves need at least, within the
/// executor's most.
fn time_for(args: &Value, learnable: &Learnable) -> f64 {
    let asked = args["max_s"].as_f64().unwrap_or(DEFAULT_S);
    asked
        .max(body::least_s(&args["body"], learnable))
        .min(learnable.max_duration_s)
}

/// Each learnable leaf in a line: what it moves and its inputs, with what each is for when
/// `explained`.
fn leaf_lines(learnable: &Learnable, explained: bool) -> Vec<String> {
    learnable
        .leaves
        .iter()
        .map(|(name, leaf)| {
            let label = |p: &str| {
                let world = leaf
                    .world_ids
                    .as_ref()
                    .is_some_and(|w| w.iter().any(|x| x == p));
                let what = leaf.ports.get(p).filter(|w| explained && !w.is_empty());
                match (world, what) {
                    (true, Some(what)) => format!("{p} (a world id; {what})"),
                    (true, None) => format!("{p} (a world id)"),
                    (false, Some(what)) => format!("{p} ({what})"),
                    (false, None) => p.to_owned(),
                }
            };
            // A group shows once, where its first input would: "target or frame, exactly one".
            let ports: Vec<String> = leaf
                .ports
                .keys()
                .filter_map(|p| match leaf.one_of.iter().find(|g| g.contains(p)) {
                    Some(g) if g.first() != Some(p) => None,
                    Some(g) => Some(format!(
                        "{}, exactly one",
                        g.iter().map(|x| label(x)).collect::<Vec<_>>().join(" or ")
                    )),
                    None => Some(label(p)),
                })
                .collect();
            let holds = match leaf.holds.as_str() {
                "arms" => "moves an arm",
                "base" => "moves the base",
                _ => "moves nothing",
            };
            format!("{name}, {holds}: {}", ports.join(", "))
        })
        .collect()
}

/// The taught skills, a line each.
fn taught(catalog: &Catalog) -> Vec<String> {
    catalog
        .skills
        .iter()
        .filter(|s| !s.learned.is_empty())
        .map(|s| {
            let mut line = format!("{} ({}): {}", s.name, s.learned, s.description);
            if !s.taught_for.is_empty() {
                let _ = write!(line, " Taught for \"{}\".", s.taught_for);
            }
            line
        })
        .collect()
}

#[async_trait]
impl Tool for TeachSkill {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Owned(self.teach_spec())
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let annotate = |reason: String| Assessment {
            risk: Risk::WorldEdit,
            resources: Vec::new(),
            reason,
            args: None,
        };
        Some(match args["action"].as_str() {
            Some("list") => Ok(Assessment {
                risk: Risk::Observe,
                ..annotate("lists the taught skills".to_owned())
            }),
            Some("forget") => Ok(annotate(format!(
                "forgets the taught skill {}",
                args["name"].as_str().unwrap_or_default()
            ))),
            Some("define") => self.request(args, false).await.map(|request| {
                let description = request["description"].as_str().unwrap_or_default().trim();
                // Models write the description without a full stop, and a sentence follows it.
                let stop = if description.ends_with(['.', '!', '?']) {
                    ""
                } else {
                    "."
                };
                annotate(format!(
                    "teaches the robot {}: {description}{stop} It is tried, with your approval, \
                     in the next mission that uses it",
                    request["name"].as_str().unwrap_or_default(),
                ))
            }),
            _ => Err(ToolOutcome::failed("`action` is define, list or forget")),
        })
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str() {
            Some("define") => self.define(&args).await,
            Some("forget") => self.forget(&args).await,
            Some("list") => match self.0.catalog().await {
                Ok(c) => ToolOutcome::ok(json!({
                    "taught": taught(&c),
                    "leaves": c.learnable.as_ref().map(|l| leaf_lines(l, true)).unwrap_or_default(),
                })),
                Err(e) => ToolOutcome::failed(e),
            },
            _ => ToolOutcome::failed("`action` is define, list or forget"),
        }
    }
}

impl Missions {
    /// Sends a `DefineSkill` request, and reads the catalog again once it took.
    ///
    /// # Errors
    ///
    /// The executor's problems with the skill, or that it did not answer.
    pub(super) async fn define_skill(&self, request: &Value) -> Result<(), String> {
        let service = self
            .config
            .define_skill
            .as_deref()
            .ok_or("no DefineSkill service")?;
        let reply = self
            .robot
            .call(service, DEFINE, request.clone(), SERVICE_TIMEOUT)
            .await
            .map_err(|e| format!("the executor did not answer: {e}"))?;
        if reply["ok"].as_bool() != Some(true) {
            let problems: Vec<String> = reply["diagnostics_json"]
                .as_str()
                .and_then(|d| serde_json::from_str::<Value>(d).ok())
                .and_then(|d| d.as_array().cloned())
                .unwrap_or_default()
                .iter()
                .filter_map(|p| p["message"].as_str().map(str::to_owned))
                .collect();
            return Err(if problems.is_empty() {
                reply["message"]
                    .as_str()
                    .unwrap_or("the executor refused it")
                    .to_owned()
            } else {
                problems.join("; ")
            });
        }
        self.reload_catalog().await;
        Ok(())
    }

    /// Sends a `ForgetSkill` request, and reads the catalog again.
    ///
    /// # Errors
    ///
    /// There is no such taught skill, or the executor did not answer.
    pub(super) async fn forget_skill(&self, name: &str) -> Result<(), String> {
        let service = self
            .config
            .forget_skill
            .as_deref()
            .ok_or("no ForgetSkill service")?;
        let reply = self
            .robot
            .call(service, FORGET, json!({"name": name}), SERVICE_TIMEOUT)
            .await
            .map_err(|e| format!("the executor did not answer: {e}"))?;
        self.reload_catalog().await;
        if reply["ok"].as_bool() == Some(true) {
            Ok(())
        } else {
            Err(reply["message"]
                .as_str()
                .unwrap_or("it was not forgotten")
                .to_owned())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::catalog::tests::CATALOG;

    #[test]
    fn a_new_skill_s_pinned_values_are_named_for_its_first_plan() {
        let catalog = Catalog::parse(CATALOG).unwrap();
        assert_eq!(
            pinned(catalog.skill("PickObject").unwrap()),
            "; its arm takes only left or right"
        );
        assert_eq!(pinned(catalog.skill("TuckForTravel").unwrap()), "");
    }
}
