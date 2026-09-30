//! Plans: what the model sends, and how one becomes a behaviour tree for the executor.
//!
//! A plan is a list of steps, each naming a catalog skill. Compiling checks every step against
//! the catalog and the world model, and writes a `Mission` tree of `SubTree` references to the
//! executor's own macros, each wrapped in its step's timeout, retries and optional flag. The
//! executor validates the tree again; the hash of the exact XML is what the operator approves.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use serde::{Deserialize, Deserializer, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};

use super::catalog::{Catalog, GO_TO_PLACE, GO_TO_POSE, GO_TO_TARGET, Skill};
use crate::profile::PlaceConfig;

/// Plans longer than this are split by the model into several missions.
pub const MAX_STEPS: usize = 12;
const MAX_RETRIES: u8 = 2;
/// The mission's own timeout over its worst case, so a step's timeout fires first.
const SLACK: f64 = 1.2;

/// What the model sends to `plan_mission`.
#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Plan {
    /// What the operator asked for, in a few words.
    pub intent: String,
    /// Predicates that should hold at the end, such as `at(kitchen)`.
    #[serde(default)]
    pub goal: Vec<String>,
    /// In order.
    pub steps: Vec<Step>,
}

/// One step.
#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Step {
    /// A catalog skill, or `GoToPlace`.
    pub skill: String,
    /// The skill's arguments. A list of `{name, value}` or an object both work.
    #[serde(default, deserialize_with = "args_list_or_map")]
    pub args: Vec<StepArg>,
    /// Extra attempts after a failure, 0 to 2.
    #[serde(default)]
    pub retries: u8,
    /// At most the skill's own timeout, which is also the default.
    #[serde(default)]
    pub timeout_s: Option<f64>,
    /// A failure here does not fail the mission.
    #[serde(default)]
    pub optional: bool,
    /// Why this step, for the operator.
    #[serde(default)]
    pub why: String,
}

/// One argument.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct StepArg {
    /// The skill's port name.
    pub name: String,
    /// Its value.
    pub value: String,
}

// Small models often send `{"arm": "right"}` for the list form; both mean the same.
fn args_list_or_map<'de, D: Deserializer<'de>>(d: D) -> Result<Vec<StepArg>, D::Error> {
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum Form {
        List(Vec<StepArg>),
        Map(BTreeMap<String, Value>),
    }
    Ok(match Form::deserialize(d)? {
        Form::List(list) => list,
        Form::Map(map) => map
            .into_iter()
            .map(|(name, v)| StepArg {
                name,
                value: v.as_str().map_or_else(|| v.to_string(), str::to_owned),
            })
            .collect(),
    })
}

/// What exists when a plan is compiled: the profile's places and the world model's rooms and
/// objects, as `(id, name)`.
#[derive(Debug, Clone, Default)]
pub struct World {
    /// Named places.
    pub places: Vec<PlaceConfig>,
    /// Rooms.
    pub rooms: Vec<(String, String)>,
    /// Objects.
    pub objects: Vec<(String, String)>,
}

/// A step as the operator sees it.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct PlannedStep {
    /// `s1`, `s2`, ...: the executor's events carry it.
    pub id: String,
    /// The skill as planned, such as `GoToPlace`.
    pub skill: String,
    /// One line, such as `PickObject(object_id=O17, arm=right)`.
    pub summary: String,
    /// Its timeout.
    pub timeout_s: f64,
}

/// A plan that compiled.
#[derive(Debug, Clone, PartialEq)]
pub struct Compiled {
    /// The plan.
    pub plan: Plan,
    /// The tree.
    pub xml: String,
    /// SHA-256 of the XML's bytes, lowercase hex.
    pub sha256: String,
    /// The steps.
    pub steps: Vec<PlannedStep>,
    /// The longest it can take, from the steps' timeouts and retries.
    pub worst_case_s: f64,
    /// The catalog it was checked against.
    pub catalog_version: String,
}

/// Something wrong with a plan, for the model to fix.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Problem {
    /// `s1`, or empty for the whole plan.
    pub step: String,
    /// The argument, or empty.
    pub field: String,
    /// What to change.
    pub message: String,
}

impl Problem {
    fn new(step: &str, field: &str, message: impl Into<String>) -> Self {
        Self {
            step: step.to_owned(),
            field: field.to_owned(),
            message: message.into(),
        }
    }
}

/// Compiles a plan.
///
/// # Errors
///
/// Every problem found, not just the first, so the model fixes them in one go.
pub fn compile(plan: &Plan, catalog: &Catalog, world: &World) -> Result<Compiled, Vec<Problem>> {
    let mut problems = Vec::new();
    if plan.steps.is_empty() {
        problems.push(Problem::new("", "steps", "a plan needs at least one step"));
    }
    if plan.steps.len() > MAX_STEPS {
        problems.push(Problem::new(
            "",
            "steps",
            format!("at most {MAX_STEPS} steps; split the task into several missions"),
        ));
    }
    let mut body = String::new();
    let mut steps = Vec::new();
    let mut worst = 0.0;
    for (i, step) in plan.steps.iter().enumerate() {
        let id = format!("s{}", i + 1);
        match compile_step(&id, step, catalog, world) {
            Ok((node, timeout_s)) => {
                let attempts = f64::from(step.retries) + 1.0;
                worst += timeout_s * attempts;
                body.push_str(&wrap(&node, step, timeout_s));
                steps.push(PlannedStep {
                    id,
                    skill: step.skill.clone(),
                    summary: summary(step),
                    timeout_s,
                });
            }
            Err(mut p) => problems.append(&mut p),
        }
    }
    if !problems.is_empty() {
        return Err(problems);
    }
    let xml = format!(
        "<root BTCPP_format=\"4\" main_tree_to_execute=\"Mission\">\n  \
         <BehaviorTree ID=\"Mission\">\n    <Timeout msec=\"{}\">\n      <Sequence name=\"{}\">\n\
         {body}      </Sequence>\n    </Timeout>\n  </BehaviorTree>\n</root>\n",
        msec(worst * SLACK),
        escape(&plan.intent),
    );
    let sha256 = format!("{:x}", Sha256::digest(xml.as_bytes()));
    Ok(Compiled {
        plan: plan.clone(),
        xml,
        sha256,
        steps,
        worst_case_s: worst,
        catalog_version: catalog.catalog_version.clone(),
    })
}

/// The step's `SubTree` element and its timeout in seconds.
fn compile_step(
    id: &str,
    step: &Step,
    catalog: &Catalog,
    world: &World,
) -> Result<(String, f64), Vec<Problem>> {
    let mut problems = Vec::new();
    if step.retries > MAX_RETRIES {
        problems.push(Problem::new(
            id,
            "retries",
            format!("retries is 0 to {MAX_RETRIES}"),
        ));
    }
    let (skill, ports) = if step.skill == GO_TO_PLACE {
        match go_to_place(id, step, catalog, world) {
            Ok(v) => v,
            Err(p) => {
                problems.push(p);
                return Err(problems);
            }
        }
    } else if step.skill == GO_TO_POSE || step.skill == GO_TO_TARGET {
        problems.push(Problem::new(
            id,
            "skill",
            format!(
                "use {GO_TO_PLACE}(place) to move; it picks {} itself",
                step.skill
            ),
        ));
        return Err(problems);
    } else if let Some(skill) = catalog.skill(&step.skill) {
        match ports_for(id, step, skill, world) {
            Ok(ports) => (skill, ports),
            Err(mut p) => {
                problems.append(&mut p);
                return Err(problems);
            }
        }
    } else {
        problems.push(Problem::new(
            id,
            "skill",
            format!(
                "`{}` is not a skill of this robot; use one of: {}",
                step.skill,
                catalog.plan_skills().join(", ")
            ),
        ));
        return Err(problems);
    };
    let timeout_s = match step.timeout_s {
        Some(t) if t.is_finite() && t > 0.0 => t.clamp(1.0, skill.max_duration_s),
        Some(_) => {
            problems.push(Problem::new(id, "timeout_s", "timeout_s must be positive"));
            skill.max_duration_s
        }
        None => skill.max_duration_s,
    };
    if !problems.is_empty() {
        return Err(problems);
    }
    let mut node = format!(
        "<SubTree ID=\"{}\" name=\"{id}_{}\"",
        escape(&skill.template),
        escape(&step.skill)
    );
    for (name, value) in ports {
        let _ = write!(node, " {name}=\"{}\"", escape(&value));
    }
    node.push_str("/>");
    Ok((node, timeout_s))
}

/// Checks a skill's arguments: all given, none unknown, enums respected, ids known.
fn ports_for(
    id: &str,
    step: &Step,
    skill: &Skill,
    world: &World,
) -> Result<Vec<(String, String)>, Vec<Problem>> {
    let mut problems = Vec::new();
    for a in &step.args {
        if !skill.args.iter().any(|s| s.name == a.name) {
            let names: Vec<&str> = skill.args.iter().map(|s| s.name.as_str()).collect();
            problems.push(Problem::new(
                id,
                &a.name,
                format!(
                    "{} has no argument `{}`; its arguments are: {}",
                    skill.name,
                    a.name,
                    names.join(", ")
                ),
            ));
        }
    }
    let mut ports = Vec::new();
    for arg in &skill.args {
        let Some(given) = step.args.iter().find(|a| a.name == arg.name) else {
            problems.push(Problem::new(
                id,
                &arg.name,
                format!("{} needs `{}`", skill.name, arg.name),
            ));
            continue;
        };
        let value = given.value.trim();
        if !arg.choices.is_empty() && !arg.choices.iter().any(|c| c == value) {
            problems.push(Problem::new(
                id,
                &arg.name,
                format!("`{}` must be one of: {}", arg.name, arg.choices.join(", ")),
            ));
        }
        // By the catalog's convention, `*_id` arguments are world model ids.
        if arg.name.ends_with("_id") && !world.objects.iter().any(|(o, _)| o == value) {
            problems.push(Problem::new(
                id,
                &arg.name,
                format!(
                    "{value} is not in the world model; call find_objects and use an id it returns"
                ),
            ));
        }
        ports.push((arg.name.clone(), value.to_owned()));
    }
    if problems.is_empty() {
        Ok(ports)
    } else {
        Err(problems)
    }
}

/// A navigation macro and its one port.
type Resolved<'c> = (&'c Skill, Vec<(String, String)>);

/// `GoToPlace(place)`: a profile place becomes `GoToPose` at its pose; a room or object id, or a
/// room's name, becomes `GoToTarget`.
fn go_to_place<'c>(
    id: &str,
    step: &Step,
    catalog: &'c Catalog,
    world: &World,
) -> Result<Resolved<'c>, Problem> {
    let Some(place) = step
        .args
        .iter()
        .find(|a| a.name == "place")
        .map(|a| a.value.trim())
    else {
        return Err(Problem::new(id, "place", "GoToPlace needs `place`"));
    };
    let same = |a: &str| a.eq_ignore_ascii_case(place);
    let named = world
        .places
        .iter()
        .find(|p| same(&p.name) || p.aliases.iter().any(|a| same(a)));
    if let (Some(p), Some(skill)) = (named, catalog.skill(GO_TO_POSE)) {
        let station = format!("{};{};{}", p.pose.x, p.pose.y, p.pose.yaw);
        return Ok((skill, vec![(port(skill, "station"), station)]));
    }
    let target = world
        .rooms
        .iter()
        .find(|(rid, name)| rid == place || same(name))
        .map(|(rid, _)| rid)
        .or_else(|| {
            world
                .objects
                .iter()
                .find(|(oid, _)| oid == place)
                .map(|(oid, _)| oid)
        });
    if let (Some(t), Some(skill)) = (target, catalog.skill(GO_TO_TARGET)) {
        return Ok((skill, vec![(port(skill, "target"), t.clone())]));
    }
    Err(Problem::new(
        id,
        "place",
        format!("`{place}` is not a known place, room or object; call list_places or find_objects"),
    ))
}

/// The navigation macro's single port; the conventional name if the catalog lists none.
fn port(skill: &Skill, conventional: &str) -> String {
    skill
        .args
        .first()
        .map_or_else(|| conventional.to_owned(), |a| a.name.clone())
}

fn wrap(node: &str, step: &Step, timeout_s: f64) -> String {
    let mut inner = format!("<Timeout msec=\"{}\">{node}</Timeout>", msec(timeout_s));
    if step.retries > 0 {
        inner = format!(
            "<RetryUntilSuccessful num_attempts=\"{}\">{inner}</RetryUntilSuccessful>",
            step.retries + 1
        );
    }
    if step.optional {
        inner = format!("<ForceSuccess>{inner}</ForceSuccess>");
    }
    format!("        {inner}\n")
}

fn summary(step: &Step) -> String {
    let args: Vec<String> = step
        .args
        .iter()
        .map(|a| format!("{}={}", a.name, a.value))
        .collect();
    format!("{}({})", step.skill, args.join(", "))
}

#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "timeouts are positive and far below u64::MAX milliseconds"
)]
fn msec(seconds: f64) -> u64 {
    (seconds * 1000.0).ceil() as u64
}

fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::catalog::tests::CATALOG;
    use crate::profile::PlacePose;
    use serde_json::json;

    fn world() -> World {
        World {
            places: vec![PlaceConfig {
                name: "dock".to_owned(),
                aliases: vec!["charging dock".to_owned()],
                frame: "map".to_owned(),
                pose: PlacePose {
                    x: 1.5,
                    y: -2.0,
                    yaw: 0.5,
                },
            }],
            rooms: vec![("R2".to_owned(), "kitchen".to_owned())],
            objects: vec![("O17".to_owned(), "red mug".to_owned())],
        }
    }

    fn plan(steps: &Value) -> Plan {
        serde_json::from_value(json!({"intent": "fetch the mug", "steps": steps})).unwrap()
    }

    fn catalog() -> Catalog {
        Catalog::parse(CATALOG).unwrap()
    }

    #[test]
    fn a_plan_compiles_to_subtree_references() {
        let p = plan(&json!([
            {"skill": "GoToPlace", "args": [{"name": "place", "value": "kitchen"}]},
            {"skill": "PickObject", "args": {"object_id": "O17", "phrase": "red mug", "arm": "right"}, "retries": 1},
            {"skill": "GoToPlace", "args": {"place": "Charging Dock"}, "optional": true, "timeout_s": 60}
        ]));
        let c = compile(&p, &catalog(), &world()).unwrap();
        assert!(
            c.xml
                .contains(r#"<SubTree ID="GoToTarget" name="s1_GoToPlace" target="R2"/>"#)
        );
        assert!(c.xml.contains(
            r#"<RetryUntilSuccessful num_attempts="2"><Timeout msec="420000"><SubTree ID="PickObject" name="s2_PickObject" object_id="O17" phrase="red mug" arm="right"/></Timeout></RetryUntilSuccessful>"#
        ));
        assert!(c.xml.contains(
            r#"<ForceSuccess><Timeout msec="60000"><SubTree ID="GoToPose" name="s3_GoToPlace" station="1.5;-2;0.5"/></Timeout></ForceSuccess>"#
        ));
        assert!((c.worst_case_s - (300.0 + 840.0 + 60.0)).abs() < 1e-9);
        assert!(c.xml.contains(r#"<Timeout msec="1440000">"#));
        assert_eq!(c.sha256, format!("{:x}", Sha256::digest(c.xml.as_bytes())));
        assert_eq!(
            c.steps[1].summary,
            "PickObject(arm=right, object_id=O17, phrase=red mug)"
        );
    }

    #[test]
    fn every_problem_is_reported_at_once() {
        let p = plan(&json!([
            {"skill": "PickObject", "args": {"object_id": "O99", "arm": "middle", "grip": "firm"}},
            {"skill": "GoToPose", "args": {"station": "1;2;3"}},
            {"skill": "Fly"},
            {"skill": "GoToPlace", "args": {"place": "moon"}},
            {"skill": "TuckForTravel", "retries": 5}
        ]));
        let problems = compile(&p, &catalog(), &world()).unwrap_err();
        let has =
            |step: &str, field: &str| problems.iter().any(|q| q.step == step && q.field == field);
        assert!(
            has("s1", "grip") && has("s1", "phrase") && has("s1", "arm") && has("s1", "object_id")
        );
        assert!(
            has("s2", "skill") && has("s3", "skill") && has("s4", "place") && has("s5", "retries")
        );
        assert!(problems.iter().any(|q| q.message.contains("find_objects")));
    }

    #[test]
    fn text_is_escaped() {
        let mut p = plan(
            &json!([{"skill": "PickObject", "args": {"object_id": "O17", "phrase": "\"mug\" & <cup>", "arm": "left"}}]),
        );
        p.intent = "a <b> & \"c\"".to_owned();
        let c = compile(&p, &catalog(), &world()).unwrap();
        assert!(
            c.xml
                .contains("phrase=\"&quot;mug&quot; &amp; &lt;cup&gt;\"")
        );
        assert!(c.xml.contains("name=\"a &lt;b&gt; &amp; &quot;c&quot;\""));
    }
}
