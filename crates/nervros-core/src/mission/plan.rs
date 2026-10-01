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
/// How near the robot must already stand for a skill that needs it near an object; the executor's
/// skills ask for about half a metre.
const NEAR_M: f64 = 0.6;

/// What the model sends to `run_mission`.
#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Plan {
    /// What the operator asked for, in a few words.
    pub intent: String,
    /// A model's own end state, accepted and ignored: [`Compiled::goal`] comes from the steps.
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

/// Something in the world model a plan can name.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Thing {
    /// Its id, such as `O17` or `R2`.
    pub id: String,
    /// Its name or label.
    pub name: String,
    /// Where it is on the map, when known.
    pub xy: Option<(f64, f64)>,
}

/// What exists when a plan is compiled, and the robot as it starts.
#[derive(Debug, Clone, Default)]
pub struct World {
    /// Named places.
    pub places: Vec<PlaceConfig>,
    /// Rooms, placed at their centroids.
    pub rooms: Vec<Thing>,
    /// Objects.
    pub objects: Vec<Thing>,
    /// The robot on the map, when known.
    pub robot: Option<(f64, f64)>,
    /// What each hand holds, by arm; empty when nothing.
    pub holding: BTreeMap<String, String>,
}

impl World {
    /// Whether an object or room with this id is in the world model.
    fn knows(&self, id: &str) -> bool {
        self.objects.iter().chain(&self.rooms).any(|t| t.id == id)
    }
}

/// Where a `GoToPlace` leaves the robot.
struct Spot {
    id: String,
    xy: Option<(f64, f64)>,
}

/// The robot as the plan so far leaves it.
struct Sim {
    at: Option<String>,
    xy: Option<(f64, f64)>,
    holding: BTreeMap<String, String>,
    /// What went into what, object first.
    inside: Vec<(String, String)>,
}

impl Sim {
    fn near(&self, target: &str, world: &World) -> bool {
        if self.at.as_deref() == Some(target) {
            return true;
        }
        if let Some(at) = &self.at
            && world
                .places
                .iter()
                .any(|p| p.name == *at && p.near.iter().any(|n| n == target))
        {
            return true;
        }
        let thing = world.objects.iter().find(|o| o.id == target);
        match (self.xy, thing.and_then(|t| t.xy)) {
            (Some((x, y)), Some((tx, ty))) => (tx - x).hypot(ty - y) <= NEAR_M,
            _ => false,
        }
    }

    /// Checks a skill's needs against the robot so far, then applies what it does. Understands
    /// `near(x)`, `holding(arm, x)` and `hand_empty(arm)`, where `x` names an argument or, as a
    /// capital letter, any object; other predicates are left to the executor.
    fn run(&mut self, id: &str, step: &Step, skill: &Skill, world: &World) -> Vec<Problem> {
        let value = |name: &str| {
            step.args
                .iter()
                .find(|a| a.name == name)
                .map(|a| a.value.trim().to_owned())
        };
        let mut problems = Vec::new();
        let mut problem = |field: &str, text: String| {
            problems.push(Problem::new(id, field, format!("{} {text}", skill.name)));
        };
        for need in &skill.requires {
            let Some((name, args)) = predicate(need) else {
                continue;
            };
            match (name, args.as_slice()) {
                // Only what the world model knows can be judged; nearness to anything else, such
                // as a detector's name for an object, is the executor's to check.
                ("near", [x]) => {
                    if let Some(target) = value(x)
                        && world.knows(&target)
                        && !self.near(&target, world)
                    {
                        problem(
                            x,
                            format!(
                                "needs the robot near {target} first: add {GO_TO_PLACE}(place={target}) before this step"
                            ),
                        );
                    }
                }
                ("holding", [a, x]) => {
                    let Some(arm) = value(a) else { continue };
                    let held = self.held(&arm);
                    let wanted = value(x);
                    let ok = wanted.as_ref().map_or(!held.is_empty(), |obj| held == *obj);
                    if !ok {
                        let what = wanted.unwrap_or_else(|| "something".to_owned());
                        problem(
                            a,
                            format!(
                                "needs the {arm} hand to hold {what} first: pick it up earlier in the plan, or use the arm that holds it"
                            ),
                        );
                    }
                }
                ("hand_empty", [a]) => {
                    let Some(arm) = value(a) else { continue };
                    let held = self.held(&arm);
                    if !held.is_empty() {
                        problem(
                            a,
                            format!(
                                "needs the {arm} hand empty, but it holds {held}: place it first or use the other hand"
                            ),
                        );
                    }
                }
                _ => {}
            }
        }
        problems.append(&mut self.apply(id, step, skill));
        // A skill that takes the base may move it, as a pick backs off the surface afterwards, so
        // the robot's position is unknown until the next walk.
        if skill.resources.iter().any(|r| r == "base") {
            self.at = None;
            self.xy = None;
        }
        problems
    }

    /// Applies what a skill does, as `run` checked it may.
    fn apply(&mut self, id: &str, step: &Step, skill: &Skill) -> Vec<Problem> {
        let value = |name: &str| {
            step.args
                .iter()
                .find(|a| a.name == name)
                .map(|a| a.value.trim().to_owned())
        };
        let mut problems = Vec::new();
        for effect in &skill.effects {
            let Some((name, args)) = predicate(effect) else {
                continue;
            };
            match (name, args.as_slice()) {
                ("holding", [a, x]) => {
                    if let (Some(arm), Some(obj)) = (value(a), value(x)) {
                        self.inside.retain(|(o, _)| *o != obj);
                        // A replan after a failed place tends to pick the object up again.
                        if let Some((hand, _)) = self.holding.iter().find(|(_, held)| **held == obj)
                        {
                            problems.push(Problem::new(
                                id,
                                x,
                                format!(
                                    "{obj} is already in the {hand} hand: place it instead of picking it up again"
                                ),
                            ));
                        }
                        self.holding.insert(arm, obj);
                    }
                }
                ("hand_empty", [a]) => {
                    if let Some(arm) = value(a) {
                        self.holding.insert(arm, String::new());
                    }
                }
                // `X`, a capital, is what the hand the skill needs holding holds: its effects
                // come before `hand_empty` empties it.
                ("inside", [x, c]) => {
                    let obj = if x.starts_with(char::is_uppercase) {
                        skill
                            .requires
                            .iter()
                            .find_map(|need| match predicate(need) {
                                Some(("holding", r)) if r.len() == 2 && r[1] == *x => {
                                    value(r[0]).map(|arm| self.held(&arm))
                                }
                                _ => None,
                            })
                            .filter(|o| !o.is_empty())
                    } else {
                        value(x)
                    };
                    if let (Some(obj), Some(container)) = (obj, value(c)) {
                        self.inside.retain(|(o, _)| *o != obj);
                        self.inside.push((obj, container));
                    }
                }
                _ => {}
            }
        }
        problems
    }

    fn held(&self, arm: &str) -> String {
        self.holding.get(arm).cloned().unwrap_or_default()
    }

    /// What holds once every step has run: where the last walk leaves the robot, what each hand
    /// still holds and what went into what.
    fn end_state(&self) -> Vec<String> {
        let mut out: Vec<String> = self.at.iter().map(|p| format!("at({p})")).collect();
        out.extend(
            self.holding
                .iter()
                .filter(|(_, held)| !held.is_empty())
                .map(|(arm, held)| format!("holding({arm}, {held})")),
        );
        out.extend(self.inside.iter().map(|(o, c)| format!("inside({o}, {c})")));
        out
    }

    /// The walk a step needs first: a `GoToPlace` up to the object or room, known to the world
    /// model, that the step's skill needs the robot near and the plan has not reached yet.
    fn walk_for(&self, step: &Step, catalog: &Catalog, world: &World) -> Option<Step> {
        let skill = catalog.skill(&step.skill)?;
        catalog.skill(GO_TO_TARGET)?;
        skill.requires.iter().find_map(|need| {
            let ("near", args) = predicate(need)? else {
                return None;
            };
            let [x] = args.as_slice() else { return None };
            let target = step
                .args
                .iter()
                .find(|a| a.name == *x)?
                .value
                .trim()
                .to_owned();
            if self.near(&target, world) {
                return None;
            }
            // A thing only the detector knows is reached from the place the profile says.
            let place = if world.knows(&target) {
                target
            } else {
                world
                    .places
                    .iter()
                    .find(|p| p.near.contains(&target))?
                    .name
                    .clone()
            };
            Some(Step {
                skill: GO_TO_PLACE.to_owned(),
                args: vec![StepArg {
                    name: "place".to_owned(),
                    value: place,
                }],
                retries: 0,
                timeout_s: None,
                optional: false,
                why: format!("{} needs the robot near it", step.skill),
            })
        })
    }
}

/// `name(a, b)` as `("name", ["a", "b"])`.
pub(crate) fn predicate(text: &str) -> Option<(&str, Vec<&str>)> {
    let (name, rest) = text.trim().split_once('(')?;
    let args = rest.strip_suffix(')')?;
    Some((name.trim(), args.split(',').map(str::trim).collect()))
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
    /// What should hold at the end, from the steps' effects: a model's own goals once
    /// contradicted its steps, and their failed checks sent it to repeat a mission that worked.
    pub goal: Vec<String>,
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
    let mut sim = Sim {
        at: None,
        xy: world.robot,
        holding: world.holding.clone(),
        inside: Vec::new(),
    };
    let mut n = 0;
    for planned in &plan.steps {
        let walk = sim.walk_for(planned, catalog, world);
        for (step, added) in walk.iter().map(|w| (w, true)).chain([(planned, false)]) {
            n += 1;
            let id = format!("s{n}");
            match compile_step(&id, step, catalog, world) {
                Ok((node, timeout_s, spot)) => {
                    if let Some(spot) = spot {
                        sim.at = Some(spot.id);
                        sim.xy = spot.xy;
                    } else if let Some(skill) = catalog.skill(&step.skill) {
                        problems.append(&mut sim.run(&id, step, skill, world));
                    }
                    let attempts = f64::from(step.retries) + 1.0;
                    worst += timeout_s * attempts;
                    body.push_str(&wrap(&node, step, timeout_s));
                    let mut text = summary(step);
                    if added {
                        text.push_str(" (added)");
                    }
                    steps.push(PlannedStep {
                        id,
                        skill: step.skill.clone(),
                        summary: text,
                        timeout_s,
                    });
                }
                Err(mut p) => problems.append(&mut p),
            }
        }
    }
    if !problems.is_empty() {
        return Err(problems);
    }
    let xml = format!(
        "<root BTCPP_format=\"4\" main_tree_to_execute=\"Mission\">\n  \
         <BehaviorTree ID=\"Mission\">\n    <Timeout msec=\"{}\">\n      <Sequence name=\"mission\">\n\
         {body}      </Sequence>\n    </Timeout>\n  </BehaviorTree>\n</root>\n",
        msec(worst * SLACK),
    );
    let sha256 = format!("{:x}", Sha256::digest(xml.as_bytes()));
    Ok(Compiled {
        plan: plan.clone(),
        xml,
        sha256,
        steps,
        worst_case_s: worst,
        catalog_version: catalog.catalog_version.clone(),
        goal: sim.end_state(),
    })
}

/// The step's `SubTree` element, its timeout in seconds and, for `GoToPlace`, where it leaves
/// the robot.
fn compile_step(
    id: &str,
    step: &Step,
    catalog: &Catalog,
    world: &World,
) -> Result<(String, f64, Option<Spot>), Vec<Problem>> {
    let mut problems = Vec::new();
    if step.retries > MAX_RETRIES {
        problems.push(Problem::new(
            id,
            "retries",
            format!("retries is 0 to {MAX_RETRIES}"),
        ));
    }
    let mut spot = None;
    let (skill, ports) = if step.skill == GO_TO_PLACE {
        match go_to_place(id, step, catalog, world) {
            Ok((skill, ports, to)) => {
                spot = Some(to);
                (skill, ports)
            }
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
                catalog.signatures().join(", ")
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
    Ok((node, timeout_s, spot))
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
        let derived;
        let value = match (
            step.args.iter().find(|a| a.name == arg.name),
            &arg.default_from,
        ) {
            (Some(given), _) => given.value.trim(),
            (None, Some(rule)) => {
                if let Some(v) = derive(rule, step, world) {
                    derived = v;
                    derived.as_str()
                } else {
                    problems.push(Problem::new(
                        id,
                        &arg.name,
                        format!(
                            "give `{}`: it could not be filled in from `{rule}`",
                            arg.name
                        ),
                    ));
                    continue;
                }
            }
            (None, None) => {
                problems.push(Problem::new(
                    id,
                    &arg.name,
                    format!("{} needs `{}`", skill.name, arg.name),
                ));
                continue;
            }
        };
        if !arg.choices.is_empty() && !arg.choices.iter().any(|c| c == value) {
            problems.push(Problem::new(
                id,
                &arg.name,
                format!("`{}` must be one of: {}", arg.name, arg.choices.join(", ")),
            ));
        }
        // BehaviorTree.CPP reads `{name}` as a blackboard entry, not text.
        if value.contains(['{', '}']) {
            problems.push(Problem::new(
                id,
                &arg.name,
                format!("`{}` may not contain {{ or }}", arg.name),
            ));
        }
        if arg.is_world_id() && !world.knows(value) {
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

/// A navigation macro, its one port, and where it leaves the robot.
type Resolved<'c> = (&'c Skill, Vec<(String, String)>, Spot);

/// A derived argument's value: `label(x)` is the world model's label of what argument `x` names,
/// or, for something it does not know, such as a detector's `red_block`, that name in words.
fn derive(rule: &str, step: &Step, world: &World) -> Option<String> {
    let ("label", args) = predicate(rule)? else {
        return None;
    };
    let from = args.first()?;
    let id = step.args.iter().find(|a| a.name == *from)?.value.trim();
    let label = world
        .objects
        .iter()
        .chain(&world.rooms)
        .find(|t| t.id == id)
        .map(|t| t.name.clone())
        .filter(|n| !n.is_empty());
    label.or_else(|| Some(id.replace('_', " ")).filter(|w| !w.trim().is_empty()))
}

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
        let spot = Spot {
            id: p.name.clone(),
            xy: Some((p.pose.x, p.pose.y)),
        };
        return Ok((skill, vec![(port(skill, "station"), station)], spot));
    }
    let mut target = world
        .rooms
        .iter()
        .find(|r| r.id == place || same(&r.name))
        .or_else(|| world.objects.iter().find(|o| o.id == place));
    // "the sofa": an object by what it is, when only one object is that.
    if target.is_none() {
        let named: Vec<&Thing> = world.objects.iter().filter(|o| same(&o.name)).collect();
        match named.as_slice() {
            [] => {}
            [one] => target = Some(one),
            many => {
                let ids: Vec<&str> = many.iter().take(8).map(|o| o.id.as_str()).collect();
                return Err(Problem::new(
                    id,
                    "place",
                    format!(
                        "`{place}` could be {} objects ({}); use one id, as find_objects gives it",
                        many.len(),
                        ids.join(", ")
                    ),
                ));
            }
        }
    }
    if let (Some(t), Some(skill)) = (target, catalog.skill(GO_TO_TARGET)) {
        let spot = Spot {
            id: t.id.clone(),
            xy: t.xy,
        };
        return Ok((skill, vec![(port(skill, "target"), t.id.clone())], spot));
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

    fn thing(id: &str, name: &str, x: f64, y: f64) -> Thing {
        Thing {
            id: id.to_owned(),
            name: name.to_owned(),
            xy: Some((x, y)),
        }
    }

    fn world() -> World {
        World {
            places: vec![PlaceConfig {
                near: Vec::new(),
                name: "dock".to_owned(),
                aliases: vec!["charging dock".to_owned()],
                frame: "map".to_owned(),
                pose: PlacePose {
                    x: 1.5,
                    y: -2.0,
                    yaw: 0.5,
                },
            }],
            rooms: vec![thing("R2", "kitchen", 5.2, 1.3)],
            objects: vec![
                thing("O17", "red mug", 5.0, 1.0),
                thing("O31", "basket", 5.3, 1.2),
            ],
            robot: Some((0.0, 0.0)),
            holding: BTreeMap::new(),
        }
    }

    fn plan(steps: &Value) -> Plan {
        serde_json::from_value(json!({"intent": "fetch the mug", "steps": steps})).unwrap()
    }

    fn catalog() -> Catalog {
        Catalog::parse(CATALOG).unwrap()
    }

    #[test]
    fn an_object_is_walked_to_by_what_it_is_and_the_end_state_follows_the_steps() {
        let p = plan(&json!([
            {"skill": "GoToPlace", "args": {"place": "Basket"}},
            {"skill": "PickObject", "args": {"object_id": "O17", "phrase": "red mug", "arm": "right"}},
            {"skill": "PlaceInto", "args": {"container_id": "O31", "phrase": "basket", "arm": "right"}}
        ]));
        let c = compile(&p, &catalog(), &world()).unwrap();
        assert!(c.xml.contains(r#"target="O31""#), "{}", c.xml);
        // The mug ends in the basket and the hand empty; the pick and place moved the base.
        assert_eq!(c.goal, ["inside(O17, O31)"]);
        let mut two = world();
        two.objects.push(thing("O32", "basket", 1.0, 1.0));
        let problems = compile(&p, &catalog(), &two).unwrap_err();
        assert!(
            problems[0]
                .message
                .contains("could be 2 objects (O31, O32)"),
            "{problems:?}"
        );
    }

    #[test]
    fn a_thing_only_the_detector_knows_is_walked_to_from_its_place() {
        let mut w = world();
        for (name, thing, x) in [("table_side", "cup_9", 2.0), ("shelf", "bin_2", 4.0)] {
            w.places.push(PlaceConfig {
                near: vec![thing.to_owned()],
                name: name.to_owned(),
                aliases: Vec::new(),
                frame: "map".to_owned(),
                pose: PlacePose {
                    x,
                    y: 0.0,
                    yaw: 0.0,
                },
            });
        }
        let p = plan(&json!([
            {"skill": "PickObject", "args": {"object_id": "cup_9", "phrase": "cup", "arm": "right"}},
            {"skill": "PlaceInto", "args": {"container_id": "bin_2", "phrase": "bin", "arm": "right"}}
        ]));
        let c = compile(&p, &catalog(), &w).unwrap();
        let summaries: Vec<&str> = c.steps.iter().map(|s| s.summary.as_str()).collect();
        assert_eq!(
            summaries[0], "GoToPlace(place=table_side) (added)",
            "{summaries:?}"
        );
        assert_eq!(
            summaries[2], "GoToPlace(place=shelf) (added)",
            "{summaries:?}"
        );
        assert_eq!(c.goal, ["inside(cup_9, bin_2)"]);
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
        assert!(has("s1", "grip") && has("s1", "phrase") && has("s1", "arm"));
        assert!(
            has("s2", "skill") && has("s3", "skill") && has("s4", "place") && has("s5", "retries")
        );
        assert!(problems.iter().any(|q| q.message.contains("find_objects")));
    }

    #[test]
    fn a_walk_is_added_before_a_skill_that_needs_the_robot_near() {
        let pick = json!({"skill": "PickObject", "args": {"object_id": "O17", "phrase": "red mug", "arm": "right"}});
        let c = compile(&plan(&json!([pick])), &catalog(), &world()).unwrap();
        let summaries: Vec<&str> = c.steps.iter().map(|s| s.summary.as_str()).collect();
        assert_eq!(
            summaries,
            [
                "GoToPlace(place=O17) (added)",
                "PickObject(arm=right, object_id=O17, phrase=red mug)"
            ]
        );
        assert!(
            c.xml
                .contains(r#"<SubTree ID="GoToTarget" name="s1_GoToPlace" target="O17"/>"#)
        );
        assert!(c.xml.contains(r#"name="s2_PickObject""#));
        let walked = plan(&json!([{"skill": "GoToPlace", "args": {"place": "O17"}}, pick]));
        assert_eq!(
            compile(&walked, &catalog(), &world()).unwrap().steps.len(),
            2
        );
        let mut close = world();
        close.robot = Some((5.2, 1.1));
        assert_eq!(
            compile(&plan(&json!([pick])), &catalog(), &close)
                .unwrap()
                .steps
                .len(),
            1
        );
        // An object the world model does not know, such as a detector's name for it, cannot be
        // judged or walked to: the plan stands as written and the executor checks it.
        let unknown = json!({"skill": "PickObject", "args": {"object_id": "red_block", "phrase": "red block", "arm": "right"}});
        let c = compile(&plan(&json!([unknown])), &catalog(), &world()).unwrap();
        assert_eq!(c.steps.len(), 1);
    }

    #[test]
    fn a_place_needs_the_hand_that_holds_the_object() {
        let steps = |place_arm: &str| {
            plan(&json!([
                {"skill": "GoToPlace", "args": {"place": "O17"}},
                {"skill": "PickObject", "args": {"object_id": "O17", "phrase": "red mug", "arm": "right"}},
                {"skill": "PlaceInto", "args": {"container_id": "O31", "phrase": "basket", "arm": place_arm}},
                {"skill": "PlaceInto", "args": {"container_id": "O31", "phrase": "basket", "arm": "right"}}
            ]))
        };
        let wrong = compile(&steps("left"), &catalog(), &world()).unwrap_err();
        assert!(
            wrong
                .iter()
                .any(|p| p.step == "s4" && p.message.contains("left hand to hold something")),
            "{wrong:?}"
        );
        // After the first place the right hand is empty, so placing again fails too.
        let twice = compile(&steps("right"), &catalog(), &world()).unwrap_err();
        assert_eq!(twice.len(), 1, "{twice:?}");
        assert_eq!(twice[0].step, "s6", "a walk is added before each place");
        let mut held = world();
        held.robot = Some((5.0, 1.0));
        held.holding.insert("left".to_owned(), "O17".to_owned());
        let place = plan(
            &json!([{"skill": "PlaceInto", "args": {"container_id": "O31", "phrase": "basket", "arm": "left"}}]),
        );
        assert!(
            compile(&place, &catalog(), &held).is_ok(),
            "a hand may hold something from before"
        );
    }

    #[test]
    fn after_a_skill_that_moves_the_base_the_next_one_walks_again() {
        let p = plan(&json!([
            {"skill": "GoToPlace", "args": {"place": "O17"}},
            {"skill": "PickObject", "args": {"object_id": "O17", "phrase": "red mug", "arm": "right"}},
            {"skill": "PlaceInto", "args": {"container_id": "O31", "phrase": "basket", "arm": "right"}}
        ]));
        let c = compile(&p, &catalog(), &world()).unwrap();
        let summaries: Vec<&str> = c.steps.iter().map(|s| s.summary.as_str()).collect();
        assert_eq!(
            summaries[2], "GoToPlace(place=O31) (added)",
            "{summaries:?}"
        );
        // Folding the arms does not take the base, so the robot stays where it was.
        let tuck = plan(&json!([
            {"skill": "GoToPlace", "args": {"place": "O17"}},
            {"skill": "TuckForTravel"},
            {"skill": "PickObject", "args": {"object_id": "O17", "phrase": "red mug", "arm": "right"}}
        ]));
        assert_eq!(compile(&tuck, &catalog(), &world()).unwrap().steps.len(), 3);
    }

    #[test]
    fn a_pick_needs_an_empty_hand() {
        let mut full = world();
        full.robot = Some((5.0, 1.0));
        full.holding.insert("left".to_owned(), "O31".to_owned());
        let pick = |arm: &str| {
            plan(
                &json!([{"skill": "PickObject", "args": {"object_id": "O17", "phrase": "red mug", "arm": arm}}]),
            )
        };
        let problems = compile(&pick("left"), &catalog(), &full).unwrap_err();
        assert!(
            problems[0]
                .message
                .contains("left hand empty, but it holds O31"),
            "{problems:?}"
        );
        assert!(compile(&pick("right"), &catalog(), &full).is_ok());
    }

    #[test]
    fn an_object_in_hand_is_not_picked_up_again() {
        let mut held = world();
        held.robot = Some((5.0, 1.0));
        held.holding.insert("right".to_owned(), "O17".to_owned());
        let p = plan(
            &json!([{"skill": "PickObject", "args": {"object_id": "O17", "phrase": "red mug", "arm": "left"}}]),
        );
        let problems = compile(&p, &catalog(), &held).unwrap_err();
        assert!(
            problems[0].message.contains("already in the right hand"),
            "{problems:?}"
        );
    }

    #[test]
    fn a_derived_argument_comes_from_the_world_model() {
        let mut c = catalog();
        let pick = c
            .skills
            .iter_mut()
            .find(|s| s.name == "PickObject")
            .unwrap();
        pick.args
            .iter_mut()
            .find(|a| a.name == "phrase")
            .unwrap()
            .default_from = Some("label(object_id)".to_owned());
        assert!(c.describe().contains("PickObject(object_id, arm)"));
        let p = plan(&json!([
            {"skill": "GoToPlace", "args": {"place": "O17"}},
            {"skill": "PickObject", "args": {"object_id": "O17", "arm": "right"}}
        ]));
        let compiled = compile(&p, &c, &world()).unwrap();
        assert!(
            compiled
                .xml
                .contains(r#"object_id="O17" phrase="red mug" arm="right""#),
            "{}",
            compiled.xml
        );
    }

    #[test]
    fn text_is_escaped() {
        let mut p = plan(
            &json!([{"skill": "PickObject", "args": {"object_id": "O17", "phrase": "\"mug\" & <cup>", "arm": "left"}}]),
        );
        p.intent = "a <b> & \"c\"".to_owned();
        let mut near = world();
        near.robot = Some((5.0, 1.0));
        let c = compile(&p, &catalog(), &near).unwrap();
        assert!(
            c.xml
                .contains("phrase=\"&quot;mug&quot; &amp; &lt;cup&gt;\"")
        );
        // Node names must be identifiers, so the intent stays out of the tree.
        assert!(c.xml.contains("<Sequence name=\"mission\">"));
        assert!(!c.xml.contains("&lt;b&gt;"));
    }

    #[test]
    fn a_name_the_world_model_does_not_know_gives_its_own_words() {
        let mut c = catalog();
        let pick = c
            .skills
            .iter_mut()
            .find(|s| s.name == "PickObject")
            .unwrap();
        pick.args
            .iter_mut()
            .find(|a| a.name == "phrase")
            .unwrap()
            .default_from = Some("label(object_id)".to_owned());
        let p = plan(
            &json!([{"skill": "PickObject", "args": {"object_id": "red_block", "arm": "left"}}]),
        );
        let compiled = compile(&p, &c, &world()).unwrap();
        assert!(
            compiled
                .xml
                .contains(r#"object_id="red_block" phrase="red block""#),
            "{}",
            compiled.xml
        );
    }

    #[test]
    fn braces_are_refused() {
        let p = plan(
            &json!([{"skill": "PickObject", "args": {"object_id": "O17", "phrase": "{secret}", "arm": "left"}}]),
        );
        let mut near = world();
        near.robot = Some((5.0, 1.0));
        let problems = compile(&p, &catalog(), &near).unwrap_err();
        assert!(
            problems[0].message.contains("may not contain { or }"),
            "{problems:?}"
        );
    }
}
