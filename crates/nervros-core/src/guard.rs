//! The guard. Every tool call passes it before it runs.
//!
//! Its rules are deterministic and come from the profile; it never asks a model. It refuses tools
//! whose ROS names are on the hard-deny list, keeps act-lane tools off until the operator arms the
//! agent, enforces per-turn budgets, breaks call loops, and serialises motion with resource locks.
//! Refusals carry a message the model can read and act on.

use std::collections::{BTreeSet, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde::Deserialize;
use serde_json::Value;

use crate::lock;
use crate::tools::{Lane, Resource, ToolSpec};

/// How much the agent may do without asking.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Autonomy {
    /// Observe tools only.
    Observe,
    /// Edit and act tools need the operator's approval.
    #[default]
    Supervised,
    /// Edit and act tools run without approval.
    Autonomous,
}

/// Per-turn limits.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Budgets {
    /// Model calls per user turn.
    #[serde(default = "d_model_calls")]
    pub model_calls: u32,
    /// Tool calls per user turn.
    #[serde(default = "d_tool_calls")]
    pub tool_calls: u32,
    /// Wall time per user turn, not counting mission time.
    #[serde(default = "d_wall", deserialize_with = "crate::profile::duration")]
    pub wall_time: Duration,
    /// The same call this many times in a row ends the loop.
    #[serde(default = "d_repeat")]
    pub repeat_break: u32,
}

// A mission takes a lookup, a plan or two, the run and the reply; small models need more.
fn d_model_calls() -> u32 {
    10
}
fn d_tool_calls() -> u32 {
    12
}
fn d_wall() -> Duration {
    Duration::from_secs(90)
}
fn d_repeat() -> u32 {
    3
}

impl Default for Budgets {
    fn default() -> Self {
        Self {
            model_calls: d_model_calls(),
            tool_calls: d_tool_calls(),
            wall_time: d_wall(),
            repeat_break: d_repeat(),
        }
    }
}

/// The `[policy]` table.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Policy {
    /// Armed at start; the safe default is false.
    #[serde(default)]
    pub start_armed: bool,
    /// Autonomy level.
    #[serde(default)]
    pub autonomy: Autonomy,
    /// How long an approval request stays open.
    #[serde(default = "d_ttl", deserialize_with = "crate::profile::duration")]
    pub approval_ttl: Duration,
    /// Per-turn limits.
    #[serde(default)]
    pub budgets: Budgets,
    /// ROS names no tool may reach; `*` matches any run of characters. Setting this replaces the
    /// defaults, so a robot that truly needs one of them must list the rest itself.
    #[serde(default = "default_hard_deny")]
    pub hard_deny: Vec<String>,
    /// Interface types no generic tool may call, send or publish, globbed like `hard_deny`:
    /// velocity and joint commands, controller switches, lifecycle changes.
    #[serde(default = "default_hard_deny_types")]
    pub hard_deny_types: Vec<String>,
    /// Refusals or approvals the profile adds for particular arguments, `[[policy.rule]]`. They
    /// only ever tighten: no rule lets through what the guard would stop.
    #[serde(default, rename = "rule")]
    pub rules: Vec<ArgRule>,
}

/// A tightening for one tool's arguments: never this value, or ask the operator first.
#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct ArgRule {
    /// The tool, or `*` for every tool.
    pub tool: String,
    /// The argument, as a dotted path; `*` stands for every entry of a list, as `steps.*.skill`.
    pub arg: String,
    /// The values it applies to, globbed like `hard_deny`; a number or a flag as its text.
    pub matches: String,
    /// Refuse the call, or ask the operator first.
    pub then: RuleAction,
    /// Why, for the model and the operator.
    pub reason: String,
}

/// What a rule does.
#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum RuleAction {
    /// The call is refused.
    Deny,
    /// The operator approves the call first, even where nothing else would ask.
    Ask,
}

/// The values at a dotted path, every entry of a list where the path says `*`.
fn values_at<'a>(value: &'a Value, path: &[&str]) -> Vec<&'a Value> {
    let Some((first, rest)) = path.split_first() else {
        return vec![value];
    };
    match (*first, value) {
        ("*", Value::Array(items)) => items.iter().flat_map(|v| values_at(v, rest)).collect(),
        ("*", Value::Object(fields)) => fields.values().flat_map(|v| values_at(v, rest)).collect(),
        (key, Value::Object(fields)) => fields
            .get(key)
            .map_or_else(Vec::new, |v| values_at(v, rest)),
        (index, Value::Array(items)) => index
            .parse::<usize>()
            .ok()
            .and_then(|i| items.get(i))
            .map_or_else(Vec::new, |v| values_at(v, rest)),
        _ => Vec::new(),
    }
}

fn d_ttl() -> Duration {
    Duration::from_mins(1)
}

/// Names that command motors, velocities, controllers or the collision picture directly, under
/// any namespace: a robot launched as `/g1` has `/g1/clear_octomap`.
#[must_use]
pub fn default_hard_deny() -> Vec<String> {
    [
        "rt/lowcmd",
        "*/lowcmd",
        "*/rt/*",
        "*cmd_vel*",
        "*/controller_manager/*",
        "*/set_parameters",
        "*/set_parameters_atomically",
        "*/joint_trajectory",
        "*/follow_joint_trajectory",
        "*/servo_node/*",
        "*/apply_planning_scene",
        "*/clear_octomap",
    ]
    .map(str::to_owned)
    .to_vec()
}

/// Interface types that command motors, velocities, controllers or lifecycles directly.
#[must_use]
pub fn default_hard_deny_types() -> Vec<String> {
    [
        "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/TwistStamped",
        "trajectory_msgs/msg/*",
        "control_msgs/action/*",
        "control_msgs/msg/JointJog",
        "controller_manager_msgs/srv/*",
        "lifecycle_msgs/srv/ChangeState",
        "rcl_interfaces/srv/SetParameters*",
    ]
    .map(str::to_owned)
    .to_vec()
}

impl Default for Policy {
    fn default() -> Self {
        Self {
            start_armed: false,
            autonomy: Autonomy::default(),
            approval_ttl: d_ttl(),
            budgets: Budgets::default(),
            hard_deny: default_hard_deny(),
            hard_deny_types: default_hard_deny_types(),
            rules: Vec::new(),
        }
    }
}

/// A ROS name from the model, as it will be used: absolute, each part a letter or `_` then
/// letters, digits or `_`. A relative name is refused rather than resolved, since `cmd_vel` would
/// reach `/cmd_vel` past a deny list written for absolute names.
///
/// # Errors
///
/// Why the name is not one.
pub fn canonical_ros_name(name: &str) -> Result<String, String> {
    let name = name.trim();
    let Some(rest) = name.strip_prefix('/') else {
        return Err(format!(
            "`{name}` must be an absolute ROS name, starting with /"
        ));
    };
    let valid = !rest.is_empty()
        && rest.split('/').all(|part| {
            let mut chars = part.chars();
            chars
                .next()
                .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
                && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
        });
    if valid {
        Ok(name.to_owned())
    } else {
        Err(format!(
            "`{name}` is not a ROS name: parts of letters, digits and _, split by /"
        ))
    }
}

/// Why a call may not run. The message is shown to the model.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Refusal {
    /// Which rule refused.
    pub rule: &'static str,
    /// What to tell the model.
    pub message: String,
}

impl Refusal {
    /// A refusal by `rule`, telling the model `message`.
    #[must_use]
    pub fn new(rule: &'static str, message: impl Into<String>) -> Self {
        Self {
            rule,
            message: message.into(),
        }
    }
}

/// What the guard says about one call.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Decision {
    /// Run it.
    Allow,
    /// Ask the operator first.
    NeedApproval {
        /// Why approval is needed.
        reason: String,
    },
    /// Do not run it.
    Deny(Refusal),
}

#[derive(Debug, Default)]
struct State {
    armed: bool,
    /// Stops of the robot so far: what started before one ends with it.
    stops: u64,
    tool_calls: u32,
    last_calls: VecDeque<(String, String)>,
    locked: BTreeSet<Resource>,
}

/// The guard, shared by the session and its tools.
#[derive(Debug)]
pub struct Guard {
    policy: Policy,
    state: Arc<Mutex<State>>,
}

/// `*` matches any run of characters, including none; everything else matches itself.
#[must_use]
pub fn glob_match(pattern: &str, name: &str) -> bool {
    let parts: Vec<&str> = pattern.split('*').collect();
    let [first, middle @ .., last] = parts.as_slice() else {
        return pattern == name;
    };
    if parts.len() == 1 {
        return pattern == name;
    }
    let Some(mut rest) = name.strip_prefix(first) else {
        return false;
    };
    for part in middle {
        match rest.find(part) {
            Some(i) => rest = &rest[i + part.len()..],
            None => return false,
        }
    }
    rest.len() >= last.len() && rest.ends_with(last)
}

impl Guard {
    /// A guard for a policy.
    #[must_use]
    pub fn new(policy: Policy) -> Self {
        let state = State {
            armed: policy.start_armed,
            ..State::default()
        };
        Self {
            policy,
            state: Arc::new(Mutex::new(state)),
        }
    }

    /// The policy it enforces.
    #[must_use]
    pub fn policy(&self) -> &Policy {
        &self.policy
    }

    /// Whether a ROS name is on the hard-deny list. A relative name is taken as the node's own
    /// namespace takes it, under `/`.
    #[must_use]
    pub fn hard_denied(&self, ros_name: &str) -> bool {
        let name = ros_name.trim();
        let absolute = if name.starts_with('/') {
            name.to_owned()
        } else {
            format!("/{name}")
        };
        self.policy
            .hard_deny
            .iter()
            .any(|p| glob_match(p, name) || glob_match(p, &absolute))
    }

    /// Whether an interface type is on the hard-deny list for generic calls and publishing. A
    /// type written `pkg/Name` is checked as every kind it could be.
    #[must_use]
    pub fn hard_denied_type(&self, ros_type: &str) -> bool {
        let ty = ros_type.trim();
        let spellings: Vec<String> = match ty.split('/').collect::<Vec<_>>().as_slice() {
            [package, name] => ["msg", "srv", "action"]
                .iter()
                .map(|kind| format!("{package}/{kind}/{name}"))
                .collect(),
            _ => vec![ty.to_owned()],
        };
        spellings
            .iter()
            .any(|t| self.policy.hard_deny_types.iter().any(|p| glob_match(p, t)))
    }

    /// Whether act-lane tools are enabled.
    #[must_use]
    pub fn armed(&self) -> bool {
        lock(&self.state).armed
    }

    /// Arms or disarms. Disarming blocks new motion and cancels nothing by itself.
    pub fn set_armed(&self, armed: bool) {
        lock(&self.state).armed = armed;
    }

    /// Counts a stop of the robot, from the window or the model.
    pub fn note_stop(&self) {
        lock(&self.state).stops += 1;
    }

    /// Stops of the robot so far. Work started on the robot's behalf, such as a schedule, ends
    /// once this differs from what it was when the work began.
    #[must_use]
    pub fn stops(&self) -> u64 {
        lock(&self.state).stops
    }

    /// Resets the per-turn counters at the start of a user turn.
    pub fn begin_turn(&self) {
        let mut s = lock(&self.state);
        s.tool_calls = 0;
        s.last_calls.clear();
    }

    /// For a call the operator asks for directly, such as a plan from a click on the map: the
    /// budgets and the loop breaker are for the model, so they do not count it; an observe-only
    /// profile and a disarmed robot still refuse it; and it asks before it acts, as the plan's
    /// card is how the operator sees what they asked for.
    #[must_use]
    pub fn decide_operator(&self, spec: &ToolSpec) -> Decision {
        let lane = spec.lane();
        if lane != Lane::Observe && self.policy.autonomy == Autonomy::Observe {
            return Decision::Deny(Refusal::new(
                "observe_only",
                "this profile only observes the robot",
            ));
        }
        if lane == Lane::Act && !lock(&self.state).armed {
            return Decision::Deny(Refusal::new(
                "disarmed",
                "the robot is disarmed: arm it in the top bar first",
            ));
        }
        if lane == Lane::Act {
            Decision::NeedApproval {
                reason: format!("`{}` acts on the robot", spec.name),
            }
        } else {
            Decision::Allow
        }
    }

    /// The profile's rule for a call: a refusal over a request to ask. Values match whatever
    /// their case and surrounding spaces, as the tools read them.
    #[must_use]
    pub fn rule(&self, tool: &str, args: &Value) -> Option<&ArgRule> {
        let hits = || {
            self.policy.rules.iter().filter(|r| {
                let pattern = r.matches.trim().to_lowercase();
                (r.tool == "*" || r.tool == tool)
                    && values_at(args, &r.arg.split('.').collect::<Vec<_>>())
                        .iter()
                        .any(|v| {
                            let text = v.as_str().map_or_else(|| v.to_string(), str::to_owned);
                            glob_match(&pattern, &text.trim().to_lowercase())
                        })
            })
        };
        hits()
            .find(|r| r.then == RuleAction::Deny)
            .or_else(|| hits().next())
    }

    /// Decides one call. An allowed call counts against the turn's budget.
    #[must_use]
    pub fn decide(&self, spec: &ToolSpec, args: &Value) -> Decision {
        let mut s = lock(&self.state);
        let budgets = &self.policy.budgets;
        if s.tool_calls >= budgets.tool_calls {
            return Decision::Deny(Refusal::new(
                "budget",
                format!(
                    "this turn has used its {} tool calls; answer with what you have",
                    budgets.tool_calls
                ),
            ));
        }
        let key = (spec.name.clone(), args.to_string());
        let repeats = s.last_calls.iter().rev().take_while(|k| **k == key).count();
        if u32::try_from(repeats).unwrap_or(u32::MAX) + 1 >= budgets.repeat_break {
            return Decision::Deny(Refusal::new(
                "loop",
                format!(
                    "`{}` was just called with the same arguments; do something else or answer",
                    spec.name
                ),
            ));
        }
        if spec.lane() != Lane::Observe && self.policy.autonomy == Autonomy::Observe {
            return Decision::Deny(Refusal::new(
                "observe_only",
                "the agent is in observe mode; describe instead of acting",
            ));
        }
        if spec.lane() == Lane::Act && !s.armed {
            return Decision::Deny(Refusal::new(
                "disarmed",
                "the robot is disarmed; tell the user to arm it in the app before it can act",
            ));
        }
        s.tool_calls += 1;
        s.last_calls.push_back(key);
        if s.last_calls.len() > 16 {
            s.last_calls.pop_front();
        }
        if self.policy.autonomy == Autonomy::Supervised {
            match spec.lane() {
                Lane::Act => {
                    return Decision::NeedApproval {
                        reason: format!("`{}` acts on the robot", spec.name),
                    };
                }
                Lane::Edit => {
                    return Decision::NeedApproval {
                        reason: format!("`{}` changes what the robot remembers", spec.name),
                    };
                }
                Lane::Observe => {}
            }
        }
        Decision::Allow
    }

    /// Takes the resources a call needs for as long as the returned guard lives.
    ///
    /// # Errors
    ///
    /// A refusal naming the resource already in use: one motion at a time.
    pub fn lock(&self, resources: &[Resource]) -> Result<ResourceLock, Refusal> {
        let mut s = lock(&self.state);
        if let Some(busy) = resources.iter().find(|r| s.locked.contains(r)) {
            return Err(Refusal::new(
                "busy",
                format!("the {busy:?} is already in use by another action; wait for it or stop it"),
            ));
        }
        s.locked.extend(resources.iter().copied());
        Ok(ResourceLock {
            state: Arc::clone(&self.state),
            resources: resources.to_vec(),
        })
    }
}

/// Resources held until drop.
#[derive(Debug)]
pub struct ResourceLock {
    state: Arc<Mutex<State>>,
    resources: Vec<Resource>,
}

impl Drop for ResourceLock {
    fn drop(&mut self) {
        let mut s = lock(&self.state);
        for r in &self.resources {
            s.locked.remove(r);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::Risk;
    use serde_json::json;

    fn spec(name: &str, risk: Risk) -> ToolSpec {
        ToolSpec::new(name, "test", json!({"type": "object"}), risk)
    }

    #[test]
    fn a_rule_reads_the_arguments_by_path_and_a_refusal_beats_a_question() {
        let policy: Policy = toml::from_str(
            r#"
            [[rule]]
            tool = "run_mission"
            arg = "steps.*.skill"
            matches = "Pick*"
            then = "ask"
            reason = "picking needs a person watching"
            [[rule]]
            tool = "*"
            arg = "steps.*.args.object_id"
            matches = "knife*"
            then = "deny"
            reason = "the robot does not handle knives"
            [[rule]]
            tool = "schedule"
            arg = "times"
            matches = "4*"
            then = "ask"
            reason = "a schedule this long"
            "#,
        )
        .unwrap();
        let guard = Guard::new(policy);
        let plan = |object: &str| {
            json!({"steps": [{"skill": "GoToPlace"},
                             {"skill": "PickObject", "args": {"object_id": object}}]})
        };
        let ask = guard.rule("run_mission", &plan("mug_4")).unwrap();
        assert_eq!(ask.then, RuleAction::Ask);
        let deny = guard.rule("run_mission", &plan("knife_2")).unwrap();
        assert_eq!(deny.reason, "the robot does not handle knives");
        assert_eq!(
            guard.rule("schedule", &json!({"times": 48})).unwrap().then,
            RuleAction::Ask,
            "a number by its text"
        );
        assert!(guard.rule("schedule", &json!({"times": 5})).is_none());
        assert!(
            guard
                .rule("run_mission", &json!({"hash": "ab12"}))
                .is_none()
        );
    }

    #[test]
    fn glob_matches_like_a_shell() {
        assert!(glob_match(
            "/controller_manager/*",
            "/controller_manager/switch_controller"
        ));
        assert!(glob_match(
            "*/set_parameters",
            "/g1_detector/set_parameters"
        ));
        assert!(!glob_match(
            "*/set_parameters",
            "/g1_detector/get_parameters"
        ));
        assert!(glob_match("/cmd_vel", "/cmd_vel"));
        assert!(!glob_match("/cmd_vel", "/cmd_vel_nav"));
        assert!(glob_match("a*b*c", "axxbyyc"));
        assert!(!glob_match("a*b*c", "axxc"));
    }

    #[test]
    fn an_edit_needs_no_arming_but_is_approved_when_supervised() {
        let g = Guard::new(Policy::default());
        let edit = spec("edit_world", Risk::Annotate);
        match g.decide(&edit, &json!({})) {
            Decision::NeedApproval { reason } => {
                assert!(reason.contains("remembers"), "{reason}");
            }
            other => panic!("{other:?}"),
        }
        assert!(matches!(
            g.decide(&spec("walk", Risk::Motion), &json!({})),
            Decision::Deny(r) if r.rule == "disarmed"
        ));
        let observing = Guard::new(Policy {
            autonomy: Autonomy::Observe,
            ..Policy::default()
        });
        assert!(matches!(
            observing.decide(&edit, &json!({})),
            Decision::Deny(r) if r.rule == "observe_only"
        ));
    }

    #[test]
    fn default_list_denies_motor_and_controller_names() {
        let g = Guard::new(Policy::default());
        for name in [
            "rt/lowcmd",
            "/cmd_vel",
            "/controller_manager/switch_controller",
            "/arm/joint_trajectory",
            "/cmd_vel_nav",
            "/g1/cmd_vel_smoothed",
            "/rt/lowcmd",
        ] {
            assert!(g.hard_denied(name), "{name}");
        }
        assert!(!g.hard_denied("/canopy/find_objects"));
        for name in [
            "/g1/clear_octomap",
            "/g1/apply_planning_scene",
            "/g1/servo_node/start_servo",
            "controller_manager/switch_controller",
        ] {
            assert!(g.hard_denied(name), "{name}, under a namespace or relative");
        }
        assert!(g.hard_denied_type("geometry_msgs/Twist"), "two parts");
        for ty in [
            "geometry_msgs/msg/Twist",
            "trajectory_msgs/msg/JointTrajectory",
            "control_msgs/action/FollowJointTrajectory",
            "controller_manager_msgs/srv/SwitchController",
            "rcl_interfaces/srv/SetParametersAtomically",
        ] {
            assert!(g.hard_denied_type(ty), "{ty}");
        }
        assert!(!g.hard_denied_type("std_srvs/srv/Trigger"));
    }

    #[test]
    fn a_name_from_the_model_must_be_absolute_and_plain() {
        assert_eq!(canonical_ros_name(" /a/b_2 "), Ok("/a/b_2".to_owned()));
        for bad in [
            "cmd_vel",
            "~/cmd_vel",
            "//cmd_vel",
            "/cmd_vel/",
            "/",
            "/a/{x}",
            "/1a",
            "/a b",
        ] {
            assert!(canonical_ros_name(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn act_tools_need_arming_then_approval() {
        let g = Guard::new(Policy::default());
        let act = spec("run_mission", Risk::Motion);
        assert!(matches!(
            g.decide(&act, &json!({})),
            Decision::Deny(Refusal {
                rule: "disarmed",
                ..
            })
        ));
        g.set_armed(true);
        assert!(matches!(
            g.decide(&act, &json!({})),
            Decision::NeedApproval { .. }
        ));
        let look = spec("look", Risk::Observe);
        assert_eq!(g.decide(&look, &json!({})), Decision::Allow);
    }

    #[test]
    fn budgets_and_loops_are_enforced() {
        let policy = Policy {
            budgets: Budgets {
                tool_calls: 4,
                repeat_break: 3,
                ..Budgets::default()
            },
            ..Policy::default()
        };
        let g = Guard::new(policy);
        let t = spec("find_objects", Risk::Observe);
        assert_eq!(g.decide(&t, &json!({"q": 1})), Decision::Allow);
        assert_eq!(g.decide(&t, &json!({"q": 1})), Decision::Allow);
        assert!(matches!(
            g.decide(&t, &json!({"q": 1})),
            Decision::Deny(Refusal { rule: "loop", .. })
        ));
        assert_eq!(g.decide(&t, &json!({"q": 2})), Decision::Allow);
        assert_eq!(g.decide(&t, &json!({"q": 3})), Decision::Allow);
        assert!(matches!(
            g.decide(&t, &json!({"q": 4})),
            Decision::Deny(Refusal { rule: "budget", .. })
        ));
        g.begin_turn();
        assert_eq!(g.decide(&t, &json!({"q": 4})), Decision::Allow);
    }

    #[test]
    fn one_motion_at_a_time() {
        let g = Guard::new(Policy::default());
        let held = g.lock(&[Resource::Base]).unwrap();
        assert_eq!(
            g.lock(&[Resource::Base, Resource::RightArm])
                .unwrap_err()
                .rule,
            "busy"
        );
        let arm = g.lock(&[Resource::LeftArm]).unwrap();
        drop(held);
        assert!(g.lock(&[Resource::Base]).is_ok());
        drop(arm);
    }
}
