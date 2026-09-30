//! The guard. Every tool call passes it before it runs.
//!
//! Its rules are deterministic and come from the profile; it never asks a model. It refuses tools
//! whose ROS names are on the hard-deny list, keeps act-lane tools off until the operator arms the
//! agent, enforces per-turn budgets, breaks call loops, and serialises motion with resource locks.
//! Refusals carry a message the model can read and act on.

use std::collections::{BTreeSet, VecDeque};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use serde::Deserialize;
use serde_json::Value;

use crate::tools::{Lane, Resource, ToolSpec};

/// How much the agent may do without asking.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Autonomy {
    /// Observe tools only.
    Observe,
    /// Act-lane tools need the operator's approval.
    #[default]
    Supervised,
    /// Act-lane tools run without approval.
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

fn d_model_calls() -> u32 {
    6
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
}

fn d_ttl() -> Duration {
    Duration::from_mins(1)
}

/// Names that command motors, velocities, controllers or the collision picture directly.
#[must_use]
pub fn default_hard_deny() -> Vec<String> {
    [
        "rt/lowcmd",
        "/lowcmd",
        "/cmd_vel",
        "*/cmd_vel",
        "/controller_manager/*",
        "*/set_parameters",
        "*/set_parameters_atomically",
        "*/joint_trajectory",
        "*/follow_joint_trajectory",
        "/servo_node/*",
        "/apply_planning_scene",
        "/clear_octomap",
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
        }
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
    fn new(rule: &'static str, message: impl Into<String>) -> Self {
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

fn lock(state: &Mutex<State>) -> MutexGuard<'_, State> {
    state.lock().unwrap_or_else(PoisonError::into_inner)
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

    /// Whether a ROS name is on the hard-deny list.
    #[must_use]
    pub fn hard_denied(&self, ros_name: &str) -> bool {
        self.policy
            .hard_deny
            .iter()
            .any(|p| glob_match(p, ros_name))
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

    /// Resets the per-turn counters at the start of a user turn.
    pub fn begin_turn(&self) {
        let mut s = lock(&self.state);
        s.tool_calls = 0;
        s.last_calls.clear();
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
        if spec.lane() == Lane::Act {
            if self.policy.autonomy == Autonomy::Observe {
                return Decision::Deny(Refusal::new(
                    "observe_only",
                    "the agent is in observe mode; describe instead of acting",
                ));
            }
            if !s.armed {
                return Decision::Deny(Refusal::new(
                    "disarmed",
                    "the robot is disarmed; tell the user to arm it in the app before it can act",
                ));
            }
        }
        s.tool_calls += 1;
        s.last_calls.push_back(key);
        if s.last_calls.len() > 16 {
            s.last_calls.pop_front();
        }
        if spec.lane() == Lane::Act && self.policy.autonomy == Autonomy::Supervised {
            return Decision::NeedApproval {
                reason: format!("`{}` acts on the robot", spec.name),
            };
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
    fn default_list_denies_motor_and_controller_names() {
        let g = Guard::new(Policy::default());
        for name in [
            "rt/lowcmd",
            "/cmd_vel",
            "/controller_manager/switch_controller",
            "/arm/joint_trajectory",
        ] {
            assert!(g.hard_denied(name), "{name}");
        }
        assert!(!g.hard_denied("/canopy/find_objects"));
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
