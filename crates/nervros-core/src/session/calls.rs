//! Tool calls: the guard's decision, the profile's rules, and running a call.

use std::borrow::Cow;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Instant;

use serde_json::Value;
use serde_json::json;

use super::Shared;
use super::approval::DENIAL_BREAK;
use super::event::Event;
use super::millis;
use super::turn::TurnFlags;
use crate::guard::Decision;
use crate::guard::Refusal;
use crate::guard::RuleAction;
use crate::lock;
use crate::tools::Assessment;
use crate::tools::Lane;
use crate::tools::Resource;
use crate::tools::Risk;
use crate::tools::Status;
use crate::tools::Tool;
use crate::tools::ToolOutcome;
use crate::tools::ToolSpec;

/// Who asked for a tool call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Caller {
    /// The model, in a turn.
    Model,
    /// The operator, from the window.
    Operator,
}

/// What an approval settled: the arguments to run with and what they occupy.
pub(super) struct Approved {
    pub(super) args: Value,
    pub(super) resources: Vec<Resource>,
}

/// The same failure this many times in a row in a turn is stuck, not unlucky.
const STUCK_AFTER: u32 = 3;

/// Ends a tool call's chip however the call ends: a stop or the turn's time limit drops the
/// call before it returns.
struct Calling<'a> {
    shared: &'a Shared,
    turn: u64,
    call: u64,
    tool: String,
    started: Instant,
    done: bool,
}

impl Drop for Calling<'_> {
    fn drop(&mut self) {
        if !self.done {
            self.shared.emit(Event::ToolFinished {
                turn: self.turn,
                call: self.call,
                tool: std::mem::take(&mut self.tool),
                status: Status::Stopped,
                message: "stopped before it finished".to_owned(),
                ms: millis(self.started.elapsed()),
            });
        }
    }
}

/// The stronger of two rules: a refusal over a request to ask.
fn stronger(
    a: Option<crate::guard::ArgRule>,
    b: Option<crate::guard::ArgRule>,
) -> Option<crate::guard::ArgRule> {
    match (a, b) {
        (Some(a), _) if a.then == RuleAction::Deny => Some(a),
        (_, Some(b)) if b.then == RuleAction::Deny => Some(b),
        (a, b) => a.or(b),
    }
}

impl Shared {
    async fn run(&self, tool: &Arc<dyn Tool>, args: Value, resources: &[Resource]) -> ToolOutcome {
        let _held = if resources.is_empty() {
            None
        } else {
            match self.guard.lock(resources) {
                Ok(l) => Some(l),
                Err(r) => return ToolOutcome::refused(r.message),
            }
        };
        tool.call(args).await
    }

    /// What a call would do, for a tool whose risk depends on its arguments, and the answer when
    /// it cannot go out at all, before anyone is asked to approve it. The model's arguments are
    /// checked against the schema first: a skill name it made up ends here.
    async fn assess(
        tool: &Arc<dyn Tool>,
        args: &Value,
        caller: Caller,
    ) -> Option<Result<Assessment, ToolOutcome>> {
        match caller {
            Caller::Model => {
                let invalid = crate::argcheck::check(&tool.spec().parameters, args);
                if invalid.is_empty() {
                    tool.assess(args).await
                } else {
                    Some(Err(ToolOutcome::failed(format!(
                        "{}; call it again with arguments that fit",
                        invalid.join("; ")
                    ))))
                }
            }
            Caller::Operator => match tool.assess_operator(args.clone()).await {
                Some(checked) => Some(checked),
                None => tool.assess(args).await,
            },
        }
    }

    /// Asks the operator, then runs it if they approve. After `DENIAL_BREAK` denials in a row the
    /// model is told to stop asking, and asks no more until the operator approves or speaks.
    async fn approved_run(
        &self,
        tool: &Arc<dyn Tool>,
        spec: &ToolSpec,
        args: Value,
        reason: String,
        caller: Caller,
    ) -> ToolOutcome {
        if caller == Caller::Model && self.denials.load(Ordering::SeqCst) >= DENIAL_BREAK {
            return ToolOutcome::refused(format!(
                "the operator turned down the last {DENIAL_BREAK} requests: stop asking, and ask \
                 them what they want instead"
            ));
        }
        let asked = Approved {
            args,
            resources: spec.resources.clone(),
        };
        if let Some(a) = self
            .ask_approval(tool, &spec.name, reason, spec.risk, asked)
            .await
        {
            self.denials.store(0, Ordering::SeqCst);
            // Disarming during the wait outranks an approval given before it.
            if spec.lane() == Lane::Act && !self.guard.armed() {
                return ToolOutcome::refused(
                    "the robot was disarmed while this waited for approval; nothing ran",
                );
            }
            return self.run(tool, a.args, &a.resources).await;
        }
        let n = self.denials.fetch_add(1, Ordering::SeqCst) + 1;
        ToolOutcome::refused(if n >= DENIAL_BREAK {
            format!(
                "the operator did not approve this, {n} times in a row: stop acting and ask them \
                 what they want"
            )
        } else {
            "the operator did not approve this".to_owned()
        })
    }

    /// The guard's decision, tightened by the profile's rule for this call: its refusal stands
    /// over everything, and its question makes an allowed call ask.
    fn decide(
        &self,
        spec: &ToolSpec,
        args: &Value,
        caller: Caller,
        rule: Option<crate::guard::ArgRule>,
    ) -> Decision {
        if let Some(r) = rule.as_ref().filter(|r| r.then == RuleAction::Deny) {
            return Decision::Deny(Refusal::new("rule", r.reason.clone()));
        }
        let decision = match caller {
            Caller::Model => self.guard.decide(spec, args),
            Caller::Operator => self.guard.decide_operator(spec),
        };
        let decision = match (decision, rule) {
            (Decision::Allow, Some(r)) => Decision::NeedApproval { reason: r.reason },
            // Allowed for the session, unless this call moves the robot.
            (Decision::NeedApproval { .. }, None)
                if caller == Caller::Model
                    && !matches!(spec.risk, Risk::Motion | Risk::Manipulation)
                    && lock(&self.allowed).contains(&spec.name) =>
            {
                Decision::Allow
            }
            (decision, _) => decision,
        };
        let (verdict, why) = match &decision {
            Decision::Allow => ("allow", ""),
            Decision::NeedApproval { reason } => ("ask", reason.as_str()),
            Decision::Deny(r) => ("deny", r.message.as_str()),
        };
        tracing::info!(nervros.tool = %spec.name, nervros.guard = verdict, why, "guard");
        decision
    }

    pub(super) async fn invoke(
        &self,
        tool: &Arc<dyn Tool>,
        turn: u64,
        args: Value,
        flags: &TurnFlags,
        caller: Caller,
    ) -> Value {
        let call = self.calls.fetch_add(1, Ordering::Relaxed) + 1;
        // The stop is never held up: no budget, loop breaker, rule or argument check applies.
        let stopping = self.stop.as_ref().is_some_and(|s| Arc::ptr_eq(s, tool));
        let (assessed, rule) = if stopping {
            (None, None)
        } else {
            // The profile's rules read the arguments as they were sent, before checking settled
            // them.
            let rule = self.guard.rule(&tool.spec().name, &args).cloned();
            (Self::assess(tool, &args, caller).await, rule)
        };
        let (mut assessment, early) = match assessed {
            Some(Ok(a)) => (Some(a), None),
            Some(Err(out)) => (None, Some(out)),
            None => (None, None),
        };
        let args = assessment
            .as_mut()
            .and_then(|a| a.args.take())
            .unwrap_or(args);
        // And as the call will run, such as a plan by its hash: the model's spelling of a place
        // does not slip past a rule on where it resolves to.
        let rule = if stopping {
            None
        } else {
            let view = tool.rule_view(&args);
            let settled = view.and_then(|v| self.guard.rule(&tool.spec().name, &v).cloned());
            stronger(rule, settled)
        };
        let asked_by_rule = rule
            .as_ref()
            .filter(|r| r.then == RuleAction::Ask)
            .map(|r| r.reason.clone());
        let spec = match &assessment {
            Some(a) => {
                let mut spec = tool.spec().into_owned();
                spec.risk = a.risk;
                spec.resources.clone_from(&a.resources);
                Cow::Owned(spec)
            }
            None => tool.spec(),
        };
        self.emit(Event::ToolStarted {
            turn,
            call,
            tool: spec.name.clone(),
            args: args.clone(),
        });
        let mut calling = Calling {
            shared: self,
            turn,
            call,
            tool: spec.name.clone(),
            started: Instant::now(),
            done: false,
        };
        let outcome = match early {
            Some(out) => out,
            None if stopping => {
                self.guard.note_stop();
                tool.call(args).await
            }
            None => match self.decide(&spec, &args, caller, rule) {
                Decision::Deny(r) => ToolOutcome::refused(r.message),
                Decision::NeedApproval { reason } => {
                    // What the call does, and why the profile asks when it does.
                    let reason = match (assessment.map(|a| a.reason), asked_by_rule) {
                        (Some(what), Some(why)) => format!("{what}; the profile asks: {why}"),
                        (Some(what), None) => what,
                        (None, _) => reason,
                    };
                    self.approved_run(tool, &spec, args, reason, caller).await
                }
                Decision::Allow => self.run(tool, args, &spec.resources).await,
            },
        };
        self.took_effect(&spec, &outcome, flags);
        calling.done = true;
        self.emit(Event::ToolFinished {
            turn,
            call,
            tool: spec.name.clone(),
            status: outcome.status,
            message: crate::tools::clip(&outcome.message, 2000),
            ms: millis(calling.started.elapsed()),
        });
        self.for_model(&spec.name, &outcome, caller)
    }

    /// Notes what a call did for the turn, and shows its images.
    fn took_effect(&self, spec: &ToolSpec, outcome: &ToolOutcome, flags: &TurnFlags) {
        // An edit is not repeated by the next model either.
        if spec.lane() != Lane::Observe && outcome.status.ok() {
            flags.acted.store(true, Ordering::SeqCst);
            lock(&flags.done).push(format!(
                "{} {}: {}",
                spec.name,
                outcome.status,
                crate::tools::clip(&outcome.message, 200)
            ));
        }
        // Something now runs that will report back, such as a mission: the rest is the answer.
        if outcome.status == Status::Accepted {
            flags.started.store(true, Ordering::SeqCst);
        }
        for image in &outcome.images {
            self.emit(image.into());
        }
    }

    /// What the model reads of an outcome: what the operator said meanwhile, and whether the turn
    /// is stuck, go in fields of their own, past the cut of the message, so they always reach it.
    fn for_model(&self, tool: &str, outcome: &ToolOutcome, caller: Caller) -> Value {
        let mut out = outcome.for_model(self.config.result_chars);
        if caller == Caller::Model {
            let said = std::mem::take(&mut *lock(&self.steer));
            if !said.is_empty() {
                out["operator"] = json!(format!(
                    "said while you worked: \"{}\"; take it into account now",
                    said.join(" ")
                ));
            }
        }
        if let Some(n) = self.stuck(tool, outcome) {
            out["stuck"] = json!(format!(
                "came back the same {n} times in a row: do not try it again; tell the operator \
                 what is stuck"
            ));
        }
        out
    }

    /// How often this turn's failures have come back the same in a row, once that is stuck.
    fn stuck(&self, tool: &str, outcome: &ToolOutcome) -> Option<u32> {
        let mut last = lock(&self.repeated);
        if outcome.status.ok() {
            *last = (String::new(), 0);
            return None;
        }
        let said = format!("{tool}: {}", outcome.message);
        if last.0 == said {
            last.1 += 1;
        } else {
            *last = (said, 1);
        }
        (last.1 >= STUCK_AFTER).then_some(last.1)
    }
}
