//! Approvals: asking the operator, their answers, and the requests a session leaves waiting.

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Instant;

use serde::{Deserialize, Serialize};
use serde_json::Value;
use tokio::sync::oneshot;

use super::calls::Approved;
use super::event::Event;
use super::{Shared, millis};
use crate::lock;
use crate::tools::{Lane, Risk, Tool};

/// A request the operator had not answered when its session ended.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Unanswered {
    /// The tool.
    pub tool: String,
    /// Its arguments, as they would be sent anew.
    pub args: Value,
    /// Why it asked.
    pub reason: String,
}

/// The requests an earlier session left unanswered in `file`; the file goes, so each is offered
/// once.
#[must_use]
pub fn take_unanswered(file: &std::path::Path) -> Vec<Unanswered> {
    // Read before it goes: an unreadable file is moved aside, not lost.
    let left: Vec<Unanswered> = crate::persist::read_or_default(file);
    if let Err(e) = std::fs::remove_file(file)
        && e.kind() != std::io::ErrorKind::NotFound
    {
        tracing::warn!(error = %e, "the unanswered requests stay");
    }
    left
}

/// The operator's answer to an approval.
#[derive(Debug)]
pub(super) enum Answer {
    Approve,
    /// Approve, and stop asking about this tool for the session.
    AllowForSession,
    Deny,
    /// Check these arguments instead, and wait again.
    Edit(Value),
}

/// Denials in a row after which the model is told to stop asking and ask the operator instead.
pub(super) const DENIAL_BREAK: u32 = 3;

/// An approval waiting on the operator.
pub(super) struct Asked {
    answer: oneshot::Sender<Answer>,
    /// It would act on the robot, so disarming turns it down.
    acts: bool,
}

/// Closes an approval however its wait ends: answered, or dropped by a stop or the end of the
/// session, so the window never keeps a card nobody can answer.
struct Asking<'a> {
    shared: &'a Shared,
    id: u64,
    approved: bool,
}

impl Drop for Asking<'_> {
    fn drop(&mut self) {
        lock(&self.shared.approvals).remove(&self.id);
        self.shared.emit(Event::ApprovalResolved {
            id: self.id,
            approved: self.approved,
        });
    }
}

impl Shared {
    /// Asks the operator, who may approve, deny or edit; an edit that passes its checks replaces
    /// what is asked and the wait starts again. `None` when denied or expired.
    #[tracing::instrument(
        name = "nervros.approval",
        skip_all,
        fields(nervros.tool = name, nervros.outcome = tracing::field::Empty)
    )]
    pub(super) async fn ask_approval(
        &self,
        tool: &Arc<dyn Tool>,
        name: &str,
        reason: String,
        risk: Risk,
        mut approved: Approved,
    ) -> Option<Approved> {
        let can_allow = !matches!(risk, Risk::Motion | Risk::Manipulation);
        let acts = risk.lane() == Lane::Act;
        let id = self.next_approval.fetch_add(1, Ordering::Relaxed) + 1;
        let mut asking = Asking {
            shared: self,
            id,
            approved: false,
        };
        let mut rx = self.listen(id, acts);
        self.keep_waiting(id, tool, name, &reason, &approved.args);
        self.emit(Event::ApprovalRequested {
            id,
            tool: name.to_owned(),
            args: approved.args.clone(),
            reason: reason.clone(),
            can_allow,
        });
        let asked = Instant::now();
        let mut early = None;
        let yes = loop {
            let answer = match early.take() {
                Some(answer) => Ok(Ok(answer)),
                None => tokio::time::timeout(self.config.approval_ttl, &mut rx).await,
            };
            match answer {
                Ok(Ok(Answer::Approve)) => break true,
                Ok(Ok(Answer::AllowForSession)) => {
                    let text = if can_allow {
                        lock(&self.allowed).insert(name.to_owned());
                        format!(
                            "`{name}` runs without asking for the rest of this session, except \
                             what moves the robot"
                        )
                    } else {
                        format!("`{name}` moves the robot, so it still asks every time")
                    };
                    self.emit(Event::Notice { text });
                    break true;
                }
                Ok(Ok(Answer::Edit(edited))) => {
                    // Waiting again before the check, so a stop during it still lands.
                    rx = self.listen(id, acts);
                    let passed = match tool.assess_operator(edited).await {
                        Some(Ok(a)) => {
                            if let Some(args) = a.args {
                                approved.args = args;
                            }
                            approved.resources = a.resources;
                            self.keep_waiting(id, tool, name, &reason, &approved.args);
                            self.emit(Event::ApprovalEdited {
                                id,
                                args: approved.args.clone(),
                                reason: a.reason,
                            });
                            true
                        }
                        Some(Err(out)) => {
                            self.emit(Event::EditRejected {
                                id,
                                message: out.message,
                            });
                            false
                        }
                        None => {
                            self.emit(Event::EditRejected {
                                id,
                                message: format!("{name} cannot be edited"),
                            });
                            false
                        }
                    };
                    // An approval sent during the check was for the plan before it, so it does
                    // not approve an edit nobody has seen; a denial or a newer edit stands.
                    match rx.try_recv() {
                        Ok(Answer::Approve) if passed => rx = self.listen(id, acts),
                        Ok(answer) => early = Some(answer),
                        Err(_) => {}
                    }
                }
                Ok(Ok(Answer::Deny) | Err(_)) | Err(_) => break false,
            }
        };
        self.waited_ms
            .fetch_add(millis(asked.elapsed()), Ordering::Relaxed);
        self.done_waiting(id);
        tracing::Span::current().record("nervros.outcome", if yes { "approved" } else { "denied" });
        asking.approved = yes;
        drop(asking);
        yes.then_some(approved)
    }

    /// Keeps approval `id` where the next session finds it if this one ends first.
    fn keep_waiting(&self, id: u64, tool: &Arc<dyn Tool>, name: &str, reason: &str, args: &Value) {
        if self.config.pending_file.is_none() {
            return;
        }
        if let Some(args) = tool.ask_again(args) {
            let entry = Unanswered {
                tool: name.to_owned(),
                args,
                reason: reason.to_owned(),
            };
            lock(&self.waiting).insert(id, entry);
            self.write_waiting();
        }
    }

    fn done_waiting(&self, id: u64) {
        if lock(&self.waiting).remove(&id).is_some() {
            self.write_waiting();
        }
    }

    pub(super) fn write_waiting(&self) {
        let Some(file) = &self.config.pending_file else {
            return;
        };
        let waiting = lock(&self.waiting);
        let written = if waiting.is_empty() {
            match std::fs::remove_file(file) {
                Err(e) if e.kind() != std::io::ErrorKind::NotFound => Err(e),
                _ => Ok(()),
            }
        } else {
            serde_json::to_vec(&waiting.values().collect::<Vec<_>>())
                .map_err(std::io::Error::other)
                .and_then(|json| crate::persist::write_atomic(file, &json))
        };
        if let Err(e) = written {
            tracing::warn!(error = %e, "the requests waiting on the operator were not kept");
        }
    }

    /// A fresh channel for the next answer to approval `id`.
    fn listen(&self, id: u64, acts: bool) -> oneshot::Receiver<Answer> {
        let (answer, rx) = oneshot::channel();
        lock(&self.approvals).insert(id, Asked { answer, acts });
        rx
    }

    pub(super) fn answer(&self, id: u64, answer: Answer) {
        if let Some(asked) = lock(&self.approvals).remove(&id) {
            let _ = asked.answer.send(answer);
        }
    }

    /// Turns down every request that would act on the robot, as disarming does.
    pub(super) fn deny_acts(&self) {
        let acting: Vec<u64> = lock(&self.approvals)
            .iter()
            .filter(|(_, a)| a.acts)
            .map(|(id, _)| *id)
            .collect();
        for id in acting {
            self.answer(id, Answer::Deny);
        }
    }
}
