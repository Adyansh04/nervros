//! The loop that owns the conversation and serves every command at once.

use std::sync::Arc;
use std::sync::atomic::Ordering;

use serde_json::json;
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tracing::Instrument as _;

use super::approval::Answer;
use super::calls::Caller;
use super::config::PULSE_PERIOD;
use super::event::{Command, Event};
use super::turn::{Origin, TurnFlags, limited, run_turn, turn_span};
use super::window::{COMPACT_ROOM, Condense, compact, room_for};
use super::{Shared, save};
use crate::llm::{AgentSource, History};
use crate::lock;
use crate::tools::{Registry, Status};

#[expect(
    clippy::too_many_lines,
    reason = "one arm per command, each a few lines"
)]
pub(super) async fn actor(
    mut rx: mpsc::UnboundedReceiver<Command>,
    shared: Arc<Shared>,
    source: Arc<dyn AgentSource>,
    registry: Arc<Registry>,
) {
    let mut history = shared.config.resume.clone().unwrap_or_default();
    let mut turns = 0u64;
    let mut pulse = tokio::time::interval(PULSE_PERIOD);
    pulse.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut running: Option<(u64, JoinHandle<Option<History>>)> = None;
    // Calls the operator made from the window, which a stop of the robot ends too.
    let mut operator_runs: Vec<JoinHandle<()>> = Vec::new();
    let mut reports: Vec<String> = Vec::new();
    // Nobody asked for a report's reply, so a message sent during one waits for it, not refused.
    let (mut answering_report, mut queued) = (false, None::<String>);
    let start = |turn: u64, origin: Origin, history: &History| {
        let span = turn_span(turn, &origin);
        let task = run_turn(
            turn,
            origin,
            history.clone(),
            Arc::clone(&shared),
            Arc::clone(&source),
            Arc::clone(&registry),
        );
        let task = limited(turn, Arc::clone(&shared), task).instrument(span);
        (turn, tokio::spawn(task))
    };
    loop {
        let finished = async {
            match running.as_mut() {
                Some((_, handle)) => handle.await,
                None => std::future::pending().await,
            }
        };
        tokio::select! {
            biased;
            _ = pulse.tick(), if shared.config.pulse.is_some() => {
                if let Some(p) = &shared.config.pulse {
                    p.pulse();
                }
            }
            cmd = rx.recv() => {
                let Some(cmd) = cmd else { break };
                match cmd {
                    Command::User(text) => {
                        // The operator speaking is a new say on what they want.
                        shared.denials.store(0, Ordering::SeqCst);
                        if let Some((turn, _)) = &running {
                            if answering_report {
                                if queued.is_none() {
                                    queued = Some(text);
                                    shared.emit(Event::Notice { text: "queued until the reply to the report ends".into() });
                                } else {
                                    shared.emit(Event::Notice { text: "one message is queued already; wait or stop it".into() });
                                }
                            } else {
                                // Steering: the model reads it with its next tool result.
                                lock(&shared.steer).push(text.clone());
                                shared.emit(Event::Steer { turn: *turn, text });
                            }
                            continue;
                        }
                        turns += 1;
                        running = Some(start(turns, Origin::User(text), &history));
                        answering_report = false;
                    }
                    Command::Report(text) => {
                        reports.push(text);
                        if running.is_none() {
                            turns += 1;
                            running = Some(start(turns, Origin::Report(reports.split_off(0).join("\n\n")), &history));
                            answering_report = true;
                        }
                    }
                    Command::StopGeneration | Command::StopMission => {
                        let mission = cmd == Command::StopMission;
                        if queued.take().is_some() {
                            shared.emit(Event::Notice { text: "the queued message was dropped".into() });
                        }
                        if let Some((turn, handle)) = running.take() {
                            handle.abort();
                            shared.emit(Event::TurnFinished { turn });
                        }
                        if mission {
                            operator_runs.drain(..).for_each(|h| h.abort());
                        }
                        halt(&shared, mission);
                        // Esc stops the reply to say something else: what was said meanwhile goes
                        // now. A stop of the robot drops it.
                        let said = std::mem::take(&mut *lock(&shared.steer));
                        if !said.is_empty() && !mission {
                            turns += 1;
                            running = Some(start(turns, Origin::User(said.join(" ")), &history));
                            answering_report = false;
                        }
                    }
                    Command::Compact | Command::CompactUpTo { .. } => {
                        if running.is_some() {
                            shared.emit(Event::Notice { text: "still working on the last message; compact after it".into() });
                            continue;
                        }
                        let how = match cmd {
                            Command::CompactUpTo { keep } => Condense::UpTo(keep),
                            _ => Condense::Now,
                        };
                        turns += 1;
                        running = Some((turns, compaction(turns, &shared, &source, &registry, history.clone(), how)));
                        answering_report = false;
                    }
                    Command::Restore(earlier) => {
                        if running.is_some() {
                            shared.emit(Event::Notice { text: "still working on the last message; resume after it".into() });
                            continue;
                        }
                        shared.emit(Event::Restored { exchanges: earlier.exchanges() });
                        history = earlier;
                    }
                    Command::Approve(id) => shared.answer(id, Answer::Approve),
                    Command::AllowForSession(id) => shared.answer(id, Answer::AllowForSession),
                    Command::Deny(id) => shared.answer(id, Answer::Deny),
                    Command::Edit { id, args } => shared.answer(id, Answer::Edit(args)),
                    Command::Run { tool, args } => match registry.get(&tool).cloned() {
                        Some(tool) => {
                            let shared = Arc::clone(&shared);
                            operator_runs.retain(|h| !h.is_finished());
                            operator_runs.push(tokio::spawn(async move {
                                let flags = TurnFlags::default();
                                shared.invoke(&tool, 0, args, &flags, Caller::Operator).await;
                            }));
                        }
                        None => shared.emit(Event::Notice { text: format!("there is no tool {tool}") }),
                    },
                    Command::Arm | Command::Disarm => {
                        let armed = cmd == Command::Arm;
                        shared.guard.set_armed(armed);
                        if !armed {
                            shared.deny_acts();
                        }
                        shared.emit(Event::Armed { armed });
                    }
                }
            }
            done = finished => {
                if let Some((turn, _)) = running.take() {
                    if let Ok(Some(updated)) = done {
                        history = updated;
                        save(&shared, &history);
                    }
                    shared.emit(Event::TurnFinished { turn });
                }
                // What the operator said that no tool result carried goes in a turn of its own.
                let unread = std::mem::take(&mut *lock(&shared.steer));
                if let Some(text) = queued.take().or_else(|| (!unread.is_empty()).then(|| unread.join(" "))) {
                    turns += 1;
                    running = Some(start(turns, Origin::User(text), &history));
                    answering_report = false;
                } else if !reports.is_empty() {
                    turns += 1;
                    running = Some(start(turns, Origin::Report(reports.split_off(0).join("\n\n")), &history));
                    answering_report = true;
                }
            }
        }
    }
}

/// Condenses the conversation as a turn of its own, so a stop is never held up by a summary.
fn compaction(
    turn: u64,
    shared: &Arc<Shared>,
    source: &Arc<dyn AgentSource>,
    registry: &Arc<Registry>,
    mut history: History,
    how: Condense,
) -> JoinHandle<Option<History>> {
    let (shared, source, registry) = (Arc::clone(shared), Arc::clone(source), Arc::clone(registry));
    tokio::spawn(async move {
        shared.emit(Event::TurnStarted { turn });
        let room = room_for(&shared, &source, &registry).unwrap_or(COMPACT_ROOM);
        compact(&shared, &source, &mut history, room, how).await;
        Some(history)
    })
}

/// Denies what waits for approval and, for a stop of the robot, calls its stop too.
fn halt(shared: &Arc<Shared>, robot: bool) {
    let pending: Vec<u64> = lock(&shared.approvals).keys().copied().collect();
    for id in pending {
        shared.answer(id, Answer::Deny);
    }
    // Turned down, so not left for the next session to ask again.
    let waited = !std::mem::take(&mut *lock(&shared.waiting)).is_empty();
    if waited {
        shared.write_waiting();
    }
    let stop = shared.stop.as_ref().filter(|_| robot);
    if robot {
        shared.guard.note_stop();
    }
    if let Some(stop) = stop.cloned() {
        let shared = Arc::clone(shared);
        tokio::spawn(async move {
            let out = stop.call(json!({"reason": "operator"})).await;
            let ok = out.status == Status::Succeeded;
            shared.emit(Event::Stopped {
                ok,
                detail: if ok {
                    out.data.to_string()
                } else {
                    out.message
                },
            });
        });
    }
    let reason = if stop.is_some() {
        "stopped by the operator: reply and robot"
    } else {
        "reply stopped by the operator"
    };
    shared.emit(Event::Halted {
        reason: reason.into(),
    });
}
