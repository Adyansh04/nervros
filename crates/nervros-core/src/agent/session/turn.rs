//! One turn: its tools, its time limit, and trying the routine role's models in order.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use super::Shared;
use super::calls::Caller;
use super::event::Event;
use super::window::{fit_window, preamble, report_context};
use crate::llm::{AgentSource, History, LoopTool};
use crate::providers::Role;
use crate::providers::router::Need;
use crate::tools::Registry;
use crate::{llm, lock};

/// Each model call's cost, to the log and to `used`, the context gauge's count.
fn costs(shared: &Arc<Shared>, turn: u64, model: &str, used: &Arc<AtomicU64>) -> llm::OnCall {
    let (shared, model, used) = (Arc::clone(shared), model.to_owned(), Arc::clone(used));
    Arc::new(move |cost: llm::CallCost| {
        if cost.input_tokens > 0 {
            used.store(cost.input_tokens, Ordering::Relaxed);
        }
        shared.emit(Event::ModelCall {
            turn,
            model: model.clone(),
            input_tokens: cost.input_tokens,
            cached_tokens: cost.cached_tokens,
            output_tokens: cost.output_tokens,
            ms: cost.ms,
        });
    })
}

/// Passes a streamed reply's pieces to the UI.
fn deltas(shared: &Arc<Shared>, turn: u64) -> llm::OnDelta {
    let shared = Arc::clone(shared);
    Arc::new(move |text: &str| {
        shared.emit(Event::ReplyDelta {
            turn,
            text: text.to_owned(),
        });
    })
}

/// A turn's span, named and shaped as OpenTelemetry's `GenAI` conventions name an agent's run, so
/// rig records the turn's token use on it and nests its model and tool calls inside.
pub(super) fn turn_span(turn: u64, origin: &Origin) -> tracing::Span {
    let empty = tracing::field::Empty;
    tracing::info_span!(
        "invoke_agent",
        otel.name = "invoke_agent nervros",
        gen_ai.operation.name = "invoke_agent",
        gen_ai.agent.name = "nervros",
        nervros.turn = turn,
        nervros.origin = match origin {
            Origin::User(_) => "operator",
            Origin::Report(_) => "report",
        },
        gen_ai.prompt = empty,
        gen_ai.completion = empty,
        gen_ai.usage.input_tokens = empty,
        gen_ai.usage.output_tokens = empty,
        gen_ai.usage.cache_read.input_tokens = empty,
        gen_ai.usage.cache_creation.input_tokens = empty,
        gen_ai.usage.tool_use_prompt_tokens = empty,
        gen_ai.usage.reasoning_tokens = empty,
    )
}

/// Runs a turn under its time limit. A turn over it ends with an error, as a stopped one does;
/// the operator's time on approvals is added back, and never runs out while one is open.
pub(super) async fn limited(
    turn: u64,
    shared: Arc<Shared>,
    task: impl Future<Output = Option<History>>,
) -> Option<History> {
    tokio::pin!(task);
    shared.waited_ms.store(0, Ordering::Relaxed);
    let started = Instant::now();
    let limit = shared.config.turn_time;
    let mut deadline = started + limit;
    loop {
        tokio::select! {
            out = &mut task => return out,
            () = tokio::time::sleep_until(deadline.into()) => {
                if !lock(&shared.approvals).is_empty() {
                    deadline = Instant::now() + Duration::from_secs(1);
                    continue;
                }
                let credited = started + limit + Duration::from_millis(shared.waited_ms.load(Ordering::Relaxed));
                if credited > Instant::now() {
                    deadline = credited;
                    continue;
                }
                shared.emit(Event::Error {
                    turn,
                    text: format!("the turn took longer than {} s and was stopped", limit.as_secs()),
                });
                return None;
            }
        }
    }
}

/// Who started a turn.
pub(super) enum Origin {
    User(String),
    Report(String),
}

/// What the tools of one turn have done so far.
#[derive(Debug, Default)]
pub(super) struct TurnFlags {
    /// An act ran, so the turn is not retried on another model.
    pub(super) acted: AtomicBool,
    /// What each act did, for the history when the reply after it is lost.
    pub(super) done: Mutex<Vec<String>>,
    /// Something started that reports back later, so no more tools are offered.
    pub(super) started: Arc<AtomicBool>,
}

/// This turn's tools, each call going through the guard and recorded in `flags`.
fn loop_tools(
    registry: &Registry,
    shared: &Arc<Shared>,
    turn: u64,
    flags: &Arc<TurnFlags>,
) -> Vec<LoopTool> {
    registry
        .iter()
        .map(|tool| {
            let (tool, shared, flags) = (Arc::clone(tool), Arc::clone(shared), Arc::clone(flags));
            let spec = tool.spec().into_owned();
            LoopTool {
                name: spec.name,
                description: spec.description,
                parameters: spec.parameters,
                invoke: Arc::new(move |args| {
                    let (tool, shared, flags) =
                        (Arc::clone(&tool), Arc::clone(&shared), Arc::clone(&flags));
                    Box::pin(async move {
                        let caller = Caller::Model;
                        shared.invoke(&tool, turn, args, &flags, caller).await
                    })
                }),
            }
        })
        .collect()
}

pub(super) async fn run_turn(
    turn: u64,
    origin: Origin,
    history: History,
    shared: Arc<Shared>,
    source: Arc<dyn AgentSource>,
    registry: Arc<Registry>,
) -> Option<History> {
    shared.guard.begin_turn();
    *lock(&shared.repeated) = (String::new(), 0);
    shared.emit(Event::TurnStarted { turn });
    let text = match origin {
        Origin::User(text) => {
            shared.emit(Event::User {
                turn,
                text: text.clone(),
            });
            text
        }
        Origin::Report(text) => {
            shared.emit(Event::Report {
                turn,
                text: text.clone(),
            });
            format!("{}\n{text}", llm::REPORT_MARK)
        }
    };
    let flags = Arc::new(TurnFlags::default());
    for tool in registry.iter() {
        tool.ready().await;
    }
    let tools = loop_tools(&registry, &shared, turn, &flags);
    let need = Need {
        tools: !tools.is_empty(),
        ..Need::default()
    };
    let candidates = source.candidates(Role::Routine, need);
    let mut history = history;
    fit_window(&shared, &source, &candidates, &tools, &mut history).await;
    let mut failures = Vec::new();
    for model in candidates {
        let mut updated = history.clone();
        let used = Arc::new(AtomicU64::new(0));
        let window = source.context(&model);
        let system = preamble(&shared);
        let setup = llm::TurnSetup {
            preamble: &system,
            max_turns: shared.config.max_model_calls,
            tools: &tools,
            started: Arc::clone(&flags.started),
            window,
            on_call: Some(costs(&shared, turn, &model, &used)),
            delta: Some(deltas(&shared, turn)),
        };
        let result = llm::chat(&model, Arc::clone(&source), setup, &mut updated, &text).await;
        match result {
            Ok(reply) => {
                shared.emit(Event::Reply {
                    turn,
                    text: reply,
                    model,
                });
                if let Some(window) = window {
                    report_context(
                        &shared,
                        &tools,
                        &updated,
                        used.load(Ordering::Relaxed),
                        window,
                    );
                }
                updated.trim(shared.config.history_max);
                return Some(updated);
            }
            Err(e) => {
                if let llm::LlmError::Turn {
                    retry_after: Some(wait),
                    ..
                } = &e
                {
                    source.park(&model, *wait);
                }
                if !flags.acted.load(Ordering::SeqCst) {
                    shared.emit(Event::Notice {
                        text: format!("{e}; trying the next model"),
                    });
                    failures.push(e.to_string());
                    continue;
                }
                // What the robot was asked to do goes on, and its report will follow; only the
                // reply is lost. The history keeps the request and the act, or the next turn
                // would not know of either and might ask for it again.
                shared.emit(Event::Notice {
                    text: format!("{e}; the robot already acted, so the turn is not retried"),
                });
                history.lost_reply(&text, &lock(&flags.done));
                return Some(history);
            }
        }
    }
    let text = if failures.is_empty() {
        "no model is available".to_owned()
    } else {
        failures.join("; ")
    };
    shared.emit(Event::Error { turn, text });
    None
}
