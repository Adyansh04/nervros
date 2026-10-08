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

/// Each model call's cost, to the log and to `used`, the context gauge's count; `answered` notes
/// that a call came back.
fn costs(
    shared: &Arc<Shared>,
    turn: u64,
    model: &str,
    used: &Arc<AtomicU64>,
    answered: &Arc<AtomicBool>,
) -> llm::OnCall {
    let (shared, model) = (Arc::clone(shared), model.to_owned());
    let (used, answered) = (Arc::clone(used), Arc::clone(answered));
    Arc::new(move |cost: llm::CallCost| {
        answered.store(true, Ordering::Relaxed);
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

/// Passes a streamed reply's pieces to the UI; `answered` notes that the reply began.
fn deltas(shared: &Arc<Shared>, turn: u64, answered: &Arc<AtomicBool>) -> llm::OnDelta {
    let (shared, answered) = (Arc::clone(shared), Arc::clone(answered));
    Arc::new(move |text: &str| {
        answered.store(true, Ordering::Relaxed);
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
/// the operator's time on approvals and a wait for a model's limit are added back, and it never
/// runs out while an approval is open.
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

/// Logs the turn's start and what started it, and gives the text the model gets.
fn announce(shared: &Shared, turn: u64, origin: Origin) -> String {
    shared.emit(Event::TurnStarted { turn });
    match origin {
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
    }
}

/// One model's go at the turn, asked once more when busy: its result, the history it leaves and
/// the tokens it used.
async fn ask(
    model: &str,
    (shared, source, flags): (&Arc<Shared>, &Arc<dyn AgentSource>, &Arc<TurnFlags>),
    (turn, text, tools): (u64, &str, &[LoopTool]),
    history: &History,
) -> (Result<String, llm::LlmError>, History, u64) {
    let system = preamble(shared);
    // The reasoning in the history is the last turn's model's, and only it takes it back.
    let theirs = lock(&shared.answered_by).as_deref() != Some(model);
    let mut retried = false;
    loop {
        let mut updated = history.clone();
        if theirs {
            updated.drop_reasoning();
        }
        let used = Arc::new(AtomicU64::new(0));
        let answered = Arc::new(AtomicBool::new(false));
        let setup = llm::TurnSetup {
            preamble: &system,
            max_turns: shared.config.max_model_calls,
            tools,
            started: Arc::clone(&flags.started),
            window: source.context(model),
            on_call: Some(costs(shared, turn, model, &used, &answered)),
            delta: Some(deltas(shared, turn, &answered)),
        };
        let result = llm::chat(model, Arc::clone(source), setup, &mut updated, text).await;
        // A busy provider often answers a moment later: once more, while nothing came back or was
        // done that a second try would repeat.
        let busy = matches!(&result, Err(e) if e.setback() == Some(llm::Setback::Busy));
        if busy
            && !retried
            && !answered.load(Ordering::Relaxed)
            && !flags.acted.load(Ordering::SeqCst)
        {
            retried = true;
            tokio::time::sleep(llm::BUSY_RETRY).await;
            continue;
        }
        return (result, updated, used.load(Ordering::Relaxed));
    }
}

/// Waits for the first model that its per-minute limit or a 429 holds back, when that lifts within
/// [`llm::MINUTE_WAIT`], and gives the models to try again.
async fn wait_for_a_model(
    shared: &Shared,
    source: &Arc<dyn AgentSource>,
    need: Need,
    tools: &[LoopTool],
    history: &mut History,
) -> Option<Vec<String>> {
    let wait = source
        .ready_in(Role::Routine, need)
        .filter(|w| *w <= llm::MINUTE_WAIT)?;
    shared.emit(Event::Notice {
        text: format!(
            "every model is at its limit for now; waiting {} s for the first to free up",
            wait.as_secs().max(1)
        ),
    });
    // Not the turn's own time, as the operator's on approvals is not.
    shared.waited_ms.fetch_add(
        u64::try_from(wait.as_millis()).unwrap_or(u64::MAX),
        Ordering::Relaxed,
    );
    tokio::time::sleep(wait).await;
    let candidates = source.candidates(Role::Routine, need);
    fit_window(shared, source, &candidates, tools, history).await;
    Some(candidates)
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
    let text = announce(&shared, turn, origin);
    let flags = Arc::new(TurnFlags::default());
    for tool in registry.iter() {
        tool.ready().await;
    }
    let tools = loop_tools(&registry, &shared, turn, &flags);
    let need = Need {
        tools: !tools.is_empty(),
        ..Need::default()
    };
    let mut candidates = source.candidates(Role::Routine, need).into_iter();
    let mut history = history;
    fit_window(
        &shared,
        &source,
        candidates.as_slice(),
        &tools,
        &mut history,
    )
    .await;
    let mut failures = Vec::new();
    let mut waited = false;
    loop {
        let Some(model) = candidates.next() else {
            // Free tiers count requests a minute, and a burst of calls can spend every model's:
            // once a turn, waiting for the first to free up beats failing it.
            if waited {
                break;
            }
            waited = true;
            let Some(again) = wait_for_a_model(&shared, &source, need, &tools, &mut history).await
            else {
                break;
            };
            candidates = again.into_iter();
            failures.clear();
            continue;
        };
        let window = source.context(&model);
        let (result, mut updated, used) = ask(
            &model,
            (&shared, &source, &flags),
            (turn, &text, &tools),
            &history,
        )
        .await;
        match result {
            Ok(reply) => {
                *lock(&shared.answered_by) = Some(model.clone());
                shared.emit(Event::Reply {
                    turn,
                    text: reply,
                    model,
                });
                if let Some(window) = window {
                    report_context(&shared, &tools, &updated, used, window);
                }
                updated.trim(shared.config.history_max);
                return Some(updated);
            }
            Err(e) => {
                if let Some(setback) = e.setback() {
                    source.set_back(&model, setback);
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
