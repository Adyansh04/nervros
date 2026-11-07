//! A turn's model loop: the tools it offers, the hook that watches each call, and streaming the
//! reply.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime};

use futures::future::BoxFuture;
use rig::AgentBuilder;
use rig::agent::{
    AgentHook, CompletionCallAction, CompletionCallEvent, HookContext, InvalidToolCallAction,
    InvalidToolCallContext, OutcomeAction, OutcomeEvent, RequestPatch,
};
use rig::completion::PromptError;
use rig::message::ToolName;
use rig::tool::{DynamicTool, ToolOutput};

use super::client::{Llm, overflowed, prompt_setback};
use super::history::{History, RESERVE_TOKENS, cut_old, fit, size, tokens_of};
use super::{LlmError, Setback};
use crate::providers::Role;
use crate::providers::budget::Tokens;
use crate::providers::router::{Need, Skip};

/// Model-facing JSON a tool returns.
pub type ToolFuture = BoxFuture<'static, serde_json::Value>;

/// A tool as the agent loop sees it: a spec plus an async call that returns the JSON the model
/// reads. The session wraps each registry tool this way, guard and events included.
#[derive(Clone)]
pub struct LoopTool {
    /// The name the model uses.
    pub name: String,
    /// What it does.
    pub description: String,
    /// JSON Schema of the arguments.
    pub parameters: serde_json::Value,
    /// Runs the tool.
    pub invoke: std::sync::Arc<dyn Fn(serde_json::Value) -> ToolFuture + Send + Sync>,
}

impl std::fmt::Debug for LoopTool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LoopTool")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

/// What the session asks of its model layer; [`Llm`] implements it, tests use a scripted model.
pub trait AgentSource: Send + Sync {
    /// Model ids to try for a role, in order.
    fn candidates(&self, role: Role, need: Need) -> Vec<String>;

    /// Why each model was passed over for a role, as "model: reason": what to tell the operator
    /// when none is left.
    fn passed_over(&self, _role: Role, _need: Need) -> Vec<String> {
        Vec::new()
    }

    /// A rig builder for one model.
    ///
    /// # Errors
    ///
    /// The model cannot be built.
    fn builder(&self, model_id: &str) -> Result<AgentBuilder, LlmError>;

    /// Counts one request against the model's quota.
    ///
    /// # Errors
    ///
    /// Why the quota refuses the request, which is then not counted.
    fn take_request(&self, model_id: &str) -> Result<(), String>;

    /// Sets a model aside for `for_how_long` after a 429.
    fn park(&self, model_id: &str, for_how_long: Duration);

    /// Counts what one call cost against its provider's budget, if it has one.
    fn spent(&self, _model_id: &str, _cost: CallCost) {}

    /// Tries a model whose server did not answer last, for `for_how_long`.
    fn set_aside(&self, model_id: &str, for_how_long: Duration) {
        self.park(model_id, for_how_long);
    }

    /// Sets a model back after a failure: a 429 parks it for every process that shares the quota
    /// file, and a server that did not answer is tried last for a while, by this one alone.
    fn set_back(&self, model_id: &str, setback: Setback) {
        match setback {
            Setback::Limited(wait) => self.park(model_id, wait),
            Setback::Unreachable => self.set_aside(model_id, super::UNREACHABLE_PARK),
            Setback::Busy => {}
        }
    }

    /// How long until the first model for a call that its per-minute limit or a 429 holds back
    /// may take a request again; none when nothing holds a model back that briefly.
    fn ready_in(&self, _role: Role, _need: Need) -> Option<Duration> {
        None
    }

    /// The model's context window in tokens, when the models file gives it.
    fn context(&self, _model_id: &str) -> Option<usize> {
        None
    }

    /// Whether the model's replies stream.
    fn streams(&self, _model_id: &str) -> bool {
        false
    }
}

impl AgentSource for Llm {
    fn candidates(&self, role: Role, need: Need) -> Vec<String> {
        self.router
            .candidates(role, need, SystemTime::now())
            .0
            .into_iter()
            .map(|m| m.id.clone())
            .collect()
    }

    fn passed_over(&self, role: Role, need: Need) -> Vec<String> {
        self.router
            .candidates(role, need, SystemTime::now())
            .1
            .into_iter()
            .map(|(id, skip)| {
                let why = match skip {
                    Skip::NoVision => "it takes no images".to_owned(),
                    Skip::NoTools => "it calls no tools".to_owned(),
                    Skip::Privacy => "the privacy mode keeps it out".to_owned(),
                    Skip::Refused(refused) => refused.to_string(),
                };
                format!("{id}: {why}")
            })
            .collect()
    }

    fn builder(&self, model_id: &str) -> Result<AgentBuilder, LlmError> {
        let model = self
            .router
            .config()
            .model(model_id)
            .ok_or_else(|| LlmError::Client {
                provider: String::new(),
                message: format!("unknown model `{model_id}`"),
            })?;
        self.agent_builder(model)
    }

    fn take_request(&self, model_id: &str) -> Result<(), String> {
        self.router
            .take_request(model_id, SystemTime::now())
            .map_err(|refused| refused.to_string())
    }

    fn park(&self, model_id: &str, for_how_long: Duration) {
        self.router.park(model_id, SystemTime::now(), for_how_long);
    }

    fn spent(&self, model_id: &str, cost: CallCost) {
        self.router.record_spend(
            model_id,
            Tokens {
                input: cost.input_tokens,
                cached: cost.cached_tokens,
                cache_write: cost.cache_write_tokens,
                output: cost.output_tokens,
            },
        );
    }

    fn set_aside(&self, model_id: &str, for_how_long: Duration) {
        self.router
            .set_aside(model_id, SystemTime::now(), for_how_long);
    }

    fn ready_in(&self, role: Role, need: Need) -> Option<Duration> {
        self.router.ready_in(role, need, SystemTime::now())
    }

    fn context(&self, model_id: &str) -> Option<usize> {
        self.router.config().model(model_id).and_then(|m| m.context)
    }

    fn streams(&self, model_id: &str) -> bool {
        self.router
            .config()
            .model(model_id)
            .is_some_and(|m| m.stream)
    }
}

/// Steers one turn's agent loop.
pub(super) struct TurnHook {
    source: Arc<dyn AgentSource>,
    model: String,
    started: Arc<AtomicBool>,
    /// The turn's last model call, which must answer: a tool called then ends the turn without one.
    last: usize,
    /// Tokens left for the history and the prompt in the model's window, when it is known.
    room: Option<usize>,
    on_call: Option<OnCall>,
    /// When the pending call was sent, and its prompt's tokens as counted here.
    sent: Mutex<Option<(Instant, u64)>>,
    /// The preamble's and the tools' tokens, sent with every call.
    fixed_tokens: usize,
    /// The system prompt for the last call, which says no tool can be called.
    last_word: String,
}

impl TurnHook {
    /// The hook for a turn of `model` from `source`, as `setup` asks.
    pub(super) fn new(source: Arc<dyn AgentSource>, model: &str, setup: &TurnSetup<'_>) -> Self {
        let fixed_tokens = fixed_cost(setup.preamble, setup.tools);
        Self {
            source,
            model: model.to_owned(),
            started: Arc::clone(&setup.started),
            last: setup.max_turns,
            room: setup
                .window
                .map(|w| w.saturating_sub(fixed_tokens + RESERVE_TOKENS)),
            on_call: setup.on_call.clone(),
            sent: Mutex::new(None),
            fixed_tokens,
            last_word: format!("{}\n\n{LAST_CALL}", setup.preamble),
        }
    }
}

impl TurnHook {
    fn charge(&self, cost: CallCost) {
        self.source.spent(&self.model, cost);
        if let Some(on_call) = &self.on_call {
            on_call(cost);
        }
    }
}

impl Drop for TurnHook {
    /// A call cut off in flight, by a stop or the turn's end, is billed for its prompt at least;
    /// its reply's tokens are not known.
    fn drop(&mut self) {
        let in_flight = self.sent.get_mut().ok().and_then(Option::take);
        if let Some((at, prompt)) = in_flight {
            self.charge(CallCost {
                input_tokens: prompt,
                ms: millis(at),
                ..CallCost::default()
            });
        }
    }
}

fn millis(since: Instant) -> u64 {
    u64::try_from(since.elapsed().as_millis()).unwrap_or(u64::MAX)
}

impl AgentHook for TurnHook {
    /// Takes each request from the model's quota before it is sent (a turn is up to `max_turns`
    /// requests, not one). Once a mission has started, and on the turn's last call, it offers no
    /// more tools, so the model answers instead of spending the turn on checks the mission's report
    /// will answer anyway, or being cut off mid-plan.
    async fn on_completion_call(
        &self,
        ctx: &HookContext,
        event: CompletionCallEvent<'_>,
    ) -> CompletionCallAction {
        if let Err(why) = self.source.take_request(&self.model) {
            return CompletionCallAction::stop(format!("{}: {why}", self.model));
        }
        let mut patch = None;
        if let Some(room) = self.room {
            // Within a turn the results pile up: what the model sees is cut to fit, while the
            // session keeps every message for its own compaction between turns.
            let room = room.saturating_sub(size(std::slice::from_ref(event.prompt)));
            if size(event.history) > room {
                patch = Some(RequestPatch::new().history(fit(cut_old(event.history, 2), room)));
            }
        }
        if self.started.load(Ordering::SeqCst) {
            patch = Some(patch.unwrap_or_default().active_tools(Vec::<String>::new()));
        } else if ctx.turn() >= self.last {
            // Told nothing, a small model writes its next tool call as text.
            patch = Some(
                patch
                    .unwrap_or_default()
                    .active_tools(Vec::<String>::new())
                    .preamble(self.last_word.clone()),
            );
        }
        if let Ok(mut sent) = self.sent.lock() {
            let prompt =
                self.fixed_tokens + size(event.history) + size(std::slice::from_ref(event.prompt));
            *sent = Some((Instant::now(), u64::try_from(prompt).unwrap_or(u64::MAX)));
        }
        patch.map_or_else(
            CompletionCallAction::continue_run,
            CompletionCallAction::patch,
        )
    }

    /// Charges every call that brought a whole reply, the ones the run retried or recovered from
    /// an invalid tool call too: the provider bills them all. Each call is reported with its time.
    /// A call that failed is not billed; one cut off in flight is charged when the hook drops.
    async fn on_outcome(&self, _ctx: &HookContext, event: OutcomeEvent<'_>) -> OutcomeAction {
        if !matches!(event.kind, rig::effect::EffectKind::Completion { .. }) {
            return OutcomeAction::proceed();
        }
        let sent = self.sent.lock().ok().and_then(|mut s| s.take());
        if let Some(response) = event.completion() {
            let mut cost = CallCost {
                ms: sent.map_or(0, |(t, _)| millis(t)),
                ..CallCost::of(&response.usage)
            };
            // A reply that carries no count is priced on its prompt as counted here.
            if cost.input_tokens == 0 {
                cost.input_tokens = sent.map_or(0, |(_, prompt)| prompt);
            }
            self.charge(cost);
        }
        OutcomeAction::proceed()
    }

    /// Small models invent tool names; the model gets the real ones back as the call's result
    /// and can try again, where rig would otherwise end the turn.
    async fn on_invalid_tool_call(
        &self,
        _ctx: &HookContext,
        event: &InvalidToolCallContext,
    ) -> Option<InvalidToolCallAction> {
        let tools = if event.allowed_tools.is_empty() {
            "none: answer the operator instead".to_owned()
        } else {
            event.allowed_tools.join(", ")
        };
        Some(InvalidToolCallAction::Skip {
            reason: format!(
                "There is no tool called `{}`. The tools you can call now: {tools}.",
                event.tool_name
            ),
        })
    }
}

/// What one turn offers the model.
pub struct TurnSetup<'a> {
    /// The system prompt.
    pub preamble: &'a str,
    /// Model calls allowed in the turn, tool rounds included.
    pub max_turns: usize,
    /// The tools on offer.
    pub tools: &'a [LoopTool],
    /// Set once a tool has started something that reports back, such as a mission; the rest of
    /// the turn is the answer.
    pub started: Arc<AtomicBool>,
    /// The model's context window in tokens, when known.
    pub window: Option<usize>,
    /// Given what each model call cost.
    pub on_call: Option<OnCall>,
    /// Given each piece of reply text as it arrives, when the model streams.
    pub delta: Option<OnDelta>,
}

/// What is given each piece of a streamed reply.
pub type OnDelta = Arc<dyn Fn(&str) + Send + Sync>;

/// What one model call cost, as the provider reported it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CallCost {
    /// Prompt tokens.
    pub input_tokens: u64,
    /// Of those, read from the provider's prompt cache.
    pub cached_tokens: u64,
    /// Of those, written to the provider's prompt cache.
    pub cache_write_tokens: u64,
    /// Reply tokens.
    pub output_tokens: u64,
    /// From sending the request to the whole reply.
    pub ms: u64,
}

impl CallCost {
    /// A call's cost from rig's count, its time unknown; a count left out is zero tokens.
    #[must_use]
    pub fn of(usage: &rig::completion::Usage) -> Self {
        Self {
            input_tokens: usage.input_tokens.unwrap_or(0),
            cached_tokens: usage.cached_input_tokens.unwrap_or(0),
            cache_write_tokens: usage.cache_creation_input_tokens.unwrap_or(0),
            output_tokens: usage.output_tokens.unwrap_or(0),
            ms: 0,
        }
    }
}

/// What is given each model call's cost.
pub type OnCall = Arc<dyn Fn(CallCost) + Send + Sync>;

/// Added to the system prompt for a turn's last model call.
const LAST_CALL: &str = "You cannot call a tool now: this request is out of steps. Answer the \
    operator in plain words with what you found, and what is left to do.";

/// A reply without the tool calls a small model sometimes writes as text: what is before them,
/// or a plain word that the request ran out of steps.
pub(super) fn plain(reply: String) -> String {
    let cut = ["<tool_call>", "<function=", "<|tool_call"]
        .iter()
        .filter_map(|m| reply.find(m))
        .min();
    match cut {
        None => reply,
        Some(at) => {
            let before = reply[..at].trim();
            if before.is_empty() {
                "I ran out of steps for this request before finishing; ask me to carry on."
                    .to_owned()
            } else {
                before.to_owned()
            }
        }
    }
}

/// What the system prompt and the tools' schemas take of every request.
#[must_use]
pub fn fixed_cost(preamble: &str, tools: &[LoopTool]) -> usize {
    fixed_tokens(
        preamble,
        tools
            .iter()
            .map(|t| definition_chars(&t.name, &t.description, &t.parameters)),
    )
}

/// What the system prompt and tool definitions of the given sizes take of every request.
#[must_use]
pub fn fixed_tokens(preamble: &str, definitions: impl Iterator<Item = usize>) -> usize {
    tokens_of(preamble.len() + definitions.sum::<usize>())
}

/// The characters one tool's definition takes in a request.
#[must_use]
pub fn definition_chars(name: &str, description: &str, parameters: &serde_json::Value) -> usize {
    name.len() + description.len() + parameters.to_string().len()
}

/// Runs one user turn on a model from `source`: the model may call the tools up to
/// `setup.max_turns` model calls in total, each taken from its quota. Committed messages, tool
/// calls and results included, are appended to `history`.
///
/// # Errors
///
/// The model cannot be built, or [`LlmError::Turn`] when the provider or the loop fails or the
/// quota ends the turn.
pub async fn chat(
    model_id: &str,
    source: Arc<dyn AgentSource>,
    setup: TurnSetup<'_>,
    history: &mut History,
    text: &str,
) -> Result<String, LlmError> {
    // A tool's name is never empty: the registry names every tool it holds.
    let dynamic: Vec<DynamicTool> = setup
        .tools
        .iter()
        .filter_map(|t| {
            let invoke = std::sync::Arc::clone(&t.invoke);
            Some(DynamicTool::new(
                ToolName::new(t.name.clone()).ok()?,
                t.description.clone(),
                t.parameters.clone(),
                move |args| {
                    let fut = invoke(args);
                    Box::pin(async move { Ok(ToolOutput::json(fut.await)) })
                },
            ))
        })
        .collect();
    let delta = setup.delta.clone().filter(|_| source.streams(model_id));
    let builder = source
        .builder(model_id)?
        .preamble(setup.preamble)
        .default_max_turns(setup.max_turns)
        .dynamic_tools(dynamic)
        .add_hook(TurnHook::new(source, model_id, &setup));
    let agent = builder.build();
    let failed = |e: &PromptError| LlmError::Turn {
        model: model_id.to_owned(),
        message: without_provider_body(&failure(e, model_id)),
        setback: prompt_setback(e),
        overflowed: overflowed(e),
    };
    if let Some(delta) = delta {
        return match streamed(&agent, text, history, delta.as_ref()).await {
            Ok(Some(reply)) => Ok(plain(reply)),
            Ok(None) => Err(LlmError::Turn {
                model: model_id.to_owned(),
                message: "the reply stream ended without a reply".to_owned(),
                setback: None,
                overflowed: false,
            }),
            Err(e) => Err(failed(&e)),
        };
    }
    agent
        .chat(text, &mut history.0)
        .await
        .map(|response| plain(response.output()))
        .map_err(|e| failed(&e))
}

/// One turn with the reply streamed: each text delta to `delta`, and the run's transcript into
/// `history` at the end, as `chat` appends it. `None` when the stream ended without a reply.
pub(super) async fn streamed(
    agent: &rig::Agent,
    text: &str,
    history: &mut History,
    delta: &(dyn Fn(&str) + Send + Sync),
) -> Result<Option<String>, PromptError> {
    use futures::StreamExt as _;
    use rig::agent::MultiTurnStreamItem;
    use rig::streaming::{Item, StreamEvent};
    let mut stream = agent.prompt(text).history(history.0.clone()).stream();
    let mut done = None;
    while let Some(item) = stream.next().await {
        match item? {
            MultiTurnStreamItem::StreamAssistantItem(Item::Event(StreamEvent::Text {
                text,
                ..
            })) => delta(&text),
            MultiTurnStreamItem::FinalResponse(response) => done = Some(response),
            _ => {}
        }
    }
    Ok(done.map(|response| {
        // The run's transcript, without the history it was given.
        history.0.extend(response.messages.iter().cloned());
        response.output()
    }))
}

/// What a run's failure says: for a turn our own hook cancelled (`<model>: <why>`), the reason
/// alone, since the error names the model already.
pub(super) fn failure(e: &PromptError, model: &str) -> String {
    match e {
        PromptError::Cancelled { reason, .. } => reason
            .strip_prefix(model)
            .and_then(|rest| rest.strip_prefix(": "))
            .unwrap_or(reason)
            .to_owned(),
        other => other.to_string(),
    }
}

/// A provider error with its JSON body replaced by the provider's own message: the body can
/// carry account ids (OpenRouter's `user_id`) that belong in neither the chat nor the log.
pub(super) fn without_provider_body(text: &str) -> String {
    let Some(start) = text.find('{') else {
        // Not JSON, as an HTML page or plain text: its first line only, short, the rest held back.
        let first: String = text
            .lines()
            .next()
            .unwrap_or_default()
            .chars()
            .take(200)
            .collect();
        return if first.len() < text.trim_end().len() {
            format!("{first} (the rest withheld)")
        } else {
            first
        };
    };
    let (head, body) = text.split_at(start);
    let parsed = serde_json::Deserializer::from_str(body)
        .into_iter::<serde_json::Value>()
        .next()
        .and_then(Result::ok);
    let error = parsed.as_ref().map(|v| &v["error"]);
    // OpenRouter's `metadata.raw` is the upstream provider's words, more specific than `message`.
    let said = error
        .and_then(|e| {
            e["metadata"]["raw"]
                .as_str()
                .or_else(|| e["message"].as_str())
        })
        .unwrap_or("details withheld");
    format!("{head}{said}")
}
