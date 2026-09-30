//! The only module that imports `rig`.
//!
//! It turns a model from `models.toml` into a rig agent builder, and runs one prompt against the
//! router's candidates in order: a 429 parks the model and moves on, any other provider or
//! transport failure moves on without parking, and the answer names the model that gave it.

use std::collections::HashMap;
use std::time::{Duration, SystemTime};

use base64::Engine as _;
use futures::future::BoxFuture;
use rig::agent::tool::ToolOutput;
use rig::client::CompletionClient as _;
use rig::completion::{Chat as _, Message, Prompt as _, PromptError};
use rig::message::{ImageMediaType, UserContent};
use secrecy::{ExposeSecret, SecretString};

use crate::providers::router::{Need, Router, Skip};
use crate::providers::{ModelConfig, ProviderConfig, ProviderKind, Role, free_only};
use crate::secret::SecretError;

pub use rig::AgentBuilder;
pub use rig::agent::tool::DynamicTool;

/// How long a model stays parked after a 429 that gave no retry time.
const DEFAULT_PARK: Duration = Duration::from_mins(1);

/// An image attached to a prompt.
#[derive(Debug, Clone)]
pub struct ImageInput {
    /// Encoded image bytes.
    pub bytes: Vec<u8>,
    /// JPEG or PNG.
    pub format: ImageFormat,
}

/// Encodings the providers all accept.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageFormat {
    /// `image/jpeg`.
    Jpeg,
    /// `image/png`.
    Png,
}

/// One prompt to run.
#[derive(Debug, Clone)]
pub struct Ask<'a> {
    /// Which role's chain to use.
    pub role: Role,
    /// The system prompt.
    pub preamble: &'a str,
    /// The user text.
    pub prompt: &'a str,
    /// An optional image.
    pub image: Option<ImageInput>,
}

/// A model's reply and which model gave it.
#[derive(Debug, Clone)]
pub struct Answer {
    /// The reply text.
    pub text: String,
    /// The model id from `models.toml`.
    pub model: String,
    /// Prompt tokens, as the provider reported them.
    pub input_tokens: u64,
    /// Reply tokens, as the provider reported them.
    pub output_tokens: u64,
}

/// Why no model answered.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum LlmError {
    /// Every candidate was skipped or failed.
    #[error("no model could answer: {}", describe(.skipped, .failed))]
    NoModel {
        /// Models passed over and why.
        skipped: Vec<(String, Skip)>,
        /// Models tried and their errors.
        failed: Vec<(String, String)>,
    },
    /// A provider key could not be loaded.
    #[error("provider `{provider}`: {source}")]
    Key {
        /// The provider id.
        provider: String,
        /// The load error.
        source: SecretError,
    },
    /// A client could not be built.
    #[error("provider `{provider}`: {message}")]
    Client {
        /// The provider id.
        provider: String,
        /// What went wrong.
        message: String,
    },
    /// A model broke the free-only rule at request time.
    #[error(transparent)]
    NotFree(#[from] free_only::FreeOnlyError),
    /// The model or provider failed during a turn.
    #[error("{model}: {message}")]
    Turn {
        /// The model id.
        model: String,
        /// What went wrong.
        message: String,
        /// The provider answered 429.
        rate_limited: bool,
    },
}

fn describe(skipped: &[(String, Skip)], failed: &[(String, String)]) -> String {
    let skipped = skipped.iter().map(|(m, s)| format!("{m} skipped ({s:?})"));
    let failed = failed.iter().map(|(m, e)| format!("{m} failed ({e})"));
    let all: Vec<String> = skipped.chain(failed).collect();
    if all.is_empty() {
        "no models configured".to_owned()
    } else {
        all.join("; ")
    }
}

/// Models ready to call, with their keys loaded once.
pub struct Llm {
    router: Router,
    keys: HashMap<String, SecretString>,
}

impl std::fmt::Debug for Llm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Keys are deliberately left out.
        f.debug_struct("Llm")
            .field("router", &self.router)
            .finish_non_exhaustive()
    }
}

impl Llm {
    /// Loads every provider key.
    ///
    /// # Errors
    ///
    /// [`LlmError::Key`] for a provider whose key cannot be read.
    pub fn new(router: Router) -> Result<Self, LlmError> {
        let mut keys = HashMap::new();
        for provider in &router.config().providers {
            if let Some(source) = &provider.key {
                let key = source.load().map_err(|source| LlmError::Key {
                    provider: provider.id.clone(),
                    source,
                })?;
                keys.insert(provider.id.clone(), key);
            }
        }
        Ok(Self { router, keys })
    }

    /// The router, for quota display and pool updates.
    #[must_use]
    pub fn router(&self) -> &Router {
        &self.router
    }

    /// A rig agent builder for one model, before tools, preamble or hooks are added.
    ///
    /// # Errors
    ///
    /// Unknown ids, a missing base URL, the free-only rule, or a rig client error.
    pub fn agent_builder(&self, model: &ModelConfig) -> Result<AgentBuilder, LlmError> {
        let provider = self
            .router
            .config()
            .provider_of(model)
            .ok_or_else(|| LlmError::Client {
                provider: model.provider.clone(),
                message: "unknown provider".to_owned(),
            })?;
        if provider.free_only
            && provider
                .base_url
                .as_deref()
                .is_some_and(|u| u.contains("openrouter.ai"))
        {
            free_only::check_openrouter_id(&model.model)?;
        }
        let key = self.keys.get(&provider.id);
        let http = http_client(provider)?;
        match provider.kind {
            ProviderKind::OpenaiCompat => {
                let base = provider.base_url.as_deref().unwrap_or_default();
                // Keyless local servers ignore the bearer value.
                let key = key.map_or("none", |k| k.expose_secret());
                let client = rig::providers::openai::Client::builder()
                    .api_key(key)
                    .base_url(base)
                    .http_client(http)
                    .build()
                    .map_err(|e| client_error(provider, &e))?
                    .completions_api();
                Ok(AgentBuilder::new(client.completion_model(&model.model)))
            }
            ProviderKind::GeminiInteractions => {
                let key = key.ok_or_else(|| LlmError::Client {
                    provider: provider.id.clone(),
                    message: "gemini_interactions needs a key".to_owned(),
                })?;
                // The Interactions client sends the key in x-goog-api-key, never in the URL.
                let client = rig::providers::gemini::Client::builder()
                    .api_key(key.expose_secret())
                    .http_client(http)
                    .build()
                    .map_err(|e| client_error(provider, &e))?
                    .interactions_api();
                Ok(AgentBuilder::new(client.completion_model(&model.model)))
            }
        }
    }

    /// Runs one prompt, trying the role's candidates in order.
    ///
    /// # Errors
    ///
    /// [`LlmError::NoModel`] when every candidate is skipped or fails.
    pub async fn ask(&self, ask: Ask<'_>) -> Result<Answer, LlmError> {
        let need = Need {
            vision: ask.image.is_some(),
            ..Need::default()
        };
        let (candidates, skipped) = self.router.candidates(ask.role, need, SystemTime::now());
        let mut failed = Vec::new();
        for model in candidates {
            let agent = self.agent_builder(model)?.preamble(ask.preamble).build();
            let message = user_message(ask.prompt, ask.image.as_ref());
            let result = agent.prompt(message).extended_details().await;
            // A failure to persist the ledger must not hide the answer; it is logged instead.
            if let Err(e) = self.router.record_use(&model.id, SystemTime::now()) {
                tracing::warn!(model = %model.id, error = %e, "could not save the quota ledger");
            }
            match result {
                Ok(response) => {
                    return Ok(Answer {
                        text: response.output,
                        model: model.id.clone(),
                        input_tokens: response.usage.input_tokens,
                        output_tokens: response.usage.output_tokens,
                    });
                }
                Err(e) => {
                    if is_rate_limited(&e)
                        && let Err(io) =
                            self.router.park(&model.id, SystemTime::now(), DEFAULT_PARK)
                    {
                        tracing::warn!(model = %model.id, error = %io, "could not save the quota ledger");
                    }
                    tracing::info!(model = %model.id, error = %e, "model failed, trying the next one");
                    failed.push((model.id.clone(), e.to_string()));
                }
            }
        }
        Err(LlmError::NoModel { skipped, failed })
    }
}

fn http_client(provider: &ProviderConfig) -> Result<reqwest::Client, LlmError> {
    let mut headers = reqwest::header::HeaderMap::new();
    for (name, value) in &provider.headers {
        let name = reqwest::header::HeaderName::from_bytes(name.as_bytes())
            .map_err(|e| client_error(provider, &e))?;
        let value = reqwest::header::HeaderValue::from_str(value)
            .map_err(|e| client_error(provider, &e))?;
        headers.insert(name, value);
    }
    reqwest::Client::builder()
        .default_headers(headers)
        .timeout(Duration::from_secs(provider.timeout_s))
        .build()
        .map_err(|e| client_error(provider, &e))
}

fn client_error(provider: &ProviderConfig, e: &dyn std::fmt::Display) -> LlmError {
    LlmError::Client {
        provider: provider.id.clone(),
        message: e.to_string(),
    }
}

/// A user message with optional image content, image first as most vision models prefer.
#[must_use]
pub fn user_message(text: &str, image: Option<&ImageInput>) -> Message {
    let mut content = Vec::new();
    if let Some(image) = image {
        let media = match image.format {
            ImageFormat::Jpeg => ImageMediaType::JPEG,
            ImageFormat::Png => ImageMediaType::PNG,
        };
        let data = base64::engine::general_purpose::STANDARD.encode(&image.bytes);
        content.push(UserContent::image_base64(data, Some(media), None));
    }
    content.push(UserContent::text(text));
    Message::User { content }
}

fn is_rate_limited(error: &PromptError) -> bool {
    match error {
        PromptError::CompletionError(e) => {
            e.provider_response_status().map(|s| s.as_u16()) == Some(429)
        }
        _ => false,
    }
}

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

/// The conversation so far, tool calls and results included, in rig's message form.
#[derive(Debug, Clone, Default)]
pub struct History(Vec<Message>);

impl History {
    /// Number of messages.
    #[must_use]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Whether it is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Drops the oldest messages beyond `max`, starting at a user message so no tool result is
    /// left without its call.
    pub fn trim(&mut self, max: usize) {
        if self.0.len() <= max {
            return;
        }
        let mut start = self.0.len() - max;
        while start < self.0.len() && !is_user_text(&self.0[start]) {
            start += 1;
        }
        self.0.drain(..start);
    }
}

fn is_user_text(m: &Message) -> bool {
    matches!(m, Message::User { content } if content.iter().all(|c| matches!(c, UserContent::Text(_))))
}

/// What the session asks of its model layer; [`Llm`] implements it, tests use a scripted model.
pub trait AgentSource: Send + Sync {
    /// Model ids to try for a role, in order.
    fn candidates(&self, role: Role, need: Need) -> Vec<String>;

    /// A rig builder for one model.
    ///
    /// # Errors
    ///
    /// The model cannot be built.
    fn builder(&self, model_id: &str) -> Result<AgentBuilder, LlmError>;

    /// Counts one request against the model's quota.
    fn record_use(&self, model_id: &str);

    /// Sets a model aside after a 429.
    fn park(&self, model_id: &str);
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

    fn record_use(&self, model_id: &str) {
        if let Err(e) = self.router.record_use(model_id, SystemTime::now()) {
            tracing::warn!(model = %model_id, error = %e, "could not save the quota ledger");
        }
    }

    fn park(&self, model_id: &str) {
        if let Err(e) = self.router.park(model_id, SystemTime::now(), DEFAULT_PARK) {
            tracing::warn!(model = %model_id, error = %e, "could not save the quota ledger");
        }
    }
}

/// Runs one user turn: the model may call the tools up to `max_turns` model calls in total.
/// Committed messages, tool calls and results included, are appended to `history`.
///
/// # Errors
///
/// [`LlmError::Turn`] when the provider or the loop fails.
pub async fn chat(
    model_id: &str,
    builder: AgentBuilder,
    preamble: &str,
    max_turns: usize,
    tools: &[LoopTool],
    history: &mut History,
    text: &str,
) -> Result<String, LlmError> {
    let dynamic: Vec<DynamicTool> = tools
        .iter()
        .map(|t| {
            let invoke = std::sync::Arc::clone(&t.invoke);
            DynamicTool::new(
                t.name.clone(),
                t.description.clone(),
                t.parameters.clone(),
                move |_cx, args| {
                    let fut = invoke(args);
                    Box::pin(async move { Ok(ToolOutput::json(fut.await)) })
                },
            )
        })
        .collect();
    let builder = builder
        .preamble(preamble)
        .default_max_turns(max_turns)
        .dynamic_tools(dynamic);
    let agent = builder.build();
    agent
        .chat(text, &mut history.0)
        .await
        .map_err(|e| LlmError::Turn {
            model: model_id.to_owned(),
            message: e.to_string(),
            rate_limited: is_rate_limited(&e),
        })
}
