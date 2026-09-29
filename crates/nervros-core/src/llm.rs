//! The only module that imports `rig`.
//!
//! It turns a model from `models.toml` into a rig agent builder, and runs one prompt against the
//! router's candidates in order: a 429 parks the model and moves on, any other provider or
//! transport failure moves on without parking, and the answer names the model that gave it.

use std::collections::HashMap;
use std::time::{Duration, SystemTime};

use base64::Engine as _;
use rig::client::CompletionClient as _;
use rig::completion::{Message, Prompt as _, PromptError};
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
