//! The client: each model a rig agent builder from `models.toml`, asked in the order the router
//! gives, a 429 parking the model.

use std::collections::HashMap;
use std::time::{Duration, SystemTime};

use rig::AgentBuilder;
use rig::client::CompletionClient as _;
use rig::completion::{Prompt as _, PromptError};
use secrecy::{ExposeSecret as _, SecretString};

use super::turn::{AgentSource as _, without_provider_body};
use super::{Answer, Ask, LlmError, user_message};
use crate::providers::router::{Need, Router, Skip};
use crate::providers::{ModelConfig, ProviderConfig, ProviderKind, Role, free_only};

/// How long a model stays parked after a 429 that gave no retry time.
const DEFAULT_PARK: Duration = Duration::from_mins(1);

pub(super) fn describe(skipped: &[(String, Skip)], failed: &[(String, String)]) -> String {
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
    pub(super) router: Router,
    keys: HashMap<String, SecretString>,
    /// Why a provider's key could not be loaded; its models fail with this when asked.
    missing_keys: HashMap<String, String>,
    /// One HTTP client per provider, made once: it keeps its connections open between calls and
    /// reads the system's certificates once, not on every request.
    http: HashMap<String, Result<reqwest::Client, String>>,
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
    /// Loads every provider key. A key that cannot be read leaves its provider's models failing
    /// with the reason, so a missing optional key does not stop the app.
    #[must_use]
    pub fn new(router: Router) -> Self {
        let mut keys = HashMap::new();
        let mut missing_keys = HashMap::new();
        for provider in &router.config().providers {
            if let Some(source) = &provider.key {
                match source.load() {
                    Ok(key) => {
                        keys.insert(provider.id.clone(), key);
                    }
                    Err(e) => {
                        let error = LlmError::Key {
                            provider: provider.id.clone(),
                            source: e,
                        };
                        tracing::warn!(%error, "its models are left out");
                        missing_keys.insert(provider.id.clone(), error.to_string());
                    }
                }
            }
        }
        let http = router
            .config()
            .providers
            .iter()
            .map(|p| (p.id.clone(), http_client(p).map_err(|e| e.to_string())))
            .collect();
        Self {
            router,
            keys,
            missing_keys,
            http,
        }
    }

    /// Why each provider whose key could not be read is left out, for the operator at start.
    #[must_use]
    pub fn missing_keys(&self) -> Vec<String> {
        let mut why: Vec<String> = self
            .missing_keys
            .values()
            .map(|e| format!("{e}: its models are left out"))
            .collect();
        why.sort();
        why
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
        let builder = self.bare_builder(model)?;
        Ok(match &model.params {
            Some(params) => builder.additional_params(params.clone()),
            None => builder,
        })
    }

    fn bare_builder(&self, model: &ModelConfig) -> Result<AgentBuilder, LlmError> {
        let provider = self
            .router
            .config()
            .provider_of(model)
            .ok_or_else(|| LlmError::Client {
                provider: model.provider.clone(),
                message: "unknown provider".to_owned(),
            })?;
        if provider.is_openrouter() {
            free_only::check_openrouter_id(&model.model)?;
        }
        if let Some(why) = self.missing_keys.get(&provider.id) {
            return Err(LlmError::Client {
                provider: provider.id.clone(),
                message: why.clone(),
            });
        }
        let key = self.keys.get(&provider.id);
        let http = match self.http.get(&provider.id) {
            Some(Ok(client)) => client.clone(),
            Some(Err(why)) => {
                return Err(LlmError::Client {
                    provider: provider.id.clone(),
                    message: why.clone(),
                });
            }
            None => http_client(provider)?,
        };
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
            let agent = match self.agent_builder(model) {
                Ok(builder) => builder.preamble(ask.preamble).build(),
                Err(e) => {
                    failed.push((model.id.clone(), e.to_string()));
                    continue;
                }
            };
            // Checked and counted before the call, as a turn's are: a call cut short by a timeout
            // was still made, and the next candidate's limits are read anew.
            if let Err(refused) = self.router.take_request(&model.id, SystemTime::now()) {
                failed.push((model.id.clone(), refused.to_string()));
                continue;
            }
            // Gemini segments from the prompt before the image; given the image first, Flash-Lite
            // answered with boxes where the outlines belong.
            let message = user_message(ask.prompt, ask.image.as_ref(), ask.role == Role::Segment);
            let result = agent.prompt(message).extended_details().await;
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
                    if let Some(wait) = prompt_retry_after(&e) {
                        self.park(&model.id, wait);
                    }
                    let said = without_provider_body(&e.to_string());
                    tracing::info!(model = %model.id, error = %said, "model failed, trying the next one");
                    failed.push((model.id.clone(), said));
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

/// The longest a model is set aside: a `Retry-After` past a day is not believed.
const MAX_PARK: Duration = Duration::from_hours(24);

/// For a 429, how long to set the model aside: what the provider's `Retry-After` asks, else a
/// minute. `None` for any other error, whatever its text says.
pub(super) fn retry_after(error: &rig::completion::CompletionError) -> Option<Duration> {
    if error.provider_response_status()?.as_u16() != 429 {
        return None;
    }
    let asked = error
        .provider_response_headers()
        .and_then(|h| h.get("retry-after"))
        .and_then(|v| v.to_str().ok())
        .and_then(|s| s.trim().parse::<u64>().ok())
        .map(Duration::from_secs);
    Some(
        asked
            .unwrap_or(DEFAULT_PARK)
            .clamp(Duration::from_secs(1), MAX_PARK),
    )
}

pub(super) fn prompt_retry_after(error: &PromptError) -> Option<Duration> {
    match error {
        PromptError::CompletionError(e) => retry_after(e),
        _ => None,
    }
}

pub(super) fn stream_retry_after(error: &rig::agent::StreamingError) -> Option<Duration> {
    match error {
        rig::agent::StreamingError::Completion(e) => retry_after(e),
        rig::agent::StreamingError::Prompt(e) => prompt_retry_after(e),
    }
}
