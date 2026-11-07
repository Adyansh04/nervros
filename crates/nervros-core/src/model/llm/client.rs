//! The client: each model a rig agent builder from `models.toml`, asked in the order the router
//! gives, a 429 or a silent server parking the model and a busy one asked once more.

use std::collections::HashMap;
use std::time::{Duration, SystemTime};

use rig::completion::{CacheRetention, Message, PromptError};
use rig::error::ErrorReport;
use rig::http_client::ReqwestClient;
use rig::providers::anthropic::AnthropicConfig;
use rig::providers::gemini::GeminiConfig;
use rig::providers::openai::OpenAIConfig;
use rig::run::PromptResponse;
use rig::{Agent, AgentBuilder, ErrorKind};
use secrecy::{ExposeSecret as _, SecretString};

use super::turn::{AgentSource, CallCost, without_provider_body};
use super::{Answer, Ask, BUSY_RETRY, LlmError, Setback, user_message};
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
    /// One HTTP client per provider, and one more for its streamed models, made once: it keeps
    /// its connections open between calls and reads the system's certificates once, not on every
    /// request. Keyed by provider id and whether the model streams.
    http: HashMap<(String, bool), Result<reqwest::Client, String>>,
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
        let config = router.config();
        let http = config
            .providers
            .iter()
            .flat_map(|p| {
                let streams = config.models.iter().any(|m| m.provider == p.id && m.stream);
                [false, true]
                    .into_iter()
                    .filter(move |&streamed| !streamed || streams)
                    .map(move |streamed| {
                        let client = http_client(p, streamed).map_err(|e| e.to_string());
                        ((p.id.clone(), streamed), client)
                    })
            })
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
        let mut builder = self.bare_builder(model)?;
        if let Some(params) = &model.params {
            builder = builder.additional_params(params.clone());
        }
        if let Some(max) = model.max_tokens {
            builder = builder.max_tokens(max);
        }
        Ok(builder.record_content_telemetry(crate::telemetry::content()))
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
            free_only::check_openrouter_params(&model.model, model.params.as_ref())?;
        }
        if let Some(why) = self.missing_keys.get(&provider.id) {
            return Err(LlmError::Client {
                provider: provider.id.clone(),
                message: why.clone(),
            });
        }
        let key = self.keys.get(&provider.id);
        // A key goes out encrypted, or to this machine only.
        if key.is_some()
            && provider
                .base_url
                .as_deref()
                .is_some_and(|url| !secure_or_local(url))
        {
            return Err(LlmError::Client {
                provider: provider.id.clone(),
                message: "a key is sent over https only, or to this machine".to_owned(),
            });
        }
        let http = match self.http.get(&(provider.id.clone(), model.stream)) {
            Some(Ok(client)) => client.clone(),
            Some(Err(why)) => {
                return Err(LlmError::Client {
                    provider: provider.id.clone(),
                    message: why.clone(),
                });
            }
            None => http_client(provider, model.stream)?,
        };
        // How long a streamed reply may stay silent; a unary one is timed by its HTTP client.
        let idle = Some(Duration::from_secs(provider.timeout_s));
        match provider.kind {
            ProviderKind::OpenaiCompat => {
                let base = provider.base_url.as_deref().unwrap_or_default();
                // Keyless local servers ignore the bearer value.
                let key = key.map_or("none", |k| k.expose_secret());
                let client = OpenAIConfig::new(key)
                    .with_base_url(base)
                    .connect(ReqwestClient::from(http));
                Ok(AgentBuilder::new(
                    client.chat(&model.model).with_idle_timeout(idle),
                ))
            }
            ProviderKind::GeminiInteractions => {
                let key = key.ok_or_else(|| LlmError::Client {
                    provider: provider.id.clone(),
                    message: "gemini_interactions needs a key".to_owned(),
                })?;
                // The Interactions wire sends the key in x-goog-api-key, never in the URL.
                let config = GeminiConfig::new(key.expose_secret());
                let config = match &provider.base_url {
                    Some(base) => config.with_base_url(base.as_str()),
                    None => config,
                };
                let client = config.connect(ReqwestClient::from(http));
                Ok(AgentBuilder::new(
                    client.interactions(&model.model).with_idle_timeout(idle),
                ))
            }
            ProviderKind::Anthropic => {
                let key = key.ok_or_else(|| LlmError::Client {
                    provider: provider.id.clone(),
                    message: "anthropic needs a key".to_owned(),
                })?;
                let config = AnthropicConfig::new(key.expose_secret());
                let config = match &provider.base_url {
                    Some(base) => config.with_base_url(base),
                    None => config,
                };
                let client = config.connect(ReqwestClient::from(http));
                let mut messages = client.completion(&model.model).with_idle_timeout(idle);
                // Read back from the cache, a prompt costs a tenth of sending it again. The tools
                // and the system prompt are cached on their own and the conversation at its end,
                // so a system prompt whose stacks' status changed still finds the tools cached.
                messages.wire = messages.wire.with_prompt_caching();
                Ok(AgentBuilder::new(messages).cache(CacheRetention::Short))
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
            vision: !ask.images.is_empty(),
            ..Need::default()
        };
        let (candidates, skipped) = self.router.candidates(ask.role, need, SystemTime::now());
        let models: Vec<String> = candidates.into_iter().map(|m| m.id.clone()).collect();
        // Gemini segments from the prompt before the image; given the image first, Flash-Lite
        // answered with boxes where the outlines belong.
        let message = user_message(ask.prompt, &ask.images, ask.role == Role::Segment);
        match first_answer(self, models, ask.preamble, &message, |_| true).await {
            // A count the provider leaves out is zero tokens to the report.
            Ok((model, response)) => Ok(Answer {
                text: response.output(),
                model,
                input_tokens: response.usage.input_tokens.unwrap_or(0),
                output_tokens: response.usage.output_tokens.unwrap_or(0),
            }),
            Err(failed) => Err(LlmError::NoModel { skipped, failed }),
        }
    }
}

/// The first answer to `message` that `accept` takes, from `models` in turn, each built with
/// `preamble`. Each call is checked and counted before it is made, as a turn's are: a call cut
/// short by a timeout was still made, and the next model's limits are read anew. A failed model is
/// set back; each failure comes back as the model and why.
pub(super) async fn first_answer(
    source: &dyn AgentSource,
    models: Vec<String>,
    preamble: &str,
    message: &Message,
    accept: impl Fn(&PromptResponse) -> bool,
) -> Result<(String, PromptResponse), Vec<(String, String)>> {
    let mut failed = Vec::new();
    for model in models {
        let agent = match source.builder(&model) {
            Ok(builder) => builder.preamble(preamble).build(),
            Err(e) => {
                failed.push((model, e.to_string()));
                continue;
            }
        };
        if let Err(refused) = source.take_request(&model) {
            failed.push((model, refused));
            continue;
        }
        let again = || source.take_request(&model).is_ok();
        match prompt_once_more(&agent, message.clone(), again).await {
            Ok(response) => {
                source.spent(&model, CallCost::of(&response.usage));
                if accept(&response) {
                    return Ok((model, response));
                }
                failed.push((model, "its answer was empty".to_owned()));
            }
            Err(e) => {
                if let Some(setback) = prompt_setback(&e) {
                    source.set_back(&model, setback);
                }
                let said = without_provider_body(&e.to_string());
                tracing::info!(model = %model, error = %said, "model failed, trying the next one");
                failed.push((model, said));
            }
        }
    }
    Err(failed)
}

/// Whether a key may go to `url`: over https, or over http to this machine.
fn secure_or_local(url: &str) -> bool {
    let url = url.trim().to_ascii_lowercase();
    if url.starts_with("https://") {
        return true;
    }
    let Some(rest) = url.strip_prefix("http://") else {
        return false;
    };
    let authority = rest.split('/').next().unwrap_or_default();
    // The host alone: `localhost.example.com` or a port does not make it this machine.
    let host = match authority.strip_prefix('[') {
        Some(v6) => v6.split(']').next().unwrap_or_default(),
        None => authority.split(':').next().unwrap_or_default(),
    };
    matches!(host, "localhost" | "127.0.0.1" | "::1")
}

/// The provider's HTTP client: `timeout_s` bounds a unary call whole, and the connection and each
/// silence of a streaming model's calls, whose replies may take long while words keep coming.
/// A unary call to a streaming model (a compaction, a check) is then bounded by its silence too.
fn http_client(provider: &ProviderConfig, streamed: bool) -> Result<reqwest::Client, LlmError> {
    let mut headers = reqwest::header::HeaderMap::new();
    for (name, value) in &provider.headers {
        let name = reqwest::header::HeaderName::from_bytes(name.as_bytes())
            .map_err(|e| client_error(provider, &e))?;
        let value = reqwest::header::HeaderValue::from_str(value)
            .map_err(|e| client_error(provider, &e))?;
        headers.insert(name, value);
    }
    let timeout = Duration::from_secs(provider.timeout_s);
    let builder = reqwest::Client::builder()
        .default_headers(headers)
        .connect_timeout(timeout);
    if streamed {
        builder.read_timeout(timeout)
    } else {
        builder.timeout(timeout)
    }
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

/// What a failure says about the model: limited by a 429 for as long as the provider asks (its
/// `Retry-After`, or the wait Gemini writes in the body), else a minute; busy on a server error;
/// unreachable when nothing answered at all or a streamed reply went silent. `None` for any other
/// failure, whatever its text says, but for [`malformed_call`].
pub(super) fn setback(report: &ErrorReport) -> Option<Setback> {
    if matches!(report.kind, ErrorKind::Http | ErrorKind::Timeout) {
        return Some(Setback::Unreachable);
    }
    let status = report.provider_response_status();
    if status == Some(reqwest::StatusCode::TOO_MANY_REQUESTS) {
        let wait = report
            .retry_after(SystemTime::now())
            .unwrap_or(DEFAULT_PARK)
            .clamp(Duration::from_secs(1), MAX_PARK);
        return Some(Setback::Limited(wait));
    }
    if status.is_some_and(|s| s.is_server_error()) {
        return Some(Setback::Busy);
    }
    malformed_call(report)
}

/// Gemini's 400 for a tool call it wrote as broken JSON, which it asks to be retried: the same
/// request usually comes back whole. The one 400 judged by its text, since its status alone would
/// also retry a request that can never succeed.
fn malformed_call(report: &ErrorReport) -> Option<Setback> {
    let said = |text: &str| text.contains("Model generated invalid JSON");
    (report.provider_response_status() == Some(reqwest::StatusCode::BAD_REQUEST)
        && (said(&report.message) || report.provider_response_body().is_some_and(said)))
    .then_some(Setback::Busy)
}

/// What a run's failure says about its model; see [`setback`].
pub(super) fn prompt_setback(error: &PromptError) -> Option<Setback> {
    with_report(error, setback).flatten()
}

/// `read` of the provider's report on a failed run, when it gave one.
fn with_report<T>(error: &PromptError, read: impl FnOnce(&ErrorReport) -> T) -> Option<T> {
    match error {
        PromptError::Provider(e) => Some(read(&e.report())),
        PromptError::Report(report) => Some(read(report)),
        _ => None,
    }
}

/// Whether a run failed because its request outgrew the model's context window, as the provider
/// says it in any of its words.
pub(super) fn overflowed(error: &PromptError) -> bool {
    with_report(error, ErrorReport::is_context_overflow).unwrap_or(false)
}

/// `agent`'s answer to `message`, asked once more [`BUSY_RETRY`] later when the provider was busy
/// and `again` counts a second request against the model's limits.
pub(super) async fn prompt_once_more(
    agent: &Agent,
    message: impl Into<Message>,
    again: impl FnOnce() -> bool,
) -> Result<PromptResponse, PromptError> {
    let message = message.into();
    match agent.prompt(message.clone()).await {
        Err(e) if prompt_setback(&e) == Some(Setback::Busy) && again() => {
            tokio::time::sleep(BUSY_RETRY).await;
            agent.prompt(message).await
        }
        answered => answered,
    }
}

#[cfg(test)]
mod tests {
    use super::secure_or_local;

    #[test]
    fn a_key_goes_over_https_or_to_this_machine_only() {
        assert!(secure_or_local("https://api.anthropic.com"));
        assert!(secure_or_local("http://127.0.0.1:18081/v1"));
        assert!(secure_or_local("http://[::1]:8080"));
        assert!(!secure_or_local("http://localhost.example.com/v1"));
        assert!(!secure_or_local("http://10.0.0.2:8080"));
    }
}
