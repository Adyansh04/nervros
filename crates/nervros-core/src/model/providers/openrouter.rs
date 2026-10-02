//! OpenRouter's public price list and the key's free-request counter.
//!
//! Neither call runs a model, so neither spends free requests.

use secrecy::{ExposeSecret, SecretString};
use serde::Deserialize;
use serde_json::Value;

/// The key's free-model counter for the current UTC day.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub struct FreeDaily {
    /// Free-model requests recorded so far today.
    pub used: u32,
    /// Free-model requests allowed per day.
    pub limit: u32,
    /// Free-model requests left today.
    pub remaining: u32,
}

/// A failed call to OpenRouter's account endpoints.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum OpenRouterError {
    /// The request failed or returned an error status.
    #[error("OpenRouter request failed: {0}")]
    Http(#[from] reqwest::Error),
    /// The response lacked the expected field.
    #[error("OpenRouter response has no `{0}`")]
    Missing(&'static str),
}

/// Fetches `GET {base}/models`, the public price list.
///
/// # Errors
///
/// Transport or status errors.
pub async fn fetch_models(
    http: &reqwest::Client,
    base_url: &str,
) -> Result<Value, OpenRouterError> {
    let url = format!("{}/models", base_url.trim_end_matches('/'));
    Ok(http
        .get(url)
        .send()
        .await?
        .error_for_status()?
        .json()
        .await?)
}

/// Fetches `GET {base}/key` and reads `free_model_daily_requests`.
///
/// # Errors
///
/// Transport or status errors, or a response without the counter.
pub async fn fetch_free_daily(
    http: &reqwest::Client,
    base_url: &str,
    key: &SecretString,
) -> Result<FreeDaily, OpenRouterError> {
    let url = format!("{}/key", base_url.trim_end_matches('/'));
    let body: Value = http
        .get(url)
        .bearer_auth(key.expose_secret())
        .send()
        .await?
        .error_for_status()?
        .json()
        .await?;
    let counter = body
        .pointer("/data/free_model_daily_requests")
        .cloned()
        .ok_or(OpenRouterError::Missing("data.free_model_daily_requests"))?;
    serde_json::from_value(counter)
        .map_err(|_| OpenRouterError::Missing("free_model_daily_requests"))
}
