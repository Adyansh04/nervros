//! The free-only rule: a provider marked `free_only` may never reach a paid model.
//!
//! Two checks. At config load, every OpenRouter model id must be an explicit `:free` variant (the
//! random `openrouter/free` router and `stealth/*` models are refused). At start-up, OpenRouter's
//! public price list must show zero for every price of every configured model.

use serde_json::Value;

use super::{ModelsConfig, ProviderConfig};

/// A model that could cost money.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum FreeOnlyError {
    /// An OpenRouter id is not an explicit `:free` variant.
    #[error("model `{0}` is not an explicit :free variant; free-only providers refuse it")]
    NotFreeVariant(String),
    /// The id is a router or stealth model, which can change underneath us.
    #[error("model `{0}` is a router or stealth model; free-only providers refuse it")]
    Unstable(String),
    /// The price list shows a non-zero price.
    #[error("model `{model}` has a non-zero `{field}` price ({price})")]
    Priced {
        /// The model id.
        model: String,
        /// Which price.
        field: String,
        /// The listed price.
        price: String,
    },
    /// The price list has no entry for the model.
    #[error("model `{0}` is not in the provider's price list")]
    NotListed(String),
}

fn is_openrouter(provider: &ProviderConfig) -> bool {
    provider
        .base_url
        .as_deref()
        .is_some_and(|url| url.contains("openrouter.ai"))
}

/// Checks the ids of every model on a free-only OpenRouter provider.
///
/// # Errors
///
/// The first offending model.
pub fn check_ids(config: &ModelsConfig) -> Result<(), FreeOnlyError> {
    for model in &config.models {
        let Some(provider) = config.provider_of(model) else {
            continue;
        };
        if !(provider.free_only && is_openrouter(provider)) {
            continue;
        }
        check_openrouter_id(&model.model)?;
    }
    Ok(())
}

/// The id rule on its own, also applied before every request.
///
/// # Errors
///
/// [`FreeOnlyError::Unstable`] or [`FreeOnlyError::NotFreeVariant`].
pub fn check_openrouter_id(id: &str) -> Result<(), FreeOnlyError> {
    if id.starts_with("openrouter/") || id.starts_with("stealth/") {
        return Err(FreeOnlyError::Unstable(id.to_owned()));
    }
    if !id.ends_with(":free") {
        return Err(FreeOnlyError::NotFreeVariant(id.to_owned()));
    }
    Ok(())
}

/// Checks OpenRouter's `/api/v1/models` response: every price of every id must be zero.
///
/// # Errors
///
/// The first model that is missing or priced.
pub fn check_prices(models_json: &Value, ids: &[&str]) -> Result<(), FreeOnlyError> {
    let listed = models_json.get("data").and_then(Value::as_array);
    for id in ids {
        let entry = listed
            .and_then(|list| {
                list.iter()
                    .find(|m| m.get("id").and_then(Value::as_str) == Some(id))
            })
            .ok_or_else(|| FreeOnlyError::NotListed((*id).to_owned()))?;
        let Some(pricing) = entry.get("pricing").and_then(Value::as_object) else {
            return Err(FreeOnlyError::NotListed((*id).to_owned()));
        };
        for (field, price) in pricing {
            // Prices are decimal strings in USD; anything unparseable counts as priced.
            let text = price
                .as_str()
                .map_or_else(|| price.to_string(), str::to_owned);
            let zero = text.parse::<f64>().is_ok_and(|p| p == 0.0);
            if !zero {
                return Err(FreeOnlyError::Priced {
                    model: (*id).to_owned(),
                    field: field.clone(),
                    price: text,
                });
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn only_explicit_free_variants_pass() {
        assert!(check_openrouter_id("qwen/qwen3.8-27b:free").is_ok());
        assert_eq!(
            check_openrouter_id("qwen/qwen3.8-27b"),
            Err(FreeOnlyError::NotFreeVariant("qwen/qwen3.8-27b".into()))
        );
        assert!(matches!(
            check_openrouter_id("openrouter/free"),
            Err(FreeOnlyError::Unstable(_))
        ));
        assert!(matches!(
            check_openrouter_id("stealth/space-bunny-alpha"),
            Err(FreeOnlyError::Unstable(_))
        ));
    }

    #[test]
    fn a_paid_id_on_a_free_only_provider_fails_config_validation() {
        let text = r#"
            [[provider]]
            id = "openrouter"
            kind = "openai_compat"
            base_url = "https://openrouter.ai/api/v1"
            free_only = true
            [[model]]
            id = "paid"
            provider = "openrouter"
            model = "anthropic/claude-sonnet-5.5"
            [roles]
        "#;
        let err = ModelsConfig::parse(text).unwrap_err();
        assert!(
            err.to_string().contains("not an explicit :free variant"),
            "{err}"
        );
    }

    #[test]
    fn prices_must_all_be_zero() {
        let list = json!({"data": [
            {"id": "a:free", "pricing": {"prompt": "0", "completion": "0", "image": "0"}},
            {"id": "b:free", "pricing": {"prompt": "0", "completion": "0.000002"}},
            {"id": "c:free", "pricing": {"prompt": "zero"}}
        ]});
        assert!(check_prices(&list, &["a:free"]).is_ok());
        assert!(matches!(
            check_prices(&list, &["b:free"]),
            Err(FreeOnlyError::Priced { .. })
        ));
        assert!(matches!(
            check_prices(&list, &["c:free"]),
            Err(FreeOnlyError::Priced { .. })
        ));
        assert_eq!(
            check_prices(&list, &["d:free"]),
            Err(FreeOnlyError::NotListed("d:free".into()))
        );
    }
}
