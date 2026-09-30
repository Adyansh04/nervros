//! Model providers and models, declared in `models.toml`.
//!
//! A provider is an endpoint with an auth method; a model is one model on a provider with its
//! capabilities, limits and privacy terms; roles are ordered chains of models. Nothing outside this
//! file names a model, so switching provider is a config edit.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use serde::Deserialize;

use crate::secret::KeySource;

pub mod free_only;
pub mod ledger;
pub mod openrouter;
pub mod router;

/// The whole `models.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelsConfig {
    /// Endpoints.
    #[serde(rename = "provider", default)]
    pub providers: Vec<ProviderConfig>,
    /// Models on those endpoints.
    #[serde(rename = "model", default)]
    pub models: Vec<ModelConfig>,
    /// Shared request budgets, such as OpenRouter's account-wide free pool.
    #[serde(default)]
    pub pools: BTreeMap<String, PoolConfig>,
    /// Ordered model chains per role.
    pub roles: RolesConfig,
}

/// How a provider is spoken to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum ProviderKind {
    /// OpenAI Chat Completions: llama.cpp, OpenRouter, Groq and similar.
    OpenaiCompat,
    /// Google's Gemini Interactions API, which sends the key only in the `x-goog-api-key` header.
    GeminiInteractions,
}

/// One endpoint.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProviderConfig {
    /// Referenced by models.
    pub id: String,
    /// Wire protocol.
    pub kind: ProviderKind,
    /// Base URL; required for `openai_compat`.
    pub base_url: Option<String>,
    /// API key source; absent for keyless local servers.
    pub key: Option<KeySource>,
    /// Refuse any model that is not free (see [`free_only`]).
    #[serde(default)]
    pub free_only: bool,
    /// Extra HTTP headers sent with every request.
    #[serde(default)]
    pub headers: BTreeMap<String, String>,
    /// Per-request timeout in seconds.
    #[serde(default = "default_timeout_s")]
    pub timeout_s: u64,
}

fn default_timeout_s() -> u64 {
    60
}

/// Whether a model can be forced into a JSON schema.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Structured {
    /// Plain text only.
    #[default]
    None,
    /// JSON Schema constrained output.
    JsonSchema,
}

/// How an image produced by a tool reaches this model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImageInToolResult {
    /// Inside the tool result.
    Native,
    /// As a follow-up user message, for chat-completion endpoints that take text-only tool results.
    #[default]
    FollowUp,
    /// Not at all.
    None,
}

/// Request limits of one model. A missing field means unlimited.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Limits {
    /// Requests per minute.
    pub rpm: Option<u32>,
    /// Requests per day for this model alone.
    pub rpd: Option<u32>,
    /// A shared pool from `[pools]` this model also draws on.
    pub pool: Option<String>,
}

/// Data terms of one model.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Privacy {
    /// Runs on this machine.
    #[serde(default)]
    pub local: bool,
    /// The provider may train on or have people read prompts.
    #[serde(default)]
    pub trains: bool,
}

/// One model on one provider.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelConfig {
    /// Referenced by roles.
    pub id: String,
    /// The provider's `id`.
    pub provider: String,
    /// The provider's model name.
    pub model: String,
    /// Accepts images.
    #[serde(default)]
    pub vision: bool,
    /// Calls tools.
    #[serde(default)]
    pub tools: bool,
    /// Honours a forced tool choice.
    #[serde(default)]
    pub tool_choice: bool,
    /// Constrained output support.
    #[serde(default)]
    pub structured: Structured,
    /// How tool images reach it.
    #[serde(default)]
    pub image_in_tool_result: ImageInToolResult,
    /// Request limits.
    #[serde(default)]
    pub limits: Limits,
    /// Data terms.
    #[serde(default)]
    pub privacy: Privacy,
    /// Request fields passed to the provider as they are, such as Gemini's
    /// `{ generation_config = { thinking_level = "low" } }`.
    pub params: Option<serde_json::Value>,
}

/// A shared daily budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PoolConfig {
    /// Requests per day across every model in the pool.
    pub rpd: u32,
}

/// What a model call is for. Each role has its own ordered chain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Role {
    /// Conversation and tool calls.
    Routine,
    /// Writing a mission plan.
    Plan,
    /// Spatial checks and pointing.
    VisionCheck,
    /// Captions and summaries.
    Summarise,
    /// Outlining what a prompt names in a camera frame, for `segment`.
    Segment,
}

impl Role {
    /// Every role.
    pub const ALL: [Self; 5] = [
        Self::Routine,
        Self::Plan,
        Self::VisionCheck,
        Self::Summarise,
        Self::Segment,
    ];
}

/// Ordered model ids per role; the first usable one is tried first.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RolesConfig {
    /// See [`Role::Routine`].
    #[serde(default)]
    pub routine: Vec<String>,
    /// See [`Role::Plan`].
    #[serde(default)]
    pub plan: Vec<String>,
    /// See [`Role::VisionCheck`].
    #[serde(default)]
    pub vision_check: Vec<String>,
    /// See [`Role::Summarise`].
    #[serde(default)]
    pub summarise: Vec<String>,
    /// See [`Role::Segment`].
    #[serde(default)]
    pub segment: Vec<String>,
}

impl RolesConfig {
    /// The chain for a role.
    #[must_use]
    pub fn chain(&self, role: Role) -> &[String] {
        match role {
            Role::Routine => &self.routine,
            Role::Plan => &self.plan,
            Role::VisionCheck => &self.vision_check,
            Role::Summarise => &self.summarise,
            Role::Segment => &self.segment,
        }
    }
}

/// A `models.toml` that cannot be used.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ConfigError {
    /// The file could not be read.
    #[error("cannot read {path}: {source}")]
    Read {
        /// The file.
        path: String,
        /// The underlying error.
        source: std::io::Error,
    },
    /// The TOML does not match the schema.
    #[error("invalid models config: {0}")]
    Parse(#[from] toml::de::Error),
    /// Two providers or two models share an id.
    #[error("duplicate {kind} id `{id}`")]
    Duplicate {
        /// `provider` or `model`.
        kind: &'static str,
        /// The repeated id.
        id: String,
    },
    /// A reference names nothing.
    #[error("{from} refers to unknown {kind} `{id}`")]
    Unknown {
        /// Where the reference is.
        from: String,
        /// `provider`, `model` or `pool`.
        kind: &'static str,
        /// The missing id.
        id: String,
    },
    /// An `openai_compat` provider has no base URL.
    #[error("provider `{0}` needs a base_url")]
    NoBaseUrl(String),
    /// A model breaks its provider's free-only rule.
    #[error(transparent)]
    NotFree(#[from] free_only::FreeOnlyError),
}

impl ModelsConfig {
    /// Parses and validates a `models.toml` file.
    ///
    /// # Errors
    ///
    /// [`ConfigError`] for unreadable files, schema errors and every check in [`Self::validate`].
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let text = std::fs::read_to_string(path).map_err(|source| ConfigError::Read {
            path: path.display().to_string(),
            source,
        })?;
        Self::parse(&text)
    }

    /// Parses and validates TOML text.
    ///
    /// # Errors
    ///
    /// As [`Self::load`].
    pub fn parse(text: &str) -> Result<Self, ConfigError> {
        let config: Self = toml::from_str(text)?;
        config.validate()?;
        Ok(config)
    }

    /// Checks ids, references, base URLs and the free-only rule.
    ///
    /// # Errors
    ///
    /// The first problem found, as a [`ConfigError`].
    pub fn validate(&self) -> Result<(), ConfigError> {
        let mut providers = BTreeSet::new();
        for p in &self.providers {
            if !providers.insert(p.id.as_str()) {
                return Err(ConfigError::Duplicate {
                    kind: "provider",
                    id: p.id.clone(),
                });
            }
            if p.kind == ProviderKind::OpenaiCompat && p.base_url.is_none() {
                return Err(ConfigError::NoBaseUrl(p.id.clone()));
            }
        }
        let mut models = BTreeSet::new();
        for m in &self.models {
            if !models.insert(m.id.as_str()) {
                return Err(ConfigError::Duplicate {
                    kind: "model",
                    id: m.id.clone(),
                });
            }
            if !providers.contains(m.provider.as_str()) {
                return Err(ConfigError::Unknown {
                    from: format!("model `{}`", m.id),
                    kind: "provider",
                    id: m.provider.clone(),
                });
            }
            if let Some(pool) = &m.limits.pool
                && !self.pools.contains_key(pool)
            {
                return Err(ConfigError::Unknown {
                    from: format!("model `{}`", m.id),
                    kind: "pool",
                    id: pool.clone(),
                });
            }
        }
        for role in Role::ALL {
            for id in self.roles.chain(role) {
                if !models.contains(id.as_str()) {
                    return Err(ConfigError::Unknown {
                        from: format!("role {role:?}"),
                        kind: "model",
                        id: id.clone(),
                    });
                }
            }
        }
        free_only::check_ids(self)?;
        Ok(())
    }

    /// The provider a model runs on.
    #[must_use]
    pub fn provider_of(&self, model: &ModelConfig) -> Option<&ProviderConfig> {
        self.providers.iter().find(|p| p.id == model.provider)
    }

    /// A model by id.
    #[must_use]
    pub fn model(&self, id: &str) -> Option<&ModelConfig> {
        self.models.iter().find(|m| m.id == id)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// The example profile's models file, shared by the provider tests.
    pub(crate) const EXAMPLE: &str = include_str!("../../../profiles/example/models.toml");

    #[test]
    fn the_example_file_is_valid() {
        let config = ModelsConfig::parse(EXAMPLE).unwrap();
        assert!(!config.roles.routine.is_empty());
        let local = config.model("qwen3.5-9b-local").unwrap();
        assert!(local.privacy.local && local.vision && local.tools);
    }

    #[test]
    fn rejects_unknown_references_and_duplicates() {
        let dup = r#"
            [[provider]]
            id = "a"
            kind = "openai_compat"
            base_url = "http://x"
            [[provider]]
            id = "a"
            kind = "openai_compat"
            base_url = "http://y"
            [roles]
        "#;
        assert!(matches!(
            ModelsConfig::parse(dup),
            Err(ConfigError::Duplicate { .. })
        ));
        let unknown = r#"
            [[provider]]
            id = "a"
            kind = "openai_compat"
            base_url = "http://x"
            [[model]]
            id = "m"
            provider = "b"
            model = "x"
            [roles]
        "#;
        assert!(matches!(
            ModelsConfig::parse(unknown),
            Err(ConfigError::Unknown { .. })
        ));
        let role = r#"
            [roles]
            routine = ["ghost"]
        "#;
        assert!(matches!(
            ModelsConfig::parse(role),
            Err(ConfigError::Unknown { .. })
        ));
    }

    #[test]
    fn openai_compat_needs_a_base_url() {
        let text = r#"
            [[provider]]
            id = "a"
            kind = "openai_compat"
            [roles]
        "#;
        assert!(matches!(
            ModelsConfig::parse(text),
            Err(ConfigError::NoBaseUrl(_))
        ));
    }
}
