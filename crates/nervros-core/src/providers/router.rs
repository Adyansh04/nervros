//! Picks the models for one call, in order.
//!
//! The role's chain is filtered by what the call needs, by the privacy mode and by the ledger.
//! Local models that fit are appended at the end, so a working local server is always the last
//! resort. The caller tries the candidates in order and reports what happened.

use std::path::Path;
use std::sync::{Mutex, MutexGuard, PoisonError};
use std::time::{Duration, SystemTime};

use super::ledger::{Ledger, Refused, ResetZone};
use super::{ModelConfig, ModelsConfig, ProviderKind, Role, Structured};

/// What a call needs from a model.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Need {
    /// An image is attached.
    pub vision: bool,
    /// Tools are offered.
    pub tools: bool,
    /// The answer must follow a JSON schema.
    pub structured: bool,
}

/// Where camera images may go.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum PrivacyMode {
    /// Simulated frames: any provider.
    #[default]
    Sim,
    /// Real frames: images only to local models, text only to providers that do not train.
    Home,
}

/// Why a model in the chain was passed over.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Skip {
    /// The call has an image and the model takes none.
    NoVision,
    /// The call offers tools and the model calls none.
    NoTools,
    /// The call needs structured output.
    NoStructured,
    /// The privacy mode forbids it.
    Privacy,
    /// A limit or a 429 blocks it.
    Refused(Refused),
}

/// The router: config, ledger and privacy mode behind one lock.
#[derive(Debug)]
pub struct Router {
    config: ModelsConfig,
    ledger: Mutex<Ledger>,
    privacy: PrivacyMode,
}

impl Router {
    /// A router over a validated config and a ledger.
    #[must_use]
    pub fn new(config: ModelsConfig, ledger: Ledger, privacy: PrivacyMode) -> Self {
        Self {
            config,
            ledger: Mutex::new(ledger),
            privacy,
        }
    }

    /// A router whose ledger persists at `path`.
    ///
    /// # Errors
    ///
    /// An unreadable ledger file.
    pub fn with_ledger_file(
        config: ModelsConfig,
        path: &Path,
        privacy: PrivacyMode,
    ) -> std::io::Result<Self> {
        Ok(Self::new(config, Ledger::load(path)?, privacy))
    }

    /// The config it routes over.
    #[must_use]
    pub fn config(&self) -> &ModelsConfig {
        &self.config
    }

    /// The privacy mode.
    #[must_use]
    pub fn privacy(&self) -> PrivacyMode {
        self.privacy
    }

    fn ledger(&self) -> MutexGuard<'_, Ledger> {
        // A panic while holding the lock leaves counts that are still valid numbers.
        self.ledger.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn zone(&self, model: &ModelConfig) -> ResetZone {
        match self.config.provider_of(model).map(|p| p.kind) {
            Some(ProviderKind::GeminiInteractions) => ResetZone::Pacific,
            _ => ResetZone::Utc,
        }
    }

    fn pool_of<'a>(&'a self, model: &'a ModelConfig) -> Option<(&'a str, u32)> {
        let name = model.limits.pool.as_deref()?;
        self.config.pools.get(name).map(|p| (name, p.rpd))
    }

    fn skip_reason(&self, model: &ModelConfig, need: Need, now: SystemTime) -> Option<Skip> {
        if need.vision && !model.vision {
            return Some(Skip::NoVision);
        }
        if need.tools && !model.tools {
            return Some(Skip::NoTools);
        }
        if need.structured && model.structured == Structured::None {
            return Some(Skip::NoStructured);
        }
        let private = match self.privacy {
            PrivacyMode::Sim => true,
            PrivacyMode::Home if need.vision => model.privacy.local,
            PrivacyMode::Home => model.privacy.local || !model.privacy.trains,
        };
        if !private {
            return Some(Skip::Privacy);
        }
        let ledger = self.ledger();
        ledger
            .check(
                &model.id,
                &model.limits,
                self.pool_of(model),
                now,
                self.zone(model),
            )
            .err()
            .map(Skip::Refused)
    }

    /// The models to try for a call, in order, and the ones passed over with the reason.
    #[must_use]
    pub fn candidates(
        &self,
        role: Role,
        need: Need,
        now: SystemTime,
    ) -> (Vec<&ModelConfig>, Vec<(String, Skip)>) {
        let chain = self.config.roles.chain(role);
        let local_tail = self
            .config
            .models
            .iter()
            .filter(|m| m.privacy.local && !chain.contains(&m.id))
            .map(|m| m.id.as_str());
        let mut take = Vec::new();
        let mut skipped = Vec::new();
        for id in chain.iter().map(String::as_str).chain(local_tail) {
            let Some(model) = self.config.model(id) else {
                continue;
            };
            match self.skip_reason(model, need, now) {
                None => take.push(model),
                Some(reason) => skipped.push((model.id.clone(), reason)),
            }
        }
        (take, skipped)
    }

    /// Counts one request; persists the ledger.
    ///
    /// # Errors
    ///
    /// A failure to write the ledger file.
    pub fn record_use(&self, model_id: &str, now: SystemTime) -> std::io::Result<()> {
        let Some(model) = self.config.model(model_id) else {
            return Ok(());
        };
        let mut ledger = self.ledger();
        ledger.record(
            &model.id,
            model.limits.pool.as_deref(),
            now,
            self.zone(model),
        );
        ledger.save()
    }

    /// Parks a model after a 429; persists the ledger.
    ///
    /// # Errors
    ///
    /// A failure to write the ledger file.
    pub fn park(
        &self,
        model_id: &str,
        now: SystemTime,
        for_how_long: Duration,
    ) -> std::io::Result<()> {
        let mut ledger = self.ledger();
        ledger.park(model_id, now, for_how_long);
        ledger.save()
    }

    /// Sets a pool's count from the provider's own figure; persists the ledger.
    ///
    /// # Errors
    ///
    /// A failure to write the ledger file.
    pub fn set_pool_used(&self, pool: &str, used: u32, now: SystemTime) -> std::io::Result<()> {
        let mut ledger = self.ledger();
        ledger.set_pool_used(pool, used, now, ResetZone::Utc);
        ledger.save()
    }

    /// Requests counted today for a model or pool.
    #[must_use]
    pub fn used_today(&self, key: &str, now: SystemTime) -> u32 {
        let zone = self
            .config
            .model(key)
            .map_or(ResetZone::Utc, |m| self.zone(m));
        self.ledger().used_today(key, now, zone)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CONFIG: &str = r#"
        [[provider]]
        id = "local"
        kind = "openai_compat"
        base_url = "http://127.0.0.1:8081/v1"

        [[provider]]
        id = "openrouter"
        kind = "openai_compat"
        base_url = "https://openrouter.ai/api/v1"
        free_only = true

        [[model]]
        id = "big"
        provider = "openrouter"
        model = "qwen/qwen3.8-27b:free"
        vision = true
        tools = true
        structured = "json_schema"
        limits = { rpm = 20, pool = "or_free" }

        [[model]]
        id = "text"
        provider = "openrouter"
        model = "nvidia/nemotron-3-super-120b-a12b:free"
        tools = true
        limits = { pool = "or_free" }
        privacy = { trains = true }

        [[model]]
        id = "local9b"
        provider = "local"
        model = "qwen3.5-9b"
        vision = true
        tools = true
        structured = "json_schema"
        privacy = { local = true }

        [pools]
        or_free = { rpd = 2 }

        [roles]
        routine = ["big", "text"]
        plan = ["text", "big"]
    "#;

    fn router(privacy: PrivacyMode) -> Router {
        Router::new(
            ModelsConfig::parse(CONFIG).unwrap(),
            Ledger::default(),
            privacy,
        )
    }

    fn ids(models: &[&ModelConfig]) -> Vec<String> {
        models.iter().map(|m| m.id.clone()).collect()
    }

    #[test]
    fn follows_the_chain_then_appends_local_models() {
        let r = router(PrivacyMode::Sim);
        let (take, _) = r.candidates(Role::Routine, Need::default(), SystemTime::now());
        assert_eq!(ids(&take), ["big", "text", "local9b"]);
    }

    #[test]
    fn filters_by_capability() {
        let r = router(PrivacyMode::Sim);
        let need = Need {
            vision: true,
            ..Need::default()
        };
        let (take, skipped) = r.candidates(Role::Plan, need, SystemTime::now());
        assert_eq!(ids(&take), ["big", "local9b"]);
        assert_eq!(skipped, [("text".to_owned(), Skip::NoVision)]);
    }

    #[test]
    fn home_mode_keeps_images_local_and_text_off_training_providers() {
        let r = router(PrivacyMode::Home);
        let image = Need {
            vision: true,
            ..Need::default()
        };
        let (take, _) = r.candidates(Role::Routine, image, SystemTime::now());
        assert_eq!(ids(&take), ["local9b"]);
        let (take, skipped) = r.candidates(Role::Routine, Need::default(), SystemTime::now());
        assert_eq!(ids(&take), ["big", "local9b"]);
        assert_eq!(skipped, [("text".to_owned(), Skip::Privacy)]);
    }

    #[test]
    fn a_used_up_pool_skips_every_model_in_it() {
        let r = router(PrivacyMode::Sim);
        let now = SystemTime::now();
        r.record_use("big", now).unwrap();
        r.record_use("text", now).unwrap();
        let (take, skipped) = r.candidates(Role::Routine, Need::default(), now);
        assert_eq!(ids(&take), ["local9b"]);
        assert!(
            skipped
                .iter()
                .all(|(_, s)| *s == Skip::Refused(Refused::Pool("or_free".into())))
        );
        assert_eq!(r.used_today("or_free", now), 2);
    }

    #[test]
    fn a_parked_model_is_skipped_until_its_time() {
        let r = router(PrivacyMode::Sim);
        let now = SystemTime::now();
        r.park("big", now, Duration::from_mins(1)).unwrap();
        let (take, _) = r.candidates(Role::Routine, Need::default(), now);
        assert_eq!(ids(&take), ["text", "local9b"]);
        let later = now + Duration::from_secs(61);
        let (take, _) = r.candidates(Role::Routine, Need::default(), later);
        assert_eq!(ids(&take), ["big", "text", "local9b"]);
    }
}
