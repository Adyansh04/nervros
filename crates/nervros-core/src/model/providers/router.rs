//! Picks the models for one call, in order.
//!
//! The role's chain is filtered by what the call needs, by the privacy mode and by the ledger.
//! Local models that fit are appended at the end, so a working local server is always the last
//! resort. The caller tries the candidates in order and reports what happened.

use std::path::Path;
use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, SystemTime};

use super::ledger::{Ledger, Refused, ResetZone, until_next_day};
use super::{ModelConfig, ModelsConfig, ProviderKind, Role};

/// What a call needs from a model.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Need {
    /// An image is attached.
    pub vision: bool,
    /// Tools are offered.
    pub tools: bool,
}

/// Where camera images may go, as the profile's `[privacy] mode` says.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
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
    #[must_use]
    pub fn with_ledger_file(config: ModelsConfig, path: &Path, privacy: PrivacyMode) -> Self {
        Self::new(config, Ledger::load(path), privacy)
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
        crate::lock(&self.ledger)
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
        // Outlining is a skill few models have, and a second opinion or advice from the model that
        // planned is neither: these roles ask only the models listed for them, as does a routine
        // role pinned to one model.
        let opt_in = matches!(role, Role::Segment | Role::PlanCheck | Role::Plan)
            || (role == Role::Routine && self.config.roles.routine_only);
        let local_tail = self
            .config
            .models
            .iter()
            .filter(|m| !opt_in && m.privacy.local && !chain.contains(&m.id))
            .map(|m| m.id.as_str());
        let mut take = Vec::new();
        let mut silent = Vec::new();
        let mut skipped = Vec::new();
        for id in chain.iter().map(String::as_str).chain(local_tail) {
            let Some(model) = self.config.model(id) else {
                continue;
            };
            match self.skip_reason(model, need, now) {
                None if self.ledger().silent(&model.id, now) => silent.push(model),
                None => take.push(model),
                Some(reason) => skipped.push((model.id.clone(), reason)),
            }
        }
        // A server that did not answer a moment ago may be up again: tried last, not left out.
        take.append(&mut silent);
        (take, skipped)
    }

    /// Counts one request if the model's limits still allow it, checked and counted under one
    /// lock; persists the ledger. A failure to write the file is logged: the count stands.
    ///
    /// # Errors
    ///
    /// [`Refused`] naming the limit that blocks the request, which is then not counted.
    pub fn take_request(&self, model_id: &str, now: SystemTime) -> Result<(), Refused> {
        let Some(model) = self.config.model(model_id) else {
            return Ok(());
        };
        let zone = self.zone(model);
        let mut ledger = self.ledger();
        ledger.check(&model.id, &model.limits, self.pool_of(model), now, zone)?;
        ledger.record(&model.id, model.limits.pool.as_deref(), now, zone);
        if let Err(e) = ledger.save() {
            tracing::warn!(model = %model.id, error = %e, "could not save the quota ledger");
        }
        Ok(())
    }

    /// Parks a model for `for_how_long`, at most until its provider's next daily reset; persists
    /// the ledger.
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
        // A spent daily quota comes back at the provider's midnight, whatever its Retry-After said:
        // Gemini's ran to UTC midnight, 17 hours past its own Pacific reset.
        let zone = self
            .config
            .model(model_id)
            .map_or(ResetZone::Utc, |m| self.zone(m));
        let for_how_long = for_how_long.min(until_next_day(now, zone));
        let mut ledger = self.ledger();
        ledger.park(model_id, now, for_how_long);
        ledger.save()
    }

    /// Tries a model whose server did not answer last for `for_how_long`, in this process only.
    pub fn set_aside(&self, model_id: &str, now: SystemTime, for_how_long: Duration) {
        self.ledger().set_aside(model_id, now, for_how_long);
    }

    /// How long until the first model for a call that its per-minute limit or a 429 holds back
    /// may take a request again; none when nothing holds a model back that briefly.
    #[must_use]
    pub fn ready_in(&self, role: Role, need: Need, now: SystemTime) -> Option<Duration> {
        let (_, skipped) = self.candidates(role, need, now);
        skipped
            .iter()
            .filter(|(_, skip)| matches!(skip, Skip::Refused(Refused::Minute | Refused::Parked(_))))
            .filter_map(|(id, _)| {
                let model = self.config.model(id)?;
                let (pool, zone) = (self.pool_of(model), self.zone(model));
                let ledger = self.ledger();
                ledger.lifts(id, now).into_iter().find(|d| {
                    ledger
                        .check(id, &model.limits, pool, now + *d, zone)
                        .is_ok()
                })
            })
            .min()
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
    fn segment_asks_only_the_models_listed_for_it() {
        let r = router(PrivacyMode::Sim);
        let need = Need {
            vision: true,
            ..Need::default()
        };
        let (take, _) = r.candidates(Role::Segment, need, SystemTime::now());
        assert!(take.is_empty(), "{:?}", ids(&take));
    }

    #[test]
    fn filters_by_capability() {
        let r = router(PrivacyMode::Sim);
        let need = Need {
            vision: true,
            ..Need::default()
        };
        let (take, skipped) = r.candidates(Role::Plan, need, SystemTime::now());
        assert_eq!(
            ids(&take),
            ["big"],
            "the plan role asks only its own models"
        );
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
        r.take_request("big", now).unwrap();
        r.take_request("text", now).unwrap();
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
    fn a_request_is_taken_only_while_the_pool_has_room() {
        let r = router(PrivacyMode::Sim);
        let now = SystemTime::now();
        assert_eq!(r.take_request("big", now), Ok(()));
        assert_eq!(r.take_request("text", now), Ok(()));
        assert_eq!(
            r.take_request("big", now),
            Err(Refused::Pool("or_free".into()))
        );
        assert_eq!(
            r.used_today("or_free", now),
            2,
            "a refused request is not counted"
        );
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

    #[test]
    fn says_when_a_short_limit_lifts_but_not_a_daily_one() {
        let r = router(PrivacyMode::Sim);
        let now = SystemTime::now();
        assert_eq!(r.ready_in(Role::Routine, Need::default(), now), None);
        r.park("big", now, Duration::from_secs(30)).unwrap();
        r.set_aside("text", now, Duration::from_secs(10));
        let (take, _) = r.candidates(Role::Routine, Need::default(), now);
        assert_eq!(
            ids(&take),
            ["local9b", "text"],
            "a silent server is tried last"
        );
        assert_eq!(
            r.ready_in(Role::Routine, Need::default(), now),
            Some(Duration::from_secs(30))
        );
        // The pool is spent by the time the park ends: waiting for it would be for nothing.
        r.take_request("text", now).unwrap();
        r.take_request("text", now).unwrap();
        assert_eq!(r.ready_in(Role::Routine, Need::default(), now), None);
    }

    #[test]
    fn a_park_ends_at_the_providers_midnight_whatever_it_asked() {
        let r = router(PrivacyMode::Sim);
        let late = std::time::UNIX_EPOCH + Duration::from_hours(20_000 * 24 + 23);
        r.park("big", late, Duration::from_hours(20)).unwrap();
        let (take, _) = r.candidates(Role::Routine, Need::default(), late);
        assert_eq!(ids(&take), ["text", "local9b"]);
        let past_midnight = late + Duration::from_secs(3601);
        let (take, _) = r.candidates(Role::Routine, Need::default(), past_midnight);
        assert_eq!(ids(&take), ["big", "text", "local9b"]);
    }
}
