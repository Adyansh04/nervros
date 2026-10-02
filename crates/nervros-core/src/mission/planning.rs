//! Planning: the catalog and the world a plan compiles against, the checks a plan passes, and what the model is told when it fails.

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;
use std::time::Instant;

use super::catalog::Catalog;
use super::check::Observed;
use super::plan::Author as By;
use super::plan::Compiled;
use super::plan::Plan;
use super::plan::PlannedStep;
use super::plan::Thing;
use super::plan::World;
use super::sanity::Concern;
use super::sanity::Verdict;
use super::{advice, ledger, plan, preview, sanity};
use serde_json::Value;
use serde_json::json;

use super::CATALOG_RETRY;
use super::CRITIC_TIMEOUT;
use super::KEPT_PLANS;
use super::MAX_PLAN_ATTEMPTS;
use super::MIN_HASH_PREFIX;
use super::Missions;
use super::SERVICE_TIMEOUT;
use super::STATE;
use super::STATE_FRESH;
use super::WORLD_WAIT;
use super::held_by;
use super::tool::concern_line;
use super::tool::questioned;
use crate::lock;
use crate::session::Event;
use crate::tools::Status;
use crate::tools::ToolOutcome;

impl Missions {
    /// Asks the executor where a checked plan would take the robot, and tells the window: the
    /// approval card shows at once, and the picture follows when the planner has answered.
    pub(super) async fn preview(&self, hash: &str) {
        let Some(service) = &self.config.preview else {
            return;
        };
        let Ok(compiled) = self.find(hash) else {
            return;
        };
        let reply = self
            .robot
            .call(
                service,
                "nervros_interfaces/srv/PreviewMission",
                json!({"tree_xml": compiled.xml}),
                SERVICE_TIMEOUT,
            )
            .await;
        match reply
            .map_err(|e| e.to_string())
            .and_then(|r| preview::parse(&r))
        {
            Ok(steps) => self.emit(Event::MissionPreview {
                hash: compiled.sha256,
                steps,
            }),
            Err(why) => tracing::info!(%why, "no preview of the plan"),
        }
    }

    /// Why the robot cannot run a plan that walks now, from the executor's own state.
    async fn cannot_walk(&self, compiled: &Compiled, catalog: &Catalog) -> Option<String> {
        let walks = compiled.steps.iter().any(|s| {
            s.skill == "GoToPlace"
                || catalog
                    .skill(&s.skill)
                    .is_some_and(|k| k.resources.iter().any(|r| r == "base"))
        });
        if !walks {
            return None;
        }
        let state = self
            .robot
            .latest_fresh(
                &self.config.state,
                STATE,
                Duration::from_secs(1),
                STATE_FRESH,
            )
            .await
            .ok()?;
        if state["can_move"].as_bool() != Some(false) {
            return None;
        }
        let reason = state["cannot_move_reason"]
            .as_str()
            .filter(|r| !r.is_empty())
            .unwrap_or("the robot says so");
        Some(format!(
            "the robot cannot walk now: {reason}. Tell the operator; plan no walk until they say it is fixed"
        ))
    }

    /// The catalog, fetched once and kept.
    pub(super) async fn catalog(&self) -> Result<Arc<Catalog>, String> {
        if let Some(c) = lock(&self.catalog).clone() {
            return Ok(c);
        }
        let fetched = self
            .robot
            .call(
                &self.config.catalog,
                "nervros_interfaces/srv/GetCatalog",
                json!({}),
                SERVICE_TIMEOUT,
            )
            .await
            .map_err(|e| format!("the robot's skill catalog is not available: {e}"))
            .and_then(|reply| {
                let text = reply["catalog_json"].as_str().unwrap_or_default();
                Catalog::parse(text).map_err(|e| format!("the skill catalog is malformed: {e}"))
            });
        match fetched {
            Ok(catalog) => {
                let catalog = Arc::new(catalog);
                *lock(&self.catalog) = Some(Arc::clone(&catalog));
                Ok(catalog)
            }
            Err(e) => {
                *lock(&self.catalog_failed) = Some(Instant::now());
                Err(e)
            }
        }
    }

    /// Waits for the catalog before a turn, unless it is in hand or failed lately.
    pub(super) async fn catalog_ready(&self) {
        let failed_lately = lock(&self.catalog_failed).is_some_and(|t| t.elapsed() < CATALOG_RETRY);
        if lock(&self.catalog).is_some() || failed_lately {
            return;
        }
        let _ = tokio::time::timeout(SERVICE_TIMEOUT / 2, self.catalog()).await;
    }

    /// The newest message on a world topic, or null when it has none.
    async fn latest(&self, topic: Option<&crate::profile::TopicRef>) -> Value {
        match topic {
            Some(t) => self
                .robot
                .latest(&t.topic, &t.msg_type, WORLD_WAIT)
                .await
                .unwrap_or(Value::Null),
            None => Value::Null,
        }
    }

    /// What the plan compiler knows: places, rooms and objects with their positions, where the
    /// robot is and what it holds.
    fn world_of(&self, seen: &Observed) -> World {
        let things = |msg: &Value, list: &str, name: &str, at: &str| -> Vec<Thing> {
            msg[list]
                .as_array()
                .map(|items| {
                    items
                        .iter()
                        .filter(|o| o["state"].as_u64() != Some(2))
                        .map(|o| {
                            let text = |k: &str| o[k].as_str().unwrap_or_default().to_owned();
                            let p = o.pointer(at);
                            let xy = p.and_then(|p| Some((p["x"].as_f64()?, p["y"].as_f64()?)));
                            Thing {
                                id: text("id"),
                                name: text(name),
                                xy,
                            }
                        })
                        .collect()
                })
                .unwrap_or_default()
        };
        let holding = ["left", "right"]
            .into_iter()
            .map(|arm| (arm.to_owned(), held_by(&seen.state, arm)))
            .collect();
        World {
            places: self.places.all(),
            rooms: things(&seen.rooms, "rooms", "name", "/centroid"),
            objects: things(&seen.objects, "objects", "label", "/pose/position"),
            robot: seen.pose,
            holding,
        }
    }

    /// Compiles a plan, has the executor check it, and holds it for the operator's approval.
    pub(super) async fn plan(&self, args: Value, by: By) -> ToolOutcome {
        if by == By::Model
            && let Some(refusal) = self.may_plan()
        {
            return refusal;
        }
        // As sent, for the advisor: the plan below loses what did not parse.
        let sent = args.clone();
        let mut args = args;
        // A plan saved by name runs as its steps.
        if let Some(name) = args["template"].as_str().map(str::to_owned) {
            match self.template(&name).await {
                Ok(saved) => args = json!({"intent": saved.intent, "steps": saved.steps}),
                Err(e) => return ToolOutcome::refused(e),
            }
        }
        // The tool's own switches, not the plan's.
        if let Some(fields) = args.as_object_mut() {
            fields.remove("check_only");
            fields.remove("hash");
        }
        let mut plan: Plan = match serde_json::from_value(args) {
            Ok(p) => p,
            Err(e) => {
                return self
                    .rejected(
                        &json!([{"step": "", "field": "", "message": e.to_string()}]),
                        &sent,
                        by,
                    )
                    .await;
            }
        };
        // The local model leaves the label out, and once resent an unchanged plan until refused.
        if plan.intent.trim().is_empty() {
            plan.intent = self.intent_of(&plan);
        }
        let catalog = match self.catalog().await {
            Ok(c) => c,
            Err(e) => return ToolOutcome::failed(e),
        };
        let world = self.world_of(&self.observe().await);
        let mut compiled = match plan::compile(&plan, &catalog, &world, by) {
            Ok(c) => c,
            Err(problems) => return self.rejected(&json!(problems), &sent, by).await,
        };
        let reply = match self
            .robot
            .call(
                &self.config.validate,
                "nervros_interfaces/srv/ValidateMission",
                json!({"tree_xml": compiled.xml}),
                SERVICE_TIMEOUT,
            )
            .await
        {
            Ok(r) => r,
            Err(e) => {
                return ToolOutcome::failed(format!("the executor could not check the plan: {e}"));
            }
        };
        if reply["ok"].as_bool() != Some(true) {
            let diagnostics = reply["diagnostics_json"]
                .as_str()
                .and_then(|d| serde_json::from_str::<Value>(d).ok())
                .unwrap_or_else(|| json!([{"message": "the executor rejected the plan"}]));
            return self.rejected(&diagnostics, &sent, by).await;
        }
        if by == By::Model {
            self.plan_failures.store(0, Ordering::SeqCst);
        }
        // Not the model's fault, so not counted against its attempts.
        if let Some(why) = self.cannot_walk(&compiled, &catalog).await {
            return ToolOutcome::refused(why);
        }
        let (blocking, concerns) = self.concerns(&compiled, &catalog, by).await;
        if blocking && by == By::Model && !self.questioned.swap(true, Ordering::SeqCst) {
            return questioned(&concerns);
        }
        self.add_tracks(&mut compiled.steps).await;
        let worst = reply["worst_case_duration_s"]
            .as_f64()
            .filter(|w| *w > 0.0)
            .unwrap_or(compiled.worst_case_s);
        self.checked(compiled, worst, concerns)
    }

    /// Ways a plan may not do what the operator asked, and whether any is plain enough to send
    /// the plan back: the rules first, then the critic on what they let through.
    async fn concerns(
        &self,
        compiled: &Compiled,
        catalog: &Catalog,
        by: By,
    ) -> (bool, Vec<Concern>) {
        let request = self.request();
        let found = sanity::check(&request, &compiled.steps, &self.config.checks);
        if !found.is_empty() {
            return (true, found);
        }
        let none = (false, Vec::new());
        // The operator's own edit needs no second opinion.
        let Some(critic) = self
            .critic
            .get()
            .filter(|_| by == By::Model && !request.is_empty())
        else {
            return none;
        };
        let prompt = sanity::critic_prompt(&request, &compiled.steps, catalog);
        let reply = match tokio::time::timeout(CRITIC_TIMEOUT, critic.judge(&prompt)).await {
            Ok(Ok(reply)) => reply,
            Ok(Err(e)) => {
                tracing::info!(error = %e, "no critic for this plan");
                return none;
            }
            Err(_) => {
                tracing::info!("the critic took too long");
                return none;
            }
        };
        let whole = |message: String| Concern {
            step: String::new(),
            message: if message.is_empty() {
                "the plan checker has doubts".to_owned()
            } else {
                message
            },
            fix: None,
        };
        match sanity::verdict(&reply) {
            Some(Verdict::Reject(why)) => (true, vec![whole(why)]),
            Some(Verdict::Ask(why)) => (false, vec![whole(why)]),
            Some(Verdict::Ok) => none,
            None => {
                tracing::info!(%reply, "the critic gave no verdict");
                none
            }
        }
    }

    /// A plan that passed every check: shown to the window, kept by its hash, and described to
    /// the model with what went wrong before with its steps.
    fn checked(&self, compiled: Compiled, worst: f64, concerns: Vec<Concern>) -> ToolOutcome {
        let noted: Vec<String> = concerns.iter().map(concern_line).collect();
        self.emit(Event::MissionPlanned {
            hash: compiled.sha256.clone(),
            intent: compiled.plan.intent.clone(),
            steps: compiled.steps.clone(),
            worst_case_s: worst,
            concerns,
        });
        let mut out = json!({
            "hash": compiled.sha256,
            "steps": compiled.steps.iter().map(|s| format!("{} {}", s.id, s.summary)).collect::<Vec<_>>(),
            "worst_case_s": worst.round(),
            "next": "only checked: to run it, call run_mission with this hash; the operator approves it then"
        });
        let history: Vec<String> = compiled
            .steps
            .iter()
            .filter_map(|s| {
                let track = s.track.as_ref()?;
                let failed = track.runs - track.succeeded;
                (failed > 0).then(|| {
                    format!(
                        "{} {}: failed {failed} of its last {} runs, last because {}",
                        s.id,
                        s.summary,
                        track.runs,
                        track
                            .last_failure
                            .as_deref()
                            .unwrap_or("of something unknown")
                    )
                })
            })
            .collect();
        if !history.is_empty() {
            out["history"] = json!(history);
        }
        if !noted.is_empty() {
            out["concerns"] = json!(noted);
        }
        let mut planned = lock(&self.planned);
        // The same plan compiles to the same tree: keep one copy, or its hash reads as ambiguous.
        planned.retain(|c| c.sha256 != compiled.sha256);
        planned.push_back(compiled);
        if planned.len() > KEPT_PLANS {
            planned.pop_front();
        }
        ToolOutcome::ok(out)
    }

    /// A plan that failed its checks: the problems for the model, with the advisor's word once it keeps failing.
    async fn rejected(&self, problems: &Value, plan: &Value, by: By) -> ToolOutcome {
        let lines: Vec<String> = problems
            .as_array()
            .map_or(&[][..], Vec::as_slice)
            .iter()
            .filter_map(|p| {
                let message = p["message"].as_str()?;
                Some(match p["step"].as_str().filter(|s| !s.is_empty()) {
                    Some(step) => format!("{step}: {message}"),
                    None => message.to_owned(),
                })
            })
            .collect();
        if by == By::Operator {
            return ToolOutcome::failed(if lines.is_empty() {
                "the edited plan failed its checks".to_owned()
            } else {
                lines.join("; ")
            });
        }
        let n = self.plan_failures.fetch_add(1, Ordering::SeqCst) + 1;
        let count = problems.as_array().map_or(1, Vec::len);
        if n == MAX_PLAN_ATTEMPTS {
            // Planning gives up here: what was asked goes in the skill-gap log.
            let why: Vec<&str> = problems
                .as_array()
                .map_or(&[][..], Vec::as_slice)
                .iter()
                .filter_map(|p| p["message"].as_str())
                .take(3)
                .collect();
            self.note_gap(
                &format!("plans kept failing their checks: {}", why.join("; ")),
                "",
            );
        }
        // The problems first: what a reader of the first line most needs.
        let mut message = if lines.is_empty() {
            format!(
                "the plan has {count} problem(s); fix them all and call run_mission again \
                 (attempt {n} of {MAX_PLAN_ATTEMPTS})"
            )
        } else {
            format!(
                "{}. Fix all {count} problem(s) and call run_mission again (attempt {n} of \
                 {MAX_PLAN_ATTEMPTS})",
                lines.iter().take(2).cloned().collect::<Vec<_>>().join("; ")
            )
        };
        let mut data = json!({"ok": false, "problems": problems});
        // In a field of its own: appended to the message, the cut at 600 characters took its end.
        if n == advice::ADVISE_AFTER
            && let Some(advice) = self.advice(plan, problems).await
        {
            message.push_str(". A stronger model's advice is in `advice`: follow it");
            data["advice"] = Value::String(advice);
        }
        ToolOutcome {
            status: Status::Failed,
            data,
            message,
            images: Vec::new(),
        }
    }

    /// The advisor's word on a failed plan, when there is an advisor and it answers.
    async fn advice(&self, plan: &Value, problems: &Value) -> Option<String> {
        let advisor = self.advisor.get()?;
        let skills = self.catalog().await.ok()?.describe();
        let request = lock(&self.request).clone();
        let said = advisor
            .advise(&advice::prompt(&request, &skills, plan, problems))
            .await?;
        Some(crate::tools::clip(said.trim(), advice::ADVICE_CHARS))
    }

    /// The checked plan a hash, or a unique prefix of one, names.
    pub(crate) fn find(&self, hash: &str) -> Result<Compiled, String> {
        let hash = hash.trim().to_ascii_lowercase();
        let planned = lock(&self.planned);
        let hits: Vec<&Compiled> = planned
            .iter()
            .filter(|c| hash.len() >= MIN_HASH_PREFIX && c.sha256.starts_with(&hash))
            .collect();
        match hits.as_slice() {
            [one] => Ok((*one).clone()),
            [] => {
                Err("no checked plan has this hash; give run_mission the steps instead".to_owned())
            }
            _ => Err("the hash is ambiguous; use the full hash".to_owned()),
        }
    }

    /// Why the model may not plan for this request any more, or at all yet.
    fn may_plan(&self) -> Option<ToolOutcome> {
        if self.plan_failures.load(Ordering::SeqCst) >= MAX_PLAN_ATTEMPTS {
            return Some(ToolOutcome::refused(format!(
                "{MAX_PLAN_ATTEMPTS} plans in a row failed their checks; tell the operator what is missing instead"
            )));
        }
        let failed = self.run_failures.load(Ordering::SeqCst);
        if failed > self.config.max_replans {
            return Some(ToolOutcome::refused(format!(
                "this request already failed {failed} times; tell the operator what went wrong instead of retrying"
            )));
        }
        // A guess at what "it" is moves the wrong thing; asking costs a sentence.
        let unnamed = sanity::unnamed(&lock(&self.request));
        if let Some(word) = unnamed.filter(|_| self.said.load(Ordering::SeqCst) <= 1) {
            return Some(ToolOutcome::refused(format!(
                "the operator said \"{word}\" and nothing before it says what that is; ask \
                 them which thing they mean"
            )));
        }
        None
    }

    /// A name for a plan the model left unnamed: its first reason, else the operator's words.
    fn intent_of(&self, plan: &Plan) -> String {
        let asked = lock(&self.request)
            .trim()
            .trim_end_matches(['.', '!', '?'])
            .to_owned();
        plan.steps
            .iter()
            .map(|s| s.why.trim())
            .find(|w| !w.is_empty())
            .map(str::to_owned)
            .or_else(|| (!asked.is_empty()).then(|| crate::tools::clip(&asked, 60)))
            .unwrap_or_else(|| "the plan".to_owned())
    }

    /// The refusal for running the plan that just ran and failed, unchanged.
    pub(super) fn unchanged(&self, hash: &str) -> Option<ToolOutcome> {
        let last = lock(&self.last);
        let (_, how) = last.as_ref().filter(|(ran, _)| ran == hash)?;
        Some(ToolOutcome::refused(format!(
            "this exact plan just ran and {how}; running it unchanged would repeat that. Change \
             the plan to deal with what the report said, or tell the operator"
        )))
    }

    /// The plan with hash `hash` as the profile's rules read it: its steps as they will run.
    pub(crate) fn rule_view(&self, hash: &str) -> Option<Value> {
        let compiled = self.find(hash).ok()?;
        let steps: Vec<Value> = compiled.steps.iter().map(PlannedStep::for_rules).collect();
        Some(json!({"intent": compiled.plan.intent, "steps": steps}))
    }

    /// The plan saved as `name`, counted as used.
    async fn template(&self, name: &str) -> Result<ledger::Template, String> {
        let Some(ledger) = self.ledger.get().cloned() else {
            return Err("no plans are saved on this robot".to_owned());
        };
        let wanted = name.to_owned();
        tokio::task::spawn_blocking(move || ledger.use_template(&wanted))
            .await
            .map_err(|e| e.to_string())?
            .map_err(|e| e.to_string())?
            .ok_or_else(|| format!("no plan is saved as \"{name}\"; the plans tool lists them"))
    }

    /// Logs what the operator asked as something no skill could do.
    pub(crate) fn note_gap(&self, reason: &str, nearest: &str) {
        let Some(ledger) = self.ledger.get().cloned() else {
            return;
        };
        let (request, reason, nearest) = (self.request(), reason.to_owned(), nearest.to_owned());
        tokio::task::spawn_blocking(move || {
            if let Err(e) = ledger.note_gap(&request, &reason, &nearest) {
                tracing::warn!(error = %e, "the skill gap was not logged");
            }
        });
    }

    /// Each step's track record from the ledger, read off the async threads.
    async fn add_tracks(&self, steps: &mut [PlannedStep]) {
        let Some(ledger) = self.ledger.get().cloned() else {
            return;
        };
        let keys: Vec<(String, String)> = steps
            .iter()
            .map(|s| (s.skill.clone(), ledger::target_of(&s.args)))
            .collect();
        let tracks = tokio::task::spawn_blocking(move || {
            keys.iter()
                .map(|(skill, target)| ledger.track(skill, target).ok().flatten())
                .collect::<Vec<_>>()
        })
        .await
        .unwrap_or_default();
        for (step, track) in steps.iter_mut().zip(tracks) {
            step.track = track;
        }
    }

    /// The executor's state and the world model now, read together.
    pub(crate) async fn observe(&self) -> Observed {
        let world = self.profile.world.as_ref();
        // Read together: with the world model down, each waits its full time.
        let state = async {
            self.robot
                .latest(&self.config.state, STATE, WORLD_WAIT)
                .await
                .unwrap_or(Value::Null)
        };
        let (rooms, objects, state) = tokio::join!(
            self.latest(world.and_then(|w| w.rooms.as_ref())),
            self.latest(world.and_then(|w| w.objects.as_ref())),
            state
        );
        let (map, base) = (&self.profile.ros.map_frame, &self.profile.ros.base_frame);
        let pose = self
            .robot
            .transform(map, base)
            .ok()
            .map(|t| (t.translation[0], t.translation[1]));
        Observed {
            pose,
            places: self.places.all(),
            rooms,
            objects,
            state,
        }
    }
}
