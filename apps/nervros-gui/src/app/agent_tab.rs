//! The Agent tab: earlier sessions, the operator's notes and goals, the fixes proposed after a
//! step failed again, and the models with their quotas.

use std::path::Path;
use std::time::{Duration, Instant, SystemTime};

use nervros_core::providers::Role;
use nervros_core::providers::router::Need;
use nervros_core::session::Command;
use rerun::external::egui;
use rerun::external::egui::{Align, Layout, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _};

use nervros_core::mission::inbox::{Proposal, State};
use nervros_core::mission::ledger::Track;
use nervros_core::session::Event;
use nervros_core::tools::Status;

use super::Gui;
use super::bars::quota;
use crate::theme;
use crate::toasts::Kind;

/// How often the accepted fixes' records are read again.
const FIX_RECORDS_FRESH: Duration = Duration::from_secs(5);

/// A fix whose call was sent as the operator's, waiting for it to finish.
struct Accepting {
    id: u32,
    tool: String,
    /// Its step's record when Accept was clicked.
    before: Option<Track>,
}

/// The proposed fixes as the window keeps them.
#[derive(Default)]
pub(super) struct Fixes {
    /// The one being accepted: accepted once its call succeeds.
    accepting: Option<Accepting>,
    /// When the records since were read.
    read: Option<Instant>,
    /// Each accepted fix's step record since, by the fix's number.
    since: std::collections::BTreeMap<u32, Option<Track>>,
}

/// A record as the operator reads it.
fn record(t: Option<&Track>) -> String {
    t.map_or_else(
        || "no runs".to_owned(),
        |t| format!("{} of {}", t.succeeded, t.runs),
    )
}

impl Gui {
    /// Settles the fix being accepted once its call ends: accepted when it succeeded, still open
    /// when the operator turned the call down or it failed.
    pub(super) fn settle_fix(&mut self, e: &Event) {
        let Event::ToolFinished {
            tool,
            status,
            message,
            ..
        } = e
        else {
            return;
        };
        let Some(asked) = self.fixes.accepting.take_if(|a| a.tool == *tool) else {
            return;
        };
        let Some(inbox) = self.agent.inbox.clone() else {
            return;
        };
        if *status != Status::Succeeded {
            self.toasts.add(
                Kind::Failure,
                format!("Fix {} did not run, so it stays open: {message}", asked.id),
            );
            return;
        }
        match inbox.accept(asked.id, asked.before) {
            Ok(_) => {
                self.fixes.read = None;
                self.toasts
                    .add(Kind::Success, format!("Fix {} accepted", asked.id));
            }
            Err(e) => self.chat.items.push(crate::chat::Item::error(e)),
        }
    }

    /// Saves a session as an eval case, and says where.
    fn save_as_case(&mut self, log: &Path) {
        match crate::sessions::save_as_case(log) {
            Ok((id, suite)) => self.toasts.add(
                Kind::Success,
                format!("Saved as test case {id} in {}", suite.display()),
            ),
            Err(e) => self.toasts.add(Kind::Failure, format!("Not saved: {e}")),
        }
    }

    /// Earlier sessions to carry on or keep as tests, then memory, then the models and quotas.
    pub(super) fn agent_tab(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            theme::section(ui, "Sessions");
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui
                    .add(ReButton::new("Save this one as a test").small().secondary())
                    .on_hover_text(
                        "Keep this conversation as an eval case, expecting what happened; run \
                         it with nervros-cli eval",
                    )
                    .clicked()
                {
                    let log = self.log_path.clone();
                    self.save_as_case(&log);
                }
            });
        });
        let fresh = self
            .sessions
            .as_ref()
            .is_some_and(|(at, _)| at.elapsed() < Duration::from_secs(5));
        if !fresh {
            let logs = self
                .log_path
                .parent()
                .map(Path::to_path_buf)
                .unwrap_or_default();
            self.sessions = Some((Instant::now(), self.listing.list(&logs, &self.log_path)));
        }
        let (mut resume, mut keep) = (None, None);
        let listed = self
            .sessions
            .as_ref()
            .map_or(&[][..], |(_, l)| l.as_slice());
        if listed.is_empty() {
            ui.label(
                RichText::new("No earlier session saved its conversation yet.")
                    .small()
                    .color(theme::DIM),
            );
        }
        for s in listed {
            theme::row(
                ui,
                |ui| {
                    if ui
                        .add(ReButton::new("Resume").small().secondary())
                        .on_hover_text("Carry on this conversation")
                        .clicked()
                    {
                        resume = Some(s.path.clone());
                    }
                    if ui
                        .add(ReButton::new("Save as test").small().secondary())
                        .on_hover_text("Keep it as an eval case, expecting what happened")
                        .clicked()
                    {
                        keep = Some(crate::sessions::log_of(&s.path));
                    }
                },
                |ui| {
                    ui.label(RichText::new(&s.when).monospace().small());
                    ui.label(RichText::new(&s.first).size(13.0))
                        .on_hover_text(format!("{} ({} messages)", s.first, s.messages));
                },
            );
        }
        if let Some(log) = keep {
            self.save_as_case(&log);
        }
        if let Some(path) = resume {
            match nervros_core::llm::History::load(&path) {
                Ok(history) => self.agent.session.send(Command::Restore(history)),
                Err(e) => self
                    .chat
                    .items
                    .push(crate::chat::Item::error(format!("{}: {e}", path.display()))),
            }
        }
        ui.add_space(12.0);
        self.memory_list(ui);
        self.goals_list(ui);
        self.inbox_list(ui);
        ui.add_space(12.0);
        self.models_list(ui);
    }

    /// The goals and where each stands, each with a way to drop it.
    fn goals_list(&mut self, ui: &mut egui::Ui) {
        let Some(goals) = self.agent.missions.as_ref().and_then(|m| m.goals()) else {
            return;
        };
        let all = goals.all();
        if all.is_empty() {
            return;
        }
        ui.add_space(16.0);
        theme::section(ui, "Goals");
        let mut drop = None;
        for g in all.iter().rev() {
            theme::row(
                ui,
                |ui| {
                    if ui.add(ReButton::new("Drop").small().secondary()).clicked() {
                        drop = Some(g.id);
                    }
                },
                |ui| {
                    ui.label(
                        RichText::new(g.id.to_string())
                            .monospace()
                            .color(theme::FAINT),
                    );
                    let line = g.line();
                    ui.label(RichText::new(&line).size(13.0))
                        .on_hover_text(&line);
                },
            );
        }
        if let Some(id) = drop
            && let Err(e) = goals.drop_goal(id)
        {
            self.chat.items.push(crate::chat::Item::error(e));
        }
    }

    /// The proposed fixes: an open one to accept, which runs its call as the operator's, or to
    /// dismiss; an accepted one with its step's record before and since.
    fn inbox_list(&mut self, ui: &mut egui::Ui) {
        let Some(inbox) = self.agent.inbox.clone() else {
            return;
        };
        let proposals = inbox.all();
        if proposals.is_empty() {
            return;
        }
        ui.add_space(16.0);
        theme::section(ui, "Proposed fixes");
        let ledger = self
            .agent
            .missions
            .as_ref()
            .and_then(|m| m.ledger())
            .cloned();
        self.refresh_fix_records(&proposals, ledger.as_deref());
        let waiting = self.fixes.accepting.as_ref().map(|a| a.id);
        let (mut accept, mut dismiss) = (None, None);
        for p in proposals.iter().rev() {
            let open = p.state == State::Open;
            theme::row(
                ui,
                |ui| {
                    if waiting == Some(p.id) {
                        ui.label(RichText::new("Running…").small().color(theme::DIM));
                    } else if open && waiting.is_none() {
                        if ui
                            .add(ReButton::new("Dismiss").small().secondary())
                            .on_hover_text("No fix like it is proposed for this step again")
                            .clicked()
                        {
                            dismiss = Some(p.id);
                        }
                        if ui
                            .add(ReButton::new("Accept").small())
                            .on_hover_text(format!(
                                "Runs {} {} as yours, through its own approval",
                                p.tool, p.args
                            ))
                            .clicked()
                        {
                            accept = Some(p.id);
                        }
                    }
                },
                |ui| {
                    ui.label(
                        RichText::new(p.id.to_string())
                            .monospace()
                            .color(theme::FAINT),
                    );
                    let fix = p.summary();
                    ui.label(RichText::new(&fix).size(13.0))
                        .on_hover_text(format!("{fix}\n{} {}", p.tool, p.args));
                },
            );
            // Why, beside what failed: the reason to accept or dismiss it.
            ui.label(
                RichText::new(format!(
                    "{} on {} failed: {}. {}",
                    p.skill, p.target, p.failure, p.why
                ))
                .small()
                .color(theme::DIM),
            );
            if p.state == State::Accepted {
                let since = self.fixes.since.get(&p.id).and_then(Option::as_ref);
                ui.label(
                    RichText::new(format!(
                        "accepted; before {}, since {}",
                        record(p.before.as_ref()),
                        record(since)
                    ))
                    .small()
                    .color(theme::DIM),
                );
            }
        }
        if let Some(id) = accept {
            self.accept_fix(&inbox, id, ledger.as_deref());
        }
        if let Some(id) = dismiss
            && let Err(e) = inbox.dismiss(id)
        {
            self.chat.items.push(crate::chat::Item::error(e));
        }
    }

    /// Runs fix `id`'s call as the operator's, through its own approval; it is accepted once that
    /// succeeds.
    fn accept_fix(
        &mut self,
        inbox: &nervros_core::mission::inbox::Inbox,
        id: u32,
        ledger: Option<&nervros_core::mission::ledger::Ledger>,
    ) {
        // Checked again: the file is read as trusted, and a hand edit could change the call.
        match inbox.check(id) {
            Ok(p) => {
                let before = ledger.and_then(|l| l.track(&p.skill, &p.target).ok().flatten());
                self.fixes.accepting = Some(Accepting {
                    id,
                    tool: p.tool.clone(),
                    before,
                });
                self.agent.session.send(Command::Run {
                    tool: p.tool,
                    args: p.args,
                });
            }
            Err(e) => self.chat.items.push(crate::chat::Item::error(e)),
        }
    }

    /// Reads the accepted fixes' records since again, off the hot path at most every few seconds.
    fn refresh_fix_records(
        &mut self,
        proposals: &[Proposal],
        ledger: Option<&nervros_core::mission::ledger::Ledger>,
    ) {
        if self
            .fixes
            .read
            .is_some_and(|at| at.elapsed() < FIX_RECORDS_FRESH)
        {
            return;
        }
        self.fixes.read = Some(Instant::now());
        let Some(ledger) = ledger else {
            return;
        };
        self.fixes.since = proposals
            .iter()
            .filter_map(|p| {
                #[expect(clippy::cast_precision_loss, reason = "seconds since 1970")]
                let at = p.accepted_at? as f64;
                Some((
                    p.id,
                    ledger.track_since(&p.skill, &p.target, at).ok().flatten(),
                ))
            })
            .collect();
    }

    /// What the operator asked the agent to remember, each with a way to forget it.
    fn memory_list(&mut self, ui: &mut egui::Ui) {
        ui.add_space(16.0);
        theme::section(ui, "Memory");
        let notes = self.agent.memory.all();
        if notes.is_empty() {
            ui.label(
                RichText::new("Nothing yet: say \"remember that…\" in the chat.")
                    .small()
                    .color(theme::DIM),
            );
        }
        let mut forget = None;
        for n in &notes {
            theme::row(
                ui,
                |ui| {
                    if ui
                        .add(ReButton::new("Forget").small().secondary())
                        .clicked()
                    {
                        forget = Some(n.id);
                    }
                },
                |ui| {
                    ui.label(
                        RichText::new(n.id.to_string())
                            .monospace()
                            .color(theme::FAINT),
                    );
                    ui.label(RichText::new(&n.text).size(13.0))
                        .on_hover_text(&n.text);
                },
            );
        }
        if let Some(id) = forget
            && let Err(e) = self.agent.memory.forget(id)
        {
            self.chat.items.push(crate::chat::Item::error(e));
        }
    }

    fn models_list(&self, ui: &mut egui::Ui) {
        ui.add_space(16.0);
        theme::section(ui, "Models");
        let router = self.agent.llm.router();
        let now = SystemTime::now();
        for (role, name) in [
            (Role::Routine, "Routine"),
            (Role::Plan, "Plan advice"),
            (Role::VisionCheck, "Vision check"),
            (Role::Summarise, "Summarise"),
            (Role::Segment, "Segment"),
            (Role::PlanCheck, "Plan check"),
            (Role::Witness, "Witness"),
            (Role::Moment, "Moment"),
            (Role::Watch, "Watch"),
            (Role::SceneCheck, "Scene check"),
        ] {
            ui.label(RichText::new(name).strong());
            let (take, skipped) = router.candidates(role, Need::default(), now);
            if take.is_empty() && skipped.is_empty() {
                ui.label(
                    RichText::new("off: models.toml lists no model for it")
                        .small()
                        .color(theme::DIM),
                );
            }
            for m in take {
                ui.horizontal(|ui| {
                    ui.bullet(theme::SUCCESS);
                    ui.label(RichText::new(&m.id).monospace());
                    let q = quota(router, m, now);
                    ui.label(RichText::new(q).small().color(theme::DIM));
                });
            }
            for (id, why) in skipped {
                ui.horizontal(|ui| {
                    ui.bullet(theme::DIM);
                    ui.label(RichText::new(id).monospace().color(theme::DIM));
                    ui.label(RichText::new(format!("{why:?}")).small().color(theme::DIM));
                });
            }
            ui.add_space(8.0);
        }
    }
}
