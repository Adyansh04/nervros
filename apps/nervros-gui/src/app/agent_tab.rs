//! The Agent tab: earlier sessions, the operator's notes, and the models with their quotas.

use std::path::Path;
use std::time::{Duration, Instant, SystemTime};

use nervros_core::providers::Role;
use nervros_core::providers::router::Need;
use nervros_core::session::Command;
use rerun::external::egui;
use rerun::external::egui::{Align, Layout, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _};

use super::Gui;
use super::bars::quota;
use crate::toasts::Kind;

impl Gui {
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
            ui.label(RichText::new("Sessions").strong());
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
                    .color(ui.tokens().text_subdued),
            );
        }
        for s in listed {
            ui.horizontal(|ui| {
                ui.label(RichText::new(&s.when).monospace().small());
                ui.label(RichText::new(&s.first).small())
                    .on_hover_text(format!("{} messages", s.messages));
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
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
                });
            });
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
        ui.add_space(12.0);
        self.models_list(ui);
    }

    /// What the operator asked the agent to remember, each with a way to forget it.
    fn memory_list(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Memory").strong());
        let notes = self.agent.memory.all();
        if notes.is_empty() {
            ui.label(
                RichText::new("Nothing yet: say \"remember that…\" in the chat.")
                    .small()
                    .color(ui.tokens().text_subdued),
            );
        }
        let mut forget = None;
        for n in &notes {
            ui.horizontal(|ui| {
                ui.label(RichText::new(n.id.to_string()).monospace().small());
                ui.label(RichText::new(&n.text).small());
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui
                        .add(ReButton::new("Forget").small().secondary())
                        .clicked()
                    {
                        forget = Some(n.id);
                    }
                });
            });
        }
        if let Some(id) = forget
            && let Err(e) = self.agent.memory.forget(id)
        {
            self.chat.items.push(crate::chat::Item::error(e));
        }
    }

    fn models_list(&self, ui: &mut egui::Ui) {
        let router = self.agent.llm.router();
        let now = SystemTime::now();
        for (role, name) in [
            (Role::Routine, "Routine"),
            (Role::Plan, "Plan advice"),
            (Role::VisionCheck, "Vision check"),
            (Role::Summarise, "Summarise"),
            (Role::Segment, "Segment"),
            (Role::PlanCheck, "Plan check"),
        ] {
            ui.label(RichText::new(name).strong());
            let (take, skipped) = router.candidates(role, Need::default(), now);
            if take.is_empty() && skipped.is_empty() {
                ui.label(
                    RichText::new("off: models.toml lists no model for it")
                        .small()
                        .color(ui.tokens().text_subdued),
                );
            }
            for m in take {
                ui.horizontal(|ui| {
                    ui.bullet(ui.tokens().success_text_color);
                    ui.label(RichText::new(&m.id).monospace());
                    let q = quota(router, m, now);
                    ui.label(RichText::new(q).small().color(ui.tokens().text_subdued));
                });
            }
            for (id, why) in skipped {
                ui.horizontal(|ui| {
                    ui.bullet(ui.tokens().text_subdued);
                    ui.label(
                        RichText::new(id)
                            .monospace()
                            .color(ui.tokens().text_subdued),
                    );
                    ui.label(
                        RichText::new(format!("{why:?}"))
                            .small()
                            .color(ui.tokens().text_subdued),
                    );
                });
            }
            ui.add_space(8.0);
        }
    }
}
