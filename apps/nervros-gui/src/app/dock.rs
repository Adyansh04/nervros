//! The dock's tabs: the mission, the robot, approvals, events and the health checks.

use nervros_core::session::Command;
use rerun::external::egui;
use rerun::external::egui::{Align, Layout, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _};

use super::{Gui, Tab};

impl Gui {
    pub(super) fn dock(&mut self, ui: &mut egui::Ui) {
        ui.add_space(8.0);
        // Wrapped, so the tabs never widen the dock past what the operator gave it.
        ui.horizontal_wrapped(|ui| {
            for (tab, name) in Tab::ALL {
                let count = if tab == Tab::Approvals {
                    self.chat.pending().count()
                } else {
                    0
                };
                let label = if count > 0 {
                    format!("{name} ({count})")
                } else {
                    name.to_owned()
                };
                if ui
                    .add(
                        ReButton::new(label)
                            .small()
                            .ghost()
                            .selected(self.tab == tab),
                    )
                    .clicked()
                {
                    self.tab = tab;
                }
            }
        });
        ui.full_span_separator();
        ui.add_space(8.0);
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| match self.tab {
                Tab::Mission => self.mission_tab(ui),
                Tab::World => self.world_tab(ui),
                Tab::Layers => self.layers_tab(ui),
                Tab::Approvals => self.approvals_tab(ui),
                Tab::Events => self.events_tab(ui),
                Tab::Agent => self.agent_tab(ui),
                Tab::Doctor => self.doctor_tab(ui),
                Tab::Robot => self.robot_tab(ui),
            });
    }

    fn mission_tab(&mut self, ui: &mut egui::Ui) {
        let mut actions = Vec::new();
        match self.chat.latest_plan() {
            Some(p) => {
                crate::chat::plan_card(ui, p, &mut actions);
                crate::chat::tree_panel(ui, p);
            }
            None => empty(
                ui,
                "No plan yet. Ask the robot to do something; its plan appears here.",
            ),
        }
        self.schedules_list(ui);
        let ledger = self.agent.missions.as_ref().and_then(|m| m.ledger());
        crate::history::show(ui, &mut self.records, ledger, &mut actions);
        self.act(ui.ctx(), actions);
    }

    /// Missions that run again and again, each with a way to end it.
    fn schedules_list(&self, ui: &mut egui::Ui) {
        let Some(schedules) = &self.agent.schedules else {
            return;
        };
        let list = schedules.list();
        if list.is_empty() {
            return;
        }
        ui.add_space(12.0);
        ui.label(RichText::new("Schedules").strong());
        let mut cancel = None;
        for s in &list {
            ui.horizontal(|ui| {
                ui.label(RichText::new(&s.id).monospace().small());
                ui.label(RichText::new(&s.intent).small());
                let when = if s.when.is_empty() {
                    format!(
                        "every {} min, {} of {} runs left",
                        s.every_min, s.left, s.times
                    )
                } else {
                    format!("when {}, {} of {} runs left", s.when, s.left, s.times)
                };
                ui.label(RichText::new(when).small().color(ui.tokens().text_subdued));
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui
                        .add(ReButton::new("Cancel").small().secondary())
                        .clicked()
                    {
                        cancel = Some(s.id.clone());
                    }
                });
            });
        }
        if let Some(id) = cancel {
            schedules.cancel(&id);
        }
    }

    fn robot_tab(&mut self, ui: &mut egui::Ui) {
        let status = {
            let l = self.live();
            crate::robot::Status {
                state: l.executor.clone(),
                pose: l.pose,
                rooms: l.rooms.clone(),
                names: l.object_names.clone(),
                battery: l.battery.clone(),
                motors: l.motors.clone(),
                armed: self.agent.guard.armed(),
            }
        };
        crate::robot::tab(ui, &status, self.drive.as_mut());
    }

    fn approvals_tab(&mut self, ui: &mut egui::Ui) {
        let pending: Vec<(u64, String, String)> = self
            .chat
            .pending()
            .map(|a| (a.id(), a.tool().to_owned(), a.reason().to_owned()))
            .collect();
        if pending.is_empty() {
            empty(
                ui,
                "Nothing to approve. Requests to act appear here and in the chat.",
            );
            return;
        }
        for (id, tool, reason) in pending {
            ui.horizontal(|ui| {
                ui.label(RichText::new(tool).monospace());
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.add(ReButton::new("Approve").small().primary()).clicked() {
                        self.agent.session.send(Command::Approve(id));
                    }
                    if ui.add(ReButton::new("Deny").small().secondary()).clicked() {
                        self.agent.session.send(Command::Deny(id));
                    }
                });
            });
            // What it would do, so the dock alone is enough to decide.
            ui.add(
                egui::Label::new(
                    RichText::new(reason)
                        .small()
                        .color(ui.tokens().text_subdued),
                )
                .wrap(),
            );
            ui.add_space(6.0);
        }
    }

    fn events_tab(&self, ui: &mut egui::Ui) {
        if self.event_log.is_empty() {
            empty(ui, "Session events appear here, newest first.");
        }
        for line in &self.event_log {
            ui.label(
                RichText::new(line)
                    .monospace()
                    .size(11.0)
                    .color(ui.tokens().text_subdued),
            );
        }
    }

    fn doctor_tab(&self, ui: &mut egui::Ui) {
        // What the MCP servers brought at start goes with what the robot answers now.
        let checks = self.live().checks.clone().map(|mut c| {
            c.extend(self.agent.mcp.iter().cloned());
            c
        });
        match checks {
            None => {
                ui.horizontal(|ui| {
                    crate::chat::spinner(ui);
                    ui.label("Checking the robot…");
                });
            }
            Some(checks) => {
                let bad = checks.iter().filter(|c| !c.ok).count();
                if bad == 0 {
                    ui.success_label("Everything the profile names is there.");
                } else {
                    ui.warning_label(format!("{bad} of {} checks failed.", checks.len()));
                }
                ui.add_space(8.0);
                for c in &checks {
                    ui.horizontal(|ui| {
                        let t = ui.tokens();
                        ui.bullet(if c.ok {
                            t.success_text_color
                        } else {
                            t.error_fg_color
                        });
                        ui.label(RichText::new(&c.what).size(12.0));
                    });
                }
                ui.add_space(8.0);
                if ui
                    .add(ReButton::new("Check again").small().secondary())
                    .clicked()
                {
                    self.live().checks = None;
                    let _ = self.recheck.send(());
                }
            }
        }
    }
}

pub(super) fn empty(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).color(ui.tokens().text_subdued));
}
