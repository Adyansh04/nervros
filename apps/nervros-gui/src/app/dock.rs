//! The dock's tabs: the mission, the robot, approvals, events and the health checks.

use nervros_core::session::Command;
use rerun::external::egui;
use rerun::external::egui::{Align, CornerRadius, Frame, Layout, Margin, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _, icons};

use super::{Gui, Tab};
use crate::theme;

impl Gui {
    pub(super) fn dock(&mut self, ui: &mut egui::Ui) {
        ui.add_space(12.0);
        self.tab_bar(ui);
        ui.add_space(14.0);
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                match self.tab {
                    Tab::Mission => self.mission_tab(ui),
                    Tab::World => self.world_tab(ui),
                    Tab::Layers => self.layers_tab(ui),
                    Tab::Approvals => self.approvals_tab(ui),
                    Tab::Events => self.events_tab(ui),
                    Tab::Agent => self.agent_tab(ui),
                    Tab::Doctor => self.doctor_tab(ui),
                    Tab::Robot => self.robot_tab(ui),
                }
                ui.add_space(12.0);
            });
    }

    /// The tabs as two rows of four equal segments: eight names never fit one row of a dock,
    /// and a row that wraps wherever the width runs out reads as a mistake.
    fn tab_bar(&mut self, ui: &mut egui::Ui) {
        let pending = self.chat.pending().count();
        Frame::new()
            .fill(theme::SURFACE)
            .stroke(egui::Stroke::new(1.0, theme::BORDER))
            .corner_radius(CornerRadius::same(10))
            .inner_margin(Margin::same(3))
            .show(ui, |ui| {
                ui.spacing_mut().item_spacing = egui::vec2(3.0, 3.0);
                ui.spacing_mut().button_padding = egui::vec2(4.0, 4.0);
                let width = ((ui.available_width() - 9.0) / 4.0).floor();
                for row in Tab::ALL.chunks(4) {
                    ui.horizontal(|ui| {
                        for &(tab, name) in row {
                            let selected = self.tab == tab;
                            // Amber while a request waits, so it shows from any tab.
                            let waiting = tab == Tab::Approvals && pending > 0;
                            let colour = match (waiting, selected) {
                                (true, _) => theme::WARN,
                                (false, true) => theme::TEXT,
                                (false, false) => theme::DIM,
                            };
                            let label = RichText::new(name).size(13.0).color(colour);
                            let button = egui::Button::new(label)
                                .fill(if selected {
                                    theme::RAISED
                                } else {
                                    egui::Color32::TRANSPARENT
                                })
                                .stroke(egui::Stroke::NONE)
                                .corner_radius(CornerRadius::same(7))
                                .min_size(egui::vec2(width, 28.0));
                            let response = ui.add(button);
                            let response = if waiting {
                                response.on_hover_text(format!("{pending} waiting for you"))
                            } else {
                                response
                            };
                            if response.clicked() {
                                self.tab = tab;
                            }
                        }
                    });
                }
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
        ui.add_space(16.0);
        theme::section(ui, "Schedules");
        let mut cancel = None;
        for s in &list {
            ui.horizontal(|ui| {
                ui.label(RichText::new(&s.id).monospace().color(theme::FAINT));
                ui.label(&s.intent);
                let when = if s.when.is_empty() {
                    format!(
                        "every {} min, {} of {} runs left",
                        s.every_min, s.left, s.times
                    )
                } else {
                    format!("when {}, {} of {} runs left", s.when, s.left, s.times)
                };
                ui.label(RichText::new(when).small().color(theme::DIM));
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
            theme::status_card(ui, theme::WARN, |ui| {
                ui.set_width(ui.available_width());
                ui.label(RichText::new(tool).monospace().strong());
                // What it would do, so the dock alone is enough to decide.
                ui.label(RichText::new(reason).color(theme::DIM));
                ui.horizontal(|ui| {
                    if ui.add(ReButton::new("Approve").small().blue()).clicked() {
                        self.agent.session.send(Command::Approve(id));
                    }
                    if ui.add(ReButton::new("Deny").small().secondary()).clicked() {
                        self.agent.session.send(Command::Deny(id));
                    }
                });
            });
            ui.add_space(8.0);
        }
    }

    fn events_tab(&self, ui: &mut egui::Ui) {
        if self.event_log.is_empty() {
            empty(ui, "Session events appear here, newest first.");
        }
        for line in &self.event_log {
            ui.label(RichText::new(line).monospace().size(11.5).color(theme::DIM));
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
                let (colour, head) = if bad == 0 {
                    (
                        theme::SUCCESS,
                        "Everything the profile names is there".to_owned(),
                    )
                } else {
                    (
                        theme::WARN,
                        format!("{bad} of {} checks failed", checks.len()),
                    )
                };
                theme::status_card(ui, colour, |ui| {
                    ui.set_width(ui.available_width());
                    ui.label(RichText::new(head).strong());
                });
                ui.add_space(10.0);
                for c in &checks {
                    ui.horizontal(|ui| {
                        let (icon, colour) = if c.ok {
                            (&icons::SUCCESS, theme::SUCCESS)
                        } else {
                            (&icons::ERROR, theme::ERROR)
                        };
                        ui.small_icon(icon, Some(colour));
                        ui.label(&c.what);
                    });
                }
                ui.add_space(10.0);
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
    ui.horizontal(|ui| {
        ui.small_icon(&icons::INFO, Some(theme::FAINT));
        ui.label(RichText::new(text).color(theme::DIM));
    });
}
