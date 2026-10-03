//! The bars above and below the chat: stop, arming, the model and its quota, the robot at a glance.

use std::time::{Duration, SystemTime};

use nervros_core::providers::router::{Need, Router};
use nervros_core::providers::{ModelConfig, Role};
use nervros_core::session::Command;
use rerun::external::egui::{Align, Color32, CornerRadius, Frame, Layout, Margin, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _};
use rerun::external::{eframe, egui, re_memory};

use super::{Gui, STOP_HINT, Tab};
use crate::theme;

impl Gui {
    pub(super) fn top_bar(&mut self, ui: &mut egui::Ui) {
        // With the viewer drawing the window's decorations, its hidden top bar took the window
        // buttons and the drag handle with it; ours has them instead.
        let window_chrome = self.viewer.app_options().custom_window_decorations;
        if window_chrome {
            let bar = ui.interact(ui.max_rect(), ui.id().with("drag"), egui::Sense::click());
            if bar.double_clicked() {
                let maximized = ui.input(|i| i.viewport().maximized.unwrap_or(false));
                ui.send_viewport_cmd(egui::ViewportCommand::Maximized(!maximized));
            } else if bar.is_pointer_button_down_on() {
                ui.send_viewport_cmd(egui::ViewportCommand::StartDrag);
            }
        }
        ui.horizontal_centered(|ui| {
            ui.spacing_mut().item_spacing.x = 8.0;
            ui.label(RichText::new("NervROS").strong().size(17.0));
            ui.label(RichText::new(&self.agent.profile.robot.name).color(theme::DIM));
            ui.add_space(6.0);
            self.status_chips(ui);
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if window_chrome {
                    ui.native_window_buttons_ui();
                    ui.add_space(4.0);
                }
                let stop = egui::Button::new(
                    RichText::new(self.stop_label())
                        .strong()
                        .color(Color32::WHITE),
                )
                .fill(theme::STOP)
                .corner_radius(CornerRadius::same(8))
                .min_size(egui::vec2(0.0, 30.0));
                if ui.add(stop).on_hover_text(STOP_HINT).clicked() {
                    self.agent.session.send(Command::StopMission);
                }
                ui.add_space(4.0);
                self.view_buttons(ui);
                ui.add_space(8.0);
                let mut armed = self.agent.guard.armed();
                let (label, colour) = if armed {
                    ("Armed", theme::WARN)
                } else {
                    ("Observe only", theme::DIM)
                };
                ui.label(RichText::new(label).strong().color(colour))
                    .on_hover_text(
                        "Armed, the agent may move the robot once you approve; observing, it \
                         only looks",
                    );
                if ui.toggle_switch(15.0, &mut armed).changed() {
                    self.agent
                        .session
                        .send(if armed { Command::Arm } else { Command::Disarm });
                }
            });
        });
    }

    /// The dock, the world editor and the viewer's layout and eye, right to left.
    fn view_buttons(&mut self, ui: &mut egui::Ui) {
        let toggle = ReButton::new("Dock")
            .small()
            .secondary()
            .selected(self.dock_open);
        if ui
            .add(toggle)
            .on_hover_text("Show or hide the panel on the right")
            .clicked()
        {
            self.dock_open = !self.dock_open;
        }
        if self.editor.is_some() {
            let edit = ReButton::new("Edit world")
                .small()
                .secondary()
                .selected(self.editing);
            if ui
                .add(edit)
                .on_hover_text("The saved world on its floor plan, to fix by hand")
                .clicked()
            {
                self.editing = !self.editing;
                if self.editing {
                    self.tab = Tab::World;
                    self.dock_open = true;
                }
            }
        }
        let following = self.bridge.following();
        let follow = ReButton::new("Follow robot")
            .small()
            .secondary()
            .selected(following);
        if ui
            .add(follow)
            .on_hover_text(
                "Keep the 3D view on the robot as it moves; off, it shows the whole map. \
                 Either puts the viewer's panes back as NervROS lays them out",
            )
            .clicked()
        {
            self.bridge.follow(!following);
        }
        let reset = ReButton::new("Reset layout").small().secondary();
        if ui
            .add(reset)
            .on_hover_text("Put the viewer's panes back the way NervROS lays them out")
            .clicked()
        {
            self.bridge.reset_layout();
        }
    }

    /// How the robot, the executor and the model are, and a way back to the live 3D view.
    fn status_chips(&mut self, ui: &mut egui::Ui) {
        let (topics, executor) = {
            let live = self.live();
            (live.topics, live.executor.is_some())
        };
        match topics {
            Some(n) if n > 2 => chip(ui, theme::SUCCESS, format!("ROS · {n} topics")),
            Some(_) => chip(ui, theme::ERROR, "ROS · empty graph"),
            None => chip(ui, theme::WARN, "ROS · connecting"),
        }
        if self.agent.profile.mission.is_some() {
            if executor {
                chip(ui, theme::SUCCESS, "Executor");
            } else {
                chip(ui, theme::WARN, "Executor offline");
            }
        }
        let (model, quota) = self.model_status();
        chip(ui, theme::INFO, format!("{model} · {quota}"));
        if let Some(at) = self.viewing {
            let secs = nervros_core::unix_secs(at);
            let back = ReButton::new(format!(
                "Viewing {} · back to live",
                crate::sessions::ago(secs)
            ))
            .small()
            .blue();
            if ui.add(back).clicked() {
                self.follow_live();
            }
        }
    }

    /// The model answering now, and its quota for today.
    fn model_status(&self) -> (String, String) {
        let router = self.agent.llm.router();
        let now = SystemTime::now();
        let need = Need {
            tools: true,
            ..Need::default()
        };
        let id = self.chat.model.clone().or_else(|| {
            let (take, _) = router.candidates(Role::Routine, need, now);
            take.first().map(|m| m.id.clone())
        });
        let Some(id) = id else {
            return ("no model".to_owned(), "none usable".to_owned());
        };
        let quota = router
            .config()
            .model(&id)
            .map(|m| quota(router, m, now))
            .unwrap_or_default();
        (id, quota)
    }

    pub(super) fn status_bar(&self, ui: &mut egui::Ui, frame: &eframe::Frame) {
        let item = |s: String| RichText::new(s).small().color(theme::DIM);
        ui.horizontal_centered(|ui| {
            ui.spacing_mut().item_spacing.x = 18.0;
            match self.chat.turn {
                Some(since) => {
                    let text = format!("Working {} s", since.elapsed().as_secs());
                    ui.label(RichText::new(text).small().color(theme::INFO));
                }
                None => {
                    ui.label(item("Idle".to_owned()));
                }
            }
            ui.label(item(format!("{} tools", self.agent.tools.len())));
            if let Some((used, window)) = self.chat.context {
                let text = format!(
                    "Context {} / {}",
                    crate::chat::thousands(used),
                    crate::chat::thousands(window)
                );
                // Past three quarters a compaction is near; past the window, a request fails.
                let full = used.saturating_mul(4) >= window.saturating_mul(3);
                let colour = if full { theme::WARN } else { theme::DIM };
                ui.label(RichText::new(text).small().color(colour))
                    .on_hover_text(
                        "Tokens the latest request took; /compact condenses the conversation",
                    );
            }
            let spent = self.chat.spent;
            if spent.calls > 0 {
                let cached = spent.cached_tokens * 100 / spent.input_tokens.max(1);
                ui.label(item(format!("{} calls · {cached}% cached", spent.calls)))
                    .on_hover_text(format!(
                        "Model calls this session: {} tokens in, {} of them read from the model's \
                         prompt cache, {} out, {:.0} s waiting",
                        crate::chat::thousands(spent.input_tokens),
                        crate::chat::thousands(spent.cached_tokens),
                        crate::chat::thousands(spent.output_tokens),
                        Duration::from_millis(spent.ms).as_secs_f64(),
                    ));
            }
            if let Some(bytes) = re_memory::MemoryUse::capture().counted {
                #[expect(clippy::cast_precision_loss, reason = "shown to one decimal")]
                let gb = bytes as f64 / 1e9;
                ui.label(item(format!("Memory {gb:.1} GB")));
            }
            if let Some(cpu) = frame.info().cpu_usage {
                ui.label(item(format!("UI {:.1} ms", cpu * 1000.0)));
            }
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                // The file's name tells sessions apart; the hover says where it is.
                let name = self.log_path.file_name().map_or_else(
                    || self.log_path.display().to_string(),
                    |n| n.to_string_lossy().into_owned(),
                );
                ui.label(item(format!("Log {name}")))
                    .on_hover_text(self.log_path.display().to_string());
            });
        });
    }
}

/// Today's use against the tightest daily limit: the shared pool's if the model is in one.
pub(super) fn quota(router: &Router, m: &ModelConfig, now: SystemTime) -> String {
    if m.privacy.local {
        return "local".to_owned();
    }
    if let Some(name) = m.limits.pool.as_deref()
        && let Some(pool) = router.config().pools.get(name)
    {
        return format!("{}/{} today", router.used_today(name, now), pool.rpd);
    }
    let used = router.used_today(&m.id, now);
    m.limits.rpd.map_or_else(
        || format!("{used} today"),
        |rpd| format!("{used}/{rpd} today"),
    )
}

fn chip(ui: &mut egui::Ui, color: Color32, text: impl Into<String>) {
    Frame::new()
        .fill(theme::SURFACE)
        .stroke(egui::Stroke::new(1.0, theme::BORDER))
        .corner_radius(CornerRadius::same(12))
        .inner_margin(Margin::symmetric(9, 3))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                let (rect, _) = ui.allocate_exact_size(egui::vec2(8.0, 8.0), egui::Sense::hover());
                ui.painter().circle_filled(rect.center(), 4.0, color);
                ui.label(RichText::new(text.into()).small());
            });
        });
}
