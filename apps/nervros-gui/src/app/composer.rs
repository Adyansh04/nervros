//! The chat column: the conversation, what is picked in the viewer, and the message box.

use nervros_core::session::Command;
use rerun::external::egui;
use rerun::external::egui::{Frame, Key, Margin, Modifiers, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _, icons};
use serde_json::Value;

use super::{COMPOSER, Gui, Picked};
use crate::chat::Action;

impl Gui {
    pub(super) fn chat_panel(&mut self, ui: &mut egui::Ui) {
        let mut actions = Vec::new();
        egui::Panel::bottom("nervros_composer_panel")
            .frame(Frame::new().inner_margin(Margin::symmetric(0, 8)))
            .show(ui, |ui| {
                self.picked_bar(ui, &mut actions);
                self.composer(ui, &mut actions);
            });
        egui::ScrollArea::vertical()
            .stick_to_bottom(true)
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.add_space(8.0);
                self.chat
                    .show(ui, self.agent.profile.policy.approval_ttl, &mut actions);
            });
        self.act(ui.ctx(), actions);
    }

    /// What was clicked in the viewer: a walk there to approve at once, and messages about it
    /// filled in for the operator to read and send.
    fn picked_bar(&mut self, ui: &mut egui::Ui, actions: &mut Vec<Action>) {
        let Some(picked) = self.picked.borrow().clone() else {
            return;
        };
        let (what, go, prompts) = {
            let live = self.live();
            match &picked {
                Picked::Object(id) => {
                    let what = live
                        .object_names
                        .get(id)
                        .map_or_else(|| id.clone(), |n| format!("{id} ({n})"));
                    let prompts = vec![
                        ("What is it?", format!("Tell me about {what}.")),
                        ("Pick it up", format!("Pick up {what}.")),
                    ];
                    (what.clone(), walk_to_target(id, &what), prompts)
                }
                Picked::Room(id) => {
                    let name = live.rooms.as_ref().and_then(|m| {
                        m["rooms"]
                            .as_array()?
                            .iter()
                            .find(|r| r["id"] == id.as_str())
                            .and_then(|r| {
                                r["type"]
                                    .as_str()
                                    .filter(|t| !t.is_empty())
                                    .or_else(|| r["name"].as_str())
                                    .filter(|n| !n.is_empty())
                                    .map(str::to_owned)
                            })
                    });
                    let what = name.map_or_else(|| id.clone(), |n| format!("{id} ({n})"));
                    let prompts = vec![("What is in it?", format!("What objects are in {what}?"))];
                    (what.clone(), walk_to_target(id, &what), prompts)
                }
                Picked::Point([x, y]) => {
                    let what = format!("x {x:.2}, y {y:.2} on the map");
                    (what, walk_to_point(*x, *y, live.pose), Vec::new())
                }
            }
        };
        ui.horizontal(|ui| {
            ui.label(
                RichText::new(format!("Selected {what}"))
                    .small()
                    .color(ui.tokens().text_subdued),
            );
            if ui
                .small_button("Go there")
                .on_hover_text("Plan the walk now; you approve it before the robot moves")
                .clicked()
            {
                actions.push(Action::Send(Command::Run {
                    tool: "run_mission".to_owned(),
                    args: go,
                }));
            }
            for (label, text) in prompts {
                if ui.small_button(label).clicked() {
                    actions.push(Action::Prefill(text));
                }
            }
            if ui
                .small_button("✕")
                .on_hover_text("Clear the selection")
                .clicked()
            {
                self.picked.replace(None);
            }
        });
        ui.add_space(4.0);
    }

    fn composer(&mut self, ui: &mut egui::Ui, actions: &mut Vec<Action>) {
        let id = egui::Id::new(COMPOSER);
        let focused = ui.memory(|m| m.has_focus(id));
        let browsing = self.input.is_empty() || self.history_pos.is_some();
        let (enter, up, down) = ui.input_mut(|i| {
            if !focused {
                return (false, false, false);
            }
            // consume_key matches an extra Shift too: Shift+Enter must reach the field as a
            // new line, and Shift+arrows select.
            let plain = i.modifiers.is_none();
            let enter = plain && i.consume_key(Modifiers::NONE, Key::Enter);
            let up = plain && browsing && i.consume_key(Modifiers::NONE, Key::ArrowUp);
            let down = plain && browsing && i.consume_key(Modifiers::NONE, Key::ArrowDown);
            (enter, up, down)
        });
        if up {
            let pos = self
                .history_pos
                .unwrap_or(self.history.len())
                .saturating_sub(1);
            if let Some(h) = self.history.get(pos) {
                self.input.clone_from(h);
                self.history_pos = Some(pos);
            }
        }
        if down && let Some(pos) = self.history_pos {
            if let Some(h) = self.history.get(pos + 1) {
                self.input.clone_from(h);
                self.history_pos = Some(pos + 1);
            } else {
                self.input.clear();
                self.history_pos = None;
            }
        }
        let working = self.chat.turn.is_some();
        let sees = self.agent.tools.iter().any(|t| t == "look");
        if sees {
            self.attachments.show(ui);
        }
        ui.horizontal(|ui| {
            if sees
                && ui
                    .small_icon_button(&icons::ADD, "Paste an image")
                    .on_hover_text(
                        "Paste an image from the clipboard for the agent to look at; or drop \
                         one on the window",
                    )
                    .clicked()
            {
                self.attachments.paste(ui.ctx());
            }
            let hint = if working {
                "Say something while it works: it reads it at its next step · Esc stops it"
            } else {
                "Message the robot · Enter sends, Shift+Enter adds a line"
            };
            let edit = egui::TextEdit::multiline(&mut self.input)
                .id(id)
                .hint_text(hint)
                .desired_rows(2)
                .desired_width(ui.available_width() - 72.0)
                .margin(Margin::symmetric(8, 6));
            ui.add(edit);
            let has_text = !self.input.trim().is_empty();
            if working && !has_text {
                if ui
                    .add(ReButton::new("Stop").secondary())
                    .on_hover_text("Stops the reply (Esc)")
                    .clicked()
                {
                    actions.push(Action::Send(Command::StopGeneration));
                }
            } else {
                let send = ui.add_enabled(has_text, ReButton::new("Send").primary());
                if has_text && (enter || send.clicked()) {
                    actions.push(Action::Say(
                        std::mem::take(&mut self.input).trim().to_owned(),
                    ));
                }
            }
        });
    }
}

/// A plan that walks to a room or an object of the world model.
pub(super) fn walk_to_target(id: &str, what: &str) -> Value {
    serde_json::json!({
        "intent": format!("walk to {what}"),
        "steps": [{"skill": "GoToPlace", "args": [{"name": "place", "value": id}]}]
    })
}

/// A plan that walks to a point on the map, facing the way it walked from where it stands.
pub(super) fn walk_to_point(x: f32, y: f32, from: Option<crate::robot::Pose>) -> Value {
    let (x, y) = (f64::from(x), f64::from(y));
    let yaw = from.map_or(0.0, |(fx, fy, heading)| {
        if (x - fx).hypot(y - fy) < 0.05 {
            heading
        } else {
            (y - fy).atan2(x - fx)
        }
    });
    serde_json::json!({
        "intent": format!("walk to x {x:.2}, y {y:.2}"),
        "steps": [{"skill": "GoToPose", "args": [
            {"name": "station", "value": format!("{x:.2};{y:.2};{yaw:.3}")}
        ]}]
    })
}
