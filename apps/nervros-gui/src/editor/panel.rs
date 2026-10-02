//! The side panel: the selected room or object, its fields and actions, and the review queue.

use rerun::external::egui;
use rerun::external::egui::{Color32, RichText, Vec2};
use rerun::external::re_ui::{ReButton, UiExt as _};
use serde_json::json;

use super::world::{CHECKED, OBJECT, Object, REMOVED, Room, SUSPECT, World};
use super::{Call, Form, Mode, Picked, WorldEditor, lock};
use crate::theme;

impl WorldEditor {
    /// The dock's side of the editor: the toolbar, the selection, the review and removed lists.
    pub(crate) fn panel(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        let Some(world) = self.world() else {
            ui.label(RichText::new("No world from the editor yet.").color(theme::DIM));
            return;
        };
        let (busy, status) = {
            let s = lock(&self.shared);
            (s.busy, s.status.clone())
        };
        ui.horizontal_wrapped(|ui| {
            ui.add_enabled_ui(!busy, |ui| {
                if ui
                    .add_enabled(world.can_undo, ReButton::new("Undo").small())
                    .clicked()
                {
                    self.call(&ctx, Call::Command("undo", "undone"));
                }
                if ui
                    .add_enabled(world.can_redo, ReButton::new("Redo").small())
                    .clicked()
                {
                    self.call(&ctx, Call::Command("redo", "redone"));
                }
                if ui.add(ReButton::new("Add object").small()).clicked() {
                    self.start_mode(Mode::Add, &world);
                }
                let save = if world.unsaved > 0 {
                    format!("Save ({})", world.unsaved)
                } else {
                    "Save".to_owned()
                };
                if ui
                    .add_enabled(world.unsaved > 0, ReButton::new(save).small().primary())
                    .clicked()
                {
                    self.call(&ctx, Call::Save);
                }
                if ui
                    .add(ReButton::new("Reload").small().ghost())
                    .on_hover_text("Read the saved world again, dropping edits not saved")
                    .clicked()
                {
                    self.call(
                        &ctx,
                        Call::Command("reload", "read the world from disk again"),
                    );
                }
            });
        });
        if let Some((text, error)) = status {
            let colour = if error { REMOVED } else { theme::DIM };
            ui.label(RichText::new(text).small().color(colour));
        }
        ui.add_space(6.0);
        ui.full_span_separator();
        ui.add_space(6.0);
        self.card(ui, &world);
        ui.add_space(8.0);
        self.review(ui, &world);
    }

    fn card(&mut self, ui: &mut egui::Ui, world: &World) {
        let ctx = ui.ctx().clone();
        if self.mode == Mode::Add
            && let Some(drawn) = self.drawn
        {
            ui.label(RichText::new("New object").strong());
            ui.label(
                RichText::new("Saved as one the operator saw: canopy keeps its box.")
                    .small()
                    .color(theme::DIM),
            );
            text_field(ui, "What it is", &mut self.form.label, &world.labels);
            text_field(ui, "Name", &mut self.form.name, &[]);
            ui.horizontal(|ui| {
                ui.label("From");
                ui.add(
                    egui::DragValue::new(&mut self.form.low)
                        .speed(0.01)
                        .range(0.0..=3.0)
                        .suffix(" m"),
                );
                ui.label("to");
                ui.add(
                    egui::DragValue::new(&mut self.form.high)
                        .speed(0.01)
                        .range(0.0..=3.0)
                        .suffix(" m"),
                );
                ui.label("high");
            });
            ui.horizontal(|ui| {
                let ok = !self.form.label.trim().is_empty() && self.form.high > self.form.low;
                if ui.add_enabled(ok, ReButton::new("Add").small().primary()).clicked() {
                    let op = json!({"op": "add", "label": self.form.label, "name": self.form.name, "centre": drawn.centre,
                        "size": drawn.size, "yaw": drawn.yaw, "z_min": self.form.low, "z_max": self.form.high});
                    self.edit(&ctx, op, format!("added a {}", self.form.label.trim()));
                }
                if ui.add(ReButton::new("Cancel").small().ghost()).clicked() {
                    self.stop_mode();
                }
            });
            return;
        }
        if self.mode == Mode::Split
            && let (Some(drawn), Some(object)) = (self.drawn, self.selected_object(world))
        {
            let id = object.id;
            ui.label(RichText::new(format!("Split off part of O{id}")).strong());
            ui.label(
                RichText::new("The voxels inside the drawn box become a new object.")
                    .small()
                    .color(theme::DIM),
            );
            text_field(ui, "What the part is", &mut self.form.label, &world.labels);
            text_field(ui, "Name", &mut self.form.name, &[]);
            ui.horizontal(|ui| {
                if ui.add_enabled(!self.form.label.trim().is_empty(), ReButton::new("Split").small().primary()).clicked() {
                    let op = json!({"op": "split", "id": id, "polygon": drawn.corners(), "label": self.form.label, "name": self.form.name});
                    self.edit(&ctx, op, format!("split a {} off O{id}", self.form.label.trim()));
                }
                if ui.add(ReButton::new("Cancel").small().ghost()).clicked() {
                    self.stop_mode();
                }
            });
            return;
        }
        match self.selected.clone() {
            Some(Picked::Object(id)) => match world.objects.iter().find(|o| o.id == id) {
                Some(object) => self.object_card(ui, world, object),
                None => self.selected = None,
            },
            Some(Picked::Room(id)) => match world.rooms.iter().find(|r| r.id == id) {
                Some(room) => self.room_card(ui, world, room),
                None => self.selected = None,
            },
            None => {
                ui.label(
                    RichText::new(
                        "Click an object or a room on the map. Drag to pan, scroll to zoom.",
                    )
                    .color(theme::DIM),
                );
            }
        }
    }

    fn object_card(&mut self, ui: &mut egui::Ui, world: &World, object: &Object) {
        let ctx = ui.ctx().clone();
        let (id, picked) = (object.id, Picked::Object(object.id));
        if self.form.of.as_ref() != Some(&picked) {
            self.form = Form {
                label: object.label.clone(),
                name: object.name.clone(),
                of: Some(picked.clone()),
                ..Form::default()
            };
        }
        ui.horizontal_wrapped(|ui| {
            let title = RichText::new(format!("O{id} {}", object.label));
            ui.label(title.strong().size(15.0));
            if object.checked {
                badge(ui, "checked", CHECKED);
            }
            if object.box_pinned {
                badge(ui, "box set by hand", OBJECT);
            }
            if object.state != "active" {
                badge(ui, &object.state, REMOVED);
            }
        });
        if object.crop
            && let Some(texture) = self.crop(&ctx, id)
        {
            let size = texture.size_vec2();
            let width = size.x.min(ui.available_width()).min(300.0);
            let fit = Vec2::new(width, size.y * width / size.x.max(1.0));
            ui.add(
                egui::Image::new(&texture)
                    .fit_to_exact_size(fit)
                    .corner_radius(4.0),
            );
        }
        let reasons = Self::suspicion(world, &picked).map_or(&[][..], |s| s.reasons.as_slice());
        for reason in reasons {
            ui.label(RichText::new(format!("• {reason}")).small().color(SUSPECT));
        }
        if !object.removed_by.is_empty() {
            let why = RichText::new(format!("• {}", object.removed_by));
            ui.label(why.small().color(REMOVED));
        }
        ui.add_space(4.0);
        self.object_fields(ui, world, object);
        ui.add_space(6.0);
        if !self.object_confirm(ui, world, object) {
            self.object_actions(ui, world, object);
        }
    }

    /// The label and name, with the facts behind them.
    fn object_fields(&mut self, ui: &mut egui::Ui, world: &World, object: &Object) {
        let (ctx, id) = (ui.ctx().clone(), object.id);
        text_field(ui, "What it is", &mut self.form.label, &world.labels);
        ui.horizontal(|ui| {
            if ui.add(ReButton::new("Set label").small()).clicked() {
                let label = self.form.label.trim().to_owned();
                let op = json!({"op": "label", "id": id, "label": label});
                self.edit(&ctx, op, format!("O{id} is now {label}"));
            }
            let votes = ReButton::new("Use the votes").small().ghost();
            if !object.operator_label.is_empty() && ui.add(votes).clicked() {
                let op = json!({"op": "label", "id": id, "label": ""});
                self.edit(&ctx, op, format!("O{id} goes back to the detector's label"));
            }
        });
        text_field(ui, "Name", &mut self.form.name, &[]);
        if ui.add(ReButton::new("Set name").small()).clicked() {
            let op = json!({"op": "name", "id": id, "name": self.form.name.trim()});
            self.edit(&ctx, op, format!("O{id} named"));
        }
        ui.add_space(4.0);
        let mut votes: Vec<(&String, f64)> = object
            .votes
            .iter()
            .map(|(k, v)| (k, v.as_f64().unwrap_or(0.0)))
            .collect();
        votes.sort_by(|a, b| b.1.total_cmp(&a.1));
        let votes: Vec<String> = votes
            .iter()
            .take(4)
            .map(|(k, v)| format!("{k} {v:.1}"))
            .collect();
        let or = |text: String, none: &str| {
            if text.is_empty() {
                none.to_owned()
            } else {
                text
            }
        };
        let facts = [
            ("votes", or(votes.join(" · "), "none")),
            ("describer", or(object.caption.clone(), "no caption")),
            (
                "sightings",
                format!("{}, vote weight {:.1}", object.observations, object.weight),
            ),
            (
                "size",
                format!(
                    "{:.2} × {:.2} m, {:.2}–{:.2} m high",
                    object.size[0], object.size[1], object.z_min, object.z_max
                ),
            ),
            ("voxels", object.voxels.to_string()),
        ];
        egui::Grid::new(("facts", id))
            .num_columns(2)
            .spacing([10.0, 2.0])
            .show(ui, |ui| {
                let dim = theme::DIM;
                for (k, v) in facts {
                    ui.label(RichText::new(k).small().color(dim));
                    ui.label(RichText::new(v).small());
                    ui.end_row();
                }
            });
    }

    /// A merge or a delete waiting for the operator's word; `false` when there is none.
    fn object_confirm(&mut self, ui: &mut egui::Ui, world: &World, object: &Object) -> bool {
        let (ctx, id) = (ui.ctx().clone(), object.id);
        if let Some(other) = self.merge_with {
            let label = world
                .objects
                .iter()
                .find(|o| o.id == other)
                .map_or("?", |o| o.label.as_str());
            ui.label(format!("Merge O{other} ({label}) into O{id}?"));
            ui.horizontal(|ui| {
                if ui.add(ReButton::new("Merge").small().primary()).clicked() {
                    self.stop_mode();
                    let op = json!({"op": "merge", "into": id, "ids": [id, other]});
                    self.edit(&ctx, op, format!("O{other} merged into O{id}"));
                }
                if ui.add(ReButton::new("Cancel").small().ghost()).clicked() {
                    self.stop_mode();
                }
            });
            return true;
        }
        if !self.confirm_delete {
            return false;
        }
        ui.label(format!(
            "Delete O{id} ({}) for good? Remove keeps it, restorable.",
            object.label
        ));
        ui.horizontal(|ui| {
            if ui.add(ReButton::new("Delete").small().primary()).clicked() {
                self.confirm_delete = false;
                self.edit(
                    &ctx,
                    json!({"op": "delete", "id": id}),
                    format!("O{id} deleted"),
                );
            }
            if ui.add(ReButton::new("Remove instead").small()).clicked() {
                self.confirm_delete = false;
                let op = json!({"op": "remove", "ids": [id], "reason": "removed by hand"});
                self.edit(&ctx, op, format!("O{id} removed"));
            }
            if ui.add(ReButton::new("Cancel").small().ghost()).clicked() {
                self.confirm_delete = false;
            }
        });
        true
    }

    fn object_actions(&mut self, ui: &mut egui::Ui, world: &World, object: &Object) {
        let (ctx, id) = (ui.ctx().clone(), object.id);
        ui.horizontal_wrapped(|ui| {
            let restore = ReButton::new("Restore").small().primary();
            if object.state == "removed" && ui.add(restore).clicked() {
                let op = json!({"op": "restore", "ids": [id]});
                self.edit(&ctx, op, format!("O{id} restored, and checked"));
            }
            let (check, done) = if object.checked {
                ("Uncheck", "unchecked")
            } else {
                ("Mark checked", "checked")
            };
            if ui
                .add(ReButton::new(check).small())
                .on_hover_text("C")
                .clicked()
            {
                let op = json!({"op": "check", "id": id, "checked": !object.checked});
                self.edit(&ctx, op, format!("O{id} {done}"));
            }
            if ui.add(ReButton::new("Merge with…").small()).clicked() {
                self.start_mode(Mode::Merge, world);
            }
            if ui.add(ReButton::new("Split off…").small()).clicked() {
                self.start_mode(Mode::Split, world);
            }
            let delete = ReButton::new("Delete").small().ghost();
            if ui.add(delete).on_hover_text("Del").clicked() {
                self.confirm_delete = true;
            }
        });
    }

    fn room_card(&mut self, ui: &mut egui::Ui, world: &World, room: &Room) {
        let ctx = ui.ctx().clone();
        let (id, picked) = (room.id.clone(), Picked::Room(room.id.clone()));
        if self.form.of.as_ref() != Some(&picked) {
            self.form = Form {
                kind: room.kind.clone(),
                name: room.name.clone(),
                of: Some(picked.clone()),
                ..Form::default()
            };
        }
        ui.horizontal_wrapped(|ui| {
            ui.label(
                RichText::new(format!(
                    "{id} {}",
                    if room.kind.is_empty() {
                        &room.name
                    } else {
                        &room.kind
                    }
                ))
                .strong()
                .size(15.0),
            );
            if room.checked {
                badge(ui, "checked", CHECKED);
            }
        });
        if let Some(s) = Self::suspicion(world, &picked) {
            for reason in &s.reasons {
                ui.label(RichText::new(format!("• {reason}")).small().color(SUSPECT));
            }
        }
        text_field(ui, "Type", &mut self.form.kind, &world.room_types);
        if ui.add(ReButton::new("Set type").small()).clicked() {
            let kind = self.form.kind.trim().to_owned();
            self.edit(
                &ctx,
                json!({"op": "room_type", "room": id, "type": kind}),
                format!("{id} is a {kind}"),
            );
        }
        text_field(ui, "Name", &mut self.form.name, &[]);
        if ui.add(ReButton::new("Set name").small()).clicked() {
            let name = self.form.name.trim().to_owned();
            self.edit(
                &ctx,
                json!({"op": "room_name", "room": id, "name": name}),
                format!("{id} renamed"),
            );
        }
        let typed = if room.type_source.is_empty() {
            "nothing yet".to_owned()
        } else {
            format!("{}, {:.2}", room.type_source, room.type_confidence)
        };
        ui.label(
            RichText::new(format!("typed by {typed}"))
                .small()
                .color(theme::DIM),
        );
        let check = if room.checked {
            "Uncheck"
        } else {
            "Mark checked"
        };
        if ui
            .add(ReButton::new(check).small())
            .on_hover_text("C")
            .clicked()
        {
            self.edit(
                &ctx,
                json!({"op": "room_check", "room": id, "checked": !room.checked}),
                format!(
                    "{id} {}",
                    if room.checked { "unchecked" } else { "checked" }
                ),
            );
        }
    }

    fn review(&mut self, ui: &mut egui::Ui, world: &World) {
        let ctx = ui.ctx().clone();
        let phantoms: Vec<u64> = world
            .suggestions
            .iter()
            .filter(|s| s.phantom)
            .filter_map(|s| s.id.as_u64())
            .collect();
        ui.horizontal(|ui| {
            ui.label(RichText::new(format!("To review ({})", world.suggestions.len())).strong());
            if !phantoms.is_empty()
                && ui
                    .add(
                        ReButton::new(format!("Remove {} likely phantoms…", phantoms.len()))
                            .small()
                            .ghost(),
                    )
                    .clicked()
            {
                self.confirm_phantoms = true;
            }
        });
        if self.confirm_phantoms {
            ui.label(RichText::new(format!("Remove {} objects that look like no object at all? They stay in the file and each can be restored.", phantoms.len())).small());
            ui.horizontal(|ui| {
                if ui.add(ReButton::new("Remove them").small().primary()).clicked() {
                    self.confirm_phantoms = false;
                    let op = json!({"op": "remove", "ids": phantoms, "reason": "removed by hand: a likely phantom"});
                    self.edit(&ctx, op, format!("{} likely phantoms removed", phantoms.len()));
                }
                if ui.add(ReButton::new("Cancel").small().ghost()).clicked() {
                    self.confirm_phantoms = false;
                }
            });
        }
        if world.suggestions.is_empty() {
            ui.label(
                RichText::new("Nothing to review.")
                    .small()
                    .color(theme::DIM),
            );
        }
        for item in &world.suggestions {
            let title = match item.picked() {
                Some(Picked::Object(id)) => format!(
                    "O{id} {}",
                    world
                        .objects
                        .iter()
                        .find(|o| o.id == id)
                        .map_or("", |o| o.label.as_str())
                ),
                Some(Picked::Room(id)) => format!(
                    "{id} {}",
                    world
                        .rooms
                        .iter()
                        .find(|r| r.id == id)
                        .map_or("", |r| r.kind.as_str())
                ),
                None => continue,
            };
            let reason = item.reasons.first().cloned().unwrap_or_default();
            let chosen = item.picked() == self.selected;
            let text = RichText::new(format!("{title} · {reason}")).small();
            if ui.selectable_label(chosen, text).clicked() {
                self.go_to(world, item);
            }
        }
        let removed: Vec<&Object> = world
            .objects
            .iter()
            .filter(|o| o.state == "removed" && !o.removed_by.is_empty())
            .collect();
        if !removed.is_empty() {
            ui.add_space(6.0);
            egui::CollapsingHeader::new(
                RichText::new(format!("Removed ({})", removed.len())).strong(),
            )
            .default_open(false)
            .show(ui, |ui| {
                for o in removed {
                    let chosen = self.selected == Some(Picked::Object(o.id));
                    if ui
                        .selectable_label(
                            chosen,
                            RichText::new(format!("O{} {} · {}", o.id, o.label, o.removed_by))
                                .small(),
                        )
                        .clicked()
                    {
                        self.selected = Some(Picked::Object(o.id));
                    }
                }
            });
        }
    }
}

fn badge(ui: &mut egui::Ui, text: &str, colour: Color32) {
    egui::Frame::new()
        .fill(colour.gamma_multiply(0.18))
        .corner_radius(4.0)
        .inner_margin(egui::Margin::symmetric(6, 1))
        .show(ui, |ui| ui.label(RichText::new(text).small().color(colour)));
}

/// A labelled text field with a menu of known words beside it.
fn text_field(ui: &mut egui::Ui, title: &str, value: &mut String, words: &[String]) {
    ui.label(RichText::new(title).small().color(theme::DIM));
    ui.horizontal(|ui| {
        let width = if words.is_empty() {
            ui.available_width()
        } else {
            ui.available_width() - 30.0
        };
        ui.add(egui::TextEdit::singleline(value).desired_width(width.max(60.0)));
        if !words.is_empty() {
            ui.menu_button("…", |ui| {
                egui::ScrollArea::vertical()
                    .max_height(260.0)
                    .show(ui, |ui| {
                        for word in words {
                            if ui.button(word).clicked() {
                                value.clone_from(word);
                                ui.close();
                            }
                        }
                    });
            });
        }
    });
}
