//! The World and Layers tabs: the world model's rooms and objects, and the viewer's layers.

use std::fmt::Write as _;

use rerun::external::egui;
use rerun::external::egui::{Align, Layout, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _};
use serde_json::Value;

use super::dock::empty;
use super::{EXPLORE_REQUEST, Gui};
use crate::chat::Action;

impl Gui {
    pub(super) fn world_tab(&mut self, ui: &mut egui::Ui) {
        if let (true, Some(editor)) = (self.editing, &mut self.editor) {
            editor.panel(ui);
            return;
        }
        if self.editor.is_some()
            && ui
                .add(ReButton::new("Edit the world").small())
                .on_hover_text("Fix labels, boxes and rooms on the saved floor plan")
                .clicked()
        {
            self.editing = true;
        }
        let (rooms, objects) = {
            let l = self.live();
            (l.rooms.clone(), l.objects)
        };
        let Some(rooms) = rooms else {
            empty(
                ui,
                "No world model yet. With one running, its rooms and how much of each the camera \
                 has seen appear here.",
            );
            return;
        };
        let actions = world_view(ui, &rooms, objects);
        self.act(ui.ctx(), actions);
    }

    pub(super) fn layers_tab(&self, ui: &mut egui::Ui) {
        layers_view(ui, &self.bridge.layers);
    }
}

/// The World tab: the world model's rooms, how much of each the camera has seen, and a button that
/// asks the agent to explore.
pub(super) fn world_view(ui: &mut egui::Ui, rooms: &Value, objects: Option<usize>) -> Vec<Action> {
    let list = rooms["rooms"].as_array().cloned().unwrap_or_default();
    let fraction = |r: &Value, k: &str| r[k].as_f64().map(|v| v.clamp(0.0, 1.0));
    let seen: Vec<f64> = list
        .iter()
        .filter_map(|r| {
            let floor = fraction(r, "floor_coverage")?;
            Some(f64::midpoint(
                floor,
                fraction(r, "face_coverage").unwrap_or(floor),
            ))
        })
        .collect();
    let mut actions = Vec::new();
    ui.horizontal(|ui| {
        let mut head = format!("{} rooms · {} objects", list.len(), objects.unwrap_or(0));
        if !seen.is_empty() {
            #[expect(clippy::cast_precision_loss, reason = "a handful of rooms")]
            let mean = seen.iter().sum::<f64>() / seen.len() as f64;
            let _ = write!(head, " · {:.0}% seen", mean * 100.0);
        }
        ui.label(RichText::new(head).strong());
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            if ui
                .button("Explore")
                .on_hover_text(
                    "Ask the robot to explore the building. You approve the mission it plans.",
                )
                .clicked()
            {
                actions.push(Action::Say(EXPLORE_REQUEST.to_owned()));
            }
        });
    });
    ui.add_space(4.0);
    egui::ScrollArea::vertical().show(ui, |ui| {
        egui::Grid::new("world_rooms")
            .num_columns(4)
            .striped(true)
            .spacing([10.0, 6.0])
            .show(ui, |ui| {
                for head in ["Room", "Floor seen", "Walls seen", "Objects"] {
                    ui.label(RichText::new(head).small().color(ui.tokens().text_subdued));
                }
                ui.end_row();
                for r in &list {
                    let text = |k: &str| r[k].as_str().unwrap_or_default().to_owned();
                    let (id, name, kind) = (text("id"), text("name"), text("type"));
                    // canopy's names are often its types; say it once.
                    let name = [
                        id,
                        name.clone(),
                        if kind == name { String::new() } else { kind },
                    ]
                    .into_iter()
                    .filter(|t| !t.is_empty())
                    .collect::<Vec<_>>()
                    .join(" ");
                    ui.label(name);
                    for k in ["floor_coverage", "face_coverage"] {
                        match fraction(r, k) {
                            #[expect(clippy::cast_possible_truncation, reason = "a fraction")]
                            Some(f) => {
                                ui.add(
                                    egui::ProgressBar::new(f as f32)
                                        .desired_width(90.0)
                                        .text(format!("{:.0}%", f * 100.0)),
                                );
                            }
                            None => {
                                ui.label("·");
                            }
                        }
                    }
                    ui.label(
                        r["object_count"]
                            .as_u64()
                            .map_or("·".to_owned(), |n| n.to_string()),
                    );
                    ui.end_row();
                }
            });
    });
    actions
}

/// A switch per layer the viewer draws, as RViz's displays list has.
pub(super) fn layers_view(ui: &mut egui::Ui, layers: &nervros_viz::Layers) {
    let list = layers.list();
    if list.is_empty() {
        empty(ui, "The profile gives the viewer nothing to draw.");
        return;
    }
    ui.label(
        RichText::new("What the world and camera views draw. A hidden layer is cleared.")
            .small()
            .color(ui.tokens().text_subdued),
    );
    ui.add_space(6.0);
    for (name, mut shown) in list {
        ui.horizontal(|ui| {
            if ui.toggle_switch(12.0, &mut shown).changed() {
                layers.set(&name, shown);
            }
            let text = RichText::new(&name);
            ui.label(if shown {
                text
            } else {
                text.color(ui.tokens().text_subdued)
            });
        });
        ui.add_space(2.0);
    }
}
