//! The World and Layers tabs: the world model's rooms and objects, the zones, and the
//! viewer's layers.

use std::fmt::Write as _;

use nervros_core::profile::PlaceConfig;
use nervros_core::zones::{Rule, Zone};
use rerun::external::egui;
use rerun::external::egui::{Align, CornerRadius, Frame, Layout, Margin, RichText, Stroke};
use rerun::external::re_ui::{ReButton, UiExt as _};
use serde_json::Value;

use super::dock::empty;
use super::{EXPLORE_REQUEST, Gui};
use crate::chat::Action;
use crate::theme;

impl Gui {
    pub(super) fn world_tab(&mut self, ui: &mut egui::Ui) {
        if let (true, Some(editor)) = (self.editing, &mut self.editor) {
            editor.panel(ui);
            return;
        }
        if self.editor.is_some()
            && ui
                .add(ReButton::new("Edit the world").small().secondary())
                .on_hover_text("Fix labels, boxes and rooms on the saved floor plan")
                .clicked()
        {
            self.editing = true;
        }
        let (rooms, objects) = {
            let l = self.live();
            (l.rooms.clone(), l.objects)
        };
        let zones = self.agent.zones.all();
        let mut actions = if let Some(rooms) = rooms {
            world_view(ui, &rooms, objects, &zones)
        } else {
            empty(
                ui,
                "No world model yet. With one running, its rooms and how much of each the camera \
                 has seen appear here.",
            );
            zones_view(ui, &zones)
        };
        actions.extend(places_view(ui, &self.agent.places.all()));
        self.act(ui.ctx(), actions);
    }

    pub(super) fn layers_tab(&self, ui: &mut egui::Ui) {
        let config = self
            .agent
            .profile
            .viz
            .as_ref()
            .map_or(&[][..], |v| v.layers.as_slice());
        layers_view(ui, &self.bridge.layers, config);
    }
}

/// The named places, the profile's and the operator's, each with a button that puts a walk there
/// in the composer.
pub(super) fn places_view(ui: &mut egui::Ui, places: &[PlaceConfig]) -> Vec<Action> {
    let mut actions = Vec::new();
    ui.add_space(14.0);
    theme::section(ui, "Places");
    if places.is_empty() {
        ui.label(
            RichText::new("None yet: ask the agent to remember where the robot stands.")
                .color(theme::DIM),
        );
        return actions;
    }
    ui.add_space(6.0);
    for place in places {
        let name = place.name.replace('_', " ");
        let mut about = format!(
            "{} at ({:.2}, {:.2})",
            place.name, place.pose.x, place.pose.y
        );
        if !place.aliases.is_empty() {
            let _ = write!(about, "\nAlso: {}", place.aliases.join(", "));
        }
        if !place.near.is_empty() {
            let _ = write!(about, "\nNear: {}", place.near.join(", "));
        }
        card(ui, |ui| {
            // In a row: a right-to-left layout on its own takes all the height there is.
            ui.horizontal(|ui| {
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui
                        .add(ReButton::new("Go").small().secondary())
                        .on_hover_text("Put a walk here in the composer")
                        .clicked()
                    {
                        actions.push(Action::Prefill(format!("Walk to {name}.")));
                    }
                    ui.label(
                        RichText::new(format!("{:.1}, {:.1}", place.pose.x, place.pose.y))
                            .monospace()
                            .size(11.0)
                            .color(theme::FAINT),
                    );
                    ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                        swatch(ui, theme::ACCENT);
                        ui.add(egui::Label::new(RichText::new(&name).size(13.5)).truncate())
                            .on_hover_text(about);
                    });
                });
            });
        });
    }
    actions
}

/// A row of the World tab's lists, on its own surface.
fn card(ui: &mut egui::Ui, body: impl FnOnce(&mut egui::Ui)) {
    Frame::new()
        .fill(theme::SURFACE)
        .stroke(Stroke::new(1.0, theme::BORDER))
        .corner_radius(CornerRadius::same(8))
        .inner_margin(Margin::symmetric(10, 7))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            body(ui);
        });
    ui.add_space(4.0);
}

fn swatch(ui: &mut egui::Ui, colour: egui::Color32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
    ui.painter()
        .rect_filled(rect, CornerRadius::same(2), colour);
}

/// The world model's rooms by number, R2 before R10: canopy lists them in the order it segmented
/// them.
fn by_number(rooms: &Value) -> Vec<Value> {
    let mut list = rooms["rooms"].as_array().cloned().unwrap_or_default();
    list.sort_by_cached_key(|r| {
        let id = r["id"].as_str().unwrap_or_default();
        let number = id
            .trim_start_matches(|c: char| !c.is_ascii_digit())
            .parse::<u32>();
        (number.unwrap_or(u32::MAX), id.to_owned())
    });
    list
}

/// The World tab: the world model's rooms, how much of each the camera has seen, a button that
/// asks the agent to explore, and the zones.
pub(super) fn world_view(
    ui: &mut egui::Ui,
    rooms: &Value,
    objects: Option<usize>,
    zones: &[Zone],
) -> Vec<Action> {
    let list = by_number(rooms);
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
                .add(ReButton::new("Explore").small().blue())
                .on_hover_text(
                    "Ask the robot to explore the building. You approve the mission it plans.",
                )
                .clicked()
            {
                actions.push(Action::Say(EXPLORE_REQUEST.to_owned()));
            }
        });
    });
    ui.add_space(8.0);
    // The dock scrolls the tab; the scope keeps the grid's truncation to the grid.
    ui.scope(|ui| {
        // Long room names are cut, whole in their hover: wrapped, a grid row grows ragged.
        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Truncate);
        egui::Grid::new("world_rooms")
            .num_columns(4)
            .striped(true)
            .max_col_width(110.0)
            .spacing([10.0, 8.0])
            .show(ui, |ui| {
                for head in ["Room", "Floor seen", "Walls seen", "Objects"] {
                    ui.label(RichText::new(head).small().color(theme::FAINT));
                }
                ui.end_row();
                for r in &list {
                    // As the 3D view names it: its type while its name is canopy's "room C".
                    let id = r["id"].as_str().unwrap_or_default();
                    let name = match nervros_core::room_label(r) {
                        Some(label) => format!("{id} {}", label.replace('_', " ")),
                        None => id.to_owned(),
                    };
                    let whole = [r["name"].as_str(), r["type"].as_str()]
                        .into_iter()
                        .flatten()
                        .filter(|t| !t.is_empty())
                        .collect::<Vec<_>>()
                        .join(", ");
                    ui.label(RichText::new(&name).size(13.5))
                        .on_hover_text(format!("{id}: {whole}"));
                    for k in ["floor_coverage", "face_coverage"] {
                        match fraction(r, k) {
                            #[expect(clippy::cast_possible_truncation, reason = "a fraction")]
                            Some(f) => {
                                // Green once the camera has seen nearly all of it.
                                let fill = if f >= 0.9 {
                                    theme::SUCCESS
                                } else if f >= 0.5 {
                                    theme::WARN
                                } else {
                                    theme::ERROR
                                };
                                ui.add(
                                    egui::ProgressBar::new(f as f32)
                                        .desired_width(70.0)
                                        .desired_height(16.0)
                                        .fill(fill.gamma_multiply(0.75))
                                        .corner_radius(CornerRadius::same(4))
                                        .text(
                                            RichText::new(format!("{:.0}%", f * 100.0))
                                                .size(12.0)
                                                .color(theme::TEXT),
                                        ),
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
    actions.extend(zones_view(ui, zones));
    actions
}

/// The zones, the profile's and the operator's: keep-out zones in red, slow zones in amber with
/// their speed. Zones change only through the agent, so the buttons ask it: removing one lets the
/// robot go where or as fast as it could not, and the operator approves that.
pub(super) fn zones_view(ui: &mut egui::Ui, zones: &[Zone]) -> Vec<Action> {
    let mut actions = Vec::new();
    ui.add_space(14.0);
    ui.horizontal(|ui| {
        theme::section(ui, "Zones");
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            if ui
                .add(ReButton::new("Slow zone").small().secondary())
                .on_hover_text(
                    "Say where the robot must slow down, and how much: a room, an object, a \
                     place, or corners on the map",
                )
                .clicked()
            {
                actions.push(Action::Prefill(
                    "Make a slow zone at 30% around ".to_owned(),
                ));
            }
            if ui
                .add(ReButton::new("Keep out").small().secondary())
                .on_hover_text(
                    "Say where the robot must not go: a room, an object, a place, or corners on \
                     the map",
                )
                .clicked()
            {
                actions.push(Action::Prefill("Keep the robot out of ".to_owned()));
            }
        });
    });
    if zones.is_empty() {
        ui.label(
            RichText::new("None: the robot may go anywhere on the map, at full speed.")
                .color(theme::DIM),
        );
        return actions;
    }
    ui.add_space(6.0);
    for zone in zones {
        let (colour, kind) = match zone.rule {
            Rule::KeepOut => (theme::ERROR, "keep-out"),
            Rule::Slow(_) => (theme::WARN, "slow"),
        };
        let corners = zone
            .polygon
            .iter()
            .map(|[x, y]| format!("({x:.2}, {y:.2})"))
            .collect::<Vec<_>>()
            .join(" ");
        card(ui, |ui| {
            // In a row: a right-to-left layout on its own takes all the height there is.
            ui.horizontal(|ui| {
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if zone.fixed {
                        ui.label(
                            RichText::new("from the profile")
                                .small()
                                .color(theme::FAINT),
                        )
                        .on_hover_text("Only an edit of the robot's profile removes this zone");
                    } else if ui
                        .add(ReButton::new("Remove").small().secondary())
                        .on_hover_text("Ask the agent to remove it; you approve")
                        .clicked()
                    {
                        actions.push(Action::Say(format!(
                            "Remove the {kind} zone \"{}\".",
                            zone.name
                        )));
                    }
                    ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                        // The swatch is the 3D view's colour for a zone.
                        swatch(ui, colour);
                        ui.add(
                            egui::Label::new(RichText::new(&zone.name).strong().size(13.5))
                                .truncate(),
                        )
                        .on_hover_text(format!("Corners on the map: {corners}"));
                    });
                });
            });
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                ui.label(RichText::new(kind).size(12.0).color(colour));
                if let Rule::Slow(limit) = zone.rule {
                    ui.label(
                        RichText::new(format!("· at most {}", limit.words()))
                            .size(12.0)
                            .color(theme::DIM),
                    )
                    .on_hover_text("Nav2 slows the robot to this inside the zone");
                }
                ui.label(
                    RichText::new(format!("· {:.1} m²", zone.area()))
                        .size(12.0)
                        .color(theme::DIM),
                );
            });
        });
    }
    actions
}

/// A switch per layer the viewer draws, as RViz's displays list has.
/// The headings of the Layers tab, in order.
const GROUPS: [&str; 6] = [
    "Robot",
    "Navigation",
    "Arms",
    "World model",
    "Sensors",
    "Other",
];

/// The heading a layer goes under, and its colour when it has one of its own: the profile's word
/// for one of its layers, else what its type or name says.
fn group_of<'a>(
    name: &str,
    config: &'a [nervros_core::profile::LayerConfig],
) -> (&'a str, Option<[u8; 3]>) {
    if let Some(c) = config.iter().find(|c| c.name == name) {
        let costmap = c.style.as_deref() == Some("costmap");
        let by_type = match c.msg_type.as_str() {
            _ if costmap => "Navigation",
            "geometry_msgs/msg/PolygonStamped" | "nav_msgs/msg/Path" => "Navigation",
            "sensor_msgs/msg/LaserScan" => "Sensors",
            _ => "World model",
        };
        // A costmap is a gradient; its swatch is the inscribed radius's violet.
        let colour = c.color.or(costmap.then_some([150, 95, 235]));
        return (c.group.as_deref().unwrap_or(by_type), colour);
    }
    let group = match name {
        "Robot model" | "Trail" | "Plan" => "Robot",
        "Zones" => "Navigation",
        "Arm obstacles" => "Arms",
        "Map" | "Rooms" | "Objects" | "Object names" | "Camera coverage" => "World model",
        "Detections" => "Sensors",
        _ => "Other",
    };
    let colour = match name {
        "Zones" => Some([226, 76, 76]),
        "Plan" => Some([255, 120, 200]),
        "Arm obstacles" => Some([150, 120, 255]),
        _ => None,
    };
    (group, colour)
}

/// The Layers tab: each layer under its heading with a switch, its colour beside its name.
pub(super) fn layers_view(
    ui: &mut egui::Ui,
    layers: &nervros_viz::Layers,
    config: &[nervros_core::profile::LayerConfig],
) {
    let list = layers.list();
    if list.is_empty() {
        empty(ui, "The profile gives the viewer nothing to draw.");
        return;
    }
    ui.label(
        RichText::new("What the world and camera views draw. A hidden layer is cleared.")
            .color(theme::DIM),
    );
    for group in GROUPS {
        let rows: Vec<&(String, bool)> = list
            .iter()
            .filter(|(name, _)| group_of(name, config).0 == group)
            .collect();
        if rows.is_empty() {
            continue;
        }
        ui.add_space(14.0);
        theme::section(ui, group);
        for (name, shown) in rows {
            let mut shown = *shown;
            ui.horizontal(|ui| {
                let toggle = ui.toggle_switch(14.0, &mut shown);
                // Named for a screen reader, and for a script that clicks it.
                toggle.widget_info(|| {
                    egui::WidgetInfo::selected(egui::WidgetType::Checkbox, true, shown, name)
                });
                if toggle.changed() {
                    layers.set(name, shown);
                }
                if let (_, Some([r, g, b])) = group_of(name, config) {
                    let colour = egui::Color32::from_rgb(r, g, b);
                    swatch(
                        ui,
                        if shown {
                            colour
                        } else {
                            colour.gamma_multiply(0.35)
                        },
                    );
                }
                let text = RichText::new(name);
                ui.label(if shown {
                    text
                } else {
                    text.color(theme::FAINT)
                });
            });
            ui.add_space(2.0);
        }
    }
}
