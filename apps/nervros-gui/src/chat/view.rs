//! The chat's cards and bubbles: messages, replies, tool chips, images, notices and errors.

use std::time::{Duration, SystemTime};

use nervros_core::session::Command;
use nervros_core::tools::Status;
use rerun::external::egui;
use rerun::external::egui::collapsing_header::CollapsingState;
use rerun::external::egui::{Align, Color32, CornerRadius, Frame, Layout, Margin, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _, icons};
use serde_json::Value;

use super::{Action, SUGGESTIONS, ToolCall};
use crate::theme;

/// A token count as people read it: 812, 9.8k.
#[must_use]
pub fn thousands(n: u64) -> String {
    if n < 1000 {
        n.to_string()
    } else {
        #[expect(clippy::cast_precision_loss, reason = "shown to one decimal")]
        let k = n as f64 / 1000.0;
        format!("{k:.1}k")
    }
}

/// Running is blue, success green, failure red and waiting amber, everywhere in the app; a
/// call cut short by a stop is grey.
pub fn status_color(status: Option<Status>) -> Color32 {
    match status {
        None => theme::INFO,
        Some(Status::Succeeded | Status::Accepted) => theme::SUCCESS,
        Some(Status::Refused) => theme::WARN,
        Some(Status::Stopped) => theme::FAINT,
        Some(Status::Failed) => theme::ERROR,
    }
}

pub(super) fn empty_state(ui: &mut egui::Ui, actions: &mut Vec<Action>) {
    ui.add_space(ui.available_height() * 0.28);
    ui.vertical_centered(|ui| {
        ui.label(
            RichText::new("Ask about the robot and its surroundings")
                .strong()
                .size(18.0),
        );
        ui.add_space(6.0);
        ui.label(
            RichText::new("The agent can look, find objects and check the robot's state.")
                .color(theme::DIM),
        );
        ui.add_space(18.0);
        for s in SUGGESTIONS {
            if ui.add(ReButton::new(s).secondary()).clicked() {
                actions.push(Action::Say(s.to_owned()));
            }
            ui.add_space(6.0);
        }
    });
}

/// The agent's name over the first thing it says or does in a turn, with the model answering.
pub(super) fn agent_header(ui: &mut egui::Ui, model: Option<&str>) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 7.0;
        let (rect, _) = ui.allocate_exact_size(egui::vec2(20.0, 20.0), egui::Sense::hover());
        ui.painter()
            .circle_filled(rect.center(), 10.0, theme::ACCENT);
        ui.painter().text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "N",
            egui::FontId::proportional(11.5),
            Color32::WHITE,
        );
        ui.label(RichText::new("NervROS").strong().size(13.5));
        if let Some(model) = model {
            ui.label(RichText::new(model).small().color(theme::FAINT));
        }
    });
}

/// A message of the operator's; `later` is how many turns of theirs came after it, for a message
/// that started one: a right click condenses the conversation up to it.
pub(super) fn user_bubble(
    ui: &mut egui::Ui,
    text: &str,
    later: Option<usize>,
    actions: &mut Vec<Action>,
) {
    ui.with_layout(Layout::right_to_left(Align::Min), |ui| {
        let bubble = Frame::new()
            .fill(theme::ACCENT)
            .corner_radius(CornerRadius {
                nw: 14,
                ne: 14,
                sw: 14,
                se: 4,
            })
            .inner_margin(Margin::symmetric(14, 9))
            .show(ui, |ui| {
                ui.set_max_width(ui.available_width() * 0.82);
                ui.label(RichText::new(text).color(Color32::WHITE));
            })
            .response
            .interact(egui::Sense::click());
        let Some(later) = later else {
            return;
        };
        bubble.context_menu(|ui| {
            if ui
                .button("Condense up to here")
                .on_hover_text(
                    "Summarise the conversation up to this message; what came after stays as it is",
                )
                .clicked()
            {
                actions.push(Action::Send(Command::CompactUpTo { keep: later }));
                ui.close();
            }
        });
    });
}

pub(super) fn reply(ui: &mut egui::Ui, text: &str) {
    // An image in a reply would be fetched from wherever it points, a URL prompt injection can
    // write: shown as a link, it goes nowhere unless clicked.
    ui.markdown_ui(&text.replace("![", "["));
}

/// A tool call as one row: what ran and the first line of what it said; a click opens its
/// arguments and full result.
pub(super) fn tool_chip(ui: &mut egui::Ui, t: &ToolCall) {
    let colour = status_color(t.status);
    let summary = t.message.lines().next().unwrap_or_default();
    let id = ui.make_persistent_id(("tool", t.turn, t.call));
    let mut state = CollapsingState::load_with_default_open(ui.ctx(), id, false);
    Frame::new()
        .fill(theme::SURFACE)
        .corner_radius(CornerRadius::same(8))
        .inner_margin(Margin::symmetric(10, 6))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            let row = ui
                .horizontal(|ui| {
                    if t.status.is_none() {
                        spinner(ui);
                    } else {
                        let icon = match t.status {
                            Some(Status::Succeeded | Status::Accepted) => &icons::SUCCESS,
                            Some(Status::Refused) => &icons::WARNING,
                            Some(Status::Stopped) => &icons::PAUSE,
                            _ => &icons::ERROR,
                        };
                        ui.small_icon(icon, Some(colour));
                    }
                    ui.label(RichText::new(&t.tool).monospace().color(theme::TEXT));
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        theme::chevron(ui, state.is_open());
                        ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                            ui.add(
                                egui::Label::new(
                                    RichText::new(summary).size(13.0).color(theme::DIM),
                                )
                                .truncate(),
                            );
                        });
                    });
                })
                .response;
            let row = ui.interact(row.rect, id.with("row"), egui::Sense::click());
            if row.clicked() {
                state.toggle(ui);
            }
            state.show_body_unindented(ui, |ui| {
                ui.add_space(4.0);
                ui.label(RichText::new("Arguments").small().color(theme::FAINT));
                code(ui, &pretty(&t.args));
                if !t.message.is_empty() {
                    ui.label(RichText::new("Result").small().color(theme::FAINT));
                    code(ui, &t.message);
                }
                if t.status.is_some() {
                    ui.label(
                        RichText::new(format!("Took {} ms", t.ms))
                            .small()
                            .color(theme::FAINT),
                    );
                }
            });
        });
}

/// A marked image: its marks as a legend to ask about one by one, when it was taken, and the
/// 3D view as it was then.
pub(super) fn image_card(
    ui: &mut egui::Ui,
    id: &str,
    texture: &egui::TextureHandle,
    marks: &[String],
    at: SystemTime,
    actions: &mut Vec<Action>,
) {
    theme::card().show(ui, |ui| {
        let [w, h] = texture.size();
        ui.add(
            egui::Image::new(texture)
                .max_width(ui.available_width())
                .corner_radius(CornerRadius::same(6)),
        );
        if !marks.is_empty() {
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().item_spacing.x = 4.0;
                for (i, label) in marks.iter().enumerate() {
                    let n = i + 1;
                    if ui
                        .add(ReButton::new(format!("{n} {label}")).small().secondary())
                        .on_hover_text(format!("Ask about mark {n}"))
                        .clicked()
                    {
                        actions.push(Action::Prefill(format!(
                            "In snapshot {id}, mark {n} ({label}): "
                        )));
                    }
                }
            });
        }
        ui.horizontal(|ui| {
            let taken = nervros_core::unix_secs(at);
            let caption = format!("Snapshot {id} · {} · {w}×{h}", crate::sessions::ago(taken));
            ui.label(RichText::new(caption).small().color(theme::FAINT));
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui
                    .add(ReButton::new("Ask about it").small().blue())
                    .clicked()
                {
                    actions.push(Action::Prefill(format!("In snapshot {id}, ")));
                }
                if ui
                    .add(ReButton::new("Show in 3D").small().secondary())
                    .on_hover_text("Pause the 3D view at the moment this was taken")
                    .clicked()
                {
                    actions.push(Action::ShowAt(at));
                }
            });
        });
    });
}

/// A JPEG as egui pixels, or `None` if it does not decode.
pub(crate) fn decode(jpeg: &[u8]) -> Option<egui::ColorImage> {
    // A JPEG has no alpha: straight to RGB, one pass fewer.
    let rgb = image::load_from_memory_with_format(jpeg, image::ImageFormat::Jpeg)
        .ok()?
        .into_rgb8();
    let size = [rgb.width(), rgb.height()].map(|v| usize::try_from(v).unwrap_or(0));
    Some(egui::ColorImage::from_rgb(size, rgb.as_raw()))
}

/// A spinner that moves eight times a second. egui's own redraws the whole window, viewer
/// included, at the screen's rate for as long as it shows.
pub fn spinner(ui: &mut egui::Ui) {
    let size = ui.spacing().interact_size.y * 0.6;
    let (rect, _) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::hover());
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "a step of eight, from a time that is never negative"
    )]
    let lit = (ui.input(|i| i.time) * 8.0).rem_euclid(8.0) as u8;
    for k in 0u8..8 {
        let angle = f32::from(k) * std::f32::consts::TAU / 8.0;
        let at = rect.center() + size * 0.38 * egui::vec2(angle.cos(), angle.sin());
        let alpha = if k == lit { 1.0 } else { 0.3 };
        ui.painter()
            .circle_filled(at, size * 0.1, theme::INFO.gamma_multiply(alpha));
    }
    ui.ctx().request_repaint_after(Duration::from_millis(125));
}

pub(super) fn notice(ui: &mut egui::Ui, text: &str) {
    ui.horizontal(|ui| {
        ui.small_icon(&icons::INFO, Some(theme::FAINT));
        ui.add(egui::Label::new(RichText::new(text).size(13.0).color(theme::DIM)).wrap());
    });
}

pub(super) fn error_card(
    ui: &mut egui::Ui,
    text: &str,
    retry: Option<&str>,
    actions: &mut Vec<Action>,
) {
    theme::status_card(ui, theme::ERROR, |ui| {
        ui.set_width(ui.available_width());
        ui.horizontal(|ui| {
            ui.small_icon(&icons::ERROR, Some(theme::ERROR));
            ui.add(egui::Label::new(RichText::new(text)).wrap());
        });
        if let Some(again) = retry
            && ui.add(ReButton::new("Retry").small().secondary()).clicked()
        {
            actions.push(Action::Say(again.to_owned()));
        }
    });
}

pub(super) fn pretty(v: &Value) -> String {
    serde_json::to_string_pretty(v).unwrap_or_else(|_| v.to_string())
}

/// Code in a well a shade darker than the card it is on.
pub(super) fn code(ui: &mut egui::Ui, text: &str) {
    Frame::new()
        .fill(theme::BG)
        .stroke(egui::Stroke::new(1.0, theme::BORDER))
        .corner_radius(CornerRadius::same(6))
        .inner_margin(Margin::same(8))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(text).monospace());
        });
}
