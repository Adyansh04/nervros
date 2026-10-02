//! The chat's cards and bubbles: messages, replies, tool chips, images, notices and errors.

use std::time::{Duration, SystemTime};

use nervros_core::session::Command;
use nervros_core::tools::Status;
use rerun::external::egui;
use rerun::external::egui::{Align, Color32, CornerRadius, Frame, Layout, Margin, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _, icons};
use serde_json::Value;

use super::{Action, SUGGESTIONS, ToolCall};

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
pub fn status_color(ui: &egui::Ui, status: Option<Status>) -> Color32 {
    let t = ui.tokens();
    match status {
        None => t.info_text_color,
        Some(Status::Succeeded | Status::Accepted) => t.success_text_color,
        Some(Status::Refused) => t.warn_fg_color,
        Some(Status::Stopped) => t.text_subdued,
        Some(Status::Failed) => t.error_fg_color,
    }
}

pub(super) fn card(ui: &egui::Ui, stroke: Color32) -> Frame {
    Frame::new()
        .fill(ui.tokens().panel_bg_color)
        .stroke(egui::Stroke::new(1.0, stroke))
        .corner_radius(CornerRadius::same(10))
        .inner_margin(Margin::same(12))
}

pub(super) fn empty_state(ui: &mut egui::Ui, actions: &mut Vec<Action>) {
    ui.add_space(ui.available_height() * 0.3);
    ui.vertical_centered(|ui| {
        ui.label(
            RichText::new("Ask about the robot and its surroundings")
                .strong()
                .size(16.0),
        );
        ui.add_space(8.0);
        ui.label(
            RichText::new("The agent can look, find objects and check the robot's state.")
                .color(ui.tokens().text_subdued),
        );
        ui.add_space(16.0);
        for s in SUGGESTIONS {
            if ui.add(ReButton::new(s).secondary()).clicked() {
                actions.push(Action::Say(s.to_owned()));
            }
            ui.add_space(4.0);
        }
    });
}

/// The operator's message, with "condense up to here" on a right click; `later` is how many of
/// their messages came after it.
/// A message of the operator's; `later` is how many turns of theirs came after it, for a message
/// that started one.
pub(super) fn user_bubble(
    ui: &mut egui::Ui,
    text: &str,
    later: Option<usize>,
    actions: &mut Vec<Action>,
) {
    ui.with_layout(Layout::right_to_left(Align::Min), |ui| {
        let bubble = Frame::new()
            .fill(ui.tokens().selection_bg_fill)
            .corner_radius(CornerRadius::same(12))
            .inner_margin(Margin::symmetric(12, 8))
            .show(ui, |ui| {
                ui.set_max_width(ui.available_width() * 0.8);
                ui.label(RichText::new(text).color(ui.tokens().text_strong));
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

pub(super) fn reply(ui: &mut egui::Ui, text: &str, model: &str) {
    // An image in a reply would be fetched from wherever it points, a URL prompt injection can
    // write: shown as a link, it goes nowhere unless clicked.
    ui.markdown_ui(&text.replace("![", "["));
    ui.label(RichText::new(model).small().color(ui.tokens().text_subdued));
}

pub(super) fn tool_chip(ui: &mut egui::Ui, t: &ToolCall) {
    let color = status_color(ui, t.status);
    let summary = t.message.lines().next().unwrap_or_default();
    let header = format!(
        "{}  {}{}",
        t.tool,
        summary,
        if t.status.is_some() {
            format!("  · {} ms", t.ms)
        } else {
            String::new()
        }
    );
    Frame::new()
        .fill(ui.tokens().faint_bg_color)
        .corner_radius(CornerRadius::same(8))
        .inner_margin(Margin::symmetric(8, 4))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                if t.status.is_none() {
                    spinner(ui);
                } else {
                    let icon = match t.status {
                        Some(Status::Succeeded | Status::Accepted) => &icons::SUCCESS,
                        Some(Status::Refused) => &icons::WARNING,
                        Some(Status::Stopped) => &icons::PAUSE,
                        _ => &icons::ERROR,
                    };
                    ui.small_icon(icon, Some(color));
                }
                egui::CollapsingHeader::new(RichText::new(header).monospace().size(12.0))
                    .id_salt(("tool", t.turn, t.call))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("arguments")
                                .small()
                                .color(ui.tokens().text_subdued),
                        );
                        code(ui, &pretty(&t.args));
                        if !t.message.is_empty() {
                            ui.label(
                                RichText::new("result")
                                    .small()
                                    .color(ui.tokens().text_subdued),
                            );
                            code(ui, &t.message);
                        }
                    });
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
    let stroke = ui.tokens().widget_noninteractive_bg_stroke;
    card(ui, stroke).show(ui, |ui| {
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
            let caption = RichText::new(caption);
            ui.label(caption.small().color(ui.tokens().text_subdued));
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui
                    .add(ReButton::new("Ask about it").small().primary())
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
    let colour = ui.tokens().info_text_color;
    for k in 0u8..8 {
        let angle = f32::from(k) * std::f32::consts::TAU / 8.0;
        let at = rect.center() + size * 0.38 * egui::vec2(angle.cos(), angle.sin());
        let alpha = if k == lit { 1.0 } else { 0.3 };
        ui.painter()
            .circle_filled(at, size * 0.1, colour.gamma_multiply(alpha));
    }
    ui.ctx().request_repaint_after(Duration::from_millis(125));
}

pub(super) fn notice(ui: &mut egui::Ui, text: &str) {
    ui.horizontal(|ui| {
        ui.small_icon(&icons::INFO, Some(ui.tokens().text_subdued));
        // The viewer's style extends labels, so a long notice would widen the panel instead.
        ui.add(egui::Label::new(RichText::new(text).color(ui.tokens().text_subdued)).wrap());
    });
}

pub(super) fn error_card(
    ui: &mut egui::Ui,
    text: &str,
    retry: Option<&str>,
    actions: &mut Vec<Action>,
) {
    let t = ui.tokens();
    card(ui, t.error_fg_color).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.small_icon(&icons::ERROR, Some(t.error_fg_color));
            ui.add(egui::Label::new(RichText::new(text).color(t.error_fg_color)).wrap());
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

pub(super) fn code(ui: &mut egui::Ui, text: &str) {
    Frame::new()
        .fill(ui.tokens().extreme_bg_color)
        .corner_radius(CornerRadius::same(6))
        .inner_margin(Margin::same(8))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(text).monospace().size(12.0));
        });
}
