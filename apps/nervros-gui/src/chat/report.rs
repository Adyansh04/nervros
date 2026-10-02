//! Mission reports, as the model read them.

use rerun::external::egui;
use rerun::external::egui::{CornerRadius, Frame, Margin, RichText};
use rerun::external::re_ui::{UiExt as _, icons};

use crate::theme;

/// A report from the robot, not the agent: its own colour, so the two never read as one voice.
pub(super) fn report(ui: &mut egui::Ui, text: &str) {
    Frame::new()
        .fill(theme::SURFACE)
        .stroke(egui::Stroke::new(1.0, theme::BORDER))
        .corner_radius(CornerRadius::same(8))
        .inner_margin(Margin::symmetric(10, 7))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.small_icon(&icons::AGENT, Some(theme::ROBOT));
                ui.label(
                    RichText::new("Robot")
                        .strong()
                        .size(13.0)
                        .color(theme::ROBOT),
                );
                ui.add(egui::Label::new(RichText::new(report_text(text)).size(13.0)).wrap());
            });
        });
}

/// A robot report as the operator reads it: what happened, with mission ids cut to their first
/// eight characters and without the marks that tell the model what came from the world; the
/// line after it tells the model what to do next.
pub(super) fn report_text(text: &str) -> String {
    let first = text.lines().next().unwrap_or(text);
    first
        .replace("<world>", "")
        .replace("</world>", "")
        .split(' ')
        .map(|w| {
            let id = w.len() == 36 && w.chars().all(|c| c.is_ascii_hexdigit() || c == '-');
            match w.get(..8) {
                Some(short) if id => short,
                _ => w,
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}
