//! Mission reports, as the model read them.

use rerun::external::egui;
use rerun::external::egui::{CornerRadius, Frame, Margin, RichText};
use rerun::external::re_ui::{UiExt as _, icons};

pub(super) fn report(ui: &mut egui::Ui, text: &str) {
    Frame::new()
        .fill(ui.tokens().faint_bg_color)
        .corner_radius(CornerRadius::same(8))
        .inner_margin(Margin::symmetric(10, 6))
        .show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.small_icon(&icons::AGENT, Some(ui.tokens().text_subdued));
                ui.label(RichText::new("Robot report").small().strong());
                ui.label(
                    RichText::new(report_text(text))
                        .small()
                        .color(ui.tokens().text_subdued),
                );
            });
        });
}

/// A robot report as the operator reads it: what happened, with mission ids cut to their first
/// eight characters; the line after it tells the model what to do next.
pub(super) fn report_text(text: &str) -> String {
    let first = text.lines().next().unwrap_or(text);
    first
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
