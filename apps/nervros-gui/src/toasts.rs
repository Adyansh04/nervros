//! Short notes in the window's corner for what ends while the operator looks elsewhere, such as
//! a mission finishing. Bottom right, so they never cover the viewer's own notes at the top.

use std::time::{Duration, Instant};

use rerun::external::egui::{self, Align2, CornerRadius, Frame, Margin, RichText, Stroke};
use rerun::external::re_ui::{UiExt as _, icons};

/// How long a note stays; a failure stays longer, as it needs reading.
const SHOWN: Duration = Duration::from_secs(6);
const FAILURE_SHOWN: Duration = Duration::from_secs(15);
/// The oldest go first past this many.
const MOST: usize = 4;

/// How a note looks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Success,
    Failure,
    Info,
}

#[derive(Debug)]
struct Toast {
    kind: Kind,
    text: String,
    until: Instant,
}

/// The notes on screen now.
#[derive(Debug, Default)]
pub struct Toasts(Vec<Toast>);

impl Toasts {
    pub fn add(&mut self, kind: Kind, text: impl Into<String>) {
        let shown = if kind == Kind::Failure {
            FAILURE_SHOWN
        } else {
            SHOWN
        };
        self.0.push(Toast {
            kind,
            text: text.into(),
            until: Instant::now() + shown,
        });
        if self.0.len() > MOST {
            self.0.remove(0);
        }
    }

    /// Draws them, oldest on top; a click dismisses one.
    pub fn show(&mut self, ctx: &egui::Context) {
        let now = Instant::now();
        self.0.retain(|t| t.until > now);
        if self.0.is_empty() {
            return;
        }
        let mut dismissed = None;
        // One area each, stacked up from the bottom right corner, newest lowest.
        let mut offset = egui::vec2(-16.0, -36.0);
        for (i, toast) in self.0.iter().enumerate().rev() {
            let shown = egui::Area::new(egui::Id::new(("nervros_toast", i)))
                .anchor(Align2::RIGHT_BOTTOM, offset)
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    let t = ui.tokens();
                    let (colour, icon) = match toast.kind {
                        Kind::Success => (t.success_text_color, &icons::SUCCESS),
                        Kind::Failure => (t.error_fg_color, &icons::ERROR),
                        Kind::Info => (t.info_text_color, &icons::INFO),
                    };
                    Frame::new()
                        .fill(t.panel_bg_color)
                        .stroke(Stroke::new(1.0, colour))
                        .corner_radius(CornerRadius::same(8))
                        .inner_margin(Margin::symmetric(12, 8))
                        .show(ui, |ui| {
                            ui.set_max_width(360.0);
                            ui.horizontal(|ui| {
                                ui.small_icon(icon, Some(colour));
                                ui.add(egui::Label::new(RichText::new(&toast.text)).wrap());
                            });
                        })
                        .response
                })
                .inner;
            if shown.interact(egui::Sense::click()).clicked() {
                dismissed = Some(i);
            }
            offset.y -= shown.rect.height() + 6.0;
        }
        if let Some(i) = dismissed {
            self.0.remove(i);
        }
        ctx.request_repaint_after(Duration::from_millis(500));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_newest_few_stay_and_a_failure_stays_longest() {
        let mut toasts = Toasts::default();
        for n in 0..=MOST {
            toasts.add(Kind::Success, format!("done {n}"));
        }
        toasts.add(Kind::Failure, "failed");
        assert_eq!(toasts.0.len(), MOST);
        assert_eq!(toasts.0[0].text, "done 2");
        let longest = toasts.0.iter().max_by_key(|t| t.until).unwrap();
        assert_eq!(longest.kind, Kind::Failure);
    }

    #[test]
    fn snapshot_toasts() {
        let mut toasts = Toasts::default();
        toasts.add(Kind::Success, "Done: bring the mug to the tray in 94 s");
        toasts.add(
            Kind::Failure,
            "failure: put the cup away at s3: the grasp slipped, and the cup is on the floor",
        );
        toasts.add(Kind::Info, "Stopped: patrol the kitchen");
        let mut harness = egui_kittest::Harness::builder()
            .wgpu()
            .with_size(egui::vec2(440.0, 260.0))
            .build_ui(move |ui| toasts.show(ui.ctx()));
        crate::chat::style_for_tests(&harness.ctx);
        harness.run_steps(2);
        crate::chat::compare(
            &mut harness,
            "toasts",
            &egui_kittest::SnapshotOptions::new(),
        );
    }
}
