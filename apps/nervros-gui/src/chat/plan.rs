//! Plan cards: a plan's steps with their live state, its behaviour tree, and how each step went before.

use std::fmt::Write as _;
use std::time::Duration;

use nervros_core::mission::Outcome;
use nervros_core::mission::plan::PlannedStep;
use nervros_core::session::Command;
use rerun::external::egui;
use rerun::external::egui::{Align, Color32, Layout, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _, icons};

use super::view::card;
use super::{Action, PlanCard};

/// The plan's steps with each one's state, how it has gone before, and what the preview warns of.
fn steps_table(ui: &mut egui::Ui, p: &PlanCard) {
    let t = ui.tokens();
    egui::Grid::new(("plan_steps", &p.hash))
        .num_columns(3)
        .spacing([8.0, 4.0])
        .show(ui, |ui| {
            for s in &p.steps {
                let (status, node) = p.progress.get(&s.id).cloned().unwrap_or_default();
                let failed = p.finished.as_ref().is_some_and(|f| f.1 == s.id);
                let (color, mark) = if failed {
                    (t.error_fg_color, "✗")
                } else {
                    node_look(ui, &status)
                };
                ui.label(RichText::new(mark).color(color));
                ui.label(
                    RichText::new(&s.id)
                        .monospace()
                        .size(12.0)
                        .color(t.text_subdued),
                );
                let text = if status == "running" && !node.is_empty() {
                    format!("{} · {node}", s.summary)
                } else {
                    s.summary.clone()
                };
                let preview_note = p
                    .preview
                    .iter()
                    .find(|v| v.id == s.id)
                    .map(|v| v.note.as_str())
                    .filter(|n| !n.is_empty());
                ui.horizontal(|ui| {
                    let summary = ui.label(RichText::new(text).monospace().size(12.0));
                    if let Some(note) = preview_note {
                        summary.on_hover_text(note);
                    }
                    track_label(ui, s);
                });
                ui.end_row();
                // Only a warning earns a line of its own; the rest is in the hover.
                let warnings = preview_note
                    .filter(|n| n.starts_with("no path"))
                    .into_iter()
                    .chain(
                        p.concerns
                            .iter()
                            .filter(|c| c.step == s.id)
                            .map(|c| c.message.as_str()),
                    );
                for warning in warnings {
                    ui.label("");
                    ui.label("");
                    let text = RichText::new(warning).small().color(t.warn_fg_color);
                    ui.add(egui::Label::new(text).wrap());
                    ui.end_row();
                }
            }
        });
}

/// A step's or node's status as a coloured mark.
fn node_look(ui: &egui::Ui, status: &str) -> (Color32, &'static str) {
    let t = ui.tokens();
    match status {
        "failure" => (t.error_fg_color, "✗"),
        "success" => (t.success_text_color, "✓"),
        "running" => (t.info_text_color, "●"),
        "skipped" => (t.text_subdued, "–"),
        _ => (t.text_subdued, "○"),
    }
}

/// The mission's behaviour tree as it has run so far, as the executor reports its nodes: each
/// under the subtrees it is in, marked with its latest status.
pub fn tree_panel(ui: &mut egui::Ui, p: &PlanCard) {
    if p.nodes.is_empty() {
        return;
    }
    ui.add_space(8.0);
    egui::CollapsingHeader::new(RichText::new("Behaviour tree").strong())
        .id_salt(("tree", &p.hash))
        .default_open(true)
        .show(ui, |ui| {
            for (path, status) in &p.nodes {
                let depth = u16::try_from(path.matches('/').count()).unwrap_or(u16::MAX);
                let name = path.rsplit('/').next().unwrap_or(path);
                // BehaviorTree.CPP names an unnamed node by its type and uid, as "Sequence::3".
                let name = name.split_once("::").map_or(name, |(n, _)| n);
                let (colour, mark) = node_look(ui, status);
                ui.horizontal(|ui| {
                    ui.add_space(14.0 * f32::from(depth));
                    ui.label(RichText::new(mark).color(colour));
                    ui.label(RichText::new(name).monospace().size(12.0))
                        .on_hover_text(format!("{path}: {status}"));
                });
            }
        });
}

/// How a step has gone before, small and quiet, with the details in its hover.
fn track_label(ui: &mut egui::Ui, s: &PlannedStep) {
    let Some(track) = &s.track else {
        return;
    };
    let t = ui.tokens();
    let mut text = format!("{} of {}", track.succeeded, track.runs);
    if let Some(typical) = track.typical_s {
        let _ = write!(text, " · {}", minutes(typical));
    }
    // Under three in four is worth a second look before approving.
    let color = if track.succeeded * 4 < track.runs * 3 {
        t.warn_fg_color
    } else {
        t.text_subdued
    };
    let mut hover = format!(
        "{} succeeded {} of its last {} runs",
        s.summary, track.succeeded, track.runs
    );
    if let Some(why) = &track.last_failure {
        let _ = write!(hover, "; it last failed because {why}");
    }
    ui.label(RichText::new(text).small().color(color))
        .on_hover_text(hover);
}

pub(super) fn minutes(seconds: f64) -> String {
    if seconds < 90.0 {
        format!("{seconds:.0} s")
    } else {
        format!("{:.0} min", seconds / 60.0)
    }
}

/// A plan and its mission: steps with live state, and one action while it runs.
pub fn plan_card(ui: &mut egui::Ui, p: &PlanCard, actions: &mut Vec<Action>) {
    let t = ui.tokens();
    let stroke = match &p.finished {
        Some((Outcome::Success, ..)) => t.success_text_color,
        Some(_) => t.error_fg_color,
        None if p.mission.is_some() => t.info_text_color,
        None => t.widget_noninteractive_bg_stroke,
    };
    card(ui, stroke).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.small_icon(&icons::PLAN_PENDING, Some(t.text_subdued));
            ui.label(RichText::new(format!("Plan · {}", p.intent)).strong());
        });
        steps_table(ui, p);
        for c in p.concerns.iter().filter(|c| c.step.is_empty()) {
            ui.horizontal(|ui| {
                ui.small_icon(&icons::WARNING, Some(t.warn_fg_color));
                ui.add(
                    egui::Label::new(RichText::new(&c.message).small().color(t.warn_fg_color))
                        .wrap(),
                );
            });
        }
        ui.horizontal(|ui| {
            let state = match (&p.finished, p.started) {
                (Some((Outcome::Success, .., secs)), _) => {
                    RichText::new(format!("Done in {}", minutes(*secs))).color(t.success_text_color)
                }
                (Some((outcome, step, reason, _)), _) => {
                    let at = if step.is_empty() {
                        String::new()
                    } else {
                        format!(" at {step}")
                    };
                    RichText::new(format!("{}{at}: {reason}", capitalise(outcome.as_str())))
                        .color(t.error_fg_color)
                }
                (None, Some(since)) => {
                    ui.ctx().request_repaint_after(Duration::from_secs(1));
                    RichText::new(format!(
                        "Running {}",
                        minutes(since.elapsed().as_secs_f64())
                    ))
                    .color(t.info_text_color)
                }
                (None, None) if p.replaced => {
                    RichText::new("Replaced by an edited plan").color(t.text_subdued)
                }
                (None, None) => RichText::new(format!(
                    "Checked · at most {} · waiting to run",
                    minutes(p.worst_case_s)
                ))
                .color(t.text_subdued),
            };
            ui.label(state.small());
            if p.mission.is_some() && p.finished.is_none() {
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.add(ReButton::new("Stop").small().secondary()).clicked() {
                        actions.push(Action::Send(Command::StopMission));
                    }
                });
            }
        });
    });
}

/// `word` with its first letter in capitals.
pub fn capitalise(word: &str) -> String {
    let mut c = word.chars();
    c.next()
        .map_or_else(String::new, |f| f.to_uppercase().chain(c).collect())
}
