//! Job cards: a robot program, a search or a patrol the operator approved once, with what it may
//! do, its latest lines and how it ended.

use std::collections::VecDeque;

use nervros_core::job::{JobInfo, JobKind, JobState};
use rerun::external::egui;
use rerun::external::egui::RichText;
use rerun::external::re_ui::ReButton;
use serde_json::Value;

use super::Action;
use crate::theme;

/// Lines of its log a card keeps; the rest are in the session log.
const LINES: usize = 8;
/// What a card's Stop button takes of its first line.
const STOP_ROOM: f32 = 56.0;

/// A job, updated as it runs.
#[derive(Debug, Clone)]
pub struct JobCard {
    pub(super) job: JobInfo,
    lines: VecDeque<String>,
}

impl JobCard {
    pub(super) fn new(job: JobInfo) -> Self {
        Self {
            job,
            lines: VecDeque::new(),
        }
    }

    /// Notes a line of what it did.
    pub(super) fn log(&mut self, line: &str) {
        if self.lines.len() == LINES {
            self.lines.pop_front();
        }
        self.lines.push_back(line.to_owned());
    }

    /// Takes the job as it ended.
    pub(super) fn ended(&mut self, job: &JobInfo) {
        self.job = job.clone();
    }
}

/// The card: what kind of job, what for, what it may do, its latest lines, and Stop while it runs.
pub(super) fn job_card(ui: &mut egui::Ui, card: &JobCard, actions: &mut Vec<Action>) {
    let job = &card.job;
    let (colour, state) = match job.state {
        JobState::Running => (theme::ACCENT, "running"),
        JobState::Done => (theme::SUCCESS, "done"),
        JobState::Failed => (theme::ERROR, "failed"),
        JobState::Stopped => (theme::FAINT, "stopped"),
        JobState::TimedOut => (theme::WARN, "out of time"),
    };
    theme::status_card(ui, colour, |ui| {
        ui.set_width(ui.available_width());
        let kind = match job.kind {
            JobKind::Program => "Program",
            JobKind::Search => "Search",
            JobKind::Patrol => "Patrol",
        };
        let header = |ui: &mut egui::Ui| {
            ui.label(RichText::new(kind).strong().size(14.0));
            ui.label(RichText::new(&job.name).size(14.0));
            ui.label(RichText::new(format!("{} · {state}", job.id)).color(colour));
        };
        let running = job.state == JobState::Running;
        ui.horizontal_top(|ui| {
            // The name wraps in the room Stop leaves it, so a long one never squeezes the button.
            let room = ui.available_width() - if running { STOP_ROOM } else { 0.0 };
            ui.allocate_ui_with_layout(
                egui::vec2(room.max(0.0), 0.0),
                egui::Layout::left_to_right(egui::Align::Min).with_main_wrap(true),
                header,
            );
            if running {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if ui.add(ReButton::new("Stop").small()).clicked() {
                        actions.push(Action::StopJob(job.id.clone()));
                    }
                });
            }
        });
        if !job.allows.is_empty() {
            ui.label(
                RichText::new(format!("It {}", job.allows))
                    .size(12.5)
                    .color(theme::DIM),
            );
        }
        if !card.lines.is_empty() {
            ui.add_space(4.0);
            for line in &card.lines {
                let text = RichText::new(line).monospace().size(12.5).color(theme::DIM);
                ui.add(egui::Label::new(text).wrap());
            }
        }
        if !job.summary.is_empty() {
            ui.add_space(4.0);
            // The result is what the card is for: it reads first.
            let result = RichText::new(readable(&job.summary))
                .size(13.5)
                .color(theme::TEXT);
            ui.add(egui::Label::new(result).wrap());
        }
    });
}

/// A job's result as the operator reads it: a program's counts by room read "bedroom 2 ·
/// kitchen 4", a list its items; anything else as the job said it.
pub(crate) fn readable(summary: &str) -> String {
    let flat = |v: &Value| !v.is_object() && !v.is_array();
    let text = |v: &Value| v.as_str().map_or_else(|| v.to_string(), str::to_owned);
    match serde_json::from_str::<Value>(summary) {
        Ok(Value::Object(map)) if !map.is_empty() && map.values().all(flat) => map
            .iter()
            .map(|(k, v)| format!("{k} {}", text(v)))
            .collect::<Vec<_>>()
            .join(" · "),
        Ok(Value::Array(items)) if !items.is_empty() && items.iter().all(flat) => {
            items.iter().map(text).collect::<Vec<_>>().join(", ")
        }
        _ => summary.to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_programs_counts_read_as_words() {
        assert_eq!(
            readable(r#"{"bedroom":2,"living room":3}"#),
            "bedroom 2 · living room 3"
        );
        assert_eq!(readable(r#"["O1","O7"]"#), "O1, O7");
        assert_eq!(readable("found keys at R2"), "found keys at R2");
        assert_eq!(
            readable(r#"[{"id":"O1"}]"#),
            r#"[{"id":"O1"}]"#,
            "nested stays as it is"
        );
    }
}
