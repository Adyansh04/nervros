//! Job cards: a robot program or a search the operator approved once, with what it may do, its
//! latest lines and how it ended.

use std::collections::VecDeque;

use nervros_core::job::{JobInfo, JobState};
use rerun::external::egui;
use rerun::external::egui::RichText;
use rerun::external::re_ui::ReButton;

use super::Action;
use crate::theme;

/// Lines of its log a card keeps; the rest are in the session log.
const LINES: usize = 8;

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
        ui.horizontal(|ui| {
            let kind = match job.kind.as_str() {
                "program" => "Program",
                _ => "Search",
            };
            ui.label(RichText::new(kind).strong().size(14.0));
            ui.label(RichText::new(&job.name).size(14.0));
            ui.label(RichText::new(format!("{} · {state}", job.id)).color(colour));
            if job.state == JobState::Running {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
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
            ui.add(egui::Label::new(RichText::new(&job.summary).size(13.0)).wrap());
        }
    });
}
