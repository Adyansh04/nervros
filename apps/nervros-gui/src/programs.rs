//! The Mission tab's programs: those running, each with Stop, and those kept by name, to run
//! again or forget.

use nervros_core::job::{JobKind, JobState, Jobs};
use nervros_core::program::Store;
use nervros_core::session::Command;
use rerun::external::egui::{self, RichText};
use rerun::external::re_ui::ReButton;
use serde_json::json;

use crate::chat::Action;
use crate::theme;

/// The running and kept programs.
pub fn show(ui: &mut egui::Ui, store: &Store, jobs: &Jobs, actions: &mut Vec<Action>) {
    ui.add_space(16.0);
    theme::section(ui, "Programs");
    let running: Vec<_> = jobs
        .list()
        .into_iter()
        .filter(|j| j.kind == JobKind::Program && j.state == JobState::Running)
        .collect();
    for job in &running {
        theme::row(
            ui,
            |ui| {
                if ui.add(ReButton::new("Stop").small()).clicked() {
                    actions.push(Action::StopJob(job.id.clone()));
                }
            },
            |ui| {
                ui.label(RichText::new(&job.name).strong());
                // What it may do is long: it shows on hover, so the row keeps to one line.
                ui.label(
                    RichText::new(format!("{} · running", job.id))
                        .small()
                        .color(theme::DIM),
                )
                .on_hover_text(format!("It {}", job.allows));
            },
        );
    }
    let kept = store.list();
    if kept.is_empty() && running.is_empty() {
        ui.label(
            RichText::new(
                "None yet: ask for something that takes counting or a loop, such as \"count \
                 the mugs in every room\", and keep the program that did it by name.",
            )
            .small()
            .color(theme::DIM),
        );
        return;
    }
    for program in kept {
        let runs = match program.runs {
            0 => "never run".to_owned(),
            1 => "ran once".to_owned(),
            n => format!("ran {n} times"),
        };
        theme::row(
            ui,
            |ui| {
                if ui
                    .add(ReButton::new("Forget").small().secondary())
                    .clicked()
                {
                    let _ = store.forget(&program.name);
                }
                if ui
                    .add(ReButton::new("Run").small().secondary())
                    .on_hover_text("You approve it, with what it may run, before it starts")
                    .clicked()
                {
                    actions.push(Action::Send(Command::Run {
                        tool: "program".to_owned(),
                        args: json!({"action": "run", "name": program.name}),
                    }));
                }
            },
            |ui| {
                ui.label(RichText::new(&program.name).strong());
                let lines = program
                    .source
                    .lines()
                    .filter(|l| !l.trim().is_empty())
                    .count();
                let lines = if lines == 1 {
                    "1 line".to_owned()
                } else {
                    format!("{lines} lines")
                };
                ui.label(
                    RichText::new(format!(
                        "{lines} · at most {} min · {runs}",
                        program.max_minutes
                    ))
                    .small()
                    .color(theme::DIM),
                )
                .on_hover_text(&program.source);
            },
        );
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use nervros_core::guard::{Guard, Policy};
    use nervros_core::job::{Approval, Envelope};
    use nervros_core::program::Saved;

    use super::*;

    #[test]
    fn snapshot_programs() {
        let runtime = tokio::runtime::Runtime::new().unwrap();
        let _inside = runtime.enter();
        let jobs = Jobs::new(Arc::new(Guard::new(Policy::default())));
        let approval = Approval {
            envelope: Envelope::new(["GoToPlace".to_owned(), "FindInView".to_owned()], 10),
            moves: true,
            text: "may move the base (GoToPlace) and look (FindInView), for at most 10 min"
                .to_owned(),
        };
        jobs.spawn(
            JobKind::Program,
            "look for a tray in each room",
            approval,
            |ctx| async move {
                ctx.until_stopped().await;
                Ok(String::new())
            },
        )
        .unwrap();
        let store = Store::open(None);
        store
            .save(Saved {
                name: "count chairs".to_owned(),
                source: "let n = #{};\nfor r in rooms() { n[r.name] = objects(\"chair\", r.id).len(); }\nn"
                    .to_owned(),
                max_minutes: 10,
                saved_s: 0.0,
                runs: 3,
            })
            .unwrap();
        let mut harness = egui_kittest::Harness::builder()
            .wgpu()
            .with_size(egui::vec2(380.0, 300.0))
            .build_ui(move |ui| {
                egui::Frame::new()
                    .fill(theme::BG)
                    .inner_margin(egui::Margin::same(12))
                    .show(ui, |ui| show(ui, &store, &jobs, &mut Vec::new()));
            });
        crate::testkit::style_for_tests(&harness.ctx);
        harness.run_steps(2);
        harness.fit_contents();
        crate::testkit::compare(
            &mut harness,
            "programs",
            &egui_kittest::SnapshotOptions::new(),
        );
    }
}
