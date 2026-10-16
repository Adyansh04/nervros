//! The Mission tab's taught skills: each skill the agent taught the robot, whether it has proven
//! itself yet, what it was taught for, and a way to forget it.

use nervros_core::mission::catalog::Catalog;
use nervros_core::session::Command;
use rerun::external::egui::{self, RichText};
use rerun::external::re_ui::ReButton;
use serde_json::json;

use crate::chat::Action;
use crate::theme;

/// The taught skills, when the robot's executor takes taught skills.
pub fn show(ui: &mut egui::Ui, catalog: &Catalog, actions: &mut Vec<Action>) {
    if catalog.learnable.is_none() {
        return;
    }
    ui.add_space(16.0);
    theme::section(ui, "Taught skills");
    let taught: Vec<_> = catalog
        .skills
        .iter()
        .filter(|s| !s.learned.is_empty())
        .collect();
    if taught.is_empty() {
        ui.label(
            RichText::new(
                "None yet: ask for something no skill does, such as \"point at the red mug\", \
                 and the agent can teach the robot a skill for it.",
            )
            .small()
            .color(theme::DIM),
        );
        return;
    }
    for skill in taught {
        // A draft may have been tried and failed: it is kept only once a mission with it works.
        let state = if skill.learned == "draft" {
            "not kept yet"
        } else {
            "kept"
        };
        theme::row(
            ui,
            |ui| {
                if ui
                    .add(ReButton::new("Forget").small().secondary())
                    .on_hover_text("You approve it before the robot forgets the skill")
                    .clicked()
                {
                    actions.push(Action::Send(Command::Run {
                        tool: "teach_skill".to_owned(),
                        args: json!({"action": "forget", "name": skill.name}),
                    }));
                }
            },
            |ui| {
                ui.label(RichText::new(&skill.name).strong());
                let why = if skill.taught_for.is_empty() {
                    state.to_owned()
                } else {
                    format!("{state} · taught for \"{}\"", skill.taught_for)
                };
                ui.label(RichText::new(why).small().color(theme::DIM))
                    .on_hover_text(&skill.description);
            },
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapshot_taught() {
        let catalog = Catalog::parse(
            r#"{"catalog_version": "v1", "skills": [
              {"name": "GoToTarget", "max_duration_s": 120, "template": ""},
              {"name": "PointAt", "description": "Points a hand at an object.", "max_duration_s": 30,
               "template": "", "learned": "kept", "taught_for": "point at the red mug"},
              {"name": "WaveHello", "description": "Waves the right hand.", "max_duration_s": 20,
               "template": "", "learned": "draft", "taught_for": "wave at me"}],
             "learnable": {"leaves": {}, "max_duration_s": 300, "max_nodes": 40}}"#,
        )
        .unwrap();
        let mut harness = egui_kittest::Harness::builder()
            .wgpu()
            .with_size(egui::vec2(380.0, 300.0))
            .build_ui(move |ui| {
                egui::Frame::new()
                    .fill(theme::BG)
                    .inner_margin(egui::Margin::same(12))
                    .show(ui, |ui| show(ui, &catalog, &mut Vec::new()));
            });
        crate::testkit::style_for_tests(&harness.ctx);
        harness.run_steps(2);
        harness.fit_contents();
        crate::testkit::compare(
            &mut harness,
            "taught",
            &egui_kittest::SnapshotOptions::new(),
        );
    }
}
