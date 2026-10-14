//! The Doctor tab's incidents: what went wrong lately, each with its likely causes, what they rest
//! on and the fix, what was ruled out and the minutes before it; and a check of the robot now.

use nervros_core::incident::Incident;
use nervros_core::session::Command;
use rerun::external::egui::{self, RichText};
use rerun::external::re_ui::ReButton;

use crate::chat::Action;
use crate::theme;

/// Incidents listed, the newest.
const SHOWN: usize = 12;
/// Timeline lines shown under an incident, the newest.
const TIMELINE: usize = 12;

/// The incidents, newest first. Returns whether "Check now" was clicked; `checking` shows that a
/// check runs.
pub fn show(
    ui: &mut egui::Ui,
    incidents: &[Incident],
    now_s: f64,
    checking: bool,
    actions: &mut Vec<Action>,
) -> bool {
    theme::section(ui, "Incidents");
    ui.label(
        RichText::new(
            "A failed mission, a watched topic going quiet, a node stopping or a burst of \
             errors opens one: the robot's last minutes, checked against what is known to go \
             wrong on it.",
        )
        .small()
        .color(theme::DIM),
    );
    ui.add_space(6.0);
    let mut check = false;
    if checking {
        ui.horizontal(|ui| {
            crate::chat::spinner(ui);
            ui.label("Checking the robot…");
        });
    } else {
        check = ui
            .add(ReButton::new("Check now").small().secondary())
            .on_hover_text("Runs every known check on the robot as it is, and keeps what it finds")
            .clicked();
    }
    ui.add_space(8.0);
    if incidents.is_empty() {
        ui.label(RichText::new("None yet.").color(theme::FAINT));
    }
    for (n, incident) in incidents.iter().rev().take(SHOWN).enumerate() {
        row(ui, incident, now_s, n == 0, actions);
    }
    check
}

fn row(
    ui: &mut egui::Ui,
    incident: &Incident,
    now_s: f64,
    newest: bool,
    actions: &mut Vec<Action>,
) {
    let ago = ago(now_s - incident.opened_s);
    let (colour, cause) = match incident.causes.first() {
        Some(c) => (theme::WARN, c.title.clone()),
        None => (theme::FAINT, "no known cause matched".to_owned()),
    };
    let label = RichText::new(format!("{} · {ago} · {cause}", incident.id)).color(colour);
    let id = ui.id().with(("incident", &incident.id));
    // The newest opens by itself, as what the operator came to read; a toggle of theirs stays.
    if newest {
        egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), id, true)
            .store(ui.ctx());
    }
    theme::disclosure(ui, id, label, |ui| {
        ui.label(RichText::new(&incident.trigger).color(theme::DIM));
        ui.add_space(4.0);
        for c in &incident.causes {
            theme::status_card(ui, theme::WARN, |ui| {
                ui.set_width(ui.available_width());
                ui.label(RichText::new(&c.title).strong());
                ui.label(&c.verdict);
                for e in &c.evidence {
                    ui.label(RichText::new(e).small().color(theme::DIM));
                }
                ui.label(RichText::new(format!("Fix: {}", c.fix)).color(theme::TEXT));
            });
            ui.add_space(4.0);
        }
        if !incident.ruled_out.is_empty() {
            let titles: Vec<&str> = incident
                .ruled_out
                .iter()
                .map(|v| v.title.as_str())
                .collect();
            ui.label(
                RichText::new(format!("Ruled out: {}", titles.join(", ")))
                    .small()
                    .color(theme::FAINT),
            )
            .on_hover_text(
                incident
                    .ruled_out
                    .iter()
                    .map(|v| format!("{}: {}", v.title, v.evidence.join("; ")))
                    .collect::<Vec<_>>()
                    .join("\n"),
            );
        }
        if !incident.timeline.is_empty() {
            ui.add_space(4.0);
            let skip = incident.timeline.len().saturating_sub(TIMELINE);
            for line in &incident.timeline[skip..] {
                ui.label(RichText::new(line).monospace().size(11.5).color(theme::DIM));
            }
        }
        ui.add_space(4.0);
        if ui
            .add(ReButton::new("Ask the agent").small().secondary())
            .on_hover_text("The agent explains it from the evidence, in the chat")
            .clicked()
        {
            actions.push(Action::Send(Command::User(format!(
                "Explain incident {}: what went wrong, and what should I do?",
                incident.id
            ))));
        }
    });
}

fn ago(seconds: f64) -> String {
    match seconds {
        s if s < 90.0 => format!("{s:.0} s ago"),
        s if s < 5400.0 => format!("{:.0} min ago", s / 60.0),
        s => format!("{:.0} h ago", s / 3600.0),
    }
}

#[cfg(test)]
mod tests {
    use nervros_core::incident::{Kind, Verdict};

    use super::*;

    #[test]
    fn snapshot_incidents() {
        let verdict = |title: &str, evidence: &str| Verdict {
            playbook: title.to_lowercase(),
            title: title.to_owned(),
            verdict: "AMCL settled far from where the robot stands, so every walk ends somewhere \
                      else."
                .to_owned(),
            fix: "Set the robot's pose again (2D Pose Estimate), or restart the stack.".to_owned(),
            procedure: String::new(),
            evidence: vec![evidence.to_owned()],
        };
        let incidents = vec![
            Incident {
                id: "i1".into(),
                opened_s: 400.0,
                kind: Kind::TopicSilent,
                trigger: "head camera (/head_camera/color/image_raw) went quiet".into(),
                mission: String::new(),
                causes: Vec::new(),
                ruled_out: vec![verdict("Stale shared memory", "head camera: 0.0 Hz")],
                unchecked: Vec::new(),
                timeline: vec!["-4 s head camera went quiet: 0.0 Hz, from 15.0 Hz".into()],
            },
            Incident {
                id: "i2".into(),
                opened_s: 880.0,
                kind: Kind::MissionFailed,
                trigger: "mission m7 ended, failure at s1: the goal was aborted".into(),
                mission: "m7".into(),
                causes: vec![verdict(
                    "The robot is lost on the map",
                    "map to pelvis: 1.24 m and 91 degrees from /g1_sensor_relay/base_state",
                )],
                ruled_out: vec![verdict("The robot fell", "tilt_deg is 2.1")],
                unchecked: Vec::new(),
                timeline: vec![
                    "-48 s mission m7 started".into(),
                    "-20 s WARN controller_server: Failed to make progress (x3)".into(),
                    "-2 s mission m7 ended, failure at s1: the goal was aborted".into(),
                ],
            },
        ];
        let mut harness = egui_kittest::Harness::builder()
            .wgpu()
            .with_size(egui::vec2(420.0, 600.0))
            .build_ui(move |ui| {
                egui::Frame::new()
                    .fill(theme::BG)
                    .inner_margin(egui::Margin::same(12))
                    .show(ui, |ui| {
                        show(ui, &incidents, 1000.0, false, &mut Vec::new())
                    });
            });
        crate::testkit::style_for_tests(&harness.ctx);
        harness.run_steps(3);
        harness.fit_contents();
        crate::testkit::compare(
            &mut harness,
            "incidents",
            &egui_kittest::SnapshotOptions::new(),
        );
    }
}
