//! What the mission ledger keeps, in the Mission tab: the latest missions, each to open or run
//! again; plans saved by name, to run or forget; and the requests no skill could do.

use std::sync::Arc;
use std::time::{Duration, Instant};

use nervros_core::mission::ledger::{Gap, Ledger, MissionRecord, Template};
use nervros_core::session::Command;
use rerun::external::egui::{self, Align, Layout, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _};
use serde_json::{Value, json};

use crate::chat::Action;
use crate::theme;

/// How many of each list the tab shows.
const SHOWN: usize = 8;
/// The ledger is read again after this, or at once after a mission ends.
const FRESH: Duration = Duration::from_secs(5);

/// The ledger's lists, read off the UI thread's hot path at most every few seconds.
#[derive(Default)]
pub struct History {
    read: Option<Instant>,
    missions: Vec<MissionRecord>,
    templates: Vec<Template>,
    gaps: Vec<Gap>,
    error: Option<String>,
}

impl History {
    /// Reads the ledger again on the next draw, as after a mission ends.
    pub fn stale(&mut self) {
        self.read = None;
    }

    fn refresh(&mut self, ledger: &Ledger) {
        if self.read.is_some_and(|at| at.elapsed() < FRESH) {
            return;
        }
        self.read = Some(Instant::now());
        match (ledger.recent(SHOWN), ledger.templates(), ledger.gaps(SHOWN)) {
            (Ok(missions), Ok(templates), Ok(gaps)) => {
                (self.missions, self.templates, self.gaps) = (missions, templates, gaps);
                self.error = None;
            }
            (Err(e), ..) | (_, Err(e), _) | (.., Err(e)) => self.error = Some(e.to_string()),
        }
    }
}

/// The plan of a recorded mission, to run it again as the operator's own.
fn again(m: &MissionRecord) -> Value {
    let steps: Vec<Value> = m
        .steps
        .iter()
        .map(|s| json!({"skill": s.skill, "args": s.args}))
        .collect();
    json!({"intent": m.intent, "steps": steps})
}

#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "seconds since 1970, positive and far below u64::MAX"
)]
fn ago(at_s: f64) -> String {
    crate::sessions::ago(at_s.max(0.0) as u64)
}

/// The three lists, or nothing when the robot keeps no ledger.
pub fn show(
    ui: &mut egui::Ui,
    history: &mut History,
    ledger: Option<&Arc<Ledger>>,
    actions: &mut Vec<Action>,
) {
    let Some(ledger) = ledger else {
        return;
    };
    history.refresh(ledger);
    if let Some(e) = &history.error {
        ui.warning_label(format!("The mission ledger cannot be read: {e}"));
    }
    missions(ui, &history.missions, actions);
    if let Some(name) = templates(ui, &history.templates, actions) {
        if let Err(e) = ledger.forget_template(&name) {
            history.error = Some(e.to_string());
        }
        history.stale();
    }
    gaps(ui, &history.gaps);
}

fn heading(ui: &mut egui::Ui, text: &str) {
    ui.add_space(16.0);
    theme::section(ui, text);
}

fn run(args: Value) -> Action {
    Action::Send(Command::Run {
        tool: "run_mission".to_owned(),
        args,
    })
}

fn missions(ui: &mut egui::Ui, list: &[MissionRecord], actions: &mut Vec<Action>) {
    heading(ui, "Recent missions");
    if list.is_empty() {
        ui.label(
            RichText::new("None yet: a mission is kept here when it ends.")
                .small()
                .color(theme::DIM),
        );
        return;
    }
    for m in list {
        let colour = match m.outcome.as_str() {
            "success" => ui.visuals().text_color(),
            "canceled" => theme::DIM,
            _ => theme::ERROR,
        };
        let header = format!(
            "{} · {} · {} · {:.0} s",
            m.intent,
            m.outcome,
            ago(m.started),
            m.ended - m.started
        );
        let id = ui.make_persistent_id(("mission", &m.id));
        let label = RichText::new(header).size(13.0).color(colour);
        theme::disclosure(ui, id, label, |ui| {
            if !m.request.is_empty() {
                ui.label(
                    RichText::new(format!("Asked: {}", m.request))
                        .small()
                        .color(theme::DIM),
                );
            }
            for s in &m.steps {
                let time = s
                    .seconds
                    .map_or_else(String::new, |x| format!(" · {x:.0} s"));
                let why = if s.reason.is_empty() {
                    String::new()
                } else {
                    format!(": {}", s.reason)
                };
                let line = format!("{} {} {}{time}{why}", s.id, s.skill, s.outcome);
                ui.label(RichText::new(line).monospace());
            }
            if ui
                .add(ReButton::new("Run again").small().secondary())
                .on_hover_text("Plan the same steps; you approve them before the robot moves")
                .clicked()
            {
                actions.push(run(again(m)));
            }
        })
        .on_hover_text(format!("mission {}", m.id));
    }
}

/// The saved plans; the name of one to forget, when asked.
fn templates(ui: &mut egui::Ui, list: &[Template], actions: &mut Vec<Action>) -> Option<String> {
    heading(ui, "Saved plans");
    if list.is_empty() {
        ui.label(
            RichText::new("None yet: ask the agent to save a plan that worked, by name.")
                .small()
                .color(theme::DIM),
        );
        return None;
    }
    let mut forget = None;
    for plan in list {
        ui.horizontal(|ui| {
            ui.label(RichText::new(&plan.name).strong());
            let runs = match plan.runs {
                0 => "never run".to_owned(),
                1 => "run once".to_owned(),
                n => format!("run {n} times"),
            };
            ui.label(
                RichText::new(format!("{} · {runs}", plan.intent))
                    .small()
                    .color(theme::DIM),
            );
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui
                    .add(ReButton::new("Forget").small().secondary())
                    .clicked()
                {
                    forget = Some(plan.name.clone());
                }
                if ui
                    .add(ReButton::new("Run").small().secondary())
                    .on_hover_text("You approve it before the robot moves")
                    .clicked()
                {
                    actions.push(run(json!({"template": plan.name})));
                }
            });
        });
    }
    forget
}

fn gaps(ui: &mut egui::Ui, list: &[Gap]) {
    heading(ui, "Skill gaps");
    if list.is_empty() {
        ui.label(
            RichText::new("None: every request so far had a skill for it.")
                .small()
                .color(theme::DIM),
        );
        return;
    }
    for g in list {
        let nearest = if g.nearest.is_empty() {
            String::new()
        } else {
            format!("; nearest: {}", g.nearest)
        };
        ui.label(RichText::new(format!("\"{}\"", g.request)).size(13.0));
        ui.label(
            RichText::new(format!("{}: {}{nearest}", ago(g.at), g.reason))
                .small()
                .color(theme::DIM),
        );
        ui.add_space(4.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nervros_core::mission::ledger::StepRecord;

    #[test]
    fn a_mission_runs_again_as_its_recorded_steps() {
        let m = MissionRecord {
            intent: "fetch the mug".to_owned(),
            steps: vec![StepRecord {
                id: "s1".to_owned(),
                skill: "PickObject".to_owned(),
                args: json!({"object_id": "O17", "arm": "left"}),
                ..StepRecord::default()
            }],
            ..MissionRecord::default()
        };
        assert_eq!(
            again(&m),
            json!({"intent": "fetch the mug", "steps": [
                {"skill": "PickObject", "args": {"object_id": "O17", "arm": "left"}}]})
        );
    }

    #[test]
    fn snapshot_history() {
        let ledger = Ledger::in_memory().unwrap();
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        let step = |id: &str, skill: &str, outcome: &str, seconds: f64, reason: &str| StepRecord {
            id: id.to_owned(),
            skill: skill.to_owned(),
            outcome: outcome.to_owned(),
            seconds: Some(seconds),
            reason: reason.to_owned(),
            ..StepRecord::default()
        };
        ledger
            .record(&MissionRecord {
                id: "m1".to_owned(),
                intent: "bring the mug to the tray".to_owned(),
                request: "Bring the small white mug to the tray".to_owned(),
                started: now - 3600.0,
                ended: now - 3506.0,
                outcome: "success".to_owned(),
                steps: vec![
                    step("s1", "GoToTarget", "success", 21.0, ""),
                    step("s2", "PickObject", "success", 44.0, ""),
                ],
                ..MissionRecord::default()
            })
            .unwrap();
        ledger
            .record(&MissionRecord {
                id: "m2".to_owned(),
                intent: "put the cup in the sink".to_owned(),
                started: now - 600.0,
                ended: now - 540.0,
                outcome: "failure".to_owned(),
                failed_step: "s2".to_owned(),
                reason: "the grasp slipped".to_owned(),
                steps: vec![step(
                    "s2",
                    "PickObject",
                    "failure",
                    31.0,
                    "the grasp slipped",
                )],
                ..MissionRecord::default()
            })
            .unwrap();
        ledger
            .save_template("evening check", "look at the kitchen", &json!([]))
            .unwrap();
        ledger
            .note_gap("open the fridge", "no skill opens doors", "PickObject")
            .unwrap();
        let mut history = History::default();
        let mut harness = egui_kittest::Harness::builder()
            .wgpu()
            .with_size(egui::vec2(380.0, 600.0))
            .build_ui(move |ui| {
                egui::Frame::new()
                    .fill(theme::BG)
                    .inner_margin(egui::Margin::same(12))
                    .show(ui, |ui| {
                        show(ui, &mut history, Some(&ledger), &mut Vec::new());
                    });
            });
        crate::testkit::style_for_tests(&harness.ctx);
        harness.run_steps(2);
        harness.fit_contents();
        crate::testkit::compare(
            &mut harness,
            "history",
            &egui_kittest::SnapshotOptions::new(),
        );
    }
}
