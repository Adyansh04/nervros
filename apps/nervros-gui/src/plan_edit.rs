//! Editing a plan while it waits for approval: change a step's arguments, move or drop a step.
//! The session checks the edit as it checks the model's plans; until it passes, the plan that was
//! asked about stays the one that runs if approved.

use crate::theme;
use nervros_core::mission::plan::PlannedStep;
use nervros_core::mission::sanity::Concern;
use rerun::external::egui::{self, Align, CornerRadius, Frame, Layout, Margin, RichText};
use rerun::external::re_ui::{UiExt as _, icons};
use serde_json::{Value, json};

/// A step being edited. The planner's own walks are left out: it adds them again where needed.
#[derive(Debug, Clone, PartialEq)]
pub struct EditStep {
    skill: String,
    args: Vec<(String, String)>,
    retries: u8,
    optional: bool,
    timeout_s: f64,
}

/// The steps the operator can edit.
pub fn editable(steps: &[PlannedStep]) -> Vec<EditStep> {
    steps
        .iter()
        .filter(|s| !s.added)
        .map(|s| EditStep {
            skill: s.skill.clone(),
            args: s
                .args
                .iter()
                .map(|a| (a.name.clone(), a.value.clone()))
                .collect(),
            retries: s.retries,
            optional: s.optional,
            timeout_s: s.timeout_s,
        })
        .collect()
}

/// The plan with every concern's fix applied, or `None` when no concern has one. Fixes are
/// matched to steps by id before the planner's walks are dropped.
pub fn fixed(steps: &[PlannedStep], concerns: &[Concern]) -> Option<Vec<EditStep>> {
    if concerns.iter().all(|c| c.fix.is_none()) {
        return None;
    }
    let mut steps = steps.to_vec();
    for c in concerns {
        let Some(fix) = &c.fix else { continue };
        if let Some(arg) = steps
            .iter_mut()
            .find(|s| s.id == c.step)
            .and_then(|s| s.args.iter_mut().find(|a| a.name == fix.name))
        {
            arg.value.clone_from(&fix.value);
        }
    }
    Some(editable(&steps))
}

/// The edit as `run_mission`'s arguments.
pub fn plan_args(intent: &str, steps: &[EditStep]) -> Value {
    let steps: Vec<Value> = steps
        .iter()
        .map(|s| {
            json!({
                "skill": s.skill,
                "args": s.args.iter().map(|(name, value)| json!({"name": name, "value": value})).collect::<Vec<_>>(),
                "retries": s.retries,
                "optional": s.optional,
                "timeout_s": s.timeout_s,
            })
        })
        .collect();
    json!({"intent": intent, "steps": steps})
}

/// Draws each step with its arguments as fields, and buttons to move and drop it.
pub fn editor(ui: &mut egui::Ui, salt: egui::Id, steps: &mut Vec<EditStep>) {
    let mut moved: Option<(usize, usize)> = None;
    let mut dropped: Option<usize> = None;
    let last = steps.len().saturating_sub(1);
    for (i, step) in steps.iter_mut().enumerate() {
        Frame::new()
            .fill(theme::SURFACE)
            .corner_radius(CornerRadius::same(6))
            .inner_margin(Margin::symmetric(8, 6))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.horizontal(|ui| {
                    let number = RichText::new(format!("{}", i + 1)).monospace().size(12.0);
                    ui.label(number.color(theme::DIM));
                    ui.label(RichText::new(&step.skill).monospace().size(12.0).strong());
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        let drop = ui.small_icon_button_widget(&icons::TRASH, "Drop this step");
                        if ui.add_enabled(last > 0, drop).clicked() {
                            dropped = Some(i);
                        }
                        let down = ui.small_icon_button_widget(&icons::ARROW_DOWN, "Move down");
                        if ui.add_enabled(i < last, down).clicked() {
                            moved = Some((i, i + 1));
                        }
                        let up = ui.small_icon_button_widget(&icons::ARROW_UP, "Move up");
                        if ui.add_enabled(i > 0, up).clicked() {
                            moved = Some((i, i - 1));
                        }
                    });
                });
                if step.args.is_empty() {
                    return;
                }
                ui.horizontal_wrapped(|ui| {
                    for (name, value) in &mut step.args {
                        ui.label(RichText::new(name.as_str()).small().color(theme::DIM));
                        // As wide as the value, so "dining_table_side" and "90" both read whole.
                        let chars = u16::try_from(value.chars().count()).unwrap_or(u16::MAX);
                        let width = (f32::from(chars) * 7.5 + 16.0).clamp(48.0, 180.0);
                        ui.add(
                            egui::TextEdit::singleline(value)
                                .id_salt(salt.with((i, name.as_str())))
                                .desired_width(width)
                                .font(egui::TextStyle::Monospace),
                        );
                        ui.add_space(6.0);
                    }
                });
            });
    }
    if let Some((from, to)) = moved {
        steps.swap(from, to);
    }
    if let Some(i) = dropped {
        steps.remove(i);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nervros_core::mission::plan::StepArg;

    fn step(id: &str, skill: &str, args: &[(&str, &str)], added: bool) -> PlannedStep {
        PlannedStep {
            id: id.to_owned(),
            skill: skill.to_owned(),
            args: args
                .iter()
                .map(|(n, v)| StepArg {
                    name: (*n).to_owned(),
                    value: (*v).to_owned(),
                })
                .collect(),
            timeout_s: 30.0,
            added,
            ..PlannedStep::default()
        }
    }

    #[test]
    fn fixes_land_on_their_steps_and_the_planners_walks_are_left_to_it() {
        let steps = [
            step("s1", "GoToPlace", &[("place", "near_O18")], true),
            step("s2", "TurnInPlace", &[("degrees", "-90")], false),
        ];
        let concerns = [Concern {
            step: "s2".to_owned(),
            message: "turns the other way".to_owned(),
            fix: Some(StepArg {
                name: "degrees".to_owned(),
                value: "90".to_owned(),
            }),
        }];
        let edit = fixed(&steps, &concerns).unwrap();
        assert_eq!(edit.len(), 1);
        assert_eq!(
            plan_args("turn left", &edit),
            json!({"intent": "turn left", "steps": [{"skill": "TurnInPlace",
                "args": [{"name": "degrees", "value": "90"}], "retries": 0, "optional": false,
                "timeout_s": 30.0}]})
        );
        let no_fix = [Concern {
            fix: None,
            ..concerns[0].clone()
        }];
        assert_eq!(fixed(&steps, &no_fix), None);
    }
}
