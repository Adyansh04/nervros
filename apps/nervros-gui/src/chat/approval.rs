//! Approval cards: what the agent asks to do, with its buttons and the time left to answer.

use std::cell::Cell;
use std::time::Duration;

use nervros_core::session::{Command, Unanswered};
use rerun::external::egui;
use rerun::external::egui::{Align, Layout, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _, icons};

use super::plan::minutes;
use super::view::{card, code, pretty, spinner};
use super::{Action, Approval, PlanCard};
use crate::plan_edit;
use crate::plan_edit::EditStep;

pub(super) fn approval_card(
    ui: &mut egui::Ui,
    a: &Approval,
    plan: Option<&PlanCard>,
    ttl: Duration,
    actions: &mut Vec<Action>,
) {
    let t = ui.tokens();
    let stroke = if a.answer.is_none() {
        t.warn_fg_color
    } else {
        t.widget_noninteractive_bg_stroke
    };
    // An edit in progress lives in egui's memory under the plan it started from: once an edit
    // passes, the approval asks about a new plan and the editor closes by itself.
    let key = plan.map(|p| egui::Id::new(("plan_edit", a.id, p.hash.as_str())));
    let mut editing: Option<Vec<EditStep>> = key
        .filter(|_| a.answer.is_none())
        .and_then(|k| ui.data(|d| d.get_temp(k)));
    card(ui, stroke).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.small_icon(&icons::WARNING, Some(t.warn_fg_color));
            let title = match plan {
                Some(p) => format!("Run \"{}\"?", p.intent),
                None => format!("Approve {}?", a.tool),
            };
            ui.label(RichText::new(title).strong());
        });
        match (plan, editing.as_mut(), key) {
            (Some(_), Some(steps), Some(key)) => {
                ui.label(
                    RichText::new(
                        "Change the steps; the plan is checked again before you approve it",
                    )
                    .small()
                    .color(t.text_subdued),
                );
                plan_edit::editor(ui, key, steps);
            }
            (Some(p), ..) => {
                // The plan's card above lists the steps and follows them; this only asks.
                let short = p.hash.get(..8).unwrap_or(&p.hash);
                let steps = match p.steps.len() {
                    1 => "1 step".to_owned(),
                    n => format!("{n} steps"),
                };
                ui.label(
                    RichText::new(format!(
                        "plan {short} · {steps} · at most {}",
                        minutes(p.worst_case_s)
                    ))
                    .small()
                    .color(t.text_subdued),
                );
            }
            (None, ..) => {
                ui.label(&a.reason);
                code(ui, &pretty(&a.args));
            }
        }
        if let Some(problem) = a.problem.as_deref().filter(|_| a.answer.is_none()) {
            ui.add(egui::Label::new(RichText::new(problem).small().color(t.error_fg_color)).wrap());
        }
        ui.horizontal(|ui| match a.answer {
            Some(true) => {
                ui.label(RichText::new("Approved").color(t.success_text_color));
            }
            Some(false) => {
                ui.label(RichText::new("Denied").color(t.text_subdued));
            }
            None => {
                if a.checking {
                    spinner(ui);
                    ui.label(RichText::new("Checking the edit…").color(t.text_subdued));
                } else {
                    let left = time_left(ttl.saturating_sub(a.asked.elapsed()));
                    ui.label(RichText::new(left).color(t.text_subdued));
                }
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    approval_buttons(ui, a, plan, &mut editing, actions);
                });
                ui.ctx().request_repaint_after(Duration::from_secs(1));
            }
        });
    });
    if let Some(key) = key {
        ui.data_mut(|d| match editing {
            Some(steps) => {
                d.insert_temp(key, steps);
            }
            None => d.remove::<Vec<EditStep>>(key),
        });
    }
}

/// A request the last session ended waiting on: asked again only if the operator wants it, and
/// checked again then, since the robot and its world have moved on.
pub(super) fn unanswered_card(
    ui: &mut egui::Ui,
    u: &Unanswered,
    done: &Cell<bool>,
    actions: &mut Vec<Action>,
) {
    let t = ui.tokens();
    card(ui, t.widget_noninteractive_bg_stroke).show(ui, |ui| {
        ui.label(RichText::new("Waiting for your approval when the app last closed").strong());
        ui.label(&u.reason);
        // A plan's reason names it already; anything else shows what it would send.
        if u.args.get("steps").is_none() {
            code(ui, &pretty(&u.args));
        }
        ui.horizontal(|ui| {
            if done.get() {
                ui.label(RichText::new("Done").color(t.text_subdued));
                return;
            }
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui
                    .add(ReButton::new("Ask again").primary())
                    .on_hover_text("Check it again now and ask for your approval")
                    .clicked()
                {
                    actions.push(Action::Send(Command::Run {
                        tool: u.tool.clone(),
                        args: u.args.clone(),
                    }));
                    done.set(true);
                }
                if ui.add(ReButton::new("Dismiss").secondary()).clicked() {
                    done.set(true);
                }
            });
        });
    });
}

/// How long an approval has left: whole minutes while there are more than 90 s, as a count of
/// seconds would only distract, then seconds.
pub(super) fn time_left(left: Duration) -> String {
    let secs = left.as_secs_f32().ceil();
    if secs > 90.0 {
        format!("{} min left", (secs / 60.0).ceil())
    } else {
        format!("{secs} s left")
    }
}

/// Approve, deny, edit or apply the suggested fixes; while editing, check or cancel. Nothing
/// can be approved while an edit is checked: it would approve a plan about to change.
fn approval_buttons(
    ui: &mut egui::Ui,
    a: &Approval,
    plan: Option<&PlanCard>,
    editing: &mut Option<Vec<EditStep>>,
    actions: &mut Vec<Action>,
) {
    let idle = !a.checking;
    let edit = |args| Action::Send(Command::Edit { id: a.id, args });
    if let (Some(p), Some(steps)) = (plan, editing.as_ref()) {
        if ui
            .add_enabled(idle && !steps.is_empty(), ReButton::new("Check").primary())
            .clicked()
        {
            actions.push(edit(plan_edit::plan_args(&p.intent, steps)));
        }
        if ui.add(ReButton::new("Cancel").secondary()).clicked() {
            *editing = None;
        }
        return;
    }
    if ui
        .add_enabled(idle, ReButton::new("Approve").primary())
        .clicked()
    {
        actions.push(Action::Send(Command::Approve(a.id)));
    }
    if ui.add(ReButton::new("Deny").secondary()).clicked() {
        actions.push(Action::Send(Command::Deny(a.id)));
    }
    let Some(p) = plan else {
        if a.can_allow
            && ui
                .add_enabled(idle, ReButton::new("Allow for session").secondary())
                .on_hover_text(
                    "Approve, and let this tool run without asking for the rest of the session; \
                 what moves the robot still asks",
                )
                .clicked()
        {
            actions.push(Action::Send(Command::AllowForSession(a.id)));
        }
        return;
    };
    if ui
        .add_enabled(idle, ReButton::new("Edit").secondary())
        .on_hover_text("Change, move or drop steps before approving")
        .clicked()
    {
        *editing = Some(plan_edit::editable(&p.steps));
    }
    if let Some(fixed) = plan_edit::fixed(&p.steps, &p.concerns)
        && ui
            .add_enabled(idle, ReButton::new("Apply fixes").secondary())
            .on_hover_text(
                "Change the plan as its checks suggest; it is checked again before you approve it",
            )
            .clicked()
    {
        actions.push(edit(plan_edit::plan_args(&p.intent, &fixed)));
    }
}
