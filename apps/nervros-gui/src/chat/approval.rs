//! Approval cards: what the agent asks to do, with its buttons and the time left to answer.

use std::cell::Cell;
use std::time::Duration;

use nervros_core::session::{Command, Unanswered};
use rerun::external::egui;
use rerun::external::egui::{Align, CornerRadius, Frame, Layout, Margin, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _, icons};

use super::plan::minutes;
use super::view::{code, pretty, spinner};
use super::{Action, Approval, PlanCard};
use crate::plan_edit::EditStep;
use crate::{plan_edit, theme};

pub(super) fn approval_card(
    ui: &mut egui::Ui,
    a: &Approval,
    plan: Option<&PlanCard>,
    ttl: Duration,
    actions: &mut Vec<Action>,
) {
    // An edit in progress lives in egui's memory under the plan it started from: once an edit
    // passes, the approval asks about a new plan and the editor closes by itself.
    let key = plan.map(|p| egui::Id::new(("plan_edit", a.id, p.hash.as_str())));
    let mut editing: Option<Vec<EditStep>> = key
        .filter(|_| a.answer.is_none())
        .and_then(|k| ui.data(|d| d.get_temp(k)));
    match a.answer {
        Some(approved) => answered(ui, a, plan, approved),
        None => theme::status_card(ui, theme::WARN, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.small_icon(&icons::WARNING, Some(theme::WARN));
                let title = match plan {
                    Some(p) => format!("Run \"{}\"?", p.intent),
                    None => format!("Approve {}?", a.tool),
                };
                ui.add(egui::Label::new(RichText::new(title).strong().size(14.5)).wrap());
            });
            match (plan, editing.as_mut(), key) {
                (Some(_), Some(steps), Some(key)) => {
                    ui.label(
                        RichText::new(
                            "Change the steps; the plan is checked again before you approve it",
                        )
                        .small()
                        .color(theme::DIM),
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
                            "{steps} · at most {} · plan {short}",
                            minutes(p.worst_case_s)
                        ))
                        .size(13.0)
                        .color(theme::DIM),
                    );
                }
                (None, ..) => {
                    ui.label(&a.reason);
                    code(ui, &pretty(&a.args));
                }
            }
            if let Some(problem) = a.problem.as_deref() {
                ui.add(
                    egui::Label::new(RichText::new(problem).size(13.0).color(theme::ERROR)).wrap(),
                );
            }
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                if a.checking {
                    spinner(ui);
                    ui.label(RichText::new("Checking the edit…").color(theme::DIM));
                } else {
                    let left = time_left(ttl.saturating_sub(a.asked.elapsed()));
                    ui.label(RichText::new(left).small().color(theme::FAINT));
                }
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    approval_buttons(ui, a, plan, &mut editing, actions);
                });
                ui.ctx().request_repaint_after(Duration::from_secs(1));
            });
        }),
    }
    if let Some(key) = key {
        ui.data_mut(|d| match editing {
            Some(steps) => {
                d.insert_temp(key, steps);
            }
            None => d.remove::<Vec<EditStep>>(key),
        });
    }
}

/// An answered request as one line: the plan's card or the tool's row says the rest.
fn answered(ui: &mut egui::Ui, a: &Approval, plan: Option<&PlanCard>, approved: bool) {
    let (icon, colour, verdict) = if approved {
        (&icons::SUCCESS, theme::SUCCESS, "Approved")
    } else {
        (&icons::CLOSE_SMALL, theme::FAINT, "Denied")
    };
    let what = plan.map_or_else(|| a.tool.clone(), |p| format!("run \"{}\"", p.intent));
    Frame::new()
        .fill(theme::SURFACE)
        .corner_radius(CornerRadius::same(8))
        .inner_margin(Margin::symmetric(10, 6))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.small_icon(icon, Some(colour));
                ui.label(RichText::new(verdict).color(colour));
                ui.add(
                    egui::Label::new(RichText::new(what).size(13.0).color(theme::DIM)).truncate(),
                );
            });
        });
}

/// A request the last session ended waiting on: asked again only if the operator wants it, and
/// checked again then, since the robot and its world have moved on.
pub(super) fn unanswered_card(
    ui: &mut egui::Ui,
    u: &Unanswered,
    done: &Cell<bool>,
    actions: &mut Vec<Action>,
) {
    theme::status_card(ui, theme::FAINT, |ui| {
        ui.set_width(ui.available_width());
        ui.label(RichText::new("Waiting for your approval when the app last closed").strong());
        ui.label(&u.reason);
        // A plan's reason names it already; anything else shows what it would send.
        if u.args.get("steps").is_none() {
            code(ui, &pretty(&u.args));
        }
        ui.horizontal(|ui| {
            if done.get() {
                ui.label(RichText::new("Done").color(theme::DIM));
                return;
            }
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui
                    .add(ReButton::new("Ask again").small().blue())
                    .on_hover_text("Check it again now and ask for your approval")
                    .clicked()
                {
                    actions.push(Action::Send(Command::Run {
                        tool: u.tool.clone(),
                        args: u.args.clone(),
                    }));
                    done.set(true);
                }
                if ui
                    .add(ReButton::new("Dismiss").small().secondary())
                    .clicked()
                {
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
            .add_enabled(
                idle && !steps.is_empty(),
                ReButton::new("Check").small().blue(),
            )
            .clicked()
        {
            actions.push(edit(plan_edit::plan_args(&p.intent, steps)));
        }
        if ui
            .add(ReButton::new("Cancel").small().secondary())
            .clicked()
        {
            *editing = None;
        }
        return;
    }
    if ui
        .add_enabled(idle, ReButton::new("Approve").small().blue())
        .clicked()
    {
        actions.push(Action::Send(Command::Approve(a.id)));
    }
    if ui.add(ReButton::new("Deny").small().secondary()).clicked() {
        actions.push(Action::Send(Command::Deny(a.id)));
    }
    let Some(p) = plan else {
        if a.can_allow
            && ui
                .add_enabled(idle, ReButton::new("Allow for session").small().secondary())
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
        .add_enabled(idle, ReButton::new("Edit").small().secondary())
        .on_hover_text("Change, move or drop steps before approving")
        .clicked()
    {
        *editing = Some(plan_edit::editable(&p.steps));
    }
    if let Some(fixed) = plan_edit::fixed(&p.steps, &p.concerns)
        && ui
            .add_enabled(idle, ReButton::new("Apply fixes").small().secondary())
            .on_hover_text(
                "Change the plan as its checks suggest; it is checked again before you approve it",
            )
            .clicked()
    {
        actions.push(edit(plan_edit::plan_args(&p.intent, &fixed)));
    }
}
