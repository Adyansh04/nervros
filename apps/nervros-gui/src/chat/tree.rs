//! The Tree tab: a mission's behaviour tree as an operator reads it. Its steps hang off a spine in
//! the order they run, each a card with its arguments, the wrappers the plan gave it and how it is
//! going; under the step running or failed, the nodes that ran inside it, with their times.

use std::time::Duration;

use nervros_core::mission::Outcome;
use nervros_core::mission::plan::PlannedStep;
use nervros_core::session::Command;
use rerun::external::egui::{
    self, Align, Color32, CornerRadius, Frame, Layout, Margin, RichText, Stroke, pos2, vec2,
};
use rerun::external::re_ui::ReButton;

use super::plan::{capitalise, minutes};
use super::{Action, Node, PlanCard};
use crate::theme;

/// How far right of the spine a step's card starts.
const GUTTER: f32 = 26.0;
/// The spine's distance from the left edge.
const SPINE_X: f32 = 9.0;

/// How a step or node stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Pending,
    Running,
    Succeeded,
    Failed,
    Stopped,
}

impl State {
    fn of(status: &str) -> Self {
        match status {
            "running" => Self::Running,
            "success" => Self::Succeeded,
            "failure" => Self::Failed,
            "idle" | "stopped" => Self::Stopped,
            _ => Self::Pending,
        }
    }

    fn colour(self) -> Color32 {
        match self {
            Self::Pending => theme::FAINT,
            Self::Running => theme::INFO,
            Self::Succeeded => theme::SUCCESS,
            Self::Failed => theme::ERROR,
            Self::Stopped => theme::WARN,
        }
    }
}

/// The tab: the newest plan's tree, or what will show here.
pub fn tree_tab(
    ui: &mut egui::Ui,
    plan: Option<&PlanCard>,
    can_pause: bool,
    actions: &mut Vec<Action>,
) {
    let Some(p) = plan else {
        theme::section(ui, "Behaviour tree");
        ui.label(
            RichText::new(
                "No mission yet. A plan's tree shows here as soon as it is checked, and comes \
                 alive while it runs: each step, what runs inside it, and how long it takes.",
            )
            .color(theme::DIM),
        );
        return;
    };
    header(ui, p, can_pause, actions);
    ui.add_space(14.0);
    steps(ui, p);
    if p.mission.is_some() && p.finished.is_none() {
        ui.ctx().request_repaint_after(Duration::from_millis(120));
    }
}

/// Seconds since the mission started, while it runs.
fn now_s(p: &PlanCard) -> f64 {
    p.started.map_or(0.0, |s| s.elapsed().as_secs_f64())
}

/// A step's state: from its events, a failed step's from the outcome, and a running one stopped
/// when the mission ended under it.
fn step_state(p: &PlanCard, s: &PlannedStep) -> State {
    let status = p.progress.get(&s.id).map_or("", |(st, _)| st.as_str());
    match (&p.finished, State::of(status)) {
        (Some((_, failed, ..)), _) if *failed == s.id => State::Failed,
        (Some((Outcome::Canceled, ..)), State::Running) => State::Stopped,
        (Some(_), State::Running) => State::Failed,
        (_, state) => state,
    }
}

/// A node's path without the uids BehaviorTree.CPP adds to a name, `s2_Walk::5/Sequence::6`
/// becoming `s2_Walk/Sequence`.
fn clean(path: &str) -> String {
    path.split('/')
        .map(|part| part.split_once("::").map_or(part, |(name, _)| name))
        .collect::<Vec<_>>()
        .join("/")
}

/// The node the step itself reports as, `s2_GoToPlace`.
fn step_node<'a>(p: &'a PlanCard, s: &PlannedStep) -> Option<&'a Node> {
    let name = format!("{}_{}", s.id, s.skill);
    p.nodes.iter().find(|n| clean(&n.path) == name)
}

/// What ran inside the step, by path under it.
fn leaves<'a>(p: &'a PlanCard, s: &PlannedStep) -> Vec<&'a Node> {
    let under = format!("{}_{}/", s.id, s.skill);
    p.nodes
        .iter()
        .filter(|n| clean(&n.path).starts_with(&under))
        .collect()
}

/// How long a node ran, or has run.
fn took(p: &PlanCard, n: &Node) -> f64 {
    n.ended_s.unwrap_or_else(|| now_s(p)) - n.started_s
}

fn header(ui: &mut egui::Ui, p: &PlanCard, can_pause: bool, actions: &mut Vec<Action>) {
    let (label, colour) = match (&p.finished, p.mission.is_some()) {
        (Some((Outcome::Success, ..)), _) => ("Succeeded".to_owned(), theme::SUCCESS),
        (Some((Outcome::Canceled, ..)), _) => ("Stopped".to_owned(), theme::WARN),
        (Some((outcome, ..)), _) => (capitalise(outcome.as_str()), theme::ERROR),
        (None, true) if p.paused => ("Paused".to_owned(), theme::WARN),
        (None, true) => ("Running".to_owned(), theme::INFO),
        (None, false) if p.replaced => ("Replaced".to_owned(), theme::FAINT),
        (None, false) if p.not_run.is_some() => ("Did not run".to_owned(), theme::FAINT),
        (None, false) => ("Waiting to run".to_owned(), theme::DIM),
    };
    theme::card().show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.horizontal(|ui| {
            pill(ui, &label, colour, true);
            if p.mission.is_some() && p.finished.is_none() {
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.add(ReButton::new("Stop").small().secondary()).clicked() {
                        actions.push(Action::Send(Command::StopMission));
                    }
                    let pause = if p.paused { "Resume" } else { "Pause" };
                    if can_pause && ui.add(ReButton::new(pause).small().secondary()).clicked() {
                        actions.push(Action::Pause(!p.paused));
                    }
                });
            }
        });
        ui.add_space(6.0);
        ui.add(egui::Label::new(RichText::new(&p.intent).strong().size(15.0)).wrap());
        ui.add_space(10.0);
        progress_bar(ui, p);
        ui.add_space(6.0);
        let done = p
            .steps
            .iter()
            .filter(|s| step_state(p, s) == State::Succeeded)
            .count();
        let elapsed = match (&p.finished, p.started) {
            (Some((.., secs)), _) => format!(" · took {}", minutes(*secs)),
            (None, Some(_)) => format!(" · {}", minutes(now_s(p))),
            (None, None) => String::new(),
        };
        let meta = format!(
            "{done} of {} steps{elapsed} · at most {}",
            p.steps.len(),
            minutes(p.worst_case_s)
        );
        ui.label(RichText::new(meta).size(12.5).color(theme::FAINT));
    });
}

/// A segment per step, coloured by how it stands.
fn progress_bar(ui: &mut egui::Ui, p: &PlanCard) {
    let n = p.steps.len().max(1);
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), 6.0), egui::Sense::hover());
    let gap = 3.0;
    #[expect(
        clippy::cast_precision_loss,
        reason = "a plan has a few dozen steps at most"
    )]
    let width = (rect.width() - gap * (n as f32 - 1.0)) / n as f32;
    for (i, s) in p.steps.iter().enumerate() {
        #[expect(
            clippy::cast_precision_loss,
            reason = "a plan has a few dozen steps at most"
        )]
        let x = rect.left() + i as f32 * (width + gap);
        let segment = egui::Rect::from_min_size(pos2(x, rect.top()), vec2(width, rect.height()));
        let colour = match step_state(p, s) {
            State::Pending => theme::RAISED,
            state => state.colour(),
        };
        ui.painter()
            .rect_filled(segment, CornerRadius::same(3), colour);
    }
}

/// The steps on their spine.
fn steps(ui: &mut egui::Ui, p: &PlanCard) {
    let mut dots: Vec<(f32, State)> = Vec::with_capacity(p.steps.len());
    for s in &p.steps {
        let state = step_state(p, s);
        let top = ui.cursor().top();
        ui.horizontal_top(|ui| {
            ui.add_space(GUTTER);
            ui.vertical(|ui| step_card(ui, p, s, state));
        });
        dots.push((top + 17.0, state));
        ui.add_space(8.0);
    }
    let left = ui.min_rect().left() + SPINE_X;
    let painter = ui.painter();
    if let (Some((first, _)), Some((last, _))) = (dots.first(), dots.last()) {
        painter.line_segment(
            [pos2(left, *first), pos2(left, *last)],
            Stroke::new(2.0, theme::BORDER),
        );
    }
    let t = ui.input(|i| i.time);
    for (y, state) in dots {
        let centre = pos2(left, y);
        painter.line_segment(
            [centre, pos2(left + GUTTER - 4.0, y)],
            Stroke::new(2.0, theme::BORDER),
        );
        match state {
            State::Pending => {
                painter.circle_filled(centre, 5.5, theme::BG);
                painter.circle_stroke(centre, 5.5, Stroke::new(1.5, theme::FAINT));
            }
            State::Running => {
                // A ring that breathes while the step runs.
                #[expect(clippy::cast_possible_truncation, reason = "an alpha in 0..=1")]
                let glow = (0.35 + 0.25 * (t * 4.0).sin()) as f32;
                painter.circle_filled(centre, 9.0, state.colour().gamma_multiply(glow));
                painter.circle_filled(centre, 5.5, state.colour());
            }
            _ => {
                painter.circle_filled(centre, 5.5, state.colour());
            }
        }
    }
}

fn step_card(ui: &mut egui::Ui, p: &PlanCard, s: &PlannedStep, state: State) {
    let border = match state {
        State::Running => theme::INFO,
        State::Failed => theme::ERROR.gamma_multiply(0.7),
        _ => theme::BORDER,
    };
    Frame::new()
        .fill(if state == State::Pending {
            theme::SURFACE
        } else {
            theme::RAISED
        })
        .stroke(Stroke::new(1.0, border))
        .corner_radius(CornerRadius::same(9))
        .inner_margin(Margin::symmetric(12, 9))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.spacing_mut().item_spacing.y = 4.0;
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new(&s.id)
                        .monospace()
                        .size(12.0)
                        .color(theme::FAINT),
                );
                let name = s.name.as_ref().map_or(s.skill.as_str(), String::as_str);
                ui.label(RichText::new(name).strong().size(14.0));
                if !s.native.is_empty() {
                    pill(ui, &s.native, theme::ROBOT, false);
                }
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    status_text(ui, p, step_node(p, s), state);
                });
            });
            let args = arguments(s);
            if !args.is_empty() {
                ui.add(egui::Label::new(RichText::new(args).size(12.5).color(theme::DIM)).wrap());
            }
            badges(ui, s);
            let inner = leaves(p, s);
            let open = matches!(state, State::Running | State::Failed | State::Stopped);
            if !inner.is_empty() {
                if open {
                    ui.add_space(4.0);
                    nodes(ui, p, s, &inner);
                } else {
                    let ran = if inner.len() == 1 {
                        "1 node ran inside it".to_owned()
                    } else {
                        format!("{} nodes ran inside it", inner.len())
                    };
                    ui.label(RichText::new(ran).size(11.5).color(theme::FAINT))
                        .on_hover_ui(|ui| nodes(ui, p, s, &inner));
                }
            }
            if let Some((_, failed, reason, _)) = &p.finished
                && *failed == s.id
                && !reason.is_empty()
            {
                ui.add_space(4.0);
                Frame::new()
                    .fill(theme::ERROR.gamma_multiply(0.12))
                    .corner_radius(CornerRadius::same(6))
                    .inner_margin(Margin::symmetric(8, 6))
                    .show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        ui.add(
                            egui::Label::new(RichText::new(reason).size(12.5).color(theme::ERROR))
                                .wrap(),
                        );
                    });
            }
        });
}

/// What the step was asked, without the skill's name: `places: hallway, kitchen`.
fn arguments(s: &PlannedStep) -> String {
    s.args
        .iter()
        .map(|a| format!("{}: {}", a.name, a.value))
        .collect::<Vec<_>>()
        .join("  ·  ")
}

/// The wrappers the plan put round the step, and how it went before.
fn badges(ui: &mut egui::Ui, s: &PlannedStep) {
    let mut items: Vec<(String, Color32)> =
        vec![(format!("at most {}", minutes(s.timeout_s)), theme::FAINT)];
    if s.retries > 0 {
        items.push((format!("{} tries", s.retries + 1), theme::FAINT));
    }
    if s.optional {
        items.push(("optional".to_owned(), theme::FAINT));
    }
    if s.added {
        items.push(("added by the planner".to_owned(), theme::FAINT));
    }
    if let Some(track) = &s.track {
        let colour = if track.succeeded * 4 < track.runs * 3 {
            theme::WARN
        } else {
            theme::SUCCESS
        };
        items.push((
            format!("{} of {} ok before", track.succeeded, track.runs),
            colour,
        ));
    }
    ui.horizontal_wrapped(|ui| {
        ui.spacing_mut().item_spacing = vec2(5.0, 4.0);
        for (text, colour) in &items {
            pill(ui, text, *colour, false);
        }
    });
}

/// The nodes that ran inside a step, as a small tree.
fn nodes(ui: &mut egui::Ui, p: &PlanCard, s: &PlannedStep, inner: &[&Node]) {
    let under = format!("{}_{}/", s.id, s.skill);
    let last = inner.len().saturating_sub(1);
    for (i, n) in inner.iter().enumerate() {
        let path = clean(&n.path);
        let rest = path.strip_prefix(&under).unwrap_or(&path);
        let depth = rest.matches('/').count();
        let name = rest.rsplit('/').next().unwrap_or(rest).replace('_', " ");
        let state = match (p.finished.is_some(), State::of(&n.status)) {
            (true, State::Running) => State::Stopped,
            (_, state) => state,
        };
        ui.horizontal(|ui| {
            #[expect(clippy::cast_precision_loss, reason = "a tree is a few levels deep")]
            ui.add_space(12.0 * depth as f32);
            let branch = if i == last { "└" } else { "├" };
            ui.label(RichText::new(branch).monospace().color(theme::FAINT));
            ui.label(RichText::new("●").size(9.0).color(state.colour()));
            ui.label(RichText::new(name).size(12.5).color(theme::TEXT))
                .on_hover_text(format!("{}: {}", n.path, n.status));
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                ui.label(
                    RichText::new(seconds(took(p, n)))
                        .monospace()
                        .size(11.5)
                        .color(theme::FAINT),
                );
            });
        });
    }
}

/// The step's state and time, at the card's right.
fn status_text(ui: &mut egui::Ui, p: &PlanCard, node: Option<&Node>, state: State) {
    let time = node.map(|n| seconds(took(p, n))).unwrap_or_default();
    let (text, colour) = match state {
        State::Pending => ("waiting".to_owned(), theme::FAINT),
        State::Running if p.paused => (format!("paused {time}"), theme::WARN),
        State::Running => (format!("running {time}"), theme::INFO),
        State::Succeeded => (format!("✓ {time}"), theme::SUCCESS),
        State::Failed => (format!("✗ {time}"), theme::ERROR),
        State::Stopped => ("stopped".to_owned(), theme::WARN),
    };
    ui.label(RichText::new(text.trim()).size(12.0).color(colour));
}

fn seconds(s: f64) -> String {
    if s < 10.0 {
        format!("{:.1} s", s.max(0.0))
    } else {
        minutes(s)
    }
}

/// A small rounded label; `strong` fills it with its colour, else only tints it.
fn pill(ui: &mut egui::Ui, text: &str, colour: Color32, strong: bool) {
    let (fill, ink) = if strong {
        (colour.gamma_multiply(0.18), colour)
    } else {
        (theme::BG.gamma_multiply(0.6), colour)
    };
    Frame::new()
        .fill(fill)
        .stroke(Stroke::new(
            1.0,
            colour.gamma_multiply(if strong { 0.6 } else { 0.25 }),
        ))
        .corner_radius(CornerRadius::same(255))
        .inner_margin(Margin::symmetric(8, 2))
        .show(ui, |ui| {
            ui.label(RichText::new(text).size(11.5).color(ink));
        });
}
