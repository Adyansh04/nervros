//! The Events tab: the session's events as a timeline to read, each a line in words with its kind,
//! the raw event in its hover.

use std::time::{SystemTime, UNIX_EPOCH};

use nervros_core::session::Event;
use rerun::external::egui::{self, Color32, CornerRadius, Frame, Margin, RichText, Stroke};
use serde_json::Value;

use crate::theme;

/// What a line is about, for the filter and its colour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Group {
    #[default]
    All,
    Missions,
    Tools,
    Model,
    Problems,
}

impl Group {
    const SHOWN: [(Self, &'static str); 5] = [
        (Self::All, "All"),
        (Self::Missions, "Missions"),
        (Self::Tools, "Tools"),
        (Self::Model, "Model"),
        (Self::Problems, "Problems"),
    ];
}

/// How many events the tab keeps.
const KEPT: usize = 500;

/// The tab's state: the lines, newest first, and which of them show.
#[derive(Debug, Default)]
pub struct Events {
    lines: std::collections::VecDeque<Logged>,
    filter: Group,
}

impl Events {
    /// Keeps `event`'s line, if it has one.
    pub fn push(&mut self, event: &Event) {
        if let Some(line) = Logged::of(event) {
            if self.lines.len() == KEPT {
                self.lines.pop_back();
            }
            self.lines.push_front(line);
        }
    }

    /// The tab: the filter, then the events.
    pub fn show(&mut self, ui: &mut egui::Ui) {
        tab(ui, self.lines.iter(), &mut self.filter);
    }
}

/// One event as the timeline shows it.
#[derive(Debug, Clone)]
pub struct Logged {
    /// When it arrived, `HH:MM:SS` in UTC as the session log keeps time.
    at: String,
    kind: String,
    summary: String,
    group: Group,
    problem: bool,
    raw: String,
}

impl Logged {
    /// The line for `event`, or none for one too frequent to read, as a streamed reply's pieces.
    pub fn of(event: &Event) -> Option<Self> {
        let value = serde_json::to_value(event).ok()?;
        let kind = value["kind"].as_str().unwrap_or_default().to_owned();
        // Too frequent, or pictures and drawings the views show: nothing to read.
        if matches!(
            kind.as_str(),
            "reply_delta"
                | "context"
                | "plot"
                | "job_log"
                | "turn_started"
                | "turn_finished"
                | "snapshot"
                | "drawn"
                | "restored"
                | "mission_preview"
        ) {
            return None;
        }
        let (summary, group, problem) = summarise(&kind, &value);
        let since = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0, |d| d.as_secs());
        Some(Self {
            at: format!(
                "{:02}:{:02}:{:02}",
                since / 3600 % 24,
                since / 60 % 60,
                since % 60
            ),
            kind: kind.replace('_', " "),
            summary,
            group,
            problem,
            raw: serde_json::to_string_pretty(&value).unwrap_or_default(),
        })
    }

    fn shown_in(&self, filter: Group) -> bool {
        match filter {
            Group::All => true,
            Group::Problems => self.problem,
            group => self.group == group,
        }
    }

    fn colour(&self) -> Color32 {
        if self.problem {
            return theme::ERROR;
        }
        match self.group {
            Group::Missions => theme::INFO,
            Group::Tools => theme::ROBOT,
            Group::Model => theme::DIM,
            Group::All | Group::Problems => theme::FAINT,
        }
    }
}

/// An event in words, what it is about, and whether it is a problem.
fn summarise(kind: &str, value: &Value) -> (String, Group, bool) {
    if let Some(words) = mission_words(kind, value).or_else(|| problem_words(kind, value)) {
        return words;
    }
    let text = |k: &str| value[k].as_str().unwrap_or_default().to_owned();
    match kind {
        "user" | "notice" => (text("text"), Group::All, false),
        "steer" => (format!("steer: {}", text("text")), Group::All, false),
        "reply" => (text("text"), Group::Model, false),
        "report" => (text("text"), Group::Missions, false),
        "tool_started" => (
            format!(
                "{} {}",
                text("tool"),
                short(&arguments(&value["args"]), 120)
            ),
            Group::Tools,
            false,
        ),
        "tool_finished" => {
            let status = text("status");
            let message = text("message");
            let failed = matches!(status.as_str(), "failed" | "refused");
            let tail = if message.is_empty() {
                String::new()
            } else {
                format!(": {message}")
            };
            (
                format!("{} {status} in {} ms{tail}", text("tool"), value["ms"]),
                Group::Tools,
                failed,
            )
        }
        "model_call" => (
            format!(
                "{} · {} in, {} out · {} ms",
                text("model"),
                value["input_tokens"],
                value["output_tokens"],
                value["ms"]
            ),
            Group::Model,
            false,
        ),
        "approval_requested" => (
            format!("{} asks: {}", text("tool"), text("reason")),
            Group::All,
            false,
        ),
        "approval_edited" => (format!("edited: {}", text("reason")), Group::All, false),
        "edit_rejected" => (
            format!("edit refused: {}", text("message")),
            Group::All,
            true,
        ),
        "approval_resolved" => (
            if value["approved"].as_bool() == Some(true) {
                "approved".to_owned()
            } else {
                "not approved".to_owned()
            },
            Group::All,
            false,
        ),
        "compacted" => (
            format!(
                "context compacted: {} → {} tokens",
                value["before"], value["after"]
            ),
            Group::Model,
            false,
        ),
        "armed" => (
            if value["armed"].as_bool() == Some(true) {
                "armed".to_owned()
            } else {
                "disarmed".to_owned()
            },
            Group::All,
            false,
        ),
        _ => (String::new(), Group::All, false),
    }
}

/// A mission's or a job's event in words.
fn mission_words(kind: &str, value: &Value) -> Option<(String, Group, bool)> {
    let text = |k: &str| value[k].as_str().unwrap_or_default().to_owned();
    Some(match kind {
        "mission_planned" => (
            format!(
                "planned \"{}\": {} steps",
                text("intent"),
                value["steps"].as_array().map_or(0, Vec::len)
            ),
            Group::Missions,
            false,
        ),
        "mission_started" => (format!("started {}", text("id")), Group::Missions, false),
        "mission_paused" => (
            if value["paused"].as_bool() == Some(true) {
                "paused"
            } else {
                "resumed"
            }
            .to_owned(),
            Group::Missions,
            false,
        ),
        "mission_progress" => (
            format!(
                "{} {} {}",
                text("step"),
                if text("node").is_empty() {
                    String::new()
                } else {
                    format!("· {}", text("node"))
                },
                text("status")
            ),
            Group::Missions,
            text("status") == "failure",
        ),
        "mission_finished" => {
            let outcome = text("outcome");
            let step = text("failed_step");
            let at = if step.is_empty() {
                String::new()
            } else {
                format!(" at {step}")
            };
            (
                format!(
                    "{outcome}{at} after {:.0} s {}",
                    value["elapsed_s"].as_f64().unwrap_or(0.0),
                    text("reason")
                ),
                Group::Missions,
                outcome != "success",
            )
        }
        "job_started" | "job_finished" => {
            let job = &value["job"];
            let state = job["state"].as_str().unwrap_or_default();
            (
                format!(
                    "{} {}",
                    job["name"].as_str().unwrap_or_default(),
                    state.replace('_', " ")
                ),
                Group::Tools,
                matches!(state, "failed" | "timed_out"),
            )
        }
        _ => return None,
    })
}

/// An error, an incident or a stop in words.
fn problem_words(kind: &str, value: &Value) -> Option<(String, Group, bool)> {
    let text = |k: &str| value[k].as_str().unwrap_or_default().to_owned();
    Some(match kind {
        "error" => (text("text"), Group::Problems, true),
        "incident" => {
            let i = &value["incident"];
            (
                format!(
                    "{}: {}",
                    i["kind"].as_str().unwrap_or("incident").replace('_', " "),
                    i["trigger"].as_str().unwrap_or_default()
                ),
                Group::Problems,
                true,
            )
        }
        "halted" => (format!("halted: {}", text("reason")), Group::Problems, true),
        "stopped" => {
            let ok = value["ok"].as_bool() == Some(true);
            (format!("stopped: {}", text("detail")), Group::All, !ok)
        }
        _ => return None,
    })
}

/// A tool's arguments as words: `action: path · place: office_desk_tray`.
fn arguments(args: &Value) -> String {
    match args.as_object() {
        Some(map) => map
            .iter()
            .map(|(k, v)| match v.as_str() {
                Some(text) => format!("{k}: {text}"),
                None => format!("{k}: {v}"),
            })
            .collect::<Vec<_>>()
            .join(" · "),
        None => args.to_string(),
    }
}

fn short(text: &str, max: usize) -> String {
    if text.chars().count() <= max {
        return text.to_owned();
    }
    let cut: String = text.chars().take(max).collect();
    format!("{cut}…")
}

/// The tab: the filter, then the events, newest first.
pub fn tab<'a>(ui: &mut egui::Ui, log: impl Iterator<Item = &'a Logged>, filter: &mut Group) {
    ui.horizontal_wrapped(|ui| {
        ui.spacing_mut().item_spacing = egui::vec2(5.0, 5.0);
        ui.spacing_mut().button_padding = egui::vec2(10.0, 3.0);
        for (group, name) in Group::SHOWN {
            let on = *filter == group;
            let label =
                RichText::new(name)
                    .size(12.0)
                    .color(if on { theme::TEXT } else { theme::DIM });
            let (fill, edge) = if on {
                (theme::ACCENT.gamma_multiply(0.35), theme::ACCENT)
            } else {
                (theme::SURFACE, theme::BORDER)
            };
            let chip = egui::Button::new(label)
                .fill(fill)
                .stroke(Stroke::new(1.0, edge))
                .corner_radius(CornerRadius::same(255))
                .min_size(egui::vec2(0.0, 24.0));
            if ui.add(chip).clicked() {
                *filter = group;
            }
        }
    });
    ui.add_space(10.0);
    let mut any = false;
    for line in log.filter(|l| l.shown_in(*filter)) {
        any = true;
        row(ui, line);
    }
    if !any {
        ui.label(
            RichText::new(
                "Nothing here yet: the session's events show as they happen, newest first.",
            )
            .color(theme::DIM),
        );
    }
}

fn row(ui: &mut egui::Ui, line: &Logged) {
    let colour = line.colour();
    let response = Frame::new()
        .inner_margin(Margin::symmetric(8, 6))
        .corner_radius(CornerRadius::same(7))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new(&line.at)
                        .monospace()
                        .size(11.0)
                        .color(theme::FAINT),
                );
                Frame::new()
                    .fill(colour.gamma_multiply(0.14))
                    .corner_radius(CornerRadius::same(255))
                    .inner_margin(Margin::symmetric(7, 1))
                    .show(ui, |ui| {
                        ui.label(RichText::new(&line.kind).size(11.0).color(colour));
                    });
            });
            if !line.summary.is_empty() {
                ui.add(
                    egui::Label::new(
                        RichText::new(short(&line.summary, 280))
                            .size(12.5)
                            .color(theme::TEXT),
                    )
                    .wrap(),
                );
            }
        })
        .response;
    let raw = line.raw.clone();
    response.on_hover_ui(move |ui| {
        ui.label(RichText::new(raw.as_str()).monospace().size(11.0));
    });
    ui.add_space(2.0);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn events_read_as_words_and_filter_by_group() {
        let failed = Logged::of(&Event::ToolFinished {
            turn: 1,
            call: 2,
            tool: "nav_check".into(),
            status: nervros_core::tools::Status::Failed,
            message: "no path".into(),
            ms: 12,
        })
        .unwrap();
        assert!(
            failed
                .summary
                .starts_with("nav_check failed in 12 ms: no path"),
            "{}",
            failed.summary
        );
        assert!(failed.shown_in(Group::Problems) && failed.shown_in(Group::Tools));
        assert!(!failed.shown_in(Group::Missions));
        assert!(
            Logged::of(&Event::ReplyDelta {
                turn: 1,
                text: "a".into()
            })
            .is_none()
        );
        let error = Logged::of(&Event::Error {
            turn: 1,
            text: "the model did not answer".into(),
        })
        .unwrap();
        assert_eq!(error.summary, "the model did not answer");
        assert!(error.shown_in(Group::Problems));
    }

    #[test]
    fn snapshot_events_tab() {
        let mut events = Events::default();
        for e in [
            Event::User {
                turn: 1,
                text: "Walk through the hallway to the office desk tray".into(),
            },
            Event::ToolStarted {
                turn: 1,
                call: 1,
                tool: "nav_check".into(),
                args: serde_json::json!({"action": "path", "place": "office_desk_tray"}),
            },
            Event::ToolFinished {
                turn: 1,
                call: 1,
                tool: "nav_check".into(),
                status: nervros_core::tools::Status::Succeeded,
                message: String::new(),
                ms: 412,
            },
            Event::ToolFinished {
                turn: 1,
                call: 2,
                tool: "geofence".into(),
                status: nervros_core::tools::Status::Failed,
                message: "there is no zone called `hallway`".into(),
                ms: 0,
            },
            Event::Notice {
                text: "Armed: the agent may now act".into(),
            },
        ] {
            events.push(&e);
        }
        // The clock's time would differ on every run.
        for line in &mut events.lines {
            line.at = "09:41:07".to_owned();
        }
        let mut harness = egui_kittest::Harness::builder()
            .wgpu()
            .with_size(egui::vec2(340.0, 520.0))
            .build_ui(move |ui| {
                Frame::new()
                    .fill(theme::BG)
                    .inner_margin(Margin::same(12))
                    .show(ui, |ui| events.show(ui));
            });
        crate::testkit::style_for_tests(&harness.ctx);
        harness.run_steps(2);
        harness.fit_contents();
        crate::testkit::compare(
            &mut harness,
            "events_tab",
            &egui_kittest::SnapshotOptions::new(),
        );
    }
}
