//! What the operator can do from the keyboard: Ctrl+K opens a palette of commands, and the same
//! commands run when typed in the chat by their `/name`.

use nervros_core::session::Command;
use rerun::external::egui::{self, text::LayoutJob};
use rerun::external::re_ui::{
    CmdRow, CommandPaletteProvider, FuzzyMatch, FuzzyQuery, MatchGroup, MatchedCmd,
};

use crate::app::Tab;

/// What a command does.
#[derive(Debug, Clone, PartialEq)]
pub enum Cmd {
    /// Ask the agent, as if typed.
    Say(&'static str),
    /// Tell the session.
    Send(Command),
    /// Open a dock tab.
    Tab(Tab),
    /// Check the robot again and show the result.
    Doctor,
    /// Show or hide the dock.
    Dock,
    /// Put the viewer's panes back.
    ResetLayout,
    /// Open or close the world editor.
    EditWorld,
    /// Follow the newest data again after looking at an earlier moment.
    Live,
    /// Keep the 3D view on the robot, or frame the whole map again.
    Follow,
}

/// One command: what the palette lists, the `/name` that runs it from the chat, and its keys.
#[derive(Debug, Clone)]
pub struct Entry {
    pub text: &'static str,
    pub slash: &'static str,
    pub keys: &'static str,
    pub cmd: Cmd,
}

const fn entry(text: &'static str, slash: &'static str, keys: &'static str, cmd: Cmd) -> Entry {
    Entry {
        text,
        slash,
        keys,
        cmd,
    }
}

/// Every command, in the order the palette lists them before anything is typed.
pub fn entries() -> Vec<Entry> {
    vec![
        entry(
            "Look through the camera",
            "/look",
            "",
            Cmd::Say("Look through the camera and tell me what you see."),
        ),
        entry(
            "List the places the robot knows",
            "/places",
            "",
            Cmd::Say("Which places can you go to?"),
        ),
        entry(
            "Stop the mission",
            "/stop",
            "Ctrl+Shift+S",
            Cmd::Send(Command::StopMission),
        ),
        entry("Check the robot", "/doctor", "", Cmd::Doctor),
        entry(
            "Robot: its state, and driving by hand",
            "/robot",
            "Ctrl+8",
            Cmd::Tab(Tab::Robot),
        ),
        entry("Models and quotas", "/models", "", Cmd::Tab(Tab::Agent)),
        entry(
            "Missions, saved plans and skill gaps",
            "/missions",
            "",
            Cmd::Tab(Tab::Mission),
        ),
        entry("World model", "/world", "", Cmd::Tab(Tab::World)),
        entry(
            "Explore the building",
            "/explore",
            "",
            Cmd::Say(crate::app::EXPLORE_REQUEST),
        ),
        entry(
            "Condense the conversation",
            "/compact",
            "",
            Cmd::Send(Command::Compact),
        ),
        entry(
            "Arm: let the agent act",
            "/arm",
            "",
            Cmd::Send(Command::Arm),
        ),
        entry(
            "Observe only: stop the agent acting",
            "/disarm",
            "",
            Cmd::Send(Command::Disarm),
        ),
        entry("Approvals", "/approvals", "", Cmd::Tab(Tab::Approvals)),
        entry("Session events", "/events", "", Cmd::Tab(Tab::Events)),
        entry("Viewer layers", "/layers", "", Cmd::Tab(Tab::Layers)),
        entry("Show or hide the dock", "/dock", "", Cmd::Dock),
        entry("Reset the viewer layout", "/layout", "", Cmd::ResetLayout),
        entry(
            "Follow the robot in 3D, or see the whole map",
            "/follow",
            "",
            Cmd::Follow,
        ),
        entry("Edit the world", "/edit", "", Cmd::EditWorld),
        entry("Back to the live view", "/live", "", Cmd::Live),
    ]
}

/// The command a chat message names, as `/look`; none for anything else.
pub fn slash<'a>(entries: &'a [Entry], text: &str) -> Option<&'a Cmd> {
    let name = text.split_whitespace().next()?;
    entries.iter().find(|e| e.slash == name).map(|e| &e.cmd)
}

/// The palette's view of the commands: matched by what they do, or by `/name` when typed so.
pub struct Provider<'a> {
    pub entries: &'a [Entry],
    /// Whether a command can run now, such as editing the world only when there is an editor.
    pub available: &'a dyn Fn(&Cmd) -> bool,
}

impl CommandPaletteProvider<Cmd> for Provider<'_> {
    fn initial_hint_ui(&mut self, ui: &mut egui::Ui) {
        ui.weak("Find a command, or type its /name in the chat");
        ui.add_space(4.0);
    }

    fn all_matching(&mut self, query: &FuzzyQuery) -> Vec<MatchGroup<Cmd>> {
        let by_name = query.raw_query().starts_with('/');
        let group = self
            .entries
            .iter()
            .filter_map(|e| {
                let target = if by_name { e.slash } else { e.text };
                let fuzzy_match = if query.is_empty() {
                    FuzzyMatch::lowest(target.to_owned())
                } else {
                    query.try_match(target.to_owned())?
                };
                Some(MatchedCmd {
                    command: e.cmd.clone(),
                    fuzzy_match,
                    enabled: (self.available)(&e.cmd),
                })
            })
            .collect();
        vec![group]
    }

    fn cmd_row(&self, ui: &egui::Ui, matched: &MatchedCmd<Cmd>, selected: bool) -> CmdRow {
        let visuals = ui.visuals();
        let colour = if !matched.enabled {
            visuals.weak_text_color()
        } else if selected {
            visuals.selection.stroke.color
        } else {
            visuals.widgets.inactive.fg_stroke.color
        };
        let job = LayoutJob::simple(
            matched.fuzzy_match.target().to_owned(),
            egui::TextStyle::Button.resolve(ui.style()),
            colour,
            f32::INFINITY,
        );
        let job = if matched.enabled {
            matched
                .fuzzy_match
                .highlight_matching_text(ui.style(), &job, selected)
        } else {
            job
        };
        let entry = self.entries.iter().find(|e| e.cmd == matched.command);
        let kb_shortcut = entry.map_or_else(String::new, |e| {
            let name = if e.slash == matched.fuzzy_match.target() {
                e.text
            } else {
                e.slash
            };
            if e.keys.is_empty() {
                name.to_owned()
            } else {
                format!("{name} · {}", e.keys)
            }
        });
        CmdRow {
            job,
            kb_shortcut,
            tooltip: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_chat_message_runs_the_command_its_slash_names() {
        let all = entries();
        assert_eq!(
            slash(&all, "/stop now"),
            Some(&Cmd::Send(Command::StopMission))
        );
        assert_eq!(slash(&all, "/models"), Some(&Cmd::Tab(Tab::Agent)));
        assert_eq!(slash(&all, "/nonsense"), None);
        assert_eq!(slash(&all, "stop"), None);
    }

    #[test]
    fn snapshot_palette() {
        let all = entries();
        let mut palette = rerun::external::re_ui::CommandPalette::default();
        palette.toggle();
        let mut harness = egui_kittest::Harness::builder()
            .wgpu()
            .with_size(egui::vec2(720.0, 420.0))
            .build_ui(move |ui| {
                let available = |cmd: &Cmd| !matches!(cmd, Cmd::Live | Cmd::EditWorld);
                let mut provider = Provider {
                    entries: &all,
                    available: &available,
                };
                let _ = palette.show(ui.ctx(), &mut provider);
            });
        crate::testkit::style_for_tests(&harness.ctx);
        harness.run_steps(2);
        crate::testkit::compare(
            &mut harness,
            "palette",
            &egui_kittest::SnapshotOptions::new(),
        );
    }

    #[test]
    fn every_command_has_its_own_slash_name() {
        let all = entries();
        let mut names: Vec<&str> = all.iter().map(|e| e.slash).collect();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), all.len());
        assert!(names.iter().all(|n| n.starts_with('/')));
    }
}
