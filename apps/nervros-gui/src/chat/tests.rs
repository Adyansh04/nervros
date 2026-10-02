use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::Arc;
use std::time::Duration;

use egui_kittest::kittest::Queryable as _;
use egui_kittest::{Harness, SnapshotOptions};
use nervros_core::mission::Outcome;
use nervros_core::mission::plan::{PlannedStep, StepArg};
use nervros_core::mission::preview::PreviewStep;
use nervros_core::mission::sanity::Concern;
use nervros_core::session::{Command, Event, Unanswered};
use nervros_core::tools::Status;
use rerun::external::egui;
use rerun::external::egui::{Frame, Margin};

use super::approval::time_left;
use super::plan::tree_panel;
use super::report::report_text;
use super::*;
use crate::plan_edit;
use crate::testkit::{compare, style_for_tests};

pub(crate) fn sample() -> Chat {
    let mut chat = Chat::default();
    chat.push_user("What do you see right now?".to_owned(), true);
    chat.apply(&Event::TurnStarted { turn: 1 });
    chat.apply(&Event::ToolStarted {
        turn: 1,
        call: 1,
        tool: "look".to_owned(),
        args: serde_json::json!({}),
    });
    chat.apply(&Event::ToolFinished {
        turn: 1,
        call: 1,
        tool: "look".to_owned(),
        status: Status::Succeeded,
        message: "2 marks: 1 shelf, 2 cardboard box".to_owned(),
        ms: 427,
    });
    chat.apply(&Event::Reply {
        turn: 1,
        text: "I see a **shelf** (mark 1) and a **cardboard box** (mark 2).".to_owned(),
        model: "qwen3.5-9b-local".to_owned(),
    });
    chat.apply(&Event::ApprovalRequested {
        id: 7,
        tool: "navigate".to_owned(),
        args: serde_json::json!({"place": "kitchen"}),
        reason: "moves the base".to_owned(),
        can_allow: false,
    });
    chat.apply(&Event::TurnFinished { turn: 1 });
    chat
}

#[test]
fn events_fold_into_items() {
    let chat = sample();
    assert_eq!(chat.items.len(), 4);
    assert!(
        matches!(&chat.items[1], Item::Tool(t) if t.status == Some(Status::Succeeded) && t.ms == 427)
    );
    assert_eq!(chat.pending().count(), 1);
    assert_eq!(chat.model.as_deref(), Some("qwen3.5-9b-local"));
    assert!(chat.turn.is_none());
}

#[test]
fn a_long_wait_counts_in_minutes_and_the_last_ninety_seconds_in_seconds() {
    assert_eq!(time_left(Duration::from_mins(10)), "10 min left");
    assert_eq!(time_left(Duration::from_secs(91)), "2 min left");
    assert_eq!(time_left(Duration::from_millis(59_200)), "60 s left");
    assert_eq!(time_left(Duration::ZERO), "0 s left");
}

#[test]
fn a_resolved_approval_is_no_longer_pending() {
    let mut chat = sample();
    chat.apply(&Event::ApprovalResolved {
        id: 7,
        approved: false,
    });
    assert_eq!(chat.pending().count(), 0);
}

#[test]
fn a_report_shows_what_happened_with_short_ids() {
    let text = "Mission 01a0f95c-2c09-7662-b4ee-222fda068459 (turn left) ended: failure after \
                2 s. Failed at s1.\nFind out why and say it in one sentence.";
    assert_eq!(
        report_text(text),
        "Mission 01a0f95c (turn left) ended: failure after 2 s. Failed at s1."
    );
}

#[test]
fn condensing_up_to_a_message_keeps_the_ones_after_it() {
    let mut chat = Chat::default();
    for (turn, text) in [(1, "Where is the mug?"), (2, "Bring it here.")] {
        chat.apply(&Event::User {
            turn,
            text: text.to_owned(),
        });
    }
    let sent = Rc::new(RefCell::new(Vec::new()));
    let seen = Rc::clone(&sent);
    let mut harness = Harness::builder()
        .with_size(egui::vec2(440.0, 400.0))
        .build_ui(move |ui| {
            let mut actions = Vec::new();
            chat.show(ui, Duration::from_mins(10), &mut actions);
            seen.borrow_mut().extend(actions);
        });
    harness.run();
    harness.get_by_label("Where is the mug?").click_secondary();
    harness.run();
    harness.get_by_label("Condense up to here").click();
    harness.run();
    assert!(
        sent.borrow()
            .iter()
            .any(|a| matches!(a, Action::Send(Command::CompactUpTo { keep: 1 }))),
        "{:?}",
        sent.borrow().len()
    );
}

/// A harness drawing `chat` on the panel background, cropped to what it draws.
fn render(chat: Chat, name: &str) {
    render_with(chat, name, |_| {});
}

/// As [`render`], with `setup` run on the context first, such as to open an editor.
fn render_with(chat: Chat, name: &str, setup: impl FnOnce(&egui::Context)) {
    let mut harness = Harness::builder()
        .wgpu()
        .with_size(egui::vec2(440.0, 800.0))
        .build_ui(move |ui| {
            Frame::new()
                .fill(ui.tokens().panel_bg_color)
                .inner_margin(Margin::same(16))
                .show(ui, |ui| {
                    // Long, so the countdown reads the same however slowly the test runs.
                    chat.show(ui, Duration::from_mins(10), &mut Vec::new());
                });
        });
    style_for_tests(&harness.ctx);
    setup(&harness.ctx);
    harness.run();
    harness.fit_contents();
    compare(&mut harness, name, &SnapshotOptions::new());
}

/// A plan for "turn left, then walk half a metre" that turns right and walks a metre.
fn doubtful_plan() -> Chat {
    let mut chat = Chat::default();
    chat.apply(&Event::User {
        turn: 1,
        text: "Turn left 90 degrees, then walk forward half a metre".to_owned(),
    });
    let step = |id: &str, skill: &str, args: &[(&str, &str)]| {
        let args: Vec<StepArg> = args
            .iter()
            .map(|(n, v)| StepArg {
                name: (*n).to_owned(),
                value: (*v).to_owned(),
            })
            .collect();
        let shown: Vec<String> = args
            .iter()
            .map(|a| format!("{}={}", a.name, a.value))
            .collect();
        PlannedStep {
            id: id.to_owned(),
            skill: skill.to_owned(),
            summary: format!("{skill}({})", shown.join(", ")),
            args,
            timeout_s: 30.0,
            ..PlannedStep::default()
        }
    };
    let concern = |step: &str, message: &str, fix: Option<(&str, &str)>| Concern {
        step: step.to_owned(),
        message: message.to_owned(),
        fix: fix.map(|(name, value)| StepArg {
            name: name.to_owned(),
            value: value.to_owned(),
        }),
    };
    chat.apply(&Event::MissionPlanned {
        hash: "5be0c1d9f2a4".to_owned(),
        intent: "turn left and walk half a metre".to_owned(),
        steps: vec![
            step("s1", "TurnInPlace", &[("degrees", "-90")]),
            step(
                "s2",
                "WalkStraight",
                &[("direction", "forward"), ("distance_m", "1.0")],
            ),
        ],
        worst_case_s: 60.0,
        concerns: vec![
            concern(
                "s1",
                "the operator said turn left, but degrees=-90 turns the other way \
                 (positive degrees turn left)",
                Some(("degrees", "90")),
            ),
            concern(
                "s2",
                "the operator asked for 0.5 m, but the walks add up to 1 m",
                Some(("distance_m", "0.5")),
            ),
            concern("", "the walk may end close to the table", None),
        ],
    });
    chat.apply(&Event::ApprovalRequested {
        id: 9,
        tool: "run_mission".to_owned(),
        args: serde_json::json!({"hash": "5be0c1d9f2a4"}),
        reason: "runs it".to_owned(),
        can_allow: false,
    });
    chat
}

#[test]
fn snapshot_plan_concerns() {
    render(doubtful_plan(), "chat_plan_concerns");
}

#[test]
fn snapshot_plan_edit() {
    let mut chat = doubtful_plan();
    chat.edit_sent(9);
    chat.apply(&Event::EditRejected {
        id: 9,
        message: "s2: distance_m must be from 0.1 to 2.0".to_owned(),
    });
    let Some(Item::Plan(p)) = chat.items.get(1).cloned() else {
        panic!("the plan comes second");
    };
    render_with(chat, "chat_plan_edit", move |ctx| {
        let key = egui::Id::new(("plan_edit", 9_u64, p.hash.as_str()));
        ctx.data_mut(|d| d.insert_temp(key, plan_edit::editable(&p.steps)));
    });
}

#[test]
fn snapshot_tree_panel() {
    let mut chat = doubtful_plan();
    chat.apply(&Event::MissionStarted {
        id: "m2".to_owned(),
        hash: "5be0c1d9f2a4".to_owned(),
    });
    for (step, node, path, status) in [
        ("s1", "", "s1_TurnInPlace::2", "running"),
        ("s1", "Turn", "s1_TurnInPlace::2/Turn", "running"),
        ("s1", "Turn", "s1_TurnInPlace::2/Turn", "success"),
        ("s1", "", "s1_TurnInPlace::2", "success"),
        ("s2", "", "s2_WalkStraight::5", "running"),
        (
            "s2",
            "Sequence",
            "s2_WalkStraight::5/Sequence::6",
            "running",
        ),
        (
            "s2",
            "CheckClear",
            "s2_WalkStraight::5/Sequence::6/CheckClear",
            "success",
        ),
        (
            "s2",
            "Walk",
            "s2_WalkStraight::5/Sequence::6/Walk",
            "running",
        ),
    ] {
        chat.apply(&Event::MissionProgress {
            id: "m2".to_owned(),
            step: step.to_owned(),
            node: node.to_owned(),
            path: path.to_owned(),
            status: status.to_owned(),
            elapsed_s: 3.0,
        });
    }
    let plan = chat.latest_plan().cloned().unwrap();
    let mut harness = Harness::builder()
        .wgpu()
        .with_size(egui::vec2(440.0, 600.0))
        .build_ui(move |ui| {
            Frame::new()
                .fill(ui.tokens().panel_bg_color)
                .inner_margin(Margin::same(16))
                .show(ui, |ui| tree_panel(ui, &plan));
        });
    style_for_tests(&harness.ctx);
    harness.run();
    harness.fit_contents();
    compare(&mut harness, "tree_panel", &SnapshotOptions::new());
}

#[test]
fn a_passed_edit_moves_the_approval_below_its_new_plan() {
    let mut chat = doubtful_plan();
    chat.edit_sent(9);
    assert!(matches!(chat.items.last(), Some(Item::Approval(a)) if a.checking));
    chat.apply(&Event::MissionPlanned {
        hash: "e7f3aa01c2d4".to_owned(),
        intent: "turn left and walk half a metre".to_owned(),
        steps: Vec::new(),
        worst_case_s: 60.0,
        concerns: Vec::new(),
    });
    chat.apply(&Event::ApprovalEdited {
        id: 9,
        args: serde_json::json!({"hash": "e7f3aa01c2d4"}),
        reason: "runs the edited plan".to_owned(),
    });
    let Some(Item::Approval(a)) = chat.items.last() else {
        panic!("the approval comes last");
    };
    assert!(!a.checking);
    assert_eq!(
        chat.plan_for(a).map(|p| p.hash.as_str()),
        Some("e7f3aa01c2d4")
    );
    assert!(matches!(&chat.items[1], Item::Plan(p) if p.replaced));
}

#[test]
fn snapshot_chat() {
    render(sample(), "chat");
}

#[test]
fn snapshot_unanswered_and_allow() {
    let mut chat = Chat::default();
    chat.items.push(Item::Unanswered(
        Unanswered {
            tool: "run_mission".to_owned(),
            args: serde_json::json!({"intent": "bring the red mug to the sofa", "steps": []}),
            reason: "runs \"bring the red mug to the sofa\": 4 step(s), at most 6 min".to_owned(),
        },
        Cell::new(false),
    ));
    chat.apply(&Event::ApprovalRequested {
        id: 3,
        tool: "set_parameter".to_owned(),
        args: serde_json::json!({"node": "/controller_server", "name": "max_vel_x", "value": 0.3}),
        reason: "`set_parameter` acts on the robot".to_owned(),
        can_allow: true,
    });
    render(chat, "chat_unanswered");
}

#[test]
fn snapshot_draft_and_compaction() {
    let mut chat = Chat::default();
    chat.apply(&Event::Compacted {
        before: 9800,
        after: 2100,
        summarised: true,
    });
    chat.apply(&Event::User {
        turn: 3,
        text: "Where is the mug?".to_owned(),
    });
    for piece in ["The small white mug ", "is on the dining table, "] {
        chat.apply(&Event::ReplyDelta {
            turn: 3,
            text: piece.to_owned(),
        });
    }
    render(chat, "chat_draft");
}

#[test]
fn snapshot_empty_state() {
    render(Chat::default(), "chat_empty");
}

#[test]
fn snapshot_mission() {
    let mut chat = Chat::default();
    chat.apply(&Event::User {
        turn: 1,
        text: "Put the red mug in the basket".to_owned(),
    });
    let step = |id: &str, summary: &str| PlannedStep {
        id: id.to_owned(),
        skill: summary.split('(').next().unwrap_or_default().to_owned(),
        summary: summary.to_owned(),
        timeout_s: 300.0,
        ..PlannedStep::default()
    };
    chat.apply(&Event::MissionPlanned {
        hash: "a91f3c2e77d04b1e".to_owned(),
        intent: "put the red mug in the basket".to_owned(),
        steps: vec![
            step("s1", "GoToPlace(place=kitchen)"),
            step("s2", "PickObject(object_id=O17, phrase=red mug, arm=right)"),
            step(
                "s3",
                "PlaceInto(container_id=O31, phrase=basket, arm=right)",
            ),
        ],
        worst_case_s: 1140.0,
        concerns: Vec::new(),
    });
    chat.apply(&Event::ApprovalRequested {
        id: 3,
        tool: "run_mission".to_owned(),
        args: serde_json::json!({"hash": "a91f3c2e"}),
        reason: "`run_mission` acts on the robot".to_owned(),
        can_allow: false,
    });
    chat.apply(&Event::ApprovalResolved {
        id: 3,
        approved: true,
    });
    chat.apply(&Event::MissionStarted {
        id: "m1".to_owned(),
        hash: "a91f3c2e77d04b1e".to_owned(),
    });
    for (step, node, status) in [
        ("s1", "", "running"),
        ("s1", "", "success"),
        ("s2", "", "running"),
        ("s2", "Pick", "running"),
        ("s2", "", "failure"),
    ] {
        chat.apply(&Event::MissionProgress {
            id: "m1".to_owned(),
            step: step.to_owned(),
            node: node.to_owned(),
            path: String::new(),
            status: status.to_owned(),
            elapsed_s: 0.0,
        });
    }
    chat.apply(&Event::MissionFinished {
        id: "m1".to_owned(),
        outcome: Outcome::Failure,
        failed_step: "s2".to_owned(),
        reason: "the grasp slipped".to_owned(),
        elapsed_s: 94.0,
    });
    chat.apply(&Event::Report {
        turn: 2,
        text: "Mission m1 ended: failure after 94 s. Failed at s2: the grasp slipped.".to_owned(),
    });
    render(chat, "chat_mission");
}

#[test]
fn snapshot_plan_with_records_and_a_preview_warning() {
    use nervros_core::mission::ledger::Track;
    let mut chat = Chat::default();
    chat.apply(&Event::User {
        turn: 1,
        text: "Bring the small white mug to the tray".to_owned(),
    });
    let step = |id: &str, summary: &str, track: Option<Track>| PlannedStep {
        id: id.to_owned(),
        skill: summary.split('(').next().unwrap_or_default().to_owned(),
        summary: summary.to_owned(),
        timeout_s: 300.0,
        track,
        ..PlannedStep::default()
    };
    let track = |succeeded, runs, typical_s, last: Option<&str>| Track {
        runs,
        succeeded,
        typical_s,
        last_failure: last.map(str::to_owned),
    };
    chat.apply(&Event::MissionPlanned {
        hash: "7c01d2aa9e3b".to_owned(),
        intent: "bring the mug to the tray".to_owned(),
        steps: vec![
            step(
                "s1",
                "GoToPlace(place=dining_table_side)",
                Some(track(8, 8, Some(21.0), None)),
            ),
            step(
                "s2",
                "PickObject(object_id=mug_4, arm=left)",
                Some(track(
                    1,
                    3,
                    Some(44.0),
                    Some("nothing called mug_4 on /objects"),
                )),
            ),
            step("s3", "GoToPlace(place=office_desk_tray)", None),
            step("s4", "PlaceInto(container_id=tray_1, arm=left)", None),
        ],
        worst_case_s: 1500.0,
        concerns: Vec::new(),
    });
    let preview = |id: &str, note: &str| PreviewStep {
        id: id.to_owned(),
        goal: None,
        path: Vec::new(),
        note: note.to_owned(),
    };
    chat.apply(&Event::MissionPreview {
        hash: "7c01d2aa9e3b".to_owned(),
        steps: vec![
            preview("s1", "walks there along Nav2's path"),
            preview("s2", "closes in on what it sees; ends within reach of it"),
            preview("s3", "no path: the goal is inside an obstacle"),
        ],
    });
    chat.apply(&Event::ApprovalRequested {
        id: 4,
        tool: "run_mission".to_owned(),
        args: serde_json::json!({"hash": "7c01d2aa9e3b"}),
        reason: "`run_mission` acts on the robot".to_owned(),
        can_allow: false,
    });
    render(chat, "chat_plan_records");
}

#[test]
fn snapshot_error_and_image() {
    let mut chat = Chat::default();
    chat.apply(&Event::User {
        turn: 2,
        text: "Look again".to_owned(),
    });
    let mut jpeg = Vec::new();
    let img = image::RgbImage::from_pixel(96, 54, image::Rgb([60, 90, 140]));
    image::codecs::jpeg::JpegEncoder::new(&mut jpeg)
        .encode_image(&img)
        .unwrap();
    chat.apply(&Event::Snapshot {
        id: "S3".to_owned(),
        jpeg: Arc::new(jpeg),
        width: 96,
        height: 54,
        marks: vec!["red mug".to_owned(), "fruit bowl".to_owned()],
    });
    chat.apply(&Event::Notice {
        text: "The robot is running mission m7, at s2_GoToPlace, started before this session. \
               It carries on unwatched until it ends; Stop ends it now."
            .to_owned(),
    });
    chat.apply(&Event::Error {
        turn: 2,
        text: "every model failed: qwen3.5-9b-local: connection refused, and \
               qwen3.8-27b-or: the free tier's daily quota is spent"
            .to_owned(),
    });
    render(chat, "chat_error_image");
}
