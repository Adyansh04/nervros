use std::borrow::Cow;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use rig::test_utils::{MockCompletionModel, MockTurn};
use serde_json::{Value, json};
use tokio::sync::broadcast;

use super::approval::{Unanswered, take_unanswered};
use super::config::SessionConfig;
use super::event::{Command, Event, is_stop_word};
use super::*;
use crate::guard::{Guard, Policy};
use crate::llm::{AgentBuilder, AgentSource, History, LlmError};
use crate::lock;
use crate::providers::Role;
use crate::providers::router::Need;
use crate::tools::{Assessment, Registry, Risk, Status, Tool, ToolOutcome, ToolSpec};

/// A model every role asks, answering from its script; it streams, holds to a quota and notes
/// each time it is set aside, as a test sets it.
struct Scripted {
    model: MockCompletionModel,
    streams: bool,
    /// Requests the quota grants; no limit when `None`.
    allowed: Option<usize>,
    /// Requests granted so far.
    taken: std::sync::atomic::AtomicUsize,
    /// How long it was set aside, each time.
    parked: Mutex<Vec<Duration>>,
}

impl Scripted {
    fn new(model: MockCompletionModel) -> Self {
        Self {
            model,
            streams: false,
            allowed: None,
            taken: std::sync::atomic::AtomicUsize::new(0),
            parked: Mutex::default(),
        }
    }
}

impl AgentSource for Scripted {
    fn candidates(&self, _role: Role, _need: Need) -> Vec<String> {
        vec!["mock".into()]
    }
    fn builder(&self, _id: &str) -> Result<AgentBuilder, LlmError> {
        Ok(AgentBuilder::new(self.model.clone()))
    }
    fn take_request(&self, _id: &str) -> Result<(), String> {
        let allowed = self.allowed.unwrap_or(usize::MAX);
        self.taken
            .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |n| {
                (n < allowed).then_some(n + 1)
            })
            .map(|_| ())
            .map_err(|_| "its daily limit is used up".to_owned())
    }
    fn park(&self, _id: &str, for_how_long: Duration) {
        lock(&self.parked).push(for_how_long);
    }
    fn streams(&self, _id: &str) -> bool {
        self.streams
    }
}

fn metered(allowed: usize) -> Arc<Scripted> {
    Arc::new(Scripted {
        allowed: Some(allowed),
        ..Scripted::new(MockCompletionModel::from_turns([
            MockTurn::tool_call("c1", "find_objects", json!({"query": "cup"})),
            MockTurn::text("The cup is in the kitchen."),
        ]))
    })
}

async fn ask_where_the_cup_is(source: &Arc<Scripted>) -> Vec<Event> {
    let session = Session::start(
        Arc::clone(source) as Arc<dyn AgentSource>,
        registry(Risk::Observe),
        Arc::new(Guard::new(Policy::default())),
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("Where is the cup?".into()));
    collect_until_finished(&mut rx, |_| None, &session).await
}

#[tokio::test]
async fn every_request_of_a_turn_counts_against_the_quota() {
    let source = metered(10);
    let events = ask_where_the_cup_is(&source).await;
    assert!(events.iter().any(|e| matches!(e, Event::Reply { .. })));
    assert_eq!(
        source.taken.load(Ordering::SeqCst),
        2,
        "the tool call and the reply are two requests"
    );
    let costed = events
        .iter()
        .filter(|e| matches!(e, Event::ModelCall { model, turn: 1, .. } if model == "mock"))
        .count();
    assert_eq!(costed, 2, "each request's cost is logged: {events:?}");
}

#[tokio::test]
async fn only_a_real_429_sets_the_model_aside_streamed_or_not() {
    use rig::test_utils::MockStreamEvent;
    let replies = MockCompletionModel::from_turns([
        MockTurn::provider_response_error(
            reqwest::StatusCode::BAD_REQUEST,
            r#"{"error": {"message": "you asked for 14290 tokens of 8192"}}"#,
            "req-429",
        ),
        MockTurn::provider_response_error(reqwest::StatusCode::TOO_MANY_REQUESTS, "{}", "req-2"),
    ]);
    // A streamed failure that only mentions 429 is no rate limit either.
    let streamed = MockCompletionModel::from_stream_turns([
        vec![MockStreamEvent::error(
            "status 400: asked for 14290 tokens (request id 4291)",
        )],
        vec![MockStreamEvent::error("status 400: no")],
    ]);
    for (model, streams, parks) in [(replies, false, 1), (streamed, true, 0)] {
        let source = Arc::new(Scripted {
            streams,
            ..Scripted::new(model)
        });
        let session = Session::start(
            Arc::clone(&source) as Arc<dyn AgentSource>,
            registry(Risk::Observe),
            Arc::new(Guard::new(Policy::default())),
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        for text in ["hello", "again"] {
            session.send(Command::User(text.into()));
            collect_until_finished(&mut rx, |_| None, &session).await;
        }
        let parked = lock(&source.parked).clone();
        assert_eq!(parked.len(), parks, "streams: {streams}");
        assert!(parked.iter().all(|d| *d == Duration::from_mins(1)));
    }
}

#[tokio::test]
async fn a_spent_quota_ends_the_turn() {
    let source = metered(1);
    let events = ask_where_the_cup_is(&source).await;
    assert!(!events.iter().any(|e| matches!(e, Event::Reply { .. })));
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::Error { text, .. } if text.contains("daily limit"))),
        "{events:?}"
    );
    assert_eq!(source.taken.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn a_synchronous_act_leaves_the_tools_offered_to_check_its_effect() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "find_objects", json!({})),
        MockTurn::text("Done."),
    ]);
    let guard = Arc::new(Guard::new(Policy::default()));
    guard.set_armed(true);
    let session = Session::start(
        Arc::new(Scripted::new(model.clone())),
        registry(Risk::Motion),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("reset it".into()));
    let approve = |e: &Event| match e {
        Event::ApprovalRequested { id, .. } => Some(Command::Approve(*id)),
        _ => None,
    };
    collect_until_finished(&mut rx, approve, &session).await;
    let offered: Vec<usize> = model.requests().iter().map(|r| r.tools.len()).collect();
    assert_eq!(offered, [1, 1]);
}

#[tokio::test]
async fn once_a_mission_has_started_the_model_is_offered_no_tools() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "find_objects", json!({})),
        MockTurn::text("Started."),
    ]);
    let guard = Arc::new(Guard::new(Policy::default()));
    guard.set_armed(true);
    let mut r = Registry::default();
    let spec = ToolSpec::new(
        "find_objects",
        "Starts.",
        json!({"type": "object"}),
        Risk::Motion,
    );
    r.add(Arc::new(Starter(spec))).unwrap();
    let session = Session::start(
        Arc::new(Scripted::new(model.clone())),
        Arc::new(r),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("go".into()));
    let approve = |e: &Event| match e {
        Event::ApprovalRequested { id, .. } => Some(Command::Approve(*id)),
        _ => None,
    };
    let events = collect_until_finished(&mut rx, approve, &session).await;
    assert!(events.iter().any(|e| matches!(e, Event::Reply { .. })));
    let offered: Vec<usize> = model.requests().iter().map(|r| r.tools.len()).collect();
    assert_eq!(
        offered,
        [1, 0],
        "the reply after the act is asked for without tools"
    );
}

#[tokio::test]
async fn an_invented_tool_name_gets_the_real_ones_back() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "default_api", json!({})),
        MockTurn::text("The cup is in the kitchen."),
    ]);
    let session = Session::start(
        Arc::new(Scripted::new(model.clone())),
        registry(Risk::Observe),
        Arc::new(Guard::new(Policy::default())),
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("Where is the cup?".into()));
    let events = collect_until_finished(&mut rx, |_| None, &session).await;
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::Reply { text, .. } if text.contains("kitchen"))),
        "{events:?}"
    );
    let feedback = format!("{:?}", model.requests()[1].chat_history);
    assert!(feedback.contains("find_objects"), "{feedback}");
}

struct Echo(ToolSpec);

#[async_trait]
impl Tool for Echo {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.0)
    }
    async fn call(&self, args: Value) -> ToolOutcome {
        ToolOutcome::ok(json!({"echo": args}))
    }
}

/// Starts something in the background, as `run_mission` does.
struct Starter(ToolSpec);

#[async_trait]
impl Tool for Starter {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.0)
    }
    async fn call(&self, _args: Value) -> ToolOutcome {
        ToolOutcome {
            status: Status::Accepted,
            ..ToolOutcome::ok(json!({"mission_id": "m1"}))
        }
    }
}

fn registry(risk: Risk) -> Arc<Registry> {
    let mut r = Registry::default();
    let spec = ToolSpec::new(
        "find_objects",
        "Finds things.",
        json!({"type": "object"}),
        risk,
    );
    r.add(Arc::new(Echo(spec))).unwrap();
    Arc::new(r)
}

async fn collect_until_finished(
    rx: &mut broadcast::Receiver<Event>,
    on: impl Fn(&Event) -> Option<Command>,
    session: &Session,
) -> Vec<Event> {
    let mut out = Vec::new();
    loop {
        let e = tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .unwrap()
            .unwrap();
        if let Some(cmd) = on(&e) {
            session.send(cmd);
        }
        let done = matches!(e, Event::TurnFinished { .. });
        out.push(e);
        if done {
            return out;
        }
    }
}

#[tokio::test]
async fn a_turn_calls_a_tool_and_replies() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "find_objects", json!({"query": "cup"})),
        MockTurn::text("The cup is in the kitchen."),
    ]);
    let guard = Arc::new(Guard::new(Policy::default()));
    let session = Session::start(
        Arc::new(Scripted::new(model)),
        registry(Risk::Observe),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("Where is the cup?".into()));
    let events = collect_until_finished(&mut rx, |_| None, &session).await;
    assert!(events.iter().any(|e| matches!(e, Event::ToolFinished { tool, status: Status::Succeeded, .. } if tool == "find_objects")));
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::Reply { text, .. } if text.contains("kitchen")))
    );
}

#[tokio::test]
async fn a_streamed_reply_arrives_in_pieces_then_whole() {
    use rig::test_utils::MockStreamEvent;
    let model = MockCompletionModel::from_stream_turns([vec![
        MockStreamEvent::text("The cup "),
        MockStreamEvent::text("is in the kitchen."),
        MockStreamEvent::final_response_with_default_usage(),
    ]]);
    let guard = Arc::new(Guard::new(Policy::default()));
    let session = Session::start(
        Arc::new(Scripted {
            streams: true,
            ..Scripted::new(model)
        }),
        registry(Risk::Observe),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("Where is the cup?".into()));
    let events = collect_until_finished(&mut rx, |_| None, &session).await;
    let pieces: Vec<&str> = events
        .iter()
        .filter_map(|e| match e {
            Event::ReplyDelta { text, .. } => Some(text.as_str()),
            _ => None,
        })
        .collect();
    assert_eq!(pieces, ["The cup ", "is in the kitchen."]);
    assert!(
        events.iter().any(
            |e| matches!(e, Event::Reply { text, .. } if text == "The cup is in the kitchen.")
        )
    );
}

#[tokio::test]
async fn the_operator_compacts_and_resumes_a_conversation() {
    let model = MockCompletionModel::from_turns([MockTurn::text("They asked for ten things.")]);
    let config = SessionConfig {
        resume: Some(History::sample(10, 3000)),
        ..SessionConfig::default()
    };
    let guard = Arc::new(Guard::new(Policy::default()));
    let session = Session::start(
        Arc::new(Scripted::new(model)),
        registry(Risk::Observe),
        guard,
        None,
        config,
    );
    let mut rx = session.subscribe();
    session.send(Command::Compact);
    let events = collect_until_finished(&mut rx, |_| None, &session).await;
    assert!(
        events.iter().any(
            |e| matches!(e, Event::Compacted { summarised: true, before, after } if after < before)
        ),
        "{events:?}"
    );
    session.send(Command::CompactUpTo { keep: 1 });
    let events = collect_until_finished(&mut rx, |_| None, &session).await;
    assert!(
        events.iter().any(|e| matches!(
            e,
            Event::Compacted {
                summarised: false,
                ..
            }
        )),
        "a second summary needs a model turn the script lacks, so it only cuts: {events:?}"
    );
    session.send(Command::Restore(History::sample(2, 10)));
    let restored = tokio::time::timeout(Duration::from_secs(5), rx.recv())
        .await
        .unwrap()
        .unwrap();
    assert!(matches!(restored, Event::Restored { exchanges } if exchanges.len() == 4));
}

struct Slow(ToolSpec, Duration);

#[async_trait]
impl Tool for Slow {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.0)
    }
    async fn call(&self, _args: Value) -> ToolOutcome {
        tokio::time::sleep(self.1).await;
        ToolOutcome::ok(json!({}))
    }
}

#[tokio::test]
async fn a_message_sent_while_a_report_is_answered_runs_after_it() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "find_objects", json!({})),
        MockTurn::text("noted"),
        MockTurn::text("hello"),
    ]);
    let mut r = Registry::default();
    let spec = ToolSpec::new(
        "find_objects",
        "Slow.",
        json!({"type": "object"}),
        Risk::Observe,
    );
    r.add(Arc::new(Slow(spec, Duration::from_millis(300))))
        .unwrap();
    let guard = Arc::new(Guard::new(Policy::default()));
    let session = Session::start(
        Arc::new(Scripted::new(model)),
        Arc::new(r),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::Report("the camera slowed".into()));
    let mut events = Vec::new();
    let mut finished = 0;
    while finished < 2 {
        let e = tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .unwrap()
            .unwrap();
        if matches!(e, Event::ToolStarted { .. }) {
            session.send(Command::User("hi".into()));
        }
        finished += usize::from(matches!(e, Event::TurnFinished { .. }));
        events.push(e);
    }
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::Notice { text } if text.starts_with("queued")))
    );
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::User { turn: 2, text } if text == "hi"))
    );
    assert!(
        matches!(events.iter().rev().nth(1), Some(Event::Reply { text, .. }) if text == "hello")
    );
}

#[tokio::test]
async fn a_turn_over_its_time_limit_is_stopped() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "find_objects", json!({})),
        MockTurn::text("done"),
    ]);
    let mut r = Registry::default();
    let spec = ToolSpec::new(
        "find_objects",
        "Slow.",
        json!({"type": "object"}),
        Risk::Observe,
    );
    r.add(Arc::new(Slow(spec, Duration::from_secs(5)))).unwrap();
    let config = SessionConfig {
        turn_time: Duration::from_millis(200),
        ..SessionConfig::default()
    };
    let guard = Arc::new(Guard::new(Policy::default()));
    let session = Session::start(
        Arc::new(Scripted::new(model)),
        Arc::new(r),
        guard,
        None,
        config,
    );
    let mut rx = session.subscribe();
    session.send(Command::User("go".into()));
    let events = collect_until_finished(&mut rx, |_| None, &session).await;
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::Error { text, .. } if text.contains("longer than")))
    );
    assert!(!events.iter().any(|e| matches!(e, Event::Reply { .. })));
}

/// Two calls of the one tool in one turn, the second with other arguments.
fn twice() -> MockCompletionModel {
    MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "find_objects", json!({"n": 1})),
        MockTurn::tool_call("c2", "find_objects", json!({"n": 2})),
        MockTurn::text("done"),
    ])
}

#[tokio::test]
async fn a_tool_allowed_for_the_session_stops_asking_unless_it_moves_the_robot() {
    for (risk, asked) in [(Risk::WorldEdit, 1), (Risk::Motion, 2)] {
        let guard = Arc::new(Guard::new(Policy::default()));
        guard.set_armed(true);
        let session = Session::start(
            Arc::new(Scripted::new(twice())),
            registry(risk),
            guard,
            None,
            SessionConfig::default(),
        );
        let mut rx = session.subscribe();
        session.send(Command::User("go".into()));
        let allow = |e: &Event| match e {
            Event::ApprovalRequested { id, .. } => Some(Command::AllowForSession(*id)),
            _ => None,
        };
        let events = collect_until_finished(&mut rx, allow, &session).await;
        let n = events
            .iter()
            .filter(|e| matches!(e, Event::ApprovalRequested { .. }))
            .count();
        assert_eq!(n, asked, "{risk:?}: {events:?}");
        let ran = events
            .iter()
            .filter(|e| {
                matches!(
                    e,
                    Event::ToolFinished {
                        status: Status::Succeeded,
                        ..
                    }
                )
            })
            .count();
        assert_eq!(ran, 2, "{risk:?}");
    }
}

#[tokio::test]
async fn a_request_left_waiting_is_kept_for_the_next_session_and_dropped_once_answered() {
    let dir = tempfile::tempdir().unwrap();
    let file = dir.path().join("pending.json");
    let guard = Arc::new(Guard::new(Policy::default()));
    guard.set_armed(true);
    let config = SessionConfig {
        pending_file: Some(file.clone()),
        ..SessionConfig::default()
    };
    let session = Session::start(
        Arc::new(Scripted::new(twice())),
        registry(Risk::WorldEdit),
        guard,
        None,
        config,
    );
    let mut rx = session.subscribe();
    session.send(Command::User("go".into()));
    let id = loop {
        if let Event::ApprovalRequested { id, .. } = rx.recv().await.unwrap() {
            break id;
        }
    };
    let kept: Vec<Unanswered> = serde_json::from_slice(&std::fs::read(&file).unwrap()).unwrap();
    assert_eq!(kept.len(), 1);
    assert_eq!(
        (kept[0].tool.as_str(), &kept[0].args),
        ("find_objects", &json!({"n": 1}))
    );
    session.send(Command::Deny(id));
    let deny = |e: &Event| match e {
        Event::ApprovalRequested { id, .. } => Some(Command::Deny(*id)),
        _ => None,
    };
    collect_until_finished(&mut rx, deny, &session).await;
    assert!(!file.exists(), "answered, nothing waits");

    std::fs::write(&file, serde_json::to_vec(&kept).unwrap()).unwrap();
    assert_eq!(take_unanswered(&file), kept);
    assert!(take_unanswered(&file).is_empty(), "offered once");
}

#[tokio::test]
async fn act_tools_are_refused_while_disarmed_and_run_after_approval() {
    let script = || {
        MockCompletionModel::from_turns([
            MockTurn::tool_call("c1", "find_objects", json!({})),
            MockTurn::text("done"),
        ])
    };
    let guard = Arc::new(Guard::new(Policy::default()));
    let session = Session::start(
        Arc::new(Scripted::new(script())),
        registry(Risk::Motion),
        Arc::clone(&guard),
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("go".into()));
    let events = collect_until_finished(&mut rx, |_| None, &session).await;
    assert!(events.iter().any(|e| matches!(
        e,
        Event::ToolFinished {
            status: Status::Refused,
            ..
        }
    )));

    guard.set_armed(true);
    let session = Session::start(
        Arc::new(Scripted::new(script())),
        registry(Risk::Motion),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("go".into()));
    let approve = |e: &Event| match e {
        Event::ApprovalRequested { id, .. } => Some(Command::Approve(*id)),
        _ => None,
    };
    let events = collect_until_finished(&mut rx, approve, &session).await;
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::ApprovalResolved { approved: true, .. }))
    );
    assert!(events.iter().any(|e| matches!(
        e,
        Event::ToolFinished {
            status: Status::Succeeded,
            ..
        }
    )));
}

#[tokio::test]
async fn stop_ends_the_turn_and_denies_pending_approvals() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "find_objects", json!({})),
        MockTurn::text("done"),
    ]);
    let guard = Arc::new(Guard::new(Policy {
        start_armed: true,
        ..Policy::default()
    }));
    let session = Session::start(
        Arc::new(Scripted::new(model)),
        registry(Risk::Motion),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("go".into()));
    let stop =
        |e: &Event| matches!(e, Event::ApprovalRequested { .. }).then_some(Command::StopMission);
    let events = collect_until_finished(&mut rx, stop, &session).await;
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::TurnFinished { .. }))
    );
    let halted = tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            if let Ok(Event::Halted { .. }) = rx.recv().await {
                return;
            }
        }
    });
    assert!(halted.await.is_ok() || events.iter().any(|e| matches!(e, Event::Halted { .. })));
}

/// Events after the ones already read, until `done` holds for one or a second passes.
async fn more_until(
    rx: &mut broadcast::Receiver<Event>,
    done: impl Fn(&[Event]) -> bool,
) -> Vec<Event> {
    let mut out = Vec::new();
    let _ = tokio::time::timeout(Duration::from_secs(1), async {
        while let Ok(e) = rx.recv().await {
            out.push(e);
            if done(&out) {
                return;
            }
        }
    })
    .await;
    out
}

#[test]
fn a_plain_order_to_stop_is_known_and_a_sentence_about_stopping_is_not() {
    for word in [
        "stop",
        "STOP",
        "Stop!",
        "halt.",
        " freeze ",
        "please stop",
        "stop the robot!",
    ] {
        assert!(is_stop_word(word), "{word}");
    }
    for text in ["stop at the kitchen", "don't stop", "bus stop", "stopwatch"] {
        assert!(!is_stop_word(text), "{text}");
    }
}

#[tokio::test]
async fn a_stop_closes_what_it_cut_short_and_forgets_the_request() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "find_objects", json!({})),
        MockTurn::text("done"),
    ]);
    let guard = Arc::new(Guard::new(Policy {
        start_armed: true,
        ..Policy::default()
    }));
    let dir = tempfile::tempdir().unwrap();
    let pending = dir.path().join("pending.json");
    let session = Session::start(
        Arc::new(Scripted::new(model)),
        registry(Risk::Motion),
        guard,
        None,
        SessionConfig {
            pending_file: Some(pending.clone()),
            ..SessionConfig::default()
        },
    );
    let mut rx = session.subscribe();
    session.send(Command::User("go".into()));
    let stop =
        |e: &Event| matches!(e, Event::ApprovalRequested { .. }).then_some(Command::StopMission);
    let mut events = collect_until_finished(&mut rx, stop, &session).await;
    let closed = |events: &[Event]| {
        events.iter().any(|e| {
            matches!(
                e,
                Event::ApprovalResolved {
                    approved: false,
                    ..
                }
            )
        }) && events.iter().any(|e| {
            matches!(
                e,
                Event::ToolFinished {
                    status: Status::Stopped,
                    ..
                }
            )
        })
    };
    if !closed(&events) {
        events.extend(more_until(&mut rx, closed).await);
    }
    assert!(closed(&events), "{events:?}");
    assert!(!pending.exists(), "a stopped request is not asked again");
}

#[tokio::test]
async fn disarming_turns_down_a_waiting_approval_however_it_is_answered() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "find_objects", json!({})),
        MockTurn::text("not moving"),
    ]);
    let guard = Arc::new(Guard::new(Policy {
        start_armed: true,
        ..Policy::default()
    }));
    let session = Session::start(
        Arc::new(Scripted::new(model)),
        registry(Risk::Motion),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("go".into()));
    let mut events = Vec::new();
    loop {
        let e = tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .unwrap()
            .unwrap();
        if let Event::ApprovalRequested { id, .. } = &e {
            // Off, then approved: the approval comes too late to move anything.
            session.send(Command::Disarm);
            session.send(Command::Approve(*id));
        }
        let done = matches!(e, Event::TurnFinished { .. });
        events.push(e);
        if done {
            break;
        }
    }
    let status = events.iter().find_map(|e| match e {
        Event::ToolFinished { status, .. } => Some(*status),
        _ => None,
    });
    assert_eq!(status, Some(Status::Refused), "{events:?}");
}

#[tokio::test]
async fn the_model_can_always_stop_the_robot() {
    let stop_spec = ToolSpec::new("stop", "Stops.", json!({"type": "object"}), Risk::Observe);
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "find_objects", json!({})),
        MockTurn::tool_call("c2", "stop", json!({"reason": null})),
        MockTurn::tool_call("c3", "stop", json!({"reason": null})),
        MockTurn::text("stopped"),
    ]);
    let guard = Arc::new(Guard::new(Policy {
        budgets: crate::guard::Budgets {
            tool_calls: 1,
            repeat_break: 2,
            ..crate::guard::Budgets::default()
        },
        ..Policy::default()
    }));
    let stop: Arc<dyn Tool> = Arc::new(Echo(stop_spec));
    let mut r = Registry::default();
    r.add(Arc::new(Echo(ToolSpec::new(
        "find_objects",
        "Finds things.",
        json!({"type": "object"}),
        Risk::Observe,
    ))))
    .unwrap();
    r.add(Arc::clone(&stop)).unwrap();
    let session = Session::start(
        Arc::new(Scripted::new(model)),
        Arc::new(r),
        Arc::clone(&guard),
        Some(stop),
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("look, then stop".into()));
    let events = collect_until_finished(&mut rx, |_| None, &session).await;
    let stops: Vec<Status> = events
        .iter()
        .filter_map(|e| match e {
            Event::ToolFinished { tool, status, .. } if tool == "stop" => Some(*status),
            _ => None,
        })
        .collect();
    assert_eq!(
        stops,
        [Status::Succeeded, Status::Succeeded],
        "past the budget, twice the same"
    );
    assert_eq!(guard.stops(), 2);
}

#[tokio::test]
async fn a_reply_lost_after_acting_keeps_the_request_in_the_history() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "run_mission", json!({})),
        MockTurn::error("the provider is down"),
        MockTurn::text("It is still running."),
    ]);
    let spec = ToolSpec::new(
        "run_mission",
        "Runs.",
        json!({"type": "object"}),
        Risk::Motion,
    );
    let guard = Arc::new(Guard::new(Policy {
        start_armed: true,
        autonomy: crate::guard::Autonomy::Autonomous,
        ..Policy::default()
    }));
    let session = Session::start(
        Arc::new(Scripted::new(model.clone())),
        one_tool(Arc::new(Starter(spec))),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("bring me the cup".into()));
    collect_until_finished(&mut rx, |_| None, &session).await;
    session.send(Command::User("is it done?".into()));
    collect_until_finished(&mut rx, |_| None, &session).await;
    let seen = format!("{:?}", model.requests().last().unwrap().chat_history);
    assert!(seen.contains("bring me the cup"), "{seen}");
    assert!(seen.contains("run_mission accepted"), "{seen}");
}

/// Checks an edit as `run_mission` checks an edited plan: `{"ok": true}` passes as a new hash.
struct Editable(ToolSpec);

#[async_trait]
impl Tool for Editable {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.0)
    }
    async fn assess_operator(&self, edited: Value) -> Option<Result<Assessment, ToolOutcome>> {
        if edited["slow"] == true {
            tokio::time::sleep(Duration::from_millis(200)).await;
        }
        Some(if edited["ok"] == true {
            Ok(Assessment {
                risk: Risk::Motion,
                resources: Vec::new(),
                reason: "runs the edited plan".into(),
                args: Some(json!({"hash": "edited"})),
            })
        } else {
            Err(ToolOutcome::failed("s1: no such place"))
        })
    }
    async fn call(&self, args: Value) -> ToolOutcome {
        ToolOutcome::ok(json!({"ran": args}))
    }
}

#[tokio::test]
async fn an_edit_that_fails_its_checks_keeps_the_request_and_one_that_passes_replaces_it() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "run_mission", json!({"hash": "planned"})),
        MockTurn::text("Done."),
    ]);
    let guard = Arc::new(Guard::new(Policy::default()));
    guard.set_armed(true);
    let mut r = Registry::default();
    let spec = ToolSpec::new(
        "run_mission",
        "Runs.",
        json!({"type": "object"}),
        Risk::Motion,
    );
    r.add(Arc::new(Editable(spec))).unwrap();
    let session = Session::start(
        Arc::new(Scripted::new(model.clone())),
        Arc::new(r),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("go".into()));
    let operator = |e: &Event| match e {
        Event::ApprovalRequested { id, .. } => Some(Command::Edit {
            id: *id,
            args: json!({"ok": false}),
        }),
        Event::EditRejected { id, .. } => Some(Command::Edit {
            id: *id,
            args: json!({"ok": true}),
        }),
        Event::ApprovalEdited { id, .. } => Some(Command::Approve(*id)),
        _ => None,
    };
    let events = collect_until_finished(&mut rx, operator, &session).await;
    assert!(
        events.iter().any(
            |e| matches!(e, Event::EditRejected { message, .. } if message == "s1: no such place")
        ),
        "{events:?}"
    );
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::ApprovalEdited { args, .. } if args["hash"] == "edited")),
        "{events:?}"
    );
    let resolved: Vec<bool> = events
        .iter()
        .filter_map(|e| match e {
            Event::ApprovalResolved { approved, .. } => Some(*approved),
            _ => None,
        })
        .collect();
    assert_eq!(resolved, [true], "one request, answered once");
    let ran = format!("{:?}", model.requests()[1].chat_history);
    assert!(ran.contains("edited"), "it ran with the edit: {ran}");
}

#[tokio::test]
async fn an_approval_sent_while_an_edit_is_checked_does_not_approve_the_edit() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "run_mission", json!({"hash": "planned"})),
        MockTurn::text("Not run."),
    ]);
    let guard = Arc::new(Guard::new(Policy::default()));
    guard.set_armed(true);
    let mut r = Registry::default();
    let spec = ToolSpec::new(
        "run_mission",
        "Runs.",
        json!({"type": "object"}),
        Risk::Motion,
    );
    r.add(Arc::new(Editable(spec))).unwrap();
    let session = Session::start(
        Arc::new(Scripted::new(model)),
        Arc::new(r),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("go".into()));
    let mut resolved = Vec::new();
    loop {
        let e = tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .unwrap()
            .unwrap();
        match e {
            Event::ApprovalRequested { id, .. } => {
                let args = json!({"ok": true, "slow": true});
                session.send(Command::Edit { id, args });
                session.send(Command::Approve(id));
            }
            // Not approved after the edit: nothing may run.
            Event::ApprovalEdited { id, .. } => session.send(Command::Deny(id)),
            Event::ApprovalResolved { approved, .. } => resolved.push(approved),
            Event::TurnFinished { .. } => break,
            _ => {}
        }
    }
    assert_eq!(resolved, [false]);
}

#[tokio::test]
async fn the_operator_runs_a_tool_checked_as_their_own_and_approves_it() {
    let guard = Arc::new(Guard::new(Policy::default()));
    let mut r = Registry::default();
    let spec = ToolSpec::new(
        "run_mission",
        "Runs.",
        json!({"type": "object"}),
        Risk::Motion,
    );
    r.add(Arc::new(Editable(spec))).unwrap();
    let session = Session::start(
        Arc::new(Scripted::new(MockCompletionModel::from_turns([]))),
        Arc::new(r),
        Arc::clone(&guard),
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    let run = || Command::Run {
        tool: "run_mission".into(),
        args: json!({"ok": true}),
    };
    let mut next = async || {
        tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .unwrap()
            .unwrap()
    };

    session.send(run());
    let refused = loop {
        if let Event::ToolFinished {
            status, message, ..
        } = next().await
        {
            break (status, message);
        }
    };
    assert_eq!(refused.0, Status::Refused);
    assert!(refused.1.contains("arm it"), "{}", refused.1);

    guard.set_armed(true);
    session.send(run());
    loop {
        match next().await {
            Event::ToolStarted { args, .. } => {
                assert_eq!(args["hash"], "edited", "run as checked");
            }
            Event::ApprovalRequested { id, .. } => session.send(Command::Approve(id)),
            Event::ToolFinished { status, .. } => {
                assert_eq!(status, Status::Succeeded);
                break;
            }
            _ => {}
        }
    }
}

/// Fails the same way every time, whatever it is asked.
struct Broken(ToolSpec);

#[async_trait]
impl Tool for Broken {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.0)
    }
    async fn call(&self, _args: Value) -> ToolOutcome {
        ToolOutcome::failed("the camera is not publishing")
    }
}

fn one_tool(tool: Arc<dyn Tool>) -> Arc<Registry> {
    let mut r = Registry::default();
    r.add(tool).unwrap();
    Arc::new(r)
}

#[tokio::test]
async fn after_three_denials_in_a_row_the_model_is_told_to_stop_asking() {
    let calls: Vec<MockTurn> = (1..=4)
        .map(|n| MockTurn::tool_call(format!("c{n}"), "find_objects", json!({"n": n})))
        .chain([MockTurn::text("I will ask what you want.")])
        .collect();
    let model = MockCompletionModel::from_turns(calls);
    let guard = Arc::new(Guard::new(Policy::default()));
    guard.set_armed(true);
    let session = Session::start(
        Arc::new(Scripted::new(model.clone())),
        registry(Risk::Motion),
        guard,
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("go".into()));
    let deny = |e: &Event| match e {
        Event::ApprovalRequested { id, .. } => Some(Command::Deny(*id)),
        _ => None,
    };
    let events = collect_until_finished(&mut rx, deny, &session).await;
    let asked = events
        .iter()
        .filter(|e| matches!(e, Event::ApprovalRequested { .. }))
        .count();
    assert_eq!(asked, 3, "the fourth is not asked");
    let seen = format!("{:?}", model.requests().last().unwrap().chat_history);
    assert!(seen.contains("3 times in a row"), "{seen}");
    assert!(seen.contains("turned down the last 3 requests"), "{seen}");
}

#[tokio::test]
async fn the_same_failure_three_times_in_a_row_is_called_stuck() {
    let calls: Vec<MockTurn> = (1..=3)
        .map(|n| MockTurn::tool_call(format!("c{n}"), "look", json!({"n": n})))
        .chain([MockTurn::text("The camera is stuck.")])
        .collect();
    let model = MockCompletionModel::from_turns(calls);
    let spec = ToolSpec::new("look", "Looks.", json!({"type": "object"}), Risk::Observe);
    let session = Session::start(
        Arc::new(Scripted::new(model.clone())),
        one_tool(Arc::new(Broken(spec))),
        Arc::new(Guard::new(Policy::default())),
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("look".into()));
    collect_until_finished(&mut rx, |_| None, &session).await;
    let requests = model.requests();
    let second = format!("{:?}", requests[2].chat_history);
    assert!(!second.contains("came back the same"), "twice is not stuck");
    let third = format!("{:?}", requests[3].chat_history);
    assert!(
        third.contains("came back the same 3 times in a row"),
        "{third}"
    );
}

#[tokio::test]
async fn a_message_sent_mid_turn_reaches_the_model_with_its_next_tool_result() {
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "look", json!({})),
        MockTurn::text("Looking at the kitchen too."),
    ]);
    let spec = ToolSpec::new("look", "Looks.", json!({"type": "object"}), Risk::Observe);
    let slow = Slow(spec, Duration::from_millis(300));
    let session = Session::start(
        Arc::new(Scripted::new(model.clone())),
        one_tool(Arc::new(slow)),
        Arc::new(Guard::new(Policy::default())),
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("look around".into()));
    let mut steered = false;
    loop {
        let e = tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .unwrap()
            .unwrap();
        match e {
            Event::ToolStarted { .. } => session.send(Command::User("and the kitchen".into())),
            Event::Steer { text, .. } => steered = text == "and the kitchen",
            Event::TurnFinished { .. } => break,
            _ => {}
        }
    }
    assert!(steered, "shown as said during the turn");
    let read = format!("{:?}", model.requests()[1].chat_history);
    assert!(
        read.contains("said while you worked: \\\"and the kitchen\\\""),
        "{read}"
    );
    assert_eq!(model.requests().len(), 2, "no turn of its own: it was read");
}

#[tokio::test]
async fn a_profile_rule_refuses_one_value_and_asks_before_another() {
    let policy: crate::guard::Policy = toml::from_str(
        r#"
        [[rule]]
        tool = "find_objects"
        arg = "query"
        matches = "*knife*"
        then = "deny"
        reason = "do not look for knives"
        [[rule]]
        tool = "find_objects"
        arg = "query"
        matches = "*bedroom*"
        then = "ask"
        reason = "the bedroom is private"
        "#,
    )
    .unwrap();
    let model = MockCompletionModel::from_turns([
        MockTurn::tool_call("c1", "find_objects", json!({"query": "the knife"})),
        MockTurn::tool_call("c2", "find_objects", json!({"query": "the bedroom lamp"})),
        MockTurn::text("Done."),
    ]);
    let session = Session::start(
        Arc::new(Scripted::new(model.clone())),
        registry(Risk::Observe),
        Arc::new(Guard::new(policy)),
        None,
        SessionConfig::default(),
    );
    let mut rx = session.subscribe();
    session.send(Command::User("find things".into()));
    let approve = |e: &Event| match e {
        Event::ApprovalRequested { id, .. } => Some(Command::Approve(*id)),
        _ => None,
    };
    let events = collect_until_finished(&mut rx, approve, &session).await;
    let finished: Vec<(Status, &str)> = events
        .iter()
        .filter_map(|e| match e {
            Event::ToolFinished {
                status, message, ..
            } => Some((*status, message.as_str())),
            _ => None,
        })
        .collect();
    assert_eq!(finished[0], (Status::Refused, "do not look for knives"));
    assert_eq!(finished[1].0, Status::Succeeded, "asked, then run");
    assert!(events.iter().any(
        |e| matches!(e, Event::ApprovalRequested { reason, .. } if reason == "the bedroom is private")
    ));
}
