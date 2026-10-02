use std::path::Path;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::fake::{FakeRobot, ScriptedRun};
use nervros_ros::{GoalResult, GoalStatus, RobotPort};
use serde_json::{Value, json};

use super::execution::step_of;
use super::ledger::Ledger;
use super::plan::Author as By;
use super::sanity::Critic;
use super::{advice, history, *};
use crate::lock;
use crate::mission::catalog::tests::CATALOG;
use crate::places::Places;
use crate::profile::Profile;
use crate::session::{Command, Event, SessionHandle};
use crate::tools::{Risk, Status};

fn profile() -> Profile {
    profile_with("")
}

/// The test profile with `mission` lines added to its `[mission]` section.
fn profile_with(mission: &str) -> Profile {
    let text = r#"
        [robot]
        name = "t"
        [world]
        objects = { topic = "/objects", type = "canopy_msgs/msg/WorldObjectArray" }
        history = "/x/history"
        [mission]
        execute = "/x/execute"
        validate = "/x/validate"
        catalog = "/x/catalog"
        stop = "/x/stop"
        state = "/x/state"
        max_replans = 1
        checks = { turn = { skill = "TurnInPlace", degrees = "degrees" }, arm = "arm" }
        MISSION_EXTRA
        [[place]]
        name = "dock"
        pose = { x = 1.0, y = 2.0 }
        [models]
        file = "m.toml"
    "#;
    Profile::from_toml(
        &text.replace("MISSION_EXTRA", mission),
        Path::new("nervros.toml"),
    )
    .unwrap()
}

fn robot(run: ScriptedRun) -> FakeRobot {
    let run = Arc::new(Mutex::new(Some(run)));
    FakeRobot::new()
        .with_service("/x/catalog", |_| Ok(json!({"catalog_json": CATALOG})))
        .with_service("/x/validate", |req| {
            let xml = req["tree_xml"].as_str().unwrap_or_default();
            Ok(json!({"ok": !xml.contains("Parallel"), "diagnostics_json": "[]", "worst_case_duration_s": 0.0}))
        })
        .with_action("/x/execute", move |_| lock(&run).take().unwrap_or_default())
        .with_topic(
            "/objects",
            json!({"objects": [
                {"id": "O17", "label": "red mug", "state": 0,
                 "pose": {"position": {"x": 1.0, "y": 2.0}}, "size": {"x": 0.1, "y": 0.1}},
                {"id": "O18", "label": "blue cup", "state": 0,
                 "pose": {"position": {"x": 1.1, "y": 2.0}}, "size": {"x": 0.1, "y": 0.1}}]}),
        )
        .with_topic("/x/state", json!({"holding_left": "", "holding_right": "O17"}))
        .with_transform(
            "map",
            "base_footprint",
            nervros_ros::Transform {
                translation: [1.1, 2.0, 0.0],
                ..nervros_ros::Transform::IDENTITY
            },
        )
}

fn steps() -> Value {
    json!({"intent": "fetch the mug", "goal": ["at(dock)", "holding(right, O17)"], "steps": [
        {"skill": "GoToPlace", "args": [{"name": "place", "value": "dock"}]},
        {"skill": "PickObject", "args": {"object_id": "O18", "phrase": "blue cup", "arm": "left"}}
    ]})
}

/// Waits up to five seconds for `ready`.
async fn eventually(mut ready: impl FnMut() -> bool) {
    let wait = async {
        while !ready() {
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    };
    tokio::time::timeout(Duration::from_secs(5), wait)
        .await
        .expect("it never happened");
}

#[tokio::test]
async fn a_mission_is_kept_recalled_saved_by_name_and_run_again_and_gaps_are_logged() {
    let run = || ScriptedRun {
        result: Ok(GoalResult {
            status: GoalStatus::Succeeded,
            result: json!({"outcome": 0, "failed_step_id": "", "failure_reason": ""}),
        }),
        ..ScriptedRun::default()
    };
    let robot: Arc<dyn RobotPort> = Arc::new(robot(run()));
    let ledger = Ledger::in_memory().unwrap();
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot)
        .unwrap()
        .with_ledger(Arc::clone(&ledger));
    let (tx, _commands) = tokio::sync::mpsc::unbounded_channel();
    let (events, _rx) = tokio::sync::broadcast::channel(64);
    missions.attach(SessionHandle::for_tests(&tx, events.clone()));
    let _ = events.send(Event::User {
        turn: 1,
        text: "fetch me the mug from the dock".to_owned(),
    });
    eventually(|| missions.request().contains("mug")).await;

    let hash = missions.plan(steps(), By::Model).await.data["hash"]
        .as_str()
        .unwrap()
        .to_owned();
    assert_eq!(
        missions.run(&json!({"hash": hash})).await.status,
        Status::Accepted
    );
    eventually(|| ledger.recent(1).is_ok_and(|r| !r.is_empty())).await;
    let kept = &ledger.recent(1).unwrap()[0];
    assert_eq!(
        (kept.outcome.as_str(), kept.request.as_str()),
        ("success", "fetch me the mug from the dock")
    );

    let [recall, plans, gap] = history::tools(&missions);
    let lately = recall.call(json!({"about": "missions"})).await;
    assert!(
        lately.data["missions"][0]
            .as_str()
            .unwrap()
            .contains("fetch the mug (success)")
    );
    let proven = recall
        .call(json!({"about": "plans", "query": "bring the mug"}))
        .await;
    assert!(
        proven.data["plans"][0]
            .as_str()
            .unwrap()
            .contains("GoToPlace(place=dock)"),
        "{}",
        proven.data
    );

    let saved = plans
        .call(json!({"action": "save", "name": "mug run", "hash": hash}))
        .await;
    assert_eq!(saved.status, Status::Succeeded, "{}", saved.message);
    let again = missions
        .plan(json!({"template": "mug run"}), By::Model)
        .await;
    assert_eq!(again.status, Status::Succeeded, "{}", again.message);
    assert_eq!(
        again.data["hash"],
        json!(hash),
        "the same plan, checked again"
    );

    gap.call(json!({"missing": "no skill puts a mug on a sofa", "nearest": "PlaceInto"}))
        .await;
    eventually(|| ledger.gaps(1).is_ok_and(|g| !g.is_empty())).await;
    assert_eq!(ledger.gaps(1).unwrap()[0].nearest, "PlaceInto");
}

fn ended(outcome: u64, step: &str, reason: &str) -> ScriptedRun {
    ScriptedRun {
        result: Ok(GoalResult {
            status: GoalStatus::Succeeded,
            result: json!({"outcome": outcome, "failed_step_id": step, "failure_reason": reason}),
        }),
        ..ScriptedRun::default()
    }
}

async fn report_from(commands: &mut tokio::sync::mpsc::UnboundedReceiver<Command>) -> String {
    match tokio::time::timeout(Duration::from_mins(1), commands.recv()).await {
        Ok(Some(Command::Report(text))) => text,
        other => panic!("no report: {other:?}"),
    }
}

#[tokio::test]
async fn two_launches_at_once_start_one_mission() {
    let robot: Arc<dyn RobotPort> =
        Arc::new(robot(ended(0, "", "")).with_latency(Duration::from_millis(200)));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let hash = missions.plan(steps(), By::Model).await.data["hash"]
        .as_str()
        .unwrap()
        .to_owned();
    let args = json!({"hash": hash});
    let (a, b) = tokio::join!(missions.run(&args), missions.run(&args));
    let started = [a.status, b.status]
        .iter()
        .filter(|s| **s == Status::Accepted)
        .count();
    assert_eq!(started, 1, "{} / {}", a.message, b.message);
}

#[tokio::test]
async fn a_mission_and_another_act_never_overlap() {
    let slow = ScriptedRun {
        step: Duration::from_millis(300),
        ..ended(0, "", "")
    };
    let robot: Arc<dyn RobotPort> = Arc::new(robot(slow));
    let guard = Arc::new(crate::guard::Guard::new(crate::guard::Policy::default()));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot)
        .unwrap()
        .with_guard(Arc::clone(&guard));
    let hash = missions.plan(steps(), By::Model).await.data["hash"]
        .as_str()
        .unwrap()
        .to_owned();
    let held = guard.lock(&[crate::tools::Resource::Base]).unwrap();
    let busy = missions.run(&json!({"hash": hash})).await;
    assert_eq!(
        busy.status,
        Status::Failed,
        "an act holds the base: {}",
        busy.message
    );
    assert!(!missions.busy(), "the slot is free again");
    drop(held);
    assert_eq!(
        missions.run(&json!({"hash": hash})).await.status,
        Status::Accepted
    );
    assert!(
        guard.lock(&[crate::tools::Resource::RightArm]).is_err(),
        "the mission holds it"
    );
    eventually(|| !missions.busy()).await;
    assert!(
        guard.lock(&crate::tools::Resource::ALL).is_ok(),
        "released at its end"
    );
}

#[tokio::test(start_paused = true)]
async fn a_mission_the_executor_no_longer_runs_is_given_up() {
    let forever = ScriptedRun {
        step: Duration::from_hours(1),
        ..ended(0, "", "")
    };
    let robot: Arc<dyn RobotPort> = Arc::new(
        robot(forever).with_topic("/x/state", json!({"mission_id": "", "holding_right": ""})),
    );
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let (tx, mut commands) = tokio::sync::mpsc::unbounded_channel();
    let (events, _rx) = tokio::sync::broadcast::channel(64);
    missions.attach(SessionHandle::for_tests(&tx, events));
    let hash = missions.plan(steps(), By::Model).await.data["hash"]
        .as_str()
        .unwrap()
        .to_owned();
    assert_eq!(
        missions.run(&json!({"hash": hash})).await.status,
        Status::Accepted
    );
    let text = report_from(&mut commands).await;
    assert!(text.contains("ended: error"), "{text}");
    assert!(!missions.busy(), "the slot is free for the next mission");
}

#[tokio::test]
async fn a_stopped_mission_is_reported_as_stopped_and_an_operator_may_run_it_again() {
    let robot: Arc<dyn RobotPort> = Arc::new(robot(ended(2, "s1", "stopped: operator")));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let (tx, mut commands) = tokio::sync::mpsc::unbounded_channel();
    let (events, _rx) = tokio::sync::broadcast::channel(64);
    missions.attach(SessionHandle::for_tests(&tx, events));
    let hash = missions.plan(steps(), By::Model).await.data["hash"]
        .as_str()
        .unwrap()
        .to_owned();
    assert_eq!(
        missions.run(&json!({"hash": hash})).await.status,
        Status::Accepted
    );
    let text = report_from(&mut commands).await;
    assert!(text.contains("It was stopped during s1"), "{text}");
    assert!(!text.contains("Find out why"), "{text}");
    assert_eq!(
        missions.run_failures.load(Ordering::SeqCst),
        0,
        "not a failure"
    );
    let model = missions.run(&json!({"hash": hash})).await;
    assert_eq!(
        model.status,
        Status::Refused,
        "the model does not repeat it on its own"
    );
    let operator = missions.run(&json!({"hash": hash, "by": "operator"})).await;
    assert_eq!(operator.status, Status::Accepted, "{}", operator.message);
}

#[tokio::test]
async fn a_run_by_a_hash_that_cannot_run_is_refused_before_anyone_is_asked() {
    let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let [run] = missions.tools();
    for args in [json!({}), json!({"hash": "0123456789abcdef"})] {
        assert!(
            matches!(run.assess(&args).await, Some(Err(out)) if out.status == Status::Refused),
            "{args}"
        );
    }
}

#[tokio::test]
async fn a_plan_runs_and_reports_with_goal_checks() {
    let run = ScriptedRun {
        feedback: vec![json!({"elapsed_s": 1.0, "events": [
            {"name": "s1_GoToPlace", "path": "Mission/s1_GoToPlace", "status": 1},
            {"name": "NavigateToPose", "path": "s1_GoToPlace/NavigateToPose", "status": 1},
            {"name": "s1_GoToPlace", "path": "s1_GoToPlace", "status": 2},
            {"name": "s1_GoToPlace", "path": "s1_GoToPlace", "status": 0}]})],
        result: Ok(GoalResult {
            status: GoalStatus::Succeeded,
            result: json!({"outcome": 0, "failed_step_id": "", "failure_reason": ""}),
        }),
        ..ScriptedRun::default()
    };
    let robot: Arc<dyn RobotPort> = Arc::new(robot(run));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let (tx, mut commands) = tokio::sync::mpsc::unbounded_channel();
    let (events, mut rx) = tokio::sync::broadcast::channel(64);
    missions.attach(SessionHandle::for_tests(&tx, events));

    let planned = missions.plan(steps(), By::Model).await;
    assert_eq!(planned.status, Status::Succeeded, "{}", planned.message);
    let hash = planned.data["hash"].as_str().unwrap().to_owned();
    let started = missions.run(&json!({"hash": &hash[..10]})).await;
    assert_eq!(started.status, Status::Accepted, "{}", started.message);

    let report = tokio::time::timeout(Duration::from_secs(5), commands.recv())
        .await
        .unwrap()
        .unwrap();
    let Command::Report(text) = report else {
        panic!("not a report")
    };
    assert!(text.contains("ended: success"), "{text}");
    // The checks come from the steps, not the model's goal list: the pick moves the base, so
    // no at(dock), and the fake's hands never change.
    assert!(
        text.contains("holding(right, O17) holds")
            && text.contains("holding(left, O18) DOES NOT hold")
            && !text.contains("at(dock)"),
        "{text}"
    );
    // Unchanged, the same plan is not run again on the model's own.
    let again = missions.run(&json!({"hash": hash})).await;
    assert_eq!(again.status, Status::Refused);
    assert!(
        again.message.contains("just ran and succeeded"),
        "{}",
        again.message
    );
    let mut seen = Vec::new();
    while let Ok(e) = rx.try_recv() {
        seen.push(e);
    }
    assert!(seen.iter().any(|e| matches!(e, Event::MissionProgress { step, node, .. } if step == "s1" && node == "NavigateToPose")));
    let last = seen.iter().rev().find_map(|e| match e {
        Event::MissionProgress { node, status, .. } if node.is_empty() => Some(status.as_str()),
        _ => None,
    });
    assert_eq!(last, Some("success"), "the reset to idle is not reported");
    assert!(seen.iter().any(|e| matches!(
        e,
        Event::MissionFinished {
            outcome: Outcome::Success,
            ..
        }
    )));
}

#[tokio::test]
async fn a_walk_is_refused_while_the_robot_cannot_move_and_not_counted_as_the_models() {
    let robot = robot(ScriptedRun::default()).with_topic(
        "/x/state",
        json!({"holding_left": "", "holding_right": "", "can_move": false,
               "cannot_move_reason": "fallen: tilted 104 degrees"}),
    );
    let robot: Arc<dyn RobotPort> = Arc::new(robot);
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();

    let out = missions.plan(steps(), By::Model).await;

    assert_eq!(out.status, Status::Refused, "{}", out.message);
    assert!(
        out.message.contains("fallen: tilted 104 degrees"),
        "{}",
        out.message
    );
    assert_eq!(missions.plan_failures.load(Ordering::SeqCst), 0);
}

#[tokio::test]
async fn a_live_heartbeat_feeds_the_deadman_and_each_mission_asks_for_it() {
    let profile = profile_with("heartbeat = \"/x/heartbeat\"\nheartbeat_timeout_s = 1.0");
    let sent = Arc::new(Mutex::new(None));
    let goals = Arc::clone(&sent);
    let fake = Arc::new(
        robot(ScriptedRun::default()).with_action("/x/execute", move |goal| {
            *lock(&goals) = Some(goal.clone());
            ScriptedRun::default()
        }),
    );
    let robot: Arc<dyn RobotPort> = fake.clone();
    let missions = Missions::new(&profile, Places::new(&profile, None), robot).unwrap();
    let (tx, _commands) = tokio::sync::mpsc::unbounded_channel();
    let (events, _rx) = tokio::sync::broadcast::channel(64);
    missions.attach(SessionHandle::for_tests(&tx, events));

    // The session's loop pulses; here the test does.
    let beating = async {
        while !fake.published().iter().any(|(t, _)| t == "/x/heartbeat") {
            crate::session::Pulse::pulse(missions.as_ref());
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    };
    tokio::time::timeout(Duration::from_secs(5), beating)
        .await
        .expect("no heartbeat was published");
    let hash = missions.plan(steps(), By::Model).await.data["hash"]
        .as_str()
        .unwrap()
        .to_owned();
    let out = missions.run(&json!({"hash": hash})).await;
    assert_eq!(out.status, Status::Accepted, "{}", out.message);
    let goal = lock(&sent).clone().expect("no goal was sent");
    assert_eq!(goal["heartbeat_timeout_s"], json!(1.0));
    assert_eq!(goal["heartbeat_client"], json!(missions.client));
    let (_, beat) = fake
        .published()
        .into_iter()
        .find(|(t, _)| t == "/x/heartbeat")
        .unwrap();
    assert_eq!(beat["client"], json!(missions.client));
}

#[tokio::test]
async fn without_a_heartbeat_a_mission_asks_for_no_deadman() {
    let sent = Arc::new(Mutex::new(None));
    let goals = Arc::clone(&sent);
    let robot = robot(ScriptedRun::default()).with_action("/x/execute", move |goal| {
        *lock(&goals) = Some(goal.clone());
        ScriptedRun::default()
    });
    let robot: Arc<dyn RobotPort> = Arc::new(robot);
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let hash = missions.plan(steps(), By::Model).await.data["hash"]
        .as_str()
        .unwrap()
        .to_owned();
    missions.run(&json!({"hash": hash})).await;
    let goal = lock(&sent).clone().expect("no goal was sent");
    assert_eq!(goal["heartbeat_timeout_s"], json!(0.0));
}

#[tokio::test]
async fn a_plan_left_unanswered_is_asked_again_as_its_steps() {
    let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let [tool] = missions.tools();
    let checked = tool.assess(&steps()).await.unwrap().unwrap();
    let again = tool.ask_again(&checked.args.unwrap()).unwrap();
    assert!(
        again["steps"].is_array() && again.get("hash").is_none(),
        "{again}"
    );
    let out = missions.plan(again, By::Operator).await;
    assert_eq!(out.status, Status::Succeeded, "{}", out.message);
    assert!(tool.ask_again(&json!({"hash": "0000000000"})).is_none());
}

#[tokio::test]
async fn a_plan_without_an_intent_takes_its_first_reason() {
    let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let plan = json!({"steps": [{"skill": "GoToPlace", "why": "go to the dock",
                                 "args": [{"name": "place", "value": "dock"}]}]});
    let out = missions.plan(plan, By::Model).await;
    assert_eq!(out.status, Status::Succeeded, "{}", out.message);
    let compiled = missions.find(out.data["hash"].as_str().unwrap()).unwrap();
    assert_eq!(compiled.plan.intent, "go to the dock");
}

#[tokio::test]
async fn problems_go_back_to_the_model_and_attempts_are_capped() {
    let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let bad =
        json!({"intent": "x", "steps": [{"skill": "PickObject", "args": {"object_id": "O99"}}]});
    for n in 1..=MAX_PLAN_ATTEMPTS {
        let out = missions.plan(bad.clone(), By::Model).await;
        assert_eq!(out.status, Status::Failed);
        assert!(out.message.contains(&format!("attempt {n} of")));
    }
    assert_eq!(
        missions.plan(steps(), By::Model).await.status,
        Status::Refused
    );
}

#[tokio::test]
async fn a_failure_reports_the_step_and_limits_replans() {
    let failing = || ScriptedRun {
        result: Ok(GoalResult {
            status: GoalStatus::Aborted,
            result: json!({"outcome": 1, "failed_step_id": "s2", "failure_reason": "grasp slipped"}),
        }),
        ..ScriptedRun::default()
    };
    let now = crate::now_s();
    let history = move |req: &Value| {
        assert_eq!(req["query"], "O18", "the failed step's object");
        let at = |ago: f64| json!({"sec": (now - ago).floor(), "nanosec": 0});
        Ok(json!({"events": [
            {"id": "O18", "label": "blue cup", "kind": "appeared", "stamp": at(7200.0),
             "room_id": "R2", "other_id": "", "detail": ""},
            {"id": "O18", "label": "blue cup", "kind": "missing", "stamp": at(600.0),
             "room_id": "R2", "other_id": "", "detail": ""}]}))
    };
    let robot: Arc<dyn RobotPort> = Arc::new(robot(failing()).with_service("/x/history", history));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let (tx, mut commands) = tokio::sync::mpsc::unbounded_channel();
    let (events, _rx) = tokio::sync::broadcast::channel(64);
    missions.attach(SessionHandle::for_tests(&tx, events));
    let hash = missions.plan(steps(), By::Model).await.data["hash"]
        .as_str()
        .unwrap()
        .to_owned();
    assert_eq!(
        missions.run(&json!({"hash": hash})).await.status,
        Status::Accepted
    );
    let Some(Command::Report(text)) = commands.recv().await else {
        panic!("no report")
    };
    assert!(
        text.contains("Failed at s2 PickObject(") && text.contains("grasp slipped"),
        "{text}"
    );
    assert!(
        text.contains("run it now: the operator approves it"),
        "{text}"
    );
    assert!(
        text.contains(
            "What happened to it: O18 blue cup: appeared 2 h ago in R2; went missing 10 min ago."
        ),
        "{text}"
    );
    missions.run_failures.store(2, Ordering::SeqCst);
    assert_eq!(
        missions.plan(steps(), By::Model).await.status,
        Status::Refused
    );
}

#[tokio::test]
async fn a_plan_is_checked_before_anyone_is_asked_to_run_it() {
    let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let [tool] = missions.tools();
    let sound = tool.assess(&steps()).await.unwrap().unwrap();
    assert_eq!(sound.risk, Risk::Manipulation);
    let hash = sound.args.unwrap()["hash"].as_str().unwrap().to_owned();
    assert_eq!(hash.len(), 64);
    let bad =
        json!({"intent": "x", "steps": [{"skill": "PickObject", "args": {"object_id": "O99"}}]});
    let problems = tool.assess(&bad).await.unwrap().unwrap_err();
    assert_eq!(problems.status, Status::Failed);
    assert!(
        problems.message.contains("call run_mission again"),
        "{}",
        problems.message
    );
    let mut only = steps();
    only["check_only"] = json!(true);
    assert_eq!(
        tool.assess(&only).await.unwrap().unwrap().risk,
        Risk::Observe
    );
    let checked = tool.call(only).await;
    assert_eq!(checked.data["hash"], hash);
    // By its hash, it asks with what it runs, and with nothing else the model sent.
    let by_hash = tool
        .assess(&json!({"hash": &hash[..12], "by": "operator"}))
        .await
        .unwrap()
        .unwrap();
    assert!(
        by_hash.reason.starts_with("runs \"fetch the mug\""),
        "{}",
        by_hash.reason
    );
    assert_eq!(by_hash.args, Some(json!({"hash": hash})));
}

/// Says the same about every plan.
struct Says(&'static str);

#[async_trait]
impl Critic for Says {
    async fn judge(&self, _prompt: &str) -> Result<String, String> {
        Ok(self.0.to_owned())
    }
}

fn missions_asked(request: &str) -> Arc<Missions> {
    let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    *lock(&missions.request) = request.to_owned();
    missions
}

#[tokio::test]
async fn a_first_request_for_it_is_asked_about_not_guessed() {
    let missions = missions_asked("Pick it up.");
    missions.said.store(1, Ordering::SeqCst);
    let out = missions.plan(steps(), By::Model).await;
    assert_eq!(out.status, Status::Refused);
    assert!(
        out.message.contains("which thing they mean"),
        "{}",
        out.message
    );
    missions.said.store(2, Ordering::SeqCst);
    let later = missions.plan(steps(), By::Model).await;
    assert_eq!(
        later.status,
        Status::Succeeded,
        "said after something: {}",
        later.message
    );
}

#[tokio::test]
async fn a_repeat_goes_to_a_schedule_not_a_run() {
    let missions = missions_asked("Every minute, pick up the blue cup, 2 times in all.");
    let [tool] = missions.tools();
    let back = tool.assess(&steps()).await.unwrap().unwrap_err();
    assert_eq!(back.status, Status::Refused);
    assert!(back.message.contains("schedule"), "{}", back.message);
    let checked = tool.assess(&json!({"check_only": true, "steps": []})).await;
    assert!(matches!(checked, Some(Ok(_))), "checking one run is fine");
}

#[tokio::test]
async fn an_unnamed_plan_takes_the_operators_words_and_a_failed_rerun_is_not_asked() {
    let missions = missions_asked("Fetch the blue cup for me.");
    let mut plan = steps();
    plan.as_object_mut().unwrap().remove("intent");
    let [tool] = missions.tools();
    let sound = tool.assess(&plan).await.unwrap().unwrap();
    let hash = sound.args.unwrap()["hash"].as_str().unwrap().to_owned();
    assert_eq!(
        missions.find(&hash).unwrap().plan.intent,
        "Fetch the blue cup for me"
    );
    *lock(&missions.last) = Some((hash.clone(), "failed at s2: grasp slipped".to_owned()));
    let again = tool.assess(&plan).await.unwrap().unwrap_err();
    assert!(
        again.message.contains("just ran and failed"),
        "{}",
        again.message
    );
    let by_hash = tool
        .assess(&json!({"hash": hash}))
        .await
        .unwrap()
        .unwrap_err();
    assert_eq!(by_hash.status, Status::Refused);
}

/// An advisor that keeps what it was asked.
struct Advises(Mutex<Vec<String>>);

#[async_trait]
impl advice::Advisor for Advises {
    async fn advise(&self, prompt: &str) -> Option<String> {
        lock(&self.0).push(prompt.to_owned());
        Some("Find the object first: O99 is not in the world model.".to_owned())
    }
}

#[tokio::test]
async fn a_stronger_model_is_asked_once_the_plans_have_failed_twice() {
    let advisor = Arc::new(Advises(Mutex::new(Vec::new())));
    let missions = missions_asked("bring me the cup")
        .with_advisor(Arc::clone(&advisor) as Arc<dyn advice::Advisor>);
    let bad =
        json!({"intent": "x", "steps": [{"skill": "PickObject", "args": {"object_id": "O99"}}]});
    let first = missions.plan(bad.clone(), By::Model).await;
    assert!(first.data.get("advice").is_none(), "{}", first.data);
    let second = missions.plan(bad.clone(), By::Model).await;
    assert_eq!(
        second.data["advice"],
        "Find the object first: O99 is not in the world model."
    );
    assert!(
        second.message.contains("advice is in `advice`"),
        "{}",
        second.message
    );
    // Past the message's cut, the advice still reaches the model whole.
    assert_eq!(
        second.for_model(6000)["data"]["advice"],
        "Find the object first: O99 is not in the world model."
    );
    let third = missions.plan(bad, By::Model).await;
    assert!(third.data.get("advice").is_none(), "{}", third.data);
    let asked = lock(&advisor.0);
    assert_eq!(asked.len(), 1, "asked once");
    assert!(
        asked[0].starts_with("The operator asked: bring me the cup")
            && asked[0].contains("O99")
            && asked[0].contains("PickObject("),
        "{}",
        asked[0]
    );
}

#[tokio::test]
async fn a_plan_against_the_words_goes_back_once_and_then_shows_its_concern() {
    let missions = missions_asked("Pick up the blue cup with your right hand");
    let [tool] = missions.tools();
    let back = tool.assess(&steps()).await.unwrap().unwrap_err();
    assert_eq!(back.status, Status::Failed);
    assert!(
        back.data["concerns"][0]
            .as_str()
            .unwrap()
            .starts_with("s2: the operator said the right hand"),
        "{}",
        back.data
    );
    assert_eq!(
        missions.plan_failures.load(Ordering::SeqCst),
        0,
        "not one of the model's attempts"
    );
    let sent_again = tool.assess(&steps()).await.unwrap().unwrap();
    assert!(
        sent_again
            .reason
            .contains("; check: s2: the operator said the right hand"),
        "{}",
        sent_again.reason
    );
}

#[tokio::test]
async fn the_critic_sends_a_wrong_plan_back_and_flags_a_doubtful_one() {
    let rejecting = missions_asked("bring me the mug").with_critic(Arc::new(Says(
        r#"{"verdict": "reject", "reason": "it picks up the cup, not the mug"}"#,
    )));
    let [tool] = rejecting.tools();
    let back = tool.assess(&steps()).await.unwrap().unwrap_err();
    assert_eq!(back.data["concerns"][0], "it picks up the cup, not the mug");
    assert!(tool.assess(&steps()).await.unwrap().is_ok());

    let doubtful = missions_asked("bring me the mug").with_critic(Arc::new(Says(
        r#"{"verdict": "ask", "reason": "which mug?"}"#,
    )));
    let [tool] = doubtful.tools();
    let shown = tool.assess(&steps()).await.unwrap().unwrap();
    assert!(
        shown.reason.ends_with("; check: which mug?"),
        "{}",
        shown.reason
    );
}

#[tokio::test]
async fn the_operator_edits_a_plan_without_spending_the_models_attempts() {
    let missions = missions_asked("Pick up the blue cup with your right hand")
        .with_critic(Arc::new(Says(r#"{"verdict": "reject", "reason": "no"}"#)));
    let [tool] = missions.tools();
    let bad =
        json!({"intent": "x", "steps": [{"skill": "PickObject", "args": {"object_id": "O99"}}]});
    let problems = tool.assess_operator(bad).await.unwrap().unwrap_err();
    assert!(problems.message.starts_with("s1"), "{}", problems.message);
    assert_eq!(missions.plan_failures.load(Ordering::SeqCst), 0);

    // Its own concern is shown, not sent back; the critic is not asked.
    let kept_left = tool.assess_operator(steps()).await.unwrap().unwrap();
    assert!(
        kept_left.reason.contains("right hand"),
        "{}",
        kept_left.reason
    );
    assert!(
        !kept_left.reason.contains("check: no"),
        "{}",
        kept_left.reason
    );
    let mut shorter = steps();
    shorter["steps"].as_array_mut().unwrap().remove(0);
    let edited = tool.assess_operator(shorter).await.unwrap().unwrap();
    assert_ne!(edited.args, kept_left.args, "an edit is a new plan");
}

#[tokio::test]
async fn the_same_plan_twice_runs_by_its_hash() {
    let robot: Arc<dyn RobotPort> = Arc::new(robot(ScriptedRun::default()));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let first = missions.plan(steps(), By::Model).await.data["hash"].clone();
    let second = missions.plan(steps(), By::Model).await.data["hash"].clone();
    assert_eq!(first, second);
    assert!(missions.find(first.as_str().unwrap()).is_ok());
}

#[tokio::test]
async fn a_hand_the_executor_cannot_vouch_for_is_full() {
    let state = json!({"holding_left": "", "holding_right": "",
                       "message": "idle; left hand unknown (it may hold something)"});
    assert_eq!(held_by(&state, "left"), UNKNOWN_HELD);
    assert_eq!(held_by(&state, "right"), "");
    let robot: Arc<dyn RobotPort> =
        Arc::new(robot(ScriptedRun::default()).with_topic("/x/state", state));
    let missions = Missions::new(&profile(), Places::new(&profile(), None), robot).unwrap();
    let out = missions.plan(steps(), By::Model).await;
    assert_eq!(out.status, Status::Failed);
    let problems = out.data["problems"].to_string();
    assert!(
        problems.contains("left hand empty, but it holds something unknown"),
        "{problems}"
    );
}

#[test]
fn steps_are_found_in_names_and_paths() {
    assert_eq!(step_of("s12_PickObject").as_deref(), Some("s12"));
    assert_eq!(
        step_of("Mission/s2_GoToPlace::7/NavigateToPose::9").as_deref(),
        Some("s2")
    );
    assert_eq!(step_of("sequence_1"), None);
    assert_eq!(step_of("Pick"), None);
}
