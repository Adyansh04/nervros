//! A whole mission with the local model (`scripts/local-llm.sh start`) and a scripted executor:
//! the model plans against the catalog, the operator approves, the mission runs and the model
//! answers the robot's report.
//!
//! Ignored by default; run with `cargo test -p nervros-core --test local_mission -- --ignored
//! --nocapture` to see the conversation.

use std::sync::Arc;
use std::time::Duration;

use nervros_core::session::{Command, Event};
use nervros_ros::fake::{FakeRobot, ScriptedRun};
use nervros_ros::{GoalResult, GoalStatus, RobotPort, Transform};
use serde_json::{Value, json};

const MODELS: &str = r#"
    [[provider]]
    id = "local"
    kind = "openai_compat"
    base_url = "http://127.0.0.1:8081/v1"
    timeout_s = 120

    [[model]]
    id = "qwen3.5-9b-local"
    provider = "local"
    model = "qwen3.5-9b"
    vision = true
    tools = true
    privacy = { local = true }

    [roles]
    routine = ["qwen3.5-9b-local"]
"#;

const PROFILE: &str = r#"
    [robot]
    name = "Unitree G1 humanoid (simulated), in an apartment"
    [policy]
    start_armed = true
    [world]
    rooms = { topic = "/canopy/rooms", type = "canopy_msgs/msg/RoomArray" }
    objects = { topic = "/canopy/objects", type = "canopy_msgs/msg/WorldObjectArray" }
    [mission]
    execute = "/x/execute_mission"
    validate = "/x/validate_mission"
    catalog = "/x/get_catalog"
    stop = "/x/stop_all"
    state = "/x/robot_state"
    [[place]]
    name = "start"
    pose = { x = 0.0, y = 0.0 }
    [[tool]]
    name = "find_objects"
    kind = "service"
    ros_name = "/canopy/find_objects"
    type = "canopy_msgs/srv/FindObjects"
    risk = "observe"
    description = "Finds objects in the world model by what they are: ids, labels and rooms."
    schema = { type = "object", properties = { query = { type = "string" } }, required = ["query"] }
    [models]
    file = "models.toml"
"#;

/// The catalog grove-g1's executor serves, as fetched from it.
const CATALOG: &str = include_str!("fixtures/g1_catalog.json");

fn object(id: &str, label: &str, room: &str, x: f64, y: f64) -> Value {
    json!({"id": id, "label": label, "name": label, "room_id": room, "state": 0,
           "pose": {"position": {"x": x, "y": y, "z": 0.8}, "orientation": {"w": 1.0}},
           "size": {"x": 0.2, "y": 0.2, "z": 0.2}})
}

fn robot() -> FakeRobot {
    let square = |x0: f64, y0: f64| {
        json!({"points": [{"x": x0, "y": y0}, {"x": x0 + 4.0, "y": y0},
                          {"x": x0 + 4.0, "y": y0 + 4.0}, {"x": x0, "y": y0 + 4.0}]})
    };
    FakeRobot::new()
        .with_service("/x/get_catalog", |_| Ok(json!({"catalog_json": CATALOG})))
        .with_service("/x/validate_mission", |_| {
            Ok(json!({"ok": true, "diagnostics_json": "[]", "worst_case_duration_s": 0.0}))
        })
        .with_service("/x/stop_all", |_| Ok(json!({"ok": true, "state_after": "holding posture"})))
        .with_service("/canopy/find_objects", |_| {
            Ok(json!({"objects": [object("O17", "red mug", "R2", 5.0, 1.0), object("O31", "basket", "R2", 6.0, 1.5)]}))
        })
        .with_action("/x/execute_mission", |_| ScriptedRun {
            feedback: vec![json!({"elapsed_s": 1.0, "events": [{"name": "s1_GoToPlace", "path": "s1_GoToPlace", "status": 2}]})],
            result: Ok(GoalResult {
                status: GoalStatus::Succeeded,
                result: json!({"outcome": 0, "failed_step_id": "", "failure_reason": ""}),
            }),
            ..ScriptedRun::default()
        })
        .with_topic(
            "/canopy/rooms",
            json!({"rooms": [{"id": "R1", "name": "living room", "outline": square(0.0, 0.0)},
                             {"id": "R2", "name": "kitchen", "outline": square(4.0, 0.0)}]}),
        )
        .with_topic(
            "/canopy/objects",
            json!({"objects": [object("O17", "red mug", "R2", 5.0, 1.0), object("O31", "basket", "R2", 6.0, 1.5)]}),
        )
        .with_topic("/x/robot_state", json!({"holding_left": "", "holding_right": ""}))
        .with_transform("map", "base_footprint", Transform::IDENTITY)
}

/// Runs one request to the end of the robot's report and returns every event.
#[expect(
    clippy::expect_used,
    reason = "a live test: any failure should stop it loudly"
)]
async fn run(request: &str) -> Vec<Event> {
    let dir = tempfile::tempdir().expect("a temp dir");
    std::fs::write(dir.path().join("nervros.toml"), PROFILE).expect("write the profile");
    std::fs::write(dir.path().join("models.toml"), MODELS).expect("write the models");
    let robot: Arc<dyn RobotPort> = Arc::new(robot());
    let agent = nervros_core::app::start(
        &dir.path().join("nervros.toml"),
        robot,
        &dir.path().join("q.json"),
    )
    .expect("the agent starts");
    let mut events = agent.session.subscribe();
    agent.session.send(Command::User(request.to_owned()));
    let mut seen = Vec::new();
    let (mut missions, mut awaiting_report) = (0u32, false);
    loop {
        let e = tokio::time::timeout(Duration::from_mins(4), events.recv())
            .await
            .expect("the model answers within four minutes")
            .expect("the session runs");
        println!("{}", serde_json::to_string(&e).unwrap_or_default());
        if let Event::ApprovalRequested { id, .. } = &e {
            agent.session.send(Command::Approve(*id));
        }
        match &e {
            Event::MissionStarted { .. } => missions += 1,
            Event::MissionFinished { .. } => awaiting_report = true,
            Event::Report { .. } => awaiting_report = false,
            _ => {}
        }
        let done = matches!(e, Event::TurnFinished { .. })
            && !awaiting_report
            && (missions == 0 || seen.iter().any(|s| matches!(s, Event::Report { .. })));
        seen.push(e);
        if done {
            return seen;
        }
    }
}

fn tools_called(events: &[Event]) -> Vec<(String, &'static str)> {
    events
        .iter()
        .filter_map(|e| match e {
            Event::ToolFinished { tool, status, .. } => Some((tool.clone(), *status)),
            _ => None,
        })
        .collect()
}

#[tokio::test]
#[ignore = "needs the local model server"]
async fn walks_to_a_room() {
    let events = run("Go to the kitchen.").await;
    let calls = tools_called(&events);
    assert!(
        calls
            .iter()
            .any(|(t, s)| t == "run_mission" && *s == "accepted"),
        "{calls:?}"
    );
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::MissionFinished { outcome, .. } if outcome == "success"))
    );
}

#[tokio::test]
#[ignore = "needs the local model server"]
async fn fetches_an_object_into_a_container() {
    let events = run("Put the red mug in the basket.").await;
    let planned = events.iter().find_map(|e| match e {
        Event::MissionPlanned { steps, .. } => {
            Some(steps.iter().map(|s| s.summary.clone()).collect::<Vec<_>>())
        }
        _ => None,
    });
    let steps = planned.expect("a plan passed its checks");
    println!("plan: {steps:?}");
    assert!(
        steps
            .iter()
            .any(|s| s.starts_with("PickObject(") && s.contains("O17")),
        "{steps:?}"
    );
    assert!(
        steps
            .iter()
            .any(|s| s.starts_with("PlaceInto(") && s.contains("O31")),
        "{steps:?}"
    );
}
