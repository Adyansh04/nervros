use super::*;
use nervros_ros::fake::FakeRobot;
use std::sync::atomic::{AtomicU32, Ordering};

/// A native that counts its runs, fails the first `fails` of them, and waits `secs` (or for the
/// stop).
struct Counting {
    name: &'static str,
    runs: AtomicU32,
    fails: u32,
    secs: f64,
}

#[async_trait]
impl NativeSkill for Counting {
    fn skill(&self) -> Value {
        json!({
            "name": self.name, "description": "a test motion",
            "args": [{"name": "how", "type": "string"}],
            "outputs": [{"name": "said", "type": "string"}],
            "resources": ["base"], "max_duration_s": 30.0, "template": self.name,
        })
    }

    async fn preview(&self, args: &BTreeMap<String, String>) -> Option<Value> {
        let note = format!("{} {}", self.name, args.get("how")?);
        Some(json!({"goal": {"header": {"frame_id": ""}}, "path": {"poses": []}, "note": note}))
    }

    async fn run(
        &self,
        args: &BTreeMap<String, String>,
        ctl: &Control,
    ) -> Result<BTreeMap<String, String>, String> {
        let n = self.runs.fetch_add(1, Ordering::SeqCst);
        tokio::select! {
            () = tokio::time::sleep(Duration::from_secs_f64(self.secs)) => {}
            () = ctl.until_stopped() => return Err("stopped".to_owned()),
        }
        if n < self.fails {
            return Err(format!("failed run {n}"));
        }
        Ok(BTreeMap::from([(
            "said".to_owned(),
            format!("{} done", args["how"]),
        )]))
    }
}

fn native(name: &'static str, fails: u32, secs: f64) -> Arc<Counting> {
    Arc::new(Counting {
        name,
        runs: AtomicU32::new(0),
        fails,
        secs,
    })
}

fn composite(natives: Vec<Arc<dyn NativeSkill>>) -> Composite {
    Composite::new(Arc::new(FakeRobot::new()), None, natives)
}

const TREE: &str = r#"<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission">
    <Sequence name="mission">
      <Timeout msec="5000"><SubTree ID="Walk" name="s1_Walk" how="slowly" said="{s1_said}"/></Timeout>
      <ForceSuccess><RetryUntilSuccessful num_attempts="2"><Timeout msec="5000"><SubTree ID="Look" name="s2_Look" how="{s1_said}"/></Timeout></RetryUntilSuccessful></ForceSuccess>
    </Sequence>
  </BehaviorTree>
</root>"#;

#[test]
fn a_tree_nervros_wrote_reads_as_its_steps_and_their_wrappers() {
    let steps = parse(TREE).unwrap();
    assert_eq!(steps.len(), 2);
    assert_eq!(steps[0].template, "Walk");
    assert_eq!(steps[0].name, "s1_Walk");
    assert_eq!(steps[0].timeout, Some(Duration::from_secs(5)));
    assert_eq!(steps[1].attempts, 2);
    assert!(steps[1].forced);
    assert!(
        steps[0].xml.starts_with("<Timeout msec=\"5000\">"),
        "{}",
        steps[0].xml
    );
    assert_eq!(steps[1].args["how"], "{s1_said}");
}

#[test]
fn what_a_step_handed_on_fills_the_later_ones_and_unknown_keys_go_as_literals() {
    let outputs: Map<String, Value> = [("s1_id".to_owned(), json!("O17"))].into_iter().collect();
    assert_eq!(
        fill(r#"<SubTree ID="P" object_id="{s1_id}"/>"#, &outputs),
        r#"<SubTree ID="P" object_id="O17"/>"#
    );
    assert_eq!(
        unbind(r#"<SubTree ID="P" object_id="{s1_id}" x="1"/>"#),
        r#"<SubTree ID="P" object_id="s1_id" x="1"/>"#
    );
}

async fn run(c: &Composite, tree: &str) -> (Value, Vec<Value>) {
    let goal = c
        .send_goal(
            EXECUTE,
            EXECUTE_TYPE,
            json!({"mission_id": "m1", "tree_xml": tree}),
            CALL_WAIT,
        )
        .await
        .unwrap();
    let Goal {
        mut feedback,
        result,
        ..
    } = goal;
    let result = result.await.unwrap().unwrap().result;
    let mut events = Vec::new();
    while let Ok(fb) = feedback.try_recv() {
        events.extend(fb["events"].as_array().cloned().unwrap_or_default());
    }
    (result, events)
}

#[tokio::test]
async fn natives_run_in_order_retry_and_hand_on_without_an_executor() {
    let walk = native("Walk", 0, 0.01);
    let look = native("Look", 1, 0.01);
    let c = composite(vec![walk.clone(), look.clone()]);
    let (result, events) = run(&c, TREE).await;
    assert_eq!(result["outcome"], 0, "{result}");
    assert_eq!(look.runs.load(Ordering::SeqCst), 2, "retried once");
    let handed: Value = serde_json::from_str(result["outputs_json"].as_str().unwrap()).unwrap();
    assert_eq!(handed["s1_said"], "slowly done");
    let statuses: Vec<(String, u64)> = events
        .iter()
        .map(|e| {
            (
                e["name"].as_str().unwrap().to_owned(),
                e["status"].as_u64().unwrap(),
            )
        })
        .collect();
    assert_eq!(statuses[0], ("s1_Walk".to_owned(), 1));
    assert!(
        statuses.contains(&("s2_Look".to_owned(), 3)),
        "the failed try shows"
    );
    assert_eq!(statuses.last().unwrap(), &("s2_Look".to_owned(), 2));
}

#[tokio::test]
async fn a_stop_ends_a_native_step_as_cancelled_and_the_state_names_the_mission() {
    let c = Arc::new(composite(vec![
        native("Walk", 0, 30.0),
        native("Look", 0, 0.01),
    ]));
    let goal = c
        .send_goal(
            EXECUTE,
            EXECUTE_TYPE,
            json!({"mission_id": "m2", "tree_xml": TREE}),
            CALL_WAIT,
        )
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(50)).await;
    let state = c
        .latest_fresh(STATE, "", CALL_WAIT, CALL_WAIT)
        .await
        .unwrap();
    assert_eq!(state["mission_id"], "m2");
    assert_eq!(state["mission_step"], "s1_Walk");
    assert_eq!(state["can_move"], true);
    let stopped = c
        .call(STOP, "", json!({"reason": "test"}), CALL_WAIT)
        .await
        .unwrap();
    assert_eq!(stopped["ok"], true);
    let result = goal.result.await.unwrap().unwrap().result;
    assert_eq!(result["outcome"], u64::from(OUTCOME_CANCELED), "{result}");
    assert_eq!(result["failed_step_id"], "s1");
    let idle = c
        .latest_fresh(STATE, "", CALL_WAIT, CALL_WAIT)
        .await
        .unwrap();
    assert_eq!(idle["mission_id"], "");
}

#[tokio::test]
async fn the_catalog_holds_the_natives_and_a_step_missing_an_argument_is_refused() {
    let c = composite(vec![native("Walk", 0, 0.01)]);
    let reply = c.call(CATALOG, "", json!({}), CALL_WAIT).await.unwrap();
    let catalog =
        crate::mission::catalog::Catalog::parse(reply["catalog_json"].as_str().unwrap()).unwrap();
    assert!(catalog.skill("Walk").is_some());
    let bad = TREE.replace(r#" how="slowly""#, "");
    let checked = c
        .call(VALIDATE, "", json!({"tree_xml": bad}), CALL_WAIT)
        .await
        .unwrap();
    assert_eq!(checked["ok"], false);
    assert!(
        checked["diagnostics_json"]
            .as_str()
            .unwrap()
            .contains("needs `how`"),
        "{checked}"
    );
}

#[test]
fn arguments_are_checked_as_the_catalog_writes_them() {
    let degrees = json!({"name": "degrees", "format": "number", "min": -180, "max": 180});
    assert_eq!(arg_problem(&degrees, "90"), None);
    assert_eq!(arg_problem(&degrees, "{s1_turn}"), None);
    assert!(
        arg_problem(&degrees, "270")
            .unwrap()
            .contains("from -180 to 180")
    );
    assert!(arg_problem(&degrees, "left").unwrap().contains("a number"));
    let way = json!({"name": "direction", "enum": ["forward", "backward"]});
    assert!(
        arg_problem(&way, "sideways")
            .unwrap()
            .contains("forward, backward")
    );
    let station = json!({"name": "station", "format": "station"});
    assert_eq!(arg_problem(&station, "1;2;0.5"), None);
    assert!(arg_problem(&station, "1;2").is_some());
    assert!(arg_problem(&station, "1;x;2").is_some());
}

#[tokio::test]
async fn native_steps_preview_themselves_under_their_step_ids() {
    let c = composite(vec![native("Walk", 0, 0.01), native("Look", 0, 0.01)]);
    let reply = c
        .call(PREVIEW, "", json!({"tree_xml": TREE}), CALL_WAIT)
        .await
        .unwrap();
    let steps = crate::mission::preview::parse(&reply).unwrap();
    let seen: Vec<(&str, &str)> = steps
        .iter()
        .map(|s| (s.id.as_str(), s.note.as_str()))
        .collect();
    // The second step's `how` is what the first hands on, known only as it runs: shown as is.
    assert_eq!(seen, [("s1", "Walk slowly"), ("s2", "Look {s1_said}")]);
}

#[tokio::test]
async fn a_mixed_plan_sends_its_executor_stretch_with_what_the_natives_handed_on() {
    use nervros_ros::fake::ScriptedRun;
    use nervros_ros::{GoalResult, GoalStatus};
    let sent = Arc::new(std::sync::Mutex::new(Vec::<String>::new()));
    let seen = Arc::clone(&sent);
    let catalog = json!({"catalog_version": "v1", "skills": [{"name": "Pick", "template": "Pick",
        "max_duration_s": 30.0, "args": [{"name": "item"}], "outputs": [{"name": "got"}]}]});
    let robot = FakeRobot::new()
        .with_service("/x/get_catalog", move |_| {
            Ok(json!({"catalog_json": catalog.to_string()}))
        })
        .with_action("/x/execute_mission", move |goal| {
            let tree = goal["tree_xml"].as_str().unwrap_or_default().to_owned();
            seen.lock().unwrap().push(tree);
            ScriptedRun {
                result: Ok(GoalResult {
                    status: GoalStatus::Succeeded,
                    result: json!({"outcome": 0, "outputs_json": "{\"s2_got\": \"O17\"}"}),
                }),
                ..ScriptedRun::default()
            }
        });
    let real: MissionConfig = serde_json::from_value(json!({
        "execute": "/x/execute_mission", "validate": "/x/validate_mission",
        "catalog": "/x/get_catalog", "stop": "/x/stop_all", "state": "/x/robot_state",
    }))
    .unwrap();
    let c = Composite::new(Arc::new(robot), Some(real), vec![native("Walk", 0, 0.01)]);
    let tree = r#"<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission">
    <Sequence name="mission">
      <SubTree ID="Walk" name="s1_Walk" how="slowly" said="{s1_said}"/>
      <SubTree ID="Pick" name="s2_Pick" item="{s1_said}" got="{s2_got}"/>
      <SubTree ID="Walk" name="s3_Walk" how="{s2_got}" said="{s3_said}"/>
    </Sequence>
  </BehaviorTree>
</root>"#;
    let (result, _) = run(&c, tree).await;
    assert_eq!(result["outcome"], 0, "{result}");
    let sent = sent.lock().unwrap().clone();
    assert_eq!(sent.len(), 1, "one stretch went to the executor");
    assert!(sent[0].contains(r#"item="slowly done""#), "{}", sent[0]);
    assert!(sent[0].contains(r#"got="{s2_got}""#), "{}", sent[0]);
    assert!(!sent[0].contains("s1_Walk"), "{}", sent[0]);
    let handed: Value = serde_json::from_str(result["outputs_json"].as_str().unwrap()).unwrap();
    assert_eq!(handed["s3_said"], "O17 done");
}

/// A composite over an executor whose missions end with `result`, recording each tree it is sent.
fn with_executor(
    result: Value,
    natives: Vec<Arc<dyn NativeSkill>>,
) -> (Composite, Arc<std::sync::Mutex<Vec<String>>>) {
    use nervros_ros::fake::ScriptedRun;
    use nervros_ros::{GoalResult, GoalStatus};
    let sent = Arc::new(std::sync::Mutex::new(Vec::<String>::new()));
    let seen = Arc::clone(&sent);
    let catalog = json!({"catalog_version": "v1", "skills": [{"name": "Pick", "template": "Pick",
        "max_duration_s": 30.0, "args": [{"name": "item"}]}]});
    let robot = FakeRobot::new()
        .with_service("/x/get_catalog", move |_| {
            Ok(json!({"catalog_json": catalog.to_string()}))
        })
        .with_action("/x/execute_mission", move |goal| {
            let tree = goal["tree_xml"].as_str().unwrap_or_default().to_owned();
            seen.lock().unwrap().push(tree);
            ScriptedRun {
                result: Ok(GoalResult {
                    status: GoalStatus::Succeeded,
                    result: result.clone(),
                }),
                ..ScriptedRun::default()
            }
        });
    let real: MissionConfig = serde_json::from_value(json!({
        "execute": "/x/execute_mission", "validate": "/x/validate_mission",
        "catalog": "/x/get_catalog", "stop": "/x/stop_all", "state": "/x/robot_state",
    }))
    .unwrap();
    (Composite::new(Arc::new(robot), Some(real), natives), sent)
}

#[tokio::test]
async fn a_failed_executor_stretch_ends_the_mission_there() {
    let walk = native("Walk", 0, 0.01);
    let failed = json!({"outcome": OUTCOME_FAILURE, "failed_node": "s2_Pick",
        "failed_step_id": "s2", "failure_reason": "the grasp slipped", "outputs_json": "{}"});
    let (c, sent) = with_executor(failed, vec![walk.clone()]);
    let tree = r#"<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission">
    <Sequence name="mission">
      <SubTree ID="Walk" name="s1_Walk" how="slowly" said="{s1_said}"/>
      <SubTree ID="Pick" name="s2_Pick" item="{s1_said}"/>
      <SubTree ID="Walk" name="s3_Walk" how="again" said="{s3_said}"/>
    </Sequence>
  </BehaviorTree>
</root>"#;
    let (result, _) = run(&c, tree).await;
    assert_eq!(result["outcome"], OUTCOME_FAILURE, "{result}");
    assert_eq!(result["failure_reason"], "the grasp slipped");
    assert_eq!(sent.lock().unwrap().len(), 1);
    assert_eq!(walk.runs.load(Ordering::SeqCst), 1, "s3 never ran");
}

#[tokio::test]
async fn a_native_step_is_paused_and_resumed_through_the_composite() {
    let c = Arc::new(composite(vec![
        native("Walk", 0, 0.5),
        native("Look", 0, 0.01),
    ]));
    let _goal = c
        .send_goal(
            EXECUTE,
            EXECUTE_TYPE,
            json!({"mission_id": "m3", "tree_xml": TREE}),
            CALL_WAIT,
        )
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(50)).await;
    let paused = c
        .call(PAUSE, "", json!({"pause": true}), CALL_WAIT)
        .await
        .unwrap();
    assert_eq!(paused["ok"], true, "{paused}");
    let state = c
        .latest_fresh(STATE, "", CALL_WAIT, CALL_WAIT)
        .await
        .unwrap();
    assert_eq!(state["paused"], true);
    let resumed = c
        .call(PAUSE, "", json!({"pause": false}), CALL_WAIT)
        .await
        .unwrap();
    assert_eq!(resumed["message"], "resumed");
    let state = c
        .latest_fresh(STATE, "", CALL_WAIT, CALL_WAIT)
        .await
        .unwrap();
    assert_eq!(state["paused"], false);
}

#[tokio::test]
async fn a_failed_native_step_ends_a_mixed_mission_before_its_executor_stretch() {
    let walk = native("Walk", 9, 0.01);
    let (c, sent) = with_executor(json!({"outcome": 0, "outputs_json": "{}"}), vec![walk]);
    let tree = r#"<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission">
    <Sequence name="mission">
      <SubTree ID="Walk" name="s1_Walk" how="slowly" said="{s1_said}"/>
      <SubTree ID="Pick" name="s2_Pick" item="{s1_said}"/>
    </Sequence>
  </BehaviorTree>
</root>"#;
    let (result, _) = run(&c, tree).await;
    assert_eq!(result["outcome"], OUTCOME_FAILURE, "{result}");
    assert_eq!(result["failed_step_id"], "s1");
    assert!(
        sent.lock().unwrap().is_empty(),
        "the executor's stretch never went out"
    );
}

#[tokio::test]
async fn pause_and_stop_in_an_executor_stretch_go_to_the_executor() {
    use nervros_ros::fake::ScriptedRun;
    use nervros_ros::{GoalResult, GoalStatus};
    let asked = Arc::new(std::sync::Mutex::new(Vec::<String>::new()));
    let (pause_log, stop_log) = (Arc::clone(&asked), Arc::clone(&asked));
    let catalog = json!({"catalog_version": "v1", "skills": [{"name": "Pick", "template": "Pick",
        "max_duration_s": 30.0, "args": [{"name": "item"}]}]});
    let robot = FakeRobot::new()
        .with_service("/x/get_catalog", move |_| {
            Ok(json!({"catalog_json": catalog.to_string()}))
        })
        .with_service("/x/pause_mission", move |r| {
            pause_log
                .lock()
                .unwrap()
                .push(format!("pause {}", r["pause"]));
            Ok(json!({"ok": true, "message": "paused"}))
        })
        .with_service("/x/stop_all", move |_| {
            stop_log.lock().unwrap().push("stop".to_owned());
            Ok(json!({"ok": true, "message": "stopped", "state_after": "stopped"}))
        })
        // The executor's stretch runs a while; the stop cancels it, with no result of its own.
        .with_action("/x/execute_mission", |_| ScriptedRun {
            feedback: vec![json!({}); 10],
            step: Duration::from_millis(30),
            result: Ok(GoalResult {
                status: GoalStatus::Succeeded,
                result: json!({"outcome": OUTCOME_SUCCESS, "outputs_json": "{}"}),
            }),
        });
    let real: MissionConfig = serde_json::from_value(json!({
        "execute": "/x/execute_mission", "validate": "/x/validate_mission",
        "catalog": "/x/get_catalog", "stop": "/x/stop_all", "state": "/x/robot_state",
        "pause": "/x/pause_mission",
    }))
    .unwrap();
    let walk = native("Walk", 0, 0.01);
    let c = Composite::new(Arc::new(robot), Some(real), vec![walk.clone()]);
    let tree = r#"<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission">
    <Sequence name="mission">
      <SubTree ID="Pick" name="s1_Pick" item="mug"/>
      <SubTree ID="Walk" name="s2_Walk" how="away" said="{s2_said}"/>
    </Sequence>
  </BehaviorTree>
</root>"#;
    let goal = c
        .send_goal(
            EXECUTE,
            EXECUTE_TYPE,
            json!({"mission_id": "m4", "tree_xml": tree}),
            CALL_WAIT,
        )
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(60)).await;
    let paused = c
        .call(PAUSE, "", json!({"pause": true}), CALL_WAIT)
        .await
        .unwrap();
    assert_eq!(paused["ok"], true, "{paused}");
    c.call(STOP, "", json!({"reason": "test"}), CALL_WAIT)
        .await
        .unwrap();
    let result = goal.result.await.unwrap().unwrap().result;
    assert_eq!(result["outcome"], u64::from(OUTCOME_CANCELED), "{result}");
    assert_eq!(result["failure_reason"], "stopped: test");
    assert_eq!(*asked.lock().unwrap(), ["pause true", "stop"]);
    assert_eq!(
        walk.runs.load(Ordering::SeqCst),
        0,
        "the native step after the stop never ran"
    );
}

#[tokio::test]
async fn a_mission_of_executor_skills_alone_is_paused_by_the_executor() {
    let asked = Arc::new(std::sync::Mutex::new(Vec::<String>::new()));
    let log = Arc::clone(&asked);
    let robot = FakeRobot::new().with_service("/x/pause_mission", move |r| {
        log.lock().unwrap().push(format!("pause {}", r["pause"]));
        Ok(json!({"ok": true, "message": "paused"}))
    });
    let real: MissionConfig = serde_json::from_value(json!({
        "execute": "/x/execute_mission", "validate": "/x/validate_mission",
        "catalog": "/x/get_catalog", "stop": "/x/stop_all", "state": "/x/robot_state",
        "pause": "/x/pause_mission",
    }))
    .unwrap();
    // The executor runs such a mission without the composite, so nothing runs here.
    let c = Composite::new(Arc::new(robot), Some(real), Vec::new());
    let paused = c
        .call(PAUSE, "", json!({"pause": true}), CALL_WAIT)
        .await
        .unwrap();
    assert_eq!(paused["ok"], true, "{paused}");
    assert_eq!(*asked.lock().unwrap(), ["pause true"]);
    let none = composite(Vec::new())
        .call(PAUSE, "", json!({"pause": true}), CALL_WAIT)
        .await
        .unwrap();
    assert_eq!(none["message"], "no mission is running", "{none}");
}
