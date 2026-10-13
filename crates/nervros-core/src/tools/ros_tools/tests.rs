use nervros_ros::fake::FakeRobot;
use nervros_ros::{NodeEntities, QosInfo, TopicEndpoints};

use super::*;
use crate::guard::Policy;
use crate::tools::SchemaPart;

#[test]
fn a_missing_name_comes_back_with_the_ones_like_it() {
    let known = ["/Odometry_loc", "/g1_odometry_publisher/odom", "/scan"];
    let text = missing("topic", "/odom", known.into_iter());
    assert!(
        text.contains("/Odometry_loc, /g1_odometry_publisher/odom"),
        "{text}"
    );
    assert!(missing("topic", "/x", known.into_iter()).contains("ros_graph lists them"));
}

/// Checks array lengths like the real registry for one fixed-array request.
struct Schemas;

impl SchemaSource for Schemas {
    fn schema(&self, _t: &str, _p: SchemaPart, _h: &[String]) -> Result<Value, String> {
        Ok(json!({"type": "object"}))
    }
    fn validate(&self, ros_type: &str, _p: SchemaPart, value: &Value) -> Result<(), String> {
        if ros_type == "demo/srv/Fixed" && value["uuid"].as_array().is_some_and(|a| a.len() != 16) {
            return Err("`uuid` must have exactly 16 items".to_owned());
        }
        Ok(())
    }
    fn show(&self, ros_type: &str) -> Option<String> {
        (ros_type == "std_srvs/srv/Trigger")
            .then(|| "---\nbool success\nstring message\n".to_owned())
    }
}

fn qos(reliability: &str, durability: &str) -> QosInfo {
    QosInfo {
        reliability: reliability.into(),
        durability: durability.into(),
        history: "keep_last".into(),
        depth: 10,
    }
}

fn robot() -> FakeRobot {
    let graph = GraphDetail {
        topics: vec![
            ("/odom".into(), vec!["nav_msgs/msg/Odometry".into()]),
            ("/map".into(), vec!["nav_msgs/msg/OccupancyGrid".into()]),
            (
                "/spin/_action/feedback".into(),
                vec!["nav2_msgs/action/Spin_FeedbackMessage".into()],
            ),
            ("/status".into(), vec!["std_msgs/msg/String".into()]),
            ("/cmd_vel".into(), vec!["geometry_msgs/msg/Twist".into()]),
        ],
        services: vec![
            ("/reset".into(), vec!["std_srvs/srv/Trigger".into()]),
            ("/get_state".into(), vec!["std_srvs/srv/Trigger".into()]),
            ("/fixed".into(), vec!["demo/srv/Fixed".into()]),
            (
                "/detector/get_parameters".into(),
                vec!["rcl_interfaces/srv/GetParameters".into()],
            ),
        ],
        nodes: vec!["/detector".into(), "/nav".into()],
    };
    let reliable_sub = Endpoint {
        node: "/viewer".into(),
        topic_type: "std_msgs/msg/String".into(),
        qos: qos("reliable", "volatile"),
    };
    let lossy_pub = Endpoint {
        node: "/talker".into(),
        topic_type: "std_msgs/msg/String".into(),
        qos: qos("best_effort", "volatile"),
    };
    FakeRobot::new()
        .with_graph(graph)
        .with_endpoints(
            "/status",
            TopicEndpoints {
                publishers: vec![lossy_pub],
                subscribers: vec![reliable_sub],
            },
        )
        .with_node(
            "/nav",
            NodeEntities {
                publishers: vec![("/plan".into(), vec!["nav_msgs/msg/Path".into()])],
                ..NodeEntities::default()
            },
        )
        .with_rate("/odom", 50.0, 700)
        .with_transform(
            "map",
            "odom",
            nervros_ros::Transform {
                translation: [1.0, 0.0, 0.0],
                ..nervros_ros::Transform::IDENTITY
            },
        )
        .with_topic(
            "/status",
            json!({"data": "x".repeat(300), "list": (0..20).collect::<Vec<_>>()}),
        )
        .with_topic(
            "/rosout",
            json!({"level": 40, "name": "nav", "msg": "planner failed"}),
        )
        .with_service("/reset", |_| Ok(json!({"success": true, "message": ""})))
        .with_service("/get_state", |_| {
            Ok(json!({"success": true, "message": "ok"}))
        })
        .with_service("/detector/get_parameters", |req| {
            let n = req["names"].as_array().map_or(0, Vec::len);
            Ok(json!({"values": vec![json!({"type": 3, "double_value": 0.5}); n]}))
        })
        .with_service("/detector/set_parameters", |_| {
            Ok(json!({"results": [{"successful": true, "reason": ""}]}))
        })
        .with_service("/detector/list_parameters", |_| {
            Ok(json!({"result": {"names": ["confidence", "model_path", "use_sim_time"], "prefixes": []}}))
        })
        .with_service("/detector/describe_parameters", |req| {
            let descriptors: Vec<Value> = req["names"]
                .as_array()
                .into_iter()
                .flatten()
                .map(|n| {
                    let text = if n == "confidence" {
                        "Detections scored below this are dropped."
                    } else {
                        ""
                    };
                    json!({"name": n, "description": text, "read_only": n == "model_path"})
                })
                .collect();
            Ok(json!({"descriptors": descriptors}))
        })
}

fn all_tools(
    config: &RosToolsConfig,
    armed: bool,
) -> (Arc<FakeRobot>, Vec<Arc<dyn Tool>>, Arc<Guard>) {
    let fake = Arc::new(robot());
    let robot: Arc<dyn RobotPort> = Arc::clone(&fake) as Arc<dyn RobotPort>;
    let schemas: Arc<dyn SchemaSource> = Arc::new(Schemas);
    let guard = Arc::new(Guard::new(Policy::default()));
    guard.set_armed(armed);
    let tools = tools(config, &robot, &schemas, &guard);
    (fake, tools, guard)
}

fn open() -> RosToolsConfig {
    RosToolsConfig {
        service_call: vec!["*".into()],
        action_send: vec!["*".into()],
        publish: vec!["*".into()],
        param_set: vec!["/detector:*".into()],
        ..RosToolsConfig::default()
    }
}

fn tool<'a>(tools: &'a [Arc<dyn Tool>], name: &str) -> &'a Arc<dyn Tool> {
    tools.iter().find(|t| t.spec().name == name).unwrap()
}

#[test]
fn act_tools_exist_only_when_the_profile_lists_them() {
    let (_, closed, _) = all_tools(&RosToolsConfig::default(), false);
    let names: Vec<String> = closed.iter().map(|t| t.spec().name.clone()).collect();
    assert_eq!(
        names,
        [
            "ros_graph",
            "topic_sample",
            "interface_show",
            "tf",
            "params",
            "log_tail"
        ]
    );
    let (_, all, _) = all_tools(&open(), false);
    assert_eq!(all.len(), 10);
}

#[tokio::test]
async fn the_graph_lists_hides_internals_and_derives_actions() {
    let (_, tools, _) = all_tools(&RosToolsConfig::default(), false);
    let graph = tool(&tools, "ros_graph");
    let topics = graph.call(json!({"kind": "topics"})).await;
    let names: Vec<&str> = topics.data["items"]
        .as_array()
        .unwrap()
        .iter()
        .map(|i| i["name"].as_str().unwrap())
        .collect();
    assert!(!names.iter().any(|n| n.contains("_action")), "{names:?}");
    let actions = graph.call(json!({"kind": "actions"})).await;
    assert_eq!(
        actions.data["items"][0],
        json!({"name": "/spin", "type": "nav2_msgs/action/Spin"})
    );
    let services = graph
        .call(json!({"kind": "services", "filter": "get"}))
        .await;
    assert_eq!(
        services.data["total"], 1,
        "parameter services are hidden: {}",
        services.data
    );
}

#[tokio::test]
async fn a_container_in_a_node_list_names_the_nodes_it_hosts() {
    let graph = GraphDetail {
        services: vec![(
            "/nav2_container/_container/list_nodes".into(),
            vec!["composition_interfaces/srv/ListNodes".into()],
        )],
        nodes: vec![
            "/nav2_container".into(),
            "/amcl".into(),
            "/controller_server".into(),
        ],
        ..GraphDetail::default()
    };
    let fake = FakeRobot::new()
        .with_graph(graph)
        .with_node("/nav2_container", NodeEntities::default())
        .with_service("/nav2_container/_container/list_nodes", |_| {
            Ok(json!({"full_node_names": ["/amcl", "/controller_server"], "unique_ids": [1, 2]}))
        });
    let robot: Arc<dyn RobotPort> = Arc::new(fake);
    let schemas: Arc<dyn SchemaSource> = Arc::new(Schemas);
    let guard = Arc::new(Guard::new(Policy::default()));
    let tools = tools(&RosToolsConfig::default(), &robot, &schemas, &guard);
    let graph = tool(&tools, "ros_graph");
    // Nav2's servers do not hold "nav2" in their names; the container that runs them does.
    let listed = graph.call(json!({"kind": "nodes", "filter": "nav2"})).await;
    assert_eq!(
        listed.data["items"],
        json!([{"name": "/nav2_container", "hosts": ["/amcl", "/controller_server"]}]),
        "{}",
        listed.data
    );
    let plain = graph.call(json!({"kind": "nodes", "filter": "amcl"})).await;
    assert_eq!(plain.data["items"], json!(["/amcl"]));
    let detail = graph.call(json!({"name": "/nav2_container"})).await;
    assert_eq!(detail.data["hosts"], json!(["/amcl", "/controller_server"]));
}

#[tokio::test]
async fn topic_info_finds_the_qos_mismatch_that_drops_every_message() {
    let (_, tools, _) = all_tools(&RosToolsConfig::default(), false);
    let info = tool(&tools, "ros_graph")
        .call(json!({"name": "/status"}))
        .await;
    assert_eq!(
        info.data["qos_mismatch"][0],
        json!({"publisher": "/talker", "subscriber": "/viewer"})
    );
    let node = tool(&tools, "ros_graph")
        .call(json!({"name": "/nav"}))
        .await;
    assert_eq!(node.data["publishers"][0]["name"], "/plan");
}

#[tokio::test]
async fn hz_and_echo_measure_and_summarise() {
    let (_, tools, _) = all_tools(&RosToolsConfig::default(), false);
    let sample = tool(&tools, "topic_sample");
    let hz = sample
        .call(json!({"topic": "/odom", "mode": "hz", "seconds": 2}))
        .await;
    assert_eq!(hz.data["rate_hz"], 50.0, "{}", hz.data);
    assert_eq!(
        hz.data["bytes_per_s"], 35000.0,
        "100 messages of 700 bytes in 2 s"
    );
    let echo = sample.call(json!({"topic": "/status"})).await;
    let msg = &echo.data["messages"][0];
    assert!(msg["data"].as_str().unwrap().ends_with("(300 characters)"));
    assert_eq!(
        msg["list"].as_array().unwrap().len(),
        9,
        "8 items and a note"
    );
    let map = sample.call(json!({"topic": "/map"})).await;
    assert!(map.message.contains("too large"), "{}", map.message);
    // A read takes `odom` as `/odom`, as a model means it.
    let relative = sample
        .call(json!({"topic": "odom", "mode": "hz", "seconds": 2}))
        .await;
    assert_eq!(relative.data["rate_hz"], 50.0, "{}", relative.message);
}

#[tokio::test]
async fn logs_filter_by_level() {
    let (_, tools, _) = all_tools(&RosToolsConfig::default(), false);
    let logs = tool(&tools, "log_tail")
        .call(json!({"min_level": "error"}))
        .await;
    assert_eq!(logs.data["lines"][0], "[ERROR] nav: planner failed");
    let quiet = tool(&tools, "log_tail")
        .call(json!({"min_level": "fatal"}))
        .await;
    assert_eq!(quiet.data["matched"], 0);
}

#[tokio::test]
async fn a_call_is_assessed_denied_or_checked_before_it_reaches_the_robot() {
    let (fake, tools, _) = all_tools(&open(), true);
    let call = tool(&tools, "service_call");
    let read = call.assess(&json!({"service": "/get_state"})).await;
    assert_eq!(read.unwrap().unwrap().risk, Risk::Observe);
    let act = call
        .assess(&json!({"service": "/reset", "request": {}}))
        .await
        .unwrap()
        .unwrap();
    assert_eq!((act.risk, act.resources.len()), (Risk::Motion, 3));
    assert!(act.reason.contains("/reset"), "{}", act.reason);
    let bad = call
        .call(json!({"service": "/fixed", "request": {"uuid": [1, 2]}}))
        .await;
    assert!(bad.message.contains("exactly 16"), "{}", bad.message);
    let ok = call.call(json!({"service": "/reset"})).await;
    assert_eq!(ok.data["response"]["success"], true);
    assert_eq!(
        fake.calls().iter().filter(|(n, _)| n == "/fixed").count(),
        0,
        "never sent"
    );
}

#[tokio::test]
async fn publishing_refuses_command_topics_and_types() {
    let (fake, tools, _) = all_tools(&open(), true);
    let publish = tool(&tools, "topic_publish");
    let cmd = publish
        .call(json!({"topic": "/cmd_vel", "message": {}}))
        .await;
    assert!(cmd.message.contains("hard deny"), "{}", cmd.message);
    // An act is never resolved: a deny list is written for absolute names.
    let relative = publish
        .call(json!({"topic": "cmd_vel", "message": {}}))
        .await;
    assert!(
        relative.message.contains("absolute"),
        "{}",
        relative.message
    );
    let twist = publish
        .call(json!({"topic": "/teleop", "type": "geometry_msgs/msg/Twist", "message": {}}))
        .await;
    assert!(
        twist.message.contains("commands the robot directly"),
        "{}",
        twist.message
    );
    let ok = publish
        .call(json!({"topic": "/status", "message": {"data": "hi"}, "count": 3}))
        .await;
    assert_eq!(ok.data["published"], 3, "{}", ok.message);
    assert_eq!(fake.published().len(), 3);
}

#[tokio::test]
async fn a_parameter_list_says_what_each_holds_and_narrows_by_filter() {
    let (_, tools, _) = all_tools(&RosToolsConfig::default(), false);
    let params = tool(&tools, "params");
    let all = params
        .call(json!({"op": "list", "node": "/detector"}))
        .await;
    assert_eq!(all.data["total"], 3, "{}", all.data);
    assert_eq!(
        all.data["parameters"][1],
        json!({"name": "model_path", "value": 0.5, "read_only": true})
    );
    let narrowed = params
        .call(json!({"op": "list", "node": "/detector", "filter": "conf"}))
        .await;
    assert_eq!(
        narrowed.data["parameters"],
        json!([{"name": "confidence", "value": 0.5, "description": "Detections scored below this are dropped."}]),
        "{}",
        narrowed.data
    );
    let globbed = params
        .call(json!({"op": "list", "node": "/detector", "filter": "*_time"}))
        .await;
    assert_eq!(globbed.data["total"], 1, "{}", globbed.data);
}

#[test]
fn the_profiles_rules_see_a_relative_name_as_the_read_resolves_it() {
    let (_, tools, _) = all_tools(&RosToolsConfig::default(), false);
    for (name, key) in [
        ("topic_sample", "topic"),
        ("params", "node"),
        ("ros_graph", "name"),
    ] {
        let view = tool(&tools, name).rule_view(&json!({key: "camera/image", "mode": "echo"}));
        assert_eq!(
            view,
            Some(json!({key: "/camera/image", "mode": "echo"})),
            "{name}"
        );
        assert_eq!(
            tool(&tools, name).rule_view(&json!({key: "/odom"})),
            None,
            "{name}"
        );
    }
}

#[tokio::test]
async fn a_nav2_goal_inside_a_keep_out_zone_is_refused() {
    let (fake, tools, guard) = all_tools(&open(), true);
    let profile: crate::profile::Profile = toml::from_str(
        "[robot]\nname = \"x\"\n[models]\nfile = \"m.toml\"\n\
         [[geofence.zone]]\nname = \"stairs\"\npolygon = [[2, -1], [3, -1], [3, 1], [2, 1]]\n",
    )
    .unwrap();
    guard.set_zones(crate::zones::Zones::new(&profile, None));
    let goal = |x: f64| {
        json!({"action": "/navigate_to_pose", "type": "nav2_msgs/action/NavigateToPose",
               "goal": {"pose": {"header": {"frame_id": "map"},
                                 "pose": {"position": {"x": x, "y": 0.0}, "orientation": {"w": 1.0}}}}})
    };
    let refused = tool(&tools, "action_goal").call(goal(2.5)).await;
    assert!(refused.message.contains("`stairs`"), "{}", refused.message);
    assert!(
        fake.calls().iter().all(|(n, _)| n != "/navigate_to_pose"),
        "never sent"
    );
    let at = |frame: &str, position: Value| json!({"header": {"frame_id": frame}, "pose": {"position": position}});
    let send = |action: &str, ty: &str, goal: Value| json!({"action": action, "type": format!("nav2_msgs/action/{ty}"), "goal": goal});
    let goals = [
        // y left out goes as 0; odom sits 1 m along the map's x.
        send(
            "/compute_path_to_pose",
            "ComputePathToPose",
            json!({"goal": at("map", json!({"x": 2.5}))}),
        ),
        send(
            "/navigate_to_pose",
            "NavigateToPose",
            json!({"pose": at("odom", json!({"x": 1.5, "y": 0.0}))}),
        ),
        send(
            "/compute_path_through_poses",
            "ComputePathThroughPoses",
            json!({"goals": [at("map", json!({"x": 0.0})), at("map", json!({"x": 2.5}))]}),
        ),
        send(
            "/follow_path",
            "FollowPath",
            json!({"path": {"poses": [at("map", json!({"x": 2.2}))]}}),
        ),
    ];
    for goal in goals {
        let out = tool(&tools, "action_goal").call(goal.clone()).await;
        assert!(out.message.contains("`stairs`"), "{goal}: {}", out.message);
    }
    let unknown = tool(&tools, "action_goal")
        .call(send(
            "/navigate_to_pose",
            "NavigateToPose",
            json!({"pose": at("camera", json!({"x": 0.0}))}),
        ))
        .await;
    assert!(
        unknown.message.contains("cannot be checked"),
        "{}",
        unknown.message
    );
    let straight = tool(&tools, "action_goal")
        .call(send(
            "/drive_on_heading",
            "DriveOnHeading",
            json!({"target": {"x": 0.5}}),
        ))
        .await;
    assert!(
        straight.message.contains("run_mission"),
        "{}",
        straight.message
    );
    let mission = tool(&tools, "action_goal")
        .call(json!({"action": "/x/execute_mission", "type": "nervros_interfaces/action/ExecuteMission",
                     "goal": {"tree_xml": "<root/>"}}))
        .await;
    assert!(
        mission.message.contains("run_mission"),
        "{}",
        mission.message
    );
    let published = tool(&tools, "topic_publish")
        .call(
            json!({"topic": "/goal_pose", "type": "geometry_msgs/msg/PoseStamped",
                     "message": at("map", json!({"x": 2.5, "y": 0.5}))}),
        )
        .await;
    assert!(
        published.message.contains("`stairs`"),
        "{}",
        published.message
    );
}

#[tokio::test]
async fn a_parameter_is_set_in_its_own_type_and_read_back() {
    let (fake, tools, _) = all_tools(&open(), true);
    let set = tool(&tools, "param_set");
    let out = set
        .call(json!({"node": "/detector", "name": "confidence", "value": 1}))
        .await;
    assert_eq!(out.data["now"], 0.5, "{}", out.message);
    let request = fake
        .calls()
        .into_iter()
        .find(|(n, _)| n == "/detector/set_parameters")
        .unwrap()
        .1;
    assert_eq!(
        request["parameters"][0]["value"],
        json!({"type": 3, "double_value": 1.0})
    );
    let other = set
        .call(json!({"node": "/nav", "name": "x", "value": 1}))
        .await;
    assert!(other.message.contains("does not let"), "{}", other.message);
}
