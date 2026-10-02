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
    let relative = sample.call(json!({"topic": "odom"})).await;
    assert!(
        relative.message.contains("absolute"),
        "{}",
        relative.message
    );
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
