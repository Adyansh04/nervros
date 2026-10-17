//! `nav_admin`: what keeps Nav2 working, done from chat: clearing its costmaps, telling
//! localization where the robot is, pausing and resuming navigation, saving and loading maps.
//!
//! Each changes what the robot acts on, so each needs the robot armed and, when supervised, the
//! operator's approval; what moves localization or navigation holds the base, so it waits for no
//! mission: a running one refuses it.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::RobotPort;
use serde_json::{Value, json};

use super::Nav2Config;
use super::check::costmap_service;
use crate::integrations::{locate, stamped};
use crate::places::Places;
use crate::profile::Profile;
use crate::tools::{Assessment, Resource, Risk, Tool, ToolOutcome, ToolSpec};

const CALL_WAIT: Duration = Duration::from_secs(10);
/// A map can take a while to write.
const MAP_WAIT: Duration = Duration::from_secs(30);
/// How long a map saver is looked for before the next is tried.
const FIND_WAIT: Duration = Duration::from_secs(2);
/// The spread a set pose starts with: about half a metre and 15 degrees.
const POSE_VARIANCE: [f64; 2] = [0.25, 0.068];

/// The tool.
pub(super) struct NavAdmin {
    spec: ToolSpec,
    config: Nav2Config,
    profile: Profile,
    robot: Arc<dyn RobotPort>,
    places: Arc<Places>,
}

impl NavAdmin {
    pub(super) fn new(
        config: &Nav2Config,
        profile: &Profile,
        robot: Arc<dyn RobotPort>,
        places: Arc<Places>,
    ) -> Self {
        let spec = ToolSpec::new(
            "nav_admin",
            "Keeps Nav2 working. clear_costmaps: forget the obstacles in its global, local or \
             both costmaps (around_m: only around the robot), as when it is stuck behind \
             something gone. set_pose: tell localization the robot stands at a place, a room, an \
             object or x, y and yaw on the map. relocalize: localization searches the whole map. \
             navigation: pause, resume, reset or startup Nav2's servers. save_map: save the map \
             as name. load_map: switch to the map at url. Each needs the operator's approval; \
             none runs during a mission.",
            json!({
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["clear_costmaps", "set_pose", "relocalize", "navigation", "save_map", "load_map"]},
                    "costmap": {"type": "string", "enum": ["global", "local", "both"]},
                    "around_m": {"type": "number", "description": "For clear_costmaps: only within this distance of the robot"},
                    "place": {"type": "string"},
                    "room": {"type": "string"},
                    "object": {"type": "string"},
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "yaw": {"type": "number", "description": "Radians, counter-clockwise from the map's x axis"},
                    "command": {"type": "string", "enum": ["pause", "resume", "reset", "startup"]},
                    "name": {"type": "string", "description": "For save_map"},
                    "url": {"type": "string", "description": "For load_map: the map's YAML file on the robot"},
                },
                "required": ["action"],
            }),
            Risk::WorldEdit,
        );
        Self {
            spec,
            config: config.clone(),
            profile: profile.clone(),
            robot,
            places,
        }
    }

    async fn call_service(
        &self,
        name: &str,
        ty: &str,
        request: Value,
        wait: Duration,
    ) -> Result<Value, String> {
        self.robot
            .call(name, ty, request, wait)
            .await
            .map_err(|e| format!("{name} did not answer: {e}"))
    }

    /// The costmaps `args` name, as their node names.
    fn costmaps(&self, args: &Value) -> Vec<&String> {
        let which = args["costmap"].as_str().unwrap_or("both");
        self.config
            .costmaps
            .iter()
            .filter(|c| which == "both" || c.contains(which))
            .collect()
    }

    async fn clear(&self, args: &Value) -> ToolOutcome {
        let around = args["around_m"]
            .as_f64()
            .filter(|d| d.is_finite() && *d > 0.0);
        let mut cleared = Vec::new();
        for costmap in self.costmaps(args) {
            let (service, ty, request) = match around {
                Some(d) => (
                    costmap_service(costmap, "clear_around"),
                    "nav2_msgs/srv/ClearCostmapAroundRobot",
                    json!({"reset_distance": d}),
                ),
                None => (
                    costmap_service(costmap, "clear_entirely"),
                    "nav2_msgs/srv/ClearEntireCostmap",
                    json!({}),
                ),
            };
            let service = self.config.name(&service);
            if let Err(why) = self.call_service(&service, ty, request, CALL_WAIT).await {
                return ToolOutcome::failed(why);
            }
            cleared.push(costmap.rsplit('/').next().unwrap_or(costmap).to_owned());
        }
        if cleared.is_empty() {
            return ToolOutcome::failed("the profile names no such costmap");
        }
        ToolOutcome::ok(json!({"cleared": cleared, "around_m": around}))
    }

    async fn set_pose(&self, args: &Value) -> ToolOutcome {
        let spot = match locate(
            args,
            (&self.profile, &self.places, self.robot.as_ref()),
            None,
            0.5,
        )
        .await
        {
            Ok(s) => s,
            Err(why) => return ToolOutcome::failed(why),
        };
        let yaw = spot.yaw.unwrap_or(0.0);
        let at = stamped(&self.profile.ros.map_frame, spot.x, spot.y, yaw);
        let mut covariance = vec![0.0; 36];
        covariance[0] = POSE_VARIANCE[0];
        covariance[7] = POSE_VARIANCE[0];
        covariance[35] = POSE_VARIANCE[1];
        let pose = json!({
            "header": at["header"],
            "pose": {"pose": at["pose"], "covariance": covariance},
        });
        let service = self.config.name("set_initial_pose");
        let set = self
            .call_service(
                &service,
                "nav2_msgs/srv/SetInitialPose",
                json!({"pose": pose}),
                CALL_WAIT,
            )
            .await;
        // A localization without the service still takes a pose on `initialpose`.
        if set.is_err() {
            let topic = self.config.name("initialpose");
            if let Err(e) = self
                .robot
                .publish(
                    &topic,
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    pose,
                    1,
                    Duration::ZERO,
                )
                .await
            {
                return ToolOutcome::failed(format!("localization took no pose: {e}"));
            }
        }
        ToolOutcome::ok(json!({
            "pose": {"x": spot.x, "y": spot.y, "yaw": yaw},
            "at": spot.what,
        }))
    }

    async fn relocalize(&self) -> ToolOutcome {
        let service = self.config.name("reinitialize_global_localization");
        match self
            .call_service(&service, "std_srvs/srv/Empty", json!({}), CALL_WAIT)
            .await
        {
            Ok(_) => ToolOutcome {
                message: "localization is searching the whole map: walk or turn the robot a \
                          little so it settles"
                    .to_owned(),
                ..ToolOutcome::ok(json!({"relocalizing": true}))
            },
            Err(why) => ToolOutcome::failed(why),
        }
    }

    async fn navigation(&self, args: &Value) -> ToolOutcome {
        let command = args["command"].as_str().unwrap_or_default();
        let Some(code) = lifecycle_code(command) else {
            return ToolOutcome::failed("navigation takes command pause, resume, reset or startup");
        };
        let service = self
            .config
            .name(&format!("{}/manage_nodes", self.config.lifecycle_manager));
        match self
            .call_service(
                &service,
                "nav2_msgs/srv/ManageLifecycleNodes",
                json!({"command": code}),
                MAP_WAIT,
            )
            .await
        {
            Ok(r) if r["success"].as_bool() == Some(true) => {
                ToolOutcome::ok(json!({"navigation": command, "done": true}))
            }
            Ok(_) => ToolOutcome::failed(format!(
                "Nav2's lifecycle manager could not {command} its servers"
            )),
            Err(why) => ToolOutcome::failed(why),
        }
    }

    async fn save_map(&self, args: &Value) -> ToolOutcome {
        let name = args["name"].as_str().map(str::trim).unwrap_or_default();
        if name.is_empty() || name.contains(['/', '\\']) || name.starts_with('.') {
            return ToolOutcome::failed("save_map needs a plain name, such as `ground_floor`");
        }
        // Which saver runs is asked first: a call to one that is not there waits out its timeout.
        let saver = self.config.name("map_saver/save_map");
        if self
            .robot
            .service_available(&saver, "nav2_msgs/srv/SaveMap", FIND_WAIT)
            .await
        {
            let request = json!({
                "map_topic": self.config.name("map"),
                "map_url": name,
                "image_format": "pgm",
                "map_mode": "trinary",
                "free_thresh": 0.25,
                "occupied_thresh": 0.65,
            });
            return match self
                .call_service(&saver, "nav2_msgs/srv/SaveMap", request, MAP_WAIT)
                .await
            {
                Ok(r) if r["result"].as_bool() == Some(true) => {
                    ToolOutcome::ok(json!({"saved": name, "by": "map_saver"}))
                }
                Ok(_) => ToolOutcome::failed("Nav2's map saver could not save the map"),
                Err(why) => ToolOutcome::failed(why),
            };
        }
        // A robot mapping with SLAM Toolbox saves through it.
        let slam = self.config.name("slam_toolbox/save_map");
        if self
            .robot
            .service_available(&slam, "slam_toolbox/srv/SaveMap", FIND_WAIT)
            .await
        {
            return match self
                .call_service(
                    &slam,
                    "slam_toolbox/srv/SaveMap",
                    json!({"name": {"data": name}}),
                    MAP_WAIT,
                )
                .await
            {
                Ok(_) => ToolOutcome::ok(json!({"saved": name, "by": "slam_toolbox"})),
                Err(why) => ToolOutcome::failed(why),
            };
        }
        ToolOutcome::failed(format!(
            "this robot runs no map saver: neither {saver} (Nav2's map_saver_server) nor {slam} \
             (SLAM Toolbox, while mapping) is there"
        ))
    }

    async fn load_map(&self, args: &Value) -> ToolOutcome {
        let url = args["url"].as_str().map(str::trim).unwrap_or_default();
        if url.is_empty() {
            return ToolOutcome::failed("load_map needs the map's url, its YAML file on the robot");
        }
        let service = self.config.name("map_server/load_map");
        match self
            .call_service(
                &service,
                "nav2_msgs/srv/LoadMap",
                json!({"map_url": url}),
                MAP_WAIT,
            )
            .await
        {
            Ok(r) => match r["result"].as_u64() {
                Some(0) => ToolOutcome::ok(json!({"loaded": url})),
                Some(1) => ToolOutcome::failed(format!("there is no map at {url}")),
                Some(2 | 3) => ToolOutcome::failed(format!("the map at {url} cannot be read")),
                _ => ToolOutcome::failed(format!("the map server could not load {url}")),
            },
            Err(why) => ToolOutcome::failed(why),
        }
    }
}

/// `ManageLifecycleNodes`' command for a word.
fn lifecycle_code(command: &str) -> Option<u8> {
    match command {
        "startup" => Some(0),
        "pause" => Some(1),
        "resume" => Some(2),
        "reset" => Some(3),
        _ => None,
    }
}

#[async_trait]
impl Tool for NavAdmin {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let base = vec![Resource::Base];
        let (risk, resources, reason) = match args["action"].as_str().unwrap_or_default() {
            "clear_costmaps" => {
                let names: Vec<String> = self
                    .costmaps(args)
                    .iter()
                    .map(|c| c.rsplit('/').next().unwrap_or(c).replace('_', " "))
                    .collect();
                let around = args["around_m"]
                    .as_f64()
                    .map_or(String::new(), |d| format!(" within {d:.1} m of the robot"));
                (
                    Risk::WorldEdit,
                    Vec::new(),
                    format!(
                        "clears Nav2's {}{around}: what its sensors saw there is gone until they \
                         see it again",
                        names.join(" and ")
                    ),
                )
            }
            "set_pose" => (
                Risk::WorldEdit,
                base,
                "tells localization where the robot stands: a wrong pose sends every walk astray"
                    .to_owned(),
            ),
            "relocalize" => (
                Risk::WorldEdit,
                base,
                "makes localization search the whole map for the robot".to_owned(),
            ),
            "navigation" => (
                Risk::WorldEdit,
                base,
                format!(
                    "{}s Nav2's navigation servers",
                    args["command"].as_str().unwrap_or("change")
                ),
            ),
            "save_map" => (
                Risk::Annotate,
                Vec::new(),
                format!("saves the map as {}", args["name"].as_str().unwrap_or("?")),
            ),
            "load_map" => (
                Risk::WorldEdit,
                base,
                format!(
                    "switches Nav2 to the map {}: localization starts over on it",
                    args["url"].as_str().unwrap_or("?")
                ),
            ),
            _ => return None,
        };
        Some(Ok(Assessment {
            risk,
            resources,
            reason,
            args: None,
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str().unwrap_or_default() {
            "clear_costmaps" => self.clear(&args).await,
            "set_pose" => self.set_pose(&args).await,
            "relocalize" => self.relocalize().await,
            "navigation" => self.navigation(&args).await,
            "save_map" => self.save_map(&args).await,
            "load_map" => self.load_map(&args).await,
            other => ToolOutcome::failed(format!(
                "action `{other}` is not one of clear_costmaps, set_pose, relocalize, navigation, \
                 save_map, load_map"
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nervros_ros::fake::FakeRobot;

    fn admin(robot: FakeRobot) -> NavAdmin {
        let profile: Profile = toml::from_str(
            "[robot]\nname = \"x\"\n[models]\nfile = \"m.toml\"\n[[place]]\nname = \"dock\"\npose = { x = 1.0, y = 2.0, yaw = 0.5 }\n",
        )
        .unwrap();
        let places = Places::new(&profile, None);
        NavAdmin::new(&Nav2Config::default(), &profile, Arc::new(robot), places)
    }

    #[tokio::test]
    async fn clearing_reaches_each_costmap_the_operator_names() {
        let robot = Arc::new(
            FakeRobot::new()
                .with_service("/global_costmap/clear_entirely_global_costmap", |_| {
                    Ok(json!({}))
                })
                .with_service("/local_costmap/clear_around_local_costmap", |_| {
                    Ok(json!({}))
                }),
        );
        let tool = admin(FakeRobot::new());
        let tool = NavAdmin {
            robot: Arc::clone(&robot) as Arc<dyn RobotPort>,
            ..tool
        };
        let all = tool
            .call(json!({"action": "clear_costmaps", "costmap": "global"}))
            .await;
        assert_eq!(
            all.data["cleared"],
            json!(["global_costmap"]),
            "{}",
            all.message
        );
        let near = tool
            .call(json!({"action": "clear_costmaps", "costmap": "local", "around_m": 1.5}))
            .await;
        assert_eq!(
            near.data["cleared"],
            json!(["local_costmap"]),
            "{}",
            near.message
        );
        assert_eq!(robot.calls()[1].1, json!({"reset_distance": 1.5}));
    }

    #[tokio::test]
    async fn a_pose_set_at_a_place_goes_to_localization_and_holds_the_base() {
        let robot = FakeRobot::new().with_service("/set_initial_pose", |req| {
            let p = &req["pose"]["pose"]["pose"]["position"];
            assert_eq!((p["x"].as_f64(), p["y"].as_f64()), (Some(1.0), Some(2.0)));
            Ok(json!({}))
        });
        let tool = admin(robot);
        let assessed = tool
            .assess(&json!({"action": "set_pose", "place": "dock"}))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(assessed.resources, vec![Resource::Base]);
        let out = tool
            .call(json!({"action": "set_pose", "place": "dock"}))
            .await;
        assert_eq!(out.data["at"], "dock", "{}", out.message);
        assert_eq!(lifecycle_code("pause"), Some(1));
    }
}
