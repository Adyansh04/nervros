//! Nav2: where the robot may walk and how, as the navigation stack keeps it.
//!
//! The zones reach Nav2's costmap filters as masks: keep-out zones its keepout filter, slow zones
//! its speed filter. The guard reads Nav2's goal types to keep goals out of the zones ([`goals`]).

mod admin;
mod check;
pub(crate) mod goals;
mod masks;
mod monitor;
mod motion;
mod status;

use std::sync::Arc;

use async_trait::async_trait;
use nervros_ros::RobotPort;
use serde::Deserialize;

use super::{Goals, Integration, NativeSkill, Parts, has_action, resolve};
use crate::doctor::Check;
use crate::tools::Tool;

/// `[nav2]`: where Nav2 runs, and what NervROS gives it.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Nav2Config {
    /// The namespace Nav2's nodes and servers run in, such as `/robot1`; empty for none.
    #[serde(default)]
    pub namespace: String,
    /// Where the keep-out zones go out as a `nav_msgs/msg/OccupancyGrid` on the map's grid,
    /// latched, every cell in a zone occupied, for Nav2's keepout filter; needs `[world] map`.
    /// Without it only the agent's own checks keep the robot out.
    pub keepout_mask: Option<String>,
    /// The slow zones, for Nav2's speed filter.
    pub speed_zones: Option<SpeedZonesConfig>,
    /// The lifecycle manager of Nav2's navigation servers, whose `is_active` and
    /// `manage_nodes` NervROS calls.
    #[serde(default = "default_lifecycle_manager")]
    pub lifecycle_manager: String,
    /// The collision monitor node.
    #[serde(default = "default_collision_monitor")]
    pub collision_monitor: String,
    /// The costmaps, by the name their nodes run under: their clear services hang off these.
    #[serde(default = "default_costmaps")]
    pub costmaps: Vec<String>,
    /// How fast a straight walk NervROS runs goes, m/s.
    #[serde(default = "default_drive_speed")]
    pub drive_speed_mps: f64,
    /// Whether the robot runs Nav2's docking server, for docking and undocking steps.
    #[serde(default)]
    pub docking: bool,
}

fn default_drive_speed() -> f64 {
    0.25
}

fn default_lifecycle_manager() -> String {
    "lifecycle_manager_navigation".to_owned()
}

fn default_collision_monitor() -> String {
    "collision_monitor".to_owned()
}

fn default_costmaps() -> Vec<String> {
    vec![
        "global_costmap/global_costmap".to_owned(),
        "local_costmap/local_costmap".to_owned(),
    ]
}

impl Default for Nav2Config {
    fn default() -> Self {
        Self {
            namespace: String::new(),
            keepout_mask: None,
            speed_zones: None,
            lifecycle_manager: default_lifecycle_manager(),
            collision_monitor: default_collision_monitor(),
            costmaps: default_costmaps(),
            drive_speed_mps: default_drive_speed(),
            docking: false,
        }
    }
}

/// `[nav2.speed_zones]`: where the slow zones go out for Nav2's speed filter.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SpeedZonesConfig {
    /// Where the slow zones go out as a `nav_msgs/msg/OccupancyGrid` on the map's grid, latched:
    /// each cell the per cent of the top speed the slowest zone over it allows, 0 (no limit)
    /// elsewhere; needs `[world] map`.
    pub mask: String,
    /// Where NervROS publishes the speed filter's `nav2_msgs/msg/CostmapFilterInfo`, latched,
    /// naming the mask in per cent; leave it out when the robot runs its own filter info server.
    pub info: Option<String>,
}

impl Nav2Config {
    /// A Nav2 name as the robot's nodes resolve it.
    #[must_use]
    pub fn name(&self, name: &str) -> String {
        resolve(&self.namespace, name)
    }

    /// The collision monitor node's full name.
    #[must_use]
    pub fn monitor_node(&self) -> String {
        self.name(&self.collision_monitor)
    }
}

/// The procedures Nav2's faults call for.
pub(crate) const PROCEDURES: [&str; 2] = [
    include_str!("nav2/procedures/robot-held-back.md"),
    include_str!("nav2/procedures/no-path-there.md"),
];

/// The Nav2 integration.
pub struct Nav2 {
    watcher: Arc<status::Watcher>,
    tools: Vec<Arc<dyn Tool>>,
    motions: Arc<motion::Motions>,
    natives: Vec<Arc<dyn NativeSkill>>,
    goals: Arc<Goals>,
}

impl std::fmt::Debug for Nav2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Nav2")
            .field("seen", &self.watcher.seen())
            .finish_non_exhaustive()
    }
}

impl Nav2 {
    /// Starts publishing what Nav2 reads of the zones.
    pub(crate) fn start(config: &Nav2Config, parts: &Parts<'_>, goals: &Arc<Goals>) -> Arc<Self> {
        let robot = || Arc::clone(parts.robot);
        let map = parts.profile.world.as_ref().and_then(|w| w.map.clone());
        if let (Some(topic), Some(map)) = (config.keepout_mask.clone(), map.clone()) {
            masks::spawn(robot(), map, topic, parts.zones, masks::Layer::KeepOut);
        }
        if let (Some(speed), Some(map)) = (&config.speed_zones, map) {
            let top_mps = parts.zones.top_speed_mps();
            let layer = masks::Layer::Speed { top_mps };
            masks::spawn(robot(), map, speed.mask.clone(), parts.zones, layer);
            if let Some(info) = &speed.info {
                let frame = parts.profile.ros.map_frame.clone();
                masks::spawn_speed_info(
                    robot(),
                    info.clone(),
                    speed.mask.clone(),
                    frame,
                    parts.zones,
                );
            }
        }
        let watcher = status::Watcher::start(config, robot());
        let tools: Vec<Arc<dyn Tool>> = vec![
            Arc::new(status::NavStatus::new(
                Arc::clone(&watcher),
                robot(),
                Arc::clone(parts.zones),
            )),
            Arc::new(check::NavCheck::new(
                config,
                parts.profile,
                (robot(), Arc::clone(parts.places), Arc::clone(parts.zones)),
                Arc::clone(&watcher),
            )),
            monitor::Monitor::new(config, robot(), parts.file("collision_monitor")),
            Arc::new(admin::NavAdmin::new(
                config,
                parts.profile,
                robot(),
                Arc::clone(parts.places),
            )),
        ];
        let (motions, natives) = motion::Motions::natives(
            config,
            parts.profile,
            (robot(), Arc::clone(parts.places), goals),
        );
        let goals = Arc::clone(goals);
        Arc::new(Self {
            watcher,
            tools,
            motions,
            natives,
            goals,
        })
    }
}

#[async_trait]
impl Integration for Nav2 {
    fn id(&self) -> &'static str {
        "nav2"
    }

    fn tools(&self) -> Vec<Arc<dyn Tool>> {
        self.tools.clone()
    }

    fn status(&self) -> Vec<String> {
        self.watcher.seen().parts()
    }

    fn natives(&self) -> Vec<Arc<dyn NativeSkill>> {
        self.natives.clone()
    }

    fn attach(&self, session: &crate::session::SessionHandle) {
        self.motions.attach(session);
    }

    async fn stop(&self) {
        self.goals.cancel_all().await;
    }
}

/// What of Nav2 runs, and whether its keepout filter reads the mask.
pub(crate) async fn discover(config: &Nav2Config, robot: &dyn RobotPort) -> Vec<Check> {
    let mut out = Vec::new();
    let navigate = config.name("navigate_to_pose");
    out.push(match robot.graph_detail().await {
        Ok(graph) => Check {
            ok: has_action(&graph.topics, &navigate),
            what: format!("Nav2 navigation {navigate}"),
        },
        Err(e) => Check {
            ok: false,
            what: format!("Nav2: reading the ROS graph: {e}"),
        },
    });
    // Published for nothing, a mask would leave the zones to the agent's own checks. A speed filter
    // reads its mask only once it has the filter info NervROS publishes, so that is what is checked.
    let read_by = [
        config
            .keepout_mask
            .as_ref()
            .map(|m| (m, "keep-out mask", "keepout")),
        config.speed_zones.as_ref().map(|s| match &s.info {
            Some(info) => (info, "speed filter info", "speed"),
            None => (&s.mask, "speed mask", "speed"),
        }),
    ];
    for (topic, what, filter) in read_by.into_iter().flatten() {
        let read = robot
            .endpoints(topic)
            .await
            .is_ok_and(|e| !e.subscribers.is_empty());
        out.push(Check {
            ok: read,
            what: if read {
                format!("{what} {topic}: Nav2 reads it")
            } else {
                format!("{what} {topic}: nothing reads it; is Nav2's {filter} filter on?")
            },
        });
    }
    out
}
