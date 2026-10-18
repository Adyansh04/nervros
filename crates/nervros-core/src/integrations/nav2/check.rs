//! `nav_check`: whether the robot can get somewhere, how far and how long, and through which
//! zones, by asking Nav2's planner without moving; and what the costmap holds at a spot.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::RobotPort;
use serde_json::{Value, json};

use super::Nav2Config;
use super::status::Watcher;
use crate::integrations::{Spot, locate, stamped};
use crate::places::Places;
use crate::profile::Profile;
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};
use crate::zones::{Rule, Zones};

/// The planner may take a while on a large map.
const PLAN_WAIT: Duration = Duration::from_secs(15);
const COST_WAIT: Duration = Duration::from_secs(3);
/// How far from an object's box a path is planned to.
const BESIDE_M: f64 = 0.5;
/// Nav2's costs: no obstacle near, inside the inflation, inscribed, lethal, unknown.
const INSCRIBED: f64 = 253.0;
const LETHAL: f64 = 254.0;
const UNKNOWN: f64 = 255.0;

/// The tool.
pub(super) struct NavCheck {
    spec: ToolSpec,
    config: Nav2Config,
    profile: Profile,
    robot: Arc<dyn RobotPort>,
    places: Arc<Places>,
    zones: Arc<Zones>,
    watcher: Arc<Watcher>,
}

impl NavCheck {
    pub(super) fn new(
        config: &Nav2Config,
        profile: &Profile,
        (robot, places, zones): (Arc<dyn RobotPort>, Arc<Places>, Arc<Zones>),
        watcher: Arc<Watcher>,
    ) -> Self {
        let spec = ToolSpec::new(
            "nav_check",
            "Asks Nav2 without moving the robot. path: whether it can walk to a place, a room, an \
             object or x and y on the map, how far it is, about how long it takes, and which \
             zones the way crosses (from where it stands, or from from_place); the planned way \
             is drawn in the 3D view. cost: how close to an obstacle a spot is in Nav2's \
             costmap. Not while Nav2 is walking: a path request would cut into its own planner.",
            json!({
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["path", "cost"]},
                    "place": {"type": "string", "description": "A named place"},
                    "room": {"type": "string", "description": "A room's id or name"},
                    "object": {"type": "string", "description": "An object's id from find_objects; the spot beside it"},
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "from_place": {"type": "string", "description": "For path: start from this place instead of where the robot stands"},
                },
                "required": ["action"],
            }),
            Risk::Observe,
        );
        Self {
            spec,
            config: config.clone(),
            profile: profile.clone(),
            robot,
            places,
            zones,
            watcher,
        }
    }

    fn robot_xy(&self) -> Option<(f64, f64)> {
        let (map, base) = (&self.profile.ros.map_frame, &self.profile.ros.base_frame);
        self.robot
            .transform(map, base)
            .ok()
            .map(|t| (t.translation[0], t.translation[1]))
    }

    async fn path(&self, args: &Value) -> ToolOutcome {
        let seen = self.watcher.now(self.robot.as_ref()).await;
        if seen.moving() {
            return ToolOutcome::refused(format!(
                "Nav2 is moving the base now ({}): a path request would cut into the planner it \
                 is using. Ask again once it is still",
                seen.running.join(", ")
            ));
        }
        let from = match args["from_place"].as_str().filter(|p| !p.trim().is_empty()) {
            Some(place) => {
                let at = json!({"place": place});
                match locate(&at, self.ctx(), None, BESIDE_M).await {
                    Ok(spot) => Some(spot),
                    Err(why) => return ToolOutcome::failed(why),
                }
            }
            None => None,
        };
        let start_xy = from
            .as_ref()
            .map(|s| (s.x, s.y))
            .or_else(|| self.robot_xy());
        let to = match locate(args, self.ctx(), start_xy, BESIDE_M).await {
            Ok(spot) => to_face(spot, start_xy),
            Err(why) => return ToolOutcome::failed(why),
        };
        let frame = &self.profile.ros.map_frame;
        let goal = json!({
            "goal": stamped(frame, to.x, to.y, to.yaw.unwrap_or(0.0)),
            "start": from.as_ref().map_or_else(
                || stamped(frame, 0.0, 0.0, 0.0),
                |s| stamped(frame, s.x, s.y, s.yaw.unwrap_or(0.0)),
            ),
            "planner_id": "",
            "use_start": from.is_some(),
        });
        let action = self.config.name("compute_path_to_pose");
        let planned = async {
            let goal = self
                .robot
                .send_goal(
                    &action,
                    "nav2_msgs/action/ComputePathToPose",
                    goal,
                    PLAN_WAIT,
                )
                .await?;
            tokio::time::timeout(PLAN_WAIT, goal.result)
                .await
                .map_err(|_| nervros_ros::RosError::Timeout(action.clone()))?
                .map_err(|_| nervros_ros::RosError::Middleware("the planner went away".into()))?
        };
        let result = match planned.await {
            Ok(r) => r.result,
            Err(e) => return ToolOutcome::failed(format!("Nav2's planner did not answer: {e}")),
        };
        let code = result["error_code"].as_u64().unwrap_or(0);
        let poses: Vec<(f64, f64)> = result["path"]["poses"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|p| {
                let at = &p["pose"]["position"];
                Some((at["x"].as_f64()?, at["y"].as_f64()?))
            })
            .collect();
        if code != 0 || poses.len() < 2 {
            let why = result["error_msg"].as_str().filter(|m| !m.is_empty());
            return ToolOutcome::ok(json!({
                "reachable": false,
                "to": to.what,
                "why": no_path(code, why),
            }));
        }
        let top = self.zones.top_speed_mps();
        let way = Way::along(&poses, &self.zones, top);
        let mut out = json!({
            "reachable": true,
            "to": to.what,
            "length_m": round(way.length_m, 1),
        });
        if let Some(s) = way.seconds {
            out["about_s"] = json!(s.round());
        }
        if !way.slow.is_empty() {
            out["through_slow_zones"] = json!(way.slow);
        }
        if !way.keep_out.is_empty() {
            // Nav2 plans around its keepout mask; a way through one means it does not read it.
            out["through_keep_out_zones"] = json!(way.keep_out);
        }
        ToolOutcome::ok(out)
    }

    async fn cost(&self, args: &Value) -> ToolOutcome {
        let spot = match locate(args, self.ctx(), self.robot_xy(), 0.0).await {
            Ok(s) => s,
            Err(why) => return ToolOutcome::failed(why),
        };
        let Some(costmap) = self.config.costmaps.first() else {
            return ToolOutcome::failed("the profile names no costmap");
        };
        let service = self.config.name(&costmap_service(costmap, "get_cost"));
        let request = json!({"use_footprint": false, "x": spot.x, "y": spot.y, "theta": 0.0});
        match self
            .robot
            .call(&service, "nav2_msgs/srv/GetCost", request, COST_WAIT)
            .await
        {
            Ok(r) => {
                let cost = r["cost"].as_f64().unwrap_or(UNKNOWN);
                ToolOutcome::ok(json!({"at": spot.what, "cost": cost, "means": cost_words(cost)}))
            }
            Err(e) => ToolOutcome::failed(format!("Nav2's costmap did not answer: {e}")),
        }
    }

    fn ctx(&self) -> (&Profile, &Places, &dyn RobotPort) {
        (&self.profile, &self.places, self.robot.as_ref())
    }
}

#[async_trait]
impl Tool for NavCheck {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str().unwrap_or("path") {
            "path" => self.path(&args).await,
            "cost" => self.cost(&args).await,
            other => ToolOutcome::failed(format!("action `{other}` is not one of path, cost")),
        }
    }
}

/// A spot with no heading of its own faces along the way there.
fn to_face(mut to: Spot, from: Option<(f64, f64)>) -> Spot {
    if to.yaw.is_none()
        && let Some((fx, fy)) = from
    {
        to.yaw = Some((to.y - fy).atan2(to.x - fx));
    }
    to
}

/// One of a costmap's services: `global_costmap/global_costmap` serves
/// `global_costmap/get_cost_global_costmap`.
pub(super) fn costmap_service(costmap: &str, service: &str) -> String {
    match costmap.rsplit_once('/') {
        Some((ns, name)) => format!("{ns}/{service}_{name}"),
        None => format!("{service}_{costmap}"),
    }
}

fn round(v: f64, places: i32) -> f64 {
    let f = 10_f64.powi(places);
    (v * f).round() / f
}

/// Why the planner found no way, in words.
fn no_path(code: u64, said: Option<&str>) -> String {
    let why = match code {
        201 => "the planner it asked for does not exist",
        202 => "it could not place the robot or the goal on the map (TF)",
        203 => "the robot stands outside the map",
        204 => "the spot is outside the map",
        205 => "the robot stands on an obstacle in the costmap; clearing the costmap may help",
        206 => "the spot is on an obstacle in the costmap",
        207 => "the planner ran out of time",
        208 => "every way there is blocked",
        _ => "the planner gave up",
    };
    match said {
        Some(text) => format!("{why} ({text})"),
        None => why.to_owned(),
    }
}

/// A costmap cost in words.
fn cost_words(cost: f64) -> &'static str {
    if cost >= UNKNOWN {
        "unknown: no sensor has seen it"
    } else if cost >= LETHAL {
        "an obstacle"
    } else if cost >= INSCRIBED {
        "too close to an obstacle for the robot to stand"
    } else if cost > 0.0 {
        "near an obstacle; the robot may pass"
    } else {
        "free"
    }
}

/// A planned way, measured against the zones.
#[derive(Debug, Default, PartialEq)]
struct Way {
    length_m: f64,
    /// About how long it takes at the top speed, slowed in the slow zones.
    seconds: Option<f64>,
    /// Each slow zone it crosses, how far inside, and its speed.
    slow: Vec<Value>,
    /// Each keep-out zone it crosses.
    keep_out: Vec<String>,
}

impl Way {
    fn along(poses: &[(f64, f64)], zones: &Zones, top: Option<f64>) -> Self {
        let mut way = Self::default();
        let mut inside: Vec<(String, String, f64)> = Vec::new();
        let mut seconds = 0.0;
        for pair in poses.windows(2) {
            let ((x1, y1), (x2, y2)) = (pair[0], pair[1]);
            let d = (x2 - x1).hypot(y2 - y1);
            way.length_m += d;
            let (mx, my) = (f64::midpoint(x1, x2), f64::midpoint(y1, y2));
            let share = match zones.slow_at(mx, my) {
                Some((zone, percent)) => {
                    let words = match zone.rule {
                        Rule::Slow(limit) => limit.words(),
                        Rule::KeepOut => String::new(),
                    };
                    match inside.iter_mut().find(|(n, _, _)| *n == zone.name) {
                        Some(entry) => entry.2 += d,
                        None => inside.push((zone.name, words, d)),
                    }
                    percent / 100.0
                }
                None => 1.0,
            };
            if let Some(t) = top {
                seconds += d / (t * share);
            }
            if let Some(zone) = zones.at(mx, my)
                && !way.keep_out.contains(&zone.name)
            {
                way.keep_out.push(zone.name);
            }
        }
        way.seconds = top.map(|_| seconds);
        way.slow = inside
            .into_iter()
            .map(|(zone, speed, m)| json!({"zone": zone, "speed": speed, "inside_m": round(m, 1)}))
            .collect();
        way
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::zones::{SpeedLimit, Zone};

    fn zones() -> Arc<Zones> {
        let profile: Profile = toml::from_str(
            "[robot]\nname = \"x\"\n[models]\nfile = \"m.toml\"\n[geofence]\ntop_speed_mps = 0.5\n",
        )
        .unwrap();
        let zones = Zones::new(&profile, None);
        zones
            .add(Zone {
                name: "corridor".into(),
                polygon: vec![[2.0, -1.0], [4.0, -1.0], [4.0, 1.0], [2.0, 1.0]],
                rule: Rule::Slow(SpeedLimit::Percent(50.0)),
                fixed: false,
            })
            .unwrap();
        zones
    }

    #[test]
    fn a_way_is_measured_and_slowed_where_it_crosses_a_slow_zone() {
        let poses: Vec<(f64, f64)> = (0..=10).map(|i| (f64::from(i) * 0.5, 0.0)).collect();
        let way = Way::along(&poses, &zones(), Some(0.5));
        assert!((way.length_m - 5.0).abs() < 1e-9);
        // 3 m at 0.5 m/s and 2 m at half that: 6 s + 8 s.
        assert!(
            (way.seconds.unwrap() - 14.0).abs() < 1e-9,
            "{:?}",
            way.seconds
        );
        assert_eq!(
            way.slow,
            vec![json!({"zone": "corridor", "speed": "50%", "inside_m": 2.0})]
        );
        assert!(way.keep_out.is_empty());
    }

    #[test]
    fn the_planners_codes_and_the_costs_read_as_words() {
        assert_eq!(no_path(208, None), "every way there is blocked");
        assert!(no_path(206, Some("goal in collision")).contains("(goal in collision)"));
        assert_eq!(cost_words(0.0), "free");
        assert_eq!(cost_words(254.0), "an obstacle");
        assert_eq!(
            costmap_service("global_costmap/global_costmap", "get_cost"),
            "global_costmap/get_cost_global_costmap"
        );
    }
}
