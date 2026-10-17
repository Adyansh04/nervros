//! Zones as Nav2 reads them: masks laid on the map's grid for its costmap filters.

use std::sync::Arc;
use std::time::Duration;

use nervros_ros::RobotPort;
use serde_json::{Value, json};

use crate::profile::TopicRef;
use crate::zones::{Rule, Zone, Zones};

/// The mask's cost in a keep-out zone: Nav2's occupied, which its keepout filter makes lethal.
const KEEP_OUT: i64 = 100;
/// A speed filter's mask in per cent: `CostmapFilterInfo.type` `SPEED_FILTER_PERCENT`.
const SPEED_FILTER_PERCENT: u8 = 1;
const MAP_WAIT: Duration = Duration::from_secs(3);
/// How often a mask publisher looks for a new map.
const MAP_POLL: Duration = Duration::from_secs(10);

/// Which zones a mask carries, and what it writes in their cells.
#[derive(Debug, Clone, Copy)]
pub(super) enum Layer {
    /// Keep-out zones, as occupied cells for Nav2's keepout filter.
    KeepOut,
    /// Slow zones, as the per cent of the top speed the slowest zone over a cell allows, for
    /// Nav2's speed filter; 0 is no limit.
    Speed {
        /// The robot's top speed, which a zone's speed in m/s is a share of.
        top_mps: Option<f64>,
    },
}

impl Layer {
    /// What `zone` writes in its cells, if this mask carries it.
    fn value(self, zone: &Zone) -> Option<i64> {
        match (self, zone.rule) {
            (Self::KeepOut, Rule::KeepOut) => Some(KEEP_OUT),
            (Self::Speed { top_mps }, Rule::Slow(limit)) => limit
                .percent(top_mps)
                .filter(|p| p.is_finite())
                // A whole per cent, never 0, which would lift the limit.
                .map(|p| {
                    #[expect(clippy::cast_possible_truncation, reason = "clamped to 1..=100 first")]
                    let whole = p.round().clamp(1.0, 100.0) as i64;
                    whole
                }),
            _ => None,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Self::KeepOut => "keep-out",
            Self::Speed { .. } => "speed",
        }
    }
}

/// The mask for `layer`: `map` (a `nav_msgs/msg/OccupancyGrid` as JSON) with each cell whose
/// centre is in one of its zones set, the slowest zone winning where slow zones overlap, and every
/// other cell 0.
pub(super) fn mask(map: &Value, zones: &[Zone], layer: Layer) -> Option<Value> {
    let info = &map["info"];
    let resolution = info["resolution"].as_f64().filter(|r| *r > 0.0)?;
    let width = usize::try_from(info["width"].as_u64()?).ok()?;
    let height = usize::try_from(info["height"].as_u64()?).ok()?;
    let origin = nervros_ros::Transform::from_pose(&info["origin"])?;
    let mut data = vec![0_i64; width * height];
    // A cell's centre in the map frame, by the origin's pose: maps are rarely turned, but may be.
    let centre = |i: usize, j: usize| {
        #[expect(
            clippy::cast_precision_loss,
            reason = "grid indices are far below 2^52"
        )]
        let local = [
            (i as f64 + 0.5) * resolution,
            (j as f64 + 0.5) * resolution,
            0.0,
        ];
        let p = origin.apply(local);
        (p[0], p[1])
    };
    for zone in zones {
        let Some(value) = layer.value(zone) else {
            continue;
        };
        for j in 0..height {
            for i in 0..width {
                let (x, y) = centre(i, j);
                let cell = &mut data[j * width + i];
                if zone.contains(x, y) && (*cell == 0 || value < *cell) {
                    *cell = value;
                }
            }
        }
    }
    Some(json!({"header": map["header"], "info": info, "data": data}))
}

/// Publishes `layer`'s mask on `topic`, latched, whenever the zones or the map change: a Nav2
/// costmap filter reads it as one more layer of its costmaps. Starts with the map, and an empty
/// mask is published too, so the filter never waits for one. Ends with the session the zones are
/// of.
pub(super) fn spawn(
    robot: Arc<dyn RobotPort>,
    map: TopicRef,
    topic: String,
    zones: &Arc<Zones>,
    layer: Layer,
) {
    let mut changes = zones.changes();
    let zones = Arc::downgrade(zones);
    tokio::spawn(async move {
        let publisher = match robot
            .latched_publisher(&topic, "nav_msgs/msg/OccupancyGrid")
            .await
        {
            Ok(p) => p,
            Err(e) => {
                let mask = layer.name();
                tracing::warn!(%topic, error = %e, "no {mask} mask: its publisher failed");
                return;
            }
        };
        // The grid the last mask was laid on, and the zones' version: a map that only fills in,
        // as one being mapped does every few seconds, needs no new mask.
        let mut sent_for: Option<(Value, u64)> = None;
        loop {
            if let Ok(grid) = robot
                .latest_shared(&map.topic, &map.msg_type, MAP_WAIT)
                .await
            {
                let version = *changes.borrow_and_update();
                let info = &grid["info"];
                let laid_on = json!([
                    grid["header"]["frame_id"],
                    info["resolution"],
                    info["width"],
                    info["height"],
                    info["origin"]
                ]);
                if sent_for.as_ref() != Some(&(laid_on.clone(), version)) {
                    let Some(zones) = zones.upgrade() else {
                        return;
                    };
                    let all = zones.all();
                    drop(zones);
                    let built = tokio::task::spawn_blocking(move || mask(&grid, &all, layer)).await;
                    if let Ok(Some(message)) = built {
                        publisher.send(message);
                        sent_for = Some((laid_on, version));
                    }
                }
            }
            tokio::select! {
                alive = changes.changed() => if alive.is_err() { return },
                () = tokio::time::sleep(MAP_POLL) => {}
            }
        }
    });
}

/// Publishes the speed filter's `CostmapFilterInfo` on `topic`, latched, naming `mask` as a mask in
/// per cent, so the robot needs no filter info server of its own. Kept as long as the zones are.
pub(super) fn spawn_speed_info(
    robot: Arc<dyn RobotPort>,
    topic: String,
    mask: String,
    frame: String,
    zones: &Arc<Zones>,
) {
    let mut alive = zones.changes();
    tokio::spawn(async move {
        let publisher = match robot
            .latched_publisher(&topic, "nav2_msgs/msg/CostmapFilterInfo")
            .await
        {
            Ok(p) => p,
            Err(e) => {
                tracing::warn!(%topic, error = %e, "no speed filter info: its publisher failed");
                return;
            }
        };
        publisher.send(json!({
            "header": {"frame_id": frame},
            "type": SPEED_FILTER_PERCENT,
            "filter_mask_topic": mask,
            "base": 0.0,
            "multiplier": 1.0,
        }));
        // The publisher lives while the zones do: a latched message outlives no publisher.
        while alive.changed().await.is_ok() {}
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::zones::SpeedLimit;

    fn grid() -> Value {
        json!({
            "header": {"frame_id": "map"},
            "info": {"resolution": 1.0, "width": 4, "height": 2,
                     "origin": {"position": {"x": 0.0, "y": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
        })
    }

    fn zone(polygon: &[[f64; 2]], rule: Rule) -> Zone {
        Zone {
            name: "z".into(),
            polygon: polygon.to_vec(),
            rule,
            fixed: false,
        }
    }

    #[test]
    fn the_keep_out_mask_marks_the_cells_whose_centres_are_in_a_keep_out_zone() {
        let zones = [
            zone(
                &[[1.0, 0.0], [3.0, 0.0], [3.0, 1.0], [1.0, 1.0]],
                Rule::KeepOut,
            ),
            zone(
                &[[0.0, 1.0], [4.0, 1.0], [4.0, 2.0], [0.0, 2.0]],
                Rule::Slow(SpeedLimit::Percent(30.0)),
            ),
        ];
        let m = mask(&grid(), &zones, Layer::KeepOut).unwrap();
        assert_eq!(m["data"], json!([0, 100, 100, 0, 0, 0, 0, 0]));
        assert_eq!(m["header"]["frame_id"], "map");
    }

    #[test]
    fn the_speed_mask_holds_the_slowest_zone_over_each_cell_and_0_elsewhere() {
        let zones = [
            zone(
                &[[0.0, 0.0], [3.0, 0.0], [3.0, 1.0], [0.0, 1.0]],
                Rule::Slow(SpeedLimit::Percent(50.0)),
            ),
            zone(
                &[[2.0, 0.0], [4.0, 0.0], [4.0, 1.0], [2.0, 1.0]],
                Rule::Slow(SpeedLimit::Mps(0.15)),
            ),
            zone(
                &[[0.0, 1.0], [1.0, 1.0], [1.0, 2.0], [0.0, 2.0]],
                Rule::KeepOut,
            ),
        ];
        // 0.15 m/s of 0.6 m/s is 25%, and wins where it overlaps the 50% zone.
        let m = mask(&grid(), &zones, Layer::Speed { top_mps: Some(0.6) }).unwrap();
        assert_eq!(m["data"], json!([50, 50, 25, 25, 0, 0, 0, 0]));
        // With no top speed, a zone in m/s cannot be laid; the others still are.
        let m = mask(&grid(), &zones, Layer::Speed { top_mps: None }).unwrap();
        assert_eq!(m["data"], json!([50, 50, 50, 0, 0, 0, 0, 0]));
    }
}
