//! Zones as Nav2 reads them: masks laid on the map's grid for its costmap filters.

use std::sync::Arc;
use std::time::Duration;

use nervros_ros::RobotPort;
use serde_json::{Value, json};

use crate::profile::TopicRef;
use crate::zones::{Rule, Zone, Zones};

/// The mask's cost in a keep-out zone: Nav2's occupied, which its keepout filter makes lethal.
const KEEP_OUT: i8 = 100;
/// The most cells a mask is laid on: a grid larger than this, or one whose size overflows, is
/// taken for a broken map.
const MAX_CELLS: usize = 16_000_000;
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
    fn value(self, zone: &Zone) -> Option<i8> {
        match (self, zone.rule) {
            (Self::KeepOut, Rule::KeepOut) => Some(KEEP_OUT),
            (Self::Speed { top_mps }, Rule::Slow(limit)) => limit
                .percent(top_mps)
                .filter(|p| p.is_finite())
                // A whole per cent, never 0, which would lift the limit.
                .map(|p| {
                    #[expect(clippy::cast_possible_truncation, reason = "clamped to 1..=100 first")]
                    let whole = p.round().clamp(1.0, 100.0) as i8;
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

/// The mask for `layer`: `map` (a `nav_msgs/msg/OccupancyGrid` as JSON) with each cell one of its
/// zones overlaps set, the slowest zone winning where slow zones overlap, and every other cell 0.
pub(super) fn mask(map: &Value, zones: &[Zone], layer: Layer) -> Option<Value> {
    let info = &map["info"];
    let resolution = info["resolution"]
        .as_f64()
        .filter(|r| r.is_finite() && *r > 0.0)?;
    let width = usize::try_from(info["width"].as_u64()?).ok()?;
    let height = usize::try_from(info["height"].as_u64()?).ok()?;
    let cells = width.checked_mul(height).filter(|n| *n <= MAX_CELLS)?;
    // Corners in cells on the grid, by the origin's pose: maps are rarely turned, but may be.
    let into_grid = nervros_ros::Transform::from_pose(&info["origin"])?.inverse();
    let mut data = vec![0_i8; cells];
    for zone in zones {
        let Some(value) = layer.value(zone) else {
            continue;
        };
        let corners: Vec<(f64, f64)> = zone
            .polygon
            .iter()
            .map(|[x, y]| {
                let p = into_grid.apply([*x, *y, 0.0]);
                (p[0] / resolution, p[1] / resolution)
            })
            .collect();
        // Only the cells under the box round the zone's corners: a zone is small beside a map.
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for (x, y) in &corners {
            (lo[0], hi[0]) = (lo[0].min(*x), hi[0].max(*x));
            (lo[1], hi[1]) = (lo[1].min(*y), hi[1].max(*y));
        }
        let columns = cells_between(lo[0], hi[0], width);
        let mut set = 0_usize;
        for j in cells_between(lo[1], hi[1], height) {
            for i in columns.clone() {
                // Every cell it overlaps, not only those whose centre it holds: a narrow zone
                // would otherwise set none and Nav2 would not hold it.
                if !overlaps(&corners, i, j) {
                    continue;
                }
                let cell = &mut data[j * width + i];
                if *cell == 0 || value < *cell {
                    *cell = value;
                }
                set += 1;
            }
        }
        if set == 0 {
            tracing::warn!(zone = %zone.name, "the zone covers no cell of the map, so Nav2 does not hold it");
        }
    }
    Some(json!({"header": map["header"], "info": info, "data": data}))
}

/// Whether the polygon of `corners`, in cells, overlaps cell `(i, j)`: holds its centre, has a
/// corner in it, or crosses one of its sides.
fn overlaps(corners: &[(f64, f64)], i: usize, j: usize) -> bool {
    #[expect(
        clippy::cast_precision_loss,
        reason = "cell indices are far below 2^52"
    )]
    let (x0, y0) = (i as f64, j as f64);
    let (x1, y1) = (x0 + 1.0, y0 + 1.0);
    // Strictly: a zone that only touches a cell along its side does not cover it.
    let inside_cell = |(x, y): (f64, f64)| x0 < x && x < x1 && y0 < y && y < y1;
    if inside(corners, x0 + 0.5, y0 + 0.5) || corners.iter().any(|c| inside_cell(*c)) {
        return true;
    }
    let sides = [
        ((x0, y0), (x1, y0)),
        ((x1, y0), (x1, y1)),
        ((x1, y1), (x0, y1)),
        ((x0, y1), (x0, y0)),
    ];
    let edges = corners.iter().zip(corners.iter().cycle().skip(1));
    edges
        .into_iter()
        .any(|(a, b)| sides.iter().any(|(c, d)| cross(*a, *b, *c, *d)))
}

/// Whether `(x, y)` is inside the polygon of `corners`, by the even-odd rule.
fn inside(corners: &[(f64, f64)], x: f64, y: f64) -> bool {
    let mut odd = false;
    for (a, b) in corners.iter().zip(corners.iter().cycle().skip(1)) {
        if (a.1 > y) != (b.1 > y) && x < (b.0 - a.0) * (y - a.1) / (b.1 - a.1) + a.0 {
            odd = !odd;
        }
    }
    odd
}

/// Whether segments `a`-`b` and `c`-`d` cross, each passing to both sides of the other.
fn cross(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> bool {
    let side = |p: (f64, f64), q: (f64, f64), r: (f64, f64)| {
        (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0)
    };
    let (d1, d2) = (side(c, d, a), side(c, d, b));
    let (d3, d4) = (side(a, b, c), side(a, b, d));
    d1 * d2 < 0.0 && d3 * d4 < 0.0
}

/// The cells from `lo` to `hi`, in cells along an axis of `n`, those on the grid.
fn cells_between(lo: f64, hi: f64, n: usize) -> std::ops::Range<usize> {
    if !(lo.is_finite() && hi.is_finite()) {
        return 0..0;
    }
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss,
        reason = "clamped to 0..=n first, and n is at most MAX_CELLS"
    )]
    let at = |v: f64| v.clamp(0.0, n as f64) as usize;
    at(lo.floor())..at(hi.ceil())
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
    // Detached: it holds the zones weakly and ends with them.
    tokio::spawn(async move {
        let types = (topic.as_str(), "nav_msgs/msg/OccupancyGrid");
        let alive = || zones.strong_count() > 0;
        let Some(publisher) =
            crate::integrations::latched_retrying(robot.as_ref(), types, alive).await
        else {
            tracing::warn!(%topic, "no {} mask: this robot keeps no latched publishers", layer.name());
            return;
        };
        // The grid the last mask was laid on, and the zones it carried: a map that only fills in,
        // as one being mapped does every few seconds, and a change to the other layer's zones,
        // need no new mask.
        let mut sent_for: Option<(Value, Vec<Zone>)> = None;
        loop {
            if let Ok(grid) = robot
                .latest_shared(&map.topic, &map.msg_type, MAP_WAIT)
                .await
            {
                changes.borrow_and_update();
                let Some(zones) = zones.upgrade() else {
                    return;
                };
                let mine: Vec<Zone> = zones
                    .all()
                    .into_iter()
                    .filter(|z| layer.value(z).is_some())
                    .collect();
                drop(zones);
                let info = &grid["info"];
                let laid_on = json!([
                    grid["header"]["frame_id"],
                    info["resolution"],
                    info["width"],
                    info["height"],
                    info["origin"]
                ]);
                if sent_for
                    .as_ref()
                    .is_none_or(|(on, carried)| *on != laid_on || *carried != mine)
                {
                    let carried = mine.clone();
                    let built =
                        tokio::task::spawn_blocking(move || mask(&grid, &mine, layer)).await;
                    match built {
                        Ok(Some(message)) => {
                            publisher.send(message);
                            sent_for = Some((laid_on, carried));
                        }
                        Ok(None) => {
                            tracing::warn!(%topic, "no {} mask: the map's grid is unusable or too large", layer.name());
                            sent_for = Some((laid_on, carried));
                        }
                        Err(_) => {}
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
    // Detached: it ends when the zones' change channel closes with them.
    tokio::spawn(async move {
        let types = (topic.as_str(), "nav2_msgs/msg/CostmapFilterInfo");
        let living = || alive.has_changed().is_ok();
        let Some(publisher) =
            crate::integrations::latched_retrying(robot.as_ref(), types, living).await
        else {
            tracing::warn!(%topic, "no speed filter info: this robot keeps no latched publishers");
            return;
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
    fn a_sliver_of_a_zone_marks_every_cell_it_crosses() {
        // 0.1 m wide along y = 0.5, across all four cells of the first row, through no centre.
        let zones = [zone(
            &[[0.2, 0.45], [3.8, 0.45], [3.8, 0.47], [0.2, 0.47]],
            Rule::KeepOut,
        )];
        let m = mask(&grid(), &zones, Layer::KeepOut).unwrap();
        assert_eq!(m["data"], json!([100, 100, 100, 100, 0, 0, 0, 0]));
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

    #[test]
    fn a_grid_too_large_or_overflowing_gets_no_mask() {
        let mut huge = grid();
        huge["info"]["width"] = json!(u64::MAX);
        assert!(mask(&huge, &[], Layer::KeepOut).is_none());
        huge["info"]["width"] = json!(5000);
        huge["info"]["height"] = json!(5000);
        assert!(mask(&huge, &[], Layer::KeepOut).is_none());
    }
}
