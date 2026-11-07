//! Topics drawn as RViz draws them: occupancy grids, marker arrays and laser scans, from the
//! profile's `[[viz.layer]]` list, and the switches the operator hides any layer with.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use nervros_ros::{Transform, lock};
use rerun::RecordingStream;
use serde_json::{Value, json};

use crate::grid::{grid, put_grid};
use crate::{f32s, num, put, put_static, xyz};

/// Every layer the viewer draws, in order, each shown or hidden by the operator.
#[derive(Debug, Default)]
pub struct Layers(Mutex<Vec<(String, bool)>>);

impl Layers {
    fn lock(&self) -> MutexGuard<'_, Vec<(String, bool)>> {
        lock(&self.0)
    }

    /// Adds a layer at the end, unless one has that name.
    pub fn add(&self, name: &str, shown: bool) {
        let mut list = self.lock();
        if !list.iter().any(|(n, _)| n == name) {
            list.push((name.to_owned(), shown));
        }
    }

    /// The layers and whether each is shown.
    #[must_use]
    pub fn list(&self) -> Vec<(String, bool)> {
        self.lock().clone()
    }

    /// Shows or hides a layer.
    pub fn set(&self, name: &str, shown: bool) {
        if let Some(entry) = self.lock().iter_mut().find(|(n, _)| n == name) {
            entry.1 = shown;
        }
    }

    /// Whether a layer is shown; one never added is.
    #[must_use]
    pub fn shown(&self, name: &str) -> bool {
        self.lock()
            .iter()
            .find(|(n, _)| n == name)
            .is_none_or(|(_, s)| *s)
    }
}

/// An entity path segment from a name: lowercase letters, digits and underscores; `_` for an
/// empty name, which would leave an empty segment.
pub(crate) fn slug(name: &str) -> String {
    let s: String = name
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_lowercase()
            } else {
                '_'
            }
        })
        .collect();
    if s.is_empty() { "_".to_owned() } else { s }
}

/// [`slug`] of `name`, numbered when `used` holds it already, so names that differ only in what
/// a slug drops keep their own entity. Records the result in `used`.
pub(crate) fn unique_slug(name: &str, used: &mut BTreeSet<String>) -> String {
    let base = slug(name);
    let (mut s, mut n) = (base.clone(), 1);
    while !used.insert(s.clone()) {
        n += 1;
        s = format!("{base}_{n}");
    }
    s
}

/// An id as one entity path segment: letters, digits, `_` and `-` kept, the rest `_`, so an id
/// holding `/` or a space neither nests nor merges entities. Case is kept, as ids are shown.
pub(crate) fn path_part(id: &str) -> String {
    let s: String = id
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' {
                c
            } else {
                '_'
            }
        })
        .collect();
    if s.is_empty() { "_".to_owned() } else { s }
}

/// A grid's occupied cells (50 and over) in the layer's colour, the rest clear, so the map shows
/// through.
pub(crate) fn draw_occupied(
    rec: &RecordingStream,
    path: &str,
    msg: &Value,
    to_map: &Transform,
    (colour, lift): ([u8; 3], f32),
) {
    let [r, g, b] = colour;
    let cell = |v: i64| {
        if v >= 50 {
            [r, g, b, 190]
        } else {
            [0, 0, 0, 0]
        }
    };
    if let Some(grid) = grid(msg, cell) {
        put_grid(
            rec,
            path,
            grid.placed(to_map),
            rerun::ColorModel::RGBA,
            lift,
        );
    }
}

/// A Nav2 costmap as a gradient over the map: inflation from a faint blue to indigo as the cost
/// rises, the robot's inscribed radius in violet and the obstacles in coral, free and unknown cells
/// clear.
pub(crate) fn draw_costmap(
    rec: &RecordingStream,
    path: &str,
    msg: &Value,
    to_map: &Transform,
    lift: f32,
) {
    let shade =
        |from: i64, to: i64, v: i64| u8::try_from(from + (to - from) * v / 98).unwrap_or(u8::MAX);
    // Faint where the cost is low, so the map stays readable under a costmap.
    let cell = |v: i64| match v {
        100 => [235, 85, 95, 190],
        99 => [150, 95, 235, 120],
        1..=98 => [
            shade(60, 130, v),
            shade(140, 90, v),
            shade(250, 235, v),
            shade(10, 90, v),
        ],
        _ => [0, 0, 0, 0],
    };
    if let Some(grid) = grid(msg, cell) {
        put_grid(
            rec,
            path,
            grid.placed(to_map),
            rerun::ColorModel::RGBA,
            lift,
        );
    }
}

/// A `geometry_msgs/msg/PolygonStamped` as a closed outline on the map, placed by its frame's
/// transform, `lift` above the floor; an empty one is cleared, as a collision monitor's polygon
/// with no shape yet. The Layers tab names it by its colour, so the map stays free of labels.
pub(crate) fn draw_polygon(
    rec: &RecordingStream,
    path: &str,
    msg: &Value,
    to_map: &Transform,
    (colour, lift): ([u8; 3], f32),
) {
    let mut corners: Vec<[f32; 3]> = msg["polygon"]["points"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|p| {
            let [x, y, _] = f32s(to_map.apply([num(&p["x"]), num(&p["y"]), 0.0]));
            [x, y, lift]
        })
        .collect();
    if corners.len() < 2 {
        put(rec, path, &rerun::Clear::recursive());
        return;
    }
    let first = corners[0];
    corners.push(first);
    let [r, g, b] = colour;
    let c = rerun::Color::from_rgb(r, g, b);
    put(
        rec,
        path,
        &rerun::LineStrips3D::new([corners])
            .with_colors([c])
            .with_radii([0.02]),
    );
}

/// MoveIt's world objects, as a `moveit_msgs/msg/PlanningScene` lists them: each box, sphere and
/// cylinder translucent on the map, placed by `to_map` from its frame; hovering one names it, as
/// a label would sit on the robot. Meshes are left out; primitives gone since the last call are
/// cleared. `drawn` holds the entity of each primitive drawn. False when an object's frame has no
/// transform yet, so it was left out.
pub(crate) fn draw_scene(
    rec: &RecordingStream,
    path: &str,
    objects: &[Value],
    to_map: impl Fn(&str) -> Option<Transform>,
    drawn: &mut BTreeSet<String>,
) -> bool {
    let mut complete = true;
    let colour = json!({"r": 0.6, "g": 0.48, "b": 1.0, "a": 0.4});
    let mut now = BTreeSet::new();
    for o in objects {
        let Some(id) = o["id"].as_str() else {
            continue;
        };
        let Some(t) = o["header"]["frame_id"].as_str().and_then(&to_map) else {
            complete = false;
            continue;
        };
        let at = t.compose(&pose(&o["pose"]));
        let entity = format!("{path}/{}", slug(id));
        let poses = o["primitive_poses"]
            .as_array()
            .map_or(&[][..], Vec::as_slice);
        for (i, (p, place)) in o["primitives"]
            .as_array()
            .into_iter()
            .flatten()
            .zip(poses)
            .enumerate()
        {
            let d = |k: usize| num(&p["dimensions"][k]);
            // A solid primitive's type numbers are a marker's for the same shape.
            let size = match p["type"].as_u64() {
                Some(kind::CUBE) => [d(0), d(1), d(2)],
                Some(kind::SPHERE) => [2.0 * d(0); 3],
                Some(kind::CYLINDER) => [2.0 * d(1), 2.0 * d(1), d(0)],
                _ => continue,
            };
            let placed = at.compose(&pose(place));
            let marker = json!({
                "type": p["type"],
                "scale": {"x": size[0], "y": size[1], "z": size[2]},
                "color": colour,
            });
            let child = format!("{entity}/{i}");
            draw_marker(rec, &child, &marker, &placed);
            now.insert(child);
        }
    }
    for gone in drawn.difference(&now) {
        put_static(rec, gone, &rerun::Clear::recursive());
    }
    *drawn = now;
    complete
}

/// A `sensor_msgs/msg/LaserScan` as points in the map frame, by the scan frame's transform.
pub(crate) fn scan_points(msg: &Value, to_map: &Transform) -> Vec<[f32; 3]> {
    let (min, step) = (num(&msg["angle_min"]), num(&msg["angle_increment"]));
    let (near, far) = (num(&msg["range_min"]), num(&msg["range_max"]));
    let ranges = msg["ranges"].as_array().map_or(&[][..], Vec::as_slice);
    (0_u32..)
        .zip(ranges)
        .filter_map(|(i, r)| {
            let r = r
                .as_f64()
                .filter(|r| r.is_finite() && *r >= near && *r <= far)?;
            let a = min + step * f64::from(i);
            Some(f32s(to_map.apply([r * a.cos(), r * a.sin(), 0.0])))
        })
        .collect()
}

/// `visualization_msgs/msg/Marker` types.
mod kind {
    pub const ARROW: u64 = 0;
    pub const CUBE: u64 = 1;
    pub const SPHERE: u64 = 2;
    pub const CYLINDER: u64 = 3;
    pub const LINE_STRIP: u64 = 4;
    pub const LINE_LIST: u64 = 5;
    pub const CUBE_LIST: u64 = 6;
    pub const SPHERE_LIST: u64 = 7;
    pub const POINTS: u64 = 8;
    pub const TEXT_VIEW_FACING: u64 = 9;
    pub const TRIANGLE_LIST: u64 = 11;
    pub const DELETE: u64 = 2;
    pub const DELETE_ALL: u64 = 3;
}

fn colour(c: &Value) -> rerun::Color {
    let byte = |v: &Value| {
        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "clamped to a byte"
        )]
        let b = (num(v).clamp(0.0, 1.0) * 255.0).round() as u8;
        b
    };
    rerun::Color::from_unmultiplied_rgba(byte(&c["r"]), byte(&c["g"]), byte(&c["b"]), byte(&c["a"]))
}

fn quaternion(t: &Transform) -> rerun::Quaternion {
    rerun::Quaternion::from_xyzw(f32s(t.rotation))
}

/// Lenient where [`Transform::from_pose`] is strict: a marker may leave fields out, and a missing
/// or all-zero orientation draws as none, as RViz reads it.
pub(crate) fn pose(p: &Value) -> Transform {
    let q = &p["orientation"];
    let w = q["w"].as_f64().unwrap_or(1.0);
    let mut rotation = [num(&q["x"]), num(&q["y"]), num(&q["z"]), w];
    if rotation.iter().map(|v| v * v).sum::<f64>() < 1e-12 {
        rotation = Transform::IDENTITY.rotation;
    }
    Transform {
        translation: [
            num(&p["position"]["x"]),
            num(&p["position"]["y"]),
            num(&p["position"]["z"]),
        ],
        rotation,
    }
}

/// Draws one marker, already placed in the map frame by `to_map` (the marker's frame to the map,
/// composed with its pose). `false` when its type is not drawn.
#[expect(clippy::too_many_lines, reason = "one arm per marker type")]
fn draw_marker(rec: &RecordingStream, path: &str, m: &Value, to_map: &Transform) -> bool {
    let scale = xyz(&m["scale"]);
    let base = colour(&m["color"]);
    let points: Vec<[f32; 3]> = m["points"]
        .as_array()
        .map_or(&[][..], Vec::as_slice)
        .iter()
        .map(|p| f32s(to_map.apply([num(&p["x"]), num(&p["y"]), num(&p["z"])])))
        .collect();
    let colours: Vec<rerun::Color> = match m["colors"].as_array() {
        Some(c) if c.len() == points.len() && !c.is_empty() => c.iter().map(colour).collect(),
        _ => vec![base; points.len().max(1)],
    };
    let centre = f32s(to_map.translation);
    let half = scale.map(|v| v / 2.0);
    let solid = rerun::components::FillMode::Solid;
    match m["type"].as_u64().unwrap_or(u64::MAX) {
        kind::ARROW if points.len() == 2 => put_static(
            rec,
            path,
            &rerun::Arrows3D::from_vectors([[
                points[1][0] - points[0][0],
                points[1][1] - points[0][1],
                points[1][2] - points[0][2],
            ]])
            .with_origins([points[0]])
            .with_radii([scale[0] / 2.0])
            .with_colors([base]),
        ),
        kind::ARROW => {
            // Along the pose's x axis, scale.x long.
            let tip = f32s(to_map.apply([f64::from(scale[0]), 0.0, 0.0]));
            put_static(
                rec,
                path,
                &rerun::Arrows3D::from_vectors([[
                    tip[0] - centre[0],
                    tip[1] - centre[1],
                    tip[2] - centre[2],
                ]])
                .with_origins([centre])
                .with_radii([scale[1] / 2.0])
                .with_colors([base]),
            );
        }
        kind::CUBE => put_static(
            rec,
            path,
            &rerun::Boxes3D::from_centers_and_half_sizes([centre], [half])
                .with_quaternions([quaternion(to_map)])
                .with_fill_mode(solid)
                .with_colors([base]),
        ),
        kind::SPHERE => put_static(
            rec,
            path,
            &rerun::Ellipsoids3D::from_centers_and_half_sizes([centre], [half])
                .with_quaternions([quaternion(to_map)])
                .with_fill_mode(solid)
                .with_colors([base]),
        ),
        kind::CYLINDER => put_static(
            rec,
            path,
            &rerun::Cylinders3D::from_lengths_and_radii([scale[2]], [scale[0] / 2.0])
                .with_centers([centre])
                .with_quaternions([quaternion(to_map)])
                .with_fill_mode(solid)
                .with_colors([base]),
        ),
        kind::LINE_STRIP => put_static(
            rec,
            path,
            &rerun::LineStrips3D::new([points])
                .with_radii([scale[0] / 2.0])
                .with_colors([base]),
        ),
        kind::LINE_LIST => put_static(
            rec,
            path,
            &rerun::LineStrips3D::new(points.chunks_exact(2).map(<[[f32; 3]]>::to_vec))
                .with_radii([scale[0] / 2.0])
                .with_colors([base]),
        ),
        kind::CUBE_LIST => put_static(
            rec,
            path,
            &rerun::Boxes3D::from_centers_and_half_sizes(
                points.iter().copied(),
                std::iter::repeat_n(half, points.len()),
            )
            .with_quaternions(std::iter::repeat_n(quaternion(to_map), points.len()))
            .with_fill_mode(solid)
            .with_colors(colours),
        ),
        kind::SPHERE_LIST | kind::POINTS => put_static(
            rec,
            path,
            &rerun::Points3D::new(points)
                .with_radii([scale[0] / 2.0])
                .with_colors(colours),
        ),
        kind::TEXT_VIEW_FACING => put_static(
            rec,
            path,
            &rerun::Points3D::new([centre])
                .with_radii([0.01])
                .with_labels([m["text"].as_str().unwrap_or_default()])
                .with_show_labels(true)
                .with_colors([base]),
        ),
        kind::TRIANGLE_LIST if points.len() >= 3 => put_static(
            rec,
            path,
            &rerun::Mesh3D::new(points[..points.len() / 3 * 3].iter().copied())
                .with_vertex_colors(colours.into_iter().take(points.len() / 3 * 3)),
        ),
        _ => return false,
    }
    true
}

/// The markers a `visualization_msgs/msg/MarkerArray` topic holds now, by entity path: each
/// message adds and deletes by increments, so every one is taken as it comes, and a marker with a
/// `lifetime` ends that long after it came, as RViz shows it.
#[derive(Debug, Default)]
pub(crate) struct Markers {
    live: BTreeMap<String, (Value, Option<Instant>)>,
    /// What the viewer shows now, to clear what is gone.
    drawn: BTreeSet<String>,
}

impl Markers {
    /// Takes one message's adds and deletes under `root`, one entity per namespace and id,
    /// keeping only `namespaces` when any are listed.
    pub(crate) fn take(&mut self, root: &str, msg: &Value, namespaces: &[String], now: Instant) {
        for m in msg["markers"].as_array().map_or(&[][..], Vec::as_slice) {
            let ns = m["ns"].as_str().unwrap_or_default();
            let path = format!("{root}/{}/{}", slug(ns), m["id"].as_i64().unwrap_or(0));
            match m["action"].as_u64().unwrap_or(0) {
                kind::DELETE_ALL => self.live.clear(),
                kind::DELETE => {
                    self.live.remove(&path);
                }
                _ if !namespaces.is_empty() && !namespaces.iter().any(|n| n == ns) => {}
                _ => {
                    let lifetime =
                        num(&m["lifetime"]["sec"]) + num(&m["lifetime"]["nanosec"]) * 1e-9;
                    // None, zero or unreadable lasts until it is deleted.
                    let ends = Duration::try_from_secs_f64(lifetime)
                        .ok()
                        .filter(|d| !d.is_zero())
                        .and_then(|d| now.checked_add(d));
                    self.live.insert(path, (m.clone(), ends));
                }
            }
        }
    }

    /// Drops the markers whose lifetime has run out; whether any did.
    pub(crate) fn expire(&mut self, now: Instant) -> bool {
        let before = self.live.len();
        self.live
            .retain(|_, (_, ends)| ends.is_none_or(|e| e > now));
        self.live.len() != before
    }

    /// Draws every live marker where `to_map` places its frame, and clears what was drawn and is
    /// gone. One whose frame cannot be placed yet keeps what it showed; false when any could not.
    pub(crate) fn draw(
        &mut self,
        rec: &RecordingStream,
        to_map: &dyn Fn(&str) -> Option<Transform>,
    ) -> bool {
        let mut placed_all = true;
        let mut shown = BTreeSet::new();
        for (path, (m, _)) in &self.live {
            let Some(frame) = to_map(m["header"]["frame_id"].as_str().unwrap_or_default()) else {
                placed_all = false;
                if self.drawn.contains(path) {
                    shown.insert(path.clone());
                }
                continue;
            };
            if draw_marker(rec, path, m, &frame.compose(&pose(&m["pose"]))) {
                shown.insert(path.clone());
            }
        }
        for gone in self.drawn.difference(&shown) {
            put_static(rec, gone, &rerun::Clear::flat());
        }
        self.drawn = shown;
        placed_all
    }

    /// The viewer shows none of them any more, as after the layer was cleared.
    pub(crate) fn cleared(&mut self) {
        self.drawn.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn a_layer_never_added_is_shown_and_a_hidden_one_says_so() {
        let layers = Layers::default();
        layers.add("Scan", true);
        layers.add("Walls", false);
        layers.add("Scan", false);
        assert_eq!(
            layers.list(),
            [("Scan".to_owned(), true), ("Walls".to_owned(), false)]
        );
        layers.set("Scan", false);
        assert!(!layers.shown("Scan"));
        assert!(layers.shown("never added"));
    }

    #[test]
    fn a_scan_is_placed_by_its_frame_and_misses_are_dropped() {
        let msg = json!({"angle_min": 0.0, "angle_increment": std::f64::consts::FRAC_PI_2,
            "range_min": 0.1, "range_max": 10.0, "ranges": [1.0, "NaN", 2.0, 30.0]});
        let to_map = Transform {
            translation: [5.0, 0.0, 0.3],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        let points = scan_points(&msg, &to_map);
        assert_eq!(points.len(), 2, "{points:?}");
        assert!((points[0][0] - 6.0).abs() < 1e-5 && (points[0][2] - 0.3).abs() < 1e-5);
        assert!((points[1][0] - 3.0).abs() < 1e-5, "{points:?}");
    }

    #[test]
    fn names_become_entity_paths() {
        assert_eq!(slug("Floor plan (canopy)"), "floor_plan__canopy_");
        assert_eq!(slug(""), "_", "an empty name is no empty segment");
    }

    #[test]
    fn names_a_slug_merges_keep_their_own_entity() {
        let mut used = BTreeSet::new();
        assert_eq!(unique_slug("Local costmap", &mut used), "local_costmap");
        assert_eq!(unique_slug("Local-costmap", &mut used), "local_costmap_2");
        assert_eq!(unique_slug("local costmap", &mut used), "local_costmap_3");
    }

    #[test]
    fn an_id_is_one_path_segment() {
        assert_eq!(path_part("O12"), "O12");
        assert_eq!(path_part("a/b c"), "a_b_c");
        assert_eq!(path_part(""), "_");
    }

    #[test]
    #[expect(clippy::float_cmp, reason = "the values are set, not computed")]
    fn an_all_zero_orientation_is_the_identity() {
        let p = json!({"position": {"x": 1.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}});
        assert_eq!(pose(&p).rotation, Transform::IDENTITY.rotation);
        let turned = json!({"orientation": {"z": 1.0, "w": 0.0}});
        assert_eq!(pose(&turned).rotation, [0.0, 0.0, 1.0, 0.0]);
    }

    #[test]
    fn markers_are_kept_by_namespace_and_cleared_by_deleteall() {
        let rec = rerun::RecordingStreamBuilder::new("test")
            .buffered()
            .unwrap();
        let identity = |_: &str| Some(Transform::IDENTITY);
        let marker = |ns: &str, id: i64, kind: u64| {
            json!({"header": {"frame_id": "map"}, "ns": ns, "id": id, "type": kind, "action": 0,
                "pose": {"position": {"x": 1.0, "y": 2.0, "z": 0.0}, "orientation": {"w": 1.0}},
                "scale": {"x": 0.3, "y": 0.3, "z": 0.04}, "color": {"r": 1.0, "g": 0.5, "b": 0.0, "a": 1.0},
                "points": [], "colors": [], "text": "a"})
        };
        let now = Instant::now();
        let mut markers = Markers::default();
        let msg = json!({"markers": [marker("viewpoint", 0, kind::CYLINDER), marker("room_areas", 1, kind::CUBE_LIST), marker("names", 2, kind::TEXT_VIEW_FACING)]});
        let wanted = ["viewpoint".to_owned(), "names".to_owned()];
        markers.take("world/layers/m", &msg, &wanted, now);
        assert!(markers.draw(&rec, &identity));
        assert_eq!(
            markers.drawn.iter().map(String::as_str).collect::<Vec<_>>(),
            ["world/layers/m/names/2", "world/layers/m/viewpoint/0"]
        );
        let again = json!({"markers": [{"action": kind::DELETE_ALL}, marker("viewpoint", 5, kind::SPHERE)]});
        markers.take("world/layers/m", &again, &wanted, now);
        assert!(markers.draw(&rec, &identity));
        assert_eq!(
            markers.drawn.iter().map(String::as_str).collect::<Vec<_>>(),
            ["world/layers/m/viewpoint/5"]
        );
        let elsewhere = |_: &str| None;
        markers.take("world/layers/m", &msg, &wanted, now);
        assert!(!markers.draw(&rec, &elsewhere));
        assert_eq!(
            markers.drawn.len(),
            1,
            "markers in a frame with no transform are not drawn, and the drawn one stays"
        );
    }

    #[test]
    fn markers_sent_apart_are_all_kept_and_a_lifetime_ends_one() {
        let rec = rerun::RecordingStreamBuilder::new("test")
            .buffered()
            .unwrap();
        let identity = |_: &str| Some(Transform::IDENTITY);
        let marker = |id: i64, lifetime_s: u64| {
            json!({"markers": [{"header": {"frame_id": "map"}, "id": id, "type": kind::SPHERE,
                "scale": {"x": 0.1, "y": 0.1, "z": 0.1}, "color": {"r": 1.0, "a": 1.0},
                "lifetime": {"sec": lifetime_s, "nanosec": 0}}]})
        };
        let now = Instant::now();
        let mut markers = Markers::default();
        markers.take("m", &marker(1, 0), &[], now);
        markers.take("m", &marker(2, 1), &[], now);
        assert!(markers.draw(&rec, &identity));
        assert_eq!(
            markers.drawn.len(),
            2,
            "the first message is not lost to the second"
        );
        assert!(!markers.expire(now + Duration::from_millis(500)));
        assert!(markers.expire(now + Duration::from_secs(2)));
        assert!(markers.draw(&rec, &identity));
        assert_eq!(
            markers.drawn.iter().map(String::as_str).collect::<Vec<_>>(),
            ["m/_/1"],
            "the one with a lifetime of 1 s is gone; the other lasts"
        );
        markers.cleared();
        assert!(markers.draw(&rec, &identity));
        assert_eq!(markers.drawn.len(), 1, "shown again after a hide");
    }

    #[test]
    fn a_marker_without_a_namespace_has_no_empty_path_segment() {
        let rec = rerun::RecordingStreamBuilder::new("test")
            .buffered()
            .unwrap();
        let identity = |_: &str| Some(Transform::IDENTITY);
        let msg = json!({"markers": [{"header": {"frame_id": "map"}, "id": 3, "type": kind::SPHERE,
            "scale": {"x": 0.1, "y": 0.1, "z": 0.1}, "color": {"r": 1.0, "a": 1.0}}]});
        let mut markers = Markers::default();
        markers.take("world/layers/m", &msg, &[], Instant::now());
        assert!(markers.draw(&rec, &identity));
        assert_eq!(
            markers.drawn.iter().map(String::as_str).collect::<Vec<_>>(),
            ["world/layers/m/_/3"]
        );
    }

    #[test]
    fn scene_solids_are_drawn_and_cleared_once_gone() {
        let rec = rerun::RecordingStreamBuilder::new("test")
            .buffered()
            .unwrap();
        let identity = |_: &str| Some(Transform::IDENTITY);
        let object = |id: &str, kind: u64| {
            json!({"header": {"frame_id": "pelvis"}, "id": id,
                "primitives": [{"type": kind, "dimensions": [0.3, 0.3, 0.3]}],
                "primitive_poses": [{"position": {"x": 1.0, "y": 1.0, "z": 0.8}}]})
        };
        let mut drawn = BTreeSet::new();
        let both = [object("Box 1", kind::CUBE), object("mesh", 7)];
        assert!(draw_scene(
            &rec,
            "world/arm_scene",
            &both,
            identity,
            &mut drawn
        ));
        assert_eq!(
            drawn.iter().map(String::as_str).collect::<Vec<_>>(),
            ["world/arm_scene/box_1/0"],
            "a mesh, or any shape not a box, sphere or cylinder, is left out"
        );
        let mut two = object("Box 1", kind::CUBE);
        let cube = json!({"type": kind::CUBE, "dimensions": [0.3, 0.3, 0.3]});
        two["primitives"] = json!([cube.clone(), cube]);
        two["primitive_poses"] = json!([{}, {}]);
        draw_scene(&rec, "world/arm_scene", &[two], identity, &mut drawn);
        assert_eq!(drawn.len(), 2);
        draw_scene(
            &rec,
            "world/arm_scene",
            &[object("Box 1", kind::CUBE)],
            identity,
            &mut drawn,
        );
        assert_eq!(drawn.len(), 1, "a primitive the object lost is cleared");
        let nowhere = |_: &str| None;
        assert!(!draw_scene(
            &rec,
            "world/arm_scene",
            &both,
            nowhere,
            &mut drawn
        ));
        assert!(drawn.is_empty());
    }
}
