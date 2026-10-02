//! Topics drawn as RViz draws them: occupancy grids, marker arrays and laser scans, from the
//! profile's `[[viz.layer]]` list, and the switches the operator hides any layer with.

use std::collections::BTreeSet;
use std::sync::{Mutex, MutexGuard, PoisonError};

use nervros_ros::Transform;
use rerun::RecordingStream;
use serde_json::Value;

use crate::grid::{grid, put_grid};
use crate::{f32s, num, put_static, xyz};

/// Every layer the viewer draws, in order, each shown or hidden by the operator.
#[derive(Debug, Default)]
pub struct Layers(Mutex<Vec<(String, bool)>>);

impl Layers {
    fn lock(&self) -> MutexGuard<'_, Vec<(String, bool)>> {
        self.0.lock().unwrap_or_else(PoisonError::into_inner)
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

/// An entity path segment from a name: lowercase letters, digits and underscores.
pub(crate) fn slug(name: &str) -> String {
    name.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_lowercase()
            } else {
                '_'
            }
        })
        .collect()
}

/// A grid's occupied cells (50 and over) in the layer's colour, the rest clear, so the map shows
/// through.
pub(crate) fn draw_occupied(
    rec: &RecordingStream,
    path: &str,
    msg: &Value,
    colour: [u8; 3],
    lift: f32,
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
        put_grid(rec, path, grid, rerun::ColorModel::RGBA, lift, false);
    }
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
/// orientation draws as none.
pub(crate) fn pose(p: &Value) -> Transform {
    let q = &p["orientation"];
    let w = q["w"].as_f64().unwrap_or(1.0);
    Transform {
        translation: [
            num(&p["position"]["x"]),
            num(&p["position"]["y"]),
            num(&p["position"]["z"]),
        ],
        rotation: [num(&q["x"]), num(&q["y"]), num(&q["z"]), w],
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

/// Draws a `visualization_msgs/msg/MarkerArray` under `root`, one entity per namespace and id,
/// keeping only `namespaces` when any are listed. `to_map` places a frame in the map frame, or
/// says it cannot. Markers deleted, or dropped by a `DELETEALL`, are cleared.
pub(crate) fn draw_markers(
    rec: &RecordingStream,
    root: &str,
    msg: &Value,
    namespaces: &[String],
    to_map: &dyn Fn(&str) -> Option<Transform>,
    drawn: &mut BTreeSet<String>,
) {
    let mut kept = drawn.clone();
    for m in msg["markers"].as_array().map_or(&[][..], Vec::as_slice) {
        let ns = m["ns"].as_str().unwrap_or_default();
        let path = format!("{root}/{}/{}", slug(ns), m["id"].as_i64().unwrap_or(0));
        match m["action"].as_u64().unwrap_or(0) {
            kind::DELETE_ALL => {
                for gone in std::mem::take(&mut kept) {
                    put_static(rec, &gone, &rerun::Clear::flat());
                }
                continue;
            }
            kind::DELETE => {
                if kept.remove(&path) {
                    put_static(rec, &path, &rerun::Clear::flat());
                }
                continue;
            }
            _ => {}
        }
        if !namespaces.is_empty() && !namespaces.iter().any(|n| n == ns) {
            continue;
        }
        let Some(frame) = to_map(m["header"]["frame_id"].as_str().unwrap_or_default()) else {
            continue;
        };
        if draw_marker(rec, &path, m, &frame.compose(&pose(&m["pose"]))) {
            kept.insert(path);
        }
    }
    *drawn = kept;
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
        let mut drawn = BTreeSet::new();
        let msg = json!({"markers": [marker("viewpoint", 0, kind::CYLINDER), marker("room_areas", 1, kind::CUBE_LIST), marker("names", 2, kind::TEXT_VIEW_FACING)]});
        let wanted = ["viewpoint".to_owned(), "names".to_owned()];
        draw_markers(&rec, "world/layers/m", &msg, &wanted, &identity, &mut drawn);
        assert_eq!(
            drawn.iter().map(String::as_str).collect::<Vec<_>>(),
            ["world/layers/m/names/2", "world/layers/m/viewpoint/0"]
        );
        let again = json!({"markers": [{"action": kind::DELETE_ALL}, marker("viewpoint", 5, kind::SPHERE)]});
        draw_markers(
            &rec,
            "world/layers/m",
            &again,
            &wanted,
            &identity,
            &mut drawn,
        );
        assert_eq!(
            drawn.iter().map(String::as_str).collect::<Vec<_>>(),
            ["world/layers/m/viewpoint/5"]
        );
        let elsewhere = |_: &str| None;
        draw_markers(
            &rec,
            "world/layers/m",
            &msg,
            &wanted,
            &elsewhere,
            &mut drawn,
        );
        assert_eq!(
            drawn.len(),
            1,
            "markers in a frame with no transform are not drawn"
        );
    }
}
