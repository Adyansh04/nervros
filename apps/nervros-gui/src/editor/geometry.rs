//! Shapes on the floor: boxes and outlines, reshaping them by a handle, and which way a wall faces.

use std::collections::HashMap;

use super::world::{Object, Room};
use super::{Grip, MIN_SIDE_M};

/// A box on the floor: centre and yaw in the map frame, size along its own axes, all in metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Shape2 {
    pub(super) centre: [f64; 2],
    pub(super) size: [f64; 2],
    pub(super) yaw: f64,
}

impl Shape2 {
    pub(super) fn of(o: &Object) -> Self {
        Self {
            centre: o.centre,
            size: o.size,
            yaw: o.yaw,
        }
    }

    /// A map point in the box's own axes.
    pub(super) fn to_box(self, [x, y]: [f64; 2]) -> [f64; 2] {
        let (s, c) = self.yaw.sin_cos();
        let (dx, dy) = (x - self.centre[0], y - self.centre[1]);
        [c * dx + s * dy, -s * dx + c * dy]
    }

    /// A point in the box's axes on the map.
    pub(super) fn to_map(self, [u, v]: [f64; 2]) -> [f64; 2] {
        let (s, c) = self.yaw.sin_cos();
        [
            self.centre[0] + c * u - s * v,
            self.centre[1] + s * u + c * v,
        ]
    }

    /// Corners in the web page's order: (+,+), (-,+), (-,-), (+,-).
    pub(super) fn corners(&self) -> [[f64; 2]; 4] {
        let [hu, hv] = [self.size[0] / 2.0, self.size[1] / 2.0];
        [[hu, hv], [-hu, hv], [-hu, -hv], [hu, -hv]].map(|p| self.to_map(p))
    }

    pub(super) fn contains(&self, p: [f64; 2]) -> bool {
        let [u, v] = self.to_box(p);
        u.abs() <= self.size[0] / 2.0 && v.abs() <= self.size[1] / 2.0
    }
}

/// A box moved, turned or resized by a grip dragged from `from` to `at`, as the web page does it.
pub(super) fn reshape(origin: Shape2, grip: Grip, from: [f64; 2], at: [f64; 2]) -> Shape2 {
    let mut shape = origin;
    match grip {
        Grip::Move => {
            shape.centre = [
                origin.centre[0] + at[0] - from[0],
                origin.centre[1] + at[1] - from[1],
            ];
        }
        Grip::Rotate => {
            shape.yaw = (at[1] - origin.centre[1]).atan2(at[0] - origin.centre[0])
                - std::f64::consts::FRAC_PI_2;
        }
        Grip::Corner(i) => {
            // The opposite corner stays where it is.
            let signs = [[1.0, 1.0], [-1.0, 1.0], [-1.0, -1.0], [1.0, -1.0]][i];
            let anchor = Shape2 {
                centre: origin.to_map([
                    -signs[0] * origin.size[0] / 2.0,
                    -signs[1] * origin.size[1] / 2.0,
                ]),
                size: [0.0, 0.0],
                yaw: origin.yaw,
            };
            let [u, v] = anchor.to_box(at);
            shape.size = [u.abs().max(MIN_SIDE_M), v.abs().max(MIN_SIDE_M)];
            shape.centre = anchor.to_map([u / 2.0, v / 2.0]);
        }
    }
    shape
}

pub(super) fn in_polygon(polygon: &[[f64; 2]], [x, y]: [f64; 2]) -> bool {
    let points: Vec<(f64, f64)> = polygon.iter().map(|&[a, b]| (a, b)).collect();
    nervros_core::builtins::inside((x, y), &points)
}

/// The walls' angle, which canopy lays boxes along: the most common yaw, a quarter turn apart.
pub(super) fn wall_yaw(objects: &[Object]) -> f64 {
    let quarter = std::f64::consts::FRAC_PI_2;
    let mut counts: HashMap<i64, usize> = HashMap::new();
    for o in objects {
        #[expect(
            clippy::cast_possible_truncation,
            reason = "a bucket of a quarter turn"
        )]
        let bucket = (o.yaw.rem_euclid(quarter) * 50.0).round() as i64;
        *counts.entry(bucket).or_default() += 1;
    }
    #[expect(clippy::cast_precision_loss, reason = "a small bucket number")]
    let best = counts
        .into_iter()
        .max_by_key(|&(_, n)| n)
        .map_or(0.0, |(b, _)| b as f64 / 50.0);
    best
}

pub(super) fn room_centre(room: &Room) -> [f64; 2] {
    if room.outline.is_empty() {
        return [room.x, room.y];
    }
    #[expect(clippy::cast_precision_loss, reason = "a few hundred corners at most")]
    let n = room.outline.len() as f64;
    let (sx, sy) = room
        .outline
        .iter()
        .fold((0.0, 0.0), |(x, y), p| (x + p[0], y + p[1]));
    [sx / n, sy / n]
}
