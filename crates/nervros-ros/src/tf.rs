//! A small TF buffer: the newest transform per frame pair, chained through a common ancestor.
//!
//! ponytail: newest-only, no interpolation in time. Enough for "where is the robot" and for placing
//! detections; add time-indexed buffers if a caller needs a transform at a past stamp.

use std::collections::HashMap;

use crate::RosError;

/// A rigid transform: rotation (unit quaternion x, y, z, w) then translation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    /// Translation in metres.
    pub translation: [f64; 3],
    /// Rotation as a unit quaternion `[x, y, z, w]`.
    pub rotation: [f64; 4],
}

impl Default for Transform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Transform {
    /// No rotation, no translation.
    pub const IDENTITY: Self = Self {
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };

    /// Applies the transform to a point.
    #[must_use]
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let r = rotate(self.rotation, p);
        [
            r[0] + self.translation[0],
            r[1] + self.translation[1],
            r[2] + self.translation[2],
        ]
    }

    /// `self * other`: first `other`, then `self`.
    #[must_use]
    pub fn compose(&self, other: &Self) -> Self {
        Self {
            translation: self.apply(other.translation),
            rotation: mul(self.rotation, other.rotation),
        }
    }

    /// The inverse transform.
    #[must_use]
    pub fn inverse(&self) -> Self {
        let q = self.rotation;
        let conj = [-q[0], -q[1], -q[2], q[3]];
        let back = rotate(conj, self.translation);
        Self {
            translation: [-back[0], -back[1], -back[2]],
            rotation: conj,
        }
    }

    /// Heading about +z in radians, for a planar robot pose.
    #[must_use]
    pub fn yaw(&self) -> f64 {
        let [x, y, z, w] = self.rotation;
        (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
    }
}

fn mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let p = mul(mul(q, [v[0], v[1], v[2], 0.0]), [-q[0], -q[1], -q[2], q[3]]);
    [p[0], p[1], p[2]]
}

/// The newest transform of each child frame relative to its parent.
#[derive(Debug, Default)]
pub struct TfBuffer {
    parents: HashMap<String, (String, Transform)>,
}

/// TF allows at most this many hops; a longer walk means a loop in bad data.
const MAX_DEPTH: usize = 64;

impl TfBuffer {
    /// Records `child`'s pose in `parent`'s frame (a `geometry_msgs/TransformStamped`).
    pub fn insert(&mut self, parent: &str, child: &str, transform: Transform) {
        let strip = |f: &str| f.trim_start_matches('/').to_owned();
        self.parents
            .insert(strip(child), (strip(parent), transform));
    }

    /// Frames from `frame` up to its root, each with the transform root <- that frame.
    fn chain(&self, frame: &str) -> Vec<(String, Transform)> {
        let mut out = vec![(frame.to_owned(), Transform::IDENTITY)];
        let mut current = frame.to_owned();
        let mut up = Transform::IDENTITY;
        while let Some((parent, t)) = self.parents.get(&current) {
            if out.len() > MAX_DEPTH {
                break;
            }
            up = t.compose(&up);
            current.clone_from(parent);
            out.push((current.clone(), up));
        }
        out
    }

    /// The transform that maps points in `source` into `target`.
    ///
    /// # Errors
    ///
    /// [`RosError::NoTransform`] when the frames share no ancestor.
    pub fn lookup(&self, target: &str, source: &str) -> Result<Transform, RosError> {
        let target = target.trim_start_matches('/');
        let source = source.trim_start_matches('/');
        let from_source = self.chain(source);
        let from_target = self.chain(target);
        for (ancestor, ancestor_from_source) in &from_source {
            if let Some((_, ancestor_from_target)) = from_target.iter().find(|(f, _)| f == ancestor)
            {
                return Ok(ancestor_from_target.inverse().compose(ancestor_from_source));
            }
        }
        Err(RosError::NoTransform {
            target: target.to_owned(),
            source_frame: source.to_owned(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn yaw_quat(yaw: f64) -> [f64; 4] {
        [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()]
    }

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9)
    }

    #[test]
    fn chains_through_a_common_ancestor() {
        let mut tf = TfBuffer::default();
        // map -> odom: shifted 1 m in x; odom -> base: 2 m in y, turned 90 degrees.
        tf.insert(
            "map",
            "odom",
            Transform {
                translation: [1.0, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        );
        tf.insert(
            "odom",
            "base_footprint",
            Transform {
                translation: [0.0, 2.0, 0.0],
                rotation: yaw_quat(std::f64::consts::FRAC_PI_2),
            },
        );
        tf.insert(
            "base_footprint",
            "camera",
            Transform {
                translation: [0.5, 0.0, 1.2],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        );
        let base = tf.lookup("map", "base_footprint").unwrap();
        assert!(close(base.translation, [1.0, 2.0, 0.0]));
        assert!((base.yaw() - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        // A point 0.5 m ahead of the camera, in the map: the robot faces +y.
        let p = tf.lookup("map", "camera").unwrap().apply([0.5, 0.0, 0.0]);
        assert!(close(p, [1.0, 3.0, 1.2]));
        // And back again.
        let back = tf.lookup("camera", "map").unwrap().apply(p);
        assert!(close(back, [0.5, 0.0, 0.0]));
    }

    #[test]
    fn unrelated_frames_have_no_transform() {
        let mut tf = TfBuffer::default();
        tf.insert("a", "b", Transform::IDENTITY);
        tf.insert("c", "d", Transform::IDENTITY);
        assert!(matches!(
            tf.lookup("a", "d"),
            Err(RosError::NoTransform { .. })
        ));
    }

    #[test]
    fn inverse_undoes_compose() {
        let t = Transform {
            translation: [1.0, -2.0, 0.5],
            rotation: yaw_quat(0.7),
        };
        let id = t.compose(&t.inverse());
        assert!(close(id.translation, [0.0; 3]));
        assert!(close(id.apply([3.0, 4.0, 5.0]), [3.0, 4.0, 5.0]));
    }
}
