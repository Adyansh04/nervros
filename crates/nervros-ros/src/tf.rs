//! A small TF buffer: the newest transform per frame pair, chained through a common ancestor.
//!
//! ponytail: newest-only, no interpolation in time. Enough for "where is the robot" and for placing
//! detections; add time-indexed buffers if a caller needs a transform at a past stamp.

use std::collections::{HashMap, HashSet};
use std::time::{Duration, Instant};

use crate::{RosError, TfLink};

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

    /// `(x, y, yaw)`: where a robot on the floor is and which way it faces.
    #[must_use]
    pub fn planar(&self) -> (f64, f64, f64) {
        (self.translation[0], self.translation[1], self.yaw())
    }

    /// `translation`, turned `yaw` radians about +z.
    #[must_use]
    pub fn from_yaw(translation: [f64; 3], yaw: f64) -> Self {
        let half = yaw / 2.0;
        Self {
            translation,
            rotation: [0.0, 0.0, half.sin(), half.cos()],
        }
    }

    /// As a `geometry_msgs/Pose` in JSON.
    #[must_use]
    pub fn to_pose(&self) -> serde_json::Value {
        let [x, y, z] = self.translation;
        let [qx, qy, qz, qw] = self.rotation;
        serde_json::json!({
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        })
    }

    /// A `geometry_msgs/Pose` as JSON; `None` when x, y or the rotation is missing. A missing z
    /// is 0: plans and goals often leave it out.
    #[must_use]
    pub fn from_pose(pose: &serde_json::Value) -> Option<Self> {
        let (p, q) = (&pose["position"], &pose["orientation"]);
        let n = |v: &serde_json::Value| v.as_f64();
        Some(Self {
            translation: [n(&p["x"])?, n(&p["y"])?, n(&p["z"]).unwrap_or(0.0)],
            rotation: [n(&q["x"])?, n(&q["y"])?, n(&q["z"])?, n(&q["w"])?],
        })
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

/// One child's link: its parent, its pose there, when it last arrived and whether it came from
/// `/tf_static`.
#[derive(Debug, Clone)]
struct Link {
    parent: String,
    transform: Transform,
    at: Instant,
    is_static: bool,
}

/// The newest transform of each child frame relative to its parent.
#[derive(Debug, Default)]
pub struct TfBuffer {
    links: HashMap<String, Link>,
    /// Children whose latest transform was refused, so the warning comes once per fault.
    refused: HashSet<String>,
}

/// A transform's rotation as a unit quaternion: a near-unit one is normalised, and one that is not
/// finite, or far from unit length, is refused. A localiser that diverged can publish NaN, and a
/// NaN pose reads as outside every keep-out zone.
fn usable(transform: Transform) -> Option<Transform> {
    let finite = transform
        .translation
        .iter()
        .chain(&transform.rotation)
        .all(|v| v.is_finite());
    let norm = transform.rotation.iter().map(|v| v * v).sum::<f64>().sqrt();
    if !finite || (norm - 1.0).abs() > 0.1 {
        return None;
    }
    Some(Transform {
        translation: transform.translation,
        rotation: transform.rotation.map(|v| v / norm),
    })
}

/// The oldest `/tf` link a transform rests on: its child frame and its age.
type Oldest = Option<(String, Duration)>;

/// TF allows at most this many hops; a longer walk means a loop in bad data.
const MAX_DEPTH: usize = 64;
/// How long a child's parent holds against another publisher's: one that went quiet this long
/// ago gives way.
const PARENT_HOLD: Duration = Duration::from_secs(1);

impl TfBuffer {
    /// Records `child`'s pose in `parent`'s frame (a `geometry_msgs/TransformStamped`).
    pub fn insert(&mut self, parent: &str, child: &str, transform: Transform) {
        self.insert_from(parent, child, transform, false);
    }

    /// As [`Self::insert`], noting whether the link came from `/tf_static`.
    pub fn insert_from(
        &mut self,
        parent: &str,
        child: &str,
        transform: Transform,
        is_static: bool,
    ) {
        let strip = |f: &str| f.trim_start_matches('/').to_owned();
        let child = strip(child);
        let Some(transform) = usable(transform) else {
            if self.refused.insert(child.clone()) {
                tracing::warn!(parent, %child, "refused a transform that is not finite or not a rotation; the last good one stays");
            }
            return;
        };
        let parent = strip(parent);
        if let Some(kept) = self.links.get(&child) {
            // A mount on /tf_static stays: a stray /tf publisher does not move it.
            let over_static = kept.is_static && !is_static;
            // Two publishers giving one child two parents would flip the tree with each message:
            // the one heard last a moment ago keeps it until it goes quiet.
            let other_parent = kept.parent != parent && kept.at.elapsed() < PARENT_HOLD;
            if over_static || other_parent {
                if self.refused.insert(child.clone()) {
                    tracing::warn!(%parent, %child, kept = %kept.parent, "refused a transform that would replace a static or another parent's link");
                }
                return;
            }
        }
        self.refused.remove(&child);
        self.links.insert(
            child,
            Link {
                parent,
                transform,
                at: Instant::now(),
                is_static,
            },
        );
    }

    /// Whether any link names `frame`, as its child or its parent.
    fn knows(&self, frame: &str) -> bool {
        self.links.contains_key(frame) || self.links.values().any(|l| l.parent == frame)
    }

    /// Every link, parent first, sorted by child.
    #[must_use]
    pub fn links(&self) -> Vec<TfLink> {
        let mut out: Vec<TfLink> = self
            .links
            .iter()
            .map(|(child, link)| TfLink {
                parent: link.parent.clone(),
                child: child.clone(),
                is_static: link.is_static,
                age: link.at.elapsed(),
            })
            .collect();
        out.sort_by(|a, b| a.child.cmp(&b.child));
        out
    }

    /// Frames from `frame` up to its root, each with the transform root <- that frame and the
    /// oldest `/tf` link on the way there (its child and its age).
    /// `None` for a loop, which only bad data makes.
    fn chain(&self, frame: &str) -> Option<Vec<(String, Transform, Oldest)>> {
        let mut out = vec![(frame.to_owned(), Transform::IDENTITY, None)];
        let mut current = frame.to_owned();
        let mut up = Transform::IDENTITY;
        let mut oldest: Oldest = None;
        while let Some(link) = self.links.get(&current) {
            if out.len() > MAX_DEPTH || out.iter().any(|(f, _, _)| *f == link.parent) {
                return None;
            }
            let age = link.at.elapsed();
            if !link.is_static && oldest.as_ref().is_none_or(|(_, a)| age > *a) {
                oldest = Some((current.clone(), age));
            }
            up = link.transform.compose(&up);
            current.clone_from(&link.parent);
            out.push((current.clone(), up, oldest.clone()));
        }
        Some(out)
    }

    /// The transform that maps points in `source` into `target`, and the oldest `/tf` link it
    /// rests on.
    fn resolve(&self, target: &str, source: &str) -> Result<(Transform, Oldest), RosError> {
        let target = target.trim_start_matches('/');
        let source = source.trim_start_matches('/');
        let none = || RosError::NoTransform {
            target: target.to_owned(),
            source_frame: source.to_owned(),
        };
        // A frame is its own transform only when it is a frame at all.
        if !self.knows(source) || !self.knows(target) {
            return Err(none());
        }
        let (Some(from_source), Some(from_target)) = (self.chain(source), self.chain(target))
        else {
            return Err(none());
        };
        for (ancestor, ancestor_from_source, source_oldest) in &from_source {
            if let Some((_, ancestor_from_target, target_oldest)) =
                from_target.iter().find(|(f, _, _)| f == ancestor)
            {
                let oldest = match (source_oldest, target_oldest) {
                    (Some(a), Some(b)) => Some(if a.1 >= b.1 { a } else { b }.clone()),
                    (a, b) => a.clone().or_else(|| b.clone()),
                };
                return Ok((
                    ancestor_from_target.inverse().compose(ancestor_from_source),
                    oldest,
                ));
            }
        }
        Err(none())
    }

    /// The transform that maps points in `source` into `target`.
    ///
    /// # Errors
    ///
    /// [`RosError::NoTransform`] when the frames share no ancestor.
    pub fn lookup(&self, target: &str, source: &str) -> Result<Transform, RosError> {
        self.resolve(target, source).map(|(transform, _)| transform)
    }

    /// As [`Self::lookup`], refusing a chain with a `/tf` link older than `max_age`: a localiser
    /// or an odometry that stopped publishing leaves its last link standing, which would read as
    /// a robot that stands still where it last was.
    ///
    /// # Errors
    ///
    /// [`RosError::NoTransform`] when the frames share no ancestor, [`RosError::NoData`] when a
    /// link on the way is stale.
    pub fn lookup_within(
        &self,
        target: &str,
        source: &str,
        max_age: Duration,
    ) -> Result<Transform, RosError> {
        let (transform, oldest) = self.resolve(target, source)?;
        match oldest {
            Some((child, age)) if age > max_age => Err(RosError::NoData(format!(
                "TF from {source} to {target}: {child} was last published {:.1} s ago",
                age.as_secs_f64()
            ))),
            _ => Ok(transform),
        }
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
    fn a_transform_that_is_not_finite_is_refused_and_the_last_good_one_stays() {
        let mut tf = TfBuffer::default();
        let good = Transform {
            translation: [1.0, 2.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        tf.insert("map", "base", good);
        tf.insert(
            "map",
            "base",
            Transform {
                translation: [f64::NAN, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        );
        tf.insert(
            "map",
            "base",
            Transform {
                translation: [5.0, 5.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 0.0],
            },
        );
        let t = tf.lookup("map", "base").unwrap();
        assert!(close(t.translation, [1.0, 2.0, 0.0]), "{t:?}");
    }

    #[test]
    fn a_near_unit_rotation_is_normalised() {
        let mut tf = TfBuffer::default();
        tf.insert(
            "map",
            "base",
            Transform {
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.05],
            },
        );
        let t = tf.lookup("map", "base").unwrap();
        assert!((t.rotation[3] - 1.0).abs() < 1e-12, "{t:?}");
    }

    #[test]
    fn a_stale_dynamic_link_is_refused_but_a_static_one_never_ages() {
        let mut tf = TfBuffer::default();
        let t = Transform {
            translation: [1.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        tf.insert_from("base", "camera", t, true);
        tf.insert("map", "base", t);
        std::thread::sleep(Duration::from_millis(5));
        assert!(tf.lookup_within("base", "camera", Duration::ZERO).is_ok());
        let err = tf
            .lookup_within("map", "camera", Duration::ZERO)
            .unwrap_err();
        assert!(
            matches!(err, RosError::NoData(ref m) if m.contains("base")),
            "{err}"
        );
        assert!(
            tf.lookup_within("map", "camera", Duration::from_secs(5))
                .is_ok()
        );
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
    fn a_frame_nobody_publishes_is_no_frame_and_a_loop_no_transform() {
        let mut tf = TfBuffer::default();
        assert!(tf.lookup("ghost", "ghost").is_err());
        tf.insert("a", "b", Transform::IDENTITY);
        assert!(tf.lookup("b", "b").is_ok());
        tf.insert("b", "c", Transform::IDENTITY);
        tf.insert_from("c", "a", Transform::IDENTITY, true);
        assert!(matches!(
            tf.lookup("a", "c"),
            Err(RosError::NoTransform { .. })
        ));
    }

    #[test]
    fn a_static_mount_and_a_parent_heard_lately_hold() {
        let mut tf = TfBuffer::default();
        let moved = Transform {
            translation: [1.0, 0.0, 0.0],
            ..Transform::IDENTITY
        };
        tf.insert_from("base", "camera", Transform::IDENTITY, true);
        tf.insert("base", "camera", moved);
        assert!(close(
            tf.lookup("base", "camera").unwrap().apply([0.0; 3]),
            [0.0; 3]
        ));
        tf.insert("odom", "base", Transform::IDENTITY);
        tf.insert("other", "base", moved);
        assert_eq!(
            tf.links()
                .iter()
                .find(|l| l.child == "base")
                .unwrap()
                .parent,
            "odom"
        );
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
