//! The camera toolkit `look`, `point` and `segment` share: each camera's recent frames, the
//! detections drawn on them as numbered marks, and the snapshots the model and the user refer to.

mod detect;
mod frames;
mod marks;
mod snapshot;

pub(crate) use detect::DETECTION_MATCH_S;
pub use detect::{Detections, Instance, parse_detections};
pub use frames::{Camera, Cameras};
#[cfg(test)]
pub(crate) use marks::CLOSE_UP_PX;
pub use marks::draw_marks;
pub(crate) use marks::{PALETTE, badge, close_up, marks_json, tint};
pub use snapshot::{Snapshot, SnapshotStore};
