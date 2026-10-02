//! Snapshots: images a tool showed, kept so the model and the user can refer to them.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use super::detect::Instance;
use crate::tools::ImageArtifact;

/// Snapshots kept for mark references.
const SNAPSHOTS_KEPT: usize = 20;

/// A stored look: what was seen, as marks.
#[derive(Debug, Clone)]
pub struct Snapshot {
    /// `s1`, `s2`, ...
    pub id: String,
    /// The frame's stamp.
    pub stamp_s: f64,
    /// Marks in order; mark n is `marks[n - 1]`.
    pub marks: Vec<Instance>,
    /// The marked image.
    pub image: ImageArtifact,
}

/// The last few snapshots.
#[derive(Debug, Default)]
pub struct SnapshotStore {
    next: AtomicU64,
    kept: Mutex<VecDeque<Arc<Snapshot>>>,
}

impl SnapshotStore {
    /// A fresh id.
    pub fn next_id(&self) -> String {
        format!("s{}", self.next.fetch_add(1, Ordering::Relaxed) + 1)
    }

    /// Keeps a JPEG and the marks drawn on it as a new snapshot; its `image` is what a tool
    /// hands the window.
    pub fn store(
        &self,
        jpeg: Vec<u8>,
        (width, height): (u32, u32),
        stamp_s: f64,
        marks: Vec<Instance>,
    ) -> Arc<Snapshot> {
        let id = self.next_id();
        self.put(Snapshot {
            image: ImageArtifact {
                snapshot: id.clone(),
                jpeg: Arc::new(jpeg),
                width,
                height,
                marks: marks.iter().map(|m| m.label.clone()).collect(),
            },
            id,
            stamp_s,
            marks,
        })
    }

    /// Stores a snapshot, dropping the oldest past the limit.
    fn put(&self, snapshot: Snapshot) -> Arc<Snapshot> {
        let snapshot = Arc::new(snapshot);
        let mut kept = crate::lock(&self.kept);
        kept.push_back(Arc::clone(&snapshot));
        while kept.len() > SNAPSHOTS_KEPT {
            kept.pop_front();
        }
        snapshot
    }

    /// A snapshot by id, if still kept.
    #[must_use]
    pub fn get(&self, id: &str) -> Option<Arc<Snapshot>> {
        crate::lock(&self.kept).iter().find(|s| s.id == id).cloned()
    }
}
