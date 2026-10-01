//! The session log: every event as one JSON line, images as files beside it.
//!
//! `<dir>/<session>.ndjson` holds `{seq, ts, event}` lines; a snapshot's JPEG goes to
//! `<dir>/<session>/<snapshot>.jpg`. Nothing in the log carries a key: providers are named by id.

use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use serde_json::json;
use tokio::sync::broadcast;
use tokio::task::JoinHandle;

use crate::session::Event;

/// Writes the log until the event stream closes. Errors are logged, never fatal.
///
/// # Errors
///
/// The log directory or file cannot be created.
pub fn spawn(
    dir: &Path,
    session: &str,
    mut events: broadcast::Receiver<Event>,
) -> std::io::Result<(PathBuf, JoinHandle<()>)> {
    let blobs = dir.join(session);
    std::fs::create_dir_all(&blobs)?;
    let path = dir.join(format!("{session}.ndjson"));
    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)?;
    let handle = tokio::spawn(async move {
        let mut seq = 0u64;
        loop {
            let event = match events.recv().await {
                Ok(e) => e,
                Err(broadcast::error::RecvError::Lagged(n)) => {
                    tracing::warn!(missed = n, "the log fell behind");
                    continue;
                }
                Err(broadcast::error::RecvError::Closed) => break,
            };
            // The reply arrives whole after its pieces.
            if matches!(event, Event::ReplyDelta { .. }) {
                continue;
            }
            seq += 1;
            if let Event::Snapshot { id, jpeg, .. } = &event
                && let Err(e) = std::fs::write(blobs.join(format!("{id}.jpg")), jpeg.as_slice())
            {
                tracing::warn!(error = %e, "could not write a snapshot");
            }
            let ts = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_or(0.0, |d| d.as_secs_f64());
            let line = json!({"seq": seq, "ts": ts, "event": event});
            if let Err(e) = writeln!(file, "{line}") {
                tracing::warn!(error = %e, "could not write the log");
            }
        }
    });
    Ok((path, handle))
}
