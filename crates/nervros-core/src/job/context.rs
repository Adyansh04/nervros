//! What a running job has: its id, its envelope, a way to report, and whether it must stop.

use std::time::Duration;

use tokio::sync::watch;
use tokio::time::Instant;

use super::Envelope;
use crate::session::{Event, SessionHandle};

/// Handed to a job's work. Cheap to clone, so blocking work (a program's engine) can take one.
#[derive(Clone)]
pub struct Context {
    id: String,
    envelope: Envelope,
    deadline: Instant,
    stop: watch::Receiver<bool>,
    session: Option<SessionHandle>,
}

impl std::fmt::Debug for Context {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Context")
            .field("id", &self.id)
            .field("envelope", &self.envelope)
            .finish_non_exhaustive()
    }
}

impl Context {
    pub(super) fn new(
        id: String,
        envelope: Envelope,
        stop: watch::Receiver<bool>,
        session: Option<SessionHandle>,
    ) -> Self {
        Self {
            deadline: Instant::now() + envelope.limit(),
            id,
            envelope,
            stop,
            session,
        }
    }

    /// Its id, such as `j3`.
    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    /// What the operator approved.
    #[must_use]
    pub fn envelope(&self) -> &Envelope {
        &self.envelope
    }

    /// One line of what it does, for the operator.
    pub fn log(&self, line: impl Into<String>) {
        if let Some(s) = &self.session {
            s.emit(Event::JobLog {
                id: self.id.clone(),
                line: line.into(),
            });
        }
    }

    /// Whether it was told to stop, or ran out of time.
    #[must_use]
    pub fn stopped(&self) -> bool {
        *self.stop.borrow() || Instant::now() >= self.deadline
    }

    /// How long it may still run.
    #[must_use]
    pub fn left(&self) -> Duration {
        self.deadline.saturating_duration_since(Instant::now())
    }

    /// Resolves once it is told to stop or runs out of time.
    pub async fn until_stopped(&self) {
        let mut stop = self.stop.clone();
        let told = async {
            // An error means the job's entry went, which also ends it.
            let _ = stop.wait_for(|s| *s).await;
        };
        tokio::select! {
            () = told => {}
            () = tokio::time::sleep(self.left()) => {}
        }
    }

    /// The session, to run missions in.
    pub(crate) fn session(&self) -> Option<&SessionHandle> {
        self.session.as_ref()
    }
}
