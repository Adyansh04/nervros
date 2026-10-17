//! Jobs: work the operator approves once that then runs on its own for minutes, such as a robot
//! program or a search for something. A job moves the robot only through missions its envelope
//! covers (`Missions::run_for_job`), says what it does as it goes, ends at its time limit, and a
//! stop of the robot ends every job. A job that ends cancels the mission it runs, and only that.

mod context;
pub mod envelope;

use std::future::Future;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

use serde::Serialize;
use tokio::sync::watch;
use tracing::Instrument as _;

pub use self::context::Context;
pub use self::envelope::Envelope;
use crate::guard::Guard;
use crate::lock;
use crate::mission::catalog::Catalog;
use crate::session::{Command, Event, SessionHandle};
use crate::tools::Risk;

/// Finished jobs kept for the list.
const KEPT: usize = 12;
/// Jobs running at once; at most one of them may move the robot.
const MAX_RUNNING: usize = 3;

/// What a job is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum JobKind {
    /// A robot program the model wrote.
    Program,
    /// A search for something.
    Search,
    /// A round of places, and what changed meanwhile.
    Patrol,
}

impl JobKind {
    /// As the operator reads it.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Program => "program",
            Self::Search => "search",
            Self::Patrol => "patrol",
        }
    }
}

/// How a job stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum JobState {
    /// Still at work.
    Running,
    /// It finished what it was for.
    Done,
    /// It could not.
    Failed,
    /// The operator, or a stop of the robot, ended it.
    Stopped,
    /// It ran out of time.
    TimedOut,
}

impl JobState {
    /// As the operator reads it.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Running => "running",
            Self::Done => "done",
            Self::Failed => "failed",
            Self::Stopped => "stopped",
            Self::TimedOut => "timed out",
        }
    }
}

/// What the operator approved a job for: its envelope, read against the catalog.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Approval {
    /// The skills and the time limit.
    pub envelope: Envelope,
    /// Whether any of its skills moves the robot.
    pub moves: bool,
    /// What it may do, as the operator read it.
    pub text: String,
}

impl Approval {
    /// The approval `envelope` asks for on this catalog.
    #[must_use]
    pub fn of(envelope: Envelope, catalog: &Catalog) -> Self {
        Self {
            moves: envelope.risk(catalog) >= Risk::Motion,
            text: envelope.describe(catalog),
            envelope,
        }
    }
}

/// A job as the operator sees it.
#[derive(Debug, Clone, Serialize)]
pub struct JobInfo {
    /// Such as `j3`.
    pub id: String,
    /// What it is.
    pub kind: JobKind,
    /// What it is for, in a few words.
    pub name: String,
    /// What it may do.
    pub envelope: Envelope,
    /// The same, as the operator approved it: "may move the base (`GoToPlace`), for at most 10
    /// min".
    pub allows: String,
    /// Unix seconds.
    pub started_s: f64,
    /// How it stands.
    pub state: JobState,
    /// Its result, or why it ended; empty while it runs.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub summary: String,
}

struct Entry {
    info: JobInfo,
    stop: watch::Sender<bool>,
    moves: bool,
}

/// The running and recent jobs.
pub struct Jobs {
    guard: Arc<Guard>,
    list: Arc<Mutex<Vec<Entry>>>,
    next: AtomicU64,
    session: OnceLock<SessionHandle>,
}

impl std::fmt::Debug for Jobs {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Jobs")
            .field("jobs", &self.list().len())
            .finish_non_exhaustive()
    }
}

impl Jobs {
    /// No jobs yet; [`Self::attach`] connects them to the session.
    #[must_use]
    pub fn new(guard: Arc<Guard>) -> Arc<Self> {
        Arc::new(Self {
            guard,
            list: Arc::default(),
            next: AtomicU64::new(1),
            session: OnceLock::new(),
        })
    }

    /// Where jobs report; also ends every job as soon as the robot is stopped.
    pub fn attach(self: &Arc<Self>, session: SessionHandle) {
        if self.session.set(session).is_err() {
            return;
        }
        let me = Arc::downgrade(self);
        let mut stops = self.guard.stopped();
        tokio::spawn(async move {
            while stops.changed().await.is_ok() {
                let Some(me) = me.upgrade() else { return };
                me.stop("all");
            }
        });
    }

    /// The jobs, oldest first.
    #[must_use]
    pub fn list(&self) -> Vec<JobInfo> {
        lock(&self.list).iter().map(|e| e.info.clone()).collect()
    }

    /// Tells a running job to stop, by id or `all`; whether any was running.
    pub fn stop(&self, id: &str) -> bool {
        let list = lock(&self.list);
        let mut any = false;
        for e in list
            .iter()
            .filter(|e| e.info.state == JobState::Running && (id == "all" || e.info.id == id))
        {
            any |= !e.stop.send_replace(true);
        }
        any
    }

    /// Starts `work` as a job of `kind` within what `approval` covers. Its result or failure
    /// becomes the job's summary.
    ///
    /// # Errors
    ///
    /// Too many jobs run already, or another one moves the robot.
    pub fn spawn<F, Fut>(
        self: &Arc<Self>,
        kind: JobKind,
        name: &str,
        approval: Approval,
        work: F,
    ) -> Result<JobInfo, String>
    where
        F: FnOnce(Context) -> Fut + Send + 'static,
        Fut: Future<Output = Result<String, String>> + Send + 'static,
    {
        let Approval {
            envelope,
            moves,
            text,
        } = approval;
        let (stop, told) = watch::channel(false);
        let info = {
            let mut list = lock(&self.list);
            let running: Vec<&Entry> = list
                .iter()
                .filter(|e| e.info.state == JobState::Running)
                .collect();
            if running.len() >= MAX_RUNNING {
                return Err(format!("{MAX_RUNNING} jobs run already; stop one first"));
            }
            if let Some(other) = running.iter().find(|e| e.moves).filter(|_| moves) {
                return Err(format!(
                    "job {} ({}) moves the robot already; wait for it or stop it",
                    other.info.id, other.info.name
                ));
            }
            let info = JobInfo {
                id: format!("j{}", self.next.fetch_add(1, Ordering::Relaxed)),
                kind,
                name: name.to_owned(),
                envelope: envelope.clone(),
                allows: text,
                started_s: crate::now_s(),
                state: JobState::Running,
                summary: String::new(),
            };
            let finished = list
                .iter()
                .filter(|e| e.info.state != JobState::Running)
                .count();
            if finished >= KEPT
                && let Some(oldest) = list.iter().position(|e| e.info.state != JobState::Running)
            {
                list.remove(oldest);
            }
            list.push(Entry {
                info: info.clone(),
                stop,
                moves,
            });
            info
        };
        let session = self.session.get().cloned();
        let ctx = Context::new(info.id.clone(), envelope, told, session.clone());
        if let Some(s) = &session {
            s.emit(Event::JobStarted { job: info.clone() });
        }
        let span = crate::telemetry::job(kind.as_str(), &info.id, name);
        let (list, id, name) = (Arc::clone(&self.list), info.id.clone(), name.to_owned());
        tokio::spawn(
            async move {
                let watcher = ctx.clone();
                // A stop wins over work that ends at the same moment: it was the operator's word.
                let result = tokio::select! {
                    biased;
                    () = watcher.until_stopped() => None,
                    r = work(ctx) => Some(r),
                };
                let state = match &result {
                    Some(Ok(_)) => JobState::Done,
                    Some(Err(_)) => JobState::Failed,
                    None if watcher.left().is_zero() => JobState::TimedOut,
                    None => JobState::Stopped,
                };
                let summary = match result {
                    Some(Ok(text) | Err(text)) => text,
                    None => String::new(),
                };
                let job = finish(&list, &id, state, &summary);
                if let (Some(s), Some(job)) = (&session, job) {
                    s.emit(Event::JobFinished { job: job.clone() });
                    s.send(Command::Report(report(&job, &name)));
                }
            }
            .instrument(span),
        );
        Ok(info)
    }
}

impl Drop for Jobs {
    /// Work started on the robot's behalf ends with what started it.
    fn drop(&mut self) {
        self.stop("all");
    }
}

/// Marks job `id` ended; the job as it ended.
fn finish(list: &Mutex<Vec<Entry>>, id: &str, state: JobState, summary: &str) -> Option<JobInfo> {
    let mut list = lock(list);
    let entry = list.iter_mut().find(|e| e.info.id == id)?;
    entry.info.state = state;
    summary.clone_into(&mut entry.info.summary);
    Some(entry.info.clone())
}

/// What the model is told when a job ends. Its result can hold what the world model calls
/// things, such as a program's list of objects: world text.
fn report(job: &JobInfo, name: &str) -> String {
    let kind = job.kind.as_str();
    let said = crate::tools::from_world(&job.summary);
    match job.state {
        JobState::Done => format!("The {kind} {} ({name}) finished: {said}", job.id),
        JobState::Failed => format!("The {kind} {} ({name}) failed: {said}", job.id),
        JobState::TimedOut => format!(
            "The {kind} {} ({name}) ran out of its {} min and was ended.",
            job.id,
            job.envelope.max_minutes()
        ),
        // Only the operator, the model or a stop of the robot ends one so: there is no fault to
        // look for.
        JobState::Stopped | JobState::Running => {
            format!(
                "The {kind} {} ({name}) was stopped, as asked: not a fault.",
                job.id
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;
    use crate::guard::{Guard, Policy};

    fn jobs() -> Arc<Jobs> {
        Jobs::new(Arc::new(Guard::new(Policy::default())))
    }

    fn approval(minutes: u64, moves: bool) -> Approval {
        Approval {
            envelope: Envelope::new([], minutes),
            moves,
            text: String::new(),
        }
    }

    async fn settled(jobs: &Jobs, id: &str) -> JobInfo {
        for _ in 0..200 {
            if let Some(j) = jobs
                .list()
                .into_iter()
                .find(|j| j.id == id && j.state != JobState::Running)
            {
                return j;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        panic!("job {id} did not end");
    }

    #[tokio::test]
    async fn a_job_ends_with_its_result() {
        let jobs = jobs();
        let job = jobs
            .spawn(
                JobKind::Program,
                "count",
                approval(1, false),
                |ctx| async move {
                    ctx.log("counting");
                    Ok("3 mugs".to_owned())
                },
            )
            .unwrap();
        let done = settled(&jobs, &job.id).await;
        assert_eq!(
            (done.state, done.summary.as_str()),
            (JobState::Done, "3 mugs")
        );
    }

    #[tokio::test]
    async fn a_stopped_job_ends_at_once_and_only_one_may_move() {
        let jobs = jobs();
        let forever = |ctx: Context| async move {
            ctx.until_stopped().await;
            Ok("never".to_owned())
        };
        let first = jobs
            .spawn(JobKind::Search, "keys", approval(5, true), forever)
            .unwrap();
        let refused = jobs.spawn(JobKind::Search, "mug", approval(5, true), forever);
        assert!(refused.unwrap_err().contains("moves the robot already"));
        let looking = jobs.spawn(JobKind::Program, "look", approval(5, false), forever);
        assert!(looking.is_ok(), "one that only looks may run beside it");

        assert!(jobs.stop(&first.id));
        assert_eq!(settled(&jobs, &first.id).await.state, JobState::Stopped);
        assert!(!jobs.stop(&first.id), "it no longer runs");
        assert!(jobs.stop("all"));
    }

    #[tokio::test(start_paused = true)]
    async fn a_job_past_its_time_is_ended() {
        let jobs = jobs();
        let job = jobs
            .spawn(JobKind::Program, "slow", approval(1, false), |_| async {
                tokio::time::sleep(Duration::from_hours(1)).await;
                Ok(String::new())
            })
            .unwrap();
        tokio::time::sleep(Duration::from_secs(61)).await;
        assert_eq!(settled(&jobs, &job.id).await.state, JobState::TimedOut);
    }
}
