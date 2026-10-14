//! The incident investigator. When something goes wrong (a mission fails, a watched topic goes
//! quiet, a watched node stops, a node logs errors in a burst) or someone asks why, it freezes the
//! flight recorder's last minutes, runs every playbook that may explain it on the robot as it is,
//! and keeps an incident: the likely causes with what their checks saw, what was ruled out, and
//! the timeline before it. It only reads; nothing here moves the robot.

mod checks;
mod playbook;
mod recorder;
mod report;
mod tool;
mod trigger;

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock, Weak};
use std::time::Duration;

use nervros_ros::RobotPort;
use tokio::sync::mpsc;

use playbook::Loaded;
pub use playbook::{Kind, Playbook};
use recorder::Recorder;
pub use report::{Incident, Verdict, read};
pub use trigger::Trigger;

use crate::profile::Profile;
use crate::session::{Command, Event, SessionHandle};
use crate::tools::Tool;

/// Triggers this close together open one incident: a stack going down stops many nodes at once.
const COALESCE: Duration = Duration::from_secs(2);
/// How long `/rosout`'s kept history is read when the recorder holds no line yet.
const HISTORY: Duration = Duration::from_secs(2);
/// The longest a question waits for an investigation under way.
const SETTLE: Duration = Duration::from_secs(20);

/// The recorder, the playbooks and the incidents of one robot.
pub struct Investigator {
    robot: Arc<dyn RobotPort>,
    recorder: Arc<Recorder>,
    books: Mutex<Vec<Loaded>>,
    problems: Vec<String>,
    remembered: Option<PathBuf>,
    /// Topics the profile keeps from being read, by playbooks too.
    read_deny: Vec<String>,
    store: report::Store,
    session: OnceLock<SessionHandle>,
    /// Incidents opened and not yet investigated.
    pending: AtomicUsize,
    /// Whether `/rosout`'s kept history was read: once is enough, the recorder follows it since.
    history_read: AtomicBool,
}

impl std::fmt::Debug for Investigator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Investigator")
            .field("playbooks", &crate::lock(&self.books).len())
            .finish_non_exhaustive()
    }
}

impl Investigator {
    /// Starts recording `profile`'s robot. Incidents are kept in `file`, and the playbooks the
    /// agent is asked to remember in the folder `remembered`, when given. Must run inside a tokio
    /// runtime.
    #[must_use]
    pub fn start(
        profile: &Profile,
        robot: Arc<dyn RobotPort>,
        file: Option<PathBuf>,
        remembered: Option<PathBuf>,
    ) -> Arc<Self> {
        let config = profile.incidents.clone().unwrap_or_default();
        let mut dirs: Vec<PathBuf> = config
            .playbooks
            .iter()
            .map(|p| profile.resolve(p))
            .collect();
        dirs.extend(remembered.clone());
        let vars = [
            ("map_frame", profile.ros.map_frame.as_str()),
            ("base_frame", profile.ros.base_frame.as_str()),
        ];
        let (books, problems) = playbook::load(&dirs, &vars);
        for problem in &problems {
            tracing::warn!(%problem, "a playbook was left out");
        }
        let (tx, rx) = mpsc::unbounded_channel();
        let recorder = Recorder::start(&robot, &config, tx);
        let me = Arc::new(Self {
            robot,
            recorder,
            books: Mutex::new(books),
            problems,
            remembered,
            read_deny: profile
                .ros_tools
                .as_ref()
                .map_or_else(Vec::new, |c| c.read_deny.clone()),
            store: report::Store::open(file),
            session: OnceLock::new(),
            pending: AtomicUsize::new(0),
            history_read: AtomicBool::new(false),
        });
        tokio::spawn(investigate_triggers(Arc::downgrade(&me), rx));
        me
    }

    /// Follows `session`: its missions are recorded, and incidents are reported into it.
    pub fn attach(&self, session: &SessionHandle) {
        self.recorder.follow(session.subscribe());
        let _ = self.session.set(session.clone());
    }

    /// Opens an incident now: the recording frozen, every playbook that may explain it run, and
    /// the incident kept and reported.
    pub async fn investigate(&self, trigger: Trigger) -> Incident {
        let incident = self.examine(self.store.next_id(), &trigger).await;
        self.store.add(incident.clone());
        if let Some(session) = self.session.get() {
            session.emit(Event::Incident {
                incident: incident.clone(),
            });
            // One it opened by itself goes to the model with the operator's next message: a failed
            // mission's report says why already, and one asked about is answered.
            if !matches!(incident.kind, Kind::MissionFailed | Kind::Asked)
                && let Some(cause) = incident.causes.first()
            {
                session.send(Command::Note(format!(
                    "incident {} opened, {}: likely {} (`why` tells more)",
                    incident.id, incident.trigger, cause.title
                )));
            }
        }
        incident
    }

    /// Every playbook against the robot as it is now, kept nowhere: the causes found, if any.
    pub async fn preflight(&self) -> Incident {
        let asked = Trigger::new(Kind::Asked, "a check before work");
        self.examine("check".to_owned(), &asked).await
    }

    async fn examine(&self, id: String, trigger: &Trigger) -> Incident {
        let mut recording = self.recorder.snapshot();
        if recording.logs.is_empty() && !self.history_read.swap(true, Ordering::SeqCst) {
            // Started a moment ago, as for one question from the command line: /rosout keeps its
            // last lines for a late reader.
            let now = crate::now_s();
            if let Ok(history) = self
                .robot
                .sample_messages("/rosout", "rcl_interfaces/msg/Log", 1000, HISTORY)
                .await
            {
                self.recorder
                    .add_logs(history.iter().filter_map(|m| recorder::line_of(m, now)));
                recording = self.recorder.snapshot();
            }
        }
        let graph = self.robot.graph_detail().await.ok();
        let books: Vec<Loaded> = crate::lock(&self.books)
            .iter()
            .filter(|b| b.explains(trigger.kind))
            .cloned()
            .collect();
        let probe = checks::Probe {
            robot: self.robot.as_ref(),
            recording: &recording,
            graph: graph.as_ref(),
            failure: &trigger.failure,
            read_deny: &self.read_deny,
        };
        let judged =
            futures::future::join_all(books.iter().map(|b| checks::judge(b, &probe))).await;
        report::build(id, trigger, &recording, books.iter().zip(judged).collect())
    }

    /// The incidents kept, newest last.
    #[must_use]
    pub fn incidents(&self) -> Arc<[Incident]> {
        self.store.list()
    }

    /// Waits, a while at most, for the incidents opened to be investigated: a question asked as
    /// a mission fails is about the incident it opened.
    pub async fn settled(&self) {
        let until = tokio::time::Instant::now() + SETTLE;
        while self.pending.load(Ordering::SeqCst) > 0 && tokio::time::Instant::now() < until {
            tokio::time::sleep(Duration::from_millis(200)).await;
        }
    }

    /// The playbooks in use.
    #[cfg(test)]
    pub(crate) fn playbooks(&self) -> Vec<Loaded> {
        crate::lock(&self.books).clone()
    }

    /// The playbooks that could not be read, and why.
    #[must_use]
    pub fn problems(&self) -> &[String] {
        &self.problems
    }

    /// Topics the profile keeps from being read.
    pub(crate) fn read_deny(&self) -> &[String] {
        &self.read_deny
    }

    /// Keeps `book` with the remembered playbooks and uses it from now on, in place of one of
    /// its id.
    ///
    /// # Errors
    ///
    /// It does not check, there is nowhere to keep it, or it could not be written.
    pub fn remember(&self, book: &Playbook) -> Result<PathBuf, String> {
        let dir = self
            .remembered
            .as_deref()
            .ok_or("this robot has nowhere to keep playbooks")?;
        let loaded = book.clone().check("remembered")?;
        let path = playbook::save(dir, book)?;
        let mut books = crate::lock(&self.books);
        match books.iter_mut().find(|b| b.book.id == book.id) {
            Some(old) => *old = loaded,
            None => books.push(loaded),
        }
        Ok(path)
    }
}

/// One trigger and those right behind it, as one: a failed mission leads, since it is what the
/// operator saw.
fn merge(mut lead: Trigger, more: Vec<Trigger>) -> Trigger {
    let mut also = Vec::new();
    for t in more {
        if t.kind == Kind::MissionFailed && lead.kind != Kind::MissionFailed {
            also.push(std::mem::replace(&mut lead, t).text);
        } else {
            also.push(t.text);
        }
    }
    if !also.is_empty() {
        lead.text = format!("{}; also {}", lead.text, also.join("; "));
    }
    lead
}

async fn investigate_triggers(me: Weak<Investigator>, mut rx: mpsc::UnboundedReceiver<Trigger>) {
    while let Some(first) = rx.recv().await {
        let Some(waiting) = me.upgrade() else { return };
        waiting.pending.fetch_add(1, Ordering::SeqCst);
        drop(waiting);
        tokio::time::sleep(COALESCE).await;
        let mut more = Vec::new();
        while let Ok(t) = rx.try_recv() {
            more.push(t);
        }
        let Some(me) = me.upgrade() else { return };
        me.investigate(merge(first, more)).await;
        me.pending.fetch_sub(1, Ordering::SeqCst);
    }
}

/// The `why` and `remember_finding` tools.
#[must_use]
pub fn tools(investigator: &Arc<Investigator>) -> Vec<Arc<dyn Tool>> {
    vec![
        Arc::new(tool::Why(Arc::clone(investigator))),
        Arc::new(tool::Remember(Arc::clone(investigator))),
    ]
}

#[cfg(test)]
mod tests {
    use nervros_ros::fake::FakeRobot;

    use super::*;

    #[test]
    fn a_failed_mission_leads_the_triggers_it_came_with() {
        let quiet = Trigger::new(Kind::TopicSilent, "/camera went quiet");
        let failed = Trigger {
            kind: Kind::MissionFailed,
            text: "mission m2 ended, failure".into(),
            mission: "m2".into(),
            failure: "no object".into(),
        };
        let gone = Trigger::new(Kind::NodeGone, "/detector stopped running");
        let merged = merge(quiet, vec![failed, gone]);
        assert_eq!(merged.kind, Kind::MissionFailed);
        assert_eq!(merged.mission, "m2");
        assert_eq!(
            merged.text,
            "mission m2 ended, failure; also /camera went quiet; /detector stopped running"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn a_question_waits_for_the_incident_under_way() {
        let profile = Profile::from_toml(
            "[robot]\nname = \"t\"\n[models]\nfile = \"m.toml\"\n",
            std::path::Path::new("nervros.toml"),
        )
        .unwrap();
        let investigator = Investigator::start(&profile, Arc::new(FakeRobot::new()), None, None);
        investigator
            .recorder
            .open(Trigger::new(Kind::NodeGone, "/detector stopped running"));
        tokio::task::yield_now().await;
        investigator.settled().await;
        let incidents = investigator.incidents();
        assert_eq!(incidents.len(), 1);
        assert_eq!(incidents[0].trigger, "/detector stopped running");
    }

    #[tokio::test]
    async fn an_incident_it_opens_by_itself_goes_to_the_model_with_the_next_message() {
        let profile = Profile::from_toml(
            "[robot]\nname = \"t\"\n[ros]\nmap_frame = \"map\"\nbase_frame = \"base_link\"\n\
             [models]\nfile = \"m.toml\"\n",
            std::path::Path::new("nervros.toml"),
        )
        .unwrap();
        let dir = tempfile::tempdir().unwrap();
        let books = dir.path().join("playbooks");
        std::fs::create_dir_all(&books).unwrap();
        std::fs::write(
            books.join("detector-down.toml"),
            "id = \"detector-down\"\ntitle = \"The detector is down\"\nverdict = \"v\"\n\
             fix = \"f\"\nexplains = [\"node_gone\"]\n[[check]]\ntf_missing = [\"map\", \"base_link\"]\n",
        )
        .unwrap();
        let investigator =
            Investigator::start(&profile, Arc::new(FakeRobot::new()), None, Some(books));
        let (tx, mut commands) = tokio::sync::mpsc::unbounded_channel();
        let (events, _rx) = tokio::sync::broadcast::channel(16);
        investigator.attach(&SessionHandle::for_tests(&tx, events));
        investigator
            .investigate(Trigger::new(Kind::NodeGone, "/detector stopped running"))
            .await;
        let Ok(Command::Note(note)) = commands.try_recv() else {
            panic!("no note for the model")
        };
        assert!(note.contains("likely The detector is down"), "{note}");
        investigator
            .investigate(Trigger::new(Kind::Asked, "asked"))
            .await;
        assert!(
            commands.try_recv().is_err(),
            "one asked about is answered, not noted"
        );
    }

    #[tokio::test]
    async fn asking_why_runs_the_playbooks_and_keeps_the_incident() {
        let profile = Profile::from_toml(
            r#"
            [robot]
            name = "t"
            [ros]
            map_frame = "map"
            base_frame = "base_link"
            [models]
            file = "m.toml"
            "#,
            std::path::Path::new("nervros.toml"),
        )
        .unwrap();
        let dir = tempfile::tempdir().unwrap();
        let investigator = Investigator::start(
            &profile,
            Arc::new(FakeRobot::new()),
            Some(dir.path().join("incidents.json")),
            Some(dir.path().join("playbooks")),
        );
        let incident = investigator
            .investigate(Trigger::new(Kind::Asked, "asked"))
            .await;
        assert_eq!(incident.id, "i1");
        let lost = &incident.causes[0];
        assert_eq!(lost.playbook, "no-localization", "{incident:?}");
        assert_eq!(
            lost.evidence,
            ["the robot's place on the map: nothing links map to base_link"]
        );
        assert!(
            incident
                .unchecked
                .iter()
                .any(|v| v.playbook == "duplicate-nodes"),
            "the fake has no graph"
        );
        assert_eq!(investigator.incidents().len(), 1);

        let book: Playbook = toml::from_str(
            "id = \"no-localization\"\ntitle = \"mine\"\nverdict = \"v\"\nfix = \"f\"\n\
             [[check]]\nfailure_has = \"lost\"\n",
        )
        .unwrap();
        let path = investigator.remember(&book).unwrap();
        assert!(path.ends_with("playbooks/no-localization.toml"));
        let books = investigator.playbooks();
        assert_eq!(
            books
                .iter()
                .filter(|b| b.book.id == "no-localization")
                .count(),
            1
        );
        assert!(books.iter().any(|b| b.book.title == "mine"));
    }
}
