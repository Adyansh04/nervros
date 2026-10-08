//! Cameras: each camera's recent frames, kept so a detection is drawn on the frame it came from.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use nervros_ros::{Frame, RobotPort};
use serde_json::{Value, json};

use crate::profile::{CameraConfig, LookConfig};

/// Frames kept for matching detection stamps: 4 s at 10 Hz, 1.3 s at 30 Hz.
// ponytail: a count, not a time span; a detector that lags more reads frames of its own
// (`detection_image`), and those are kept apart.
const FRAME_HISTORY: usize = 40;

/// A camera that has sent nothing for this long has stopped, whatever its last frame shows.
const CAMERA_QUIET: Duration = Duration::from_secs(3);

/// Recent frames of one camera, and when the newest arrived.
#[derive(Debug, Default)]
pub(crate) struct History(Mutex<(VecDeque<Arc<Frame>>, Option<Instant>)>);

impl History {
    fn push(&self, f: Arc<Frame>) {
        let mut h = crate::lock(&self.0);
        h.0.push_back(f);
        while h.0.len() > FRAME_HISTORY {
            h.0.pop_front();
        }
        h.1 = Some(Instant::now());
    }

    /// The kept frame with the stamp `stamp_s`, give or take `within` seconds.
    pub(crate) fn closest(&self, stamp_s: f64, within: f64) -> Option<Arc<Frame>> {
        crate::lock(&self.0)
            .0
            .iter()
            .min_by(|a, b| {
                (a.stamp_s - stamp_s)
                    .abs()
                    .total_cmp(&(b.stamp_s - stamp_s).abs())
            })
            .filter(|f| (f.stamp_s - stamp_s).abs() <= within)
            .cloned()
    }

    pub(crate) fn newest(&self) -> Option<Arc<Frame>> {
        crate::lock(&self.0).0.back().cloned()
    }

    /// How long since a frame arrived, once one has.
    fn quiet_for(&self) -> Option<Duration> {
        crate::lock(&self.0).1.map(|t| t.elapsed())
    }

    pub(crate) fn recent(&self) -> Vec<Arc<Frame>> {
        crate::lock(&self.0).0.iter().cloned().collect()
    }
}

/// One camera and its recent frames.
#[derive(Debug)]
pub struct Camera {
    /// The name calls use.
    pub name: String,
    /// Its topics.
    pub config: CameraConfig,
    pub(crate) history: Arc<History>,
    /// The frames its detector reads, when they are not `history`'s.
    pub(crate) detected: Option<Arc<History>>,
    /// Detections of a frame older than this, against the newest, are of another moment.
    pub(crate) max_age: Duration,
}

impl Camera {
    /// The newest frame, or why there is none.
    ///
    /// # Errors
    ///
    /// No frame has arrived yet, or none for a while: an old frame is not what the camera sees.
    pub fn newest(&self) -> Result<Arc<Frame>, String> {
        let frame = self.history.newest().ok_or_else(|| {
            format!(
                "no frame from the {} camera ({}) yet",
                self.name, self.config.image
            )
        })?;
        match self.history.quiet_for() {
            Some(quiet) if quiet > CAMERA_QUIET => Err(format!(
                "the {} camera ({}) has sent nothing for {:.0} s",
                self.name,
                self.config.image,
                quiet.as_secs_f64()
            )),
            _ => Ok(frame),
        }
    }

    /// The kept frame that detections stamped `stamp_s` were cut from: one its detector reads,
    /// or one of its own.
    pub(crate) fn frame_of(&self, stamp_s: f64) -> Option<Arc<Frame>> {
        self.detected
            .iter()
            .chain(std::iter::once(&self.history))
            .find_map(|frames| frames.closest(stamp_s, super::DETECTION_MATCH_S))
    }
}

/// Starts keeping the recent frames of an image topic.
fn keep(robot: &Arc<dyn RobotPort>, topic: &str) -> Result<Arc<History>, nervros_ros::RosError> {
    let mut frames = robot.frames(topic)?;
    let history = Arc::new(History::default());
    let kept = Arc::clone(&history);
    tokio::spawn(async move {
        loop {
            let latest = frames.borrow_and_update().clone();
            if let Some(f) = latest {
                kept.push(f);
            }
            if frames.changed().await.is_err() {
                break;
            }
        }
    });
    Ok(history)
}

/// Every camera the profile names, `[look]`'s own first, each keeping its recent frames.
#[derive(Debug)]
pub struct Cameras(Vec<Camera>);

impl Cameras {
    /// Starts keeping frames of every camera. Must be called inside a tokio runtime.
    ///
    /// # Errors
    ///
    /// An image subscription could not be created.
    pub fn start(
        look: &LookConfig,
        robot: &Arc<dyn RobotPort>,
    ) -> Result<Arc<Self>, nervros_ros::RosError> {
        let mut cameras = Vec::new();
        for (name, config) in look.all_cameras() {
            let history = keep(robot, &config.image)?;
            let detected = config
                .detection_image
                .as_deref()
                .map(|topic| keep(robot, topic))
                .transpose()?;
            cameras.push(Camera {
                name,
                config,
                history,
                detected,
                max_age: look.max_age,
            });
        }
        Ok(Arc::new(Self(cameras)))
    }

    /// A camera by name, or the default one for `None`.
    ///
    /// # Errors
    ///
    /// No camera has that name.
    pub fn get(&self, name: Option<&str>) -> Result<&Camera, String> {
        let Some(name) = name.map(str::trim).filter(|n| !n.is_empty()) else {
            return self.0.first().ok_or_else(|| "no camera".to_owned());
        };
        self.0.iter().find(|c| c.name == name).ok_or_else(|| {
            format!(
                "no camera `{name}`; the cameras are {}",
                self.names().join(", ")
            )
        })
    }

    /// The names, the default first.
    #[must_use]
    pub fn names(&self) -> Vec<&str> {
        self.0.iter().map(|c| c.name.as_str()).collect()
    }

    /// Adds a `camera` argument to a tool's parameters when there is more than one camera.
    pub(crate) fn add_argument(&self, parameters: &mut Value) {
        if self.0.len() < 2 {
            return;
        }
        let about: Vec<String> = self
            .0
            .iter()
            .enumerate()
            .map(|(i, c)| {
                let default = if i == 0 { " (default)" } else { "" };
                match &c.config.about {
                    Some(a) => format!("{}{default}: {a}", c.name),
                    None => format!("{}{default}.", c.name),
                }
            })
            .collect();
        parameters["properties"]["camera"] = json!({
            "type": "string",
            "enum": self.names(),
            "description": format!("Which camera. {}", about.join(" "))
        });
    }
}
