//! The `look` builtin: the camera frame the newest detections were cut from, with each detection
//! drawn as a numbered mark (Set-of-Mark), stored as a snapshot that the model and the user can
//! refer to by mark number. The frames, marks and snapshots are the [`crate::vision`] toolkit's.
//!
//! Detections arrive as `canopy_msgs/msg/InstanceMaskArray` (masks) or
//! `vision_msgs/msg/Detection2DArray` (boxes), read as JSON. Frames are kept for a few seconds so a
//! detection that lags its camera is drawn on the frame it came from, not on a newer one; a
//! detector that reads frames of its own (`detection_image`) is matched to those.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use image::RgbImage;
use nervros_ros::{Frame, RobotPort};
use serde_json::{Value, json};

use crate::llm::ImageInput;
use crate::profile::LookConfig;
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};
use crate::vision::{
    Camera, Cameras, DETECTION_MATCH_S, Detections, Instance, SnapshotStore, close_up, draw_marks,
    marks_json, parse_detections,
};

/// What `look` asks when the model gave no question.
const DEFAULT_QUESTION: &str =
    "Describe what the robot sees, briefly, naming what matters for finding or handling things.";

/// How often a look waiting for detections it can match checks for newer ones.
const DETECTION_POLL: Duration = Duration::from_millis(200);

/// A vision model that answers a question about one frame.
#[async_trait]
pub trait Eyes: Send + Sync {
    /// The answer and the id of the model that gave it.
    ///
    /// # Errors
    ///
    /// Why no model answered.
    async fn see(&self, prompt: &str, image: ImageInput) -> Result<(String, String), String>;
}

/// What one look found: the marked frame as kept, and what the vision model said of it.
pub(crate) struct Looked {
    pub(crate) snapshot: Arc<crate::vision::Snapshot>,
    pub(crate) camera: String,
    pub(crate) age_s: f64,
    pub(crate) marks: Vec<Value>,
    pub(crate) left_out: Option<String>,
    /// The answer and the model's id, or why none answered; none without eyes.
    pub(crate) seen: Option<Result<(String, String), String>>,
}

/// The `look` tool.
pub struct LookTool {
    spec: ToolSpec,
    config: LookConfig,
    robot: Arc<dyn RobotPort>,
    cameras: Arc<Cameras>,
    snapshots: Arc<SnapshotStore>,
    eyes: Option<Arc<dyn Eyes>>,
}

impl LookTool {
    /// `look` over the given cameras.
    #[must_use]
    pub fn new(
        config: LookConfig,
        cameras: Arc<Cameras>,
        robot: Arc<dyn RobotPort>,
        snapshots: Arc<SnapshotStore>,
        eyes: Option<Arc<dyn Eyes>>,
    ) -> Self {
        let mut parameters = json!({"type": "object", "properties": {
            "question": {"type": "string", "description": "What to find out from the frame, such as \"is there a red mug on the table?\". Leave it out for a short description."},
            "snapshot": {"type": "string", "description": "Instead of looking now: an earlier snapshot, or an image the operator attached, such as s7."},
            "mark": {"type": "integer", "minimum": 1, "description": "With snapshot: a close-up of this mark alone, for small details such as text, colour or what is inside it."}
        }, "additionalProperties": false});
        cameras.add_argument(&mut parameters);
        let spec = ToolSpec::new(
            "look",
            "Looks through the robot's camera now. The detector's finds are drawn on the frame as \
             numbered marks (label, score, box), a vision model looks at that frame and answers \
             `question`, and the operator sees it too. Refer to things as `mark N` of the returned \
             snapshot. With `snapshot`, it answers about that image instead: an earlier snapshot, \
             or one the operator attached, which their message names as [image sN].",
            parameters,
            Risk::Observe,
        );
        Self {
            spec,
            config,
            robot,
            cameras,
            snapshots,
            eyes,
        }
    }

    /// Looks now: the frame the newest detections were cut from with them drawn as marks, kept
    /// as a snapshot, and the vision model's answer to `question` about it. The mission's camera
    /// check asks through this too, so it sees the marks and their labels as the agent does.
    pub(crate) async fn look_now(
        &self,
        question: Option<&str>,
        camera: Option<&str>,
    ) -> Result<Looked, String> {
        let camera = self.cameras.get(camera)?;
        let newest = camera.newest()?;
        let (mut dets, frame, age, left_out) = self.marked(camera, newest).await;
        dets.instances.sort_by(|a, b| b.score.total_cmp(&a.score));
        dets.instances.truncate(self.config.max_marks);
        // Tens of milliseconds of pixels: off the threads that serve the stop.
        let (marks_drawn, source) = (dets.instances.clone(), Arc::clone(&frame));
        let (img, jpeg) = tokio::task::spawn_blocking(move || {
            let mut img = source.to_rgb().map_err(|e| e.to_string())?;
            draw_marks(&mut img, &marks_drawn);
            let jpeg = nervros_ros::image::encode_jpeg(&img, nervros_ros::image::JPEG_QUALITY)
                .map_err(|e| e.to_string())?;
            Ok::<_, String>((img, jpeg))
        })
        .await
        .map_err(|e| e.to_string())??;
        let marks = marks_json(&dets.instances);
        let seen = self
            .see(
                question,
                &img,
                &dets.instances,
                camera.config.about.as_deref(),
            )
            .await;
        let snapshot = self
            .snapshots
            .store(jpeg, img.dimensions(), frame.stamp_s, dets.instances);
        Ok(Looked {
            snapshot,
            camera: camera.name.clone(),
            age_s: age,
            marks,
            left_out,
            seen,
        })
    }

    async fn run(
        &self,
        question: Option<&str>,
        camera: Option<&str>,
    ) -> Result<ToolOutcome, String> {
        let looked = self.look_now(question, camera).await?;
        let mut data = json!({"snapshot": looked.snapshot.id, "camera": looked.camera,
            "age_s": (looked.age_s * 10.0).round() / 10.0, "marks": looked.marks});
        if let Some(why) = looked.left_out {
            data["no_marks"] = Value::String(why);
        }
        match looked.seen {
            Some(Ok((answer, model))) => {
                data["answer"] = Value::String(crate::tools::from_world(&answer));
                data["seen_by"] = Value::String(model);
            }
            Some(Err(why)) => data["not_seen"] = Value::String(why),
            None => {}
        }
        let mut out = ToolOutcome::ok(data);
        out.message = format!(
            "{} marks; the user sees the marked image",
            looked.marks.len()
        );
        out.images.push(looked.snapshot.image.clone());
        Ok(out)
    }

    /// The detections to draw and the frame they came from; without a detector, or with none
    /// that match a kept frame within the age the profile allows, the newest frame unmarked and
    /// why, rather than marks on a frame they were not cut from.
    async fn marked(
        &self,
        camera: &Camera,
        newest: Arc<Frame>,
    ) -> (Detections, Arc<Frame>, f64, Option<String>) {
        let unmarked = |frame: Arc<Frame>, why: Option<String>| {
            let dets = Detections {
                stamp_s: frame.stamp_s,
                instances: Vec::new(),
            };
            (dets, frame, 0.0, why)
        };
        let Some(topic) = &camera.config.detections else {
            return unmarked(newest, None);
        };
        // Just after start-up the newest masks can be of a frame taken before any was kept, and
        // the detector's next ones match: wait for those while masks may still be that old.
        let deadline = tokio::time::Instant::now() + self.config.max_age;
        loop {
            // Ages count from what the camera shows now, which the wait below moves on.
            let newest = camera.newest().unwrap_or_else(|_| Arc::clone(&newest));
            // A detector may publish less often than any fixed wait; one older than max_age is
            // refused anyway.
            let msg = match self
                .robot
                .latest_shared(&topic.topic, &topic.msg_type, self.config.max_age)
                .await
            {
                Ok(msg) => msg,
                Err(e) => return unmarked(newest, Some(format!("no detections ({e})"))),
            };
            let dets = parse_detections(&msg);
            // Either way: detections much newer than the newest frame mean a stalled camera.
            let age = newest.stamp_s - dets.stamp_s;
            if age.abs() > self.config.max_age.as_secs_f64() {
                let why = format!(
                    "the detections are {:.1} s apart from the camera's newest frame; the \
                     detector or the camera may be stopped",
                    age.abs()
                );
                return unmarked(newest, Some(why));
            }
            let frame = camera
                .detected
                .iter()
                .chain(std::iter::once(&camera.history))
                .find_map(|frames| frames.closest(dets.stamp_s, DETECTION_MATCH_S));
            if let Some(frame) = frame {
                return (dets, frame, age, None);
            }
            if tokio::time::Instant::now() >= deadline {
                return unmarked(
                    newest,
                    Some("the frame the detections came from is no longer kept".to_owned()),
                );
            }
            tokio::time::sleep(DETECTION_POLL).await;
        }
    }

    /// The vision model's answer about a kept snapshot or an image the operator attached; the
    /// operator already sees it, so it is not shown again.
    async fn again(
        &self,
        id: &str,
        mark: Option<usize>,
        question: Option<&str>,
    ) -> Result<ToolOutcome, String> {
        let snapshot = self.snapshots.get(id).ok_or_else(|| {
            format!("no image {id} is kept any more; look again, or ask for it again")
        })?;
        let img = image::load_from_memory(&snapshot.image.jpeg)
            .map_err(|e| e.to_string())?
            .to_rgb8();
        let mut data = json!({"snapshot": id, "marks": marks_json(&snapshot.marks)});
        let seen = match mark {
            Some(n) => {
                let m = n
                    .checked_sub(1)
                    .and_then(|i| snapshot.marks.get(i))
                    .ok_or_else(|| format!("{id} has {} marks, not {n}", snapshot.marks.len()))?;
                data["close_up"] = json!(n);
                let what = format!("mark {n} ({}) of snapshot {id}", m.label);
                self.see_close(question, &close_up(&img, m.bbox), &what)
                    .await
            }
            None => self.see(question, &img, &snapshot.marks, None).await,
        };
        match seen {
            Some(Ok((answer, model))) => {
                data["answer"] = Value::String(crate::tools::from_world(&answer));
                data["seen_by"] = Value::String(model);
            }
            Some(Err(why)) => data["not_seen"] = Value::String(why),
            None => return Err("no vision model is set up to look at images".to_owned()),
        }
        let mut out = ToolOutcome::ok(data);
        out.message = format!("looked at {id} again");
        Ok(out)
    }

    /// The vision model's answer about a close-up cut from a frame.
    async fn see_close(
        &self,
        question: Option<&str>,
        crop: &RgbImage,
        what: &str,
    ) -> Option<Result<(String, String), String>> {
        let eyes = self.eyes.as_ref()?;
        let image = match ImageInput::jpeg(crop) {
            Ok(image) => image,
            Err(e) => return Some(Err(e)),
        };
        let question = question.map(str::trim).filter(|q| !q.is_empty());
        let prompt = format!(
            "{}\n\nThis is a close-up of {what}, cut from a frame of the robot's camera with its \
             box drawn around it. Say so when something cannot be seen.",
            question.unwrap_or(DEFAULT_QUESTION)
        );
        Some(eyes.see(&prompt, image).await)
    }

    /// The vision model's answer about the marked frame, when there is one to ask.
    async fn see(
        &self,
        question: Option<&str>,
        marked: &RgbImage,
        instances: &[Instance],
        about: Option<&str>,
    ) -> Option<Result<(String, String), String>> {
        let eyes = self.eyes.as_ref()?;
        let image = match ImageInput::jpeg(marked) {
            Ok(image) => image,
            Err(e) => return Some(Err(e)),
        };
        let question = question.map(str::trim).filter(|q| !q.is_empty());
        let marks = if instances.is_empty() {
            // Said plainly: told of "marks: none", a small model reports marks it never saw.
            "The detector marked nothing on this frame.".to_owned()
        } else {
            let list = instances
                .iter()
                .enumerate()
                .map(|(i, m)| format!("{} {} ({:.2})", i + 1, m.label, m.score))
                .collect::<Vec<_>>()
                .join(", ");
            format!(
                "The detector's numbered marks on this frame: {list}. Refer to a mark by its number \
                 when you mention it."
            )
        };
        let about = about.map_or(String::new(), |a| format!(" About this camera: {a}"));
        let prompt = format!(
            "{}\n\n{marks} Say so when something cannot be seen.{about}",
            question.unwrap_or(DEFAULT_QUESTION)
        );
        Some(eyes.see(&prompt, image).await)
    }
}

#[async_trait]
impl Tool for LookTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let question = args["question"].as_str();
        let mark = args["mark"].as_u64().and_then(|m| usize::try_from(m).ok());
        let done = match args["snapshot"].as_str().map(str::trim) {
            Some(id) if !id.is_empty() => self.again(id, mark, question).await,
            _ if mark.is_some() => Err("`mark` needs the `snapshot` it is a mark of".to_owned()),
            _ => self.run(question, args["camera"].as_str()).await,
        };
        done.unwrap_or_else(ToolOutcome::failed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    use crate::vision::{CLOSE_UP_PX, PALETTE};
    use bytes::Bytes;
    use nervros_ros::fake::FakeRobot;

    fn frame(stamp_s: f64) -> Frame {
        Frame {
            stamp_s,
            frame_id: "camera".into(),
            width: 64,
            height: 48,
            encoding: "rgb8".into(),
            step: 64 * 3,
            is_bigendian: false,
            data: Bytes::from(vec![40u8; 64 * 48 * 3]),
        }
    }

    fn masks(stamp_s: f64) -> Value {
        json!({
            "header": {"stamp": {"sec": stamp_s.floor(), "nanosec": 0}, "frame_id": "camera"},
            "image_width": 64, "image_height": 48, "model": "mock",
            "instances": [
                {"label": "cup", "score": 0.4, "roi": {"x_offset": 30, "y_offset": 30, "width": 4, "height": 2}, "data": [255, 255, 0, 0, 255, 255, 255, 255]},
                {"label": "dustbin", "score": 0.9, "roi": {"x_offset": 5, "y_offset": 28, "width": 10, "height": 10}, "data": []}
            ]
        })
    }

    fn start(
        config: LookConfig,
        robot: Arc<dyn RobotPort>,
        store: Arc<SnapshotStore>,
        eyes: Option<Arc<dyn Eyes>>,
    ) -> LookTool {
        let cameras = Cameras::start(&config, &robot).unwrap();
        LookTool::new(config, cameras, robot, store, eyes)
    }

    #[tokio::test]
    async fn a_named_camera_is_looked_through_and_one_without_detections_is_unmarked() {
        let robot: Arc<dyn RobotPort> = Arc::new(
            FakeRobot::new()
                .with_frame("/chest", frame(12.0))
                .with_frame("/head", frame(12.0))
                .with_topic("/masks", masks(12.0)),
        );
        let config: LookConfig = toml::from_str(
            "name = \"chest\"\nimage = \"/chest\"\ndetections = { topic = \"/masks\", type = \"canopy_msgs/msg/InstanceMaskArray\" }\n\
             [cameras.head]\nimage = \"/head\"\nabout = \"It looks down.\"\n",
        )
        .unwrap();
        let look = start(config, robot, Arc::new(SnapshotStore::default()), None);
        let camera = &look.spec().parameters["properties"]["camera"];
        assert_eq!(camera["enum"], json!(["chest", "head"]));
        assert!(
            camera["description"]
                .as_str()
                .unwrap()
                .contains("chest (default). head: It looks down."),
            "{camera}"
        );
        tokio::task::yield_now().await;
        let out = look.call(json!({"camera": "head"})).await;
        assert_eq!(out.data["camera"], "head", "{}", out.message);
        assert_eq!(out.data["marks"], json!([]));
        let out = look.call(json!({})).await;
        assert_eq!(out.data["camera"], "chest");
        assert_eq!(out.data["marks"][0]["label"], "dustbin");
        let out = look.call(json!({"camera": "wrist"})).await;
        assert!(
            out.message.contains("the cameras are chest, head"),
            "{}",
            out.message
        );
    }

    #[test]
    fn parses_both_detection_formats() {
        let d = parse_detections(&masks(12.0));
        assert!((d.stamp_s - 12.0).abs() < 1e-9);
        assert_eq!(d.instances.len(), 2);
        assert_eq!(d.instances[0].bbox, (30, 30, 4, 2));
        assert_eq!(d.instances[0].mask.as_ref().map(Vec::len), Some(8));
        assert!(d.instances[1].mask.is_none());
        let boxes = json!({"header": {"stamp": {"sec": 3, "nanosec": 500_000_000}}, "detections": [
            {"results": [{"hypothesis": {"class_id": "ball", "score": 0.7}}], "bbox": {"center": {"position": {"x": 20.0, "y": 10.0}}, "size_x": 8.0, "size_y": 4.0}}
        ]});
        let d = parse_detections(&boxes);
        assert!((d.stamp_s - 3.5).abs() < 1e-9);
        assert_eq!(d.instances[0].label, "ball");
        assert_eq!(d.instances[0].bbox, (16, 8, 8, 4));
    }

    #[test]
    fn marks_are_drawn_in_the_mark_colour() {
        let mut img = frame(0.0).to_rgb().unwrap();
        let inst = parse_detections(&masks(0.0)).instances;
        draw_marks(&mut img, &inst);
        // The first mark's box outline at its top-left corner.
        assert_eq!(img.get_pixel(30, 30).0, PALETTE[0]);
    }

    #[tokio::test]
    async fn look_returns_marks_best_first_and_an_image() {
        let robot: Arc<dyn RobotPort> = Arc::new(
            FakeRobot::new()
                .with_frame("/camera", frame(12.0))
                .with_topic("/masks", masks(12.0)),
        );
        let config: LookConfig = toml::from_str(
            "image = \"/camera\"\ndetections = { topic = \"/masks\", type = \"canopy_msgs/msg/InstanceMaskArray\" }\n",
        )
        .unwrap();
        let store = Arc::new(SnapshotStore::default());
        let look = start(config, robot, Arc::clone(&store), None);
        tokio::task::yield_now().await;
        let out = look.call(json!({})).await;
        assert_eq!(
            out.status,
            crate::tools::Status::Succeeded,
            "{}",
            out.message
        );
        assert_eq!(out.data["marks"][0]["label"], "dustbin");
        assert_eq!(out.data["marks"][1]["mark"], 2);
        let snap = store.get(out.data["snapshot"].as_str().unwrap()).unwrap();
        assert_eq!(snap.marks.len(), 2);
        assert_eq!(&out.images[0].jpeg[..2], &[0xFF, 0xD8]);
    }

    #[tokio::test]
    async fn detections_of_another_frame_are_left_out_not_drawn_on_this_one() {
        let robot: Arc<dyn RobotPort> = Arc::new(
            FakeRobot::new()
                .with_frame("/camera", frame(12.0))
                .with_topic("/masks", masks(4.0)),
        );
        let config: LookConfig = toml::from_str(
            "image = \"/camera\"\ndetections = { topic = \"/masks\", type = \"canopy_msgs/msg/InstanceMaskArray\" }\n",
        )
        .unwrap();
        let look = start(config, robot, Arc::new(SnapshotStore::default()), None);
        tokio::task::yield_now().await;
        let out = look.call(json!({})).await;
        assert_eq!(
            out.status,
            crate::tools::Status::Succeeded,
            "{}",
            out.message
        );
        assert_eq!(out.data["marks"], json!([]));
        assert!(
            out.data["no_marks"]
                .as_str()
                .is_some_and(|w| w.contains("8.0 s apart")),
            "{}",
            out.data
        );
    }

    #[tokio::test]
    async fn a_detector_that_reads_its_own_frames_is_drawn_on_the_frame_it_read() {
        let robot: Arc<dyn RobotPort> = Arc::new(
            FakeRobot::new()
                .with_frame("/camera", frame(12.0))
                .with_frame("/still", frame(4.0))
                .with_topic("/masks", masks(4.0)),
        );
        let config: LookConfig = toml::from_str(
            "image = \"/camera\"\ndetection_image = \"/still\"\nmax_age = \"12s\"\n\
             detections = { topic = \"/masks\", type = \"canopy_msgs/msg/InstanceMaskArray\" }\n",
        )
        .unwrap();
        let look = start(config, robot, Arc::new(SnapshotStore::default()), None);
        tokio::task::yield_now().await;
        let out = look.call(json!({})).await;
        assert_eq!(out.data["marks"][0]["label"], "dustbin", "{}", out.data);
        assert_eq!(out.data["age_s"], 8.0);
    }

    #[tokio::test]
    async fn masks_of_a_frame_never_kept_are_waited_out_for_the_next() {
        let fake = Arc::new(
            FakeRobot::new()
                .with_frame("/camera", frame(12.0))
                .with_frame("/still", frame(10.0))
                .with_topic("/masks", masks(6.0)),
        );
        let robot: Arc<dyn RobotPort> = fake.clone();
        let config: LookConfig = toml::from_str(
            "image = \"/camera\"\ndetection_image = \"/still\"\nmax_age = \"12s\"\n\
             detections = { topic = \"/masks\", type = \"canopy_msgs/msg/InstanceMaskArray\" }\n",
        )
        .unwrap();
        let look = start(config, robot, Arc::new(SnapshotStore::default()), None);
        tokio::task::yield_now().await;
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(300)).await;
            fake.set_topic("/masks", masks(10.0));
        });
        let out = look.call(json!({})).await;
        assert_eq!(out.data["marks"][0]["label"], "dustbin", "{}", out.data);
        assert_eq!(out.data["age_s"], 2.0);
    }

    struct FakeEyes {
        answer: Result<String, String>,
        asked: Mutex<Vec<(String, usize)>>,
    }

    #[async_trait]
    impl Eyes for FakeEyes {
        async fn see(&self, prompt: &str, image: ImageInput) -> Result<(String, String), String> {
            assert_eq!(&image.bytes[..2], &[0xFF, 0xD8], "a JPEG");
            crate::lock(&self.asked).push((prompt.to_owned(), image.bytes.len()));
            self.answer.clone().map(|a| (a, "fake-vlm".to_owned()))
        }
    }

    async fn look_with(eyes: Arc<FakeEyes>, question: Value) -> ToolOutcome {
        let robot: Arc<dyn RobotPort> = Arc::new(
            FakeRobot::new()
                .with_frame("/camera", frame(12.0))
                .with_topic("/masks", masks(12.0)),
        );
        let config: LookConfig = toml::from_str(
            "image = \"/camera\"\ndetections = { topic = \"/masks\", type = \"canopy_msgs/msg/InstanceMaskArray\" }\n",
        )
        .unwrap();
        let look = start(
            config,
            robot,
            Arc::new(SnapshotStore::default()),
            Some(eyes as Arc<dyn Eyes>),
        );
        tokio::task::yield_now().await;
        look.call(question).await
    }

    #[tokio::test]
    async fn a_vision_model_answers_the_question_about_the_marked_frame() {
        let eyes = Arc::new(FakeEyes {
            answer: Ok("Mark 1 is a dustbin by the wall.".into()),
            asked: Mutex::new(Vec::new()),
        });
        let out = look_with(
            Arc::clone(&eyes),
            json!({"question": "Is there a dustbin?"}),
        )
        .await;
        assert_eq!(
            out.data["answer"],
            "<world>Mark 1 is a dustbin by the wall.</world>"
        );
        assert_eq!(out.data["seen_by"], "fake-vlm");
        let asked = crate::lock(&eyes.asked);
        assert!(
            asked[0].0.starts_with("Is there a dustbin?"),
            "{}",
            asked[0].0
        );
        assert!(
            asked[0].0.contains("1 dustbin (0.90), 2 cup (0.40)"),
            "{}",
            asked[0].0
        );
        assert!(!asked[0].0.contains("About this camera"), "{}", asked[0].0);
    }

    #[test]
    fn a_close_up_takes_the_mark_with_a_margin_and_enlarges_a_small_one() {
        let img = RgbImage::new(640, 480);
        let small = close_up(&img, (100, 100, 8, 8));
        assert_eq!(
            small.dimensions(),
            (CLOSE_UP_PX, CLOSE_UP_PX),
            "56 px enlarged"
        );
        let big = close_up(&img, (0, 0, 640, 480));
        assert_eq!(
            big.dimensions(),
            (640, 480),
            "clamped to the frame, not enlarged"
        );
    }

    #[tokio::test]
    async fn an_earlier_snapshot_is_asked_about_again_without_a_new_frame() {
        let eyes = Arc::new(FakeEyes {
            answer: Ok("Mark 2 is a white cup.".into()),
            asked: Mutex::new(Vec::new()),
        });
        let robot: Arc<dyn RobotPort> = Arc::new(
            FakeRobot::new()
                .with_frame("/camera", frame(12.0))
                .with_topic("/masks", masks(12.0)),
        );
        let config: LookConfig = toml::from_str(
            "image = \"/camera\"\ndetections = { topic = \"/masks\", type = \"canopy_msgs/msg/InstanceMaskArray\" }\n",
        )
        .unwrap();
        let look = start(
            config,
            robot,
            Arc::new(SnapshotStore::default()),
            Some(Arc::clone(&eyes) as Arc<dyn Eyes>),
        );
        tokio::task::yield_now().await;
        let first = look.call(json!({})).await;
        let id = first.data["snapshot"].as_str().unwrap().to_owned();

        let again = look
            .call(json!({"snapshot": id, "question": "What is mark 2?"}))
            .await;

        assert_eq!(
            again.data["answer"],
            "<world>Mark 2 is a white cup.</world>"
        );
        assert_eq!(again.data["marks"][1]["label"], "cup");
        assert!(again.images.is_empty(), "the operator already sees it");
        let second = crate::lock(&eyes.asked)[1].0.clone();
        assert!(second.starts_with("What is mark 2?"), "{second}");
        let closer = look
            .call(json!({"snapshot": id, "mark": 2, "question": "What colour is it?"}))
            .await;
        assert_eq!(closer.data["close_up"], 2);
        let third = crate::lock(&eyes.asked)[2].0.clone();
        assert!(third.contains("close-up of mark 2 (cup)"), "{third}");
        let no_mark = look.call(json!({"snapshot": id, "mark": 9})).await;
        assert!(
            no_mark.message.contains("has 2 marks"),
            "{}",
            no_mark.message
        );
        let gone = look.call(json!({"snapshot": "s99"})).await;
        assert_eq!(gone.status, crate::tools::Status::Failed);
        assert!(gone.message.contains("no image s99"), "{}", gone.message);
    }

    #[tokio::test]
    async fn with_no_marks_the_vision_model_is_told_so_plainly() {
        let robot: Arc<dyn RobotPort> = Arc::new(
            FakeRobot::new()
                .with_frame("/camera", frame(12.0))
                .with_topic(
                    "/masks",
                    json!({"header": {"stamp": {"sec": 12, "nanosec": 0}}, "instances": []}),
                ),
        );
        let config: LookConfig = toml::from_str(
            "image = \"/camera\"\ndetections = { topic = \"/masks\", type = \"canopy_msgs/msg/InstanceMaskArray\" }\nabout = \"It points down at the floor.\"\n",
        )
        .unwrap();
        let eyes = Arc::new(FakeEyes {
            answer: Ok("A wooden floor.".into()),
            asked: Mutex::new(Vec::new()),
        });
        let look = start(
            config,
            robot,
            Arc::new(SnapshotStore::default()),
            Some(Arc::clone(&eyes) as Arc<dyn Eyes>),
        );
        tokio::task::yield_now().await;
        let _ = look.call(json!({})).await;
        let asked = crate::lock(&eyes.asked);
        assert!(asked[0].0.contains("marked nothing"), "{}", asked[0].0);
        assert!(!asked[0].0.contains("numbered"), "{}", asked[0].0);
        assert!(
            asked[0]
                .0
                .ends_with("About this camera: It points down at the floor."),
            "{}",
            asked[0].0
        );
    }

    #[tokio::test]
    async fn a_failed_vision_call_still_returns_the_marks() {
        let eyes = Arc::new(FakeEyes {
            answer: Err("no vision model is available".into()),
            asked: Mutex::new(Vec::new()),
        });
        let out = look_with(eyes, json!({})).await;
        assert_eq!(out.status, crate::tools::Status::Succeeded);
        assert_eq!(out.data["marks"][0]["label"], "dustbin");
        assert_eq!(out.data["not_seen"], "no vision model is available");
        assert!(out.data.get("answer").is_none());
    }
}
