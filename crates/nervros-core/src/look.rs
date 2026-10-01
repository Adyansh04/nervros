//! The `look` builtin: the camera frame the newest detections were cut from, with each detection
//! drawn as a numbered mark (Set-of-Mark), stored as a snapshot that the model and the user can
//! refer to by mark number. [`Cameras`] keeps the recent frames of every camera the profile names,
//! for `look` and `segment` alike.
//!
//! Detections arrive as `canopy_msgs/msg/InstanceMaskArray` (masks) or
//! `vision_msgs/msg/Detection2DArray` (boxes), read as JSON. Frames are kept for a few seconds so a
//! detection that lags its camera is drawn on the frame it came from, not on a newer one.

use std::borrow::Cow;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use async_trait::async_trait;
use image::{Rgb, RgbImage};
use nervros_ros::{Frame, RobotPort};
use serde_json::{Value, json};

use crate::llm::{ImageFormat, ImageInput};
use crate::profile::{CameraConfig, LookConfig};
use crate::tools::{ImageArtifact, Risk, Tool, ToolOutcome, ToolSpec};

/// The longest side of the frame a vision model gets: enough to read a label across a room, and
/// a fraction of the tokens a full 1280 px frame costs.
const MODEL_EDGE_PX: u32 = 768;
/// A close-up's long side at least.
const CLOSE_UP_PX: u32 = 384;

/// What `look` asks when the model gave no question.
const DEFAULT_QUESTION: &str =
    "Describe what the robot sees, briefly, naming what matters for finding or handling things.";

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

/// The frame scaled so its longest side is at most `edge` pixels.
pub(crate) fn capped(img: &RgbImage, edge: u32) -> RgbImage {
    let (w, h) = img.dimensions();
    let longest = w.max(h);
    if longest <= edge {
        return img.clone();
    }
    // side * edge / longest <= edge, so it fits a u32.
    let scaled = |side: u32| {
        u32::try_from(u64::from(side) * u64::from(edge) / u64::from(longest))
            .unwrap_or(edge)
            .max(1)
    };
    image::imageops::resize(
        img,
        scaled(w),
        scaled(h),
        image::imageops::FilterType::Triangle,
    )
}

/// Frames kept for matching detection stamps: 4 s at 10 Hz.
const FRAME_HISTORY: usize = 40;
/// Snapshots kept for mark references.
const SNAPSHOTS_KEPT: usize = 20;

/// One detection, in image pixels.
#[derive(Debug, Clone, PartialEq)]
pub struct Instance {
    /// The detector's label.
    pub label: String,
    /// Its confidence.
    pub score: f32,
    /// Box `(x, y, width, height)` in pixels.
    pub bbox: (u32, u32, u32, u32),
    /// Mask of the box's size, row-major, non-zero inside, if the detector gave one.
    pub mask: Option<Vec<u8>>,
}

/// Detections parsed from one message.
#[derive(Debug, Clone, PartialEq)]
pub struct Detections {
    /// The source image's stamp, seconds.
    pub stamp_s: f64,
    /// The instances.
    pub instances: Vec<Instance>,
}

fn f64_at(v: &Value, path: &str) -> f64 {
    v.pointer(path).and_then(Value::as_f64).unwrap_or(0.0)
}

fn px(v: f64) -> u32 {
    // Pixel coordinates from JSON: clamped to the u32 range, rounded.
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "clamped"
    )]
    let out = v.clamp(0.0, f64::from(u32::MAX)).round() as u32;
    out
}

fn stamp(header: &Value) -> f64 {
    f64_at(header, "/stamp/sec") + f64_at(header, "/stamp/nanosec") * 1e-9
}

/// Parses an `InstanceMaskArray` or a `Detection2DArray` from JSON.
#[must_use]
pub fn parse_detections(msg: &Value) -> Detections {
    let header = msg.get("header").cloned().unwrap_or(Value::Null);
    let mut instances = Vec::new();
    if let Some(list) = msg.get("instances").and_then(Value::as_array) {
        for i in list {
            let roi = &i["roi"];
            let bbox = (
                px(f64_at(roi, "/x_offset")),
                px(f64_at(roi, "/y_offset")),
                px(f64_at(roi, "/width")),
                px(f64_at(roi, "/height")),
            );
            let mask: Option<Vec<u8>> = i["data"].as_array().map(|d| {
                d.iter()
                    .map(|b| u8::try_from(b.as_u64().unwrap_or(0)).unwrap_or(u8::MAX))
                    .collect()
            });
            let mask =
                mask.filter(|m| m.len() == bbox.2 as usize * bbox.3 as usize && !m.is_empty());
            #[expect(clippy::cast_possible_truncation, reason = "scores are small")]
            let score = i["score"].as_f64().unwrap_or(0.0) as f32;
            instances.push(Instance {
                label: i["label"].as_str().unwrap_or("?").to_owned(),
                score,
                bbox,
                mask,
            });
        }
    } else if let Some(list) = msg.get("detections").and_then(Value::as_array) {
        for d in list {
            let best = d["results"].as_array().and_then(|r| {
                r.iter().max_by(|a, b| {
                    f64_at(a, "/hypothesis/score").total_cmp(&f64_at(b, "/hypothesis/score"))
                })
            });
            let (label, score) = best.map_or(("?".to_owned(), 0.0), |b| {
                (
                    b.pointer("/hypothesis/class_id")
                        .and_then(Value::as_str)
                        .unwrap_or("?")
                        .to_owned(),
                    f64_at(b, "/hypothesis/score"),
                )
            });
            let (cx, cy) = (
                f64_at(d, "/bbox/center/position/x"),
                f64_at(d, "/bbox/center/position/y"),
            );
            let (w, h) = (f64_at(d, "/bbox/size_x"), f64_at(d, "/bbox/size_y"));
            #[expect(clippy::cast_possible_truncation, reason = "scores are small")]
            let score = score as f32;
            instances.push(Instance {
                label,
                score,
                bbox: (px(cx - w / 2.0), px(cy - h / 2.0), px(w), px(h)),
                mask: None,
            });
        }
    }
    Detections {
        stamp_s: stamp(&header),
        instances,
    }
}

/// Distinct colours for marks, readable on camera images.
pub(crate) const PALETTE: [[u8; 3]; 12] = [
    [230, 25, 75],
    [60, 180, 75],
    [255, 225, 25],
    [0, 130, 200],
    [245, 130, 48],
    [145, 30, 180],
    [70, 240, 240],
    [240, 50, 230],
    [210, 245, 60],
    [250, 190, 212],
    [0, 128, 128],
    [170, 110, 40],
];

/// 5x7 digit glyphs, one bit per pixel, top row first.
const DIGITS: [[u8; 7]; 10] = [
    [0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E],
    [0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E],
    [0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F],
    [0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E],
    [0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02],
    [0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E],
    [0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E],
    [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
    [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
    [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C],
];

fn put(img: &mut RgbImage, at: (i64, i64), colour: [u8; 3]) {
    if let (Ok(x), Ok(y)) = (u32::try_from(at.0), u32::try_from(at.1))
        && x < img.width()
        && y < img.height()
    {
        img.put_pixel(x, y, Rgb(colour));
    }
}

/// A rectangle in pixels; may lie partly outside the image.
#[derive(Debug, Clone, Copy)]
struct Rect {
    left: i64,
    top: i64,
    width: i64,
    height: i64,
}

fn fill(img: &mut RgbImage, r: Rect, colour: [u8; 3]) {
    for row in r.top..r.top + r.height {
        for col in r.left..r.left + r.width {
            put(img, (col, row), colour);
        }
    }
}

/// Draws a number as a filled badge with white digits, scaled so digits are about 21 px tall.
pub(crate) fn badge(img: &mut RgbImage, at: (i64, i64), number: usize, colour: [u8; 3]) {
    const SCALE: i64 = 3;
    let digits: Vec<usize> = number
        .to_string()
        .bytes()
        .map(|b| usize::from(b - b'0'))
        .collect();
    let count = i64::try_from(digits.len()).unwrap_or(1);
    let size = Rect {
        left: at.0,
        top: at.1,
        width: count * 6 * SCALE + 2 * SCALE,
        height: 9 * SCALE,
    };
    fill(img, size, colour);
    for (i, digit) in digits.iter().enumerate() {
        let origin = at.0 + SCALE + i64::try_from(i).unwrap_or(0) * 6 * SCALE;
        for (row, bits) in (0_i64..).zip(DIGITS[*digit]) {
            for col in 0..5 {
                if bits & (0x10 >> col) != 0 {
                    let dot = Rect {
                        left: origin + col * SCALE,
                        top: at.1 + SCALE + row * SCALE,
                        width: SCALE,
                        height: SCALE,
                    };
                    fill(img, dot, [255, 255, 255]);
                }
            }
        }
    }
}

/// Blends a mask colour into the pixels a mask covers: three parts image, two parts colour.
pub(crate) fn tint(img: &mut RgbImage, inst: &Instance, colour: [u8; 3]) {
    let Some(mask) = &inst.mask else { return };
    let (bx, by, bw, bh) = inst.bbox;
    for row in 0..bh {
        for col in 0..bw {
            if mask[(row * bw + col) as usize] == 0 {
                continue;
            }
            let (x, y) = (bx + col, by + row);
            if x < img.width() && y < img.height() {
                let pixel = img.get_pixel_mut(x, y);
                for (channel, c) in pixel.0.iter_mut().zip(colour) {
                    *channel = u8::try_from((u16::from(*channel) * 3 + u16::from(c) * 2) / 5)
                        .unwrap_or(u8::MAX);
                }
            }
        }
    }
}

/// Draws the marks: a tinted mask where there is one, a box outline, and a numbered badge.
pub fn draw_marks(img: &mut RgbImage, instances: &[Instance]) {
    for (i, inst) in instances.iter().enumerate() {
        let colour = PALETTE[i % PALETTE.len()];
        tint(img, inst, colour);
        let (bx, by, bw, bh) = inst.bbox;
        let (left, top) = (i64::from(bx), i64::from(by));
        let (right, bottom) = (left + i64::from(bw), top + i64::from(bh));
        for t in 0..2 {
            for col in left..=right {
                put(img, (col, top + t), colour);
                put(img, (col, bottom - t), colour);
            }
            for row in top..=bottom {
                put(img, (left + t, row), colour);
                put(img, (right - t, row), colour);
            }
        }
        badge(img, (left, (top - 27).max(0)), i + 1, colour);
    }
}

/// A mark's box with a quarter of its size around it, at least 48 px a side, and enlarged so its
/// long side has at least `CLOSE_UP_PX`: a small mug's label is a few pixels in the full frame.
fn close_up(img: &RgbImage, (x, y, w, h): (u32, u32, u32, u32)) -> RgbImage {
    let margin = |side: u32| (side / 4).max(24);
    let (mx, my) = (margin(w), margin(h));
    let x0 = x.saturating_sub(mx);
    let y0 = y.saturating_sub(my);
    let x1 = (x + w + mx).min(img.width());
    let y1 = (y + h + my).min(img.height());
    let crop = image::imageops::crop_imm(img, x0, y0, x1 - x0, y1 - y0).to_image();
    let long = crop.width().max(crop.height()).max(1);
    if long >= CLOSE_UP_PX {
        return crop;
    }
    let scale = f64::from(CLOSE_UP_PX) / f64::from(long);
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "a positive size of a few hundred pixels"
    )]
    let size = |side: u32| (f64::from(side) * scale).round() as u32;
    image::imageops::resize(
        &crop,
        size(crop.width()),
        size(crop.height()),
        image::imageops::FilterType::CatmullRom,
    )
}

/// Marks as the model reads them: number, label, score and box.
pub(crate) fn marks_json(instances: &[Instance]) -> Vec<Value> {
    instances
        .iter()
        .enumerate()
        .map(|(i, inst)| {
            let (x, y, w, h) = inst.bbox;
            let score = (f64::from(inst.score) * 100.0).round() / 100.0;
            json!({"mark": i + 1, "label": inst.label, "score": score, "box": [x, y, w, h]})
        })
        .collect()
}

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

pub(crate) fn guard<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

impl SnapshotStore {
    /// A fresh id.
    pub fn next_id(&self) -> String {
        format!("s{}", self.next.fetch_add(1, Ordering::Relaxed) + 1)
    }

    /// Stores a snapshot, dropping the oldest past the limit.
    pub fn put(&self, snapshot: Snapshot) -> Arc<Snapshot> {
        let snapshot = Arc::new(snapshot);
        let mut kept = guard(&self.kept);
        kept.push_back(Arc::clone(&snapshot));
        while kept.len() > SNAPSHOTS_KEPT {
            kept.pop_front();
        }
        snapshot
    }

    /// A snapshot by id, if still kept.
    #[must_use]
    pub fn get(&self, id: &str) -> Option<Arc<Snapshot>> {
        guard(&self.kept).iter().find(|s| s.id == id).cloned()
    }
}

/// Recent frames of one camera.
#[derive(Debug, Default)]
pub(crate) struct History(Mutex<VecDeque<Arc<Frame>>>);

impl History {
    fn push(&self, f: Arc<Frame>) {
        let mut q = guard(&self.0);
        q.push_back(f);
        while q.len() > FRAME_HISTORY {
            q.pop_front();
        }
    }

    pub(crate) fn closest(&self, stamp_s: f64) -> Option<Arc<Frame>> {
        guard(&self.0)
            .iter()
            .min_by(|a, b| {
                (a.stamp_s - stamp_s)
                    .abs()
                    .total_cmp(&(b.stamp_s - stamp_s).abs())
            })
            .cloned()
    }

    pub(crate) fn newest(&self) -> Option<Arc<Frame>> {
        guard(&self.0).back().cloned()
    }

    pub(crate) fn recent(&self) -> Vec<Arc<Frame>> {
        guard(&self.0).iter().cloned().collect()
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
}

impl Camera {
    /// The newest frame, or why there is none.
    ///
    /// # Errors
    ///
    /// No frame has arrived yet.
    pub fn newest(&self) -> Result<Arc<Frame>, String> {
        self.history.newest().ok_or_else(|| {
            format!(
                "no frame from the {} camera ({}) yet",
                self.name, self.config.image
            )
        })
    }
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
            let mut frames = robot.frames(&config.image)?;
            let history = Arc::new(History::default());
            let keep = Arc::clone(&history);
            tokio::spawn(async move {
                loop {
                    let latest = frames.borrow_and_update().clone();
                    if let Some(f) = latest {
                        keep.push(f);
                    }
                    if frames.changed().await.is_err() {
                        break;
                    }
                }
            });
            cameras.push(Camera {
                name,
                config,
                history,
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

    async fn run(
        &self,
        question: Option<&str>,
        camera: Option<&str>,
    ) -> Result<ToolOutcome, String> {
        let camera = self.cameras.get(camera)?;
        let newest = camera.newest()?;
        let (mut dets, frame, age) = if let Some(topic) = &camera.config.detections {
            let msg = self
                .robot
                .latest(&topic.topic, &topic.msg_type, Duration::from_secs(3))
                .await
                .map_err(|e| e.to_string())?;
            let dets = parse_detections(&msg);
            let age = newest.stamp_s - dets.stamp_s;
            if age > self.config.max_age.as_secs_f64() {
                return Err(format!(
                    "the newest detections are {age:.1} s older than the camera; the detector may be stopped"
                ));
            }
            let frame = camera.history.closest(dets.stamp_s).unwrap_or(newest);
            (dets, frame, age)
        } else {
            let dets = Detections {
                stamp_s: newest.stamp_s,
                instances: Vec::new(),
            };
            (dets, newest, 0.0)
        };
        dets.instances.sort_by(|a, b| b.score.total_cmp(&a.score));
        dets.instances.truncate(self.config.max_marks);
        let mut img = frame.to_rgb().map_err(|e| e.to_string())?;
        draw_marks(&mut img, &dets.instances);
        let jpeg = nervros_ros::image::encode_jpeg(&img, 85).map_err(|e| e.to_string())?;
        let id = self.snapshots.next_id();
        let image = ImageArtifact {
            snapshot: id.clone(),
            jpeg: Arc::new(jpeg),
            width: img.width(),
            height: img.height(),
            marks: dets.instances.iter().map(|i| i.label.clone()).collect(),
        };
        let marks = marks_json(&dets.instances);
        let seen = self
            .see(
                question,
                &img,
                &dets.instances,
                camera.config.about.as_deref(),
            )
            .await;
        let snapshot = self.snapshots.put(Snapshot {
            id: id.clone(),
            stamp_s: frame.stamp_s,
            marks: dets.instances,
            image: image.clone(),
        });
        let mut data = json!({"snapshot": snapshot.id, "camera": camera.name,
            "age_s": (age * 10.0).round() / 10.0, "marks": marks});
        match seen {
            Some(Ok((answer, model))) => {
                data["answer"] = Value::String(crate::tools::from_world(&answer));
                data["seen_by"] = Value::String(model);
            }
            Some(Err(why)) => data["not_seen"] = Value::String(why),
            None => {}
        }
        let mut out = ToolOutcome::ok(data);
        out.message = format!("{} marks; the user sees the marked image", marks.len());
        out.images.push(image);
        Ok(out)
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
        let bytes = match nervros_ros::image::encode_jpeg(crop, 90) {
            Ok(b) => b,
            Err(e) => return Some(Err(e.to_string())),
        };
        let question = question.map(str::trim).filter(|q| !q.is_empty());
        let prompt = format!(
            "{}\n\nThis is a close-up of {what}, cut from a frame of the robot's camera with its \
             box drawn around it. Say so when something cannot be seen.",
            question.unwrap_or(DEFAULT_QUESTION)
        );
        let image = ImageInput {
            bytes,
            format: ImageFormat::Jpeg,
        };
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
        let small = capped(marked, MODEL_EDGE_PX);
        let bytes = match nervros_ros::image::encode_jpeg(&small, 80) {
            Ok(b) => b,
            Err(e) => return Some(Err(e.to_string())),
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
        let image = ImageInput {
            bytes,
            format: ImageFormat::Jpeg,
        };
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

    #[test]
    fn the_model_gets_a_frame_no_longer_than_768_px() {
        let wide = RgbImage::new(1280, 720);
        assert_eq!(capped(&wide, MODEL_EDGE_PX).dimensions(), (768, 432));
        let small = RgbImage::new(64, 48);
        assert_eq!(capped(&small, MODEL_EDGE_PX).dimensions(), (64, 48));
    }

    struct FakeEyes {
        answer: Result<String, String>,
        asked: Mutex<Vec<(String, usize)>>,
    }

    #[async_trait]
    impl Eyes for FakeEyes {
        async fn see(&self, prompt: &str, image: ImageInput) -> Result<(String, String), String> {
            assert_eq!(&image.bytes[..2], &[0xFF, 0xD8], "a JPEG");
            guard(&self.asked).push((prompt.to_owned(), image.bytes.len()));
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
        let asked = guard(&eyes.asked);
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
        let second = guard(&eyes.asked)[1].0.clone();
        assert!(second.starts_with("What is mark 2?"), "{second}");
        let closer = look
            .call(json!({"snapshot": id, "mark": 2, "question": "What colour is it?"}))
            .await;
        assert_eq!(closer.data["close_up"], 2);
        let third = guard(&eyes.asked)[2].0.clone();
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
        let asked = guard(&eyes.asked);
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
