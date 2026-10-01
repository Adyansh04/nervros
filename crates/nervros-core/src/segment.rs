//! The `segment` builtin: masks for whatever a text prompt names ("the floor", "every mug") in one
//! camera's newest frame. The operator sees them drawn; the model gets each region's box and share
//! of the frame, and refers to them as marks of the snapshot, as with `look`.
//!
//! Masks come from a vision model, the models file's `segment` role asked for outlines the way
//! Gemini draws them, or from a ROS service that answers with an `InstanceMaskArray`, such as
//! canopy's segmenter in front of a local Grounded SAM 2 server.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use image::{Rgb, RgbImage};
use nervros_ros::{Frame, RobotPort};
use serde_json::{Value, json};

use crate::llm::{ImageFormat, ImageInput};
use crate::look::{
    Camera, Cameras, Instance, PALETTE, Snapshot, SnapshotStore, badge, capped, parse_detections,
    tint,
};
use crate::profile::{SegmentBackend, SegmentConfig};
use crate::tools::{ImageArtifact, Risk, Tool, ToolOutcome, ToolSpec};

/// The type of `[segment] service`.
pub const SERVICE_TYPE: &str = "canopy_msgs/srv/Segment";

/// The longest side of the frame the model gets. Outlines come back normalised, so a larger frame
/// would cost tokens and add no precision the outline keeps.
const MODEL_EDGE_PX: u32 = 768;

/// A vision model that outlines what a prompt names.
#[async_trait]
pub trait Outliner: Send + Sync {
    /// The model's answer, text holding a JSON list, and the id of the model that gave it.
    ///
    /// # Errors
    ///
    /// Why no model answered.
    async fn outline(&self, prompt: &str, image: ImageInput) -> Result<(String, String), String>;
}

/// Google's segmentation prompt, with each outline asked for as named points. Asked for a
/// "mask", the Flash-Lite models answered with boxes alone or base64 that does not decode; asked
/// for `[x, y]` pairs, they mixed up the order.
fn outline_prompt(what: &str) -> String {
    format!(
        "Give the segmentation masks for {what}. Output a JSON list with one entry per region: \
         \"label\", a short description; \"box_2d\", its box as [ymin, xmin, ymax, xmax]; and \
         \"mask\", its outline as a list of points {{\"x\": x, \"y\": y}} in order around the \
         region. All coordinates are normalised to 0-1000. Output [] when there is none."
    )
}

/// `(x0, y0, x1, y1)` normalised.
type NormBox = (f64, f64, f64, f64);

/// What a scan of an answer finds.
#[derive(Debug, Clone, PartialEq)]
enum Found {
    Label(String),
    Box(NormBox),
    Point(f64, f64),
}

/// The number after a key's colon.
fn number_after_colon(text: &str) -> Option<f64> {
    let rest = text.trim_start().strip_prefix(':')?.trim_start();
    let end = rest
        .find(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | '-' | '+' | 'e' | 'E')))
        .unwrap_or(rest.len());
    rest[..end].parse().ok().filter(|v: &f64| v.is_finite())
}

/// Every label, box and point in an answer, in order. A scan rather than a JSON parse: the models
/// now and then garble one point (`{"x": 655, "y": 277},": x": 659, "y": 286}`), and a parse would
/// lose the whole answer to it.
fn scan(text: &str) -> Vec<Found> {
    let mut found: Vec<(usize, Found)> = Vec::new();
    for (at, key) in text.match_indices("\"label\"") {
        let label = text[at + key.len()..]
            .trim_start()
            .strip_prefix(':')
            .and_then(|r| r.trim_start().strip_prefix('"'))
            .and_then(|r| r.split('"').next());
        if let Some(label) = label {
            found.push((at, Found::Label(label.to_owned())));
        }
    }
    for (at, key) in text.match_indices("\"box_2d\"") {
        let rest = &text[at + key.len()..];
        let numbers: Option<Vec<f64>> = rest
            .find('[')
            .zip(rest.find(']'))
            .filter(|(a, b)| a < b)
            .and_then(|(a, b)| {
                rest[a + 1..b]
                    .split(',')
                    .map(|n| n.trim().parse().ok())
                    .collect()
            });
        if let Some(&[y0, x0, y1, x1]) = numbers.as_deref() {
            found.push((at, Found::Box((x0, y0, x1, y1))));
        }
    }
    // A point is an "x" and a "y" inside one pair of braces, in either order.
    for (at, key) in text.match_indices("\"x\"") {
        let (Some(open), Some(close)) = (text[..at].rfind('{'), text[at..].find('}')) else {
            continue;
        };
        let object = &text[open..at + close];
        if object[1..].contains('{') || text[open..at].contains('}') {
            continue;
        }
        let x = number_after_colon(&text[at + key.len()..]);
        let y = object
            .find("\"y\"")
            .and_then(|y_at| number_after_colon(&object[y_at + 3..]));
        if let (Some(x), Some(y)) = (x, y) {
            found.push((at, Found::Point(x, y)));
        }
    }
    found.sort_by_key(|(at, _)| *at);
    found.into_iter().map(|(_, f)| f).collect()
}

fn bounds(polys: &[Vec<(f64, f64)>]) -> Option<NormBox> {
    polys.iter().flatten().fold(None, |acc, &(x, y)| {
        Some(acc.map_or((x, y, x, y), |(x0, y0, x1, y1): NormBox| {
            (x0.min(x), y0.min(y), x1.max(x), y1.max(y))
        }))
    })
}

/// Fills polygons given in pixels into a mask the size of their bounding box: even-odd within a
/// polygon, a union across polygons, sampled at pixel centres.
fn fill(polys: &[Vec<(f64, f64)>], width: u32, height: u32) -> Option<(Rect, Vec<u8>)> {
    let (x0, y0, x1, y1) = bounds(polys)?;
    let clamp = |v: f64, hi: u32| {
        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "clamped to the frame"
        )]
        let out = v.clamp(0.0, f64::from(hi)) as u32;
        out
    };
    let rect = Rect {
        x: clamp(x0.floor(), width),
        y: clamp(y0.floor(), height),
        w: 0,
        h: 0,
    };
    let rect = Rect {
        w: clamp(x1.ceil(), width).saturating_sub(rect.x),
        h: clamp(y1.ceil(), height).saturating_sub(rect.y),
        ..rect
    };
    if rect.w == 0 || rect.h == 0 {
        return None;
    }
    let mut mask = vec![0_u8; rect.w as usize * rect.h as usize];
    let mut crossings = Vec::new();
    for poly in polys {
        for row in 0..rect.h {
            let yc = f64::from(rect.y + row) + 0.5;
            crossings.clear();
            for (i, &(xa, ya)) in poly.iter().enumerate() {
                let (xb, yb) = poly[(i + 1) % poly.len()];
                if (ya <= yc) != (yb <= yc) {
                    crossings.push(xa + (yc - ya) * (xb - xa) / (yb - ya));
                }
            }
            crossings.sort_by(f64::total_cmp);
            for pair in crossings.chunks_exact(2) {
                for col in 0..rect.w {
                    let xc = f64::from(rect.x + col) + 0.5;
                    if xc >= pair[0] && xc < pair[1] {
                        mask[(row * rect.w + col) as usize] = 255;
                    }
                }
            }
        }
    }
    mask.iter().any(|&m| m != 0).then_some((rect, mask))
}

/// A box in pixels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Rect {
    x: u32,
    y: u32,
    w: u32,
    h: u32,
}

fn to_px(b: NormBox, width: u32, height: u32) -> Option<Rect> {
    let (x0, y0, x1, y1) = bounds(&scale(&[vec![(b.0, b.1), (b.2, b.3)]], width, height))?;
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "clamped to the frame"
    )]
    let px = |v: f64, hi: u32| v.round().clamp(0.0, f64::from(hi)) as u32;
    let (x, y) = (px(x0, width), px(y0, height));
    let rect = Rect {
        x,
        y,
        w: px(x1, width).saturating_sub(x),
        h: px(y1, height).saturating_sub(y),
    };
    (rect.w > 0 && rect.h > 0).then_some(rect)
}

fn scale(polys: &[Vec<(f64, f64)>], width: u32, height: u32) -> Vec<Vec<(f64, f64)>> {
    let (w, h) = (f64::from(width) / 1000.0, f64::from(height) / 1000.0);
    polys
        .iter()
        .map(|p| p.iter().map(|&(x, y)| (x * w, y * h)).collect())
        .collect()
}

/// Regions from a model's answer to `outline_prompt`: per entry a label, a box and an outline of
/// named points, normalised to 0-1000. An entry without an outline keeps its box.
///
/// # Errors
///
/// The answer holds neither regions nor an empty list.
pub fn parse_outlines(text: &str, width: u32, height: u32) -> Result<Vec<Instance>, String> {
    #[derive(Default)]
    struct Entry {
        label: Option<String>,
        bbox: Option<NormBox>,
        points: Vec<(f64, f64)>,
    }
    let mut entries: Vec<Entry> = Vec::new();
    for found in scan(text) {
        // Entries end with their outline, so a label or box after points starts the next one.
        let fresh = entries.last().is_none_or(|e| match &found {
            Found::Label(_) => e.label.is_some() || !e.points.is_empty(),
            Found::Box(_) => e.bbox.is_some() || !e.points.is_empty(),
            Found::Point(..) => false,
        });
        if fresh {
            entries.push(Entry::default());
        }
        let Some(entry) = entries.last_mut() else {
            continue;
        };
        match found {
            Found::Label(label) => entry.label = Some(label),
            Found::Box(b) => entry.bbox = Some(b),
            Found::Point(x, y) => entry.points.push((x, y)),
        }
    }
    if entries.is_empty() && !text.contains('[') {
        let excerpt: String = text.chars().take(160).collect();
        return Err(format!("the model's answer holds no outlines: {excerpt}"));
    }
    let regions = entries.into_iter().filter_map(|e| {
        let outline = (e.points.len() >= 3)
            .then(|| fill(&scale(&[e.points], width, height), width, height))
            .flatten();
        let (rect, mask) = match outline {
            Some((rect, mask)) => (rect, Some(mask)),
            None => (to_px(e.bbox?, width, height)?, None),
        };
        Some(Instance {
            label: e.label.unwrap_or_else(|| "region".to_owned()),
            score: 0.0,
            bbox: (rect.x, rect.y, rect.w, rect.h),
            mask,
        })
    });
    Ok(regions.collect())
}

/// How regions are shown.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum View {
    /// The regions alone, the rest of the frame dimmed.
    Cutout,
    /// The regions tinted over the frame.
    Overlay,
}

fn inside(region: &Instance, x: u32, y: u32) -> bool {
    let (bx, by, bw, bh) = region.bbox;
    if x < bx || y < by || x >= bx + bw || y >= by + bh {
        return false;
    }
    region
        .mask
        .as_ref()
        .is_none_or(|m| m[((y - by) * bw + (x - bx)) as usize] != 0)
}

/// Pixels on a region's edge: inside, with a pixel within two that is not.
fn edge(region: &Instance, x: u32, y: u32) -> bool {
    (-2_i64..=2).any(|dy| {
        (-2_i64..=2).any(|dx| {
            match (
                u32::try_from(i64::from(x) + dx),
                u32::try_from(i64::from(y) + dy),
            ) {
                (Ok(nx), Ok(ny)) => !inside(region, nx, ny),
                _ => true,
            }
        })
    })
}

fn draw(frame: &RgbImage, regions: &[Instance], view: View) -> RgbImage {
    let mut img = frame.clone();
    if view == View::Cutout {
        for (x, y, pixel) in img.enumerate_pixels_mut() {
            if !regions.iter().any(|r| inside(r, x, y)) {
                let [r, g, b] = pixel.0.map(u32::from);
                // A dark grey of the pixel's luminance: the regions keep all the colour.
                let grey = u8::try_from((r * 77 + g * 150 + b * 29) / 256 / 4).unwrap_or(0);
                *pixel = Rgb([grey, grey, grey]);
            }
        }
    }
    for (i, region) in regions.iter().enumerate() {
        let colour = PALETTE[i % PALETTE.len()];
        if view == View::Overlay {
            tint(&mut img, region, colour);
        }
        let (bx, by, bw, bh) = region.bbox;
        let mut top = None;
        for y in by..(by + bh).min(img.height()) {
            for x in bx..(bx + bw).min(img.width()) {
                if inside(region, x, y) {
                    top.get_or_insert((x, y));
                    if edge(region, x, y) {
                        img.put_pixel(x, y, Rgb(colour));
                    }
                }
            }
        }
        if let Some((x, y)) = top {
            badge(
                &mut img,
                (i64::from(x), (i64::from(y) - 27).max(0)),
                i + 1,
                colour,
            );
        }
    }
    img
}

fn area_pct(region: &Instance, width: u32, height: u32) -> f64 {
    let (_, _, bw, bh) = region.bbox;
    let pixels = region
        .mask
        .as_ref()
        .map_or(u64::from(bw) * u64::from(bh), |m| {
            m.iter().filter(|&&v| v != 0).count() as u64
        });
    #[expect(clippy::cast_precision_loss, reason = "a percentage")]
    let pct = pixels as f64 * 100.0 / (f64::from(width) * f64::from(height)).max(1.0);
    (pct * 10.0).round() / 10.0
}

/// The `segment` tool.
pub struct SegmentTool {
    spec: ToolSpec,
    config: SegmentConfig,
    cameras: Arc<Cameras>,
    robot: Arc<dyn RobotPort>,
    snapshots: Arc<SnapshotStore>,
    outliner: Option<Arc<dyn Outliner>>,
}

impl SegmentTool {
    /// `segment` over the given cameras; the model backend needs an outliner.
    #[must_use]
    pub fn new(
        config: SegmentConfig,
        cameras: Arc<Cameras>,
        robot: Arc<dyn RobotPort>,
        snapshots: Arc<SnapshotStore>,
        outliner: Option<Arc<dyn Outliner>>,
    ) -> Self {
        let mut parameters = json!({"type": "object", "properties": {
            "prompt": {"type": "string", "description": "What to segment, such as \"the floor\" or \"every mug on the table\"."},
            "view": {"type": "string", "enum": ["cutout", "overlay"], "description": "cutout (default) shows the regions alone on a dimmed frame; overlay tints them over the frame."}
        }, "required": ["prompt"], "additionalProperties": false});
        cameras.add_argument(&mut parameters);
        if let (Some(service), Some(_)) = (&config.service, &outliner) {
            let default = match config.backend {
                SegmentBackend::Model => "model",
                SegmentBackend::Service => "service",
            };
            parameters["properties"]["backend"] = json!({
                "type": "string", "enum": ["model", "service"],
                "description": format!("model: a vision model outlines it, coarsely, in a few seconds; service: {service}, pixel masks from a local server that must be running. Default {default}.")
            });
        }
        let spec = ToolSpec::new(
            "segment",
            "Segments what `prompt` names in a camera's newest frame, including things no \
             detector marks, such as the floor, a wall or a free patch of table. The operator sees \
             the regions drawn; you get each region's box and share of the frame, as marks of the \
             returned snapshot.",
            parameters,
            Risk::Observe,
        );
        Self {
            spec,
            config,
            cameras,
            robot,
            snapshots,
            outliner,
        }
    }

    async fn run(&self, args: &Value) -> Result<ToolOutcome, String> {
        let prompt = args["prompt"].as_str().map(str::trim).unwrap_or_default();
        if prompt.is_empty() {
            return Err("say what to segment in `prompt`".to_owned());
        }
        let camera = self.cameras.get(args["camera"].as_str())?;
        let view = match args["view"].as_str() {
            None | Some("cutout") => View::Cutout,
            Some("overlay") => View::Overlay,
            Some(other) => return Err(format!("`view` is cutout or overlay, not `{other}`")),
        };
        let backend = match args["backend"].as_str() {
            None => self.config.backend,
            Some("model") => SegmentBackend::Model,
            Some("service") => SegmentBackend::Service,
            Some(other) => return Err(format!("`backend` is model or service, not `{other}`")),
        };
        let (frame, regions, by) = match backend {
            SegmentBackend::Model => self.ask_model(camera, prompt).await?,
            SegmentBackend::Service => self.ask_service(camera, prompt).await?,
        };
        let img = frame.to_rgb().map_err(|e| e.to_string())?;
        let drawn = draw(&img, &regions, view);
        let jpeg = nervros_ros::image::encode_jpeg(&drawn, 85).map_err(|e| e.to_string())?;
        let image = ImageArtifact {
            snapshot: self.snapshots.next_id(),
            jpeg: Arc::new(jpeg),
            width: drawn.width(),
            height: drawn.height(),
            marks: regions.iter().map(|r| r.label.clone()).collect(),
        };
        let listed: Vec<Value> = regions
            .iter()
            .enumerate()
            .map(|(i, r)| {
                let (x, y, w, h) = r.bbox;
                let mut entry = json!({"mark": i + 1, "label": r.label, "box": [x, y, w, h],
                    "area_pct": area_pct(r, img.width(), img.height())});
                if r.score > 0.0 {
                    entry["score"] = json!((f64::from(r.score) * 100.0).round() / 100.0);
                }
                entry
            })
            .collect();
        let snapshot = self.snapshots.put(Snapshot {
            id: image.snapshot.clone(),
            stamp_s: frame.stamp_s,
            marks: regions,
            image: image.clone(),
        });
        let mut out = ToolOutcome::ok(json!({"snapshot": snapshot.id, "camera": camera.name,
            "prompt": prompt, "by": by, "regions": listed}));
        out.message = if listed.is_empty() {
            format!("nothing matched `{prompt}`; the user sees the frame")
        } else {
            format!(
                "{} region(s); the user sees the segmented image",
                listed.len()
            )
        };
        out.images.push(image);
        Ok(out)
    }

    async fn ask_model(
        &self,
        camera: &Camera,
        prompt: &str,
    ) -> Result<(Arc<Frame>, Vec<Instance>, String), String> {
        let outliner = self
            .outliner
            .as_ref()
            .ok_or("no vision model is set up to segment")?;
        let frame = camera.newest()?;
        let img = frame.to_rgb().map_err(|e| e.to_string())?;
        let small = capped(&img, MODEL_EDGE_PX);
        let bytes = nervros_ros::image::encode_jpeg(&small, 85).map_err(|e| e.to_string())?;
        let image = ImageInput {
            bytes,
            format: ImageFormat::Jpeg,
        };
        let (answer, model) = outliner.outline(&outline_prompt(prompt), image).await?;
        tracing::debug!(%model, %answer, "segment outlines");
        let regions = parse_outlines(&answer, img.width(), img.height())?;
        Ok((frame, regions, model))
    }

    async fn ask_service(
        &self,
        camera: &Camera,
        prompt: &str,
    ) -> Result<(Arc<Frame>, Vec<Instance>, String), String> {
        let service = self
            .config
            .service
            .as_deref()
            .ok_or("no segment service is configured")?;
        // The server may take longer than the frames are kept, so hold on to them.
        let mut frames = camera.history.recent();
        let reply = self
            .robot
            .call(
                service,
                SERVICE_TYPE,
                json!({"camera": camera.name, "prompt": prompt}),
                self.config.timeout,
            )
            .await
            .map_err(|e| format!("{service} did not answer: {e}"))?;
        if reply["success"].as_bool() != Some(true) {
            return Err(format!(
                "{service}: {}",
                reply["message"].as_str().unwrap_or("it failed")
            ));
        }
        let masks = parse_detections(&reply["masks"]);
        frames.extend(camera.history.recent());
        let frame = frames
            .into_iter()
            .min_by(|a, b| {
                (a.stamp_s - masks.stamp_s)
                    .abs()
                    .total_cmp(&(b.stamp_s - masks.stamp_s).abs())
            })
            .ok_or_else(|| format!("no frame from the {} camera", camera.name))?;
        if (frame.stamp_s - masks.stamp_s).abs() > FRAME_MATCH.as_secs_f64() {
            return Err(format!(
                "{service} answered about a frame this app never saw ({:.2} s from the nearest)",
                (frame.stamp_s - masks.stamp_s).abs()
            ));
        }
        let model = reply["masks"]["model"].as_str().unwrap_or("service");
        Ok((frame, masks.instances, format!("{service} ({model})")))
    }
}

/// How far a frame's stamp may be from the masks' and still be the frame they were cut from.
const FRAME_MATCH: Duration = Duration::from_millis(20);

#[async_trait]
impl Tool for SegmentTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        self.run(&args).await.unwrap_or_else(ToolOutcome::failed)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use bytes::Bytes;
    use nervros_ros::fake::FakeRobot;

    use super::*;
    use crate::profile::LookConfig;
    use crate::tools::Status;

    #[test]
    fn an_outline_is_filled_and_its_entry_keeps_its_label() {
        // The lower half of a 100x50 frame, fenced as the models answer.
        let text = r#"```json
[{"box_2d": [500, 0, 1000, 1000], "label": "floor", "mask": [{"x": 0, "y": 500}, {"x": 1000, "y": 500}, {"y": 1000, "x": 1000}, {"x": 0, "y": 1000}]}]
```"#;
        let regions = parse_outlines(text, 100, 50).unwrap();
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].label, "floor");
        assert_eq!(regions[0].bbox, (0, 25, 100, 25));
        assert!(regions[0].mask.as_ref().unwrap().iter().all(|&m| m == 255));
    }

    #[test]
    fn entries_split_at_each_label_or_box_and_a_box_alone_is_kept() {
        let text = r#"[{"label": "ramp", "mask": [{"x": 0, "y": 0}, {"x": 1000, "y": 1000}, {"x": 0, "y": 1000}]},
                       {"box_2d": [500, 500, 1000, 1000], "label": "box only"},
                       {"box_2d": [0, 0, 100, 100], "label": "corner", "mask": [{"x": 0, "y": 0}, {"x": 100, "y": 0}, {"x": 100, "y": 100}, {"x": 0, "y": 100}]}]"#;
        let regions = parse_outlines(text, 200, 200).unwrap();
        let labels: Vec<&str> = regions.iter().map(|r| r.label.as_str()).collect();
        assert_eq!(labels, ["ramp", "box only", "corner"]);
        let pct = area_pct(&regions[0], 200, 200);
        assert!((pct - 50.0).abs() < 1.5, "{pct}");
        assert_eq!(regions[1].bbox, (100, 100, 100, 100));
        assert!(regions[1].mask.is_none());
        assert_eq!(regions[2].bbox, (0, 0, 20, 20));
    }

    #[test]
    fn a_garbled_point_is_dropped_and_the_rest_are_kept() {
        // As gemini-3.8-flash wrote it once: one point lost its opening brace and quote.
        let text = r#"[{"box_2d": [0, 0, 1000, 1000], "label": "floor", "mask": [{"x": 0, "y": 0}, {"x": 1000, "y": 0},": x": 1000, "y": 500}, {"x": 1000, "y": 1000}, {"x": 0, "y": 1000}]}"#;
        let regions = parse_outlines(text, 10, 10).unwrap();
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].bbox, (0, 0, 10, 10));
        assert!((area_pct(&regions[0], 10, 10) - 100.0).abs() < 0.1);
    }

    #[test]
    fn prose_says_so_and_an_empty_list_is_no_regions() {
        assert!(
            parse_outlines("I cannot see a floor.", 10, 10)
                .unwrap_err()
                .contains("no outlines")
        );
        assert!(
            parse_outlines("```json\n[]\n```", 10, 10)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn the_cutout_dims_what_is_outside_and_keeps_what_is_inside() {
        let frame = RgbImage::from_pixel(60, 60, Rgb([200, 100, 50]));
        let region = Instance {
            label: "floor".into(),
            score: 0.0,
            bbox: (10, 30, 40, 30),
            mask: None,
        };
        let img = draw(&frame, &[region], View::Cutout);
        assert_eq!(
            img.get_pixel(30, 45).0,
            [200, 100, 50],
            "inside keeps colour"
        );
        let [r, g, b] = img.get_pixel(5, 5).0;
        assert!(r == g && g == b && r < 60, "outside is dark grey");
        assert_eq!(img.get_pixel(10, 45).0, PALETTE[0], "the edge is outlined");
    }

    fn frame(stamp_s: f64) -> Frame {
        Frame {
            stamp_s,
            frame_id: "camera".into(),
            width: 64,
            height: 48,
            encoding: "rgb8".into(),
            step: 64 * 3,
            is_bigendian: false,
            data: Bytes::from(vec![90_u8; 64 * 48 * 3]),
        }
    }

    struct FakeOutliner(Mutex<Vec<String>>);

    #[async_trait]
    impl Outliner for FakeOutliner {
        async fn outline(
            &self,
            prompt: &str,
            image: ImageInput,
        ) -> Result<(String, String), String> {
            assert_eq!(&image.bytes[..2], &[0xFF, 0xD8], "a JPEG");
            self.0.lock().unwrap().push(prompt.to_owned());
            Ok((
                r#"[{"box_2d": [500, 0, 1000, 1000], "label": "floor", "mask": [{"x": 0, "y": 500}, {"x": 1000, "y": 500}, {"x": 1000, "y": 1000}, {"x": 0, "y": 1000}]}]"#.into(),
                "fake-gemini".into(),
            ))
        }
    }

    fn tool(robot: FakeRobot, service: bool, outliner: Option<Arc<dyn Outliner>>) -> SegmentTool {
        let robot: Arc<dyn RobotPort> = Arc::new(robot);
        let look: LookConfig = toml::from_str(
            "name = \"chest\"\nimage = \"/chest\"\ndetections = { topic = \"/masks\", type = \"canopy_msgs/msg/InstanceMaskArray\" }\n",
        )
        .unwrap();
        let config: SegmentConfig = toml::from_str(if service {
            "backend = \"service\"\nservice = \"/segmenter/segment\"\n"
        } else {
            ""
        })
        .unwrap();
        let cameras = Cameras::start(&look, &robot).unwrap();
        SegmentTool::new(
            config,
            cameras,
            robot,
            Arc::new(SnapshotStore::default()),
            outliner,
        )
    }

    #[tokio::test]
    async fn the_model_outlines_the_newest_frame_and_the_regions_become_marks() {
        let outliner = Arc::new(FakeOutliner(Mutex::new(Vec::new())));
        let segment = tool(
            FakeRobot::new().with_frame("/chest", frame(7.0)),
            false,
            Some(Arc::clone(&outliner) as Arc<dyn Outliner>),
        );
        assert!(
            segment.spec().parameters["properties"]
                .get("backend")
                .is_none()
        );
        tokio::task::yield_now().await;
        let out = segment.call(json!({"prompt": "the floor"})).await;
        assert_eq!(out.status, Status::Succeeded, "{}", out.message);
        assert_eq!(out.data["by"], "fake-gemini");
        assert_eq!(out.data["regions"][0]["label"], "floor");
        assert_eq!(out.data["regions"][0]["box"], json!([0, 24, 64, 24]));
        assert!((out.data["regions"][0]["area_pct"].as_f64().unwrap() - 50.0).abs() < 0.1);
        assert_eq!(&out.images[0].jpeg[..2], &[0xFF, 0xD8]);
        assert!(outliner.0.lock().unwrap()[0].contains("segmentation masks for the floor"));
        let empty = segment.call(json!({"prompt": "  "})).await;
        assert_eq!(empty.status, Status::Failed);
    }

    #[tokio::test]
    async fn the_service_answer_is_drawn_on_the_frame_it_was_cut_from() {
        let masks = json!({"success": true, "message": "", "masks": {
            "header": {"stamp": {"sec": 7, "nanosec": 0}}, "model": "grounded-sam2",
            "instances": [{"label": "floor", "score": 0.81, "roi": {"x_offset": 0, "y_offset": 40, "width": 4, "height": 2}, "data": [255, 255, 255, 255, 0, 0, 255, 255]}]}});
        let segment = tool(
            FakeRobot::new()
                .with_frame("/chest", frame(7.0))
                .with_service("/segmenter/segment", move |request| {
                    assert_eq!(request["camera"], "chest");
                    assert_eq!(request["prompt"], "the floor");
                    Ok(masks.clone())
                }),
            true,
            None,
        );
        tokio::task::yield_now().await;
        let out = segment.call(json!({"prompt": "the floor"})).await;
        assert_eq!(out.status, Status::Succeeded, "{}", out.message);
        assert_eq!(out.data["by"], "/segmenter/segment (grounded-sam2)");
        assert_eq!(out.data["regions"][0]["score"], 0.81);
        let refused = json!({"success": false, "message": "no frame from camera 'chest'"});
        let segment = tool(
            FakeRobot::new()
                .with_frame("/chest", frame(7.0))
                .with_service("/segmenter/segment", move |_| Ok(refused.clone())),
            true,
            None,
        );
        tokio::task::yield_now().await;
        let out = segment.call(json!({"prompt": "the floor"})).await;
        assert!(
            out.message.contains("no frame from camera"),
            "{}",
            out.message
        );
        let model = segment
            .call(json!({"prompt": "the floor", "backend": "model"}))
            .await;
        assert!(
            model.message.contains("no vision model"),
            "{}",
            model.message
        );
    }
}
