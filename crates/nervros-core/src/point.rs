//! The `point` tool: a vision model points at what the operator names in a camera's newest frame,
//! including what no detector marks ("the fridge's handle", "the gap on the shelf"). A point that
//! lands on one of the detector's marks says so, so the agent can use that mark with other tools.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use image::{Rgb, RgbImage};
use nervros_ros::RobotPort;
use serde_json::{Value, json};

use crate::llm::{ImageFormat, ImageInput};
use crate::look::{
    Cameras, Instance, Snapshot, SnapshotStore, capped, draw_marks, marks_json, parse_detections,
};
use crate::segment::{Outliner, inside};
use crate::tools::{ImageArtifact, Risk, Tool, ToolOutcome, ToolSpec};

/// The longest side of the frame the model gets; points come back normalised.
const MODEL_EDGE_PX: u32 = 768;
/// Detections this far in time from the frame are of another moment.
const SAME_MOMENT: Duration = Duration::from_millis(500);
/// A point this near a mark's box centre, and inside no mark, is on that mark, px.
const SNAP_PX: f64 = 40.0;
const MAX_POINTS: usize = 5;

/// Gemini's pointing prompt: points as `[y, x]`, normalised to 0-1000.
fn point_prompt(what: &str) -> String {
    format!(
        "Point to {what}, at most {MAX_POINTS} points. Output a JSON list with one entry per \
         point: \"point\" as [y, x] normalised to 0-1000, and \"label\", what is there in a few \
         words. Output [] when it is not in the image."
    )
}

/// One point the model gave.
#[derive(Debug, Clone, PartialEq)]
pub struct Pointed {
    /// What it says is there.
    pub label: String,
    /// Its column in the frame, px.
    pub x: u32,
    /// Its row, px.
    pub y: u32,
}

/// The points in an answer, each `"point": [y, x]` with the `"label"` beside it.
#[must_use]
pub fn parse_points(text: &str, width: u32, height: u32) -> Vec<Pointed> {
    let px = |v: f64, size: u32| {
        let size = f64::from(size);
        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "clamped to the frame"
        )]
        let p = (v.clamp(0.0, 1000.0) / 1000.0 * size).min(size - 1.0) as u32;
        p
    };
    text.split('{')
        .skip(1)
        .filter_map(|entry| {
            let after = &entry[entry.find("\"point\"")? + "\"point\"".len()..];
            let open = after.find('[')?;
            // A `]` of the prose before the point's own `[` is not its end.
            let inner = &after[open + 1..open + after[open..].find(']')?];
            let mut numbers = inner.split(',').map(|n| n.trim().parse::<f64>().ok());
            let (y, x) = (numbers.next()??, numbers.next()??);
            let label = entry
                .find("\"label\"")
                .and_then(|at| entry[at + "\"label\"".len()..].split('"').nth(1))
                .unwrap_or_default()
                .to_owned();
            Some(Pointed {
                label,
                x: px(x, width),
                y: px(y, height),
            })
        })
        .take(MAX_POINTS)
        .collect()
}

/// The mark a point is on: one whose mask holds it, else one whose box centre is near.
fn mark_at(marks: &[Instance], x: u32, y: u32) -> Option<usize> {
    marks.iter().position(|m| inside(m, x, y)).or_else(|| {
        marks
            .iter()
            .enumerate()
            .map(|(i, m)| {
                let (bx, by, bw, bh) = m.bbox;
                let cx = f64::from(bx) + f64::from(bw) / 2.0;
                let cy = f64::from(by) + f64::from(bh) / 2.0;
                (i, (cx - f64::from(x)).hypot(cy - f64::from(y)))
            })
            .filter(|(_, d)| *d <= SNAP_PX)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
    })
}

/// A ring with a dot in it, white on black so it shows on any colour.
fn ring(img: &mut RgbImage, x: u32, y: u32) {
    let (cx, cy) = (i64::from(x), i64::from(y));
    for dy in -11_i64..=11 {
        for dx in -11_i64..=11 {
            let d = dx * dx + dy * dy;
            let colour = match d {
                0..=6 | 64..=100 => Rgb([255, 255, 255]),
                101..=121 | 49..=63 => Rgb([0, 0, 0]),
                _ => continue,
            };
            if let (Ok(px), Ok(py)) = (u32::try_from(cx + dx), u32::try_from(cy + dy))
                && px < img.width()
                && py < img.height()
            {
                img.put_pixel(px, py, colour);
            }
        }
    }
}

/// The `point` tool.
pub struct PointTool {
    spec: ToolSpec,
    cameras: Arc<Cameras>,
    robot: Arc<dyn RobotPort>,
    snapshots: Arc<SnapshotStore>,
    pointer: Arc<dyn Outliner>,
}

impl PointTool {
    /// `point` over the given cameras, asking `pointer` (the `segment` role's models).
    #[must_use]
    pub fn new(
        cameras: Arc<Cameras>,
        robot: Arc<dyn RobotPort>,
        snapshots: Arc<SnapshotStore>,
        pointer: Arc<dyn Outliner>,
    ) -> Self {
        let mut parameters = json!({"type": "object", "properties": {
            "what": {"type": "string", "description": "What to point at, such as \"the fridge's handle\" or \"a free spot on the table\"."}
        }, "required": ["what"], "additionalProperties": false});
        cameras.add_argument(&mut parameters);
        let spec = ToolSpec::new(
            "point",
            "Points at what `what` names in a camera's newest frame, including things no \
             detector marks. The operator sees the points; you get each point's pixel and, when \
             it lands on one of the detector's marks, that mark, to use as with look.",
            parameters,
            Risk::Observe,
        );
        Self {
            spec,
            cameras,
            robot,
            snapshots,
            pointer,
        }
    }

    async fn run(&self, args: &Value) -> Result<ToolOutcome, String> {
        let what = args["what"].as_str().map(str::trim).unwrap_or_default();
        if what.is_empty() {
            return Err("say what to point at in `what`".to_owned());
        }
        let camera = self.cameras.get(args["camera"].as_str())?;
        let frame = camera.newest()?;
        let img = frame.to_rgb().map_err(|e| e.to_string())?;
        let marks = match &camera.config.detections {
            Some(topic) => {
                let msg = self
                    .robot
                    .latest(&topic.topic, &topic.msg_type, SAME_MOMENT)
                    .await
                    .unwrap_or(Value::Null);
                let dets = parse_detections(&msg);
                if (dets.stamp_s - frame.stamp_s).abs() <= SAME_MOMENT.as_secs_f64() {
                    dets.instances
                } else {
                    Vec::new()
                }
            }
            None => Vec::new(),
        };
        let small = capped(&img, MODEL_EDGE_PX);
        let image = ImageInput {
            bytes: nervros_ros::image::encode_jpeg(&small, 85).map_err(|e| e.to_string())?,
            format: ImageFormat::Jpeg,
        };
        let (answer, model) = self.pointer.outline(&point_prompt(what), image).await?;
        tracing::debug!(%model, %answer, "points");
        let points = parse_points(&answer, img.width(), img.height());
        let mut drawn = img;
        draw_marks(&mut drawn, &marks);
        let listed: Vec<Value> = points
            .iter()
            .enumerate()
            .map(|(i, p)| {
                ring(&mut drawn, p.x, p.y);
                let mut entry = json!({"point": i + 1, "label": p.label, "px": [p.x, p.y]});
                if let Some(m) = mark_at(&marks, p.x, p.y) {
                    entry["mark"] = json!(m + 1);
                    entry["mark_label"] = json!(marks[m].label);
                }
                entry
            })
            .collect();
        let jpeg = nervros_ros::image::encode_jpeg(&drawn, 85).map_err(|e| e.to_string())?;
        let image = ImageArtifact {
            snapshot: self.snapshots.next_id(),
            jpeg: Arc::new(jpeg),
            width: drawn.width(),
            height: drawn.height(),
            marks: marks.iter().map(|m| m.label.clone()).collect(),
        };
        let marks_listed = marks_json(&marks);
        let snapshot = self.snapshots.put(Snapshot {
            id: image.snapshot.clone(),
            stamp_s: frame.stamp_s,
            marks,
            image: image.clone(),
        });
        let mut out = ToolOutcome::ok(json!({"snapshot": snapshot.id, "camera": camera.name,
            "points": listed, "marks": marks_listed, "by": model}));
        out.message = if listed.is_empty() {
            format!("the model found no {what}; the user sees the frame")
        } else {
            format!("{} point(s); the user sees them as rings", listed.len())
        };
        out.images.push(image);
        Ok(out)
    }
}

#[async_trait]
impl Tool for PointTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        self.run(&args).await.unwrap_or_else(ToolOutcome::failed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn points_are_read_as_y_then_x_with_their_labels() {
        let answer = r#"```json
[{"point": [500, 250], "label": "fridge handle"}, {"label": "gap", "point": [1000, 1000]},
 {"point": [12], "label": "garbled"}]
```"#;
        assert_eq!(
            parse_points(answer, 800, 600),
            [
                Pointed {
                    label: "fridge handle".to_owned(),
                    x: 200,
                    y: 300
                },
                Pointed {
                    label: "gap".to_owned(),
                    x: 799,
                    y: 599
                },
            ]
        );
        assert!(parse_points("[]", 800, 600).is_empty());
    }

    #[test]
    fn a_bracket_in_the_prose_before_a_point_is_no_end_to_it() {
        let answer = "[{\"label\": \"handle\", \"point\": null}\n]\nNote: coordinates are [y, x]";
        assert!(parse_points(answer, 800, 600).is_empty());
        let answer = "[{\"point\": 536, 270], \"label\": \"box [left]\"}]";
        assert!(parse_points(answer, 800, 600).is_empty());
    }

    #[test]
    fn a_point_is_on_the_mark_that_holds_it_or_a_near_one() {
        let mark = |x, y, w, h| Instance {
            label: "mug".to_owned(),
            score: 0.9,
            bbox: (x, y, w, h),
            mask: None,
        };
        let marks = [mark(100, 100, 50, 50), mark(300, 100, 20, 20)];
        assert_eq!(mark_at(&marks, 120, 120), Some(0));
        assert_eq!(mark_at(&marks, 330, 115), Some(1), "beside the small one");
        assert_eq!(mark_at(&marks, 600, 400), None);
    }
}
