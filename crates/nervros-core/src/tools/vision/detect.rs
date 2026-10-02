//! Detections: what the detector marked on a frame, read from its JSON.

use serde_json::Value;

/// How far a frame's stamp may be from the detections' and still be the frame they came from.
pub(crate) const DETECTION_MATCH_S: f64 = 0.05;

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
