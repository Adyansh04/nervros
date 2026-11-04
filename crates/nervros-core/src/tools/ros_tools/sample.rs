//! Sampling a topic, like `ros2 topic echo`, `hz`, `bw` and `delay`.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use futures::StreamExt as _;
use serde_json::{Value, json};

use super::{
    BULK_TYPES, Ctx, FULL_CHARS, FULL_ITEMS, MAX_ECHO, MAX_SAMPLE, count, endpoint_json, failed,
    object, pick, read_name_arg, resolved_view, seconds, shortened, spec, summarize,
};
use crate::guard::glob_match;
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// `topic_sample`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(TopicSample {
        spec: spec(
            "topic_sample",
            "Sample a topic, like ros2 topic echo/hz/bw/delay: echo shows up to 10 messages (long \
             arrays and strings shortened unless full is set; fields picks dotted fields); hz \
             measures the rate and its jitter; bw the bytes per second; delay how old each \
             message is on arrival, from its header's stamp. Images and point clouds are too \
             large to echo.",
            object(
                json!({
                    "topic": {"type": "string", "description": "Absolute topic name"},
                    "mode": {"type": "string", "enum": ["echo", "hz", "bw", "delay"]},
                    "seconds": {"type": "number", "description": "How long to listen, up to 10 (3)"},
                    "count": {"type": "integer", "description": "Messages to echo, up to 10 (1)"},
                    "full": {"type": "boolean", "description": "For echo: arrays and strings whole, as far as a reply holds"},
                    "fields": {"type": "array", "items": {"type": "string"}, "description": "Only these fields, e.g. header.stamp"},
                }),
                &["topic"],
            ),
            Risk::Observe,
        ),
        ctx,
    })
}

struct TopicSample {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

fn rate_stats(arrivals: &[(Duration, usize)], window: Duration) -> Value {
    let n = arrivals.len();
    let bytes: usize = arrivals.iter().map(|(_, b)| b).sum();
    #[expect(
        clippy::cast_precision_loss,
        reason = "counts and sizes far below 2^52"
    )]
    let (n_f, bytes_f) = (n as f64, bytes as f64);
    let mut out = json!({
        "messages": n,
        "window_s": window.as_secs_f64(),
        "bytes_per_s": (bytes_f / window.as_secs_f64()).round(),
        "mean_size_bytes": if n > 0 { (bytes_f / n_f).round() } else { 0.0 },
    });
    if n >= 2 {
        let gaps: Vec<f64> = arrivals
            .windows(2)
            .map(|w| w[1].0.saturating_sub(w[0].0).as_secs_f64())
            .collect();
        #[expect(clippy::cast_precision_loss, reason = "a few thousand gaps at most")]
        let mean = gaps.iter().sum::<f64>() / gaps.len() as f64;
        #[expect(clippy::cast_precision_loss, reason = "a few thousand gaps at most")]
        let var = gaps.iter().map(|g| (g - mean).powi(2)).sum::<f64>() / gaps.len() as f64;
        let round = |x: f64| (x * 1000.0).round() / 1000.0;
        out["rate_hz"] = json!(round(1.0 / mean));
        out["gap_min_s"] = json!(round(gaps.iter().copied().fold(f64::INFINITY, f64::min)));
        out["gap_max_s"] = json!(round(gaps.iter().copied().fold(0.0, f64::max)));
        out["gap_std_s"] = json!(round(var.sqrt()));
    }
    out
}

/// A header's stamp read from a serialized message that opens with one, as every bulk type does:
/// their messages are too large to decode just for that.
fn raw_stamp(bytes: &[u8]) -> Option<f64> {
    let sec = <[u8; 4]>::try_from(bytes.get(4..8)?).ok()?;
    let nanosec = <[u8; 4]>::try_from(bytes.get(8..12)?).ok()?;
    let (sec, nanosec) = if bytes[1] & 1 == 1 {
        (i32::from_le_bytes(sec), u32::from_le_bytes(nanosec))
    } else {
        (i32::from_be_bytes(sec), u32::from_be_bytes(nanosec))
    };
    Some(f64::from(sec) + f64::from(nanosec) * 1e-9)
}

/// How old messages were when they arrived, as `ros2 topic delay` says.
fn delay_stats(delays: &[f64]) -> Value {
    #[expect(
        clippy::cast_precision_loss,
        reason = "a few thousand messages at most"
    )]
    let n = delays.len() as f64;
    let mean = delays.iter().sum::<f64>() / n;
    let var = delays.iter().map(|d| (d - mean).powi(2)).sum::<f64>() / n;
    let round = |x: f64| (x * 1000.0).round() / 1000.0;
    let mut out = json!({
        "messages": delays.len(),
        "delay_s": round(mean),
        "min_s": round(delays.iter().copied().fold(f64::INFINITY, f64::min)),
        "max_s": round(delays.iter().copied().fold(f64::NEG_INFINITY, f64::max)),
        "std_s": round(var.sqrt()),
    });
    // A day apart is not lateness: the publisher runs on another clock, such as simulated time.
    if mean.abs() > 86_400.0 {
        out["note"] =
            json!("the stamps are on another clock than this machine's, such as simulated time");
    }
    out
}

impl TopicSample {
    /// The rate and the bytes per second over `window`.
    async fn rate(&self, topic: &str, ty: &str, window: Duration) -> ToolOutcome {
        let arrivals = match self.ctx.robot.sample_sizes(topic, ty, window, 10_000).await {
            Ok(a) => a,
            Err(e) => return failed(&e),
        };
        let mut data = rate_stats(&arrivals, window);
        data["topic"] = json!(topic);
        data["type"] = json!(ty);
        if arrivals.len() < 2
            && let Ok(ends) = self.ctx.robot.endpoints(topic).await
        {
            data["publishers"] = json!(
                ends.publishers
                    .iter()
                    .map(endpoint_json)
                    .collect::<Vec<_>>()
            );
            data["note"] = json!(if ends.publishers.is_empty() {
                "nobody publishes on this topic"
            } else {
                "the publishers sent next to nothing in the window"
            });
        }
        ToolOutcome::ok(data)
    }

    /// How old messages are on arrival over `window`.
    async fn delay(&self, topic: &str, ty: &str, window: Duration) -> ToolOutcome {
        let delays = match self.delays(topic, ty, window).await {
            Ok(d) => d,
            Err(out) => return out,
        };
        if delays.is_empty() {
            return ToolOutcome::failed(format!(
                "no message on `{topic}` within {:.1} s",
                window.as_secs_f64()
            ));
        }
        let mut data = delay_stats(&delays);
        data["topic"] = json!(topic);
        data["type"] = json!(ty);
        ToolOutcome::ok(data)
    }

    /// Up to `count` messages, shortened unless asked for in full.
    async fn echo(&self, topic: &str, ty: &str, window: Duration, args: &Value) -> ToolOutcome {
        if BULK_TYPES.contains(&ty) {
            return ToolOutcome::refused(format!(
                "`{ty}` is too large to echo; use mode hz or bw, or look for images"
            ));
        }
        let n = count(args, "count", 1, MAX_ECHO);
        let messages = match self.ctx.robot.sample_messages(topic, ty, n, window).await {
            Ok(m) => m,
            Err(e) => return failed(&e),
        };
        let fields: Vec<String> = args["fields"]
            .as_array()
            .map(|f| {
                f.iter()
                    .filter_map(|v| v.as_str().map(str::to_owned))
                    .collect()
            })
            .unwrap_or_default();
        let full = args["full"].as_bool().unwrap_or(false);
        let cut = |m: &Value| {
            if full {
                shortened(m, FULL_ITEMS, FULL_CHARS)
            } else {
                summarize(m)
            }
        };
        let shown: Vec<Value> = messages
            .iter()
            .map(|m| {
                if fields.is_empty() {
                    cut(m)
                } else {
                    cut(&pick(m, &fields))
                }
            })
            .collect();
        if shown.is_empty() {
            return ToolOutcome::failed(format!(
                "no message on `{topic}` within {:.1} s",
                window.as_secs_f64()
            ));
        }
        ToolOutcome::ok(json!({"topic": topic, "type": ty, "messages": shown}))
    }

    /// The age of each message on `topic` over `window`.
    async fn delays(
        &self,
        topic: &str,
        ty: &str,
        window: Duration,
    ) -> Result<Vec<f64>, ToolOutcome> {
        let stamps: Vec<Option<f64>> = if BULK_TYPES.contains(&ty) {
            let mut raw = self
                .ctx
                .robot
                .raw(topic, ty)
                .await
                .map_err(|e| failed(&e))?;
            let end = tokio::time::Instant::now() + window;
            let mut stamps = Vec::new();
            while let Ok(Some(bytes)) = tokio::time::timeout_at(end, raw.next()).await {
                stamps.push(raw_stamp(&bytes));
            }
            stamps
        } else {
            self.ctx
                .robot
                .sample_messages(topic, ty, 10_000, window)
                .await
                .map_err(|e| failed(&e))?
                .iter()
                .map(|m| {
                    let stamp = &m["header"]["stamp"];
                    stamp.is_object().then(|| crate::stamp_s(stamp))
                })
                .collect()
        };
        if stamps.iter().any(Option::is_none) {
            return Err(ToolOutcome::failed(format!(
                "`{ty}` has no header.stamp, which delay reads"
            )));
        }
        let now = nervros_ros::now_s();
        Ok(stamps.into_iter().flatten().map(|s| now - s).collect())
    }
}

#[async_trait]
impl Tool for TopicSample {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    fn rule_view(&self, args: &Value) -> Option<Value> {
        resolved_view(args, "topic")
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let topic = match read_name_arg(&args, "topic") {
            Ok(t) => t,
            Err(why) => return ToolOutcome::refused(why),
        };
        if self
            .ctx
            .config
            .read_deny
            .iter()
            .any(|p| glob_match(p, &topic))
        {
            return ToolOutcome::refused(format!("`{topic}` may not be read by this agent"));
        }
        let ty = match self.ctx.topic_type(&topic).await {
            Ok(t) => t,
            Err(out) => return out,
        };
        let mode = args["mode"].as_str().unwrap_or("echo");
        let window = seconds(&args, "seconds", 3.0, MAX_SAMPLE);
        match mode {
            "hz" | "bw" => self.rate(&topic, &ty, window).await,
            "delay" => self.delay(&topic, &ty, window).await,
            "echo" => self.echo(&topic, &ty, window, &args).await,
            other => {
                ToolOutcome::failed(format!("mode `{other}` is not one of echo, hz, bw, delay"))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_raw_stamp_reads_either_byte_order() {
        let mut little = vec![0, 1, 0, 0];
        little.extend(5i32.to_le_bytes());
        little.extend(500_000_000u32.to_le_bytes());
        assert_eq!(raw_stamp(&little), Some(5.5));
        let mut big = vec![0, 0, 0, 0];
        big.extend(5i32.to_be_bytes());
        big.extend(500_000_000u32.to_be_bytes());
        assert_eq!(raw_stamp(&big), Some(5.5));
        assert_eq!(raw_stamp(&[0, 1, 0]), None);
    }

    #[test]
    fn a_delay_says_how_old_and_when_another_clock_makes_it_meaningless() {
        let near = delay_stats(&[0.1, 0.3]);
        assert_eq!(near["delay_s"], json!(0.2));
        assert_eq!(near["min_s"], json!(0.1));
        assert_eq!(near["max_s"], json!(0.3));
        assert!(near.get("note").is_none());
        assert!(delay_stats(&[1.7e9]).get("note").is_some());
    }
}
