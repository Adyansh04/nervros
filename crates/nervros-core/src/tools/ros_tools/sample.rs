//! Sampling a topic, like `ros2 topic echo`, `hz` and `bw`.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::{
    BULK_TYPES, Ctx, MAX_ECHO, MAX_SAMPLE, count, endpoint_json, failed, name_arg, object, pick,
    seconds, spec, summarize,
};
use crate::guard::glob_match;
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// `topic_sample`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(TopicSample {
        spec: spec(
            "topic_sample",
            "Sample a topic, like ros2 topic echo/hz/bw: echo shows up to 10 messages (long \
             arrays and strings shortened; fields picks dotted fields); hz measures the rate and \
             its jitter; bw the bytes per second. Images and point clouds are too large to echo.",
            object(
                json!({
                    "topic": {"type": "string", "description": "Absolute topic name"},
                    "mode": {"type": "string", "enum": ["echo", "hz", "bw"]},
                    "seconds": {"type": "number", "description": "How long to listen, up to 10 (3)"},
                    "count": {"type": "integer", "description": "Messages to echo, up to 10 (1)"},
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

#[async_trait]
impl Tool for TopicSample {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let topic = match name_arg(&args, "topic") {
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
            "hz" | "bw" => {
                let arrivals = match self
                    .ctx
                    .robot
                    .sample_sizes(&topic, &ty, window, 10_000)
                    .await
                {
                    Ok(a) => a,
                    Err(e) => return failed(&e),
                };
                let mut data = rate_stats(&arrivals, window);
                data["topic"] = json!(topic);
                data["type"] = json!(ty);
                if arrivals.len() < 2
                    && let Ok(ends) = self.ctx.robot.endpoints(&topic).await
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
            "echo" => {
                if BULK_TYPES.contains(&ty.as_str()) {
                    return ToolOutcome::refused(format!(
                        "`{ty}` is too large to echo; use mode hz or bw, or look for images"
                    ));
                }
                let n = count(&args, "count", 1, MAX_ECHO);
                let messages = match self.ctx.robot.sample_messages(&topic, &ty, n, window).await {
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
                let shown: Vec<Value> = messages
                    .iter()
                    .map(|m| {
                        if fields.is_empty() {
                            summarize(m)
                        } else {
                            summarize(&pick(m, &fields))
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
            other => ToolOutcome::failed(format!("mode `{other}` is not one of echo, hz, bw")),
        }
    }
}
