//! Recent `/rosout` lines.

use std::borrow::Cow;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::{Ctx, MAX_SAMPLE, count, failed, object, seconds, spec};
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// `log_tail`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(LogTail {
        spec: spec(
            "log_tail",
            "Recent ROS log lines from /rosout, newest last: warnings and errors by default.",
            object(
                json!({
                    "min_level": {"type": "string", "enum": ["debug", "info", "warn", "error", "fatal"]},
                    "node": {"type": "string", "description": "Only nodes whose name contains this"},
                    "seconds": {"type": "number", "description": "How long to listen, up to 10 (2)"},
                    "max": {"type": "integer", "description": "Most lines to return (30)"},
                }),
                &[],
            ),
            Risk::Observe,
        ),
        ctx,
    })
}

struct LogTail {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

/// A `/rosout` level by name, as `rcl_interfaces/msg/Log` numbers it; empty is `warn`.
pub(crate) fn level_number(word: &str) -> Option<u8> {
    match word {
        "debug" => Some(10),
        "info" => Some(20),
        "" | "warn" => Some(30),
        "error" => Some(40),
        "fatal" => Some(50),
        _ => None,
    }
}

/// A `/rosout` level's name.
pub(crate) fn level_word(n: u64) -> &'static str {
    match n {
        0..=10 => "DEBUG",
        11..=20 => "INFO",
        21..=30 => "WARN",
        31..=40 => "ERROR",
        _ => "FATAL",
    }
}

#[async_trait]
impl Tool for LogTail {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let min =
            level_number(args["min_level"].as_str().unwrap_or_default()).map_or(30, u64::from);
        let node = args["node"]
            .as_str()
            .unwrap_or_default()
            .trim_start_matches('/')
            .to_owned();
        let window = seconds(&args, "seconds", 2.0, MAX_SAMPLE);
        let max = count(&args, "max", 30, 100);
        // /rosout keeps recent history for late subscribers, so this sees the last few seconds too.
        let logs = match self
            .ctx
            .robot
            .sample_messages("/rosout", "rcl_interfaces/msg/Log", 500, window)
            .await
        {
            Ok(l) => l,
            Err(e) => return failed(&e),
        };
        let lines: Vec<String> = logs
            .iter()
            .filter(|l| l["level"].as_u64().unwrap_or(0) >= min)
            .filter(|l| node.is_empty() || l["name"].as_str().unwrap_or_default().contains(&node))
            .map(|l| {
                let text: String = l["msg"]
                    .as_str()
                    .unwrap_or_default()
                    .chars()
                    .take(300)
                    .collect();
                format!(
                    "[{}] {}: {text}",
                    level_word(l["level"].as_u64().unwrap_or(0)),
                    l["name"].as_str().unwrap_or("?")
                )
            })
            .collect();
        let skip = lines.len().saturating_sub(max);
        ToolOutcome::ok(json!({"lines": lines[skip..], "matched": lines.len()}))
    }
}
