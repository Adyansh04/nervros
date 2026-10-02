//! Publishing a few messages, like `ros2 topic pub --times`.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::{
    Ctx, MAX_PUBLISH, MAX_PUBLISH_HZ, Prepared, count, failed, name_arg, object, payload, spec,
};
use crate::guard::glob_match;
use crate::tools::{Assessment, Resource, Risk, SchemaPart, Tool, ToolOutcome, ToolSpec};

/// `topic_publish`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(TopicPublish {
        spec: spec(
            "topic_publish",
            "Publish a message on a topic a few times, like ros2 topic pub --times. Velocity, \
             joint and low-level command topics are refused: the robot moves through its skills. \
             Needs the robot armed and the operator's approval.",
            object(
                json!({
                    "topic": {"type": "string", "description": "Absolute topic name"},
                    "type": {"type": "string", "description": "pkg/msg/Name; needed for a topic not in the graph"},
                    "message": {"type": "object", "description": "The message fields as JSON"},
                    "count": {"type": "integer", "description": "How many times, up to 10 (1)"},
                    "rate_hz": {"type": "number", "description": "Up to 10 (1)"},
                }),
                &["topic", "message"],
            ),
            Risk::Motion,
        ),
        ctx,
    })
}

struct TopicPublish {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

impl TopicPublish {
    async fn prepare(&self, args: &Value) -> Result<Prepared, ToolOutcome> {
        let topic = name_arg(args, "topic").map_err(ToolOutcome::refused)?;
        if !self
            .ctx
            .config
            .publish
            .iter()
            .any(|p| glob_match(p, &topic))
        {
            return Err(ToolOutcome::refused(format!(
                "the profile does not let this agent publish on `{topic}`"
            )));
        }
        let ty = match args["type"].as_str().filter(|t| !t.is_empty()) {
            Some(t) => t.to_owned(),
            None => self.ctx.topic_type(&topic).await.map_err(|_| {
                ToolOutcome::failed(format!(
                    "`{topic}` is not in the graph yet; give its type, such as std_msgs/msg/String"
                ))
            })?,
        };
        if let Some(out) = self.ctx.deny_act(&topic, &ty) {
            return Err(out);
        }
        let message = payload(args, "message");
        self.ctx
            .schemas
            .validate(&ty, SchemaPart::Message, &message)
            .map_err(|why| ToolOutcome::failed(format!("the message does not fit {ty}: {why}")))?;
        Ok(Prepared {
            name: topic,
            ty,
            payload: message,
        })
    }
}

#[async_trait]
impl Tool for TopicPublish {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let times = count(args, "count", 1, MAX_PUBLISH);
        Some(self.prepare(args).await.map(|p| Assessment {
            risk: Risk::Motion,
            resources: Resource::ALL.to_vec(),
            args: None,
            reason: format!(
                "publishes {} on {} ({}) {times} time(s)",
                p.payload, p.name, p.ty
            ),
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let p = match self.prepare(&args).await {
            Ok(p) => p,
            Err(out) => return out,
        };
        let times = count(&args, "count", 1, MAX_PUBLISH);
        let hz = args["rate_hz"]
            .as_f64()
            .unwrap_or(1.0)
            .clamp(0.1, MAX_PUBLISH_HZ);
        match self
            .ctx
            .robot
            .publish(
                &p.name,
                &p.ty,
                p.payload,
                times,
                Duration::from_secs_f64(1.0 / hz),
            )
            .await
        {
            Ok(matched) => ToolOutcome::ok(json!({
                "topic": p.name,
                "type": p.ty,
                "published": times,
                "subscribers": matched,
            })),
            Err(e) => failed(&e),
        }
    }
}
