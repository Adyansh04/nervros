//! TF lookups and the frame tree.

use std::borrow::Cow;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::{Ctx, failed, object, spec};
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// `tf`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(Tf {
        spec: spec(
            "tf",
            "TF: lookup gives where the source frame is in the target frame (metres, degrees); \
             tree lists every frame link with its parent, whether it is static and how old it is.",
            object(
                json!({
                    "op": {"type": "string", "enum": ["lookup", "tree"]},
                    "target": {"type": "string", "description": "Frame to express the pose in (map)"},
                    "source": {"type": "string", "description": "Frame to locate"},
                }),
                &[],
            ),
            Risk::Observe,
        ),
        ctx,
    })
}

struct Tf {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

#[async_trait]
impl Tool for Tf {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let round = |x: f64| (x * 1000.0).round() / 1000.0;
        if args["op"].as_str() == Some("tree") {
            let links: Vec<Value> = self
                .ctx
                .robot
                .tf_links()
                .iter()
                .map(|l| {
                    json!({"parent": l.parent, "child": l.child, "static": l.is_static,
                           "age_s": round(l.age.as_secs_f64())})
                })
                .collect();
            return ToolOutcome::ok(json!({"frames": links.len(), "links": links}));
        }
        let target = args["target"].as_str().unwrap_or("map");
        let Some(source) = args["source"].as_str() else {
            return ToolOutcome::failed("a lookup needs source, the frame to locate");
        };
        match self.ctx.robot.transform(target, source) {
            Ok(t) => ToolOutcome::ok(json!({
                "target": target,
                "source": source,
                "translation_m": t.translation.map(round),
                "rotation_xyzw": t.rotation.map(round),
                "yaw_deg": round(t.yaw().to_degrees()),
            })),
            Err(e) => failed(&e),
        }
    }
}
