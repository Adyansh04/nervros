//! Interface definitions, like `ros2 interface show`.

use std::borrow::Cow;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::{Ctx, object, spec};
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// `interface_show`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(InterfaceShow {
        spec: spec(
            "interface_show",
            "Show a message, service or action definition, like ros2 interface show.",
            object(
                json!({"type": {"type": "string", "description": "pkg/msg/Name, pkg/srv/Name or pkg/action/Name"}}),
                &["type"],
            ),
            Risk::Observe,
        ),
        ctx,
    })
}

struct InterfaceShow {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

#[async_trait]
impl Tool for InterfaceShow {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let ty = args["type"].as_str().unwrap_or_default();
        match self.ctx.schemas.show(ty) {
            Some(text) => ToolOutcome::ok(json!({"type": ty, "definition": text})),
            None => ToolOutcome::failed(format!(
                "`{ty}` is not an interface this agent knows; give pkg/msg/Name, pkg/srv/Name or \
                 pkg/action/Name"
            )),
        }
    }
}
