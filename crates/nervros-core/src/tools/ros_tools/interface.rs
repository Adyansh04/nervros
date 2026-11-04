//! Interface definitions, like `ros2 interface show`, `list`, `packages` and `package`.

use std::borrow::Cow;
use std::collections::BTreeSet;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::{Ctx, LIST_LIMIT, object, spec};
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// `interface_show`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(InterfaceShow {
        spec: spec(
            "interface_show",
            "Show a message, service or action definition, like ros2 interface show. Without \
             type, lists the interfaces this agent knows, like ros2 interface list, narrowed by \
             package or filter, or the packages when neither is given.",
            object(
                json!({
                    "type": {"type": "string", "description": "pkg/msg/Name, pkg/srv/Name or pkg/action/Name"},
                    "package": {"type": "string", "description": "Without type: this package's interfaces"},
                    "filter": {"type": "string", "description": "Without type: names holding this"},
                }),
                &[],
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

impl InterfaceShow {
    /// The interfaces known, or with nothing to narrow them, their packages.
    fn list(&self, args: &Value) -> ToolOutcome {
        let names = self.ctx.schemas.names();
        let (package, filter) = (
            args["package"].as_str().unwrap_or_default(),
            args["filter"].as_str().unwrap_or_default(),
        );
        if package.is_empty() && filter.is_empty() {
            let packages: BTreeSet<&str> =
                names.iter().filter_map(|n| n.split('/').next()).collect();
            return ToolOutcome::ok(json!({"interfaces": names.len(), "packages": packages}));
        }
        let matching: Vec<&String> = names
            .iter()
            .filter(|n| package.is_empty() || n.split('/').next() == Some(package))
            .filter(|n| n.contains(filter))
            .collect();
        ToolOutcome::ok(json!({
            "total": matching.len(),
            "interfaces": matching.iter().take(LIST_LIMIT).collect::<Vec<_>>(),
        }))
    }
}

#[async_trait]
impl Tool for InterfaceShow {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let Some(ty) = args["type"].as_str().filter(|t| !t.is_empty()) else {
            return self.list(&args);
        };
        match self.ctx.schemas.show(ty) {
            Some(text) => ToolOutcome::ok(json!({"type": ty, "definition": text})),
            None => ToolOutcome::failed(format!(
                "`{ty}` is not an interface this agent knows; give pkg/msg/Name, pkg/srv/Name or \
                 pkg/action/Name"
            )),
        }
    }
}
