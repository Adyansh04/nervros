//! The `background` tool: what runs behind the conversation, whichever tool started it, and ending
//! it. Each kind keeps its own registry; their ids tell them apart.

use std::borrow::Cow;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// A registry of things that run in the background.
pub trait Background: Send + Sync {
    /// What it holds, each with its `id` and `kind`.
    fn list(&self) -> Vec<Value>;
    /// Ends one by id, or `all`; whether any ended.
    fn end(&self, id: &str) -> bool;
}

/// The `background` tool.
pub struct BackgroundTool {
    spec: ToolSpec,
    registries: Vec<Arc<dyn Background>>,
}

impl BackgroundTool {
    /// Over these registries.
    #[must_use]
    pub fn new(registries: Vec<Arc<dyn Background>>) -> Self {
        let spec = ToolSpec::new(
            "background",
            "What runs in the background: watches, schedules, and program, search and patrol \
             jobs, recently ended jobs with their results too. list shows them; cancel ends one \
             by id, such as w2, p1 or j3, or all of them.",
            json!({"type": "object", "properties": {
                "action": {"type": "string", "enum": ["list", "cancel"]},
                "id": {"type": "string", "description": "For cancel: the id, or all."}
            }, "required": ["action"], "additionalProperties": false}),
            Risk::Observe,
        );
        Self { spec, registries }
    }
}

#[async_trait]
impl Tool for BackgroundTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str() {
            Some("list") => {
                let running: Vec<Value> = self.registries.iter().flat_map(|r| r.list()).collect();
                ToolOutcome::ok(json!({"running": running}))
            }
            Some("cancel") => {
                let id = args["id"].as_str().unwrap_or("all");
                // Each registry is asked: `all` reaches every one, and an id only its own.
                let ended = self.registries.iter().filter(|r| r.end(id)).count() > 0;
                // Ending all of none is done, not a mistake.
                if ended || id == "all" {
                    ToolOutcome::ok(json!({"cancelled": id}))
                } else {
                    ToolOutcome::failed(format!("nothing runs as `{id}`; list them first"))
                }
            }
            _ => ToolOutcome::failed("`action` is list or cancel"),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use super::*;
    use crate::tools::Status;

    struct Fake(&'static str, Mutex<bool>);

    impl Background for Fake {
        fn list(&self) -> Vec<Value> {
            if *crate::lock(&self.1) {
                vec![json!({"id": self.0})]
            } else {
                Vec::new()
            }
        }
        fn end(&self, id: &str) -> bool {
            let mut running = crate::lock(&self.1);
            let hit = *running && (id == "all" || id == self.0);
            *running &= !hit;
            hit
        }
    }

    #[tokio::test]
    async fn it_lists_and_ends_across_registries() {
        let tool = BackgroundTool::new(vec![
            Arc::new(Fake("w1", Mutex::new(true))),
            Arc::new(Fake("j1", Mutex::new(true))),
        ]);
        let list = || tool.call(json!({"action": "list"}));
        let cancel = |id: &'static str| tool.call(json!({"action": "cancel", "id": id}));
        assert_eq!(list().await.data["running"].as_array().unwrap().len(), 2);
        assert_eq!(cancel("x9").await.status, Status::Failed);
        assert_eq!(cancel("j1").await.status, Status::Succeeded);
        assert_eq!(list().await.data["running"], json!([{"id": "w1"}]));
        assert_eq!(cancel("all").await.status, Status::Succeeded);
        assert_eq!(list().await.data["running"], json!([]));
        assert_eq!(cancel("all").await.status, Status::Succeeded, "all of none");
    }
}
