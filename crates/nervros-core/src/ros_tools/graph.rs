//! The graph tool, like `ros2 topic/service/action/node list` and `info`.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::GraphDetail;
use serde_json::{Value, json};

use super::{
    Ctx, LIST_LIMIT, actions, endpoint_json, failed, find_type, object, qos_matches, spec,
};
use crate::guard::{canonical_ros_name, glob_match};
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// `ros_graph`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(RosGraph {
        spec: spec(
            "ros_graph",
            "Look at the ROS graph, like ros2 topic/service/action/node list and info. Without \
             name: lists topics, services, actions or nodes, optionally filtered by a substring \
             or glob. With name: details of that topic (types, publishers, subscribers, QoS and \
             mismatches), node (what it publishes, subscribes to, serves, calls), service (type, \
             definition, reachable) or action (type, definition).",
            object(
                json!({
                    "kind": {"type": "string", "enum": ["topics", "services", "actions", "nodes"]},
                    "filter": {"type": "string", "description": "Substring or glob such as /canopy/*"},
                    "name": {"type": "string", "description": "An absolute name to describe"},
                    "all": {"type": "boolean", "description": "Include hidden names such as parameter services"},
                }),
                &[],
            ),
            Risk::Observe,
        ),
        ctx,
    })
}

struct RosGraph {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

impl RosGraph {
    fn list(&self, graph: &GraphDetail, args: &Value) -> ToolOutcome {
        let kind = args["kind"].as_str().unwrap_or("topics");
        let filter = args["filter"].as_str().unwrap_or_default();
        let all = args["all"].as_bool().unwrap_or(false);
        let items: Vec<(String, String)> = match kind {
            "topics" => graph
                .topics
                .iter()
                .map(|(n, t)| (n.clone(), t.join(", ")))
                .collect(),
            "services" => graph
                .services
                .iter()
                .map(|(n, t)| (n.clone(), t.join(", ")))
                .collect(),
            "actions" => actions(graph),
            "nodes" => graph
                .nodes
                .iter()
                .map(|n| (n.clone(), String::new()))
                .collect(),
            other => {
                return ToolOutcome::failed(format!(
                    "kind `{other}` is not one of topics, services, actions, nodes"
                ));
            }
        };
        let matching: Vec<&(String, String)> = items
            .iter()
            .filter(|(n, _)| all || !self.ctx.hidden(n))
            .filter(|(n, _)| {
                filter.is_empty()
                    || if filter.contains('*') {
                        glob_match(filter, n)
                    } else {
                        n.contains(filter)
                    }
            })
            .collect();
        let shown: Vec<Value> = matching
            .iter()
            .take(LIST_LIMIT)
            .map(|(n, t)| {
                if t.is_empty() {
                    json!(n)
                } else {
                    json!({"name": n, "type": t})
                }
            })
            .collect();
        ToolOutcome::ok(json!({
            "kind": kind,
            "total": matching.len(),
            "shown": shown.len(),
            "items": shown,
        }))
    }

    async fn topic(&self, graph: &GraphDetail, name: &str) -> ToolOutcome {
        let types = graph
            .topics
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, t)| t.clone())
            .unwrap_or_default();
        let ends = match self.ctx.robot.endpoints(name).await {
            Ok(e) => e,
            Err(e) => return failed(&e),
        };
        // The classic silent failure: a subscriber that asks for more than a publisher gives.
        let mismatched: Vec<Value> = ends
            .subscribers
            .iter()
            .flat_map(|s| {
                ends.publishers
                    .iter()
                    .filter(move |p| !qos_matches(p, s))
                    .map(move |p| json!({"publisher": p.node, "subscriber": s.node}))
            })
            .collect();
        let mut data = json!({
            "topic": name,
            "types": types,
            "publishers": ends.publishers.iter().map(endpoint_json).collect::<Vec<_>>(),
            "subscribers": ends.subscribers.iter().map(endpoint_json).collect::<Vec<_>>(),
        });
        if !mismatched.is_empty() {
            data["qos_mismatch"] = json!(mismatched);
            data["note"] = json!(
                "these subscribers ask for more (reliable or transient local) than the publishers \
                 give, so they receive nothing from them"
            );
        }
        ToolOutcome::ok(data)
    }

    async fn node(&self, name: &str) -> ToolOutcome {
        match self.ctx.robot.node_entities(name).await {
            Ok(n) => {
                let list = |items: &[(String, Vec<String>)]| {
                    items
                        .iter()
                        .filter(|(n, _)| !self.ctx.hidden(n))
                        .map(|(n, t)| json!({"name": n, "type": t.join(", ")}))
                        .collect::<Vec<_>>()
                };
                ToolOutcome::ok(json!({
                    "node": name,
                    "publishers": list(&n.publishers),
                    "subscribers": list(&n.subscribers),
                    "services": list(&n.services),
                    "clients": list(&n.clients),
                }))
            }
            Err(e) => failed(&e),
        }
    }

    async fn service(&self, name: &str, ty: &str) -> ToolOutcome {
        let available = self
            .ctx
            .robot
            .service_available(name, ty, Duration::from_secs(1))
            .await;
        ToolOutcome::ok(json!({
            "service": name,
            "type": ty,
            "reachable": available,
            "definition": self.ctx.schemas.show(ty),
        }))
    }
}

#[async_trait]
impl Tool for RosGraph {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let graph = match self.ctx.graph().await {
            Ok(g) => g,
            Err(out) => return out,
        };
        let Some(raw) = args["name"].as_str().filter(|n| !n.is_empty()) else {
            return self.list(&graph, &args);
        };
        let name = match canonical_ros_name(raw) {
            Ok(n) => n,
            Err(e) => return ToolOutcome::refused(e),
        };
        if graph.topics.iter().any(|(n, _)| *n == name) {
            return self.topic(&graph, &name).await;
        }
        if graph.nodes.contains(&name) {
            return self.node(&name).await;
        }
        if let Some(ty) = find_type(&graph.services, &name) {
            return self.service(&name, &ty).await;
        }
        if let Some((_, ty)) = actions(&graph).into_iter().find(|(n, _)| *n == name) {
            return ToolOutcome::ok(json!({
                "action": name,
                "type": ty,
                "definition": self.ctx.schemas.show(&ty),
            }));
        }
        ToolOutcome::failed(format!(
            "`{name}` is not a topic, node, service or action in the graph"
        ))
    }
}
