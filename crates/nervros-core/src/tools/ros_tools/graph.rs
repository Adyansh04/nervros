//! The graph tool, like `ros2 topic/service/action/node list` and `info`.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::GraphDetail;
use serde_json::{Value, json};

use super::{
    Ctx, LIST_LIMIT, actions, endpoint_json, failed, find_type, object, qos_matches, read_name,
    resolved_view, spec,
};
use crate::guard::glob_match;
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// `ros_graph`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(RosGraph {
        spec: spec(
            "ros_graph",
            "Look at the ROS graph, like ros2 topic/service/action/node list and info. Without \
             name: lists topics, services, actions or nodes, optionally filtered by a substring \
             or glob of the name. A filter sees names only: a stack's nodes rarely share one \
             (Nav2's are controller_server, planner_server, bt_navigator, amcl and the like), so \
             list them unfiltered. A component container in a node list or node details also \
             shows the nodes it hosts. type narrows a list to one interface, as ros2 topic find \
             does; counts adds each topic's publishers and subscribers. kind lifecycle lists the \
             managed nodes and their states. With name: details of that topic (types, publishers, \
             subscribers, full QoS and mismatches), node (what it publishes, subscribes to, \
             serves, calls, its action servers and clients, its lifecycle state and transitions), \
             service (type, definition, reachable, its servers and clients) or action (type, \
             definition, its servers and clients).",
            object(
                json!({
                    "kind": {"type": "string", "enum": ["topics", "services", "actions", "nodes", "lifecycle"]},
                    "filter": {"type": "string", "description": "Substring or glob such as /canopy/*"},
                    "type": {"type": "string", "description": "Only this interface, or part of its name, such as sensor_msgs/msg/LaserScan"},
                    "counts": {"type": "boolean", "description": "For topics: publishers and subscribers of each"},
                    "name": {"type": "string", "description": "A name to describe, such as /odom"},
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

/// The service every rclcpp component container offers to list what it runs.
const LIST_NODES: &str = "composition_interfaces/srv/ListNodes";
/// What every managed node offers to say how it stands, and what it could do next.
const GET_STATE: &str = "lifecycle_msgs/srv/GetState";
const GET_TRANSITIONS: &str = "lifecycle_msgs/srv/GetAvailableTransitions";
/// The service an action server offers and an action client calls, under the action's name.
const SEND_GOAL: &str = "/_action/send_goal";
/// How long a look at another node's state may take.
const STATE_WAIT: Duration = Duration::from_secs(2);

impl RosGraph {
    /// The nodes `node` hosts when it is a component container; none when it is not, or does
    /// not answer.
    async fn hosted(&self, graph: &GraphDetail, node: &str) -> Option<Vec<String>> {
        let service = format!("{node}/_container/list_nodes");
        find_type(&graph.services, &service)?;
        let out = self
            .ctx
            .robot
            .call(&service, LIST_NODES, json!({}), Duration::from_secs(2))
            .await
            .ok()?;
        Some(
            out["full_node_names"]
                .as_array()?
                .iter()
                .filter_map(|n| n.as_str().map(str::to_owned))
                .collect(),
        )
    }

    /// How a managed node stands; none for a node that is not one, or that does not answer.
    async fn lifecycle_state(&self, graph: &GraphDetail, node: &str) -> Option<String> {
        let service = format!("{node}/get_state");
        if find_type(&graph.services, &service).as_deref() != Some(GET_STATE) {
            return None;
        }
        let out = self
            .ctx
            .robot
            .call(&service, GET_STATE, json!({}), STATE_WAIT)
            .await
            .ok()?;
        out["current_state"]["label"].as_str().map(str::to_owned)
    }

    /// What a managed node could do next.
    async fn lifecycle_transitions(&self, graph: &GraphDetail, node: &str) -> Option<Vec<String>> {
        let service = format!("{node}/get_available_transitions");
        find_type(&graph.services, &service).filter(|t| t == GET_TRANSITIONS)?;
        let out = self
            .ctx
            .robot
            .call(&service, GET_TRANSITIONS, json!({}), STATE_WAIT)
            .await
            .ok()?;
        Some(
            out["available_transitions"]
                .as_array()?
                .iter()
                .filter_map(|t| t["transition"]["label"].as_str().map(str::to_owned))
                .collect(),
        )
    }

    /// The managed nodes and how they stand, like `ros2 lifecycle nodes` and `get`.
    async fn lifecycle_nodes(&self, graph: &GraphDetail, filter: &str) -> ToolOutcome {
        let nodes: Vec<&str> = graph
            .services
            .iter()
            .filter(|(_, t)| t.first().is_some_and(|t| t == GET_STATE))
            .filter_map(|(service, _)| service.strip_suffix("/get_state"))
            .filter(|node| name_matches(filter, node))
            .collect();
        // Only those shown are asked, all at once: a hung node costs one wait, not one each.
        let total = nodes.len();
        let shown = &nodes[..total.min(LIST_LIMIT)];
        let states =
            futures::future::join_all(shown.iter().map(|n| self.lifecycle_state(graph, n))).await;
        let items: Vec<Value> = shown
            .iter()
            .zip(states)
            .map(|(node, state)| json!({"name": node, "state": state}))
            .collect();
        ToolOutcome::ok(
            json!({"kind": "lifecycle", "total": total, "shown": items.len(), "items": items}),
        )
    }

    async fn list(&self, graph: &GraphDetail, args: &Value) -> ToolOutcome {
        let kind = args["kind"].as_str().unwrap_or("topics");
        let filter = args["filter"].as_str().unwrap_or_default();
        let all = args["all"].as_bool().unwrap_or(false);
        let of_type = args["type"].as_str().unwrap_or_default();
        if kind == "lifecycle" {
            return self.lifecycle_nodes(graph, filter).await;
        }
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
                    "kind `{other}` is not one of topics, services, actions, nodes, lifecycle"
                ));
            }
        };
        let matching: Vec<&(String, String)> = items
            .iter()
            .filter(|(n, _)| all || !self.ctx.hidden(n))
            .filter(|(n, _)| name_matches(filter, n))
            .filter(|(_, t)| of_type.is_empty() || t.contains(of_type))
            .collect();
        let mut shown = Vec::new();
        for (n, t) in matching.iter().take(LIST_LIMIT) {
            shown.push(if !t.is_empty() {
                let mut item = json!({"name": n, "type": t});
                if kind == "topics"
                    && args["counts"].as_bool().unwrap_or(false)
                    && let Ok(ends) = self.ctx.robot.endpoints(n).await
                {
                    item["publishers"] = json!(ends.publishers.len());
                    item["subscribers"] = json!(ends.subscribers.len());
                }
                item
            } else if let Some(hosts) = self.hosted(graph, n).await {
                json!({"name": n, "hosts": hosts})
            } else {
                json!(n)
            });
        }
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

    async fn node(&self, graph: &GraphDetail, name: &str) -> ToolOutcome {
        match self.ctx.robot.node_entities(name).await {
            Ok(n) => {
                let list = |items: &[(String, Vec<String>)]| {
                    items
                        .iter()
                        .filter(|(n, _)| !self.ctx.hidden(n))
                        .map(|(n, t)| json!({"name": n, "type": t.join(", ")}))
                        .collect::<Vec<_>>()
                };
                let mut data = json!({
                    "node": name,
                    "publishers": list(&n.publishers),
                    "subscribers": list(&n.subscribers),
                    "services": list(&n.services),
                    "clients": list(&n.clients),
                    "action_servers": action_ends(&n.services),
                    "action_clients": action_ends(&n.clients),
                });
                if let Some(state) = self.lifecycle_state(graph, name).await {
                    data["lifecycle_state"] = json!(state);
                    data["lifecycle_transitions"] =
                        json!(self.lifecycle_transitions(graph, name).await);
                }
                // A container's own entities are few; the work is its components'.
                if let Some(hosts) = self.hosted(graph, name).await {
                    data["hosts"] = json!(hosts);
                    data["note"] = json!(
                        "a component container: the nodes it hosts publish, subscribe and serve \
                         under their own names"
                    );
                }
                ToolOutcome::ok(data)
            }
            Err(e) => failed(&e),
        }
    }

    async fn service(&self, graph: &GraphDetail, name: &str, ty: &str) -> ToolOutcome {
        let available = self
            .ctx
            .robot
            .service_available(name, ty, Duration::from_secs(1))
            .await;
        // Only each node knows its side of a service, so this asks every one.
        let (mut servers, mut clients) = (Vec::new(), Vec::new());
        let mut nodes = graph.nodes.clone();
        nodes.dedup();
        // All at once: a node slow to answer costs one wait, not one each.
        let entities =
            futures::future::join_all(nodes.iter().map(|n| self.ctx.robot.node_entities(n))).await;
        for (node, entities) in nodes.iter().zip(entities) {
            if let Ok(n) = entities {
                let has = |list: &[(String, Vec<String>)]| list.iter().any(|(s, _)| s == name);
                if has(&n.services) {
                    servers.push(node.clone());
                }
                if has(&n.clients) {
                    clients.push(node.clone());
                }
            }
        }
        ToolOutcome::ok(json!({
            "service": name,
            "type": ty,
            "reachable": available,
            "servers": servers,
            "clients": clients,
            "definition": self.ctx.schemas.show(ty),
        }))
    }

    async fn action(&self, name: &str, ty: &str) -> ToolOutcome {
        // A server publishes an action's feedback, and each client subscribes to it.
        let ends = match self
            .ctx
            .robot
            .endpoints(&format!("{name}/_action/feedback"))
            .await
        {
            Ok(e) => e,
            Err(e) => return failed(&e),
        };
        let nodes = |list: &[nervros_ros::Endpoint]| {
            list.iter().map(|e| e.node.clone()).collect::<Vec<_>>()
        };
        ToolOutcome::ok(json!({
            "action": name,
            "type": ty,
            "servers": nodes(&ends.publishers),
            "clients": nodes(&ends.subscribers),
            "definition": self.ctx.schemas.show(ty),
        }))
    }
}

/// Whether a name passes a filter: a glob with `*`, otherwise a substring, or no filter.
fn name_matches(filter: &str, name: &str) -> bool {
    filter.is_empty()
        || if filter.contains('*') {
            glob_match(filter, name)
        } else {
            name.contains(filter)
        }
}

/// The actions among a node's services or clients, with their types.
fn action_ends(services: &[(String, Vec<String>)]) -> Vec<Value> {
    services
        .iter()
        .filter_map(|(name, types)| {
            let action = name.strip_suffix(SEND_GOAL)?;
            let ty = types.first()?.strip_suffix("_SendGoal").unwrap_or_default();
            Some(json!({"name": action, "type": ty}))
        })
        .collect()
}

#[async_trait]
impl Tool for RosGraph {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    fn rule_view(&self, args: &Value) -> Option<Value> {
        resolved_view(args, "name")
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let graph = match self.ctx.graph().await {
            Ok(g) => g,
            Err(out) => return out,
        };
        let Some(raw) = args["name"].as_str().filter(|n| !n.is_empty()) else {
            return self.list(&graph, &args).await;
        };
        let name = match read_name(raw) {
            Ok(n) => n,
            Err(e) => return ToolOutcome::refused(e),
        };
        if graph.topics.iter().any(|(n, _)| *n == name) {
            return self.topic(&graph, &name).await;
        }
        if graph.nodes.contains(&name) {
            return self.node(&graph, &name).await;
        }
        if let Some(ty) = find_type(&graph.services, &name) {
            return self.service(&graph, &name, &ty).await;
        }
        if let Some((_, ty)) = actions(&graph).into_iter().find(|(n, _)| *n == name) {
            return self.action(&name, &ty).await;
        }
        ToolOutcome::failed(format!(
            "`{name}` is not a topic, node, service or action in the graph"
        ))
    }
}
