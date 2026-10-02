//! A node's parameters: reading them, and setting one where the profile allows it.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use serde_json::{Map, Value, json};

use super::{Ctx, PARAMS_MAX, failed, name_arg, object, spec, summarize};
use crate::guard::glob_match;
use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};

/// `params`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(Params {
        spec: spec(
            "params",
            "A node's parameters, like ros2 param list/get/describe.",
            object(
                json!({
                    "op": {"type": "string", "enum": ["list", "get", "describe"]},
                    "node": {"type": "string", "description": "Absolute node name"},
                    "names": {"type": "array", "items": {"type": "string"}},
                    "prefix": {"type": "string"},
                }),
                &["node"],
            ),
            Risk::Observe,
        ),
        ctx,
    })
}

/// `param_set`.
pub(super) fn set_tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(ParamSet {
        spec: spec(
            "param_set",
            "Set one parameter of a node, like ros2 param set, and read it back. The value is \
             converted to the parameter's current type. Needs the operator's approval.",
            object(
                json!({
                    "node": {"type": "string", "description": "Absolute node name"},
                    "name": {"type": "string"},
                    "value": {"description": "The new value"},
                }),
                &["node", "name", "value"],
            ),
            Risk::WorldEdit,
        ),
        ctx,
    })
}

struct Params {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

/// A `rcl_interfaces/msg/ParameterValue` as plain JSON.
fn param_value(v: &Value) -> Value {
    match v["type"].as_u64() {
        Some(1) => v["bool_value"].clone(),
        Some(2) => v["integer_value"].clone(),
        Some(3) => v["double_value"].clone(),
        Some(4) => v["string_value"].clone(),
        Some(5) => v["byte_array_value"].clone(),
        Some(6) => v["bool_array_value"].clone(),
        Some(7) => v["integer_array_value"].clone(),
        Some(8) => v["double_array_value"].clone(),
        Some(9) => v["string_array_value"].clone(),
        _ => Value::Null,
    }
}

/// Plain JSON as a `ParameterValue` of the parameter's current type, so `1` for a double is `1.0`.
fn to_param_value(type_id: u64, value: &Value) -> Result<Value, String> {
    let bad = |what: &str| format!("the parameter holds {what}, which `{value}` is not");
    Ok(match type_id {
        1 => json!({"type": 1, "bool_value": value.as_bool().ok_or_else(|| bad("true or false"))?}),
        2 => {
            json!({"type": 2, "integer_value": value.as_i64().ok_or_else(|| bad("a whole number"))?})
        }
        3 => json!({"type": 3, "double_value": value.as_f64().ok_or_else(|| bad("a number"))?}),
        4 => json!({"type": 4, "string_value": value.as_str().ok_or_else(|| bad("text"))?}),
        7 => {
            json!({"type": 7, "integer_array_value": value.as_array().ok_or_else(|| bad("a list of whole numbers"))?})
        }
        8 => {
            json!({"type": 8, "double_array_value": value.as_array().ok_or_else(|| bad("a list of numbers"))?})
        }
        9 => {
            json!({"type": 9, "string_array_value": value.as_array().ok_or_else(|| bad("a list of text"))?})
        }
        6 => {
            json!({"type": 6, "bool_array_value": value.as_array().ok_or_else(|| bad("a list of true or false"))?})
        }
        _ => return Err("the parameter is not set or of a type this tool cannot write".to_owned()),
    })
}

impl Ctx {
    async fn param_call(
        &self,
        node: &str,
        service: &str,
        ty: &str,
        request: Value,
    ) -> Result<Value, ToolOutcome> {
        self.robot
            .call(
                &format!("{node}/{service}"),
                ty,
                request,
                Duration::from_secs(5),
            )
            .await
            .map_err(|e| failed(&e))
    }

    async fn param_values(
        &self,
        node: &str,
        names: &[String],
    ) -> Result<Vec<(String, Value, u64)>, ToolOutcome> {
        let out = self
            .param_call(
                node,
                "get_parameters",
                "rcl_interfaces/srv/GetParameters",
                json!({"names": names}),
            )
            .await?;
        let values = out["values"].as_array().cloned().unwrap_or_default();
        Ok(names
            .iter()
            .zip(values)
            .map(|(n, v)| (n.clone(), param_value(&v), v["type"].as_u64().unwrap_or(0)))
            .collect())
    }

    async fn param_names(&self, node: &str) -> Result<Vec<String>, ToolOutcome> {
        let out = self
            .param_call(
                node,
                "list_parameters",
                "rcl_interfaces/srv/ListParameters",
                json!({"prefixes": [], "depth": 0}),
            )
            .await?;
        Ok(out["result"]["names"]
            .as_array()
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str().map(str::to_owned))
                    .collect()
            })
            .unwrap_or_default())
    }
}

#[async_trait]
impl Tool for Params {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let node = match name_arg(&args, "node") {
            Ok(n) => n,
            Err(why) => return ToolOutcome::refused(why),
        };
        let mut names: Vec<String> = args["names"]
            .as_array()
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str().map(str::to_owned))
                    .collect()
            })
            .unwrap_or_default();
        if let Some(one) = args["name"].as_str() {
            names.push(one.to_owned());
        }
        match args["op"].as_str().unwrap_or("list") {
            "list" => {
                let prefix = args["prefix"].as_str().unwrap_or_default();
                let prefixes: Vec<&str> = if prefix.is_empty() {
                    vec![]
                } else {
                    vec![prefix]
                };
                match self
                    .ctx
                    .param_call(
                        &node,
                        "list_parameters",
                        "rcl_interfaces/srv/ListParameters",
                        json!({"prefixes": prefixes, "depth": 0}),
                    )
                    .await
                {
                    Ok(out) => {
                        ToolOutcome::ok(json!({"node": node, "names": out["result"]["names"]}))
                    }
                    Err(out) => out,
                }
            }
            "get" => {
                // No names asks for them all, which is what a model means by it.
                if names.is_empty() {
                    match self.ctx.param_names(&node).await {
                        Ok(all) => names = all.into_iter().take(PARAMS_MAX).collect(),
                        Err(out) => return out,
                    }
                }
                match self.ctx.param_values(&node, &names).await {
                    Ok(values) => {
                        let mut out = Map::new();
                        for (n, v, _) in values {
                            let shown = if self.ctx.masked(&n) {
                                json!("(hidden)")
                            } else {
                                v
                            };
                            out.insert(n, shown);
                        }
                        ToolOutcome::ok(json!({"node": node, "values": out}))
                    }
                    Err(out) => out,
                }
            }
            "describe" => {
                if names.is_empty() {
                    return ToolOutcome::failed("describe needs names; list them first");
                }
                match self
                    .ctx
                    .param_call(
                        &node,
                        "describe_parameters",
                        "rcl_interfaces/srv/DescribeParameters",
                        json!({"names": names}),
                    )
                    .await
                {
                    Ok(out) => ToolOutcome::ok(
                        json!({"node": node, "descriptors": summarize(&out["descriptors"])}),
                    ),
                    Err(out) => out,
                }
            }
            other => ToolOutcome::failed(format!("op `{other}` is not one of list, get, describe")),
        }
    }
}

struct ParamSet {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

/// A parameter change, resolved: the node, the name, the value in the parameter's own type and
/// the value it has now.
struct ParamChange {
    node: String,
    name: String,
    value: Value,
    was: Value,
}

impl ParamSet {
    async fn prepare(&self, args: &Value) -> Result<ParamChange, ToolOutcome> {
        let node = name_arg(args, "node").map_err(ToolOutcome::refused)?;
        let name = args["name"].as_str().unwrap_or_default().to_owned();
        let key = format!("{node}:{name}");
        if name.is_empty()
            || !self
                .ctx
                .config
                .param_set
                .iter()
                .any(|p| glob_match(p, &key))
        {
            return Err(ToolOutcome::refused(format!(
                "the profile does not let this agent set `{key}`"
            )));
        }
        // A node under a denied namespace, such as /controller_manager/*, keeps its parameters.
        if self.ctx.guard.hard_denied(&format!("{node}/")) {
            return Err(ToolOutcome::refused(format!(
                "`{node}` is on the robot's hard deny list"
            )));
        }
        let before = self
            .ctx
            .param_values(&node, std::slice::from_ref(&name))
            .await?;
        let (_, was, type_id) = before.into_iter().next().unwrap_or_default();
        let value = to_param_value(type_id, &args["value"]).map_err(ToolOutcome::failed)?;
        Ok(ParamChange {
            node,
            name,
            value,
            was,
        })
    }
}

#[async_trait]
impl Tool for ParamSet {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        Some(self.prepare(args).await.map(|c| {
            // The card and the log see the old value only when it is not masked.
            let was = if self.ctx.masked(&c.name) {
                json!("(hidden)")
            } else {
                c.was
            };
            Assessment {
                risk: Risk::WorldEdit,
                resources: Vec::new(),
                args: None,
                reason: format!(
                    "sets {} on {} from {was} to {}",
                    c.name,
                    c.node,
                    param_value(&c.value)
                ),
            }
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let c = match self.prepare(&args).await {
            Ok(c) => c,
            Err(out) => return out,
        };
        let request = json!({"parameters": [{"name": c.name, "value": c.value}]});
        let out = match self
            .ctx
            .param_call(
                &c.node,
                "set_parameters",
                "rcl_interfaces/srv/SetParameters",
                request,
            )
            .await
        {
            Ok(o) => o,
            Err(out) => return out,
        };
        let result = &out["results"][0];
        if result["successful"].as_bool() != Some(true) {
            return ToolOutcome::failed(format!(
                "{} refused: {}",
                c.node,
                result["reason"].as_str().unwrap_or("no reason given")
            ));
        }
        let after = self
            .ctx
            .param_values(&c.node, std::slice::from_ref(&c.name))
            .await
            .ok();
        let now = after.and_then(|a| a.first().map(|(_, v, _)| v.clone()));
        let (was, now) = if self.ctx.masked(&c.name) {
            (json!("(hidden)"), Some(json!("(hidden)")))
        } else {
            (c.was, now)
        };
        ToolOutcome::ok(json!({
            "node": c.node,
            "name": c.name,
            "was": was,
            "now": now,
        }))
    }
}
