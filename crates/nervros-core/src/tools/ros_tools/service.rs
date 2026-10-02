//! Calling any service, like `ros2 service call`.

use std::borrow::Cow;
use std::sync::Arc;

use async_trait::async_trait;
use serde_json::{Value, json};

use super::{Ctx, MAX_CALL, Prepared, failed, name_arg, object, payload, seconds, spec, summarize};
use crate::guard::glob_match;
use crate::tools::{Assessment, Resource, Risk, SchemaPart, Tool, ToolOutcome, ToolSpec};

/// `service_call`.
pub(super) fn tool(ctx: Arc<Ctx>) -> Arc<dyn Tool> {
    Arc::new(ServiceCall {
        spec: spec(
            "service_call",
            "Call any ROS service, like ros2 service call. The type is looked up when omitted; \
             ros_graph with the name shows the request fields. Services that only read run at \
             once; the rest need the robot armed and the operator's approval.",
            object(
                json!({
                    "service": {"type": "string", "description": "Absolute service name"},
                    "type": {"type": "string", "description": "pkg/srv/Name, if known"},
                    "request": {"type": "object", "description": "The request fields as JSON"},
                    "timeout_s": {"type": "number", "description": "Up to 30 (5)"},
                }),
                &["service"],
            ),
            Risk::Motion,
        ),
        ctx,
    })
}

struct ServiceCall {
    spec: ToolSpec,
    ctx: Arc<Ctx>,
}

impl ServiceCall {
    fn observes(&self, service: &str) -> bool {
        let last = service.rsplit('/').next().unwrap_or(service);
        self.ctx.config.service_observe.iter().any(|p| {
            if p.contains('/') {
                glob_match(p, service)
            } else {
                glob_match(p, last)
            }
        })
    }

    /// Hides the values of a `GetParameters` reply whose names the profile masks.
    fn mask_values(&self, names: &Value, response: &mut Value) {
        let names = names.as_array().map_or(&[][..], Vec::as_slice);
        if let Some(values) = response["values"].as_array_mut() {
            for (name, value) in names.iter().zip(values) {
                if name.as_str().is_some_and(|n| self.ctx.masked(n)) {
                    *value = json!("(hidden)");
                }
            }
        }
    }

    async fn prepare(&self, args: &Value) -> Result<Prepared, ToolOutcome> {
        let service = name_arg(args, "service").map_err(ToolOutcome::refused)?;
        let listed = self
            .ctx
            .config
            .service_call
            .iter()
            .any(|p| glob_match(p, &service));
        if !listed && !self.observes(&service) {
            return Err(ToolOutcome::refused(format!(
                "the profile does not let this agent call `{service}`"
            )));
        }
        let ty = match args["type"].as_str().filter(|t| !t.is_empty()) {
            Some(t) => t.to_owned(),
            None => self.ctx.service_type(&service).await?,
        };
        if let Some(out) = self.ctx.deny_act(&service, &ty) {
            return Err(out);
        }
        let request = payload(args, "request");
        self.ctx
            .schemas
            .validate(&ty, SchemaPart::Request, &request)
            .map_err(|why| ToolOutcome::failed(format!("the request does not fit {ty}: {why}")))?;
        Ok(Prepared {
            name: service,
            ty,
            payload: request,
        })
    }
}

#[async_trait]
impl Tool for ServiceCall {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        Some(self.prepare(args).await.map(|p| {
            if self.observes(&p.name) {
                Assessment {
                    risk: Risk::Observe,
                    resources: Vec::new(),
                    args: None,
                    reason: format!("reads {} ({})", p.name, p.ty),
                }
            } else {
                Assessment {
                    risk: Risk::Motion,
                    resources: Resource::ALL.to_vec(),
                    args: None,
                    reason: format!("calls {} ({}) with {}", p.name, p.ty, p.payload),
                }
            }
        }))
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let p = match self.prepare(&args).await {
            Ok(p) => p,
            Err(out) => return out,
        };
        let timeout = seconds(&args, "timeout_s", 5.0, MAX_CALL);
        let names = p.payload["names"].clone();
        match self
            .ctx
            .robot
            .call(&p.name, &p.ty, p.payload, timeout)
            .await
        {
            Ok(mut response) => {
                // Read through the service, a masked parameter stays masked as `params` keeps it.
                if p.ty == "rcl_interfaces/srv/GetParameters" {
                    self.mask_values(&names, &mut response);
                }
                ToolOutcome::ok(
                    json!({"service": p.name, "type": p.ty, "response": summarize(&response)}),
                )
            }
            Err(e) => failed(&e),
        }
    }
}
