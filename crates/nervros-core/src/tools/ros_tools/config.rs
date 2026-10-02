//! The `[ros_tools]` table of a profile.

use serde::Deserialize;

/// The `[ros_tools]` table.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RosToolsConfig {
    /// Names left out of listings unless asked for; a convenience, not a boundary.
    #[serde(default = "d_hidden")]
    pub hidden: Vec<String>,
    /// Topics never sampled or echoed.
    #[serde(default)]
    pub read_deny: Vec<String>,
    /// Parameter names whose values are masked.
    #[serde(default = "d_param_read_deny")]
    pub param_read_deny: Vec<String>,
    /// Services that only read: `service_call` runs them without arming or approval. A pattern
    /// without a `/` matches the service's last name part, so `get_*` is `/a/get_map` but not
    /// `/a/get_ready/execute`.
    #[serde(default = "d_service_observe")]
    pub service_observe: Vec<String>,
    /// Services `service_call` may call; empty leaves the tool out.
    #[serde(default)]
    pub service_call: Vec<String>,
    /// Actions `action_goal` may send goals to; empty leaves the tool out.
    #[serde(default)]
    pub action_send: Vec<String>,
    /// Topics `topic_publish` may publish on; empty leaves the tool out.
    #[serde(default)]
    pub publish: Vec<String>,
    /// `node:parameter` globs `param_set` may change; empty leaves the tool out.
    #[serde(default)]
    pub param_set: Vec<String>,
}

fn d_hidden() -> Vec<String> {
    [
        "*/_action/*",
        "/rosout",
        "/parameter_events",
        "*/describe_parameters",
        "*/get_parameters",
        "*/get_parameter_types",
        "*/list_parameters",
        "*/set_parameters",
        "*/set_parameters_atomically",
        "*/get_type_description",
    ]
    .map(str::to_owned)
    .to_vec()
}

fn d_param_read_deny() -> Vec<String> {
    ["*key*", "*token*", "*secret*", "*password*"]
        .map(str::to_owned)
        .to_vec()
}

fn d_service_observe() -> Vec<String> {
    ["get_*", "list_*", "describe_*"]
        .map(str::to_owned)
        .to_vec()
}

impl Default for RosToolsConfig {
    fn default() -> Self {
        Self {
            hidden: d_hidden(),
            read_deny: Vec::new(),
            param_read_deny: d_param_read_deny(),
            service_observe: d_service_observe(),
            service_call: Vec::new(),
            action_send: Vec::new(),
            publish: Vec::new(),
            param_set: Vec::new(),
        }
    }
}
