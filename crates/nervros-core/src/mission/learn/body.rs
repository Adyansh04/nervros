//! A taught skill's body as the model writes it, a small JSON tree, made into the one behaviour-tree
//! element the executor takes. Its shape is checked here so the model hears of a misspelt leaf at
//! once; the executor checks it all again, as it must.
//!
//! - `{"sequence": [..]}` and `{"fallback": [..]}`: their children in order;
//! - `{"retry": 2, "do": {..}}`, `{"timeout_s": 5, "do": {..}}`, `{"force_success": {..}}`;
//! - `{"leaf": "MoveHand", "arm": "{arm}", "mode": "point"}`: a leaf and its inputs, each a
//!   literal or `{arg}`, one of the skill's own arguments.

use std::fmt::Write as _;

use serde_json::{Map, Value};

use crate::mission::catalog::Learnable;

/// The body as XML, or what is wrong with it.
///
/// # Errors
///
/// Each problem in the tree, by where it is.
pub fn to_xml(body: &Value, learnable: &Learnable) -> Result<String, Vec<String>> {
    let mut problems = Vec::new();
    let mut xml = String::new();
    node(body, learnable, "body", &mut xml, &mut problems);
    if problems.is_empty() {
        Ok(xml)
    } else {
        Err(problems)
    }
}

/// Room for a leaf that moves an arm or the base: a plan and a slow move.
// ponytail: one figure for every moving leaf; the executor could report each leaf's own time.
const MOVE_S: f64 = 45.0;
/// Room for a leaf that moves nothing, such as a look.
const STILL_S: f64 = 5.0;

/// The least time a body needs to run through once, counting every branch and attempt, as the
/// executor's worst case does.
#[must_use]
pub fn least_s(body: &Value, learnable: &Learnable) -> f64 {
    let Some(fields) = body.as_object() else {
        return 0.0;
    };
    let child = |key: &str| fields.get(key).map_or(0.0, |c| least_s(c, learnable));
    if let Some(name) = fields.get("leaf").and_then(Value::as_str) {
        return match learnable.leaves.get(name) {
            Some(_) if name == "Wait" => {
                fields.get("seconds").and_then(Value::as_f64).unwrap_or(0.0)
            }
            Some(leaf) if !leaf.holds.is_empty() => MOVE_S,
            _ => STILL_S,
        };
    }
    if let Some(children) = fields.get("sequence").or_else(|| fields.get("fallback")) {
        return children
            .as_array()
            .map_or(0.0, |c| c.iter().map(|n| least_s(n, learnable)).sum());
    }
    if let Some(attempts) = fields.get("retry").and_then(Value::as_f64) {
        return attempts * child("do");
    }
    if let Some(seconds) = fields.get("timeout_s").and_then(Value::as_f64) {
        return seconds.min(child("do"));
    }
    child("force_success")
}

fn node(v: &Value, learnable: &Learnable, at: &str, xml: &mut String, problems: &mut Vec<String>) {
    let Some(fields) = v.as_object() else {
        problems.push(format!(
            "{at}: each node is an object, such as {{\"leaf\": \"Wait\", \"seconds\": 1}}"
        ));
        return;
    };
    if let Some(name) = fields.get("leaf") {
        leaf(name, fields, learnable, at, xml, problems);
    } else if let Some(children) = fields.get("sequence").or_else(|| fields.get("fallback")) {
        let tag = if fields.contains_key("sequence") {
            "Sequence"
        } else {
            "Fallback"
        };
        let Some(children) = children.as_array().filter(|c| !c.is_empty()) else {
            problems.push(format!("{at}: a {tag} lists its children, at least one"));
            return;
        };
        let _ = write!(xml, "<{tag}>");
        for (i, child) in children.iter().enumerate() {
            node(child, learnable, &format!("{at}.{}", i + 1), xml, problems);
        }
        let _ = write!(xml, "</{tag}>");
    } else if let Some(attempts) = fields.get("retry") {
        let attempts = attempts
            .as_u64()
            .filter(|n| (1..=3).contains(n))
            .unwrap_or_else(|| {
                problems.push(format!("{at}: retry is 1 to 3 attempts"));
                1
            });
        let _ = write!(xml, "<RetryUntilSuccessful num_attempts=\"{attempts}\">");
        only_child(fields, learnable, at, xml, problems);
        xml.push_str("</RetryUntilSuccessful>");
    } else if let Some(seconds) = fields.get("timeout_s") {
        let msec = seconds
            .as_f64()
            .filter(|s| s.is_finite() && *s > 0.0 && *s <= learnable.max_duration_s)
            .map_or_else(
                || {
                    problems.push(format!(
                        "{at}: timeout_s is above 0 and at most the skill's time"
                    ));
                    "1000".to_owned()
                },
                |s| format!("{:.0}", (s * 1000.0).max(1.0)),
            );
        let _ = write!(xml, "<Timeout msec=\"{msec}\">");
        only_child(fields, learnable, at, xml, problems);
        xml.push_str("</Timeout>");
    } else if let Some(child) = fields.get("force_success") {
        xml.push_str("<ForceSuccess>");
        node(
            child,
            learnable,
            &format!("{at}.force_success"),
            xml,
            problems,
        );
        xml.push_str("</ForceSuccess>");
    } else {
        problems.push(format!(
            "{at}: a node is a leaf, a sequence, a fallback, a retry, a timeout_s or a force_success"
        ));
    }
}

fn only_child(
    fields: &Map<String, Value>,
    learnable: &Learnable,
    at: &str,
    xml: &mut String,
    problems: &mut Vec<String>,
) {
    match fields.get("do") {
        Some(child) => node(child, learnable, &format!("{at}.do"), xml, problems),
        None => problems.push(format!("{at}: it wraps one node, given as \"do\"")),
    }
}

fn leaf(
    name: &Value,
    fields: &Map<String, Value>,
    learnable: &Learnable,
    at: &str,
    xml: &mut String,
    problems: &mut Vec<String>,
) {
    let name = name.as_str().unwrap_or_default();
    let Some(known) = learnable.leaves.get(name) else {
        let names: Vec<&str> = learnable.leaves.keys().map(String::as_str).collect();
        problems.push(format!(
            "{at}: no leaf {name}; a taught skill may use {}",
            names.join(", ")
        ));
        return;
    };
    let _ = write!(xml, "<{name}");
    // Sorted, so one tree gives one text whichever order serde_json keeps keys in.
    let mut inputs: Vec<(&String, &Value)> = fields
        .iter()
        .filter(|(k, _)| k.as_str() != "leaf")
        .collect();
    inputs.sort_by(|a, b| a.0.cmp(b.0));
    for (port, value) in inputs {
        if !known.ports.contains_key(port) {
            let ports: Vec<&str> = known.ports.keys().map(String::as_str).collect();
            problems.push(format!(
                "{at}: {name} has no input {port}; its inputs are {}",
                ports.join(", ")
            ));
            continue;
        }
        let text = match value {
            Value::String(s) => s.clone(),
            Value::Number(n) => n.to_string(),
            Value::Bool(b) => b.to_string(),
            _ => {
                problems.push(format!(
                    "{at}: {name}.{port} is text, a number or true/false"
                ));
                continue;
            }
        };
        let _ = write!(xml, " {port}=\"{}\"", escape(&text));
    }
    xml.push_str("/>");
}

/// Text as an XML attribute value holds it.
fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use serde_json::json;

    use super::*;
    use crate::mission::catalog::LearnableLeaf;

    fn learnable() -> Learnable {
        let leaf = |holds: &str, ports: &[&str]| LearnableLeaf {
            holds: holds.to_owned(),
            ports: ports
                .iter()
                .map(|p| ((*p).to_owned(), String::new()))
                .collect(),
        };
        Learnable {
            leaves: BTreeMap::from([
                (
                    "MoveHand".to_owned(),
                    leaf("arms", &["arm", "target", "mode"]),
                ),
                ("Wait".to_owned(), leaf("", &["seconds"])),
            ]),
            max_duration_s: 300.0,
            max_nodes: 40,
        }
    }

    #[test]
    fn a_tree_of_leaves_becomes_the_executor_s_xml() {
        let body = json!({"sequence": [
            {"leaf": "MoveHand", "arm": "{arm}", "target": "{object}", "mode": "point"},
            {"retry": 2, "do": {"leaf": "Wait", "seconds": 1.5}},
            {"timeout_s": 4, "do": {"force_success": {"leaf": "Wait", "seconds": 2}}}
        ]});
        assert_eq!(
            to_xml(&body, &learnable()).unwrap(),
            "<Sequence><MoveHand arm=\"{arm}\" mode=\"point\" target=\"{object}\"/>\
             <RetryUntilSuccessful num_attempts=\"2\"><Wait seconds=\"1.5\"/></RetryUntilSuccessful>\
             <Timeout msec=\"4000\"><ForceSuccess><Wait seconds=\"2\"/></ForceSuccess></Timeout>\
             </Sequence>"
        );
    }

    #[test]
    fn a_misspelt_leaf_or_input_is_named_with_what_there_is() {
        let body = json!({"sequence": [{"leaf": "PickUp"}, {"leaf": "Wait", "secs": 1},
                                       {"retry": 9, "do": {"leaf": "Wait", "seconds": 1}}]});
        let problems = to_xml(&body, &learnable()).unwrap_err().join("; ");
        assert!(
            problems.contains("no leaf PickUp; a taught skill may use MoveHand, Wait"),
            "{problems}"
        );
        assert!(
            problems.contains("Wait has no input secs; its inputs are seconds"),
            "{problems}"
        );
        assert!(problems.contains("retry is 1 to 3"), "{problems}");
    }

    #[test]
    fn a_body_needs_room_for_each_move_and_wait() {
        let body = json!({"sequence": [
            {"leaf": "MoveHand", "arm": "{arm}", "target": "{object}", "mode": "point"},
            {"leaf": "Wait", "seconds": 2},
            {"retry": 2, "do": {"leaf": "MoveHand", "arm": "{arm}", "mode": "reach"}}
        ]});
        assert!((least_s(&body, &learnable()) - (MOVE_S + 2.0 + 2.0 * MOVE_S)).abs() < 1e-9);
        assert_eq!(escape("a<\"b\">&"), "a&lt;&quot;b&quot;&gt;&amp;");
    }
}
