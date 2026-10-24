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

/// The body as XML, or what is wrong with it. `world_args` are the skill's arguments that take a
/// world-model id.
///
/// # Errors
///
/// Each problem in the tree, by where it is.
pub fn to_xml(
    body: &Value,
    learnable: &Learnable,
    world_args: &[&str],
) -> Result<String, Vec<String>> {
    let mut problems = Vec::new();
    let mut xml = String::new();
    let rules = Rules {
        learnable,
        world_args,
    };
    node(body, &rules, "body", &mut xml, &mut problems);
    if problems.is_empty() {
        Ok(xml)
    } else {
        Err(problems)
    }
}

/// What a body is checked against: the leaves, and which of the skill's arguments are world ids.
struct Rules<'a> {
    learnable: &'a Learnable,
    world_args: &'a [&'a str],
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

fn node(v: &Value, rules: &Rules, at: &str, xml: &mut String, problems: &mut Vec<String>) {
    let learnable = rules.learnable;
    let Some(fields) = v.as_object() else {
        problems.push(format!(
            "{at}: each node is an object, such as {{\"leaf\": \"Wait\", \"seconds\": 1}}"
        ));
        return;
    };
    if let Some(name) = fields.get("leaf") {
        leaf(name, fields, rules, at, xml, problems);
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
            node(child, rules, &format!("{at}.{}", i + 1), xml, problems);
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
        only_child(fields, rules, at, xml, problems);
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
        only_child(fields, rules, at, xml, problems);
        xml.push_str("</Timeout>");
    } else if let Some(child) = fields.get("force_success") {
        xml.push_str("<ForceSuccess>");
        node(child, rules, &format!("{at}.force_success"), xml, problems);
        xml.push_str("</ForceSuccess>");
    } else {
        problems.push(format!(
            "{at}: a node is a leaf, a sequence, a fallback, a retry, a timeout_s or a force_success"
        ));
    }
}

fn only_child(
    fields: &Map<String, Value>,
    rules: &Rules,
    at: &str,
    xml: &mut String,
    problems: &mut Vec<String>,
) {
    match fields.get("do") {
        Some(child) => node(child, rules, &format!("{at}.do"), xml, problems),
        None => problems.push(format!("{at}: it wraps one node, given as \"do\"")),
    }
}

fn leaf(
    name: &Value,
    fields: &Map<String, Value>,
    rules: &Rules,
    at: &str,
    xml: &mut String,
    problems: &mut Vec<String>,
) {
    let name = name.as_str().unwrap_or_default();
    let Some(known) = rules.learnable.leaves.get(name) else {
        let names: Vec<&str> = rules.learnable.leaves.keys().map(String::as_str).collect();
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
        if let Some(allowed) = known.values.get(port.as_str())
            && bound(&text).is_none()
            && !allowed.contains(&text)
        {
            problems.push(format!(
                "{at}: {name}.{port} takes only {}, not {text}",
                allowed.join(" or ")
            ));
        }
        if let (Some(takers), Some(arg)) = (&known.world_ids, bound(&text))
            && rules.world_args.contains(&arg)
            && !takers.contains(port)
        {
            let to = if takers.is_empty() {
                String::new()
            } else {
                format!("; give it to {}", takers.join(" or "))
            };
            problems.push(format!(
                "{at}: {{{arg}}} is a world-model id, and {name}'s {port} takes none{to}"
            ));
        }
        let _ = write!(xml, " {port}=\"{}\"", escape(&text));
    }
    xml.push_str("/>");
    for group in &known.one_of {
        let set = group
            .iter()
            .filter(|p| {
                fields
                    .get(p.as_str())
                    .is_some_and(|v| !v.is_null() && v.as_str() != Some(""))
            })
            .count();
        let choices = group
            .iter()
            .map(|p| match known.ports.get(p).filter(|w| !w.is_empty()) {
                Some(what) => format!("{p} ({what})"),
                None => p.clone(),
            })
            .collect::<Vec<_>>()
            .join(" or ");
        match set {
            0 => problems.push(format!("{at}: {name} needs one of {choices}")),
            1 => {}
            _ => problems.push(format!("{at}: {name} takes only one of {choices}")),
        }
    }
}

/// The argument a value names, `{arg}`.
fn bound(text: &str) -> Option<&str> {
    text.strip_prefix('{')?.strip_suffix('}')
}

/// A literal given where a leaf takes a world-model id: the leaf, the input, the value, and the
/// inputs it could have gone in instead (its `one_of` group).
pub(crate) struct WorldLiteral {
    pub leaf: String,
    pub input: String,
    pub value: String,
    pub instead: Vec<String>,
}

/// Every literal in the body that goes where a leaf takes a world-model id, to check against the
/// world model before the operator is asked.
pub(crate) fn world_literals(body: &Value, learnable: &Learnable) -> Vec<WorldLiteral> {
    let mut out = Vec::new();
    collect(body, learnable, &mut out);
    out
}

fn collect(v: &Value, learnable: &Learnable, out: &mut Vec<WorldLiteral>) {
    let Some(fields) = v.as_object() else { return };
    if let Some(name) = fields.get("leaf").and_then(Value::as_str) {
        let Some(leaf) = learnable.leaves.get(name) else {
            return;
        };
        for input in leaf.world_ids.iter().flatten() {
            if let Some(value) = fields.get(input).and_then(Value::as_str)
                && bound(value).is_none()
                && !value.trim().is_empty()
            {
                let instead = leaf
                    .one_of
                    .iter()
                    .filter(|g| g.contains(input))
                    .flatten()
                    .filter(|p| *p != input)
                    .cloned()
                    .collect();
                out.push(WorldLiteral {
                    leaf: name.to_owned(),
                    input: input.clone(),
                    value: value.to_owned(),
                    instead,
                });
            }
        }
        return;
    }
    for child in fields.values() {
        match child {
            Value::Array(items) => items.iter().for_each(|i| collect(i, learnable, out)),
            Value::Object(_) => collect(child, learnable, out),
            _ => {}
        }
    }
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
            ..LearnableLeaf::default()
        };
        let mut hand = leaf("arms", &["arm", "target", "mode"]);
        hand.values = BTreeMap::from([(
            "arm".to_owned(),
            vec!["left".to_owned(), "right".to_owned()],
        )]);
        Learnable {
            leaves: BTreeMap::from([
                ("MoveHand".to_owned(), hand),
                ("Wait".to_owned(), leaf("", &["seconds"])),
            ]),
            max_duration_s: 300.0,
            max_nodes: 40,
        }
    }

    #[test]
    fn a_literal_outside_an_input_s_values_is_named_with_them() {
        let body =
            json!({"leaf": "MoveHand", "arm": "left_arm", "target": "{object}", "mode": "point"});
        let problems = to_xml(&body, &learnable(), &[]).unwrap_err();
        assert!(
            problems
                .iter()
                .any(|p| p.contains("MoveHand.arm takes only left or right, not left_arm")),
            "{problems:?}"
        );
        let bound =
            json!({"leaf": "MoveHand", "arm": "{arm}", "target": "{object}", "mode": "point"});
        assert!(to_xml(&bound, &learnable(), &[]).is_ok());
    }

    #[test]
    fn a_tree_of_leaves_becomes_the_executor_s_xml() {
        let body = json!({"sequence": [
            {"leaf": "MoveHand", "arm": "{arm}", "target": "{object}", "mode": "point"},
            {"retry": 2, "do": {"leaf": "Wait", "seconds": 1.5}},
            {"timeout_s": 4, "do": {"force_success": {"leaf": "Wait", "seconds": 2}}}
        ]});
        assert_eq!(
            to_xml(&body, &learnable(), &[]).unwrap(),
            "<Sequence><MoveHand arm=\"{arm}\" mode=\"point\" target=\"{object}\"/>\
             <RetryUntilSuccessful num_attempts=\"2\"><Wait seconds=\"1.5\"/></RetryUntilSuccessful>\
             <Timeout msec=\"4000\"><ForceSuccess><Wait seconds=\"2\"/></ForceSuccess></Timeout>\
             </Sequence>"
        );
    }

    #[test]
    fn a_target_or_a_frame_and_a_world_id_only_as_the_target() {
        let mut rules = learnable();
        let hand = rules.leaves.get_mut("MoveHand").unwrap();
        hand.ports.insert("frame".to_owned(), String::new());
        hand.one_of = vec![vec!["target".to_owned(), "frame".to_owned()]];
        hand.world_ids = Some(vec!["target".to_owned()]);
        let check = |body: Value| to_xml(&body, &rules, &["object"]).map_err(|p| p.join("; "));
        assert!(check(json!({"leaf": "MoveHand", "arm": "{arm}", "target": "{object}"})).is_ok());
        assert!(check(json!({"leaf": "MoveHand", "frame": "torso_link"})).is_ok());
        // As a model taught them on 15 Oct: each failed only once its step ran.
        let both = check(json!({"leaf": "MoveHand", "target": "{object}", "frame": "map"}));
        assert!(
            both.unwrap_err()
                .contains("takes only one of target or frame")
        );
        let none = check(json!({"leaf": "MoveHand", "arm": "{arm}"}));
        assert!(none.unwrap_err().contains("needs one of target or frame"));
        let as_frame = check(json!({"leaf": "MoveHand", "frame": "{object}"})).unwrap_err();
        assert!(
            as_frame.contains(
                "{object} is a world-model id, and MoveHand's frame takes none; give it to target"
            ),
            "{as_frame}"
        );
    }

    #[test]
    fn a_misspelt_leaf_or_input_is_named_with_what_there_is() {
        let body = json!({"sequence": [{"leaf": "PickUp"}, {"leaf": "Wait", "secs": 1},
                                       {"retry": 9, "do": {"leaf": "Wait", "seconds": 1}}]});
        let problems = to_xml(&body, &learnable(), &[]).unwrap_err().join("; ");
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
