//! A tool call's arguments against the tool's schema, before the call. llama.cpp constrains a
//! tool call's JSON but not a string's enum for every model, so a small model can name a skill or
//! a direction that does not exist; this says so in one line the model can act on.
//!
//! Lenient on purpose: it checks what the schema states plainly (required fields, enums, scalar
//! types, ranges, list lengths) and never an object where a list was declared or the reverse,
//! which tools accept in either form; a tool still checks what its arguments mean.

use std::fmt::Write as _;

use serde_json::Value;

/// How many of an enum's values a message lists.
const LISTED: usize = 12;

/// What is wrong with `args` for `schema`, a line each; none when nothing plainly is.
#[must_use]
pub fn check(schema: &Value, args: &Value) -> Vec<String> {
    let mut problems = Vec::new();
    walk(schema, args, "", &mut problems);
    problems
}

fn walk(schema: &Value, value: &Value, path: &str, problems: &mut Vec<String>) {
    let at = |p: &str| {
        if p.is_empty() {
            "the arguments".to_owned()
        } else {
            format!("`{p}`")
        }
    };
    if let Some(allowed) = schema["enum"].as_array()
        && !allowed.contains(value)
    {
        problems.push(not_one_of(&at(path), value, allowed));
        return;
    }
    match (schema["type"].as_str(), value) {
        (Some("string"), v) if !v.is_string() => {
            problems.push(format!("{} must be a string, not {v}", at(path)));
        }
        (Some("boolean"), v) if !v.is_boolean() => {
            problems.push(format!("{} must be true or false, not {v}", at(path)));
        }
        (Some("integer"), v) if !v.as_f64().is_some_and(|n| n.fract() == 0.0) => {
            problems.push(format!("{} must be a whole number, not {v}", at(path)));
        }
        (Some("number"), v) if !v.is_number() => {
            problems.push(format!("{} must be a number, not {v}", at(path)));
        }
        (Some("integer" | "number"), v) => range(schema, v, &at(path), problems),
        (_, Value::Object(fields)) => {
            for name in schema["required"].as_array().into_iter().flatten() {
                if let Some(name) = name.as_str()
                    && !fields.contains_key(name)
                {
                    problems.push(format!("`{}` is missing", join(path, name)));
                }
            }
            for (name, field) in fields {
                if let Some(sub) = schema["properties"].get(name) {
                    walk(sub, field, &join(path, name), problems);
                }
            }
        }
        (_, Value::Array(items)) => {
            let count = u64::try_from(items.len()).unwrap_or(u64::MAX);
            if let Some(min) = schema["minItems"].as_u64().filter(|m| count < *m) {
                problems.push(format!("{} needs at least {min} entries", at(path)));
            }
            if let Some(max) = schema["maxItems"].as_u64().filter(|m| count > *m) {
                problems.push(format!("{} takes at most {max} entries", at(path)));
            }
            if schema["items"].is_object() {
                for (i, item) in items.iter().enumerate() {
                    walk(
                        &schema["items"],
                        item,
                        &join(path, &i.to_string()),
                        problems,
                    );
                }
            }
        }
        _ => {}
    }
}

fn range(schema: &Value, value: &Value, at: &str, problems: &mut Vec<String>) {
    let Some(n) = value.as_f64() else { return };
    if let Some(min) = schema["minimum"].as_f64().filter(|m| n < *m) {
        problems.push(format!("{at} is {n}, below its minimum {min}"));
    }
    if let Some(max) = schema["maximum"].as_f64().filter(|m| n > *m) {
        problems.push(format!("{at} is {n}, above its maximum {max}"));
    }
}

fn join(path: &str, name: &str) -> String {
    if path.is_empty() {
        name.to_owned()
    } else {
        format!("{path}.{name}")
    }
}

/// "`direction` is "forwards", not one of forward, backward: did you mean forward?"
fn not_one_of(at: &str, value: &Value, allowed: &[Value]) -> String {
    let shown = |v: &Value| v.as_str().map_or_else(|| v.to_string(), str::to_owned);
    let names: Vec<String> = allowed.iter().take(LISTED).map(shown).collect();
    let more = if allowed.len() > LISTED { ", ..." } else { "" };
    let mut line = format!("{at} is {value}, not one of {}{more}", names.join(", "));
    if let Some(near) = nearest(&shown(value), &names) {
        let _ = write!(line, ": did you mean {near}?");
    }
    line
}

/// The allowed value a wrong one most likely meant: the same but for case, spaces and
/// underscores, or one holding the other.
fn nearest<'a>(wrong: &str, names: &'a [String]) -> Option<&'a str> {
    let plain = |s: &str| {
        s.chars()
            .filter(char::is_ascii_alphanumeric)
            .collect::<String>()
            .to_ascii_lowercase()
    };
    let wrong = plain(wrong);
    if wrong.is_empty() {
        return None;
    }
    names
        .iter()
        .find(|n| plain(n) == wrong)
        .or_else(|| {
            names.iter().find(|n| {
                let n = plain(n);
                n.contains(&wrong) || wrong.contains(&n)
            })
        })
        .map(String::as_str)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn schema() -> Value {
        json!({"type": "object", "properties": {
            "skill": {"type": "string", "enum": ["TurnInPlace", "WalkStraight"]},
            "retries": {"type": "integer", "minimum": 0, "maximum": 2},
            "check_only": {"type": "boolean"},
            "steps": {"type": "array", "minItems": 1, "items": {"type": "object",
                "properties": {"direction": {"type": "string", "enum": ["forward", "backward"]}},
                "required": ["direction"]}},
            "args": {"type": "array", "items": {"type": "object"}}
        }, "required": ["skill"]})
    }

    #[test]
    fn a_wrong_enum_names_the_right_values_and_the_likely_one() {
        assert_eq!(
            check(&schema(), &json!({"skill": "turn_in_place"})),
            [
                "`skill` is \"turn_in_place\", not one of TurnInPlace, WalkStraight: did you mean TurnInPlace?"
            ]
        );
        assert_eq!(
            check(
                &schema(),
                &json!({"skill": "WalkStraight", "steps": [{"direction": "forwards"}]})
            ),
            [
                "`steps.0.direction` is \"forwards\", not one of forward, backward: did you mean forward?"
            ]
        );
    }

    #[test]
    fn required_fields_types_ranges_and_lengths_are_checked() {
        // Fields come in the map's order, which depends on serde_json's features.
        let mut problems = check(
            &schema(),
            &json!({"retries": 3.5, "check_only": "yes", "steps": []}),
        );
        problems.sort();
        assert_eq!(
            problems,
            [
                "`check_only` must be true or false, not \"yes\"",
                "`retries` must be a whole number, not 3.5",
                "`skill` is missing",
                "`steps` needs at least 1 entries",
            ]
        );
        assert_eq!(
            check(&schema(), &json!({"skill": "TurnInPlace", "retries": 9})),
            ["`retries` is 9, above its maximum 2"]
        );
    }

    #[test]
    fn a_map_where_a_list_was_declared_passes_as_tools_take_both() {
        let map = json!({"skill": "TurnInPlace", "args": {"degrees": "90"}, "extra": 1});
        assert!(check(&schema(), &map).is_empty());
    }
}
