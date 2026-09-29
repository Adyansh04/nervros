//! Comparison of this crate's parser with the reference parser's output, shared by the golden and
//! the random-input tests. See `tests/golden.rs` for the conversions that it makes.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, dead_code)]

use std::{collections::BTreeMap, fmt::Debug, path::Path, process::Command};

use rosidl_schema::{
    Array, Constant, Field, FieldType, Interface, Message, TypeName, parse_interface,
};
use serde_json::{Value, json};

/// The reference results: package, then `kind/Name`, then the dump of one interface.
pub type Golden = BTreeMap<String, BTreeMap<String, Value>>;

/// Differences found so far, and how much was compared.
#[derive(Default)]
pub struct Diffs {
    pub found: Vec<String>,
    pub fields: usize,
    pub constants: usize,
}

impl Diffs {
    pub fn check<T: PartialEq + Debug>(&mut self, at: &str, what: &str, ours: &T, reference: &T) {
        if ours != reference {
            self.found.push(format!(
                "{at}: {what}: ours {ours:?}, reference {reference:?}"
            ));
        }
    }

    pub fn assert_none(&self, interfaces: usize) {
        let shown = self
            .found
            .iter()
            .take(30)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n");
        assert!(
            self.found.is_empty(),
            "{} difference(s) in {interfaces} interfaces; first ones:\n{shown}",
            self.found.len()
        );
    }
}

fn lines(value: &Value) -> Vec<String> {
    value
        .as_array()
        .unwrap()
        .iter()
        .map(|l| l.as_str().unwrap().to_owned())
        .collect()
}

/// Conversion 2: no doc is no lines.
fn doc_lines(doc: Option<&str>) -> Vec<String> {
    doc.map_or_else(Vec::new, |d| d.split('\n').map(str::to_owned).collect())
}

/// The reference's view of a type; conversions 1, 3 and 5.
fn reference_type(ty: &FieldType, array: Array) -> Value {
    let (pkg, name, bound) = match ty {
        FieldType::Nested(t) => (Some(t.package.as_str()), t.name.as_str(), None),
        FieldType::String(b) | FieldType::WString(b) => (None, ty.primitive_name().unwrap(), *b),
        other => (None, other.primitive_name().unwrap(), None),
    };
    let (is_array, size, upper) = match array {
        Array::Scalar => (false, None, false),
        Array::Fixed(n) => (true, Some(n), false),
        Array::Bounded(n) => (true, Some(n), true),
        Array::Unbounded => (true, None, false),
    };
    json!({"pkg": pkg, "name": name, "bound": bound, "array": is_array, "size": size, "upper": upper})
}

/// Conversion 1 applied to the reference side.
fn builtin_time(mut ty: Value) -> Value {
    if ty["pkg"].is_null() {
        let name = match ty["name"].as_str() {
            Some("time") => "Time",
            Some("duration") => "Duration",
            _ => return ty,
        };
        ty["pkg"] = json!("builtin_interfaces");
        ty["name"] = json!(name);
    }
    ty
}

/// Whether a reference value holds a NaN or an infinity, which JSON, and so `default_json`, cannot.
fn has_nonfinite(value: &Value) -> bool {
    match value {
        Value::Object(map) => map.contains_key("nonfinite"),
        Value::Array(items) => items.iter().any(has_nonfinite),
        _ => false,
    }
}

/// Compares a typed value with the reference's; a non-finite one only has to be missing from JSON.
fn check_value(
    d: &mut Diffs,
    at: &str,
    what: &str,
    ours: Option<&Value>,
    reference: Option<&Value>,
) {
    if reference.is_some_and(has_nonfinite) {
        d.check(at, what, &ours, &None);
    } else {
        d.check(at, what, &ours, &reference);
    }
}

fn compare_field(d: &mut Diffs, at: &str, ours: &Field, reference: &Value) {
    d.fields += 1;
    d.check(
        at,
        "type",
        &reference_type(&ours.ty, ours.array),
        &builtin_time(reference["type"].clone()),
    );
    check_value(
        d,
        at,
        "default",
        ours.default_json().as_ref(),
        reference.get("default"),
    );
    d.check(
        at,
        "has default",
        &ours.default.is_some(),
        &reference.get("default").is_some(),
    );
    d.check(
        at,
        "comment",
        &doc_lines(ours.doc.as_deref()),
        &lines(&reference["comment"]),
    );
    d.check(
        at,
        "unit",
        &ours.unit.as_deref(),
        &reference["unit"].as_str(),
    );
}

fn compare_constant(d: &mut Diffs, at: &str, ours: &Constant, reference: &Value) {
    d.constants += 1;
    d.check(
        at,
        "type",
        &ours.ty.primitive_name(),
        &reference["type"].as_str(),
    );
    check_value(
        d,
        at,
        "value",
        ours.json_value().as_ref(),
        Some(&reference["value"]),
    );
    d.check(
        at,
        "comment",
        &doc_lines(ours.doc.as_deref()),
        &lines(&reference["comment"]),
    );
}

fn compare_message(d: &mut Diffs, at: &str, ours: &Message, reference: &Value) {
    d.check(
        at,
        "message comment",
        &doc_lines(ours.doc.as_deref()),
        &lines(&reference["comment"]),
    );
    let names = |v: &Value| -> Vec<String> {
        v.as_array()
            .unwrap()
            .iter()
            .map(|e| e["name"].as_str().unwrap().to_owned())
            .collect()
    };
    let (fields, constants) = (&reference["fields"], &reference["constants"]);
    d.check(
        at,
        "field names",
        &ours
            .fields
            .iter()
            .map(|f| f.name.clone())
            .collect::<Vec<_>>(),
        &names(fields),
    );
    d.check(
        at,
        "constant names",
        &ours
            .constants
            .iter()
            .map(|c| c.name.clone())
            .collect::<Vec<_>>(),
        &names(constants),
    );
    for (f, r) in ours.fields.iter().zip(fields.as_array().unwrap()) {
        compare_field(d, &format!("{at}.{}", f.name), f, r);
    }
    for (c, r) in ours.constants.iter().zip(constants.as_array().unwrap()) {
        compare_constant(d, &format!("{at}::{}", c.name), c, r);
    }
}

pub fn compare_interface(d: &mut Diffs, at: &str, ours: &Interface, reference: &Value) {
    match (ours, reference["kind"].as_str().unwrap()) {
        (Interface::Message(m), "msg") => compare_message(d, at, m, &reference["message"]),
        (Interface::Service(s), "srv") => {
            compare_message(
                d,
                &format!("{at}/request"),
                &s.request,
                &reference["request"],
            );
            compare_message(
                d,
                &format!("{at}/response"),
                &s.response,
                &reference["response"],
            );
        }
        (Interface::Action(a), "action") => {
            compare_message(d, &format!("{at}/goal"), &a.goal, &reference["goal"]);
            compare_message(d, &format!("{at}/result"), &a.result, &reference["result"]);
            compare_message(
                d,
                &format!("{at}/feedback"),
                &a.feedback,
                &reference["feedback"],
            );
        }
        (_, kind) => d.found.push(format!("{at}: reference says `{kind}`")),
    }
}

/// Compares one parsed interface with its reference entry `kind/Name` of `package`. A file the
/// reference rejected (only with `--lenient`) has to be rejected here too.
pub fn compare_entry(d: &mut Diffs, package: &str, key: &str, text: &str, reference: &Value) {
    let name: TypeName = format!("{package}/{key}").parse().unwrap();
    match (parse_interface(&name, text), reference.get("error")) {
        (Ok(ours), None) => compare_interface(d, &name.to_string(), &ours, reference),
        (Err(_), Some(_)) => {}
        (Ok(_), Some(error)) => d
            .found
            .push(format!("{name}: accepted, but the reference says {error}")),
        (Err(e), None) => d.found.push(format!("{name}: does not parse: {e}")),
    }
}

/// Runs `tools/dump_golden.py` with `args` and returns what it dumped.
pub fn run_reference(args: &[&Path], lenient: bool, ros_prefix: &Path) -> Golden {
    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("tools/dump_golden.py");
    let mut command = Command::new("python3");
    command.arg(script).arg("--ros-prefix").arg(ros_prefix);
    if lenient {
        command.arg("--lenient");
    }
    let output = command.args(args).output().unwrap();
    assert!(
        output.status.success(),
        "the reference run failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    serde_json::from_slice(&output.stdout).unwrap()
}
