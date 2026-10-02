//! Checks a JSON value against an interface before anything converts it to a ROS message.
//!
//! r2r asserts fixed array lengths and sequence bounds while it copies a message, so a wrong value
//! would panic its spin thread instead of failing the call. This finds the problem first and says
//! where it is.

use serde_json::Value;

use crate::{Array, FieldType, Message, Part, Registry, TypeName};

/// ROS types are not recursive; this only stops a malicious nesting.
const MAX_DEPTH: usize = 32;

/// Checks `value` against `part` of `ty`: known fields only, each of its type, with fixed and
/// bounded sizes and integer ranges. Missing fields are fine: they take their defaults.
///
/// # Errors
///
/// The first problem, as `path: what is wrong`.
pub fn validate(
    registry: &Registry,
    ty: &TypeName,
    part: Part,
    value: &Value,
) -> Result<(), String> {
    let message = message_of(registry, ty, part)?;
    check_message(registry, message, value, "", 0)
}

fn message_of<'a>(
    registry: &'a Registry,
    ty: &TypeName,
    part: Part,
) -> Result<&'a Message, String> {
    registry
        .get(ty)
        .ok_or_else(|| format!("unknown interface `{ty}`"))?
        .part(part)
        .ok_or_else(|| format!("`{ty}` has no {} part", format!("{part:?}").to_lowercase()))
}

fn at(path: &str) -> String {
    if path.is_empty() {
        "the message".to_owned()
    } else {
        format!("`{path}`")
    }
}

fn check_message(
    registry: &Registry,
    message: &Message,
    value: &Value,
    path: &str,
    depth: usize,
) -> Result<(), String> {
    if depth > MAX_DEPTH {
        return Err(format!("{}: nested too deep", at(path)));
    }
    // A whole message left out takes its defaults; a null inside one has no ROS value, and r2r
    // panics on it while it converts an action goal.
    if value.is_null() && depth == 0 {
        return Ok(());
    }
    let names = || {
        let all: Vec<&str> = message.fields.iter().map(|f| f.name.as_str()).collect();
        if all.is_empty() {
            "none".to_owned()
        } else {
            all.join(", ")
        }
    };
    let Value::Object(fields) = value else {
        return Err(format!(
            "{} must be an object with the fields: {}",
            at(path),
            names()
        ));
    };
    for (name, v) in fields {
        let here = if path.is_empty() {
            name.clone()
        } else {
            format!("{path}.{name}")
        };
        let Some(field) = message.fields.iter().find(|f| &f.name == name) else {
            return Err(format!(
                "{} has no field `{name}`; its fields: {}",
                at(path),
                names()
            ));
        };
        match field.array {
            Array::Scalar => check_scalar(registry, &field.ty, v, &here, depth)?,
            array => {
                let Value::Array(items) = v else {
                    return Err(format!("{} must be a list", at(&here)));
                };
                match array {
                    Array::Fixed(n) if items.len() != n => {
                        return Err(format!(
                            "{} must have exactly {n} items, not {}",
                            at(&here),
                            items.len()
                        ));
                    }
                    Array::Bounded(n) if items.len() > n => {
                        return Err(format!(
                            "{} may have at most {n} items, not {}",
                            at(&here),
                            items.len()
                        ));
                    }
                    _ => {}
                }
                for (i, item) in items.iter().enumerate() {
                    check_scalar(registry, &field.ty, item, &format!("{here}[{i}]"), depth)?;
                }
            }
        }
    }
    Ok(())
}

fn check_scalar(
    registry: &Registry,
    ty: &FieldType,
    value: &Value,
    path: &str,
    depth: usize,
) -> Result<(), String> {
    match ty {
        FieldType::Bool if !value.is_boolean() => {
            Err(format!("{} must be true or false", at(path)))
        }
        FieldType::Bool => Ok(()),
        FieldType::F32 | FieldType::F64 => {
            let Some(x) = value.as_f64() else {
                return Err(format!("{} must be a number", at(path)));
            };
            if matches!(ty, FieldType::F32) && x.abs() > f64::from(f32::MAX) {
                return Err(format!("{} is too large for a float32", at(path)));
            }
            Ok(())
        }
        FieldType::String(bound) | FieldType::WString(bound) => {
            let Some(s) = value.as_str() else {
                return Err(format!("{} must be text", at(path)));
            };
            // r2r copies text into C strings, which end at the first NUL; it panics on one.
            if s.contains('\0') {
                return Err(format!("{} must not contain a NUL character", at(path)));
            }
            // ROS bounds a string in bytes and a wide string in UTF-16 units.
            let len = match ty {
                FieldType::WString(_) => s.encode_utf16().count(),
                _ => s.len(),
            };
            match bound {
                Some(n) if len > *n => Err(format!(
                    "{} may have at most {n} {}",
                    at(path),
                    if matches!(ty, FieldType::WString(_)) {
                        "UTF-16 units"
                    } else {
                        "bytes"
                    }
                )),
                _ => Ok(()),
            }
        }
        FieldType::Nested(inner) => {
            let message = message_of(registry, inner, Part::Message)?;
            check_message(registry, message, value, path, depth + 1)
        }
        integer => {
            let (lo, hi) = integer.integer_range().unwrap_or((i128::MIN, i128::MAX));
            let n = value
                .as_i64()
                .map(i128::from)
                .or_else(|| value.as_u64().map(i128::from));
            match n {
                Some(n) if (lo..=hi).contains(&n) => Ok(()),
                _ => Err(format!(
                    "{} must be a whole number from {lo} to {hi}",
                    at(path)
                )),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    fn registry() -> Registry {
        let mut r = Registry::default();
        for (name, text) in [
            ("demo_msgs/msg/Point", "float64 x\nfloat64 y\n"),
            (
                "demo_msgs/msg/Pose",
                "string<=4 frame\nPoint position\nfloat32[3] rpy\nint8[<=2] flags\nbool ok\n",
            ),
            (
                "demo_msgs/srv/Go",
                "Pose target\nuint8 speed\n---\nbool done\n",
            ),
        ] {
            r.add_source(&name.parse().unwrap(), text).unwrap();
        }
        r
    }

    fn go(value: &Value) -> Result<(), String> {
        let ty: TypeName = "demo_msgs/srv/Go".parse().unwrap();
        validate(&registry(), &ty, Part::Request, value)
    }

    #[test]
    fn a_well_formed_request_passes_and_missing_fields_take_defaults() {
        let full = json!({"speed": 3, "target": {"frame": "map", "position": {"x": 1.5, "y": -2},
                          "rpy": [0.0, 0.1, 0.2], "flags": [1], "ok": true}});
        assert_eq!(go(&full), Ok(()));
        assert_eq!(go(&json!({})), Ok(()));
        assert_eq!(go(&Value::Null), Ok(()));
    }

    #[test]
    fn what_r2r_would_panic_on_is_refused_with_its_path() {
        let cases = [
            (
                json!({"target": {"rpy": [0.0, 1.0]}}),
                "`target.rpy` must have exactly 3 items",
            ),
            (
                json!({"target": {"flags": [1, 2, 3]}}),
                "`target.flags` may have at most 2 items",
            ),
            (
                json!({"target": {"frame": "odometry"}}),
                "`target.frame` may have at most 4 bytes",
            ),
            (
                json!({"speed": 300}),
                "`speed` must be a whole number from 0 to 255",
            ),
            (json!({"speed": 1.5}), "`speed` must be a whole number"),
            (
                json!({"target": {"flags": [128]}}),
                "`target.flags[0]` must be a whole number",
            ),
            (
                json!({"target": {"position": {"z": 1}}}),
                "has no field `z`; its fields: x, y",
            ),
            (
                json!({"target": {"ok": "yes"}}),
                "`target.ok` must be true or false",
            ),
            (json!({"target": 7}), "`target` must be an object"),
            (json!({"target": null}), "`target` must be an object"),
            (
                json!({"target": {"position": null}}),
                "`target.position` must be an object",
            ),
            (
                json!({"target": {"frame": "a\u{0}b"}}),
                "`target.frame` must not contain a NUL",
            ),
            (
                json!({"target": {"frame": "ééé"}}),
                "`target.frame` may have at most 4 bytes",
            ),
            (
                json!({"target": {"rpy": "flat"}}),
                "`target.rpy` must be a list",
            ),
        ];
        for (value, expected) in cases {
            let err = go(&value).unwrap_err();
            assert!(err.contains(expected), "{value}: {err}");
        }
    }

    #[test]
    fn an_unknown_type_or_part_says_so() {
        let ty: TypeName = "demo_msgs/srv/Missing".parse().unwrap();
        assert!(
            validate(&registry(), &ty, Part::Request, &json!({}))
                .unwrap_err()
                .contains("unknown")
        );
        let ty: TypeName = "demo_msgs/srv/Go".parse().unwrap();
        assert!(
            validate(&registry(), &ty, Part::Goal, &json!({}))
                .unwrap_err()
                .contains("no goal")
        );
    }
}
