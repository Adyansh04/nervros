//! JSON Schema (draft 2020-12) for the parts of an interface.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use crate::{
    error::{Error, Result},
    registry::Registry,
    types::{Array, Constant, Field, FieldType, Message, Part, TypeName},
    value::json_int,
};

/// `$ref`s nested deeper than this are taken for a cycle by [`flatten_refs`].
const MAX_REF_DEPTH: usize = 64;

/// Changes to the generated schema, keyed by dotted field path.
///
/// A path names a field of the part, such as `arm`, and goes into nested messages with dots, such
/// as `pose.position.x`; arrays are passed through without an index.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Overrides {
    /// Replaces the description of the top-level object, constants list included.
    pub description: Option<String>,
    /// Whether constants named `X_*` give the field `x` of the same type an `enum` of their values.
    pub enum_from_constants: bool,
    /// Per-field changes.
    pub fields: BTreeMap<String, FieldOverride>,
}

impl Default for Overrides {
    fn default() -> Self {
        Self {
            description: None,
            enum_from_constants: true,
            fields: BTreeMap::new(),
        }
    }
}

/// Changes to one field of the schema.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FieldOverride {
    /// Replaces the whole description, unit included.
    pub description: Option<String>,
    /// `true` leaves the field out. `false` keeps a `uint8[]` or `byte[]` field that would go.
    pub hidden: Option<bool>,
    /// Replaces the default.
    pub default: Option<Value>,
    /// Replaces the `enum`, on the items of an array field; an empty list removes the one that
    /// constants would give.
    #[serde(rename = "enum")]
    pub enum_values: Option<Vec<Value>>,
    /// Raises the lower bound of a number, or of the items of an array, within the type's range.
    pub min: Option<f64>,
    /// Lowers the upper bound of a number, or of the items of an array, within the type's range.
    pub max: Option<f64>,
    /// Lists the field in the parent's `required`, unless it is hidden. ROS fields are never
    /// required otherwise, since every field has a zero value.
    pub required: bool,
}

/// The JSON Schema of one part of an interface.
///
/// Nested messages go into `$defs` and are referenced with `$ref`, except where an override reaches
/// into them, which gets its own inline copy. Every object is closed with
/// `additionalProperties: false`, and nothing is `required` unless an override says so. The
/// `uint8[]` and `byte[]` fields of open length are left out unless an override keeps them.
///
/// # Errors
/// - [`Error::UnknownType`] when `ty`, or a message it contains, is not in the registry.
/// - [`Error::NoSuchPart`] when the interface lacks `part`, such as the goal of a message.
/// - [`Error::UnknownOverride`] when an override names a path with no field behind it.
/// - [`Error::Recursive`] for a message that contains itself.
pub fn json_schema(
    registry: &Registry,
    ty: &TypeName,
    part: Part,
    overrides: &Overrides,
) -> Result<Value> {
    let interface = registry.get(ty).ok_or_else(|| Error::UnknownType {
        name: ty.clone(),
        used_by: None,
    })?;
    let message = interface.part(part).ok_or_else(|| Error::NoSuchPart {
        ty: ty.clone(),
        part,
    })?;
    let mut builder = Builder {
        registry,
        overrides,
        defs: BTreeMap::new(),
        open: vec![ty.clone()],
        used: BTreeSet::new(),
    };
    let mut root = builder.object(message, Some(""), ty)?;
    if let Some(description) = &overrides.description {
        root.insert("description".to_owned(), Value::String(description.clone()));
    }
    root.insert(
        "$schema".to_owned(),
        Value::String("https://json-schema.org/draft/2020-12/schema".to_owned()),
    );
    if !builder.defs.is_empty() {
        root.insert(
            "$defs".to_owned(),
            Value::Object(builder.defs.into_iter().collect()),
        );
    }
    if let Some(path) = overrides.fields.keys().find(|p| !builder.used.contains(*p)) {
        return Err(Error::UnknownOverride(path.clone()));
    }
    Ok(Value::Object(root))
}

/// The schema with every `$ref` replaced by the definition it points to and `$defs` removed, for
/// providers that do not accept references. Other keywords next to a `$ref` win over the
/// definition's, so a field's own description survives.
///
/// # Errors
/// [`Error::BadRef`] for a reference that is not `#/$defs/<name>` or whose definition is missing,
/// and [`Error::RefDepth`] for references nested too deeply, which means a cycle.
pub fn flatten_refs(schema: &Value) -> Result<Value> {
    let defs = schema
        .get("$defs")
        .and_then(Value::as_object)
        .cloned()
        .unwrap_or_default();
    let mut flat = inline(schema, &defs, 0)?;
    if let Value::Object(map) = &mut flat {
        map.remove("$defs");
    }
    Ok(flat)
}

fn inline(value: &Value, defs: &Map<String, Value>, depth: usize) -> Result<Value> {
    match value {
        Value::Object(map) => {
            let mut out = Map::new();
            if let Some(Value::String(reference)) = map.get("$ref") {
                if depth >= MAX_REF_DEPTH {
                    return Err(Error::RefDepth(MAX_REF_DEPTH));
                }
                let target = reference
                    .strip_prefix("#/$defs/")
                    .and_then(|key| defs.get(key))
                    .ok_or_else(|| Error::BadRef(reference.clone()))?;
                match inline(target, defs, depth + 1)? {
                    Value::Object(definition) => out = definition,
                    other => return Ok(other),
                }
            }
            for (key, child) in map.iter().filter(|(k, _)| k.as_str() != "$ref") {
                out.insert(key.clone(), inline(child, defs, depth)?);
            }
            Ok(Value::Object(out))
        }
        Value::Array(items) => items
            .iter()
            .map(|item| inline(item, defs, depth))
            .collect::<Result<_>>()
            .map(Value::Array),
        _ => Ok(value.clone()),
    }
}

struct Builder<'a> {
    registry: &'a Registry,
    overrides: &'a Overrides,
    defs: BTreeMap<String, Value>,
    /// The messages being built, to catch one that contains itself.
    open: Vec<TypeName>,
    /// The override paths that matched a field.
    used: BTreeSet<String>,
}

impl Builder<'_> {
    /// The object schema of a message. `path` is its dotted path for overrides, `""` for the root,
    /// or `None` where no override applies, as in a shared definition.
    fn object(
        &mut self,
        message: &Message,
        path: Option<&str>,
        owner: &TypeName,
    ) -> Result<Map<String, Value>> {
        let overrides = self.overrides;
        let mut properties = Map::new();
        let mut required = Vec::new();
        for field in &message.fields {
            let field_path = path.map(|p| {
                if p.is_empty() {
                    field.name.clone()
                } else {
                    format!("{p}.{}", field.name)
                }
            });
            let over = field_path.as_deref().and_then(|p| overrides.fields.get(p));
            if let (Some(p), Some(_)) = (&field_path, over) {
                self.used.insert(p.clone());
            }
            if over
                .and_then(|o| o.hidden)
                .unwrap_or_else(|| field.is_blob())
            {
                continue;
            }
            let schema = self.field(message, field, field_path.as_deref(), over, owner)?;
            properties.insert(field.name.clone(), schema);
            if over.is_some_and(|o| o.required) {
                required.push(Value::String(field.name.clone()));
            }
        }
        let mut object = Map::new();
        object.insert("type".to_owned(), Value::String("object".to_owned()));
        if let Some(description) = message_description(message) {
            object.insert("description".to_owned(), Value::String(description));
        }
        object.insert("properties".to_owned(), Value::Object(properties));
        if !required.is_empty() {
            object.insert("required".to_owned(), Value::Array(required));
        }
        object.insert("additionalProperties".to_owned(), Value::Bool(false));
        Ok(object)
    }

    fn field(
        &mut self,
        message: &Message,
        field: &Field,
        path: Option<&str>,
        over: Option<&FieldOverride>,
        owner: &TypeName,
    ) -> Result<Value> {
        let mut item = match &field.ty {
            FieldType::Nested(name) => self.nested(name, path, owner)?,
            primitive => primitive_schema(primitive, over),
        };
        if !matches!(field.ty, FieldType::Nested(_)) {
            let choices = match over.and_then(|o| o.enum_values.as_ref()) {
                Some(values) => (!values.is_empty()).then(|| values.clone()),
                None if self.overrides.enum_from_constants => constant_choices(message, field),
                None => None,
            };
            if let Some(values) = choices {
                item.insert("enum".to_owned(), Value::Array(values));
            }
        }

        let mut schema = if field.array == Array::Scalar {
            item
        } else {
            let mut array = Map::new();
            array.insert("type".to_owned(), Value::String("array".to_owned()));
            array.insert("items".to_owned(), Value::Object(item));
            match field.array {
                Array::Fixed(n) => {
                    array.insert("minItems".to_owned(), Value::from(n));
                    array.insert("maxItems".to_owned(), Value::from(n));
                }
                Array::Bounded(n) => {
                    array.insert("maxItems".to_owned(), Value::from(n));
                }
                Array::Scalar | Array::Unbounded => {}
            }
            array
        };
        let description = over
            .and_then(|o| o.description.clone())
            .or_else(|| field_description(field));
        if let Some(description) = description {
            schema.insert("description".to_owned(), Value::String(description));
        }
        if let Some(default) = over
            .and_then(|o| o.default.clone())
            .or_else(|| field.default_json())
        {
            schema.insert("default".to_owned(), default);
        }
        Ok(Value::Object(schema))
    }

    /// A `$ref` to the shared definition of a nested message, or the message inline when an
    /// override reaches into it.
    fn nested(
        &mut self,
        name: &TypeName,
        path: Option<&str>,
        owner: &TypeName,
    ) -> Result<Map<String, Value>> {
        let registry = self.registry;
        let (found, message) = registry
            .find_message(name)
            .ok_or_else(|| Error::UnknownType {
                name: name.clone(),
                used_by: Some(owner.clone()),
            })?;
        if self.open.contains(found) {
            return Err(Error::Recursive(found.clone()));
        }
        let reaches_in = path.filter(|p| {
            let below = format!("{p}.");
            self.overrides.fields.keys().any(|k| k.starts_with(&below))
        });
        if let Some(path) = reaches_in {
            self.open.push(found.clone());
            let object = self.object(message, Some(path), found);
            self.open.pop();
            return object;
        }
        let key = format!("{}.{}", found.package, found.name);
        if !self.defs.contains_key(&key) {
            self.open.push(found.clone());
            let object = self.object(message, None, found);
            self.open.pop();
            self.defs.insert(key.clone(), Value::Object(object?));
        }
        let mut reference = Map::new();
        reference.insert("$ref".to_owned(), Value::String(format!("#/$defs/{key}")));
        Ok(reference)
    }
}

fn primitive_schema(ty: &FieldType, over: Option<&FieldOverride>) -> Map<String, Value> {
    let mut schema = Map::new();
    let mut set = |key: &str, value: Value| schema.insert(key.to_owned(), value);
    match ty {
        FieldType::Bool => {
            set("type", Value::String("boolean".to_owned()));
        }
        FieldType::F32 | FieldType::F64 => {
            set("type", Value::String("number".to_owned()));
            for (key, bound) in [
                ("minimum", over.and_then(|o| o.min)),
                ("maximum", over.and_then(|o| o.max)),
            ] {
                if let Some(number) = bound.and_then(serde_json::Number::from_f64) {
                    set(key, Value::Number(number));
                }
            }
        }
        FieldType::String(bound) | FieldType::WString(bound) => {
            set("type", Value::String("string".to_owned()));
            if let Some(max) = bound {
                set("maxLength", Value::from(*max));
            }
        }
        // The caller builds nested messages.
        FieldType::Nested(_) => {}
        integer => {
            let (mut lo, mut hi) = integer.integer_range().unwrap_or((0, 0));
            if let Some(min) = over.and_then(|o| o.min) {
                lo = lo.max(to_int(min.ceil()));
            }
            if let Some(max) = over.and_then(|o| o.max) {
                hi = hi.min(to_int(max.floor()));
            }
            set("type", Value::String("integer".to_owned()));
            set("minimum", json_int(lo).unwrap_or_default());
            set("maximum", json_int(hi).unwrap_or_default());
        }
    }
    schema
}

/// A float as an integer, saturating at the ends of the range.
#[expect(
    clippy::cast_possible_truncation,
    reason = "the cast saturates, which is what a bound needs"
)]
fn to_int(x: f64) -> i128 {
    x as i128
}

/// The values of the constants `X_*` of the same type as the field `x`.
fn constant_choices(message: &Message, field: &Field) -> Option<Vec<Value>> {
    let prefix = format!("{}_", field.name.to_uppercase());
    let values: Vec<Value> = message
        .constants
        .iter()
        .filter(|c| c.name.starts_with(&prefix) && c.ty.same_kind(&field.ty))
        .filter_map(Constant::json_value)
        .collect();
    (!values.is_empty()).then_some(values)
}

fn field_description(field: &Field) -> Option<String> {
    let doc = field.doc.as_deref().filter(|d| !d.is_empty());
    match (doc, field.unit.as_deref()) {
        (Some(doc), Some(unit)) => Some(format!("{doc} [{unit}]")),
        (Some(doc), None) => Some(doc.to_owned()),
        (None, Some(unit)) => Some(format!("[{unit}]")),
        (None, None) => None,
    }
}

/// The message doc, then the constants as `NAME=value` lines.
fn message_description(message: &Message) -> Option<String> {
    let mut parts = Vec::new();
    if let Some(doc) = message.doc.as_deref().filter(|d| !d.is_empty()) {
        parts.push(doc.to_owned());
    }
    if !message.constants.is_empty() {
        let lines: Vec<String> = message
            .constants
            .iter()
            .map(|c| format!("{}={}", c.name, constant_text(c)))
            .collect();
        parts.push(format!("Constants:\n{}", lines.join("\n")));
    }
    (!parts.is_empty()).then(|| parts.join("\n\n"))
}

fn constant_text(constant: &Constant) -> String {
    match constant.json_value() {
        Some(Value::String(s)) if s.is_empty() => "\"\"".to_owned(),
        Some(Value::String(s)) => s,
        Some(other) => other.to_string(),
        None => constant.value.clone(),
    }
}
