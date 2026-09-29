//! The parsed form of ROS 2 interface files.

use std::{fmt, str::FromStr};

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::{
    error::{Error, Result},
    parse::{is_valid_message_name, is_valid_package_name},
    value,
};

/// Which kind of interface file a type lives in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Kind {
    /// A `.msg` file.
    Msg,
    /// A `.srv` file.
    Srv,
    /// An `.action` file.
    Action,
}

impl Kind {
    /// The directory and type-string segment: `msg`, `srv` or `action`.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Msg => "msg",
            Self::Srv => "srv",
            Self::Action => "action",
        }
    }
}

impl fmt::Display for Kind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// The full name of an interface, such as `geometry_msgs/msg/PoseStamped`.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TypeName {
    /// The ROS package that defines the interface.
    pub package: String,
    /// Whether it is a message, a service or an action.
    pub kind: Kind,
    /// The interface name without the extension, in `CamelCase`.
    pub name: String,
}

impl fmt::Display for TypeName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}/{}", self.package, self.kind, self.name)
    }
}

impl FromStr for TypeName {
    type Err = Error;

    /// Accepts `pkg/msg/Name`, `pkg/srv/Name`, `pkg/action/Name` and the two-part `pkg/Name`,
    /// which means a message.
    fn from_str(s: &str) -> Result<Self> {
        let bad = |reason: &str| Error::TypeName {
            input: s.to_owned(),
            reason: reason.to_owned(),
        };
        let parts: Vec<&str> = s.split('/').collect();
        let (package, kind, name) = match parts.as_slice() {
            [package, name] => (*package, Kind::Msg, *name),
            [package, kind, name] => {
                let kind = match *kind {
                    "msg" => Kind::Msg,
                    "srv" => Kind::Srv,
                    "action" => Kind::Action,
                    _ => return Err(bad("the middle part must be `msg`, `srv` or `action`")),
                };
                (*package, kind, *name)
            }
            _ => return Err(bad("expected `package/msg/Name` or `package/Name`")),
        };
        if !is_valid_package_name(package) {
            return Err(bad(
                "the package name must be lower case letters, digits and single `_`",
            ));
        }
        if !is_valid_message_name(name) {
            return Err(bad("the interface name must be `CamelCase`"));
        }
        Ok(Self {
            package: package.to_owned(),
            kind,
            name: name.to_owned(),
        })
    }
}

impl Serialize for TypeName {
    fn serialize<S: Serializer>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error> {
        serializer.collect_str(self)
    }
}

impl<'de> Deserialize<'de> for TypeName {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> std::result::Result<Self, D::Error> {
        let text = String::deserialize(deserializer)?;
        text.parse().map_err(serde::de::Error::custom)
    }
}

/// The type of a field or constant, without its array shape.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum FieldType {
    /// `bool`.
    Bool,
    /// `byte`, an opaque octet.
    Byte,
    /// `char`, an unsigned 8-bit integer in ROS 2.
    Char,
    /// `int8`.
    I8,
    /// `uint8`.
    U8,
    /// `int16`.
    I16,
    /// `uint16`.
    U16,
    /// `int32`.
    I32,
    /// `uint32`.
    U32,
    /// `int64`.
    I64,
    /// `uint64`.
    U64,
    /// `float32`.
    F32,
    /// `float64`.
    F64,
    /// `string`, or `string<=N` when bounded.
    String(Option<usize>),
    /// `wstring`, or `wstring<=N` when bounded.
    WString(Option<usize>),
    /// Another message. `time` and `duration` become `builtin_interfaces/Time` and `Duration`.
    Nested(TypeName),
}

impl FieldType {
    /// Maps a ROS primitive name (without a bound) to its type.
    pub(crate) fn from_primitive(name: &str) -> Option<Self> {
        Some(match name {
            "bool" => Self::Bool,
            "byte" => Self::Byte,
            "char" => Self::Char,
            "int8" => Self::I8,
            "uint8" => Self::U8,
            "int16" => Self::I16,
            "uint16" => Self::U16,
            "int32" => Self::I32,
            "uint32" => Self::U32,
            "int64" => Self::I64,
            "uint64" => Self::U64,
            "float32" => Self::F32,
            "float64" => Self::F64,
            "string" => Self::String(None),
            "wstring" => Self::WString(None),
            _ => return None,
        })
    }

    /// The ROS name of a primitive type without any bound, or `None` for a nested message.
    #[must_use]
    pub fn primitive_name(&self) -> Option<&'static str> {
        Some(match self {
            Self::Bool => "bool",
            Self::Byte => "byte",
            Self::Char => "char",
            Self::I8 => "int8",
            Self::U8 => "uint8",
            Self::I16 => "int16",
            Self::U16 => "uint16",
            Self::I32 => "int32",
            Self::U32 => "uint32",
            Self::I64 => "int64",
            Self::U64 => "uint64",
            Self::F32 => "float32",
            Self::F64 => "float64",
            Self::String(_) => "string",
            Self::WString(_) => "wstring",
            Self::Nested(_) => return None,
        })
    }

    /// The smallest and largest value of an integer type, including `byte` and `char`.
    #[must_use]
    pub fn integer_range(&self) -> Option<(i128, i128)> {
        Some(match self {
            Self::Byte | Self::Char | Self::U8 => (0, u8::MAX.into()),
            Self::I8 => (i8::MIN.into(), i8::MAX.into()),
            Self::I16 => (i16::MIN.into(), i16::MAX.into()),
            Self::U16 => (0, u16::MAX.into()),
            Self::I32 => (i32::MIN.into(), i32::MAX.into()),
            Self::U32 => (0, u32::MAX.into()),
            Self::I64 => (i64::MIN.into(), i64::MAX.into()),
            Self::U64 => (0, u64::MAX.into()),
            _ => return None,
        })
    }

    /// Whether two types are the same apart from a string bound.
    pub(crate) fn same_kind(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::String(_), Self::String(_)) | (Self::WString(_), Self::WString(_)) => true,
            _ => self == other,
        }
    }
}

impl fmt::Display for FieldType {
    /// Writes the type as a `.msg` file does, without the array suffix.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::String(Some(n)) | Self::WString(Some(n)) => {
                write!(f, "{}<={n}", self.primitive_name().unwrap_or_default())
            }
            Self::Nested(t) => write!(f, "{}/{}", t.package, t.name),
            _ => f.write_str(self.primitive_name().unwrap_or_default()),
        }
    }
}

/// The array shape of a field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Array {
    /// A single value: `T`.
    Scalar,
    /// Exactly this many values: `T[N]`.
    Fixed(usize),
    /// At most this many values: `T[<=N]`.
    Bounded(usize),
    /// Any number of values: `T[]`.
    Unbounded,
}

impl fmt::Display for Array {
    /// Writes the suffix as a `.msg` file does: nothing, `[N]`, `[<=N]` or `[]`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Scalar => Ok(()),
            Self::Fixed(n) => write!(f, "[{n}]"),
            Self::Bounded(n) => write!(f, "[<={n}]"),
            Self::Unbounded => f.write_str("[]"),
        }
    }
}

/// One field of a message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Field {
    /// The field name, `snake_case`.
    pub name: String,
    /// The element type.
    pub ty: FieldType,
    /// Whether the field is a single value or an array, and of what size.
    pub array: Array,
    /// The default value exactly as written in the file (quotes included), when there is one.
    pub default: Option<String>,
    /// The comment above, beside or below the field, dedented.
    pub doc: Option<String>,
    /// The `[unit]` found in the comment and removed from it.
    pub unit: Option<String>,
}

impl Field {
    /// The default as a JSON value, or `None` when there is no default or JSON cannot hold it
    /// (a NaN or infinity).
    #[must_use]
    pub fn default_json(&self) -> Option<serde_json::Value> {
        let text = self.default.as_deref()?;
        value::parse_value(&self.ty, self.array, text)
            .ok()?
            .to_json()
    }

    /// Whether this is a `uint8[]` or `byte[]` of open length: a payload, not something to fill in.
    #[must_use]
    pub fn is_blob(&self) -> bool {
        matches!(self.ty, FieldType::U8 | FieldType::Byte)
            && matches!(self.array, Array::Unbounded | Array::Bounded(_))
    }
}

/// One constant of a message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Constant {
    /// The constant name, `UPPER_SNAKE_CASE`.
    pub name: String,
    /// A primitive type.
    pub ty: FieldType,
    /// The value exactly as written in the file (quotes included).
    pub value: String,
    /// The comment above or beside the constant, dedented.
    pub doc: Option<String>,
}

impl Constant {
    /// The value as JSON: an integer, a number, a boolean or the string without its quotes.
    #[must_use]
    pub fn json_value(&self) -> Option<serde_json::Value> {
        value::parse_value(&self.ty, Array::Scalar, &self.value)
            .ok()?
            .to_json()
    }
}

/// A message, or one half of a service or action.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Message {
    /// The comment lines at the top of the file, before the first blank or definition line.
    pub doc: Option<String>,
    /// The fields in file order.
    pub fields: Vec<Field>,
    /// The constants in file order.
    pub constants: Vec<Constant>,
}

impl Message {
    fn first_element_has_comment(&self) -> bool {
        match (self.constants.first(), self.fields.first()) {
            (Some(c), _) => c.doc.is_some(),
            (None, Some(f)) => f.doc.is_some() || f.unit.is_some(),
            (None, None) => false,
        }
    }
}

/// Writes the message back as `.msg` text: constants first, then fields, each after its comment.
///
/// Parsing the output gives an equal [`Message`]. Comment layout is not kept.
impl fmt::Display for Message {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(doc) = &self.doc {
            for line in doc.split('\n') {
                // A blank comment line needs its space, or the parser drops it as a blank line.
                writeln!(f, "# {line}")?;
            }
            writeln!(f)?;
        } else if self.first_element_has_comment() {
            // Otherwise the element's comment would read as the message's.
            writeln!(f)?;
        }
        for c in &self.constants {
            write_comment(f, c.doc.as_deref(), None)?;
            writeln!(f, "{} {}={}", c.ty, c.name, c.value)?;
        }
        for field in &self.fields {
            write_comment(f, field.doc.as_deref(), field.unit.as_deref())?;
            write!(f, "{}{} {}", field.ty, field.array, field.name)?;
            if let Some(default) = &field.default {
                write!(f, " {default}")?;
            }
            writeln!(f)?;
        }
        Ok(())
    }
}

/// Writes the comment block above an element, with its unit at the end of the last line.
fn write_comment(f: &mut fmt::Formatter<'_>, doc: Option<&str>, unit: Option<&str>) -> fmt::Result {
    let lines: Vec<&str> = doc.map_or_else(Vec::new, |d| d.split('\n').collect());
    for (i, line) in lines.iter().enumerate() {
        let unit = unit.filter(|_| i + 1 == lines.len());
        match (line.is_empty(), unit) {
            (true, None) => writeln!(f, "#")?,
            (true, Some(unit)) => writeln!(f, "# [{unit}]")?,
            (false, None) => writeln!(f, "# {line}")?,
            (false, Some(unit)) => writeln!(f, "# {line} [{unit}]")?,
        }
    }
    match (lines.is_empty(), unit) {
        (true, Some(unit)) => writeln!(f, "# [{unit}]"),
        _ => Ok(()),
    }
}

/// A service: a request and a response message.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Service {
    /// The part above `---`.
    pub request: Message,
    /// The part below `---`.
    pub response: Message,
}

/// An action: a goal, a result and feedback message.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Action {
    /// The part before the first `---`.
    pub goal: Message,
    /// The part between the two `---` lines.
    pub result: Message,
    /// The part after the second `---`.
    pub feedback: Message,
}

/// A parsed interface file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Interface {
    /// A `.msg` file.
    Message(Message),
    /// A `.srv` file.
    Service(Service),
    /// An `.action` file.
    Action(Action),
}

impl Interface {
    /// Every message the interface is made of: itself, both halves or all three.
    pub(crate) fn messages(&self) -> Vec<&Message> {
        match self {
            Self::Message(m) => vec![m],
            Self::Service(s) => vec![&s.request, &s.response],
            Self::Action(a) => vec![&a.goal, &a.result, &a.feedback],
        }
    }

    /// The part of the interface named by `part`, or `None` when it has no such part.
    #[must_use]
    pub fn part(&self, part: Part) -> Option<&Message> {
        match (self, part) {
            (Self::Message(m), Part::Message) => Some(m),
            (Self::Service(s), Part::Request) => Some(&s.request),
            (Self::Service(s), Part::Response) => Some(&s.response),
            (Self::Action(a), Part::Goal) => Some(&a.goal),
            (Self::Action(a), Part::Result) => Some(&a.result),
            (Self::Action(a), Part::Feedback) => Some(&a.feedback),
            _ => None,
        }
    }
}

/// Which message of an interface a schema describes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Part {
    /// A `.msg` file.
    Message,
    /// The request of a service.
    Request,
    /// The response of a service.
    Response,
    /// The goal of an action.
    Goal,
    /// The result of an action.
    Result,
    /// The feedback of an action.
    Feedback,
}

impl fmt::Display for Part {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Message => "message",
            Self::Request => "request",
            Self::Response => "response",
            Self::Goal => "goal",
            Self::Result => "result",
            Self::Feedback => "feedback",
        })
    }
}
