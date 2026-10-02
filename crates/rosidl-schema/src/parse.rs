//! The `.msg`, `.srv` and `.action` parser.
//!
//! A port of `rosidl_adapter/parser.py` from ROS 2 Jazzy that keeps what that parser keeps only
//! as annotations: comments and units. Where this file says "Python", it means that parser.

use std::collections::HashSet;

use crate::{
    error::{Error, ParseError, ParseErrorKind, Result},
    types::{
        Action, Array, Constant, Field, FieldType, Interface, Kind, Message, Service, TypeName,
    },
    value,
};

/// Python's `str.isspace`: Unicode white space plus the ASCII information separators.
pub(crate) fn is_space(c: char) -> bool {
    c.is_whitespace() || ('\x1c'..='\x1f').contains(&c)
}

pub(crate) fn trim(s: &str) -> &str {
    s.trim_matches(is_space)
}

fn trim_start(s: &str) -> &str {
    s.trim_start_matches(is_space)
}

fn trim_end(s: &str) -> &str {
    s.trim_end_matches(is_space)
}

/// Python's `str.splitlines()`: every Unicode line boundary, with `\r\n` counted once.
fn split_lines(text: &str) -> Vec<&str> {
    let mut lines = Vec::new();
    let mut start = 0;
    let mut chars = text.char_indices().peekable();
    while let Some((i, c)) = chars.next() {
        if !matches!(
            c,
            '\n' | '\r'
                | '\x0b'
                | '\x0c'
                | '\x1c'
                | '\x1d'
                | '\x1e'
                | '\u{85}'
                | '\u{2028}'
                | '\u{2029}'
        ) {
            continue;
        }
        lines.push(&text[start..i]);
        start = i + c.len_utf8();
        if c == '\r'
            && let Some(&(j, '\n')) = chars.peek()
        {
            chars.next();
            start = j + 1;
        }
    }
    if start < text.len() {
        lines.push(&text[start..]);
    }
    lines
}

/// Package and field names: lower case, digits and single underscores, starting with a letter.
pub(crate) fn is_valid_package_name(name: &str) -> bool {
    name.chars().next().is_some_and(|c| c.is_ascii_lowercase())
        && name
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
        && !name.contains("__")
        && !name.ends_with('_')
}

/// Message names: `CamelCase`, after the `Sample_` prefix and the service and action suffixes.
pub(crate) fn is_valid_message_name(name: &str) -> bool {
    let mut name = name.strip_prefix("Sample_").unwrap_or(name);
    for suffix in ["_Request", "_Response", "_Goal", "_Result", "_Feedback"] {
        name = name.strip_suffix(suffix).unwrap_or(name);
    }
    name.chars().next().is_some_and(|c| c.is_ascii_uppercase())
        && name.chars().all(|c| c.is_ascii_alphanumeric())
}

fn is_valid_constant_name(name: &str) -> bool {
    name.chars().next().is_some_and(|c| c.is_ascii_uppercase())
        && name
            .chars()
            .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
        && !name.contains("__")
        && !name.ends_with('_')
}

/// Parses the text of the interface a [`TypeName`] names.
///
/// # Errors
/// Returns [`Error::Parse`] with the line of the first problem.
pub fn parse_interface(name: &TypeName, text: &str) -> Result<Interface> {
    match name.kind {
        Kind::Msg => parse_message(&name.package, &name.name, text).map(Interface::Message),
        Kind::Srv => parse_service(&name.package, &name.name, text).map(Interface::Service),
        Kind::Action => parse_action(&name.package, &name.name, text).map(Interface::Action),
    }
}

/// Parses the text of a `.msg` file of `package`.
///
/// # Errors
/// Returns [`Error::TypeName`] for a bad package or message name and [`Error::Parse`] with the
/// line of the first problem in the text.
pub fn parse_message(package: &str, name: &str, text: &str) -> Result<Message> {
    let cx = Context::new(package, Kind::Msg, name)?;
    let text = text.replace('\t', " ");
    parse_lines(&cx, &split_lines(&text), 0)
}

/// Parses the text of a `.srv` file of `package`: a request, a `---` line and a response.
///
/// # Errors
/// As [`parse_message`], and [`ParseErrorKind::Separators`] unless there is exactly one `---`.
pub fn parse_service(package: &str, name: &str, text: &str) -> Result<Service> {
    let cx = Context::new(package, Kind::Srv, name)?;
    let text = text.replace('\t', " ");
    let lines = split_lines(&text);
    let [sep] = separators(&cx, &lines)?;
    Ok(Service {
        request: parse_lines(&cx, &lines[..sep], 0)?,
        response: parse_lines(&cx, &lines[sep + 1..], sep + 1)?,
    })
}

/// Parses the text of an `.action` file of `package`: goal, result and feedback between two `---`.
///
/// # Errors
/// As [`parse_message`], and [`ParseErrorKind::Separators`] unless there are exactly two `---`.
pub fn parse_action(package: &str, name: &str, text: &str) -> Result<Action> {
    let cx = Context::new(package, Kind::Action, name)?;
    let text = text.replace('\t', " ");
    let lines = split_lines(&text);
    let [first, second] = separators(&cx, &lines)?;
    Ok(Action {
        goal: parse_lines(&cx, &lines[..first], 0)?,
        result: parse_lines(&cx, &lines[first + 1..second], first + 1)?,
        feedback: parse_lines(&cx, &lines[second + 1..], second + 1)?,
    })
}

/// Where the text comes from, for errors and for resolving unqualified type names.
struct Context {
    origin: String,
    package: String,
}

impl Context {
    fn new(package: &str, kind: Kind, name: &str) -> Result<Self> {
        let origin = format!("{package}/{kind}/{name}");
        if !is_valid_package_name(package) || !is_valid_message_name(name) {
            return Err(Error::TypeName {
                input: origin,
                reason: "bad package or interface name".to_owned(),
            });
        }
        Ok(Self {
            origin,
            package: package.to_owned(),
        })
    }

    fn error(&self, line: usize, kind: ParseErrorKind) -> Error {
        Error::Parse(ParseError {
            origin: self.origin.clone(),
            line,
            kind,
        })
    }
}

/// The indices of the `---` lines, which must be exactly `N` many.
fn separators<const N: usize>(cx: &Context, lines: &[&str]) -> Result<[usize; N]> {
    let found: Vec<usize> = lines
        .iter()
        .enumerate()
        .filter(|(_, l)| **l == "---")
        .map(|(i, _)| i)
        .collect();
    found.try_into().map_err(|found: Vec<usize>| {
        // Point at the first surplus separator, or at the end when some are missing.
        let line = found.get(N).map_or(lines.len().max(1), |i| i + 1);
        cx.error(
            line,
            ParseErrorKind::Separators {
                expected: N,
                found: found.len(),
            },
        )
    })
}

/// A field or constant with the comment lines gathered for it.
struct Element<T> {
    item: T,
    comments: Vec<String>,
    line: usize,
}

#[derive(Clone, Copy)]
enum Last {
    Field,
    Constant,
}

/// Parses the lines of one message; `first_line` is the file line number of `lines[0]` minus one.
fn parse_lines(cx: &Context, lines: &[&str], first_line: usize) -> Result<Message> {
    // Comment lines at the very top are the message's own; the first other line ends them.
    let body_start = lines
        .iter()
        .position(|l| !l.starts_with('#'))
        .unwrap_or(lines.len());
    let file_comments: Vec<String> = lines[..body_start]
        .iter()
        .map(|l| l.trim_start_matches('#').to_owned())
        .collect();

    let mut fields: Vec<Element<Field>> = Vec::new();
    let mut constants: Vec<Element<Constant>> = Vec::new();
    let mut last: Option<Last> = None;
    let mut pending: Vec<String> = Vec::new();

    for (i, raw) in lines.iter().enumerate().skip(body_start) {
        let line_no = first_line + i + 1;
        let mut line = trim_end(raw);
        if line.is_empty() {
            continue;
        }
        if let Some(at) = line.find('#') {
            let comment = line[at..].trim_start_matches('#').to_owned();
            let code = &line[..at];
            if !code.is_empty() && trim_start(code).is_empty() {
                // An indented comment-only line continues the previous element, or is dropped.
                let target = match last {
                    Some(Last::Field) => fields.last_mut().map(|e| &mut e.comments),
                    Some(Last::Constant) => constants.last_mut().map(|e| &mut e.comments),
                    None => None,
                };
                if let Some(target) = target {
                    target.push(comment);
                }
                continue;
            }
            pending.push(comment);
            line = trim_end(code);
            if line.is_empty() {
                continue;
            }
        }

        let (type_string, rest) = line.split_once(' ').unwrap_or((line, ""));
        let rest = trim_start(rest);
        if rest.is_empty() {
            return Err(cx.error(line_no, ParseErrorKind::Definition(line.to_owned())));
        }
        if let Some((name, value)) = rest.split_once('=') {
            let constant =
                parse_constant(cx, line_no, type_string, trim_end(name), trim_start(value))?;
            constants.push(Element {
                item: constant,
                comments: std::mem::take(&mut pending),
                line: line_no,
            });
            last = Some(Last::Constant);
        } else {
            let (name, default) = rest.split_once(' ').unwrap_or((rest, ""));
            let field = parse_field(cx, line_no, type_string, name, trim_start(default))?;
            fields.push(Element {
                item: field,
                comments: std::mem::take(&mut pending),
                line: line_no,
            });
            last = Some(Last::Field);
        }
    }

    ensure_unique(cx, &fields, "field", |f| &f.name)?;
    ensure_unique(cx, &constants, "constant", |c| &c.name)?;

    Ok(Message {
        doc: process_comments(file_comments).0,
        fields: fields
            .into_iter()
            .map(|e| {
                let (doc, unit) = process_comments(e.comments);
                Field {
                    doc,
                    unit,
                    ..e.item
                }
            })
            .collect(),
        constants: constants
            .into_iter()
            .map(|e| Constant {
                doc: process_comments(e.comments).0,
                ..e.item
            })
            .collect(),
    })
}

/// Fails on the second element of the same name, as the reference does after reading them all.
fn ensure_unique<T>(
    cx: &Context,
    elements: &[Element<T>],
    what: &'static str,
    name: impl Fn(&T) -> &str,
) -> Result<()> {
    let mut seen = HashSet::new();
    match elements.iter().find(|e| !seen.insert(name(&e.item))) {
        Some(dup) => {
            let name = name(&dup.item).to_owned();
            Err(cx.error(dup.line, ParseErrorKind::Duplicate { what, name }))
        }
        None => Ok(()),
    }
}

fn parse_field(
    cx: &Context,
    line: usize,
    type_string: &str,
    name: &str,
    default: &str,
) -> Result<Field> {
    let (ty, array) = parse_type(type_string, &cx.package).map_err(|reason| {
        cx.error(
            line,
            ParseErrorKind::Type {
                ty: type_string.to_owned(),
                reason,
            },
        )
    })?;
    if !is_valid_package_name(name) {
        return Err(cx.error(
            line,
            ParseErrorKind::Name {
                what: "field",
                name: name.to_owned(),
            },
        ));
    }
    let default = (!default.is_empty()).then(|| default.to_owned());
    if let Some(text) = &default {
        // Messages have no defaults, but the reference reads `time` and `duration` as primitives.
        let legacy_time = ["time", "duration"].iter().any(|t| {
            type_string
                .strip_prefix(t)
                .is_some_and(|rest| rest.is_empty() || rest.starts_with('['))
        });
        let checked = if matches!(ty, FieldType::Nested(_)) && !legacy_time {
            Err("values of nested message types are not supported".to_owned())
        } else {
            value::parse_value(&ty, array, text)
        };
        checked.map_err(|reason| {
            cx.error(
                line,
                ParseErrorKind::Value {
                    ty: type_string.to_owned(),
                    value: text.clone(),
                    reason,
                },
            )
        })?;
    }
    Ok(Field {
        name: name.to_owned(),
        ty,
        array,
        default,
        doc: None,
        unit: None,
    })
}

fn parse_constant(
    cx: &Context,
    line: usize,
    type_string: &str,
    name: &str,
    value_text: &str,
) -> Result<Constant> {
    let Some(ty) = FieldType::from_primitive(type_string) else {
        let reason = "the constant type must be a primitive type".to_owned();
        let kind = ParseErrorKind::Type {
            ty: type_string.to_owned(),
            reason,
        };
        return Err(cx.error(line, kind));
    };
    if !is_valid_constant_name(name) {
        return Err(cx.error(
            line,
            ParseErrorKind::Name {
                what: "constant",
                name: name.to_owned(),
            },
        ));
    }
    value::parse_scalar(&ty, value_text).map_err(|reason| {
        cx.error(
            line,
            ParseErrorKind::Value {
                ty: type_string.to_owned(),
                value: value_text.to_owned(),
                reason,
            },
        )
    })?;
    Ok(Constant {
        name: name.to_owned(),
        ty,
        value: value_text.to_owned(),
        doc: None,
    })
}

/// Reads `T`, `T[]`, `T[N]` or `T[<=N]`, where `T` is a primitive, `string<=N` or a message.
fn parse_type(type_string: &str, package: &str) -> std::result::Result<(FieldType, Array), String> {
    let (base, array) = if type_string.ends_with(']') {
        let index = type_string
            .rfind('[')
            .ok_or("the type ends with ']' but does not contain a '['")?;
        let size = &type_string[index + 1..type_string.len() - 1];
        let array = if size.is_empty() {
            Array::Unbounded
        } else {
            let (bounded, digits) = size.strip_prefix("<=").map_or((false, size), |d| (true, d));
            let n = positive(digits)
                .ok_or("the array size must be an integer > 0, optionally after '<='")?;
            if bounded {
                Array::Bounded(n)
            } else {
                Array::Fixed(n)
            }
        };
        (&type_string[..index], array)
    } else {
        (type_string, Array::Scalar)
    };
    Ok((parse_base_type(base, package)?, array))
}

/// An array size or string bound: an integer above zero, written as Python's `int()` reads it.
fn positive(digits: &str) -> Option<usize> {
    value::parse_decimal(digits)
        .and_then(|n| usize::try_from(n).ok())
        .filter(|n| *n > 0)
}

fn parse_base_type(base: &str, package: &str) -> std::result::Result<FieldType, String> {
    if let Some(primitive) = FieldType::from_primitive(base) {
        return Ok(primitive);
    }
    // Python keeps these two as primitives "for compatibility only"; they are messages in ROS 2.
    let builtin = |name: &str| {
        FieldType::Nested(TypeName {
            package: "builtin_interfaces".to_owned(),
            kind: Kind::Msg,
            name: name.to_owned(),
        })
    };
    match base {
        "time" => return Ok(builtin("Time")),
        "duration" => return Ok(builtin("Duration")),
        _ => {}
    }
    for (prefix, make) in [
        (
            "string<=",
            FieldType::String as fn(Option<usize>) -> FieldType,
        ),
        ("wstring<=", FieldType::WString),
    ] {
        if let Some(bound) = base.strip_prefix(prefix) {
            let n = positive(bound).ok_or("the upper bound of a string must be an integer > 0")?;
            return Ok(make(Some(n)));
        }
    }
    let parts: Vec<&str> = base.split('/').collect();
    let (package, name) = match parts.as_slice() {
        [name] => (package, *name),
        [package, name] => (*package, *name),
        _ => return Err("expected a primitive type, `Name` or `package/Name`".to_owned()),
    };
    if !is_valid_package_name(package) {
        return Err(format!("`{package}` is not a valid package name"));
    }
    if !is_valid_message_name(name) {
        return Err(format!("`{name}` is not a valid message name"));
    }
    Ok(FieldType::Nested(TypeName {
        package: package.to_owned(),
        kind: Kind::Msg,
        name: name.to_owned(),
    }))
}

/// Turns the gathered comment lines into a doc and a unit, as Python's `process_comments` does.
fn process_comments(mut lines: Vec<String>) -> (Option<String>, Option<String>) {
    // The unit is a single `[...]` group without a comma; more than one group means none is a unit.
    let mut unit = None;
    if let [(whole, inner)] = bracket_groups(&lines.join("\n")).as_slice() {
        unit = Some(inner.clone());
        for line in &mut lines {
            *line = line.replace(whole.as_str(), "");
        }
    }
    let leading = lines.iter().take_while(|l| l.is_empty()).count();
    lines.drain(..leading);
    while lines.last().is_some_and(String::is_empty) {
        lines.pop();
    }
    lines.dedup_by(|a, b| a.is_empty() && b.is_empty());
    if lines.is_empty() {
        return (None, unit);
    }
    (Some(dedent(&lines)), unit)
}

/// All matches of Python's `(\s*\[([^,\]]+)\])`: the whole match and the text inside the brackets.
fn bracket_groups(text: &str) -> Vec<(String, String)> {
    let chars: Vec<char> = text.chars().collect();
    // stop[i]: where the run of characters other than `,` and `]` that starts at i ends.
    let mut stop = vec![chars.len(); chars.len() + 1];
    for i in (0..chars.len()).rev() {
        stop[i] = if matches!(chars[i], ',' | ']') {
            i
        } else {
            stop[i + 1]
        };
    }
    let mut groups = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        let open = i + chars[i..].iter().take_while(|c| is_space(**c)).count();
        let close = stop[(open + 1).min(chars.len())];
        if chars.get(open) == Some(&'[') && close > open + 1 && chars.get(close) == Some(&']') {
            groups.push((
                chars[i..=close].iter().collect(),
                chars[open + 1..close].iter().collect(),
            ));
            i = close + 1;
        } else {
            // No match can start before `open`, since they would all fail the same way.
            i = open + 1;
        }
    }
    groups
}

/// Python's `textwrap.dedent` on the lines joined by newlines: blank lines become empty and the
/// indentation shared by all other lines goes.
fn dedent(lines: &[String]) -> String {
    let lines: Vec<&str> = lines
        .iter()
        .map(|l| {
            if l.chars().all(|c| c == ' ') {
                ""
            } else {
                l.as_str()
            }
        })
        .collect();
    let margin = lines
        .iter()
        .filter(|l| !l.is_empty())
        .map(|l| l.len() - l.trim_start_matches(' ').len())
        .min()
        .unwrap_or(0);
    let dedented: Vec<&str> = lines
        .iter()
        .map(|l| l.get(margin..).unwrap_or_default())
        .collect();
    dedented.join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splitlines_matches_python() {
        assert_eq!(split_lines("a\nb"), ["a", "b"]);
        assert_eq!(split_lines("a\n"), ["a"]);
        assert_eq!(split_lines("a\n\n"), ["a", ""]);
        assert_eq!(split_lines(""), Vec::<&str>::new());
        assert_eq!(
            split_lines("a\r\nb\rc\u{2028}d\x0be"),
            ["a", "b", "c", "d", "e"]
        );
    }
    #[test]
    fn name_patterns_follow_the_reference() {
        assert!(
            is_valid_package_name("g1_msgs")
                && !is_valid_package_name("G1")
                && !is_valid_package_name("1a")
        );
        assert!(is_valid_message_name("PoseStamped") && is_valid_message_name("Add_Request"));
        assert!(
            !is_valid_message_name("pose")
                && !is_valid_message_name("Pose_Stamped")
                && !is_valid_message_name("_Request")
        );
        assert!(
            is_valid_constant_name("A")
                && is_valid_constant_name("ARM_LEFT_2")
                && !is_valid_constant_name("A__B")
        );
        assert!(
            !is_valid_constant_name("_A")
                && !is_valid_constant_name("A_")
                && !is_valid_constant_name("a")
        );
    }
}
