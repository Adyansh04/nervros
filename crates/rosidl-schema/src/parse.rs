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

    fn msg(text: &str) -> Message {
        parse_message("pkg", "Name", text).unwrap()
    }

    fn err(text: &str) -> ParseError {
        match parse_message("pkg", "Name", text) {
            Err(Error::Parse(e)) => e,
            other => panic!("expected a parse error, got {other:?}"),
        }
    }

    fn doc(text: &str) -> Option<String> {
        msg(text).fields[0].doc.clone()
    }

    #[test]
    fn leading_comment_lines_are_the_message_doc() {
        let m = msg("# First line\n#   indented\n# Last\n\nint32 x\n");
        assert_eq!(m.doc.as_deref(), Some("First line\n  indented\nLast"));
        assert_eq!(m.fields[0].doc, None);
    }

    #[test]
    fn a_comment_touching_the_first_field_is_still_the_message_doc() {
        let m = msg("# Only comment\nint32 x\n");
        assert_eq!(m.doc.as_deref(), Some("Only comment"));
        assert_eq!(m.fields[0].doc, None);
    }

    #[test]
    fn a_blank_line_after_the_doc_lets_the_next_comment_belong_to_the_field() {
        let m = msg("# Message doc\n\n# Field doc\nint32 x\n");
        assert_eq!(m.doc.as_deref(), Some("Message doc"));
        assert_eq!(m.fields[0].doc.as_deref(), Some("Field doc"));
    }

    #[test]
    fn a_comment_block_above_attaches_to_the_next_element() {
        let m = msg("\n# one\n# two\nint32 a\n\n# three\nint32 b\n");
        assert_eq!(m.fields[0].doc.as_deref(), Some("one\ntwo"));
        assert_eq!(m.fields[1].doc.as_deref(), Some("three"));
    }

    #[test]
    fn a_same_line_comment_joins_the_block_above() {
        assert_eq!(
            doc("\n# above\nint32 a # beside\n").as_deref(),
            Some("above\nbeside")
        );
    }

    #[test]
    fn an_indented_comment_line_continues_the_previous_element() {
        let m = msg("\nint32 a # first\n  # second\nint32 b\n  # for b\n");
        assert_eq!(m.fields[0].doc.as_deref(), Some("first\nsecond"));
        assert_eq!(m.fields[1].doc.as_deref(), Some("for b"));
    }

    #[test]
    fn an_indented_comment_before_any_element_is_dropped() {
        let m = msg("\n  # nobody's\nint32 a\n");
        assert_eq!(m.fields[0].doc, None);
    }

    #[test]
    fn a_comment_at_column_zero_after_an_element_belongs_to_the_next_one() {
        let m = msg("\nint32 a\n# for b\nint32 b\n");
        assert_eq!(m.fields[0].doc, None);
        assert_eq!(m.fields[1].doc.as_deref(), Some("for b"));
    }

    #[test]
    fn a_trailing_comment_block_belongs_to_nothing() {
        let m = msg("\nint32 a\n# dangling\n");
        assert_eq!(m.fields[0].doc, None);
    }

    #[test]
    fn comments_lose_all_leading_hashes_and_shared_indent() {
        assert_eq!(
            doc("\n## two\n##   three\nint32 a\n").as_deref(),
            Some("two\n  three")
        );
    }

    #[test]
    fn empty_comment_lines_inside_a_doc_stay_but_at_the_edges_go() {
        assert_eq!(
            doc("\n#\n# a\n#\n#\n# b\n#\nint32 x\n").as_deref(),
            Some("a\n\nb")
        );
    }

    #[test]
    fn a_hash_inside_a_comment_is_text() {
        assert_eq!(doc("\nint32 a # see #12\n").as_deref(), Some("see #12"));
    }

    #[test]
    fn a_single_bracket_group_is_the_unit_and_leaves_the_text() {
        let m = msg("\n# Distance to the goal [m]\nfloat64 d\n");
        assert_eq!(m.fields[0].unit.as_deref(), Some("m"));
        assert_eq!(m.fields[0].doc.as_deref(), Some("Distance to the goal"));
    }

    #[test]
    fn a_unit_may_sit_mid_sentence_or_alone() {
        let m = msg("\nfloat64 a # speed [m/s] forward\nfloat64 b # [rad]\n");
        assert_eq!(m.fields[0].unit.as_deref(), Some("m/s"));
        assert_eq!(m.fields[0].doc.as_deref(), Some("speed forward"));
        assert_eq!(m.fields[1].unit.as_deref(), Some("rad"));
        assert_eq!(m.fields[1].doc, None);
    }

    #[test]
    fn two_bracket_groups_or_a_comma_mean_no_unit() {
        let m = msg("\nfloat64 a # [m] or [ft]\nfloat64 b # range [0, 1]\n");
        assert_eq!(m.fields[0].unit, None);
        assert_eq!(m.fields[0].doc.as_deref(), Some("[m] or [ft]"));
        assert_eq!(m.fields[1].unit, None);
        assert_eq!(m.fields[1].doc.as_deref(), Some("range [0, 1]"));
    }

    #[test]
    fn a_unit_is_taken_from_the_whole_comment_across_lines() {
        let m = msg("\n# Height\n# above ground [m]\nfloat64 h\n");
        assert_eq!(m.fields[0].unit.as_deref(), Some("m"));
        assert_eq!(m.fields[0].doc.as_deref(), Some("Height\nabove ground"));
    }

    #[test]
    fn the_message_doc_loses_its_unit_too() {
        let m = msg("# Mass [kg]\n\nfloat64 m\n");
        assert_eq!(m.doc.as_deref(), Some("Mass"));
    }

    #[test]
    fn tabs_count_as_spaces() {
        let m = msg("\nint32\tx\t# a\n\t# b\n");
        assert_eq!(m.fields[0].name, "x");
        assert_eq!(m.fields[0].doc.as_deref(), Some("a\nb"));
    }

    #[test]
    fn crlf_and_lone_cr_line_ends_work() {
        let m = msg("# doc\r\n\r\nint32 a\r\nint32 b\rint32 c\n");
        assert_eq!(m.doc.as_deref(), Some("doc"));
        assert_eq!(m.fields.len(), 3);
    }

    #[test]
    fn defaults_and_constants_are_told_apart_by_the_equals_sign() {
        let m = msg(
            "\nfloat64 w 1\nstring s \"hello world\"\nuint8 MODE=2\nstring NAME = left\nint32[] xs [1, 2]\n",
        );
        assert_eq!(m.fields[0].default.as_deref(), Some("1"));
        assert_eq!(m.fields[0].default_json(), Some(serde_json::json!(1.0)));
        assert_eq!(m.fields[1].default.as_deref(), Some("\"hello world\""));
        assert_eq!(
            m.fields[1].default_json(),
            Some(serde_json::json!("hello world"))
        );
        assert_eq!(m.fields[2].default_json(), Some(serde_json::json!([1, 2])));
        assert_eq!(
            (m.constants[0].name.as_str(), m.constants[0].value.as_str()),
            ("MODE", "2")
        );
        assert_eq!(m.constants[1].json_value(), Some(serde_json::json!("left")));
        assert_eq!(m.constants[1].ty, FieldType::String(None));
    }

    #[test]
    fn an_equals_sign_in_a_default_makes_it_a_constant_like_python_does() {
        assert!(matches!(
            err("\nstring s \"a=b\"\n").kind,
            ParseErrorKind::Name {
                what: "constant",
                ..
            }
        ));
    }

    #[test]
    fn constants_can_be_empty_strings_or_hex() {
        let m = msg("\nstring EMPTY=\nuint8 FLAG=0x10\nbool YES=true\n");
        assert_eq!(m.constants[0].json_value(), Some(serde_json::json!("")));
        assert_eq!(m.constants[1].json_value(), Some(serde_json::json!(16)));
        assert_eq!(m.constants[2].json_value(), Some(serde_json::json!(true)));
    }

    #[test]
    fn constant_comments_attach_like_field_comments() {
        let m = msg("\n# the mode\nuint8 MODE=1 # fast\n");
        assert_eq!(m.constants[0].doc.as_deref(), Some("the mode\nfast"));
    }

    #[test]
    fn arrays_and_string_bounds_are_read_from_the_type() {
        let m = msg(
            "\nint32 a\nint32[] b\nint32[3] c\nint32[<=4] d\nstring<=5 e\nwstring<=6[] f\nstring<=7[2] g\n",
        );
        let shape: Vec<_> = m.fields.iter().map(|f| (f.ty.clone(), f.array)).collect();
        assert_eq!(
            shape,
            [
                (FieldType::I32, Array::Scalar),
                (FieldType::I32, Array::Unbounded),
                (FieldType::I32, Array::Fixed(3)),
                (FieldType::I32, Array::Bounded(4)),
                (FieldType::String(Some(5)), Array::Scalar),
                (FieldType::WString(Some(6)), Array::Unbounded),
                (FieldType::String(Some(7)), Array::Fixed(2)),
            ]
        );
    }

    #[test]
    fn sizes_and_bounds_are_read_like_pythons_int() {
        let m = msg(
            "\nint32[+3] a\nint32[1_0] b\nint32[<=2\u{3000}] c\nstring<=1_0 d\nint32[\u{2003}5] e\n",
        );
        let shape: Vec<_> = m.fields.iter().map(|f| (f.ty.clone(), f.array)).collect();
        assert_eq!(shape[0].1, Array::Fixed(3));
        assert_eq!(shape[1].1, Array::Fixed(10));
        assert_eq!(shape[2].1, Array::Bounded(2));
        assert_eq!(shape[3].0, FieldType::String(Some(10)));
        assert_eq!(shape[4].1, Array::Fixed(5));
        for bad in [
            "int32[-1] a",
            "int32[0x3] a",
            "int32[1__0] a",
            "int32[3_] a",
            "string<=+ a",
        ] {
            assert!(parse_message("pkg", "Name", bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn every_primitive_maps_to_its_type() {
        let names = [
            "bool", "byte", "char", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64",
            "uint64", "float32", "float64", "string", "wstring",
        ];
        let text = names
            .iter()
            .map(|n| format!("{n} f_{n}"))
            .collect::<Vec<_>>()
            .join("\n");
        let m = msg(&text);
        for (f, name) in m.fields.iter().zip(names) {
            assert_eq!(f.ty.primitive_name(), Some(name));
        }
    }

    #[test]
    fn unqualified_types_resolve_against_the_current_package() {
        let m = msg("\nOther a\nother_pkg/Thing b\nOther[] c\n");
        let nested = |f: &Field| match &f.ty {
            FieldType::Nested(t) => t.to_string(),
            other => panic!("{other:?}"),
        };
        assert_eq!(nested(&m.fields[0]), "pkg/msg/Other");
        assert_eq!(nested(&m.fields[1]), "other_pkg/msg/Thing");
        assert_eq!(nested(&m.fields[2]), "pkg/msg/Other");
        assert_eq!(m.fields[2].array, Array::Unbounded);
    }

    #[test]
    fn time_and_duration_are_the_builtin_interfaces_messages() {
        let m = msg("\ntime a\nduration[] b\n");
        let name = |f: &Field| match &f.ty {
            FieldType::Nested(t) => t.to_string(),
            other => panic!("{other:?}"),
        };
        assert_eq!(name(&m.fields[0]), "builtin_interfaces/msg/Time");
        assert_eq!(name(&m.fields[1]), "builtin_interfaces/msg/Duration");
    }

    #[test]
    fn only_an_empty_list_can_default_a_time_array_like_in_the_reference() {
        let m = msg("\ntime[] a []\nduration[<=2] b []\n");
        assert_eq!(m.fields[0].default_json(), Some(serde_json::json!([])));
        assert_eq!(m.fields[1].default.as_deref(), Some("[]"));
        for bad in [
            "time[3] a []",
            "time a []",
            "time[] a [1]",
            "duration[] a [ ]",
            "pkg/Other[] a []",
            "builtin_interfaces/Time[] a []",
        ] {
            assert!(parse_message("pkg", "Name", bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn a_service_splits_at_the_separator_and_each_half_has_its_own_doc() {
        let text = "# Request doc\n\nint32 a\n---\n# Response doc\n\nbool ok\n";
        let s = parse_service("pkg", "Name", text).unwrap();
        assert_eq!(s.request.doc.as_deref(), Some("Request doc"));
        assert_eq!(s.response.doc.as_deref(), Some("Response doc"));
        assert_eq!(s.request.fields[0].name, "a");
        assert_eq!(s.response.fields[0].name, "ok");
    }

    #[test]
    fn an_action_has_goal_result_and_feedback() {
        let a = parse_action("pkg", "Name", "int32 g\n---\nint32 r\n---\nint32 f\n").unwrap();
        assert_eq!(a.goal.fields[0].name, "g");
        assert_eq!(a.result.fields[0].name, "r");
        assert_eq!(a.feedback.fields[0].name, "f");
    }

    #[test]
    fn empty_halves_are_fine() {
        let a = parse_action("pkg", "Name", "---\n---\n").unwrap();
        assert!(
            a.goal.fields.is_empty() && a.result.fields.is_empty() && a.feedback.fields.is_empty()
        );
        let s = parse_service("pkg", "Name", "---").unwrap();
        assert_eq!(s, Service::default());
    }

    #[test]
    fn a_separator_with_trailing_text_is_not_a_separator() {
        assert!(parse_service("pkg", "Name", "int32 a\n--- \nint32 b\n").is_err());
    }

    #[test]
    fn wrong_separator_counts_are_errors_with_a_line() {
        let e = |r: Result<Service>| match r {
            Err(Error::Parse(e)) => e,
            other => panic!("{other:?}"),
        };
        let none = e(parse_service("pkg", "Name", "int32 a\nint32 b\n"));
        assert_eq!(
            none.kind,
            ParseErrorKind::Separators {
                expected: 1,
                found: 0
            }
        );
        let two = e(parse_service(
            "pkg",
            "Name",
            "int32 a\n---\nint32 b\n---\nint32 c\n",
        ));
        assert_eq!(
            (two.line, two.kind),
            (
                4,
                ParseErrorKind::Separators {
                    expected: 1,
                    found: 2
                }
            )
        );
        assert!(parse_action("pkg", "Name", "int32 a\n---\nint32 b\n").is_err());
    }

    #[test]
    fn errors_name_the_file_the_line_and_the_problem() {
        let e = err("# doc\n\nint32 a\nfloat64\n");
        assert_eq!((e.line, e.origin.as_str()), (4, "pkg/msg/Name"));
        assert_eq!(e.kind, ParseErrorKind::Definition("float64".into()));
        assert_eq!(
            e.to_string(),
            "pkg/msg/Name:4: expected `TYPE NAME`, found `float64`"
        );
    }

    #[test]
    fn service_errors_count_lines_from_the_top_of_the_file() {
        let Err(Error::Parse(e)) = parse_service("pkg", "Name", "int32 a\n---\nint32 b\nbogus\n")
        else {
            panic!("expected a parse error");
        };
        assert_eq!(e.line, 4);
    }

    #[test]
    fn bad_input_is_rejected() {
        let kind = |text: &str| err(text).kind;
        assert!(matches!(
            kind("\nint32 A\n"),
            ParseErrorKind::Name { what: "field", .. }
        ));
        assert!(matches!(
            kind("\nint32 a__b\n"),
            ParseErrorKind::Name { .. }
        ));
        assert!(matches!(kind("\nint32 a_\n"), ParseErrorKind::Name { .. }));
        assert!(matches!(
            kind("\nint32 lower=1\n"),
            ParseErrorKind::Name {
                what: "constant",
                ..
            }
        ));
        assert!(matches!(
            kind("\nint32 A_=1\n"),
            ParseErrorKind::Name { .. }
        ));
        assert!(matches!(
            kind("\nint32[0] a\n"),
            ParseErrorKind::Type { .. }
        ));
        assert!(matches!(
            kind("\nint32[<=] a\n"),
            ParseErrorKind::Type { .. }
        ));
        assert!(matches!(
            kind("\nstring<=0 a\n"),
            ParseErrorKind::Type { .. }
        ));
        assert!(matches!(kind("\na/b/C x\n"), ParseErrorKind::Type { .. }));
        assert!(matches!(kind("\nlower x\n"), ParseErrorKind::Type { .. }));
        assert!(matches!(kind("\n  int32 x\n"), ParseErrorKind::Type { .. }));
        assert!(matches!(
            kind("\nint32[] x [1\n"),
            ParseErrorKind::Value { .. }
        ));
        assert!(matches!(
            kind("\nuint8 x 256\n"),
            ParseErrorKind::Value { .. }
        ));
        assert!(matches!(
            kind("\nOther x 1\n"),
            ParseErrorKind::Value { .. }
        ));
        assert!(matches!(
            kind("\nstring<=5 X=abcdef\n"),
            ParseErrorKind::Type { .. }
        ));
        assert!(matches!(kind("\ntime X=1\n"), ParseErrorKind::Type { .. }));
        assert!(matches!(
            kind("\nint32 a\nint32 a\n"),
            ParseErrorKind::Duplicate { what: "field", .. }
        ));
        assert!(matches!(
            kind("\nint32 A=1\nint32 A=2\n"),
            ParseErrorKind::Duplicate {
                what: "constant",
                ..
            }
        ));
    }

    #[test]
    fn bad_package_or_message_names_are_rejected_up_front() {
        assert!(matches!(
            parse_message("Bad", "Name", ""),
            Err(Error::TypeName { .. })
        ));
        assert!(matches!(
            parse_message("pkg", "lower", ""),
            Err(Error::TypeName { .. })
        ));
        assert!(parse_service("pkg", "Name_Request", "---").is_ok());
    }

    #[test]
    fn type_names_parse_in_three_parts_or_two() {
        let t: TypeName = "geometry_msgs/msg/Pose".parse().unwrap();
        assert_eq!(
            (t.package.as_str(), t.kind, t.name.as_str()),
            ("geometry_msgs", Kind::Msg, "Pose")
        );
        assert_eq!("geometry_msgs/Pose".parse::<TypeName>().unwrap(), t);
        assert_eq!(
            "std_srvs/srv/SetBool".parse::<TypeName>().unwrap().kind,
            Kind::Srv
        );
        assert_eq!(
            "nav2_msgs/action/NavigateToPose"
                .parse::<TypeName>()
                .unwrap()
                .kind,
            Kind::Action
        );
        assert_eq!(t.to_string(), "geometry_msgs/msg/Pose");
        for bad in [
            "",
            "Pose",
            "a/b/c/D",
            "pkg/topic/Name",
            "Pkg/msg/Name",
            "pkg/msg/name",
            "pkg//Name",
            "pkg/msg/",
        ] {
            assert!(bad.parse::<TypeName>().is_err(), "{bad:?}");
        }
    }

    #[test]
    fn a_type_name_round_trips_through_serde() {
        let t: TypeName = serde_json::from_str("\"canopy_msgs/srv/FindObjects\"").unwrap();
        assert_eq!(
            serde_json::to_string(&t).unwrap(),
            "\"canopy_msgs/srv/FindObjects\""
        );
        assert!(serde_json::from_str::<TypeName>("\"nope\"").is_err());
    }

    #[test]
    fn printing_gives_text_that_parses_back_to_the_same_message() {
        let text = "# Doc line one\n#   indented\n#\n# after a blank\n\n# Mode.\nuint8 MODE_A=1\nstring NAME=\"x y\"\n\n\
                    # Distance to the goal [m]\nfloat64 d 1.5\nint32[<=3] xs [1, 2] # beside\nOther o\ntime t\n";
        let m = msg(text);
        let printed = m.to_string();
        assert_eq!(msg(&printed), m, "{printed}");
    }

    #[test]
    fn printing_keeps_a_field_comment_from_turning_into_the_message_doc() {
        let m = msg("\n# only the field\nint32 a\n");
        assert_eq!(m.doc, None);
        assert_eq!(msg(&m.to_string()), m);
    }

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
