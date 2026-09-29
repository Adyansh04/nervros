//! Default and constant values, read the way `rosidl_adapter` reads them.

use serde_json::Value;

use crate::{
    parse::trim,
    types::{Array, FieldType},
};

/// One value of a primitive type.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Scalar {
    Bool(bool),
    Int(i128),
    Float(f64),
    Str(String),
}

/// A default or constant, checked against its type.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Parsed {
    Scalar(Scalar),
    List(Vec<Scalar>),
}

impl Scalar {
    fn to_json(&self) -> Option<Value> {
        match self {
            Self::Bool(b) => Some(Value::Bool(*b)),
            Self::Int(i) => json_int(*i),
            // JSON has no NaN or infinity.
            Self::Float(f) => serde_json::Number::from_f64(*f).map(Value::Number),
            Self::Str(s) => Some(Value::String(s.clone())),
        }
    }
}

impl Parsed {
    /// The value as JSON, or `None` when it holds a NaN or an infinity.
    pub(crate) fn to_json(&self) -> Option<Value> {
        match self {
            Self::Scalar(s) => s.to_json(),
            Self::List(items) => items
                .iter()
                .map(Scalar::to_json)
                .collect::<Option<_>>()
                .map(Value::Array),
        }
    }
}

/// An integer as a JSON number; `None` outside the 64-bit ranges.
pub(crate) fn json_int(i: i128) -> Option<Value> {
    i64::try_from(i)
        .map(Value::from)
        .ok()
        .or_else(|| u64::try_from(i).map(Value::from).ok())
}

/// Checks `text` against the type and returns the value, or why it does not fit.
pub(crate) fn parse_value(ty: &FieldType, array: Array, text: &str) -> Result<Parsed, String> {
    if matches!(ty, FieldType::Nested(_)) {
        // Only the empty list is readable, which is what the reference accepts for `time[]`.
        return match array {
            Array::Unbounded | Array::Bounded(_) if text == "[]" => Ok(Parsed::List(Vec::new())),
            _ => Err("values of nested message types are not supported".to_owned()),
        };
    }
    if array == Array::Scalar {
        return parse_scalar(ty, text).map(Parsed::Scalar);
    }
    let inner = text
        .strip_prefix('[')
        .and_then(|t| t.strip_suffix(']'))
        .ok_or("array value must start with '[' and end with ']'")?;
    let elements: Vec<String> = if matches!(ty, FieldType::String(_) | FieldType::WString(_)) {
        split_string_elements(inner)?
    } else if inner.is_empty() {
        Vec::new()
    } else {
        inner.split(',').map(str::to_owned).collect()
    };
    match array {
        Array::Fixed(n) if elements.len() != n => {
            return Err(format!(
                "array must have exactly {n} elements, not {}",
                elements.len()
            ));
        }
        Array::Bounded(n) if elements.len() > n => {
            return Err(format!(
                "array must have not more than {n} elements, not {}",
                elements.len()
            ));
        }
        _ => {}
    }
    let items = elements
        .iter()
        .enumerate()
        .map(|(i, e)| parse_scalar(ty, trim(e)).map_err(|reason| format!("element {i}: {reason}")))
        .collect::<Result<_, _>>()?;
    Ok(Parsed::List(items))
}

/// Checks `text` against a primitive, non-array type.
pub(crate) fn parse_scalar(ty: &FieldType, text: &str) -> Result<Scalar, String> {
    match ty {
        FieldType::Bool => match text.to_lowercase().as_str() {
            "true" | "1" => Ok(Scalar::Bool(true)),
            "false" | "0" => Ok(Scalar::Bool(false)),
            _ => Err("must be either 'true' / '1' or 'false' / '0'".to_owned()),
        },
        FieldType::F32 | FieldType::F64 => parse_float(text)
            .map(Scalar::Float)
            .ok_or_else(|| "must be a floating point number using '.' as the separator".to_owned()),
        FieldType::String(bound) | FieldType::WString(bound) => {
            decode_string(text, *bound).map(Scalar::Str)
        }
        FieldType::Nested(_) => Err("values of nested message types are not supported".to_owned()),
        _ => {
            let (lo, hi) = ty.integer_range().ok_or("not an integer type")?;
            match parse_int(text) {
                Some(v) if (lo..=hi).contains(&v) => Ok(Scalar::Int(v)),
                _ => Err(format!("must be a valid integer value >= {lo} and <= {hi}")),
            }
        }
    }
}

/// Python's `int(text)` and then `int(text, 0)`: decimal, or with a `0x`, `0o` or `0b` prefix.
fn parse_int(text: &str) -> Option<i128> {
    parse_decimal(text).or_else(|| parse_prefixed(text))
}

/// Python's `int(text)`: an optional sign and digits with single underscores between them, with
/// white space around.
pub(crate) fn parse_decimal(text: &str) -> Option<i128> {
    // `int()` and `float()` skip Unicode white space but, unlike `str.strip()`, not U+001C-1F.
    let (negative, digits) = split_sign(text.trim());
    let digits = without_underscores(digits, 10)?;
    if digits.is_empty() || !digits.chars().all(|c| c.is_ascii_digit()) {
        return None;
    }
    let magnitude: i128 = digits.parse().ok()?;
    Some(if negative { -magnitude } else { magnitude })
}

/// Python's `int(text, 0)` for the prefixed forms; one underscore may follow the prefix.
fn parse_prefixed(text: &str) -> Option<i128> {
    let (negative, unsigned) = split_sign(text.trim());
    let (radix, digits) = match unsigned.get(..2).map(str::to_ascii_lowercase).as_deref() {
        Some("0x") => (16, &unsigned[2..]),
        Some("0o") => (8, &unsigned[2..]),
        Some("0b") => (2, &unsigned[2..]),
        _ => return None,
    };
    let digits = without_underscores(digits.strip_prefix('_').unwrap_or(digits), radix)?;
    if digits.is_empty() || !digits.chars().all(|c| c.is_digit(radix)) {
        return None;
    }
    let magnitude = i128::from_str_radix(&digits, radix).ok()?;
    Some(if negative { -magnitude } else { magnitude })
}

fn split_sign(text: &str) -> (bool, &str) {
    match text.as_bytes().first() {
        Some(b'-') => (true, &text[1..]),
        Some(b'+') => (false, &text[1..]),
        _ => (false, text),
    }
}

/// Python's `float(text)`, which also takes underscores between digits.
fn parse_float(text: &str) -> Option<f64> {
    without_underscores(text.trim(), 10)?.parse().ok()
}

/// `text` without its underscores, which must each sit between two digits of the radix.
fn without_underscores(text: &str, radix: u32) -> Option<String> {
    let chars: Vec<char> = text.chars().collect();
    let mut out = String::with_capacity(text.len());
    for (i, c) in chars.iter().enumerate() {
        if *c != '_' {
            out.push(*c);
        } else if !(i > 0
            && chars.get(i + 1).is_some_and(|n| n.is_digit(radix))
            && chars[i - 1].is_digit(radix))
        {
            return None;
        }
    }
    Some(out)
}

/// Removes one pair of outer quotes and the escapes of the quote character inside.
fn decode_string(text: &str, bound: Option<usize>) -> Result<String, String> {
    let mut value = text.to_owned();
    for quote in ['"', '\''] {
        if text.starts_with(quote) && text.ends_with(quote) {
            // A lone quote character counts as both the opening and the closing one.
            let inner = text
                .get(1..text.len().saturating_sub(1))
                .unwrap_or_default();
            let mut previous = None;
            for c in inner.chars() {
                if c == quote && previous != Some('\\') {
                    return Err("string inner quotes not properly escaped".to_owned());
                }
                previous = Some(c);
            }
            value = inner.replace(&format!("\\{quote}"), &quote.to_string());
            break;
        }
    }
    match bound {
        Some(max) if value.chars().count() > max => Err(format!(
            "string must not exceed the maximum length of {max} characters"
        )),
        _ => Ok(value),
    }
}

/// Splits the inside of `["a", 'b,c', d]` into its elements: a port of the reference's
/// `parse_string_array_value_string`, quirks included. Quotes come off here, and a quoted element
/// may hold commas. Backslash-quote pairs are unescaped, but the reference misjudges where a
/// string ends after its second escape, and so does this port.
fn split_string_elements(inner: &str) -> Result<Vec<String>, String> {
    let chars: Vec<char> = inner.chars().collect();
    let mut rest: &[char] = &chars;
    let mut elements = Vec::new();
    while !rest.is_empty() {
        rest = strip_spaces(rest);
        match rest.first() {
            // Python fails with an index error on a string that is left with spaces only.
            None => return Err("nothing after the separator".to_owned()),
            Some(',') => {
                return Err(format!(
                    "unexpected ',' at beginning of [{}]",
                    rest.iter().collect::<String>()
                ));
            }
            Some(_) => {}
        }
        let mut quoted = false;
        for quote in ['"', '\''] {
            if rest.first() == Some(&quote) {
                quoted = true;
                let end = end_of_quoted(rest, quote).ok_or_else(|| {
                    format!(
                        "string [{}] incorrectly quoted",
                        rest.iter().collect::<String>()
                    )
                })?;
                let content: String = rest.get(1..=end).unwrap_or_default().iter().collect();
                elements.push(content.replace(&format!("\\{quote}"), &quote.to_string()));
                rest = rest.get(end + 2..).unwrap_or_default();
            }
        }
        if !quoted {
            let comma = rest.iter().position(|c| *c == ',').unwrap_or(rest.len());
            elements.push(rest[..comma].iter().collect());
            rest = &rest[comma..];
        }
        rest = strip_spaces(rest);
        rest = rest.strip_prefix(&[',']).unwrap_or(rest);
    }
    Ok(elements)
}

fn strip_spaces(chars: &[char]) -> &[char] {
    &chars[chars.iter().take_while(|c| **c == ' ').count()..]
}

/// The reference's `find_matching_end_quote`: the index of the last character before the quote
/// that closes the string `text` starts with, or `None`.
fn end_of_quoted(text: &[char], quote: char) -> Option<usize> {
    let mut text = text;
    let mut base = 0;
    while !text.is_empty() {
        let at = text
            .get(1..)
            .unwrap_or_default()
            .iter()
            .position(|c| *c == quote)?;
        if text[at..at + 2] != ['\\', quote] {
            return Some(base + at);
        }
        text = text.get(at + 2..).unwrap_or_default();
        base = at + 2;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn int(ty: &FieldType, text: &str) -> Option<i128> {
        match parse_scalar(ty, text) {
            Ok(Scalar::Int(i)) => Some(i),
            _ => None,
        }
    }

    #[test]
    fn integers_accept_decimal_prefixed_and_signed_forms() {
        assert_eq!(int(&FieldType::I32, "-12"), Some(-12));
        assert_eq!(int(&FieldType::U8, "0xff"), Some(255));
        assert_eq!(int(&FieldType::U8, "0b101"), Some(5));
        assert_eq!(int(&FieldType::U16, "0o17"), Some(15));
        assert_eq!(int(&FieldType::U32, "1_000"), Some(1000));
        assert_eq!(int(&FieldType::I8, "010"), Some(10));
        assert_eq!(
            int(&FieldType::U32, "0x_ff_ff"),
            Some(0xffff),
            "one underscore may follow the prefix"
        );
    }

    #[test]
    fn integers_reject_out_of_range_and_garbage() {
        assert_eq!(int(&FieldType::U8, "256"), None);
        assert_eq!(int(&FieldType::U8, "-1"), None);
        assert_eq!(int(&FieldType::I8, "128"), None);
        assert_eq!(int(&FieldType::I32, "1.5"), None);
        assert_eq!(int(&FieldType::I32, "--5"), None);
        assert_eq!(int(&FieldType::I32, "_5"), None);
        assert_eq!(int(&FieldType::I32, "5_"), None);
        assert_eq!(int(&FieldType::I32, "5__0"), None);
        assert_eq!(int(&FieldType::I32, "0x__f"), None);
        assert_eq!(int(&FieldType::I32, "0_x1"), None);
        assert_eq!(int(&FieldType::I32, ""), None);
        assert_eq!(
            int(&FieldType::U64, "18446744073709551615"),
            Some(u64::MAX.into())
        );
        assert_eq!(int(&FieldType::U64, "18446744073709551616"), None);
    }

    #[test]
    fn bools_take_true_false_zero_one_in_any_case() {
        for (text, want) in [
            ("true", true),
            ("TRUE", true),
            ("1", true),
            ("False", false),
            ("0", false),
        ] {
            assert_eq!(
                parse_scalar(&FieldType::Bool, text),
                Ok(Scalar::Bool(want)),
                "{text}"
            );
        }
        assert!(parse_scalar(&FieldType::Bool, "yes").is_err());
    }

    #[test]
    fn floats_take_integers_exponents_and_specials() {
        assert_eq!(parse_scalar(&FieldType::F64, "1"), Ok(Scalar::Float(1.0)));
        assert_eq!(
            parse_scalar(&FieldType::F32, "-2.5e3"),
            Ok(Scalar::Float(-2500.0))
        );
        assert!(matches!(parse_scalar(&FieldType::F64, "nan"), Ok(Scalar::Float(f)) if f.is_nan()));
        assert_eq!(
            parse_scalar(&FieldType::F64, "1_47.6_5e1_0"),
            Ok(Scalar::Float(147.65e10))
        );
        for bad in ["1_", "_1", "1__0", "1_.5", "1._5", "1_e5", "in_f"] {
            assert!(parse_scalar(&FieldType::F64, bad).is_err(), "{bad}");
        }
        assert!(parse_scalar(&FieldType::F64, "1,5").is_err());
    }

    #[test]
    fn strings_lose_one_pair_of_outer_quotes_and_their_escapes() {
        let s = |t: &str| parse_scalar(&FieldType::String(None), t).unwrap();
        assert_eq!(s("hello world"), Scalar::Str("hello world".into()));
        assert_eq!(s("\"a b\""), Scalar::Str("a b".into()));
        assert_eq!(s("'it\\'s'"), Scalar::Str("it's".into()));
        assert_eq!(s("\"'x'\""), Scalar::Str("'x'".into()));
        assert_eq!(s("\"\""), Scalar::Str(String::new()));
        assert_eq!(s("\""), Scalar::Str(String::new()));
        assert!(parse_scalar(&FieldType::String(None), "\"a\"b\"").is_err());
    }

    #[test]
    fn string_bounds_count_characters() {
        let bounded = FieldType::String(Some(3));
        assert!(parse_scalar(&bounded, "\"abc\"").is_ok());
        assert!(parse_scalar(&bounded, "\"abcd\"").is_err());
        assert!(parse_scalar(&bounded, "äöü").is_ok());
    }

    #[test]
    fn arrays_need_brackets_and_the_right_length() {
        let list = |ty: &FieldType, array, text| parse_value(ty, array, text);
        assert_eq!(
            list(&FieldType::I32, Array::Unbounded, "[1, 2,3]"),
            Ok(Parsed::List(vec![
                Scalar::Int(1),
                Scalar::Int(2),
                Scalar::Int(3)
            ]))
        );
        assert_eq!(
            list(&FieldType::I32, Array::Unbounded, "[]"),
            Ok(Parsed::List(vec![]))
        );
        assert!(list(&FieldType::I32, Array::Fixed(2), "[1]").is_err());
        assert!(list(&FieldType::I32, Array::Fixed(2), "[1, 2]").is_ok());
        assert!(list(&FieldType::I32, Array::Bounded(2), "[1, 2, 3]").is_err());
        assert!(list(&FieldType::I32, Array::Bounded(2), "[1]").is_ok());
        assert!(list(&FieldType::I32, Array::Unbounded, "1, 2").is_err());
        assert!(list(&FieldType::I32, Array::Unbounded, "[1,,2]").is_err());
        assert!(list(&FieldType::I32, Array::Unbounded, "[").is_err());
    }

    /// The expected results were taken from `rosidl_adapter.parser.parse_value_string`.
    #[test]
    fn string_arrays_read_like_the_reference_quirks_included() {
        let read = |ty: FieldType, array, text: &str| match parse_value(&ty, array, text) {
            Ok(Parsed::List(items)) => Some(
                items
                    .into_iter()
                    .map(|s| match s {
                        Scalar::Str(s) => s,
                        other => panic!("{other:?}"),
                    })
                    .collect::<Vec<_>>(),
            ),
            Ok(other) => panic!("{other:?}"),
            Err(_) => None,
        };
        let plain = |text: &str| read(FieldType::String(None), Array::Unbounded, text);
        let strings =
            |items: &[&str]| Some(items.iter().map(|s| (*s).to_owned()).collect::<Vec<_>>());
        assert_eq!(plain(r#"["a,b", 'c', d,]"#), strings(&["a,b", "c", "d"]));
        assert_eq!(
            plain(r#"[" a ", 'b ']"#),
            strings(&["a", "b"]),
            "elements are stripped after unquoting"
        );
        assert_eq!(plain("[  a  ,  b  ]"), strings(&["a", "b"]));
        assert_eq!(plain("[]"), strings(&[]));
        assert_eq!(
            plain(r#"["a"'b']"#),
            strings(&["a", "b"]),
            "no comma is needed after a quote"
        );
        assert_eq!(plain(r#"["a\"b", "c"]"#), strings(&["a\"b", "c"]));
        assert_eq!(
            plain(r#"["'x'"]"#),
            strings(&["x"]),
            "a second layer of quotes comes off as well"
        );
        assert_eq!(
            plain(r#"["a\"b\"c", x]"#),
            strings(&["a\"", "\\\"c\"", "x"]),
            "the reference's slip after two escapes"
        );
        for bad in [
            r#"["a", ]"#,
            "[   ]",
            r#"["a"#,
            r#"["a]"#,
            "[,a]",
            r"['\'']",
            r#"["\"x\""]"#,
        ] {
            assert_eq!(plain(bad), None, "{bad}");
        }
        assert_eq!(
            read(FieldType::String(None), Array::Fixed(3), "[a, b]"),
            None
        );
        assert_eq!(
            read(FieldType::String(None), Array::Bounded(2), "[a, b, c]"),
            None
        );
        assert_eq!(
            read(FieldType::String(Some(3)), Array::Unbounded, r#"["abcd"]"#),
            None
        );
    }

    #[test]
    fn json_conversion_keeps_types_and_drops_non_finite_floats() {
        let json = |ty, array, text| parse_value(&ty, array, text).unwrap().to_json();
        assert_eq!(
            json(FieldType::F64, Array::Scalar, "1"),
            Some(serde_json::json!(1.0))
        );
        assert_eq!(
            json(FieldType::U64, Array::Scalar, "18446744073709551615"),
            Some(serde_json::json!(u64::MAX))
        );
        assert_eq!(
            json(FieldType::I64, Array::Scalar, "-9223372036854775808"),
            Some(serde_json::json!(i64::MIN))
        );
        assert_eq!(
            json(FieldType::Bool, Array::Unbounded, "[true, 0]"),
            Some(serde_json::json!([true, false]))
        );
        assert_eq!(json(FieldType::F32, Array::Scalar, "inf"), None);
    }
}
