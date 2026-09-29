//! Property tests: the parser never panics, and a valid message survives being printed.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::{collections::BTreeMap, fs, path::PathBuf};

use proptest::{
    prelude::*,
    strategy::ValueTree,
    test_runner::{Config, RngAlgorithm, TestRng, TestRunner},
};
use rosidl_schema::{
    Overrides, Part, Registry, TypeName, flatten_refs, json_schema, parse_action, parse_message,
    parse_service,
};

#[path = "common/reference.rs"]
mod reference;

/// Pieces that make a line look like an interface file, so random text reaches the deep paths.
const TOKENS: &[&str] = &[
    "int32",
    "uint8",
    "float64",
    "bool",
    "string",
    "wstring",
    "string<=",
    "wstring<=",
    "byte",
    "char",
    "time",
    "duration",
    "Header",
    "pkg/Name",
    "Name",
    "other/Thing",
    "x",
    "foo_bar",
    "A_B",
    "ARM_LEFT",
    "[]",
    "[3]",
    "[<=4]",
    "[<=",
    "[",
    "]",
    ",",
    "=",
    "<=",
    "-",
    "1",
    "0x1f",
    "-2.5e3",
    "nan",
    "true",
    "\"a b\"",
    "'q'",
    "\\\"",
    "\"",
    "'",
    "#",
    "# ",
    "## ",
    "# [m]",
    " [rad] ",
    "---",
    "--- ",
    " ",
    "  ",
    "\t",
    "\n",
    "\n",
    "\n",
    "\r\n",
    "\r",
    "\u{85}",
    "\u{2028}",
    "\u{1f}",
    "\u{1c}",
    "\u{a0}",
    "\u{3000}",
    "\u{2003}",
    "\u{feff}",
    "\u{200b}",
    "\x0b",
    "\x0c",
    "é",
    "日本",
    "\0",
    "\\",
    "/",
    "_",
    "__",
    "9",
    "0",
    "0x_",
    "1_0",
    ".",
    "e",
];

fn soup() -> impl Strategy<Value = String> {
    prop::collection::vec(prop::sample::select(TOKENS), 0..60).prop_map(|t| t.concat())
}

fn any_text() -> impl Strategy<Value = String> {
    prop_oneof![soup(), soup(), any::<String>(), "(?s).{0,200}"]
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(4000))]

    #[test]
    fn the_parser_never_panics(text in any_text()) {
        let _ = parse_message("pkg", "Name", &text);
        let _ = parse_service("pkg", "Name", &text);
        let _ = parse_action("pkg", "Name", &text);
    }

    #[test]
    fn type_names_never_panic(text in any_text()) {
        let _ = text.parse::<TypeName>();
    }

    #[test]
    fn schemas_of_whatever_parses_never_panic(text in soup()) {
        let name: TypeName = "pkg/msg/Name".parse().unwrap();
        let mut registry = Registry::default();
        if registry.add_source(&name, &text).is_ok()
            && let Ok(schema) = json_schema(&registry, &name, Part::Message, &Overrides::default())
        {
            let _ = flatten_refs(&schema);
        }
    }
}

// ----- a generator of valid messages, written as file text -----

/// Words for comments: no `#`, brackets or commas, which have meaning to the parser.
fn words() -> impl Strategy<Value = String> {
    "[A-Za-z0-9.;:()/'-]{1,8}( [A-Za-z0-9.;:()/'-]{1,8}){0,3}"
}

/// A comment line with 0 to 3 spaces after the `#`, and maybe a unit at its end.
fn comment_line(unit: bool) -> impl Strategy<Value = String> {
    let unit = if unit {
        prop::option::weighted(0.2, "[a-z/]{1,4}").boxed()
    } else {
        Just(None).boxed()
    };
    (0..4usize, words(), unit).prop_map(|(indent, text, unit)| {
        let unit = unit.map_or_else(String::new, |u| format!(" [{u}]"));
        format!("#{}{text}{unit}", " ".repeat(indent))
    })
}

#[derive(Debug, Clone, Copy)]
enum Shape {
    Scalar,
    Fixed,
    Bounded,
    Unbounded,
}

/// A type with a strategy for one literal of it.
fn scalar_type() -> impl Strategy<Value = (String, BoxedStrategy<String>)> {
    let ints = |ty: &str, lo: i64, hi: i64| {
        (
            ty.to_owned(),
            prop_oneof![
                (lo..=hi).prop_map(|i| i.to_string()),
                (lo.max(0)..=hi).prop_map(|i| format!("0x{i:x}"))
            ]
            .boxed(),
        )
    };
    let float = prop_oneof![
        (-1000i32..1000, 0u32..100).prop_map(|(a, b)| format!("{a}.{b}")),
        Just("1e3".to_owned()),
        Just("-2.5e-2".to_owned()),
        (-50i32..50).prop_map(|i| i.to_string()),
    ]
    .boxed();
    let quoted = "[a-z ]{0,6}".prop_map(|s| format!("\"{s}\"")).boxed();
    let text = prop_oneof![quoted.clone(), "[a-z]{1,6}".prop_map(|s| format!("'{s}'"))].boxed();
    prop::sample::select(vec![
        (
            "bool".to_owned(),
            prop::sample::select(vec!["true", "false", "True", "0", "1"])
                .prop_map(str::to_owned)
                .boxed(),
        ),
        ints("int8", -128, 127),
        ints("uint8", 0, 255),
        ints("byte", 0, 255),
        ints("char", 0, 255),
        ints("int16", -300, 300),
        ints("uint32", 0, 100_000),
        ints("int64", -100_000, 100_000),
        ("float32".to_owned(), float.clone()),
        ("float64".to_owned(), float),
        ("string".to_owned(), text.clone()),
        ("wstring".to_owned(), quoted.clone()),
        ("string<=6".to_owned(), quoted),
    ])
}

fn shape() -> impl Strategy<Value = (Shape, &'static str)> {
    prop::sample::select(vec![
        (Shape::Scalar, ""),
        (Shape::Fixed, "[3]"),
        (Shape::Bounded, "[<=4]"),
        (Shape::Unbounded, "[]"),
    ])
}

/// `type name [default]`, without the name.
fn field_code() -> impl Strategy<Value = (String, Option<String>)> {
    let primitive = (scalar_type(), shape(), any::<bool>()).prop_flat_map(
        |((ty, literal), (shape, suffix), with_default)| {
            let list = |n: std::ops::RangeInclusive<usize>| {
                prop::collection::vec(literal.clone(), n)
                    .prop_map(|v| format!("[{}]", v.join(", ")))
            };
            let default = match shape {
                Shape::Scalar => literal.clone().boxed(),
                Shape::Fixed => list(3..=3).boxed(),
                Shape::Bounded => list(0..=4).boxed(),
                Shape::Unbounded => list(0..=3).boxed(),
            };
            (
                Just(format!("{ty}{suffix}")),
                prop::option::of(default).prop_map(move |d| d.filter(|_| with_default)),
            )
        },
    );
    let nested = (
        prop::sample::select(vec![
            "Other",
            "pkg/Other",
            "other_pkg/Thing",
            "time",
            "duration",
        ]),
        shape(),
    )
        .prop_map(|(ty, (_, suffix))| (format!("{ty}{suffix}"), None));
    prop_oneof![4 => primitive, 1 => nested]
}

/// Where the comments of one element go: above it, beside it, and on indented lines after it.
#[derive(Debug, Clone)]
struct Layout {
    blank_before: bool,
    block: Vec<String>,
    beside: Option<String>,
    after: Vec<String>,
}

fn layout() -> impl Strategy<Value = Layout> {
    (
        any::<bool>(),
        prop::collection::vec(comment_line(true), 0..3),
        prop::option::of(comment_line(true)),
        prop::collection::vec(comment_line(false), 0..2),
    )
        .prop_map(|(blank_before, block, beside, after)| Layout {
            blank_before,
            block,
            beside,
            after,
        })
}

fn emit(lines: &mut Vec<String>, layout: &Layout, code: &str) {
    if layout.blank_before {
        lines.push(String::new());
    }
    lines.extend(layout.block.iter().cloned());
    let beside = layout
        .beside
        .as_ref()
        .map_or_else(String::new, |c| format!(" {c}"));
    lines.push(format!("{code}{beside}"));
    lines.extend(layout.after.iter().map(|c| format!("  {c}")));
}

/// A constant's type and one value of it. Bounded strings cannot be constants.
fn constant() -> impl Strategy<Value = (String, String)> {
    scalar_type()
        .prop_flat_map(|(ty, literal)| (Just(ty), literal))
        .prop_filter("constants have no string bounds", |(ty, _)| {
            !ty.contains('<')
        })
}

/// A valid message as file text: names come from the position, which keeps them unique.
fn valid_message_text() -> impl Strategy<Value = String> {
    let constants = prop::collection::vec(
        (
            prop::sample::select(vec!["A", "MODE", "ARM"]),
            constant(),
            layout(),
        ),
        0..4,
    );
    let fields = prop::collection::vec(
        (
            prop::sample::select(vec!["a", "pos", "frame"]),
            field_code(),
            layout(),
        ),
        0..6,
    );
    (
        prop::collection::vec(comment_line(true), 0..3),
        any::<bool>(),
        constants,
        fields,
    )
        .prop_map(|(doc, blank_after_doc, constants, fields)| {
            let mut lines = doc;
            if blank_after_doc {
                lines.push(String::new());
            }
            for (i, (base, (ty, value), layout)) in constants.iter().enumerate() {
                emit(&mut lines, layout, &format!("{ty} {base}_{i}={value}"));
            }
            for (i, (base, (ty, default), layout)) in fields.iter().enumerate() {
                let default = default
                    .as_ref()
                    .map_or_else(String::new, |d| format!(" {d}"));
                emit(&mut lines, layout, &format!("{ty} {base}{i}{default}"));
            }
            lines.join("\n") + "\n"
        })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(3000))]

    #[test]
    fn a_valid_message_survives_print_and_parse(text in valid_message_text()) {
        let first = parse_message("pkg", "Name", &text).unwrap_or_else(|e| panic!("{e}\n{text}"));
        let printed = first.to_string();
        let second = parse_message("pkg", "Name", &printed).unwrap_or_else(|e| panic!("{e}\n{printed}"));
        prop_assert_eq!(&second, &first, "\n--- generated:\n{}\n--- printed:\n{}", text, printed);
        prop_assert_eq!(second.to_string(), printed);
    }

    #[test]
    fn a_service_and_an_action_split_at_their_separators(
        parts in prop::collection::vec(valid_message_text(), 3..=3)
    ) {
        let service = parse_service("pkg", "Name", &format!("{}---\n{}", parts[0], parts[1])).unwrap();
        prop_assert_eq!(&service.request, &parse_message("pkg", "Name", &parts[0]).unwrap());
        prop_assert_eq!(&service.response, &parse_message("pkg", "Name", &parts[1]).unwrap());
        let action = parse_action("pkg", "Name", &format!("{}---\n{}---\n{}", parts[0], parts[1], parts[2])).unwrap();
        prop_assert_eq!(&action.goal, &service.request);
        prop_assert_eq!(&action.feedback, &parse_message("pkg", "Name", &parts[2]).unwrap());
    }
}

/// A valid message with a few random tokens pushed into it: near the edge of what parses.
fn noisy_message() -> impl Strategy<Value = String> {
    let edits = prop::collection::vec(
        (any::<prop::sample::Index>(), prop::sample::select(TOKENS)),
        0..3,
    );
    (valid_message_text(), edits).prop_map(|(text, edits)| {
        let mut chars: Vec<char> = text.chars().collect();
        for (at, token) in edits {
            let i = at.index(chars.len() + 1);
            chars.splice(i..i, token.chars());
        }
        chars.into_iter().collect()
    })
}

/// Random files, valid and broken, read by this parser and by the reference one: they must accept
/// the same files and read them the same way. The samples repeat from run to run; set
/// `FUZZ_CASES` for more of them and `FUZZ_SEED` for other ones.
#[test]
#[ignore = "needs a ROS 2 install (ROS_PREFIX, default /opt/ros/jazzy) and python3"]
fn random_files_are_read_like_the_reference_parser() {
    let cases: usize = std::env::var("FUZZ_CASES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(3000);
    let prefix =
        PathBuf::from(std::env::var_os("ROS_PREFIX").unwrap_or_else(|| "/opt/ros/jazzy".into()));
    let root = std::env::temp_dir().join(format!("rosidl-schema-fuzz-{}", std::process::id()));
    let package = root.join("fuzzpkg");
    for kind in ["msg", "srv", "action"] {
        fs::create_dir_all(package.join(kind)).unwrap();
    }

    let part = prop_oneof![3 => noisy_message(), 1 => soup(), 2 => valid_message_text()];
    let strategy = (0..3usize, prop::collection::vec(part, 3..=3));
    let seed: u8 = std::env::var("FUZZ_SEED")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let mut runner = TestRunner::new_with_rng(
        Config::default(),
        TestRng::from_seed(RngAlgorithm::ChaCha, &[seed; 32]),
    );
    let mut texts = BTreeMap::new();
    for i in 0..cases {
        let (kind, parts) = strategy.new_tree(&mut runner).unwrap().current();
        let (kind, text) = match kind {
            0 => ("msg", parts[0].clone()),
            1 => ("srv", format!("{}\n---\n{}", parts[0], parts[1])),
            _ => (
                "action",
                format!("{}\n---\n{}\n---\n{}", parts[0], parts[1], parts[2]),
            ),
        };
        fs::write(package.join(kind).join(format!("F{i}.{kind}")), &text).unwrap();
        texts.insert(format!("{kind}/F{i}"), text);
    }

    let golden = reference::run_reference(&[&package], true, &prefix);
    let mut d = reference::Diffs::default();
    let mut rejected = 0;
    for (key, entry) in &golden["fuzzpkg"] {
        rejected += usize::from(entry.get("error").is_some());
        reference::compare_entry(&mut d, "fuzzpkg", key, &texts[key], entry);
    }
    println!(
        "{cases} random files: {rejected} rejected by both, {} read by both ({} fields, {} constants compared)",
        cases - rejected,
        d.fields,
        d.constants
    );
    if !d.found.is_empty() {
        let inputs: Vec<_> = d
            .found
            .iter()
            .take(5)
            .map(|line| {
                let key = line
                    .strip_prefix("fuzzpkg/")
                    .and_then(|l| l.split([':', '.']).next())
                    .unwrap_or("");
                format!(
                    "{line}\n  input: {:?}",
                    texts.get(key).map_or("?", String::as_str)
                )
            })
            .collect();
        panic!(
            "{} difference(s); the files stay in {}:\n{}",
            d.found.len(),
            root.display(),
            inputs.join("\n")
        );
    }
    fs::remove_dir_all(root).unwrap();
}
