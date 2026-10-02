//! The `.msg`, `.srv` and `.action` parser through its public functions: documentation, fields,
//! constants, defaults, bounds and the errors a malformed file gets.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use rosidl_schema::{
    Array, Error, Field, FieldType, Kind, Message, ParseError, ParseErrorKind, Result, Service,
    TypeName, parse_action, parse_message, parse_service,
};

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
    assert!(a.goal.fields.is_empty() && a.result.fields.is_empty() && a.feedback.fields.is_empty());
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
