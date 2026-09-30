//! JSON Schema generation: the mapping rules on small inline interfaces, and snapshots of the
//! schemas the agent is built for.
//!
//! Snapshots are plain JSON files in `tests/snapshots/`. After an intended change, rewrite them
//! with `UPDATE_SNAPSHOTS=1 cargo test -p rosidl-schema --test schema` and review the diff.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::{fs, path::PathBuf};

use rosidl_schema::{
    Error, FieldOverride, Overrides, Part, Registry, SearchPath, TypeName, flatten_refs,
    json_schema,
};
use serde_json::{Value, json};

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// A registry of inline interfaces: `("pkg/msg/Name", "file text")`.
fn registry(sources: &[(&str, &str)]) -> Registry {
    let mut registry = Registry::default();
    for (name, text) in sources {
        registry.add_source(&name.parse().unwrap(), text).unwrap();
    }
    registry
}

fn schema_of(registry: &Registry, name: &str, part: Part, overrides: &Overrides) -> Value {
    json_schema(registry, &name.parse().unwrap(), part, overrides).unwrap()
}

/// The schema of `pkg/msg/Name` built from one inline message.
fn msg_schema(text: &str) -> Value {
    schema_of(
        &registry(&[("pkg/msg/Name", text)]),
        "pkg/msg/Name",
        Part::Message,
        &Overrides::default(),
    )
}

fn prop<'a>(schema: &'a Value, name: &str) -> &'a Value {
    &schema["properties"][name]
}

fn with_field(path: &str, over: FieldOverride) -> Overrides {
    let mut overrides = Overrides::default();
    overrides.fields.insert(path.to_owned(), over);
    overrides
}

#[test]
fn the_root_is_a_closed_object_of_draft_2020_12() {
    let s = msg_schema("# Doc.\n\nint32 a\n");
    assert_eq!(s["$schema"], "https://json-schema.org/draft/2020-12/schema");
    assert_eq!(s["type"], "object");
    assert_eq!(s["additionalProperties"], false);
    assert_eq!(s["description"], "Doc.");
    assert!(s.get("required").is_none() && s.get("$defs").is_none());
}

#[test]
fn integers_carry_the_range_of_their_type() {
    let s = msg_schema(
        "\nint8 a\nuint8 b\nint16 c\nuint16 d\nint32 e\nuint32 f\nint64 g\nuint64 h\nchar i\nbyte j\n",
    );
    let range = |name: &str| {
        (
            prop(&s, name)["minimum"].clone(),
            prop(&s, name)["maximum"].clone(),
        )
    };
    assert_eq!(range("a"), (json!(-128), json!(127)));
    assert_eq!(range("b"), (json!(0), json!(255)));
    assert_eq!(range("c"), (json!(i16::MIN), json!(i16::MAX)));
    assert_eq!(range("d"), (json!(0), json!(u16::MAX)));
    assert_eq!(range("e"), (json!(i32::MIN), json!(i32::MAX)));
    assert_eq!(range("f"), (json!(0), json!(u32::MAX)));
    assert_eq!(range("g"), (json!(i64::MIN), json!(i64::MAX)));
    assert_eq!(range("h"), (json!(0), json!(u64::MAX)));
    assert_eq!(range("i"), (json!(0), json!(255)), "char is uint8 in ROS 2");
    assert_eq!(range("j"), (json!(0), json!(255)));
    assert!(
        ["a", "b", "c", "d", "e", "f", "g", "h", "i", "j"]
            .iter()
            .all(|n| prop(&s, n)["type"] == "integer")
    );
}

#[test]
fn bool_float_and_string_types_map_to_their_json_types() {
    let s = msg_schema(
        "\nbool a\nfloat32 b\nfloat64 c\nstring d\nwstring e\nstring<=8 f\nwstring<=9 g\n",
    );
    assert_eq!(prop(&s, "a"), &json!({"type": "boolean"}));
    assert_eq!(prop(&s, "b"), &json!({"type": "number"}));
    assert_eq!(prop(&s, "c"), &json!({"type": "number"}));
    assert_eq!(prop(&s, "d"), &json!({"type": "string"}));
    assert_eq!(prop(&s, "e"), &json!({"type": "string"}));
    assert_eq!(prop(&s, "f"), &json!({"type": "string", "maxLength": 8}));
    assert_eq!(prop(&s, "g"), &json!({"type": "string", "maxLength": 9}));
}

#[test]
fn arrays_get_their_length_limits() {
    let s =
        msg_schema("\nfloat64[3] fixed\nfloat64[<=4] bounded\nfloat64[] open\nstring<=2[] names\n");
    assert_eq!(
        prop(&s, "fixed"),
        &json!({"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3})
    );
    assert_eq!(
        prop(&s, "bounded"),
        &json!({"type": "array", "items": {"type": "number"}, "maxItems": 4})
    );
    assert_eq!(
        prop(&s, "open"),
        &json!({"type": "array", "items": {"type": "number"}})
    );
    assert_eq!(
        prop(&s, "names")["items"],
        json!({"type": "string", "maxLength": 2})
    );
}

#[test]
fn descriptions_are_the_doc_with_the_unit_after_it() {
    let s = msg_schema(
        "\n# Distance to the goal [m]\nfloat64 with_both\n# [rad]\nfloat64 unit_only\n# Just text.\nfloat64 doc_only\nfloat64 neither\n",
    );
    assert_eq!(
        prop(&s, "with_both")["description"],
        "Distance to the goal [m]"
    );
    assert_eq!(prop(&s, "unit_only")["description"], "[rad]");
    assert_eq!(prop(&s, "doc_only")["description"], "Just text.");
    assert!(prop(&s, "neither").get("description").is_none());
}

#[test]
fn multi_line_comments_keep_their_line_breaks() {
    let s = msg_schema("\n# first\n# second\nint32 a\n");
    assert_eq!(prop(&s, "a")["description"], "first\nsecond");
}

#[test]
fn defaults_are_typed_json() {
    let s = msg_schema(
        "\nfloat64 w 1\nstring s \"hi\"\nbool b true\nint32[] xs [1, 2]\nuint8 h 0x10\nint32 none\n",
    );
    assert_eq!(prop(&s, "w")["default"], json!(1.0));
    assert_eq!(prop(&s, "s")["default"], "hi");
    assert_eq!(prop(&s, "b")["default"], true);
    assert_eq!(prop(&s, "xs")["default"], json!([1, 2]));
    assert_eq!(prop(&s, "h")["default"], 16);
    assert!(prop(&s, "none").get("default").is_none());
}

#[test]
fn constants_are_listed_in_the_description() {
    let s = msg_schema(
        "# Doc.\n\nuint8 MODE_FAST=1\nstring NAME=\"x y\"\nstring EMPTY=\nbool ON=true\nfloat64 G=9.81\nint32 a\n",
    );
    assert_eq!(
        s["description"],
        "Doc.\n\nConstants:\nMODE_FAST=1\nNAME=x y\nEMPTY=\"\"\nON=true\nG=9.81"
    );
    let only = msg_schema("\nuint8 A_B=2\nint32 a\n");
    assert_eq!(only["description"], "Constants:\nA_B=2");
}

#[test]
fn a_constant_prefix_gives_the_field_an_enum() {
    let s = msg_schema(
        "\nstring ARM_LEFT=left\nstring ARM_RIGHT=right\nstring arm\nuint8 STATE_A=0\nuint8 STATE_B=2\nuint8 state\nuint8 OTHER=5\nuint8 other_field\n",
    );
    assert_eq!(prop(&s, "arm")["enum"], json!(["left", "right"]));
    assert_eq!(prop(&s, "state")["enum"], json!([0, 2]));
    assert!(
        prop(&s, "other_field").get("enum").is_none(),
        "OTHER is not OTHER_FIELD_*"
    );
}

#[test]
fn the_prefix_rule_needs_the_same_type() {
    let s = msg_schema("\nuint8 ARM_LEFT=1\nstring arm\nstring MODE_A=ab\nstring<=4 mode\n");
    assert!(prop(&s, "arm").get("enum").is_none());
    assert_eq!(
        prop(&s, "mode")["enum"],
        json!(["ab"]),
        "a string bound does not matter"
    );
}

#[test]
fn the_prefix_rule_reaches_into_arrays_and_can_be_switched_off() {
    let r = registry(&[(
        "pkg/msg/Name",
        "\nstring ARM_LEFT=left\nstring[] arm\nstring ARM_X=x\nstring arm2\n",
    )]);
    let name = "pkg/msg/Name";
    let on = schema_of(&r, name, Part::Message, &Overrides::default());
    assert_eq!(prop(&on, "arm")["items"]["enum"], json!(["left", "x"]));
    let off = Overrides {
        enum_from_constants: false,
        ..Overrides::default()
    };
    let off = schema_of(&r, name, Part::Message, &off);
    assert!(prop(&off, "arm")["items"].get("enum").is_none());
}

#[test]
fn an_enum_override_replaces_the_constants_and_an_empty_one_removes_them() {
    let r = registry(&[(
        "pkg/msg/Name",
        "\nstring ARM_LEFT=left\nstring arm\nstring ROLE_A=a\nstring role\nstring free\n",
    )]);
    let over = |values: Vec<Value>| FieldOverride {
        enum_values: Some(values),
        ..FieldOverride::default()
    };
    let mut overrides = with_field("arm", over(vec![json!("both"), json!("left")]));
    overrides.fields.insert("role".to_owned(), over(vec![]));
    overrides
        .fields
        .insert("free".to_owned(), over(vec![json!("x")]));
    let s = schema_of(&r, "pkg/msg/Name", Part::Message, &overrides);
    assert_eq!(prop(&s, "arm")["enum"], json!(["both", "left"]));
    assert!(prop(&s, "role").get("enum").is_none());
    assert_eq!(prop(&s, "free")["enum"], json!(["x"]));
}

#[test]
fn byte_blobs_are_hidden_unless_an_override_keeps_them() {
    let text = "\nuint8[] blob\nbyte[] raw\nuint8[<=16] short\nuint8[16] uuid\nint8[] signed\nfloat32[] embedding\n";
    let r = registry(&[("pkg/msg/Name", text)]);
    let s = schema_of(&r, "pkg/msg/Name", Part::Message, &Overrides::default());
    // Sorted: key order depends on whether serde_json's `preserve_order` is on in the build.
    let mut names: Vec<_> = s["properties"]
        .as_object()
        .unwrap()
        .keys()
        .cloned()
        .collect();
    names.sort();
    assert_eq!(names, ["embedding", "signed", "uuid"]);

    let keep = FieldOverride {
        hidden: Some(false),
        ..FieldOverride::default()
    };
    let s = schema_of(&r, "pkg/msg/Name", Part::Message, &with_field("blob", keep));
    assert_eq!(prop(&s, "blob")["items"]["maximum"], 255);
    assert!(s["properties"].get("raw").is_none());
}

#[test]
fn any_field_can_be_hidden() {
    let hide = FieldOverride {
        hidden: Some(true),
        ..FieldOverride::default()
    };
    let r = registry(&[(
        "pkg/msg/Name",
        "\nfloat32[] query_embedding\nstring query\n",
    )]);
    let s = schema_of(
        &r,
        "pkg/msg/Name",
        Part::Message,
        &with_field("query_embedding", hide),
    );
    assert_eq!(
        s["properties"]
            .as_object()
            .unwrap()
            .keys()
            .collect::<Vec<_>>(),
        ["query"]
    );
}

#[test]
fn nothing_is_required_unless_an_override_says_so() {
    let r = registry(&[("pkg/msg/Name", "\nstring a\nstring b\nstring c\n")]);
    let req = FieldOverride {
        required: true,
        ..FieldOverride::default()
    };
    let mut overrides = with_field("c", req.clone());
    overrides.fields.insert("a".to_owned(), req);
    let s = schema_of(&r, "pkg/msg/Name", Part::Message, &overrides);
    assert_eq!(s["required"], json!(["a", "c"]));
}

#[test]
fn overrides_replace_description_and_default_and_narrow_ranges() {
    let r = registry(&[(
        "pkg/msg/Name",
        "\n# Old.\nuint8 level 3\nfloat64 x\nint32 n\nfloat64 y\n",
    )]);
    let mut overrides = with_field(
        "level",
        FieldOverride {
            description: Some("New.".into()),
            default: Some(json!(7)),
            min: Some(1.2),
            max: Some(9.9),
            ..FieldOverride::default()
        },
    );
    overrides.fields.insert(
        "x".into(),
        FieldOverride {
            min: Some(-1.5),
            max: Some(2.5),
            ..FieldOverride::default()
        },
    );
    overrides.fields.insert(
        "n".into(),
        FieldOverride {
            min: Some(-1e30),
            max: Some(1e30),
            ..FieldOverride::default()
        },
    );
    overrides.description = Some("Top.".into());
    let s = schema_of(&r, "pkg/msg/Name", Part::Message, &overrides);
    assert_eq!(s["description"], "Top.");
    let level = prop(&s, "level");
    assert_eq!(
        (
            &level["description"],
            &level["default"],
            &level["minimum"],
            &level["maximum"]
        ),
        (&json!("New."), &json!(7), &json!(2), &json!(9))
    );
    assert_eq!(
        (&prop(&s, "x")["minimum"], &prop(&s, "x")["maximum"]),
        (&json!(-1.5), &json!(2.5))
    );
    assert_eq!(
        (&prop(&s, "n")["minimum"], &prop(&s, "n")["maximum"]),
        (&json!(i32::MIN), &json!(i32::MAX)),
        "an override cannot widen the type's range"
    );
    assert!(prop(&s, "y").get("minimum").is_none());
}

#[test]
fn nested_messages_are_shared_definitions() {
    let r = registry(&[
        (
            "pkg/msg/Name",
            "\n# The start.\nOuter a\nOuter[] more\nOuter[2] pair\n",
        ),
        ("pkg/msg/Outer", "# An outer.\n\nInner i\n"),
        ("pkg/msg/Inner", "\nfloat64 x\n"),
    ]);
    let s = schema_of(&r, "pkg/msg/Name", Part::Message, &Overrides::default());
    assert_eq!(
        prop(&s, "a"),
        &json!({"$ref": "#/$defs/pkg.Outer", "description": "The start."})
    );
    assert_eq!(
        prop(&s, "more")["items"],
        json!({"$ref": "#/$defs/pkg.Outer"})
    );
    assert_eq!(prop(&s, "pair")["minItems"], 2);
    let defs = s["$defs"].as_object().unwrap();
    let mut def_names: Vec<_> = defs.keys().collect();
    def_names.sort();
    assert_eq!(def_names, ["pkg.Inner", "pkg.Outer"]);
    assert_eq!(defs["pkg.Outer"]["description"], "An outer.");
    assert_eq!(defs["pkg.Outer"]["additionalProperties"], false);
    assert_eq!(
        defs["pkg.Outer"]["properties"]["i"],
        json!({"$ref": "#/$defs/pkg.Inner"})
    );
}

#[test]
fn an_override_below_a_nested_field_gets_an_inline_copy() {
    let r = registry(&[
        ("pkg/msg/Name", "\nPoint a\nPoint b\n"),
        ("pkg/msg/Point", "\nfloat64 x\nfloat64 y\n"),
    ]);
    let over = FieldOverride {
        hidden: Some(true),
        ..FieldOverride::default()
    };
    let s = schema_of(&r, "pkg/msg/Name", Part::Message, &with_field("a.y", over));
    assert_eq!(
        prop(&s, "a")["properties"]
            .as_object()
            .unwrap()
            .keys()
            .collect::<Vec<_>>(),
        ["x"]
    );
    assert_eq!(prop(&s, "b"), &json!({"$ref": "#/$defs/pkg.Point"}));
    assert_eq!(
        s["$defs"]["pkg.Point"]["properties"]
            .as_object()
            .unwrap()
            .len(),
        2
    );
}

#[test]
fn overrides_reach_several_levels_down_and_hidden_wins_over_required() {
    let r = registry(&[
        ("pkg/msg/Name", "\nOuter o\nstring s\n"),
        ("pkg/msg/Outer", "\nInner i\nfloat64 z\n"),
        ("pkg/msg/Inner", "\nfloat64 x\nfloat64 y\n"),
    ]);
    let field = |f: fn(&mut FieldOverride)| {
        let mut over = FieldOverride::default();
        f(&mut over);
        over
    };
    let mut overrides = with_field("o.i.x", field(|o| o.required = true));
    overrides.fields.insert(
        "o.i.y".into(),
        field(|o| o.description = Some("Sideways.".into())),
    );
    overrides.fields.insert(
        "s".into(),
        field(|o| (o.required, o.hidden) = (true, Some(true))),
    );
    let s = schema_of(&r, "pkg/msg/Name", Part::Message, &overrides);
    let inner = &prop(&s, "o")["properties"]["i"];
    assert_eq!(inner["required"], json!(["x"]));
    assert_eq!(inner["properties"]["y"]["description"], "Sideways.");
    assert!(prop(&s, "o")["properties"]["z"].is_object());
    assert!(s.get("required").is_none() && s["properties"].get("s").is_none());
    assert!(
        s.get("$defs").is_none(),
        "every message on the path is inline: {s}"
    );
}

#[test]
fn an_override_can_name_a_field_through_an_array_of_messages() {
    let r = registry(&[
        ("pkg/msg/Name", "\nPoint[] points\n"),
        ("pkg/msg/Point", "\nfloat64 x\n"),
    ]);
    let over = FieldOverride {
        description: Some("Metres.".into()),
        ..FieldOverride::default()
    };
    let s = schema_of(
        &r,
        "pkg/msg/Name",
        Part::Message,
        &with_field("points.x", over),
    );
    assert_eq!(
        prop(&s, "points")["items"]["properties"]["x"]["description"],
        "Metres."
    );
}

#[test]
fn overrides_are_keyed_from_the_part_not_the_interface() {
    let r = registry(&[("pkg/srv/Name", "string q\n---\nstring q\n")]);
    let hide = FieldOverride {
        hidden: Some(true),
        ..FieldOverride::default()
    };
    let overrides = with_field("q", hide);
    let request = schema_of(&r, "pkg/srv/Name", Part::Request, &overrides);
    assert!(request["properties"].as_object().unwrap().is_empty());
}

#[test]
fn an_override_for_a_missing_field_is_an_error() {
    let r = registry(&[("pkg/msg/Name", "\nstring a\n")]);
    let name: TypeName = "pkg/msg/Name".parse().unwrap();
    let err = json_schema(
        &r,
        &name,
        Part::Message,
        &with_field("typo", FieldOverride::default()),
    )
    .unwrap_err();
    assert!(
        matches!(&err, Error::UnknownOverride(p) if p == "typo"),
        "{err}"
    );
    let err = json_schema(
        &r,
        &name,
        Part::Message,
        &with_field("a.b", FieldOverride::default()),
    )
    .unwrap_err();
    assert!(matches!(err, Error::UnknownOverride(_)));
}

#[test]
fn services_and_actions_have_their_parts() {
    let r = registry(&[
        (
            "pkg/srv/Find",
            "# In.\n\nstring q\n---\n# Out.\n\nbool ok\n",
        ),
        ("pkg/action/Go", "int32 g\n---\nint32 r\n---\nint32 f\n"),
    ]);
    let s = |name, part| schema_of(&r, name, part, &Overrides::default());
    assert_eq!(s("pkg/srv/Find", Part::Request)["description"], "In.");
    assert!(prop(&s("pkg/srv/Find", Part::Request), "q").is_object());
    assert!(prop(&s("pkg/srv/Find", Part::Response), "ok").is_object());
    assert!(prop(&s("pkg/action/Go", Part::Goal), "g").is_object());
    assert!(prop(&s("pkg/action/Go", Part::Result), "r").is_object());
    assert!(prop(&s("pkg/action/Go", Part::Feedback), "f").is_object());

    let goal = "pkg/srv/Find".parse().unwrap();
    let err = json_schema(&r, &goal, Part::Goal, &Overrides::default()).unwrap_err();
    assert!(matches!(
        err,
        Error::NoSuchPart {
            part: Part::Goal,
            ..
        }
    ));
    assert_eq!(err.to_string(), "`pkg/srv/Find` has no goal part");
}

#[test]
fn a_missing_type_is_reported_with_the_type_that_needs_it() {
    let r = registry(&[("pkg/msg/Name", "\nother_pkg/Gone g\n")]);
    let name: TypeName = "pkg/msg/Name".parse().unwrap();
    let err = json_schema(&r, &name, Part::Message, &Overrides::default()).unwrap_err();
    assert_eq!(
        err.to_string(),
        "unknown type `other_pkg/msg/Gone`: no search path provides it (needed by `pkg/msg/Name`)"
    );
    let err = json_schema(
        &r,
        &"pkg/msg/Nope".parse().unwrap(),
        Part::Message,
        &Overrides::default(),
    )
    .unwrap_err();
    assert_eq!(
        err.to_string(),
        "unknown type `pkg/msg/Nope`: no search path provides it"
    );
    assert!(r.resolve(&name).is_err());
}

#[test]
fn a_message_that_contains_itself_is_an_error_not_a_loop() {
    let r = registry(&[("pkg/msg/A", "\nB b\n"), ("pkg/msg/B", "\nA a\n")]);
    let err = json_schema(
        &r,
        &"pkg/msg/A".parse().unwrap(),
        Part::Message,
        &Overrides::default(),
    )
    .unwrap_err();
    assert!(matches!(err, Error::Recursive(_)), "{err}");
    assert!(
        r.resolve(&"pkg/msg/A".parse().unwrap()).is_ok(),
        "resolve just stops at a repeat"
    );
}

#[test]
fn a_bare_header_is_std_msgs_header_unless_the_package_has_its_own() {
    let header = "\nbuiltin_interfaces/Time stamp\nstring frame_id\n";
    let r = registry(&[
        ("std_msgs/msg/Header", header),
        (
            "builtin_interfaces/msg/Time",
            "\nint32 sec\nuint32 nanosec\n",
        ),
        ("pkg/msg/Stamped", "\nHeader header\n"),
        ("other/msg/Own", "\nHeader header\n"),
        ("other/msg/Header", "\nstring mine\n"),
    ]);
    let s = schema_of(&r, "pkg/msg/Stamped", Part::Message, &Overrides::default());
    assert_eq!(
        prop(&s, "header"),
        &json!({"$ref": "#/$defs/std_msgs.Header"})
    );
    assert!(s["$defs"]["builtin_interfaces.Time"].is_object());
    let own = schema_of(&r, "other/msg/Own", Part::Message, &Overrides::default());
    assert_eq!(
        prop(&own, "header"),
        &json!({"$ref": "#/$defs/other.Header"})
    );
    let names: Vec<_> = r
        .resolve(&"pkg/msg/Stamped".parse().unwrap())
        .unwrap()
        .iter()
        .map(ToString::to_string)
        .collect();
    assert_eq!(
        names,
        [
            "pkg/msg/Stamped",
            "std_msgs/msg/Header",
            "builtin_interfaces/msg/Time"
        ]
    );
}

#[test]
fn flattening_inlines_every_reference() {
    let r = registry(&[
        ("pkg/msg/Name", "\n# The start.\nOuter a\nOuter[] more\n"),
        ("pkg/msg/Outer", "# An outer.\n\nInner i\n"),
        ("pkg/msg/Inner", "\nfloat64 x\n"),
    ]);
    let s = schema_of(&r, "pkg/msg/Name", Part::Message, &Overrides::default());
    let flat = flatten_refs(&s).unwrap();
    let text = flat.to_string();
    assert!(!text.contains("$ref") && !text.contains("$defs"), "{text}");
    let a = prop(&flat, "a");
    assert_eq!(
        a["description"], "The start.",
        "the field's own description wins over the definition's"
    );
    assert_eq!(
        a["properties"]["i"]["properties"]["x"],
        json!({"type": "number"})
    );
    assert_eq!(prop(&flat, "more")["items"]["description"], "An outer.");
    assert_eq!(flat["$schema"], s["$schema"]);
}

#[test]
fn flattening_rejects_bad_and_circular_references() {
    let bad = json!({"properties": {"a": {"$ref": "#/$defs/Nope"}}, "$defs": {}});
    assert!(matches!(flatten_refs(&bad), Err(Error::BadRef(_))));
    let elsewhere = json!({"properties": {"a": {"$ref": "http://example.com/x"}}});
    assert!(matches!(flatten_refs(&elsewhere), Err(Error::BadRef(_))));
    let cycle = json!({
        "properties": {"a": {"$ref": "#/$defs/A"}},
        "$defs": {"A": {"properties": {"b": {"$ref": "#/$defs/A"}}}}
    });
    assert!(matches!(flatten_refs(&cycle), Err(Error::RefDepth(_))));
    let plain = json!({"type": "object", "properties": {"a": {"type": "integer"}}});
    assert_eq!(flatten_refs(&plain).unwrap(), plain);
}

#[test]
fn overrides_come_from_config_files_as_json() {
    let o: Overrides = serde_json::from_value(json!({
        "description": "Top.",
        "fields": {"arm": {"enum": ["left"], "required": true}, "q": {"hidden": true, "min": 1}}
    }))
    .unwrap();
    assert!(o.enum_from_constants, "on unless switched off");
    assert_eq!(o.fields["arm"].enum_values, Some(vec![json!("left")]));
    assert!(o.fields["arm"].required && o.fields["q"].hidden == Some(true));
    assert!(serde_json::from_value::<Overrides>(json!({"nonsense": 1})).is_err());
    assert!(serde_json::from_value::<Overrides>(json!({"fields": {"a": {"typo": 1}}})).is_err());
    let off: Overrides = serde_json::from_value(json!({"enum_from_constants": false})).unwrap();
    assert!(!off.enum_from_constants);
}

fn fixture_registry() -> Registry {
    let root = manifest_dir().join("tests/fixtures");
    let search: Vec<_> = [
        "canopy_msgs",
        "g1_msgs",
        "geometry_msgs",
        "std_msgs",
        "builtin_interfaces",
        "sensor_msgs",
    ]
    .iter()
    .map(|p| SearchPath::Source(root.join(p)))
    .collect();
    let registry = Registry::load(&search, &[]).unwrap();
    assert!(registry.issues().is_empty(), "{:?}", registry.issues());
    registry
}

/// Compares `schema` with `tests/snapshots/<file>`, or rewrites the file when asked to.
fn assert_snapshot(file: &str, schema: &Value) {
    let path = manifest_dir().join("tests/snapshots").join(file);
    if std::env::var_os("UPDATE_SNAPSHOTS").is_some() {
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        fs::write(&path, serde_json::to_string_pretty(schema).unwrap() + "\n").unwrap();
        return;
    }
    let stored: Value =
        serde_json::from_str(&fs::read_to_string(&path).unwrap_or_else(|e| {
            panic!("{}: {e}; create it with UPDATE_SNAPSHOTS=1", path.display())
        }))
        .unwrap();
    assert!(
        &stored == schema,
        "{file} differs from the generated schema; if the change is intended, run with UPDATE_SNAPSHOTS=1.\nnow:\n{}",
        serde_json::to_string_pretty(schema).unwrap()
    );
}

fn snapshot(file: &str, name: &str, part: Part) {
    let schema = schema_of(&fixture_registry(), name, part, &Overrides::default());
    assert_snapshot(file, &schema);
}

#[test]
fn snapshot_find_objects_request() {
    snapshot(
        "canopy_msgs_FindObjects_request.json",
        "canopy_msgs/srv/FindObjects",
        Part::Request,
    );
}

#[test]
fn snapshot_find_objects_response() {
    snapshot(
        "canopy_msgs_FindObjects_response.json",
        "canopy_msgs/srv/FindObjects",
        Part::Response,
    );
}

#[test]
fn snapshot_pick_goal() {
    snapshot("g1_msgs_Pick_goal.json", "g1_msgs/action/Pick", Part::Goal);
}

#[test]
fn snapshot_pose_stamped() {
    snapshot(
        "geometry_msgs_PoseStamped.json",
        "geometry_msgs/msg/PoseStamped",
        Part::Message,
    );
}

#[test]
fn snapshot_of_a_flattened_and_overridden_schema() {
    let overrides = Overrides {
        description: Some("Find objects by name.".into()),
        fields: [
            (
                "query_embedding".to_owned(),
                FieldOverride {
                    hidden: Some(true),
                    ..FieldOverride::default()
                },
            ),
            (
                "max_results".to_owned(),
                FieldOverride {
                    default: Some(json!(5)),
                    min: Some(1.0),
                    max: Some(20.0),
                    ..FieldOverride::default()
                },
            ),
            (
                "query".to_owned(),
                FieldOverride {
                    required: true,
                    ..FieldOverride::default()
                },
            ),
        ]
        .into(),
        ..Overrides::default()
    };
    let name = "canopy_msgs/srv/FindObjects";
    let full = schema_of(&fixture_registry(), name, Part::Request, &overrides);
    assert_snapshot("canopy_msgs_FindObjects_request_overridden.json", &full);
    let objects = schema_of(
        &fixture_registry(),
        name,
        Part::Response,
        &Overrides::default(),
    );
    let flat = flatten_refs(&objects).unwrap();
    assert!(!flat.to_string().contains("$ref"));
    assert_snapshot("canopy_msgs_FindObjects_response_flat.json", &flat);
}
