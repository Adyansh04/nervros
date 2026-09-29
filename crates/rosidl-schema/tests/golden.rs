//! The parser against ROS 2's own: every fixture is parsed here and by `rosidl_adapter`, and the
//! two results must agree on names, types, array shapes, defaults, constants, comments and units.
//!
//! The reference results are the JSON files in `tests/golden/`, made from the files in
//! `tests/fixtures/` by `tools/dump_golden.py`. The ignored test at the bottom does the same for
//! every interface of an installed ROS 2, running the script on the fly.
//!
//! Where the reference's representation differs from ours by definition, the comparison in
//! `tests/common/reference.rs` converts it; these are all the conversions:
//! 1. `time` and `duration` are primitives there and the `builtin_interfaces` messages here.
//! 2. No comment is an empty list there and `None` here.
//! 3. An array is `array`, `size` and `upper` there and one of four variants here.
//! 4. A default or constant is a typed Python value there and text here, compared through
//!    `Field::default_json` and `Constant::json_value`.
//! 5. A `Nested` type carries `Kind::Msg`, which the reference leaves implicit.
//! 6. Units of messages and constants are not compared. The reference extracts them, and the
//!    comment text without them is compared, but this crate keeps a unit only for fields.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::{
    fs,
    path::{Path, PathBuf},
};

use rosidl_schema::{Interface, Registry, SearchPath, TypeName, parse_interface};

#[path = "common/reference.rs"]
mod reference;

use reference::{Diffs, Golden, compare_entry, compare_interface, run_reference};

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn read_golden(path: &Path) -> Golden {
    serde_json::from_str(&fs::read_to_string(path).unwrap()).unwrap()
}

#[test]
fn fixtures_match_the_reference_parser() {
    let fixtures = manifest_dir().join("tests/fixtures");
    let mut d = Diffs::default();
    let mut checked = 0;
    let mut packages: Vec<_> = fs::read_dir(&fixtures)
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    packages.sort();
    assert!(
        packages.len() >= 14,
        "fixture packages are missing: {packages:?}"
    );
    for dir in packages.iter().filter(|p| p.is_dir()) {
        let package = dir.file_name().unwrap().to_str().unwrap().to_owned();
        let golden = read_golden(&manifest_dir().join(format!("tests/golden/{package}.json")));
        let reference = &golden[&package];

        let mut on_disk = Vec::new();
        for kind in ["msg", "srv", "action"] {
            let Ok(entries) = fs::read_dir(dir.join(kind)) else {
                continue;
            };
            for entry in entries {
                let path = entry.unwrap().path();
                let stem = path.file_stem().unwrap().to_str().unwrap();
                let key = format!("{kind}/{stem}");
                let text = fs::read_to_string(&path).unwrap();
                let Some(entry) = reference.get(&key) else {
                    d.found
                        .push(format!("{package}/{key}: missing from the golden file"));
                    continue;
                };
                compare_entry(&mut d, &package, &key, &text, entry);
                on_disk.push(key);
                checked += 1;
            }
        }
        on_disk.sort();
        let mut expected: Vec<_> = reference.keys().cloned().collect();
        expected.sort();
        d.check(&package, "interface set", &on_disk, &expected);
    }
    assert!(checked > 200, "only {checked} interfaces were compared");
    println!(
        "compared {checked} interfaces, {} fields, {} constants",
        d.fields, d.constants
    );
    d.assert_none(checked);
}

/// The grove-g1 interfaces are the ones the agent is built for; a few hand checks on top of the
/// golden comparison keep the fixtures honest.
#[test]
fn grove_g1_interfaces_keep_their_comments_and_constants() {
    let fixtures = manifest_dir().join("tests/fixtures/g1_msgs/action/Pick.action");
    let name: TypeName = "g1_msgs/action/Pick".parse().unwrap();
    let Interface::Action(pick) =
        parse_interface(&name, &fs::read_to_string(fixtures).unwrap()).unwrap()
    else {
        panic!("Pick is an action");
    };
    assert!(
        pick.goal
            .doc
            .unwrap()
            .starts_with("Pick a known object off a surface with one arm.")
    );
    // The comment above the ARM_* constants belongs to ARM_LEFT, as in rosidl_adapter, not to `arm`.
    let arm = pick.goal.fields.iter().find(|f| f.name == "arm").unwrap();
    assert_eq!(arm.doc, None);
    let constants: Vec<_> = pick
        .goal
        .constants
        .iter()
        .map(|c| (c.name.as_str(), c.value.as_str()))
        .collect();
    assert_eq!(constants, [("ARM_LEFT", "left"), ("ARM_RIGHT", "right")]);
    assert_eq!(
        pick.goal.constants[0].doc.as_deref(),
        Some("Selects the arm group, its hand group and the palm to attach to.")
    );
    assert_eq!(pick.feedback.constants.len(), 5);
}

/// Every interface of an installed ROS 2 against the reference parser, with the reference results
/// made on the fly. Run with `cargo test -p rosidl-schema --test golden -- --ignored --nocapture`.
#[test]
#[ignore = "needs a ROS 2 install (ROS_PREFIX, default /opt/ros/jazzy) and python3"]
fn every_installed_interface_matches_the_reference_parser() {
    let prefix =
        PathBuf::from(std::env::var_os("ROS_PREFIX").unwrap_or_else(|| "/opt/ros/jazzy".into()));
    let share = prefix.join("share");
    let golden = run_reference(&[Path::new("--scan"), &share], false, &prefix);

    let registry = Registry::load(&[SearchPath::Prefix(prefix)], &[]).unwrap();
    assert!(
        registry.issues().is_empty(),
        "files that did not load: {:#?}",
        registry.issues()
    );

    let mut d = Diffs::default();
    let mut checked = 0;
    for (package, interfaces) in &golden {
        for (key, reference) in interfaces {
            let name: TypeName = format!("{package}/{key}").parse().unwrap();
            match registry.get(&name) {
                Some(ours) => compare_interface(&mut d, &name.to_string(), ours, reference),
                None => d
                    .found
                    .push(format!("{name}: the registry does not have it")),
            }
            checked += 1;
        }
    }
    d.check("registry", "interface count", &registry.len(), &checked);
    println!(
        "compared {checked} interfaces in {} packages, {} fields and {} constants: {} difference(s)",
        golden.len(),
        d.fields,
        d.constants,
        d.found.len()
    );
    d.assert_none(checked);
}
