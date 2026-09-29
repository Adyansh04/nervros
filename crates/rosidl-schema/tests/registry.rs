//! The registry: source trees, install prefixes, symlinks that a container made, precedence and
//! problems that must not stop the rest from loading.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::{
    fs,
    path::{Path, PathBuf},
};

use rosidl_schema::{Error, Interface, Registry, SearchPath, TypeName};

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn name(text: &str) -> TypeName {
    text.parse().unwrap()
}

fn fixture(package: &str) -> SearchPath {
    SearchPath::Source(manifest_dir().join("tests/fixtures").join(package))
}

/// A scratch folder that is removed afterwards.
struct Scratch(PathBuf);

impl Scratch {
    fn new(tag: &str) -> Self {
        let path = std::env::temp_dir().join(format!("rosidl-schema-{tag}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&path);
        fs::create_dir_all(&path).unwrap();
        Self(path)
    }

    fn path(&self, relative: &str) -> PathBuf {
        self.0.join(relative)
    }

    fn write(&self, relative: &str, text: &str) {
        let path = self.path(relative);
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        fs::write(path, text).unwrap();
    }

    #[cfg(unix)]
    fn link(&self, relative: &str, target: &Path) {
        let path = self.path(relative);
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::os::unix::fs::symlink(target, path).unwrap();
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

#[test]
fn a_source_tree_is_named_by_its_package_xml() {
    let registry = Registry::load(&[fixture("g1_msgs"), fixture("canopy_msgs")], &[]).unwrap();
    assert!(registry.issues().is_empty(), "{:?}", registry.issues());
    let Some(Interface::Action(pick)) = registry.get(&name("g1_msgs/action/Pick")) else {
        panic!("g1_msgs/action/Pick is missing");
    };
    assert_eq!(pick.goal.fields.len(), 2);
    assert!(matches!(
        registry.get(&name("canopy_msgs/srv/FindObjects")),
        Some(Interface::Service(_))
    ));
    assert!(matches!(
        registry.get(&name("canopy_msgs/msg/WorldObject")),
        Some(Interface::Message(_))
    ));
    assert!(registry.get(&name("g1_msgs/msg/Nope")).is_none());
    assert_eq!(registry.len(), 1 + 3 + 7 + 9 + 4);
    assert!(!registry.is_empty());
}

#[test]
fn the_package_name_comes_from_package_xml_not_from_the_folder() {
    let scratch = Scratch::new("named");
    scratch.write(
        "checkout-of-it/package.xml",
        "<package><name>real_name</name></package>",
    );
    scratch.write("checkout-of-it/msg/Thing.msg", "int32 a\n");
    let registry =
        Registry::load(&[SearchPath::Source(scratch.path("checkout-of-it"))], &[]).unwrap();
    assert!(registry.get(&name("real_name/msg/Thing")).is_some());
    assert_eq!(registry.types().count(), 1);
}

#[test]
fn an_install_prefix_is_read_from_its_share_folder() {
    let scratch = Scratch::new("prefix");
    scratch.write("share/foo_msgs/msg/Bar.msg", "int32 a\n");
    scratch.write(
        "share/foo_msgs/msg/Bar.idl",
        "// not an interface file we read\n",
    );
    scratch.write("share/foo_msgs/srv/Baz.srv", "int32 a\n---\nint32 b\n");
    scratch.write(
        "share/foo_msgs/action/Qux.action",
        "int32 g\n---\nint32 r\n---\nint32 f\n",
    );
    scratch.write("share/no_interfaces/README", "nothing here\n");
    let registry = Registry::load(&[SearchPath::Prefix(scratch.path(""))], &[]).unwrap();
    // Types order by package, then message before service before action.
    let types: Vec<_> = registry.types().map(ToString::to_string).collect();
    assert_eq!(
        types,
        [
            "foo_msgs/msg/Bar",
            "foo_msgs/srv/Baz",
            "foo_msgs/action/Qux"
        ]
    );
    assert!(registry.issues().is_empty());
}

#[test]
fn the_first_search_path_wins() {
    let scratch = Scratch::new("first");
    scratch.write("a/share/foo_msgs/msg/Bar.msg", "int32 from_a\n");
    scratch.write(
        "b/share/foo_msgs/msg/Bar.msg",
        "int32 from_b\nint32 extra\n",
    );
    scratch.write("b/share/foo_msgs/msg/Only.msg", "int32 x\n");
    let a = SearchPath::Prefix(scratch.path("a"));
    let b = SearchPath::Prefix(scratch.path("b"));
    let field_count = |registry: &Registry| match registry.get(&name("foo_msgs/msg/Bar")) {
        Some(Interface::Message(m)) => m.fields.len(),
        other => panic!("{other:?}"),
    };
    let ab = Registry::load(&[a.clone(), b.clone()], &[]).unwrap();
    assert_eq!(field_count(&ab), 1);
    assert!(ab.get(&name("foo_msgs/msg/Only")).is_some());
    assert_eq!(field_count(&Registry::load(&[b, a], &[]).unwrap()), 2);
}

#[cfg(unix)]
#[test]
fn symlinks_are_followed_and_dangling_ones_are_remapped() {
    let scratch = Scratch::new("remap");
    // What a container build leaves: links into a path that only exists in the container.
    let container = Path::new("/nonexistent-container-root/ws/src/foo_msgs");
    scratch.write("host/ws/src/foo_msgs/msg/Bar.msg", "int32 a\n");
    scratch.write(
        "host/ws/src/foo_msgs/srv/Baz.srv",
        "int32 a\n---\nint32 b\n",
    );
    scratch.link(
        "prefix/share/foo_msgs/msg/Bar.msg",
        &container.join("msg/Bar.msg"),
    );
    scratch.link("prefix/share/foo_msgs/srv", &container.join("srv"));
    // A link that works as it is, and a relative one.
    scratch.link(
        "prefix/share/valid_pkg/msg/Direct.msg",
        &scratch.path("host/ws/src/foo_msgs/msg/Bar.msg"),
    );
    scratch.link(
        "prefix/share/relative_pkg/msg/Rel.msg",
        Path::new("../../../../host/ws/src/foo_msgs/msg/Bar.msg"),
    );
    let prefix = SearchPath::Prefix(scratch.path("prefix"));

    let plain = Registry::load(std::slice::from_ref(&prefix), &[]).unwrap();
    let types: Vec<_> = plain.types().map(ToString::to_string).collect();
    assert_eq!(
        types,
        ["relative_pkg/msg/Rel", "valid_pkg/msg/Direct"],
        "the dangling ones are skipped"
    );
    let dangling: Vec<_> = plain
        .issues()
        .iter()
        .filter(|i| matches!(i.error, Error::DanglingLink { .. }))
        .collect();
    assert_eq!(dangling.len(), 2, "{:?}", plain.issues());
    assert!(
        dangling[0].error.to_string().contains("remap"),
        "{}",
        dangling[0].error
    );

    let remap = [(
        PathBuf::from("/nonexistent-container-root/ws"),
        scratch.path("host/ws"),
    )];
    let mapped = Registry::load(&[prefix], &remap).unwrap();
    assert!(mapped.issues().is_empty(), "{:?}", mapped.issues());
    let types: Vec<_> = mapped.types().map(ToString::to_string).collect();
    assert_eq!(
        types,
        [
            "foo_msgs/msg/Bar",
            "foo_msgs/srv/Baz",
            "relative_pkg/msg/Rel",
            "valid_pkg/msg/Direct"
        ]
    );
}

#[cfg(unix)]
#[test]
fn a_dangling_link_that_a_remap_cannot_fix_stays_an_issue() {
    let scratch = Scratch::new("stillbad");
    scratch.link(
        "share/foo_msgs/msg/Bar.msg",
        Path::new("/nonexistent-container-root/Bar.msg"),
    );
    let remap = [(
        PathBuf::from("/nonexistent-container-root"),
        scratch.path("also-missing"),
    )];
    let registry = Registry::load(&[SearchPath::Prefix(scratch.path(""))], &remap).unwrap();
    assert!(registry.is_empty());
    assert_eq!(registry.issues().len(), 1);
}

#[test]
fn one_broken_file_does_not_stop_the_rest() {
    let scratch = Scratch::new("broken");
    scratch.write("share/foo_msgs/msg/Good.msg", "int32 a\n");
    scratch.write("share/foo_msgs/msg/Bad.msg", "int32 a\nfloat64\n");
    scratch.write("share/foo_msgs/msg/lower.msg", "int32 a\n");
    scratch.write("share/foo_msgs/msg/Utf16.msg", "\u{0}\n");
    fs::write(
        scratch.path("share/foo_msgs/msg/NotUtf8.msg"),
        [0xff, 0xfe, b'\n'],
    )
    .unwrap();
    let registry = Registry::load(&[SearchPath::Prefix(scratch.path(""))], &[]).unwrap();
    assert_eq!(
        registry
            .types()
            .map(ToString::to_string)
            .collect::<Vec<_>>(),
        ["foo_msgs/msg/Good"]
    );
    assert_eq!(registry.issues().len(), 4, "{:?}", registry.issues());
    let bad = registry
        .issues()
        .iter()
        .find(|i| i.path.ends_with("Bad.msg"))
        .unwrap();
    let text = bad.error.to_string();
    assert!(
        text.ends_with("Bad.msg:2: expected `TYPE NAME`, found `float64`"),
        "{text}"
    );
    assert!(
        text.starts_with(scratch.path("").to_str().unwrap()),
        "the error names the file: {text}"
    );
}

#[test]
fn a_search_path_that_does_not_work_is_an_error() {
    let scratch = Scratch::new("nopath");
    let gone = scratch.path("gone");
    assert!(matches!(
        Registry::load(&[SearchPath::Prefix(gone.clone())], &[]),
        Err(Error::Io { .. })
    ));
    assert!(matches!(
        Registry::load(&[SearchPath::Source(gone)], &[]),
        Err(Error::Io { .. })
    ));
    scratch.write("plain/msg/Thing.msg", "int32 a\n");
    let err = Registry::load(&[SearchPath::Source(scratch.path("plain"))], &[]).unwrap_err();
    assert!(matches!(err, Error::NotAPackage(_)), "{err}");
    scratch.write("nameless/package.xml", "<package></package>");
    assert!(matches!(
        Registry::load(&[SearchPath::Source(scratch.path("nameless"))], &[]),
        Err(Error::NotAPackage(_))
    ));
    // A prefix without a share folder is simply empty.
    fs::create_dir_all(scratch.path("bare")).unwrap();
    assert!(
        Registry::load(&[SearchPath::Prefix(scratch.path("bare"))], &[])
            .unwrap()
            .is_empty()
    );
}

#[test]
fn resolve_follows_nested_types_across_packages() {
    let search: Vec<_> = [
        "nav2_msgs",
        "geometry_msgs",
        "std_msgs",
        "builtin_interfaces",
        "nav_msgs",
    ]
    .map(fixture)
    .into();
    let registry = Registry::load(&search, &[]).unwrap();
    let all = registry
        .resolve(&name("nav2_msgs/action/NavigateToPose"))
        .unwrap();
    let names: Vec<_> = all.iter().map(ToString::to_string).collect();
    assert_eq!(names[0], "nav2_msgs/action/NavigateToPose");
    for expected in [
        "geometry_msgs/msg/PoseStamped",
        "geometry_msgs/msg/Pose",
        "builtin_interfaces/msg/Duration",
        "std_msgs/msg/Header",
    ] {
        assert!(
            names.iter().any(|n| n == expected),
            "{expected} is missing from {names:?}"
        );
    }
    let unique: std::collections::BTreeSet<_> = names.iter().collect();
    assert_eq!(unique.len(), names.len(), "each type once");
}

#[test]
fn resolve_names_the_missing_type_and_who_needs_it() {
    // nav2_msgs alone lacks geometry_msgs and the rest.
    let registry = Registry::load(&[fixture("nav2_msgs")], &[]).unwrap();
    let err = registry
        .resolve(&name("nav2_msgs/action/NavigateToPose"))
        .unwrap_err();
    let Error::UnknownType {
        name: missing,
        used_by,
    } = &err
    else {
        panic!("{err}");
    };
    assert_eq!(missing.package, "geometry_msgs");
    assert_eq!(
        used_by.as_ref().unwrap().to_string(),
        "nav2_msgs/action/NavigateToPose"
    );
    assert!(
        err.to_string().contains("no search path provides it"),
        "{err}"
    );
    assert!(matches!(
        registry.resolve(&name("nav2_msgs/msg/DoesNotExist")),
        Err(Error::UnknownType { used_by: None, .. })
    ));
}

/// The install tree that a build in the grove-g1 container leaves on the host: every link points
/// into `/root/workspace`, which is `workspace/` in the repository.
#[cfg(unix)]
#[test]
#[ignore = "needs the grove-g1 workspace install; set GROVE_G1_WORKSPACE if it is not at ~/grove-g1/workspace"]
fn the_grove_g1_install_tree_loads_through_a_remap() {
    let workspace = std::env::var_os("GROVE_G1_WORKSPACE").map_or_else(
        || PathBuf::from(std::env::var_os("HOME").unwrap()).join("grove-g1/workspace"),
        PathBuf::from,
    );
    let search = [
        SearchPath::Prefix(workspace.join("install/g1_msgs")),
        SearchPath::Prefix(workspace.join("install/canopy_msgs")),
    ];
    let remap = [(PathBuf::from("/root/workspace"), workspace.clone())];

    let without = Registry::load(&search, &[]).unwrap();
    assert!(
        without.is_empty(),
        "the container's links dangle on the host"
    );
    assert!(!without.issues().is_empty());

    let registry = Registry::load(&search, &remap).unwrap();
    assert!(registry.issues().is_empty(), "{:?}", registry.issues());
    for wanted in [
        "g1_msgs/action/Pick",
        "g1_msgs/srv/GenerateGrasps",
        "canopy_msgs/srv/FindObjects",
    ] {
        assert!(registry.get(&name(wanted)).is_some(), "{wanted}");
    }
    println!("{} interfaces from the install tree", registry.len());
}
