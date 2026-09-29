//! Parses ROS 2 `.msg`, `.srv` and `.action` files with their comments and turns them into
//! JSON Schema.
//!
//! A model that calls a ROS service needs to know what each field of the request means, and the
//! interface file says it in the comments. ROS 2's own parser (`rosidl_adapter`) drops them after
//! generating code; this crate keeps them, and needs neither ROS nor a build.
//!
//! - [`parse_message`], [`parse_service`] and [`parse_action`] read the text of one file.
//! - [`Registry`] finds the files of installed packages and source trees, and follows the
//!   symlinks of an install tree that was built in a container.
//! - [`json_schema`] turns one part of an interface into a JSON Schema, and [`flatten_refs`]
//!   inlines its `$ref`s for providers that do not accept them.
//!
//! # Example
//!
//! ```
//! use rosidl_schema::{Overrides, Part, Registry, TypeName, json_schema};
//!
//! let pick: TypeName = "g1_msgs/action/Pick".parse()?;
//! let text = [
//!     "# Pick a known object off a surface.",
//!     "",
//!     "# A class_id on /objects.",
//!     "string object_id",
//!     "string ARM_LEFT=left",
//!     "string ARM_RIGHT=right",
//!     "string arm",
//!     "---",
//!     "bool success",
//!     "---",
//!     "string phase",
//! ]
//! .join("\n");
//! let mut registry = Registry::default();
//! registry.add_source(&pick, &text)?;
//!
//! let schema = json_schema(&registry, &pick, Part::Goal, &Overrides::default())?;
//! assert_eq!(schema["properties"]["object_id"]["description"], "A class_id on /objects.");
//! assert_eq!(schema["properties"]["arm"]["enum"], serde_json::json!(["left", "right"]));
//! # Ok::<(), rosidl_schema::Error>(())
//! ```
//!
//! On a real system the registry is loaded from disk. Install prefixes are read from
//! `<prefix>/share/<package>`, source packages from their `msg`, `srv` and `action` folders, and
//! the remap turns the container paths in an install tree's symlinks into host paths:
//!
//! ```no_run
//! use rosidl_schema::{Registry, SearchPath};
//!
//! let registry = Registry::load(
//!     &[
//!         SearchPath::Prefix("/opt/ros/jazzy".into()),
//!         SearchPath::Source("workspace/src/g1_msgs".into()),
//!     ],
//!     &[("/root/workspace".into(), "/home/me/grove-g1/workspace".into())],
//! )?;
//! for issue in registry.issues() {
//!     eprintln!("skipped {}: {}", issue.path.display(), issue.error);
//! }
//! # Ok::<(), rosidl_schema::Error>(())
//! ```
//!
//! # How comments are read
//!
//! The rules are those of `rosidl_adapter/parser.py`, which the tests check against the real
//! thing on every interface of a ROS 2 install.
//!
//! - A comment block above a field or constant belongs to it, and so does a comment at the end of
//!   its line. An indented comment-only line continues the element above.
//! - The comment lines at the top of a file, up to the first blank or definition line, are the
//!   message's own doc. A service or action has one for each of its parts.
//! - A single `[unit]` in a comment, without a comma, becomes the field's unit and leaves the text.
//! - `TYPE NAME DEFAULT` sets a default and `TYPE NAME=VALUE` declares a constant. Both are kept
//!   as written and read as typed values on demand ([`Field::default_json`] and
//!   [`Constant::json_value`]).
//! - `time` and `duration` are the `builtin_interfaces` messages.
//!
//! A comment above a group of constants belongs to the first constant, not to the field they
//! describe. Such a field has no description of its own; write the comment after the constants,
//! directly above the field.
//!
//! # Schema mapping
//!
//! Integers carry the range of their type (`char` is an unsigned byte, as in ROS 2), floats are
//! numbers, strings are strings with `maxLength` when bounded, fixed arrays have `minItems` and
//! `maxItems`, bounded ones `maxItems`. Nested messages become `$defs` and every object is closed
//! with `additionalProperties: false`. Nothing is `required`, because every ROS field has a zero
//! value. A field's description is its comment plus its unit, and constants are listed as
//! `NAME=value` lines in the description of their message. Constants named `X_*` give the field
//! `x` of the same type an `enum`. `uint8[]` and `byte[]` fields, bounded or not, are payloads
//! and are left out.
//! [`Overrides`] adjusts all of this per field.

mod error;
mod parse;
mod registry;
mod schema;
mod types;
mod value;

pub use error::{Error, ParseError, ParseErrorKind, Result};
pub use parse::{parse_action, parse_interface, parse_message, parse_service};
pub use registry::{LoadIssue, Registry, SearchPath};
pub use schema::{FieldOverride, Overrides, flatten_refs, json_schema};
pub use types::{
    Action, Array, Constant, Field, FieldType, Interface, Kind, Message, Part, Service, TypeName,
};
