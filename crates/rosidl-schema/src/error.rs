//! Errors of the parser, the registry and the schema generator.

use std::path::{Path, PathBuf};

use crate::types::{Part, TypeName};

/// The result type of every fallible function in this crate.
pub type Result<T, E = Error> = std::result::Result<T, E>;

/// A problem found in one interface file, with the place it was found.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{origin}:{line}: {kind}")]
pub struct ParseError {
    /// The file path, or the type name when the text did not come from a file.
    pub origin: String,
    /// One-based line number in the file (in the whole `.srv` or `.action` file).
    pub line: usize,
    /// What is wrong with the line.
    pub kind: ParseErrorKind,
}

/// What is wrong with a line of an interface file.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum ParseErrorKind {
    /// A line that is neither `TYPE NAME [DEFAULT]` nor `TYPE NAME=VALUE`.
    #[error("expected `TYPE NAME`, found `{0}`")]
    Definition(String),
    /// A type that is not a primitive, an array of one, or a `package/Name` reference.
    #[error("invalid type `{ty}`: {reason}")]
    Type {
        /// The type as written.
        ty: String,
        /// Why it was rejected.
        reason: String,
    },
    /// A field, constant, package or message name outside the ROS naming patterns.
    #[error("invalid {what} name `{name}`")]
    Name {
        /// `field`, `constant`, `package` or `message`.
        what: &'static str,
        /// The name as written.
        name: String,
    },
    /// A default or constant value that does not fit its type.
    #[error("value `{value}` cannot be converted to `{ty}`: {reason}")]
    Value {
        /// The type as written.
        ty: String,
        /// The value as written.
        value: String,
        /// Why it was rejected.
        reason: String,
    },
    /// The same field or constant name twice in one message.
    #[error("duplicate {what} `{name}`")]
    Duplicate {
        /// `field` or `constant`.
        what: &'static str,
        /// The repeated name.
        name: String,
    },
    /// A `.srv` or `.action` file with the wrong number of `---` lines.
    #[error("expected {expected} `---` separator line(s), found {found}")]
    Separators {
        /// One for a service, two for an action.
        expected: usize,
        /// How many the file has.
        found: usize,
    },
}

/// Everything that can go wrong in this crate.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// An interface file does not parse.
    #[error(transparent)]
    Parse(#[from] ParseError),
    /// A type string that is not `pkg/msg/Name`, `pkg/srv/Name`, `pkg/action/Name` or `pkg/Name`.
    #[error("invalid type name `{input}`: {reason}")]
    TypeName {
        /// The string as given.
        input: String,
        /// What is wrong with it.
        reason: String,
    },
    /// A file or directory could not be read.
    #[error("cannot read {}: {source}", path.display())]
    Io {
        /// The path that failed.
        path: PathBuf,
        /// The operating system error.
        #[source]
        source: std::io::Error,
    },
    /// A symlink whose target does not exist and that no remap rule reaches.
    #[error(
        "symlink {} points to {}, which does not exist; a remap rule could map it",
        link.display(),
        target.display()
    )]
    DanglingLink {
        /// The symlink.
        link: PathBuf,
        /// Where it points.
        target: PathBuf,
    },
    /// A source search path without a `package.xml` that names the package.
    #[error("{} is not a ROS package: no package.xml with a <name>", .0.display())]
    NotAPackage(PathBuf),
    /// A type that no search path provides.
    #[error(
        "unknown type `{name}`: no search path provides it{}",
        .used_by.as_ref().map(|t| format!(" (needed by `{t}`)")).unwrap_or_default()
    )]
    UnknownType {
        /// The missing type.
        name: TypeName,
        /// The type whose field refers to it, when it was found through one.
        used_by: Option<TypeName>,
    },
    /// A schema part that the interface does not have, such as the goal of a message.
    #[error("`{ty}` has no {part} part")]
    NoSuchPart {
        /// The interface.
        ty: TypeName,
        /// The part that was asked for.
        part: Part,
    },
    /// An override for a field path that the schema does not contain.
    #[error("override for `{0}` matches no field in the schema")]
    UnknownOverride(String),
    /// A message that contains itself, directly or through other messages.
    #[error("message `{0}` contains itself")]
    Recursive(TypeName),
    /// A `$ref` that is not `#/$defs/<name>` or names a missing definition.
    #[error("cannot resolve $ref `{0}`")]
    BadRef(String),
    /// `$ref`s nested deeper than [`flatten_refs`](crate::flatten_refs) follows.
    #[error("$ref nesting is deeper than {0}")]
    RefDepth(usize),
}

impl Error {
    /// Names the file a parse error came from, for errors raised on text read from disk.
    #[must_use]
    pub fn in_file(mut self, path: &Path) -> Self {
        if let Self::Parse(e) = &mut self {
            e.origin = path.display().to_string();
        }
        self
    }
}
