//! Tool argument schemas from the robot's interface files, through `rosidl-schema`.

use std::path::PathBuf;

use rosidl_schema::{FieldOverride, Interface, Overrides, Part, Registry, SearchPath, TypeName};
use serde_json::Value;

use crate::profile::Profile;
use crate::tools::{SchemaPart, SchemaSource};

/// Interface definitions loaded from the profile's `ros.interfaces` paths.
#[derive(Debug)]
pub struct RosidlSchemas {
    registry: Registry,
}

impl RosidlSchemas {
    /// Loads every path: a directory with a `package.xml` is a package source, anything else an
    /// install prefix. Unreadable files are logged and skipped.
    ///
    /// # Errors
    ///
    /// A search path that cannot be read at all.
    pub fn load(profile: &Profile) -> Result<Self, rosidl_schema::Error> {
        let search: Vec<SearchPath> = profile
            .ros
            .interfaces
            .iter()
            .map(|p| {
                let p = profile.resolve(p);
                if p.join("package.xml").is_file() {
                    SearchPath::Source(p)
                } else {
                    SearchPath::Prefix(p)
                }
            })
            .collect();
        let remap: Vec<(PathBuf, PathBuf)> = profile
            .ros
            .remap
            .iter()
            .map(|(a, b)| (a.clone(), b.clone()))
            .collect();
        let registry = Registry::load(&search, &remap)?;
        for issue in registry.issues() {
            tracing::warn!(?issue, "skipped an interface file");
        }
        Ok(Self { registry })
    }

    /// Number of interfaces loaded.
    #[must_use]
    pub fn len(&self) -> usize {
        self.registry.len()
    }

    /// Whether none were loaded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.registry.is_empty()
    }
}

fn rosidl_part(part: SchemaPart) -> Part {
    match part {
        SchemaPart::Request => Part::Request,
        SchemaPart::Response => Part::Response,
        SchemaPart::Message => Part::Message,
        SchemaPart::Goal => Part::Goal,
        SchemaPart::Result => Part::Result,
        SchemaPart::Feedback => Part::Feedback,
    }
}

impl SchemaSource for RosidlSchemas {
    fn schema(&self, ros_type: &str, part: SchemaPart, hide: &[String]) -> Result<Value, String> {
        let ty: TypeName = ros_type
            .parse()
            .map_err(|e| format!("bad type `{ros_type}`: {e}"))?;
        let part = rosidl_part(part);
        let mut overrides = Overrides::default();
        for field in hide {
            overrides.fields.insert(
                field.clone(),
                FieldOverride {
                    hidden: Some(true),
                    ..FieldOverride::default()
                },
            );
        }
        let schema = rosidl_schema::json_schema(&self.registry, &ty, part, &overrides)
            .map_err(|e| e.to_string())?;
        // Inlined so providers that ignore `$ref` see every field.
        rosidl_schema::flatten_refs(&schema).map_err(|e| e.to_string())
    }

    fn validate(&self, ros_type: &str, part: SchemaPart, value: &Value) -> Result<(), String> {
        let ty: TypeName = ros_type
            .parse()
            .map_err(|e| format!("bad type `{ros_type}`: {e}"))?;
        rosidl_schema::validate(&self.registry, &ty, rosidl_part(part), value)
    }

    fn show(&self, ros_type: &str) -> Option<String> {
        let ty: TypeName = ros_type.parse().ok()?;
        Some(match self.registry.get(&ty)? {
            Interface::Message(m) => m.to_string(),
            Interface::Service(s) => format!("{}---\n{}", s.request, s.response),
            Interface::Action(a) => format!("{}---\n{}---\n{}", a.goal, a.result, a.feedback),
        })
    }
}
