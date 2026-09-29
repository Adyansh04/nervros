//! Finds interface files on disk and keeps them parsed.

use std::{
    collections::{BTreeMap, BTreeSet},
    fs, io,
    path::{Path, PathBuf},
};

use crate::{
    error::{Error, Result},
    parse::parse_interface,
    types::{FieldType, Interface, Kind, Message, TypeName},
};

/// The most symlinks followed by hand for one path, which also stops a symlink loop.
const MAX_LINK_HOPS: usize = 40;

/// A place to look for interface files.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SearchPath {
    /// An install prefix such as `/opt/ros/jazzy`, or one `install/<package>` folder of a colcon
    /// workspace. Reads `<prefix>/share/<package>/{msg,srv,action}/*`.
    Prefix(PathBuf),
    /// A package's source folder. Its `package.xml` names the package, and its `msg`, `srv` and
    /// `action` folders hold the files.
    Source(PathBuf),
}

/// A file that was skipped while loading, and why. One bad file does not stop the others.
#[derive(Debug)]
pub struct LoadIssue {
    /// The file, or the folder that could not be read.
    pub path: PathBuf,
    /// What went wrong with it.
    pub error: Error,
}

/// Every interface found in the search paths, parsed.
#[derive(Debug, Default)]
pub struct Registry {
    interfaces: BTreeMap<TypeName, Interface>,
    issues: Vec<LoadIssue>,
}

impl Registry {
    /// Reads every interface in `search`, in order. When two paths hold the same type, the first
    /// one wins.
    ///
    /// A symlink whose target is missing, such as an install tree made in a container, is followed
    /// through `remap`: each `(from, to)` replaces the prefix `from` of the target with `to`, and
    /// the first pair that matches is used.
    ///
    /// Files that cannot be read or parsed are skipped and listed in [`Registry::issues`].
    ///
    /// # Errors
    /// Fails when a search path itself is unusable: it does not exist, or a source path has no
    /// `package.xml` with a name.
    pub fn load(search: &[SearchPath], remap: &[(PathBuf, PathBuf)]) -> Result<Self> {
        let mut registry = Self::default();
        for path in search {
            match path {
                SearchPath::Prefix(prefix) => registry.scan_prefix(prefix, remap)?,
                SearchPath::Source(dir) => registry.scan_source(dir, remap)?,
            }
        }
        Ok(registry)
    }

    /// The parsed interface, if the registry holds it.
    #[must_use]
    pub fn get(&self, name: &TypeName) -> Option<&Interface> {
        self.interfaces.get(name)
    }

    /// Parses `text` as the file of `name` and adds it, replacing an earlier entry.
    ///
    /// # Errors
    /// Returns the parse error.
    pub fn add_source(&mut self, name: &TypeName, text: &str) -> Result<()> {
        self.interfaces
            .insert(name.clone(), parse_interface(name, text)?);
        Ok(())
    }

    /// Every type in the registry, in name order.
    pub fn types(&self) -> impl Iterator<Item = &TypeName> {
        self.interfaces.keys()
    }

    /// How many interfaces the registry holds.
    #[must_use]
    pub fn len(&self) -> usize {
        self.interfaces.len()
    }

    /// Whether the registry holds nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.interfaces.is_empty()
    }

    /// The files [`Registry::load`] skipped.
    #[must_use]
    pub fn issues(&self) -> &[LoadIssue] {
        &self.issues
    }

    /// `name` and every message reachable from it through fields, each once, in depth-first order.
    ///
    /// A `Header` that its own package does not define counts as `std_msgs/Header`, the ROS 1
    /// habit of writing `Header` alone.
    ///
    /// # Errors
    /// [`Error::UnknownType`] for the first type the registry lacks, naming the type that needs it.
    pub fn resolve(&self, name: &TypeName) -> Result<Vec<TypeName>> {
        let mut order = Vec::new();
        self.walk(name, None, &mut order, &mut BTreeSet::new())?;
        Ok(order)
    }

    fn walk(
        &self,
        name: &TypeName,
        used_by: Option<&TypeName>,
        order: &mut Vec<TypeName>,
        seen: &mut BTreeSet<TypeName>,
    ) -> Result<()> {
        let (found, interface) = self.find(name).ok_or_else(|| Error::UnknownType {
            name: name.clone(),
            used_by: used_by.cloned(),
        })?;
        if !seen.insert(found.clone()) {
            return Ok(());
        }
        order.push(found.clone());
        for message in interface.messages() {
            for field in &message.fields {
                if let FieldType::Nested(nested) = &field.ty {
                    self.walk(nested, Some(found), order, seen)?;
                }
            }
        }
        Ok(())
    }

    /// Looks a type up, with the ROS 1 habit of writing `Header` for `std_msgs/Header`.
    fn find(&self, name: &TypeName) -> Option<(&TypeName, &Interface)> {
        self.interfaces.get_key_value(name).or_else(|| {
            if name.kind != Kind::Msg || name.name != "Header" {
                return None;
            }
            let std_header = TypeName {
                package: "std_msgs".to_owned(),
                kind: Kind::Msg,
                name: "Header".to_owned(),
            };
            self.interfaces.get_key_value(&std_header)
        })
    }

    /// As [`Registry::find`], for a message only.
    pub(crate) fn find_message(&self, name: &TypeName) -> Option<(&TypeName, &Message)> {
        match self.find(name)? {
            (found, Interface::Message(message)) => Some((found, message)),
            _ => None,
        }
    }

    /// Resolves `path`. A symlink that leads nowhere is recorded as an issue, a missing path not.
    fn resolve_or_report(&mut self, path: &Path, remap: &[(PathBuf, PathBuf)]) -> Option<PathBuf> {
        let resolved = resolve_path(path, remap);
        if resolved.is_none() && fs::read_link(path).is_ok() {
            self.issues.push(LoadIssue {
                path: path.to_owned(),
                error: unreachable_path(path),
            });
        }
        resolved
    }

    fn scan_prefix(&mut self, prefix: &Path, remap: &[(PathBuf, PathBuf)]) -> Result<()> {
        let root = resolve_path(prefix, remap).ok_or_else(|| unreachable_path(prefix))?;
        let Some(share) = resolve_path(&root.join("share"), remap) else {
            return Ok(());
        };
        for (package, path) in sorted_entries(&share)? {
            if let Some(dir) = self.resolve_or_report(&path, remap).filter(|d| d.is_dir()) {
                self.scan_package(&package, &dir, remap);
            }
        }
        Ok(())
    }

    fn scan_source(&mut self, dir: &Path, remap: &[(PathBuf, PathBuf)]) -> Result<()> {
        let dir = resolve_path(dir, remap).ok_or_else(|| unreachable_path(dir))?;
        let package = fs::read_to_string(dir.join("package.xml"))
            .ok()
            .and_then(|xml| package_name(&xml))
            .ok_or_else(|| Error::NotAPackage(dir.clone()))?;
        self.scan_package(&package, &dir, remap);
        Ok(())
    }

    fn scan_package(&mut self, package: &str, dir: &Path, remap: &[(PathBuf, PathBuf)]) {
        for kind in [Kind::Msg, Kind::Srv, Kind::Action] {
            let folder = self.resolve_or_report(&dir.join(kind.as_str()), remap);
            let Some(folder) = folder.filter(|d| d.is_dir()) else {
                continue;
            };
            let entries = match sorted_entries(&folder) {
                Ok(entries) => entries,
                Err(error) => {
                    self.issues.push(LoadIssue {
                        path: folder,
                        error,
                    });
                    continue;
                }
            };
            for (file_name, path) in entries {
                let Some(stem) = file_name.strip_suffix(&format!(".{kind}")) else {
                    continue;
                };
                if let Err(error) = self.load_file(package, kind, stem, &path, remap) {
                    self.issues.push(LoadIssue { path, error });
                }
            }
        }
    }

    fn load_file(
        &mut self,
        package: &str,
        kind: Kind,
        stem: &str,
        path: &Path,
        remap: &[(PathBuf, PathBuf)],
    ) -> Result<()> {
        let name: TypeName = format!("{package}/{kind}/{stem}").parse()?;
        if self.interfaces.contains_key(&name) {
            return Ok(());
        }
        let file = resolve_path(path, remap).ok_or_else(|| unreachable_path(path))?;
        let text = fs::read_to_string(&file).map_err(|source| Error::Io {
            path: file.clone(),
            source,
        })?;
        let interface = parse_interface(&name, &text).map_err(|e| e.in_file(&file))?;
        self.interfaces.insert(name, interface);
        Ok(())
    }
}

/// Why `resolve_path` found nothing: a dangling symlink, or no such file.
fn unreachable_path(path: &Path) -> Error {
    match fs::read_link(path) {
        Ok(target) => Error::DanglingLink {
            link: path.to_owned(),
            target,
        },
        Err(_) => Error::Io {
            path: path.to_owned(),
            source: io::ErrorKind::NotFound.into(),
        },
    }
}

/// The folder's entries by name, so that loading does not depend on the file system's order.
fn sorted_entries(dir: &Path) -> Result<Vec<(String, PathBuf)>> {
    let io_error = |source| Error::Io {
        path: dir.to_owned(),
        source,
    };
    let mut entries = fs::read_dir(dir)
        .map_err(io_error)?
        .map(|e| e.map(|e| (e.file_name().to_string_lossy().into_owned(), e.path())))
        .collect::<io::Result<Vec<_>>>()
        .map_err(io_error)?;
    entries.sort();
    Ok(entries)
}

/// The path itself when it exists; when it is a symlink to a missing target, the first hop
/// through `remap` that leads somewhere real.
fn resolve_path(path: &Path, remap: &[(PathBuf, PathBuf)]) -> Option<PathBuf> {
    let mut current = path.to_owned();
    for _ in 0..MAX_LINK_HOPS {
        if current.exists() {
            return Some(current);
        }
        // `exists` follows links, so a dangling one lands here: take a single hop by hand.
        let target = fs::read_link(&current).ok()?;
        let target = if target.is_absolute() {
            target
        } else {
            current.parent()?.join(target)
        };
        current = remapped(&target, remap).unwrap_or(target);
    }
    None
}

/// `target` with the first matching `from` prefix replaced by its `to`.
fn remapped(target: &Path, remap: &[(PathBuf, PathBuf)]) -> Option<PathBuf> {
    remap
        .iter()
        .find_map(|(from, to)| target.strip_prefix(from).ok().map(|rest| to.join(rest)))
}

/// The text of the first `<name>` element outside comments.
fn package_name(xml: &str) -> Option<String> {
    let mut clean = String::with_capacity(xml.len());
    let mut rest = xml;
    while let Some(start) = rest.find("<!--") {
        clean.push_str(&rest[..start]);
        rest = rest[start..]
            .find("-->")
            .map_or("", |end| &rest[start + end + 3..]);
    }
    clean.push_str(rest);
    let start = clean.find("<name>")? + "<name>".len();
    let end = start + clean[start..].find("</name>")?;
    let name = clean[start..end].trim();
    (!name.is_empty()).then(|| name.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_package_name_is_the_first_name_element_outside_comments() {
        let xml = "<?xml version=\"1.0\"?>\n<!-- <name>decoy</name> -->\n<package format=\"3\">\n  <name> g1_msgs </name>\n  <maintainer>x</maintainer>\n</package>";
        assert_eq!(package_name(xml).as_deref(), Some("g1_msgs"));
        assert_eq!(package_name("<package></package>"), None);
        assert_eq!(package_name("<package><name></name></package>"), None);
        assert_eq!(package_name("<!-- never closed <name>x</name>"), None);
    }

    #[test]
    fn remapping_replaces_whole_path_components_only_and_the_first_match_wins() {
        let remap = [
            (PathBuf::from("/root/ws"), PathBuf::from("/host/ws")),
            (PathBuf::from("/root"), PathBuf::from("/elsewhere")),
        ];
        assert_eq!(
            remapped(Path::new("/root/ws/src/a"), &remap),
            Some(PathBuf::from("/host/ws/src/a"))
        );
        assert_eq!(
            remapped(Path::new("/root/ws2/a"), &remap),
            Some(PathBuf::from("/elsewhere/ws2/a"))
        );
        assert_eq!(remapped(Path::new("/other/a"), &remap), None);
        assert_eq!(
            remapped(Path::new("/root/ws"), &remap),
            Some(PathBuf::from("/host/ws"))
        );
    }
}
