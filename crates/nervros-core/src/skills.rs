//! Agent skills: procedures for what the agent meets rarely and must get right, such as Nav2 that
//! never came up, kept as `SKILL.md` files in folders the profile names. The system prompt
//! carries one line per skill, its name and when to use it; the `skill` tool reads one in full
//! when it fits. Each file starts with a front matter of `name` and `description`, as Agent
//! Skills write it.

use std::borrow::Cow;
use std::path::{Path, PathBuf};

use async_trait::async_trait;
use serde_json::{Value, json};

use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// A skill's text past this many characters is cut: a procedure, not a manual.
const BODY_CHARS: usize = 6000;
/// A description past this many characters is cut in the prompt's index.
const DESCRIPTION_CHARS: usize = 200;

/// One skill.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Skill {
    /// As the `skill` tool takes it.
    pub name: String,
    /// When to use it, for the index.
    pub description: String,
    /// The procedure.
    pub body: String,
}

/// The skills in `dirs`, each a folder holding a `SKILL.md`, by name; and what could not be read.
#[must_use]
pub fn load(dirs: &[PathBuf]) -> (Vec<Skill>, Vec<String>) {
    let mut skills: Vec<Skill> = Vec::new();
    let mut problems = Vec::new();
    for dir in dirs {
        let Ok(entries) = std::fs::read_dir(dir) else {
            problems.push(format!("no skills folder {}", dir.display()));
            continue;
        };
        let mut files: Vec<PathBuf> = entries
            .filter_map(Result::ok)
            .map(|e| e.path().join("SKILL.md"))
            .filter(|p| p.is_file())
            .collect();
        files.sort();
        for file in files {
            match read(&file) {
                Ok(skill) if skills.iter().any(|s| s.name == skill.name) => {
                    problems.push(format!(
                        "{}: a second skill named {}",
                        file.display(),
                        skill.name
                    ));
                }
                Ok(skill) => skills.push(skill),
                Err(e) => problems.push(format!("{}: {e}", file.display())),
            }
        }
    }
    (skills, problems)
}

fn read(file: &Path) -> Result<Skill, String> {
    let text = std::fs::read_to_string(file).map_err(|e| e.to_string())?;
    parse(&text)
}

/// A `SKILL.md`: its front matter's `name` and `description`, then the procedure.
fn parse(text: &str) -> Result<Skill, String> {
    let rest = text
        .strip_prefix("---")
        .ok_or("it starts without its front matter (---)")?;
    let (front, body) = rest
        .split_once("\n---")
        .ok_or("its front matter does not end (---)")?;
    let field = |key: &str| {
        front.lines().find_map(|l| {
            let (k, v) = l.split_once(':')?;
            (k.trim() == key).then(|| v.trim().trim_matches('"').to_owned())
        })
    };
    let name = field("name")
        .filter(|n| !n.is_empty() && n.chars().all(|c| c.is_ascii_alphanumeric() || c == '-'))
        .ok_or("its name is missing, or not lower-case words joined by -")?;
    let description = field("description")
        .filter(|d| !d.is_empty())
        .ok_or("it says nothing of when to use it (description)")?;
    Ok(Skill {
        name,
        description,
        body: body.trim().to_owned(),
    })
}

/// The index the system prompt carries: a line per skill.
#[must_use]
pub fn index(skills: &[Skill]) -> Option<String> {
    if skills.is_empty() {
        return None;
    }
    let lines: Vec<String> = skills
        .iter()
        .map(|s| {
            format!(
                "- {}: {}",
                s.name,
                crate::tools::clip(&s.description, DESCRIPTION_CHARS)
            )
        })
        .collect();
    Some(format!(
        "Skills: procedures for rare situations. When one fits, read it with `skill` first and \
         follow it.\n{}",
        lines.join("\n")
    ))
}

/// The `skill` tool.
pub struct SkillTool {
    spec: ToolSpec,
    skills: Vec<Skill>,
}

impl SkillTool {
    /// Over `skills`, which must not be empty to be of use.
    #[must_use]
    pub fn new(skills: Vec<Skill>) -> Self {
        let names: Vec<&str> = skills.iter().map(|s| s.name.as_str()).collect();
        let spec = ToolSpec::new(
            "skill",
            "Reads a skill: the procedure for a rare situation the system prompt lists. Read it \
             before acting on that situation, then follow it.",
            json!({"type": "object", "properties": {
                "name": {"type": "string", "enum": names}
            }, "required": ["name"], "additionalProperties": false}),
            Risk::Observe,
        );
        Self { spec, skills }
    }
}

#[async_trait]
impl Tool for SkillTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let name = args["name"].as_str().unwrap_or_default();
        match self.skills.iter().find(|s| s.name == name) {
            Some(s) => {
                let mut out = ToolOutcome::ok(json!({
                    "skill": s.name,
                    "procedure": crate::tools::clip(&s.body, BODY_CHARS),
                }));
                out.message = format!("read the {} skill", s.name);
                out
            }
            None => ToolOutcome::failed(format!("there is no skill {name}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NAV2: &str = "---\nname: nav2-not-up\ndescription: \"Use when Nav2 refuses goals.\"\n---\n\n# Nav2\n1. Check robot_state.\n";

    #[test]
    fn a_skill_is_its_front_matter_and_its_procedure() {
        let skill = parse(NAV2).unwrap();
        assert_eq!(skill.name, "nav2-not-up");
        assert_eq!(skill.description, "Use when Nav2 refuses goals.");
        assert_eq!(skill.body, "# Nav2\n1. Check robot_state.");
        assert!(parse("# no front matter").is_err());
        assert!(parse("---\nname: Bad Name\ndescription: d\n---\nx").is_err());
    }

    #[test]
    fn skills_load_from_their_folders_and_a_second_with_one_name_is_refused() {
        let dir = tempfile::tempdir().unwrap();
        for (folder, text) in [("a", NAV2), ("b", NAV2), ("c", "nothing here")] {
            std::fs::create_dir(dir.path().join(folder)).unwrap();
            std::fs::write(dir.path().join(folder).join("SKILL.md"), text).unwrap();
        }
        let (skills, problems) = load(&[dir.path().to_path_buf(), dir.path().join("none")]);
        assert_eq!(skills.len(), 1);
        assert_eq!(problems.len(), 3, "{problems:?}");
        let index = index(&skills).unwrap();
        assert!(
            index.ends_with("- nav2-not-up: Use when Nav2 refuses goals."),
            "{index}"
        );
    }

    #[tokio::test]
    async fn the_tool_reads_a_skill_by_name() {
        let tool = SkillTool::new(vec![parse(NAV2).unwrap()]);
        let read = tool.call(json!({"name": "nav2-not-up"})).await;
        assert_eq!(read.data["procedure"], "# Nav2\n1. Check robot_state.");
        let none = tool.call(json!({"name": "other"})).await;
        assert!(none.message.contains("no skill other"));
    }
}
