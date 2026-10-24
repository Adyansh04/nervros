//! The skills a program's text says it runs, read before it runs so the approval can name them:
//! each `run("Skill", ...)`, a call of a skill by its name, `find(...)` for `FindInView` and
//! `walk(...)` for `GoToPlace`. Comments
//! and strings are skipped, so text inside them names nothing. The envelope is enforced again on
//! every mission the program sends, so a call this misses is refused, never let through.

use std::collections::BTreeSet;

use super::api::SkillCall;
use crate::mission::catalog::{FIND_IN_VIEW, GO_TO_PLACE};

/// The skills `source` runs, by the names plans use.
///
/// # Errors
///
/// A `run` whose skill is not a string literal: the approval could not say what it runs.
pub fn named_in(source: &str, callable: &[SkillCall]) -> Result<BTreeSet<String>, String> {
    let tokens = tokens(source);
    let mut skills = BTreeSet::new();
    for (i, token) in tokens.iter().enumerate() {
        let Token::Ident(name) = token else { continue };
        // A method of that name, such as `x.find(...)`, is not the robot's.
        let called = matches!(tokens.get(i + 1), Some(Token::Punct('(')))
            && !matches!(
                i.checked_sub(1).and_then(|p| tokens.get(p)),
                Some(Token::Punct('.'))
            );
        if !called {
            continue;
        }
        match *name {
            "find" => {
                skills.insert(FIND_IN_VIEW.to_owned());
            }
            "walk" => {
                skills.insert(GO_TO_PLACE.to_owned());
            }
            "run" => match tokens.get(i + 2) {
                Some(Token::Str(skill)) if is_skill_name(skill) => {
                    skills.insert((*skill).to_owned());
                }
                _ => {
                    return Err(
                        "name each skill run() runs as a string, as in run(\"PickObject\", \
                         #{object_id: id, arm: \"right\"}), so the approval can say what it runs"
                            .to_owned(),
                    );
                }
            },
            other if callable.iter().any(|s| s.name == other) => {
                skills.insert(other.to_owned());
            }
            _ => {}
        }
    }
    Ok(skills)
}

fn is_skill_name(s: &str) -> bool {
    s.chars().next().is_some_and(|c| c.is_ascii_uppercase())
        && s.chars().all(|c| c.is_ascii_alphanumeric())
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum Token<'s> {
    Ident(&'s str),
    /// A string literal's contents, escapes left as written.
    Str(&'s str),
    Punct(char),
}

/// Rhai's text cut into what matters here: identifiers, string literals and punctuation.
fn tokens(source: &str) -> Vec<Token<'_>> {
    let bytes = source.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i];
        match c {
            b'/' if bytes.get(i + 1) == Some(&b'/') => {
                while i < bytes.len() && bytes[i] != b'\n' {
                    i += 1;
                }
            }
            b'/' if bytes.get(i + 1) == Some(&b'*') => {
                i += 2;
                while i < bytes.len() && !(bytes[i] == b'*' && bytes.get(i + 1) == Some(&b'/')) {
                    i += 1;
                }
                i += 2;
            }
            b'"' | b'`' | b'\'' => {
                let start = i + 1;
                i += 1;
                while i < bytes.len() && bytes[i] != c {
                    i += if bytes[i] == b'\\' { 2 } else { 1 };
                }
                if c == b'"' {
                    out.push(Token::Str(
                        source.get(start..i.min(bytes.len())).unwrap_or(""),
                    ));
                }
                i += 1;
            }
            c if c.is_ascii_alphabetic() || c == b'_' => {
                let start = i;
                while i < bytes.len() && (bytes[i].is_ascii_alphanumeric() || bytes[i] == b'_') {
                    i += 1;
                }
                out.push(Token::Ident(&source[start..i]));
            }
            c if c.is_ascii_whitespace() || c.is_ascii_digit() => i += 1,
            c => {
                out.push(Token::Punct(char::from(c)));
                i += 1;
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set(names: &[&str]) -> BTreeSet<String> {
        names.iter().map(|s| (*s).to_owned()).collect()
    }

    #[test]
    fn the_skills_a_program_runs_are_read_from_its_calls() {
        let source = r#"
            // run("Dance", #{}) in a comment names nothing
            let rooms = rooms();
            for r in rooms {
                walk(r.id);
                let id = find("mug");
                if id != () { run("PickObject", #{object_id: id, arm: "right"}); }
            }
            say("run(\"Wave\") in a string names nothing either");
            /* run("Jump") */
        "#;
        assert_eq!(
            named_in(source, &[]).unwrap(),
            set(&["FindInView", "GoToPlace", "PickObject"])
        );
    }

    #[test]
    fn a_skill_named_at_run_time_is_refused_and_a_method_is_not_the_robots() {
        assert!(named_in("let s = \"PickObject\"; run(s, #{});", &[]).is_err());
        assert!(named_in("run(\"pick object\", #{})", &[]).is_err());
        assert_eq!(
            named_in("let i = \"text\".find(\"x\");", &[]).unwrap(),
            set(&[])
        );
        assert_eq!(
            named_in("let walked = 1; let run_count = 2;", &[]).unwrap(),
            set(&[])
        );
        let place = [SkillCall {
            name: "PlaceInto".to_owned(),
            args: vec!["container_id".to_owned(), "arm".to_owned()],
        }];
        assert_eq!(
            named_in("PlaceInto(\"O30\");", &place).unwrap(),
            set(&["PlaceInto"])
        );
    }
}
