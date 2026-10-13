//! What a job may do, as the operator approved it: the skills its missions may use, by the names
//! plans write (`GoToPlace` for any walk), and how long it may run.

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::time::Duration;

use serde::{Deserialize, Serialize};

use crate::mission::catalog::{Catalog, GO_TO_PLACE, Skill};
use crate::tools::Risk;

/// The longest any job may run.
pub const MAX_MINUTES: u64 = 60;

/// The skills a job's missions may use, and its time limit.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Envelope {
    /// Skill names as plans write them, such as `GoToPlace` or `FindInView`.
    pub skills: BTreeSet<String>,
    /// Minutes it may run, at most [`MAX_MINUTES`].
    pub max_minutes: u64,
}

impl Envelope {
    /// An envelope of these skills, the minutes clamped to 1 to [`MAX_MINUTES`].
    #[must_use]
    pub fn new(skills: impl IntoIterator<Item = String>, max_minutes: u64) -> Self {
        Self {
            skills: skills.into_iter().collect(),
            max_minutes: max_minutes.clamp(1, MAX_MINUTES),
        }
    }

    /// Whether its missions may use `skill`.
    #[must_use]
    pub fn allows(&self, skill: &str) -> bool {
        self.skills.contains(skill)
    }

    /// How long it may run.
    #[must_use]
    pub fn limit(&self) -> Duration {
        Duration::from_secs(self.max_minutes * 60)
    }

    /// The most its skills can do: what the operator is asked to approve.
    #[must_use]
    pub fn risk(&self, catalog: &Catalog) -> Risk {
        self.skills
            .iter()
            .map(|s| skill_risk(catalog, s))
            .max()
            .unwrap_or(Risk::Observe)
    }

    /// The skills it names that the catalog does not have.
    #[must_use]
    pub fn unknown<'e>(&'e self, catalog: &Catalog) -> Vec<&'e str> {
        self.skills
            .iter()
            .filter(|s| {
                catalog.skill(s).is_none() && !(s.as_str() == GO_TO_PLACE && catalog.can_travel())
            })
            .map(String::as_str)
            .collect()
    }

    /// What it lets a job do, for the approval: "may move the base (`GoToPlace`) and look
    /// (`FindInView`), for at most 10 min".
    #[must_use]
    pub fn describe(&self, catalog: &Catalog) -> String {
        let group = |risk: Risk| -> Vec<&str> {
            self.skills
                .iter()
                .filter(|s| skill_risk(catalog, s) == risk)
                .map(String::as_str)
                .collect()
        };
        let parts: Vec<String> = [
            (Risk::Manipulation, "move the arms"),
            (Risk::Motion, "move the base"),
            (Risk::Observe, "look"),
        ]
        .into_iter()
        .filter_map(|(risk, verb)| {
            let skills = group(risk);
            (!skills.is_empty()).then(|| format!("{verb} ({})", skills.join(", ")))
        })
        .collect();
        let mut text = if parts.is_empty() {
            "moves nothing".to_owned()
        } else {
            format!("may {}", parts.join(" and "))
        };
        let _ = write!(text, ", for at most {} min", self.max_minutes);
        text
    }
}

/// The risk of a skill a plan names; one the catalog lost counts as the worst it could be.
fn skill_risk(catalog: &Catalog, name: &str) -> Risk {
    match catalog.skill(name) {
        Some(skill) => risk_of(skill),
        None if name == GO_TO_PLACE && catalog.can_travel() => Risk::Motion,
        None => Risk::Manipulation,
    }
}

/// A skill's risk, from what it holds: the arms are manipulation, the base motion.
#[must_use]
pub fn risk_of(skill: &Skill) -> Risk {
    let holds = |r: &str| skill.resources.iter().any(|x| x == r);
    if holds("left_arm") || holds("right_arm") {
        Risk::Manipulation
    } else if holds("base") {
        Risk::Motion
    } else {
        Risk::Observe
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn catalog() -> Catalog {
        Catalog::parse(
            r#"{"catalog_version": "v", "skills": [
                {"name": "GoToTarget", "resources": ["base"], "max_duration_s": 60,
                 "template": "GoToTarget"},
                {"name": "FindInView", "resources": [], "max_duration_s": 10,
                 "template": "FindInView"},
                {"name": "PickObject", "resources": ["right_arm", "left_arm"],
                 "max_duration_s": 90, "template": "PickObject"}
            ]}"#,
        )
        .unwrap()
    }

    #[test]
    fn its_risk_is_the_most_its_skills_can_do() {
        let c = catalog();
        let look = Envelope::new(["FindInView".to_owned()], 5);
        let walk = Envelope::new(["FindInView".to_owned(), "GoToPlace".to_owned()], 5);
        let pick = Envelope::new(["PickObject".to_owned()], 5);
        assert_eq!(look.risk(&c), Risk::Observe);
        assert_eq!(walk.risk(&c), Risk::Motion);
        assert_eq!(pick.risk(&c), Risk::Manipulation);
        assert_eq!(Envelope::new([], 5).risk(&c), Risk::Observe);
        // A skill the catalog lost counts as the worst it could be.
        assert_eq!(
            Envelope::new(["Dance".to_owned()], 5).risk(&c),
            Risk::Manipulation
        );
    }

    #[test]
    fn the_approval_names_each_skill_by_what_it_moves() {
        let c = catalog();
        let e = Envelope::new(["FindInView".to_owned(), "GoToPlace".to_owned()], 10);
        assert_eq!(
            e.describe(&c),
            "may move the base (GoToPlace) and look (FindInView), for at most 10 min"
        );
        assert!(e.unknown(&c).is_empty(), "a walk is a plan's own skill");
        assert_eq!(
            Envelope::new([], 0).describe(&c),
            "moves nothing, for at most 1 min"
        );
        assert_eq!(Envelope::new([], 500).max_minutes, MAX_MINUTES);
    }
}
