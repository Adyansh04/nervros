//! The robot's skill catalog, as its executor's `GetCatalog` serves it.

use serde::Deserialize;

/// The navigation macros a plan never names: it writes `GoToPlace`, which becomes one of these.
pub const GO_TO_POSE: &str = "GoToPose";
/// See [`GO_TO_POSE`].
pub const GO_TO_TARGET: &str = "GoToTarget";
/// The skill plans use to move the base.
pub const GO_TO_PLACE: &str = "GoToPlace";

/// Every skill the executor can run.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Catalog {
    /// Changes whenever the catalog does; approvals name it.
    pub catalog_version: String,
    /// The skills.
    pub skills: Vec<Skill>,
}

/// One skill: a behaviour-tree macro in the executor's library.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Skill {
    /// The name plans use.
    pub name: String,
    /// For the model.
    #[serde(default)]
    pub description: String,
    /// Its inputs, all required.
    #[serde(default)]
    pub args: Vec<Arg>,
    /// Predicates that must hold before it runs.
    #[serde(default)]
    pub requires: Vec<String>,
    /// Predicates that hold after it succeeds.
    #[serde(default)]
    pub effects: Vec<String>,
    /// `base`, `left_arm`, `right_arm`.
    #[serde(default)]
    pub resources: Vec<String>,
    /// Whether running it twice is harmless.
    #[serde(default)]
    pub idempotent: bool,
    /// `motion` or `manipulation`.
    #[serde(default)]
    pub risk: String,
    /// Its own timeout; a step may ask for less, never more.
    pub max_duration_s: f64,
    /// The `SubTree` ID in the executor's library.
    pub template: String,
}

/// One input of a skill.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Arg {
    /// The port name.
    pub name: String,
    /// For the model.
    #[serde(default)]
    pub description: String,
    /// Allowed values; empty means any.
    #[serde(default, rename = "enum")]
    pub choices: Vec<String>,
}

impl Catalog {
    /// Parses `GetCatalog.catalog_json`.
    ///
    /// # Errors
    ///
    /// The JSON does not have the catalog's shape.
    pub fn parse(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }

    /// A skill by name.
    #[must_use]
    pub fn skill(&self, name: &str) -> Option<&Skill> {
        self.skills.iter().find(|s| s.name == name)
    }

    /// Whether the base can be sent somewhere: the executor has either navigation macro.
    #[must_use]
    pub fn can_travel(&self) -> bool {
        self.skill(GO_TO_POSE).is_some() || self.skill(GO_TO_TARGET).is_some()
    }

    /// The skill names a plan may use: the catalog's, with the navigation macros folded into
    /// `GoToPlace`.
    #[must_use]
    pub fn plan_skills(&self) -> Vec<String> {
        let mut names: Vec<String> = self
            .skills
            .iter()
            .filter(|s| s.name != GO_TO_POSE && s.name != GO_TO_TARGET)
            .map(|s| s.name.clone())
            .collect();
        if self.can_travel() {
            names.insert(0, GO_TO_PLACE.to_owned());
        }
        names
    }

    /// One line per plan skill, with its arguments, for the model.
    #[must_use]
    pub fn describe(&self) -> String {
        let mut lines = Vec::new();
        if self.can_travel() {
            lines.push(format!(
                "- {GO_TO_PLACE}(place): walk to a named place, a room id or an object id from \
                 list_places or find_objects"
            ));
        }
        for s in self
            .skills
            .iter()
            .filter(|s| s.name != GO_TO_POSE && s.name != GO_TO_TARGET)
        {
            let args: Vec<String> = s
                .args
                .iter()
                .map(|a| {
                    if a.choices.is_empty() {
                        a.name.clone()
                    } else {
                        format!("{}={}", a.name, a.choices.join("|"))
                    }
                })
                .collect();
            lines.push(format!(
                "- {}({}): {}",
                s.name,
                args.join(", "),
                s.description
            ));
        }
        lines.join("\n")
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// The catalog shape grove-g1's executor serves, trimmed.
    pub(crate) const CATALOG: &str = r#"{"catalog_version": "a1b2c3d4e5f6", "skills": [
        {"name": "GoToPose", "description": "Walk to a pose.", "args": [{"name": "station", "type": "string", "description": "x;y;yaw", "enum": []}],
         "requires": [], "effects": ["at(station)"], "resources": ["base"], "idempotent": true, "risk": "motion", "max_duration_s": 300, "template": "GoToPose"},
        {"name": "GoToTarget", "description": "Walk to a room or object.", "args": [{"name": "target", "type": "string", "description": "id", "enum": []}],
         "requires": [], "effects": ["at(target)"], "resources": ["base"], "idempotent": true, "risk": "motion", "max_duration_s": 300, "template": "GoToTarget"},
        {"name": "PickObject", "description": "Pick an object up.", "args": [
            {"name": "object_id", "type": "string", "description": "world model id", "enum": []},
            {"name": "phrase", "type": "string", "description": "what the detector looks for", "enum": []},
            {"name": "arm", "type": "string", "description": "which arm", "enum": ["left", "right"]}],
         "requires": ["near(object_id)"], "effects": ["holding(arm, object_id)"], "resources": ["base", "left_arm", "right_arm"], "idempotent": false, "risk": "manipulation", "max_duration_s": 420, "template": "PickObject"},
        {"name": "TuckForTravel", "description": "Fold the arms for walking.", "args": [],
         "requires": [], "effects": [], "resources": ["left_arm", "right_arm"], "idempotent": true, "risk": "manipulation", "max_duration_s": 30, "template": "TuckForTravel"}
    ]}"#;

    #[test]
    fn navigation_folds_into_go_to_place() {
        let c = Catalog::parse(CATALOG).unwrap();
        assert_eq!(
            c.plan_skills(),
            ["GoToPlace", "PickObject", "TuckForTravel"]
        );
        let text = c.describe();
        assert!(text.contains("PickObject(object_id, phrase, arm=left|right)"));
        assert!(!text.contains("GoToPose"));
    }
}
