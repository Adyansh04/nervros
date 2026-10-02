//! Named places: the profile's, and the ones the operator has the robot remember where it stands
//! ("remember this spot as the reading corner"), kept in the state directory across sessions.
//! `list_places` and the plan compiler read both; `tag_place` and `forget_place` change the second.

use std::borrow::Cow;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};

use async_trait::async_trait;
use nervros_ros::RobotPort;
use serde_json::{Value, json};

use crate::profile::{PlaceConfig, PlacePose, Profile};
use crate::tools::{Risk, Tool, ToolOutcome, ToolSpec};

/// Every named place.
#[derive(Debug)]
pub struct Places {
    fixed: Vec<PlaceConfig>,
    tagged: Mutex<Vec<PlaceConfig>>,
    file: Option<PathBuf>,
}

fn to_json(p: &PlaceConfig) -> Value {
    json!({"name": p.name, "aliases": p.aliases, "frame": p.frame,
           "pose": {"x": p.pose.x, "y": p.pose.y, "yaw": p.pose.yaw}})
}

impl Places {
    /// The profile's places, and those remembered in `file` when it exists.
    #[must_use]
    pub fn new(profile: &Profile, file: Option<PathBuf>) -> Arc<Self> {
        let tagged: Vec<PlaceConfig> = file
            .as_deref()
            .map(crate::persist::read_or_default)
            .unwrap_or_default();
        Arc::new(Self {
            fixed: profile.places.clone(),
            tagged: Mutex::new(tagged),
            file,
        })
    }

    fn lock(&self) -> MutexGuard<'_, Vec<PlaceConfig>> {
        crate::lock(&self.tagged)
    }

    /// The profile's places, then the remembered ones.
    #[must_use]
    pub fn all(&self) -> Vec<PlaceConfig> {
        let mut all = self.fixed.clone();
        all.extend(self.lock().iter().cloned());
        all
    }

    /// Whether a place was remembered rather than written in the profile.
    #[must_use]
    pub fn is_tagged(&self, name: &str) -> bool {
        self.lock().iter().any(|p| p.name == name)
    }

    fn save(&self, tagged: &[PlaceConfig]) -> Result<(), String> {
        let Some(file) = &self.file else {
            return Ok(());
        };
        let text = serde_json::to_vec_pretty(&tagged.iter().map(to_json).collect::<Vec<_>>())
            .map_err(|e| e.to_string())?;
        crate::persist::write_atomic(file, &text)
            .map_err(|e| format!("saving {}: {e}", file.display()))
    }

    /// Remembers a place, replacing one remembered under that name before.
    ///
    /// # Errors
    ///
    /// The profile already has a place of that name, or the file cannot be written.
    pub fn tag(&self, place: PlaceConfig) -> Result<(), String> {
        if self.fixed.iter().any(|p| p.name == place.name) {
            return Err(format!(
                "`{}` is one of the profile's places; pick another name",
                place.name
            ));
        }
        let mut tagged = self.lock();
        tagged.retain(|p| p.name != place.name);
        tagged.push(place);
        self.save(&tagged)
    }

    /// Forgets a remembered place; `false` when there was none of that name.
    ///
    /// # Errors
    ///
    /// The file cannot be written.
    pub fn forget(&self, name: &str) -> Result<bool, String> {
        let mut tagged = self.lock();
        let before = tagged.len();
        tagged.retain(|p| p.name != name);
        if tagged.len() == before {
            return Ok(false);
        }
        self.save(&tagged).map(|()| true)
    }
}

/// Lowercase letters, digits and underscores.
pub(crate) fn slug(text: &str) -> String {
    let name: String = text
        .trim()
        .to_lowercase()
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect();
    name.trim_matches('_').to_owned()
}

/// A place name plans can use.
fn place_name(text: &str) -> Option<String> {
    let name = slug(text);
    (!name.is_empty() && name.len() <= 40).then_some(name)
}

/// The `tag_place` tool.
pub struct TagPlace {
    spec: ToolSpec,
    places: Arc<Places>,
    robot: Arc<dyn RobotPort>,
    frames: (String, String),
}

impl TagPlace {
    /// Remembers where the robot stands, in the profile's map frame.
    #[must_use]
    pub fn new(profile: &Profile, places: Arc<Places>, robot: Arc<dyn RobotPort>) -> Self {
        let spec = ToolSpec::new(
            "tag_place",
            "Remembers where the robot stands now, and which way it faces, as a named place, for \
             plans to walk back to (\"remember this spot as the reading corner\"). It is kept \
             across sessions; list_places lists it.",
            json!({"type": "object", "properties": {
                "name": {"type": "string", "description": "The place's name, such as reading_corner."},
                "aliases": {"type": "array", "items": {"type": "string"}, "description": "Other names for it, such as \"the reading corner\"."}
            }, "required": ["name"], "additionalProperties": false}),
            Risk::Annotate,
        );
        Self {
            spec,
            places,
            robot,
            frames: (
                profile.ros.map_frame.clone(),
                profile.ros.base_frame.clone(),
            ),
        }
    }
}

#[async_trait]
impl Tool for TagPlace {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let Some(name) = args["name"].as_str().and_then(place_name) else {
            return ToolOutcome::failed("`name` is a short name, such as reading_corner");
        };
        let (map, base) = &self.frames;
        let at = match self.robot.transform(map, base) {
            Ok(t) => t,
            Err(e) => return ToolOutcome::failed(format!("where the robot is is unknown: {e}")),
        };
        let aliases = args["aliases"]
            .as_array()
            .map(|a| {
                a.iter()
                    .filter_map(Value::as_str)
                    .map(str::to_owned)
                    .collect()
            })
            .unwrap_or_default();
        let round = |v: f64| (v * 1000.0).round() / 1000.0;
        let place = PlaceConfig {
            near: Vec::new(),
            name: name.clone(),
            aliases,
            frame: map.clone(),
            pose: PlacePose {
                x: round(at.translation[0]),
                y: round(at.translation[1]),
                yaw: round(at.yaw()),
            },
        };
        let data = to_json(&place);
        match self.places.tag(place) {
            Ok(()) => {
                let mut out = ToolOutcome::ok(data);
                out.message = format!("remembered {name}");
                out
            }
            Err(why) => ToolOutcome::failed(why),
        }
    }
}

/// The `forget_place` tool.
pub struct ForgetPlace {
    spec: ToolSpec,
    places: Arc<Places>,
}

impl ForgetPlace {
    /// Forgets remembered places, never the profile's.
    #[must_use]
    pub fn new(places: Arc<Places>) -> Self {
        let spec = ToolSpec::new(
            "forget_place",
            "Forgets a place tag_place remembered. The profile's own places stay.",
            json!({"type": "object", "properties": {
                "name": {"type": "string"}
            }, "required": ["name"], "additionalProperties": false}),
            Risk::Annotate,
        );
        Self { spec, places }
    }
}

#[async_trait]
impl Tool for ForgetPlace {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let name = args["name"]
            .as_str()
            .and_then(place_name)
            .unwrap_or_default();
        match self.places.forget(&name) {
            Ok(true) => ToolOutcome::ok(json!({"forgot": name})),
            Ok(false) => ToolOutcome::failed(format!("no remembered place `{name}`")),
            Err(why) => ToolOutcome::failed(why),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nervros_ros::Transform;
    use nervros_ros::fake::FakeRobot;

    fn profile() -> Profile {
        toml::from_str(
            "[robot]\nname = \"r\"\n[models]\nfile = \"m.toml\"\n\
             [[place]]\nname = \"dock\"\npose = { x = 0.0, y = 0.0 }\n",
        )
        .unwrap()
    }

    #[tokio::test]
    async fn a_tagged_place_is_kept_across_sessions_and_forgotten() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("places").join("r.json");
        let robot: Arc<dyn RobotPort> = Arc::new(FakeRobot::new().with_transform(
            "map",
            "base_footprint",
            Transform {
                translation: [1.5, -2.0, 0.0],
                rotation: [
                    0.0,
                    0.0,
                    std::f64::consts::FRAC_1_SQRT_2,
                    std::f64::consts::FRAC_1_SQRT_2,
                ],
            },
        ));
        let places = Places::new(&profile(), Some(file.clone()));
        let tag = TagPlace::new(&profile(), Arc::clone(&places), Arc::clone(&robot));
        let out = tag
            .call(json!({"name": "Reading corner", "aliases": ["the reading corner"]}))
            .await;
        assert_eq!(out.message, "remembered reading_corner");
        assert!(
            (out.data["pose"]["yaw"].as_f64().unwrap() - std::f64::consts::FRAC_PI_2).abs() < 1e-3
        );
        let again = Places::new(&profile(), Some(file.clone()));
        let names: Vec<String> = again.all().into_iter().map(|p| p.name).collect();
        assert_eq!(names, ["dock", "reading_corner"]);
        assert!(again.is_tagged("reading_corner") && !again.is_tagged("dock"));
        let clash = tag.call(json!({"name": "dock"})).await;
        assert!(
            clash.message.contains("profile's places"),
            "{}",
            clash.message
        );
        let forget = ForgetPlace::new(Arc::clone(&again));
        assert_eq!(
            forget.call(json!({"name": "reading_corner"})).await.status,
            crate::tools::Status::Succeeded
        );
        assert_eq!(Places::new(&profile(), Some(file)).all().len(), 1);
    }
}
