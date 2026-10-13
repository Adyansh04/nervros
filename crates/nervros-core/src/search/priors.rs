//! Where things usually are in a home, for the rooms worth looking in when the world model has
//! never seen the thing. Robot-neutral; a profile adds its own or changes these.

use std::collections::BTreeMap;

/// Words of a thing, and the room types it is most often in, most likely first.
const DEFAULTS: &[(&str, &[&str])] = &[
    ("key", &["hallway", "living_room", "kitchen", "bedroom"]),
    ("wallet", &["bedroom", "living_room", "hallway"]),
    ("phone", &["living_room", "bedroom", "office", "kitchen"]),
    ("remote", &["living_room", "bedroom"]),
    ("glasses", &["bedroom", "living_room", "office"]),
    ("mug", &["kitchen", "dining_room", "office", "living_room"]),
    ("cup", &["kitchen", "dining_room", "living_room", "office"]),
    ("glass", &["kitchen", "dining_room", "living_room"]),
    ("plate", &["kitchen", "dining_room"]),
    ("bowl", &["kitchen", "dining_room"]),
    ("bottle", &["kitchen", "dining_room", "office"]),
    ("book", &["living_room", "bedroom", "office"]),
    ("laptop", &["office", "living_room", "bedroom"]),
    ("towel", &["bathroom", "bedroom"]),
    ("shoe", &["hallway", "bedroom"]),
    ("toy", &["living_room", "bedroom"]),
    ("tray", &["kitchen", "dining_room", "living_room"]),
    ("pillow", &["bedroom", "living_room"]),
];

/// The room types a thing is most often in.
#[derive(Debug, Clone, Default)]
pub struct Priors {
    /// A profile's own, which come before the defaults.
    own: BTreeMap<String, Vec<String>>,
}

impl Priors {
    /// The defaults with `own` before them: a word to the room types it is in.
    #[must_use]
    pub fn new(own: BTreeMap<String, Vec<String>>) -> Self {
        Self {
            own: own
                .into_iter()
                .map(|(k, v)| {
                    (
                        k.to_lowercase(),
                        v.into_iter().map(|t| room_type(&t)).collect(),
                    )
                })
                .collect(),
        }
    }

    /// The room types `words` is most often in, most likely first; none for a thing no prior
    /// knows.
    #[must_use]
    pub fn rooms_for(&self, words: &str) -> Vec<String> {
        let words = words.to_lowercase();
        let hit = |key: &str| {
            words
                .split_whitespace()
                .any(|w| w.trim_end_matches('s') == key)
        };
        self.own
            .iter()
            .find(|(k, _)| hit(k))
            .map(|(_, rooms)| rooms.clone())
            .or_else(|| {
                DEFAULTS
                    .iter()
                    .find(|(k, _)| hit(k))
                    .map(|(_, rooms)| rooms.iter().map(|r| (*r).to_owned()).collect())
            })
            .unwrap_or_default()
    }
}

/// A room type as the world model writes it: "living room" is `living_room`.
#[must_use]
pub fn room_type(text: &str) -> String {
    text.trim().to_lowercase().replace([' ', '-'], "_")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_thing_finds_its_rooms_by_any_of_its_words_and_a_profile_comes_first() {
        let p = Priors::default();
        assert_eq!(p.rooms_for("my keys")[0], "hallway");
        assert_eq!(p.rooms_for("the red Mug")[0], "kitchen");
        assert!(p.rooms_for("a spaceship").is_empty());

        let own = BTreeMap::from([("mug".to_owned(), vec!["Office".to_owned()])]);
        assert_eq!(Priors::new(own).rooms_for("mug"), ["office"]);
        assert_eq!(room_type("Living room"), "living_room");
    }
}
