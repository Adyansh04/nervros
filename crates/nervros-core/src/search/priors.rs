//! Where things usually are in a home, for the rooms worth looking in when the world model has
//! never seen the thing. Robot-neutral.

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

/// The room types `words` is most often in, most likely first; none for a thing no prior
/// knows. A word matches a key as it is or with a plural `s`, punctuation aside.
#[must_use]
pub(crate) fn rooms_for(words: &str) -> Vec<String> {
    let words = words.to_lowercase();
    let hit = |key: &str| {
        words
            .split_whitespace()
            .map(|w| w.trim_matches(|c: char| !c.is_alphanumeric()))
            .any(|w| w == key || w.strip_suffix('s') == Some(key))
    };
    DEFAULTS
        .iter()
        .find(|(k, _)| hit(k))
        .map(|(_, rooms)| rooms.iter().map(|r| (*r).to_owned()).collect())
        .unwrap_or_default()
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
    fn a_thing_finds_its_rooms_by_any_of_its_words_singular_or_plural() {
        assert_eq!(rooms_for("my keys")[0], "hallway");
        assert_eq!(rooms_for("the red Mug?")[0], "kitchen");
        assert_eq!(
            rooms_for("my glasses")[0],
            "bedroom",
            "a key that ends in s"
        );
        assert_eq!(rooms_for("a glass")[0], "kitchen");
        assert!(rooms_for("a spaceship").is_empty());
        assert_eq!(room_type("Living room"), "living_room");
    }
}
