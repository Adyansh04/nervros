//! Where to look for something, best first: where the world model saw it, where it went after,
//! then the rooms such things are usually in, the less seen of them first, and the rest. Pure, so
//! the order can be tested without a robot.

use serde::Serialize;
use serde_json::Value;

use super::priors::{Priors, room_type};
use crate::mission::check::{how_long, named};

/// How fast what the world model saw goes stale: an hour old counts about 0.96 of new, a day 0.37.
const DAY_S: f64 = 86_400.0;
/// What each metre to walk takes off a lookout's score: a near room beats a far one of the same
/// odds, a far sighting still beats a near guess.
const PER_METRE: f64 = 0.01;

/// A place to look.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Lookout {
    /// What to walk to: an object's or a room's id, as `GoToPlace` takes it.
    pub target: String,
    /// Why look there, for the operator: "a mug was seen there 3 h ago".
    pub why: String,
    /// The room it is in, when known.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub room: String,
    /// Higher is likelier.
    pub score: f64,
    /// Where it is on the map, when known.
    #[serde(skip)]
    pub xy: Option<(f64, f64)>,
}

/// What a search is ranked from.
#[derive(Debug, Clone, Copy)]
pub struct Seen<'a> {
    /// The thing, in the operator's words.
    pub words: &'a str,
    /// A `WorldObjectArray`, or null.
    pub objects: &'a Value,
    /// A `RoomArray`, or null.
    pub rooms: &'a Value,
    /// The world model's events for the thing, oldest first (`ObjectEvent`s).
    pub events: &'a [Value],
    /// The robot on the map.
    pub robot: Option<(f64, f64)>,
    /// Unix seconds now.
    pub now_s: f64,
}

/// The places to look, best first, one per target.
#[must_use]
pub fn rank(seen: &Seen<'_>, priors: &Priors) -> Vec<Lookout> {
    let mut out: Vec<Lookout> = sightings(seen)
        .into_iter()
        .chain(moves(seen))
        .chain(rooms(seen, priors))
        .collect();
    for l in &mut out {
        if let (Some((x, y)), Some((rx, ry))) = (l.xy, seen.robot) {
            l.score -= PER_METRE * (x - rx).hypot(y - ry);
        }
    }
    out.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut seen_targets = std::collections::BTreeSet::new();
    out.retain(|l| seen_targets.insert(l.target.clone()));
    out
}

fn text<'v>(v: &'v Value, key: &str) -> &'v str {
    v[key].as_str().unwrap_or_default()
}

fn stamp_s(stamp: &Value) -> f64 {
    stamp["sec"].as_f64().unwrap_or(0.0) + stamp["nanosec"].as_f64().unwrap_or(0.0) * 1e-9
}

/// How much what was seen `at` still counts, now.
fn fresh(at: f64, now_s: f64) -> f64 {
    if at <= 0.0 {
        return 0.3;
    }
    (-(now_s - at).max(0.0) / DAY_S).exp().max(0.1)
}

fn ago(at: f64, now_s: f64) -> String {
    if at <= 0.0 {
        "some time ago".to_owned()
    } else {
        format!("{} ago", how_long((now_s - at).max(0.0)))
    }
}

/// The operator's words without the article or owner they start with: "my keys" is "keys".
pub(super) fn bare(words: &str) -> String {
    const LEADING: [&str; 9] = ["the", "a", "an", "my", "our", "your", "his", "her", "some"];
    let mut rest: Vec<&str> = words.split_whitespace().collect();
    while rest
        .first()
        .is_some_and(|w| LEADING.contains(&w.to_lowercase().as_str()))
    {
        rest.remove(0);
    }
    rest.join(" ")
}

/// Whether an object is the thing: its label or name holds the words, or, less surely, the
/// last of them, as a plain "mug" may be the "red mug" asked for.
fn matches(o: &Value, words: &str) -> Option<bool> {
    if named(o, words) {
        return Some(true);
    }
    let head = words.split_whitespace().last()?;
    named(o, head.trim_end_matches('s')).then_some(false)
}

/// Each object the world model has under these words, where it stood.
fn sightings(seen: &Seen<'_>) -> Vec<Lookout> {
    let words = bare(seen.words);
    seen.objects["objects"]
        .as_array()
        .map_or(&[][..], Vec::as_slice)
        .iter()
        .filter_map(|o| matches(o, &words).map(|exact| (o, exact)))
        .map(|(o, exact)| {
            let at = stamp_s(&o["last_seen"]);
            let name = [text(o, "name"), text(o, "label")]
                .into_iter()
                .find(|n| !n.is_empty())
                .unwrap_or("it");
            let label = if exact {
                name.to_owned()
            } else {
                format!("a {name}")
            };
            let (weight, why) = match o["state"].as_u64() {
                Some(1) => (
                    0.5,
                    format!(
                        "{label} was seen there {}, and not found there when looked at since",
                        ago(at, seen.now_s)
                    ),
                ),
                Some(2) => (
                    0.25,
                    format!("{label} used to be there, {}", ago(at, seen.now_s)),
                ),
                _ => (
                    1.0,
                    format!("{label} was seen there {}", ago(at, seen.now_s)),
                ),
            };
            let p = &o["pose"]["position"];
            let sure = if exact { 1.0 } else { 0.6 };
            Lookout {
                target: text(o, "id").to_owned(),
                why,
                room: text(o, "room_id").to_owned(),
                score: 1.0 + sure * weight * fresh(at, seen.now_s),
                xy: p["x"].as_f64().zip(p["y"].as_f64()),
            }
        })
        .collect()
}

/// Where the world model saw it go: the room of each move, newest counting most.
fn moves(seen: &Seen<'_>) -> Vec<Lookout> {
    seen.events
        .iter()
        .filter(|e| matches!(text(e, "kind"), "moved" | "appeared" | "seen_again"))
        .filter(|e| !text(e, "room_id").is_empty())
        .map(|e| {
            let at = stamp_s(&e["stamp"]);
            let p = &e["position"];
            Lookout {
                target: text(e, "room_id").to_owned(),
                why: format!(
                    "{} {} there {}",
                    text(e, "label"),
                    if text(e, "kind") == "moved" {
                        "was moved"
                    } else {
                        "turned up"
                    },
                    ago(at, seen.now_s)
                ),
                room: text(e, "room_id").to_owned(),
                score: 0.9 + 0.5 * fresh(at, seen.now_s),
                xy: p["x"].as_f64().zip(p["y"].as_f64()),
            }
        })
        .collect()
}

/// Every room: those such things are usually in first, each counting more the less of it the
/// camera has seen, since a seen room would have shown the thing.
fn rooms(seen: &Seen<'_>, priors: &Priors) -> Vec<Lookout> {
    let usual = priors.rooms_for(seen.words);
    seen.rooms["rooms"]
        .as_array()
        .map_or(&[][..], Vec::as_slice)
        .iter()
        .filter(|r| !text(r, "id").is_empty())
        .map(|r| {
            let kind = room_type(text(r, "type"));
            let rank = usual.iter().position(|u| *u == kind);
            let seen_share = r["floor_coverage"].as_f64().unwrap_or(0.0).clamp(0.0, 1.0);
            let unseen = 0.5 + 0.5 * (1.0 - seen_share);
            let (odds, why) = match rank {
                Some(i) => {
                    let place = if kind.is_empty() {
                        "room"
                    } else {
                        kind.as_str()
                    };
                    (
                        0.8 - 0.1 * f64::from(u32::try_from(i).unwrap_or(4).min(4)),
                        format!("such things are often in the {}", place.replace('_', " ")),
                    )
                }
                None => (0.2, "not looked at for it yet".to_owned()),
            };
            let why = if seen_share < 0.5 {
                format!(
                    "{why}; the camera has seen {:.0}% of it",
                    seen_share * 100.0
                )
            } else {
                why
            };
            let c = &r["centroid"];
            Lookout {
                target: text(r, "id").to_owned(),
                why,
                room: text(r, "id").to_owned(),
                score: odds * unseen,
                xy: c["x"].as_f64().zip(c["y"].as_f64()),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    const NOW: f64 = 1_000_000.0;

    fn rooms() -> Value {
        json!({"rooms": [
            {"id": "R1", "type": "kitchen", "centroid": {"x": 0.0, "y": 0.0}, "floor_coverage": 0.9},
            {"id": "R2", "type": "office", "centroid": {"x": 8.0, "y": 0.0}, "floor_coverage": 0.2},
            {"id": "R3", "type": "bathroom", "centroid": {"x": 2.0, "y": 0.0}, "floor_coverage": 1.0}]})
    }

    fn order(seen: &Seen<'_>) -> Vec<String> {
        rank(seen, &Priors::default())
            .into_iter()
            .map(|l| l.target)
            .collect()
    }

    #[test]
    fn a_sighting_comes_first_and_a_stale_one_after_a_fresh_one() {
        let objects = json!({"objects": [
            {"id": "O1", "label": "mug", "room_id": "R1", "state": 1,
             "pose": {"position": {"x": 0.5, "y": 0.0}}, "last_seen": {"sec": NOW - 600.0}},
            {"id": "O2", "label": "mug", "room_id": "R2", "state": 0,
             "pose": {"position": {"x": 8.5, "y": 0.0}}, "last_seen": {"sec": NOW - 3600.0}}]});
        let rooms = rooms();
        let seen = Seen {
            words: "mug",
            objects: &objects,
            rooms: &rooms,
            events: &[],
            robot: Some((0.0, 0.0)),
            now_s: NOW,
        };
        let ranked = rank(&seen, &Priors::default());
        assert_eq!(
            ranked.iter().map(|l| l.target.as_str()).collect::<Vec<_>>(),
            ["O2", "O1", "R2", "R1", "R3"],
            "the office, a fifth seen, can hide another mug more than the kitchen, nine tenths seen"
        );
        assert_eq!(ranked[0].why, "mug was seen there 60 min ago");
        assert!(
            ranked[1]
                .why
                .contains("not found there when looked at since")
        );
    }

    #[test]
    fn with_no_sighting_the_usual_rooms_come_first_the_least_seen_most() {
        let objects = json!({"objects": []});
        let rooms = rooms();
        let seen = Seen {
            words: "my keys",
            objects: &objects,
            rooms: &rooms,
            events: &[],
            robot: Some((0.0, 0.0)),
            now_s: NOW,
        };
        // The kitchen is one of the rooms keys are in; then the less seen of the others.
        assert_eq!(order(&seen), ["R1", "R2", "R3"]);

        let seen = Seen {
            words: "cup",
            ..seen
        };
        let ranked = rank(&seen, &Priors::default());
        assert_eq!(ranked[0].target, "R1", "cups are kitchen things");
        assert_eq!(ranked[1].target, "R2", "then the office, little of it seen");
        assert!(
            ranked[1].why.ends_with("the camera has seen 20% of it"),
            "{}",
            ranked[1].why
        );
    }

    #[test]
    fn a_thing_asked_for_with_more_words_still_finds_a_plainer_one_after_an_exact_one() {
        let objects = json!({"objects": [
            {"id": "O1", "label": "mug", "room_id": "R1", "state": 0, "last_seen": {"sec": NOW}},
            {"id": "O2", "label": "mug", "name": "red mug", "room_id": "R2", "state": 0,
             "last_seen": {"sec": NOW}}]});
        let rooms = rooms();
        let seen = Seen {
            words: "my red mug",
            objects: &objects,
            rooms: &rooms,
            events: &[],
            robot: None,
            now_s: NOW,
        };
        let ranked = rank(&seen, &Priors::default());
        assert_eq!(
            (ranked[0].target.as_str(), ranked[1].target.as_str()),
            ("O2", "O1")
        );
        assert_eq!(ranked[1].why, "a mug was seen there 0 s ago");
        assert_eq!(bare("The keys"), "keys");
    }

    #[test]
    fn a_move_points_at_the_room_it_went_to_and_each_target_comes_once() {
        let objects = json!({"objects": []});
        let rooms = rooms();
        let events = [json!({"kind": "moved", "label": "mug", "room_id": "R3",
                             "position": {"x": 2.0, "y": 0.0}, "stamp": {"sec": NOW - 60.0}})];
        let seen = Seen {
            words: "mug",
            objects: &objects,
            rooms: &rooms,
            events: &events,
            robot: None,
            now_s: NOW,
        };
        let ranked = rank(&seen, &Priors::default());
        assert_eq!(ranked[0].target, "R3");
        assert_eq!(ranked[0].why, "mug was moved there 60 s ago");
        assert_eq!(ranked.iter().filter(|l| l.target == "R3").count(), 1);
    }
}
