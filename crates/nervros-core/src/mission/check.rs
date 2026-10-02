//! Goal checks after a mission reports success: the executor says the tree finished, these say
//! whether the world agrees.

use std::fmt::Write as _;

use serde::Serialize;
use serde_json::Value;

use super::plan::{PlannedStep, predicate as predicate_of};
use crate::builtins::inside;
use crate::profile::PlaceConfig;

/// How far from a place's pose still counts as there.
const AT_PLACE_M: f64 = 0.5;
/// How far from an object still counts as at it: arm's reach.
const AT_OBJECT_M: f64 = 1.2;
/// Slack around a container's footprint for `inside` and `on`.
const FOOTPRINT_SLACK_M: f64 = 0.05;

/// One predicate's verdict.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Verdict {
    /// As the plan wrote it.
    pub predicate: String,
    /// `None` when it cannot be checked.
    pub ok: Option<bool>,
    /// Why.
    pub detail: String,
}

/// What the checks read, gathered once after the mission.
#[derive(Debug, Clone, Default)]
pub struct Observed {
    /// The robot on the map, `(x, y)`.
    pub pose: Option<(f64, f64)>,
    /// The profile's places.
    pub places: Vec<PlaceConfig>,
    /// A `RoomArray` message, or null.
    pub rooms: Value,
    /// A `WorldObjectArray` message, or null.
    pub objects: Value,
    /// The executor's `RobotState`, or null.
    pub state: Value,
}

pub(crate) fn object<'v>(objects: &'v Value, id: &str) -> Option<&'v Value> {
    objects["objects"]
        .as_array()?
        .iter()
        .find(|o| o["id"] == id)
}

fn xy(v: &Value) -> (f64, f64) {
    (
        v["x"].as_f64().unwrap_or(0.0),
        v["y"].as_f64().unwrap_or(0.0),
    )
}

/// Checks each predicate.
#[must_use]
pub fn check(goal: &[String], seen: &Observed) -> Vec<Verdict> {
    goal.iter().map(|p| verdict(p, seen)).collect()
}

fn verdict(predicate: &str, seen: &Observed) -> Verdict {
    let out = |ok: Option<bool>, detail: String| Verdict {
        predicate: predicate.to_owned(),
        ok,
        detail,
    };
    match predicate_of(predicate) {
        Some(("at", args)) if args.len() == 1 => at(args[0], seen, out),
        Some(("holding", args)) if args.len() == 2 => {
            let held = seen.state[format!("holding_{}", args[0])].as_str();
            match held {
                Some(h) => out(
                    Some(h == args[1]),
                    format!(
                        "the {} hand holds {}",
                        args[0],
                        if h.is_empty() { "nothing" } else { h }
                    ),
                ),
                None => out(None, "the executor's state is not available".to_owned()),
            }
        }
        Some((name @ ("inside" | "on"), args)) if args.len() == 2 => {
            let (Some(obj), Some(host)) = (
                object(&seen.objects, args[0]),
                object(&seen.objects, args[1]),
            ) else {
                return out(
                    None,
                    format!("{} or {} is not in the world model", args[0], args[1]),
                );
            };
            if obj["support_id"] == args[1] {
                return out(Some(true), format!("{} rests on {}", args[0], args[1]));
            }
            let (ox, oy) = xy(&obj["pose"]["position"]);
            let (hx, hy) = xy(&host["pose"]["position"]);
            let (sx, sy) = xy(&host["size"]);
            let within = (ox - hx).abs() <= sx / 2.0 + FOOTPRINT_SLACK_M
                && (oy - hy).abs() <= sy / 2.0 + FOOTPRINT_SLACK_M;
            out(
                Some(within),
                format!(
                    "{} is {}within {name}'s footprint",
                    args[0],
                    if within { "" } else { "not " }
                ),
            )
        }
        _ => out(None, "not a predicate NervROS can check".to_owned()),
    }
}

fn at(target: &str, seen: &Observed, out: impl Fn(Option<bool>, String) -> Verdict) -> Verdict {
    let Some((x, y)) = seen.pose else {
        return out(None, "the robot's pose is not available".to_owned());
    };
    let same = |a: &str| a.eq_ignore_ascii_case(target);
    if let Some(p) = seen
        .places
        .iter()
        .find(|p| same(&p.name) || p.aliases.iter().any(|a| same(a)))
    {
        let d = (p.pose.x - x).hypot(p.pose.y - y);
        return out(Some(d <= AT_PLACE_M), format!("{d:.2} m from {}", p.name));
    }
    let rooms = seen.rooms["rooms"]
        .as_array()
        .map_or(&[][..], Vec::as_slice);
    if let Some(room) = rooms
        .iter()
        .find(|r| r["id"] == target || r["name"].as_str().is_some_and(same))
    {
        let outline: Vec<(f64, f64)> = room["outline"]["points"]
            .as_array()
            .map(|pts| pts.iter().map(xy).collect())
            .unwrap_or_default();
        let ok = inside((x, y), &outline);
        return out(
            Some(ok),
            format!("the robot is {}in {target}", if ok { "" } else { "not " }),
        );
    }
    if let Some(o) = object(&seen.objects, target) {
        let (ox, oy) = xy(&o["pose"]["position"]);
        // From its edge: a walk up to a sofa stops beside it, two metres from its middle.
        let (sx, sy) = xy(&o["size"]);
        let d = ((ox - x).hypot(oy - y) - sx.max(sy) / 2.0).max(0.0);
        return out(
            Some(d <= AT_OBJECT_M),
            format!("{d:.2} m from {target}'s edge"),
        );
    }
    out(
        None,
        format!("{target} is not a known place, room or object"),
    )
}

/// Where and when the world model last saw what a failed step was about, such as "small white
/// mug O244: last seen 3 min ago in the office, on wooden tray O31". The model reads it before
/// it replans: a pick that found nothing is often a pick of something already moved.
#[must_use]
pub fn last_seen(step: &PlannedStep, seen: &Observed, now_s: f64) -> Option<String> {
    Some(describe_seen(step_object(step, seen)?, seen, now_s))
}

/// The world model's object a step is about, by its id, name or phrase arguments.
#[must_use]
pub fn step_object<'a>(step: &PlannedStep, seen: &'a Observed) -> Option<&'a Value> {
    let arg = |name: &str| super::plan::arg(&step.args, name);
    let keys: Vec<&str> = ["object_id", "container_id", "target", "place"]
        .iter()
        .filter_map(|k| arg(k))
        .collect();
    find_object(seen, &keys, arg("phrase"))
}

/// canopy's object history (`ObjectEvent`s, oldest first) as a line per object: "O12 mug:
/// appeared 2 h ago in R2; went missing 1 h ago; removed 58 min ago".
#[must_use]
pub fn story(events: &[Value], now_s: f64) -> Vec<String> {
    let mut lines: Vec<(String, String)> = Vec::new();
    for e in events {
        let id = text(e, "id");
        let ago = how_long(now_s - stamp_s(&e["stamp"]));
        let room = text(e, "room_id");
        let other = text(e, "other_id");
        let mut phrase = match text(e, "kind") {
            "appeared" => format!("appeared {ago} ago"),
            "moved" => format!("moved {ago} ago"),
            "missing" => format!("went missing {ago} ago"),
            "seen_again" => format!("seen again {ago} ago"),
            "removed" => format!("removed {ago} ago"),
            "merged" => format!("found to be part of {other} {ago} ago"),
            other => format!("{other} {ago} ago"),
        };
        if !room.is_empty() && matches!(text(e, "kind"), "appeared" | "moved") {
            let _ = write!(phrase, " in {room}");
        }
        if text(e, "kind") == "appeared" && !other.is_empty() {
            let _ = write!(phrase, ", maybe {other} moved there");
        }
        if !text(e, "detail").is_empty() {
            let _ = write!(phrase, " ({})", text(e, "detail"));
        }
        match lines.iter_mut().find(|(i, _)| i == id) {
            Some((_, line)) => {
                let _ = write!(line, "; {phrase}");
            }
            None => lines.push((
                id.to_owned(),
                format!("{id} {}: {phrase}", text(e, "label")),
            )),
        }
    }
    lines.into_iter().map(|(_, line)| line).collect()
}

fn stamp_s(stamp: &Value) -> f64 {
    let sec = stamp["sec"].as_f64().unwrap_or(0.0);
    sec + stamp["nanosec"].as_f64().unwrap_or(0.0) * 1e-9
}

/// The world model's object that `keys` (ids or names such as `mug_4`) or `phrase` (the
/// operator's words) refer to: an exact id first, then words in a label or name, the most
/// recently seen winning.
#[must_use]
pub fn find_object<'a>(
    seen: &'a Observed,
    keys: &[&str],
    phrase: Option<&str>,
) -> Option<&'a Value> {
    let objects = seen.objects["objects"].as_array()?;
    keys.iter()
        .find_map(|k| {
            objects
                .iter()
                .find(|o| text(o, "id").eq_ignore_ascii_case(k))
        })
        .or_else(|| {
            phrase
                .map(str::to_owned)
                .into_iter()
                .chain(keys.iter().map(|k| words_of(k)))
                .filter(|w| !w.is_empty())
                .find_map(|w| {
                    objects
                        .iter()
                        .filter(|o| named(o, &w))
                        .max_by(|a, b| seen_at(a).total_cmp(&seen_at(b)))
                })
        })
}

/// Where and when the world model last saw `found`: "small white mug O244: last seen 3 min ago
/// in the office, on wooden tray O31".
#[must_use]
pub fn describe_seen(found: &Value, seen: &Observed, now_s: f64) -> String {
    let label = [text(found, "name"), text(found, "label")]
        .into_iter()
        .find(|n| !n.is_empty())
        .unwrap_or("object");
    let mut line = format!("{label} {}: last seen", text(found, "id"));
    let age = now_s - seen_at(found);
    if seen_at(found) > 0.0 && (0.0..86_400.0).contains(&age) {
        let _ = write!(line, " {} ago", how_long(age));
    }
    let room_id = text(found, "room_id");
    if !room_id.is_empty() {
        // A room's type says more than its name, which is often a letter ("room C").
        let room = seen.rooms["rooms"]
            .as_array()
            .and_then(|rooms| rooms.iter().find(|r| text(r, "id") == room_id));
        let place = room.map_or_else(
            || format!("in {room_id}"),
            |r| match (text(r, "type"), text(r, "name")) {
                ("", name) if name.starts_with("room ") => format!("in {name}"),
                ("", name) => format!("in the {name}"),
                (kind, _) => format!("in the {} ({room_id})", kind.replace('_', " ")),
            },
        );
        let _ = write!(line, " {place}");
    }
    let support_id = text(found, "support_id");
    let support = seen.objects["objects"].as_array().and_then(|objects| {
        objects
            .iter()
            .find(|o| !support_id.is_empty() && text(o, "id") == support_id)
    });
    if let Some(support) = support {
        let _ = write!(line, ", on {} {support_id}", text(support, "label"));
    }
    match found["state"].as_u64() {
        Some(1) => line.push_str("; looked for there since and not found"),
        Some(2) => line.push_str("; no longer there"),
        _ => {}
    }
    line
}

fn text<'a>(v: &'a Value, key: &str) -> &'a str {
    v[key].as_str().unwrap_or_default()
}

/// When an object was last seen, in seconds since the epoch; 0 when it never was.
fn seen_at(o: &Value) -> f64 {
    stamp_s(&o["last_seen"])
}

/// Whether an object's label or name is, or holds, `words`.
pub(crate) fn named(o: &Value, words: &str) -> bool {
    let words = words.to_lowercase();
    [text(o, "label"), text(o, "name")]
        .iter()
        .any(|n| !n.is_empty() && n.to_lowercase().contains(&words))
}

/// The words of a name such as `mug_4` or `small_white_mug`, without its number.
pub(crate) fn words_of(name: &str) -> String {
    name.split(['_', ' '])
        .filter(|p| !p.is_empty() && !p.chars().all(|c| c.is_ascii_digit()))
        .collect::<Vec<_>>()
        .join(" ")
}

/// A duration for people: "40 s", "12 min", "3 h".
pub(crate) fn how_long(seconds: f64) -> String {
    match seconds {
        s if s < 90.0 => format!("{s:.0} s"),
        s if s < 90.0 * 60.0 => format!("{:.0} min", s / 60.0),
        s => format!("{:.0} h", s / 3600.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::plan::StepArg;
    use crate::profile::PlacePose;
    use serde_json::json;

    fn seen() -> Observed {
        Observed {
            pose: Some((1.0, 1.0)),
            places: vec![PlaceConfig {
                near: Vec::new(),
                name: "dock".to_owned(),
                aliases: vec![],
                frame: "map".to_owned(),
                pose: PlacePose {
                    x: 1.2,
                    y: 1.1,
                    yaw: 0.0,
                },
            }],
            rooms: json!({"rooms": [{"id": "R1", "name": "kitchen", "outline": {"points": [
                {"x": 0.0, "y": 0.0}, {"x": 4.0, "y": 0.0}, {"x": 4.0, "y": 4.0}, {"x": 0.0, "y": 4.0}]}}]}),
            objects: json!({"objects": [
                {"id": "O17", "support_id": "", "pose": {"position": {"x": 3.15, "y": 3.02}}, "size": {"x": 0.1, "y": 0.1}},
                {"id": "O31", "support_id": "", "pose": {"position": {"x": 3.0, "y": 3.0}}, "size": {"x": 0.4, "y": 0.3}}]}),
            state: json!({"holding_left": "", "holding_right": "O17"}),
        }
    }

    #[test]
    fn an_objects_story_reads_as_a_line_per_object_and_a_move_links_them() {
        let event = |id: &str, kind: &str, ago: f64, room: &str, other: &str| {
            json!({"id": id, "label": "mug", "kind": kind, "stamp": {"sec": 10_000.0 - ago, "nanosec": 0},
                   "room_id": room, "other_id": other, "detail": ""})
        };
        let events = [
            event("O12", "appeared", 7200.0, "R2", ""),
            event("O12", "missing", 3600.0, "R2", ""),
            event("O31", "appeared", 2400.0, "R3", "O12"),
            event("O12", "removed", 1800.0, "R2", ""),
        ];
        assert_eq!(
            story(&events, 10_000.0),
            [
                "O12 mug: appeared 2 h ago in R2; went missing 60 min ago; removed 30 min ago",
                "O31 mug: appeared 40 min ago in R3, maybe O12 moved there",
            ]
        );
    }

    #[test]
    fn predicates_are_checked_against_the_world() {
        let goal: Vec<String> = [
            "at(dock)",
            "at(kitchen)",
            "at(O31)",
            "holding(right, O17)",
            "holding(left, O17)",
            "inside(O17, O31)",
            "on(O31, O17)",
            "near(O31)",
        ]
        .map(str::to_owned)
        .to_vec();
        let v = check(&goal, &seen());
        let oks: Vec<Option<bool>> = v.iter().map(|v| v.ok).collect();
        assert_eq!(
            oks,
            [
                Some(true),
                Some(true),
                Some(false),
                Some(true),
                Some(false),
                Some(true),
                Some(false),
                None
            ]
        );
    }

    #[test]
    fn a_failed_pick_says_where_the_object_was_last_seen_and_on_what() {
        let mut seen = seen();
        seen.rooms = json!({"rooms": [{"id": "R3", "name": "office"}]});
        seen.objects = json!({"objects": [
            {"id": "O31", "label": "wooden tray", "room_id": "R3", "last_seen": {"sec": 1000, "nanosec": 0}},
            {"id": "O244", "label": "small white mug", "room_id": "R3", "support_id": "O31",
             "state": 0, "last_seen": {"sec": 880, "nanosec": 0}},
            {"id": "O9", "label": "mug", "last_seen": {"sec": 100, "nanosec": 0}}]});
        let step = PlannedStep {
            id: "s2".to_owned(),
            skill: "PickObject".to_owned(),
            summary: String::new(),
            args: vec![
                StepArg {
                    name: "object_id".to_owned(),
                    value: "mug_4".to_owned(),
                },
                StepArg {
                    name: "phrase".to_owned(),
                    value: "small white mug".to_owned(),
                },
            ],
            ..PlannedStep::default()
        };
        assert_eq!(
            last_seen(&step, &seen, 1000.0).as_deref(),
            Some("small white mug O244: last seen 2 min ago in the office, on wooden tray O31")
        );
        seen.rooms = json!({"rooms": [{"id": "R3", "name": "room A", "type": "living_room"}]});
        let typed = last_seen(&step, &seen, 1000.0).unwrap_or_default();
        assert!(typed.contains(" in the living room (R3),"), "{typed}");

        let by_id = PlannedStep {
            args: vec![StepArg {
                name: "container_id".to_owned(),
                value: "o31".to_owned(),
            }],
            ..step.clone()
        };
        assert!(
            last_seen(&by_id, &seen, 1000.0)
                .unwrap()
                .starts_with("wooden tray O31")
        );

        let unknown = PlannedStep {
            args: vec![StepArg {
                name: "object_id".to_owned(),
                value: "kettle_2".to_owned(),
            }],
            ..step
        };
        assert_eq!(last_seen(&unknown, &seen, 1000.0), None);
    }
}
