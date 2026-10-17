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
/// How much lower a support's middle is than what rests on it: a mug's box in a tray can share
/// the tray's height.
const SUPPORT_BELOW_M: f64 = 0.1;

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

/// Whether [`check`] can judge `predicate`: `at` of one thing, or `holding`, `inside` or `on` of
/// two.
#[must_use]
pub fn checkable(predicate: &str) -> bool {
    let Some((name, args)) = predicate_of(predicate) else {
        return false;
    };
    let arity = match name {
        "at" => 1,
        "holding" | "inside" | "on" => 2,
        _ => return false,
    };
    // `holding` is judged by the executor's hands, which are left and right.
    args.len() == arity
        && args.iter().all(|a| !a.is_empty())
        && (name != "holding" || matches!(args[0], "left" | "right"))
}

/// Checks each predicate. The world model gives an object it saw moved another id, a new one or
/// that of an old entry where it now stands, so an `inside` or `on` whose object is not there also
/// holds when an object of its label is, seen there after the object itself was last seen: once
/// the object is gone from where it was, or when the one there was seen at `since_s` or later, as
/// after the mission or the goal that put it there.
#[must_use]
pub fn check(goal: &[String], seen: &Observed, since_s: f64) -> Vec<Verdict> {
    goal.iter().map(|p| verdict(p, seen, since_s)).collect()
}

fn verdict(predicate: &str, seen: &Observed, since_s: f64) -> Verdict {
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
        Some(("inside" | "on", args)) if args.len() == 2 => {
            let (Some(obj), Some(host)) = (
                object(&seen.objects, args[0]),
                object(&seen.objects, args[1]),
            ) else {
                return out(
                    None,
                    format!("{} or {} is not in the world model", args[0], args[1]),
                );
            };
            if let Some(how) = placed(obj, host) {
                return out(Some(true), how.said(args[0], args[1]));
            }
            if let Some((moved, how)) = moved_as(&seen.objects, obj, host, since_s) {
                let moved = format!(
                    "{}, a {} seen there after {} was,",
                    text(moved, "id"),
                    text(obj, "label"),
                    args[0]
                );
                return out(
                    Some(true),
                    format!(
                        "{}: the world model gives a moved object another id",
                        how.said(&moved, args[1])
                    ),
                );
            }
            out(
                Some(false),
                format!("{} is not within {}'s footprint", args[0], args[1]),
            )
        }
        _ => out(None, "not a predicate NervROS can check".to_owned()),
    }
}

/// How an object is in or on another.
#[derive(Debug, Clone, Copy)]
enum Placed {
    /// The world model says it rests on it.
    On,
    /// It stands within its footprint.
    Within,
}

impl Placed {
    fn said(self, thing: &str, host: &str) -> String {
        match self {
            Self::On => format!("{thing} rests on {host}"),
            Self::Within => format!("{thing} is within {host}'s footprint"),
        }
    }
}

/// How `obj` is in or on `host`; none when it is not.
fn placed(obj: &Value, host: &Value) -> Option<Placed> {
    if obj["support_id"] == host["id"] {
        return Some(Placed::On);
    }
    let (ox, oy) = xy(&obj["pose"]["position"]);
    let (hx, hy) = xy(&host["pose"]["position"]);
    let (sx, sy) = xy(&host["size"]);
    ((ox - hx).abs() <= sx / 2.0 + FOOTPRINT_SLACK_M
        && (oy - hy).abs() <= sy / 2.0 + FOOTPRINT_SLACK_M)
        .then_some(Placed::Within)
}

/// The object of `obj`'s label in or on `host`, seen there after `obj` was last seen, the latest
/// seen: `obj` moved, under the id the world model gave it there. It counts once `obj` is gone
/// from where it was, or when it was seen there at `since_s` or later; while `obj` is still seen
/// where it was, the one in `host` is another object.
fn moved_as<'v>(
    objects: &'v Value,
    obj: &Value,
    host: &Value,
    since_s: f64,
) -> Option<(&'v Value, Placed)> {
    let label = text(obj, "label");
    if label.is_empty() {
        return None;
    }
    let gone = obj["state"].as_u64().unwrap_or(0) != 0;
    objects["objects"]
        .as_array()?
        .iter()
        .filter(|o| {
            text(o, "label").eq_ignore_ascii_case(label)
                && o["id"] != obj["id"]
                && o["state"].as_u64() == Some(0)
                && seen_at(o) > seen_at(obj)
                && (gone || seen_at(o) >= since_s)
        })
        .filter_map(|o| Some((o, placed(o, host)?)))
        .max_by(|(a, _), (b, _)| seen_at(a).total_cmp(&seen_at(b)))
}

/// The profile's place called `target`, by its name or an alias.
fn place<'s>(seen: &'s Observed, target: &str) -> Option<&'s PlaceConfig> {
    let same = |a: &str| a.eq_ignore_ascii_case(target);
    seen.places
        .iter()
        .find(|p| same(&p.name) || p.aliases.iter().any(|a| same(a)))
}

/// The world model's room `target`, by its id or name.
fn room<'s>(seen: &'s Observed, target: &str) -> Option<&'s Value> {
    seen.rooms["rooms"].as_array()?.iter().find(|r| {
        r["id"] == target
            || r["name"]
                .as_str()
                .is_some_and(|n| n.eq_ignore_ascii_case(target))
    })
}

/// The arguments of `predicate` that [`check`] cannot find in what is `seen`: objects the world
/// model does not know, or for `at` a place, room or object; none when it finds them all.
#[must_use]
pub fn unknown_in<'p>(predicate: &'p str, seen: &Observed) -> Vec<&'p str> {
    let Some((name, args)) = predicate_of(predicate) else {
        return Vec::new();
    };
    let known = |id: &str| object(&seen.objects, id).is_some();
    match (name, args.as_slice()) {
        ("at", [t]) if place(seen, t).is_none() && room(seen, t).is_none() && !known(t) => {
            vec![*t]
        }
        ("inside" | "on", [o, h]) => [*o, *h].into_iter().filter(|id| !known(id)).collect(),
        _ => Vec::new(),
    }
}

/// `predicate` with each argument `names` maps renamed: `inside(mug_4, tray_1)` as the world
/// model's `inside(O27, O261)`.
#[must_use]
pub fn renamed(predicate: &str, names: &[(&str, String)]) -> String {
    let Some((name, args)) = predicate_of(predicate) else {
        return predicate.to_owned();
    };
    let args: Vec<&str> = args
        .iter()
        .map(|a| {
            names
                .iter()
                .find(|(from, _)| from == a)
                .map_or(*a, |(_, to)| to.as_str())
        })
        .collect();
    format!("{name}({})", args.join(", "))
}

/// The world model's object that `id`'s words likeliest name, by its id and as `O27 (mug, last
/// seen 2 min ago in the office, on desk O47)`: what a detector's own name, such as `mug_4`, is
/// likely called there. A live one seen last wins: a stale one is rarely the one meant.
#[must_use]
pub fn likeliest(seen: &Observed, id: &str, now_s: f64) -> Option<(String, String)> {
    let words: Vec<String> = words_of(id)
        .split_whitespace()
        .filter(|w| w.len() > 2)
        .map(str::to_ascii_lowercase)
        .collect();
    let gone = |o: &Value| o["state"].as_u64().is_some_and(|s| s != 0);
    let best = seen.objects["objects"]
        .as_array()?
        .iter()
        .filter(|o| {
            let label = text(o, "label").to_ascii_lowercase();
            words.iter().any(|w| label.contains(w.as_str()))
        })
        .min_by(|a, b| {
            gone(a)
                .cmp(&gone(b))
                .then(seen_at(b).total_cmp(&seen_at(a)))
        })?;
    let described = describe_seen(best, seen, now_s);
    let (what, when) = described
        .split_once(": last seen")
        .unwrap_or((&described, ""));
    let what = what.rsplit_once(' ').map_or(what, |(w, _)| w);
    let id = text(best, "id");
    let said = match when.trim() {
        "" => format!("{id} ({what})"),
        when => format!("{id} ({what}, last seen {when})"),
    };
    Some((id.to_owned(), said))
}

fn at(target: &str, seen: &Observed, out: impl Fn(Option<bool>, String) -> Verdict) -> Verdict {
    let Some((x, y)) = seen.pose else {
        return out(None, "the robot's pose is not available".to_owned());
    };
    if let Some(p) = place(seen, target) {
        let d = (p.pose.x - x).hypot(p.pose.y - y);
        return out(Some(d <= AT_PLACE_M), format!("{d:.2} m from {}", p.name));
    }
    if let Some(room) = room(seen, target) {
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
/// operator's words) refer to: an exact id first, then the phrase in a label or name, the most
/// recently seen winning, then a key's words only when they name one object: `mug_4` is not
/// whichever mug the camera saw last.
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
            let phrase = phrase.filter(|p| !p.is_empty())?;
            objects
                .iter()
                .filter(|o| named(o, phrase))
                .max_by(|a, b| seen_at(a).total_cmp(&seen_at(b)))
        })
        .or_else(|| {
            keys.iter()
                .map(|k| words_of(k))
                .filter(|w| !w.is_empty())
                .find_map(|w| {
                    let mut found = objects.iter().filter(|o| named(o, &w));
                    let first = found.next()?;
                    found.next().is_none().then_some(first)
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
        let _ = write!(line, " in {}", room_said(seen, room_id));
    }
    if let Some(support) = support_of(found, seen) {
        let _ = write!(
            line,
            ", on {} {}",
            text(support, "label"),
            text(support, "id")
        );
    }
    match found["state"].as_u64() {
        Some(1) => line.push_str("; looked for there since and not found"),
        Some(2) => line.push_str("; no longer there"),
        _ => {}
    }
    line
}

/// Room `room_id` as the operator reads it: "the living room (R1)", or its name when it has no
/// type, as its name is often a letter ("room C").
#[must_use]
pub fn room_said(seen: &Observed, room_id: &str) -> String {
    let room = seen.rooms["rooms"]
        .as_array()
        .and_then(|rooms| rooms.iter().find(|r| text(r, "id") == room_id));
    room.map_or_else(
        || room_id.to_owned(),
        |r| match (text(r, "type"), text(r, "name")) {
            ("", name) if name.starts_with("room ") => name.to_owned(),
            ("", name) => format!("the {name}"),
            (kind, _) => format!("the {} ({room_id})", kind.replace('_', " ")),
        },
    )
}

/// The id of the room the robot stands in, by the rooms' outlines; none off them or unknown.
#[must_use]
pub fn room_of_robot(seen: &Observed) -> Option<&str> {
    let pose = seen.pose?;
    seen.rooms["rooms"].as_array()?.iter().find_map(|r| {
        let outline: Vec<(f64, f64)> = r["outline"]["points"]
            .as_array()
            .map(|pts| pts.iter().map(xy).collect())
            .unwrap_or_default();
        inside(pose, &outline).then(|| text(r, "id"))
    })
}

/// What `found` rests on: the world model's support, else the smallest object whose footprint holds
/// it and whose middle is [`SUPPORT_BELOW_M`] lower, as a table under a mug: which of three mugs
/// is the one on the dining table is how a model tells them apart.
fn support_of<'a>(found: &Value, seen: &'a Observed) -> Option<&'a Value> {
    let objects = seen.objects["objects"].as_array()?;
    let support_id = text(found, "support_id");
    if !support_id.is_empty() {
        return objects.iter().find(|o| text(o, "id") == support_id);
    }
    let (x, y) = xy(&found["pose"]["position"]);
    let z = found["pose"]["position"]["z"].as_f64()?;
    let area = |o: &Value| {
        let (sx, sy) = xy(&o["size"]);
        sx * sy
    };
    objects
        .iter()
        .filter(|o| o["id"] != found["id"] && o["state"].as_u64().unwrap_or(0) == 0)
        .filter(|o| {
            let (hx, hy) = xy(&o["pose"]["position"]);
            let (sx, sy) = xy(&o["size"]);
            o["pose"]["position"]["z"]
                .as_f64()
                .is_some_and(|hz| hz <= z - SUPPORT_BELOW_M)
                && (x - hx).abs() <= sx / 2.0
                && (y - hy).abs() <= sy / 2.0
        })
        .min_by(|a, b| area(a).total_cmp(&area(b)))
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

/// A duration for people: "40 s", "12 min", "3 h", "5 days".
#[must_use]
pub fn how_long(seconds: f64) -> String {
    match seconds {
        s if s < 90.0 => format!("{s:.0} s"),
        s if s < 90.0 * 60.0 => format!("{:.0} min", s / 60.0),
        s if s < 48.0 * 3600.0 => format!("{:.0} h", s / 3600.0),
        s => format!("{:.0} days", s / 86_400.0),
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
        let v = check(&goal, &seen(), 0.0);
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
    fn an_object_seen_again_under_another_id_stands_in_for_the_one_moved() {
        let mug = |id: &str, x: f64, last: f64, state: u64| {
            json!({"id": id, "label": "mug", "state": state, "support_id": "",
                   "pose": {"position": {"x": x, "y": 3.0}}, "size": {"x": 0.1, "y": 0.1},
                   "last_seen": {"sec": last, "nanosec": 0}})
        };
        let mut seen = seen();
        seen.objects = json!({"objects": [
            {"id": "O31", "label": "tray", "pose": {"position": {"x": 3.0, "y": 3.0}},
             "size": {"x": 0.4, "y": 0.3}},
            mug("O17", 0.0, 550.0, 0),
            mug("O90", 3.05, 600.0, 0)]});
        let goal = ["inside(O17, O31)".to_owned()];
        let moved = &check(&goal, &seen, 500.0)[0];
        assert_eq!(
            moved.ok,
            Some(true),
            "seen there since the goal was set: {}",
            moved.detail
        );
        assert!(
            moved
                .detail
                .starts_with("O90, a mug seen there after O17 was, is within O31's footprint"),
            "{}",
            moved.detail
        );
        assert_eq!(
            check(&goal, &seen, 700.0)[0].ok,
            Some(false),
            "seen there only before the goal was set, with O17 not known to be gone"
        );
        seen.objects["objects"][1]["state"] = json!(1);
        assert_eq!(
            check(&goal, &seen, 700.0)[0].ok,
            Some(true),
            "O17 gone from where it was: the goal set after the move holds at once"
        );
        seen.objects["objects"][1]["last_seen"]["sec"] = json!(650.0);
        assert_eq!(
            check(&goal, &seen, 500.0)[0].ok,
            Some(false),
            "O17 seen elsewhere after the one in the tray: that one is another mug"
        );
        seen.objects["objects"][1]["last_seen"]["sec"] = json!(550.0);
        seen.objects["objects"][2]["state"] = json!(1);
        assert_eq!(
            check(&goal, &seen, 500.0)[0].ok,
            Some(false),
            "one gone from the tray too"
        );
    }

    #[test]
    fn an_object_with_no_support_known_is_on_the_smallest_thing_under_it() {
        let mut seen = seen();
        seen.objects = json!({"objects": [
            {"id": "O9", "label": "dining table", "state": 0,
             "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.4}}, "size": {"x": 2.0, "y": 1.2}},
            {"id": "O3", "label": "rug", "state": 0,
             "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}}, "size": {"x": 4.0, "y": 3.0}},
            {"id": "O244", "label": "mug", "state": 0, "support_id": "",
             "pose": {"position": {"x": 0.5, "y": 0.2, "z": 0.76}}, "size": {"x": 0.1, "y": 0.1}},
            {"id": "O27", "label": "mug", "state": 0, "support_id": "",
             "pose": {"position": {"x": 1.8, "y": 1.0, "z": 0.44}},
             "size": {"x": 0.1, "y": 0.1}}]});
        let objects = seen.objects["objects"].as_array().unwrap().clone();
        let on = |id: &str| {
            let found = objects.iter().find(|o| o["id"] == id).unwrap();
            support_of(found, &seen).map(|s| s["id"].as_str().unwrap().to_owned())
        };
        assert_eq!(
            on("O244").as_deref(),
            Some("O9"),
            "the table, not the rug under both"
        );
        assert_eq!(
            on("O27").as_deref(),
            Some("O3"),
            "off the table: only the rug holds it"
        );
        assert_eq!(on("O9").as_deref(), Some("O3"));
        let mut level = seen.clone();
        level.objects["objects"][0]["pose"]["position"]["z"] = json!(0.72);
        let found = &objects[2];
        assert_eq!(
            support_of(found, &level).map(|s| s["id"].clone()),
            Some(json!("O3")),
            "a box at about its height is no support"
        );
    }

    #[test]
    fn a_detectors_name_is_likeliest_the_live_object_seen_last() {
        let mut seen = seen();
        seen.objects = json!({"objects": [
            {"id": "O27", "label": "mug", "state": 0, "last_seen": {"sec": 100, "nanosec": 0}},
            {"id": "O46", "label": "mug", "state": 1, "last_seen": {"sec": 990, "nanosec": 0}},
            {"id": "O818", "label": "mug", "state": 0, "last_seen": {"sec": 880, "nanosec": 0}},
            {"id": "O9", "label": "dining table"}]});
        assert_eq!(
            likeliest(&seen, "mug_4", 1000.0),
            Some((
                "O818".to_owned(),
                "O818 (mug, last seen 2 min ago)".to_owned()
            )),
            "not O46, seen later but looked for there since and not found"
        );
        assert_eq!(likeliest(&seen, "kettle_2", 1000.0), None);
        assert_eq!(
            renamed("inside(mug_4, O31)", &[("mug_4", "O818".to_owned())]),
            "inside(O818, O31)"
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
            ..step.clone()
        };
        assert_eq!(last_seen(&unknown, &seen, 1000.0), None);

        // A phrase no label holds leaves the id's kind: naming one of two mugs would be a guess.
        let mut red = step;
        red.args[1].value = "red mug".to_owned();
        assert_eq!(last_seen(&red, &seen, 1000.0), None);
        seen.objects["objects"].as_array_mut().unwrap().truncate(2);
        assert!(
            last_seen(&red, &seen, 1000.0)
                .unwrap()
                .starts_with("small white mug O244")
        );
    }
}
