//! Goal checks after a mission reports success: the executor says the tree finished, these say
//! whether the world agrees.

use serde::Serialize;
use serde_json::Value;

use super::plan::{NEAR_M, predicate as predicate_of};
use crate::builtins::inside;
use crate::profile::PlaceConfig;

/// How far from a place's pose still counts as there.
const AT_PLACE_M: f64 = 0.5;
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

fn object<'v>(objects: &'v Value, id: &str) -> Option<&'v Value> {
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
        let d = (ox - x).hypot(oy - y);
        return out(Some(d <= NEAR_M), format!("{d:.2} m from {target}"));
    }
    out(
        None,
        format!("{target} is not a known place, room or object"),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::PlacePose;
    use serde_json::json;

    fn seen() -> Observed {
        Observed {
            pose: Some((1.0, 1.0)),
            places: vec![PlaceConfig {
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
}
