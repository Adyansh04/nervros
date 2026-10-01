//! Where a plan would take the robot, from the executor's `PreviewMission`: drawn in the viewer
//! beside the approval card, so the operator approves a picture of the walk, not only its words.

use serde::Serialize;
use serde_json::Value;

/// The most points a path keeps: Nav2 writes one every few centimetres, and the viewer needs the
/// shape, not every pose.
const MAX_POINTS: usize = 200;

/// One step's predicted end and path, in the map frame.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct PreviewStep {
    /// `s1`, `s2`, ...
    pub id: String,
    /// Where the base ends the step, `(x, y, yaw)`; none when it does not move the base or where
    /// it ends is decided as it goes.
    pub goal: Option<(f64, f64, f64)>,
    /// Nav2's planned path to `goal`, thinned; empty when none was planned.
    pub path: Vec<(f64, f64)>,
    /// For people, such as "walks 1.0 m forward" or "no path: ...".
    pub note: String,
}

/// The steps of a `PreviewMission` reply.
///
/// # Errors
///
/// The executor could not preview the plan; its reason.
pub fn parse(reply: &Value) -> Result<Vec<PreviewStep>, String> {
    if reply["ok"].as_bool() != Some(true) {
        return Err(reply["message"]
            .as_str()
            .filter(|m| !m.is_empty())
            .unwrap_or("the executor gave no preview")
            .to_owned());
    }
    Ok(reply["steps"]
        .as_array()
        .map(|steps| steps.iter().map(step).collect())
        .unwrap_or_default())
}

fn step(s: &Value) -> PreviewStep {
    let goal = (!s["goal"]["header"]["frame_id"]
        .as_str()
        .unwrap_or_default()
        .is_empty())
    .then(|| pose(&s["goal"]["pose"]))
    .flatten();
    let path: Vec<(f64, f64)> = s["path"]["poses"]
        .as_array()
        .map(|poses| {
            poses
                .iter()
                .filter_map(|p| {
                    let at = &p["pose"]["position"];
                    Some((at["x"].as_f64()?, at["y"].as_f64()?))
                })
                .collect()
        })
        .unwrap_or_default();
    PreviewStep {
        id: s["step_id"].as_str().unwrap_or_default().to_owned(),
        goal,
        path: thin(path),
        note: s["note"].as_str().unwrap_or_default().to_owned(),
    }
}

/// `(x, y, yaw)` of a `geometry_msgs/Pose`.
fn pose(p: &Value) -> Option<(f64, f64, f64)> {
    let (at, q) = (&p["position"], &p["orientation"]);
    let (qx, qy, qz, qw) = (
        q["x"].as_f64()?,
        q["y"].as_f64()?,
        q["z"].as_f64()?,
        q["w"].as_f64()?,
    );
    let yaw = (2.0 * (qw * qz + qx * qy)).atan2(1.0 - 2.0 * (qy * qy + qz * qz));
    Some((at["x"].as_f64()?, at["y"].as_f64()?, yaw))
}

/// At most [`MAX_POINTS`], evenly spread, always keeping both ends.
fn thin(points: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    if points.len() <= MAX_POINTS {
        return points;
    }
    let last = points.len() - 1;
    let step = last.div_ceil(MAX_POINTS - 1);
    let mut out: Vec<(f64, f64)> = points.iter().step_by(step).copied().collect();
    if out.last() != points.last() {
        out.push(points[last]);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::f64::consts::FRAC_1_SQRT_2;

    #[test]
    fn a_reply_becomes_steps_with_goals_and_thinned_paths() {
        let poses: Vec<Value> = (0..1000)
            .map(|i| json!({"pose": {"position": {"x": f64::from(i) * 0.01, "y": 0.0}}}))
            .collect();
        let reply = json!({"ok": true, "message": "", "steps": [
            {"step_id": "s1", "note": "turns 90 degrees left in place",
             "goal": {"header": {"frame_id": "map"}, "pose": {"position": {"x": 1.0, "y": 2.0},
                      "orientation": {"x": 0.0, "y": 0.0, "z": FRAC_1_SQRT_2, "w": FRAC_1_SQRT_2}}},
             "path": {"poses": []}},
            {"step_id": "s2", "note": "walks there along Nav2's path",
             "goal": {"header": {"frame_id": "map"}, "pose": {"position": {"x": 9.99, "y": 0.0},
                      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
             "path": {"poses": poses}},
            {"step_id": "s3", "note": "closes in on what it sees",
             "goal": {"header": {"frame_id": ""}}, "path": {"poses": []}}]});

        let steps = parse(&reply).unwrap();

        assert_eq!(steps.len(), 3);
        let (x, y, yaw) = steps[0].goal.unwrap();
        assert!((x - 1.0).abs() < 1e-9 && (y - 2.0).abs() < 1e-9);
        assert!((yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
        assert!(steps[1].path.len() <= MAX_POINTS);
        assert_eq!(steps[1].path.first(), Some(&(0.0, 0.0)));
        assert_eq!(steps[1].path.last(), Some(&(f64::from(999) * 0.01, 0.0)));
        assert_eq!(steps[2].goal, None);
    }

    #[test]
    fn a_refusal_is_its_reason() {
        assert_eq!(
            parse(&json!({"ok": false, "message": "no pose for base_footprint"})),
            Err("no pose for base_footprint".to_owned())
        );
    }
}
