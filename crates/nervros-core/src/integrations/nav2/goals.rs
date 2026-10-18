//! What Nav2's goals and messages ask of the base, as the keep-out checks read them.

use serde_json::Value;

/// Where a goal or message of some type sends the base.
#[derive(Debug, PartialEq)]
pub(crate) enum Destinations<'a> {
    /// The stamped poses it goes to, or plans a path to.
    Poses(Vec<&'a Value>),
    /// A move relative to where the robot stands: a straight drive or a back-up, which only a
    /// mission step can check against the zones.
    Relative,
    /// Not a move of the base.
    Elsewhere,
}

/// Where a goal or message of type `ty` with this payload sends the base: Nav2's pose goals and
/// paths, and a pose published for Nav2 to go to.
pub(crate) fn destinations<'a>(ty: &str, payload: &'a Value) -> Destinations<'a> {
    fn list(v: &Value) -> Vec<&Value> {
        v.as_array().map(|a| a.iter().collect()).unwrap_or_default()
    }
    match ty {
        "nav2_msgs/action/NavigateToPose" => Destinations::Poses(vec![&payload["pose"]]),
        "nav2_msgs/action/ComputePathToPose" => Destinations::Poses(vec![&payload["goal"]]),
        "nav2_msgs/action/NavigateThroughPoses" | "nav2_msgs/action/FollowWaypoints" => {
            Destinations::Poses(list(&payload["poses"]))
        }
        "nav2_msgs/action/ComputePathThroughPoses" => Destinations::Poses(list(&payload["goals"])),
        "nav2_msgs/action/FollowPath" => Destinations::Poses(list(&payload["path"]["poses"])),
        "geometry_msgs/msg/PoseStamped" => Destinations::Poses(vec![payload]),
        "nav2_msgs/action/DriveOnHeading" | "nav2_msgs/action/BackUp" => Destinations::Relative,
        _ => Destinations::Elsewhere,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn each_nav2_goal_names_where_it_sends_the_base() {
        let goal = json!({"pose": {"header": {"frame_id": "map"}}});
        assert_eq!(
            destinations("nav2_msgs/action/NavigateToPose", &goal),
            Destinations::Poses(vec![&goal["pose"]])
        );
        let route = json!({"poses": [{"pose": {}}, {"pose": {}}]});
        let Destinations::Poses(stops) = destinations("nav2_msgs/action/FollowWaypoints", &route)
        else {
            panic!("a waypoint round has poses");
        };
        assert_eq!(stops.len(), 2);
        assert_eq!(
            destinations("nav2_msgs/action/BackUp", &json!({})),
            Destinations::Relative
        );
        assert_eq!(
            destinations("nav2_msgs/action/Spin", &json!({})),
            Destinations::Elsewhere
        );
    }
}
