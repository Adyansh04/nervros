//! Whether a plan sends the robot into a keep-out zone, checked before the executor or the
//! operator sees it: where each walk to a place ends, and what a straight walk passes through,
//! followed from where the robot stands.
//!
//! A turn changes only the heading. Any other skill that takes the base, as a pick backing off a
//! table does, leaves the pose unknown until the next walk to a place, and a straight walk from an
//! unknown pose is left to Nav2's keepout filter, which stops it at the zone's edge.

use super::catalog::{Catalog, GO_TO_PLACE, GO_TO_POSE, GO_TO_TARGET, WALK_ROUTE};
use super::plan::{PlannedStep, Problem, World, arg};
use crate::profile::PlanChecks;
use crate::zones::Zones;

/// The robot on the map: x, y and its heading in radians, when known.
type Pose = (f64, f64, Option<f64>);

fn number(step: &PlannedStep, name: &str) -> Option<f64> {
    arg(&step.args, name)?.parse().ok()
}

/// What a walk to a place or a target heads for: a place's name, an id, or a label.
fn target(step: &PlannedStep) -> Option<String> {
    if step.skill == GO_TO_TARGET {
        arg(&step.args, "target").map(str::to_owned)
    } else {
        step.target.clone()
    }
}

/// Where a walk to a place, a pose or a target leaves the robot.
fn destination(step: &PlannedStep, world: &World) -> Option<Pose> {
    if step.skill == GO_TO_POSE {
        let station = arg(&step.args, "station")?;
        let v: Vec<f64> = station
            .split(';')
            .map(|p| p.trim().parse().ok())
            .collect::<Option<_>>()?;
        return match v.as_slice() {
            [x, y, yaw] => Some((*x, *y, Some(*yaw))),
            _ => None,
        };
    }
    let target = target(step)?;
    if let Some(p) = world.places.iter().find(|p| p.name == target) {
        return Some((p.pose.x, p.pose.y, Some(p.pose.yaw)));
    }
    // A room or an object: the robot stops near it, facing it, its heading unknown here.
    let thing = world
        .rooms
        .iter()
        .chain(&world.objects)
        .find(|t| t.id == target)?;
    thing.xy.map(|(x, y)| (x, y, None))
}

/// Where a route ends, each place it passes checked against the zones.
fn route(
    step: &PlannedStep,
    world: &World,
    zones: &Zones,
    problems: &mut Vec<Problem>,
) -> Option<Pose> {
    let mut pose = None;
    let names = arg(&step.args, "places").unwrap_or_default();
    for name in names
        .split([',', ';'])
        .map(str::trim)
        .filter(|n| !n.is_empty())
    {
        pose = waypoint(name, world);
        if let Some((x, y, _)) = pose
            && let Some(zone) = zones.at(x, y)
        {
            problems.push(Problem::new(
                &step.id,
                "places",
                format!(
                    "{} passes through `{name}`, inside the keep-out zone `{}`: leave it out, or \
                     ask the operator to remove the zone",
                    step.summary, zone.name
                ),
            ));
        }
    }
    pose
}

/// Where a route passes `name`: a place, or a room's centre.
fn waypoint(name: &str, world: &World) -> Option<Pose> {
    let same = |a: &str| crate::places::same_words(a, name);
    let place = world
        .places
        .iter()
        .find(|p| same(&p.name) || p.aliases.iter().any(|a| same(a)));
    if let Some(p) = place {
        return Some((p.pose.x, p.pose.y, Some(p.pose.yaw)));
    }
    let room = world.rooms.iter().find(|r| r.id == name || same(&r.name))?;
    room.xy.map(|(x, y)| (x, y, None))
}

/// Every step of `steps` that would take the robot into a zone, from `start` (x, y, heading).
pub(crate) fn check(
    steps: &[PlannedStep],
    start: Option<(f64, f64, f64)>,
    world: &World,
    catalog: &Catalog,
    checks: &PlanChecks,
    zones: &Zones,
) -> Vec<Problem> {
    let mut problems = Vec::new();
    let mut pose: Option<Pose> = start.map(|(x, y, yaw)| (x, y, Some(yaw)));
    for step in steps {
        if [GO_TO_PLACE, GO_TO_POSE, GO_TO_TARGET].contains(&step.skill.as_str()) {
            pose = destination(step, world);
            if let Some((x, y, _)) = pose
                && let Some(zone) = zones.at(x, y)
            {
                problems.push(Problem::new(
                    &step.id,
                    "",
                    format!(
                        "{} ends inside the keep-out zone `{}`: go somewhere outside it, or ask \
                         the operator to remove the zone",
                        step.summary, zone.name
                    ),
                ));
            }
            // A label several objects share: the walk ends at the nearest, known only as it runs.
            let label = target(step).filter(|_| pose.is_none()).unwrap_or_default();
            let inside = world
                .objects
                .iter()
                .filter(|o| !label.is_empty() && o.name.eq_ignore_ascii_case(&label))
                .find_map(|o| {
                    let (x, y) = o.xy?;
                    zones.at(x, y).map(|zone| (o, zone))
                });
            if let Some((object, zone)) = inside {
                problems.push(Problem::new(
                    &step.id,
                    "",
                    format!(
                        "{} may end at {}, inside the keep-out zone `{}`: name one outside it by \
                         its id, or ask the operator to remove the zone",
                        step.summary, object.id, zone.name
                    ),
                ));
            }
            continue;
        }
        if step.skill == WALK_ROUTE {
            pose = route(step, world, zones, &mut problems);
            continue;
        }
        if let Some(walk) = checks.walk.as_ref().filter(|w| w.skill == step.skill) {
            let back = arg(&step.args, &walk.direction).is_some_and(|d| d.starts_with("back"));
            pose = match (pose, number(step, &walk.metres)) {
                (Some((x, y, Some(yaw))), Some(metres)) if metres > 0.0 => {
                    let along = if back { -metres } else { metres };
                    let end = (x + along * yaw.cos(), y + along * yaw.sin());
                    if let Some((zone, d)) = zones.crossed((x, y), end) {
                        let why = if zone.contains(x, y) && zone.contains(end.0, end.1) {
                            format!(
                                "{} stays inside the keep-out zone `{}`, where the robot stands: \
                                 walk it out",
                                step.summary, zone.name
                            )
                        } else {
                            format!(
                                "{} would enter the keep-out zone `{}` after {d:.1} m: walk less, \
                                 or around it",
                                step.summary, zone.name
                            )
                        };
                        problems.push(Problem::new(&step.id, &walk.metres, why));
                    }
                    Some((end.0, end.1, Some(yaw)))
                }
                _ => None,
            };
            continue;
        }
        if let Some(turn) = checks.turn.as_ref().filter(|t| t.skill == step.skill) {
            pose = pose.map(|(x, y, yaw)| {
                let left =
                    number(step, &turn.degrees).map(|d| if turn.positive_left { d } else { -d });
                (x, y, yaw.zip(left).map(|(h, d)| h + d.to_radians()))
            });
            continue;
        }
        if catalog
            .skill(&step.skill)
            .is_some_and(|s| s.resources.iter().any(|r| r == "base"))
        {
            pose = None;
        }
    }
    problems
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::plan::{StepArg, Thing};
    use crate::profile::{PlaceConfig, PlacePose, TurnSkill, WalkSkill};
    use crate::zones::Zone;

    fn step(id: &str, skill: &str, args: &[(&str, &str)], target: Option<&str>) -> PlannedStep {
        PlannedStep {
            id: id.into(),
            skill: skill.into(),
            summary: format!(
                "{skill}({})",
                args.iter().map(|(_, v)| *v).collect::<Vec<_>>().join(", ")
            ),
            args: args
                .iter()
                .map(|(n, v)| StepArg {
                    name: (*n).into(),
                    value: (*v).into(),
                })
                .collect(),
            target: target.map(Into::into),
            ..PlannedStep::default()
        }
    }

    fn checks() -> PlanChecks {
        PlanChecks {
            walk: Some(WalkSkill {
                skill: "WalkStraight".into(),
                metres: "distance_m".into(),
                direction: "direction".into(),
                max_m: 2.0,
            }),
            turn: Some(TurnSkill {
                skill: "TurnInPlace".into(),
                degrees: "degrees".into(),
                positive_left: true,
            }),
            arm: None,
        }
    }

    fn zones() -> std::sync::Arc<Zones> {
        let profile: crate::profile::Profile =
            toml::from_str("[robot]\nname = \"x\"\n[models]\nfile = \"m.toml\"\n").unwrap();
        let zones = Zones::new(&profile, None);
        // A strip from x = 2 to 3 across the robot's way, and the kitchen around O17.
        zones
            .add(Zone {
                name: "stairs".into(),
                polygon: vec![[2.0, -5.0], [3.0, -5.0], [3.0, 5.0], [2.0, 5.0]],
                rule: crate::zones::Rule::KeepOut,
                fixed: false,
            })
            .unwrap();
        zones
            .add(Zone {
                name: "kitchen".into(),
                polygon: vec![[10.0, 10.0], [12.0, 10.0], [12.0, 12.0], [10.0, 12.0]],
                rule: crate::zones::Rule::KeepOut,
                fixed: false,
            })
            .unwrap();
        zones
    }

    fn world() -> World {
        World {
            places: vec![PlaceConfig {
                name: "dock".into(),
                aliases: Vec::new(),
                frame: "map".into(),
                pose: PlacePose {
                    x: 0.0,
                    y: 0.0,
                    yaw: std::f64::consts::PI,
                },
                near: Vec::new(),
            }],
            objects: vec![Thing {
                id: "O17".into(),
                name: "red mug".into(),
                xy: Some((11.0, 11.0)),
            }],
            ..World::default()
        }
    }

    fn catalog() -> Catalog {
        serde_json::from_value(serde_json::json!({"catalog_version": "t", "skills": []})).unwrap()
    }

    #[test]
    fn a_route_through_a_place_in_a_zone_is_refused() {
        let mut world = world();
        world.places.push(PlaceConfig {
            name: "landing".into(),
            aliases: Vec::new(),
            frame: "map".into(),
            pose: PlacePose {
                x: 2.5,
                y: 0.0,
                yaw: 0.0,
            },
            near: Vec::new(),
        });
        let route = [step("s1", WALK_ROUTE, &[("places", "dock, landing")], None)];
        let problems = check(&route, None, &world, &catalog(), &checks(), &zones());
        assert_eq!(problems.len(), 1, "{problems:?}");
        assert!(
            problems[0]
                .message
                .contains("`landing`, inside the keep-out zone `stairs`")
        );
        let fine = [step("s1", WALK_ROUTE, &[("places", "dock")], None)];
        assert!(check(&fine, None, &world, &catalog(), &checks(), &zones()).is_empty());
    }

    #[test]
    fn a_walk_through_a_zone_is_refused_and_one_short_of_it_is_not() {
        let start = Some((0.0, 0.0, 0.0));
        let far = [step(
            "s1",
            "WalkStraight",
            &[("direction", "forward"), ("distance_m", "2.5")],
            None,
        )];
        let problems = check(&far, start, &world(), &catalog(), &checks(), &zones());
        assert_eq!(problems.len(), 1, "{problems:?}");
        assert!(
            problems[0].message.contains("`stairs` after 2.0 m"),
            "{}",
            problems[0].message
        );
        let short = [step(
            "s1",
            "WalkStraight",
            &[("direction", "forward"), ("distance_m", "1.5")],
            None,
        )];
        assert!(check(&short, start, &world(), &catalog(), &checks(), &zones()).is_empty());
        // Turned round, the same walk goes away from the stairs.
        let turned = [
            step("s1", "TurnInPlace", &[("degrees", "180")], None),
            step(
                "s2",
                "WalkStraight",
                &[("direction", "forward"), ("distance_m", "2.5")],
                None,
            ),
        ];
        assert!(check(&turned, start, &world(), &catalog(), &checks(), &zones()).is_empty());
        let back = [step(
            "s1",
            "WalkStraight",
            &[("direction", "backward"), ("distance_m", "2.5")],
            None,
        )];
        assert!(check(&back, start, &world(), &catalog(), &checks(), &zones()).is_empty());
    }

    #[test]
    fn a_walk_to_a_label_is_refused_when_any_object_it_names_is_in_a_zone() {
        let mut world = world();
        world.objects.push(Thing {
            id: "O31".into(),
            name: "Red mug".into(),
            xy: Some((1.0, 1.0)),
        });
        let to_mugs = [step(
            "s1",
            "GoToPlace",
            &[("place", "red mug")],
            Some("red mug"),
        )];
        let problems = check(&to_mugs, None, &world, &catalog(), &checks(), &zones());
        assert_eq!(problems.len(), 1, "{problems:?}");
        assert!(
            problems[0]
                .message
                .contains("may end at O17, inside the keep-out zone `kitchen`"),
            "{}",
            problems[0].message
        );
        world.objects.remove(0);
        assert!(check(&to_mugs, None, &world, &catalog(), &checks(), &zones()).is_empty());
    }

    #[test]
    fn a_robot_standing_in_a_zone_may_walk_out_of_it() {
        let inside = Some((2.5, 0.0, 0.0));
        let out = [step(
            "s1",
            "WalkStraight",
            &[("direction", "forward"), ("distance_m", "1.0")],
            None,
        )];
        assert!(check(&out, inside, &world(), &catalog(), &checks(), &zones()).is_empty());
        let stay = [step(
            "s1",
            "WalkStraight",
            &[("direction", "forward"), ("distance_m", "0.2")],
            None,
        )];
        let problems = check(&stay, inside, &world(), &catalog(), &checks(), &zones());
        assert!(
            problems[0]
                .message
                .contains("stays inside the keep-out zone `stairs`"),
            "{problems:?}"
        );
    }

    #[test]
    fn a_walk_to_something_in_a_zone_is_refused() {
        let to_mug = [step("s1", "GoToPlace", &[("place", "O17")], Some("O17"))];
        let problems = check(&to_mug, None, &world(), &catalog(), &checks(), &zones());
        assert!(problems[0].message.contains("`kitchen`"), "{problems:?}");
        // The dock faces back towards x < 0: a walk from it heads away from the stairs.
        let from_dock = [
            step("s1", "GoToPlace", &[("place", "dock")], Some("dock")),
            step(
                "s2",
                "WalkStraight",
                &[("direction", "forward"), ("distance_m", "2")],
                None,
            ),
        ];
        assert!(check(&from_dock, None, &world(), &catalog(), &checks(), &zones()).is_empty());
        let pose = [step("s1", "GoToPose", &[("station", "2.5;0;0")], None)];
        assert_eq!(
            check(&pose, None, &world(), &catalog(), &checks(), &zones()).len(),
            1
        );
    }
}
