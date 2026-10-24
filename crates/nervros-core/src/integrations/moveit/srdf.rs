//! The robot's SRDF, as MoveIt reads it: its planning groups, end effectors and named poses.

use std::collections::BTreeMap;

/// One planning group.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct Group {
    /// Its name, such as `right_arm`.
    pub(crate) name: String,
    /// Its joints, when the SRDF lists them.
    pub(crate) joints: Vec<String>,
    /// Its chains, base link to tip link.
    pub(crate) chains: Vec<(String, String)>,
    /// Its links, when the SRDF lists them.
    pub(crate) links: Vec<String>,
}

/// A named pose of a group: `home`, `tucked`.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct NamedPose {
    /// Its name.
    pub(crate) name: String,
    /// Its group.
    pub(crate) group: String,
    /// Each joint's position, radians or metres.
    pub(crate) joints: BTreeMap<String, f64>,
}

/// An end effector: a hand or gripper group, and where it hangs.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct EndEffector {
    /// Its name.
    pub(crate) name: String,
    /// The group that is the hand.
    pub(crate) group: String,
    /// The link it hangs from: the arm's tip.
    pub(crate) parent_link: String,
    /// The arm group it hangs from, when the SRDF says.
    pub(crate) parent_group: String,
}

/// What the SRDF says.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct Srdf {
    /// The planning groups.
    pub(crate) groups: Vec<Group>,
    /// The named poses.
    pub(crate) poses: Vec<NamedPose>,
    /// The end effectors.
    pub(crate) end_effectors: Vec<EndEffector>,
    /// The frame a virtual joint ties the robot to, if any: where its plans are drawn.
    pub(crate) root_frame: Option<String>,
}

impl Srdf {
    /// Reads an SRDF document.
    ///
    /// # Errors
    ///
    /// The text is not XML, or not a `robot` element.
    pub(crate) fn parse(text: &str) -> Result<Self, String> {
        let doc =
            roxmltree::Document::parse(text).map_err(|e| format!("the SRDF is not XML: {e}"))?;
        let robot = doc.root_element();
        if robot.tag_name().name() != "robot" {
            return Err("the SRDF has no robot element".to_owned());
        }
        let attr =
            |n: roxmltree::Node<'_, '_>, a: &str| n.attribute(a).unwrap_or_default().to_owned();
        let mut srdf = Self::default();
        for node in robot.children().filter(roxmltree::Node::is_element) {
            match node.tag_name().name() {
                "group" => {
                    let mut group = Group {
                        name: attr(node, "name"),
                        ..Group::default()
                    };
                    for part in node.children().filter(roxmltree::Node::is_element) {
                        match part.tag_name().name() {
                            "joint" => group.joints.push(attr(part, "name")),
                            "link" => group.links.push(attr(part, "name")),
                            "chain" => group
                                .chains
                                .push((attr(part, "base_link"), attr(part, "tip_link"))),
                            _ => {}
                        }
                    }
                    srdf.groups.push(group);
                }
                "group_state" => {
                    let joints = node
                        .children()
                        .filter(|j| j.has_tag_name("joint"))
                        .filter_map(|j| {
                            Some((attr(j, "name"), j.attribute("value")?.trim().parse().ok()?))
                        })
                        .collect();
                    srdf.poses.push(NamedPose {
                        name: attr(node, "name"),
                        group: attr(node, "group"),
                        joints,
                    });
                }
                "end_effector" => srdf.end_effectors.push(EndEffector {
                    name: attr(node, "name"),
                    group: attr(node, "group"),
                    parent_link: attr(node, "parent_link"),
                    parent_group: attr(node, "parent_group"),
                }),
                "virtual_joint" => {
                    srdf.root_frame = node.attribute("parent_frame").map(str::to_owned);
                }
                _ => {}
            }
        }
        Ok(srdf)
    }

    /// Adds to each group the links its joints move, from the robot's URDF: an SRDF may give a
    /// hand by its joints alone. A URDF that does not parse adds nothing.
    #[must_use]
    pub(crate) fn with_urdf(mut self, urdf: &str) -> Self {
        let Ok(doc) = roxmltree::Document::parse(urdf) else {
            return self;
        };
        let moves: BTreeMap<&str, &str> = doc
            .root_element()
            .children()
            .filter(|n| n.has_tag_name("joint"))
            .filter_map(|j| {
                let child = j.children().find(|c| c.has_tag_name("child"))?;
                Some((j.attribute("name")?, child.attribute("link")?))
            })
            .collect();
        for group in &mut self.groups {
            for joint in &group.joints {
                if let Some(&link) = moves.get(joint.as_str())
                    && !group.links.iter().any(|l| l == link)
                {
                    group.links.push(link.to_owned());
                }
            }
        }
        self
    }

    /// The group called `name`.
    pub(crate) fn group(&self, name: &str) -> Option<&Group> {
        self.groups.iter().find(|g| g.name == name)
    }

    /// The named poses of `group`.
    pub(crate) fn poses_of(&self, group: &str) -> impl Iterator<Item = &NamedPose> {
        self.poses.iter().filter(move |p| p.group == group)
    }

    /// The link a group moves to a goal: the end effector hanging from it, else its chain's tip.
    pub(crate) fn tip(&self, group: &str) -> Option<String> {
        self.end_effectors
            .iter()
            .find(|e| e.parent_group == group)
            .map(|e| e.parent_link.clone())
            .or_else(|| {
                self.group(group)
                    .and_then(|g| g.chains.last())
                    .map(|(_, tip)| tip.clone())
            })
    }

    /// The links of the end effector hanging from `group`, which an object it holds may touch.
    pub(crate) fn hand_links(&self, group: &str) -> Vec<String> {
        self.end_effectors
            .iter()
            .filter(|e| e.parent_group == group)
            .filter_map(|e| self.group(&e.group))
            .flat_map(|g| g.links.clone())
            .collect()
    }

    /// The groups an arm move can be asked of: those with a chain or that an end effector hangs
    /// from, rather than hands or whole-body groups.
    pub(crate) fn arms(&self) -> Vec<&Group> {
        self.groups
            .iter()
            .filter(|g| {
                !g.chains.is_empty() || self.end_effectors.iter().any(|e| e.parent_group == g.name)
            })
            .collect()
    }

    /// The named pose a group is at, within `tolerance` of every joint the pose names, given each
    /// joint's position now.
    pub(crate) fn pose_at(
        &self,
        group: &str,
        now: &BTreeMap<String, f64>,
        tolerance: f64,
    ) -> Option<&NamedPose> {
        self.poses_of(group).find(|p| {
            !p.joints.is_empty()
                && p.joints
                    .iter()
                    .all(|(j, v)| now.get(j).is_some_and(|q| (q - v).abs() <= tolerance))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SRDF: &str = r#"<?xml version="1.0"?>
<robot name="arm">
  <virtual_joint name="world_joint" type="fixed" parent_frame="world" child_link="base"/>
  <group name="right_arm"><chain base_link="torso" tip_link="right_palm"/></group>
  <group name="right_hand"><link name="right_palm"/><link name="right_finger"/><joint name="right_finger_joint"/></group>
  <group_state name="home" group="right_arm"><joint name="shoulder" value="0"/><joint name="elbow" value="1.2"/></group_state>
  <group_state name="tucked" group="right_arm"><joint name="shoulder" value="-0.4"/><joint name="elbow" value="2.0"/></group_state>
  <group_state name="open" group="right_hand"><joint name="right_finger_joint" value="0.0"/></group_state>
  <end_effector name="right_gripper" parent_link="right_palm" group="right_hand" parent_group="right_arm"/>
  <disable_collisions link1="a" link2="b" reason="Never"/>
</robot>"#;

    #[test]
    fn the_srdf_gives_the_groups_their_poses_and_tips() {
        let srdf = Srdf::parse(SRDF).unwrap();
        assert_eq!(srdf.root_frame.as_deref(), Some("world"));
        let arms: Vec<&str> = srdf.arms().iter().map(|g| g.name.as_str()).collect();
        assert_eq!(arms, ["right_arm"]);
        assert_eq!(srdf.tip("right_arm").as_deref(), Some("right_palm"));
        assert_eq!(srdf.hand_links("right_arm"), ["right_palm", "right_finger"]);
        let names: Vec<&str> = srdf
            .poses_of("right_arm")
            .map(|p| p.name.as_str())
            .collect();
        assert_eq!(names, ["home", "tucked"]);
        let now = BTreeMap::from([("shoulder".to_owned(), -0.39), ("elbow".to_owned(), 2.02)]);
        assert_eq!(
            srdf.pose_at("right_arm", &now, 0.05)
                .map(|p| p.name.as_str()),
            Some("tucked")
        );
        assert!(srdf.pose_at("right_arm", &now, 0.001).is_none());
        assert!(Srdf::parse("<robot").is_err());
    }

    #[test]
    fn a_hand_given_by_its_joints_touches_the_links_they_move() {
        let urdf = r#"<robot name="arm">
  <link name="right_palm"/><link name="right_finger"/><link name="right_finger_tip"/>
  <joint name="right_finger_joint" type="revolute"><parent link="right_finger"/><child link="right_finger_tip"/></joint>
</robot>"#;
        let srdf = Srdf::parse(SRDF).unwrap().with_urdf(urdf);
        assert_eq!(
            srdf.hand_links("right_arm"),
            ["right_palm", "right_finger", "right_finger_tip"]
        );
        assert_eq!(
            Srdf::parse(SRDF).unwrap().with_urdf("not xml"),
            Srdf::parse(SRDF).unwrap()
        );
    }
}
