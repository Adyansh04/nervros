//! What a watch or a trigger waits for, and telling a new occurrence from one already there: a
//! topic condition is new when it turns true, an object when it has not been seen before.

use std::collections::BTreeSet;
use std::sync::Arc;
use std::time::Duration;

use nervros_ros::RobotPort;
use serde_json::Value;

use super::{Condition, Looker, PERIOD, topic_of};
use crate::mission::check::named;
use crate::profile::TopicRef;

/// How often the world model is looked at.
const OBJECT_PERIOD: Duration = Duration::from_secs(2);
const OBJECT_WAIT: Duration = Duration::from_secs(1);
/// The canopy object state for an object it removed.
const REMOVED: u64 = 2;

/// What to wait for.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Trigger {
    /// A condition on a topic.
    Topic {
        topic: String,
        ty: String,
        condition: Condition,
    },
    /// An object like `phrase` appearing in the world model, in `room` when it names one.
    Object {
        phrase: String,
        room: Option<String>,
    },
}

impl Trigger {
    /// From `{object, room}`, or `{topic, condition, ...}`.
    ///
    /// # Errors
    ///
    /// Neither is well formed, or the topic is not one the agent may read.
    pub(crate) async fn parse(
        args: &Value,
        robot: &dyn RobotPort,
        read_deny: &[String],
    ) -> Result<Self, String> {
        if let Some(phrase) = args["object"]
            .as_str()
            .map(str::trim)
            .filter(|p| !p.is_empty())
        {
            let room = args["room"]
                .as_str()
                .map(str::trim)
                .filter(|r| !r.is_empty())
                .map(str::to_owned);
            return Ok(Self::Object {
                phrase: phrase.to_owned(),
                room,
            });
        }
        let condition = Condition::parse(args)?;
        let (topic, ty) = topic_of(robot, args, read_deny, condition.reads_messages()).await?;
        Ok(Self::Topic {
            topic,
            ty,
            condition,
        })
    }

    pub(crate) fn describe(&self) -> String {
        match self {
            Self::Topic {
                topic, condition, ..
            } => condition.describe(topic),
            Self::Object { phrase, room } => match room {
                Some(r) => format!("a {phrase} appears in {r}"),
                None => format!("a {phrase} appears"),
            },
        }
    }
}

enum Source {
    Topic {
        topic: String,
        ty: String,
        condition: Condition,
        looker: Looker,
    },
    Object {
        feed: TopicRef,
        phrase: String,
        room: Option<String>,
    },
}

/// A [`Trigger`] being looked for.
pub(crate) struct Detector {
    robot: Arc<dyn RobotPort>,
    source: Source,
    was: bool,
    known: BTreeSet<String>,
}

impl Detector {
    /// Starts looking; `objects` is where the robot's world model publishes them. What holds
    /// already is news to the first look, unless [`Self::settle`] takes it in.
    ///
    /// # Errors
    ///
    /// The topic cannot be subscribed to, or the robot has no world model to read.
    pub(crate) async fn open(
        trigger: Trigger,
        robot: &Arc<dyn RobotPort>,
        objects: Option<TopicRef>,
    ) -> Result<Self, String> {
        let source = match trigger {
            Trigger::Topic {
                topic,
                ty,
                condition,
            } => {
                let looker = Looker::open(robot.as_ref(), &topic, &ty, &condition)
                    .await
                    .map_err(|e| format!("cannot read {topic}: {e}"))?;
                Source::Topic {
                    topic,
                    ty,
                    condition,
                    looker,
                }
            }
            Trigger::Object { phrase, room } => Source::Object {
                feed: objects.ok_or("this robot has no world model to watch for objects")?,
                phrase,
                room,
            },
        };
        Ok(Self {
            robot: Arc::clone(robot),
            source,
            was: false,
            known: BTreeSet::new(),
        })
    }

    /// How long a look's turn takes, its own time included.
    pub(crate) fn period(&self) -> Duration {
        match self.source {
            Source::Topic { .. } => PERIOD,
            Source::Object { .. } => OBJECT_PERIOD,
        }
    }

    /// Takes in what holds now without firing.
    ///
    /// # Errors
    ///
    /// The topic cannot be read.
    pub(crate) async fn settle(&mut self) -> Result<(), String> {
        self.newly().await.map(drop)
    }

    /// What newly holds, if anything.
    ///
    /// # Errors
    ///
    /// The topic cannot be read.
    pub(crate) async fn newly(&mut self) -> Result<Option<String>, String> {
        match &mut self.source {
            Source::Topic {
                topic,
                ty,
                condition,
                looker,
            } => {
                let seen = looker
                    .look(self.robot.as_ref(), topic, ty, condition)
                    .await
                    .map_err(|e| format!("it cannot read {topic}: {e}"))?;
                let fired = !self.was && seen.is_some();
                self.was = seen.is_some();
                Ok(seen.filter(|_| fired))
            }
            Source::Object { feed, phrase, room } => {
                let msg = self
                    .robot
                    .latest(&feed.topic, &feed.msg_type, OBJECT_WAIT)
                    .await
                    .unwrap_or(Value::Null);
                let now = objects(&msg, phrase, room.as_deref());
                let new: Vec<String> = now.difference(&self.known).cloned().collect();
                self.known.extend(now);
                Ok((!new.is_empty()).then(|| format!("{} appeared", new.join(" and "))))
            }
        }
    }
}

/// The ids of the objects like `phrase` the world model holds, in `room` when given.
fn objects(msg: &Value, phrase: &str, room: Option<&str>) -> BTreeSet<String> {
    msg["objects"]
        .as_array()
        .map_or(&[][..], Vec::as_slice)
        .iter()
        .filter(|o| o["state"].as_u64() != Some(REMOVED) && named(o, phrase))
        .filter(|o| room.is_none_or(|r| o["room_id"].as_str() == Some(r)))
        .filter_map(|o| o["id"].as_str().map(str::to_owned))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use nervros_ros::fake::FakeRobot;
    use serde_json::json;

    fn object(id: &str, label: &str, room: &str, state: u64) -> Value {
        json!({"id": id, "label": label, "room_id": room, "state": state})
    }

    fn feed() -> TopicRef {
        TopicRef {
            topic: "/objects".to_owned(),
            msg_type: "canopy_msgs/msg/WorldObjectArray".to_owned(),
        }
    }

    #[tokio::test]
    async fn a_topic_condition_fires_when_it_turns_true_not_while_it_stays() {
        let fake = Arc::new(FakeRobot::new().with_topic("/door", json!({"data": true})));
        let robot = Arc::clone(&fake) as Arc<dyn RobotPort>;
        let condition = Condition::parse(
            &json!({"condition": "value", "field": "data", "op": "==", "value": true}),
        )
        .unwrap();
        let trigger = Trigger::Topic {
            topic: "/door".to_owned(),
            ty: "std_msgs/msg/Bool".to_owned(),
            condition,
        };
        let mut detector = Detector::open(trigger, &robot, None).await.unwrap();
        detector.settle().await.unwrap();
        assert_eq!(detector.newly().await, Ok(None), "open already when set");
        fake.set_topic("/door", json!({"data": false}));
        assert_eq!(detector.newly().await, Ok(None));
        fake.set_topic("/door", json!({"data": true}));
        assert_eq!(
            detector.newly().await.unwrap().as_deref(),
            Some("data is true")
        );
        assert_eq!(detector.newly().await, Ok(None), "once per opening");
    }

    #[tokio::test]
    async fn an_object_fires_when_new_in_its_room() {
        let fake = Arc::new(FakeRobot::new().with_topic(
            "/objects",
            json!({"objects": [object("O1", "white cup", "R2", 0)]}),
        ));
        let robot = Arc::clone(&fake) as Arc<dyn RobotPort>;
        let trigger = Trigger::Object {
            phrase: "cup".to_owned(),
            room: Some("R2".to_owned()),
        };
        assert_eq!(trigger.describe(), "a cup appears in R2");
        let mut detector = Detector::open(trigger, &robot, Some(feed())).await.unwrap();
        detector.settle().await.unwrap();
        assert_eq!(detector.newly().await, Ok(None), "O1 was there already");
        fake.set_topic(
            "/objects",
            json!({"objects": [
                object("O1", "white cup", "R2", 0),
                object("O2", "red cup", "R1", 0),
                object("O3", "plate", "R2", 0),
                object("O4", "blue cup", "R2", 2),
            ]}),
        );
        assert_eq!(
            detector.newly().await,
            Ok(None),
            "another room, not a cup, removed"
        );
        fake.set_topic(
            "/objects",
            json!({"objects": [object("O1", "white cup", "R2", 0), object("O5", "cup", "R2", 0)]}),
        );
        assert_eq!(
            detector.newly().await.unwrap().as_deref(),
            Some("O5 appeared")
        );
        assert_eq!(detector.newly().await, Ok(None));
    }

    #[tokio::test]
    async fn an_object_needs_a_world_model() {
        let robot = Arc::new(FakeRobot::new()) as Arc<dyn RobotPort>;
        let trigger = Trigger::Object {
            phrase: "cup".to_owned(),
            room: None,
        };
        let Err(why) = Detector::open(trigger, &robot, None).await else {
            panic!("no world model, no detector");
        };
        assert!(why.contains("no world model"), "{why}");
    }

    #[tokio::test]
    async fn the_arguments_name_an_object_or_a_topic_condition() {
        let robot = FakeRobot::new();
        let object = Trigger::parse(&json!({"object": " cup ", "room": ""}), &robot, &[])
            .await
            .unwrap();
        assert_eq!(
            object,
            Trigger::Object {
                phrase: "cup".to_owned(),
                room: None
            }
        );
        let neither = Trigger::parse(&json!({"room": "R2"}), &robot, &[]).await;
        assert!(neither.unwrap_err().contains("`condition`"));
    }
}
