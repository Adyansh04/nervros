//! The robot a program runs on: the world model and the robot port read through the missions,
//! and every skill run as one mission of the program's job.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde_json::{Map, Value, json};

use super::api::Host;
use crate::job::Context;
use crate::lock;
use crate::mission::{Missions, Outcome};

/// How long a read waits for a first message.
const READ_WAIT: Duration = Duration::from_secs(2);
/// Messages a rate counts at most.
const RATE_MAX: usize = 100_000;
/// The shortest and the longest a rate measures.
const RATE_S: (f64, f64) = (0.5, 30.0);
/// The longest a wait lasts.
const WAIT_MAX_S: f64 = 3600.0;
/// How often a wait looks whether the program must stop.
const WAIT_SLICE: Duration = Duration::from_millis(100);

/// The robot of one program's run.
pub struct RobotHost {
    missions: Arc<Missions>,
    ctx: Context,
    runtime: tokio::runtime::Handle,
    /// The map frame and the robot's base frame, for `pose`.
    frames: (String, String),
    last_said: Mutex<Option<String>>,
    skills_run: std::sync::atomic::AtomicUsize,
}

impl RobotHost {
    /// A host on `missions`' robot for the job `ctx`, blocking on `runtime`.
    #[must_use]
    pub fn new(
        missions: Arc<Missions>,
        ctx: Context,
        runtime: tokio::runtime::Handle,
        frames: (String, String),
    ) -> Self {
        Self {
            missions,
            ctx,
            runtime,
            frames,
            last_said: Mutex::default(),
            skills_run: std::sync::atomic::AtomicUsize::new(0),
        }
    }

    /// How many skills the program has run.
    #[must_use]
    pub fn skills_run(&self) -> usize {
        self.skills_run.load(std::sync::atomic::Ordering::Relaxed)
    }

    /// The last line the program said.
    #[must_use]
    pub fn last_said(&self) -> Option<String> {
        lock(&self.last_said).clone()
    }

    /// A topic the profile lets programs read, and its type; `whole` when it is read message by
    /// message, which large types are not.
    fn topic(&self, topic: &str, whole: bool) -> Result<(String, String), String> {
        let args = json!({"topic": topic});
        self.runtime.block_on(crate::watch::topic_of(
            self.missions.robot().as_ref(),
            &args,
            self.missions.read_deny(),
            whole,
        ))
    }
}

impl Host for RobotHost {
    fn objects(&self, words: &str, room: Option<&str>) -> Result<Value, String> {
        let seen = self.runtime.block_on(self.missions.observe());
        if seen.objects.is_null() {
            return Err("the world model has not said what objects it knows".to_owned());
        }
        Ok(objects_matching(&seen.objects, &seen.rooms, words, room))
    }

    fn rooms(&self) -> Result<Value, String> {
        let seen = self.runtime.block_on(self.missions.observe());
        if seen.rooms.is_null() {
            return Err("the world model has not said what rooms it knows".to_owned());
        }
        Ok(rooms_of(&seen.rooms))
    }

    fn places(&self) -> Result<Value, String> {
        let seen = self.runtime.block_on(self.missions.observe());
        let rooms = seen.rooms["rooms"].as_array().cloned().unwrap_or_default();
        Ok(Value::Array(
            seen.places
                .iter()
                .map(|p| {
                    let room = rooms
                        .iter()
                        .find(|r| crate::search::holds(&r["outline"], p.pose.x, p.pose.y))
                        .and_then(|r| r["id"].as_str())
                        .unwrap_or_default();
                    json!({"name": p.name, "x": p.pose.x, "y": p.pose.y, "room": room})
                })
                .collect(),
        ))
    }

    fn pose(&self) -> Result<Value, String> {
        let (map, base) = &self.frames;
        let t = self
            .missions
            .robot()
            .transform(map, base)
            .map_err(|e| e.to_string())?;
        let (x, y, yaw) = t.planar();
        Ok(json!({"x": x, "y": y, "yaw": yaw}))
    }

    fn run(&self, skill: &str, args: Value) -> Result<Value, String> {
        let steps = json!([{"skill": skill, "args": args}]);
        let intent = format!("{skill} for program {}", self.ctx.id());
        self.skills_run
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        self.ctx.log(format!("running {skill}"));
        let end = self
            .runtime
            .block_on(self.missions.run_for_job(&self.ctx, &intent, steps))?;
        // A fault is the robot's or the executor's: a program that retried would only meet it again.
        if let Some(fault) = end.fault {
            return Err(format!(
                "{skill} was cut short: {}. {}",
                end.reason,
                fault.next()
            ));
        }
        self.ctx.log(match end.outcome {
            Outcome::Success => format!("{skill} succeeded"),
            other => format!("{skill} ended {other}: {}", end.reason),
        });
        Ok(json!({
            "ok": end.outcome == Outcome::Success,
            "outcome": end.outcome,
            "reason": end.reason,
            "outputs": own_outputs(&end.outputs),
        }))
    }

    fn read(&self, topic: &str) -> Result<Value, String> {
        let (topic, msg_type) = self.topic(topic, true)?;
        self.runtime
            .block_on(self.missions.robot().latest(&topic, &msg_type, READ_WAIT))
            .map_err(|e| e.to_string())
    }

    fn rate(&self, topic: &str, seconds: f64) -> Result<f64, String> {
        let (topic, msg_type) = self.topic(topic, false)?;
        let seconds = within(seconds, RATE_S);
        let window = Duration::from_secs_f64(seconds);
        let arrivals = self
            .runtime
            .block_on(
                self.missions
                    .robot()
                    .sample_sizes(&topic, &msg_type, window, RATE_MAX),
            )
            .map_err(|e| e.to_string())?;
        let count = u32::try_from(arrivals.len()).unwrap_or(u32::MAX);
        Ok(f64::from(count) / seconds)
    }

    fn tf(&self, parent: &str, child: &str) -> Result<Value, String> {
        let t = self
            .missions
            .robot()
            .transform(parent, child)
            .map_err(|e| e.to_string())?;
        let [x, y, z] = t.translation;
        Ok(json!({"x": x, "y": y, "z": z, "yaw": t.yaw()}))
    }

    fn wait(&self, seconds: f64) {
        let until =
            std::time::Instant::now() + Duration::from_secs_f64(within(seconds, (0.0, WAIT_MAX_S)));
        while !self.ctx.stopped() && std::time::Instant::now() < until {
            std::thread::sleep(WAIT_SLICE);
        }
    }

    fn say(&self, text: &str) {
        *lock(&self.last_said) = Some(text.to_owned());
        self.ctx.log(text);
    }
}

/// `seconds` within `(least, most)`; NaN, which a program's arithmetic can make and a `Duration`
/// cannot hold, is the least.
fn within(seconds: f64, (least, most): (f64, f64)) -> f64 {
    seconds.max(least).min(most)
}

/// A one-step mission's outputs by their own names: `s1_object_id` is the step's `object_id`.
fn own_outputs(outputs: &Map<String, Value>) -> Value {
    Value::Object(
        outputs
            .iter()
            .map(|(key, v)| {
                let name = key.split_once('_').map_or(key.as_str(), |(_, n)| n);
                (name.to_owned(), v.clone())
            })
            .collect(),
    )
}

/// Whether `text` holds every word of `words`, ignoring case.
fn holds_words(text: &str, words: &str) -> bool {
    let text = text.to_lowercase();
    words
        .split_whitespace()
        .all(|w| text.contains(&w.to_lowercase()))
}

/// The ids of the rooms `room` names: an id, or words of a name or type.
fn room_ids(rooms: &Value, room: &str) -> Vec<String> {
    rooms["rooms"]
        .as_array()
        .map_or(&[][..], Vec::as_slice)
        .iter()
        .filter(|r| {
            r["id"] == room
                || holds_words(r["name"].as_str().unwrap_or_default(), room)
                || holds_words(r["type"].as_str().unwrap_or_default(), room)
        })
        .filter_map(|r| r["id"].as_str().map(str::to_owned))
        .collect()
}

/// The objects whose label or name holds `words`, as the planner matches them, in `room` when
/// given, as a program sees them.
fn objects_matching(objects: &Value, rooms: &Value, words: &str, room: Option<&str>) -> Value {
    let inside = room.map(|r| room_ids(rooms, r));
    Value::Array(
        objects["objects"]
            .as_array()
            .map_or(&[][..], Vec::as_slice)
            .iter()
            .filter(|o| {
                crate::mission::check::named(o, words)
                    && inside.as_ref().is_none_or(|ids| {
                        ids.iter().any(|id| o["room_id"] == id.as_str())
                    })
            })
            .map(|o| {
                let at = &o["pose"]["position"];
                json!({
                    "id": o["id"], "label": o["label"], "name": o["name"],
                    "room": o["room_id"], "x": at["x"], "y": at["y"], "z": at["z"],
                    "state": match o["state"].as_u64() { Some(1) => "stale", Some(2) => "removed", _ => "active" },
                    "last_seen_s": o["last_seen"]["sec"],
                })
            })
            .collect(),
    )
}

/// The rooms as a program sees them, each named by its type while its name is a placeholder
/// ("room C"), so a program's counts read as the operator says the rooms.
fn rooms_of(rooms: &Value) -> Value {
    Value::Array(
        rooms["rooms"]
            .as_array()
            .map_or(&[][..], Vec::as_slice)
            .iter()
            .map(|r| {
                let name = crate::places::room_label(r)
                    .unwrap_or_default()
                    .replace('_', " ");
                json!({
                    "id": r["id"], "name": name, "type": r["type"],
                    "x": r["centroid"]["x"], "y": r["centroid"]["y"],
                    "area": r["area"], "seen": r["floor_coverage"],
                })
            })
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world() -> (Value, Value) {
        let objects = json!({"objects": [
            {"id": "O1", "label": "mug", "name": "red mug", "room_id": "R1", "state": 0,
             "pose": {"position": {"x": 1.0, "y": 2.0, "z": 0.8}}, "last_seen": {"sec": 100}},
            {"id": "O2", "label": "cup", "name": "", "room_id": "R2", "state": 1,
             "pose": {"position": {"x": 5.0, "y": 2.0, "z": 0.8}}, "last_seen": {"sec": 50}}]});
        let rooms = json!({"rooms": [
            {"id": "R1", "name": "room A", "type": "kitchen", "centroid": {"x": 1.0, "y": 1.0},
             "area": 12.0, "floor_coverage": 0.9},
            {"id": "R2", "name": "room B", "type": "office", "centroid": {"x": 5.0, "y": 1.0},
             "area": 9.0, "floor_coverage": 0.4}]});
        (objects, rooms)
    }

    #[test]
    fn objects_match_every_word_and_a_room_by_id_name_or_type() {
        let (objects, rooms) = world();
        let red = objects_matching(&objects, &rooms, "Red MUG", None);
        assert_eq!(red[0]["id"], "O1");
        assert_eq!(red[0]["last_seen_s"], 100);
        assert_eq!(
            objects_matching(&objects, &rooms, "cup", Some("office"))[0]["state"],
            "stale"
        );
        assert_eq!(
            objects_matching(&objects, &rooms, "cup", Some("R1")),
            json!([])
        );
        assert_eq!(
            objects_matching(&objects, &rooms, "mug", Some("room A"))[0]["id"],
            "O1"
        );
        assert_eq!(rooms_of(&rooms)[1]["seen"], 0.4);
        assert_eq!(rooms_of(&rooms)[0]["name"], "kitchen", "room A is its type");
    }

    #[test]
    fn any_float_a_program_waits_for_is_a_duration() {
        for (seconds, expected) in [
            (f64::NAN, 0.0),
            (f64::INFINITY, WAIT_MAX_S),
            (f64::NEG_INFINITY, 0.0),
            (-3.0, 0.0),
            (2.5, 2.5),
        ] {
            assert!((within(seconds, (0.0, WAIT_MAX_S)) - expected).abs() < 1e-9);
        }
        assert!((within(f64::NAN, RATE_S) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn a_steps_outputs_go_by_their_own_names() {
        let mut outputs = Map::new();
        outputs.insert("s1_object_id".to_owned(), json!("mug_4"));
        assert_eq!(own_outputs(&outputs), json!({"object_id": "mug_4"}));
    }
}
