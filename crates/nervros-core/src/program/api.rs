//! The robot as a program sees it: what the world model knows, what the robot reads, and the
//! skills its envelope allows, each a plain function. Every call goes through a [`Host`], so a
//! test stands in for the robot.

use std::sync::Arc;

use rhai::{Dynamic, Engine, EvalAltResult, Map};
use serde_json::{Value, json};

use crate::mission::catalog::{FIND_IN_VIEW, GO_TO_PLACE};

/// Answers a program's calls. A call may block the program's own thread, never a runtime's.
pub trait Host: Send + Sync {
    /// Objects the world model knows whose label or name holds `words`, in `room` (an id, a name
    /// or a type) when given, as `[{id, label, name, room, x, y, z, state, last_seen_s}]`.
    ///
    /// # Errors
    ///
    /// The world model did not answer.
    fn objects(&self, words: &str, room: Option<&str>) -> Result<Value, String>;
    /// The rooms, as `[{id, name, type, x, y, area, seen}]`.
    ///
    /// # Errors
    ///
    /// The world model did not answer.
    fn rooms(&self) -> Result<Value, String>;
    /// The named places, as `[{name, x, y, room}]`, `room` the id of the room it is in or empty.
    ///
    /// # Errors
    ///
    /// Never for the profile's places; a robot port may fail.
    fn places(&self) -> Result<Value, String>;
    /// The robot on the map, as `{x, y, yaw}`.
    ///
    /// # Errors
    ///
    /// No transform to the map.
    fn pose(&self) -> Result<Value, String>;
    /// Runs one skill as a mission and waits for its end: `{ok, outcome, reason, outputs}`.
    ///
    /// # Errors
    ///
    /// The skill is not one the program was approved for, the plan failed its checks, the robot
    /// cannot run a mission now, or the robot or its executor cut the mission short.
    fn run(&self, skill: &str, args: Value) -> Result<Value, String>;
    /// The newest message on a topic, as JSON.
    ///
    /// # Errors
    ///
    /// The topic is not on the graph, the profile keeps it from being read, or nothing came.
    fn read(&self, topic: &str) -> Result<Value, String>;
    /// How often a topic publishes, measured over `seconds`, which may be any float.
    ///
    /// # Errors
    ///
    /// The topic is not on the graph or cannot be read.
    fn rate(&self, topic: &str, seconds: f64) -> Result<f64, String>;
    /// Where frame `child` is in frame `parent`, as `{x, y, z, yaw}`.
    ///
    /// # Errors
    ///
    /// No chain of transforms links them.
    fn tf(&self, parent: &str, child: &str) -> Result<Value, String>;
    /// Waits, returning early when the program must stop. `seconds` may be any float the
    /// program computed, NaN included.
    fn wait(&self, seconds: f64);
    /// Tells the operator a line of what it does.
    fn say(&self, text: &str);
}

/// What the functions look like to the model, for the tool's description.
pub const REFERENCE: &str = "objects(words) and objects(words, room) -> [#{id, label, name, \
     room, x, y, z, state, last_seen_s}]; rooms() -> [#{id, name, type, x, y, area, seen}]; \
     places() -> [#{name, x, y, room}] (room: the id of the room it is in); pose() -> \
     #{x, y, yaw}; walk(place) -> true once there (a place, room or object id, or words); \
     find(words) -> the id the detector reports, or () when it sees none; any skill by its name, such as PickObject(#{object_id: id, arm: \"right\"}), \
     PickObject(id) or TuckForTravel(), or run(\"Skill\", #{args}), -> #{ok, outcome, reason, \
     outputs}; read(topic) -> \
     the newest message; rate(topic, seconds) -> Hz; tf(parent, child) -> #{x, y, z, yaw}; \
     wait(seconds); say(text) tells the operator. The last value is the program's result.";

/// A skill as a program may call it by name: the skill and the argument a bare value fills.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SkillCall {
    /// As plans name it, such as `PickObject`.
    pub name: String,
    /// The argument `PickObject("O17")` sets: its first.
    pub first_arg: String,
}

type Fallible = Result<Dynamic, Box<EvalAltResult>>;

/// A list of words as the one phrase they make.
fn words_of(list: &rhai::Array) -> String {
    list.iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(" ")
}

fn count(n: usize) -> i64 {
    i64::try_from(n).unwrap_or(i64::MAX)
}

/// Whole seconds as a program writes them, within a day: no wait or count needs more.
fn whole_seconds(n: i64) -> f64 {
    f64::from(i32::try_from(n.clamp(0, 86_400)).unwrap_or(0))
}

fn to_dynamic(value: Result<Value, String>) -> Fallible {
    rhai::serde::to_dynamic(value.map_err(Box::<EvalAltResult>::from)?)
}

fn to_json(value: &Dynamic) -> Result<Value, Box<EvalAltResult>> {
    rhai::serde::from_dynamic(value)
}

/// Adds the robot's functions to `engine`, each answered by `host`.
pub fn register(engine: &mut Engine, host: &Arc<dyn Host>, skills: &[SkillCall]) {
    // What a program written as JavaScript reaches for: a list's length, words as a list.
    engine.register_get("length", |a: &mut rhai::Array| count(a.len()));
    engine.register_get("length", |m: &mut Map| count(m.len()));
    engine.register_get("length", |s: &mut rhai::ImmutableString| {
        count(s.chars().count())
    });
    let h = Arc::clone(host);
    engine.register_fn("objects", move |words: rhai::Array| {
        to_dynamic(h.objects(&words_of(&words), None))
    });
    let h = Arc::clone(host);
    engine.register_fn("objects", move |words: rhai::Array, room: &str| {
        to_dynamic(h.objects(&words_of(&words), Some(room)))
    });
    let h = Arc::clone(host);
    engine.register_fn("objects", move |words: &str| {
        to_dynamic(h.objects(words, None))
    });
    let h = Arc::clone(host);
    engine.register_fn("objects", move |words: &str, room: &str| {
        to_dynamic(h.objects(words, Some(room)))
    });
    let h = Arc::clone(host);
    engine.register_fn("rooms", move || to_dynamic(h.rooms()));
    let h = Arc::clone(host);
    engine.register_fn("places", move || to_dynamic(h.places()));
    let h = Arc::clone(host);
    engine.register_fn("pose", move || to_dynamic(h.pose()));
    let h = Arc::clone(host);
    engine.register_fn("run", move |skill: &str, args: Map| -> Fallible {
        let args = to_json(&Dynamic::from_map(args))?;
        to_dynamic(h.run(skill, args))
    });
    let h = Arc::clone(host);
    engine.register_fn(
        "walk",
        move |place: &str| -> Result<bool, Box<EvalAltResult>> {
            let ended = h
                .run(GO_TO_PLACE, json!({"place": place}))
                .map_err(Box::<EvalAltResult>::from)?;
            Ok(ended["ok"].as_bool().unwrap_or(false))
        },
    );
    let h = Arc::clone(host);
    engine.register_fn("find", move |words: &str| find(h.as_ref(), words));
    let h = Arc::clone(host);
    engine.register_fn("find", move |words: rhai::Array| {
        find(h.as_ref(), &words_of(&words))
    });
    for skill in skills {
        let (h, name) = (Arc::clone(host), skill.name.clone());
        engine.register_fn(&skill.name, move |args: Map| -> Fallible {
            let args = to_json(&Dynamic::from_map(args))?;
            to_dynamic(h.run(&name, args))
        });
        let (h, name, arg) = (
            Arc::clone(host),
            skill.name.clone(),
            skill.first_arg.clone(),
        );
        engine.register_fn(&skill.name, move |value: &str| -> Fallible {
            to_dynamic(h.run(&name, json!({ arg.as_str(): value })))
        });
        // ExploreBuilding() takes nothing: approved as a skill it ran, it then failed as no function.
        let (h, name) = (Arc::clone(host), skill.name.clone());
        engine.register_fn(&skill.name, move || -> Fallible {
            to_dynamic(h.run(&name, json!({})))
        });
    }
    let h = Arc::clone(host);
    engine.register_fn("read", move |topic: &str| to_dynamic(h.read(topic)));
    let h = Arc::clone(host);
    engine.register_fn(
        "rate",
        move |topic: &str, seconds: f64| -> Result<f64, Box<EvalAltResult>> {
            h.rate(topic, seconds).map_err(Box::<EvalAltResult>::from)
        },
    );
    let h = Arc::clone(host);
    engine.register_fn(
        "rate",
        move |topic: &str, seconds: i64| -> Result<f64, Box<EvalAltResult>> {
            h.rate(topic, whole_seconds(seconds))
                .map_err(Box::<EvalAltResult>::from)
        },
    );
    let h = Arc::clone(host);
    engine.register_fn("tf", move |parent: &str, child: &str| {
        to_dynamic(h.tf(parent, child))
    });
    // Rhai's own sleep would hold the program through a stop; registered here, it is wait.
    for name in ["wait", "sleep"] {
        let h = Arc::clone(host);
        engine.register_fn(name, move |seconds: f64| h.wait(seconds));
        let h = Arc::clone(host);
        engine.register_fn(name, move |seconds: i64| h.wait(whole_seconds(seconds)));
    }
    let h = Arc::clone(host);
    engine.register_fn("say", move |text: &str| h.say(text));
}

/// What the detector reports `words` under, or nothing when it sees none.
fn find(host: &dyn Host, words: &str) -> Fallible {
    let ended = host
        .run(FIND_IN_VIEW, json!({"phrase": words}))
        .map_err(Box::<EvalAltResult>::from)?;
    Ok(match ended["outputs"]["object_id"].as_str() {
        Some(id) if ended["ok"] == true => Dynamic::from(id.to_owned()),
        _ => Dynamic::UNIT,
    })
}
