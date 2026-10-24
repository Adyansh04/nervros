//! A program's sandbox: a Rhai engine with no files, network or `eval`, bounded in operations,
//! depth and sizes, that stops between any two operations once its job must stop.

use std::sync::Arc;

use rhai::module_resolvers::DummyModuleResolver;
use rhai::{AST, Dynamic, Engine, EvalAltResult};

use super::api::{self, Host, SkillCall};

/// Operations a program may take in all: a few seconds of work, not an endless loop.
const MAX_OPERATIONS: u64 = 5_000_000;
const MAX_CALL_LEVELS: usize = 32;
const MAX_EXPR_DEPTH: usize = 64;
const MAX_FN_EXPR_DEPTH: usize = 32;
const MAX_STRING: usize = 64 * 1024;
const MAX_ARRAY: usize = 10_000;
const MAX_MAP: usize = 10_000;
/// A result is cut past this many characters, as a tool's is.
const MAX_RESULT: usize = 2_000;

/// An engine with the limits and none of the robot's functions: what checking a program needs.
fn bare() -> Engine {
    let mut e = Engine::new();
    e.set_module_resolver(DummyModuleResolver::new());
    e.disable_symbol("eval");
    e.set_max_operations(MAX_OPERATIONS);
    e.set_max_call_levels(MAX_CALL_LEVELS);
    e.set_max_expr_depths(MAX_EXPR_DEPTH, MAX_FN_EXPR_DEPTH);
    e.set_max_string_size(MAX_STRING);
    e.set_max_array_size(MAX_ARRAY);
    e.set_max_map_size(MAX_MAP);
    e
}

/// Parses a program, without running any of it.
///
/// # Errors
///
/// It is not valid Rhai, or uses what the sandbox leaves out.
pub fn compile(source: &str) -> Result<AST, String> {
    bare().compile(source).map_err(|e| {
        let e = e.to_string();
        format!("the program does not parse: {e}{}", hint(&e, source))
    })
}

/// What a model that writes JavaScript gets wrong in Rhai, said where the error shows it.
fn hint(error: &str, source: &str) -> &'static str {
    if source.contains("= {}") || source.contains("({})") || source.contains(" {};") {
        "; an empty map is #{}, not {}"
    } else if error.contains("Expecting ';'") {
        "; end every statement with ;"
    } else if error.contains("Unknown operator: '?'") {
        "; Rhai has no ?: , write if c { a } else { b }"
    } else if error.contains("Function not found: range (i64)") {
        // Python's range(3), which a model wrote twice in a row.
        "; Rhai counts with 0..3 or range(0, 3), never range(3)"
    } else if error.contains("Function not found") || error.contains("Property not found") {
        "; the robot's functions are objects, rooms, places, pose, walk, find, run, read, rate, \
         tf, wait and say, a list counts with .len(), and a value cannot be called"
    } else {
        ""
    }
}

/// An engine whose robot is `host`; it stops at the next operation once `stopped` says so.
pub fn engine(
    host: &Arc<dyn Host>,
    skills: &[SkillCall],
    stopped: impl Fn() -> bool + Send + Sync + 'static,
) -> Engine {
    let mut e = bare();
    // Looked at every few operations: the stop is a lock read, and a few operations are nothing.
    e.on_progress(move |ops| (ops % 64 == 0 && stopped()).then(|| Dynamic::from("stopped")));
    let h = Arc::clone(host);
    e.on_print(move |text| h.say(text));
    let h = Arc::clone(host);
    e.on_debug(move |text, _, _| h.say(text));
    api::register(&mut e, host, skills);
    e
}

/// Runs a program to its end; its last value, as text, or `None` when that was `()`.
///
/// # Errors
///
/// It failed, ran out of operations, or was stopped, said as the model can act on.
pub fn run(engine: &Engine, ast: &AST) -> Result<Option<String>, String> {
    match engine.eval_ast::<Dynamic>(ast) {
        Ok(value) => Ok((!value.is_unit()).then(|| result_text(&value))),
        Err(e) => Err(match *e {
            EvalAltResult::ErrorTerminated(..) => "it was stopped".to_owned(),
            EvalAltResult::ErrorTooManyOperations(_) => {
                format!("it took more than {MAX_OPERATIONS} operations: a loop that never ends?")
            }
            other => {
                let text = other.to_string();
                format!("it failed: {text}{}", hint(&text, ""))
            }
        }),
    }
}

/// The result of a program that ended on `()`. That is also "found nothing", as a search loop
/// that never set its answer ("done" read as success), and what it said last may be the answer.
#[must_use]
pub fn nothing(last_said: Option<&str>) -> String {
    match last_said {
        Some(said) => format!(
            "nothing: its last value was (); the last thing it said was \"{}\"",
            crate::tools::clip(said, MAX_RESULT)
        ),
        None => "nothing: its last value was ()".to_owned(),
    }
}

/// A program's result as the operator and the model read it.
fn result_text(value: &Dynamic) -> String {
    let text = if let Some(s) = value.read_lock::<rhai::ImmutableString>() {
        s.to_string()
    } else {
        rhai::serde::from_dynamic::<serde_json::Value>(value)
            .map_or_else(|_| value.to_string(), |v| v.to_string())
    };
    crate::tools::clip(&text, MAX_RESULT)
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicBool, Ordering};

    use serde_json::{Value, json};

    use super::*;

    /// A robot with two rooms and a mug, that records what it is asked to run.
    #[derive(Default)]
    struct Fake {
        ran: Mutex<Vec<(String, Value)>>,
        said: Mutex<Vec<String>>,
        waited: Mutex<Vec<f64>>,
    }

    impl Host for Fake {
        fn objects(&self, words: &str, room: Option<&str>) -> Result<Value, String> {
            let all = json!([{"id": "O1", "label": "mug", "room": "R1"},
                             {"id": "O2", "label": "mug", "room": "R2"}]);
            Ok(Value::Array(
                all.as_array()
                    .unwrap()
                    .iter()
                    .filter(|o| o["label"] == words && room.is_none_or(|r| o["room"] == r))
                    .cloned()
                    .collect(),
            ))
        }
        fn rooms(&self) -> Result<Value, String> {
            Ok(json!([{"id": "R1", "type": "kitchen"}, {"id": "R2", "type": "office"}]))
        }
        fn places(&self) -> Result<Value, String> {
            Ok(json!([]))
        }
        fn pose(&self) -> Result<Value, String> {
            Ok(json!({"x": 1.0, "y": 2.0, "yaw": 0.0}))
        }
        fn run(&self, skill: &str, args: Value) -> Result<Value, String> {
            self.ran.lock().unwrap().push((skill.to_owned(), args));
            let outputs = if skill == "FindInView" {
                json!({"object_id": "mug_4"})
            } else {
                json!({})
            };
            Ok(json!({"ok": true, "outcome": "success", "reason": "", "outputs": outputs}))
        }
        fn read(&self, _topic: &str) -> Result<Value, String> {
            Err("no such topic".to_owned())
        }
        fn rate(&self, _topic: &str, _seconds: f64) -> Result<f64, String> {
            Ok(10.0)
        }
        fn tf(&self, _parent: &str, _child: &str) -> Result<Value, String> {
            Ok(json!({"x": 0.0}))
        }
        fn wait(&self, seconds: f64) {
            self.waited.lock().unwrap().push(seconds);
        }
        fn say(&self, text: &str) {
            self.said.lock().unwrap().push(text.to_owned());
        }
    }

    fn callable() -> Vec<SkillCall> {
        vec![SkillCall {
            name: "PickObject".to_owned(),
            args: vec!["object_id".to_owned(), "arm".to_owned()],
        }]
    }

    fn run_on(fake: &Arc<Fake>, source: &str) -> Result<String, String> {
        let host: Arc<dyn Host> = Arc::clone(fake) as Arc<dyn Host>;
        let value = run(&engine(&host, &callable(), || false), &compile(source)?)?;
        let said = fake.said.lock().unwrap().last().cloned();
        Ok(value.unwrap_or_else(|| nothing(said.as_deref())))
    }

    #[test]
    fn a_program_counts_from_the_world_model_and_runs_its_skills() {
        let fake = Arc::new(Fake::default());
        let source = r#"
            let counts = #{};
            for r in rooms() { counts[r.type] = objects(["mug"], r.id).length; }
            say(`kitchen has ${counts.kitchen}`);
            if walk("R1") {
                let id = find("mug");
                run("PickObject", #{object_id: id, arm: "right"});
                PickObject(id);
                PickObject();
                PickObject(id, "left");
                PickObject(17);
                PickObject(#{object_id: id, arm: "left"});
            }
            counts
        "#;
        let out = run_on(&fake, source).unwrap();
        assert_eq!(out, r#"{"kitchen":1,"office":1}"#);
        let ran = fake.ran.lock().unwrap().clone();
        let skills: Vec<&str> = ran.iter().map(|(s, _)| s.as_str()).collect();
        assert_eq!(
            skills,
            [
                "GoToPlace",
                "FindInView",
                "PickObject",
                "PickObject",
                "PickObject",
                "PickObject",
                "PickObject",
                "PickObject"
            ]
        );
        assert_eq!(ran[2].1, json!({"object_id": "mug_4", "arm": "right"}));
        assert_eq!(
            ran[3].1,
            json!({"object_id": "mug_4"}),
            "a skill called by its name"
        );
        assert_eq!(ran[4].1, json!({}), "a skill called with nothing");
        assert_eq!(
            ran[5].1,
            json!({"object_id": "mug_4", "arm": "left"}),
            "values without names fill the arguments in order"
        );
        assert_eq!(
            ran[6].1,
            json!({"object_id": 17}),
            "a number as it is written"
        );
        assert_eq!(
            ran[7].1,
            json!({"object_id": "mug_4", "arm": "left"}),
            "a map alone is the arguments by name"
        );
        assert_eq!(fake.said.lock().unwrap().as_slice(), ["kitchen has 1"]);
    }

    #[test]
    fn a_program_that_ends_on_nothing_says_so() {
        // A search loop that never set its answer must not read as having found something.
        let fake = Arc::new(Fake::default());
        assert_eq!(
            run_on(&fake, "let found = (); found").unwrap(),
            "nothing: its last value was ()"
        );
        // Its answer can be what it said, as a loop that says where it found the mug.
        assert_eq!(
            run_on(&fake, r#"say("Found a mug at the dining table!");"#).unwrap(),
            r#"nothing: its last value was (); the last thing it said was "Found a mug at the dining table!""#
        );
    }

    #[test]
    fn the_sandbox_ends_endless_loops_refuses_eval_and_stops_when_told() {
        let fake = Arc::new(Fake::default());
        let endless = run_on(&fake, "let n = 0; loop { n += 1; }").unwrap_err();
        assert!(endless.contains("operations"), "{endless}");
        assert!(compile("eval(\"1 + 1\")").is_err());
        assert!(
            run_on(&fake, "import \"x\" as x; 1")
                .unwrap_err()
                .contains("failed")
        );

        let stop = Arc::new(AtomicBool::new(true));
        let host: Arc<dyn Host> = fake;
        let flag = Arc::clone(&stop);
        let e = engine(&host, &[], move || flag.load(Ordering::SeqCst));
        let stopped = run(&e, &compile("let n = 0; loop { n += 1; }").unwrap()).unwrap_err();
        assert_eq!(stopped, "it was stopped");
    }

    #[test]
    fn sleep_is_the_robots_wait() {
        let fake = Arc::new(Fake::default());
        run_on(&fake, "sleep(2); sleep(0.0 / 0.0); wait(1.5)").unwrap();
        let waited = fake.waited.lock().unwrap().clone();
        assert_eq!(waited.len(), 3, "Rhai's own sleep would not reach the host");
        assert!(waited[1].is_nan(), "the host settles what a NaN wait means");
    }
}
