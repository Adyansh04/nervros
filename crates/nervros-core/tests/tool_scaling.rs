//! How the local model picks its first tool as the tool list grows: the same requests, each with
//! one right first tool among the agent's own, offered with distractors up to 8, 16, 32 and 48
//! tools in all. Prints first-call accuracy, mean latency and prompt tokens per size, to choose
//! a cap on the tools a session offers.
//!
//! Ignored by default: it needs the local model server (`scripts/local-llm.sh start`).
//!
//! ```bash
//! cargo test -p nervros-core --test tool_scaling -- --ignored --nocapture
//! ```

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use nervros_core::llm::{self, AgentSource, History, Llm, LoopTool, TurnSetup};
use nervros_core::providers::ModelsConfig;
use nervros_core::providers::ledger::Ledger;
use nervros_core::providers::router::{PrivacyMode, Router};
use serde_json::{Value, json};

const LOCAL_ONLY: &str = r#"
    [[provider]]
    id = "local"
    kind = "openai_compat"
    base_url = "http://127.0.0.1:8081/v1"
    timeout_s = 120

    [[model]]
    id = "qwen3.5-9b-local"
    provider = "local"
    model = "qwen3.5-9b"
    vision = true
    tools = true
    privacy = { local = true }

    [roles]
    routine = ["qwen3.5-9b-local"]
"#;

const PREAMBLE: &str = "You are the agent of a humanoid robot in an apartment. Use a tool when \
    the operator's request needs one; answer briefly.";

/// The agent's own tools, each a name and what it does.
const OWN: [(&str, &str); 8] = [
    (
        "find_objects",
        "Finds objects in the world model by what they are: ids, labels and rooms.",
    ),
    (
        "look",
        "Looks through a camera and answers a question about what it sees.",
    ),
    (
        "run_mission",
        "Plans and runs a mission: walking, turning, picking and placing.",
    ),
    (
        "health_check",
        "Checks the robot's connections, cameras, localization and executor.",
    ),
    (
        "memory",
        "Remembers what the operator asks it to, across sessions, and forgets it.",
    ),
    (
        "ros_graph",
        "Lists the ROS topics, services, actions or nodes, or describes one.",
    ),
    ("plot", "Plots a number from a topic over time in the app."),
    (
        "recall",
        "What the robot did lately and what happened to an object.",
    ),
];

/// Tools a home robot's agent might also be given, none of them right for these requests.
const DISTRACTORS: [(&str, &str); 40] = [
    ("get_weather", "The weather forecast for a city."),
    ("send_email", "Sends an email."),
    ("read_calendar", "Reads the operator's calendar."),
    ("translate_text", "Translates text between languages."),
    ("set_timer", "Sets a kitchen timer."),
    ("play_music", "Plays music on the speakers."),
    ("order_groceries", "Orders groceries online."),
    ("read_news", "Reads the latest news."),
    ("lights_on", "Turns the lights in a room on."),
    ("lights_off", "Turns the lights in a room off."),
    ("set_thermostat", "Sets the heating's temperature."),
    ("lock_door", "Locks the front door."),
    ("open_blinds", "Opens the blinds in a room."),
    ("vacuum_room", "Starts the robot vacuum in a room."),
    ("call_contact", "Calls a contact by phone."),
    ("take_note", "Writes a note in the notes app."),
    ("search_web", "Searches the web."),
    ("convert_units", "Converts between units."),
    ("calculate", "Evaluates arithmetic."),
    ("list_devices", "Lists the smart home's devices."),
    (
        "doorbell_camera",
        "Shows the doorbell camera's last visitor.",
    ),
    ("water_plants", "Starts the plant watering."),
    ("feed_pet", "Starts the pet feeder."),
    ("start_washer", "Starts the washing machine."),
    ("tv_power", "Turns the TV on or off."),
    ("set_alarm", "Sets a wake-up alarm."),
    ("read_mail", "Reads the latest emails."),
    ("book_taxi", "Books a taxi."),
    ("track_package", "Tracks a parcel."),
    ("recipe_search", "Finds a recipe."),
    ("stock_price", "A stock's price."),
    ("define_word", "A word's definition."),
    ("currency_rate", "An exchange rate."),
    ("air_quality", "The indoor air quality."),
    ("energy_use", "The home's energy use today."),
    ("garage_door", "Opens or closes the garage door."),
    ("sprinklers", "Runs the garden sprinklers."),
    ("intercom", "Speaks through the intercom."),
    ("photo_album", "Shows photos from an album."),
    ("battery_report", "The phone's battery level."),
];

/// Requests with the one tool that should come first.
const CASES: [(&str, &str); 10] = [
    ("Where is the red mug?", "find_objects"),
    ("What can you see right now?", "look"),
    ("Turn left 90 degrees.", "run_mission"),
    ("Walk to the kitchen.", "run_mission"),
    ("Check the robot's health.", "health_check"),
    ("Remember that the kitchen door sticks.", "memory"),
    ("Which ROS nodes are running?", "ros_graph"),
    ("Plot the robot's forward speed.", "plot"),
    ("What did you do in the last hour?", "recall"),
    ("Is there a chair in the bedroom?", "find_objects"),
];

fn tools(n: usize, first: &Arc<Mutex<Option<String>>>) -> Vec<LoopTool> {
    OWN.iter()
        .chain(DISTRACTORS.iter())
        .take(n)
        .map(|(name, description)| {
            let (name, first) = ((*name).to_owned(), Arc::clone(first));
            let called = name.clone();
            LoopTool {
                name,
                description: (*description).to_owned(),
                parameters: json!({"type": "object", "properties": {
                    "query": {"type": "string", "description": "What to look for or do, in words."}
                }}),
                invoke: Arc::new(move |_args: Value| {
                    first
                        .lock()
                        .unwrap_or_else(std::sync::PoisonError::into_inner)
                        .get_or_insert_with(|| called.clone());
                    Box::pin(async { json!({"ok": true}) })
                }),
            }
        })
        .collect()
}

#[tokio::test]
#[ignore = "needs the local model server"]
async fn first_calls_as_the_tool_list_grows() {
    let config = ModelsConfig::parse(LOCAL_ONLY).unwrap();
    let source: Arc<dyn AgentSource> = Arc::new(Llm::new(Router::new(
        config,
        Ledger::default(),
        PrivacyMode::Sim,
    )));
    println!("| Tools | Right first tool | Mean s | Prompt tokens |\n|---|---|---|---|");
    for n in [8, 16, 32, 48] {
        let (mut right, mut seconds, tokens) = (0, 0.0, Arc::new(AtomicU64::new(0)));
        for (request, want) in CASES {
            let first = Arc::new(Mutex::new(None));
            let offered = tools(n, &first);
            let used = Arc::clone(&tokens);
            let setup = TurnSetup {
                preamble: PREAMBLE,
                max_turns: 2,
                tools: &offered,
                started: Arc::new(AtomicBool::new(false)),
                window: None,
                on_call: Some(Arc::new(move |cost: llm::CallCost| {
                    used.fetch_max(cost.input_tokens, Ordering::Relaxed);
                })),
                delta: None,
            };
            let began = Instant::now();
            let _ = llm::chat(
                "qwen3.5-9b-local",
                Arc::clone(&source),
                setup,
                &mut History::default(),
                request,
            )
            .await;
            seconds += began.elapsed().as_secs_f64();
            let got = first.lock().unwrap().clone();
            if got.as_deref() == Some(want) {
                right += 1;
            } else {
                println!("   {n} tools: \"{request}\" called {got:?}, wanted {want}");
            }
        }
        println!(
            "| {n} | {right} of {} | {:.1} | {} |",
            CASES.len(),
            seconds / f64::from(u8::try_from(CASES.len()).unwrap_or(u8::MAX)),
            tokens.load(Ordering::Relaxed)
        );
    }
}
