//! One tool-calling turn on each wire NervROS speaks, replayed from a recording of the real
//! provider: Claude's Messages, Gemini's Interactions and an OpenAI-compatible server (llama.cpp)
//! streaming. Each turn runs through NervROS's own client and turn loop with no key and no
//! network, so a rig upgrade that reads a reply differently shows here.
//!
//! `RIG_PROVIDER_TEST_MODE=record` asks the providers instead and writes the recordings again,
//! credentials scrubbed: Claude's key from `ANTHROPIC_API_KEY`, a free-tier Gemini key from
//! `GEMINI_API_KEY`, and a llama.cpp server on `http://127.0.0.1:8081/v1`
//! (`scripts/local-llm.sh start`).

use std::io::Write as _;
use std::path::Path;
use std::sync::{Arc, Mutex};

use nervros_core::llm::{AgentSource, CallCost, History, Llm, LoopTool, TurnSetup, chat};
use nervros_core::providers::ModelsConfig;
use nervros_core::providers::ledger::Ledger;
use nervros_core::providers::router::{PrivacyMode, Router};
use rig_cassette::http::{CassetteSpec, ProviderCassette};
use serde_json::{Value, json};

/// Where the recordings are kept.
const CASSETTES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/cassettes");

/// The recording of a scenario, matched by the shape of each request: a reworded prompt still
/// replays.
fn spec(scenario: &'static str) -> CassetteSpec {
    CassetteSpec::new(scenario).shape_matched()
}

/// What one turn did.
struct Turn {
    reply: String,
    /// Each `find_objects` call's arguments.
    calls: Vec<Value>,
    /// What each model call was charged.
    costs: Vec<CallCost>,
    history: History,
}

/// One turn asking where the red mug is, on the model `m` of a models file, with a
/// `find_objects` that finds it in the kitchen.
#[expect(
    clippy::expect_used,
    reason = "a test helper: a step that fails, fails the test"
)]
async fn ask_for_the_mug(models: &str) -> Turn {
    let config = ModelsConfig::parse(models).expect("a valid models file");
    let llm = Llm::new(Router::new(config, Ledger::default(), PrivacyMode::Sim));
    let source: Arc<dyn AgentSource> = Arc::new(llm);
    let calls = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::clone(&calls);
    let tools = [LoopTool {
        name: "find_objects".to_owned(),
        description: "Finds objects the robot has seen, by what they are: their ids and rooms."
            .to_owned(),
        parameters: json!({
            "type": "object",
            "properties": {"query": {"type": "string", "description": "What to look for"}},
            "required": ["query"]
        }),
        invoke: Arc::new(move |args| {
            seen.lock().expect("not poisoned").push(args);
            Box::pin(async {
                json!({"objects": [{"id": "O12", "label": "red mug", "room": "kitchen"}]})
            })
        }),
    }];
    let costs = Arc::new(Mutex::new(Vec::new()));
    let charged = Arc::clone(&costs);
    let setup = TurnSetup {
        preamble: "You are a home robot's assistant. Find things with your tool, then answer in \
                   one sentence.",
        max_turns: 4,
        tools: &tools,
        started: Arc::default(),
        window: None,
        on_call: Some(Arc::new(move |cost| {
            charged.lock().expect("not poisoned").push(cost);
        })),
        // Taken only by a model that streams; the reply is checked whole.
        delta: Some(Arc::new(|_: &str| {})),
    };
    let mut history = History::default();
    let reply = chat("m", source, setup, &mut history, "Where is the red mug?")
        .await
        .expect("the turn answers");
    let calls = calls.lock().expect("not poisoned").clone();
    let costs = costs.lock().expect("not poisoned").clone();
    Turn {
        reply,
        calls,
        costs,
        history,
    }
}

/// What every wire's turn must do: find the mug with the tool, say where it is, charge each call,
/// and keep a history that saves and loads.
#[expect(
    clippy::expect_used,
    reason = "a test helper: a step that fails, fails the test"
)]
fn check(turn: &Turn) {
    let asked_for_the_mug =
        |args: &Value| args["query"].as_str().is_some_and(|q| q.contains("mug"));
    assert!(turn.calls.iter().any(asked_for_the_mug), "{:?}", turn.calls);
    assert!(
        turn.reply.to_lowercase().contains("kitchen"),
        "{}",
        turn.reply
    );
    assert!(
        turn.costs.len() >= 2
            && turn
                .costs
                .iter()
                .all(|c| c.input_tokens > 0 && c.output_tokens > 0),
        "{:?}",
        turn.costs
    );
    let dir = tempfile::tempdir().expect("a temp dir");
    let file = dir.path().join("turn.history.json");
    turn.history.save(&file).expect("the history saves");
    assert_eq!(History::load(&file).expect("and loads"), turn.history);
}

/// The key a recording session hands out (the real one only when recording), in a file for the
/// models file to name.
#[expect(
    clippy::expect_used,
    reason = "a test helper: a step that fails, fails the test"
)]
fn key_file(key: &str) -> tempfile::NamedTempFile {
    let mut file = tempfile::NamedTempFile::new().expect("a temp file");
    file.write_all(key.as_bytes()).expect("the key written");
    file
}

#[tokio::test]
async fn claude_finds_the_mug_with_a_tool() {
    let cassette = ProviderCassette::start(
        Path::new(CASSETTES),
        "anthropic",
        spec("find_mug"),
        "https://api.anthropic.com",
    )
    .await;
    let key = key_file(&cassette.api_key("ANTHROPIC_API_KEY"));
    let spent = tempfile::tempdir().expect("a temp dir");
    let models = format!(
        r#"
[[provider]]
id = "anthropic"
kind = "anthropic"
base_url = "{base}"
key = {{ file = "{key}" }}
budget = {{ usd = 1.0, spent_file = "{spent}" }}

[[model]]
id = "m"
provider = "anthropic"
model = "claude-haiku-5-5"
tools = true
max_tokens = 1024
price = {{ input = 0.10, output = 0.50, cache_read = 0.01, cache_write = 0.125 }}

[roles]
routine = ["m"]
"#,
        base = cassette.base_url(),
        key = key.path().display(),
        spent = spent.path().join("spend.json").display(),
    );
    let turn = ask_for_the_mug(&models).await;
    check(&turn);
    cassette.finish().await;
}

#[tokio::test]
async fn gemini_finds_the_mug_with_a_tool() {
    let cassette = ProviderCassette::start(
        Path::new(CASSETTES),
        "gemini",
        spec("interactions_api/find_mug"),
        "https://generativelanguage.googleapis.com",
    )
    .await;
    let key = key_file(&cassette.api_key("GEMINI_API_KEY"));
    let models = format!(
        r#"
[[provider]]
id = "gemini"
kind = "gemini_interactions"
base_url = "{base}"
key = {{ file = "{key}" }}

[[model]]
id = "m"
provider = "gemini"
model = "gemini-3.1-flash-lite"
tools = true

[roles]
routine = ["m"]
"#,
        base = cassette.base_url(),
        key = key.path().display(),
    );
    let turn = ask_for_the_mug(&models).await;
    check(&turn);
    cassette.finish().await;
}

#[tokio::test]
async fn a_local_server_streams_a_turn_that_finds_the_mug() {
    let cassette = ProviderCassette::start(
        Path::new(CASSETTES),
        "llamacpp",
        spec("find_mug_streamed"),
        "http://127.0.0.1:8081/v1",
    )
    .await;
    let models = format!(
        r#"
[[provider]]
id = "local"
kind = "openai_compat"
base_url = "{base}"

[[model]]
id = "m"
provider = "local"
model = "qwen3.5-9b"
tools = true
stream = true
max_tokens = 1024

[roles]
routine = ["m"]
"#,
        base = cassette.base_url(),
    );
    let turn = ask_for_the_mug(&models).await;
    check(&turn);
    cassette.finish().await;
}
