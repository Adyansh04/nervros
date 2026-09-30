//! Live checks against the local model server (`scripts/local-llm.sh start`).
//!
//! Ignored by default; run with `cargo test -p nervros-core --test local_model -- --ignored`.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use nervros_core::llm::{Ask, DynamicTool, ImageFormat, ImageInput, Llm};
use nervros_core::providers::ledger::Ledger;
use nervros_core::providers::router::{PrivacyMode, Router};
use nervros_core::providers::{ModelsConfig, Role};
use rig::agent::tool::ToolOutput;
use rig::completion::Prompt as _;

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

#[expect(
    clippy::unwrap_used,
    reason = "a test helper; a bad fixture should fail loudly"
)]
fn llm() -> Llm {
    let config = ModelsConfig::parse(LOCAL_ONLY).unwrap();
    Llm::new(Router::new(config, Ledger::default(), PrivacyMode::Sim))
}

#[tokio::test]
#[ignore = "needs the local model server"]
async fn describes_an_image() {
    let image = ImageInput {
        bytes: std::fs::read("tests/fixtures/red.png").unwrap(),
        format: ImageFormat::Png,
    };
    let answer = llm()
        .ask(Ask {
            role: Role::Routine,
            preamble: "Answer in one word.",
            prompt: "What colour fills this image?",
            image: Some(image),
        })
        .await
        .unwrap();
    assert_eq!(answer.model, "qwen3.5-9b-local");
    assert!(
        answer.text.to_lowercase().contains("red"),
        "{}",
        answer.text
    );
}

#[tokio::test]
#[ignore = "needs the local model server"]
async fn calls_a_runtime_declared_tool_and_uses_its_result() {
    let llm = llm();
    let model = llm
        .router()
        .config()
        .model("qwen3.5-9b-local")
        .unwrap()
        .clone();
    let called = Arc::new(AtomicBool::new(false));
    let flag = Arc::clone(&called);
    let tool = DynamicTool::new(
        "find_objects",
        "Find objects the robot has seen, by label. Returns their ids and rooms.",
        serde_json::json!({
            "type": "object",
            "properties": {"query": {"type": "string", "description": "What to look for"}},
            "required": ["query"],
            "additionalProperties": false
        }),
        move |_cx, args| {
            let flag = Arc::clone(&flag);
            Box::pin(async move {
                flag.store(true, Ordering::SeqCst);
                let query = args["query"].as_str().unwrap_or_default().to_owned();
                Ok(ToolOutput::json(serde_json::json!({
                    "found": true,
                    "objects": [{"id": "O12", "label": query, "room": "kitchen"}]
                })))
            })
        },
    );
    let agent = llm
        .agent_builder(&model)
        .unwrap()
        .preamble("You are a robot's assistant. Use tools to answer, then reply in one sentence.")
        .dynamic_tool(tool)
        .default_max_turns(4)
        .build();
    let reply = agent.prompt("Where is the red ball?").await.unwrap();
    assert!(
        called.load(Ordering::SeqCst),
        "the model never called the tool"
    );
    assert!(reply.to_lowercase().contains("kitchen"), "{reply}");
}
