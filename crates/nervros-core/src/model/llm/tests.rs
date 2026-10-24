use std::sync::Arc;

use rig::completion::Message;
use rig::message::{AssistantContent, CallId, ToolName, UserContent};

use super::client::Llm;
use super::history::{CUT_MARK, History, is_user_text, result_text};
use super::turn::{cancel_reason, plain, streamed, without_provider_body};
use super::*;
use crate::providers::Role;
use crate::providers::router::Router;

fn exchange(n: usize, chars: usize) -> Vec<Message> {
    vec![
        Message::User {
            content: vec![UserContent::text(format!("request {n}"))],
        },
        Message::Assistant {
            id: None,
            content: vec![AssistantContent::text("x".repeat(chars))],
        },
    ]
}

#[test]
fn a_long_history_is_cut_from_the_oldest_exchange_and_summarised() {
    let history = History((0..10).flat_map(|n| exchange(n, 3000)).collect());
    assert!(history.size() > 9000, "{}", history.size());
    let (n, text) = history.older(2500).unwrap();
    assert_eq!(n % 2, 0, "a cut starts at an operator's message");
    assert!(text.starts_with("Operator: request 0"), "{text}");
    let mut summarised = history.clone();
    summarised.summarised(n, "the operator asked for ten things");
    let first = summarised.exchanges();
    assert!(
        !first[0].0 && first[0].1.contains("ten things"),
        "a summary is not the operator's words: {first:?}"
    );
    assert!(summarised.size() <= 2600, "{}", summarised.size());
    let mut squeezed = history;
    squeezed.squeeze(2500);
    assert!(squeezed.size() <= 2500 && is_user_text(&squeezed.0[0]));
}

#[test]
fn old_results_are_cut_once_and_up_to_here_counts_the_operator_only() {
    let said = |text: &str| Message::User {
        content: vec![UserContent::text(text)],
    };
    let found = |call: &str, text: String| {
        Message::tool_result(
            CallId::from_wire(call),
            ToolName::new("find_objects").unwrap(),
            text,
        )
    };
    let mut history = History(vec![
        said("where is the mug?"),
        found("c1", "y".repeat(2000)),
        found("c3", "succeeded".to_owned()),
        said(&format!("{REPORT_MARK}\nthe mission succeeded")),
        said("and the basket?"),
        found("c2", "z".repeat(2000)),
    ]);
    let before = history.size();
    history.mask();
    assert!(history.size() < before, "a cut never grows the history");
    let results: Vec<String> = history
        .0
        .iter()
        .filter_map(|m| match m {
            Message::User { content } => content.iter().find_map(|c| match c {
                UserContent::ToolResult(r) => Some(result_text(&r.content)),
                _ => None,
            }),
            Message::Assistant { .. } | Message::System { .. } => None,
        })
        .collect();
    assert!(
        results[0].starts_with(CUT_MARK) && results[0].len() < 300,
        "{results:?}"
    );
    assert_eq!(results[1], "succeeded", "a short one stays as it is");
    assert_eq!(results[2].len(), 2000, "the newest request's result stays");
    let once = history.clone();
    history.mask();
    assert_eq!(history.0, once.0, "a cut result is not cut again");

    let (n, text) = history.before_last(1).unwrap();
    assert_eq!(n, 4, "the report is not the operator's");
    assert!(
        text.contains("where is the mug?") && !text.contains("basket"),
        "{text}"
    );
    assert_eq!(
        history.before_last(0).unwrap().0,
        6,
        "keeping none condenses it all"
    );
    assert!(
        history.before_last(2).is_none(),
        "nothing is before the operator's first"
    );
    assert!(history.before_last(3).is_none());
}

#[tokio::test]
async fn a_streamed_turn_adds_to_the_history_it_was_given() {
    use rig::test_utils::{MockCompletionModel, MockStreamEvent};
    let model = MockCompletionModel::from_stream_turns([vec![
        MockStreamEvent::text("Noted."),
        MockStreamEvent::final_response_with_default_usage(),
    ]]);
    let agent = AgentBuilder::new(model).build();
    let mut history = History::sample(2, 10);
    let reply = streamed(&agent, "and one more", &mut history, &|_| {})
        .await
        .unwrap();
    assert_eq!(reply, "Noted.");
    let exchanges = history.exchanges();
    assert_eq!(exchanges.len(), 6, "{exchanges:?}");
    assert_eq!(exchanges[0].1, "request 0");
    assert_eq!(exchanges[4].1, "and one more");
}

#[test]
fn a_tool_call_written_as_text_is_not_shown() {
    assert_eq!(plain("Done.".into()), "Done.");
    assert_eq!(
        plain("I checked it. <tool_call> <function=ros_graph> </function>".into()),
        "I checked it."
    );
    assert!(plain("<tool_call>x".into()).contains("ran out of steps"));
}

#[tokio::test]
async fn a_provider_without_its_key_is_passed_over_with_the_reason() {
    let config = crate::providers::ModelsConfig::parse(
        "[[provider]]\nid = \"gemini\"\nkind = \"gemini_interactions\"\n\
         key = { file = \"/nonexistent/gemini.key\" }\n\
         [[model]]\nid = \"g\"\nprovider = \"gemini\"\nmodel = \"m\"\nvision = true\n\
         [roles]\nsegment = [\"g\"]\n",
    )
    .unwrap();
    let llm = Llm::new(Router::new(
        config,
        crate::providers::ledger::Ledger::default(),
        crate::providers::router::PrivacyMode::Sim,
    ));
    let err = llm
        .ask(Ask {
            role: Role::Segment,
            preamble: "",
            prompt: "the floor",
            images: Vec::new(),
        })
        .await
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("g failed") && err.contains("/nonexistent/gemini.key"),
        "{err}"
    );
}

#[test]
fn the_image_goes_first_unless_the_text_must() {
    let image = ImageInput {
        bytes: vec![0xFF, 0xD8],
        format: ImageFormat::Jpeg,
    };
    let shots = [Shot::bare(image)];
    let first = |text_first| match user_message("t", &shots, text_first) {
        Message::User { content } => matches!(content.first(), Some(UserContent::Text(_))),
        _ => unreachable!("a user message"),
    };
    assert!(!first(false));
    assert!(first(true));
    let labelled = Shot {
        label: "end:".to_owned(),
        image: shots[0].image.clone(),
    };
    let kinds = match user_message("t", &[shots[0].clone(), labelled], false) {
        Message::User { content } => content
            .iter()
            .map(|c| match c {
                UserContent::Text(t) => t.text.clone(),
                UserContent::Image(_) => "image".to_owned(),
                _ => unreachable!("text and images only"),
            })
            .collect::<Vec<_>>(),
        _ => unreachable!("a user message"),
    };
    assert_eq!(kinds, ["image", "end:", "image", "t"]);
}

#[test]
fn a_cancelled_turn_says_why_once() {
    assert_eq!(
        cancel_reason("PromptCancelled: m1: its per-minute limit is used up", "m1"),
        "its per-minute limit is used up"
    );
    assert_eq!(
        cancel_reason("PromptCancelled: the turn was stopped", "m1"),
        "the turn was stopped"
    );
    assert_eq!(cancel_reason("status 429", "m1"), "status 429");
}

#[test]
fn a_provider_error_keeps_its_words_and_loses_its_body() {
    let raw = r#"CompletionError: ProviderResponseError: status 429 Too Many Requests: {"error":{"message":"Provider returned error","code":429,"metadata":{"raw":"qwen/qwen3.8-27b:free is temporarily rate-limited upstream.","provider_name":"ModelRun"}},"user_id":"user_0000example"}"#;
    let shown = without_provider_body(raw);
    assert_eq!(
        shown,
        "CompletionError: ProviderResponseError: status 429 Too Many Requests: qwen/qwen3.8-27b:free is temporarily rate-limited upstream."
    );
    let plain = r#"status 403 Forbidden: {"error":{"message":"only available on agentic harnesses","code":403}}"#;
    assert_eq!(
        without_provider_body(plain),
        "status 403 Forbidden: only available on agentic harnesses"
    );
    assert_eq!(
        without_provider_body("connection refused"),
        "connection refused"
    );
    assert_eq!(
        without_provider_body("oops: {not json"),
        "oops: details withheld"
    );
}

#[test]
fn a_failure_says_how_the_model_fares() {
    use super::client::setback;
    use reqwest::StatusCode;
    use rig::ErrorKind;
    let reply = ErrorKind::ProviderResponse;
    let mut headers = reqwest::header::HeaderMap::new();
    headers.insert(
        "retry-after",
        reqwest::header::HeaderValue::from_static("30"),
    );
    assert_eq!(
        setback(ErrorKind::Http, None, None),
        Some(Setback::Unreachable)
    );
    assert_eq!(
        setback(reply, Some(StatusCode::TOO_MANY_REQUESTS), Some(&headers)),
        Some(Setback::Limited(Duration::from_secs(30)))
    );
    assert_eq!(
        setback(reply, Some(StatusCode::TOO_MANY_REQUESTS), None),
        Some(Setback::Limited(Duration::from_mins(1)))
    );
    assert_eq!(
        setback(reply, Some(StatusCode::SERVICE_UNAVAILABLE), None),
        Some(Setback::Busy)
    );
    assert_eq!(setback(reply, Some(StatusCode::BAD_REQUEST), None), None);
}

#[test]
fn a_429_parks_a_silent_server_goes_last_and_a_busy_one_stays() {
    #[derive(Default)]
    struct Kept(std::sync::Mutex<Vec<String>>);
    impl AgentSource for Kept {
        fn candidates(&self, _: Role, _: crate::providers::router::Need) -> Vec<String> {
            Vec::new()
        }
        fn builder(&self, id: &str) -> Result<AgentBuilder, LlmError> {
            Err(LlmError::Client {
                provider: String::new(),
                message: id.to_owned(),
            })
        }
        fn take_request(&self, _: &str) -> Result<(), String> {
            Ok(())
        }
        fn park(&self, id: &str, wait: Duration) {
            crate::lock(&self.0).push(format!("park {id} {}", wait.as_secs()));
        }
        fn set_aside(&self, id: &str, wait: Duration) {
            crate::lock(&self.0).push(format!("last {id} {}", wait.as_secs()));
        }
    }
    let kept = Kept::default();
    kept.set_back("a", Setback::Limited(Duration::from_secs(30)));
    kept.set_back("b", Setback::Unreachable);
    kept.set_back("c", Setback::Busy);
    assert_eq!(*crate::lock(&kept.0), ["park a 30", "last b 60"]);
}

#[test]
fn earlier_reasoning_is_dropped_and_turns_left_empty_go_with_it() {
    use rig::message::{Issuer, Reasoning, Sealed};
    let thought = |text: &str| {
        AssistantContent::Reasoning(Sealed::new(
            Issuer::from("gcp.gemini"),
            Reasoning::new(text),
        ))
    };
    let mut history = History(vec![
        Message::User {
            content: vec![UserContent::text("where is the mug?")],
        },
        Message::Assistant {
            id: None,
            content: vec![
                thought("the kitchen, probably"),
                AssistantContent::text("In the kitchen."),
            ],
        },
        Message::Assistant {
            id: None,
            content: vec![thought("nothing to add")],
        },
    ]);
    history.drop_reasoning();
    assert_eq!(history.0.len(), 2);
    assert!(matches!(
        &history.0[1],
        Message::Assistant { content, .. } if content.len() == 1
            && matches!(&content[0], AssistantContent::Text(t) if t.text == "In the kitchen.")
    ));
}

#[tokio::test]
async fn a_call_recovered_from_an_invalid_tool_call_is_charged_once_like_the_rest() {
    use rig::test_utils::{MockCompletionModel, MockTurn};
    #[derive(Default)]
    struct Billed(std::sync::Mutex<Vec<CallCost>>);
    impl AgentSource for Billed {
        fn candidates(&self, _: Role, _: crate::providers::router::Need) -> Vec<String> {
            Vec::new()
        }
        fn builder(&self, _: &str) -> Result<AgentBuilder, LlmError> {
            let used = |input, output| rig::completion::Usage {
                input_tokens: Some(input),
                output_tokens: Some(output),
                ..rig::completion::Usage::default()
            };
            Ok(AgentBuilder::new(MockCompletionModel::from_turns([
                MockTurn::tool_call("c1", "no_such_tool", serde_json::json!({}))
                    .with_usage(used(100, 10)),
                MockTurn::text("Done.").with_usage(used(50, 5)),
            ])))
        }
        fn take_request(&self, _: &str) -> Result<(), String> {
            Ok(())
        }
        fn park(&self, _: &str, _: Duration) {}
        fn spent(&self, _: &str, cost: CallCost) {
            crate::lock(&self.0).push(cost);
        }
    }
    let billed = Arc::new(Billed::default());
    let setup = TurnSetup {
        preamble: "",
        max_turns: 4,
        tools: &[],
        started: Arc::default(),
        window: None,
        on_call: None,
        delta: None,
    };
    let mut history = History::default();
    let source: Arc<dyn AgentSource> = billed.clone();
    let reply = chat("m", source, setup, &mut history, "go").await.unwrap();
    assert_eq!(reply, "Done.");
    let costs = crate::lock(&billed.0).clone();
    assert_eq!(
        costs.iter().map(|c| c.input_tokens).sum::<u64>(),
        150,
        "{costs:?}"
    );
    assert_eq!(
        costs.iter().map(|c| c.output_tokens).sum::<u64>(),
        15,
        "{costs:?}"
    );
}
