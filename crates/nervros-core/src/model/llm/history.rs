//! The conversation: its messages, their size in tokens, and how it is cut, condensed and kept.

use rig::completion::Message;
use rig::message::{AssistantContent, ToolResultContent, UserContent};

/// The conversation so far, tool calls and results included, in rig's message form.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct History(pub(super) Vec<Message>);

impl History {
    /// Drops the oldest messages beyond `max`, starting at a user message so no tool result is
    /// left without its call.
    pub fn trim(&mut self, max: usize) {
        if self.0.len() <= max {
            return;
        }
        let mut start = self.0.len() - max;
        while start < self.0.len() && !is_user_text(&self.0[start]) {
            start += 1;
        }
        self.0.drain(..start);
    }
}

impl History {
    /// Roughly how many tokens it takes.
    #[must_use]
    pub fn size(&self) -> usize {
        size(&self.0)
    }

    /// The part a summary would replace, as text, and how many messages it is: everything
    /// before the newest exchanges that fit `keep` tokens. `None` when that is nothing.
    #[must_use]
    pub fn older(&self, keep: usize) -> Option<(usize, String)> {
        let starts: Vec<usize> = (0..self.0.len())
            .filter(|&i| is_user_text(&self.0[i]))
            .collect();
        let split = starts
            .iter()
            .copied()
            .find(|&i| size(&self.0[i..]) <= keep)
            .unwrap_or_else(|| starts.last().copied().unwrap_or(0));
        (split > 0).then(|| (split, transcript(&self.0[..split])))
    }

    /// The part before the operator's last `keep` messages, as [`Self::older`] gives it: "condense
    /// up to here". `None` when there is nothing before them.
    #[must_use]
    pub fn before_last(&self, keep: usize) -> Option<(usize, String)> {
        let operator: Vec<usize> = (0..self.0.len())
            .filter(|&i| is_operator(&self.0[i]))
            .collect();
        let split = match keep {
            0 => self.0.len(),
            k => operator[operator.len().checked_sub(k)?],
        };
        (split > 0).then(|| (split, transcript(&self.0[..split])))
    }

    /// Cuts every tool result and image before the operator's newest message to a line. What was
    /// said and called stays word for word, which serves a later turn about as well as a summary
    /// and costs no model call.
    pub fn mask(&mut self) {
        let newest = self.0.iter().rposition(is_operator).unwrap_or(0);
        self.0 = cut_old(&self.0, self.0.len() - newest);
    }

    /// Replaces the first `n` messages with a summary of them.
    pub fn summarised(&mut self, n: usize, summary: &str) {
        let kept = self.0.split_off(n.min(self.0.len()));
        self.0 = vec![
            Message::User {
                content: vec![UserContent::text(format!(
                    "{SUMMARY_MARK}\n{}",
                    summary.trim()
                ))],
            },
            Message::Assistant {
                id: None,
                content: vec![AssistantContent::text("Noted.")],
            },
        ];
        self.0.extend(kept);
    }

    /// Cuts old results and images and drops the oldest exchanges until it fits `budget`.
    pub fn squeeze(&mut self, budget: usize) {
        self.0 = fit(cut_old(&self.0, 4), budget);
    }

    /// Records a request whose reply was lost after the tools had acted on it: the request, and
    /// what was done, as the reply.
    pub(crate) fn lost_reply(&mut self, request: &str, done: &[String]) {
        self.0.push(Message::User {
            content: vec![UserContent::text(request)],
        });
        self.0.push(Message::Assistant {
            id: None,
            content: vec![AssistantContent::text(format!(
                "(My reply was lost after I acted: {}.)",
                done.join("; ")
            ))],
        });
    }

    /// Writes it as JSON, for a later session to resume.
    ///
    /// # Errors
    ///
    /// The file cannot be written.
    pub fn save(&self, path: &std::path::Path) -> std::io::Result<()> {
        crate::persist::write_atomic(path, &serde_json::to_vec(&self.0)?)
    }

    /// Reads one [`Self::save`] wrote.
    ///
    /// # Errors
    ///
    /// The file cannot be read or is not a saved history.
    pub fn load(path: &std::path::Path) -> std::io::Result<Self> {
        Ok(Self(serde_json::from_slice(&std::fs::read(path)?)?))
    }

    /// The operator's messages and the replies, in order, for showing a resumed conversation; a
    /// robot's report or a summary comes as text of the robot's, not as the operator's words.
    #[must_use]
    pub fn exchanges(&self) -> Vec<(bool, String)> {
        self.0
            .iter()
            .filter_map(|m| match m {
                Message::User { content } if is_user_text(m) => Some((
                    is_operator(m),
                    content
                        .iter()
                        .filter_map(|c| match c {
                            UserContent::Text(t) => Some(t.text.as_str()),
                            _ => None,
                        })
                        .collect::<Vec<_>>()
                        .join(" "),
                )),
                Message::Assistant { content, .. } => {
                    let text: Vec<&str> = content
                        .iter()
                        .filter_map(|c| match c {
                            AssistantContent::Text(t) => Some(t.text.as_str()),
                            _ => None,
                        })
                        .collect();
                    (!text.is_empty()).then(|| (false, text.join(" ")))
                }
                _ => None,
            })
            .collect()
    }
}

#[cfg(test)]
impl History {
    /// `n` exchanges whose replies are `chars` long.
    pub(crate) fn sample(n: usize, chars: usize) -> Self {
        Self(
            (0..n)
                .flat_map(|i| {
                    [
                        Message::User {
                            content: vec![UserContent::text(format!("request {i}"))],
                        },
                        Message::Assistant {
                            id: None,
                            content: vec![AssistantContent::text("x".repeat(chars))],
                        },
                    ]
                })
                .collect(),
        )
    }
}

pub(super) fn is_user_text(m: &Message) -> bool {
    matches!(m, Message::User { content } if content.iter().all(|c| matches!(c, UserContent::Text(_))))
}

/// How a robot's report starts, as the model gets it.
pub(crate) const REPORT_MARK: &str = "[Report from the robot, not the operator]";

/// How a summary of the earlier conversation starts.
const SUMMARY_MARK: &str = "[The earlier conversation, summarised]";

/// How a tool result cut to save room starts.
pub(super) const CUT_MARK: &str = "[an earlier result, cut to save room]";

/// The operator's own words: not a robot's report or a summary, which come as user text too.
fn is_operator(m: &Message) -> bool {
    let Message::User { content } = m else {
        return false;
    };
    is_user_text(m)
        && !content.iter().any(|c| {
            matches!(c, UserContent::Text(t) if t.text.starts_with(REPORT_MARK) || t.text.starts_with(SUMMARY_MARK))
        })
}

/// Characters per token: JSON-heavy text runs about three, so this errs high.
const CHARS_PER_TOKEN: usize = 3;

/// A camera frame, whatever its bytes: a model sizes an image by its tiles, not its base64.
const IMAGE_TOKENS: usize = 800;

/// What a reply, and the next tool call with its arguments, need on top of the request.
pub const RESERVE_TOKENS: usize = 2048;

/// Roughly how many tokens text of `chars` characters takes.
#[must_use]
pub fn tokens_of(chars: usize) -> usize {
    chars.div_ceil(CHARS_PER_TOKEN)
}

fn json_len<T: serde::Serialize>(value: &T) -> usize {
    serde_json::to_string(value).map_or(0, |s| s.len())
}

/// Roughly how many tokens messages take on the wire.
pub(super) fn size(messages: &[Message]) -> usize {
    let (mut chars, mut images) = (0, 0);
    for m in messages {
        match m {
            Message::System { content } => chars += content.len(),
            Message::User { content } => {
                for c in content {
                    match c {
                        UserContent::Image(_) => images += 1,
                        UserContent::ToolResult(r) => {
                            for part in &r.content {
                                match part {
                                    ToolResultContent::Image(_) => images += 1,
                                    other => chars += json_len(other),
                                }
                            }
                        }
                        other => chars += json_len(other),
                    }
                }
            }
            Message::Assistant { content, .. } => {
                for c in content {
                    match c {
                        AssistantContent::Image(_) => images += 1,
                        other => chars += json_len(other),
                    }
                }
            }
        }
    }
    tokens_of(chars) + images * IMAGE_TOKENS
}

/// A tool result's text, its JSON as JSON and its images as a word.
pub(super) fn result_text(parts: &[ToolResultContent]) -> String {
    parts
        .iter()
        .map(|p| match p {
            ToolResultContent::Text(t) => t.text.clone(),
            ToolResultContent::Json { value } => value.to_string(),
            ToolResultContent::Image(_) => "[an image]".to_owned(),
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// The messages with every tool result and image before the last `keep` cut to a line; one cut
/// before stays as it is.
pub(super) fn cut_old(messages: &[Message], keep: usize) -> Vec<Message> {
    let edge = messages.len().saturating_sub(keep);
    let cut = |c: &UserContent| match c {
        UserContent::ToolResult(r) if !result_text(&r.content).starts_with(CUT_MARK) => {
            let mut r = r.clone();
            r.content = vec![ToolResultContent::text(format!(
                "{CUT_MARK} {}",
                crate::tools::clip(&result_text(&r.content), 160)
            ))];
            UserContent::ToolResult(r)
        }
        UserContent::Image(_) => UserContent::text("[an earlier image]"),
        other => other.clone(),
    };
    messages
        .iter()
        .enumerate()
        .map(|(i, m)| match m {
            Message::User { content } if i < edge => Message::User {
                content: content.iter().map(cut).collect(),
            },
            other => other.clone(),
        })
        .collect()
}

/// The newest messages that fit `budget`, from an operator's message on so that no tool result
/// is left without its call; the last exchange stays whatever its size.
pub(super) fn fit(mut messages: Vec<Message>, budget: usize) -> Vec<Message> {
    while size(&messages) > budget {
        let Some(next) = messages.iter().skip(1).position(is_user_text) else {
            break;
        };
        messages.drain(..=next);
    }
    messages
}

/// Messages as plain text, for a summary.
pub(super) fn transcript(messages: &[Message]) -> String {
    use std::fmt::Write as _;
    let mut out = String::new();
    for m in messages {
        match m {
            Message::User { content } => {
                for c in content {
                    match c {
                        UserContent::Text(t) => {
                            let _ = writeln!(out, "Operator: {}", t.text);
                        }
                        UserContent::ToolResult(r) => {
                            let text = crate::tools::clip(&result_text(&r.content), 300);
                            let _ = writeln!(out, "  {} returned: {text}", r.name);
                        }
                        UserContent::Image(_) => out.push_str("  (an image)\n"),
                        _ => {}
                    }
                }
            }
            Message::Assistant { content, .. } => {
                for c in content {
                    match c {
                        AssistantContent::Text(t) => {
                            let _ = writeln!(out, "Assistant: {}", t.text);
                        }
                        AssistantContent::ToolCall(call) => {
                            let args =
                                crate::tools::clip(&call.function.arguments.to_string(), 200);
                            let _ = writeln!(out, "  called {} {args}", call.function.name);
                        }
                        _ => {}
                    }
                }
            }
            Message::System { .. } => {}
        }
    }
    out
}
