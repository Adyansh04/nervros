//! The only module that imports `rig`.
//!
//! It turns a model from `models.toml` into a rig agent builder, and runs one prompt against the
//! router's candidates in order: a 429 or a server that does not answer parks the model, a busy
//! one (5xx) is asked once more, any other failure moves on, and the answer names the model that
//! gave it.

use std::time::Duration;

use base64::Engine as _;
use rig::completion::Message;
use rig::message::{ImageMediaType, UserContent};

use crate::providers::router::Skip;
use crate::providers::{Role, free_only};
use crate::secret::SecretError;

pub use rig::AgentBuilder;
pub use rig::agent::tool::DynamicTool;

mod client;
mod history;
mod roles;
#[cfg(test)]
mod tests;
mod turn;

pub use client::Llm;
use client::describe;
pub(crate) use history::REPORT_MARK;
pub use history::{History, RESERVE_TOKENS, tokens_of};
pub use roles::summarise;
pub use turn::{
    AgentSource, CallCost, LoopTool, OnCall, OnDelta, ToolFuture, TurnSetup, chat, fixed_cost,
};

/// The longest side of a frame a vision model gets: enough to read a label across a room, and
/// a fraction of the tokens a full 1280 px frame costs.
pub const MODEL_EDGE_PX: u32 = 768;

/// An image attached to a prompt.
#[derive(Debug, Clone)]
pub struct ImageInput {
    /// Encoded image bytes.
    pub bytes: Vec<u8>,
    /// JPEG or PNG.
    pub format: ImageFormat,
}

impl ImageInput {
    /// `img` as a JPEG, its longest side cut to [`MODEL_EDGE_PX`].
    ///
    /// # Errors
    ///
    /// The encoder failed.
    pub fn jpeg(img: &image::RgbImage) -> Result<Self, String> {
        let small = nervros_ros::image::capped(img, MODEL_EDGE_PX);
        let bytes = nervros_ros::image::encode_jpeg(&small, nervros_ros::image::JPEG_QUALITY)
            .map_err(|e| e.to_string())?;
        Ok(Self {
            bytes,
            format: ImageFormat::Jpeg,
        })
    }
}

/// Encodings the providers all accept.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageFormat {
    /// `image/jpeg`.
    Jpeg,
    /// `image/png`.
    Png,
}

/// One prompt to run.
#[derive(Debug, Clone)]
pub struct Ask<'a> {
    /// Which role's chain to use.
    pub role: Role,
    /// The system prompt.
    pub preamble: &'a str,
    /// The user text.
    pub prompt: &'a str,
    /// An optional image.
    pub image: Option<ImageInput>,
}

/// A model's reply and which model gave it.
#[derive(Debug, Clone)]
pub struct Answer {
    /// The reply text.
    pub text: String,
    /// The model id from `models.toml`.
    pub model: String,
    /// Prompt tokens, as the provider reported them.
    pub input_tokens: u64,
    /// Reply tokens, as the provider reported them.
    pub output_tokens: u64,
}

/// Why no model answered.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum LlmError {
    /// Every candidate was skipped or failed.
    #[error("no model could answer: {}", describe(.skipped, .failed))]
    NoModel {
        /// Models passed over and why.
        skipped: Vec<(String, Skip)>,
        /// Models tried and their errors.
        failed: Vec<(String, String)>,
    },
    /// A provider key could not be loaded.
    #[error("provider `{provider}`: {source}")]
    Key {
        /// The provider id.
        provider: String,
        /// The load error.
        source: SecretError,
    },
    /// A client could not be built.
    #[error("provider `{provider}`: {message}")]
    Client {
        /// The provider id.
        provider: String,
        /// What went wrong.
        message: String,
    },
    /// A model broke the free-only rule at request time.
    #[error(transparent)]
    NotFree(#[from] free_only::FreeOnlyError),
    /// The model or provider failed during a turn.
    #[error("{model}: {message}")]
    Turn {
        /// The model id.
        model: String,
        /// What went wrong.
        message: String,
        /// What the failure says about the model, when it says anything.
        setback: Option<Setback>,
    },
}

impl LlmError {
    /// What a turn's failure says about the model that failed it.
    #[must_use]
    pub fn setback(&self) -> Option<Setback> {
        match self {
            Self::Turn { setback, .. } => *setback,
            _ => None,
        }
    }
}

/// What a failed call says about the model that made it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Setback {
    /// The provider answered 429: set the model aside this long.
    Limited(Duration),
    /// The provider answered with a server error, as Gemini's 503 under load: it often answers a
    /// moment later.
    Busy,
    /// Nothing answered, as from a local server that is not running.
    Unreachable,
}

/// How long a model whose server does not answer is passed over: its turns go to the next model
/// without announcing the same failure each time.
const UNREACHABLE_PARK: Duration = Duration::from_mins(1);

/// How long before a busy model is asked once more.
pub const BUSY_RETRY: Duration = Duration::from_secs(2);

impl Setback {
    /// How long to set the model aside after this, if at all.
    #[must_use]
    pub fn park(self) -> Option<Duration> {
        match self {
            Self::Limited(wait) => Some(wait),
            Self::Unreachable => Some(UNREACHABLE_PARK),
            Self::Busy => None,
        }
    }
}

/// A user message with optional image content, image first as most vision models prefer, or
/// after the text when `text_first`.
#[must_use]
pub fn user_message(text: &str, image: Option<&ImageInput>, text_first: bool) -> Message {
    let mut content = vec![UserContent::text(text)];
    if let Some(image) = image {
        let media = match image.format {
            ImageFormat::Jpeg => ImageMediaType::JPEG,
            ImageFormat::Png => ImageMediaType::PNG,
        };
        let data = base64::engine::general_purpose::STANDARD.encode(&image.bytes);
        content.push(UserContent::image_base64(data, Some(media), None));
    }
    if !text_first {
        content.rotate_left(1);
    }
    Message::User { content }
}
