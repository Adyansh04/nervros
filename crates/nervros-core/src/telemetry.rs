//! Tracing for the apps: logs to stderr as `RUST_LOG` asks and, built with the `otlp` feature and
//! run with `OTEL_EXPORTER_OTLP_ENDPOINT` set, spans to an OpenTelemetry collector over OTLP/HTTP.
//!
//! A turn is one span named as the `GenAI` conventions name an agent run (`invoke_agent`), so rig
//! records its token use on it and nests its `chat` and `execute_tool` spans inside. Work that
//! outlives its turn (a mission, a schedule, a watch) is a job: a trace of its own, linked to the
//! turn that started it. Prompts, replies and tool results stay out: rig records them only when an
//! agent asks it to, and none here does.

use tracing_subscriber::layer::SubscriberExt as _;
use tracing_subscriber::util::SubscriberInitExt as _;
use tracing_subscriber::{EnvFilter, Layer as _};

/// Keeps the span exporter for as long as the app runs; dropped, it sends what is left.
#[must_use = "dropping it stops the span export"]
pub struct Telemetry {
    #[cfg(feature = "otlp")]
    provider: Option<opentelemetry_sdk::trace::SdkTracerProvider>,
}

impl Drop for Telemetry {
    fn drop(&mut self) {
        #[cfg(feature = "otlp")]
        if let Some(provider) = self.provider.take()
            && let Err(e) = provider.shutdown()
        {
            tracing::warn!(error = %e, "the last spans were not sent");
        }
    }
}

/// Installs the process's tracing: once, at start.
pub fn init() -> Telemetry {
    let logs = tracing_subscriber::fmt::layer()
        .with_writer(std::io::stderr)
        .with_filter(EnvFilter::from_default_env());
    let registry = tracing_subscriber::registry().with(logs);
    #[cfg(feature = "otlp")]
    match otlp::provider() {
        Ok(Some(provider)) => {
            use opentelemetry::trace::TracerProvider as _;
            let spans = tracing_opentelemetry::layer()
                .with_tracer(provider.tracer("nervros"))
                .with_filter(otlp::targets());
            registry.with(spans).init();
            return Telemetry {
                provider: Some(provider),
            };
        }
        Ok(None) => {}
        Err(e) => {
            registry.init();
            tracing::warn!(error = %e, "spans are not exported");
            return Telemetry { provider: None };
        }
    }
    registry.init();
    Telemetry {
        #[cfg(feature = "otlp")]
        provider: None,
    }
}

/// The span of work that outlives the turn that started it, such as a mission: a trace of its own,
/// linked to the turn's. `kind` names it in a trace viewer; record `nervros.outcome` at its end.
pub(crate) fn job(kind: &'static str, id: &str, what: &str) -> tracing::Span {
    let span = tracing::info_span!(
        parent: None,
        "nervros.job",
        otel.name = kind,
        nervros.job.id = id,
        nervros.job.what = what,
        nervros.outcome = tracing::field::Empty,
    );
    span.follows_from(tracing::Span::current());
    span
}

#[cfg(feature = "otlp")]
mod otlp {
    use opentelemetry_otlp::SpanExporter;
    use opentelemetry_sdk::Resource;
    use opentelemetry_sdk::trace::SdkTracerProvider;
    use tracing::Level;
    use tracing_subscriber::filter::Targets;

    /// A provider sending where the standard variables say, or none when neither is set: the
    /// exporter reads them itself, adding `/v1/traces` to the general one.
    pub(super) fn provider() -> Result<Option<SdkTracerProvider>, String> {
        let set = |v: &str| std::env::var_os(v).is_some_and(|v| !v.is_empty());
        if !set("OTEL_EXPORTER_OTLP_ENDPOINT") && !set("OTEL_EXPORTER_OTLP_TRACES_ENDPOINT") {
            return Ok(None);
        }
        let exporter = SpanExporter::builder()
            .with_http()
            .build()
            .map_err(|e| e.to_string())?;
        Ok(Some(
            SdkTracerProvider::builder()
                .with_batch_exporter(exporter)
                .with_resource(Resource::builder().with_service_name("nervros").build())
                .build(),
        ))
    }

    /// The agent's spans and rig's, not the libraries' underneath them.
    pub(super) fn targets() -> Targets {
        Targets::new()
            .with_target("nervros_core", Level::INFO)
            .with_target("rig", Level::INFO)
            .with_target("rig_agent", Level::INFO)
            .with_target("rig_core", Level::INFO)
    }
}
