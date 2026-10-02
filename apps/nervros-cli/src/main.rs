//! The `nervros-cli` binary; the commands are in the library.

use clap::Parser as _;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let _telemetry = nervros_core::telemetry::init();
    nervros_cli::run(nervros_cli::Cli::parse()).await
}
