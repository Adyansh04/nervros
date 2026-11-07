//! `nervros-cli mcp`: the profile's MCP servers as they answer now, and approving their tools.
//! A tool's definition is printed in full before it is pinned: approving is reading.

use std::path::Path;

use anyhow::{Context as _, Result, bail};
use nervros_core::mcp::{self, Lock, McpServer, Pin};
use nervros_core::tools::visible;

/// Asks `question` on the terminal: only "y" or "yes" approves, and no answer is a no.
fn confirmed(question: &str) -> Result<bool> {
    use std::io::Write as _;
    print!("  {question} [y/N] ");
    std::io::stdout().flush()?;
    let mut answer = String::new();
    std::io::stdin().read_line(&mut answer)?;
    Ok(matches!(answer.trim().to_lowercase().as_str(), "y" | "yes"))
}

/// Lists every server's named tools with their pins, or pins one server's tools.
pub async fn run(
    profile: &Path,
    action: Option<&str>,
    server: Option<&str>,
    tools: &[String],
) -> Result<()> {
    let profile = crate::load(profile)?;
    let lock_path = profile.resolve(Path::new(mcp::LOCK_FILE));
    let mut lock = Lock::load(&lock_path).map_err(anyhow::Error::msg)?;
    if profile.mcp_servers.is_empty() {
        bail!("the profile names no MCP server ([[mcp_server]])");
    }
    let pinning = match action {
        None => None,
        Some("pin") => Some(server.context("pin needs the server's id")?),
        Some(other) => bail!("`{other}`: the action is pin, or none to list"),
    };
    let home = profile.privacy.mode == nervros_core::providers::router::PrivacyMode::Home;
    for config in &profile.mcp_servers {
        if pinning.is_some_and(|id| id != config.id) {
            continue;
        }
        // As a session does: a server that reaches the internet stays off in the home mode.
        if config.open_world && home {
            println!(
                "{}: reaches the internet, so it is off in the home privacy mode",
                config.id
            );
            continue;
        }
        let connected = McpServer::connect(config, |p| profile.resolve(p), &lock).await;
        let server = match connected {
            Ok(s) => s,
            Err(e) => {
                println!("{}: {e}", config.id);
                continue;
            }
        };
        println!("{}:", config.id);
        for listed in &server.listed {
            let mark = match listed.pin {
                Pin::Approved => "approved",
                Pin::Changed => "CHANGED since approved: hidden",
                Pin::New => "not approved: hidden",
            };
            println!("  {} ({mark})", listed.name);
            let chosen = tools.is_empty() || tools.contains(&listed.name);
            if pinning.is_some() && chosen && listed.pin != Pin::Approved {
                // What changed is read beside what was approved, every hidden mark spelt out.
                if listed.pin == Pin::Changed
                    && let Some(old) = lock
                        .0
                        .get(&config.id)
                        .and_then(|tools| tools.get(&listed.name))
                {
                    println!(
                        "  approved before:\n{}",
                        visible(&serde_json::to_string_pretty(&old.definition)?)
                    );
                    println!("  now:");
                }
                println!(
                    "{}",
                    visible(&serde_json::to_string_pretty(&mcp::canonical(
                        &listed.definition
                    ))?)
                );
                if !confirmed(&format!("approve {} as above?", listed.name))? {
                    println!("  {} left unapproved", listed.name);
                    continue;
                }
                lock.pin(&config.id, &listed.definition);
                println!("  approved {} as above", listed.name);
            }
        }
        for name in &server.missing {
            println!("  {name} (the server has no such tool)");
        }
    }
    if pinning.is_some() {
        lock.save(&lock_path).map_err(anyhow::Error::msg)?;
        println!("written to {}", lock_path.display());
    }
    Ok(())
}
