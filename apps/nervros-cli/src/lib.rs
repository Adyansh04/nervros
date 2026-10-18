//! Headless NervROS: chat, doctor, evals and replay.

use std::path::{Path, PathBuf};
use std::time::SystemTime;

use anyhow::{Context as _, Result, bail};
use clap::builder::PossibleValuesParser;
use clap::{Parser, Subcommand};
use nervros_core::llm::{Ask, ImageFormat, ImageInput, Llm, Shot};
use nervros_core::profile::Profile;
use nervros_core::providers::router::Router;
use nervros_core::providers::{Role, free_only, openrouter};

#[cfg(feature = "ros")]
mod eval;
mod mcp;
#[cfg(feature = "ros")]
mod record;
mod records;
#[cfg(feature = "ros")]
mod robot;

/// The command line.
#[derive(Parser)]
#[command(version, about = "Headless NervROS")]
pub struct Cli {
    /// The robot profile, which also names the models file.
    #[arg(long, global = true, default_value = "profiles/example/nervros.toml")]
    profile: PathBuf,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List roles and models, with today's request counts.
    Models {
        /// Also check the free-only rule against live price lists and read free quotas. This runs
        /// no model.
        #[arg(long)]
        check: bool,
    },
    /// Send one prompt, optionally with images, and print the reply.
    Ask {
        /// The prompt.
        prompt: String,
        /// Which role's chain to use.
        #[arg(long, default_value = "routine", value_parser = PossibleValuesParser::new(Role::ALL.map(Role::name)))]
        role: String,
        /// A JPEG or PNG to attach; repeated, several in order, each labelled with its file name.
        #[arg(long)]
        image: Vec<PathBuf>,
    },
    /// Chat with the robot. Lines starting with `/` are commands: `/arm`, `/disarm`, `/stop`,
    /// `/pause`, `/resume`, `/yes N`, `/allow N` (yes, and no more asking for that tool this
    /// session unless it moves the robot), `/no N`, `/again N` (a request the last session left
    /// waiting), `/compact`, `/quit`; a plain `stop` (or `halt`, `freeze`, `stop!`) also stops the
    /// robot, and a plain `pause` or `resume` pauses or resumes a mission that can be.
    #[cfg(feature = "ros")]
    Chat {
        /// Send these messages in order and exit, instead of reading stdin.
        #[arg(long)]
        say: Vec<String>,
        /// Arm at start.
        #[arg(long)]
        arm: bool,
        /// Approve every request; for scripted runs only.
        #[arg(long)]
        approve: bool,
        /// Carry on a saved conversation: a `.history.json` path, or `last`.
        #[arg(long)]
        resume: Option<String>,
        /// Talk on this `models.toml` model alone, with no fallback, as an eval's `--models` does.
        #[arg(long)]
        model: Option<String>,
    },
    /// Run a suite of requests against the live robot, or with `--robot` a scripted one, each in
    /// a fresh session approving every request, and report which cases did what they expect.
    #[cfg(feature = "ros")]
    Eval {
        /// The suite, a TOML file of `[[case]]`s.
        suite: PathBuf,
        /// Only the cases whose ids hold this.
        #[arg(long)]
        only: Option<String>,
        /// Trials per case and model; the report gives pass^k for k = this.
        #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u16).range(1..))]
        repeat: u16,
        /// Run the routine role on each of these `models.toml` ids in turn, instead of the
        /// profile's chain, and compare them.
        #[arg(long, value_delimiter = ',')]
        models: Vec<String>,
        /// Where the report goes (default: the state directory's `evals/`).
        #[arg(long)]
        out: Option<PathBuf>,
        /// Run against the robot these files describe, merged in order, instead of the live
        /// graph: models compared in seconds, without a simulator (`record` writes one).
        #[arg(long)]
        robot: Vec<PathBuf>,
    },
    /// Print a session log as it happened: a `.ndjson` path, or `last`.
    Replay {
        /// The log.
        #[arg(default_value = "last")]
        log: String,
    },
    /// The missions the robot ran, newest first, from its ledger; with an id, that one in full.
    Missions {
        /// A mission id, or its start.
        id: Option<String>,
        /// How many.
        #[arg(long, default_value_t = 10)]
        limit: usize,
    },
    /// The requests no skill could do, newest first.
    Gaps {
        /// How many.
        #[arg(long, default_value_t = 20)]
        limit: usize,
    },
    /// A session as an eval case, with what happened as its expectations, to trim and keep.
    Case {
        /// The session log: a `.ndjson` path, or `last`.
        #[arg(default_value = "last")]
        log: String,
        /// The case's id.
        #[arg(long, default_value = "from-session")]
        id: String,
        /// Append it to this suite instead of printing it.
        #[arg(long)]
        append: Option<PathBuf>,
    },
    /// The profile's MCP servers: each tool it names, and whether its definition is approved.
    /// With `pin`, approve a server's tools as they are now, after reading them.
    Mcp {
        /// `pin` to approve.
        action: Option<String>,
        /// For pin: the server's id.
        server: Option<String>,
        /// For pin: its tools to approve; all it names when none.
        tools: Vec<String>,
    },
    /// The profile's skills, as the agent's index lists them, and any that could not be read.
    Skills,
    /// Check the profile's tools, topics and mission services against the live graph.
    #[cfg(feature = "ros")]
    Doctor,
    /// Investigate what is wrong with the robot now, with the profile's playbooks and those
    /// NervROS carries, and keep it as an incident.
    #[cfg(feature = "ros")]
    Why,
    /// What the robot did over the last hours: missions by how they ended, skills' records,
    /// incidents and goals, from its files. The world model's changes need the agent's `digest`.
    Digest {
        /// How far back.
        #[arg(long, default_value_t = 24.0)]
        hours: f64,
    },
    /// The fixes proposed after a step failed again: each with its step's record before and
    /// since, once accepted. Accepting runs a tool, so it is the app's or the chat's to do.
    Inbox {
        /// Turn down this proposal, by its number.
        #[arg(long)]
        dismiss: Option<u32>,
    },
    /// The incidents kept for the robot, newest first, or one in full by its id.
    Incidents {
        /// An incident's id, such as i3.
        id: Option<String>,
        /// How many to list.
        #[arg(long, default_value_t = 20)]
        limit: usize,
    },
    /// Write the live robot as a scripted robot file for `eval --robot`: its graph, nodes,
    /// parameters, the profile's topics and the executor's catalog.
    #[cfg(feature = "ros")]
    Record {
        /// The file to write.
        out: PathBuf,
    },
    /// Draft the robot's self-description from the live robot (its skills, sensors, body, rooms
    /// and places) in the persona's style, into `<persona>.draft.md` beside it, for you to edit.
    #[cfg(feature = "ros")]
    Whoami {
        /// Print the gathered facts as JSON and draft nothing.
        #[arg(long)]
        facts: bool,
    },
    /// Look through the robot's camera: print the marks and save the marked image.
    #[cfg(feature = "ros")]
    Look {
        /// Which camera, by the profile's names; the `[look]` one by default.
        #[arg(long)]
        camera: Option<String>,
        /// Where to write the marked JPEG.
        #[arg(long, default_value = "look.jpg")]
        out: PathBuf,
    },
    /// Segment what a prompt names in a camera's newest frame: print the regions and save the
    /// drawn image. The model backend asks the `segment` role's models, as the app does.
    #[cfg(feature = "ros")]
    Segment {
        /// What to segment, such as "the floor".
        prompt: String,
        /// Which camera, by the profile's names; the `[look]` one by default.
        #[arg(long)]
        camera: Option<String>,
        /// `model` or `service`; the profile's `[segment] backend` by default.
        #[arg(long)]
        backend: Option<String>,
        /// `cutout` or `overlay`.
        #[arg(long, default_value = "cutout")]
        view: String,
        /// Where to write the JPEG.
        #[arg(long, default_value = "segment.jpg")]
        out: PathBuf,
    },
    /// Run one read-only ROS tool against the live graph, without a model: `ros_graph`,
    /// `topic_sample`, `interface_show`, `tf`, `params`, `log_tail`, or `service_call` on a
    /// service that only reads.
    #[cfg(feature = "ros")]
    Ros {
        /// The tool.
        tool: String,
        /// Its arguments as JSON, such as '{"topic": "/odom", "mode": "hz"}'.
        #[arg(default_value = "{}")]
        args: String,
    },
}

/// The profile's models, with its privacy mode: a frame from a home robot stays on local models
/// here as it does in a session.
fn router(profile: &Path) -> Result<Router> {
    let profile = Profile::load(profile).context("loading the profile")?;
    nervros_core::app::router(&profile).context("loading the models")
}

/// Runs one command.
///
/// # Errors
///
/// The command failed, or found failures it reports, such as failed eval trials.
pub async fn run(cli: Cli) -> Result<()> {
    match cli.command {
        Command::Models { check } => models(&cli.profile, check).await,
        Command::Ask {
            prompt,
            role,
            image,
        } => {
            let role = role.parse().map_err(anyhow::Error::msg)?;
            ask(&cli.profile, &prompt, role, image).await
        }
        #[cfg(feature = "ros")]
        Command::Look { camera, out } => robot::look(&cli.profile, camera, &out).await,
        #[cfg(feature = "ros")]
        Command::Segment {
            prompt,
            camera,
            backend,
            view,
            out,
        } => {
            let args = serde_json::json!({"prompt": prompt, "camera": camera, "backend": backend,
                "view": view});
            robot::segment(&cli.profile, args, &out).await
        }
        #[cfg(feature = "ros")]
        Command::Chat {
            say,
            arm,
            approve,
            resume,
            model,
        } => {
            robot::chat(
                &cli.profile,
                &nervros_core::app::state_dir(),
                robot::ChatOptions {
                    say,
                    arm,
                    approve,
                    resume,
                    model,
                },
            )
            .await
        }
        #[cfg(feature = "ros")]
        Command::Eval {
            suite,
            only,
            repeat,
            models,
            out,
            robot,
        } => {
            let out = out.unwrap_or_else(eval::default_out);
            let options = eval::Options {
                only,
                repeat: usize::from(repeat),
                models,
                robot,
            };
            let failed = eval::run(&cli.profile, &suite, &options, &out).await?;
            if failed > 0 {
                anyhow::bail!("{failed} trial(s) failed");
            }
            Ok(())
        }
        Command::Replay { log } => records::replay(&log),
        Command::Missions { id, limit } => records::missions(&cli.profile, id.as_deref(), limit),
        Command::Digest { hours } => records::digest(&cli.profile, hours),
        Command::Inbox { dismiss } => records::inbox(&cli.profile, dismiss),
        Command::Gaps { limit } => records::gaps(&cli.profile, limit),
        Command::Case { log, id, append } => records::case(&log, &id, append.as_deref()),
        Command::Mcp {
            action,
            server,
            tools,
        } => mcp::run(&cli.profile, action.as_deref(), server.as_deref(), &tools).await,
        Command::Skills => skills(&cli.profile),
        #[cfg(feature = "ros")]
        Command::Doctor => robot::doctor(&cli.profile).await,
        #[cfg(feature = "ros")]
        Command::Why => robot::why(&cli.profile).await,
        Command::Incidents { id, limit } => records::incidents(&cli.profile, id.as_deref(), limit),
        #[cfg(feature = "ros")]
        Command::Record { out } => record::record(&cli.profile, &out).await,
        #[cfg(feature = "ros")]
        Command::Whoami { facts } => robot::whoami(&cli.profile, facts).await,
        #[cfg(feature = "ros")]
        Command::Ros { tool, args } => robot::ros(&cli.profile, &tool, &args).await,
    }
}

fn skills(profile: &Path) -> Result<()> {
    let profile = nervros_core::profile::Profile::load(profile).context("loading the profile")?;
    let (skills, problems) = nervros_core::skills::load_for(&profile);
    match nervros_core::skills::index(&skills) {
        Some(index) => println!("{index}"),
        None => println!("no skills"),
    }
    for problem in problems {
        println!("left out: {problem}");
    }
    Ok(())
}

async fn models(profile: &Path, check: bool) -> Result<()> {
    let router = router(profile)?;
    let now = SystemTime::now();
    let config = router.config();
    for role in Role::ALL {
        let chain = config.roles.chain(role);
        let models = if chain.is_empty() {
            "none".to_owned()
        } else {
            chain.join(" > ")
        };
        println!("{}: {models}", role.name());
    }
    for model in &config.models {
        println!(
            "  {} = {} on {} (used today: {})",
            model.id,
            model.model,
            model.provider,
            router.used_today(&model.id, now)
        );
    }
    if !check {
        return Ok(());
    }
    let llm = Llm::new(router);
    let missing = llm.missing_keys();
    for why in &missing {
        println!("{why}");
    }
    let http = reqwest::Client::new();
    for provider in &llm.router().config().providers {
        let Some(base) = provider
            .base_url
            .as_deref()
            .filter(|_| provider.is_openrouter())
        else {
            continue;
        };
        let models: Vec<_> = llm
            .router()
            .config()
            .models
            .iter()
            .filter(|m| m.provider == provider.id)
            .collect();
        let ids: Vec<&str> = models.iter().map(|m| m.model.as_str()).collect();
        let list = openrouter::fetch_models(&http, base)
            .await
            .context("fetching the price list")?;
        free_only::check_prices(&list, &ids).context("free-only check")?;
        println!(
            "{}: free-only check passed for {} models",
            provider.id,
            ids.len()
        );
        let Some(source) = &provider.key else {
            continue;
        };
        let key = source.load().context("loading the key")?;
        let daily = openrouter::fetch_free_daily(&http, base, &key)
            .await
            .context("reading free quota")?;
        println!(
            "{}: free requests today {} used, {} left of {}",
            provider.id, daily.used, daily.remaining, daily.limit
        );
        // OpenRouter's own count replaces ours for the pools its models share.
        let pools: std::collections::BTreeSet<&str> = models
            .iter()
            .filter_map(|m| m.limits.pool.as_deref())
            .collect();
        for pool in pools {
            llm.router()
                .set_pool_used(pool, daily.used, SystemTime::now())
                .context("saving the quota ledger")?;
            println!(
                "  pool {pool}: {} used today, as OpenRouter counts",
                daily.used
            );
        }
    }
    if !missing.is_empty() {
        bail!("{} provider key(s) could not be read", missing.len());
    }
    Ok(())
}

async fn ask(profile: &Path, prompt: &str, role: Role, images: Vec<PathBuf>) -> Result<()> {
    let one = images.len() == 1;
    let images = images
        .iter()
        .map(|file| {
            let format = match file
                .extension()
                .and_then(|e| e.to_str())
                .map(str::to_ascii_lowercase)
                .as_deref()
            {
                Some("png") => ImageFormat::Png,
                Some("jpg" | "jpeg") => ImageFormat::Jpeg,
                _ => bail!("{} is not a .png or .jpg file", file.display()),
            };
            let bytes =
                std::fs::read(file).with_context(|| format!("reading {}", file.display()))?;
            let image = ImageInput { bytes, format };
            // A lone image needs no name; several are told apart by theirs.
            Ok(if one {
                Shot::bare(image)
            } else {
                let name = file.file_name().unwrap_or_default().to_string_lossy();
                Shot {
                    label: format!("{name}:"),
                    image,
                }
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let llm = Llm::new(router(profile)?);
    let answer = llm
        .ask(Ask {
            role,
            preamble: "You are NervROS, a robot's assistant. Be brief.",
            prompt,
            images,
        })
        .await?;
    println!("{}", answer.text);
    eprintln!(
        "[{} | {} in, {} out]",
        answer.model, answer.input_tokens, answer.output_tokens
    );
    Ok(())
}
