//! Headless NervROS: chat, doctor, evals and replay.

use std::path::{Path, PathBuf};
use std::time::SystemTime;

use anyhow::{Context as _, Result, bail};
use clap::{Parser, Subcommand, ValueEnum};
use nervros_core::llm::{Ask, ImageFormat, ImageInput, Llm};
use nervros_core::profile::Profile;
use nervros_core::providers::router::{PrivacyMode, Router};
use nervros_core::providers::{ModelsConfig, Role, free_only, openrouter};

#[cfg(feature = "ros")]
mod eval;
mod mcp;
mod records;
#[cfg(feature = "ros")]
mod robot;

#[derive(Parser)]
#[command(version, about = "Headless NervROS")]
struct Cli {
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
    /// Send one prompt, optionally with an image, and print the reply.
    Ask {
        /// The prompt.
        prompt: String,
        /// Which role's chain to use.
        #[arg(long, value_enum, default_value_t = RoleArg::Routine)]
        role: RoleArg,
        /// A JPEG or PNG to attach.
        #[arg(long)]
        image: Option<PathBuf>,
    },
    /// Chat with the robot. Lines starting with `/` are commands: `/arm`, `/disarm`, `/stop`,
    /// `/yes N`, `/no N`, `/compact`, `/quit`; a line that is exactly `stop` also stops the robot.
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
    },
    /// Run a suite of requests against the live robot, each in a fresh session approving every
    /// request, and report which cases did what they expect.
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

#[derive(Clone, Copy, ValueEnum)]
enum RoleArg {
    Routine,
    Plan,
    VisionCheck,
    Summarise,
    Segment,
}

impl From<RoleArg> for Role {
    fn from(role: RoleArg) -> Self {
        match role {
            RoleArg::Routine => Self::Routine,
            RoleArg::Plan => Self::Plan,
            RoleArg::VisionCheck => Self::VisionCheck,
            RoleArg::Summarise => Self::Summarise,
            RoleArg::Segment => Self::Segment,
        }
    }
}

/// The models file the profile names, the one the app and `chat` use too.
fn models_file(profile: &Path) -> Result<PathBuf> {
    let profile = Profile::load(profile).context("loading the profile")?;
    Ok(profile.resolve(&profile.models.file))
}

fn router(models: &Path) -> Result<Router> {
    let config = ModelsConfig::load(models).context("loading the models file")?;
    let ledger = nervros_core::app::state_dir().join("quota.json");
    Router::with_ledger_file(config, &ledger, PrivacyMode::Sim).context("loading the quota ledger")
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_writer(std::io::stderr)
        .init();
    let cli = Cli::parse();
    match cli.command {
        Command::Models { check } => models(&models_file(&cli.profile)?, check).await,
        Command::Ask {
            prompt,
            role,
            image,
        } => ask(&models_file(&cli.profile)?, &prompt, role.into(), image).await,
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
        } => {
            robot::chat(
                &cli.profile,
                &nervros_core::app::state_dir(),
                robot::ChatOptions {
                    say,
                    arm,
                    approve,
                    resume,
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
        } => {
            let out = out.unwrap_or_else(eval::default_out);
            let options = eval::Options {
                only,
                repeat: usize::from(repeat),
                models,
            };
            let failed = eval::run(&cli.profile, &suite, &options, &out).await?;
            if failed > 0 {
                anyhow::bail!("{failed} trial(s) failed");
            }
            Ok(())
        }
        Command::Replay { log } => records::replay(&log),
        Command::Missions { id, limit } => records::missions(&cli.profile, id.as_deref(), limit),
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
        Command::Ros { tool, args } => robot::ros(&cli.profile, &tool, &args).await,
    }
}

fn skills(profile: &Path) -> Result<()> {
    let profile = nervros_core::profile::Profile::load(profile).context("loading the profile")?;
    let dirs: Vec<PathBuf> = profile.skills.iter().map(|d| profile.resolve(d)).collect();
    let (skills, problems) = nervros_core::skills::load(&dirs);
    match nervros_core::skills::index(&skills) {
        Some(index) => println!("{index}"),
        None => println!("no skills"),
    }
    for problem in problems {
        println!("left out: {problem}");
    }
    Ok(())
}

async fn models(path: &Path, check: bool) -> Result<()> {
    let router = router(path)?;
    let now = SystemTime::now();
    let config = router.config();
    for (name, role) in [
        ("routine", Role::Routine),
        ("plan", Role::Plan),
        ("vision_check", Role::VisionCheck),
        ("summarise", Role::Summarise),
        ("segment", Role::Segment),
        ("plan_check", Role::PlanCheck),
    ] {
        println!("{name}: {}", config.roles.chain(role).join(" > "));
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
    let http = reqwest::Client::new();
    for provider in &llm.router().config().providers {
        let Some(base) = provider
            .base_url
            .as_deref()
            .filter(|u| u.contains("openrouter.ai"))
        else {
            continue;
        };
        let ids: Vec<&str> = llm
            .router()
            .config()
            .models
            .iter()
            .filter(|m| m.provider == provider.id)
            .map(|m| m.model.as_str())
            .collect();
        let list = openrouter::fetch_models(&http, base)
            .await
            .context("fetching the price list")?;
        if provider.free_only {
            free_only::check_prices(&list, &ids).context("free-only check")?;
            println!(
                "{}: free-only check passed for {} models",
                provider.id,
                ids.len()
            );
        }
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
    }
    Ok(())
}

async fn ask(path: &Path, prompt: &str, role: Role, image: Option<PathBuf>) -> Result<()> {
    let image = match image {
        None => None,
        Some(file) => {
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
                std::fs::read(&file).with_context(|| format!("reading {}", file.display()))?;
            Some(ImageInput { bytes, format })
        }
    };
    let llm = Llm::new(router(path)?);
    let answer = llm
        .ask(Ask {
            role,
            preamble: "You are NervROS, a robot's assistant. Be brief.",
            prompt,
            image,
        })
        .await?;
    println!("{}", answer.text);
    eprintln!(
        "[{} | {} in, {} out]",
        answer.model, answer.input_tokens, answer.output_tokens
    );
    Ok(())
}
