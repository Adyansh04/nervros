//! The NervROS desktop app: chat beside an embedded Rerun viewer, in one process with the agent.

mod app;
mod attach;
mod chat;
mod editor;
mod history;
mod palette;
mod plan_edit;
mod robot;
mod sessions;
mod toasts;

use std::path::PathBuf;

use anyhow::{Context as _, Result, anyhow};
use clap::Parser;
use nervros_core::profile::Profile;
use rerun::external::{eframe, re_crash_handler, re_memory, re_viewer};

// Lets the viewer see its own memory use and prune its store at the limit.
#[global_allocator]
static GLOBAL: re_memory::AccountingAllocator<mimalloc::MiMalloc> =
    re_memory::AccountingAllocator::new(mimalloc::MiMalloc);

#[derive(Parser)]
#[command(version, about = "The NervROS desktop app")]
struct Args {
    /// The robot profile.
    #[arg(long, default_value = "profiles/example/nervros.toml")]
    profile: PathBuf,
    /// The viewer drops its oldest data past this, such as `4GB`.
    #[arg(long, default_value = "4GB")]
    memory_limit: String,
}

fn main() -> Result<()> {
    let _telemetry = nervros_core::telemetry::init();
    let main_thread = re_viewer::MainThreadToken::i_promise_i_am_on_the_main_thread();
    let args = Args::parse();
    let memory_limit = re_memory::MemoryLimit::parse(&args.memory_limit).map_err(|e| anyhow!(e))?;
    re_crash_handler::install_crash_handlers(re_viewer::build_info());

    let runtime = tokio::runtime::Runtime::new().context("starting tokio")?;
    let _entered = runtime.enter();
    let profile = Profile::load(&args.profile).context("loading the profile")?;
    let robot = nervros_core::app::connect(&profile)?;
    let state = nervros_core::app::state_dir();
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let files = nervros_core::app::StartOptions {
        history: Some(
            state
                .join("logs")
                .join(format!("session-{stamp}.history.json")),
        ),
        ..Default::default()
    };
    let agent =
        nervros_core::app::start_with(&args.profile, robot, &state.join("quota.json"), files)
            .context("starting the agent")?;
    // At once: start-up notices, such as a heartbeat that could not start, come before the window.
    let events = agent.session.subscribe();
    let (log_path, _log) = nervros_core::log::spawn(
        &state.join("logs"),
        &format!("session-{stamp}"),
        agent.session.subscribe(),
    )
    .context("opening the session log")?;
    let (rec, viewer_input) = nervros_viz::in_process().context("creating the recording")?;
    let bridge = nervros_viz::spawn(
        &rec,
        &agent.robot,
        &agent.profile,
        agent.session.subscribe(),
    );

    let mut options = re_viewer::native::eframe_options(None);
    options.viewport = options
        .viewport
        .with_app_id("nervros")
        .with_title("NervROS")
        .with_inner_size([1600.0, 960.0])
        // Rerun asks for a transparent window for its own decorations, which we do not use.
        .with_transparent(false);
    let handle = runtime.handle().clone();
    eframe::run_native(
        "NervROS",
        options,
        Box::new(move |cc| {
            let feed = app::ViewerFeed {
                input: viewer_input,
                bridge,
                memory_limit,
            };
            let gui = app::Gui::start(main_thread, cc, agent, events, feed, handle, log_path)?;
            Ok(Box::new(gui))
        }),
    )
    .map_err(|e| anyhow!("the window failed: {e}"))?;
    drop(rec);
    Ok(())
}
