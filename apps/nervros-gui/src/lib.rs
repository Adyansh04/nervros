//! The NervROS desktop app: chat beside an embedded Rerun viewer, in one process with the agent.

mod app;
mod attach;
mod chat;
mod editor;
mod history;
mod incidents;
mod palette;
mod plan_edit;
mod programs;
mod robot;
mod sessions;
mod taught;
#[cfg(test)]
mod testkit;
mod theme;
mod toasts;

use std::path::PathBuf;

use anyhow::{Context as _, Result, anyhow};
use clap::Parser;
use rerun::external::{eframe, re_crash_handler, re_memory, re_viewer};

/// The command line.
#[derive(Parser)]
#[command(version, about = "The NervROS desktop app")]
pub struct Args {
    /// The robot profile.
    #[arg(long, default_value = "profiles/example/nervros.toml")]
    profile: PathBuf,
    /// The viewer drops its oldest data past this, such as `4GB`.
    #[arg(long, default_value = "4GB")]
    memory_limit: String,
}

/// Runs `reset` on the shared state when dropped, so a background task that panics or is
/// cancelled still clears the busy flag it set, and wakes the window to show it.
pub(crate) struct Reset<T, F: FnMut(&mut T)> {
    shared: std::sync::Arc<std::sync::Mutex<T>>,
    then: F,
    wake: rerun::external::egui::Context,
}

impl<T, F: FnMut(&mut T)> Reset<T, F> {
    pub(crate) fn new(
        shared: std::sync::Arc<std::sync::Mutex<T>>,
        reset: F,
        wake: rerun::external::egui::Context,
    ) -> Self {
        Self {
            shared,
            then: reset,
            wake,
        }
    }
}

impl<T, F: FnMut(&mut T)> Drop for Reset<T, F> {
    fn drop(&mut self) {
        (self.then)(&mut nervros_ros::lock(&self.shared));
        self.wake.request_repaint();
    }
}

/// Opens the window and runs the agent until it closes. Must run on the main thread.
///
/// # Errors
///
/// The agent, the recording or the window could not start.
pub fn run(args: &Args) -> Result<()> {
    let main_thread = re_viewer::MainThreadToken::i_promise_i_am_on_the_main_thread();
    let memory_limit = re_memory::MemoryLimit::parse(&args.memory_limit).map_err(|e| anyhow!(e))?;
    re_crash_handler::install_crash_handlers(re_viewer::build_info());

    let runtime = tokio::runtime::Runtime::new().context("starting tokio")?;
    let _entered = runtime.enter();
    // The robot's lock is held as long as this window runs.
    let nervros_core::app::LoggedSession {
        agent,
        events,
        log_path,
        lock: _robot,
        ..
    } = nervros_core::app::start_logged(&args.profile, &nervros_core::app::state_dir(), None, None)
        .context("starting the agent")?;
    let (rec, viewer_input) = nervros_viz::in_process().context("creating the recording")?;
    let bridge = nervros_viz::spawn(
        &rec,
        &agent.robot,
        &agent.profile,
        &agent.zones,
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
    // Kept past the window: what NervROS itself set moving is stopped once it closes, before the
    // runtime that would carry the cancels is gone.
    let integrations = std::sync::Arc::clone(&agent.integrations);
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
    let stopped = runtime.block_on(tokio::time::timeout(
        std::time::Duration::from_secs(10),
        integrations.stop(),
    ));
    match stopped {
        Ok(moving) if moving.is_empty() => {}
        Ok(moving) => tracing::error!(?moving, "closed; these may still be moving"),
        Err(_) => {
            tracing::error!("closed; NervROS's own goals did not answer their cancel in 10 s");
        }
    }
    drop(rec);
    Ok(())
}
