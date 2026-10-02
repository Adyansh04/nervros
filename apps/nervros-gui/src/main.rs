//! The `nervros-gui` binary; the window is in the library.

use clap::Parser as _;
use rerun::external::re_memory;

// Lets the viewer see its own memory use and prune its store at the limit.
#[global_allocator]
static GLOBAL: re_memory::AccountingAllocator<mimalloc::MiMalloc> =
    re_memory::AccountingAllocator::new(mimalloc::MiMalloc);

fn main() -> anyhow::Result<()> {
    let _telemetry = nervros_core::telemetry::init();
    nervros_gui::run(&nervros_gui::Args::parse())
}
