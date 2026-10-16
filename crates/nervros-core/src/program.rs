//! Robot programs: short scripts the model writes for what takes loops, counting or measuring,
//! approved once with the skills they may run and their time limit, then run as a job. A program
//! moves the robot only through missions (`Missions::run_for_job`), so the plan checks, the
//! keep-out zones and the executor's own validation all still hold, and a stop ends it.

mod api;
mod engine;
mod host;
mod skills;
mod store;
mod tool;

pub use store::{Saved, Store};
pub use tool::ProgramTool;
