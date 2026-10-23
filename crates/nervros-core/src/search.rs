//! Searching for something: where to look, best first, from what the world model saw and where
//! such things usually are, and a job that walks to each place and looks until the detector sees
//! it. And patrols, a round of places and what changed meanwhile.

mod patrol;
mod priors;
mod rank;
mod tool;

pub use patrol::PatrolTool;
pub use rank::Lookout;
pub(crate) use rank::holds;
pub use tool::SearchTool;
