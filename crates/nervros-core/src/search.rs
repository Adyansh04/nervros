//! Searching for something: where to look, best first, from what the world model saw and where
//! such things usually are, and a job that walks to each place and looks until the detector sees
//! it.

mod priors;
mod rank;
mod tool;

pub use rank::Lookout;
pub use tool::SearchTool;
