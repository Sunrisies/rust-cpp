// CoverMapObsPlan Rust Library
// This file provides the library interface

pub mod ffi;
pub mod types;
pub mod geometry;
pub mod planner;

// Re-export commonly used types
pub use types::*;
pub use geometry::*;
pub use planner::*;

// Library initialization and termination
pub fn initialize() {
    ffi::initialize();
}

pub fn terminate() {
    ffi::terminate();
}
