// CoverMapObsPlan Rust Library
// This file provides the library interface

pub mod ffi;
pub mod geometry;
pub mod planner;
pub mod types;

// Re-export commonly used types
pub use geometry::*;
pub use planner::*;
pub use types::*;

// Library initialization and termination
pub fn initialize() {
    ffi::initialize();
}

pub fn terminate() {
    ffi::terminate();
}
