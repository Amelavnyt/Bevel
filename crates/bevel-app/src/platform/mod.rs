// SPDX-License-Identifier: GPL-3.0-or-later

//! Platform layer: everything that differs between the desktop and the web.
//!
//! This is the only place in Bevel with `cfg(target_arch = "wasm32")` code.
//! File dialogs, storage, task spawning, clipboard and time will live here
//! too as they arrive.

#[cfg(not(target_arch = "wasm32"))]
mod native;
#[cfg(target_arch = "wasm32")]
mod web;

#[cfg(not(target_arch = "wasm32"))]
pub use native::{run, save_file};
#[cfg(target_arch = "wasm32")]
pub use web::{run, save_file};

/// What happened to a file the user asked to save.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SaveOutcome {
    /// Written to the given place (a path on the desktop, the file name of a
    /// browser download).
    Saved(String),
    /// The user closed the dialog without choosing a file.
    #[cfg_attr(
        target_arch = "wasm32",
        expect(dead_code, reason = "browser downloads have no dialog to cancel")
    )]
    Cancelled,
}

use tracing_subscriber::filter::{LevelFilter, Targets};

/// Log filter shared by both platforms: our own crates at debug level,
/// dependencies only when they warn.
fn log_filter() -> Targets {
    Targets::new()
        .with_default(LevelFilter::WARN)
        .with_target("bevel", LevelFilter::DEBUG)
        .with_target("eframe", LevelFilter::INFO)
}
