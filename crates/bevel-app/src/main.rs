// SPDX-License-Identifier: GPL-3.0-or-later

//! The Bevel application: one eframe app for the desktop and the browser.
//!
//! Everything that differs between the two targets lives in [`platform`].

mod app;
mod document;
mod geometry_debug;
mod part;
mod platform;
mod rebuild;
mod viewport;
#[allow(dead_code, reason = "each binary uses one side of the protocol")]
mod wire;

fn main() {
    platform::run();
}
