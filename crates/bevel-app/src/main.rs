// SPDX-License-Identifier: GPL-3.0-or-later

//! The Bevel application: one eframe app for the desktop and the browser.
//!
//! Everything that differs between the two targets lives in [`platform`].

mod app;
mod platform;
mod scene;
mod viewport;

fn main() {
    platform::run();
}
