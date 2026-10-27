// SPDX-License-Identifier: GPL-3.0-or-later

//! The rebuild worker binary: in the browser it runs in a Web Worker next
//! to the app; see `platform/worker.rs`.

mod rebuild;
#[allow(dead_code, reason = "each binary uses one side of the protocol")]
mod wire;

#[path = "platform/worker.rs"]
mod worker;

fn main() {
    worker::run();
}
