// SPDX-License-Identifier: GPL-3.0-or-later

//! `bevel-cli`: headless Bevel for scripts, CI and power users.
//!
//! Planned commands: `rebuild`, `export`, `import`, `validate`, `run`,
//! `info`, `pack`, `unpack` and `kernel-repro`. Each arrives with the
//! phase that provides what it needs; see the roadmap.

use std::process::ExitCode;

const USAGE: &str = "\
Usage: bevel-cli <command> [options]

Commands:
  help       Show this message
  version    Print the version

Document commands (rebuild, export, import, validate, run, info, pack,
unpack, kernel-repro) are not available yet.";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        None | Some("help" | "-h" | "--help") => {
            println!("{USAGE}");
            ExitCode::SUCCESS
        }
        Some("version" | "-V" | "--version") => {
            println!("bevel-cli {}", env!("CARGO_PKG_VERSION"));
            ExitCode::SUCCESS
        }
        Some(other) => {
            eprintln!("bevel-cli: unknown command `{other}`\n\n{USAGE}");
            ExitCode::from(2)
        }
    }
}
