// SPDX-License-Identifier: GPL-3.0-or-later

//! Repository automation for Bevel, run as `cargo xtask <command>`.

mod commits;
mod layers;
mod metadata;
mod pure_rust;

use anyhow::{Context as _, bail};
use std::path::{Path, PathBuf};
use std::process::Command;

const USAGE: &str = "\
Usage: cargo xtask <command>

Commands:
  check-pure-rust         Fail if C build tools (cc, cmake, bindgen, pkg-config)
                          appear in the dependency tree of any shipped target
  check-layers            Fail if a crate depends on a crate in its own or a higher layer
  check-commits [RANGE]   Check commit messages in RANGE (default origin/master..HEAD)
  check-commit-msg FILE   Check one commit message file (used by .githooks/commit-msg)
  wasm                    Build the web app for wasm32-unknown-unknown
  ci                      Run the local pre-commit checks: fmt, clippy, tests,
                          wasm build, pure-Rust guard and layer check";

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let args: Vec<&str> = args.iter().map(String::as_str).collect();
    match args.as_slice() {
        ["check-pure-rust"] => pure_rust::check(&workspace_root()?),
        ["check-layers"] => layers::check(&workspace_root()?),
        ["check-commits"] => commits::check_range("origin/master..HEAD"),
        ["check-commits", range] => commits::check_range(range),
        ["check-commit-msg", file] => commits::check_file(Path::new(file)),
        ["wasm"] => wasm(),
        ["ci"] => ci(),
        [] | ["help" | "-h" | "--help"] => {
            println!("{USAGE}");
            Ok(())
        }
        _ => bail!("unknown arguments: {}\n\n{USAGE}", args.join(" ")),
    }
}

/// The workspace root: the parent of this crate's directory.
fn workspace_root() -> anyhow::Result<PathBuf> {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .map(Path::to_path_buf)
        .context("xtask has no parent directory")
}

fn cargo() -> Command {
    Command::new(std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into()))
}

fn run(mut command: Command) -> anyhow::Result<()> {
    eprintln!("$ {command:?}");
    let status = command
        .status()
        .with_context(|| format!("failed to start {command:?}"))?;
    if !status.success() {
        bail!("{command:?} failed with {status}");
    }
    Ok(())
}

fn wasm() -> anyhow::Result<()> {
    let mut command = cargo();
    command.args([
        "build",
        "-p",
        "bevel-app",
        "--target",
        "wasm32-unknown-unknown",
    ]);
    run(command)
}

fn ci() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let steps: [&[&str]; 4] = [
        &["fmt", "--all", "--check"],
        &[
            "clippy",
            "--workspace",
            "--all-targets",
            "--",
            "-D",
            "warnings",
        ],
        &[
            "clippy",
            "-p",
            "bevel-app",
            "--target",
            "wasm32-unknown-unknown",
            "--",
            "-D",
            "warnings",
        ],
        &["test", "--workspace"],
    ];
    for step in steps {
        let mut command = cargo();
        command.args(step).current_dir(&root);
        run(command)?;
    }
    wasm()?;
    pure_rust::check(&root)?;
    layers::check(&root)?;
    eprintln!("all checks passed");
    Ok(())
}
