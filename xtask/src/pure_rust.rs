// SPDX-License-Identifier: GPL-3.0-or-later

//! The pure-Rust guard: nothing written in C or C++ may be compiled into
//! Bevel, so the crates that drive C toolchains must not appear in the
//! dependency graph of any target we ship.
//!
//! Exceptions live in `xtask/pure-rust-allowlist.txt`, one
//! `<dependent> <banned crate>` pair per line with the reason it is safe.
//! Every entry needs the maintainers' approval.

use anyhow::{Context as _, bail};
use std::collections::BTreeSet;
use std::path::Path;

/// Crates whose presence means C/C++ code may be built or linked.
const BANNED: [&str; 4] = ["cc", "cmake", "bindgen", "pkg-config"];

/// Every target Bevel ships for.
const TARGETS: [&str; 6] = [
    "x86_64-unknown-linux-gnu",
    "aarch64-unknown-linux-gnu",
    "x86_64-pc-windows-msvc",
    "x86_64-apple-darwin",
    "aarch64-apple-darwin",
    "wasm32-unknown-unknown",
];

const ALLOWLIST: &str = "xtask/pure-rust-allowlist.txt";

pub fn check(root: &Path) -> anyhow::Result<()> {
    let allowlist = read_allowlist(&root.join(ALLOWLIST))?;
    let mut violations = BTreeSet::new();
    let mut used = BTreeSet::new();

    for target in TARGETS {
        for banned in BANNED {
            for dependent in dependents(root, target, banned)? {
                let edge = (dependent, banned.to_owned());
                if allowlist.contains(&edge) {
                    used.insert(edge);
                } else {
                    violations.insert(format!("{} -> {banned} (target {target})", edge.0));
                }
            }
        }
    }

    for stale in allowlist.difference(&used) {
        eprintln!(
            "warning: {ALLOWLIST}: `{} {}` is no longer needed",
            stale.0, stale.1
        );
    }
    if !violations.is_empty() {
        let list: Vec<String> = violations.into_iter().collect();
        bail!(
            "C build tooling found in the dependency graph:
  {}
             Remove the dependency, or if its build script provably never builds              or links C code, propose an entry in {ALLOWLIST}.",
            list.join("
  ")
        );
    }
    eprintln!("pure-Rust guard: ok ({} allow-listed edges)", used.len());
    Ok(())
}

/// Names of the crates that directly depend on `banned` when building the
/// workspace for `target`.
///
/// This uses `cargo tree` rather than `cargo metadata` because only the
/// former applies cargo's per-target feature resolution; `cargo metadata`
/// unions features across all platforms and reports optional dependencies
/// that are never enabled for the targets we ship.
fn dependents(root: &Path, target: &str, banned: &str) -> anyhow::Result<BTreeSet<String>> {
    let output = crate::cargo()
        .args([
            "tree",
            "--workspace",
            "--locked",
            "--edges",
            "normal,build,dev",
        ])
        .args(["--depth", "1"])
        .args(["--prefix", "none", "--target", target, "--invert", banned])
        .current_dir(root)
        .output()
        .context("failed to run cargo tree")?;
    let stderr = String::from_utf8_lossy(&output.stderr);
    if !output.status.success() {
        if stderr.contains("did not match any packages") {
            return Ok(BTreeSet::new());
        }
        bail!(
            "cargo tree failed:
{stderr}"
        );
    }
    let stdout = String::from_utf8_lossy(&output.stdout);
    // The first line is the banned crate itself; the rest are its dependents.
    Ok(stdout
        .lines()
        .skip(1)
        .filter_map(|line| line.split_whitespace().next())
        .filter(|name| !name.is_empty() && *name != banned)
        .map(str::to_owned)
        .collect())
}

fn read_allowlist(path: &Path) -> anyhow::Result<BTreeSet<(String, String)>> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    parse_allowlist(&text)
}

fn parse_allowlist(text: &str) -> anyhow::Result<BTreeSet<(String, String)>> {
    let mut entries = BTreeSet::new();
    for (number, line) in text.lines().enumerate() {
        let line = line.split('#').next().unwrap_or_default().trim();
        if line.is_empty() {
            continue;
        }
        let words: Vec<&str> = line.split_whitespace().collect();
        let [dependent, banned] = words.as_slice() else {
            bail!(
                "{ALLOWLIST}:{}: expected `<dependent> <banned crate>`",
                number + 1
            );
        };
        if !BANNED.contains(banned) {
            bail!(
                "{ALLOWLIST}:{}: `{banned}` is not a banned crate",
                number + 1
            );
        }
        entries.insert(((*dependent).to_owned(), (*banned).to_owned()));
    }
    Ok(entries)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn allowlist_ignores_comments_and_blank_lines() {
        let entries = parse_allowlist("# header\n\nwayland-sys pkg-config  # dlopen\n").unwrap();
        assert!(entries.contains(&("wayland-sys".to_owned(), "pkg-config".to_owned())));
        assert_eq!(entries.len(), 1);
    }

    #[test]
    fn allowlist_rejects_unknown_banned_crate() {
        assert!(parse_allowlist("foo serde\n").is_err());
        assert!(parse_allowlist("just-one-word\n").is_err());
    }
}
