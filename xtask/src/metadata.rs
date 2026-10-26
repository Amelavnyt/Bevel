// SPDX-License-Identifier: GPL-3.0-or-later

//! Minimal reader for `cargo metadata` output.

use anyhow::{Context as _, bail};
use serde_json::Value;
use std::path::Path;

/// Runs `cargo metadata` in `root` with `extra` arguments and parses the JSON.
pub fn cargo_metadata(root: &Path, extra: &[&str]) -> anyhow::Result<Value> {
    let output = crate::cargo()
        .args(["metadata", "--format-version", "1", "--locked"])
        .args(extra)
        .current_dir(root)
        .output()
        .context("failed to run cargo metadata")?;
    if !output.status.success() {
        bail!(
            "cargo metadata failed:\n{}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
    serde_json::from_slice(&output.stdout).context("cargo metadata printed invalid JSON")
}

pub fn string(value: &Value) -> anyhow::Result<String> {
    value
        .as_str()
        .map(str::to_owned)
        .with_context(|| format!("expected a string in cargo metadata, found {value}"))
}
