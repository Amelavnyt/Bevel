// SPDX-License-Identifier: GPL-3.0-or-later

//! The layering rule: a crate may depend only on crates in lower layers,
//! never sideways and never up (see docs/ARCHITECTURE.md).

use crate::metadata;
use anyhow::bail;
use std::path::Path;

/// Layer of every workspace crate. Adding a crate means adding it here.
const LAYERS: [(&str, u8); 15] = [
    ("bevel-math", 0),
    ("bevel-geom", 1),
    ("bevel-solver", 1),
    ("bevel-expr", 1),
    ("bevel-timeline", 1),
    ("bevel-render", 1),
    ("bevel-kernel", 2),
    ("bevel-sketch", 2),
    ("bevel-model", 3),
    ("bevel-io", 3),
    ("bevel-doc", 4),
    ("bevel-script", 5),
    ("bevel-ui", 6),
    ("bevel-app", 7),
    ("bevel-cli", 7),
];

fn layer(name: &str) -> Option<u8> {
    LAYERS.iter().find(|(n, _)| *n == name).map(|&(_, l)| l)
}

pub fn check(root: &Path) -> anyhow::Result<()> {
    let json = metadata::cargo_metadata(root, &["--no-deps"])?;
    let mut problems = Vec::new();
    for package in json["packages"].as_array().into_iter().flatten() {
        let name = metadata::string(&package["name"])?;
        if name == "xtask" {
            continue;
        }
        let Some(own) = layer(&name) else {
            problems.push(format!(
                "{name} has no layer; add it to xtask/src/layers.rs"
            ));
            continue;
        };
        for dep in package["dependencies"].as_array().into_iter().flatten() {
            let dep_name = metadata::string(&dep["name"])?;
            if let Some(dep_layer) = layer(&dep_name)
                && dep_layer >= own
            {
                problems.push(format!(
                    "{name} (layer {own}) depends on {dep_name} (layer {dep_layer})"
                ));
            }
        }
    }
    if !problems.is_empty() {
        bail!("layering rule violated:\n  {}", problems.join("\n  "));
    }
    eprintln!("layer check: ok");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_has_no_duplicates() {
        let mut names: Vec<&str> = LAYERS.iter().map(|(n, _)| *n).collect();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), LAYERS.len());
    }
}
