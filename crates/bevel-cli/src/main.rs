// SPDX-License-Identifier: GPL-3.0-or-later

//! `bevel-cli`: headless Bevel for scripts, CI and power users.
//!
//! `rebuild`, `info` and `export` work on `.bevel` documents. Still to come
//! with their phases: `import`, `validate`, `run`, `pack`, `unpack` and
//! `kernel-repro`.

use anyhow::{Context as _, Result, bail};
use bevel_doc::DocState;
use bevel_expr::format_quantity;
use bevel_kernel::{OpContext, TessellationOptions, mass_properties, tessellate};
use bevel_model::{FeatureStatus, Regenerated, Regenerator};
use bevel_timeline::{SkipReason, StepStatus};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

const USAGE: &str = "\
Usage: bevel-cli <command> [options]

Commands:
  rebuild <file.bevel>
      Regenerate the document and report each feature's status. Exits
      with status 1 if any feature fails or is skipped.
  info <file.bevel>
      List the features, parameters and bodies with their volumes.
  export <file.bevel> --format stl|3mf [--body NAME] -o <out>
      Write the visible bodies (or one body) as a mesh.
  help       Show this message
  version    Print the version";

/// sRGB colour of bodies without their own.
const DEFAULT_COLOR: [u8; 3] = [184, 190, 194];

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        None | Some("help" | "-h" | "--help") => {
            println!("{USAGE}");
            Ok(ExitCode::SUCCESS)
        }
        Some("version" | "-V" | "--version") => {
            println!("bevel-cli {}", env!("CARGO_PKG_VERSION"));
            Ok(ExitCode::SUCCESS)
        }
        Some("rebuild") => rebuild(&args[1..]),
        Some("info") => info(&args[1..]),
        Some("export") => export(&args[1..]),
        Some(other) => {
            eprintln!("bevel-cli: unknown command `{other}`\n\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    result.unwrap_or_else(|e| {
        eprintln!("bevel-cli: {e:#}");
        ExitCode::from(2)
    })
}

/// The one positional argument of a command that takes only a file.
fn single_file(args: &[String], command: &str) -> Result<PathBuf> {
    match args {
        [file] => Ok(PathBuf::from(file)),
        _ => bail!("usage: bevel-cli {command} <file.bevel>"),
    }
}

fn load(path: &Path) -> Result<(DocState, Regenerated)> {
    let bytes = std::fs::read(path).with_context(|| format!("cannot read {}", path.display()))?;
    let state =
        bevel_doc::file::read(&bytes).with_context(|| format!("cannot open {}", path.display()))?;
    let regenerated = Regenerator::new()
        .regenerate(&state.part, None)
        .context("regeneration was cancelled")?;
    Ok((state, regenerated))
}

fn describe(status: &FeatureStatus, state: &DocState) -> String {
    let name = |id| {
        state
            .feature(id)
            .map_or_else(|| format!("{id}"), |f| f.name.clone())
    };
    match status {
        StepStatus::Ok => "ok".to_owned(),
        StepStatus::Warning(w) => format!(
            "warning: {}",
            w.iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join("; ")
        ),
        StepStatus::Error(e) => format!("error: {e}"),
        StepStatus::Skipped(SkipReason::InputUnavailable(id)) => {
            format!("skipped: {} did not build", name(*id))
        }
        StepStatus::Skipped(SkipReason::InputMissing(id)) => {
            format!("skipped: {} was deleted", name(*id))
        }
        StepStatus::Suppressed => "suppressed".to_owned(),
        StepStatus::AfterMarker => "rolled back".to_owned(),
    }
}

fn rebuild(args: &[String]) -> Result<ExitCode> {
    let path = single_file(args, "rebuild")?;
    let (state, regenerated) = load(&path)?;
    let mut failed = 0;
    for (id, status) in &regenerated.statuses {
        let name = state.feature(*id).map_or("?", |f| f.name.as_str());
        println!("{name}: {}", describe(status, &state));
        if matches!(status, StepStatus::Error(_) | StepStatus::Skipped(_)) {
            failed += 1;
        }
    }
    for (name, value) in regenerated.parameters.iter() {
        if let Err(e) = value {
            println!("parameter {name}: error: {e}");
            failed += 1;
        }
    }
    println!(
        "{} features, {} bodies, {failed} problems",
        regenerated.statuses.len(),
        regenerated.state.bodies.len()
    );
    Ok(if failed == 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    })
}

fn info(args: &[String]) -> Result<ExitCode> {
    let path = single_file(args, "info")?;
    let (state, regenerated) = load(&path)?;
    let units = state.part.units;
    println!("Features:");
    for (id, status) in &regenerated.statuses {
        if let Some(f) = state.feature(*id) {
            println!(
                "  {} ({}): {}",
                f.name,
                f.def.kind_name(),
                describe(status, &state)
            );
        }
    }
    println!("Parameters:");
    for p in state.part.parameters.params() {
        let value = match regenerated.parameters.get(&p.name) {
            Some(Ok(q)) => format_quantity(*q, units, 3),
            Some(Err(e)) => format!("error: {e}"),
            None => "?".to_owned(),
        };
        println!("  {} = {} = {value}", p.name, p.expression);
    }
    println!("Bodies:");
    let cx = OpContext::default();
    for body in regenerated.state.bodies.iter() {
        let mass = mass_properties(&body.body, &cx).context("mass properties")?;
        println!(
            "  {}: volume {:.3} mm^3, area {:.3} mm^2",
            state.body_name(body.id),
            mass.volume,
            mass.area
        );
    }
    Ok(ExitCode::SUCCESS)
}

struct ExportArgs {
    file: PathBuf,
    format: String,
    body: Option<String>,
    out: PathBuf,
}

fn export_args(args: &[String]) -> Result<ExportArgs> {
    let mut file = None;
    let mut format = None;
    let mut body = None;
    let mut out = None;
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        let mut value = |flag: &str| {
            it.next()
                .cloned()
                .with_context(|| format!("{flag} needs a value"))
        };
        match arg.as_str() {
            "--format" => format = Some(value("--format")?),
            "--body" => body = Some(value("--body")?),
            "-o" | "--output" => out = Some(PathBuf::from(value("-o")?)),
            other if other.starts_with('-') => bail!("unknown option `{other}`"),
            other if file.is_none() => file = Some(PathBuf::from(other)),
            other => bail!("unexpected argument `{other}`"),
        }
    }
    let usage = "usage: bevel-cli export <file.bevel> --format stl|3mf [--body NAME] -o <out>";
    Ok(ExportArgs {
        file: file.context(usage)?,
        format: format.context(usage)?,
        body,
        out: out.context(usage)?,
    })
}

fn export(args: &[String]) -> Result<ExitCode> {
    let args = export_args(args)?;
    let (state, regenerated) = load(&args.file)?;
    let cx = OpContext::default();
    let mut meshes = Vec::new();
    for body in regenerated.state.bodies.iter() {
        let name = state.body_name(body.id);
        let props = state.body_props(body.id);
        let wanted = match &args.body {
            Some(only) => *only == name,
            None => props.visible,
        };
        if !wanted {
            continue;
        }
        let options = TessellationOptions::export(&body.body.bounds());
        let mesh = tessellate(&body.body, &options, &cx)
            .with_context(|| format!("tessellating {name}"))?;
        let [r, g, b] = props.color.unwrap_or(DEFAULT_COLOR);
        meshes.push(bevel_io::export_mesh(&name, Some([r, g, b, 255]), &mesh)?);
    }
    if meshes.is_empty() {
        match &args.body {
            Some(only) => bail!("no body is named `{only}`"),
            None => bail!("there are no visible bodies to export"),
        }
    }
    let mut bytes = std::io::Cursor::new(Vec::new());
    match args.format.as_str() {
        "stl" => bevel_io::stl::write_binary(&meshes, &mut bytes)?,
        "3mf" => bevel_io::threemf::write(&meshes, &mut bytes)?,
        other => bail!("unknown format `{other}`; use stl or 3mf"),
    }
    std::fs::write(&args.out, bytes.into_inner())
        .with_context(|| format!("cannot write {}", args.out.display()))?;
    println!("wrote {} bodies to {}", meshes.len(), args.out.display());
    Ok(ExitCode::SUCCESS)
}
