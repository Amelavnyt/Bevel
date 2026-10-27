// SPDX-License-Identifier: GPL-3.0-or-later

//! The CLI driven over the fixture documents.

#![allow(clippy::unwrap_used, clippy::panic, reason = "tests")]

use std::path::PathBuf;
use std::process::{Command, Output};

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/bevel/v1/primitives.bevel")
}

fn run(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_bevel-cli"))
        .args(args)
        .output()
        .unwrap()
}

fn stdout(o: &Output) -> String {
    String::from_utf8_lossy(&o.stdout).into_owned()
}

fn out_dir() -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("cli");
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn rebuild_reports_every_feature() {
    let o = run(&["rebuild", fixture().to_str().unwrap()]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let text = stdout(&o);
    for name in [
        "Box1: ok",
        "Cylinder1: ok",
        "Sphere1: ok",
        "Cone1: ok",
        "Torus1: ok",
    ] {
        assert!(text.contains(name), "{text}");
    }
    assert!(text.contains("5 features, 5 bodies, 0 problems"), "{text}");
}

#[test]
fn info_lists_parameters_and_volumes() {
    let o = run(&["info", fixture().to_str().unwrap()]);
    assert!(o.status.success());
    let text = stdout(&o);
    assert!(text.contains("d1 = 40 mm = 40.000 mm"), "{text}");
    assert!(text.contains("Box1: volume 64000.000 mm^3"), "{text}");
}

#[test]
fn export_writes_meshes() {
    let stl = out_dir().join("all.stl");
    let o = run(&[
        "export",
        fixture().to_str().unwrap(),
        "--format",
        "stl",
        "-o",
        stl.to_str().unwrap(),
    ]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let bytes = std::fs::read(&stl).unwrap();
    assert!(bevel_io::stl::read(&bytes).unwrap().len() > 12);

    let one = out_dir().join("box.3mf");
    let o = run(&[
        "export",
        fixture().to_str().unwrap(),
        "--format",
        "3mf",
        "--body",
        "Box1",
        "-o",
        one.to_str().unwrap(),
    ]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let meshes = bevel_io::threemf::read(&std::fs::read(&one).unwrap()).unwrap();
    assert_eq!(meshes.len(), 1);
    assert!(meshes[0].is_watertight());
}

#[test]
fn bad_input_fails_with_a_message() {
    let o = run(&["rebuild", "does-not-exist.bevel"]);
    assert_eq!(o.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&o.stderr).contains("cannot read"));
    let o = run(&[
        "export",
        fixture().to_str().unwrap(),
        "--format",
        "obj",
        "-o",
        "x",
    ]);
    assert_eq!(o.status.code(), Some(2));
    let o = run(&["frobnicate"]);
    assert_eq!(o.status.code(), Some(2));
}
