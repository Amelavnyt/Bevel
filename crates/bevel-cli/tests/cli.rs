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

fn block(origin: [f64; 3], tag: u64) -> bevel_kernel::Body {
    use bevel_math::{Frame3, Point3, Vector3};
    let params = bevel_kernel::BoxParams {
        frame: Frame3 {
            origin: Point3::from_array(origin),
            ..Frame3::WORLD
        },
        size: Vector3::from_array([10.0; 3]),
    };
    let mut cx = bevel_kernel::OpContext::new();
    let out = bevel_kernel::box_(&params, bevel_kernel::OpTag(tag), &mut cx).unwrap();
    (*out.bodies[0]).clone()
}

#[test]
fn kernel_repro_replays_a_dump() {
    use bevel_kernel::{BooleanDump, BooleanOp, KernelError, OpContext, OpTag};
    let dump = BooleanDump::new(
        &block([0.0; 3], 1),
        &block([5.0; 3], 2),
        BooleanOp::Union,
        OpTag(3),
        &OpContext::new(),
        &KernelError::Cancelled,
    );
    let path = out_dir().join("union.json");
    std::fs::write(&path, dump.to_json().unwrap()).unwrap();
    let o = run(&["kernel-repro", path.to_str().unwrap()]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let text = stdout(&o);
    assert!(text.contains("succeeds: 1 bodies"), "{text}");
    assert!(text.contains("volume 1875.000 mm^3"), "{text}");
}

#[test]
fn kernel_repro_fails_on_a_missing_file() {
    let o = run(&["kernel-repro", "does-not-exist.json"]);
    assert!(!o.status.success());
}
