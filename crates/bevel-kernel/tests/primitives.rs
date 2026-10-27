// SPDX-License-Identifier: GPL-3.0-or-later

//! Primitives: entity counts, names, determinism, parameter errors,
//! serialization and a snapshot of the unit box.

// Helpers outside `#[test]` functions unwrap freely too.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeSet;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use bevel_geom::Surface;
use bevel_kernel::{
    Body, BoxParams, ConeParams, CylinderParams, EntityKey, EntityKind, KernelError, OpContext,
    OpOutput, OpTag, Role, Source, SphereParams, TopoName, TorusParams, box_, cone, cylinder,
    sphere, torus, validate,
};
use bevel_math::{Frame3, Point3, Vector3};
use serde_json::Value;

fn cx() -> OpContext {
    OpContext::new()
}

fn unit_box() -> OpOutput {
    let params = BoxParams {
        frame: Frame3::WORLD,
        size: Vector3::ONE,
    };
    box_(&params, OpTag(1), &mut cx()).unwrap()
}

/// Every primitive once, with a name for messages.
fn all(tag: OpTag) -> Vec<(&'static str, OpOutput)> {
    let f = Frame3::WORLD;
    let mut c = cx();
    let cone_with = |bottom_radius, top_radius, c: &mut OpContext| {
        let params = ConeParams {
            frame: f,
            bottom_radius,
            top_radius,
            height: 2.0,
        };
        cone(&params, tag, c).unwrap()
    };
    vec![
        (
            "box",
            box_(
                &BoxParams {
                    frame: f,
                    size: Vector3::new(1.0, 2.0, 3.0),
                },
                tag,
                &mut c,
            )
            .unwrap(),
        ),
        (
            "cylinder",
            cylinder(
                &CylinderParams {
                    frame: f,
                    radius: 1.0,
                    height: 2.0,
                },
                tag,
                &mut c,
            )
            .unwrap(),
        ),
        (
            "sphere",
            sphere(
                &SphereParams {
                    frame: f,
                    radius: 1.5,
                },
                tag,
                &mut c,
            )
            .unwrap(),
        ),
        ("frustum up", cone_with(1.0, 2.0, &mut c)),
        ("frustum down", cone_with(2.0, 0.5, &mut c)),
        ("cone apex top", cone_with(2.0, 0.0, &mut c)),
        ("cone apex bottom", cone_with(0.0, 1.0, &mut c)),
        (
            "torus",
            torus(
                &TorusParams {
                    frame: f,
                    major_radius: 3.0,
                    minor_radius: 1.0,
                },
                tag,
                &mut c,
            )
            .unwrap(),
        ),
    ]
}

/// (vertices, edges, of which degenerate, faces, loops, coedges, genus).
fn counts(b: &Body) -> (usize, usize, usize, usize, usize, usize, u32) {
    let degenerate = b.edges().filter(|(_, e)| e.degenerate).count();
    let shell = b.shell_keys()[0];
    (
        b.vertex_count(),
        b.edge_count(),
        degenerate,
        b.face_count(),
        b.loop_count(),
        b.coedge_count(),
        b.euler_counts(shell).unwrap().genus().unwrap(),
    )
}

#[test]
fn entity_counts_per_primitive() {
    let expected = [
        ("box", (8, 12, 0, 6, 6, 24, 0)),
        ("cylinder", (2, 3, 0, 3, 3, 6, 0)),
        ("sphere", (2, 3, 2, 1, 1, 4, 0)),
        ("frustum up", (2, 3, 0, 3, 3, 6, 0)),
        ("frustum down", (2, 3, 0, 3, 3, 6, 0)),
        ("cone apex top", (2, 3, 1, 2, 2, 5, 0)),
        ("cone apex bottom", (2, 3, 1, 2, 2, 5, 0)),
        ("torus", (1, 2, 0, 1, 1, 4, 1)),
    ];
    for ((name, out), (expected_name, expected)) in all(OpTag(1)).iter().zip(expected) {
        assert_eq!(*name, expected_name);
        let b = &out.bodies[0];
        assert_eq!(counts(b), expected, "{name}");
        assert_eq!(validate(b), vec![], "{name}");
        assert_eq!(b.shell_count(), 1, "{name}");
    }
}

#[test]
fn seams_and_adjacency() {
    for (name, out) in all(OpTag(1)) {
        let b = &out.bodies[0];
        for (ek, e) in b.edges() {
            let uses = b.edge_coedges(ek).len();
            if e.degenerate {
                assert_eq!(uses, 1, "{name}");
            } else {
                assert_eq!(uses, 2, "{name}");
            }
            let seam = matches!(
                e.name,
                TopoName::Generated {
                    role: Role::Seam(_),
                    ..
                }
            );
            assert_eq!(b.is_seam(ek), seam, "{name}: {}", e.name);
            assert_eq!(
                b.edge_faces(ek).len(),
                if seam || e.degenerate { 1 } else { 2 }
            );
        }
        for (vk, _) in b.vertices() {
            assert!(!b.vertex_edges(vk).is_empty());
        }
        for (fk, _) in b.faces() {
            let edges = b.face_edges(fk);
            assert!(b.face_coedges(fk).count() >= edges.len());
        }
    }
}

#[test]
fn names_are_unique_and_recorded_in_the_history() {
    for (name, out) in all(OpTag(7)) {
        let b = &out.bodies[0];
        let mut names = BTreeSet::new();
        for (_, v) in b.vertices() {
            assert!(names.insert(v.name.clone()), "{name}");
            assert_eq!(out.history.get(&v.name).unwrap().kind, EntityKind::Vertex);
        }
        for (_, e) in b.edges() {
            assert!(names.insert(e.name.clone()), "{name}");
            assert_eq!(out.history.get(&e.name).unwrap().kind, EntityKind::Edge);
        }
        for (_, f) in b.faces() {
            assert!(names.insert(f.name.clone()), "{name}");
            assert_eq!(out.history.get(&f.name).unwrap().kind, EntityKind::Face);
        }
        assert_eq!(out.history.len(), names.len(), "{name}");
        for entry in out.history.iter() {
            assert!(entry.from.is_empty(), "primitives generate from nothing");
            assert_eq!(entry.name.op(), OpTag(7));
            assert!(matches!(
                entry.name,
                TopoName::Generated {
                    source: Source::Primitive,
                    ..
                }
            ));
            assert!(b.find_name(&entry.name).is_some());
        }
    }
}

#[test]
fn box_faces_have_their_roles_and_outward_planes() {
    let out = unit_box();
    let b = &out.bodies[0];
    let normals = [
        (Role::Bottom, -Vector3::Z),
        (Role::Top, Vector3::Z),
        (Role::Front, -Vector3::Y),
        (Role::Back, Vector3::Y),
        (Role::Left, -Vector3::X),
        (Role::Right, Vector3::X),
    ];
    for (role, normal) in normals {
        let name = TopoName::generated(OpTag(1), Source::Primitive, role);
        let Some(EntityKey::Face(fk)) = b.find_name(&name) else {
            panic!("no {role:?} face")
        };
        let face = b.face(fk).unwrap();
        assert!(!face.reversed);
        let Surface::Plane(plane) = &face.surface else {
            panic!("not planar")
        };
        assert!(plane.normal(0.0, 0.0).unwrap().distance(normal) < 1e-15);
    }
    // Vertex 7 is the far corner, edge 4 runs along y from the origin.
    let v7 = TopoName::generated(OpTag(1), Source::Primitive, Role::Vertex(7));
    let Some(EntityKey::Vertex(vk)) = b.find_name(&v7) else {
        panic!("no vertex 7")
    };
    assert!(b.vertex(vk).unwrap().point.distance(Point3::ONE) < 1e-15);
    let e4 = TopoName::generated(OpTag(1), Source::Primitive, Role::Edge(4));
    let Some(EntityKey::Edge(ek)) = b.find_name(&e4) else {
        panic!("no edge 4")
    };
    let e = b.edge(ek).unwrap();
    assert!(e.point(e.range.max).distance(Point3::Y) < 1e-15);
}

#[test]
fn building_twice_gives_identical_bodies() {
    let first = all(OpTag(3));
    let second = all(OpTag(3));
    for ((name, a), (_, b)) in first.iter().zip(&second) {
        assert_eq!(a.bodies[0], b.bodies[0], "{name}");
        assert_eq!(a.history, b.history, "{name}");
        let ja = serde_json::to_string(&*a.bodies[0]).unwrap();
        let jb = serde_json::to_string(&*b.bodies[0]).unwrap();
        assert_eq!(ja, jb, "{name}");
    }
    // A different tag gives different names, so different bodies.
    let other = all(OpTag(4));
    assert_ne!(first[0].1.bodies[0], other[0].1.bodies[0]);
}

#[test]
fn bounds_are_exact_for_axis_aligned_primitives() {
    let out = all(OpTag(1));
    let expect = [
        (Point3::ZERO, Point3::new(1.0, 2.0, 3.0)),
        (Point3::new(-1.0, -1.0, 0.0), Point3::new(1.0, 1.0, 2.0)),
        (Point3::splat(-1.5), Point3::splat(1.5)),
    ];
    for ((name, o), (min, max)) in out.iter().zip(expect) {
        let b = o.bodies[0].bounds();
        assert!(
            b.min.distance(min) < 1e-12 && b.max.distance(max) < 1e-12,
            "{name}: {b:?}"
        );
    }
    let torus = out[7].1.bodies[0].bounds();
    assert!(torus.max.distance(Point3::new(4.0, 4.0, 1.0)) < 1e-12);
}

/// Asserts that `result` is an `InvalidParameter` error for `name`.
fn invalid(result: Result<OpOutput, KernelError>, name: &str) {
    match result {
        Err(KernelError::InvalidParameter { name: n, .. }) => assert_eq!(n, name),
        other => panic!("expected an invalid {name}, got {other:?}"),
    }
}

#[test]
fn invalid_parameters_are_refused() {
    let f = Frame3::WORLD;
    let t = OpTag(1);
    let boxed = |x, y, z| {
        box_(
            &BoxParams {
                frame: f,
                size: Vector3::new(x, y, z),
            },
            t,
            &mut cx(),
        )
    };
    invalid(boxed(-1.0, 1.0, 1.0), "box size x");
    invalid(boxed(1.0, 0.0, 1.0), "box size y");
    invalid(boxed(1.0, 1.0, f64::NAN), "box size z");
    invalid(boxed(1.0, 1e-6, 1.0), "box size y");
    invalid(boxed(1.0, 3e5, 1.0), "box size y");

    let cyl = |radius, height| {
        cylinder(
            &CylinderParams {
                frame: f,
                radius,
                height,
            },
            t,
            &mut cx(),
        )
    };
    invalid(cyl(0.0, 1.0), "cylinder radius");
    invalid(cyl(1.0, -2.0), "cylinder height");
    invalid(cyl(f64::INFINITY, 1.0), "cylinder radius");

    let sph = |radius| sphere(&SphereParams { frame: f, radius }, t, &mut cx());
    invalid(sph(0.0), "sphere radius");
    invalid(sph(-3.0), "sphere radius");

    let con = |bottom_radius, top_radius, height| {
        cone(
            &ConeParams {
                frame: f,
                bottom_radius,
                top_radius,
                height,
            },
            t,
            &mut cx(),
        )
    };
    invalid(con(0.0, 0.0, 1.0), "cone top radius");
    invalid(con(1.0, 1.0, 1.0), "cone top radius");
    invalid(con(-1.0, 1.0, 1.0), "cone bottom radius");
    invalid(con(1.0, 2.0, 0.0), "cone height");
    assert!(con(1e-7, 2.0, 1.0).is_ok(), "a tiny radius is an apex");

    let tor = |major_radius, minor_radius| {
        torus(
            &TorusParams {
                frame: f,
                major_radius,
                minor_radius,
            },
            t,
            &mut cx(),
        )
    };
    invalid(tor(1.0, 1.0), "torus minor radius");
    invalid(tor(1.0, 2.0), "torus minor radius");
    invalid(tor(2.0, 0.0), "torus minor radius");
    invalid(tor(0.0, 0.0), "torus major radius");
}

#[test]
fn too_flat_cones_are_refused() {
    // Found by the property tests: with a half-angle this close to 90
    // degrees the cone's radius is off by 1.5e-6 mm, more than the
    // tolerance.
    let flat = ConeParams {
        frame: Frame3::WORLD,
        bottom_radius: 7_648.869_257_817_664,
        top_radius: 0.0,
        height: 0.001_752_091_240_025_291_9,
    };
    invalid(cone(&flat, OpTag(1), &mut cx()), "cone height");
    // A flat cone that the representation handles is fine.
    let washer = ConeParams {
        frame: Frame3::WORLD,
        bottom_radius: 100.0,
        top_radius: 0.0,
        height: 0.1,
    };
    let out = cone(&washer, OpTag(1), &mut cx()).unwrap();
    assert_eq!(validate(&out.bodies[0]), vec![]);
}

#[test]
fn frames_model_space_tolerance_and_cancellation() {
    let skewed = Frame3 {
        y: Vector3::new(0.1, 1.0, 0.0),
        ..Frame3::WORLD
    };
    let far = Frame3 {
        origin: Point3::new(0.0, 2e5, 0.0),
        ..Frame3::WORLD
    };
    for frame in [skewed, far] {
        let r = sphere(&SphereParams { frame, radius: 1.0 }, OpTag(1), &mut cx());
        assert_eq!(r.unwrap_err(), KernelError::InvalidFrame);
    }
    let edge = Frame3 {
        origin: Point3::new(99_990.0, 0.0, 0.0),
        ..Frame3::WORLD
    };
    let r = sphere(
        &SphereParams {
            frame: edge,
            radius: 20.0,
        },
        OpTag(1),
        &mut cx(),
    );
    match r {
        Err(e @ KernelError::OutsideModelSpace { .. }) => {
            assert!(e.to_string().contains("100010"), "{e}");
        }
        other => panic!("expected an out-of-space error, got {other:?}"),
    }
    let mut loose = OpContext {
        tolerance: 1.0,
        ..cx()
    };
    let params = SphereParams {
        frame: Frame3::WORLD,
        radius: 1.0,
    };
    invalid(sphere(&params, OpTag(1), &mut loose), "tolerance");

    let token = Arc::new(AtomicBool::new(true));
    let mut cancelled = OpContext::with_cancel(token);
    assert_eq!(
        sphere(&params, OpTag(1), &mut cancelled).unwrap_err(),
        KernelError::Cancelled
    );

    // A coarser tolerance is carried by every vertex and edge.
    let mut coarse = OpContext {
        tolerance: 1e-3,
        ..cx()
    };
    let out = sphere(&params, OpTag(1), &mut coarse).unwrap();
    let b = &out.bodies[0];
    assert!(b.vertices().all(|(_, v)| (v.tol - 1e-3).abs() < 1e-18));
    assert!(b.edges().all(|(_, e)| (e.tol - 1e-3).abs() < 1e-18));
}

#[test]
fn bodies_round_trip_through_json() {
    for (name, out) in all(OpTag(2)) {
        let b = &*out.bodies[0];
        let json = serde_json::to_string(b).unwrap();
        let back: Body = serde_json::from_str(&json).unwrap();
        assert_eq!(&back, b, "{name}");
        assert_eq!(validate(&back), vec![]);
        assert_eq!(serde_json::to_string(&back).unwrap(), json, "{name}");
    }
}

/// The unit box as a JSON value, for corrupting.
fn box_json() -> Value {
    serde_json::to_value(&*unit_box().bodies[0]).unwrap()
}

/// Asserts that `value` does not deserialize as a body, and that the
/// message contains `needle`.
fn refused(value: Value, needle: &str) {
    match serde_json::from_value::<Body>(value) {
        Ok(_) => panic!("a corrupt body was accepted"),
        Err(e) => assert!(e.to_string().contains(needle), "{e}"),
    }
}

#[test]
fn corrupt_bodies_fail_to_deserialize() {
    let mut v = box_json();
    v["vertices"][0]["point"][2] = Value::from(0.25);
    refused(v, "ends 0.25 mm away from vertex");

    let mut v = box_json();
    v["edges"][0]["end"] = Value::from(99);
    refused(v, "index refers past the end");

    let mut v = box_json();
    v["faces"][0]["loops"][0][0]["reversed"] = Value::from(true);
    refused(v, "do not join up");

    let mut v = box_json();
    v["shells"][0]["closed"] = Value::from(false);
    refused(v, "is not closed");

    let mut v = box_json();
    v["faces"][1]["name"] = v["faces"][0]["name"].clone();
    refused(v, "share the name");

    let mut v = box_json();
    v["extra"] = Value::from(1);
    refused(v, "unknown field");

    // Corrupt geometry is refused by the geometry types themselves.
    let mut v = box_json();
    v["faces"][0]["surface"]["Plane"]["frame"]["x"] = serde_json::json!([2.0, 0.0, 0.0]);
    refused(v, "orthonormal");

    let mut c = serde_json::to_value(&*all(OpTag(1))[1].1.bodies[0]).unwrap();
    c["faces"][2]["surface"]["Cylinder"]["radius"] = Value::from(0.0);
    refused(c, "radius must be positive");

    let mut s = serde_json::to_value(&*all(OpTag(1))[2].1.bodies[0]).unwrap();
    let pole = s["edges"]
        .as_array()
        .unwrap()
        .iter()
        .position(|e| e["degenerate"] == true)
        .unwrap();
    s["edges"][pole]["curve"]["Nurbs"]["knots"] = serde_json::json!([0.0, 3.0, 1.0, 6.0]);
    refused(s, "non-decreasing");
}

#[test]
fn unit_box_snapshot() {
    insta::assert_json_snapshot!("unit_box", &*unit_box().bodies[0]);
}
