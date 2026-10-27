// SPDX-License-Identifier: GPL-3.0-or-later

//! Property tests: primitives with random valid sizes under random rigid
//! frames validate, have the right genus, point every face outwards
//! (checked against an independent analytic inside test) and survive a
//! serialization round trip.

// Helpers outside `#[test]` functions unwrap freely too.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use bevel_geom::{CurveGeom, SurfaceGeom};
use bevel_kernel::{
    Body, BoxParams, ConeParams, CylinderParams, OpContext, OpOutput, OpTag, SphereParams,
    TorusParams, box_, cone, cylinder, sphere, torus, validate,
};
use bevel_math::{DQuat, DVec3, Frame3, Point2, Point3, Vector3};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed, TestCaseError};

/// A fixed-seed configuration so that runs are reproducible.
///
/// For exploratory runs, `BEVEL_PROPTEST_SEED` replaces the seed and
/// `BEVEL_PROPTEST_CASES` the number of cases.
fn config(cases: u32) -> Config {
    let env = |name: &str| std::env::var(name).ok().and_then(|v| v.parse::<u64>().ok());
    Config {
        cases: env("BEVEL_PROPTEST_CASES").map_or(cases, |c| u32::try_from(c).unwrap_or(cases)),
        rng_seed: RngSeed::Fixed(env("BEVEL_PROPTEST_SEED").unwrap_or(0x00b0_d1e5)),
        failure_persistence: None,
        ..Config::default()
    }
}

/// A random rigid frame with its origin within 500 mm of the world origin.
fn frame() -> impl Strategy<Value = Frame3> {
    (
        prop::array::uniform3(-1.0f64..1.0),
        0.0f64..TAU,
        prop::array::uniform3(-500.0f64..500.0),
    )
        .prop_map(|(a, angle, o)| {
            let axis = DVec3::from_array(a).try_normalize().unwrap_or(DVec3::Z);
            let q = DQuat::from_axis_angle(axis, angle);
            Frame3 {
                origin: DVec3::from_array(o),
                x: q * DVec3::X,
                y: q * DVec3::Y,
                z: q * DVec3::Z,
            }
        })
}

/// A random length in millimetres, log-uniform from 1 um to 20 m, so that
/// tiny, huge and badly proportioned primitives all appear.
fn length() -> impl Strategy<Value = f64> {
    (-3.0f64..4.3).prop_map(|e| 10f64.powf(e))
}

/// Interior points of each face in `(u, v)`: the centroid of the outer
/// loop's samples and the points halfway from it to some of them. Every
/// primitive's faces are convex in `(u, v)`, so all of these are inside.
fn face_samples(body: &Body, face: bevel_kernel::FaceKey) -> Vec<Point2> {
    let f = body.face(face).unwrap();
    let outer = body.loop_(f.loops[0]).unwrap();
    let mut boundary = Vec::new();
    for &ck in &outer.coedges {
        let c = body.coedge(ck).unwrap();
        let e = body.edge(c.edge).unwrap();
        for i in 0..8 {
            let t = e.range.lerp(f64::from(i) / 8.0);
            boundary.push(c.pcurve.point(t));
        }
    }
    let centroid = boundary.iter().copied().sum::<Point2>() / boundary.len() as f64;
    let mut out = vec![centroid];
    out.extend(boundary.iter().step_by(3).map(|&b| centroid.lerp(b, 0.5)));
    out
}

/// Checks everything the brief asks of a primitive: no issues, the genus,
/// outward normals against `inside` (in the frame's local coordinates),
/// bounds that contain the faces, and an exact serialization round trip.
fn check(
    out: &OpOutput,
    frame: &Frame3,
    genus: u32,
    scale: f64,
    inside: impl Fn(Point3) -> bool,
) -> Result<(), TestCaseError> {
    prop_assert_eq!(out.bodies.len(), 1);
    let body = &*out.bodies[0];
    let issues = validate(body);
    prop_assert!(issues.is_empty(), "{:?}", issues);
    for &shell in body.shell_keys() {
        let counts = body.euler_counts(shell).unwrap();
        prop_assert_eq!(counts.genus(), Some(genus), "{:?}", counts);
    }

    let step = 1e-3 * scale;
    let bounds = body.bounds();
    for (fk, f) in body.faces() {
        for uv in face_samples(body, fk) {
            let p = f.surface.point(uv.x, uv.y);
            let Some(n) = f.surface.normal(uv.x, uv.y) else {
                continue;
            };
            let n = if f.reversed { -n } else { n };
            let outside = frame.to_local(p + n * step);
            let within = frame.to_local(p - n * step);
            prop_assert!(!inside(outside), "{}: {:?} steps inside", f.name, uv);
            prop_assert!(inside(within), "{}: {:?} steps outside", f.name, uv);
            let slack = Vector3::splat(1e-9 * (1.0 + p.abs().max_element()));
            prop_assert!(
                p.cmpge(bounds.min - slack).all() && p.cmple(bounds.max + slack).all(),
                "{:?} is outside the bounds {:?}",
                p,
                bounds
            );
        }
    }
    // Edge curves lie in the bounds too.
    for (_, e) in body.edges() {
        let p = e.curve.point(e.range.mid());
        let slack = Vector3::splat(1e-9 * (1.0 + p.abs().max_element()));
        prop_assert!(p.cmpge(bounds.min - slack).all() && p.cmple(bounds.max + slack).all());
    }

    let json = serde_json::to_string(body).unwrap();
    let back: Body = serde_json::from_str(&json).unwrap();
    prop_assert!(&back == body, "the round trip changed the body");
    prop_assert!(validate(&back).is_empty());
    prop_assert_eq!(serde_json::to_string(&back).unwrap(), json);
    Ok(())
}

/// The distance from the local `z` axis.
fn rho(q: Point3) -> f64 {
    q.x.hypot(q.y)
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn boxes_are_valid_and_outward(frame in frame(), x in length(), y in length(), z in length()) {
        let params = BoxParams { frame, size: Vector3::new(x, y, z) };
        let out = box_(&params, OpTag(1), &mut OpContext::new()).unwrap();
        let inside = |q: Point3| {
            (0.0..x).contains(&q.x) && (0.0..y).contains(&q.y) && (0.0..z).contains(&q.z)
        };
        check(&out, &frame, 0, x.min(y).min(z), inside)?;
    }

    #[test]
    fn cylinders_are_valid_and_outward(frame in frame(), r in length(), h in length()) {
        let params = CylinderParams { frame, radius: r, height: h };
        let out = cylinder(&params, OpTag(1), &mut OpContext::new()).unwrap();
        let inside = |q: Point3| rho(q) < r && (0.0..h).contains(&q.z);
        check(&out, &frame, 0, r.min(h), inside)?;
    }

    #[test]
    fn spheres_are_valid_and_outward(frame in frame(), r in length()) {
        let params = SphereParams { frame, radius: r };
        let out = sphere(&params, OpTag(1), &mut OpContext::new()).unwrap();
        check(&out, &frame, 0, r, |q: Point3| q.length() < r)?;
    }

    #[test]
    fn cones_are_valid_and_outward(
        frame in frame(),
        r0 in prop_oneof![1 => Just(0.0), 3 => length()],
        r1 in prop_oneof![1 => Just(0.0), 3 => length()],
        h in length(),
    ) {
        let dr = (r1 - r0).abs();
        prop_assume!(dr > 1e-3 * r0.max(r1));
        // Cones too flat for the half-angle representation are refused
        // (see `too_flat_cones_are_refused`).
        prop_assume!(4.0 * f64::EPSILON * dr * (dr / h).max(1.0) <= 1e-7);
        let params = ConeParams { frame, bottom_radius: r0, top_radius: r1, height: h };
        let out = cone(&params, OpTag(1), &mut OpContext::new()).unwrap();
        let inside = |q: Point3| {
            (0.0..h).contains(&q.z) && rho(q) < r0 + (r1 - r0) * q.z / h
        };
        let scale = h.min(r0.max(r1)).min((r1 - r0).abs());
        check(&out, &frame, 0, scale, inside)?;
    }

    #[test]
    fn tori_are_valid_and_outward(frame in frame(), big in length(), fraction in 0.02f64..0.98) {
        let small = big * fraction;
        prop_assume!(big - small > 1e-4);
        let params = TorusParams { frame, major_radius: big, minor_radius: small };
        let out = torus(&params, OpTag(1), &mut OpContext::new()).unwrap();
        let inside = |q: Point3| (rho(q) - big).hypot(q.z) < small;
        check(&out, &frame, 1, small.min(big - small), inside)?;
    }
}
