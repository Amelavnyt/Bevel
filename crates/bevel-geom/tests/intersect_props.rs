// SPDX-License-Identifier: GPL-3.0-or-later

//! Property tests for surface–surface intersection: the exact fast paths
//! agree with the general method on random placements, and results do not
//! depend on where the pair sits in space.

#![allow(clippy::unwrap_used, clippy::panic, missing_docs, reason = "tests")]

use bevel_geom::intersect::{Rect, SsiResult, surface_surface, surface_surface_general};
use bevel_geom::{CurveGeom, Cylinder, Plane, Sphere, Surface, SurfaceGeom};
use bevel_math::{DMat4, DQuat, DVec3, Frame3, Interval, Point3, Vector3};
use proptest::prelude::*;
use proptest::test_runner::RngSeed;
use std::f64::consts::{FRAC_PI_2, TAU};

const TOL: f64 = 1e-6;

fn length(r: &SsiResult, a: &Surface, b: &Surface) -> f64 {
    let SsiResult::Meet { curves, .. } = r else {
        panic!("unexpected coincidence");
    };
    let mut total = 0.0;
    for c in curves {
        let n = 400;
        let mut prev: Option<Point3> = None;
        for i in 0..=n {
            let t = c.range.min + c.range.length() * f64::from(i) / f64::from(n);
            let p = c.curve.point(t);
            let (pa, pb) = (c.pcurve_a.point(t), c.pcurve_b.point(t));
            assert!((a.point(pa.x, pa.y) - p).length() <= 2.0 * TOL);
            assert!((b.point(pb.x, pb.y) - p).length() <= 2.0 * TOL);
            if let Some(q) = prev {
                total += (p - q).length();
            }
            prev = Some(p);
        }
    }
    total
}

fn unit(v: [f64; 3]) -> Option<Vector3> {
    let v = Vector3::from_array(v);
    (v.length() > 0.1).then(|| v / v.length())
}

fn config() -> ProptestConfig {
    ProptestConfig {
        cases: 48,
        rng_seed: RngSeed::Fixed(0x551C_0FFE),
        failure_persistence: None,
        ..ProptestConfig::default()
    }
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn plane_sphere_fast_path_matches_general(
        centre in prop::array::uniform3(-10.0..10.0f64),
        r in 2.0..12.0f64,
        normal in prop::array::uniform3(-1.0..1.0f64),
        offset in -0.8..0.8f64,
    ) {
        let Some(n) = unit(normal) else { return Ok(()) };
        let c = Point3::from_array(centre);
        let s = Surface::Sphere(Sphere::new(Frame3::from_normal(c, Vector3::Z).unwrap(), r).unwrap());
        let rs = Rect::new(Interval::new(0.0, TAU), Interval::new(-FRAC_PI_2, FRAC_PI_2));
        // A plane cutting the sphere at a fraction of its radius from the
        // centre (well away from tangency).
        let p = Surface::Plane(Plane::new(Frame3::from_normal(c + n * (offset * r), n).unwrap()).unwrap());
        let rp = Rect::new(Interval::new(-40.0, 40.0), Interval::new(-40.0, 40.0));
        let fast = surface_surface(&p, rp, &s, rs, TOL).unwrap();
        let expected = TAU * r * (1.0 - offset * offset).sqrt();
        let lf = length(&fast, &p, &s);
        prop_assert!((lf - expected).abs() < 1e-3, "fast {lf} vs {expected}");
        // The general method stops just short of the sphere's poles, where
        // its parametrization is singular (see the robustness log in
        // docs/KERNEL.md); compare it only on circles that keep clear.
        let pole_gap = [c + Vector3::Z * r, c - Vector3::Z * r]
            .iter()
            .map(|q| (*q - (c + n * (offset * r))).dot(n).abs())
            .fold(f64::INFINITY, f64::min);
        if pole_gap > 0.05 * r {
            let general = surface_surface_general(&p, rp, &s, rs, TOL).unwrap();
            let lg = length(&general, &p, &s);
            prop_assert!((lg - expected).abs() < 1e-3, "general {lg} vs {expected}");
        }
    }

    #[test]
    fn crossing_cylinders_do_not_care_where_they_are(
        axis in prop::array::uniform3(-1.0..1.0f64),
        angle in 0.0..TAU,
        shift in prop::array::uniform3(-50.0..50.0f64),
    ) {
        let Some(axis) = unit(axis) else { return Ok(()) };
        let m = DMat4::from_rotation_translation(DQuat::from_axis_angle(axis, angle), DVec3::from_array(shift));
        let small = Surface::Cylinder(Cylinder::new(Frame3::WORLD, 6.0).unwrap());
        let big_frame = Frame3::from_axes(Point3::ZERO, Vector3::Y, Vector3::Z).unwrap();
        let big = Surface::Cylinder(Cylinder::new(Frame3 { z: Vector3::X, ..big_frame }, 10.0).unwrap());
        let r = Rect::new(Interval::new(0.0, TAU), Interval::new(-20.0, 20.0));
        let here = surface_surface_general(&small, r, &big, r, TOL).unwrap();
        let (ms, mb) = (small.transformed(&m).unwrap(), big.transformed(&m).unwrap());
        let there = surface_surface_general(&ms, r, &mb, r, TOL).unwrap();
        let (lh, lt) = (length(&here, &small, &big), length(&there, &ms, &mb));
        prop_assert!((lh - lt).abs() < 1e-3, "{lh} vs {lt}");
    }
}
