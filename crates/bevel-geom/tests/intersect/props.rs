// SPDX-License-Identifier: GPL-3.0-or-later

//! Properties of random line and circle pairs, and invariance under rigid
//! motions.

use std::f64::consts::{FRAC_PI_2, TAU};

use bevel_geom::intersect::{
    CurveCurveHit, CurveSurfaceHit, Rect, curve_curve_2d, curve_curve_3d, curve_surface,
};
use bevel_geom::{
    Circle2, Circle3, Curve2, Curve3, CurveGeom, Line2, Line3, Sphere, Surface, SurfaceGeom, Torus,
};
use bevel_math::{DMat4, DQuat, DVec3, Frame3, Interval, Point2, Point3, Vector2, Vector3};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};

use crate::helpers::{TOL, assert_same_spots, check_hits2, nurbs2, spots2};

/// A fixed seed, so that runs are reproducible.
fn config() -> Config {
    Config {
        cases: 128,
        rng_seed: RngSeed::Fixed(0x1e7_5ec7),
        failure_persistence: None,
        ..Config::default()
    }
}

fn point2(r: f64) -> impl Strategy<Value = Point2> {
    (-r..r, -r..r).prop_map(|(x, y)| Point2::new(x, y))
}

fn point3(r: f64) -> impl Strategy<Value = Point3> {
    (-r..r, -r..r, -r..r).prop_map(|(x, y, z)| Point3::new(x, y, z))
}

fn rotation() -> impl Strategy<Value = DQuat> {
    (point3(1.0), -3.0..3.0f64).prop_filter_map("axis", |(axis, angle)| {
        axis.try_normalize()
            .map(|a| DQuat::from_axis_angle(a, angle))
    })
}

/// A random range of length at least 0.5 inside `[-lim, lim]`.
fn range(lim: f64) -> impl Strategy<Value = Interval> {
    (-lim..lim, 0.5..2.0 * lim).prop_map(move |(a, len)| Interval::new(a, (a + len).min(lim + 0.5)))
}

/// A random line or circle in the plane.
fn curve2() -> impl Strategy<Value = (Curve2, Interval)> {
    prop_oneof![
        (point2(3.0), -3.2..3.2f64, range(6.0)).prop_map(|(o, a, r)| {
            let l = Line2::new(o, Vector2::from_angle(a)).unwrap();
            (Curve2::Line(l), r)
        }),
        (
            point2(3.0),
            0.2..3.0f64,
            -3.2..3.2f64,
            any::<bool>(),
            range(7.0)
        )
            .prop_map(|(c, rad, a, ccw, r)| {
                let c = Circle2::from_axis(c, Vector2::from_angle(a), ccw, rad).unwrap();
                (Curve2::Circle(c), r)
            }),
    ]
}

/// A random line or circle in space.
fn curve3() -> impl Strategy<Value = (Curve3, Interval)> {
    prop_oneof![
        (point3(3.0), point3(1.0), range(6.0)).prop_filter_map("direction", |(o, d, r)| {
            Line3::new(o, d).ok().map(|l| (Curve3::Line(l), r))
        }),
        (point3(3.0), point3(1.0), 0.2..3.0f64, range(7.0)).prop_filter_map(
            "normal",
            |(c, n, rad, r)| Circle3::from_normal(c, n, rad)
                .ok()
                .map(|c| (Curve3::Circle(c), r))
        ),
    ]
}

/// Asserts that every hit of two space curves lies on both within `tol`
/// with parameters in the ranges.
fn check_hits3(a: &Curve3, ra: Interval, b: &Curve3, rb: Interval, hits: &[CurveCurveHit<Point3>]) {
    for h in hits {
        match *h {
            CurveCurveHit::Point { a: s, b: t, point } => {
                assert!(ra.contains(s) && rb.contains(t), "{h:?}");
                assert!((a.point(s) - point).length() <= 1e-12 * (1.0 + point.length()));
                assert!((b.point(t) - point).length() <= TOL, "{h:?}");
            }
            CurveCurveHit::Overlap { a: oa, b: ob, .. } => {
                assert!(ra.contains(oa.min) && ra.contains(oa.max), "{h:?}");
                assert!(rb.contains_with(ob.min, 1e-12) && rb.contains_with(ob.max, 1e-12));
                for k in 0..=4 {
                    let p = a.point(oa.lerp(f64::from(k) / 4.0));
                    assert!(b.closest_point(p).unwrap().distance <= 2.0 * TOL, "{h:?}");
                }
            }
        }
    }
}

/// A random rigid motion.
fn motion() -> impl Strategy<Value = DMat4> {
    (rotation(), point3(50.0)).prop_map(|(q, t)| DMat4::from_rotation_translation(q, t))
}

fn count_points_cc(hits: &[CurveCurveHit<Point3>]) -> usize {
    hits.iter()
        .filter(|h| matches!(h, CurveCurveHit::Point { .. }))
        .count()
}

fn count_points_cs(hits: &[CurveSurfaceHit]) -> usize {
    hits.iter()
        .filter(|h| matches!(h, CurveSurfaceHit::Point { .. }))
        .count()
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn plane_hits_lie_on_both_curves((a, ra) in curve2(), (b, rb) in curve2()) {
        let hits = curve_curve_2d(&a, ra, &b, rb, TOL).unwrap();
        check_hits2(&a, ra, &b, rb, &hits, TOL);
        // Exchanging the curves finds the same places.
        let back = curve_curve_2d(&b, rb, &a, ra, TOL).unwrap();
        check_hits2(&b, rb, &a, ra, &back, TOL);
        prop_assert_eq!(hits.len(), back.len());
    }

    #[test]
    fn space_hits_lie_on_both_curves((a, ra) in curve3(), (b, rb) in curve3()) {
        let hits = curve_curve_3d(&a, ra, &b, rb, TOL).unwrap();
        check_hits3(&a, ra, &b, rb, &hits);
    }

    #[test]
    fn fast_and_general_paths_agree(
        o in point2(3.0),
        dir in -3.2..3.2f64,
        c in point2(2.0),
        rad in 0.3..3.0f64,
    ) {
        let l = Line2::new(o, Vector2::from_angle(dir)).unwrap();
        let foot = l.closest_point(c).unwrap().distance;
        // Away from tangency, where the tolerance band could go either way.
        prop_assume!((foot - rad).abs() > 1e-3);
        let a = Curve2::Line(l);
        let b = Curve2::Circle(Circle2::new(c, rad).unwrap());
        let (ra, rb) = (Interval::new(-12.0, 12.0), Interval::new(0.0, TAU));
        let direct = curve_curve_2d(&a, ra, &b, rb, TOL).unwrap();
        let (na, nra) = nurbs2(&a, ra);
        let (nb, nrb) = nurbs2(&b, rb);
        let general = curve_curve_2d(&na, nra, &nb, nrb, TOL).unwrap();
        assert_same_spots(&spots2(&na, &general), &spots2(&a, &direct), 1e-6);
    }

    #[test]
    fn rigid_motions_keep_the_hits(m in motion()) {
        // A line through a torus (4 points), two linked circles (2) and a
        // line through a sphere (2).
        let torus = Surface::Torus(Torus::new(Frame3::WORLD, 3.0, 1.0).unwrap());
        let across = Curve3::Line(Line3::new(Point3::new(-6.0, 0.4, 0.2), Vector3::X).unwrap());
        let full = Rect::new(Interval::new(0.0, TAU), Interval::new(0.0, TAU));
        let sphere = Surface::Sphere(Sphere::new(Frame3::WORLD, 2.0).unwrap());
        let poles = Rect::new(Interval::new(0.0, TAU), Interval::new(-FRAC_PI_2, FRAC_PI_2));
        let c1 = Curve3::Circle(Circle3::from_normal(Point3::ZERO, Vector3::Z, 1.0).unwrap());
        let c2 = Curve3::Circle(
            Circle3::from_normal(Point3::new(0.0, 0.0, 0.5), Vector3::Y, 1.25_f64.sqrt()).unwrap(),
        );
        let turn = Interval::new(0.0, TAU);
        let line_range = Interval::new(0.0, 12.0);

        let before = (
            count_points_cs(&curve_surface(&across, line_range, &torus, full, TOL).unwrap()),
            count_points_cc(&curve_curve_3d(&c1, turn, &c2, turn, TOL).unwrap()),
            count_points_cs(&curve_surface(&across, line_range, &sphere, poles, TOL).unwrap()),
        );
        prop_assert_eq!(before, (4, 2, 2));

        let moved = |c: &Curve3| c.transformed(&m).unwrap();
        let moved_s = |s: &Surface| s.transformed(&m).unwrap();
        let (across, c1, c2) = (moved(&across), moved(&c1), moved(&c2));
        let (torus, sphere) = (moved_s(&torus), moved_s(&sphere));
        let hits = curve_surface(&across, line_range, &torus, full, TOL).unwrap();
        let after = (
            count_points_cs(&hits),
            count_points_cc(&curve_curve_3d(&c1, turn, &c2, turn, TOL).unwrap()),
            count_points_cs(&curve_surface(&across, line_range, &sphere, poles, TOL).unwrap()),
        );
        prop_assert_eq!(after, before);
        for h in &hits {
            if let CurveSurfaceHit::Point { t, u, v, .. } = *h {
                let gap: DVec3 = across.point(t) - torus.point(u, v);
                prop_assert!(gap.length() <= TOL);
            }
        }
    }
}
