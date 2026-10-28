// SPDX-License-Identifier: GPL-3.0-or-later

//! Curve–curve intersection in the plane and in space.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use bevel_geom::intersect::{CurveCurveHit, curve_curve_2d, curve_curve_3d};
use bevel_geom::{
    Circle2, Circle3, Curve2, Curve3, CurveGeom, GeomError, Line2, Line3, NurbsCurve2, NurbsCurve3,
};
use bevel_math::{Frame3, Interval, Point2, Point3, Vector2, Vector3};

use crate::helpers::{
    Spot, TOL, assert_same_spots, check_hits2, nurbs2, nurbs3, points2, spots2, spots3,
};

fn iv(a: f64, b: f64) -> Interval {
    Interval::new(a, b)
}

fn line2(o: (f64, f64), d: (f64, f64)) -> Curve2 {
    Curve2::Line(Line2::new(Point2::new(o.0, o.1), Vector2::new(d.0, d.1)).unwrap())
}

fn circle2(c: (f64, f64), r: f64) -> Curve2 {
    Curve2::Circle(Circle2::new(Point2::new(c.0, c.1), r).unwrap())
}

fn line3(o: Point3, d: Vector3) -> Curve3 {
    Curve3::Line(Line3::new(o, d).unwrap())
}

fn circle3(c: Point3, n: Vector3, r: f64) -> Curve3 {
    Curve3::Circle(Circle3::from_normal(c, n, r).unwrap())
}

/// Intersects the pair directly (fast path) and as NURBS (general method),
/// checks both results and that they agree in space; returns the direct
/// hits.
fn cross_check_2d(
    a: &Curve2,
    ra: Interval,
    b: &Curve2,
    rb: Interval,
) -> Vec<CurveCurveHit<Point2>> {
    let direct = curve_curve_2d(a, ra, b, rb, TOL).unwrap();
    check_hits2(a, ra, b, rb, &direct, TOL);
    let (na, nra) = nurbs2(a, ra);
    let (nb, nrb) = nurbs2(b, rb);
    let general = curve_curve_2d(&na, nra, &nb, nrb, TOL).unwrap();
    check_hits2(&na, nra, &nb, nrb, &general, TOL);
    assert_same_spots(&spots2(&na, &general), &spots2(a, &direct), 1e-5);
    direct
}

/// As [`cross_check_2d`] in space.
fn cross_check_3d(
    a: &Curve3,
    ra: Interval,
    b: &Curve3,
    rb: Interval,
) -> Vec<CurveCurveHit<Point3>> {
    let direct = curve_curve_3d(a, ra, b, rb, TOL).unwrap();
    for h in &direct {
        if let CurveCurveHit::Point { a: s, b: t, point } = *h {
            assert!(ra.contains(s) && rb.contains(t));
            assert!((b.point(t) - point).length() <= TOL);
        }
    }
    let (na, nra) = nurbs3(a, ra);
    let (nb, nrb) = nurbs3(b, rb);
    let general = curve_curve_3d(&na, nra, &nb, nrb, TOL).unwrap();
    assert_same_spots(&spots3(&na, &general), &spots3(a, &direct), 1e-5);
    direct
}

fn single_point<P: Copy>(hits: &[CurveCurveHit<P>]) -> (f64, f64, P) {
    match hits {
        [CurveCurveHit::Point { a, b, point }] => (*a, *b, *point),
        _ => panic!("expected one point"),
    }
}

#[test]
fn crossing_lines_meet_once() {
    let a = line2((0.0, 0.0), (1.0, 0.0));
    let b = line2((5.0, -5.0), (0.0, 1.0));
    let hits = cross_check_2d(&a, iv(-10.0, 10.0), &b, iv(-10.0, 10.0));
    let (s, t, p) = single_point(&hits);
    assert!((s - 5.0).abs() < 1e-14 && (t - 5.0).abs() < 1e-14);
    assert!((p - Point2::new(5.0, 0.0)).length() < 1e-14);
}

#[test]
fn parallel_lines_do_not_meet() {
    let a = line2((0.0, 0.0), (1.0, 0.0));
    let b = line2((0.0, 1e-3), (1.0, 0.0));
    assert!(cross_check_2d(&a, iv(0.0, 10.0), &b, iv(0.0, 10.0)).is_empty());
}

#[test]
fn coincident_lines_overlap() {
    let a = line2((0.0, 0.0), (1.0, 0.0));
    // Same direction: b covers x in [3, 15].
    let b = line2((5.0, 0.0), (1.0, 0.0));
    let hits = cross_check_2d(&a, iv(0.0, 10.0), &b, iv(-2.0, 10.0));
    let [
        CurveCurveHit::Overlap {
            a: ra,
            b: rb,
            reversed,
        },
    ] = hits[..]
    else {
        panic!("{hits:?}");
    };
    assert!(!reversed);
    assert!((ra.min - 3.0).abs() < 1e-14 && (ra.max - 10.0).abs() < 1e-14);
    assert!((rb.min + 2.0).abs() < 1e-14 && (rb.max - 5.0).abs() < 1e-14);
    // Opposite directions: b covers x in [-5, 7]; b.min matches a.max.
    let b = line2((5.0, 0.0), (-1.0, 0.0));
    let hits = cross_check_2d(&a, iv(0.0, 10.0), &b, iv(-2.0, 10.0));
    let [
        CurveCurveHit::Overlap {
            a: ra,
            b: rb,
            reversed,
        },
    ] = hits[..]
    else {
        panic!("{hits:?}");
    };
    assert!(reversed);
    assert!(ra.min.abs() < 1e-14 && (ra.max - 7.0).abs() < 1e-14);
    assert!((rb.min + 2.0).abs() < 1e-14 && (rb.max - 5.0).abs() < 1e-14);
    assert!((a.point(ra.max) - b.point(rb.min)).length() < 1e-12);
}

#[test]
fn general_overlap_of_nurbs_lines_reports_direction() {
    let a = line2((0.0, 0.0), (1.0, 1.0));
    let b = line2((10.0, 10.0), (-1.0, -1.0));
    let (na, ra) = nurbs2(&a, iv(0.0, 10.0));
    let (nb, rb) = nurbs2(&b, iv(2.0, 12.0));
    let hits = curve_curve_2d(&na, ra, &nb, rb, TOL).unwrap();
    let [
        CurveCurveHit::Overlap {
            a: oa,
            b: ob,
            reversed,
        },
    ] = hits[..]
    else {
        panic!("{hits:?}");
    };
    assert!(reversed);
    // b over [2, 12] runs back from (10, 10) to x = y = 10 - 12 / sqrt(2),
    // which a reaches at 10 sqrt(2) - 12; a ends first.
    let start = 10.0 * 2.0_f64.sqrt() - 12.0;
    assert!(
        (oa.min - start).abs() < 1e-9 && (oa.max - 10.0).abs() < 1e-12,
        "{oa:?}"
    );
    assert!((ob.max - 12.0).abs() < 1e-9, "{ob:?}");
    assert!((na.point(oa.max) - nb.point(ob.min)).length() < 1e-6);
    assert!((na.point(oa.min) - nb.point(ob.max)).length() < 1e-6);
}

#[test]
fn touching_ranges_of_collinear_lines_give_a_point() {
    let a = line2((0.0, 0.0), (1.0, 0.0));
    let b = line2((0.0, 0.0), (1.0, 0.0));
    let hits = cross_check_2d(&a, iv(0.0, 5.0), &b, iv(5.0, 9.0));
    let (s, t, _) = single_point(&hits);
    assert!((s - 5.0).abs() < 1e-12 && (t - 5.0).abs() < 1e-12);
}

#[test]
fn line_tangent_to_circle_touches_once() {
    let a = line2((-5.0, 2.0), (1.0, 0.0));
    let b = circle2((0.0, 0.0), 2.0);
    let hits = cross_check_2d(&a, iv(0.0, 10.0), &b, iv(0.0, TAU));
    let (s, t, p) = single_point(&hits);
    assert!((s - 5.0).abs() < 1e-9 && (t - FRAC_PI_2).abs() < 1e-9);
    assert!((p - Point2::new(0.0, 2.0)).length() < 1e-9);
}

#[test]
fn line_through_circle_meets_twice() {
    let a = line2((-5.0, 1.0), (1.0, 0.0));
    let b = circle2((0.0, 0.0), 2.0);
    let hits = cross_check_2d(&a, iv(0.0, 10.0), &b, iv(0.0, TAU));
    let pts = points2(&hits);
    assert_eq!(pts.len(), 2);
    let x = 3.0_f64.sqrt();
    assert!((pts[0] - Point2::new(-x, 1.0)).length() < 1e-12);
    assert!((pts[1] - Point2::new(x, 1.0)).length() < 1e-12);
}

#[test]
fn circles_cross_twice_and_touch_once() {
    let a = circle2((0.0, 0.0), 2.0);
    let b = circle2((3.0, 0.0), 2.0);
    let hits = cross_check_2d(&a, iv(0.0, TAU), &b, iv(0.0, TAU));
    let pts = points2(&hits);
    assert_eq!(pts.len(), 2);
    let y = (4.0_f64 - 2.25).sqrt();
    assert!(
        pts.iter()
            .any(|p| (*p - Point2::new(1.5, y)).length() < 1e-12)
    );
    assert!(
        pts.iter()
            .any(|p| (*p - Point2::new(1.5, -y)).length() < 1e-12)
    );
    // Touching from outside and from inside.
    for b in [circle2((4.0, 0.0), 2.0), circle2((1.0, 0.0), 1.0)] {
        let hits = cross_check_2d(&a, iv(-PI, PI), &b, iv(-PI, PI));
        let (_, _, p) = single_point(&hits);
        assert!((p - Point2::new(2.0, 0.0)).length() < 1e-9, "{p:?}");
    }
}

#[test]
fn coincident_circles_overlap_with_direction() {
    let a = circle2((1.0, 1.0), 2.0);
    let b = Curve2::Circle(
        Circle2::from_axis(Point2::new(1.0, 1.0), Vector2::new(0.0, 1.0), false, 2.0).unwrap(),
    );
    // b's angle is pi / 2 - a's: over [-pi, pi] the match does not wrap.
    let rb = iv(-PI, PI);
    let hits = curve_curve_2d(&a, iv(0.0, PI), &b, rb, TOL).unwrap();
    let [
        CurveCurveHit::Overlap {
            a: oa,
            b: ob,
            reversed,
        },
    ] = hits[..]
    else {
        panic!("{hits:?}");
    };
    assert!(reversed);
    assert!(oa.min.abs() < 1e-12 && (oa.max - PI).abs() < 1e-12);
    assert!((a.point(oa.min) - b.point(ob.max)).length() < 1e-12);
    assert!((a.point(oa.max) - b.point(ob.min)).length() < 1e-12);
    // The general method agrees on the same arcs.
    let (na, ra) = nurbs2(&a, iv(0.0, PI));
    let (nb, nrb) = nurbs2(&b, rb);
    let general = curve_curve_2d(&na, ra, &nb, nrb, TOL).unwrap();
    assert_same_spots(&spots2(&na, &general), &spots2(&a, &hits), 1e-5);
    let [CurveCurveHit::Overlap { reversed, .. }] = general[..] else {
        panic!("{general:?}");
    };
    assert!(reversed);
    // Over b's whole turn from 0 the match wraps across b's seam: two
    // overlaps, meeting at a's pi / 2.
    let hits = curve_curve_2d(&a, iv(0.0, PI), &b, iv(0.0, TAU), TOL).unwrap();
    let [
        CurveCurveHit::Overlap { a: first, .. },
        CurveCurveHit::Overlap { a: second, .. },
    ] = hits[..]
    else {
        panic!("{hits:?}");
    };
    assert!((first.max - FRAC_PI_2).abs() < 1e-12 && (second.min - FRAC_PI_2).abs() < 1e-12);
}

#[test]
fn ranges_beyond_a_period_count_every_pass() {
    let a = line2((-5.0, 1.0), (1.0, 0.0));
    let b = circle2((0.0, 0.0), 2.0);
    let hits = curve_curve_2d(&a, iv(0.0, 10.0), &b, iv(0.0, 2.0 * TAU), TOL).unwrap();
    assert_eq!(hits.len(), 4);
    check_hits2(&a, iv(0.0, 10.0), &b, iv(0.0, 2.0 * TAU), &hits, TOL);
}

/// Crossings of two plane polylines through dense samples, for counting.
fn polyline_crossings(a: &Curve2, ra: Interval, b: &Curve2, rb: Interval) -> Vec<Point2> {
    let n = 2000;
    let sample = |c: &Curve2, r: Interval| -> Vec<Point2> {
        (0..=n)
            .map(|k| c.point(r.lerp(f64::from(k) / f64::from(n))))
            .collect()
    };
    let (pa, pb) = (sample(a, ra), sample(b, rb));
    let mut out = Vec::new();
    for w in pa.windows(2) {
        for v in pb.windows(2) {
            let (p, r) = (w[0], w[1] - w[0]);
            let (q, s) = (v[0], v[1] - v[0]);
            let den = r.perp_dot(s);
            if den.abs() < 1e-300 {
                continue;
            }
            let t = (q - p).perp_dot(s) / den;
            let u = (q - p).perp_dot(r) / den;
            if (0.0..1.0).contains(&t) && (0.0..1.0).contains(&u) {
                out.push(p + r * t);
            }
        }
    }
    out
}

#[test]
fn cubic_beziers_cross_several_times() {
    let a = Curve2::Nurbs(
        NurbsCurve2::bezier(&[
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 3.0),
            Point2::new(2.0, -3.0),
            Point2::new(3.0, 0.0),
        ])
        .unwrap(),
    );
    let b = Curve2::Nurbs(
        NurbsCurve2::bezier(&[
            Point2::new(0.0, 0.3),
            Point2::new(1.0, -2.0),
            Point2::new(2.0, 2.0),
            Point2::new(3.0, -0.3),
        ])
        .unwrap(),
    );
    let r = iv(0.0, 1.0);
    let hits = curve_curve_2d(&a, r, &b, r, TOL).unwrap();
    check_hits2(&a, r, &b, r, &hits, TOL);
    let want = polyline_crossings(&a, r, &b, r);
    assert!(want.len() >= 3, "{want:?}");
    let pts = points2(&hits);
    assert_eq!(pts.len(), want.len(), "{pts:?} vs {want:?}");
    for (p, q) in pts.iter().zip(&want) {
        assert!((*p - *q).length() < 1e-4);
    }
    for h in &hits {
        let CurveCurveHit::Point { a: s, b: t, .. } = *h else {
            panic!("{h:?}");
        };
        // Refined to full precision.
        assert!((a.point(s) - b.point(t)).length() < 1e-13);
    }
}

#[test]
fn circle_meets_fitted_curve() {
    let pts: Vec<Point2> = (0..=8)
        .map(|k| {
            let x = -3.0 + 0.75 * f64::from(k);
            Point2::new(x, 0.3 * x * x - 1.0)
        })
        .collect();
    let fitted =
        bevel_geom::fit::interpolate(&pts, 3, bevel_geom::fit::Parametrization::Centripetal)
            .unwrap();
    let a = circle2((0.2, 0.1), 1.5);
    let b = Curve2::Nurbs(fitted);
    let (ra, rb) = (iv(0.0, TAU), iv(0.0, 1.0));
    let hits = curve_curve_2d(&a, ra, &b, rb, TOL).unwrap();
    check_hits2(&a, ra, &b, rb, &hits, TOL);
    let want = polyline_crossings(&a, ra, &b, rb);
    assert_eq!(points2(&hits).len(), want.len(), "{hits:?} vs {want:?}");
    // The same with the circle as a NURBS.
    let (na, nra) = nurbs2(&a, ra);
    let general = curve_curve_2d(&na, nra, &b, rb, TOL).unwrap();
    assert_same_spots(&spots2(&na, &general), &spots2(&a, &hits), 1e-6);
}

#[test]
fn hits_at_range_ends_count() {
    let a = line2((0.0, 0.0), (1.0, 0.0));
    let b = line2((5.0, -1.0), (0.0, 1.0));
    // Exactly at the end of a's range.
    let hits = cross_check_2d(&a, iv(0.0, 5.0), &b, iv(-2.0, 2.0));
    let (s, t, _) = single_point(&hits);
    assert!((s - 5.0).abs() < 1e-14 && (t - 1.0).abs() < 1e-14);
    // Just short of it, but within the tolerance.
    let hits = cross_check_2d(&a, iv(0.0, 5.0 - 0.5 * TOL), &b, iv(-2.0, 2.0));
    let (s, _, _) = single_point(&hits);
    assert!((s - (5.0 - 0.5 * TOL)).abs() < 1e-14);
    // Too short.
    assert!(cross_check_2d(&a, iv(0.0, 4.999), &b, iv(-2.0, 2.0)).is_empty());
    // At the ends of both ranges, on circles.
    let c = circle2((0.0, 0.0), 1.0);
    let d = circle2((2.0, 0.0), 1.0);
    let hits = cross_check_2d(&c, iv(-1.0, 0.0), &d, iv(PI, 4.0));
    let (s, t, _) = single_point(&hits);
    assert!(s.abs() < 1e-7 && (t - PI).abs() < 1e-7);
}

#[test]
fn pieces_apart_do_not_meet() {
    let a = circle2((0.0, 0.0), 1.0);
    let b = circle2((5.0, 0.0), 1.0);
    assert!(cross_check_2d(&a, iv(0.0, TAU), &b, iv(0.0, TAU)).is_empty());
    let l = line2((-5.0, 3.0), (1.0, 0.1));
    assert!(cross_check_2d(&l, iv(0.0, 10.0), &a, iv(0.0, TAU)).is_empty());
    // The circle meets the line only outside its range.
    let l = line2((-5.0, 0.0), (1.0, 0.0));
    assert!(cross_check_2d(&l, iv(0.0, 3.0), &a, iv(0.0, TAU)).is_empty());
    assert!(cross_check_2d(&l, iv(0.0, 10.0), &a, iv(0.5, 1.5)).is_empty());
}

#[test]
fn bad_input_is_rejected() {
    let a = line2((0.0, 0.0), (1.0, 0.0));
    let b = line2((0.0, 0.0), (0.0, 1.0));
    let r = iv(-1.0, 1.0);
    assert_eq!(
        curve_curve_2d(&a, r, &b, r, f64::NAN),
        Err(GeomError::NonFinite)
    );
    assert_eq!(
        curve_curve_2d(&a, r, &b, r, f64::INFINITY),
        Err(GeomError::NonFinite)
    );
    assert!(matches!(
        curve_curve_2d(&a, r, &b, r, 0.0),
        Err(GeomError::InvalidArgument(_))
    ));
    assert!(matches!(
        curve_curve_2d(&a, r, &b, r, -1.0),
        Err(GeomError::InvalidArgument(_))
    ));
    assert_eq!(
        curve_curve_2d(&a, iv(f64::NAN, 1.0), &b, r, TOL),
        Err(GeomError::NonFinite)
    );
    assert!(matches!(
        curve_curve_2d(&a, iv(1.0, 1.0), &b, r, TOL),
        Err(GeomError::InvalidArgument(_))
    ));
    let (n, _) = nurbs2(&a, r);
    assert!(matches!(
        curve_curve_2d(&n, iv(0.0, 5.0), &b, r, TOL),
        Err(GeomError::ParameterOutOfDomain { .. })
    ));
    let a3 = line3(Point3::ZERO, Vector3::X);
    assert_eq!(
        curve_curve_3d(&a3, r, &a3, r, f64::NAN),
        Err(GeomError::NonFinite)
    );
}

#[test]
fn lines_in_space() {
    // Crossing.
    let a = line3(Point3::new(0.0, 0.0, 1.0), Vector3::X);
    let b = line3(Point3::new(2.0, -1.0, 1.0), Vector3::Y);
    let hits = cross_check_3d(&a, iv(-5.0, 5.0), &b, iv(-5.0, 5.0));
    let (s, t, _) = single_point(&hits);
    assert!((s - 2.0).abs() < 1e-14 && (t - 1.0).abs() < 1e-14);
    // Skew within the tolerance, and well apart.
    let b = line3(Point3::new(2.0, -1.0, 1.0 + 0.5 * TOL), Vector3::Y);
    assert_eq!(
        cross_check_3d(&a, iv(-5.0, 5.0), &b, iv(-5.0, 5.0)).len(),
        1
    );
    let b = line3(Point3::new(2.0, -1.0, 1.1), Vector3::Y);
    assert!(cross_check_3d(&a, iv(-5.0, 5.0), &b, iv(-5.0, 5.0)).is_empty());
    // Parallel and coincident, running against each other.
    let b = line3(Point3::new(1.0, 0.0, 1.0), -Vector3::X);
    let hits = cross_check_3d(&a, iv(0.0, 5.0), &b, iv(-1.0, 3.0));
    let [
        CurveCurveHit::Overlap {
            a: oa,
            b: ob,
            reversed,
        },
    ] = hits[..]
    else {
        panic!("{hits:?}");
    };
    assert!(reversed && oa.min.abs() < 1e-14 && (oa.max - 2.0).abs() < 1e-14);
    assert!((a.point(oa.max) - b.point(ob.min)).length() < 1e-14);
    let b = line3(Point3::new(1.0, 0.5, 1.0), -Vector3::X);
    assert!(cross_check_3d(&a, iv(0.0, 5.0), &b, iv(-1.0, 3.0)).is_empty());
}

#[test]
fn lines_and_circles_in_space() {
    let frame =
        Frame3::from_normal(Point3::new(1.0, 2.0, 3.0), Vector3::new(1.0, 1.0, 1.0)).unwrap();
    let c = Curve3::Circle(Circle3::new(frame, 2.0).unwrap());
    // In the circle's plane, through it.
    let o = frame.to_world(Point3::new(-5.0, 1.0, 0.0));
    let a = line3(o, frame.x);
    let hits = cross_check_3d(&a, iv(0.0, 10.0), &c, iv(0.0, TAU));
    assert_eq!(hits.len(), 2);
    // Tangent in the plane.
    let o = frame.to_world(Point3::new(-5.0, 2.0, 0.0));
    let a = line3(o, frame.x);
    assert_eq!(cross_check_3d(&a, iv(0.0, 10.0), &c, iv(0.0, TAU)).len(), 1);
    // Piercing the plane on the circle, at an angle.
    let p = frame.to_world(Point3::new(2.0 * 0.6, 2.0 * 0.8, 0.0));
    let a = line3(p - (frame.z + frame.y * 0.3) * 2.0, frame.z + frame.y * 0.3);
    let hits = cross_check_3d(&a, iv(-1.0, 5.0), &c, iv(0.0, TAU));
    let (_, _, q) = single_point(&hits);
    assert!((q - p).length() < 1e-12);
    // Piercing the plane inside the circle.
    let a = line3(frame.origin, frame.z);
    assert!(cross_check_3d(&a, iv(-1.0, 1.0), &c, iv(0.0, TAU)).is_empty());
}

#[test]
fn circles_in_space() {
    let a = circle3(Point3::ZERO, Vector3::Z, 1.0);
    // In the same plane, crossing twice.
    let b = circle3(Point3::new(1.0, 0.0, 0.0), -Vector3::Z, 1.0);
    assert_eq!(cross_check_3d(&a, iv(0.0, TAU), &b, iv(0.0, TAU)).len(), 2);
    // In perpendicular planes: linked through two points, and once.
    let b = circle3(Point3::new(0.0, 0.0, 0.5), Vector3::Y, 1.25_f64.sqrt());
    let hits = cross_check_3d(&a, iv(0.0, TAU), &b, iv(0.0, TAU));
    assert_eq!(hits.len(), 2);
    for h in &hits {
        let CurveCurveHit::Point { point, .. } = *h else {
            panic!("{h:?}");
        };
        assert!((point.x.abs() - 1.0).abs() < 1e-12 && point.y.abs() < 1e-12);
    }
    let b = circle3(Point3::new(1.0, 0.0, 1.0), Vector3::Y, 1.0);
    let hits = cross_check_3d(&a, iv(-PI, PI), &b, iv(0.0, TAU));
    let (_, _, p) = single_point(&hits);
    assert!((p - Point3::X).length() < 1e-12);
    // In parallel planes.
    let b = circle3(Point3::new(0.0, 0.0, 0.1), Vector3::Z, 1.0);
    assert!(cross_check_3d(&a, iv(0.0, TAU), &b, iv(0.0, TAU)).is_empty());
    // The same circle.
    let b = circle3(Point3::ZERO, -Vector3::Z, 1.0);
    let hits = cross_check_3d(&a, iv(0.0, PI), &b, iv(0.0, TAU));
    assert!(matches!(
        hits[..],
        [CurveCurveHit::Overlap { reversed: true, .. }]
    ));
}

#[test]
fn nurbs_curves_in_space() {
    let a = Curve3::Nurbs(
        NurbsCurve3::bezier(&[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 1.0),
            Point3::new(2.0, -2.0, 1.0),
            Point3::new(3.0, 0.0, 0.0),
        ])
        .unwrap(),
    );
    // A twisted cubic and the chord through two of its points, which meets
    // it nowhere else; and a skew line passing 0.5 tol away from a point.
    let p = a.point(0.3);
    let q = a.point(0.8);
    let b = line3(p, q - p);
    let rb = iv(-1.0, (q - p).length() + 1.0);
    let hits = curve_curve_3d(&a, iv(0.0, 1.0), &b, rb, TOL).unwrap();
    assert_same_spots(&spots3(&a, &hits), &[Spot::Point(p), Spot::Point(q)], 1e-9);
    let tangent = a.derivatives(0.6, 1)[1];
    let normal = tangent.cross(Vector3::new(0.3, -0.2, 1.0)).normalize();
    let near = a.point(0.6) + normal * (0.5 * TOL);
    let b = line3(near - normal.cross(tangent) * 2.0, normal.cross(tangent));
    let hits = curve_curve_3d(&a, iv(0.0, 1.0), &b, iv(0.0, 10.0), TOL).unwrap();
    let (s, _, _) = single_point(&hits);
    assert!((s - 0.6).abs() < 1e-6);
}
