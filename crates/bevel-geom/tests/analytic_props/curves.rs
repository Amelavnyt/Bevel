// SPDX-License-Identifier: GPL-3.0-or-later

//! Lines, circles and ellipses: closed forms against finite differences,
//! closest points against dense sampling, bounding boxes and reversal.

use std::f64::consts::TAU;

use bevel_geom::{Circle2, Circle3, Ellipse2, Ellipse3, Line2, Line3};
use bevel_math::{Interval, Point2, Point3, Vector2, Vector3};
use proptest::prelude::*;

use crate::strategies::{
    CurveCase, angle, check_bounds, check_curve_closest, check_curve_differentials, config, frame,
    params, point2, point3, radius,
};

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn line3_closed_forms(
        o in point3(50.0),
        d in point3(1.0),
        t in -100.0f64..100.0,
        target in point3(150.0),
    ) {
        prop_assume!(d.length() > 1e-3);
        let l = Line3::new(o, d).unwrap();
        let case = CurveCase {
            point: &|t| l.point(t),
            ders: &|t, n| l.derivatives(t, n),
            tangent: &|t| l.tangent(t),
            curvature: &|t| l.curvature(t),
            closest: &|p| l.closest_point(p).unwrap(),
            scale: 1.0,
        };
        check_curve_differentials(&case, t)?;
        let cp = l.closest_point(target).unwrap();
        check_curve_closest(&case, target, &params(cp.t - 300.0, cp.t + 300.0, 2001))?;
        // The foot of the perpendicular.
        prop_assert!((target - cp.point).dot(l.direction()).abs() <= 1e-12 * 300.0);
        let r = l.reversed();
        prop_assert!((r.point(r.reversed_parameter(t)) - l.point(t)).length() <= 1e-12 * 300.0);
        let b = l.bounds(Interval::new(t, t + 5.0)).unwrap();
        prop_assert!(b.contains(l.point(t)) && b.contains(l.point(t + 5.0)));
        prop_assert!((b.size() - (l.point(t + 5.0) - l.point(t)).abs()).length() < 1e-12 * 300.0);
    }

    #[test]
    fn line2_closed_forms(
        o in point2(50.0),
        d in point2(1.0),
        t in -100.0f64..100.0,
        target in point2(150.0),
    ) {
        prop_assume!(d.length() > 1e-3);
        let l = Line2::new(o, d).unwrap();
        let case = CurveCase {
            point: &|t| l.point(t),
            ders: &|t, n| l.derivatives(t, n),
            tangent: &|t| l.tangent(t),
            curvature: &|t| l.curvature(t),
            closest: &|p| l.closest_point(p).unwrap(),
            scale: 1.0,
        };
        check_curve_differentials(&case, t)?;
        let cp = l.closest_point(target).unwrap();
        check_curve_closest(&case, target, &params(cp.t - 300.0, cp.t + 300.0, 2001))?;
        let r = l.reversed();
        prop_assert!((r.point(r.reversed_parameter(t)) - l.point(t)).length() <= 1e-12 * 300.0);
    }

    #[test]
    fn circle3_closed_forms(
        f in frame(),
        r in radius(),
        t in angle(),
        target in point3(100.0),
        range in (angle(), 0.01f64..8.0),
    ) {
        let c = Circle3::new(f, r).unwrap();
        let case = CurveCase {
            point: &|t| c.point(t),
            ders: &|t, n| c.derivatives(t, n),
            tangent: &|t| c.tangent(t),
            curvature: &|t| c.curvature(t),
            closest: &|p| c.closest_point(p).unwrap(),
            scale: r,
        };
        check_curve_differentials(&case, t)?;
        prop_assert!((c.curvature(t).unwrap() - 1.0 / r).abs() <= 1e-12 / r);
        check_curve_closest(&case, target, &params(0.0, TAU, 4001))?;
        // A point on the circle is its own closest point.
        let cp = c.closest_point(c.point(t)).unwrap();
        prop_assert!(cp.distance <= 1e-12 * 100.0 && (0.0..TAU).contains(&cp.t));
        let rev = c.reversed();
        prop_assert!((rev.point(rev.reversed_parameter(t)) - c.point(t)).length() <= 1e-12 * 100.0);
        let range = Interval::new(range.0, range.0 + range.1);
        let b = c.bounds(range).unwrap();
        let pts: Vec<Point3> = params(range.min, range.max, 4001).into_iter().map(|t| c.point(t)).collect();
        check_bounds(b.min, b.max, &pts, 1e-5 * r)?;
    }

    #[test]
    fn circle2_closed_forms(
        center in point2(50.0),
        axis in point2(1.0),
        ccw in any::<bool>(),
        r in radius(),
        t in angle(),
        target in point2(100.0),
        range in (angle(), 0.01f64..8.0),
    ) {
        prop_assume!(axis.length() > 1e-3);
        let c = Circle2::from_axis(center, axis, ccw, r).unwrap();
        let case = CurveCase {
            point: &|t| c.point(t),
            ders: &|t, n| c.derivatives(t, n),
            tangent: &|t| c.tangent(t),
            curvature: &|t| c.curvature(t),
            closest: &|p| c.closest_point(p).unwrap(),
            scale: r,
        };
        check_curve_differentials(&case, t)?;
        check_curve_closest(&case, target, &params(0.0, TAU, 4001))?;
        let rev = c.reversed();
        prop_assert!(rev.is_counter_clockwise() != c.is_counter_clockwise());
        prop_assert!((rev.point(rev.reversed_parameter(t)) - c.point(t)).length() <= 1e-12 * 100.0);
        let range = Interval::new(range.0, range.0 + range.1);
        let b = c.bounds(range).unwrap();
        let pts: Vec<Point2> = params(range.min, range.max, 4001).into_iter().map(|t| c.point(t)).collect();
        check_bounds(b.min, b.max, &pts, 1e-5 * r)?;
    }

    #[test]
    fn ellipse3_closed_forms(
        f in frame(),
        a in radius(),
        ratio in 0.05f64..=1.0,
        t in angle(),
        target in point3(100.0),
        range in (angle(), 0.01f64..8.0),
    ) {
        let e = Ellipse3::new(f, a, a * ratio).unwrap();
        let case = CurveCase {
            point: &|t| e.point(t),
            ders: &|t, n| e.derivatives(t, n),
            tangent: &|t| e.tangent(t),
            curvature: &|t| e.curvature(t),
            closest: &|p| e.closest_point(p).unwrap(),
            scale: a,
        };
        check_curve_differentials(&case, t)?;
        check_curve_closest(&case, target, &params(0.0, TAU, 8001))?;
        let cp = e.closest_point(e.point(t)).unwrap();
        prop_assert!(cp.distance <= 1e-12 * 100.0);
        let rev = e.reversed();
        prop_assert!((rev.point(rev.reversed_parameter(t)) - e.point(t)).length() <= 1e-12 * 100.0);
        let range = Interval::new(range.0, range.0 + range.1);
        let b = e.bounds(range).unwrap();
        let pts: Vec<Point3> = params(range.min, range.max, 4001).into_iter().map(|t| e.point(t)).collect();
        check_bounds(b.min, b.max, &pts, 1e-5 * a)?;
    }

    #[test]
    fn ellipse2_closed_forms(
        center in point2(50.0),
        axis in point2(1.0),
        ccw in any::<bool>(),
        a in radius(),
        ratio in 0.05f64..=1.0,
        t in angle(),
        target in point2(100.0),
        range in (angle(), 0.01f64..8.0),
    ) {
        prop_assume!(axis.length() > 1e-3);
        let e = Ellipse2::from_axis(center, axis, ccw, a, a * ratio).unwrap();
        let case = CurveCase {
            point: &|t| e.point(t),
            ders: &|t, n| e.derivatives(t, n),
            tangent: &|t| e.tangent(t),
            curvature: &|t| e.curvature(t),
            closest: &|p| e.closest_point(p).unwrap(),
            scale: a,
        };
        check_curve_differentials(&case, t)?;
        check_curve_closest(&case, target, &params(0.0, TAU, 8001))?;
        let rev = e.reversed();
        prop_assert!((rev.point(rev.reversed_parameter(t)) - e.point(t)).length() <= 1e-12 * 100.0);
        let range = Interval::new(range.0, range.0 + range.1);
        let b = e.bounds(range).unwrap();
        let pts: Vec<Point2> = params(range.min, range.max, 4001).into_iter().map(|t| e.point(t)).collect();
        check_bounds(b.min, b.max, &pts, 1e-5 * a)?;
    }
}

#[test]
fn closest_points_on_degenerate_targets() {
    // The centre of a circle: every point is closest, t = 0 is returned.
    let c = Circle2::new(Point2::new(1.0, 2.0), 3.0).unwrap();
    let cp = c.closest_point(Point2::new(1.0, 2.0)).unwrap();
    assert!(cp.t.abs() < 1e-300 && (cp.distance - 3.0).abs() < 1e-15);
    // The centre of an ellipse: the end of the minor axis.
    let e = Ellipse2::new(Point2::ZERO, Vector2::new(1.0, 1.0), 2.0, 1.0).unwrap();
    let cp = e.closest_point(Point2::ZERO).unwrap();
    assert!((cp.distance - 1.0).abs() < 1e-15);
    // Points on the ellipse's axes, inside and outside.
    let e = Ellipse3::new(bevel_math::Frame3::WORLD, 5.0, 2.0).unwrap();
    for p in [
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(-7.0, 0.0, 1.0),
        Point3::new(0.0, 0.5, 0.0),
        Point3::new(0.0, -9.0, 0.0),
    ] {
        let cp = e.closest_point(p).unwrap();
        let best = (0..20000)
            .map(|i| (e.point(TAU * f64::from(i) / 20000.0) - p).length())
            .fold(f64::INFINITY, f64::min);
        assert!(
            cp.distance <= best + 1e-12,
            "{p:?}: {} vs {best}",
            cp.distance
        );
    }
    // A line's closest point to a point on it.
    let l = Line3::new(Point3::ZERO, Vector3::new(1.0, 1.0, 0.0)).unwrap();
    let cp = l.closest_point(Point3::new(2.0, 2.0, 0.0)).unwrap();
    assert!((cp.t - 8f64.sqrt()).abs() < 1e-15 && cp.distance < 1e-15);
}
