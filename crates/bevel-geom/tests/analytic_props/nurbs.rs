// SPDX-License-Identifier: GPL-3.0-or-later

//! NURBS forms of the analytic types: the geometry agrees (every NURBS
//! sample lies on the analytic shape, both cover the same range, the end
//! points agree), the parameter maps relate the two, and the NURBS normals
//! and curvatures match the closed forms at corresponding points.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use bevel_geom::{
    Circle2, Circle3, Cone, ConvertedCurve, ConvertedSurface, Curvatures, Cylinder, Ellipse2,
    Ellipse3, GeomError, Homogeneous, Line2, Line3, Plane, Sphere, Torus,
};
use bevel_math::{Frame3, Interval, Point3, Vector3};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{Pt, angle, config, frame, params, point2, point3, radius};

/// An angular range: a random start and a sweep up to a full turn, often
/// exactly one.
fn turn_range() -> impl Strategy<Value = Interval> {
    (angle(), prop_oneof![Just(TAU), 0.05f64..TAU]).prop_map(|(a, l)| Interval::new(a, a + l))
}

/// A range of lengths along an axis.
fn axial_range() -> impl Strategy<Value = Interval> {
    (-50.0f64..50.0, 0.1f64..60.0).prop_map(|(a, l)| Interval::new(a, a + l))
}

/// A latitude range inside `[-pi / 2, pi / 2]`, often the whole of it.
fn latitude_range() -> impl Strategy<Value = Interval> {
    prop_oneof![
        Just(Interval::new(-FRAC_PI_2, FRAC_PI_2)),
        (-FRAC_PI_2..1.4f64, 0.05f64..PI)
            .prop_map(|(a, l)| Interval::new(a, (a + l).min(FRAC_PI_2))),
    ]
}

/// The NURBS form of a curve against the analytic curve: the domain is the
/// range, every NURBS sample is on the curve (`distance` measures that) and
/// is the analytic point at the mapped parameter, every analytic sample is
/// the NURBS point at the mapped parameter, and the end points agree.
fn check_curve_conversion<H: Homogeneous>(
    conv: &ConvertedCurve<H>,
    range: Interval,
    point: &dyn Fn(f64) -> H::Point,
    distance: &dyn Fn(H::Point) -> f64,
    size: f64,
) -> Result<(), TestCaseError>
where
    H::Point: Pt,
{
    let tol = 1e-12 * size;
    let (a, b) = conv.nurbs.domain();
    prop_assert!((a - range.min).abs() <= 1e-12 * range.min.abs().max(1.0));
    prop_assert!((b - range.max).abs() <= 1e-12 * range.max.abs().max(1.0));
    prop_assert!((conv.nurbs.point(a) - point(range.min)).len() <= tol);
    prop_assert!((conv.nurbs.point(b) - point(range.max)).len() <= tol);
    for s in params(a, b, 201) {
        let p = conv.nurbs.point(s);
        prop_assert!(
            distance(p) <= tol,
            "NURBS point at {s} is {} off",
            distance(p)
        );
        let t = conv.map.analytic_parameter(s);
        prop_assert!(range.contains_with(t, 1e-12 * range.max.abs().max(1.0)));
        prop_assert!(
            (point(t) - p).len() <= tol,
            "map at {s}: {:?} vs {:?}",
            point(t),
            p
        );
    }
    for t in params(range.min, range.max, 201) {
        let s = conv.map.nurbs_parameter(t);
        prop_assert!((conv.nurbs.point(s) - point(t)).len() <= tol);
    }
    Ok(())
}

/// The surface counterpart of [`check_curve_conversion`].
fn check_surface_conversion(
    conv: &ConvertedSurface,
    u: Interval,
    v: Interval,
    point: &dyn Fn(f64, f64) -> Point3,
    distance: &dyn Fn(Point3) -> f64,
    size: f64,
) -> Result<(), TestCaseError> {
    let tol = 1e-12 * size;
    let ((ua, ub), (va, vb)) = conv.nurbs.domain();
    for (got, want) in [(ua, u.min), (ub, u.max), (va, v.min), (vb, v.max)] {
        prop_assert!((got - want).abs() <= 1e-12 * want.abs().max(1.0));
    }
    for (s, t, uu, vv) in [
        (ua, va, u.min, v.min),
        (ub, va, u.max, v.min),
        (ua, vb, u.min, v.max),
        (ub, vb, u.max, v.max),
    ] {
        prop_assert!((conv.nurbs.point(s, t) - point(uu, vv)).length() <= tol);
    }
    for s in params(ua, ub, 31) {
        for t in params(va, vb, 31) {
            let p = conv.nurbs.point(s, t);
            prop_assert!(
                distance(p) <= tol,
                "NURBS point at ({s}, {t}) is {} off",
                distance(p)
            );
            let (uu, vv) = (
                conv.map_u.analytic_parameter(s),
                conv.map_v.analytic_parameter(t),
            );
            prop_assert!((point(uu, vv) - p).length() <= tol);
        }
    }
    for uu in params(u.min, u.max, 31) {
        for vv in params(v.min, v.max, 31) {
            let (s, t) = (
                conv.map_u.nurbs_parameter(uu),
                conv.map_v.nurbs_parameter(vv),
            );
            prop_assert!((conv.nurbs.point(s, t) - point(uu, vv)).length() <= tol);
        }
    }
    Ok(())
}

/// The closed-form normal and curvatures at `(u, v)` against those of the
/// NURBS form at the point found by its closest-point search.
fn check_against_nurbs(
    conv: &ConvertedSurface,
    u: f64,
    v: f64,
    point: &dyn Fn(f64, f64) -> Point3,
    normal: Vector3,
    k: Curvatures,
) -> Result<(), TestCaseError> {
    let p = point(u, v);
    let cp = conv.nurbs.closest_point(p).unwrap();
    prop_assert!(cp.distance <= 1e-9 * p.length().max(1.0), "{cp:?}");
    let n = conv.nurbs.normal(cp.u, cp.v).unwrap();
    prop_assert!((n - normal).length() <= 1e-8, "normal {n:?} vs {normal:?}");
    let kn = conv.nurbs.principal_curvatures(cp.u, cp.v).unwrap();
    let scale = k.k1.abs().max(k.k2.abs()).max(1e-2);
    prop_assert!(
        (kn.k1 - k.k1).abs() <= 1e-9 * scale && (kn.k2 - k.k2).abs() <= 1e-9 * scale,
        "NURBS {kn:?} vs closed form {k:?}"
    );
    if k.k1 - k.k2 > 1e-3 * scale {
        prop_assert!(kn.dir1.cross(k.dir1).length() <= 1e-6, "{kn:?} vs {k:?}");
    }
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn lines_convert_exactly(o in point3(50.0), d in point3(1.0), o2 in point2(50.0), range in axial_range()) {
        prop_assume!(d.length() > 1e-3);
        let l = Line3::new(o, d).unwrap();
        let conv = l.to_nurbs_mapped(range).unwrap();
        check_curve_conversion(&conv, range, &|t| l.point(t), &|p| l.closest_point(p).unwrap().distance, 200.0)?;
        let l2 = Line2::new(o2, d.truncate() + d.z).unwrap();
        let conv = l2.to_nurbs_mapped(range).unwrap();
        check_curve_conversion(&conv, range, &|t| l2.point(t), &|p| l2.closest_point(p).unwrap().distance, 200.0)?;
    }

    #[test]
    fn circles_and_ellipses_convert_exactly(
        f in frame(),
        r in radius(),
        ratio in 0.05f64..=1.0,
        c2 in point2(50.0),
        ccw in any::<bool>(),
        range in turn_range(),
    ) {
        let size = 50.0 * 3f64.sqrt() + r;
        let c = Circle3::new(f, r).unwrap();
        let conv = c.to_nurbs_mapped(range).unwrap();
        check_curve_conversion(&conv, range, &|t| c.point(t), &|p| c.closest_point(p).unwrap().distance, size)?;
        let e = Ellipse3::new(f, r, r * ratio).unwrap();
        let conv = e.to_nurbs_mapped(range).unwrap();
        check_curve_conversion(&conv, range, &|t| e.point(t), &|p| e.closest_point(p).unwrap().distance, size)?;
        let c = Circle2::from_axis(c2, f.x.truncate() + f.z.truncate(), ccw, r).unwrap();
        let conv = c.to_nurbs_mapped(range).unwrap();
        check_curve_conversion(&conv, range, &|t| c.point(t), &|p| c.closest_point(p).unwrap().distance, size)?;
        let e = Ellipse2::from_axis(c2, f.y.truncate() + f.z.truncate(), ccw, r, r * ratio).unwrap();
        let conv = e.to_nurbs_mapped(range).unwrap();
        check_curve_conversion(&conv, range, &|t| e.point(t), &|p| e.closest_point(p).unwrap().distance, size)?;
    }

    #[test]
    fn planes_convert_exactly(f in frame(), u in axial_range(), v in axial_range()) {
        let s = Plane::new(f).unwrap();
        let conv = s.to_nurbs_mapped(u, v).unwrap();
        check_surface_conversion(&conv, u, v, &|u, v| s.point(u, v), &|p| s.closest_point(p).unwrap().distance, 300.0)?;
        let (uu, vv) = (u.mid(), v.lerp(0.3));
        check_against_nurbs(&conv, uu, vv, &|u, v| s.point(u, v), s.normal(uu, vv).unwrap(), s.principal_curvatures(uu, vv).unwrap())?;
    }

    #[test]
    fn cylinders_convert_exactly(f in frame(), r in radius(), u in turn_range(), v in axial_range(), fr in prop::array::uniform2(0.1f64..0.9)) {
        let s = Cylinder::new(f, r).unwrap();
        let conv = s.to_nurbs_mapped(u, v).unwrap();
        let size = 200.0 + r;
        check_surface_conversion(&conv, u, v, &|u, v| s.point(u, v), &|p| s.closest_point(p).unwrap().distance, size)?;
        let (uu, vv) = (u.lerp(fr[0]), v.lerp(fr[1]));
        check_against_nurbs(&conv, uu, vv, &|u, v| s.point(u, v), s.normal(uu, vv).unwrap(), s.principal_curvatures(uu, vv).unwrap())?;
    }

    #[test]
    fn cones_convert_exactly(
        f in frame(),
        r in 0.0f64..40.0,
        alpha in 0.05f64..1.5,
        u in turn_range(),
        v in axial_range(),
        fr in prop::array::uniform2(0.1f64..0.9),
    ) {
        let s = Cone::new(f, r, alpha).unwrap();
        let conv = s.to_nurbs_mapped(u, v).unwrap();
        let size = 200.0 + s.radius_at(v.min).abs().max(s.radius_at(v.max).abs());
        check_surface_conversion(&conv, u, v, &|u, v| s.point(u, v), &|p| s.closest_point(p).unwrap().distance, size)?;
        let (uu, vv) = (u.lerp(fr[0]), v.lerp(fr[1]));
        prop_assume!(s.radius_at(vv).abs() > 0.05);
        check_against_nurbs(&conv, uu, vv, &|u, v| s.point(u, v), s.normal(uu, vv).unwrap(), s.principal_curvatures(uu, vv).unwrap())?;
    }

    #[test]
    fn spheres_convert_exactly(f in frame(), r in radius(), u in turn_range(), v in latitude_range(), fr in prop::array::uniform2(0.1f64..0.9)) {
        let s = Sphere::new(f, r).unwrap();
        let conv = s.to_nurbs_mapped(u, v).unwrap();
        let size = 100.0 + r;
        check_surface_conversion(&conv, u, v, &|u, v| s.point(u, v), &|p| s.closest_point(p).unwrap().distance, size)?;
        let (uu, vv) = (u.lerp(fr[0]), v.lerp(fr[1]));
        prop_assume!(vv.cos() > 0.05);
        check_against_nurbs(&conv, uu, vv, &|u, v| s.point(u, v), s.normal(uu, vv).unwrap(), s.principal_curvatures(uu, vv).unwrap())?;
    }

    #[test]
    fn tori_convert_exactly(
        f in frame(),
        big in radius(),
        ratio in 0.05f64..0.9,
        u in turn_range(),
        v in turn_range(),
        fr in prop::array::uniform2(0.1f64..0.9),
    ) {
        let s = Torus::new(f, big, big * ratio).unwrap();
        let conv = s.to_nurbs_mapped(u, v).unwrap();
        let size = 100.0 + 2.0 * big;
        check_surface_conversion(&conv, u, v, &|u, v| s.point(u, v), &|p| s.closest_point(p).unwrap().distance, size)?;
        let (uu, vv) = (u.lerp(fr[0]), v.lerp(fr[1]));
        check_against_nurbs(&conv, uu, vv, &|u, v| s.point(u, v), s.normal(uu, vv).unwrap(), s.principal_curvatures(uu, vv).unwrap())?;
    }
}

#[test]
fn full_conversions_are_closed() {
    let f = Frame3 {
        origin: Point3::new(3.0, -2.0, 1.0),
        ..Frame3::WORLD
    };
    let c = Circle3::new(f, 2.0).unwrap().to_nurbs_full().unwrap();
    assert_eq!(c.domain(), (0.0, TAU));
    assert!((c.point(0.0) - c.point(TAU)).length() < 1e-15);
    let s = Sphere::new(f, 2.0).unwrap().to_nurbs_full().unwrap();
    assert!(s.is_closed_u(0.0));
    let t = Torus::new(f, 5.0, 1.0).unwrap().to_nurbs_full().unwrap();
    assert!(t.is_closed_u(0.0) && t.is_closed_v(0.0));
}

#[test]
fn bad_ranges_are_rejected() {
    let f = Frame3::WORLD;
    let c = Circle3::new(f, 1.0).unwrap();
    assert!(matches!(
        c.to_nurbs(Interval::new(0.0, 7.0)),
        Err(GeomError::InvalidArgument(_))
    ));
    assert!(matches!(
        c.to_nurbs(Interval::new(1.0, 1.0)),
        Err(GeomError::InvalidArgument(_))
    ));
    assert_eq!(
        c.to_nurbs(Interval::new(0.0, f64::INFINITY)),
        Err(GeomError::NonFinite)
    );
    let l = Line3::new(Point3::ZERO, Vector3::X).unwrap();
    assert!(matches!(
        l.to_nurbs(Interval::new(2.0, 2.0)),
        Err(GeomError::InvalidArgument(_))
    ));
    assert_eq!(
        l.to_nurbs(Interval::new(f64::NEG_INFINITY, 0.0)),
        Err(GeomError::NonFinite)
    );
    let s = Sphere::new(f, 1.0).unwrap();
    assert_eq!(
        s.to_nurbs(Interval::new(0.0, 1.0), Interval::new(-2.0, 0.0)),
        Err(GeomError::ParameterOutOfDomain { t: -2.0 })
    );
    let cyl = Cylinder::new(f, 1.0).unwrap();
    assert!(
        cyl.to_nurbs(Interval::new(0.0, 1.0), Interval::new(0.0, 0.0))
            .is_err()
    );
    assert!(
        cyl.to_nurbs(Interval::new(0.0, 7.0), Interval::new(0.0, 1.0))
            .is_err()
    );
    let p = Plane::new(f).unwrap();
    assert!(
        p.to_nurbs(Interval::new(0.0, 1.0), Interval::new(0.0, f64::NAN))
            .is_err()
    );
}
