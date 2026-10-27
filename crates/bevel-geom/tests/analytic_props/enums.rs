// SPDX-License-Identifier: GPL-3.0-or-later

//! The curve and surface enums dispatch to their variants, and the traits
//! behave alike on analytic and NURBS geometry.

use std::f64::consts::{FRAC_PI_2, TAU};

use bevel_geom::{
    Circle2, Circle3, Cone, Curve2, Curve3, CurveGeom, Cylinder, Domain, Ellipse2, Ellipse3,
    GeomError, Line2, Line3, NurbsCurve3, ParamMap, Plane, Sphere, Surface, SurfaceGeom, Torus,
    circle_arc,
};
use bevel_math::{Aabb3, Frame3, Interval, Point2, Point3, Vector2, Vector3};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{Pt, angle, config, frame, point3, radius};

/// The enum gives the same answers as the curve it wraps.
fn check_curve_dispatch<C, E>(c: &C, e: &E, t: f64, target: C::Point) -> Result<(), TestCaseError>
where
    C: CurveGeom,
    E: CurveGeom<Point = C::Point, Homogeneous = C::Homogeneous>,
    C::Point: Pt,
{
    prop_assert!((c.point(t) - e.point(t)).len() <= 0.0);
    let (dc, de) = (c.derivatives(t, 3), e.derivatives(t, 3));
    prop_assert_eq!(dc.len(), de.len());
    for (a, b) in dc.iter().zip(&de) {
        prop_assert!((*a - *b).len() <= 0.0);
    }
    prop_assert!((c.tangent(t).unwrap() - e.tangent(t).unwrap()).len() <= 0.0);
    prop_assert!((c.curvature(t).unwrap() - e.curvature(t).unwrap()).abs() <= 0.0);
    prop_assert_eq!(c.domain(), e.domain());
    let (cc, ce) = (
        c.closest_point(target).unwrap(),
        e.closest_point(target).unwrap(),
    );
    prop_assert!((cc.t - ce.t).abs() <= 0.0 && (cc.distance - ce.distance).abs() <= 0.0);
    prop_assert!((c.reversed_parameter(t) - e.reversed_parameter(t)).abs() <= 0.0);
    let (rc, re) = (c.reversed(), e.reversed());
    prop_assert!((rc.point(rc.reversed_parameter(t)) - c.point(t)).len() <= 1e-12 * 200.0);
    prop_assert!((re.point(re.reversed_parameter(t)) - c.point(t)).len() <= 1e-12 * 200.0);
    let range = Interval::new(t, t + 1.0);
    prop_assert_eq!(
        c.to_nurbs_mapped(range).unwrap(),
        e.to_nurbs_mapped(range).unwrap()
    );
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn curve_enums_dispatch(f in frame(), r in radius(), t in angle(), target in point3(100.0)) {
        let l = Line3::new(f.origin, f.x).unwrap();
        check_curve_dispatch(&l, &Curve3::from(l), t, target)?;
        let c = Circle3::new(f, r).unwrap();
        check_curve_dispatch(&c, &Curve3::from(c), t, target)?;
        let e = Ellipse3::new(f, r, 0.5 * r).unwrap();
        check_curve_dispatch(&e, &Curve3::from(e), t, target)?;
        let arc = circle_arc(&f, r, -1.0, 2.5).unwrap();
        check_curve_dispatch(&arc, &Curve3::from(arc.clone()), t.clamp(-1.0, 1.4), target)?;
        let t2 = target.truncate();
        let l2 = Line2::new(Point2::ZERO, Vector2::new(1.0, 2.0)).unwrap();
        check_curve_dispatch(&l2, &Curve2::from(l2), t, t2)?;
        let c2 = Circle2::new(Point2::new(1.0, 2.0), r).unwrap();
        check_curve_dispatch(&c2, &Curve2::from(c2), t, t2)?;
        let e2 = Ellipse2::new(Point2::ZERO, Vector2::Y, r, 0.3 * r).unwrap();
        check_curve_dispatch(&e2, &Curve2::from(e2), t, t2)?;
        let n2 = c2.to_nurbs(Interval::new(0.0, 3.0)).unwrap();
        check_curve_dispatch(&n2, &Curve2::from(n2.clone()), t.clamp(0.0, 2.0), t2)?;
        for curve in [Curve3::from(l), Curve3::from(arc)] {
            let b = curve.bounds(Interval::new(0.0, 1.0)).unwrap();
            prop_assert!(b.contains(curve.point(0.5)));
        }
    }

    #[test]
    fn surface_enum_dispatches(f in frame(), r in radius(), u in 0.1f64..1.4, v in -1.4f64..1.4, target in point3(100.0)) {
        let surfaces: Vec<Surface> = vec![
            Plane::new(f).unwrap().into(),
            Cylinder::new(f, r).unwrap().into(),
            Cone::new(f, r + 2.0, 0.4).unwrap().into(),
            Sphere::new(f, r).unwrap().into(),
            Torus::new(f, 2.0 * r, r).unwrap().into(),
            Sphere::new(f, r).unwrap().to_nurbs_full().unwrap().into(),
        ];
        for s in &surfaces {
            let p = s.point(u, v);
            let d = s.derivatives(u, v, 2);
            prop_assert!((d[0][0] - p).length() <= 1e-14 * 200.0);
            let (su, sv) = s.partials(u, v);
            prop_assert!((su - d[1][0]).length() <= 1e-13 * r.max(1.0));
            prop_assert!((sv - d[0][1]).length() <= 1e-13 * r.max(1.0));
            let n = s.normal(u, v).unwrap();
            prop_assert!(n.dot(su).abs() <= 1e-9 * su.length() && n.dot(sv).abs() <= 1e-9 * sv.length());
            let k = s.principal_curvatures(u, v).unwrap();
            prop_assert!(k.k1 >= k.k2);
            let cp = s.closest_point(target).unwrap();
            prop_assert!((s.point(cp.u, cp.v) - cp.point).length() <= 1e-9 * 200.0);
            // Isocurves pass through the surface point.
            let iu = s.isocurve_u(u).unwrap();
            let iv = s.isocurve_v(v).unwrap();
            let on_u = iu.closest_point(p).unwrap().distance;
            let on_v = iv.closest_point(p).unwrap().distance;
            prop_assert!(on_u <= 1e-9 * 200.0 && on_v <= 1e-9 * 200.0, "{s:?}: {on_u} {on_v}");
            let b = s.bounds(Interval::new(u, u + 0.5), Interval::new(v, v + 0.1)).unwrap();
            let slack = Vector3::splat(1e-12 * 200.0);
            prop_assert!(Aabb3::new(b.min - slack, b.max + slack).contains(p));
        }
    }
}

#[test]
fn isocurves_keep_their_analytic_kind() {
    let f = Frame3::WORLD;
    let cyl = Surface::from(Cylinder::new(f, 2.0).unwrap());
    assert!(matches!(cyl.isocurve_u(0.3), Ok(Curve3::Line(_))));
    assert!(matches!(cyl.isocurve_v(0.3), Ok(Curve3::Circle(_))));
    let cone = Surface::from(Cone::new(f, 2.0, 0.5).unwrap());
    assert!(matches!(cone.isocurve_u(0.3), Ok(Curve3::Line(_))));
    assert!(matches!(cone.isocurve_v(0.3), Ok(Curve3::Circle(_))));
    let plane = Surface::from(Plane::new(f).unwrap());
    assert!(matches!(plane.isocurve_u(0.3), Ok(Curve3::Line(_))));
    assert!(matches!(plane.isocurve_v(0.3), Ok(Curve3::Line(_))));
    for s in [
        Surface::from(Sphere::new(f, 2.0).unwrap()),
        Surface::from(Torus::new(f, 3.0, 1.0).unwrap()),
    ] {
        assert!(matches!(s.isocurve_u(0.3), Ok(Curve3::Circle(_))));
        assert!(matches!(s.isocurve_v(0.3), Ok(Curve3::Circle(_))));
    }
    let nurbs = Surface::from(Sphere::new(f, 2.0).unwrap().to_nurbs_full().unwrap());
    assert!(!nurbs.is_analytic());
    assert!(matches!(nurbs.isocurve_u(0.3), Ok(Curve3::Nurbs(_))));
    // A sphere's parallel at a pole is a point.
    let sphere = Surface::from(Sphere::new(f, 2.0).unwrap());
    assert!(matches!(
        sphere.isocurve_v(FRAC_PI_2),
        Err(GeomError::InvalidArgument(_))
    ));
}

#[test]
fn domains_and_periods() {
    let f = Frame3::WORLD;
    let full = Interval::new(0.0, TAU);
    let s = Surface::from(Sphere::new(f, 1.0).unwrap());
    assert_eq!(s.domain_u(), Domain::Periodic(full));
    assert!((s.period_u().unwrap() - TAU).abs() < 1e-15);
    assert!(s.period_v().is_none());
    assert_eq!(
        s.domain_v(),
        Domain::Bounded(Interval::new(-FRAC_PI_2, FRAC_PI_2))
    );
    let t = Surface::from(Torus::new(f, 3.0, 1.0).unwrap());
    assert!(t.domain_u().is_periodic() && t.domain_v().is_periodic());
    let c = Surface::from(Cylinder::new(f, 1.0).unwrap());
    assert_eq!(c.domain_v(), Domain::Unbounded);
    assert!(matches!(
        c.to_nurbs_full(),
        Err(GeomError::InvalidArgument(_))
    ));
    let l = Curve3::from(Line3::new(Point3::ZERO, Vector3::X).unwrap());
    assert_eq!(l.domain(), Domain::Unbounded);
    assert!(l.to_nurbs_full().is_err());
    let arc = circle_arc(&f, 1.0, 0.5, 2.0).unwrap();
    assert_eq!(
        CurveGeom::domain(&arc),
        Domain::Bounded(Interval::new(0.5, 2.0))
    );
    let circle = Curve3::from(Circle3::new(f, 1.0).unwrap());
    assert_eq!(circle.to_nurbs_full().unwrap().domain(), (0.0, TAU));
}

#[test]
fn nurbs_pieces_restrict_to_their_range() {
    let arc = circle_arc(&Frame3::WORLD, 2.0, 0.0, 3.0).unwrap();
    let c = Curve3::from(arc.clone());
    // The whole domain gives the curve itself.
    let whole = c.to_nurbs_mapped(Interval::new(0.0, 3.0)).unwrap();
    assert_eq!(whole.nurbs, arc);
    assert_eq!(whole.map, ParamMap::Identity);
    // A piece keeps the parametrisation.
    let piece = c.to_nurbs(Interval::new(0.5, 2.0)).unwrap();
    assert_eq!(piece.domain(), (0.5, 2.0));
    for i in 0..=20 {
        let t = 0.5 + 1.5 * f64::from(i) / 20.0;
        assert!((piece.point(t) - arc.point(t)).length() < 1e-13);
    }
    // Boxes of pieces contain them and are no bigger than the whole.
    let b = c.bounds(Interval::new(0.5, 2.0)).unwrap();
    for i in 0..=20 {
        assert!(b.contains(arc.point(0.5 + 1.5 * f64::from(i) / 20.0)));
    }
    let point_box = c.bounds(Interval::new(1.0, 1.0)).unwrap();
    assert!((point_box.min - arc.point(1.0)).length() < 1e-15);
    assert!(matches!(
        c.to_nurbs(Interval::new(-1.0, 2.0)),
        Err(GeomError::ParameterOutOfDomain { .. })
    ));
    assert!((c.reversed_parameter(0.5) - 2.5).abs() < 1e-15);
    let r = c.reversed();
    assert!((r.point(2.5) - c.point(0.5)).length() < 1e-14);
    // Surfaces too.
    let s = Sphere::new(Frame3::WORLD, 1.0).unwrap();
    let n = Surface::from(s.to_nurbs_full().unwrap());
    let patch = n
        .to_nurbs(Interval::new(1.0, 2.0), Interval::new(-0.5, 0.5))
        .unwrap();
    assert_eq!(patch.domain(), ((1.0, 2.0), (-0.5, 0.5)));
    let b = n
        .bounds(Interval::new(1.0, 2.0), Interval::new(-0.5, 0.5))
        .unwrap();
    // Up to rounding: the corners of the patch lie on the box.
    let b = Aabb3::new(b.min - Vector3::splat(1e-15), b.max + Vector3::splat(1e-15));
    for i in 0..=10 {
        for j in 0..=10 {
            let (u, v) = (1.0 + f64::from(i) / 10.0, -0.5 + f64::from(j) / 10.0);
            assert!(b.contains(patch.point(u, v)));
            assert!((patch.point(u, v) - n.point(u, v)).length() < 1e-13);
        }
    }
    assert!(
        n.bounds(Interval::new(1.0, 1.0), Interval::new(-0.5, 0.5))
            .is_ok()
    );
}

/// Generic code runs on 2D and 3D curves alike.
fn sampled_length<C: CurveGeom>(c: &C, range: Interval) -> f64
where
    C::Point: Pt,
{
    let n = 2000;
    (0..n)
        .map(|i| {
            let a = range.lerp(f64::from(i) / f64::from(n));
            let b = range.lerp(f64::from(i + 1) / f64::from(n));
            (c.point(b) - c.point(a)).len()
        })
        .sum()
}

#[test]
fn generic_code_runs_in_both_dimensions() {
    let half = Interval::new(0.0, std::f64::consts::PI);
    let c2 = Curve2::from(Circle2::new(Point2::ZERO, 2.0).unwrap());
    let c3 = Curve3::from(Circle3::new(Frame3::WORLD, 2.0).unwrap());
    let n3: NurbsCurve3 = c3.to_nurbs(half).unwrap();
    for len in [
        sampled_length(&c2, half),
        sampled_length(&c3, half),
        sampled_length(&n3, half),
    ] {
        assert!((len - 2.0 * std::f64::consts::PI).abs() < 1e-5);
    }
}
