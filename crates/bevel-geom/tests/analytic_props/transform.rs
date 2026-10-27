// SPDX-License-Identifier: GPL-3.0-or-later

//! Transforms: rigid motions and uniform scales keep every analytic type
//! and commute with evaluation (with the documented parameter scaling),
//! reflections reverse `u` of the surfaces of revolution, non-uniform maps
//! turn circles into ellipses and spheres and tori into NURBS, and
//! singular or projective matrices are rejected.

use std::f64::consts::{FRAC_PI_2, PI, TAU};
use std::mem::discriminant;

use bevel_geom::{
    Circle2, Circle3, Cone, Curve2, Curve3, CurveGeom, Cylinder, Ellipse2, Ellipse3, GeomError,
    Line2, Line3, Plane, Sphere, Surface, SurfaceGeom, Torus, circle_arc,
};
use bevel_math::{DMat3, DMat4, DQuat, DVec2, DVec3, Frame3, Interval, Point3};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{angle, config, frame, params, point3, radius, rotation};

/// The tolerance on points: coordinates are within a few hundred of the
/// origin.
const TOL: f64 = 1e-12 * 500.0;

/// A random rigid motion (rotation and translation).
fn rigid() -> impl Strategy<Value = DMat4> {
    (rotation(), point3(50.0)).prop_map(|(q, t)| DMat4::from_rotation_translation(q, t))
}

/// The analytic surfaces of one random case.
fn surfaces(f: Frame3, r: f64) -> Vec<Surface> {
    vec![
        Plane::new(f).unwrap().into(),
        Cylinder::new(f, r).unwrap().into(),
        Cone::new(f, r, 0.6).unwrap().into(),
        Sphere::new(f, r).unwrap().into(),
        Torus::new(f, 2.0 * r, r).unwrap().into(),
    ]
}

/// The analytic space curves of one random case.
fn curves(f: Frame3, r: f64) -> Vec<Curve3> {
    vec![
        Line3::new(f.origin, f.x + f.z).unwrap().into(),
        Circle3::new(f, r).unwrap().into(),
        Ellipse3::new(f, r, 0.4 * r).unwrap().into(),
    ]
}

/// `m` applied to `c` commutes with evaluation, with the analytic
/// parameter `t` of `c` becoming `map(t)`; the kind is kept.
fn check_curve(c: &Curve3, m: &DMat4, map: impl Fn(f64) -> f64) -> Result<Curve3, TestCaseError> {
    let t = c.transformed(m).unwrap();
    prop_assert_eq!(discriminant(c), discriminant(&t), "{:?} became {:?}", c, t);
    for s in params(-4.0, 4.0, 17) {
        let want = m.transform_point3(c.point(s));
        prop_assert!((t.point(map(s)) - want).length() <= TOL, "{c:?} at {s}");
    }
    Ok(t)
}

/// `m` applied to `s` commutes with evaluation, with `(u, v)` becoming
/// `map(u, v)`; the kind is kept.
fn check_surface(
    s: &Surface,
    m: &DMat4,
    map: impl Fn(f64, f64) -> (f64, f64),
) -> Result<Surface, TestCaseError> {
    let t = s.transformed(m).unwrap();
    prop_assert_eq!(discriminant(s), discriminant(&t), "{:?} became {:?}", s, t);
    for u in params(-3.0, 3.0, 9) {
        for v in params(-1.4, 1.4, 9) {
            let (uu, vv) = map(u, v);
            let want = m.transform_point3(s.point(u, v));
            prop_assert!(
                (t.point(uu, vv) - want).length() <= TOL,
                "{s:?} at ({u}, {v})"
            );
        }
    }
    Ok(t)
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn rigid_motions_commute_with_evaluation(f in frame(), r in radius(), m in rigid(), u in angle(), v in -1.4f64..1.4) {
        let inverse = m.inverse();
        for c in curves(f, r) {
            let t = check_curve(&c, &m, |t| t)?;
            // Tangents rotate and curvatures stay.
            let (t0, t1) = (c.tangent(u).unwrap(), t.tangent(u).unwrap());
            prop_assert!((m.transform_vector3(t0) - t1).length() <= 1e-12);
            prop_assert!((c.curvature(u).unwrap() - t.curvature(u).unwrap()).abs() <= 1e-12 * c.curvature(u).unwrap().max(1.0));
            // And back.
            let back = t.transformed(&inverse).unwrap();
            prop_assert!((back.point(u) - c.point(u)).length() <= TOL);
        }
        for s in surfaces(f, r) {
            let t = check_surface(&s, &m, |u, v| (u, v))?;
            let (n0, n1) = (s.normal(u, v).unwrap(), t.normal(u, v).unwrap());
            prop_assert!((m.transform_vector3(n0) - n1).length() <= 1e-12);
            let (k0, k1) = (s.principal_curvatures(u, v).unwrap(), t.principal_curvatures(u, v).unwrap());
            prop_assert!((k0.k1 - k1.k1).abs() <= 1e-12 * (1.0 + k0.k1.abs()) && (k0.k2 - k1.k2).abs() <= 1e-12 * (1.0 + k0.k2.abs()));
            let back = t.transformed(&inverse).unwrap();
            prop_assert!((back.point(u, v) - s.point(u, v)).length() <= TOL);
        }
    }

    #[test]
    fn rigid_motions_commute_in_the_plane(c in prop::array::uniform2(-50.0f64..50.0), r in radius(), a in angle(), t in prop::array::uniform2(-50.0f64..50.0), ccw in any::<bool>()) {
        let m = DMat3::from_scale_angle_translation(DVec2::ONE, a, DVec2::from_array(t));
        let center = DVec2::from_array(c);
        let all: Vec<Curve2> = vec![
            Line2::new(center, DVec2::new(1.0, 2.0)).unwrap().into(),
            Circle2::from_axis(center, DVec2::new(1.0, -1.0), ccw, r).unwrap().into(),
            Ellipse2::from_axis(center, DVec2::new(0.5, 1.0), ccw, r, 0.3 * r).unwrap().into(),
        ];
        for curve in all {
            let moved = curve.transformed(&m).unwrap();
            prop_assert_eq!(discriminant(&curve), discriminant(&moved));
            for s in params(-4.0, 4.0, 17) {
                prop_assert!((moved.point(s) - m.transform_point2(curve.point(s))).length() <= TOL);
                // A rotation keeps the signed curvature.
                prop_assert!((moved.curvature(s).unwrap() - curve.curvature(s).unwrap()).abs() <= 1e-12 * (1.0 + curve.curvature(s).unwrap().abs()));
            }
        }
    }

    #[test]
    fn uniform_scales_keep_the_kind(f in frame(), r in radius(), q in rotation(), s in 0.1f64..10.0, t in point3(50.0)) {
        let m = DMat4::from_scale_rotation_translation(DVec3::splat(s), q, t);
        let cs = curves(f, r);
        check_curve(&cs[0], &m, |t| s * t)?;
        for c in &cs[1..] {
            let moved = check_curve(c, &m, |t| t)?;
            prop_assert!((moved.curvature(0.3).unwrap() - c.curvature(0.3).unwrap() / s).abs() <= 1e-12 * c.curvature(0.3).unwrap() / s);
        }
        let ss = surfaces(f, r);
        check_surface(&ss[0], &m, |u, v| (s * u, s * v))?;
        check_surface(&ss[1], &m, |u, v| (u, s * v))?;
        check_surface(&ss[2], &m, |u, v| (u, s * v))?;
        check_surface(&ss[3], &m, |u, v| (u, v))?;
        check_surface(&ss[4], &m, |u, v| (u, v))?;
    }

    #[test]
    fn reflections_reverse_u_of_surfaces_of_revolution(f in frame(), r in radius(), m in rigid(), u in angle(), v in -1.4f64..1.4) {
        // A mirror in the world XY plane, then a rigid motion.
        let m = m * DMat4::from_scale(DVec3::new(1.0, 1.0, -1.0));
        for c in curves(f, r) {
            let moved = check_curve(&c, &m, |t| t)?;
            if let (Curve3::Circle(a), Curve3::Circle(b)) = (&c, &moved) {
                prop_assert!((m.transform_vector3(a.normal()) + b.normal()).length() <= 1e-12);
            }
        }
        let ss = surfaces(f, r);
        let plane = check_surface(&ss[0], &m, |u, v| (u, v))?;
        // The plane keeps (u, v), so its normal is opposite the image.
        let n = plane.normal(u, v).unwrap();
        prop_assert!((n + m.transform_vector3(ss[0].normal(u, v).unwrap())).length() <= 1e-12);
        for s in &ss[1..] {
            let moved = check_surface(s, &m, |u, v| (-u, v))?;
            // Normals stay outward: they are the images of the old ones.
            let n = moved.normal(-u, v).unwrap();
            prop_assert!((n - m.transform_vector3(s.normal(u, v).unwrap())).length() <= 1e-12);
        }
    }

    #[test]
    fn non_uniform_scales_make_ellipses(f in frame(), r in radius(), q in rotation(), k in prop::array::uniform3(0.2f64..5.0), t in point3(20.0)) {
        let (lo, hi) = (k[0].min(k[1]).min(k[2]), k[0].max(k[1]).max(k[2]));
        prop_assume!(hi - lo > 0.05);
        let m = DMat4::from_scale_rotation_translation(DVec3::from_array(k), q, t);
        let inverse = m.inverse();
        let size = 300.0 * hi.max(1.0);
        for c in curves(f, r) {
            let moved = c.transformed(&m).unwrap();
            // The same point sets: images on the new curve and new points
            // back on the old one.
            for s in params(0.0, TAU, 25) {
                let image = m.transform_point3(c.point(s));
                prop_assert!(moved.closest_point(image).unwrap().distance <= 1e-12 * size);
                let back = inverse.transform_point3(moved.point(s));
                prop_assert!(c.closest_point(back).unwrap().distance <= 1e-12 * size);
            }
            if matches!(c, Curve3::Line(_)) {
                prop_assert!(matches!(moved, Curve3::Line(_)));
            } else {
                prop_assert!(moved.is_analytic());
            }
        }
        let plane = Plane::new(f).unwrap();
        let moved = SurfaceGeom::transformed(&plane, &m).unwrap();
        prop_assert!(matches!(moved, Surface::Plane(_)));
        for (u, v) in [(0.0, 0.0), (3.0, -2.0), (-10.0, 7.0)] {
            prop_assert!(moved.closest_point(m.transform_point3(plane.point(u, v))).unwrap().distance <= 1e-12 * size);
        }
        // Spheres and tori become NURBS through the full conversion, with
        // its parameter maps.
        for s in [Surface::from(Sphere::new(f, r).unwrap()), Surface::from(Torus::new(f, 2.0 * r, r).unwrap())] {
            let moved = s.transformed(&m).unwrap();
            prop_assert!(matches!(moved, Surface::Nurbs(_)));
            let conv = s.to_nurbs_mapped(Interval::new(0.0, TAU), s.domain_v().interval().unwrap()).unwrap();
            // Inside the base periods (the maps clamp to them).
            for (u, v) in [(0.3, 0.2), (2.0, 1.0), (5.0, 1.2)] {
                let (su, sv) = (conv.map_u.nurbs_parameter(u), conv.map_v.nurbs_parameter(v));
                prop_assert!((moved.point(su, sv) - m.transform_point3(s.point(u, v))).length() <= 1e-12 * size);
            }
        }
    }

    #[test]
    fn stretching_along_the_axis_keeps_cylinders_and_cones(f in frame(), r in radius(), s in 0.2f64..5.0, k in 0.2f64..5.0, g in rigid()) {
        // Scale by s across the axis and k along it, in the frame, then move.
        let m = g * f.matrix() * DMat4::from_scale(DVec3::new(s, s, k)) * f.inverse_matrix();
        let cyl = Surface::from(Cylinder::new(f, r).unwrap());
        let moved = check_surface(&cyl, &m, |u, v| (u, k * v))?;
        if let Surface::Cylinder(c) = moved {
            prop_assert!((c.radius() - s * r).abs() <= 1e-12 * s * r);
        }
        let cone = Surface::from(Cone::new(f, r, 0.6).unwrap());
        let moved = check_surface(&cone, &m, |u, v| (u, k * v))?;
        if let Surface::Cone(c) = moved {
            prop_assert!((c.half_angle().tan() - 0.6f64.tan() * s / k).abs() <= 1e-12);
        }
        // A shear across the axis makes them elliptic: rejected.
        let shear = f.matrix() * DMat4::from_scale(DVec3::new(s, 2.0 * s + 0.5, k)) * f.inverse_matrix();
        prop_assert!(matches!(cyl.transformed(&shear), Err(GeomError::InvalidArgument(_))));
        prop_assert!(matches!(cone.transformed(&shear), Err(GeomError::InvalidArgument(_))));
    }
}

#[test]
fn circles_under_non_uniform_scales() {
    // A circle in the XY plane scaled (2, 1, 1): the ellipse with radii 2r
    // and r and the same parameter.
    let c = Circle3::new(Frame3::WORLD, 3.0).unwrap();
    let m = DMat4::from_scale(DVec3::new(2.0, 1.0, 1.0));
    let Curve3::Ellipse(e) = CurveGeom::transformed(&c, &m).unwrap() else {
        panic!("expected an ellipse");
    };
    assert!((e.major_radius() - 6.0).abs() < 1e-14 && (e.minor_radius() - 3.0).abs() < 1e-14);
    assert!((e.point(0.4) - m.transform_point3(c.point(0.4))).length() < 1e-14);
    // Scaled (1, 2, 1) the major axis is the old y axis: t shifts by pi / 2.
    let m = DMat4::from_scale(DVec3::new(1.0, 2.0, 1.0));
    let Curve3::Ellipse(e) = CurveGeom::transformed(&c, &m).unwrap() else {
        panic!("expected an ellipse");
    };
    assert!((e.point(0.4 - FRAC_PI_2) - m.transform_point3(c.point(0.4))).length() < 1e-14);
    // Scaled across its plane only, the circle stays a circle.
    let m = DMat4::from_scale(DVec3::new(2.0, 2.0, 5.0));
    assert!(matches!(
        CurveGeom::transformed(&c, &m),
        Ok(Curve3::Circle(_))
    ));
    // An ellipse can become a circle.
    let e = Ellipse3::new(Frame3::WORLD, 4.0, 2.0).unwrap();
    let m = DMat4::from_scale(DVec3::new(0.5, 1.0, 1.0));
    let Ok(Curve3::Circle(c)) = CurveGeom::transformed(&e, &m) else {
        panic!("expected a circle");
    };
    assert!((c.radius() - 2.0).abs() < 1e-14);
    // In the plane: a skew map gives an ellipse whose angle is offset.
    let c2 = Circle2::new(DVec2::ZERO, 1.0).unwrap();
    let m = DMat3::from_cols_array(&[1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 3.0, 4.0, 1.0]);
    let moved = CurveGeom::transformed(&c2, &m).unwrap();
    assert!(matches!(moved, Curve2::Ellipse(_)));
    for i in 0..16 {
        let p = m.transform_point2(c2.point(f64::from(i)));
        assert!(moved.closest_point(p).unwrap().distance < 1e-13);
    }
    // A mirror reverses a planar circle.
    let mirror = DMat3::from_scale(DVec2::new(-1.0, 1.0));
    let Curve2::Circle(m2) = CurveGeom::transformed(&c2, &mirror).unwrap() else {
        panic!("expected a circle");
    };
    assert!(!m2.is_counter_clockwise());
    assert!((m2.point(0.7) - mirror.transform_point2(c2.point(0.7))).length() < 1e-15);
}

#[test]
fn bad_matrices_are_rejected() {
    let f = Frame3::WORLD;
    let flat = DMat4::from_scale(DVec3::new(1.0, 1.0, 0.0));
    let mut projective = DMat4::IDENTITY;
    projective.x_axis.w = 0.1;
    let mut nan = DMat4::IDENTITY;
    nan.w_axis.x = f64::NAN;
    let arc = circle_arc(&f, 1.0, 0.0, PI).unwrap();
    let curves: Vec<Curve3> = vec![
        Line3::new(Point3::ZERO, DVec3::X).unwrap().into(),
        Circle3::new(f, 1.0).unwrap().into(),
        Ellipse3::new(f, 2.0, 1.0).unwrap().into(),
        arc.clone().into(),
    ];
    for c in &curves {
        assert_eq!(
            c.transformed(&flat),
            Err(GeomError::InvalidArgument(
                "the transform must be invertible"
            ))
        );
        assert_eq!(c.transformed(&nan), Err(GeomError::NonFinite));
        let projected = c.transformed(&projective);
        assert_eq!(projected.is_ok(), !c.is_analytic(), "{c:?}");
    }
    let mut surfaces = surfaces(f, 1.0);
    surfaces.push(Sphere::new(f, 1.0).unwrap().to_nurbs_full().unwrap().into());
    for s in &surfaces {
        assert_eq!(
            s.transformed(&flat),
            Err(GeomError::InvalidArgument(
                "the transform must be invertible"
            ))
        );
        assert_eq!(s.transformed(&nan), Err(GeomError::NonFinite));
        assert_eq!(
            s.transformed(&projective).is_ok(),
            !s.is_analytic(),
            "{s:?}"
        );
    }
    let flat2 = DMat3::from_scale(DVec2::new(0.0, 1.0));
    let c2 = Curve2::from(Circle2::new(DVec2::ZERO, 1.0).unwrap());
    assert!(c2.transformed(&flat2).is_err());
    let n2 = Curve2::from(c2.to_nurbs_full().unwrap());
    assert!(n2.transformed(&flat2).is_err());
    assert!(n2.transformed(&DMat3::from_angle(0.3)).is_ok());
    // A NURBS arc under a rigid motion matches its analytic circle's image.
    let m = DMat4::from_rotation_translation(DQuat::from_rotation_x(0.7), DVec3::ONE);
    let moved = CurveGeom::transformed(&arc, &m).unwrap();
    assert!((moved.point(1.0) - m.transform_point3(arc.point(1.0))).length() < 1e-14);
}
