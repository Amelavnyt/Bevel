// SPDX-License-Identifier: GPL-3.0-or-later

//! Planes, cylinders, cones, spheres and tori: closed forms against finite
//! differences and the fundamental forms, closest points against dense
//! sampling, isocurves and bounding boxes.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use bevel_geom::{Cone, Cylinder, GeomError, Plane, Sphere, Torus};
use bevel_math::{Aabb3, Frame3, Interval, Point3, Vector3};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{
    SurfaceCase, angle, check_bounds, check_surface_closest, check_surface_differentials, config,
    frame, params, point3, radius,
};

/// Samples of a patch for bounding-box checks.
fn patch_points(point: impl Fn(f64, f64) -> Point3, u: Interval, v: Interval) -> Vec<Point3> {
    let mut out = Vec::new();
    for uu in params(u.min, u.max, 301) {
        for vv in params(v.min, v.max, 301) {
            out.push(point(uu, vv));
        }
    }
    out
}

/// Checks a bounding box against dense samples of the patch.
fn check_patch_bounds(
    b: Aabb3,
    point: impl Fn(f64, f64) -> Point3,
    u: Interval,
    v: Interval,
    slack: f64,
) -> Result<(), TestCaseError> {
    check_bounds(b.min, b.max, &patch_points(point, u, v), slack)
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn plane_closed_forms(
        f in frame(),
        uv in prop::array::uniform2(-100.0f64..100.0),
        target in point3(100.0),
        range in prop::array::uniform4(-50.0f64..50.0),
    ) {
        let s = Plane::new(f).unwrap();
        let case = SurfaceCase {
            point: &|u, v| s.point(u, v),
            ders: &|u, v, d| s.derivatives(u, v, d),
            partials: &|u, v| s.partials(u, v),
            normal: &|u, v| s.normal(u, v),
            curvatures: &|u, v| s.principal_curvatures(u, v),
            closest: &|p| s.closest_point(p).unwrap(),
            scale: 1.0,
        };
        check_surface_differentials(&case, uv[0], uv[1])?;
        let grid = params(-300.0, 300.0, 241);
        let cp = check_surface_closest(&case, target, &grid, &grid)?;
        prop_assert!((cp.distance - s.signed_distance(target).abs()).abs() < 1e-12 * 300.0);
        let (iu, iv) = (s.isocurve_u(uv[0]).unwrap(), s.isocurve_v(uv[1]).unwrap());
        prop_assert!((iu.point(uv[1]) - s.point(uv[0], uv[1])).length() < 1e-12 * 300.0);
        prop_assert!((iv.point(uv[0]) - s.point(uv[0], uv[1])).length() < 1e-12 * 300.0);
        let (u, v) = (Interval::new(range[0], range[1]), Interval::new(range[2], range[3]));
        check_patch_bounds(s.bounds(u, v).unwrap(), |u, v| s.point(u, v), u, v, 1e-9)?;
    }

    #[test]
    fn cylinder_closed_forms(
        f in frame(),
        r in radius(),
        u in angle(),
        v in -50.0f64..50.0,
        target in point3(100.0),
        range in (angle(), 0.01f64..7.0, -50.0f64..50.0, 0.0f64..40.0),
    ) {
        let s = Cylinder::new(f, r).unwrap();
        let case = SurfaceCase {
            point: &|u, v| s.point(u, v),
            ders: &|u, v, d| s.derivatives(u, v, d),
            partials: &|u, v| s.partials(u, v),
            normal: &|u, v| s.normal(u, v),
            curvatures: &|u, v| s.principal_curvatures(u, v),
            closest: &|p| s.closest_point(p).unwrap(),
            scale: r,
        };
        check_surface_differentials(&case, u, v)?;
        let k = s.principal_curvatures(u, v).unwrap();
        prop_assert!(k.k1.abs() < 1e-15 && (k.k2 + 1.0 / r).abs() < 1e-12 / r);
        check_surface_closest(&case, target, &params(0.0, TAU, 181), &params(-300.0, 300.0, 601))?;
        let (iu, iv) = (s.isocurve_u(u).unwrap(), s.isocurve_v(v).unwrap());
        prop_assert!((iu.point(v) - s.point(u, v)).length() < 1e-12 * 300.0);
        prop_assert!((iv.point(u) - s.point(u, v)).length() < 1e-12 * 300.0);
        let (bu, bv) = (Interval::new(range.0, range.0 + range.1), Interval::new(range.2, range.2 + range.3));
        check_patch_bounds(s.bounds(bu, bv).unwrap(), |u, v| s.point(u, v), bu, bv, 1e-4 * r)?;
    }

    #[test]
    fn cone_closed_forms(
        f in frame(),
        r in 0.0f64..40.0,
        alpha in 0.05f64..1.5,
        u in angle(),
        v in -50.0f64..50.0,
        target in point3(100.0),
        range in (angle(), 0.01f64..7.0, -50.0f64..50.0, 0.0f64..40.0),
    ) {
        let s = Cone::new(f, r, alpha).unwrap();
        prop_assume!(s.radius_at(v).abs() > 1e-3);
        let case = SurfaceCase {
            point: &|u, v| s.point(u, v),
            ders: &|u, v, d| s.derivatives(u, v, d),
            partials: &|u, v| s.partials(u, v),
            normal: &|u, v| s.normal(u, v),
            curvatures: &|u, v| s.principal_curvatures(u, v),
            closest: &|p| s.closest_point(p).unwrap(),
            scale: r.max(1.0),
        };
        check_surface_differentials(&case, u, v)?;
        let k = s.principal_curvatures(u, v).unwrap();
        prop_assert!((k.k2 + alpha.cos() / s.radius_at(v).abs()).abs() < 1e-12 * k.k2.abs());
        // Normals point away from the axis on both nappes.
        let n = s.normal(u, v).unwrap();
        let p = f.to_local(s.point(u, v));
        prop_assert!(n.dot(f.vector_to_world(Vector3::new(p.x, p.y, 0.0))) > 0.0);
        check_surface_closest(&case, target, &params(0.0, TAU, 181), &params(-300.0, 300.0, 601))?;
        let (iu, iv) = (s.isocurve_u(u).unwrap(), s.isocurve_v(v).unwrap());
        prop_assert!((iu.point(v / alpha.cos()) - s.point(u, v)).length() < 1e-11 * 300.0);
        prop_assert!((iv.point(u) - s.point(u, v)).length() < 1e-12 * 300.0);
        let (bu, bv) = (Interval::new(range.0, range.0 + range.1), Interval::new(range.2, range.2 + range.3));
        let size = s.radius_at(bv.min).abs().max(s.radius_at(bv.max).abs());
        check_patch_bounds(s.bounds(bu, bv).unwrap(), |u, v| s.point(u, v), bu, bv, 1e-4 * size + 1e-9)?;
    }

    #[test]
    fn sphere_closed_forms(
        f in frame(),
        r in radius(),
        u in angle(),
        v in -1.5f64..1.5,
        target in point3(100.0),
        range in (angle(), 0.01f64..7.0, -FRAC_PI_2..FRAC_PI_2, 0.0f64..PI),
    ) {
        let s = Sphere::new(f, r).unwrap();
        let case = SurfaceCase {
            point: &|u, v| s.point(u, v),
            ders: &|u, v, d| s.derivatives(u, v, d),
            partials: &|u, v| s.partials(u, v),
            normal: &|u, v| s.normal(u, v),
            curvatures: &|u, v| s.principal_curvatures(u, v),
            closest: &|p| s.closest_point(p).unwrap(),
            scale: r,
        };
        check_surface_differentials(&case, u, v)?;
        let cp = check_surface_closest(&case, target, &params(0.0, TAU, 181), &params(-FRAC_PI_2, FRAC_PI_2, 181))?;
        prop_assert!((cp.distance - ((target - f.origin).length() - r).abs()).abs() < 1e-12 * 300.0);
        prop_assert!((-FRAC_PI_2..=FRAC_PI_2).contains(&cp.v) && (0.0..TAU).contains(&cp.u));
        let (iu, iv) = (s.isocurve_u(u).unwrap(), s.isocurve_v(v).unwrap());
        prop_assert!((iu.point(v) - s.point(u, v)).length() < 1e-12 * 300.0);
        prop_assert!((iv.point(u) - s.point(u, v)).length() < 1e-12 * 300.0);
        let bu = Interval::new(range.0, range.0 + range.1);
        let bv = Interval::new(range.2, (range.2 + range.3).min(FRAC_PI_2));
        check_patch_bounds(s.bounds(bu, bv).unwrap(), |u, v| s.point(u, v), bu, bv, 2e-4 * r)?;
    }

    #[test]
    fn torus_closed_forms(
        f in frame(),
        big in radius(),
        ratio in 0.05f64..0.9,
        u in angle(),
        v in angle(),
        target in point3(100.0),
        range in (angle(), 0.01f64..7.0, angle(), 0.01f64..7.0),
    ) {
        let s = Torus::new(f, big, big * ratio).unwrap();
        let case = SurfaceCase {
            point: &|u, v| s.point(u, v),
            ders: &|u, v, d| s.derivatives(u, v, d),
            partials: &|u, v| s.partials(u, v),
            normal: &|u, v| s.normal(u, v),
            curvatures: &|u, v| s.principal_curvatures(u, v),
            closest: &|p| s.closest_point(p).unwrap(),
            scale: big,
        };
        check_surface_differentials(&case, u, v)?;
        let grid = params(0.0, TAU, 181);
        check_surface_closest(&case, target, &grid, &grid)?;
        let (iu, iv) = (s.isocurve_u(u).unwrap(), s.isocurve_v(v).unwrap());
        prop_assert!((iu.point(v) - s.point(u, v)).length() < 1e-12 * 300.0);
        prop_assert!((iv.point(u) - s.point(u, v)).length() < 1e-12 * 300.0);
        let (bu, bv) = (Interval::new(range.0, range.0 + range.1), Interval::new(range.2, range.2 + range.3));
        check_patch_bounds(s.bounds(bu, bv).unwrap(), |u, v| s.point(u, v), bu, bv, 4e-4 * big)?;
    }
}

#[test]
fn degenerate_closest_points_have_documented_answers() {
    let f = Frame3 {
        origin: Point3::new(1.0, 2.0, 3.0),
        ..Frame3::WORLD
    };
    // The centre of a sphere: (0, 0).
    let s = Sphere::new(f, 2.0).unwrap();
    let cp = s.closest_point(f.origin).unwrap();
    assert!(cp.u.abs() < 1e-300 && cp.v.abs() < 1e-300 && (cp.distance - 2.0).abs() < 1e-15);
    // A point on the axis of a sphere: a pole with u = 0.
    let cp = s.closest_point(f.origin + Vector3::Z * 5.0).unwrap();
    assert!(cp.u.abs() < 1e-300 && (cp.v - FRAC_PI_2).abs() < 1e-15);
    assert!((cp.distance - 3.0).abs() < 1e-14);
    // The axis of a cylinder: u = 0.
    let c = Cylinder::new(f, 2.0).unwrap();
    let cp = c.closest_point(f.origin + Vector3::Z * 4.0).unwrap();
    assert!(cp.u.abs() < 1e-300 && (cp.v - 4.0).abs() < 1e-14);
    // The axis of a torus: u = 0; its centre circle: v = 0.
    let t = Torus::new(f, 5.0, 1.0).unwrap();
    let cp = t.closest_point(f.origin).unwrap();
    assert!(cp.u.abs() < 1e-300 && (cp.distance - 4.0).abs() < 1e-14);
    let cp = t.closest_point(f.origin + Vector3::Y * 5.0).unwrap();
    assert!((cp.u - FRAC_PI_2).abs() < 1e-15 && cp.v.abs() < 1e-300);
    assert!((cp.distance - 1.0).abs() < 1e-14);
    // The apex of a cone, and the axis below it, where the other nappe is
    // closer than the ruling at u.
    let k = Cone::new(f, 1.0, PI / 4.0).unwrap();
    assert!((k.apex() - (f.origin - Vector3::Z)).length() < 1e-15);
    assert!(k.normal(0.3, k.apex_parameter()).is_none());
    assert!(k.principal_curvatures(0.3, -1.0).is_none());
    assert!(k.isocurve_v(-1.0).is_err());
    let cp = k.closest_point(k.apex()).unwrap();
    assert!(cp.distance < 1e-15);
    let target = f.origin + Vector3::new(0.1, 0.0, -5.0);
    let cp = k.closest_point(target).unwrap();
    assert!((cp.distance - 3.9 / 2f64.sqrt()).abs() < 1e-14, "{cp:?}");
    assert!((cp.u - PI).abs() < 1e-15);
}

#[test]
fn degenerate_surfaces_are_rejected() {
    let f = Frame3::WORLD;
    let bad = GeomError::InvalidArgument("a radius must be positive");
    assert_eq!(Cylinder::new(f, 0.0), Err(bad));
    assert_eq!(Sphere::new(f, -1.0), Err(bad));
    assert_eq!(Torus::new(f, 0.0, 0.0), Err(bad));
    assert_eq!(Sphere::new(f, f64::NAN), Err(GeomError::NonFinite));
    for alpha in [0.0, -0.1, FRAC_PI_2, 2.0] {
        assert_eq!(
            Cone::new(f, 1.0, alpha),
            Err(GeomError::InvalidArgument(
                "the half-angle of a cone must lie strictly between 0 and pi / 2"
            ))
        );
    }
    assert!(Cone::new(f, -1.0, 0.5).is_err());
    assert!(Cone::new(f, 0.0, 0.5).is_ok());
    for (big, small) in [(1.0, 1.0), (1.0, 2.0)] {
        assert_eq!(
            Torus::new(f, big, small),
            Err(GeomError::InvalidArgument(
                "the minor radius of a torus must be less than its major radius"
            ))
        );
    }
    assert!(Plane::from_normal(Point3::ZERO, Vector3::ZERO).is_err());
    let skew = Frame3 {
        z: -Vector3::Z,
        ..Frame3::WORLD
    };
    assert!(matches!(
        Plane::new(skew),
        Err(GeomError::InvalidArgument(_))
    ));
    // The isocurves at a pole of a sphere are points.
    let s = Sphere::new(f, 1.0).unwrap();
    assert!(s.isocurve_v(FRAC_PI_2).is_err());
    assert!(s.isocurve_v(f64::NAN).is_err());
    // The normal at a pole of a sphere exists.
    let n = s.normal(0.7, FRAC_PI_2).unwrap();
    assert!((n - Vector3::Z).length() < 1e-15);
}
