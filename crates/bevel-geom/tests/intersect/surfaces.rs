// SPDX-License-Identifier: GPL-3.0-or-later

//! Curve–surface intersection.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

use bevel_geom::intersect::{CurveSurfaceHit, Rect, curve_surface};
use bevel_geom::{
    Circle3, Cone, Curve3, CurveGeom, Cylinder, GeomError, KnotVector, Line3, NurbsCurve3,
    NurbsSurface, Plane, Sphere, Surface, SurfaceGeom, Torus,
};
use bevel_math::{Frame3, Interval, Point3, Vector3};

use crate::helpers::{
    Spot, TOL, assert_same_spots, check_hits_cs, nurbs_surface, nurbs3, spots_cs,
};

fn iv(a: f64, b: f64) -> Interval {
    Interval::new(a, b)
}

fn line(o: Point3, d: Vector3) -> Curve3 {
    Curve3::Line(Line3::new(o, d).unwrap())
}

fn tilted(origin: Point3) -> Frame3 {
    Frame3::from_axes(
        origin,
        Vector3::new(1.0, 0.2, -0.1),
        Vector3::new(-0.3, 1.0, 0.4),
    )
    .unwrap()
}

/// Intersects the pair directly (fast path) and as NURBS (general method),
/// checks both and that they agree in space; returns the direct hits.
fn cross_check(c: &Curve3, rc: Interval, s: &Surface, rs: Rect) -> Vec<CurveSurfaceHit> {
    let direct = curve_surface(c, rc, s, rs, TOL).unwrap();
    check_hits_cs(c, rc, s, rs, &direct, TOL);
    let (nc, nrc) = nurbs3(c, rc);
    let (ns, nrs) = nurbs_surface(s, rs);
    let general = curve_surface(&nc, nrc, &ns, nrs, TOL).unwrap();
    check_hits_cs(&nc, nrc, &ns, nrs, &general, TOL);
    assert_same_spots(&spots_cs(&nc, &general), &spots_cs(c, &direct), 1e-5);
    direct
}

fn points(hits: &[CurveSurfaceHit]) -> Vec<Point3> {
    hits.iter()
        .filter_map(|h| match *h {
            CurveSurfaceHit::Point { point, .. } => Some(point),
            CurveSurfaceHit::Overlap { .. } => None,
        })
        .collect()
}

#[test]
fn line_through_sphere_meets_twice_and_touches_once() {
    let f = tilted(Point3::new(1.0, -2.0, 0.5));
    let s = Surface::Sphere(Sphere::new(f, 2.0).unwrap());
    let rs = Rect::new(iv(0.0, TAU), iv(-FRAC_PI_2, FRAC_PI_2));
    let a = line(f.to_world(Point3::new(-5.0, 1.0, 0.3)), f.x);
    let hits = cross_check(&a, iv(0.0, 10.0), &s, rs);
    assert_eq!(points(&hits).len(), 2);
    let tangent = line(f.to_world(Point3::new(-5.0, 0.0, 2.0)), f.x);
    let hits = cross_check(&tangent, iv(0.0, 10.0), &s, rs);
    let pts = points(&hits);
    assert_eq!(pts.len(), 1);
    assert!((pts[0] - f.to_world(Point3::new(0.0, 0.0, 2.0))).length() < 1e-9);
    let miss = line(f.to_world(Point3::new(-5.0, 0.0, 2.1)), f.x);
    assert!(cross_check(&miss, iv(0.0, 10.0), &s, rs).is_empty());
    // Only the northern half: one of the two crossings.
    let a = line(f.to_world(Point3::new(0.5, 0.2, -5.0)), f.z);
    let north = Rect::new(iv(0.0, TAU), iv(0.0, FRAC_PI_2));
    assert_eq!(cross_check(&a, iv(0.0, 10.0), &s, north).len(), 1);
}

#[test]
fn line_in_plane_overlaps() {
    let f = tilted(Point3::new(0.0, 1.0, 2.0));
    let s = Surface::Plane(Plane::new(f).unwrap());
    let rs = Rect::new(iv(-1.0, 3.0), iv(-2.0, 2.0));
    let a = line(f.to_world(Point3::new(-5.0, 0.5, 0.0)), f.x);
    let hits = cross_check(&a, iv(0.0, 10.0), &s, rs);
    let [CurveSurfaceHit::Overlap { t }] = hits[..] else {
        panic!("{hits:?}");
    };
    assert!((t.min - 4.0).abs() < 1e-12 && (t.max - 8.0).abs() < 1e-12);
    // Crossing the plane at an angle, inside and outside the rectangle.
    let a = line(f.to_world(Point3::new(1.0, 1.0, -1.0)), f.z + f.x * 0.5);
    assert_eq!(points(&cross_check(&a, iv(-3.0, 3.0), &s, rs)).len(), 1);
    let a = line(f.to_world(Point3::new(5.0, 1.0, -1.0)), f.z);
    assert!(cross_check(&a, iv(-3.0, 3.0), &s, rs).is_empty());
    // Parallel above it.
    let a = line(f.to_world(Point3::new(-5.0, 0.5, 0.1)), f.x);
    assert!(cross_check(&a, iv(0.0, 10.0), &s, rs).is_empty());
}

#[test]
fn lines_and_cylinders() {
    let f = tilted(Point3::new(1.0, 1.0, 1.0));
    let s = Surface::Cylinder(Cylinder::new(f, 1.5).unwrap());
    let rs = Rect::new(iv(0.0, PI), iv(-3.0, 3.0));
    // Along a ruling: an overlap clipped to the rectangle's heights.
    let a = line(f.to_world(Point3::new(0.0, 1.5, -5.0)), f.z);
    let hits = cross_check(&a, iv(0.0, 10.0), &s, rs);
    let [CurveSurfaceHit::Overlap { t }] = hits[..] else {
        panic!("{hits:?}");
    };
    assert!((t.min - 2.0).abs() < 1e-12 && (t.max - 8.0).abs() < 1e-12);
    // Across: two crossings, one of them on the half the rectangle keeps.
    let a = line(f.to_world(Point3::new(-5.0, 0.5, 0.2)), f.x);
    assert_eq!(points(&cross_check(&a, iv(0.0, 10.0), &s, rs)).len(), 2);
    let a = line(f.to_world(Point3::new(0.5, -5.0, 0.2)), f.y);
    assert_eq!(points(&cross_check(&a, iv(0.0, 10.0), &s, rs)).len(), 1);
    // Tangent.
    let a = line(f.to_world(Point3::new(-5.0, 1.5, 0.2)), f.x + f.z * 0.1);
    assert_eq!(points(&cross_check(&a, iv(0.0, 10.0), &s, rs)).len(), 1);
}

#[test]
fn lines_and_cones() {
    let f = tilted(Point3::new(0.0, 0.0, 1.0));
    let cone = Cone::new(f, 1.0, FRAC_PI_4).unwrap();
    let s = Surface::Cone(cone);
    let rs = Rect::new(iv(0.0, TAU), iv(-0.5, 2.0));
    let a = line(f.to_world(Point3::new(-5.0, 0.3, 0.5)), f.x);
    assert_eq!(points(&cross_check(&a, iv(0.0, 10.0), &s, rs)).len(), 2);
    // Along a ruling.
    let p = cone.point(1.0, -0.5);
    let q = cone.point(1.0, 2.0);
    let a = line(p - (q - p) * 0.5, q - p);
    let len = (q - p).length();
    let hits = cross_check(&a, iv(0.0, 2.0 * len), &s, rs);
    let [CurveSurfaceHit::Overlap { t }] = hits[..] else {
        panic!("{hits:?}");
    };
    assert!((t.min - 0.5 * len).abs() < 1e-9 && (t.max - 1.5 * len).abs() < 1e-9);
    // Tangent to a circle of the cone, in a plane across the axis.
    let a = line(f.to_world(Point3::new(-5.0, 2.0, 1.0)), f.x);
    assert_eq!(points(&cross_check(&a, iv(0.0, 10.0), &s, rs)).len(), 1);
}

#[test]
fn line_through_torus_meets_four_times() {
    let f = tilted(Point3::new(0.5, 0.0, -1.0));
    let s = Surface::Torus(Torus::new(f, 3.0, 1.0).unwrap());
    let full = Rect::new(iv(0.0, TAU), iv(0.0, TAU));
    let a = line(f.to_world(Point3::new(-6.0, 0.0, 0.0)), f.x);
    let hits = cross_check(&a, iv(0.0, 12.0), &s, full);
    let pts = points(&hits);
    assert_eq!(pts.len(), 4);
    for (p, x) in pts.iter().zip([-4.0, -2.0, 2.0, 4.0]) {
        assert!((*p - f.to_world(Point3::new(x, 0.0, 0.0))).length() < 1e-12);
    }
    // Along the top: two tangential touches.
    let a = line(f.to_world(Point3::new(-6.0, 0.0, 1.0)), f.x);
    let pts = points(&cross_check(&a, iv(0.0, 12.0), &s, full));
    assert_eq!(pts.len(), 2);
    // Through the hole along the axis: nothing.
    let a = line(f.to_world(Point3::new(0.0, 0.0, -5.0)), f.z);
    assert!(cross_check(&a, iv(0.0, 10.0), &s, full).is_empty());
    // A skew line.
    let a = line(f.to_world(Point3::new(-6.0, 1.0, 0.3)), f.x + f.y * 0.1);
    let hits = cross_check(&a, iv(0.0, 12.0), &s, full);
    assert_eq!(points(&hits).len(), 4);
}

#[test]
fn circles_and_planes() {
    let f = tilted(Point3::new(0.0, 0.0, 0.0));
    let s = Surface::Plane(Plane::new(f).unwrap());
    let rs = Rect::new(iv(-5.0, 5.0), iv(-5.0, 5.0));
    // In the plane: the whole circle.
    let c =
        Curve3::Circle(Circle3::new(tilted(f.to_world(Point3::new(1.0, 1.0, 0.0))), 2.0).unwrap());
    let hits = cross_check(&c, iv(0.0, TAU), &s, rs);
    let [CurveSurfaceHit::Overlap { t }] = hits[..] else {
        panic!("{hits:?}");
    };
    assert!(t.min.abs() < 1e-12 && (t.max - TAU).abs() < 1e-12);
    // In the plane, partly outside the rectangle: two arcs, or one.
    let narrow = Rect::new(iv(-5.0, 5.0), iv(0.0, 5.0));
    let hits = cross_check(&c, iv(0.0, TAU), &s, narrow);
    assert!(!hits.is_empty());
    assert!(
        hits.iter()
            .all(|h| matches!(h, CurveSurfaceHit::Overlap { .. }))
    );
    // Crossing the plane, and parallel to it.
    let up = Frame3::from_normal(f.origin, f.x).unwrap();
    let c = Curve3::Circle(Circle3::new(up, 2.0).unwrap());
    assert_eq!(points(&cross_check(&c, iv(0.0, TAU), &s, rs)).len(), 2);
    let above = Frame3 {
        origin: f.to_world(Point3::new(0.0, 0.0, 0.5)),
        ..f
    };
    let c = Curve3::Circle(Circle3::new(above, 2.0).unwrap());
    assert!(cross_check(&c, iv(0.0, TAU), &s, rs).is_empty());
    // Touching it.
    let touch = Frame3 {
        origin: f.to_world(Point3::new(0.0, 0.0, 2.0)),
        ..up
    };
    let c = Curve3::Circle(Circle3::new(touch, 2.0).unwrap());
    assert_eq!(points(&cross_check(&c, iv(0.0, TAU), &s, rs)).len(), 1);
}

#[test]
fn nurbs_curve_through_nurbs_surface() {
    // A bicubic patch z = 0.3 sin(x) cos(y) over [-3, 3]^2.
    let n: u32 = 6;
    let grid: Vec<Point3> = (0..n)
        .flat_map(|i| {
            (0..n).map(move |j| {
                let x = -3.0 + 6.0 * f64::from(i) / f64::from(n - 1);
                let y = -3.0 + 6.0 * f64::from(j) / f64::from(n - 1);
                Point3::new(x, y, 0.3 * x.sin() * y.cos())
            })
        })
        .collect();
    let knots = KnotVector::clamped_uniform(3, n as usize).unwrap();
    let surface = NurbsSurface::from_points(3, 3, knots.clone(), knots, &grid, None).unwrap();
    let s = Surface::Nurbs(surface.clone());
    let rs = Rect::new(iv(0.0, 1.0), iv(0.0, 1.0));
    // A wavy cubic crossing the patch several times.
    let c = Curve3::Nurbs(
        NurbsCurve3::bezier(&[
            Point3::new(-2.5, -2.0, -1.0),
            Point3::new(-1.0, -0.5, 3.0),
            Point3::new(1.0, 0.5, -3.0),
            Point3::new(2.5, 2.0, 1.0),
        ])
        .unwrap(),
    );
    let rc = iv(0.0, 1.0);
    let hits = curve_surface(&c, rc, &s, rs, TOL).unwrap();
    check_hits_cs(&c, rc, &s, rs, &hits, TOL);
    // Count the crossings from the signed height above the patch.
    let height = |t: f64| {
        let p = c.point(t);
        let cp = surface.closest_point(p).unwrap();
        let n = surface.normal(cp.u, cp.v).unwrap();
        (p - cp.point).dot(n)
    };
    let mut want = 0;
    let mut prev = height(0.0);
    for k in 1..=2000 {
        let h = height(f64::from(k) / 2000.0);
        if (h > 0.0) != (prev > 0.0) {
            want += 1;
        }
        prev = h;
    }
    assert!(want >= 3, "{want}");
    assert_eq!(points(&hits).len(), want, "{hits:?}");
    for h in &hits {
        let CurveSurfaceHit::Point { t, u, v, .. } = *h else {
            panic!("{h:?}");
        };
        assert!((c.point(t) - s.point(u, v)).length() < 1e-12);
    }
}

#[test]
fn spots_agree_for_a_line_through_a_nurbs_sphere() {
    let f = tilted(Point3::new(0.0, 0.0, 0.0));
    let s = Surface::Sphere(Sphere::new(f, 1.0).unwrap());
    let (ns, nrs) = nurbs_surface(&s, Rect::new(iv(0.0, TAU), iv(-FRAC_PI_2, FRAC_PI_2)));
    // Through both poles.
    let a = line(f.to_world(Point3::new(0.0, 0.0, -2.0)), f.z);
    let hits = curve_surface(&a, iv(0.0, 4.0), &ns, nrs, TOL).unwrap();
    assert_same_spots(
        &spots_cs(&a, &hits),
        &[
            Spot::Point(f.to_world(Point3::new(0.0, 0.0, -1.0))),
            Spot::Point(f.to_world(Point3::new(0.0, 0.0, 1.0))),
        ],
        1e-6,
    );
}

#[test]
fn bad_input_is_rejected() {
    let s = Surface::Plane(Plane::new(Frame3::WORLD).unwrap());
    let a = line(Point3::ZERO, Vector3::Z);
    let r = Rect::new(iv(-1.0, 1.0), iv(-1.0, 1.0));
    assert_eq!(
        curve_surface(&a, iv(-1.0, 1.0), &s, r, f64::NAN),
        Err(GeomError::NonFinite)
    );
    assert!(matches!(
        curve_surface(&a, iv(-1.0, 1.0), &s, r, 0.0),
        Err(GeomError::InvalidArgument(_))
    ));
    let bad = Rect::new(iv(-1.0, f64::INFINITY), iv(-1.0, 1.0));
    assert_eq!(
        curve_surface(&a, iv(-1.0, 1.0), &s, bad, TOL),
        Err(GeomError::NonFinite)
    );
    let sphere = Surface::Sphere(Sphere::new(Frame3::WORLD, 1.0).unwrap());
    let beyond = Rect::new(iv(0.0, TAU), iv(0.0, 2.0));
    assert!(matches!(
        curve_surface(&a, iv(-1.0, 1.0), &sphere, beyond, TOL),
        Err(GeomError::ParameterOutOfDomain { .. })
    ));
}
