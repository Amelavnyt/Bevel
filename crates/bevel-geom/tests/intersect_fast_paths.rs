// SPDX-License-Identifier: GPL-3.0-or-later

//! Exact surface–surface fast paths, cross-checked against the general
//! method, and the tangent and coincident cases only they decide.

#![allow(clippy::unwrap_used, clippy::panic, missing_docs, reason = "tests")]

use bevel_geom::intersect::{
    Contact, Rect, SsiCurve, SsiResult, surface_surface, surface_surface_general,
};
use bevel_geom::{Cone, Curve3, CurveGeom, Cylinder, Plane, Sphere, Surface, SurfaceGeom, Torus};
use bevel_math::{Frame3, Interval, Point3, Vector3};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, TAU};

const TOL: f64 = 1e-6;

fn frame(origin: Point3, normal: Vector3) -> Frame3 {
    Frame3::from_normal(origin, normal).unwrap()
}

fn plane(origin: Point3, normal: Vector3) -> (Surface, Rect) {
    let s = Surface::Plane(Plane::new(frame(origin, normal)).unwrap());
    (
        s,
        Rect::new(Interval::new(-30.0, 30.0), Interval::new(-30.0, 30.0)),
    )
}

fn sphere(centre: Point3, r: f64) -> (Surface, Rect) {
    let s = Surface::Sphere(Sphere::new(frame(centre, Vector3::Z), r).unwrap());
    (
        s,
        Rect::new(
            Interval::new(0.0, TAU),
            Interval::new(-FRAC_PI_2, FRAC_PI_2),
        ),
    )
}

fn cylinder(origin: Point3, axis: Vector3, r: f64) -> (Surface, Rect) {
    let s = Surface::Cylinder(Cylinder::new(frame(origin, axis), r).unwrap());
    (
        s,
        Rect::new(Interval::new(0.0, TAU), Interval::new(-30.0, 30.0)),
    )
}

fn split(r: SsiResult) -> (Vec<SsiCurve>, usize) {
    match r {
        SsiResult::Meet { curves, points } => (curves, points.len()),
        SsiResult::Coincident { .. } => panic!("unexpected coincidence"),
    }
}

/// Checks every curve lies on both surfaces with matching pcurves; returns
/// the total length.
fn check(curves: &[SsiCurve], a: &Surface, b: &Surface) -> f64 {
    let mut length = 0.0;
    for c in curves {
        assert!(c.tolerance <= TOL, "tolerance {}", c.tolerance);
        let n = 600;
        let mut prev: Option<Point3> = None;
        for i in 0..=n {
            let t = c.range.min + c.range.length() * f64::from(i) / f64::from(n);
            let p = c.curve.point(t);
            let (pa, pb) = (c.pcurve_a.point(t), c.pcurve_b.point(t));
            let ea = (a.point(pa.x, pa.y) - p).length();
            let eb = (b.point(pb.x, pb.y) - p).length();
            assert!(
                ea <= 2.0 * TOL && eb <= 2.0 * TOL,
                "pcurves off at t = {t}: {ea} {eb}"
            );
            if let Some(q) = prev {
                length += (p - q).length();
            }
            prev = Some(p);
        }
    }
    length
}

/// Runs the fast path and the general method; both must be valid and agree
/// on the total length. Returns the fast path's curves.
fn compare(a: &(Surface, Rect), b: &(Surface, Rect)) -> Vec<SsiCurve> {
    let (fast, _) = split(surface_surface(&a.0, a.1, &b.0, b.1, TOL).unwrap());
    let (general, _) = split(surface_surface_general(&a.0, a.1, &b.0, b.1, TOL).unwrap());
    let lf = check(&fast, &a.0, &b.0);
    let lg = check(&general, &a.0, &b.0);
    assert!((lf - lg).abs() <= 1e-3, "fast {lf} vs general {lg}");
    assert!(
        fast.iter().all(|c| c.curve.is_analytic()),
        "fast paths give exact curves"
    );
    fast
}

fn kinds(curves: &[SsiCurve]) -> (usize, usize, usize) {
    let count = |f: fn(&Curve3) -> bool| curves.iter().filter(|c| f(&c.curve)).count();
    (
        count(|c| matches!(c, Curve3::Line(_))),
        count(|c| matches!(c, Curve3::Circle(_))),
        count(|c| matches!(c, Curve3::Ellipse(_))),
    )
}

#[test]
fn plane_plane_is_a_clipped_line() {
    let a = plane(Point3::ZERO, Vector3::Z);
    let b = plane(Point3::new(5.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 0.0));
    let curves = compare(&a, &b);
    assert_eq!(kinds(&curves), (1, 0, 0));
}

#[test]
fn plane_sphere_and_sphere_sphere_are_circles() {
    let s = sphere(Point3::new(1.0, 2.0, 3.0), 10.0);
    let p = plane(Point3::new(0.0, 0.0, 6.0), Vector3::new(0.2, 0.1, 1.0));
    let curves = compare(&p, &s);
    assert!(curves.iter().all(|c| matches!(c.curve, Curve3::Circle(_))));
    // Not 7: that circle passes exactly through the second sphere's pole,
    // which the general method cannot fit yet (docs/KERNEL.md).
    let t = sphere(Point3::new(9.0, 2.0, 4.0), 7.5);
    let curves = compare(&s, &t);
    assert!(curves.iter().all(|c| matches!(c.curve, Curve3::Circle(_))));
}

#[test]
fn plane_cylinder_gives_circle_ellipse_or_rulings() {
    let c = cylinder(Point3::ZERO, Vector3::Z, 5.0);
    let flat = plane(Point3::new(0.0, 0.0, 2.0), Vector3::Z);
    assert!(kinds(&compare(&flat, &c)).1 > 0);
    let slant = plane(Point3::new(0.0, 0.0, 2.0), Vector3::new(0.0, 0.5, 1.0));
    assert!(kinds(&compare(&slant, &c)).2 > 0);
    let along = plane(Point3::new(3.0, 0.0, 0.0), Vector3::X);
    assert_eq!(kinds(&compare(&along, &c)).0, 2);
}

#[test]
fn parallel_and_coaxial_pairs() {
    let c = cylinder(Point3::ZERO, Vector3::Z, 5.0);
    let d = cylinder(Point3::new(6.0, 1.0, 0.0), Vector3::Z, 4.0);
    assert_eq!(kinds(&compare(&c, &d)).0, 2);
    let s = sphere(Point3::new(0.0, 0.0, 1.0), 8.0);
    let curves = compare(&c, &s);
    assert_eq!(kinds(&curves).1, 2);
    let cone = (
        Surface::Cone(Cone::new(frame(Point3::ZERO, Vector3::Z), 3.0, FRAC_PI_4 / 2.0).unwrap()),
        Rect::new(Interval::new(0.0, TAU), Interval::new(-5.0, 20.0)),
    );
    assert!(
        kinds(&compare(
            &cone,
            &plane(Point3::new(0.0, 0.0, 4.0), Vector3::Z)
        ))
        .1 > 0
    );
    assert!(kinds(&compare(&cone, &s)).1 > 0);
    let torus = (
        Surface::Torus(Torus::new(frame(Point3::ZERO, Vector3::Z), 10.0, 3.0).unwrap()),
        Rect::new(Interval::new(0.0, TAU), Interval::new(0.0, TAU)),
    );
    assert_eq!(
        kinds(&compare(
            &torus,
            &plane(Point3::new(0.0, 0.0, 1.0), Vector3::Z)
        ))
        .1,
        2
    );
}

#[test]
fn tangent_contacts_are_explicit() {
    let s = sphere(Point3::ZERO, 10.0);
    let (curves, points) = split(
        surface_surface(
            &s.0,
            s.1,
            &plane(Point3::new(0.0, 0.0, 10.0), Vector3::Z).0,
            plane(Point3::ZERO, Vector3::Z).1,
            TOL,
        )
        .unwrap(),
    );
    assert!(curves.is_empty());
    assert_eq!(points, 1, "a plane touching a sphere");

    let t = sphere(Point3::new(25.0, 0.0, 0.0), 15.0);
    let (curves, points) = split(surface_surface(&s.0, s.1, &t.0, t.1, TOL).unwrap());
    assert!(curves.is_empty());
    assert_eq!(points, 1, "spheres touching outside");

    let c = cylinder(Point3::ZERO, Vector3::Z, 5.0);
    let p = plane(Point3::new(5.0, 0.0, 0.0), Vector3::X);
    let (curves, _) = split(surface_surface(&c.0, c.1, &p.0, p.1, TOL).unwrap());
    assert_eq!(curves.len(), 1);
    assert_eq!(curves[0].contact, Contact::Tangent);

    let inside = sphere(Point3::new(0.0, 0.0, 3.0), 5.0);
    let (curves, _) = split(surface_surface(&c.0, c.1, &inside.0, inside.1, TOL).unwrap());
    assert!(!curves.is_empty());
    assert!(
        curves.iter().all(|c| c.contact == Contact::Tangent),
        "a sphere inscribed in a cylinder"
    );
}

#[test]
fn coincident_surfaces_are_recognised() {
    let s = sphere(Point3::new(1.0, 1.0, 1.0), 4.0);
    let r = surface_surface(&s.0, s.1, &s.0, s.1, TOL).unwrap();
    assert_eq!(r, SsiResult::Coincident { same_sense: true });
    let up = plane(Point3::ZERO, Vector3::Z);
    let down = plane(Point3::new(3.0, 1.0, 0.0), -Vector3::Z);
    let r = surface_surface(&up.0, up.1, &down.0, down.1, TOL).unwrap();
    assert_eq!(r, SsiResult::Coincident { same_sense: false });
    let c = cylinder(Point3::ZERO, Vector3::Z, 5.0);
    let d = cylinder(Point3::new(0.0, 0.0, 7.0), Vector3::Z, 5.0);
    let r = surface_surface(&c.0, c.1, &d.0, d.1, TOL).unwrap();
    assert_eq!(r, SsiResult::Coincident { same_sense: true });
}
