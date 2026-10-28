// SPDX-License-Identifier: GPL-3.0-or-later

//! Surface–surface intersection: the general method on analytic and NURBS
//! surfaces, checked against closed-form answers.

#![allow(clippy::unwrap_used, clippy::panic, missing_docs, reason = "tests")]

use bevel_geom::intersect::{Rect, SsiCurve, SsiResult, surface_surface_general};
use bevel_geom::{CurveGeom, Cylinder, Plane, Sphere, Surface, SurfaceGeom};
use bevel_math::{Frame3, Interval, Point3, Vector3};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

const TOL: f64 = 1e-6;

fn plane_at(z: f64) -> Surface {
    Surface::Plane(
        Plane::new(Frame3 {
            origin: Point3::new(0.0, 0.0, z),
            ..Frame3::WORLD
        })
        .unwrap(),
    )
}

fn sphere(r: f64) -> Surface {
    Surface::Sphere(Sphere::new(Frame3::WORLD, r).unwrap())
}

fn square(half: f64) -> Rect {
    Rect::new(Interval::new(-half, half), Interval::new(-half, half))
}

fn whole_sphere() -> Rect {
    Rect::new(
        Interval::new(0.0, TAU),
        Interval::new(-FRAC_PI_2, FRAC_PI_2),
    )
}

/// The curves of a result.
fn curves(r: SsiResult) -> Vec<SsiCurve> {
    match r {
        SsiResult::Meet { curves, .. } => curves,
        SsiResult::Coincident { .. } => panic!("unexpected coincidence"),
    }
}

/// Checks every curve against both surfaces at many parameters, and
/// returns the total length.
fn check(curves: &[SsiCurve], a: &Surface, b: &Surface) -> f64 {
    let mut length = 0.0;
    for c in curves {
        assert!(c.tolerance <= TOL, "tolerance {}", c.tolerance);
        let n = 400;
        let mut prev: Option<Point3> = None;
        for i in 0..=n {
            let t = c.range.min + c.range.length() * f64::from(i) / f64::from(n);
            let p = c.curve.point(t);
            let pa = c.pcurve_a.point(t);
            let pb = c.pcurve_b.point(t);
            let on_a = (a.point(pa.x, pa.y) - p).length();
            let on_b = (b.point(pb.x, pb.y) - p).length();
            assert!(
                on_a <= 2.0 * TOL && on_b <= 2.0 * TOL,
                "t {t}: {on_a} {on_b}"
            );
            let ca = a.closest_point(p).unwrap().distance;
            let cb = b.closest_point(p).unwrap().distance;
            assert!(
                ca <= 2.0 * TOL && cb <= 2.0 * TOL,
                "t {t}: off the surfaces {ca} {cb}"
            );
            if let Some(q) = prev {
                length += (p - q).length();
            }
            prev = Some(p);
        }
    }
    length
}

#[test]
fn plane_cuts_sphere_in_a_circle() {
    let (a, b) = (plane_at(3.0), sphere(10.0));
    let r = surface_surface_general(&a, square(20.0), &b, whole_sphere(), TOL).unwrap();
    let curves = curves(r);
    let length = check(&curves, &a, &b);
    let expected = TAU * (100.0f64 - 9.0).sqrt();
    assert!((length - expected).abs() < 1e-3, "{length} vs {expected}");
}

#[test]
fn perpendicular_cylinders_meet_in_two_loops() {
    let small = Surface::Cylinder(Cylinder::new(Frame3::WORLD, 6.0).unwrap());
    let big_frame = Frame3::from_axes(Point3::ZERO, Vector3::Y, Vector3::Z).unwrap();
    let big = Surface::Cylinder(
        Cylinder::new(
            Frame3 {
                z: Vector3::X,
                ..big_frame
            },
            10.0,
        )
        .unwrap(),
    );
    let ra = Rect::new(Interval::new(0.0, TAU), Interval::new(-20.0, 20.0));
    let rb = Rect::new(Interval::new(0.0, TAU), Interval::new(-20.0, 20.0));
    let curves = curves(surface_surface_general(&small, ra, &big, rb, TOL).unwrap());
    let length = check(&curves, &small, &big);
    // Each loop: (6 cos t, 6 sin t, +-sqrt(100 - 36 sin^2 t)).
    let n = 200_000;
    let mut expected = 0.0;
    for i in 0..n {
        let t = TAU * f64::from(i) / f64::from(n);
        let s = (100.0 - 36.0 * t.sin().powi(2)).sqrt();
        let dz = -36.0 * t.sin() * t.cos() / s;
        expected += (36.0 + dz * dz).sqrt() * TAU / f64::from(n);
    }
    expected *= 2.0;
    assert!((length - expected).abs() < 1e-3, "{length} vs {expected}");
}

#[test]
fn nurbs_forms_give_the_same_circle() {
    let (a, b) = (plane_at(-4.0), sphere(10.0));
    let na = Surface::Nurbs(
        a.to_nurbs(Interval::new(-20.0, 20.0), Interval::new(-20.0, 20.0))
            .unwrap(),
    );
    let nb = Surface::Nurbs(
        b.to_nurbs(
            Interval::new(0.0, TAU),
            Interval::new(-FRAC_PI_2, FRAC_PI_2),
        )
        .unwrap(),
    );
    let ra = Rect::new(
        na.domain_u().interval().unwrap(),
        na.domain_v().interval().unwrap(),
    );
    let rb = Rect::new(
        nb.domain_u().interval().unwrap(),
        nb.domain_v().interval().unwrap(),
    );
    let curves = curves(surface_surface_general(&na, ra, &nb, rb, TOL).unwrap());
    let length = check(&curves, &na, &nb);
    let expected = TAU * (100.0f64 - 16.0).sqrt();
    assert!((length - expected).abs() < 1e-3, "{length} vs {expected}");
}

#[test]
fn rectangles_clip_the_curves() {
    // Only the quarter of the plane with x, y >= 0: a quarter circle.
    let (a, b) = (plane_at(0.0), sphere(10.0));
    let ra = Rect::new(Interval::new(0.0, 20.0), Interval::new(0.0, 20.0));
    let curves = curves(surface_surface_general(&a, ra, &b, whole_sphere(), TOL).unwrap());
    let length = check(&curves, &a, &b);
    assert!((length - 10.0 * PI / 2.0).abs() < 1e-3, "{length}");
}

#[test]
fn apart_coincident_and_touching() {
    let (a, b) = (plane_at(30.0), sphere(10.0));
    let r = surface_surface_general(&a, square(20.0), &b, whole_sphere(), TOL).unwrap();
    assert!(curves(r).is_empty());

    let r = surface_surface_general(
        &plane_at(1.0),
        square(5.0),
        &plane_at(1.0),
        square(8.0),
        TOL,
    )
    .unwrap();
    assert_eq!(r, SsiResult::Coincident { same_sense: true });

    // A plane touching the sphere at (0, 10, 0), away from its poles and seam.
    let side = Surface::Plane(
        Plane::new(Frame3::from_normal(Point3::new(0.0, 10.0, 0.0), Vector3::Y).unwrap()).unwrap(),
    );
    let r = surface_surface_general(&side, square(20.0), &b, whole_sphere(), TOL).unwrap();
    match r {
        SsiResult::Meet { curves, points } => {
            assert!(curves.is_empty(), "{curves:?}");
            assert_eq!(points.len(), 1);
            assert!((points[0].point - Point3::new(0.0, 10.0, 0.0)).length() < 1e-3);
        }
        SsiResult::Coincident { .. } => panic!("not coincident"),
    }
}

#[test]
fn bad_arguments_are_refused() {
    let (a, b) = (plane_at(0.0), sphere(1.0));
    assert!(surface_surface_general(&a, square(1.0), &b, whole_sphere(), 0.0).is_err());
    assert!(surface_surface_general(&a, square(1.0), &b, whole_sphere(), f64::NAN).is_err());
}
