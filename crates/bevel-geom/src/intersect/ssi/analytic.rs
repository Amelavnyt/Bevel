// SPDX-License-Identifier: GPL-3.0-or-later

//! Exact intersections of analytic surfaces.
//!
//! - **Plane–plane**: a line, or nothing, or coincidence.
//! - **Coaxial surfaces of revolution**: spheres, cylinders, cones, tori
//!   and planes perpendicular to the axis, sharing an axis (a sphere shares
//!   any axis through its centre, so plane–sphere and sphere–sphere belong
//!   here too). Their profiles in the half-plane through the axis are lines
//!   and circles; each point where the profiles meet sweeps a circle about
//!   the axis, and a point where they touch sweeps a tangent circle.
//! - **Plane–cylinder**: two rulings, a tangent ruling or nothing when the
//!   axis is parallel to the plane; an ellipse when it is oblique.
//! - **Parallel cylinders**: rulings through the points where their cross
//!   sections meet.
//!
//! Every other pair goes to the general method.

use super::super::Contact;
use crate::{Circle3, Curve3, Ellipse3, GeomError, Line3, Surface};
use bevel_math::{Aabb3, Frame3, Interval, Point3, Vector3};
use std::f64::consts::TAU;

/// Below this sine two directions count as parallel.
const PARALLEL: f64 = 1e-10;

/// What an exact method found, before clipping to the rectangles.
pub(super) enum Exact {
    /// Curves over parameter ranges, with how the surfaces meet on them.
    Curves(Vec<(Curve3, Interval, Contact)>, Vec<Point3>),
    /// The surfaces coincide; whether their normals agree.
    Coincident(bool),
}

/// The exact intersection of `a` and `b`, if their pair has one. `boxes`
/// bounds both patches; lines are cut to it.
pub(super) fn exact(
    a: &Surface,
    b: &Surface,
    boxes: &Aabb3,
    tol: f64,
) -> Option<Result<Exact, GeomError>> {
    if let (Surface::Plane(p), Surface::Plane(q)) = (a, b) {
        return Some(plane_plane(p.frame(), q.frame(), boxes, tol));
    }
    if let Some(r) = coaxial(a, b, tol) {
        return Some(r);
    }
    match (a, b) {
        (Surface::Plane(p), Surface::Cylinder(c)) | (Surface::Cylinder(c), Surface::Plane(p)) => {
            Some(plane_cylinder(p.frame(), c.frame(), c.radius(), boxes, tol))
        }
        (Surface::Cylinder(c), Surface::Cylinder(d)) => {
            parallel_cylinders(c.frame(), c.radius(), d.frame(), d.radius(), boxes, tol)
        }
        _ => None,
    }
}

/// The range of the line `o + t d` inside `boxes` (grown by `tol`), if it
/// passes through.
fn line_range(o: Point3, d: Vector3, boxes: &Aabb3, tol: f64) -> Option<Interval> {
    let (min, max) = (
        boxes.min - Vector3::splat(tol),
        boxes.max + Vector3::splat(tol),
    );
    let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
    for axis in 0..3 {
        let (oa, da) = (o[axis], d[axis]);
        if da.abs() <= 1e-300 {
            if oa < min[axis] || oa > max[axis] {
                return None;
            }
            continue;
        }
        let (t0, t1) = ((min[axis] - oa) / da, (max[axis] - oa) / da);
        lo = lo.max(t0.min(t1));
        hi = hi.min(t0.max(t1));
    }
    (hi > lo).then(|| Interval::new(lo, hi))
}

fn line(
    o: Point3,
    d: Vector3,
    boxes: &Aabb3,
    tol: f64,
    contact: Contact,
) -> Result<Option<(Curve3, Interval, Contact)>, GeomError> {
    let l = Line3::new(o, d)?;
    Ok(line_range(l.origin(), l.direction(), boxes, tol).map(|r| (Curve3::Line(l), r, contact)))
}

fn plane_plane(p: &Frame3, q: &Frame3, boxes: &Aabb3, tol: f64) -> Result<Exact, GeomError> {
    let d = p.z.cross(q.z);
    if d.length() <= PARALLEL {
        let gap = (q.origin - p.origin).dot(p.z);
        return Ok(if gap.abs() <= tol {
            Exact::Coincident(p.z.dot(q.z) > 0.0)
        } else {
            Exact::Curves(Vec::new(), Vec::new())
        });
    }
    // The point on the line nearest the origin of `p`: solve n1.x = c1,
    // n2.x = c2, d.x = d.o1.
    let (n1, n2) = (p.z, q.z);
    let (c1, c2) = (n1.dot(p.origin), n2.dot(q.origin));
    let d2 = d.length_squared();
    let o = (n2.cross(d) * c1 + d.cross(n1) * c2) / d2;
    let o = o + d * (d.dot(p.origin - o) / d2);
    Ok(Exact::Curves(
        line(o, d, boxes, tol, Contact::Transversal)?
            .into_iter()
            .collect(),
        Vec::new(),
    ))
}

// ---------------------------------------------------------------- coaxial

/// A surface of revolution about an axis: its profile in the half-plane
/// `(rho, z)` through the axis, with `z` measured along the axis direction
/// from the axis origin. Profiles are mirrored into `rho < 0`, where
/// solutions are discarded.
#[derive(Clone, Copy, Debug)]
enum Profile {
    /// `rho = r` (a cylinder; both mirror lines are listed).
    Line { p: [f64; 2], d: [f64; 2] },
    /// A circle (sphere: centred on the axis; torus: off it).
    Circle { c: [f64; 2], r: f64 },
}

/// What a surface needs of an axis to be a surface of revolution about it.
enum Axis {
    /// It has one axis (cylinder, cone, torus).
    Fixed(Point3, Vector3),
    /// Any axis along this direction (a plane).
    Direction(Vector3),
    /// Any axis through this point (a sphere).
    Through(Point3),
}

fn axis_of(s: &Surface) -> Option<Axis> {
    match s {
        Surface::Plane(p) => Some(Axis::Direction(p.frame().z)),
        Surface::Sphere(s) => Some(Axis::Through(s.center())),
        Surface::Cylinder(c) => Some(Axis::Fixed(c.frame().origin, c.frame().z)),
        Surface::Cone(c) => Some(Axis::Fixed(c.frame().origin, c.frame().z)),
        Surface::Torus(t) => Some(Axis::Fixed(t.frame().origin, t.frame().z)),
        Surface::Nurbs(_) => None,
    }
}

fn on_line(p: Point3, o: Point3, d: Vector3, tol: f64) -> bool {
    let v = p - o;
    (v - d * v.dot(d)).length() <= tol
}

/// A common axis of the two surfaces, if they share one.
fn common_axis(a: &Axis, b: &Axis, tol: f64) -> Option<(Point3, Vector3)> {
    use Axis::{Direction, Fixed, Through};
    let parallel = |u: Vector3, v: Vector3| u.cross(v).length() <= PARALLEL;
    match (a, b) {
        (Fixed(o, d), Fixed(p, e)) => {
            (parallel(*d, *e) && on_line(*p, *o, *d, tol)).then_some((*o, *d))
        }
        (Fixed(o, d), Direction(e)) | (Direction(e), Fixed(o, d)) => {
            parallel(*d, *e).then_some((*o, *d))
        }
        (Fixed(o, d), Through(c)) | (Through(c), Fixed(o, d)) => {
            on_line(*c, *o, *d, tol).then_some((*o, *d))
        }
        (Through(c), Direction(e)) | (Direction(e), Through(c)) => Some((*c, *e)),
        (Through(c), Through(q)) => {
            let d = *q - *c;
            (d.length() > tol).then(|| (*c, d / d.length()))
        }
        (Direction(_), Direction(_)) => None,
    }
}

/// The profiles of `s` about the axis `(o, d)`.
fn profiles(s: &Surface, o: Point3, d: Vector3) -> Vec<Profile> {
    let z_of = |p: Point3| (p - o).dot(d);
    match s {
        Surface::Plane(p) => vec![Profile::Line {
            p: [0.0, z_of(p.origin())],
            d: [1.0, 0.0],
        }],
        Surface::Sphere(s) => vec![Profile::Circle {
            c: [0.0, z_of(s.center())],
            r: s.radius(),
        }],
        Surface::Cylinder(c) => vec![
            Profile::Line {
                p: [c.radius(), 0.0],
                d: [0.0, 1.0],
            },
            Profile::Line {
                p: [-c.radius(), 0.0],
                d: [0.0, 1.0],
            },
        ],
        Surface::Cone(c) => {
            // rho = r0 + v tan(alpha) with v along the cone's own z.
            let sign = c.frame().z.dot(d).signum();
            let z0 = z_of(c.frame().origin);
            let t = c.half_angle().tan() * sign;
            let line = |s: f64| Profile::Line {
                p: [s * c.radius(), z0],
                d: [s * t, 1.0],
            };
            vec![line(1.0), line(-1.0)]
        }
        Surface::Torus(t) => {
            let zc = z_of(t.frame().origin);
            vec![
                Profile::Circle {
                    c: [t.major_radius(), zc],
                    r: t.minor_radius(),
                },
                Profile::Circle {
                    c: [-t.major_radius(), zc],
                    r: t.minor_radius(),
                },
            ]
        }
        Surface::Nurbs(_) => Vec::new(),
    }
}

/// Where two profiles meet: points `(rho, z)` and whether they touch there.
/// `None` when they coincide.
fn meet(p: &Profile, q: &Profile, tol: f64) -> Option<Vec<([f64; 2], bool)>> {
    use Profile::{Circle, Line};
    let sub = |a: [f64; 2], b: [f64; 2]| [a[0] - b[0], a[1] - b[1]];
    let dot = |a: [f64; 2], b: [f64; 2]| a[0] * b[0] + a[1] * b[1];
    let cross = |a: [f64; 2], b: [f64; 2]| a[0] * b[1] - a[1] * b[0];
    let len = |a: [f64; 2]| dot(a, a).sqrt();
    let unit = |a: [f64; 2]| {
        let l = len(a);
        [a[0] / l, a[1] / l]
    };
    let at = |p: [f64; 2], d: [f64; 2], t: f64| [p[0] + d[0] * t, p[1] + d[1] * t];
    let line_circle = |p: [f64; 2], d: [f64; 2], c: [f64; 2], r: f64| {
        let d = unit(d);
        let t0 = dot(sub(c, p), d);
        let foot = at(p, d, t0);
        let dist = len(sub(c, foot));
        if dist > r + tol {
            Vec::new()
        } else if (dist - r).abs() <= tol {
            vec![(foot, true)]
        } else {
            let h = (r * r - dist * dist).max(0.0).sqrt();
            vec![(at(p, d, t0 - h), false), (at(p, d, t0 + h), false)]
        }
    };
    match (*p, *q) {
        (Line { p: p1, d: d1 }, Line { p: p2, d: d2 }) => {
            let (d1, d2) = (unit(d1), unit(d2));
            let c = cross(d1, d2);
            if c.abs() <= PARALLEL {
                // Parallel: the same line, or apart.
                let gap = cross(sub(p2, p1), d1).abs();
                return if gap <= tol { None } else { Some(Vec::new()) };
            }
            let t = cross(sub(p2, p1), d2) / c;
            Some(vec![(at(p1, d1, t), false)])
        }
        (Line { p, d }, Circle { c, r }) | (Circle { c, r }, Line { p, d }) => {
            Some(line_circle(p, d, c, r))
        }
        (Circle { c: c1, r: r1 }, Circle { c: c2, r: r2 }) => {
            let v = sub(c2, c1);
            let dist = len(v);
            if dist <= tol && (r1 - r2).abs() <= tol {
                return None;
            }
            if dist <= tol || dist > r1 + r2 + tol || dist < (r1 - r2).abs() - tol {
                return Some(Vec::new());
            }
            let e = unit(v);
            if (dist - (r1 + r2)).abs() <= tol || (dist - (r1 - r2).abs()).abs() <= tol {
                // Touching, outside or inside.
                let s = if dist - (r1 - r2).abs() <= tol && r2 > r1 {
                    -r1
                } else {
                    r1
                };
                return Some(vec![(at(c1, e, s), true)]);
            }
            let x = (dist * dist + r1 * r1 - r2 * r2) / (2.0 * dist);
            let h = (r1 * r1 - x * x).max(0.0).sqrt();
            let m = at(c1, e, x);
            let n = [-e[1], e[0]];
            Some(vec![(at(m, n, h), false), (at(m, n, -h), false)])
        }
    }
}

/// Exact intersection of two surfaces of revolution about a common axis.
fn coaxial(a: &Surface, b: &Surface, tol: f64) -> Option<Result<Exact, GeomError>> {
    let (o, d) = common_axis(&axis_of(a)?, &axis_of(b)?, tol)?;
    let (pa, pb) = (profiles(a, o, d), profiles(b, o, d));
    if pa.is_empty() || pb.is_empty() {
        return None;
    }
    let mut circles: Vec<(f64, f64, bool)> = Vec::new();
    for p in &pa {
        for q in &pb {
            let Some(hits) = meet(p, q, tol) else {
                // The profiles coincide: so do the surfaces. Compare the
                // normals at a point of `a` with those of `b` there.
                return Some(Ok(Exact::Coincident(same_sense(a, b))));
            };
            for ([rho, z], touch) in hits {
                if rho < -tol {
                    continue;
                }
                let rho = rho.max(0.0);
                if !circles
                    .iter()
                    .any(|&(r, h, _)| (r - rho).abs() <= tol && (h - z).abs() <= tol)
                {
                    circles.push((rho, z, touch));
                }
            }
        }
    }
    circles.sort_by(|x, y| x.1.total_cmp(&y.1).then(x.0.total_cmp(&y.0)));
    let mut curves = Vec::new();
    let mut points = Vec::new();
    for (rho, z, touch) in circles {
        let centre = o + d * z;
        if rho <= tol {
            points.push(centre);
            continue;
        }
        let Ok(frame) = Frame3::from_normal(centre, d) else {
            return Some(Err(GeomError::InvalidArgument("a degenerate axis")));
        };
        let circle = match Circle3::new(frame, rho) {
            Ok(c) => c,
            Err(e) => return Some(Err(e)),
        };
        let contact = if touch {
            Contact::Tangent
        } else {
            Contact::Transversal
        };
        curves.push((Curve3::Circle(circle), Interval::new(0.0, TAU), contact));
    }
    Some(Ok(Exact::Curves(curves, points)))
}

/// Whether the normals of two coincident surfaces agree, compared at the
/// point of `b` nearest a point of `a`.
fn same_sense(a: &Surface, b: &Surface) -> bool {
    use crate::SurfaceGeom as _;
    let (u, v) = (0.3, 0.2);
    let p = a.point(u, v);
    match (
        a.normal(u, v),
        b.closest_point(p).ok().and_then(|c| b.normal(c.u, c.v)),
    ) {
        (Some(na), Some(nb)) => na.dot(nb) > 0.0,
        _ => true,
    }
}

// ------------------------------------------------------------ cylinders

fn plane_cylinder(
    p: &Frame3,
    c: &Frame3,
    r: f64,
    boxes: &Aabb3,
    tol: f64,
) -> Result<Exact, GeomError> {
    let (n, a) = (p.z, c.z);
    let cos = a.dot(n);
    if cos.abs() <= PARALLEL {
        // The axis is parallel to the plane.
        let dist = (c.origin - p.origin).dot(n);
        let foot = c.origin - n * dist;
        let mut curves = Vec::new();
        if (dist.abs() - r).abs() <= tol {
            curves.extend(line(foot, a, boxes, tol, Contact::Tangent)?);
        } else if dist.abs() < r {
            let w = a.cross(n);
            let h = (r * r - dist * dist).sqrt();
            curves.extend(line(foot - w * h, a, boxes, tol, Contact::Transversal)?);
            curves.extend(line(foot + w * h, a, boxes, tol, Contact::Transversal)?);
        }
        return Ok(Exact::Curves(curves, Vec::new()));
    }
    // Oblique (perpendicular is coaxial): an ellipse centred where the axis
    // meets the plane, minor radius r across, r / |cos| along the slope.
    let centre = c.origin + a * ((p.origin - c.origin).dot(n) / cos);
    let w = a.cross(n);
    let w = w / w.length();
    let m = n.cross(w);
    let frame = Frame3 {
        origin: centre,
        x: m,
        y: n.cross(m),
        z: n,
    };
    let ellipse = Ellipse3::new(frame, r / cos.abs(), r)?;
    Ok(Exact::Curves(
        vec![(
            Curve3::Ellipse(ellipse),
            Interval::new(0.0, TAU),
            Contact::Transversal,
        )],
        Vec::new(),
    ))
}

fn parallel_cylinders(
    c: &Frame3,
    r1: f64,
    d: &Frame3,
    r2: f64,
    boxes: &Aabb3,
    tol: f64,
) -> Option<Result<Exact, GeomError>> {
    let a = c.z;
    if a.cross(d.z).length() > PARALLEL {
        return None;
    }
    // Cross sections in the plane through c.origin normal to the axis.
    let v = d.origin - c.origin;
    let v = v - a * v.dot(a);
    let dist = v.length();
    if dist <= tol {
        // Coaxial: handled with the surfaces of revolution.
        return None;
    }
    let mut curves = Vec::new();
    let result = (|| {
        if dist > r1 + r2 + tol || dist < (r1 - r2).abs() - tol {
            return Ok(Exact::Curves(Vec::new(), Vec::new()));
        }
        let e = v / dist;
        if (dist - (r1 + r2)).abs() <= tol || (dist - (r1 - r2).abs()).abs() <= tol {
            let s = if dist - (r1 - r2).abs() <= tol && r2 > r1 {
                -r1
            } else {
                r1
            };
            curves.extend(line(c.origin + e * s, a, boxes, tol, Contact::Tangent)?);
            return Ok(Exact::Curves(curves, Vec::new()));
        }
        let x = (dist * dist + r1 * r1 - r2 * r2) / (2.0 * dist);
        let h = (r1 * r1 - x * x).max(0.0).sqrt();
        let n = a.cross(e);
        let m = c.origin + e * x;
        curves.extend(line(m + n * h, a, boxes, tol, Contact::Transversal)?);
        curves.extend(line(m - n * h, a, boxes, tol, Contact::Transversal)?);
        Ok(Exact::Curves(curves, Vec::new()))
    })();
    Some(result)
}
