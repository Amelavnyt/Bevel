// SPDX-License-Identifier: GPL-3.0-or-later

//! Closed-form intersections of lines and circles.
//!
//! Each case produces exact candidate parameter pairs and overlaps; the
//! candidates are then settled by Newton within the ranges (which leaves an
//! exact root where it is and moves a near miss just outside a range onto
//! the closest pair of points inside), kept if the curves come within the
//! tolerance there, and deduplicated like the results of the general
//! method.
//!
//! Tangency is decided with the tolerance: a line whose distance from a
//! circle's centre is within `tol` of the radius touches it at the foot of
//! the perpendicular; two circles touch when the distance between their
//! centres is within `tol` of the sum or difference of the radii.
//! Coplanar 3D cases are solved in the plane of the circle.

use std::f64::consts::TAU;

use std::marker::PhantomData;

use bevel_math::tol::ANGULAR;
use bevel_math::{DVec2, Interval};

use super::common::{
    Ctx, CurveTarget, Found, Path, Raw, copies, curve_residual, finish, minimize, scale_of,
};
use crate::{Circle2, Circle3, Curve2, Curve3, Line2, Line3};

/// Candidates and overlaps of a closed-form case: parameter pairs `(s, t)`
/// and overlaps `(s_lo, s_hi, t_lo, t_hi)` (`t_lo` at `s_lo`).
#[derive(Debug, Default)]
struct Exact {
    points: Vec<(f64, f64)>,
    overlaps: Vec<(f64, f64, f64, f64)>,
}

impl Exact {
    /// The same result with the curves exchanged.
    fn swapped(self) -> Self {
        Self {
            points: self.points.into_iter().map(|(s, t)| (t, s)).collect(),
            overlaps: self
                .overlaps
                .into_iter()
                .map(|(s0, s1, t0, t1)| {
                    if t0 <= t1 {
                        (t0, t1, s0, s1)
                    } else {
                        (t1, t0, s1, s0)
                    }
                })
                .collect(),
        }
    }
}

/// The plane fast paths, or `None` when the pair has none.
pub(super) fn curves_2d(
    a: &Curve2,
    ra: Interval,
    b: &Curve2,
    rb: Interval,
    tol: f64,
) -> Option<Vec<Found<1>>> {
    let exact = match (a, b) {
        (Curve2::Line(l), Curve2::Line(m)) => line_line_2d(l, m, ra, rb, tol),
        (Curve2::Line(l), Curve2::Circle(c)) => line_circle_2d(l, c, rb, tol),
        (Curve2::Circle(c), Curve2::Line(l)) => line_circle_2d(l, c, ra, tol).swapped(),
        (Curve2::Circle(c), Curve2::Circle(d)) => circle_circle_2d(c, d, ra, rb, tol),
        _ => return None,
    };
    Some(settle(a, ra, b, rb, tol, exact))
}

/// The space fast paths, or `None` when the pair has none.
pub(super) fn curves_3d(
    a: &Curve3,
    ra: Interval,
    b: &Curve3,
    rb: Interval,
    tol: f64,
) -> Option<Vec<Found<1>>> {
    let exact = match (a, b) {
        (Curve3::Line(l), Curve3::Line(m)) => line_line_3d(l, m, ra, rb, tol),
        (Curve3::Line(l), Curve3::Circle(c)) => line_circle_3d(l, c, ra, rb, tol),
        (Curve3::Circle(c), Curve3::Line(l)) => line_circle_3d(l, c, rb, ra, tol).swapped(),
        (Curve3::Circle(c), Curve3::Circle(d)) => circle_circle_3d(c, d, ra, rb, tol),
        _ => return None,
    };
    Some(settle(a, ra, b, rb, tol, exact))
}

/// Settles the candidates with Newton within the ranges and finishes the
/// result like the general method's.
fn settle<P: Path>(
    a: &P,
    ra: Interval,
    b: &P,
    rb: Interval,
    tol: f64,
    exact: Exact,
) -> Vec<Found<1>> {
    let ends = [ra.min, ra.max, ra.mid()];
    let boxes: Vec<_> = ends
        .iter()
        .map(|&s| bevel_math::Aabb3::from_points([a.at(s)]))
        .chain([rb.min, rb.max, rb.mid()].map(|t| bevel_math::Aabb3::from_points([b.at(t)])))
        .collect();
    let scale = scale_of(&boxes);
    let target = CurveTarget {
        path: b,
        bounds: [rb],
    };
    let ctx = Ctx {
        a,
        ra,
        target: &target,
        tol,
        scale,
        dims: PhantomData,
    };
    let bounds = [ra, rb];
    let mut found = Vec::new();
    for (s, t) in exact.points {
        if let Some((v, d)) = minimize(|v| curve_residual(a, b, v), [s, t], &bounds, scale)
            && d <= tol
        {
            found.push(Found::Point(Raw {
                s: v[0],
                x: [v[1]],
                d,
            }));
        }
    }
    for (lo, hi, t_lo, t_hi) in exact.overlaps {
        found.push(Found::Overlap {
            lo,
            hi,
            x_lo: [t_lo],
            x_hi: [t_hi],
        });
    }
    finish(&ctx, found)
}

/// Where the first curve over `ra` coincides with the second over `rb`,
/// given that the point at `s` on the first is the point at
/// `t = sigma s + offset + k period` on the second (`sigma = +-1`), with
/// the first curve's speed `speed`: overlaps longer than `tol`, and a
/// candidate where the ranges just touch.
fn coincident(
    ra: Interval,
    rb: Interval,
    sigma: f64,
    offset: f64,
    period: Option<f64>,
    speed: f64,
    tol: f64,
) -> Exact {
    let mut out = Exact::default();
    let shifts: Vec<f64> = match period {
        None => vec![0.0],
        Some(p) => {
            let c = rb.mid() - offset - sigma * ra.mid();
            let h = 0.5 * (ra.length() + rb.length()) + p;
            let mut k = ((c - h) / p).ceil();
            let mut v = Vec::new();
            while k * p <= c + h && v.len() < 64 {
                v.push(k * p);
                k += 1.0;
            }
            v
        }
    };
    for shift in shifts {
        let off = offset + shift;
        let (s1, s2) = (sigma * (rb.min - off), sigma * (rb.max - off));
        let lo = ra.min.max(s1.min(s2));
        let hi = ra.max.min(s1.max(s2));
        let len = (hi - lo) * speed;
        if len > tol {
            out.overlaps
                .push((lo, hi, sigma * lo + off, sigma * hi + off));
        } else if len >= -tol {
            let m = 0.5 * (lo + hi);
            out.points.push((m, sigma * m + off));
        }
    }
    out
}

/// Two lines in the plane.
fn line_line_2d(l: &Line2, m: &Line2, ra: Interval, rb: Interval, tol: f64) -> Exact {
    let (o1, d1, o2, d2) = (l.origin(), l.direction(), m.origin(), m.direction());
    let cross = d1.perp_dot(d2);
    let w = o2 - o1;
    if cross.abs() <= ANGULAR {
        if w.perp_dot(d1).abs() > tol {
            return Exact::default();
        }
        let sigma = if d1.dot(d2) < 0.0 { -1.0 } else { 1.0 };
        return coincident(ra, rb, sigma, (o1 - o2).dot(d2), None, 1.0, tol);
    }
    Exact {
        points: vec![(w.perp_dot(d2) / cross, w.perp_dot(d1) / cross)],
        overlaps: Vec::new(),
    }
}

/// The angle of `p` on a plane circle.
fn angle_on(c: &Circle2, p: DVec2) -> f64 {
    let d = p - c.center();
    d.dot(c.y_axis()).atan2(d.dot(c.x_axis()))
}

/// A line and a circle in the plane; the circle's range is `rb`.
fn line_circle_2d(l: &Line2, c: &Circle2, rb: Interval, tol: f64) -> Exact {
    let (o, d, r) = (l.origin(), l.direction(), c.radius());
    let foot = -(o - c.center()).dot(d);
    let h = (l.point(foot) - c.center()).length();
    let params = if h > r + tol {
        return Exact::default();
    } else if (h - r).abs() <= tol {
        vec![foot]
    } else {
        let half = ((r - h) * (r + h)).sqrt();
        vec![foot - half, foot + half]
    };
    let mut out = Exact::default();
    for s in params {
        let theta = angle_on(c, l.point(s));
        for t in copies(theta, rb, TAU, tol / r) {
            out.points.push((s, t));
        }
    }
    out
}

/// The orientation of a plane circle: `1` counter-clockwise, `-1`
/// clockwise.
fn orientation(c: &Circle2) -> f64 {
    if c.is_counter_clockwise() { 1.0 } else { -1.0 }
}

/// Two circles in the plane.
fn circle_circle_2d(c1: &Circle2, c2: &Circle2, ra: Interval, rb: Interval, tol: f64) -> Exact {
    let (r1, r2) = (c1.radius(), c2.radius());
    let dv = c2.center() - c1.center();
    let d = dv.length();
    if d + (r1 - r2).abs() <= tol {
        // The same circle: the point at `s` on the first is at
        // `beta + sigma s` on the second.
        let x1 = c1.x_axis();
        let beta = x1.dot(c2.y_axis()).atan2(x1.dot(c2.x_axis()));
        let sigma = orientation(c1) * orientation(c2);
        return coincident(ra, rb, sigma, beta, Some(TAU), r1, tol);
    }
    let scale = 1.0 + r1 + r2 + c1.center().abs().max_element();
    if d <= 1e-15 * scale {
        return Exact::default();
    }
    let u = dv / d;
    let external = d - (r1 + r2);
    let internal = d - (r1 - r2).abs();
    if external > tol || internal < -tol {
        return Exact::default();
    }
    let points = if external.abs() <= tol {
        vec![c1.center() + u * r1]
    } else if internal.abs() <= tol {
        let sign = if r1 >= r2 { 1.0 } else { -1.0 };
        vec![c1.center() + u * (sign * r1)]
    } else {
        let a = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
        let h = (r1 * r1 - a * a).max(0.0).sqrt();
        let base = c1.center() + u * a;
        vec![base - u.perp() * h, base + u.perp() * h]
    };
    let mut out = Exact::default();
    for p in points {
        for s in copies(angle_on(c1, p), ra, TAU, tol / r1) {
            for t in copies(angle_on(c2, p), rb, TAU, tol / r2) {
                out.points.push((s, t));
            }
        }
    }
    out
}

/// Two lines in space: the closest points of skew lines, or the overlap of
/// parallel ones.
fn line_line_3d(l: &Line3, m: &Line3, ra: Interval, rb: Interval, tol: f64) -> Exact {
    let (o1, d1, o2, d2) = (l.origin(), l.direction(), m.origin(), m.direction());
    if d1.cross(d2).length() <= ANGULAR {
        if (o2 - o1).cross(d1).length() > tol {
            return Exact::default();
        }
        let sigma = if d1.dot(d2) < 0.0 { -1.0 } else { 1.0 };
        return coincident(ra, rb, sigma, (o1 - o2).dot(d2), None, 1.0, tol);
    }
    let r = o1 - o2;
    let (b, c, f) = (d1.dot(d2), d1.dot(r), d2.dot(r));
    let s = (b * f - c) / (1.0 - b * b);
    Exact {
        points: vec![(s, b * s + f)],
        overlaps: Vec::new(),
    }
}

/// A line and a circle in space; `ra` is the line's range, `rb` the
/// circle's.
fn line_circle_3d(l: &Line3, c: &Circle3, ra: Interval, rb: Interval, tol: f64) -> Exact {
    let f = c.frame();
    let o = f.to_local(l.origin());
    let d = f.vector_to_local(l.direction());
    let r = c.radius();
    let near = |s: f64| (o.z + s * d.z).abs() <= tol;
    if near(ra.min) && near(ra.max) {
        // In the plane of the circle: solve there.
        let dxy = d.truncate();
        let k = dxy.length();
        let (Ok(l2), Ok(c2)) = (Line2::new(o.truncate(), dxy), Circle2::new(DVec2::ZERO, r)) else {
            return Exact::default();
        };
        let mut e = line_circle_2d(&l2, &c2, rb, tol);
        for p in &mut e.points {
            p.0 /= k;
        }
        return e;
    }
    if d.z.abs() <= ANGULAR {
        return Exact::default();
    }
    let s = -o.z / d.z;
    let p = o + d * s;
    let mut out = Exact::default();
    for t in copies(p.y.atan2(p.x), rb, TAU, tol / r) {
        out.points.push((s, t));
    }
    out
}

/// Two circles in space: in their common plane, or where the first meets
/// the plane of the second.
fn circle_circle_3d(c1: &Circle3, c2: &Circle3, ra: Interval, rb: Interval, tol: f64) -> Exact {
    let f = c1.frame();
    let (r1, r2) = (c1.radius(), c2.radius());
    let center = f.to_local(c2.center());
    let normal = f.vector_to_local(c2.normal());
    if center.z.abs() <= tol && normal.truncate().length() * r2 <= tol {
        let x2 = f.vector_to_local(c2.frame().x).truncate();
        let (Ok(a2), Ok(b2)) = (
            Circle2::new(DVec2::ZERO, r1),
            Circle2::from_axis(center.truncate(), x2, normal.z > 0.0, r2),
        ) else {
            return Exact::default();
        };
        return circle_circle_2d(&a2, &b2, ra, rb, tol);
    }
    let n = c2.normal();
    let c = (c1.center() - c2.center()).dot(n);
    let a = r1 * f.x.dot(n);
    let b = r1 * f.y.dot(n);
    let big = a.hypot(b);
    if big <= 1e-15 * r1 || c.abs() > big + tol {
        return Exact::default();
    }
    let phi = b.atan2(a);
    let delta = (-c / big).clamp(-1.0, 1.0).acos();
    let mut out = Exact::default();
    let f2 = c2.frame();
    for s0 in [phi - delta, phi + delta] {
        for s in copies(s0, ra, TAU, tol / r1) {
            let p = f2.to_local(c1.point(s));
            for t in copies(p.y.atan2(p.x), rb, TAU, tol / r2) {
                out.points.push((s, t));
            }
        }
    }
    out
}
