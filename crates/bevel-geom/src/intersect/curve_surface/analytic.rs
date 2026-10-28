// SPDX-License-Identifier: GPL-3.0-or-later

//! Closed-form intersections of lines and circles with analytic surfaces:
//! line–plane, line–sphere, line–cylinder, line–cone, line–torus (a
//! quartic) and circle–plane.
//!
//! As for curves, each case produces exact candidates `(t, u, v)` and
//! overlaps, the candidates are settled by Newton within the range and the
//! rectangle and kept where the curve comes within the tolerance of the
//! surface. The surface parameters of a candidate come from the surface's
//! own point inversion, repeated for every period inside the rectangle.
//! Tangency of a line to a sphere or cylinder is decided with the
//! tolerance (the foot of the perpendicular when the distance from the
//! centre or axis is within `tol` of the radius); for cones and tori the
//! extrema of the implicit equation along the line are added as
//! candidates, so that near misses within the tolerance are found.

use std::f64::consts::TAU;

use std::marker::PhantomData;

use bevel_math::tol::ANGULAR;
use bevel_math::{Aabb3, DVec3, Interval, roots};

use super::super::Rect;
use super::super::curve_curve::common::{
    Ctx, Found, Path, Raw, SurfaceTarget, Target, copies, copies_inside, finish, minimize,
    scale_of, surface_residual,
};
use crate::{Circle3, Cone, Curve3, Cylinder, Line3, Plane, Sphere, Surface, SurfaceGeom, Torus};

/// Candidates `(t, [u, v])` and overlaps `(t_lo, t_hi)` of a closed-form
/// case.
#[derive(Debug, Default)]
struct Exact {
    points: Vec<(f64, [f64; 2])>,
    overlaps: Vec<(f64, f64)>,
}

impl Exact {
    /// Adds the candidates for the point at `t` on the curve: the
    /// surface's point inversion, with every period of `u` and `v` inside
    /// the rectangle.
    fn add(&mut self, s: &Surface, rs: Rect, t: f64, p: DVec3) {
        let Ok(cp) = SurfaceGeom::closest_point(s, p) else {
            return;
        };
        let us = match s.period_u() {
            Some(period) => copies(cp.u, rs.u, period, 0.0),
            None => vec![cp.u],
        };
        let vs = match s.period_v() {
            Some(period) => copies(cp.v, rs.v, period, 0.0),
            None => vec![cp.v],
        };
        for &u in &us {
            for &v in &vs {
                self.points.push((t, [u, v]));
            }
        }
    }

    /// Adds the stretch `[lo, hi]` of the curve as an overlap if it is
    /// longer than `tol` (the curve having unit speed), or a candidate at
    /// its middle if the stretch only just closes.
    fn add_stretch(&mut self, s: &Surface, rs: Rect, line: &Line3, lo: f64, hi: f64, tol: f64) {
        if hi - lo > tol {
            self.overlaps.push((lo, hi));
        } else if hi - lo >= -tol {
            let m = 0.5 * (lo + hi);
            self.add(s, rs, m, line.point(m));
        }
    }
}

/// The fast paths, or `None` when the pair has none.
pub(super) fn curve_surface(
    c: &Curve3,
    rc: Interval,
    s: &Surface,
    rs: Rect,
    tol: f64,
) -> Option<Vec<Found<2>>> {
    let exact = match (c, s) {
        (Curve3::Line(l), Surface::Plane(p)) => line_plane(l, p, rc, s, rs, tol),
        (Curve3::Line(l), Surface::Sphere(sp)) => line_sphere(l, sp, s, rs, tol),
        (Curve3::Line(l), Surface::Cylinder(cy)) => line_cylinder(l, cy, rc, s, rs, tol),
        (Curve3::Line(l), Surface::Cone(co)) => line_cone(l, co, rc, s, rs, tol),
        (Curve3::Line(l), Surface::Torus(to)) => line_torus(l, to, rc, s, rs, tol),
        (Curve3::Circle(ci), Surface::Plane(p)) => circle_plane(ci, p, rc, s, rs, tol),
        _ => return None,
    };
    Some(settle(c, rc, s, rs, tol, exact))
}

/// Settles the candidates with Newton within the range and the rectangle
/// and finishes the result like the general method's.
fn settle(
    c: &Curve3,
    rc: Interval,
    s: &Surface,
    rs: Rect,
    tol: f64,
    exact: Exact,
) -> Vec<Found<2>> {
    let samples = [rc.min, rc.mid(), rc.max].map(|t| Aabb3::from_points([c.at(t)]));
    let corners = [
        (rs.u.min, rs.v.min),
        (rs.u.max, rs.v.max),
        (rs.u.mid(), rs.v.mid()),
    ]
    .map(|(u, v)| Aabb3::from_points([SurfaceGeom::point(s, u, v)]));
    let scale = scale_of(&[samples, corners].concat());
    let target = SurfaceTarget {
        surface: s,
        bounds: [rs.u, rs.v],
    };
    let ctx = Ctx {
        a: c,
        ra: rc,
        target: &target,
        tol,
        scale,
        dims: PhantomData,
    };
    let bounds = [rc, rs.u, rs.v];
    let mut found = Vec::new();
    for (t, [u, v]) in exact.points {
        let solved = minimize(
            |x| surface_residual(c, &target, x),
            [t, u, v],
            &bounds,
            scale,
        );
        if let Some((x, d)) = solved
            && d <= tol
        {
            found.push(Found::Point(Raw {
                s: x[0],
                x: [x[1], x[2]],
                d,
            }));
        }
    }
    let corner = [rs.u.min, rs.v.min];
    let on_surface = |t: f64| {
        let p = c.at(t);
        let start = target.closest(p, &corner).unwrap_or(corner);
        ctx.project_local(p, start).map_or(start, |(x, _)| x)
    };
    for (lo, hi) in exact.overlaps {
        found.push(Found::Overlap {
            lo,
            hi,
            x_lo: on_surface(lo),
            x_hi: on_surface(hi),
        });
    }
    finish(&ctx, found)
}

/// The part `(lo, hi)` of `(lo, hi)` where `c0 + c1 t` lies in `target`
/// (`hi < lo` when there is none; for a constant outside `target` the gap
/// is infinite).
fn clip(lo: f64, hi: f64, c0: f64, c1: f64, target: Interval) -> (f64, f64) {
    if c1.abs() <= f64::MIN_POSITIVE {
        return if target.contains(c0) {
            (lo, hi)
        } else {
            (lo, f64::NEG_INFINITY)
        };
    }
    let (a, b) = ((target.min - c0) / c1, (target.max - c0) / c1);
    (lo.max(a.min(b)), hi.min(a.max(b)))
}

/// The candidates of a line with the given parameters.
fn line_points(l: &Line3, params: &[f64], s: &Surface, rs: Rect) -> Exact {
    let mut out = Exact::default();
    for &t in params {
        out.add(s, rs, t, l.point(t));
    }
    out
}

/// The parameters where a line meets a round surface whose distance from
/// the line's foot point `foot` is `h` with radius `r` (sphere or
/// cylinder), `speed` being the line's speed across it.
fn round_params(foot: f64, h: f64, r: f64, speed: f64, tol: f64) -> Vec<f64> {
    if h > r + tol {
        Vec::new()
    } else if (h - r).abs() <= tol {
        vec![foot]
    } else {
        let half = ((r - h) * (r + h)).sqrt() / speed;
        vec![foot - half, foot + half]
    }
}

/// A line and a plane: the crossing, or the stretch of a line in it.
fn line_plane(l: &Line3, p: &Plane, rc: Interval, s: &Surface, rs: Rect, tol: f64) -> Exact {
    let f = p.frame();
    let o = f.to_local(l.origin());
    let d = f.vector_to_local(l.direction());
    if d.z.abs() <= ANGULAR {
        if (o.z + rc.mid() * d.z).abs() > tol {
            return Exact::default();
        }
        let (lo, hi) = clip(rc.min, rc.max, o.x, d.x, rs.u);
        let (lo, hi) = clip(lo, hi, o.y, d.y, rs.v);
        let mut out = Exact::default();
        out.add_stretch(s, rs, l, lo, hi, tol);
        return out;
    }
    line_points(l, &[-o.z / d.z], s, rs)
}

/// A line and a sphere.
fn line_sphere(l: &Line3, sp: &Sphere, s: &Surface, rs: Rect, tol: f64) -> Exact {
    let foot = -(l.origin() - sp.center()).dot(l.direction());
    let h = (l.point(foot) - sp.center()).length();
    let params = round_params(foot, h, sp.radius(), 1.0, tol);
    line_points(l, &params, s, rs)
}

/// A line and a cylinder: crossings, or the stretch of a line along a
/// ruling.
fn line_cylinder(l: &Line3, cy: &Cylinder, rc: Interval, s: &Surface, rs: Rect, tol: f64) -> Exact {
    let f = cy.frame();
    let o = f.to_local(l.origin());
    let d = f.vector_to_local(l.direction());
    let r = cy.radius();
    let across = d.x * d.x + d.y * d.y;
    if across <= ANGULAR * ANGULAR {
        let rho = o.x.hypot(o.y);
        let mut out = Exact::default();
        if (rho - r).abs() > tol {
            return out;
        }
        for _ in copies_inside(o.y.atan2(o.x), rs.u, TAU, tol / r) {
            let (lo, hi) = clip(rc.min, rc.max, o.z, d.z, rs.v);
            out.add_stretch(s, rs, l, lo, hi, tol);
        }
        return out;
    }
    let foot = -(o.x * d.x + o.y * d.y) / across;
    let h = (o.x + foot * d.x).hypot(o.y + foot * d.y);
    let params = round_params(foot, h, r, across.sqrt(), tol);
    line_points(l, &params, s, rs)
}

/// A line and a (double) cone: the roots of the quadratic along the line
/// and its extremum, or the stretch of a line along a ruling.
fn line_cone(l: &Line3, co: &Cone, rc: Interval, s: &Surface, rs: Rect, tol: f64) -> Exact {
    let on_cone = [rc.min, rc.mid(), rc.max]
        .iter()
        .all(|&t| SurfaceGeom::closest_point(s, l.point(t)).is_ok_and(|cp| cp.distance <= tol));
    if on_cone {
        return cone_ruling(l, co, rc, s, rs, tol);
    }
    let f = co.frame();
    let o = f.to_local(l.origin());
    let d = f.vector_to_local(l.direction());
    let k = co.half_angle().tan();
    let rho = co.radius() + k * o.z;
    let a = d.x * d.x + d.y * d.y - k * k * d.z * d.z;
    let b = 2.0 * (o.x * d.x + o.y * d.y - k * d.z * rho);
    let c = o.x * o.x + o.y * o.y - rho * rho;
    let mut params = roots::quadratic(c, b, a);
    if a.abs() > ANGULAR {
        params.push(-b / (2.0 * a));
    }
    line_points(l, &params, s, rs)
}

/// A line lying on a cone: the ruling through its point farthest from the
/// apex, over the part of the line inside the rectangle.
fn cone_ruling(l: &Line3, co: &Cone, rc: Interval, s: &Surface, rs: Rect, tol: f64) -> Exact {
    let apex = co.apex();
    let t_ref = if (l.point(rc.min) - apex).length() >= (l.point(rc.max) - apex).length() {
        rc.min
    } else {
        rc.max
    };
    let mut out = Exact::default();
    let Ok(cp) = SurfaceGeom::closest_point(s, l.point(t_ref)) else {
        return out;
    };
    let f = co.frame();
    let o = f.to_local(l.origin());
    let d = f.vector_to_local(l.direction());
    let scale = co.radius_at(cp.v).abs().max(tol);
    for _ in copies_inside(cp.u, rs.u, TAU, tol / scale) {
        let (lo, hi) = clip(rc.min, rc.max, o.z, d.z, rs.v);
        out.add_stretch(s, rs, l, lo, hi, tol);
    }
    out
}

/// A line and a torus: the real roots of the quartic along the line, and
/// its critical points for near-tangent misses.
fn line_torus(l: &Line3, to: &Torus, rc: Interval, s: &Surface, rs: Rect, tol: f64) -> Exact {
    let f = to.frame();
    let o = f.to_local(l.origin());
    let d = f.vector_to_local(l.direction());
    // Measure along the line from the foot of the perpendicular from the
    // centre, which keeps the coefficients small.
    let shift = -o.dot(d);
    let q = o + d * shift;
    let (big, small) = (to.major_radius(), to.minor_radius());
    let alpha = q.length_squared() + big * big - small * small;
    let beta = q.dot(d);
    let r2 = 4.0 * big * big;
    let coeffs = [
        alpha * alpha - r2 * (q.x * q.x + q.y * q.y),
        4.0 * alpha * beta - 2.0 * r2 * (q.x * d.x + q.y * d.y),
        4.0 * beta * beta + 2.0 * alpha - r2 * (d.x * d.x + d.y * d.y),
        4.0 * beta,
        1.0,
    ];
    let window = Interval {
        min: rc.min - shift - tol,
        max: rc.max - shift + tol,
    };
    let mut sigmas = roots::roots_in(&coeffs, window);
    sigmas.extend(roots::roots_in(&roots::derivative(&coeffs), window));
    let params: Vec<f64> = sigmas.into_iter().map(|sigma| sigma + shift).collect();
    line_points(l, &params, s, rs)
}

/// A circle and a plane: the crossings, or the arcs of a circle in the
/// plane that lie inside the rectangle.
fn circle_plane(ci: &Circle3, p: &Plane, rc: Interval, s: &Surface, rs: Rect, tol: f64) -> Exact {
    let f = p.frame();
    let cf = ci.frame();
    let r = ci.radius();
    let center = f.to_local(cf.origin);
    let (x, y) = (f.vector_to_local(cf.x), f.vector_to_local(cf.y));
    // The height above the plane is c + a cos(t) + b sin(t).
    let (a, b, c) = (r * x.z, r * y.z, center.z);
    let big = a.hypot(b);
    if c.abs() + big <= tol {
        return circle_in_plane(ci, rc, center, x, y, s, rs, tol);
    }
    if big <= 1e-15 * r || c.abs() > big + tol {
        return Exact::default();
    }
    let phi = b.atan2(a);
    let delta = (-c / big).clamp(-1.0, 1.0).acos();
    let mut out = Exact::default();
    for t0 in [phi - delta, phi + delta] {
        for t in copies(t0, rc, TAU, tol / r) {
            out.add(s, rs, t, ci.point(t));
        }
    }
    out
}

/// The arcs of a circle lying in a plane inside the rectangle: the circle
/// in plane coordinates is `center + r (cos(t) x + sin(t) y)`.
#[allow(
    clippy::too_many_arguments,
    reason = "the circle's plane coordinates are passed in parts"
)]
fn circle_in_plane(
    ci: &Circle3,
    rc: Interval,
    center: DVec3,
    x: DVec3,
    y: DVec3,
    s: &Surface,
    rs: Rect,
    tol: f64,
) -> Exact {
    let r = ci.radius();
    let mut cuts = vec![rc.min, rc.max];
    // Where u(t) or v(t) crosses a side of the rectangle.
    for (c0, ca, cb, side) in [(center.x, x.x, y.x, rs.u), (center.y, x.y, y.y, rs.v)] {
        let (a, b) = (r * ca, r * cb);
        let big = a.hypot(b);
        if big <= f64::MIN_POSITIVE {
            continue;
        }
        let phi = b.atan2(a);
        for level in [side.min, side.max] {
            let ratio = (level - c0) / big;
            if ratio.abs() <= 1.0 {
                let delta = ratio.acos();
                for t0 in [phi - delta, phi + delta] {
                    cuts.extend(copies_inside(t0, rc, TAU, 0.0));
                }
            }
        }
    }
    cuts.sort_by(f64::total_cmp);
    let inside = |t: f64| {
        let (sn, cs) = t.sin_cos();
        let q = center + (x * cs + y * sn) * r;
        rs.contains_with(q.x, q.y, 1e-12 * (1.0 + q.x.abs().max(q.y.abs())))
    };
    let mut arcs: Vec<(f64, f64)> = Vec::new();
    for w in cuts.windows(2) {
        let (t0, t1) = (w[0], w[1]);
        if t1 <= t0 || !inside(0.5 * (t0 + t1)) {
            continue;
        }
        match arcs.last_mut() {
            Some(last) if last.1 >= t0 => last.1 = t1,
            _ => arcs.push((t0, t1)),
        }
    }
    let mut out = Exact::default();
    for (lo, hi) in arcs {
        if (hi - lo) * r > tol {
            out.overlaps.push((lo, hi));
        } else {
            let m = 0.5 * (lo + hi);
            out.add(s, rs, m, ci.point(m));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clip_keeps_the_part_inside() {
        let (lo, hi) = clip(0.0, 10.0, 1.0, 2.0, Interval::new(3.0, 5.0));
        assert!((lo - 1.0).abs() < 1e-15 && (hi - 2.0).abs() < 1e-15);
        let (lo, hi) = clip(0.0, 10.0, 4.0, 0.0, Interval::new(3.0, 5.0));
        assert!(lo.abs() < 1e-15 && (hi - 10.0).abs() < 1e-15);
        let (_, hi) = clip(0.0, 10.0, 6.0, 0.0, Interval::new(3.0, 5.0));
        assert!(hi < 0.0);
    }
}
