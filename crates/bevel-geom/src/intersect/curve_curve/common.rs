// SPDX-License-Identifier: GPL-3.0-or-later

//! Machinery shared by curve–curve and curve–surface intersection.
//!
//! - [`minimize`]: a small Levenberg–Marquardt solver for `F(x) = 0` with
//!   one to three unknowns kept inside a box. Near a transversal root it is
//!   Newton's method (quadratic convergence); at a tangential root, where
//!   the Jacobian degenerates, the damping keeps it converging (linearly);
//!   elsewhere it stops at a local minimum of `|F|`, which is how distances
//!   between pieces are measured.
//! - [`Path`] and [`Target`]: a curve embedded in space (plane curves get
//!   `z = 0`), and what it is intersected with (another curve or a
//!   surface), as the solver and the subdivision see them.
//! - [`Shape`]: the chord, deviation from the chord and tangent cone of a
//!   curve piece, from its control points.
//! - [`resolve`] and [`finish`]: turning raw solutions into points and
//!   overlaps. Every raw solution is grown into the stretch of the first
//!   curve that stays within the tolerance of the target; a stretch longer
//!   than the tolerance along which the distance stays well inside it is an
//!   overlap, anything else (a crossing, a tangential touch) is one point,
//!   the best-converged raw solution in it.

use std::cmp::Ordering;
use std::marker::PhantomData;

use bevel_math::{Aabb2, Aabb3, DVec2, DVec3, Interval};

use crate::analytic::Domain;
use crate::{Curve2, Curve3, CurveGeom, GeomError, Surface, SurfaceGeom};

/// The deepest subdivision level of a piece.
pub(in crate::intersect) const MAX_LEVEL: u32 = 60;
/// The most pairs of pieces one intersection may examine.
pub(in crate::intersect) const MAX_WORK: usize = 200_000;
/// Pieces whose tangent (or normal) cone is at most this wide (radians)
/// count as flat.
pub(in crate::intersect) const FLAT_ANGLE: f64 = 0.35;
/// The angle (radians) by which two cones must miss each other to count
/// as transversal.
pub(in crate::intersect) const CONE_MARGIN: f64 = 0.02;
/// Pairs whose boxes are both below this multiple of the tolerance are not
/// split further.
pub(in crate::intersect) const TINY: f64 = 64.0;

/// Outer iterations of [`minimize`].
const MAX_ITERATIONS: usize = 100;
/// Damping increases per iteration of [`minimize`].
const MAX_DAMPING_STEPS: usize = 40;
/// The fractions of a stretch where [`Ctx::is_overlap`] samples the
/// distance.
const PROFILE: [f64; 5] = [0.1, 0.25, 0.5, 0.75, 0.9];
/// The largest steps (as a fraction of the range) when growing a stretch.
const STEPS_PER_RANGE: f64 = 32.0;
/// The most steps when growing a stretch in one direction.
const MAX_STEPS: usize = 4096;
/// The most bisection steps when locating the end of a stretch.
const MAX_BISECTIONS: usize = 80;

/// Checks the tolerance of an intersection: finite and positive.
pub(in crate::intersect) fn check_tol(tol: f64) -> Result<(), GeomError> {
    if !tol.is_finite() {
        return Err(GeomError::NonFinite);
    }
    if tol <= 0.0 {
        return Err(GeomError::InvalidArgument(
            "an intersection tolerance must be positive",
        ));
    }
    Ok(())
}

/// Checks a parameter range: finite, `min < max` and, for a bounded
/// domain, inside it (up to rounding, to which it is clamped).
pub(in crate::intersect) fn check_range(
    r: Interval,
    domain: Domain,
) -> Result<Interval, GeomError> {
    if !(r.min.is_finite() && r.max.is_finite()) {
        return Err(GeomError::NonFinite);
    }
    let empty = GeomError::InvalidArgument("a parameter range must have min < max");
    if r.min >= r.max {
        return Err(empty);
    }
    let Domain::Bounded(d) = domain else {
        return Ok(r);
    };
    let slack = 1e-9 * (1.0 + d.min.abs().max(d.max.abs()));
    for t in [r.min, r.max] {
        if !d.contains_with(t, slack) {
            return Err(GeomError::ParameterOutOfDomain { t });
        }
    }
    let c = Interval {
        min: r.min.max(d.min),
        max: r.max.min(d.max),
    };
    if c.min >= c.max {
        return Err(empty);
    }
    Ok(c)
}

/// A curve embedded in space.
pub(in crate::intersect) trait Path {
    /// The point at `t`.
    fn at(&self, t: f64) -> DVec3;

    /// The point and the first derivative at `t`.
    fn jet(&self, t: f64) -> (DVec3, DVec3);

    /// A box around the piece over `r` and, where the piece converts to a
    /// NURBS, its control points (whose convex hull contains it).
    fn hull(&self, r: Interval) -> Result<(Aabb3, Option<Vec<DVec3>>), GeomError>;

    /// The parameter of the point of the whole curve closest to `p` (for a
    /// plane curve, to its projection), and the period of the curve.
    fn closest(&self, p: DVec3) -> (Option<f64>, Option<f64>);
}

/// A plane point in space.
fn lift(p: DVec2) -> DVec3 {
    p.extend(0.0)
}

/// A plane box in space.
fn lift_box(b: Aabb2) -> Aabb3 {
    Aabb3 {
        min: lift(b.min),
        max: lift(b.max),
    }
}

impl Path for Curve2 {
    fn at(&self, t: f64) -> DVec3 {
        lift(CurveGeom::point(self, t))
    }

    fn jet(&self, t: f64) -> (DVec3, DVec3) {
        let d = CurveGeom::derivatives(self, t, 1);
        let p = d.first().copied().unwrap_or(DVec2::NAN);
        let v = d.get(1).copied().unwrap_or(DVec2::NAN);
        (lift(p), lift(v))
    }

    fn hull(&self, r: Interval) -> Result<(Aabb3, Option<Vec<DVec3>>), GeomError> {
        let lifted = |points: Vec<DVec2>| points.into_iter().map(lift).collect();
        if let Curve2::Nurbs(n) = self {
            return Ok(match CurveGeom::to_nurbs(n, r) {
                Ok(p) => (
                    lift_box(p.control_bounds()),
                    Some(lifted(p.control_points())),
                ),
                Err(_) => (lift_box(CurveGeom::bounds(n, r)?), None),
            });
        }
        let bounds = lift_box(CurveGeom::bounds(self, r)?);
        let points = CurveGeom::to_nurbs(self, r)
            .ok()
            .map(|p| lifted(p.control_points()));
        Ok((bounds, points))
    }

    fn closest(&self, p: DVec3) -> (Option<f64>, Option<f64>) {
        let t = CurveGeom::closest_point(self, p.truncate())
            .ok()
            .map(|c| c.t);
        (t, self.domain().period())
    }
}

impl Path for Curve3 {
    fn at(&self, t: f64) -> DVec3 {
        CurveGeom::point(self, t)
    }

    fn jet(&self, t: f64) -> (DVec3, DVec3) {
        let d = CurveGeom::derivatives(self, t, 1);
        let p = d.first().copied().unwrap_or(DVec3::NAN);
        let v = d.get(1).copied().unwrap_or(DVec3::NAN);
        (p, v)
    }

    fn hull(&self, r: Interval) -> Result<(Aabb3, Option<Vec<DVec3>>), GeomError> {
        if let Curve3::Nurbs(n) = self {
            return Ok(match CurveGeom::to_nurbs(n, r) {
                Ok(p) => (p.control_bounds(), Some(p.control_points())),
                Err(_) => (CurveGeom::bounds(n, r)?, None),
            });
        }
        let bounds = CurveGeom::bounds(self, r)?;
        let points = CurveGeom::to_nurbs(self, r)
            .ok()
            .map(|p| p.control_points());
        Ok((bounds, points))
    }

    fn closest(&self, p: DVec3) -> (Option<f64>, Option<f64>) {
        let t = CurveGeom::closest_point(self, p).ok().map(|c| c.t);
        (t, self.domain().period())
    }
}

/// What a curve is intersected with: a map from `N` parameters into space.
pub(in crate::intersect) trait Target<const N: usize> {
    /// The point and its partial derivatives at `x`.
    fn jet(&self, x: &[f64; N]) -> (DVec3, [DVec3; N]);

    /// The point at `x`.
    fn at(&self, x: &[f64; N]) -> DVec3 {
        self.jet(x).0
    }

    /// The parameter box.
    fn bounds(&self) -> &[Interval; N];

    /// Whether two overlaps that meet end to end join even where the
    /// target's parameters jump (across the seam of a periodic surface);
    /// overlaps with a curve need the curve's parameter to continue.
    fn joins_across_seams(&self) -> bool;

    /// Whether the side of the parameter box where coordinate `i` of `x`
    /// sits is an edge in space, where a stretch must end: not a seam
    /// inside the box, nor a side that collapses to a point (a pole).
    fn is_edge(&self, x: &[f64; N], i: usize) -> bool;

    /// A start for refining the point nearest to `p` found by the target's
    /// own (global) point inversion, inside the box and, in a periodic
    /// direction, in the period nearest to `near`.
    fn closest(&self, p: DVec3, near: &[f64; N]) -> Option<[f64; N]>;
}

/// The copy of the periodic parameter `t` nearest to `near`, clamped into
/// `r`.
fn nearest_copy(t: f64, period: Option<f64>, r: Interval, near: f64) -> f64 {
    let t = match period {
        Some(p) if p > 0.0 => t + ((near - t) / p).round() * p,
        _ => t,
    };
    r.clamp(t)
}

/// Whether `t` sits at an end of `r`.
fn at_end(r: Interval, t: f64) -> bool {
    let eps = 1e-12 * (1.0 + r.min.abs().max(r.max.abs()));
    (t - r.min).abs() <= eps || (r.max - t).abs() <= eps
}

/// Whether the parameters `x` and `y` of the map `at` over the box
/// `bounds` name one place across a seam: in every coordinate they agree,
/// or lie near opposite ends of the range, so near that moving each to its
/// end moves its point by at most `slack`.
fn across_seam<const N: usize>(
    at: impl Fn(&[f64; N]) -> DVec3,
    bounds: &[Interval; N],
    x: &[f64; N],
    y: &[f64; N],
    slack: f64,
) -> bool {
    (0..N).all(|i| {
        if (x[i] - y[i]).abs() <= 1e-9 * (1.0 + x[i].abs()) {
            return true;
        }
        let (lo, hi) = if x[i] <= y[i] { (x, y) } else { (y, x) };
        let (mut l, mut h) = (*lo, *hi);
        l[i] = bounds[i].min;
        h[i] = bounds[i].max;
        (at(&l) - at(lo)).length() <= slack && (at(&h) - at(hi)).length() <= slack
    })
}

/// A curve over a range as a [`Target`].
pub(in crate::intersect) struct CurveTarget<'a, P> {
    /// The curve.
    pub path: &'a P,
    /// Its range.
    pub bounds: [Interval; 1],
}

impl<P: Path> Target<1> for CurveTarget<'_, P> {
    fn jet(&self, x: &[f64; 1]) -> (DVec3, [DVec3; 1]) {
        let (p, d) = self.path.jet(x[0]);
        (p, [d])
    }

    fn at(&self, x: &[f64; 1]) -> DVec3 {
        self.path.at(x[0])
    }

    fn bounds(&self) -> &[Interval; 1] {
        &self.bounds
    }

    fn joins_across_seams(&self) -> bool {
        false
    }

    fn is_edge(&self, _x: &[f64; 1], _i: usize) -> bool {
        true
    }

    fn closest(&self, p: DVec3, near: &[f64; 1]) -> Option<[f64; 1]> {
        let (t, period) = self.path.closest(p);
        Some([nearest_copy(t?, period, self.bounds[0], near[0])])
    }
}

/// A surface over a rectangle as a [`Target`].
pub(in crate::intersect) struct SurfaceTarget<'a> {
    /// The surface.
    pub surface: &'a Surface,
    /// The rectangle, `[u, v]`.
    pub bounds: [Interval; 2],
}

impl Target<2> for SurfaceTarget<'_> {
    fn jet(&self, x: &[f64; 2]) -> (DVec3, [DVec3; 2]) {
        let d = SurfaceGeom::derivatives(self.surface, x[0], x[1], 1);
        let at = |k: usize, l: usize| {
            d.get(k)
                .and_then(|row| row.get(l))
                .copied()
                .unwrap_or(DVec3::NAN)
        };
        (at(0, 0), [at(1, 0), at(0, 1)])
    }

    fn at(&self, x: &[f64; 2]) -> DVec3 {
        SurfaceGeom::point(self.surface, x[0], x[1])
    }

    fn bounds(&self) -> &[Interval; 2] {
        &self.bounds
    }

    fn joins_across_seams(&self) -> bool {
        true
    }

    fn is_edge(&self, x: &[f64; 2], i: usize) -> bool {
        let period = if i == 0 {
            self.surface.period_u()
        } else {
            self.surface.period_v()
        };
        let b = self.bounds[i];
        if period.is_some_and(|p| b.length() >= p * (1.0 - 1e-12)) {
            return false;
        }
        // The side runs along the other parameter.
        let (p, [su, sv]) = self.jet(x);
        let (along, across) = if i == 0 { (sv, su) } else { (su, sv) };
        if along.length() <= 1e-9 * (along.length() + across.length()) {
            return false;
        }
        // A side that meets the opposite one is a seam (a closed NURBS).
        let mut y = *x;
        y[i] = if (x[i] - b.min).abs() <= (b.max - x[i]).abs() {
            b.max
        } else {
            b.min
        };
        (self.at(&y) - p).length() > 1e-12 * (1.0 + p.length())
    }

    fn closest(&self, p: DVec3, near: &[f64; 2]) -> Option<[f64; 2]> {
        let c = SurfaceGeom::closest_point(self.surface, p).ok()?;
        Some([
            nearest_copy(c.u, self.surface.period_u(), self.bounds[0], near[0]),
            nearest_copy(c.v, self.surface.period_v(), self.bounds[1], near[1]),
        ])
    }
}

/// The rows of the damped least-squares system of [`step`]: three for the
/// residual and one damping row per unknown.
const ROWS: usize = 6;

/// The step `d` minimising `|J d + r|^2 + lambda |D d|^2` with the
/// Marquardt scaling `D = diag(|J_i|)`, by modified Gram–Schmidt QR of the
/// stacked system (which, unlike the normal equations, keeps the accuracy
/// of nearly parallel columns, as at a tangential root); `None` if it is
/// singular.
///
/// Unknowns with `free[i] == false` are held fixed (their step is zero).
fn step<const N: usize>(
    j: &[DVec3; N],
    r: DVec3,
    lambda: f64,
    free: [bool; N],
) -> Option<[f64; N]> {
    let norms: [f64; N] = std::array::from_fn(|i| j[i].length());
    let big = norms.iter().fold(0.0_f64, |m, &v| m.max(v));
    if !(big > 0.0 && big.is_finite()) {
        return None;
    }
    let damping = lambda.sqrt();
    let mut cols: [[f64; ROWS]; N] = std::array::from_fn(|i| {
        let mut c = [0.0; ROWS];
        if free[i] {
            c[..3].copy_from_slice(&j[i].to_array());
            c[3 + i] = damping * norms[i].max(1e-9 * big);
        }
        c
    });
    let mut b = [0.0; ROWS];
    b[..3].copy_from_slice(&(-r).to_array());
    let dot = |p: &[f64; ROWS], q: &[f64; ROWS]| p.iter().zip(q).map(|(a, b)| a * b).sum::<f64>();
    let mut upper = [[0.0; N]; N];
    let mut rhs = [0.0; N];
    for k in 0..N {
        if !free[k] {
            upper[k][k] = 1.0;
            continue;
        }
        let norm = dot(&cols[k], &cols[k]).sqrt();
        if norm.is_nan() || norm <= 1e-13 * norms[k].max(1e-9 * big) {
            return None;
        }
        let q = cols[k].map(|v| v / norm);
        upper[k][k] = norm;
        for (l, col) in cols.iter_mut().enumerate().skip(k + 1) {
            let c = dot(&q, col);
            upper[k][l] = c;
            for (v, qv) in col.iter_mut().zip(&q) {
                *v -= c * qv;
            }
        }
        let c = dot(&q, &b);
        rhs[k] = c;
        for (v, qv) in b.iter_mut().zip(&q) {
            *v -= c * qv;
        }
    }
    let mut x = [0.0; N];
    for k in (0..N).rev() {
        let tail: f64 = (k + 1..N).map(|l| upper[k][l] * x[l]).sum();
        x[k] = (rhs[k] - tail) / upper[k][k];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// The damped step from `x`, with the unknowns that sit on a side of the
/// box and would step out of it held there (an active set), so that the
/// others still take their best step.
fn constrained_step<const N: usize>(
    j: &[DVec3; N],
    r: DVec3,
    lambda: f64,
    x: &[f64; N],
    bounds: &[Interval; N],
) -> Option<[f64; N]> {
    let mut free = [true; N];
    let mut d = step(j, r, lambda, free)?;
    for _ in 0..N {
        let mut changed = false;
        for i in 0..N {
            let b = bounds[i];
            let pinned = (x[i] <= b.min && d[i] < 0.0) || (x[i] >= b.max && d[i] > 0.0);
            if free[i] && pinned {
                free[i] = false;
                changed = true;
            }
        }
        if !changed || !free.iter().any(|&f| f) {
            break;
        }
        d = step(j, r, lambda, free)?;
    }
    Some(d)
}

/// Minimises `|F(x)|` over the box `bounds` from `start` by
/// Levenberg–Marquardt iteration, where `f` returns `F(x)` and the columns
/// of its Jacobian; returns the final `x` and `|F(x)|`, or `None` if `F`
/// is not finite at the start.
///
/// `scale` is the size of the coordinates involved: a residual below a few
/// ulps of it counts as zero.
pub(in crate::intersect) fn minimize<const N: usize>(
    f: impl Fn(&[f64; N]) -> (DVec3, [DVec3; N]),
    start: [f64; N],
    bounds: &[Interval; N],
    scale: f64,
) -> Option<([f64; N], f64)> {
    let clamp = |x: [f64; N]| -> [f64; N] { std::array::from_fn(|i| bounds[i].clamp(x[i])) };
    let finite = |r: DVec3, j: &[DVec3; N]| r.is_finite() && j.iter().all(|c| c.is_finite());
    let mut x = clamp(start);
    if !x.iter().all(|v| v.is_finite()) {
        return None;
    }
    let (mut r, mut j) = f(&x);
    if !finite(r, &j) {
        return None;
    }
    let mut cost = r.length_squared();
    let floor = (4.0 * f64::EPSILON * scale).powi(2);
    let mut lambda = 0.0_f64;
    for _ in 0..MAX_ITERATIONS {
        if cost <= floor {
            break;
        }
        let mut next = None;
        for _ in 0..MAX_DAMPING_STEPS {
            if let Some(d) = constrained_step(&j, r, lambda, &x, bounds) {
                let trial = clamp(std::array::from_fn(|i| x[i] + d[i]));
                let (rt, jt) = f(&trial);
                if finite(rt, &jt) && rt.length_squared() < cost {
                    next = Some((trial, rt, jt));
                    break;
                }
            }
            lambda = if lambda > 0.0 { lambda * 8.0 } else { 1e-6 };
            if lambda > 1e16 {
                break;
            }
        }
        let Some((trial, rt, jt)) = next else {
            break;
        };
        let moved = (0..N).fold(0.0_f64, |m, i| m.max((trial[i] - x[i]).abs()));
        let size = 1.0 + x.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        let previous = cost;
        x = trial;
        r = rt;
        j = jt;
        cost = r.length_squared();
        lambda = if lambda > 1e-12 { lambda * 0.2 } else { 0.0 };
        if moved <= 4.0 * f64::EPSILON * size || cost > previous * (1.0 - 1e-10) {
            break;
        }
    }
    Some((x, cost.sqrt()))
}

/// The residual `a(s) - b(t)` of two curves and its Jacobian.
pub(in crate::intersect) fn curve_residual<P: Path, Q: Path>(
    a: &P,
    b: &Q,
    x: &[f64; 2],
) -> (DVec3, [DVec3; 2]) {
    let (pa, da) = a.jet(x[0]);
    let (pb, db) = b.jet(x[1]);
    (pa - pb, [da, -db])
}

/// The residual `c(t) - S(u, v)` of a curve and a surface and its
/// Jacobian.
pub(in crate::intersect) fn surface_residual<P: Path>(
    c: &P,
    s: &SurfaceTarget<'_>,
    x: &[f64; 3],
) -> (DVec3, [DVec3; 3]) {
    let (pc, dc) = c.jet(x[0]);
    let (ps, [su, sv]) = s.jet(&[x[1], x[2]]);
    (pc - ps, [dc, -su, -sv])
}

/// The chord, deviation and tangent cone of a curve piece.
#[derive(Clone, Copy, Debug)]
pub(in crate::intersect) struct Shape {
    /// The start of the piece.
    pub p0: DVec3,
    /// The end of the piece.
    pub p1: DVec3,
    /// The unit direction from `p0` to `p1`.
    pub axis: DVec3,
    /// How far the piece may be from the chord `p0 p1`.
    pub dev: f64,
    /// The half-angle of a cone about `axis` holding every tangent.
    pub half: f64,
}

/// The [`Shape`] of a piece from its control points (whose first and last
/// are its ends); `None` for a closed piece.
///
/// The cone holds every difference `P_j - P_i` (`i < j`), which bounds the
/// derivative of a Bezier piece, rational ones included; for long control
/// polygons only consecutive differences are used.
pub(in crate::intersect) fn shape_of(points: &[DVec3]) -> Option<Shape> {
    let (&p0, &p1) = (points.first()?, points.last()?);
    let chord = p1 - p0;
    let len = chord.length();
    if !(len > 0.0 && len.is_finite()) {
        return None;
    }
    let axis = chord / len;
    let dev = points
        .iter()
        .map(|&p| point_segment_distance(p, p0, p1))
        .fold(0.0, f64::max);
    let mut half = 0.0_f64;
    let mut widen = |w: DVec3| {
        if w.length() > 1e-14 * len {
            half = half.max(angle(axis, w));
        }
    };
    if points.len() <= 12 {
        for (i, &p) in points.iter().enumerate() {
            for &q in &points[i + 1..] {
                widen(q - p);
            }
        }
    } else {
        for w in points.windows(2) {
            widen(w[1] - w[0]);
        }
    }
    Some(Shape {
        p0,
        p1,
        axis,
        dev,
        half: half + 1e-3,
    })
}

/// The angle between two vectors, in `[0, pi]`.
pub(in crate::intersect) fn angle(a: DVec3, b: DVec3) -> f64 {
    a.cross(b).length().atan2(a.dot(b))
}

/// The angle between the lines along two vectors, in `[0, pi / 2]`.
pub(in crate::intersect) fn line_angle(a: DVec3, b: DVec3) -> f64 {
    a.cross(b).length().atan2(a.dot(b).abs())
}

/// The distance from `p` to the segment `a b`.
fn point_segment_distance(p: DVec3, a: DVec3, b: DVec3) -> f64 {
    let d = b - a;
    let len2 = d.length_squared();
    let t = if len2 > 0.0 {
        ((p - a).dot(d) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + d * t)).length()
}

/// The distance between the segments `p1 q1` and `p2 q2` (Ericson,
/// *Real-Time Collision Detection*, section 5.1.9).
pub(in crate::intersect) fn segment_distance(p1: DVec3, q1: DVec3, p2: DVec3, q2: DVec3) -> f64 {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    let (s, t) = if a <= 0.0 && e <= 0.0 {
        (0.0, 0.0)
    } else if a <= 0.0 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= 0.0 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let s = if denom > 1e-12 * a * e {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let t = (b * s + f) / e;
            if t < 0.0 {
                ((-c / a).clamp(0.0, 1.0), 0.0)
            } else if t > 1.0 {
                (((b - c) / a).clamp(0.0, 1.0), 1.0)
            } else {
                (s, t)
            }
        }
    };
    ((p1 + d1 * s) - (p2 + d2 * t)).length()
}

/// Whether the boxes come within `tol` of each other.
pub(in crate::intersect) fn boxes_meet(a: &Aabb3, b: &Aabb3, tol: f64) -> bool {
    let slack = DVec3::splat(tol);
    (a.min - slack).cmple(b.max).all() && (b.min - slack).cmple(a.max).all()
}

/// The length of the diagonal of a box.
pub(in crate::intersect) fn diagonal(b: &Aabb3) -> f64 {
    (b.max - b.min).length()
}

/// The size of the coordinates in the boxes, at least 1.
pub(in crate::intersect) fn scale_of(boxes: &[Aabb3]) -> f64 {
    boxes.iter().fold(1.0_f64, |m, b| {
        m.max(1.0 + b.min.abs().max_element().max(b.max.abs().max_element()))
    })
}

/// Whether a range is long enough to split.
pub(in crate::intersect) fn splittable(r: Interval) -> bool {
    r.length() > 1e-12 * (1.0 + r.min.abs().max(r.max.abs()))
}

/// The range `r` widened by half its length on both sides, within `full`.
pub(in crate::intersect) fn widened(r: Interval, full: Interval) -> Interval {
    let h = 0.5 * r.length();
    Interval {
        min: (r.min - h).max(full.min),
        max: (r.max + h).min(full.max),
    }
}

/// The copies `theta + k period` of a periodic parameter inside `range`
/// (widened by `slack`), clamped into it; when there are none, the end of
/// the range nearest to a copy, so that a nearby hit is still refined.
pub(in crate::intersect) fn copies(
    theta: f64,
    range: Interval,
    period: f64,
    slack: f64,
) -> Vec<f64> {
    let mut out = copies_inside(theta, range, period, slack);
    if out.is_empty() && theta.is_finite() {
        let below = theta + ((range.min - theta) / period).floor() * period;
        let above = below + period;
        let gap_below = (range.min - below).abs().min((range.max - below).abs());
        let gap_above = (range.min - above).abs().min((range.max - above).abs());
        let near = if gap_below <= gap_above { below } else { above };
        out.push(range.clamp(near));
    }
    out
}

/// The copies `theta + k period` inside `range` widened by `slack`,
/// clamped into it (at most 64).
pub(in crate::intersect) fn copies_inside(
    theta: f64,
    range: Interval,
    period: f64,
    slack: f64,
) -> Vec<f64> {
    let mut out = Vec::new();
    if !(theta.is_finite() && period > 0.0) {
        return out;
    }
    let first = ((range.min - slack - theta) / period).ceil();
    let last = ((range.max + slack - theta) / period).floor();
    let mut k = first;
    while k <= last && out.len() < 64 {
        out.push(range.clamp(theta + k * period));
        k += 1.0;
    }
    out
}

/// A solution of the intersection equations: the parameter `s` on the
/// first curve, the parameters `x` on the target and the distance `d`
/// between the two points.
#[derive(Clone, Copy, Debug)]
pub(in crate::intersect) struct Raw<const N: usize> {
    /// The parameter on the first curve.
    pub s: f64,
    /// The parameters on the target.
    pub x: [f64; N],
    /// The distance between the points.
    pub d: f64,
}

/// A point or an overlap, before conversion to the public hit types.
#[derive(Clone, Copy, Debug)]
pub(in crate::intersect) enum Found<const N: usize> {
    /// A point.
    Point(Raw<N>),
    /// The first curve over `[lo, hi]` lies on the target; `x_lo` and
    /// `x_hi` are the target's parameters at the ends.
    Overlap {
        /// The start on the first curve.
        lo: f64,
        /// The end on the first curve.
        hi: f64,
        /// The target's parameters at `lo`.
        x_lo: [f64; N],
        /// The target's parameters at `hi`.
        x_hi: [f64; N],
    },
}

impl<const N: usize> Found<N> {
    /// The sort key: the first curve's parameter, then the target's first.
    fn key(&self) -> (f64, f64) {
        match self {
            Self::Point(r) => (r.s, r.x.first().copied().unwrap_or(0.0)),
            Self::Overlap { lo, x_lo, x_hi, .. } => {
                let a = x_lo.first().copied().unwrap_or(0.0);
                let b = x_hi.first().copied().unwrap_or(0.0);
                (*lo, a.min(b))
            }
        }
    }
}

/// Orders by the first parameter, then the second.
fn by_key(a: (f64, f64), b: (f64, f64)) -> Ordering {
    a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1))
}

/// The componentwise midpoint of two parameter vectors.
fn midpoint<const N: usize>(a: &[f64; N], b: &[f64; N]) -> [f64; N] {
    std::array::from_fn(|i| 0.5 * (a[i] + b[i]))
}

/// A stretch of the first curve within the tolerance of the target, with
/// the target's parameters tracked along it.
struct Stretch<const N: usize> {
    lo: f64,
    hi: f64,
    x_lo: [f64; N],
    x_hi: [f64; N],
    /// Every point visited inside the stretch, for restarting projections.
    samples: Vec<(f64, [f64; N])>,
}

impl<const N: usize> Stretch<N> {
    /// The target's parameters at the visited point nearest to `s`.
    fn nearest(&self, s: f64) -> [f64; N] {
        self.samples
            .iter()
            .min_by(|a, b| (a.0 - s).abs().total_cmp(&(b.0 - s).abs()))
            .map_or(self.x_lo, |p| p.1)
    }
}

/// The context of one intersection: the first curve over its range, the
/// target, the tolerance and the coordinate scale.
pub(in crate::intersect) struct Ctx<'a, P, T, const N: usize> {
    /// The first curve.
    pub a: &'a P,
    /// Its range.
    pub ra: Interval,
    /// What it is intersected with.
    pub target: &'a T,
    /// The tolerance.
    pub tol: f64,
    /// The coordinate scale (see [`minimize`]).
    pub scale: f64,
    /// The number of target parameters.
    pub dims: PhantomData<[f64; N]>,
}

impl<P: Path, T: Target<N>, const N: usize> Ctx<'_, P, T, N> {
    /// The point of the target nearest to `p`, searched from `start`, and
    /// its distance.
    ///
    /// On a surface, where the search from `start` ends farther than `tol`
    /// away (stuck at a pole, say, or on the wrong side of a seam), it is
    /// repeated from the surface's own point inversion. A curve target is
    /// only searched locally, so that the tracked stretch of it stays one
    /// piece of its range.
    pub(in crate::intersect) fn project(
        &self,
        p: DVec3,
        start: [f64; N],
    ) -> Option<([f64; N], f64)> {
        let local = self.project_local(p, start);
        if !self.target.joins_across_seams() || local.is_some_and(|(_, d)| d <= self.tol) {
            return local;
        }
        let global = self
            .target
            .closest(p, &start)
            .and_then(|from| self.project_local(p, from));
        match (local, global) {
            (Some(l), Some(g)) => Some(if g.1 < l.1 { g } else { l }),
            (l, g) => l.or(g),
        }
    }

    /// The point of the target nearest to `p` by a local search from
    /// `start`, and its distance.
    pub(in crate::intersect) fn project_local(
        &self,
        p: DVec3,
        start: [f64; N],
    ) -> Option<([f64; N], f64)> {
        minimize(
            |x| {
                let (q, j) = self.target.jet(x);
                (q - p, j)
            },
            start,
            self.target.bounds(),
            self.scale,
        )
    }

    /// Whether `x` lies on the boundary of the target's parameter box.
    fn pinned(&self, x: &[f64; N]) -> bool {
        let bounds = self.target.bounds();
        (0..N).any(|i| at_end(bounds[i], x[i]) && self.target.is_edge(x, i))
    }

    /// The parameter step on the first curve worth about `tol` in space.
    fn step(&self, s: f64) -> f64 {
        let speed = self.a.jet(s).1.length();
        let h = if speed > 0.0 && speed.is_finite() {
            self.tol / speed
        } else {
            1e-9 * self.ra.length()
        };
        h.min(self.ra.length() / STEPS_PER_RANGE)
    }

    /// Grows `seed` into the stretch of the first curve that stays within
    /// `tol` of the target.
    fn stretch(&self, seed: &Raw<N>) -> Stretch<N> {
        let mut samples = vec![(seed.s, seed.x)];
        let free = self.pinned(&seed.x);
        let (lo, x_lo) = self.extend(seed, -1.0, free, &mut samples);
        let (hi, x_hi) = self.extend(seed, 1.0, free, &mut samples);
        Stretch {
            lo,
            hi,
            x_lo,
            x_hi,
            samples,
        }
    }

    /// Whether the first curve at `s` is within `tol` of the target (and,
    /// unless `free`, does not project onto the target's boundary, so that
    /// a stretch ends where the target's piece does): the projection
    /// started from `start`.
    fn holds(&self, s: f64, start: [f64; N], free: bool) -> Option<[f64; N]> {
        let (x, d) = self.project(self.a.at(s), start)?;
        (d <= self.tol && (free || !self.pinned(&x))).then_some(x)
    }

    /// Walks from `seed` in direction `dir` with growing steps while the
    /// curves stay together, then bisects for the end.
    fn extend(
        &self,
        seed: &Raw<N>,
        dir: f64,
        free: bool,
        samples: &mut Vec<(f64, [f64; N])>,
    ) -> (f64, [f64; N]) {
        let end = if dir > 0.0 { self.ra.max } else { self.ra.min };
        let hmax = self.ra.length() / STEPS_PER_RANGE;
        let (mut gs, mut gx) = (seed.s, seed.x);
        let mut h = self.step(seed.s);
        for _ in 0..MAX_STEPS {
            if (end - gs) * dir <= 0.0 {
                return (gs, gx);
            }
            let s1 = if dir > 0.0 {
                (gs + h).min(end)
            } else {
                (gs - h).max(end)
            };
            let sm = 0.5 * (gs + s1);
            let mid = self.holds(sm, gx, free);
            // Both pieces may end together at the end of the first's range.
            let at_end = (end - s1).abs() <= 1e-12 * (1.0 + end.abs());
            let far = mid.and_then(|xm| self.holds(s1, xm, free || at_end));
            if let (Some(xm), Some(x1)) = (mid, far) {
                samples.push((sm, xm));
                samples.push((s1, x1));
                gs = s1;
                gx = x1;
                h = (2.0 * h).min(hmax);
                continue;
            }
            let mut bs = if mid.is_some() { s1 } else { sm };
            let close = 1e-3 * self.tol;
            for _ in 0..MAX_BISECTIONS {
                if (self.a.at(gs) - self.a.at(bs)).length() <= close {
                    break;
                }
                let m = 0.5 * (gs + bs);
                match self.holds(m, gx, free) {
                    Some(x) => {
                        samples.push((m, x));
                        gs = m;
                        gx = x;
                    }
                    None => bs = m,
                }
            }
            return self.snap(gs, gx, bs, free).unwrap_or((gs, gx));
        }
        (gs, gx)
    }

    /// Where a stretch ends because the target's piece ends (the curve
    /// beyond `good` projects onto its boundary), the end found by
    /// projecting that boundary point back onto the first curve, between
    /// `good` and `bad`.
    fn snap(&self, good: f64, gx: [f64; N], bad: f64, free: bool) -> Option<(f64, [f64; N])> {
        if free {
            return None;
        }
        let (xb, db) = self.project(self.a.at(bad), gx)?;
        if db > self.tol || !self.pinned(&xb) {
            return None;
        }
        let end = self.target.at(&xb);
        let span = [Interval::new(good, bad)];
        let (s, d) = minimize(
            |v| {
                let (p, dp) = self.a.jet(v[0]);
                (p - end, [dp])
            },
            [good],
            &span,
            self.scale,
        )?;
        (d <= self.tol).then_some((s[0], xb))
    }

    /// Whether a stretch is an overlap: longer than `tol`, with the
    /// distance well inside `tol` across it (a tangential touch or a
    /// shallow crossing is within `tol` over a stretch too, but the
    /// distance there grows towards the ends).
    fn is_overlap(&self, st: &Stretch<N>) -> bool {
        let span = Interval {
            min: st.lo,
            max: st.hi,
        };
        let mut len = 0.0;
        let mut prev = self.a.at(st.lo);
        for k in 1..=8 {
            let p = self.a.at(span.lerp(f64::from(k) / 8.0));
            len += (p - prev).length();
            prev = p;
        }
        if len <= self.tol {
            return false;
        }
        PROFILE.iter().all(|&f| {
            let s = span.lerp(f);
            self.project(self.a.at(s), st.nearest(s))
                .is_some_and(|(_, d)| d <= 0.5 * self.tol)
        })
    }

    /// Whether the raw solution `r` (with `s` inside the stretch) is on the
    /// tracked branch of the target, not on another branch or another
    /// period through the same point.
    fn on_branch(&self, st: &Stretch<N>, r: &Raw<N>) -> bool {
        let Some((xp, _)) = self.project(self.a.at(r.s), st.nearest(r.s)) else {
            return false;
        };
        self.same_place(&xp, &r.x, 2.0 * self.tol)
    }

    /// Whether the target's parameters `x` and `y` name the same place:
    /// their points and the point halfway between them in parameter space
    /// are within `slack` (the second test tells periodic copies apart).
    /// The ends of a range that closes up count as one place too, so that
    /// a hit on the seam of a closed range is reported once.
    fn same_place(&self, x: &[f64; N], y: &[f64; N], slack: f64) -> bool {
        let (px, py) = (self.target.at(x), self.target.at(y));
        if (px - py).length() > slack {
            return false;
        }
        let pm = self.target.at(&midpoint(x, y));
        if (pm - py).length() <= slack {
            return true;
        }
        across_seam(|x| self.target.at(x), self.target.bounds(), x, y, slack)
    }

    /// Whether two raw solutions are the same point (see
    /// [`Ctx::same_place`]).
    fn same_point(&self, p: &Raw<N>, q: &Raw<N>) -> bool {
        let (ap, aq) = (self.a.at(p.s), self.a.at(q.s));
        if (ap - aq).length() > self.tol {
            return false;
        }
        let am = self.a.at(0.5 * (p.s + q.s));
        let seam = || {
            across_seam(
                |s: &[f64; 1]| self.a.at(s[0]),
                &[self.ra],
                &[p.s],
                &[q.s],
                self.tol,
            )
        };
        ((am - aq).length() <= self.tol || seam()) && self.same_place(&p.x, &q.x, self.tol)
    }

    /// Whether two overlaps sorted by start join end to end.
    fn joins(&self, a: &Found<N>, b: &Found<N>) -> bool {
        let (
            Found::Overlap {
                hi: hi_a,
                x_lo: xl_a,
                x_hi: xh_a,
                ..
            },
            Found::Overlap {
                lo: lo_b,
                x_lo: xl_b,
                x_hi: xh_b,
                ..
            },
        ) = (a, b)
        else {
            return false;
        };
        let eps = 1e-9 * (1.0 + hi_a.abs());
        if *lo_b > hi_a + eps {
            return false;
        }
        if self.target.joins_across_seams() {
            return true;
        }
        // The target's parameter must continue across the joint, the same
        // way along both.
        let dir = |l: &[f64; N], h: &[f64; N]| h.iter().zip(l).map(|(h, l)| h - l).sum::<f64>();
        let (da, db) = (dir(xl_a, xh_a), dir(xl_b, xh_b));
        let x_eps = 1e-9 * (1.0 + xh_a.iter().fold(0.0_f64, |m, v| m.max(v.abs())));
        (da >= 0.0) == (db >= 0.0) && xh_a.iter().zip(xl_b).all(|(p, q)| (p - q).abs() <= x_eps)
    }

    /// The speed of the first curve at `s`.
    fn speed(&self, s: f64) -> f64 {
        self.a.jet(s).1.length()
    }
}

/// Turns raw solutions of the general method into points and overlaps (see
/// the [module documentation](self)).
pub(in crate::intersect) fn resolve<P: Path, T: Target<N>, const N: usize>(
    ctx: &Ctx<'_, P, T, N>,
    mut raws: Vec<Raw<N>>,
) -> Vec<Found<N>> {
    raws.sort_by(|p, q| by_key(Found::Point(*p).key(), Found::Point(*q).key()));
    let mut used = vec![false; raws.len()];
    let mut found = Vec::new();
    for i in 0..raws.len() {
        if used[i] {
            continue;
        }
        used[i] = true;
        let seed = raws[i];
        let st = ctx.stretch(&seed);
        // Every raw solution on the stretch's branch belongs to it; the best
        // one is chosen among all of them, also those an earlier stretch
        // (ended early at a seam of the target) took.
        let mut best = seed;
        let eps = 1e-9 * (1.0 + st.lo.abs().max(st.hi.abs()));
        let first = raws.partition_point(|r| r.s < st.lo - eps);
        for j in first..raws.len() {
            let r = raws[j];
            if r.s > st.hi + eps {
                break;
            }
            if j != i && ctx.on_branch(&st, &r) {
                used[j] = true;
                if r.d < best.d {
                    best = r;
                }
            }
        }
        if ctx.is_overlap(&st) {
            found.push(Found::Overlap {
                lo: st.lo,
                hi: st.hi,
                x_lo: st.x_lo,
                x_hi: st.x_hi,
            });
        } else {
            found.push(Found::Point(best));
        }
    }
    finish(ctx, found)
}

/// Joins overlaps that meet end to end, drops points inside overlaps,
/// merges duplicate points (keeping the better-converged one) and sorts
/// everything by the first curve's parameter, then the target's.
pub(in crate::intersect) fn finish<P: Path, T: Target<N>, const N: usize>(
    ctx: &Ctx<'_, P, T, N>,
    found: Vec<Found<N>>,
) -> Vec<Found<N>> {
    let mut overlaps: Vec<Found<N>> = Vec::new();
    let mut points: Vec<Raw<N>> = Vec::new();
    for f in found {
        match f {
            Found::Point(r) => points.push(r),
            o @ Found::Overlap { .. } => overlaps.push(o),
        }
    }
    overlaps.sort_by(|p, q| by_key(p.key(), q.key()));
    let mut joined: Vec<Found<N>> = Vec::new();
    for o in overlaps {
        if let Some(last) = joined.last_mut()
            && ctx.joins(last, &o)
        {
            if let (Found::Overlap { hi, x_hi, .. }, Found::Overlap { hi: h, x_hi: x, .. }) =
                (last, o)
                && h > *hi
            {
                *hi = h;
                *x_hi = x;
            }
            continue;
        }
        joined.push(o);
    }
    // A point inside an overlap, or at one of its ends (across the seam
    // of a closed range, too), belongs to it.
    let inside = |r: &Raw<N>| {
        let speed = ctx.speed(r.s);
        let e = if speed > 0.0 { ctx.tol / speed } else { 0.0 };
        joined.iter().any(|o| match o {
            Found::Overlap { lo, hi, x_lo, x_hi } => {
                let end = |s: f64, x: [f64; N]| Raw { s, x, d: 0.0 };
                (r.s >= lo - e && r.s <= hi + e)
                    || ctx.same_point(r, &end(*lo, *x_lo))
                    || ctx.same_point(r, &end(*hi, *x_hi))
            }
            Found::Point(_) => false,
        })
    };
    points.retain(|r| !inside(r));
    points.sort_by(|p, q| by_key(Found::Point(*p).key(), Found::Point(*q).key()));
    let mut kept: Vec<Raw<N>> = Vec::new();
    for p in points {
        if let Some(q) = kept.iter_mut().find(|q| ctx.same_point(q, &p)) {
            if p.d < q.d {
                *q = p;
            }
        } else {
            kept.push(p);
        }
    }
    let mut out: Vec<Found<N>> = joined;
    out.extend(kept.into_iter().map(Found::Point));
    out.sort_by(|p, q| by_key(p.key(), q.key()));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Circle2, Line2};
    use bevel_math::{Point2, Vector2};
    use std::f64::consts::{FRAC_PI_2, TAU};

    fn line(o: (f64, f64), d: (f64, f64)) -> Curve2 {
        Curve2::Line(Line2::new(Point2::new(o.0, o.1), Vector2::new(d.0, d.1)).unwrap())
    }

    #[test]
    fn minimize_is_newton_at_a_crossing() {
        let a = line((0.0, 0.0), (1.0, 0.0));
        let b = line((3.0, -2.0), (1.0, 1.0));
        let bounds = [Interval::new(-10.0, 10.0), Interval::new(-10.0, 10.0)];
        let (x, d) = minimize(|v| curve_residual(&a, &b, v), [0.0, 0.0], &bounds, 10.0).unwrap();
        assert!(d < 1e-14 && (x[0] - 5.0).abs() < 1e-14, "{x:?} {d}");
        // Kept inside the box: the closest pair of points on its side.
        let short = [Interval::new(-10.0, 4.0), Interval::new(-10.0, 10.0)];
        let (x, d) = minimize(|v| curve_residual(&a, &b, v), [0.0, 0.0], &short, 10.0).unwrap();
        assert!(
            (x[0] - 4.0).abs() < 1e-9 && (d - 0.5_f64.sqrt()).abs() < 1e-12,
            "{x:?} {d}"
        );
    }

    #[test]
    fn minimize_reaches_a_shallow_tangency() {
        // A circle of radius 1000 touching a line, both as NURBS: the
        // Jacobian degenerates at the root and is badly scaled.
        let big = Curve2::Circle(Circle2::new(Point2::new(0.0, 1000.0), 1000.0).unwrap());
        let range = Interval::new(-FRAC_PI_2 - 0.2, -FRAC_PI_2 + 0.2);
        let b = Curve2::Nurbs(big.to_nurbs(range).unwrap());
        let a = Curve2::Nurbs(
            line((-50.0, 0.0), (1.0, 0.0))
                .to_nurbs(Interval::new(0.0, 100.0))
                .unwrap(),
        );
        let bounds = [Interval::new(0.0, 100.0), range];
        for start in [[50.1, -FRAC_PI_2 + 1e-4], [49.0, -FRAC_PI_2 - 1e-3]] {
            let (x, d) = minimize(|v| curve_residual(&a, &b, v), start, &bounds, 200.0).unwrap();
            assert!(d < 1e-12 && (x[0] - 50.0).abs() < 1e-4, "{x:?} {d}");
        }
    }

    #[test]
    fn copies_cover_the_range() {
        let r = Interval::new(-1.0, 2.0 * TAU);
        let c = copies(1.0, r, TAU, 0.0);
        assert_eq!(c.len(), 2);
        assert!((c[0] - 1.0).abs() < 1e-15 && (c[1] - 1.0 - TAU).abs() < 1e-12);
        // None inside: the nearer end.
        let c = copies(3.0, Interval::new(0.0, 2.9), TAU, 0.0);
        assert_eq!(c.len(), 1);
        assert!((c[0] - 2.9).abs() < 1e-15);
        assert!(copies_inside(3.0, Interval::new(0.0, 2.9), TAU, 0.0).is_empty());
    }

    #[test]
    fn segment_distances() {
        let p = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
        let d = segment_distance(
            p(0.0, 0.0, 0.0),
            p(1.0, 0.0, 0.0),
            p(0.5, -1.0, 1.0),
            p(0.5, 1.0, 1.0),
        );
        assert!((d - 1.0).abs() < 1e-15);
        let d = segment_distance(
            p(0.0, 0.0, 0.0),
            p(1.0, 0.0, 0.0),
            p(2.0, 0.0, 0.0),
            p(3.0, 0.0, 0.0),
        );
        assert!((d - 1.0).abs() < 1e-15);
        let d = segment_distance(
            p(0.0, 0.0, 0.0),
            p(0.0, 0.0, 0.0),
            p(0.0, 2.0, 0.0),
            p(1.0, 2.0, 0.0),
        );
        assert!((d - 2.0).abs() < 1e-15);
    }

    #[test]
    fn shapes_bound_the_tangents() {
        let arc = Curve2::Circle(Circle2::new(Point2::ZERO, 1.0).unwrap());
        let (_, points) = arc.hull(Interval::new(0.0, FRAC_PI_2)).unwrap();
        let shape = shape_of(&points.unwrap()).unwrap();
        // The end tangents are 45 degrees off the chord.
        assert!(shape.half >= std::f64::consts::FRAC_PI_4);
        assert!(shape.dev >= 1.0 - 0.5_f64.sqrt());
        assert!(shape_of(&[DVec3::ZERO, DVec3::X, DVec3::ZERO]).is_none());
    }
}
