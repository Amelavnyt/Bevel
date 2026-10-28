// SPDX-License-Identifier: GPL-3.0-or-later

//! The general surface–surface method, for any pair of surfaces.
//!
//! 1. **Seeds.** The two parameter rectangles are subdivided together,
//!    dropping pairs of pieces whose boxes do not overlap, down to pieces a
//!    sixty-fourth of the patches' size; Newton's method from each
//!    remaining pair finds a point on both surfaces.
//! 2. **Marching.** From each seed not already on a traced branch, the
//!    branch is followed both ways by predictor–corrector steps along the
//!    cross product of the normals: a step along the tangent, then Newton
//!    back onto both surfaces within the plane through the predicted point
//!    normal to the tangent. The step adapts to the curvature so that a
//!    cubic through the samples stays within tolerance. A branch ends where
//!    it leaves either rectangle or closes on itself.
//! 3. **Fitting.** Each branch becomes a cubic NURBS curve through its
//!    samples, and each pcurve a cubic through the samples' parameters at
//!    the same parameter values, so the three share a parameter. Midpoints
//!    of every span are checked against the exact intersection, and spans
//!    out of tolerance get more samples until all pass.
//!
//! Where the normals are parallel the intersection is not a transversal
//! curve: an isolated touching point is reported as such, and a tangency
//! along a curve is refused with [`GeomError::TangentContact`].

use super::super::{Contact, Rect, SsiCurve, SsiPoint};
use crate::{Curve2, Curve3, GeomError, Surface, SurfaceGeom, fit};
use bevel_math::{Aabb3, Interval, Point2, Point3, Vector3};
use nalgebra::{Matrix3, Matrix4, Vector4};

/// Subdivision depth limit (each level halves both directions).
const MAX_DEPTH: u32 = 24;
/// Box evaluations allowed while seeding.
const MAX_BOX_WORK: usize = 400_000;
/// Marching steps allowed per branch direction.
const MAX_STEPS: usize = 200_000;
/// Fitting rounds allowed per branch.
const MAX_FIT_ROUNDS: usize = 12;
/// Largest turn of the tangent per marching step, in radians.
const MAX_TURN: f64 = 0.15;
/// Below this sine of the angle between the normals the surfaces are
/// treated as tangent.
const TANGENT_SINE: f64 = 1e-7;

/// A point on both surfaces with its parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Pt {
    pub p: Point3,
    pub a: Point2,
    pub b: Point2,
}

/// The two patches and the tolerance.
pub(super) struct Problem<'s> {
    pub a: &'s Surface,
    pub ra: Rect,
    pub b: &'s Surface,
    pub rb: Rect,
    pub tol: f64,
    /// The larger diagonal of the patches' boxes.
    pub size: f64,
}

/// Whether every side of `r` that `uv` lies beyond is a single point of
/// `s` (a pole), so the parameters left `r` only by stepping over it.
fn past_pole(s: &Surface, r: &Rect, uv: Point2, tol: f64) -> bool {
    let point_side =
        |a: Point3, m: Point3, b: Point3| (a - m).length() <= tol && (b - m).length() <= tol;
    let side = |x: f64, i: Interval| {
        (x < i.min)
            .then_some(i.min)
            .or((x > i.max).then_some(i.max))
    };
    let u_ok = side(uv.x, r.u).is_none_or(|u| {
        point_side(
            s.point(u, r.v.min),
            s.point(u, r.v.mid()),
            s.point(u, r.v.max),
        )
    });
    let v_ok = side(uv.y, r.v).is_none_or(|v| {
        point_side(
            s.point(r.u.min, v),
            s.point(r.u.mid(), v),
            s.point(r.u.max, v),
        )
    });
    u_ok && v_ok
}

impl Problem<'_> {
    /// Whether `pt`'s parameters lie in both rectangles, allowing a tiny
    /// relative slack for round-off.
    fn inside(&self, pt: &Pt) -> bool {
        let slack = |r: Interval| 1e-9 * r.length().max(1.0);
        self.ra.u.contains_with(pt.a.x, slack(self.ra.u))
            && self.ra.v.contains_with(pt.a.y, slack(self.ra.v))
            && self.rb.u.contains_with(pt.b.x, slack(self.rb.u))
            && self.rb.v.contains_with(pt.b.y, slack(self.rb.v))
    }

    /// Whether `pt` lies on a side of either rectangle (where a branch
    /// may end).
    fn on_side(&self, pt: &Pt) -> bool {
        let near = |x: f64, r: Interval| {
            let eps = 1e-6 * r.length().max(1.0);
            (x - r.min).abs() <= eps || (x - r.max).abs() <= eps
        };
        near(pt.a.x, self.ra.u)
            || near(pt.a.y, self.ra.v)
            || near(pt.b.x, self.rb.u)
            || near(pt.b.y, self.rb.v)
    }

    /// The sine of the angle between the normals at `pt` (1 if a normal
    /// is undefined).
    fn normal_sine(&self, pt: &Pt) -> f64 {
        let (au, av) = self.a.partials(pt.a.x, pt.a.y);
        let (bu, bv) = self.b.partials(pt.b.x, pt.b.y);
        let (na, nb) = (au.cross(av), bu.cross(bv));
        let scale = na.length() * nb.length();
        if scale > 0.0 {
            na.cross(nb).length() / scale
        } else {
            1.0
        }
    }

    /// The unit tangent of the intersection at `pt`, or `None` where the
    /// normals are parallel.
    fn tangent(&self, pt: &Pt) -> Option<Vector3> {
        let (au, av) = self.a.partials(pt.a.x, pt.a.y);
        let (bu, bv) = self.b.partials(pt.b.x, pt.b.y);
        let (na, nb) = (au.cross(av), bu.cross(bv));
        let t = na.cross(nb);
        let scale = na.length() * nb.length();
        (scale > 0.0 && t.length() > TANGENT_SINE * scale).then(|| t / t.length())
    }

    /// Newton's method onto both surfaces from parameters `a` and `b`,
    /// taking the smallest parameter steps (the point found is the nearest
    /// intersection point, not a particular one).
    pub(super) fn refine(&self, mut a: Point2, mut b: Point2) -> Option<Pt> {
        let target = 1e-3 * self.tol;
        for _ in 0..40 {
            let (pa, pb) = (self.a.point(a.x, a.y), self.b.point(b.x, b.y));
            let f = pa - pb;
            if !f.is_finite() {
                return None;
            }
            if f.length() <= target {
                return Some(Pt { p: pa, a, b });
            }
            let (au, av) = self.a.partials(a.x, a.y);
            let (bu, bv) = self.b.partials(b.x, b.y);
            let cols = [au, av, -bu, -bv];
            // Minimum-norm step: dx = -J^T (J J^T)^-1 f.
            let mut m = Matrix3::zeros();
            for c in &cols {
                let c = c.to_array();
                for i in 0..3 {
                    for j in 0..3 {
                        m[(i, j)] += c[i] * c[j];
                    }
                }
            }
            // Regularize: at a tangency J J^T loses rank.
            let trace = m.trace();
            for i in 0..3 {
                m[(i, i)] += 1e-14 * trace;
            }
            let y = m.lu().solve(&nalgebra::Vector3::new(f.x, f.y, f.z))?;
            let y = Vector3::new(y[0], y[1], y[2]);
            let dx: Vec<f64> = cols.iter().map(|c| -c.dot(y)).collect();
            a += Point2::new(dx[0], dx[1]);
            b += Point2::new(dx[2], dx[3]);
        }
        let (pa, pb) = (self.a.point(a.x, a.y), self.b.point(b.x, b.y));
        ((pa - pb).length() <= 1e-2 * self.tol).then_some(Pt { p: pa, a, b })
    }

    /// `p` with its parameters found afresh by inversion (following
    /// periods from `near`), if that puts it inside both rectangles.
    fn reinvert(&self, p: &Pt, near: &Pt) -> Option<Pt> {
        // Only a step over a pole: through any other side the curve
        // really leaves the rectangle (a closed surface that is not
        // periodic has a seam there, and the parameters must not jump).
        if !past_pole(self.a, &self.ra, p.a, self.tol)
            || !past_pole(self.b, &self.rb, p.b, self.tol)
        {
            return None;
        }
        let ca = self.a.closest_point(p.p).ok()?;
        let cb = self.b.closest_point(p.p).ok()?;
        if ca.distance > self.tol || cb.distance > self.tol {
            return None;
        }
        let q = Pt {
            p: p.p,
            a: continuous(self.a, Point2::new(ca.u, ca.v), near.a),
            b: continuous(self.b, Point2::new(cb.u, cb.v), near.b),
        };
        self.inside(&q).then_some(q)
    }

    /// Whether the first-order prediction of a step of `h` along `dir`
    /// from `from` leaves either rectangle.
    fn predicted_outside(&self, from: &Pt, dir: Vector3, h: f64) -> bool {
        let (au, av) = self.a.partials(from.a.x, from.a.y);
        let (bu, bv) = self.b.partials(from.b.x, from.b.y);
        match (
            solve_tangent(au, av, dir * h),
            solve_tangent(bu, bv, dir * h),
        ) {
            (Some(da), Some(db)) => !self.inside(&Pt {
                p: from.p,
                a: from.a + da,
                b: from.b + db,
            }),
            _ => false,
        }
    }

    /// One marching step: the point at distance about `h` from `from` along
    /// `dir`, on both surfaces, in the plane normal to `dir` through the
    /// predicted point.
    fn step(&self, from: &Pt, dir: Vector3, h: f64) -> Option<Pt> {
        let x = from.p + dir * h;
        let (au, av) = self.a.partials(from.a.x, from.a.y);
        let (bu, bv) = self.b.partials(from.b.x, from.b.y);
        let linear = solve_tangent(au, av, dir * h)
            .zip(solve_tangent(bu, bv, dir * h))
            .and_then(|(da, db)| self.correct(from, dir, x, from.a + da, from.b + db));
        // Near a pole the parameters change too fast for a linear
        // prediction: start from the predicted point inverted onto both
        // surfaces instead.
        linear.or_else(|| {
            let ca = self.a.closest_point(x).ok()?;
            let cb = self.b.closest_point(x).ok()?;
            let a = continuous(self.a, Point2::new(ca.u, ca.v), from.a);
            let b = continuous(self.b, Point2::new(cb.u, cb.v), from.b);
            // The parameters may jump only where they move the point
            // little (around a pole), never across the seam of a closed
            // surface that is not periodic.
            let small = |s: &Surface, p: Point2, q: Point2| {
                let (su, sv) = s.partials(p.x, p.y);
                (q.x - p.x).abs() * su.length() + (q.y - p.y).abs() * sv.length() <= 8.0 * h
            };
            if !small(self.a, from.a, a) || !small(self.b, from.b, b) {
                return None;
            }
            self.correct(from, dir, x, a, b)
        })
    }

    /// Newton's method from parameters `a`, `b` onto both surfaces in the
    /// plane through `x` normal to `dir`.
    fn correct(&self, from: &Pt, dir: Vector3, x: Point3, a: Point2, b: Point2) -> Option<Pt> {
        let (mut a, mut b) = (a, b);
        let target = 1e-3 * self.tol;
        for _ in 0..16 {
            let (pa, pb) = (self.a.point(a.x, a.y), self.b.point(b.x, b.y));
            let f = pa - pb;
            let g = dir.dot(pa - x);
            if !f.is_finite() || !g.is_finite() {
                return None;
            }
            if f.length() <= target && g.abs() <= target {
                // Newton may land a whole period away (near a pole the
                // angle moves fast); keep the parameters continuous so a
                // seam crossing shows as leaving the rectangle.
                return Some(Pt {
                    p: pa,
                    a: continuous(self.a, a, from.a),
                    b: continuous(self.b, b, from.b),
                });
            }
            let (au, av) = self.a.partials(a.x, a.y);
            let (bu, bv) = self.b.partials(b.x, b.y);
            #[rustfmt::skip]
            let j = Matrix4::new(
                au.x, av.x, -bu.x, -bv.x,
                au.y, av.y, -bu.y, -bv.y,
                au.z, av.z, -bu.z, -bv.z,
                dir.dot(au), dir.dot(av), 0.0, 0.0,
            );
            let d = j.lu().solve(&Vector4::new(-f.x, -f.y, -f.z, -g))?;
            a += Point2::new(d[0], d[1]);
            b += Point2::new(d[2], d[3]);
        }
        None
    }
}

/// `p` shifted by whole periods of `s` to lie nearest `near`.
fn continuous(s: &Surface, p: Point2, near: Point2) -> Point2 {
    let shift = |x: f64, n: f64, period: Option<f64>| match period {
        Some(per) if per > 0.0 => x + ((n - x) / per).round() * per,
        _ => x,
    };
    Point2::new(
        shift(p.x, near.x, s.period_u()),
        shift(p.y, near.y, s.period_v()),
    )
}

/// The parameter change `(du, dv)` whose image under the partials `su`,
/// `sv` is closest to `d` (least squares).
fn solve_tangent(su: Vector3, sv: Vector3, d: Vector3) -> Option<Point2> {
    let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
    let det = a * c - b * b;
    if det.abs() <= 1e-300 || !det.is_finite() {
        return None;
    }
    let (r1, r2) = (su.dot(d), sv.dot(d));
    Some(Point2::new(
        (c * r1 - b * r2) / det,
        (a * r2 - b * r1) / det,
    ))
}

fn diagonal(b: &Aabb3) -> f64 {
    (b.max - b.min).length()
}

fn grown(b: Aabb3, d: f64) -> Aabb3 {
    Aabb3::new(b.min - Vector3::splat(d), b.max + Vector3::splat(d))
}

fn halves(r: Interval) -> [Interval; 2] {
    let m = r.mid();
    [Interval::new(r.min, m), Interval::new(m, r.max)]
}

fn quarters(r: Rect) -> [Rect; 4] {
    let [u0, u1] = halves(r.u);
    let [v0, v1] = halves(r.v);
    [
        Rect::new(u0, v0),
        Rect::new(u1, v0),
        Rect::new(u0, v1),
        Rect::new(u1, v1),
    ]
}

fn centre(r: &Rect) -> Point2 {
    Point2::new(r.u.mid(), r.v.mid())
}

/// Seed points: at least one on every branch longer than about a
/// sixty-fourth of the patches' size, in a deterministic order.
pub(super) fn seeds(pr: &Problem<'_>) -> Result<Vec<Pt>, GeomError> {
    let leaf = pr.size / 64.0;
    let mut stack = vec![(pr.ra, pr.rb, 0u32)];
    let mut leaves = Vec::new();
    let mut work = 0;
    while let Some((ra, rb, depth)) = stack.pop() {
        work += 1;
        if work > MAX_BOX_WORK {
            return Err(GeomError::NotConverged);
        }
        let ba = grown(pr.a.bounds(ra.u, ra.v)?, pr.tol);
        let bb = grown(pr.b.bounds(rb.u, rb.v)?, pr.tol);
        if !ba.intersects(&bb) {
            continue;
        }
        let (da, db) = (diagonal(&ba), diagonal(&bb));
        if depth >= MAX_DEPTH || (da <= leaf && db <= leaf) {
            leaves.push((ra, rb));
            continue;
        }
        // Push in reverse so pieces are visited in a fixed order.
        if da >= db {
            for q in quarters(ra).into_iter().rev() {
                stack.push((q, rb, depth + 1));
            }
        } else {
            for q in quarters(rb).into_iter().rev() {
                stack.push((ra, q, depth + 1));
            }
        }
    }
    let mut seeds: Vec<Pt> = Vec::new();
    for (ra, rb) in &leaves {
        let Some(pt) = pr.refine(centre(ra), centre(rb)) else {
            continue;
        };
        if pr.inside(&pt) && !seeds.iter().any(|s| (s.p - pt.p).length() <= 10.0 * pr.tol) {
            seeds.push(pt);
        }
    }
    Ok(seeds)
}

/// A traced branch: samples in order; a branch that closes on itself ends
/// with its first sample again.
pub(super) struct Branch {
    pub points: Vec<Pt>,
}

/// Whether `p` lies on the polyline of `branch`, within a quarter of the
/// nearest segment's length (samples are spaced so that the curve stays
/// much closer than that to its chords).
fn on_branch(branch: &Branch, q: &Pt, tol: f64) -> bool {
    let p = q.p;
    branch.points.windows(2).any(|w| {
        let (s, e) = (w[0].p, w[1].p);
        let d = e - s;
        let len2 = d.length_squared();
        let t = if len2 > 0.0 {
            ((p - s).dot(d) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        // Near in space and on the same sheet of parameters (the same
        // point across a periodic seam is another part of the curve).
        let near = (s + d * t - p).length() <= 0.25 * len2.sqrt() + 10.0 * tol;
        let mid = |x: Point2, y: Point2| x + (y - x) * t;
        let span = |x: Point2, y: Point2| (y - x).length();
        near && (mid(w[0].a, w[1].a) - q.a).length() <= span(w[0].a, w[1].a) + 1e-6
            && (mid(w[0].b, w[1].b) - q.b).length() <= span(w[0].b, w[1].b) + 1e-6
    })
}

/// The step that keeps a cubic through the samples within `tol` of a
/// curve of curvature `kappa`, and the turn per step under [`MAX_TURN`].
fn step_for(kappa: f64, tol: f64, h_min: f64, h_max: f64) -> f64 {
    if kappa <= 1e-12 {
        return h_max;
    }
    let cubic = (384.0 * tol / (kappa * kappa * kappa)).powf(0.25) * 0.7;
    cubic.min(MAX_TURN / kappa).clamp(h_min, h_max)
}

/// Follows a branch from `seed` in the direction `sign` gives.
fn trace(pr: &Problem<'_>, seed: Pt, sign: f64) -> Result<(Vec<Pt>, bool), GeomError> {
    let h_max = pr.size / 8.0;
    let h_min = (10.0 * pr.tol).min(h_max);
    let mut h = (pr.size / 64.0).clamp(h_min, h_max);
    let mut points = vec![seed];
    let Some(t0) = pr.tangent(&seed) else {
        return Err(GeomError::TangentContact);
    };
    let mut dir = t0 * sign;
    let mut travelled = 0.0;
    for _ in 0..MAX_STEPS {
        let Some(cur) = points.last().copied() else {
            return Err(GeomError::NotConverged);
        };
        let next = pr.step(&cur, dir, h).and_then(|n| {
            let t = pr.tangent(&n)?;
            let t = if t.dot(dir) < 0.0 { -t } else { t };
            let turn = dir.dot(t).clamp(-1.0, 1.0).acos();
            (turn <= MAX_TURN).then_some((n, t, turn))
        });
        let Some((next, t_next, turn)) = next else {
            // A step that would leave a rectangle may fail outright (a NURBS
            // surface is not defined past its domain): once the step has
            // shrunk and still crosses the boundary, end there. A long
            // failed step only means the step was too long.
            if h <= 64.0 * h_min && pr.predicted_outside(&cur, dir, h) {
                let end = boundary(pr, &cur, dir, h);
                if pr.on_side(&end) || h <= h_min {
                    points.push(end);
                    return Ok((points, false));
                }
                // The bisection stopped short of the side (steps failing
                // on the way, near a pole): march on from there.
                h = (h * 0.25).max(h_min);
                if let Some(t) = pr.tangent(&end).filter(|_| end.p != cur.p) {
                    dir = if t.dot(dir) < 0.0 { -t } else { t };
                    points.push(end);
                }
                continue;
            }
            if h <= h_min {
                // Parallel normals ahead: a tangency the method cannot cross.
                return Err(if pr.step(&cur, dir, h).is_some() {
                    GeomError::TangentContact
                } else {
                    GeomError::NotConverged
                });
            }
            h = (h * 0.5).max(h_min);
            continue;
        };
        // A step over a pole lands on parameters outside the rectangle
        // for a point inside it (latitude past 90 degrees): use the
        // point's parameters in the rectangle instead.
        let next = if pr.inside(&next) {
            next
        } else {
            pr.reinvert(&next, &cur).unwrap_or(next)
        };
        if !pr.inside(&next) {
            let end = boundary(pr, &cur, dir, h);
            if pr.on_side(&end) || h <= h_min {
                points.push(end);
                return Ok((points, false));
            }
            h = (h * 0.25).max(h_min);
            if let Some(t) = pr.tangent(&end).filter(|_| end.p != cur.p) {
                dir = if t.dot(dir) < 0.0 { -t } else { t };
                points.push(end);
            }
            continue;
        }
        // The branch runs into a region where the surfaces are nearly
        // tangent: the samples would no longer pin the curve down.
        if pr.normal_sine(&next) < NEAR_TANGENT_SINE {
            return Err(GeomError::TangentContact);
        }
        let chord = (next.p - cur.p).length();
        travelled += chord;
        // Back at the seed after going round: the branch is a loop. The
        // seed may lie just ahead of the new point or within the step just
        // taken. Near in space is not enough: across a periodic seam the
        // parameters differ by a period and the branch must end there.
        // Same sheet: shifting the seed's parameters by whole periods to
        // follow `q` leaves them unchanged.
        let params_back = |q: &Pt| {
            continuous(pr.a, seed.a, q.a) == seed.a && continuous(pr.b, seed.b, q.b) == seed.b
        };
        if points.len() > 3 && travelled > 2.0 * h {
            let seg = next.p - cur.p;
            let s = (seed.p - cur.p).dot(seg) / seg.length_squared().max(f64::MIN_POSITIVE);
            let off = (cur.p + seg * s.clamp(0.0, 1.0) - seed.p).length();
            let passed = (0.0..=1.0).contains(&s)
                && off <= 0.25 * chord + 10.0 * pr.tol
                && params_back(&cur);
            let to_seed = seed.p - next.p;
            let ahead = to_seed.length() <= h && to_seed.dot(t_next) > 0.0 && params_back(&next);
            if passed {
                points.push(seed);
                return Ok((points, true));
            }
            if ahead {
                points.push(next);
                points.push(seed);
                return Ok((points, true));
            }
        }
        points.push(next);
        dir = t_next;
        let kappa = if chord > 0.0 { turn / chord } else { 0.0 };
        h = step_for(kappa, pr.tol, h_min, h_max).min(2.0 * h);
    }
    Err(GeomError::NotConverged)
}

/// The last point before the branch leaves a rectangle, found by bisecting
/// the step length.
fn boundary(pr: &Problem<'_>, from: &Pt, dir: Vector3, h: f64) -> Pt {
    let (mut lo, mut hi) = (0.0, h);
    let mut best = *from;
    for _ in 0..60 {
        if hi - lo <= 0.1 * pr.tol {
            break;
        }
        let mid = 0.5 * (lo + hi);
        match pr.step(from, dir, mid) {
            Some(p) if pr.inside(&p) => {
                lo = mid;
                best = p;
            }
            _ => hi = mid,
        }
    }
    best
}

/// Below this sine of the angle between the normals (about 0.06 degrees)
/// the surfaces count as tangent: seeds there are not marched from, and a
/// branch that reaches such a point is refused.
const NEAR_TANGENT_SINE: f64 = 1e-3;

/// Traces every branch through the seeds. Seeds where the surfaces are
/// nearly tangent are returned separately: they are touching points, or
/// part of a tangency along a curve.
pub(super) fn branches(
    pr: &Problem<'_>,
    seeds: &[Pt],
) -> Result<(Vec<Branch>, Vec<Pt>), GeomError> {
    let mut out: Vec<Branch> = Vec::new();
    let mut touching = Vec::new();
    // Seeds where branches continue across a periodic seam come first, so
    // the parts of a curve on both sides of a seam are always traced, small
    // as one of them may be.
    let mut queue: std::collections::VecDeque<Pt> = seeds.iter().copied().collect();
    while let Some(seed) = queue.pop_front() {
        if !pr.inside(&seed) || out.iter().any(|b| on_branch(b, &seed, pr.tol)) {
            continue;
        }
        // Too close to tangent to march from: a touching point, or part of
        // a tangency along a curve; `touching` tells which.
        if pr.normal_sine(&seed) < NEAR_TANGENT_SINE {
            touching.push(seed);
            continue;
        }
        let (forward, closed) = trace(pr, seed, 1.0)?;
        let points = if closed {
            forward
        } else {
            let (backward, _) = trace(pr, seed, -1.0)?;
            let mut points: Vec<Pt> = backward.into_iter().skip(1).rev().collect();
            points.extend(forward);
            points
        };
        if !closed {
            for end in [points.first(), points.last()].into_iter().flatten() {
                for across in across_seams(pr, end) {
                    queue.push_front(across);
                }
            }
        }
        out.push(Branch { points });
    }
    Ok((out, touching))
}

/// The same point on the other side of each periodic seam it lies on:
/// where a rectangle spans a whole period, a branch that leaves through
/// one side continues from the other.
fn across_seams(pr: &Problem<'_>, end: &Pt) -> Vec<Pt> {
    let mut out = Vec::new();
    let wrap = |x: f64, r: Interval, period: Option<f64>| -> Option<f64> {
        let per = period?;
        if (r.length() - per).abs() > 1e-9 * per {
            return None;
        }
        let eps = 1e-6 * per;
        if (x - r.min).abs() <= eps {
            Some(x + per)
        } else if (x - r.max).abs() <= eps {
            Some(x - per)
        } else {
            None
        }
    };
    if let Some(u) = wrap(end.a.x, pr.ra.u, pr.a.period_u()) {
        out.push(Pt {
            a: Point2::new(u, end.a.y),
            ..*end
        });
    }
    if let Some(v) = wrap(end.a.y, pr.ra.v, pr.a.period_v()) {
        out.push(Pt {
            a: Point2::new(end.a.x, v),
            ..*end
        });
    }
    if let Some(u) = wrap(end.b.x, pr.rb.u, pr.b.period_u()) {
        out.push(Pt {
            b: Point2::new(u, end.b.y),
            ..*end
        });
    }
    if let Some(v) = wrap(end.b.y, pr.rb.v, pr.b.period_v()) {
        out.push(Pt {
            b: Point2::new(end.b.x, v),
            ..*end
        });
    }
    out
}

/// Fits a branch: the curve and both pcurves through its samples at shared
/// parameters, refined until every span is within tolerance.
pub(super) fn fit_branch(pr: &Problem<'_>, branch: &Branch) -> Result<Option<SsiCurve>, GeomError> {
    let mut points = branch.points.clone();
    points.dedup_by(|a, b| (a.p - b.p).length() <= 1e-3 * pr.tol);
    settle_singular_ends(pr, &mut points);
    if points.len() < 2 {
        return Ok(None);
    }
    let length: f64 = points.windows(2).map(|w| (w[1].p - w[0].p).length()).sum();
    if length <= pr.tol {
        return Ok(None);
    }
    for _ in 0..MAX_FIT_ROUNDS {
        // A cubic needs four samples; split the longest span until there.
        while points.len() < 4 {
            let i = longest_span(&points);
            let mid = midpoint(pr, &points[i], &points[i + 1])?;
            points.insert(i + 1, mid);
        }
        let params = fit::parameters(
            &points.iter().map(|p| p.p).collect::<Vec<_>>(),
            fit::Parametrization::ChordLength,
        )?;
        let degree = 3;
        let curve = fit::interpolate_at(
            &points.iter().map(|p| p.p).collect::<Vec<_>>(),
            &params,
            degree,
        )?;
        let pa = fit::interpolate_at(
            &points.iter().map(|p| p.a).collect::<Vec<_>>(),
            &params,
            degree,
        )?;
        let pb = fit::interpolate_at(
            &points.iter().map(|p| p.b).collect::<Vec<_>>(),
            &params,
            degree,
        )?;
        let mut worst = 0.0_f64;
        let mut bad = Vec::new();
        for (i, w) in params.windows(2).enumerate() {
            // Spans this short cannot stray from the curve, and refining
            // them only piles samples onto singular points such as poles.
            if (points[i + 1].p - points[i].p).length() <= 10.0 * pr.tol {
                continue;
            }
            let t = 0.5 * (w[0] + w[1]);
            let (c, ua, ub) = (curve.point(t), pa.point(t), pb.point(t));
            // What the result promises: the curve lies on both surfaces,
            // and each pcurve maps onto the curve.
            let (Ok(ca), Ok(cb)) = (pr.a.closest_point(c), pr.b.closest_point(c)) else {
                bad.push(i);
                continue;
            };
            let err = ca
                .distance
                .max(cb.distance)
                .max((pr.a.point(ua.x, ua.y) - c).length())
                .max((pr.b.point(ub.x, ub.y) - c).length());
            worst = worst.max(err);
            if err > pr.tol {
                bad.push(i);
            }
        }
        if bad.is_empty() {
            return Ok(Some(SsiCurve {
                curve: Curve3::Nurbs(curve),
                range: Interval::new(0.0, 1.0),
                pcurve_a: Curve2::Nurbs(pa),
                pcurve_b: Curve2::Nurbs(pb),
                tolerance: worst.max(1e-3 * pr.tol),
                contact: Contact::Transversal,
            }));
        }
        let mut added = false;
        for &i in bad.iter().rev() {
            let mid = midpoint(pr, &points[i], &points[i + 1])?;
            let apart = |q: &Pt| (q.p - mid.p).length() > 1e-3 * pr.tol;
            if apart(&points[i]) && apart(&points[i + 1]) {
                points.insert(i + 1, mid);
                added = true;
            }
        }
        if !added {
            break;
        }
    }
    Err(GeomError::ToleranceNotMet {
        error: f64::INFINITY,
    })
}

/// At a branch end on a singular point of a parametrization (a pole,
/// where one parameter can take any value), takes that parameter from the
/// neighbouring sample, the limit along the curve; the arbitrary value the
/// end was found with would put a jump in the pcurve.
fn settle_singular_ends(pr: &Problem<'_>, points: &mut [Pt]) {
    let n = points.len();
    if n < 2 {
        return;
    }
    let degenerate = |s: &Surface, p: Point2| {
        let (su, sv) = s.partials(p.x, p.y);
        let scale = su.length().max(sv.length()).max(f64::MIN_POSITIVE);
        (su.length() <= 1e-4 * scale, sv.length() <= 1e-4 * scale)
    };
    for (end, next) in [(0, 1), (n - 1, n - 2)] {
        let neighbour = points[next];
        let p = &mut points[end];
        let (du, dv) = degenerate(pr.a, p.a);
        if du {
            p.a.x = neighbour.a.x;
        }
        if dv {
            p.a.y = neighbour.a.y;
        }
        let (du, dv) = degenerate(pr.b, p.b);
        if du {
            p.b.x = neighbour.b.x;
        }
        if dv {
            p.b.y = neighbour.b.y;
        }
    }
}

fn longest_span(points: &[Pt]) -> usize {
    points
        .windows(2)
        .enumerate()
        .max_by(|(_, x), (_, y)| {
            (x[1].p - x[0].p)
                .length()
                .total_cmp(&(y[1].p - y[0].p).length())
        })
        .map_or(0, |(i, _)| i)
}

/// The intersection point between two samples, by Newton from the mean of
/// their parameters.
fn midpoint(pr: &Problem<'_>, s: &Pt, e: &Pt) -> Result<Pt, GeomError> {
    pr.refine((s.a + e.a) * 0.5, (s.b + e.b) * 0.5)
        .ok_or(GeomError::NotConverged)
}

/// Groups touching candidates: a group that stays small is an isolated
/// touching point; a long one is a tangency along a curve.
pub(super) fn touching(
    pr: &Problem<'_>,
    candidates: &[Pt],
    branches: &[Branch],
) -> Result<Vec<SsiPoint>, GeomError> {
    let mut groups: Vec<Vec<Pt>> = Vec::new();
    for s in candidates {
        // A touching point on a traced curve is part of that curve.
        if branches.iter().any(|b| on_branch(b, s, pr.tol)) {
            continue;
        }
        let near = groups
            .iter_mut()
            .find(|g| g.iter().any(|o| (o.p - s.p).length() <= pr.size / 16.0));
        match near {
            Some(g) => g.push(*s),
            None => groups.push(vec![*s]),
        }
    }
    let mut points = Vec::new();
    for g in groups {
        let extent = g
            .iter()
            .flat_map(|x| g.iter().map(move |y| (x.p - y.p).length()))
            .fold(0.0, f64::max);
        if extent > pr.size / 32.0 {
            return Err(GeomError::TangentContact);
        }
        // The member where the normals are closest to parallel is nearest
        // the true contact.
        if let Some(best) = g
            .iter()
            .min_by(|x, y| pr.normal_sine(x).total_cmp(&pr.normal_sine(y)))
        {
            points.push(SsiPoint {
                point: best.p,
                a: best.a,
                b: best.b,
            });
        }
    }
    Ok(points)
}
