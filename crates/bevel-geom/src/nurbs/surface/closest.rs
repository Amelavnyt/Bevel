// SPDX-License-Identifier: GPL-3.0-or-later

//! Closest points on a surface (point inversion, The NURBS Book, section
//! 6.1), in two passes.
//!
//! 1. Seeding and Newton: every knot-span patch is sampled on a grid of
//!    `max(2 (p + 1), 4) + 1` by `max(2 (q + 1), 4) + 1` parameters, edges
//!    included (the book suggests a few samples per span and degree). The
//!    sampled local minima of the distance within each patch become seeds,
//!    and the best few are refined inside their patch with Newton's method
//!    on the two equations `f = S_u . r = 0`, `g = S_v . r = 0` with
//!    `r = S(u, v) - P` (eq. 6.5), whose Jacobian is
//!    `[[|S_u|^2 + r . S_uu, S_u . S_v + r . S_uv],
//!    [S_u . S_v + r . S_uv, |S_v|^2 + r . S_vv]]` (eqs. 6.6, 6.7).
//!    Iteration stops on the book's tests: point coincidence
//!    `|r| <= e1`, zero cosine `|S_u . r| <= e2 |S_u| |r|` and
//!    `|S_v . r| <= e2 |S_v| |r|`, or a parameter step that moves the point
//!    by at most `e1` (`|(du) S_u + (dv) S_v| <= e1`).
//!
//!    Steps are kept inside the seed's patch (so the iteration never
//!    crosses a knot line, where the surface may have a crease). On a patch
//!    edge, a step that would leave the patch is replaced by the Newton
//!    step along the edge, so minima on edges and corners converge as fast
//!    as interior ones. Where the Jacobian is not positive definite (away
//!    from a minimum) the Gauss-Newton matrix, the first fundamental form
//!    with a tiny regularisation, takes its place; it always gives a
//!    descent direction. Steps are halved until the distance stops
//!    increasing (up to rounding), so a refinement never ends worse than
//!    its seed.
//! 2. Branch and bound: sampling can step over a narrow feature that comes
//!    closer to `P`. The surface is split into Bezier patches (A5.7), and
//!    each gets a lower bound on its distance to `P`: the distance from `P`
//!    to a box around its control points, which contains the patch (the
//!    convex hull property). The larger of two boxes is used: the
//!    axis-aligned one, and one aligned with the patch's corner edges,
//!    which hugs a flat patch closely so the bound is tight to second
//!    order. Patches are taken in order of their bound and halved by de
//!    Casteljau subdivision (across the longer side) until no patch can beat
//!    the best point found by more than a tiny margin; a patch that does not
//!    contain the best point gets its own Newton refinement from its
//!    centre. The pass stops after a fixed budget of patches, which only
//!    matters for points equidistant from a whole region of the surface
//!    (the centre of a sphere), where any of them is a correct answer.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use bevel_math::{DVec3, DVec4, Frame3, Point3};

use crate::basis::{Row, basis_funs_raw};
use crate::{GeomError, NurbsSurface};

/// Number of seeds refined by Newton's method.
const NEWTON_SEEDS: usize = 16;

/// Newton iterations per seed.
const NEWTON_ITERATIONS: usize = 32;

/// Step halvings per Newton iteration.
const MAX_HALVINGS: usize = 16;

/// Zero-cosine tolerance `e2` for Newton's method.
const ZERO_COSINE: f64 = 1e-13;

/// Relative regularisation of the Gauss-Newton matrix.
const REGULARISATION: f64 = 1e-12;

/// Most Bezier patches examined by the branch-and-bound pass.
const BRANCH_BUDGET: usize = 512;

/// The point of a surface closest to a given point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceClosestPoint {
    /// Parameter `u` of the closest point.
    pub u: f64,
    /// Parameter `v` of the closest point.
    pub v: f64,
    /// The closest point on the surface.
    pub point: Point3,
    /// Distance from the given point to the surface.
    pub distance: f64,
}

/// A rectangle of the parameter domain inside one knot-span patch, with the
/// span indices of that patch.
#[derive(Clone, Copy, Debug)]
struct Cell {
    su: usize,
    sv: usize,
    lo_u: f64,
    hi_u: f64,
    lo_v: f64,
    hi_v: f64,
}

impl Cell {
    fn contains(&self, u: f64, v: f64) -> bool {
        self.lo_u <= u && u <= self.hi_u && self.lo_v <= v && v <= self.hi_v
    }
}

/// A starting point for the closest-point iteration.
struct Seed {
    u: f64,
    v: f64,
    distance: f64,
    cell: Cell,
}

/// A Bezier piece of the surface with a lower bound on its distance to the
/// target. Ordered so that a [`BinaryHeap`] pops the smallest bound first,
/// ties going to the piece created first.
struct Piece {
    lower: f64,
    seq: usize,
    cell: Cell,
    /// `(p + 1) x (q + 1)` homogeneous control points, row-major.
    control: Vec<DVec4>,
}

impl PartialEq for Piece {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Piece {}

impl PartialOrd for Piece {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Piece {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .lower
            .total_cmp(&self.lower)
            .then_with(|| other.seq.cmp(&self.seq))
    }
}

/// Sample parameters and their basis functions for every knot span of one
/// direction: `(span index, start, end, [(t, basis)])`.
type SpanSamples = Vec<(usize, f64, f64, Vec<(f64, Row)>)>;

impl NurbsSurface {
    /// The point of the surface closest to `target` (see the module
    /// documentation for the method).
    ///
    /// The result is never farther than the best of the sampled points (up
    /// to rounding of the distances), and is the global minimum unless the
    /// surface has a narrow dip closer to `target` that falls between
    /// samples and that the branch-and-bound budget does not reach.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    pub fn closest_point(&self, target: Point3) -> Result<SurfaceClosestPoint, GeomError> {
        if !target.is_finite() {
            return Err(GeomError::NonFinite);
        }
        let us = self.span_samples(true);
        let vs = self.span_samples(false);
        let mut best: Option<SurfaceClosestPoint> = None;
        let mut seeds: Vec<Seed> = Vec::new();
        let mut dist = Vec::new();
        for (su, lo_u, hi_u, u_samples) in &us {
            for (sv, lo_v, hi_v, v_samples) in &vs {
                let cell = Cell {
                    su: *su,
                    sv: *sv,
                    lo_u: *lo_u,
                    hi_u: *hi_u,
                    lo_v: *lo_v,
                    hi_v: *hi_v,
                };
                let cols = v_samples.len();
                dist.clear();
                for (u, bu) in u_samples {
                    for (v, bv) in v_samples {
                        let point = self.combine(*su, *sv, bu, bv).project();
                        let distance = (point - target).length();
                        if best.is_none_or(|b| distance < b.distance) {
                            best = Some(SurfaceClosestPoint {
                                u: *u,
                                v: *v,
                                point,
                                distance,
                            });
                        }
                        dist.push(distance);
                    }
                }
                // Local minima of the sampled distance within this patch.
                for (a, (u, _)) in u_samples.iter().enumerate() {
                    for (b, (v, _)) in v_samples.iter().enumerate() {
                        let d = dist[a * cols + b];
                        if is_local_min(&dist, u_samples.len(), cols, a, b) {
                            seeds.push(Seed {
                                u: *u,
                                v: *v,
                                distance: d,
                                cell,
                            });
                        }
                    }
                }
            }
        }
        seeds.sort_by(|x, y| x.distance.total_cmp(&y.distance));
        let Some(mut best) = best else {
            return Err(GeomError::NotConverged);
        };
        // Refined points win ties with samples up to rounding.
        let slack = self.distance_slack(target);
        for seed in seeds.iter().take(NEWTON_SEEDS) {
            let refined = self.newton_closest(seed, target);
            if refined.distance <= best.distance + slack {
                best = refined;
            }
        }
        self.branch_and_bound(target, &mut best);
        Ok(best)
    }

    /// Sample parameters with their basis functions for every non-empty knot
    /// span in `u` (or `v`) inside the domain.
    fn span_samples(&self, in_u: bool) -> SpanSamples {
        let (p, knots, (a, b)) = if in_u {
            (self.degree_u, &self.knots_u, self.domain().0)
        } else {
            (self.degree_v, &self.knots_v, self.domain().1)
        };
        let count = (2 * (p + 1)).max(4);
        let breaks: Vec<f64> = knots
            .breakpoints()
            .into_iter()
            .filter(|&t| t >= a && t <= b)
            .collect();
        breaks
            .windows(2)
            .map(|w| {
                let (lo, hi) = (w[0], w[1]);
                let span = knots.span(p, lo);
                let samples = (0..=count)
                    .map(|j| {
                        let t = if j == count {
                            hi
                        } else {
                            lo + (hi - lo) * j as f64 / count as f64
                        };
                        (t, basis_funs_raw(knots.as_slice(), span, t, p))
                    })
                    .collect();
                (span, lo, hi, samples)
            })
            .collect()
    }

    /// Rounding level of distances to `target`.
    fn distance_slack(&self, target: Point3) -> f64 {
        16.0 * f64::EPSILON * (self.size() + target.length())
    }

    /// The second pass of [`NurbsSurface::closest_point`]: looks for parts
    /// of the surface that could be closer than `best` and refines them.
    fn branch_and_bound(&self, target: Point3, best: &mut SurfaceClosestPoint) {
        let margin = 1e-12 * (self.size() + target.length());
        let (p, q) = (self.degree_u, self.degree_v);
        let mut heap = BinaryHeap::new();
        let mut seq = 0;
        for patch in self.decompose_bezier().into_iter().flatten() {
            let ((lo_u, hi_u), (lo_v, hi_v)) = patch.domain();
            let lower = patch_lower_bound(&patch.control, p, q, target);
            if lower < best.distance - margin {
                let (su, sv) = self.spans(lo_u, lo_v);
                heap.push(Piece {
                    lower,
                    seq,
                    cell: Cell {
                        su,
                        sv,
                        lo_u,
                        hi_u,
                        lo_v,
                        hi_v,
                    },
                    control: patch.control,
                });
                seq += 1;
            }
        }
        let mut examined = 0;
        while let Some(piece) = heap.pop() {
            if piece.lower >= best.distance - margin || examined >= BRANCH_BUDGET {
                break;
            }
            examined += 1;
            let c = piece.cell;
            let (mid_u, mid_v) = (0.5 * (c.lo_u + c.hi_u), 0.5 * (c.lo_v + c.hi_v));
            if !c.contains(best.u, best.v) {
                let seed = Seed {
                    u: mid_u,
                    v: mid_v,
                    distance: 0.0,
                    cell: c,
                };
                let refined = self.newton_closest(&seed, target);
                if refined.distance < best.distance {
                    *best = refined;
                }
            }
            let along_u = longer_in_u(&piece.control, p, q);
            let (lo, mid, hi) = if along_u {
                (c.lo_u, mid_u, c.hi_u)
            } else {
                (c.lo_v, mid_v, c.hi_v)
            };
            if !(mid > lo && mid < hi) {
                continue;
            }
            let (first, second) = split_patch(&piece.control, p, q, along_u);
            let cells = if along_u {
                [Cell { hi_u: mid, ..c }, Cell { lo_u: mid, ..c }]
            } else {
                [Cell { hi_v: mid, ..c }, Cell { lo_v: mid, ..c }]
            };
            for (control, cell) in [first, second].into_iter().zip(cells) {
                let lower = patch_lower_bound(&control, p, q, target);
                if lower < best.distance - margin {
                    heap.push(Piece {
                        lower,
                        seq,
                        cell,
                        control,
                    });
                    seq += 1;
                }
            }
        }
    }

    /// Newton refinement of a closest-point seed inside its cell (The NURBS
    /// Book, eqs. 6.5 to 6.7, with the safeguards of the module
    /// documentation).
    fn newton_closest(&self, seed: &Seed, target: Point3) -> SurfaceClosestPoint {
        let c = seed.cell;
        let e1 = 1e-12 * self.size();
        let slack = self.distance_slack(target);
        let (floor_u, floor_v) = self.speed_floors();
        let (mut u, mut v) = (seed.u, seed.v);
        let mut d = self.derivatives2_in_spans(u, v, c.su, c.sv);
        let mut dist = (d[0][0] - target).length();
        let mut best = SurfaceClosestPoint {
            u,
            v,
            point: d[0][0],
            distance: dist,
        };
        for _ in 0..NEWTON_ITERATIONS {
            if dist <= e1 {
                break;
            }
            let r = d[0][0] - target;
            let (s_u, s_v) = (d[1][0], d[0][1]);
            let (lu, lv) = (s_u.length(), s_v.length());
            let (f, g) = (s_u.dot(r), s_v.dot(r));
            let flat_u = lu <= floor_u || f.abs() <= ZERO_COSINE * lu * dist;
            let flat_v = lv <= floor_v || g.abs() <= ZERO_COSINE * lv * dist;
            if flat_u && flat_v {
                break;
            }
            let Some((du, dv)) = newton_step(&d, r, (u, v), &c) else {
                break;
            };
            let mut scale = 1.0;
            let mut next = None;
            for _ in 0..MAX_HALVINGS {
                let un = (u + scale * du).clamp(c.lo_u, c.hi_u);
                let vn = (v + scale * dv).clamp(c.lo_v, c.hi_v);
                if (s_u * (un - u) + s_v * (vn - v)).length() <= e1 {
                    break;
                }
                let dn = self.derivatives2_in_spans(un, vn, c.su, c.sv);
                let distn = (dn[0][0] - target).length();
                if distn <= dist + slack {
                    next = Some((un, vn, dn, distn));
                    break;
                }
                scale *= 0.5;
            }
            let Some((un, vn, dn, distn)) = next else {
                break;
            };
            (u, v, d, dist) = (un, vn, dn, distn);
            if dist < best.distance {
                best = SurfaceClosestPoint {
                    u,
                    v,
                    point: d[0][0],
                    distance: dist,
                };
            }
        }
        // The last iterate is the most accurate point unless the iteration
        // wandered off; distances within rounding of each other are equal.
        if dist <= best.distance + slack {
            SurfaceClosestPoint {
                u,
                v,
                point: d[0][0],
                distance: dist,
            }
        } else {
            best
        }
    }
}

/// Returns `true` if sample `(a, b)` of a `rows x cols` grid of distances
/// is no larger than any of its (up to eight) neighbours.
fn is_local_min(dist: &[f64], rows: usize, cols: usize, a: usize, b: usize) -> bool {
    let d = dist[a * cols + b];
    for na in a.saturating_sub(1)..=(a + 1).min(rows - 1) {
        for nb in b.saturating_sub(1)..=(b + 1).min(cols - 1) {
            if dist[na * cols + nb] < d {
                return false;
            }
        }
    }
    true
}

/// The Newton step `(du, dv)` at `(u, v)` for the residual `r` and the
/// derivative table `d` (eqs. 6.6, 6.7), with the Gauss-Newton fallback and
/// the edge handling of the module documentation. `None` when the surface
/// does not move at all.
fn newton_step(d: &[[DVec3; 3]; 3], r: DVec3, (u, v): (f64, f64), c: &Cell) -> Option<(f64, f64)> {
    let (s_u, s_v) = (d[1][0], d[0][1]);
    let (f, g) = (s_u.dot(r), s_v.dot(r));
    let (g00, g01, g11) = (s_u.dot(s_u), s_u.dot(s_v), s_v.dot(s_v));
    let trace = g00 + g11;
    if trace.is_nan() || trace <= 0.0 {
        return None;
    }
    let (j00, j01, j11) = (
        g00 + r.dot(d[2][0]),
        g01 + r.dot(d[1][1]),
        g11 + r.dot(d[0][2]),
    );
    let positive = |a: f64, b: f64, c: f64| a > 0.0 && c > 0.0 && a * c - b * b > 1e-14 * a * c;
    let (a00, a01, a11) = if positive(j00, j01, j11) {
        (j00, j01, j11)
    } else {
        let mu = REGULARISATION * trace;
        (g00 + mu, g01, g11 + mu)
    };
    let det = a00 * a11 - a01 * a01;
    if det.is_nan() || det <= 0.0 {
        return None;
    }
    let mut du = (-f * a11 + g * a01) / det;
    let mut dv = (-g * a00 + f * a01) / det;
    // On an edge, a step out of the cell becomes the step along the edge.
    let out_u = (u <= c.lo_u && du < 0.0) || (u >= c.hi_u && du > 0.0);
    let out_v = (v <= c.lo_v && dv < 0.0) || (v >= c.hi_v && dv > 0.0);
    match (out_u, out_v) {
        (true, true) => return None,
        (true, false) => {
            du = 0.0;
            dv = -g / a11;
            if (v <= c.lo_v && dv < 0.0) || (v >= c.hi_v && dv > 0.0) {
                return None;
            }
        }
        (false, true) => {
            dv = 0.0;
            du = -f / a00;
            if (u <= c.lo_u && du < 0.0) || (u >= c.hi_u && du > 0.0) {
                return None;
            }
        }
        (false, false) => {}
    }
    (du.is_finite() && dv.is_finite()).then_some((du, dv))
}

/// Distance from `x` to the box `[lo, hi]`.
fn box_distance(x: DVec3, lo: DVec3, hi: DVec3) -> f64 {
    (lo - x).max(x - hi).max(DVec3::ZERO).length()
}

/// The corner edges of a Bezier patch: the sums of its two edges along `u`
/// and its two edges along `v`.
fn corner_edges(control: &[DVec4], p: usize, q: usize) -> (DVec3, DVec3) {
    let at = |r: usize, s: usize| control[r * (q + 1) + s].project();
    let x = (at(p, 0) - at(0, 0)) + (at(p, q) - at(0, q));
    let y = (at(0, q) - at(0, 0)) + (at(p, q) - at(p, 0));
    (x, y)
}

/// Returns `true` if the patch is longer along `u` than along `v`, judged
/// by its corner edges.
fn longer_in_u(control: &[DVec4], p: usize, q: usize) -> bool {
    let (x, y) = corner_edges(control, p, q);
    x.length() >= y.length()
}

/// A lower bound on the distance from `target` to the Bezier patch with
/// homogeneous control points `control` (see the module documentation).
fn patch_lower_bound(control: &[DVec4], p: usize, q: usize, target: Point3) -> f64 {
    let points: Vec<Point3> = control.iter().map(|c| c.project()).collect();
    let (lo, hi) = points
        .iter()
        .fold((DVec3::INFINITY, DVec3::NEG_INFINITY), |(lo, hi), &c| {
            (lo.min(c), hi.max(c))
        });
    let aligned = box_distance(target, lo, hi);
    let (x, y) = corner_edges(control, p, q);
    let Some(&origin) = points.first() else {
        return aligned;
    };
    let Ok(frame) = Frame3::from_axes(origin, x, y) else {
        return aligned;
    };
    let (lo, hi) = points
        .iter()
        .fold((DVec3::INFINITY, DVec3::NEG_INFINITY), |(lo, hi), &c| {
            let l = frame.to_local(c);
            (lo.min(l), hi.max(l))
        });
    aligned.max(box_distance(frame.to_local(target), lo, hi))
}

/// Halves a Bezier curve at the middle of its parameter range with de
/// Casteljau's algorithm (The NURBS Book, section 1.4), in homogeneous
/// coordinates.
fn split_bezier_half(control: &[DVec4]) -> (Vec<DVec4>, Vec<DVec4>) {
    let mut work = control.to_vec();
    let n = work.len();
    let mut left = Vec::with_capacity(n);
    let mut right = Vec::with_capacity(n);
    for k in 0..n {
        if let (Some(&l), Some(&r)) = (work.first(), work.get(n - 1 - k)) {
            left.push(l);
            right.push(r);
        }
        for i in 0..n - 1 - k {
            work[i] = (work[i] + work[i + 1]) * 0.5;
        }
    }
    right.reverse();
    (left, right)
}

/// Halves a Bezier patch of degrees `(p, q)` along `u` (every column) or
/// along `v` (every row).
fn split_patch(control: &[DVec4], p: usize, q: usize, along_u: bool) -> (Vec<DVec4>, Vec<DVec4>) {
    let cols = q + 1;
    let mut first = vec![DVec4::ZERO; control.len()];
    let mut second = vec![DVec4::ZERO; control.len()];
    if along_u {
        for s in 0..cols {
            let column: Vec<DVec4> = (0..=p).map(|r| control[r * cols + s]).collect();
            let (a, b) = split_bezier_half(&column);
            for r in 0..=p {
                first[r * cols + s] = a[r];
                second[r * cols + s] = b[r];
            }
        }
    } else {
        for r in 0..=p {
            let (a, b) = split_bezier_half(&control[r * cols..(r + 1) * cols]);
            first[r * cols..(r + 1) * cols].copy_from_slice(&a);
            second[r * cols..(r + 1) * cols].copy_from_slice(&b);
        }
    }
    (first, second)
}

#[cfg(test)]
mod tests {
    use bevel_math::DVec3;

    use crate::{GeomError, KnotVector, NurbsSurface};

    /// The paraboloid `z = x^2 + y^2` over `[-1, 1]^2`.
    fn paraboloid() -> NurbsSurface {
        let kv = KnotVector::clamped_from_interior(2, -1.0, 1.0, &[]).unwrap();
        // The control net of x^2 on [-1, 1] is 1, -1, 1.
        let z = [1.0, -1.0, 1.0];
        let mut pts = Vec::new();
        for (i, zx) in z.iter().enumerate() {
            for (j, zy) in z.iter().enumerate() {
                pts.push(DVec3::new(i as f64 - 1.0, j as f64 - 1.0, zx + zy));
            }
        }
        NurbsSurface::from_points(2, 2, kv.clone(), kv, &pts, None).unwrap()
    }

    #[test]
    fn closest_point_on_a_paraboloid() {
        let s = paraboloid();
        // Points above the surface along its normal project back.
        for (x, y) in [(0.2, 0.3), (-0.5, 0.1), (0.0, 0.0), (0.7, -0.6)] {
            let p = s.point(x, y);
            let n = s.normal(x, y).unwrap();
            let target = p + n * 0.05;
            let cp = s.closest_point(target).unwrap();
            assert!((cp.u - x).abs() < 1e-9 && (cp.v - y).abs() < 1e-9, "{cp:?}");
            assert!((cp.distance - 0.05).abs() < 1e-12);
            assert!((s.point(cp.u, cp.v) - cp.point).length() < 1e-15);
        }
        // Beyond the boundary the closest point lies on an edge or corner.
        let cp = s.closest_point(DVec3::new(3.0, 0.2, 1.0)).unwrap();
        assert!((cp.u - 1.0).abs() < 1e-15);
        let cp = s.closest_point(DVec3::new(-3.0, -3.0, 2.0)).unwrap();
        assert!((cp.u + 1.0).abs() < 1e-15 && (cp.v + 1.0).abs() < 1e-15);
        assert!(matches!(
            s.closest_point(DVec3::new(0.0, f64::NAN, 0.0)),
            Err(GeomError::NonFinite)
        ));
    }

    #[test]
    fn closest_point_on_a_plane_patch() {
        let kv = KnotVector::clamped_from_interior(1, 0.0, 1.0, &[]).unwrap();
        let pts = [
            DVec3::ZERO,
            DVec3::new(0.0, 4.0, 0.0),
            DVec3::new(10.0, 0.0, 0.0),
            DVec3::new(10.0, 4.0, 0.0),
        ];
        let s = NurbsSurface::from_points(1, 1, kv.clone(), kv, &pts, None).unwrap();
        let cp = s.closest_point(DVec3::new(3.0, 1.0, 5.0)).unwrap();
        assert!((cp.u - 0.3).abs() < 1e-14 && (cp.v - 0.25).abs() < 1e-14);
        assert!((cp.distance - 5.0).abs() < 1e-14);
    }
}
