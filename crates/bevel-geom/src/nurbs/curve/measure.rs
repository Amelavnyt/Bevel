// SPDX-License-Identifier: GPL-3.0-or-later

//! Arc length and closest points.
//!
//! # Arc length
//!
//! The length `int |C'(u)| du` is integrated span by span (the speed is
//! smooth inside a knot span but may have kinks at knots) with adaptive
//! Gauss-Legendre quadrature: an interval is accepted when its 8- and
//! 16-point results agree to within its share of the tolerance and the
//! 16-point result also agrees with the sum of the 16-point results on its
//! two halves; otherwise it is bisected. The second test catches the two
//! rules agreeing by accident where the speed varies sharply (near a point
//! where the curve almost stops). The tolerance is shared among the spans
//! in proportion to their parameter length. The sum over the halves is
//! used, so the differences overestimate its error.
//!
//! # Closest point
//!
//! Point inversion (The NURBS Book, section 6.1) in two passes.
//!
//! 1. Seeding and Newton: the curve is sampled at `max(4 (p + 1), 8) + 1`
//!    evenly spaced parameters in every knot span, ends included (more than
//!    the `2 (p + 1)` the book suggests); the sampled local minima of the
//!    distance within each span become seeds, and the best few are refined
//!    inside their span with Newton's method on `f(u) = C'(u) . (C(u) - P)`
//!    (eq. 6.3), `u <- u - f / f'` with `f' = C'' . (C - P) + |C'|^2`.
//!    Iteration stops on the book's tests: point coincidence
//!    `|C - P| <= e1`, zero cosine `|C' . (C - P)| <= e2 |C'| |C - P|`, or
//!    a parameter step that moves the point by at most `e1`. Steps are
//!    clamped to the seed's span (so the iteration never crosses a knot,
//!    where the curve may have a corner) and halved until the distance stops
//!    increasing (up to rounding, since the distance is flat near a
//!    minimum), so a refinement never ends worse than its seed.
//! 2. Branch and bound: sampling can step over a narrow feature (a hairpin,
//!    say) that comes closer to `P`. The curve is split into Bezier segments
//!    (A5.6), and each segment gets a lower bound on its distance to `P`:
//!    the distance from `P` to the chord between its end points, minus the
//!    largest distance of a control point from that chord (the segment lies
//!    in the convex hull of its control points, and the distance to the
//!    chord is convex). Segments are taken in order of their bound and
//!    halved by de Casteljau subdivision until no segment can beat the best
//!    point found by more than a tiny margin; a segment that does not
//!    contain the best point gets its own Newton refinement. The pass stops
//!    after a fixed budget of segments, which only matters for points
//!    equidistant from a whole stretch of curve (the centre of an arc), where
//!    any of them is a correct answer.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::gauss_legendre::{GL8, GL16, integrate};
use crate::{GeomError, Homogeneous, NurbsCurve};

/// Relative round-off level below which quadrature differences are noise.
const QUADRATURE_ROUNDOFF: f64 = 1e-14;

/// Bisection depth limit for one knot span.
const MAX_BISECTIONS: usize = 60;

/// Limit on the number of intervals in one knot span.
const MAX_INTERVALS: usize = 1 << 16;

/// Number of seeds refined by Newton's method.
const NEWTON_SEEDS: usize = 16;

/// Newton iterations per seed.
const NEWTON_ITERATIONS: usize = 32;

/// Zero-cosine tolerance `e2` for Newton's method.
const ZERO_COSINE: f64 = 1e-13;

/// Most Bezier segments examined by the branch-and-bound pass.
const BRANCH_BUDGET: usize = 1024;

/// A Bezier piece of the curve on `[lo, hi]`, inside knot span `span`, with
/// a lower bound on its distance to the target. Ordered so that a
/// [`BinaryHeap`] pops the smallest bound first, ties going to the piece
/// created first.
struct Piece<H> {
    lower: f64,
    seq: usize,
    span: usize,
    lo: f64,
    hi: f64,
    control: Vec<H>,
}

impl<H> PartialEq for Piece<H> {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl<H> Eq for Piece<H> {}

impl<H> PartialOrd for Piece<H> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<H> Ord for Piece<H> {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .lower
            .total_cmp(&self.lower)
            .then_with(|| other.seq.cmp(&self.seq))
    }
}

/// A starting point for the closest-point iteration: a sampled local minimum
/// of the distance inside the knot span `[lo, hi]` with index `span`.
struct Seed {
    t: f64,
    distance: f64,
    span: usize,
    lo: f64,
    hi: f64,
}

/// The point of a curve closest to a given point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosestPoint<P> {
    /// Parameter of the closest point.
    pub t: f64,
    /// The closest point on the curve.
    pub point: P,
    /// Distance from the given point to the curve.
    pub distance: f64,
}

impl<H: Homogeneous> NurbsCurve<H> {
    /// The length of the curve between parameters `start` and `end` (in
    /// either order), accurate to about `tol` (in model units).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for non-finite parameters,
    /// [`GeomError::InvalidArgument`] unless `tol` is positive,
    /// [`GeomError::ParameterOutOfDomain`] for a parameter outside the domain
    /// and [`GeomError::NotConverged`] if the quadrature cannot meet the
    /// tolerance within its subdivision budget.
    pub fn arc_length(&self, start: f64, end: f64, tol: f64) -> Result<f64, GeomError> {
        if !start.is_finite() || !end.is_finite() {
            return Err(GeomError::NonFinite);
        }
        if tol.is_nan() || tol <= 0.0 {
            return Err(GeomError::InvalidArgument("the tolerance must be positive"));
        }
        let (a, b) = self.domain();
        let ktol = self.knots.tolerance();
        for t in [start, end] {
            if t < a - ktol || t > b + ktol {
                return Err(GeomError::ParameterOutOfDomain { t });
            }
        }
        let lo = start.min(end).clamp(a, b);
        let hi = start.max(end).clamp(a, b);
        let length = hi - lo;
        if length <= 0.0 {
            return Ok(0.0);
        }
        let mut cuts = vec![lo];
        cuts.extend(
            self.knots
                .breakpoints()
                .into_iter()
                .filter(|&u| u > lo && u < hi),
        );
        cuts.push(hi);
        let mut total = 0.0;
        for w in cuts.windows(2) {
            total += self.span_length(w[0], w[1], tol * (w[1] - w[0]) / length)?;
        }
        Ok(total)
    }

    /// Adaptive quadrature of the speed over `[a, b]`, inside one knot span.
    fn span_length(&self, a: f64, b: f64, tol: f64) -> Result<f64, GeomError> {
        let speed = |u: f64| {
            let mut d = [H::point_zero(); 2];
            self.derivatives_into(u, &mut d);
            H::point_length(d[1])
        };
        // Depth-first, left to right, so the summation order is fixed. Each
        // entry carries its 16-point value, computed by its parent.
        let whole = integrate(&GL16, a, b, speed);
        let mut stack = vec![(a, b, tol, 0usize, whole)];
        let mut sum = 0.0;
        let mut intervals = 1;
        while let Some((a, b, tol, depth, fine)) = stack.pop() {
            if !fine.is_finite() {
                return Err(GeomError::NotConverged);
            }
            let mid = 0.5 * (a + b);
            if !(mid > a && mid < b) {
                sum += fine;
                continue;
            }
            let left = integrate(&GL16, a, mid, speed);
            let right = integrate(&GL16, mid, b, speed);
            let tol = tol.max(QUADRATURE_ROUNDOFF * fine.abs());
            // The two rules can agree by accident on a sharply varying speed,
            // so the 16-point value must also match that of the two halves.
            let coarse = integrate(&GL8, a, b, speed);
            if (coarse - fine).abs() <= tol && (left + right - fine).abs() <= tol {
                sum += left + right;
                continue;
            }
            intervals += 1;
            if depth >= MAX_BISECTIONS || intervals > MAX_INTERVALS {
                return Err(GeomError::NotConverged);
            }
            stack.push((mid, b, 0.5 * tol, depth + 1, right));
            stack.push((a, mid, 0.5 * tol, depth + 1, left));
        }
        Ok(sum)
    }

    /// The point of the curve closest to `target` (see the module
    /// documentation for the method).
    ///
    /// The result is never farther than the best of the sampled points (up
    /// to rounding of the distances), and is the global minimum unless the
    /// curve has a narrow dip closer to `target` that falls between samples.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    pub fn closest_point(&self, target: H::Point) -> Result<ClosestPoint<H::Point>, GeomError> {
        if !H::point_dot(target, target).is_finite() {
            return Err(GeomError::NonFinite);
        }
        let p = self.degree;
        let (a, b) = self.domain();
        let per_span = (4 * (p + 1)).max(8);
        let breaks: Vec<f64> = self
            .knots
            .breakpoints()
            .into_iter()
            .filter(|&u| u >= a && u <= b)
            .collect();
        let mut best: Option<ClosestPoint<H::Point>> = None;
        let mut seeds: Vec<Seed> = Vec::new();
        let mut samples = Vec::with_capacity(per_span + 1);
        for w in breaks.windows(2) {
            let (lo, hi) = (w[0], w[1]);
            let span = self.knots.span(p, lo);
            samples.clear();
            for j in 0..=per_span {
                let t = if j == per_span {
                    hi
                } else {
                    lo + (hi - lo) * j as f64 / per_span as f64
                };
                let mut d = [H::point_zero(); 1];
                self.derivatives_in_span(t, span, &mut d);
                let distance = H::point_length(d[0] - target);
                if best.is_none_or(|b| distance < b.distance) {
                    best = Some(ClosestPoint {
                        t,
                        point: d[0],
                        distance,
                    });
                }
                samples.push((t, distance));
            }
            // Local minima of the sampled distance within this span.
            for (i, &(t, distance)) in samples.iter().enumerate() {
                let left = i.checked_sub(1).and_then(|j| samples.get(j));
                let right = samples.get(i + 1);
                if left.is_none_or(|l| distance <= l.1) && right.is_none_or(|r| distance <= r.1) {
                    seeds.push(Seed {
                        t,
                        distance,
                        span,
                        lo,
                        hi,
                    });
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

    /// The second pass of [`NurbsCurve::closest_point`]: looks for parts of
    /// the curve that could be closer than `best` and refines them.
    fn branch_and_bound(&self, target: H::Point, best: &mut ClosestPoint<H::Point>) {
        let margin = 1e-12 * (self.size() + H::point_length(target));
        let mut heap = BinaryHeap::new();
        let mut seq = 0;
        for segment in self.decompose_bezier() {
            let (lo, hi) = segment.domain();
            let lower = hull_lower_bound(&segment.control, target);
            if lower < best.distance - margin {
                heap.push(Piece {
                    lower,
                    seq,
                    span: self.knots.span(self.degree, lo),
                    lo,
                    hi,
                    control: segment.control,
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
            let mid = 0.5 * (piece.lo + piece.hi);
            if !(piece.lo <= best.t && best.t <= piece.hi) {
                let seed = Seed {
                    t: mid,
                    distance: 0.0,
                    span: piece.span,
                    lo: piece.lo,
                    hi: piece.hi,
                };
                let refined = self.newton_closest(&seed, target);
                if refined.distance < best.distance {
                    *best = refined;
                }
            }
            if !(mid > piece.lo && mid < piece.hi) {
                continue;
            }
            let (left, right) = split_bezier_half(&piece.control);
            for (control, lo, hi) in [(left, piece.lo, mid), (right, mid, piece.hi)] {
                let lower = hull_lower_bound(&control, target);
                if lower < best.distance - margin {
                    heap.push(Piece {
                        lower,
                        seq,
                        span: piece.span,
                        lo,
                        hi,
                        control,
                    });
                    seq += 1;
                }
            }
        }
    }

    /// Newton refinement of a closest-point seed inside its knot span (The
    /// NURBS Book, eq. 6.3). Staying in one span keeps the curve smooth, so
    /// a seed at a corner knot is refined on each side separately.
    fn newton_closest(&self, seed: &Seed, target: H::Point) -> ClosestPoint<H::Point> {
        let e1 = 1e-12 * self.size();
        // Near the minimum the distance is flat to second order, so steps
        // that leave it unchanged up to rounding are still progress.
        let slack = self.distance_slack(target);
        let floor = self.speed_floor();
        let mut u = seed.t;
        let mut d = [H::point_zero(); 3];
        self.derivatives_in_span(u, seed.span, &mut d);
        let mut dist = H::point_length(d[0] - target);
        let mut best = ClosestPoint {
            t: u,
            point: d[0],
            distance: dist,
        };
        for _ in 0..NEWTON_ITERATIONS {
            let diff = d[0] - target;
            if dist <= e1 {
                break;
            }
            let speed = H::point_length(d[1]);
            if speed <= floor {
                break;
            }
            let f = H::point_dot(d[1], diff);
            if f.abs() <= ZERO_COSINE * speed * dist {
                break;
            }
            let df = H::point_dot(d[2], diff) + speed * speed;
            // Away from a minimum f' can be negative; fall back to the
            // Gauss-Newton step, which always heads downhill.
            let df = if df > 0.0 { df } else { speed * speed };
            let mut step = -f / df;
            let mut next = None;
            for _ in 0..16 {
                let un = (u + step).clamp(seed.lo, seed.hi);
                if speed * (un - u).abs() <= e1 {
                    break;
                }
                let mut dn = [H::point_zero(); 3];
                self.derivatives_in_span(un, seed.span, &mut dn);
                let distn = H::point_length(dn[0] - target);
                if distn <= dist + slack {
                    next = Some((un, dn, distn));
                    break;
                }
                step *= 0.5;
            }
            let Some((un, dn, distn)) = next else {
                break;
            };
            u = un;
            d = dn;
            dist = distn;
            if dist < best.distance {
                best = ClosestPoint {
                    t: u,
                    point: d[0],
                    distance: dist,
                };
            }
        }
        // The last iterate is the most accurate point unless the iteration
        // wandered off; distances within rounding of each other are equal.
        if dist <= best.distance + slack {
            ClosestPoint {
                t: u,
                point: d[0],
                distance: dist,
            }
        } else {
            best
        }
    }

    /// Rounding level of distances to `target`.
    fn distance_slack(&self, target: H::Point) -> f64 {
        16.0 * f64::EPSILON * (self.size() + H::point_length(target))
    }
}

/// Distance from `x` to the segment from `a` to `b`.
fn segment_distance<H: Homogeneous>(x: H::Point, a: H::Point, b: H::Point) -> f64 {
    let v = b - a;
    let vv = H::point_dot(v, v);
    let t = if vv > 0.0 {
        (H::point_dot(x - a, v) / vv).clamp(0.0, 1.0)
    } else {
        0.0
    };
    H::point_length(x - (a + v * t))
}

/// A lower bound on the distance from `target` to the Bezier curve with
/// homogeneous control points `control` (see the module documentation).
fn hull_lower_bound<H: Homogeneous>(control: &[H], target: H::Point) -> f64 {
    let (Some(first), Some(last)) = (control.first(), control.last()) else {
        return 0.0;
    };
    let (a, b) = (first.project(), last.project());
    let spread = control
        .iter()
        .map(|c| segment_distance::<H>(c.project(), a, b))
        .fold(0.0, f64::max);
    (segment_distance::<H>(target, a, b) - spread).max(0.0)
}

/// Splits a Bezier curve at the middle of its parameter range with de
/// Casteljau's algorithm (The NURBS Book, section 1.4), in homogeneous
/// coordinates.
fn split_bezier_half<H: Homogeneous>(control: &[H]) -> (Vec<H>, Vec<H>) {
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

#[cfg(test)]
mod tests {
    use bevel_math::{DVec2, DVec3};

    use crate::{GeomError, KnotVector, NurbsCurve2, NurbsCurve3};

    fn quarter_circle(r: f64) -> NurbsCurve2 {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let knots = KnotVector::clamped_from_interior(2, 0.0, 1.0, &[]).unwrap();
        NurbsCurve2::from_points(
            2,
            knots,
            &[DVec2::new(r, 0.0), DVec2::new(r, r), DVec2::new(0.0, r)],
            Some(&[1.0, h, 1.0]),
        )
        .unwrap()
    }

    #[test]
    fn quarter_circle_has_the_right_length() {
        let r = 3.0;
        let c = quarter_circle(r);
        let l = c.arc_length(0.0, 1.0, 1e-10).unwrap();
        assert!((l - 0.5 * std::f64::consts::PI * r).abs() < 1e-10);
        // Either order, and zero for an empty range.
        assert!((c.arc_length(1.0, 0.0, 1e-10).unwrap() - l).abs() < 1e-12);
        assert!(c.arc_length(0.5, 0.5, 1e-10).unwrap().abs() < 1e-300);
    }

    #[test]
    fn arc_length_validates_its_arguments() {
        let c = quarter_circle(1.0);
        assert!(matches!(
            c.arc_length(0.0, 1.5, 1e-6),
            Err(GeomError::ParameterOutOfDomain { .. })
        ));
        assert!(matches!(
            c.arc_length(0.0, 1.0, 0.0),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            c.arc_length(f64::NAN, 1.0, 1e-6),
            Err(GeomError::NonFinite)
        ));
    }

    #[test]
    fn closest_point_on_a_circle_points_at_the_centre() {
        let r = 2.0;
        let c = quarter_circle(r);
        for angle in [0.1_f64, 0.5, 0.9, 1.3] {
            let target = DVec2::new(angle.cos(), angle.sin()) * 5.0;
            let cp = c.closest_point(target).unwrap();
            assert!((cp.distance - 3.0).abs() < 1e-10, "angle {angle}");
            assert!((cp.point - DVec2::new(angle.cos(), angle.sin()) * r).length() < 1e-10);
            assert!((c.point(cp.t) - cp.point).length() < 1e-15);
        }
        // Beyond the end of the arc the end point is closest.
        let cp = c.closest_point(DVec2::new(1.0, -4.0)).unwrap();
        assert!(cp.t.abs() < 1e-15);
        assert!(matches!(
            c.closest_point(DVec2::new(f64::NAN, 0.0)),
            Err(GeomError::NonFinite)
        ));
    }

    /// Found by the property tests: the target lies just before a corner
    /// knot where the curve moves fast. The best seed is the sample at the
    /// knot, and refining it with the derivatives of the span after the knot
    /// led away to a worse local minimum.
    #[test]
    #[allow(clippy::unreadable_literal)]
    fn closest_point_refines_corner_seeds_on_both_sides() {
        let knots = vec![
            6.782035267719381,
            6.782035267719381,
            6.782035267719381,
            11.22385833835583,
            11.502556594123948,
            11.502556594123948,
            12.210733276008352,
            12.210733276008352,
            16.163832758061208,
            16.163832758061208,
            19.512696769920105,
            22.870561871702897,
            22.870561871702897,
            22.870561871702897,
        ];
        let control = vec![
            DVec3::new(0.0, 0.0, 1.704591387546554),
            DVec3::new(0.0, 0.0, 1.4161272712944595),
            DVec3::new(0.0, 0.0, 0.8341323712138682),
            DVec3::new(0.0, 0.0, 0.4868642767081584),
            DVec3::new(0.0, 0.0, 4.556611307561479),
            DVec3::new(0.0, -7.780586588913297, 0.2817132615157585),
            DVec3::new(-29.354534983026685, 7.290810862845656, 1.2466337115687869),
            DVec3::new(-14.516086131308432, -18.8777164381733, 0.28772478030521603),
            DVec3::new(-77.13268715557471, 2.434871251796972, 1.0898321553692913),
            DVec3::new(-136.18175768555182, -39.07087842949069, 2.28231970775707),
            DVec3::new(-172.18038647149442, -77.43774160995498, 3.8283770444564698),
        ];
        let c = NurbsCurve2::new(2, KnotVector::new(knots, 2).unwrap(), control).unwrap();
        let u = 16.08778287891409;
        let target = c.point(u);
        let cp = c.closest_point(target).unwrap();
        assert!(cp.distance < 1e-9, "distance {}", cp.distance);
        assert!((cp.t - u).abs() < 1e-9);
    }

    /// Found by the property tests: a quintic with a hairpin narrower than
    /// the sample spacing. The sampled minimum lies on the wrong side of the
    /// hairpin; the branch-and-bound pass finds the target on the other.
    #[test]
    #[allow(clippy::unreadable_literal)]
    fn closest_point_finds_the_far_side_of_a_hairpin() {
        let knots = vec![
            0.37804233940997806,
            0.37804233940997806,
            0.37804233940997806,
            0.37804233940997806,
            0.37804233940997806,
            0.37804233940997806,
            1.176241764083451,
            1.9744411887569242,
            17.92773597049297,
            17.92773597049297,
            19.75107082416727,
            19.75107082416727,
            19.75107082416727,
            19.75107082416727,
            19.75107082416727,
            19.75107082416727,
        ];
        let mut pts = vec![DVec2::ZERO; 10];
        pts[4] = DVec2::new(42.6504818869318, 0.0);
        pts[5] = DVec2::new(0.0, -39.295516773481395);
        pts[6] = DVec2::new(-55.627833059919794, 91.17268301749529);
        pts[7] = DVec2::new(-18.0313845185325, 19.555029537099816);
        let c =
            NurbsCurve2::from_points(5, KnotVector::new(knots, 5).unwrap(), &pts, None).unwrap();
        let u = 16.908867977370008;
        let cp = c.closest_point(c.point(u)).unwrap();
        assert!(cp.distance < 1e-9, "distance {}", cp.distance);
        assert!((cp.t - u).abs() < 1e-9);
    }

    /// Found by the property tests: the speed of this rational cubic rises
    /// from zero very steeply, and on one interval the 8- and 16-point rules
    /// agreed by accident, giving a length 5e-6 off with a 1e-6 tolerance.
    #[test]
    #[allow(clippy::unreadable_literal)]
    fn arc_length_is_not_fooled_by_agreeing_rules() {
        let knots = vec![
            -6.293959051122339,
            -6.293959051122339,
            -6.293959051122339,
            -6.293959051122339,
            -2.0517322356127545,
            -2.0517322356127545,
            0.5388953241388652,
            2.001184229068688,
            5.82541579262614,
            5.82541579262614,
            5.82541579262614,
            5.82541579262614,
        ];
        let control = vec![
            DVec3::new(0.0, 0.0, 0.2),
            DVec3::new(0.0, 0.0, 0.2),
            DVec3::new(0.0, 0.0, 0.2),
            DVec3::new(0.0, 0.0, 0.2),
            DVec3::new(178.4002350414323, -54.934050084787664, 2.3462039665160206),
            DVec3::new(328.579982831697, 331.9160796972642, 3.8211886455551283),
            DVec3::new(-14.714504711363572, -39.51953486279982, 1.4326875997602107),
            DVec3::new(78.54451713391289, 75.93431199291948, 0.7867106568479726),
        ];
        let c = NurbsCurve2::new(3, KnotVector::new(knots, 3).unwrap(), control).unwrap();
        let (a, _) = c.domain();
        let u = 4.926499325340901;
        let reference = c.arc_length(a, u, 1e-12).unwrap();
        for tol in [1e-4, 1e-6, 1e-8] {
            let l = c.arc_length(a, u, tol).unwrap();
            assert!(
                (l - reference).abs() <= tol,
                "tol {tol}: {l} vs {reference}"
            );
        }
    }

    #[test]
    fn closest_point_on_a_line() {
        let c = NurbsCurve3::bezier(&[DVec3::ZERO, DVec3::new(10.0, 0.0, 0.0)]).unwrap();
        let cp = c.closest_point(DVec3::new(3.0, 4.0, 0.0)).unwrap();
        assert!((cp.t - 0.3).abs() < 1e-14);
        assert!((cp.distance - 4.0).abs() < 1e-14);
    }
}
