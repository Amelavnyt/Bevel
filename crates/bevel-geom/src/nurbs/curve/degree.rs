// SPDX-License-Identifier: GPL-3.0-or-later

//! Degree elevation and reduction.
//!
//! # Elevation
//!
//! [`NurbsCurve::elevate_degree`] is A5.9 of The NURBS Book: it walks the
//! knot vector, turns each span into a Bezier segment by knot insertion,
//! elevates the segment with the closed-form coefficients of eq. 5.36 and
//! removes the inserted knots again as it goes, so that every knot ends up
//! with its multiplicity raised by `t` and the continuity is unchanged. The
//! curve is unchanged.
//!
//! # Reduction
//!
//! [`NurbsCurve::reduce_degree`] follows the structure of A5.11 in three
//! explicit steps instead of its fused loop:
//!
//! 1. split the curve into Bezier segments (A5.6);
//! 2. reduce each segment by one degree with the two-sided recursion of
//!    eqs. 5.41-5.43 (averaging the middle point for odd degrees);
//! 3. join the segments and remove each interior knot (A5.8) until it has
//!    its original multiplicity minus one, which restores the original
//!    continuity in the lower degree.
//!
//! Each step has a rigorous bound on how far it moves the homogeneous curve,
//! kept separately for the point part and the weight: for a segment, the
//! largest difference between its control points and those of the reduced
//! segment elevated back (the difference of two Bezier curves of the same
//! degree lies in the convex hull of the control-point differences); for a
//! knot removal, the A5.8 deviation. Removals only move the curve within `p`
//! spans of the knot, so the bounds are accumulated per segment as A5.11
//! does. Point and weight bounds `da`, `dw` become the model-space bound
//! `(da + max |P_i| dw) / min w~_i`, with `P_i` the original control points
//! and `w~_i` the reduced weights (see the knot removal module for why).

use super::knot_ops::{ModelBound, remove_raw};
use crate::{GeomError, Homogeneous, KnotVector, MAX_DEGREE, NurbsCurve};

impl<H: Homogeneous> NurbsCurve<H> {
    /// The same curve with its degree raised by `t` (The NURBS Book, A5.9).
    /// Every knot's multiplicity grows by `t`; the result is clamped.
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidDegree`] if the new degree would exceed
    /// [`MAX_DEGREE`].
    pub fn elevate_degree(&self, t: usize) -> Result<Self, GeomError> {
        if t == 0 {
            return Ok(self.clone());
        }
        let ph = self.degree.saturating_add(t);
        if ph > MAX_DEGREE {
            return Err(GeomError::InvalidDegree { degree: ph });
        }
        let c = self.clamped();
        let (knots, control) = elevate_raw(c.degree, &c.knots, &c.control, t);
        Ok(Self::from_parts(ph, KnotVector::from_raw(knots), control))
    }

    /// The curve approximated with degree one lower, moving it by at most
    /// `tol` (in model units) anywhere. Each interior knot keeps the same
    /// continuity, so its multiplicity drops by one. See the module
    /// documentation for the method and its error bound.
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidArgument`] for a negative or NaN `tol`,
    /// [`GeomError::InvalidDegree`] for a curve of degree 1, and
    /// [`GeomError::ToleranceNotMet`] with the error bound when the reduced
    /// curve could deviate by more than `tol` (the bound is infinite if the
    /// reduction would make a weight non-positive).
    pub fn reduce_degree(&self, tol: f64) -> Result<Self, GeomError> {
        if tol.is_nan() || tol < 0.0 {
            return Err(GeomError::InvalidArgument(
                "the tolerance must be a non-negative number",
            ));
        }
        let p = self.degree;
        if p < 2 {
            return Err(GeomError::InvalidDegree { degree: p });
        }
        let ph = p - 1;
        let c = self.clamped();
        let original_multiplicities = c.knots.distinct();

        // Steps 1 and 2: reduce every Bezier segment.
        let segments = c.decompose_bezier();
        let mut errors = Vec::with_capacity(segments.len());
        let mut control: Vec<H> = Vec::with_capacity(segments.len() * p + 1);
        let mut knots = Vec::with_capacity(segments.len() * ph + 2);
        for (i, seg) in segments.iter().enumerate() {
            let (q, err) = reduce_bezier(&seg.control);
            errors.push(err);
            let (a, b) = seg.domain();
            if i == 0 {
                control.extend_from_slice(&q);
                knots.extend(std::iter::repeat_n(a, ph + 1));
            } else {
                // Segments share their end points exactly.
                control.extend_from_slice(&q[1..]);
                knots.extend(std::iter::repeat_n(a, ph));
            }
            if i + 1 == segments.len() {
                knots.extend(std::iter::repeat_n(b, ph + 1));
            }
        }
        let mut kv = KnotVector::from_raw(knots);

        // Step 3: restore the continuity at each interior knot.
        let interior = original_multiplicities
            .get(1..original_multiplicities.len().saturating_sub(1))
            .unwrap_or(&[]);
        for (bi, &(u, mult)) in interior.iter().enumerate() {
            // `bi + 1` is the index of the segment that starts at `u`.
            let num = p - mult;
            if num == 0 {
                continue;
            }
            let Some(r) = kv.last_index_of(u) else {
                continue;
            };
            let mut bound = (0.0, 0.0);
            let mut accept = |d: H| {
                bound.0 += H::point_length(d.weighted());
                bound.1 += d.weight().abs();
                true
            };
            let (k, ctrl, removed) =
                remove_raw(ph, kv.as_slice(), &control, u, r, ph, num, &mut accept);
            if removed < num {
                return Err(GeomError::ToleranceNotMet {
                    error: f64::INFINITY,
                });
            }
            kv = KnotVector::from_raw(k);
            control = ctrl;
            let seg = bi + 1;
            for e in &mut errors[seg.saturating_sub(p)..(seg + p).min(segments.len())] {
                e.0 += bound.0;
                e.1 += bound.1;
            }
        }

        // Convert the worst homogeneous bound to model space.
        if !control.iter().all(|c| c.is_finite() && c.weight() > 0.0) {
            return Err(GeomError::ToleranceNotMet {
                error: f64::INFINITY,
            });
        }
        let scale = ModelBound::with_weights(&c.control, &control);
        let error = errors
            .iter()
            .map(|&(da, dw)| scale.bound(da, dw))
            .fold(0.0, f64::max);
        if error > tol {
            return Err(GeomError::ToleranceNotMet { error });
        }
        Ok(Self::from_parts(ph, kv, control))
    }
}

/// `binom(n, k)` as a float; exact for the small arguments used here.
fn binomial(n: usize, k: usize) -> f64 {
    let k = k.min(n - k.min(n));
    (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64)
}

/// A5.9 on a clamped curve of degree `p`, elevating by `t >= 1`.
///
/// The output sizes follow from the knot structure: each of the `s` distinct
/// interior knots and both ends gain `t` copies, and the control polygon
/// gains `t (s + 1)` points. All indices below stay inside these sizes for a
/// valid clamped curve.
// The loops keep the book's indices so they can be checked against it.
#[allow(
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::explicit_counter_loop
)]
fn elevate_raw<H: Homogeneous>(
    p: usize,
    knots: &KnotVector,
    pw: &[H],
    t: usize,
) -> (Vec<f64>, Vec<H>) {
    let u = knots.as_slice();
    let n = pw.len() - 1;
    let m = n + p + 1;
    let ph = p + t;
    let ph2 = ph / 2;
    let interior = knots.distinct().len().saturating_sub(2);

    // Bezier degree elevation coefficients (eq. 5.36).
    let mut bezalfs = vec![vec![0.0; p + 1]; ph + 1];
    bezalfs[0][0] = 1.0;
    bezalfs[ph][p] = 1.0;
    for i in 1..=ph2 {
        let inv = 1.0 / binomial(ph, i);
        for j in i.saturating_sub(t)..=p.min(i) {
            bezalfs[i][j] = inv * binomial(p, j) * binomial(t, i - j);
        }
    }
    for i in ph2 + 1..ph {
        for j in i.saturating_sub(t)..=p.min(i) {
            bezalfs[i][j] = bezalfs[ph - i][p - j];
        }
    }

    let mut uh = vec![0.0; m + 1 + t * (interior + 2)];
    let mut qw = vec![H::ZERO; n + 1 + t * (interior + 1)];
    let mut bpts: Vec<H> = pw[..=p].to_vec();
    let mut ebpts = vec![H::ZERO; ph + 1];
    let mut next_bpts = vec![H::ZERO; p];
    let mut alfs = vec![0.0; p];

    let mut mh = ph;
    let mut kind = ph + 1;
    // `r` is p - mul for the current knot; `None` stands for the book's -1.
    let mut r: Option<usize> = None;
    let mut a = p;
    let mut b = p + 1;
    let mut cind = 1;
    let mut ua = u[0];
    qw[0] = pw[0];
    uh[..=ph].fill(ua);

    while b < m {
        let i = b;
        while b < m && knots.same(u[b], u[b + 1]) {
            b += 1;
        }
        let mul = b - i + 1;
        mh += mul + t;
        let ub = u[b];
        let oldr = r;
        r = p.checked_sub(mul);
        // Number of points to drop on the left and the right of this segment.
        let lbz = match oldr {
            Some(o) if o > 0 => o / 2 + 1,
            _ => 1,
        };
        let rbz = match r {
            Some(r) if r > 0 => ph - r.div_ceil(2),
            _ => ph,
        };
        if let Some(r) = r
            && r > 0
        {
            // Insert ub r times to get the Bezier segment.
            let numer = ub - ua;
            for k in (mul + 1..=p).rev() {
                alfs[k - mul - 1] = numer / (u[a + k] - ua);
            }
            for j in 1..=r {
                let save = r - j;
                let s = mul + j;
                for k in (s..=p).rev() {
                    let alpha = alfs[k - s];
                    bpts[k] = bpts[k] * alpha + bpts[k - 1] * (1.0 - alpha);
                }
                next_bpts[save] = bpts[p];
            }
        }
        // Elevate the Bezier segment.
        for (i, e) in ebpts.iter_mut().enumerate().skip(lbz) {
            *e = (i.saturating_sub(t)..=p.min(i))
                .fold(H::ZERO, |acc, j| acc + bpts[j] * bezalfs[i][j]);
        }
        if let Some(oldr) = oldr
            && oldr > 1
        {
            // Remove the knot ua oldr times.
            let mut first = kind - 2;
            let mut last = kind;
            let den = ub - ua;
            let bet = (ub - uh[kind - 1]) / den;
            for tr in 1..oldr {
                let mut i = first;
                let mut j = last;
                let mut kj = j + 1 - kind;
                while j > i + tr {
                    if i < cind {
                        let alf = (ub - uh[i]) / (ua - uh[i]);
                        qw[i] = qw[i] * alf + qw[i - 1] * (1.0 - alf);
                    }
                    if j >= lbz {
                        if j - tr + ph <= kind + oldr {
                            let gam = (ub - uh[j - tr]) / den;
                            ebpts[kj] = ebpts[kj] * gam + ebpts[kj + 1] * (1.0 - gam);
                        } else {
                            ebpts[kj] = ebpts[kj] * bet + ebpts[kj + 1] * (1.0 - bet);
                        }
                    }
                    i += 1;
                    j -= 1;
                    kj -= 1;
                }
                first -= 1;
                last += 1;
            }
        }
        if a != p {
            // Load the knot ua (oldr is set here, since a segment came before).
            let copies = ph - oldr.unwrap_or(0);
            uh[kind..kind + copies].fill(ua);
            kind += copies;
        }
        for e in &ebpts[lbz..=rbz] {
            qw[cind] = *e;
            cind += 1;
        }
        if b < m {
            // Set up the next segment.
            let r = r.unwrap_or(0);
            bpts[..r].copy_from_slice(&next_bpts[..r]);
            bpts[r..].copy_from_slice(&pw[b + r - p..=b]);
            a = b;
            b += 1;
            ua = ub;
        } else {
            uh[kind..=kind + ph].fill(ub);
        }
    }
    let nh = mh - ph - 1;
    uh.truncate(nh + ph + 2);
    qw.truncate(nh + 1);
    (uh, qw)
}

/// Reduces a Bezier segment of degree `p >= 2` (`p + 1` homogeneous points)
/// to degree `p - 1` (The NURBS Book, eqs. 5.41-5.43). Returns the `p`
/// reduced points and bounds on the homogeneous deviation, for the point
/// part and the weight: the largest differences between the original points
/// and the reduced points elevated back to degree `p`.
fn reduce_bezier<H: Homogeneous>(pts: &[H]) -> (Vec<H>, (f64, f64)) {
    let p = pts.len() - 1;
    let pf = p as f64;
    let r = (p - 1) / 2;
    let mut q = vec![H::ZERO; p];
    // From the left: P_i = a_i Q_{i-1} + (1 - a_i) Q_i, a_i = i / p.
    q[0] = pts[0];
    for i in 1..=r {
        let alpha = i as f64 / pf;
        q[i] = (pts[i] - q[i - 1] * alpha) * (1.0 / (1.0 - alpha));
    }
    // From the right.
    q[p - 1] = pts[p];
    for i in (r + 1..p - 1).rev() {
        let alpha = (i + 1) as f64 / pf;
        q[i] = (pts[i + 1] - q[i + 1] * (1.0 - alpha)) * (1.0 / alpha);
    }
    if p % 2 == 1 {
        // Both sides reach Q_r; average them.
        let alpha = (r + 1) as f64 / pf;
        let right = (pts[r + 1] - q[r + 1] * (1.0 - alpha)) * (1.0 / alpha);
        q[r] = (q[r] + right) * 0.5;
    }
    // Elevate back and compare.
    let mut err = (0.0_f64, 0.0_f64);
    for (i, &pi) in pts.iter().enumerate() {
        let alpha = i as f64 / pf;
        let e = match i {
            0 => q[0],
            _ if i == p => q[p - 1],
            _ => q[i - 1] * alpha + q[i] * (1.0 - alpha),
        };
        let d = pi - e;
        err.0 = err.0.max(H::point_length(d.weighted()));
        err.1 = err.1.max(d.weight().abs());
    }
    (q, err)
}

#[cfg(test)]
mod tests {
    use bevel_math::DVec3;

    use crate::{GeomError, KnotVector, NurbsCurve3};

    fn cubic() -> NurbsCurve3 {
        let knots = KnotVector::clamped_from_interior(3, 0.0, 1.0, &[0.4, 0.4, 0.7]).unwrap();
        let pts = [
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(1.0, 3.0, 0.0),
            DVec3::new(2.0, -1.0, 1.0),
            DVec3::new(3.0, 2.0, 0.0),
            DVec3::new(4.0, 0.0, -1.0),
            DVec3::new(5.0, 1.0, 0.0),
            DVec3::new(6.0, 0.0, 2.0),
        ];
        NurbsCurve3::from_points(3, knots, &pts, Some(&[1.0, 0.8, 1.5, 1.0, 2.0, 0.7, 1.0]))
            .unwrap()
    }

    fn max_deviation(a: &NurbsCurve3, b: &NurbsCurve3) -> f64 {
        (0..=200)
            .map(|i| {
                let u = f64::from(i) / 200.0;
                (a.point(u) - b.point(u)).length()
            })
            .fold(0.0, f64::max)
    }

    #[test]
    fn reducing_an_elevated_curve_recovers_it() {
        let c = cubic();
        let e = c.elevate_degree(1).unwrap();
        assert_eq!(e.degree(), 4);
        assert_eq!(e.knots().multiplicity(0.4), 3);
        let r = e.reduce_degree(1e-9).unwrap();
        assert_eq!(r.degree(), 3);
        assert_eq!(r.knots(), c.knots());
        assert!(max_deviation(&c, &r) < 1e-10);
        // Twice up, twice down.
        let e2 = c.elevate_degree(2).unwrap();
        let r2 = e2.reduce_degree(1e-9).unwrap().reduce_degree(1e-9).unwrap();
        assert_eq!(r2.degree(), 3);
        assert!(max_deviation(&c, &r2) < 1e-10);
    }

    #[test]
    fn a_genuine_cubic_cannot_be_reduced_within_a_tiny_tolerance() {
        let c = cubic();
        let Err(GeomError::ToleranceNotMet { error }) = c.reduce_degree(1e-6) else {
            panic!("expected ToleranceNotMet");
        };
        assert!(error > 1e-3 && error.is_finite());
        // With the reported bound as the tolerance, the reduction succeeds
        // and stays within it.
        let r = c.reduce_degree(error).unwrap();
        assert_eq!(r.degree(), 2);
        assert!(max_deviation(&c, &r) <= error);
    }

    #[test]
    fn degree_limits_are_enforced() {
        let line = NurbsCurve3::bezier(&[DVec3::ZERO, DVec3::X]).unwrap();
        assert!(matches!(
            line.reduce_degree(1.0),
            Err(GeomError::InvalidDegree { degree: 1 })
        ));
        assert!(matches!(
            line.elevate_degree(usize::MAX),
            Err(GeomError::InvalidDegree { .. })
        ));
        assert!(matches!(
            cubic().reduce_degree(f64::NAN),
            Err(GeomError::InvalidArgument(_))
        ));
        assert_eq!(line.elevate_degree(0).unwrap(), line);
        let quadric = line.elevate_degree(1).unwrap();
        assert_eq!(quadric.degree(), 2);
        assert!((quadric.point(0.25) - DVec3::new(0.25, 0.0, 0.0)).length() < 1e-15);
    }
}
