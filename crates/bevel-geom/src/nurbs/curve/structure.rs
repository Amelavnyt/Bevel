// SPDX-License-Identifier: GPL-3.0-or-later

//! Clamping, Bezier decomposition, splitting, reversal and
//! reparametrisation.
//!
//! - [`NurbsCurve::clamped`] inserts the domain ends until they have
//!   multiplicity `p` (A5.1) and drops the control points and knots whose
//!   basis functions vanish on the domain.
//! - [`NurbsCurve::decompose_bezier`] is A5.6: it walks the knot vector and
//!   inserts every interior knot until it has multiplicity `p`, peeling off
//!   one Bezier segment per knot span.
//! - [`NurbsCurve::split`] inserts the split parameter to multiplicity `p`
//!   (A5.1); the control point at that knot is then the curve point, shared
//!   by both halves (The NURBS Book, section 5.3).
//! - [`NurbsCurve::reversed`] reverses the control points and maps the knots
//!   by `u -> a + b - u` (section 6.1, eq. 6.13).
//! - [`NurbsCurve::reparametrized`] maps the knots affinely; an affine map
//!   of the knots leaves the basis functions, and so the curve, unchanged up
//!   to the parameter change (section 6.1).

use super::knot_ops::insert_raw;
use crate::{GeomError, Homogeneous, KnotVector, NurbsCurve};

impl<H: Homogeneous> NurbsCurve<H> {
    /// The same curve on a clamped knot vector: the first and last knots
    /// have multiplicity `p + 1`, so the curve starts at the first control
    /// point and ends at the last. The domain and geometry are unchanged.
    pub fn clamped(&self) -> Self {
        if self.knots.is_clamped(self.degree) {
            return self.clone();
        }
        self.clamp_start().clamp_end()
    }

    /// Clamps the end of the domain only.
    fn clamp_end(&self) -> Self {
        let p = self.degree;
        let (_, b) = self.domain();
        let k = self.knots.as_slice();
        let s = self.knots.multiplicity(b);
        let (uk, ctrl) = match self.knots.last_index_of(b) {
            // Below multiplicity p, insert `b` (in the span that starts at its
            // last copy, which is not the last knot since s <= p).
            Some(last) if s < p => insert_raw(p, k, &self.control, b, last, s, p - s),
            _ => (k.to_vec(), self.control.clone()),
        };
        let kv = KnotVector::from_raw(uk);
        let Some(first) = kv.as_slice().iter().position(|&u| kv.same(u, b)) else {
            return self.clone();
        };
        // With multiplicity at least p at indices first.., the curve at `b`
        // (from the left) is control point first - 1, and the basis functions
        // of the ones after it vanish on the domain. `first >= p + 1` because
        // the domain is non-degenerate.
        let mut knots = kv.as_slice()[..first + p].to_vec();
        knots.push(b);
        Self::from_parts(p, KnotVector::from_raw(knots), ctrl[..first].to_vec())
    }

    /// Clamps the start of the domain only.
    fn clamp_start(&self) -> Self {
        let p = self.degree;
        let (a, _) = self.domain();
        let k = self.knots.as_slice();
        let s = self.knots.multiplicity(a);
        // A valid vector has s <= p + 1 at the domain start.
        let (uk, ctrl) = if s < p {
            let span = self.knots.span(p, a);
            insert_raw(p, k, &self.control, a, span, s, p - s)
        } else {
            (k.to_vec(), self.control.clone())
        };
        let kv = KnotVector::from_raw(uk);
        let Some(first) = kv.as_slice().iter().position(|&u| kv.same(u, a)) else {
            return self.clone();
        };
        // With multiplicity m = max(s, p) at indices first..first+m, the curve
        // at `a` is control point first + m - p - 1, and the basis functions
        // of the ones before it vanish on the domain.
        let m = s.max(p);
        let Some(drop) = (first + m).checked_sub(p + 1) else {
            return self.clone();
        };
        let mut knots = kv.as_slice()[drop..].to_vec();
        knots[0] = a;
        Self::from_parts(p, KnotVector::from_raw(knots), ctrl[drop..].to_vec())
    }

    /// The Bezier segments of the curve, one per non-empty knot span, in
    /// order (The NURBS Book, A5.6). Segment `i` has the domain of the
    /// `i`-th knot span and equals the curve there.
    pub fn decompose_bezier(&self) -> Vec<Self> {
        let c = self.clamped();
        let p = c.degree;
        let u = c.knots.as_slice();
        let pw = &c.control;
        let n = pw.len() - 1;
        let m = n + p + 1;
        let mut segments = Vec::new();
        let mut a = p;
        let mut b = p + 1;
        let mut qw: Vec<H> = pw[..=p].to_vec();
        let mut next = vec![H::ZERO; p + 1];
        let mut alphas = vec![0.0; p];
        while b < m {
            let i = b;
            while b < m && c.knots.same(u[b + 1], u[b]) {
                b += 1;
            }
            let mult = b - i + 1;
            if mult < p {
                // Insert u[b] until it has multiplicity p.
                let numer = u[b] - u[a];
                for j in (mult + 1..=p).rev() {
                    alphas[j - mult - 1] = numer / (u[a + j] - u[a]);
                }
                let r = p - mult;
                for j in 1..=r {
                    let save = r - j;
                    let s = mult + j;
                    for k in (s..=p).rev() {
                        let alpha = alphas[k - s];
                        qw[k] = qw[k] * alpha + qw[k - 1] * (1.0 - alpha);
                    }
                    next[save] = qw[p];
                }
            }
            let knots = [vec![u[a]; p + 1], vec![u[b]; p + 1]].concat();
            segments.push(Self::from_parts(p, KnotVector::from_raw(knots), qw.clone()));
            if b < m {
                // The next segment starts with the points saved above.
                let first_kept = p.saturating_sub(mult);
                next[first_kept..].copy_from_slice(&pw[b + first_kept - p..=b]);
                qw.copy_from_slice(&next);
                a = b;
                b += 1;
            }
        }
        segments
    }

    /// Splits the curve at `u` into the part on `[a, u]` and the part on
    /// `[u, b]`, where `[a, b]` is the domain. Both parts are clamped and
    /// keep the original parametrisation.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `u` and
    /// [`GeomError::ParameterOutOfDomain`] unless `u` lies strictly inside
    /// the domain.
    pub fn split(&self, u: f64) -> Result<(Self, Self), GeomError> {
        let u = self.interior_knot_param(u)?;
        let c = self.clamped();
        let p = c.degree;
        let s = c.knots.multiplicity(u);
        let (uk, ctrl) = if s < p {
            let span = c.knots.span(p, u);
            insert_raw(p, c.knots.as_slice(), &c.control, u, span, s, p - s)
        } else {
            (c.knots.as_slice().to_vec(), c.control.clone())
        };
        // `u` is interior, so it has multiplicity exactly p now and its
        // first index is at least p + 1.
        let Some(first) = uk.iter().position(|&k| c.knots.same(k, u)) else {
            return Err(GeomError::ParameterOutOfDomain { t: u });
        };
        let mut left_knots = uk[..first + p].to_vec();
        left_knots.push(u);
        let mut right_knots = vec![u];
        right_knots.extend_from_slice(&uk[first..]);
        let left = Self::from_parts(p, KnotVector::from_raw(left_knots), ctrl[..first].to_vec());
        let right = Self::from_parts(
            p,
            KnotVector::from_raw(right_knots),
            ctrl[first - 1..].to_vec(),
        );
        Ok((left, right))
    }

    /// The same curve traversed backwards: the reversed curve at
    /// `a + b - u` is the original at `u`, where `[a, b]` is the domain.
    pub fn reversed(&self) -> Self {
        let control = self.control.iter().rev().copied().collect();
        Self::from_parts(self.degree, self.knots.reversed(self.degree), control)
    }

    /// The same curve on the domain `[start, end]`: the new curve at
    /// `start + (u - a) (end - start) / (b - a)` is the original at `u`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for non-finite bounds,
    /// [`GeomError::InvalidArgument`] unless `start < end`, and
    /// [`GeomError::InvalidKnotVector`] if the new domain is too short to
    /// keep the knots apart.
    pub fn reparametrized(&self, start: f64, end: f64) -> Result<Self, GeomError> {
        if !start.is_finite() || !end.is_finite() {
            return Err(GeomError::NonFinite);
        }
        if end <= start {
            return Err(GeomError::InvalidArgument(
                "the new domain must have positive length",
            ));
        }
        let (a, b) = self.domain();
        let scale = (end - start) / (b - a);
        let knots: Vec<f64> = self
            .knots
            .as_slice()
            .iter()
            .map(|&u| {
                if self.knots.same(u, a) {
                    start
                } else if self.knots.same(u, b) {
                    end
                } else {
                    start + (u - a) * scale
                }
            })
            .collect();
        let knots = KnotVector::new(knots, self.degree)?;
        Ok(Self::from_parts(self.degree, knots, self.control.clone()))
    }
}

#[cfg(test)]
mod tests {
    use bevel_math::DVec2;

    use crate::{GeomError, KnotVector, NurbsCurve2};

    fn curve() -> NurbsCurve2 {
        let knots = KnotVector::clamped_from_interior(2, 0.0, 3.0, &[1.0, 2.0, 2.0]).unwrap();
        let pts = [
            DVec2::new(0.0, 0.0),
            DVec2::new(1.0, 2.0),
            DVec2::new(2.0, -1.0),
            DVec2::new(3.0, 1.0),
            DVec2::new(4.0, 0.0),
            DVec2::new(5.0, 2.0),
        ];
        NurbsCurve2::from_points(2, knots, &pts, Some(&[1.0, 2.0, 0.5, 1.0, 1.5, 1.0])).unwrap()
    }

    #[test]
    fn split_needs_an_interior_parameter() {
        let c = curve();
        assert!(matches!(
            c.split(0.0),
            Err(GeomError::ParameterOutOfDomain { .. })
        ));
        assert!(matches!(
            c.split(3.0),
            Err(GeomError::ParameterOutOfDomain { .. })
        ));
        assert!(matches!(c.split(f64::INFINITY), Err(GeomError::NonFinite)));
        // Splitting at a knot of full multiplicity needs no insertion.
        let (l, r) = c.split(2.0).unwrap();
        assert_eq!(l.domain(), (0.0, 2.0));
        assert_eq!(r.domain(), (2.0, 3.0));
        assert_eq!(l.control_weighted().len() + r.control_weighted().len(), 7);
    }

    #[test]
    fn decomposition_yields_one_segment_per_span() {
        let segments = curve().decompose_bezier();
        let domains: Vec<_> = segments.iter().map(NurbsCurve2::domain).collect();
        assert_eq!(domains, vec![(0.0, 1.0), (1.0, 2.0), (2.0, 3.0)]);
        assert!(segments.iter().all(|s| s.knots().is_clamped(2)));
    }

    #[test]
    fn reparametrization_validates_the_new_domain() {
        let c = curve();
        assert!(matches!(
            c.reparametrized(1.0, 1.0),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            c.reparametrized(0.0, f64::NAN),
            Err(GeomError::NonFinite)
        ));
        let r = c.reparametrized(-1.0, 5.0).unwrap();
        assert_eq!(r.domain(), (-1.0, 5.0));
        assert_eq!(r.knots().multiplicity(3.0), 2);
        let back = c.reversed().reversed();
        assert_eq!(back.knots(), c.knots());
    }
}
