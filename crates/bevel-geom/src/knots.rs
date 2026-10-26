// SPDX-License-Identifier: GPL-3.0-or-later

//! Knot vectors.
//!
//! A knot vector of degree `p` for `n + 1` control points is a
//! non-decreasing sequence `U = {u_0, ..., u_m}` with `m = n + p + 1`
//! (The NURBS Book, section 2.2). The curve is defined on the domain
//! `[u_p, u_{n+1}]`. A knot vector is *clamped* when its first and last
//! values are repeated `p + 1` times, so that the curve starts and ends at
//! its first and last control points; clamped vectors are the norm in Bevel,
//! but unclamped ones are accepted.
//!
//! # Validation
//!
//! [`KnotVector::new`] accepts a vector for degree `p` when
//!
//! - every knot is finite and the sequence is non-decreasing,
//! - it has at least `2 (p + 1)` knots (so at least `p + 1` control points),
//! - the domain `[u_p, u_{n+1}]` has positive length,
//! - values strictly inside the domain occur at most `p` times (so the curve
//!   stays continuous) and every value occurs at most `p + 1` times.
//!
//! # Knot equality
//!
//! Multiplicity only makes sense if equal knots are recognisably equal.
//! Knots closer than [`KnotVector::tolerance`] (1e-12 of the knot range, or
//! a few ulps for knots far from zero) are snapped to the first value of their
//! run on construction, so equal knots are bit-identical afterwards and
//! distinct knots are always more than the tolerance apart. Parameters passed
//! to knot insertion, removal and splitting are snapped to an existing knot in
//! the same way.

use crate::{GeomError, MAX_DEGREE};

/// Relative tolerance under which two knots are the same knot.
const KNOT_RELATIVE_TOLERANCE: f64 = 1e-12;

/// A validated, non-decreasing sequence of knots.
///
/// The vector does not store its degree; methods that depend on it take the
/// degree `p` as an argument; whatever owns the vector validates it against
/// its own degree.
#[derive(Clone, Debug, PartialEq)]
pub struct KnotVector {
    /// Invariant: finite, non-decreasing, at least four values, snapped (equal
    /// knots are bit-identical and distinct knots differ by more than
    /// [`KnotVector::tolerance`]), and valid for the degree of the curve or
    /// surface that owns it.
    knots: Vec<f64>,
}

/// Returns an error unless `degree` is in `1..=MAX_DEGREE`.
pub(crate) fn check_degree(degree: usize) -> Result<(), GeomError> {
    if (1..=MAX_DEGREE).contains(&degree) {
        Ok(())
    } else {
        Err(GeomError::InvalidDegree { degree })
    }
}

impl KnotVector {
    /// Validates `knots` for degree `degree` and snaps nearly equal knots
    /// together (see the module documentation).
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidDegree`] for a degree outside `1..=MAX_DEGREE`,
    /// [`GeomError::NonFinite`] for NaN or infinite knots, and
    /// [`GeomError::InvalidKnotVector`] for the other rules in the module
    /// documentation.
    pub fn new(knots: Vec<f64>, degree: usize) -> Result<Self, GeomError> {
        check_degree(degree)?;
        if knots.iter().any(|k| !k.is_finite()) {
            return Err(GeomError::NonFinite);
        }
        if knots.len() < 2 * (degree + 1) {
            return Err(GeomError::InvalidKnotVector(
                "a knot vector of degree p needs at least 2(p + 1) knots",
            ));
        }
        if knots.windows(2).any(|w| w[1] < w[0]) {
            return Err(GeomError::InvalidKnotVector("knots must be non-decreasing"));
        }
        let mut kv = Self { knots };
        kv.snap();
        kv.validate(degree)?;
        Ok(kv)
    }

    /// A clamped knot vector on `[0, 1]` with uniformly spaced interior knots
    /// for `control_count` control points.
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidDegree`] for an unsupported degree and
    /// [`GeomError::ControlPointCount`] if `control_count < degree + 1`.
    pub fn clamped_uniform(degree: usize, control_count: usize) -> Result<Self, GeomError> {
        check_degree(degree)?;
        if control_count < degree + 1 {
            return Err(GeomError::ControlPointCount {
                expected: degree + 1,
                found: control_count,
            });
        }
        let segments = control_count - degree;
        let interior: Vec<f64> = (1..segments).map(|i| i as f64 / segments as f64).collect();
        Self::clamped_from_interior(degree, 0.0, 1.0, &interior)
    }

    /// A clamped knot vector on `[domain_start, domain_end]` with the given
    /// interior knots, which must be non-decreasing and strictly inside the
    /// domain. The number of control points it fits is
    /// `interior.len() + degree + 1`.
    ///
    /// # Errors
    ///
    /// As [`KnotVector::new`], plus [`GeomError::InvalidKnotVector`] if an
    /// interior knot is not strictly inside the domain.
    pub fn clamped_from_interior(
        degree: usize,
        domain_start: f64,
        domain_end: f64,
        interior: &[f64],
    ) -> Result<Self, GeomError> {
        check_degree(degree)?;
        if !domain_start.is_finite()
            || !domain_end.is_finite()
            || interior.iter().any(|k| !k.is_finite())
        {
            return Err(GeomError::NonFinite);
        }
        if interior
            .iter()
            .any(|&k| k <= domain_start || k >= domain_end)
        {
            return Err(GeomError::InvalidKnotVector(
                "interior knots must lie strictly inside the domain",
            ));
        }
        let mut knots = Vec::with_capacity(interior.len() + 2 * (degree + 1));
        knots.extend(std::iter::repeat_n(domain_start, degree + 1));
        knots.extend_from_slice(interior);
        knots.extend(std::iter::repeat_n(domain_end, degree + 1));
        Self::new(knots, degree)
    }

    /// The knots.
    pub fn as_slice(&self) -> &[f64] {
        &self.knots
    }

    /// Two knots closer than this are the same knot: 1e-12 of the knot range
    /// `u_m - u_0`, but at least four ulps of the largest knot magnitude.
    pub fn tolerance(&self) -> f64 {
        let (first, last) = self.ends();
        let range = KNOT_RELATIVE_TOLERANCE * (last - first);
        let ulps = 4.0 * f64::EPSILON * first.abs().max(last.abs());
        range.max(ulps)
    }

    /// The highest degree whose length requirement this vector meets.
    fn max_degree(&self) -> usize {
        (self.knots.len() / 2).saturating_sub(1)
    }

    /// The domain `[u_p, u_{n+1}]` of a curve of degree `p` on this vector.
    ///
    /// A degree too high for the vector's length is reduced to the highest
    /// one it supports.
    pub fn domain(&self, p: usize) -> (f64, f64) {
        let p = p.min(self.max_degree());
        let k = &self.knots;
        match (k.get(p), k.len().checked_sub(p + 1).and_then(|i| k.get(i))) {
            (Some(&a), Some(&b)) => (a, b),
            _ => self.ends(),
        }
    }

    /// First and last knot.
    fn ends(&self) -> (f64, f64) {
        match (self.knots.first(), self.knots.last()) {
            (Some(&a), Some(&b)) => (a, b),
            _ => (0.0, 0.0),
        }
    }

    /// The knot span index `i` with `u_i <= u < u_{i+1}` for a curve of degree
    /// `p` (The NURBS Book, A2.1).
    ///
    /// The result is always a non-empty span inside the domain, so it lies in
    /// `p..=n` where `n + 1` is the number of control points. Parameters below
    /// the domain (and NaN) give the first span; parameters at or beyond the
    /// end of the domain give the last non-empty span, so that `u = u_{n+1}`
    /// evaluates the end of the curve.
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidDegree`] if the vector is too short for degree `p`.
    pub fn find_span(&self, p: usize, u: f64) -> Result<usize, GeomError> {
        if p == 0 || p > self.max_degree() {
            return Err(GeomError::InvalidDegree { degree: p });
        }
        Ok(self.span(p, u))
    }

    /// [`KnotVector::find_span`] without the degree check.
    ///
    /// Requires `1 <= p` and `knots.len() >= 2 (p + 1)`, which holds for the
    /// degree of any curve that owns this vector. The domain is non-degenerate
    /// by the type invariant, so a non-empty span always exists.
    pub(crate) fn span(&self, p: usize, u: f64) -> usize {
        let k = &self.knots;
        let n = k.len() - p - 2;
        let (lo, hi) = (k[p], k[n + 1]);
        if u.is_nan() || u <= lo {
            let mut s = p;
            while s < n && k[s + 1] <= k[s] {
                s += 1;
            }
            return s;
        }
        if u >= hi {
            let mut s = n;
            while s > p && k[s] >= k[s + 1] {
                s -= 1;
            }
            return s;
        }
        // Binary search keeping k[low] <= u < k[high].
        let (mut low, mut high) = (p, n + 1);
        while high - low > 1 {
            let mid = usize::midpoint(low, high);
            if u < k[mid] {
                high = mid;
            } else {
                low = mid;
            }
        }
        low
    }

    /// How many knots equal `u` (to within [`KnotVector::tolerance`]).
    pub fn multiplicity(&self, u: f64) -> usize {
        let tol = self.tolerance();
        self.knots.iter().filter(|&&k| (k - u).abs() <= tol).count()
    }

    /// The distinct knot values in increasing order.
    pub fn breakpoints(&self) -> Vec<f64> {
        self.distinct().into_iter().map(|(u, _)| u).collect()
    }

    /// The distinct knot values with their multiplicities.
    pub(crate) fn distinct(&self) -> Vec<(f64, usize)> {
        let mut out: Vec<(f64, usize)> = Vec::new();
        for &u in &self.knots {
            match out.last_mut() {
                Some((v, m)) if self.same(*v, u) => *m += 1,
                _ => out.push((u, 1)),
            }
        }
        out
    }

    /// Returns `true` if two knot values are the same knot, that is, closer
    /// than [`KnotVector::tolerance`].
    ///
    /// Knots taken from a snapped vector are bit-identical when equal and
    /// further apart than the tolerance otherwise, so this never depends on
    /// rounding.
    pub(crate) fn same(&self, a: f64, b: f64) -> bool {
        (a - b).abs() <= self.tolerance()
    }

    /// Returns `true` if the first and the last `p + 1` knots are equal.
    pub fn is_clamped(&self, p: usize) -> bool {
        if p == 0 || p > self.max_degree() {
            return false;
        }
        let k = &self.knots;
        let n = k.len();
        k[..=p].iter().all(|&u| self.same(u, k[0]))
            && k[n - p - 1..].iter().all(|&u| self.same(u, k[n - 1]))
    }

    /// Snaps each knot within the tolerance of the start of its run to that
    /// value.
    fn snap(&mut self) {
        let tol = self.tolerance();
        let Some(&first) = self.knots.first() else {
            return;
        };
        let mut run_start = first;
        for k in &mut self.knots {
            if *k - run_start <= tol {
                *k = run_start;
            } else {
                run_start = *k;
            }
        }
    }

    /// Checks length, domain and multiplicities for degree `p`; assumes the
    /// knots are finite, sorted and snapped.
    pub(crate) fn validate(&self, p: usize) -> Result<(), GeomError> {
        check_degree(p)?;
        if self.knots.len() < 2 * (p + 1) {
            return Err(GeomError::InvalidKnotVector(
                "a knot vector of degree p needs at least 2(p + 1) knots",
            ));
        }
        let (start, end) = self.domain(p);
        if end <= start {
            return Err(GeomError::InvalidKnotVector("the domain has zero length"));
        }
        for (u, m) in self.distinct() {
            if m > p + 1 {
                return Err(GeomError::InvalidKnotVector(
                    "no knot may occur more than p + 1 times",
                ));
            }
            if u > start && u < end && m > p {
                return Err(GeomError::InvalidKnotVector(
                    "interior knots may occur at most p times",
                ));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kv(knots: &[f64], p: usize) -> KnotVector {
        KnotVector::new(knots.to_vec(), p).unwrap()
    }

    #[test]
    fn find_span_handles_domain_ends_and_repeated_knots() {
        // The NURBS Book, example 2.3 style vector with a double knot.
        let k = kv(&[0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 5.0, 5.0, 5.0], 2);
        assert_eq!(k.find_span(2, 0.0).unwrap(), 2);
        assert_eq!(k.find_span(2, -1.0).unwrap(), 2);
        assert_eq!(k.find_span(2, 0.5).unwrap(), 2);
        assert_eq!(k.find_span(2, 1.0).unwrap(), 3);
        assert_eq!(k.find_span(2, 2.5).unwrap(), 4);
        // At a double knot the span starts at its last copy.
        assert_eq!(k.find_span(2, 4.0).unwrap(), 7);
        assert_eq!(k.find_span(2, 4.5).unwrap(), 7);
        // The end of the domain and beyond give the last non-empty span.
        assert_eq!(k.find_span(2, 5.0).unwrap(), 7);
        assert_eq!(k.find_span(2, 9.0).unwrap(), 7);
        assert_eq!(k.find_span(2, f64::NAN).unwrap(), 2);
        assert!(k.find_span(5, 0.0).is_err());
    }

    #[test]
    fn find_span_skips_empty_spans_at_unclamped_ends() {
        // Domain [1, 2] for p = 2; the start value has multiplicity 3 at
        // indices 1..=3 and the end value appears at indices 4..=6.
        let k = kv(&[0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0], 2);
        assert_eq!(k.domain(2), (1.0, 2.0));
        assert_eq!(k.find_span(2, 1.0).unwrap(), 3);
        assert_eq!(k.find_span(2, 2.0).unwrap(), 3);
    }

    #[test]
    fn validation_rejects_bad_vectors() {
        use GeomError::{InvalidDegree, InvalidKnotVector, NonFinite};
        let err = |knots: &[f64], p| KnotVector::new(knots.to_vec(), p).unwrap_err();
        assert!(matches!(
            err(&[0.0, 0.0, 1.0, 1.0], 0),
            InvalidDegree { degree: 0 }
        ));
        assert!(matches!(err(&[0.0, 0.0, 1.0], 1), InvalidKnotVector(_)));
        assert!(matches!(
            err(&[0.0, 1.0, 0.5, 1.0], 1),
            InvalidKnotVector(_)
        ));
        assert!(matches!(err(&[0.0, f64::NAN, 1.0, 1.0], 1), NonFinite));
        assert!(matches!(
            err(&[1.0, 1.0, 1.0, 1.0], 1),
            InvalidKnotVector(_)
        ));
        // Interior multiplicity p + 1 breaks continuity.
        assert!(matches!(
            err(&[0.0, 0.0, 0.5, 0.5, 1.0, 1.0], 1),
            InvalidKnotVector(_)
        ));
        // End multiplicity above p + 1.
        assert!(matches!(
            err(&[0.0, 0.0, 0.0, 1.0, 1.0], 1),
            InvalidKnotVector(_)
        ));
    }

    #[test]
    fn nearly_equal_knots_are_snapped() {
        let k = kv(&[0.0, 0.0, 0.0, 0.5, 0.5 + 1e-14, 1.0, 1.0, 1.0], 2);
        assert_eq!(k.multiplicity(0.5), 2);
        assert_eq!(k.breakpoints(), vec![0.0, 0.5, 1.0]);
        assert_eq!(k.as_slice()[4].to_bits(), 0.5_f64.to_bits());
    }

    #[test]
    fn constructors_build_clamped_vectors() {
        let k = KnotVector::clamped_uniform(3, 6).unwrap();
        assert_eq!(
            k.as_slice(),
            &[0.0, 0.0, 0.0, 0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 1.0, 1.0, 1.0]
        );
        assert!(k.is_clamped(3));
        assert!(!k.is_clamped(4));
        assert!(matches!(
            KnotVector::clamped_uniform(3, 3),
            Err(GeomError::ControlPointCount {
                expected: 4,
                found: 3
            })
        ));
        let k = KnotVector::clamped_from_interior(2, -1.0, 3.0, &[0.0, 0.0, 2.0]).unwrap();
        assert_eq!(k.domain(2), (-1.0, 3.0));
        assert_eq!(k.multiplicity(0.0), 2);
        assert!(KnotVector::clamped_from_interior(2, 0.0, 1.0, &[1.0]).is_err());
        assert!(KnotVector::clamped_from_interior(2, 0.0, 1.0, &[0.6, 0.4]).is_err());
    }
}
