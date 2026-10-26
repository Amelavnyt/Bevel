// SPDX-License-Identifier: GPL-3.0-or-later

//! B-spline basis functions and their derivatives.
//!
//! On the knot span `[u_i, u_{i+1})` only the `p + 1` functions
//! `N_{i-p,p}, ..., N_{i,p}` are non-zero. [`basis_funs`] computes them with
//! the triangular Cox-de Boor recurrence of The NURBS Book, A2.2, which never
//! divides by zero on a non-empty span. [`ders_basis_funs`] computes them and
//! their derivatives with A2.3 (eq. 2.9), reusing the same table.
//!
//! Neither function allocates: the work tables are fixed-size arrays sized
//! for [`MAX_DEGREE`].

use crate::{GeomError, KnotVector, MAX_DEGREE};

/// One row of basis function values, `N_{i-p,p}, ..., N_{i,p}` in the first
/// `p + 1` entries.
pub(crate) type Row = [f64; MAX_DEGREE + 1];

/// A2.2 on a raw knot slice.
///
/// Requires `1 <= p <= MAX_DEGREE` and `p <= span`, `span + p < knots.len()`;
/// a span from [`KnotVector::span`] on a vector valid for `p` satisfies this.
/// The span must be non-empty, which `KnotVector::span` guarantees.
pub(crate) fn basis_funs_raw(knots: &[f64], span: usize, u: f64, p: usize) -> Row {
    let mut n = [0.0; MAX_DEGREE + 1];
    let mut left = [0.0; MAX_DEGREE + 1];
    let mut right = [0.0; MAX_DEGREE + 1];
    n[0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            let temp = n[r] / (right[r + 1] + left[j - r]);
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    n
}

/// A2.3 on a raw knot slice: row `k` of the result holds the `k`-th
/// derivatives of the non-zero basis functions, for `k <= n.min(p)`; the
/// remaining rows are zero (a degree `p` polynomial has no higher
/// derivatives).
///
/// Same requirements as [`basis_funs_raw`].
pub(crate) fn ders_basis_funs_raw(
    knots: &[f64],
    span: usize,
    u: f64,
    p: usize,
    n: usize,
) -> [Row; MAX_DEGREE + 1] {
    let n = n.min(p);
    let mut ders = [[0.0; MAX_DEGREE + 1]; MAX_DEGREE + 1];
    let mut ndu = [[0.0; MAX_DEGREE + 1]; MAX_DEGREE + 1];
    let mut left = [0.0; MAX_DEGREE + 1];
    let mut right = [0.0; MAX_DEGREE + 1];
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            // Lower triangle: knot differences.
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = ndu[r][j - 1] / ndu[j][r];
            // Upper triangle: basis functions.
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    for j in 0..=p {
        ders[0][j] = ndu[j][p];
    }
    // Eq. 2.9, with the two rows of `a` used alternately.
    let mut a = [[0.0; MAX_DEGREE + 1]; 2];
    for r in 0..=p {
        let (mut s1, mut s2) = (0, 1);
        a[0][0] = 1.0;
        for k in 1..=n {
            let mut d = 0.0;
            let pk = p - k;
            if r >= k {
                let rk = r - k;
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }
            // j1 = 1 if r - k >= -1, else k - r; j2 = k - 1 if r - 1 <= p - k, else p - r.
            let j1 = if r + 1 >= k { 1 } else { k - r };
            let j2 = if r + k <= p + 1 { k - 1 } else { p - r };
            for j in j1..=j2 {
                // Here r + j >= k, so the index r - k + j is non-negative.
                let rkj = r + j - k;
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rkj];
                d += a[s2][j] * ndu[rkj][pk];
            }
            if r <= pk {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            ders[k][r] = d;
            std::mem::swap(&mut s1, &mut s2);
        }
    }
    // Multiply by the factors p! / (p - k)!.
    let mut factor = p as f64;
    for (k, row) in ders.iter_mut().enumerate().take(n + 1).skip(1) {
        for v in row.iter_mut().take(p + 1) {
            *v *= factor;
        }
        factor *= (p - k) as f64;
    }
    ders
}

/// `u` clamped to the domain of degree `p`; NaN stays NaN.
fn clamp_to_domain(knots: &KnotVector, p: usize, u: f64) -> f64 {
    let (a, b) = knots.domain(p);
    if u.is_nan() { u } else { u.clamp(a, b) }
}

/// Evaluates the `p + 1` basis functions of degree `p` that are non-zero at
/// `u` (The NURBS Book, A2.2).
///
/// Writes `N_{i-p,p}(u), ..., N_{i,p}(u)` to `out[0..=p]` and returns the
/// span index `i` (see [`KnotVector::find_span`]). Parameters outside the
/// domain are clamped to it, so the values are never negative.
///
/// # Errors
///
/// [`GeomError::InvalidDegree`] if the vector is too short for degree `p` or
/// `p > MAX_DEGREE`, and [`GeomError::InvalidArgument`] if `out` has fewer
/// than `p + 1` entries.
pub fn basis_funs(
    knots: &KnotVector,
    p: usize,
    u: f64,
    out: &mut [f64],
) -> Result<usize, GeomError> {
    if p > MAX_DEGREE {
        return Err(GeomError::InvalidDegree { degree: p });
    }
    let span = knots.find_span(p, u)?;
    let u = clamp_to_domain(knots, p, u);
    let Some(out) = out.get_mut(..=p) else {
        return Err(GeomError::InvalidArgument("output buffer too short"));
    };
    let n = basis_funs_raw(knots.as_slice(), span, u, p);
    out.copy_from_slice(&n[..=p]);
    Ok(span)
}

/// Evaluates the non-zero basis functions of degree `p` at `u` and their
/// derivatives up to order `n` (The NURBS Book, A2.3).
///
/// `out` is a row-major `(n + 1) x (p + 1)` table: `out[k * (p + 1) + j]` is
/// the `k`-th derivative of `N_{i-p+j,p}` at `u`, where `i` is the returned
/// span index. Rows with `k > p` are zero. Parameters outside the domain are
/// clamped to it.
///
/// # Errors
///
/// As [`basis_funs`], with `out` needing `(n + 1) (p + 1)` entries.
pub fn ders_basis_funs(
    knots: &KnotVector,
    p: usize,
    u: f64,
    n: usize,
    out: &mut [f64],
) -> Result<usize, GeomError> {
    if p > MAX_DEGREE {
        return Err(GeomError::InvalidDegree { degree: p });
    }
    let span = knots.find_span(p, u)?;
    let u = clamp_to_domain(knots, p, u);
    let needed = (n + 1).saturating_mul(p + 1);
    let Some(out) = out.get_mut(..needed) else {
        return Err(GeomError::InvalidArgument("output buffer too short"));
    };
    let ders = ders_basis_funs_raw(knots.as_slice(), span, u, p, n);
    for (k, row) in out.chunks_exact_mut(p + 1).enumerate() {
        match ders.get(k) {
            Some(d) => row.copy_from_slice(&d[..=p]),
            None => row.fill(0.0),
        }
    }
    Ok(span)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_nurbs_book_example_2_3() {
        // U = {0,0,0,1,2,3,4,4,5,5,5}, p = 2, u = 5/2: N_{2,2} = 1/8,
        // N_{3,2} = 6/8, N_{4,2} = 1/8 (The NURBS Book, example 2.3).
        let k = KnotVector::new(
            vec![0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 5.0, 5.0, 5.0],
            2,
        )
        .unwrap();
        let mut n = [0.0; 3];
        let span = basis_funs(&k, 2, 2.5, &mut n).unwrap();
        assert_eq!(span, 4);
        for (a, b) in n.iter().zip([0.125, 0.75, 0.125]) {
            assert!((a - b).abs() < 1e-15);
        }
        // Derivatives (example 2.4): N'_{2,2} = -1/2, N'_{3,2} = 0, N'_{4,2} = 1/2.
        let mut d = [0.0; 9];
        ders_basis_funs(&k, 2, 2.5, 2, &mut d).unwrap();
        for (a, b) in d[3..6].iter().zip([-0.5, 0.0, 0.5]) {
            assert!((a - b).abs() < 1e-14);
        }
        // Second derivatives are 1, -2, 1.
        for (a, b) in d[6..9].iter().zip([1.0, -2.0, 1.0]) {
            assert!((a - b).abs() < 1e-14);
        }
    }

    /// Found by the property tests: an evenly spaced parameter list can end
    /// an ulp past the domain, where the extrapolated basis functions go
    /// slightly negative.
    #[test]
    fn parameters_past_the_domain_are_clamped() {
        let k = KnotVector::clamped_from_interior(4, 0.3, 4.2, &[1.0, 2.5, 2.5, 3.9]).unwrap();
        let mut at_end = [0.0; 5];
        let mut past = [0.0; 5];
        basis_funs(&k, 4, 4.2, &mut at_end).unwrap();
        basis_funs(&k, 4, 4.2 + 1e-9, &mut past).unwrap();
        assert!(
            at_end
                .iter()
                .zip(&past)
                .all(|(a, b)| a.to_bits() == b.to_bits())
        );
        assert!(past.iter().all(|&v| v >= 0.0));
        let mut d = [0.0; 10];
        ders_basis_funs(&k, 4, 0.3 - 1e-9, 1, &mut d).unwrap();
        assert!(d[..5].iter().all(|&v| v >= 0.0));
    }

    #[test]
    fn rejects_short_buffers_and_bad_degrees() {
        let k = KnotVector::clamped_uniform(3, 5).unwrap();
        let mut small = [0.0; 3];
        assert!(matches!(
            basis_funs(&k, 3, 0.5, &mut small),
            Err(GeomError::InvalidArgument(_))
        ));
        let mut big = [0.0; 64];
        assert!(matches!(
            basis_funs(&k, 7, 0.5, &mut big),
            Err(GeomError::InvalidDegree { degree: 7 })
        ));
        assert!(matches!(
            ders_basis_funs(&k, 3, 0.5, 3, &mut small),
            Err(GeomError::InvalidArgument(_))
        ));
        // Derivative rows above the degree are zero.
        ders_basis_funs(&k, 3, 0.5, 5, &mut big[..24]).unwrap();
        assert!(big[16..24].iter().all(|v| v.abs() < 1e-300));
    }
}
