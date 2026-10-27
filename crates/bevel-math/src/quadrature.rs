// SPDX-License-Identifier: GPL-3.0-or-later

//! Gauss–Legendre quadrature.
//!
//! An `n`-point Gauss–Legendre rule integrates every polynomial of degree
//! up to `2n - 1` exactly (up to rounding error). Nodes and weights are
//! computed by Newton iteration on the Legendre polynomial `P_n`.

use crate::{Interval, MathError};

/// Largest number of points [`gauss_legendre`] supports.
const MAX_POINTS: usize = 64;
/// Points of the two rules compared by [`integrate_adaptive`].
const COARSE: usize = 8;
const FINE: usize = 16;

/// Nodes and weights of the `n`-point Gauss–Legendre rule on `[-1, 1]`.
///
/// Returns `(nodes, weights)` with the nodes in ascending order; the
/// weights sum to 2. Accurate to about `1e-15`.
///
/// # Errors
/// [`MathError::InvalidInput`] unless `1 <= n <= 64`.
pub fn gauss_legendre(n: usize) -> Result<(Vec<f64>, Vec<f64>), MathError> {
    if n == 0 || n > MAX_POINTS {
        return Err(MathError::InvalidInput(
            "Gauss-Legendre rules need between 1 and 64 points",
        ));
    }
    let degree = n as f64;
    // The positive roots of P_n, largest first, and their weights.
    let half = n.div_ceil(2);
    let mut xs = Vec::with_capacity(half);
    let mut ws = Vec::with_capacity(half);
    for i in 0..half {
        // Classic initial guess for the i-th root.
        let mut x = (std::f64::consts::PI * (i as f64 + 0.75) / (degree + 0.5)).cos();
        let mut converged = false;
        for _ in 0..100 {
            let (p, dp) = legendre(n, x);
            let dx = p / dp;
            x -= dx;
            if dx.abs() <= 4.0 * f64::EPSILON {
                converged = true;
                break;
            }
        }
        if !converged {
            return Err(MathError::NotConverged { iterations: 100 });
        }
        // The derivative at the converged root gives the weight.
        let (_, derivative) = legendre(n, x);
        let is_middle = n % 2 == 1 && i + 1 == half;
        xs.push(if is_middle { 0.0 } else { x });
        ws.push(2.0 / ((1.0 - x * x) * derivative * derivative));
    }

    let mut nodes = Vec::with_capacity(n);
    let mut weights = Vec::with_capacity(n);
    for (x, w) in xs.iter().zip(&ws) {
        nodes.push(0.0 - x);
        weights.push(*w);
    }
    // Mirror the positive half; for odd `n` the middle node is shared.
    for (x, w) in xs.iter().zip(&ws).rev().skip(n % 2) {
        nodes.push(*x);
        weights.push(*w);
    }
    Ok((nodes, weights))
}

/// `P_n(x)` and `P_n'(x)` for `|x| < 1`, by Bonnet's recurrence.
fn legendre(n: usize, x: f64) -> (f64, f64) {
    let (mut p, mut previous) = (1.0, 0.0);
    for k in 1..=n {
        let k = k as f64;
        (p, previous) = (((2.0 * k - 1.0) * x * p - (k - 1.0) * previous) / k, p);
    }
    (p, n as f64 * (x * p - previous) / (x * x - 1.0))
}

/// Applies a rule to `f` over `interval`.
fn apply_rule(f: &impl Fn(f64) -> f64, interval: Interval, rule: &(Vec<f64>, Vec<f64>)) -> f64 {
    let half = 0.5 * interval.length();
    let mid = interval.mid();
    let sum: f64 = rule
        .0
        .iter()
        .zip(&rule.1)
        .map(|(x, w)| w * f(mid + half * x))
        .sum();
    sum * half
}

/// Integral of `f` over `interval` by an `n`-point Gauss–Legendre rule.
///
/// Exact for polynomials of degree up to `2n - 1`.
///
/// # Errors
/// [`MathError::InvalidInput`] unless `1 <= n <= 64` and `interval` is finite.
pub fn integrate(f: impl Fn(f64) -> f64, interval: Interval, n: usize) -> Result<f64, MathError> {
    if !(interval.min.is_finite() && interval.max.is_finite()) {
        return Err(MathError::InvalidInput("the interval must be finite"));
    }
    let rule = gauss_legendre(n)?;
    Ok(apply_rule(&f, interval, &rule))
}

/// Integral of `f` over `interval` to an absolute error of about `tol`.
///
/// On each subinterval the 8-point and 16-point rules are compared. If
/// they differ by at most the subinterval's share of the tolerance
/// (`tol * length / total length`), the 16-point value is accepted;
/// otherwise the subinterval is bisected. Intervals are processed left to
/// right, so the result is deterministic.
///
/// # Errors
/// [`MathError::NotConverged`] if a subinterval still disagrees after
/// `max_depth` bisections (the error carries `max_depth`);
/// [`MathError::InvalidInput`] if `tol` is not positive and finite, the
/// interval is not finite, or `f` returns a non-finite value.
pub fn integrate_adaptive(
    f: impl Fn(f64) -> f64,
    interval: Interval,
    tol: f64,
    max_depth: usize,
) -> Result<f64, MathError> {
    if !(tol > 0.0 && tol.is_finite()) {
        return Err(MathError::InvalidInput(
            "tolerance must be positive and finite",
        ));
    }
    if !(interval.min.is_finite() && interval.max.is_finite()) {
        return Err(MathError::InvalidInput("the interval must be finite"));
    }
    let total = interval.length();
    if total <= 0.0 {
        return Ok(0.0);
    }
    let coarse = gauss_legendre(COARSE)?;
    let fine = gauss_legendre(FINE)?;
    let mut sum = 0.0;
    let mut stack = vec![(interval, 0_usize)];
    while let Some((part, depth)) = stack.pop() {
        let low = apply_rule(&f, part, &coarse);
        let high = apply_rule(&f, part, &fine);
        if !(low.is_finite() && high.is_finite()) {
            return Err(MathError::InvalidInput(
                "the integrand returned a non-finite value",
            ));
        }
        if (high - low).abs() <= tol * part.length() / total {
            sum += high;
        } else if depth >= max_depth {
            return Err(MathError::NotConverged {
                iterations: max_depth,
            });
        } else {
            let (left, right) = part.split(part.mid());
            stack.push((right, depth + 1));
            stack.push((left, depth + 1));
        }
    }
    Ok(sum)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    #[test]
    fn small_rules_match_known_values() {
        let (x, w) = gauss_legendre(1).unwrap();
        assert!(x[0].abs() < 1e-16 && (w[0] - 2.0).abs() < 1e-15);
        let (x, w) = gauss_legendre(2).unwrap();
        let g = 1.0 / 3.0_f64.sqrt();
        assert!((x[0] + g).abs() < 1e-15 && (x[1] - g).abs() < 1e-15);
        assert!((w[0] - 1.0).abs() < 1e-15 && (w[1] - 1.0).abs() < 1e-15);
        let (x, w) = gauss_legendre(3).unwrap();
        let r = (0.6_f64).sqrt();
        assert!((x[0] + r).abs() < 1e-15 && x[1].abs() < 1e-16 && (x[2] - r).abs() < 1e-15);
        assert!((w[0] - 5.0 / 9.0).abs() < 1e-15 && (w[1] - 8.0 / 9.0).abs() < 1e-15);
    }

    #[test]
    fn every_supported_size_is_symmetric_and_sums_to_two() {
        for n in 1..=64 {
            let (x, w) = gauss_legendre(n).unwrap();
            assert_eq!((x.len(), w.len()), (n, n));
            assert!((w.iter().sum::<f64>() - 2.0).abs() < 1e-14, "n={n}");
            for i in 0..n {
                assert!((x[i] + x[n - 1 - i]).abs() < 1e-15, "n={n}");
                assert!((w[i] - w[n - 1 - i]).abs() < 1e-15, "n={n}");
            }
            assert!(x.windows(2).all(|p| p[0] < p[1]), "n={n}");
        }
    }

    #[test]
    fn size_out_of_range_is_rejected() {
        assert!(matches!(gauss_legendre(0), Err(MathError::InvalidInput(_))));
        assert!(matches!(
            gauss_legendre(65),
            Err(MathError::InvalidInput(_))
        ));
        assert!(matches!(
            integrate(|x| x, Interval::new(0.0, 1.0), 0),
            Err(MathError::InvalidInput(_))
        ));
    }

    #[test]
    fn integrates_smooth_functions() {
        let pi = std::f64::consts::PI;
        let v = integrate(f64::sin, Interval::new(0.0, pi), 12).unwrap();
        assert!((v - 2.0).abs() < 1e-14);
        // Reversed bounds are sorted by Interval::new, so the sign is positive.
        let v = integrate(|x| x * x, Interval::new(3.0, 0.0), 2).unwrap();
        assert!((v - 9.0).abs() < 1e-14);
    }

    #[test]
    fn adaptive_handles_kinks_and_endpoint_singularities() {
        let v =
            integrate_adaptive(|x| (x - 0.3).abs(), Interval::new(0.0, 1.0), 1e-10, 40).unwrap();
        assert!((v - (0.045 + 0.245)).abs() < 1e-10);
        let v = integrate_adaptive(f64::sqrt, Interval::new(0.0, 1.0), 1e-9, 60).unwrap();
        assert!((v - 2.0 / 3.0).abs() < 1e-9);
    }

    #[test]
    fn adaptive_reports_non_convergence() {
        let r = integrate_adaptive(|x| (200.0 * x).sin(), Interval::new(0.0, 10.0), 1e-12, 2);
        assert_eq!(r, Err(MathError::NotConverged { iterations: 2 }));
    }

    #[test]
    fn adaptive_rejects_bad_arguments() {
        let i = Interval::new(0.0, 1.0);
        assert!(matches!(
            integrate_adaptive(|x| x, i, 0.0, 10),
            Err(MathError::InvalidInput(_))
        ));
        assert!(matches!(
            integrate_adaptive(|_| f64::NAN, i, 1e-9, 10),
            Err(MathError::InvalidInput(_))
        ));
        let v = integrate_adaptive(|x| x, Interval::new(2.0, 2.0), 1e-9, 10).unwrap();
        assert!(v.abs() < 1e-300);
    }

    proptest! {
        #[test]
        fn exact_for_polynomials_up_to_degree_2n_minus_1(
            n in 1..=20usize,
            coeffs in prop::collection::vec(-1.0..1.0f64, 1..=40),
            ends in (-1.5..1.5f64, -1.5..1.5f64),
        ) {
            let coeffs = &coeffs[..coeffs.len().min(2 * n)];
            let interval = Interval::new(ends.0, ends.1);
            let f = |t: f64| coeffs.iter().rev().fold(0.0, |acc, c| acc * t + c);
            let (mut exact, mut scale) = (0.0, 0.0);
            for (i, c) in coeffs.iter().enumerate() {
                let k = i as f64 + 1.0;
                exact += c * (interval.max.powf(k) - interval.min.powf(k)) / k;
                let r = interval.min.abs().max(interval.max.abs());
                scale += c.abs() * r.powf(k) / k;
            }
            let got = integrate(f, interval, n).unwrap();
            prop_assert!((got - exact).abs() <= 1e-12 * scale.max(1e-300) + 1e-300,
                "got {got}, exact {exact}");
        }
    }
}
