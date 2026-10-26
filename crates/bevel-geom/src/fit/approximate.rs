// SPDX-License-Identifier: GPL-3.0-or-later

//! Least-squares curve approximation (The NURBS Book, section 9.4.1).
//!
//! Given points `Q_0, ..., Q_m` and a number `n + 1 <= m + 1` of control
//! points, the curve `C(u) = sum_j N_{j,p}(u) P_j` has `P_0 = Q_0` and
//! `P_n = Q_m` fixed and the other control points chosen to minimise
//! `sum_{k=1}^{m-1} |Q_k - C(u*_k)|^2` (section 9.4.1). Setting the gradient to
//! zero gives the normal equations `(N^T N) P = R`, where `N` is
//! the `(m - 1) x (n - 1)` matrix of `N_{j,p}(u*_k)` for the free control
//! points `j = 1, ..., n - 1` and
//! `R_k = Q_k - N_{0,p}(u*_k) Q_0 - N_{n,p}(u*_k) Q_m`.
//!
//! # Knots
//!
//! The interior knots follow the density of the data parameters
//! (section 9.4.1): with `d = (m + 1) / (n - p + 1)`, `i = floor(j d)`
//! and `alpha = j d - i`, the knot is
//! `u_{p+j} = (1 - alpha) u*_{i-1} + alpha u*_i` for `j = 1, ..., n - p`.
//! This puts at least one parameter in every knot span, which keeps the
//! least-squares matrix of full rank. The knots are computed in integer
//! arithmetic, so there is no rounding in `floor`.
//!
//! # Solving
//!
//! The system is solved through a Householder QR factorisation of `N`, not by
//! forming `N^T N`: the normal equations square the condition number of the
//! problem. A rank-deficient `N` (which the knot choice rules out for valid
//! data) is reported as an error.
//!
//! # Interpolation
//!
//! With as many control points as data points there is nothing to
//! minimise, and the fit is the interpolation of A9.1 with averaged knots.
//!
//! # Deviation and the residual
//!
//! The least-squares fit is optimal for its own knot vector. Different
//! control point counts get different knot vectors (the knot sets are not
//! nested), so the residual is not guaranteed to decrease strictly with each
//! added control point, only to reach zero at the full count.

use super::interpolate::interpolate_at;
use super::solve::{matrix_to_points, points_to_matrix, set, solve_least_squares};
use super::{FitPoint, Parametrization, check_points, parameters};
use crate::basis::basis_funs_raw;
use crate::knots::check_degree;
use crate::{GeomError, KnotVector, NurbsCurve};
use nalgebra::DMatrix;

/// A non-rational curve of degree `degree` with `control_count` control
/// points that fits `points` in the least-squares sense and passes exactly
/// through the first and the last point (The NURBS Book, section 9.4.1).
///
/// The curve is defined on `[0, 1]`; the parameter of each point comes from
/// `method`. With `control_count == points.len()` the result interpolates
/// all points (A9.1 with averaged knots, see [`interpolate`](super::interpolate)).
///
/// # Errors
///
/// [`GeomError::InvalidDegree`] for a degree outside `1..=MAX_DEGREE`;
/// [`GeomError::ControlPointCount`] if `control_count <= degree`;
/// [`GeomError::InvalidArgument`] if `control_count` exceeds the number of
/// points, there are fewer than `degree + 1` points, or consecutive points
/// coincide; [`GeomError::NonFinite`] for NaN or infinite coordinates;
/// [`GeomError::NotConverged`] if the least-squares system is rank
/// deficient to working precision.
pub fn approximate<P: FitPoint>(
    points: &[P],
    degree: usize,
    control_count: usize,
    method: Parametrization,
) -> Result<NurbsCurve<P::Homogeneous>, GeomError> {
    check_degree(degree)?;
    check_points(points, degree + 1)?;
    if control_count <= degree {
        return Err(GeomError::ControlPointCount {
            expected: degree + 1,
            found: control_count,
        });
    }
    if control_count > points.len() {
        return Err(GeomError::InvalidArgument(
            "more control points than data points",
        ));
    }
    let params = parameters(points, method)?;
    fit(points, &params, degree, control_count)
}

/// A non-rational curve of degree `degree` through the first and last of
/// `points` that stays within `tolerance` of every point, with as few
/// control points as a bisection on the count finds.
///
/// The deviation of a point `Q_k` is `|Q_k - C(u*_k)|` at its own parameter
/// `u*_k` (from `method`, as in [`approximate`]). This bounds the distance
/// from `Q_k` to the curve, but says nothing about the curve between the
/// points. The search starts with `degree + 1` control points (a Bezier
/// curve) and, if that is not enough, bisects between that and the point
/// count, where the fit is an interpolation. The knot sets of different
/// counts are not nested, so the deviation is not strictly monotone in the
/// count; the bisection keeps a count that is known to meet the tolerance
/// and returns the smallest one it tried, which is not always the global
/// minimum. The search is deterministic.
///
/// # Errors
///
/// [`GeomError::ToleranceNotMet`] with the deviation of the interpolating
/// curve if even that exceeds `tolerance` (a tolerance near rounding
/// level); [`GeomError::InvalidArgument`] if `tolerance` is not positive;
/// [`GeomError::NonFinite`] for a NaN or infinite tolerance or coordinates;
/// otherwise the errors of [`approximate`].
pub fn approximate_to_tolerance<P: FitPoint>(
    points: &[P],
    degree: usize,
    tolerance: f64,
    method: Parametrization,
) -> Result<NurbsCurve<P::Homogeneous>, GeomError> {
    check_degree(degree)?;
    check_points(points, degree + 1)?;
    if !tolerance.is_finite() {
        return Err(GeomError::NonFinite);
    }
    if tolerance <= 0.0 {
        return Err(GeomError::InvalidArgument("the tolerance must be positive"));
    }
    let params = parameters(points, method)?;
    let total = points.len();

    // A count that fails to fit (rank deficiency) counts as not meeting the
    // tolerance.
    let attempt = |count: usize| {
        fit(points, &params, degree, count).ok().map(|curve| {
            let error = max_deviation(&curve, points, &params);
            (curve, error)
        })
    };

    let mut low = degree + 1;
    if let Some((curve, error)) = attempt(low)
        && error <= tolerance
    {
        return Ok(curve);
    }
    // Invariant: `low` does not meet the tolerance, `high` does.
    let mut high = total;
    let curve = fit(points, &params, degree, high)?;
    let error = max_deviation(&curve, points, &params);
    if error > tolerance {
        return Err(GeomError::ToleranceNotMet { error });
    }
    let mut best = curve;
    while high - low > 1 {
        let mid = low + (high - low) / 2;
        match attempt(mid) {
            Some((curve, error)) if error <= tolerance => {
                high = mid;
                best = curve;
            }
            _ => low = mid,
        }
    }
    Ok(best)
}

/// The largest distance between a point and the curve at the point's
/// parameter.
fn max_deviation<P: FitPoint>(
    curve: &NurbsCurve<P::Homogeneous>,
    points: &[P],
    params: &[f64],
) -> f64 {
    points
        .iter()
        .zip(params)
        .map(|(&q, &u)| (curve.point(u) - q).length())
        .fold(0.0, f64::max)
}

/// The knot vector of section 9.4.1 for `count` control points, which
/// must be fewer than `params.len()`.
fn approximation_knots(
    params: &[f64],
    degree: usize,
    count: usize,
) -> Result<KnotVector, GeomError> {
    let spans = count - degree;
    let points = params.len();
    let interior = (1..spans)
        .map(|j| {
            // jd = j (m + 1) / (n - p + 1), as quotient and remainder.
            let scaled = j * points;
            let (i, remainder) = (scaled / spans, scaled % spans);
            let alpha = remainder as f64 / spans as f64;
            match (i.checked_sub(1).and_then(|i| params.get(i)), params.get(i)) {
                (Some(&a), Some(&b)) => Ok((1.0 - alpha) * a + alpha * b),
                _ => Err(GeomError::InvalidArgument(
                    "too few data points for the control point count",
                )),
            }
        })
        .collect::<Result<Vec<f64>, GeomError>>()?;
    KnotVector::clamped_from_interior(degree, 0.0, 1.0, &interior)
}

/// The least-squares fit with `count` control points at known parameters;
/// `degree < count <= points.len()` and `params` valid for `points`.
fn fit<P: FitPoint>(
    points: &[P],
    params: &[f64],
    degree: usize,
    count: usize,
) -> Result<NurbsCurve<P::Homogeneous>, GeomError> {
    let total = points.len();
    if count == total {
        return interpolate_at(points, params, degree);
    }
    let (Some(&first), Some(&last)) = (points.first(), points.last()) else {
        return Err(GeomError::InvalidArgument("no data points"));
    };
    let knots = approximation_knots(params, degree, count)?;
    let end = count - 1;

    let mut control = vec![first];
    let free = count - 2;
    if free > 0 {
        let rows = total - 2;
        let mut a = DMatrix::zeros(rows, free);
        let mut rhs: Vec<P> = Vec::with_capacity(rows);
        for (row, (&q, &u)) in points.iter().zip(params).enumerate().skip(1).take(rows) {
            let span = knots.span(degree, u);
            let basis = basis_funs_raw(knots.as_slice(), span, u, degree);
            let mut r = q;
            for (j, &value) in basis.iter().take(degree + 1).enumerate() {
                let index = span - degree + j;
                if index == 0 {
                    r = r - first * value;
                } else if index == end {
                    r = r - last * value;
                } else {
                    set(&mut a, row - 1, index - 1, value);
                }
            }
            rhs.push(r);
        }
        let solution = solve_least_squares(a, &points_to_matrix(&rhs))?;
        control.extend(matrix_to_points::<P>(&solution));
    }
    control.push(last);
    NurbsCurve::from_points(degree, knots, &control, None)
}

#[cfg(test)]
mod tests {
    use bevel_math::{DVec2, DVec3};

    use super::*;
    use crate::fit::interpolate;

    const METHODS: [Parametrization; 3] = [
        Parametrization::Uniform,
        Parametrization::ChordLength,
        Parametrization::Centripetal,
    ];

    /// A wavy planar curve sampled at `count` points.
    fn wave(count: usize) -> Vec<DVec2> {
        (0..count)
            .map(|k| {
                let t = k as f64 / (count - 1) as f64;
                DVec2::new(10.0 * t, (6.0 * t).sin() + 0.3 * (17.0 * t).cos())
            })
            .collect()
    }

    #[test]
    fn keeps_the_end_points_and_the_requested_size() {
        let pts = wave(30);
        for method in METHODS {
            for degree in 1..=4 {
                for count in [degree + 1, 8, 15, 29] {
                    let c = approximate(&pts, degree, count, method).unwrap();
                    assert_eq!(c.control_points().len(), count);
                    assert_eq!(c.degree(), degree);
                    assert!((c.point(0.0) - pts[0]).length() < 1e-13);
                    assert!((c.point(1.0) - pts[29]).length() < 1e-13);
                }
            }
        }
    }

    #[test]
    fn full_control_point_count_interpolates() {
        let pts = wave(12);
        let u = parameters(&pts, Parametrization::ChordLength).unwrap();
        let c = approximate(&pts, 3, 12, Parametrization::ChordLength).unwrap();
        let i = interpolate(&pts, 3, Parametrization::ChordLength).unwrap();
        for (&uk, &q) in u.iter().zip(&pts) {
            assert!((c.point(uk) - q).length() < 1e-12);
        }
        assert_eq!(c, i);
    }

    #[test]
    fn a_cubic_polynomial_is_fitted_exactly_by_a_bezier_curve() {
        let cubic = |t: f64| {
            DVec3::new(
                2.0 - t + 4.0 * t * t * t,
                t * t - 3.0 * t * t * t,
                1.0 + 5.0 * t,
            )
        };
        let pts: Vec<DVec3> = (0..=40).map(|k| cubic(f64::from(k) / 40.0)).collect();
        let c = approximate(&pts, 3, 4, Parametrization::Uniform).unwrap();
        for i in 0..=50 {
            let t = f64::from(i) / 50.0;
            assert!((c.point(t) - cubic(t)).length() < 1e-10);
        }
    }

    #[test]
    fn a_line_is_fitted_by_the_minimum_curve() {
        let pts: Vec<DVec2> = (0..9)
            .map(|k| DVec2::new(1.0, 2.0) + DVec2::new(0.5, -1.0) * f64::from(k))
            .collect();
        let c = approximate(&pts, 1, 2, Parametrization::ChordLength).unwrap();
        assert_eq!(c.control_points().len(), 2);
        let u = parameters(&pts, Parametrization::ChordLength).unwrap();
        for (&uk, &q) in u.iter().zip(&pts) {
            assert!((c.point(uk) - q).length() < 1e-12);
        }
    }

    #[test]
    fn tolerance_search_meets_the_tolerance() {
        let pts = wave(60);
        let u = parameters(&pts, Parametrization::ChordLength).unwrap();
        for tol in [1.0, 0.1, 0.01, 1e-3, 1e-6] {
            let c = approximate_to_tolerance(&pts, 3, tol, Parametrization::ChordLength).unwrap();
            for (&uk, &q) in u.iter().zip(&pts) {
                assert!((c.point(uk) - q).length() <= tol);
            }
        }
        // A loose tolerance needs only the Bezier curve.
        let c = approximate_to_tolerance(&pts, 3, 100.0, Parametrization::ChordLength).unwrap();
        assert_eq!(c.control_points().len(), 4);
        // A tight one needs more control points than a loose one.
        let tight = approximate_to_tolerance(&pts, 3, 1e-3, Parametrization::ChordLength).unwrap();
        assert!(tight.control_points().len() > 4);
        assert_eq!(
            tight,
            approximate_to_tolerance(&pts, 3, 1e-3, Parametrization::ChordLength).unwrap()
        );
    }

    #[test]
    fn unreachable_tolerance_reports_the_interpolation_error() {
        let pts = wave(20);
        match approximate_to_tolerance(&pts, 3, 1e-300, Parametrization::ChordLength) {
            Err(GeomError::ToleranceNotMet { error }) => assert!(error < 1e-10),
            other => panic!("expected ToleranceNotMet, got {other:?}"),
        }
    }

    #[test]
    fn invalid_inputs_are_reported() {
        let pts = wave(10);
        let m = Parametrization::ChordLength;
        assert!(matches!(
            approximate(&pts, 0, 5, m),
            Err(GeomError::InvalidDegree { degree: 0 })
        ));
        assert!(matches!(
            approximate(&pts, 3, 3, m),
            Err(GeomError::ControlPointCount {
                expected: 4,
                found: 3
            })
        ));
        assert!(matches!(
            approximate(&pts, 3, 11, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            approximate(&pts[..3], 3, 4, m),
            Err(GeomError::InvalidArgument(_))
        ));
        let mut nan = pts.clone();
        nan[4].x = f64::NAN;
        assert_eq!(approximate(&nan, 3, 5, m), Err(GeomError::NonFinite));
        let mut dup = pts.clone();
        dup[5] = dup[4];
        assert!(matches!(
            approximate(&dup, 3, 5, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            approximate_to_tolerance(&pts, 3, 0.0, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            approximate_to_tolerance(&pts, 3, -1.0, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert_eq!(
            approximate_to_tolerance(&pts, 3, f64::NAN, m),
            Err(GeomError::NonFinite)
        );
    }
}
