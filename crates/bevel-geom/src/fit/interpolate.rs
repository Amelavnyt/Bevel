// SPDX-License-Identifier: GPL-3.0-or-later

//! Global curve interpolation (The NURBS Book, section 9.2).
//!
//! Given points `Q_0, ..., Q_n`, parameters `u*_0 < ... < u*_n` and a clamped
//! knot vector, the control points `P_j` of the interpolating curve solve
//! `sum_j N_{j,p}(u*_k) P_j = Q_k` for `k = 0, ..., n` (section 9.2.1). With the
//! averaged knots of eq. 9.8 the matrix `[N_{j,p}(u*_k)]` is non-singular and
//! well conditioned (A9.1).
//!
//! # End derivatives
//!
//! [`interpolate_with_tangents`] also prescribes the first derivatives of
//! the curve at its two ends (section 9.2.2). The curve then has `n + 3`
//! control points, two more than points, and the system gains the equations
//! `C'(0) = D_0` and `C'(1) = D_n`. For a clamped curve the first is
//! `p (P_1 - P_0) / u_{p+1} = D_0` and the last
//! `p (P_{n+2} - P_{n+1}) / (1 - u_{m-p-1}) = D_n` (section 9.2.2); this code
//! forms both rows from the basis function derivatives (A2.3) instead. The
//! knots are those of the derivative-constrained variant of eq. 9.8,
//! `u_{j+p+1} = (u*_j + ... + u*_{j+p-1}) / p` for `j = 0, ..., n - p + 1`,
//! which averages every window of `p` consecutive parameters.
//!
//! # Tangent magnitudes
//!
//! The tangent arguments are derivative vectors with respect to the curve
//! parameter, which runs over `[0, 1]`: the curve satisfies
//! `C'(0) = D_0` and `C'(1) = D_n` exactly, magnitude included. A unit
//! vector therefore means a nearly stationary start, because the curve
//! covers its whole length in one unit of parameter. To get a speed
//! comparable to that of the data, scale a direction by the length of the
//! polyline through the points, which is the speed of a curve parametrized
//! by chord length.

use nalgebra::DMatrix;

use super::solve::{matrix_to_points, points_to_matrix, set, solve_square};
use super::{FitPoint, Parametrization, averaging_knots, check_points, parameters};
use crate::basis::{basis_funs_raw, ders_basis_funs_raw};
use crate::knots::check_degree;
use crate::{GeomError, KnotVector, NurbsCurve};

/// Tangents shorter than this fraction of the data extent are treated as
/// zero.
const ZERO_TANGENT_TOLERANCE: f64 = 1e-12;

/// A non-rational curve of degree `degree` through `points`, with a
/// parameter assigned to each point by `method` (The NURBS Book, A9.1).
///
/// The curve is defined on `[0, 1]` and passes through `points[k]` at the
/// parameter [`parameters`] gives for it. It has one control point per data
/// point and the knot vector of [`averaging_knots`].
///
/// Needs at least `degree + 1` points. With exactly that many the result is
/// a Bezier curve. Degree 1 gives the polyline through the points.
///
/// # Errors
///
/// [`GeomError::InvalidDegree`] for a degree outside `1..=MAX_DEGREE`;
/// [`GeomError::InvalidArgument`] for fewer than `degree + 1` points or
/// coincident consecutive points; [`GeomError::NonFinite`] for NaN or
/// infinite coordinates; [`GeomError::NotConverged`] if the linear system is
/// singular to working precision (not expected for valid data).
pub fn interpolate<P: FitPoint>(
    points: &[P],
    degree: usize,
    method: Parametrization,
) -> Result<NurbsCurve<P::Homogeneous>, GeomError> {
    check_degree(degree)?;
    check_points(points, degree + 1)?;
    let params = parameters(points, method)?;
    interpolate_at(points, &params, degree)
}

/// Interpolation at given parameters, with averaged knots (A9.1).
///
/// `params` must be valid for `points` (one per point, strictly increasing).
pub(crate) fn interpolate_at<P: FitPoint>(
    points: &[P],
    params: &[f64],
    degree: usize,
) -> Result<NurbsCurve<P::Homogeneous>, GeomError> {
    if params.len() != points.len() {
        return Err(GeomError::InvalidArgument(
            "there must be one parameter per point",
        ));
    }
    let knots = averaging_knots(params, degree)?;
    let count = points.len();
    let mut a = DMatrix::zeros(count, count);
    for (row, &u) in params.iter().enumerate() {
        value_row(&mut a, row, &knots, degree, u);
    }
    let control = solve_square(a, &points_to_matrix(points))?;
    let control: Vec<P> = matrix_to_points(&control);
    NurbsCurve::from_points(degree, knots, &control, None)
}

/// A non-rational curve of degree `degree` through `points` whose first
/// derivative at the start and the end of its domain `[0, 1]` is
/// `start_tangent` and `end_tangent` (The NURBS Book, section 9.2.2).
///
/// The parameters of the points come from `method`. The curve has one
/// control point per data point plus two. The tangents are derivatives with
/// respect to the curve parameter and are matched exactly, magnitude
/// included; the module documentation says how to choose their length.
///
/// Needs `degree >= 2` and at least `max(2, degree - 1)` points. With
/// exactly `degree - 1` points the curve is a single Bezier segment (two
/// points and degree 3 give a cubic Hermite segment).
///
/// # Errors
///
/// [`GeomError::InvalidDegree`] for a degree below 2 or above `MAX_DEGREE`;
/// [`GeomError::InvalidArgument`] for too few points, coincident consecutive
/// points or a tangent that is zero (shorter than 1e-12 of the extent of the
/// data); [`GeomError::NonFinite`] for NaN or infinite coordinates;
/// [`GeomError::NotConverged`] if the linear system is singular to working
/// precision.
pub fn interpolate_with_tangents<P: FitPoint>(
    points: &[P],
    start_tangent: P,
    end_tangent: P,
    degree: usize,
    method: Parametrization,
) -> Result<NurbsCurve<P::Homogeneous>, GeomError> {
    check_degree(degree)?;
    if degree < 2 {
        return Err(GeomError::InvalidDegree { degree });
    }
    check_points(points, degree.saturating_sub(1).max(2))?;
    if !start_tangent.is_finite() || !end_tangent.is_finite() {
        return Err(GeomError::NonFinite);
    }
    let params = parameters(points, method)?;
    let extent = points.iter().map(|&p| p.length()).fold(1.0, f64::max);
    if start_tangent.length() <= ZERO_TANGENT_TOLERANCE * extent
        || end_tangent.length() <= ZERO_TANGENT_TOLERANCE * extent
    {
        return Err(GeomError::InvalidArgument("end tangents must be non-zero"));
    }

    // One interior knot per window of `degree` consecutive parameters.
    let interior: Vec<f64> = params
        .windows(degree)
        .map(|w| w.iter().sum::<f64>() / degree as f64)
        .collect();
    let knots = KnotVector::clamped_from_interior(degree, 0.0, 1.0, &interior)?;

    // Rows: value at u_0, derivative at 0, values at the interior
    // parameters, derivative at 1, value at u_n.
    let size = points.len() + 2;
    let last = points.len() - 1;
    let mut a = DMatrix::zeros(size, size);
    let mut rhs: Vec<P> = Vec::with_capacity(size);
    value_row(&mut a, 0, &knots, degree, 0.0);
    rhs.push(points[0]);
    derivative_row(&mut a, 1, &knots, degree, 0.0);
    rhs.push(start_tangent);
    for k in 1..last {
        value_row(&mut a, k + 1, &knots, degree, params[k]);
        rhs.push(points[k]);
    }
    derivative_row(&mut a, size - 2, &knots, degree, 1.0);
    rhs.push(end_tangent);
    value_row(&mut a, size - 1, &knots, degree, 1.0);
    rhs.push(points[last]);

    let control = solve_square(a, &points_to_matrix(&rhs))?;
    let control: Vec<P> = matrix_to_points(&control);
    NurbsCurve::from_points(degree, knots, &control, None)
}

/// Fills `row` of `a` with the basis functions `N_{j,p}(u)`.
fn value_row(a: &mut DMatrix<f64>, row: usize, knots: &KnotVector, degree: usize, u: f64) {
    let span = knots.span(degree, u);
    let basis = basis_funs_raw(knots.as_slice(), span, u, degree);
    for (j, &value) in basis.iter().take(degree + 1).enumerate() {
        set(a, row, span - degree + j, value);
    }
}

/// Fills `row` of `a` with the derivatives `N'_{j,p}(u)` (A2.3).
fn derivative_row(a: &mut DMatrix<f64>, row: usize, knots: &KnotVector, degree: usize, u: f64) {
    let span = knots.span(degree, u);
    let ders = ders_basis_funs_raw(knots.as_slice(), span, u, degree, 1);
    for (j, &value) in ders[1].iter().take(degree + 1).enumerate() {
        set(a, row, span - degree + j, value);
    }
}

#[cfg(test)]
mod tests {
    use bevel_math::{DVec2, DVec3};

    use super::*;

    const METHODS: [Parametrization; 3] = [
        Parametrization::Uniform,
        Parametrization::ChordLength,
        Parametrization::Centripetal,
    ];

    fn example_points() -> Vec<DVec2> {
        // The NURBS Book, example 9.1.
        vec![
            DVec2::new(0.0, 0.0),
            DVec2::new(3.0, 4.0),
            DVec2::new(-1.0, 4.0),
            DVec2::new(-4.0, 0.0),
            DVec2::new(-4.0, -3.0),
        ]
    }

    #[test]
    fn passes_through_the_points_of_the_book_example() {
        let pts = example_points();
        for method in METHODS {
            for degree in 1..=4 {
                let c = interpolate(&pts, degree, method).unwrap();
                assert_eq!(c.degree(), degree);
                assert_eq!(c.control_points().len(), pts.len());
                let u = parameters(&pts, method).unwrap();
                for (&uk, &q) in u.iter().zip(&pts) {
                    assert!((c.point(uk) - q).length() < 1e-12, "{method:?} {degree}");
                }
            }
        }
    }

    #[test]
    fn reproduces_a_cubic_polynomial_curve() {
        let cubic = |t: f64| {
            DVec3::new(
                1.0 + 2.0 * t - 3.0 * t * t + 0.5 * t * t * t,
                -1.0 + t * t * t,
                4.0 - t + 2.0 * t * t,
            )
        };
        for count in [4_u8, 5, 9, 17] {
            let pts: Vec<DVec3> = (0..count)
                .map(|k| cubic(f64::from(k) / f64::from(count - 1)))
                .collect();
            let c = interpolate(&pts, 3, Parametrization::Uniform).unwrap();
            for i in 0..=40 {
                let t = f64::from(i) / 40.0;
                assert!((c.point(t) - cubic(t)).length() < 1e-10, "{count} at {t}");
            }
        }
    }

    #[test]
    fn degree_one_is_the_polyline() {
        let pts = example_points();
        let c = interpolate(&pts, 1, Parametrization::ChordLength).unwrap();
        for (a, b) in c.control_points().iter().zip(&pts) {
            assert!((*a - *b).length() < 1e-14);
        }
        // Halfway between the first two points in chord length terms.
        let u = parameters(&pts, Parametrization::ChordLength).unwrap();
        let mid = c.point(0.5 * u[1]);
        assert!((mid - 0.5 * pts[1]).length() < 1e-12);
    }

    #[test]
    fn tangent_version_matches_end_derivatives() {
        let pts = example_points();
        let d0 = DVec2::new(10.0, 2.0);
        let dn = DVec2::new(-1.0, -7.0);
        for method in METHODS {
            for degree in 2..=5 {
                let c = interpolate_with_tangents(&pts, d0, dn, degree, method).unwrap();
                assert_eq!(c.control_points().len(), pts.len() + 2);
                let u = parameters(&pts, method).unwrap();
                for (&uk, &q) in u.iter().zip(&pts) {
                    assert!((c.point(uk) - q).length() < 1e-10, "{method:?} {degree}");
                }
                assert!((c.derivatives(0.0, 1)[1] - d0).length() < 1e-9);
                assert!((c.derivatives(1.0, 1)[1] - dn).length() < 1e-9);
            }
        }
    }

    #[test]
    fn two_points_and_two_tangents_give_the_cubic_hermite_segment() {
        let (q0, q1) = (DVec3::new(0.0, 0.0, 1.0), DVec3::new(4.0, 2.0, -1.0));
        let (d0, d1) = (DVec3::new(3.0, 0.0, 0.0), DVec3::new(0.0, 6.0, 3.0));
        let c =
            interpolate_with_tangents(&[q0, q1], d0, d1, 3, Parametrization::ChordLength).unwrap();
        let expected = [q0, q0 + d0 / 3.0, q1 - d1 / 3.0, q1];
        for (a, b) in c.control_points().iter().zip(expected) {
            assert!((*a - b).length() < 1e-13);
        }
    }

    #[test]
    fn invalid_inputs_are_reported() {
        let pts = example_points();
        let m = Parametrization::ChordLength;
        assert!(matches!(
            interpolate(&pts, 0, m),
            Err(GeomError::InvalidDegree { degree: 0 })
        ));
        assert!(matches!(
            interpolate(&pts, 26, m),
            Err(GeomError::InvalidDegree { degree: 26 })
        ));
        assert!(matches!(
            interpolate(&pts, 5, m),
            Err(GeomError::InvalidArgument(_))
        ));
        let mut dup = pts.clone();
        dup[2] = dup[1];
        assert!(matches!(
            interpolate(&dup, 3, m),
            Err(GeomError::InvalidArgument(_))
        ));
        let mut nan = pts.clone();
        nan[2].y = f64::INFINITY;
        assert_eq!(interpolate(&nan, 3, m), Err(GeomError::NonFinite));

        let t = DVec2::new(1.0, 0.0);
        assert!(matches!(
            interpolate_with_tangents(&pts, t, t, 1, m),
            Err(GeomError::InvalidDegree { degree: 1 })
        ));
        assert!(matches!(
            interpolate_with_tangents(&pts[..2], t, t, 4, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            interpolate_with_tangents(&pts, DVec2::ZERO, t, 3, m),
            Err(GeomError::InvalidArgument(_))
        ));
        assert_eq!(
            interpolate_with_tangents(&pts, DVec2::new(f64::NAN, 0.0), t, 3, m),
            Err(GeomError::NonFinite)
        );
    }
}
