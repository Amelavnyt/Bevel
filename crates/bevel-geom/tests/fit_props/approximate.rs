// SPDX-License-Identifier: GPL-3.0-or-later

//! Least-squares approximation and the tolerance search.

use bevel_geom::NurbsCurve;
use bevel_geom::fit::{
    FitPoint, Parametrization, approximate, approximate_to_tolerance, parameters,
};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{config, degree_and_points2, degree_and_points3, method, size};

/// The sum of squared distances between the points and the curve at the
/// points' parameters.
fn residual<P: FitPoint>(c: &NurbsCurve<P::Homogeneous>, pts: &[P], u: &[f64]) -> f64 {
    pts.iter()
        .zip(u)
        .map(|(&q, &uk)| (c.point(uk) - q).length().powi(2))
        .sum()
}

/// The fit keeps both end points whatever the control point count.
fn check_end_points<P: FitPoint>(
    pts: &[P],
    degree: usize,
    pick: usize,
    method: Parametrization,
) -> Result<(), TestCaseError> {
    // `pick` in 0..=999 spreads the count over degree + 1 ..= pts.len().
    let count = degree + 1 + pick * (pts.len() - degree - 1) / 999;
    let c = approximate(pts, degree, count, method).unwrap();
    prop_assert_eq!(c.control_points().len(), count);
    prop_assert_eq!(c.degree(), degree);
    let tol = 1e-12 * size(pts);
    let (a, b) = c.domain();
    let first = pts[0];
    let last = pts[pts.len() - 1];
    prop_assert!((c.point(a) - first).length() <= tol);
    prop_assert!((c.point(b) - last).length() <= tol);
    Ok(())
}

/// The fit improves on the Bezier curve and reaches zero at the full count.
///
/// The least-squares fit is optimal in its spline space. Every space
/// contains the Bezier curve with the same end points (a clamped knot vector
/// can represent any polynomial of its degree), so no count does worse than
/// `degree + 1` control points. The knot sets of different counts are not
/// nested, so the residual is *not* monotone in the count in general; it
/// is only known to be zero at the end, where the fit interpolates.
fn check_residual<P: FitPoint>(
    pts: &[P],
    degree: usize,
    method: Parametrization,
) -> Result<(), TestCaseError> {
    let u = parameters(pts, method).unwrap();
    let scale = size(pts).powi(2);
    let bezier = residual(
        &approximate(pts, degree, degree + 1, method).unwrap(),
        pts,
        &u,
    );
    for count in degree + 2..=pts.len() {
        let c = approximate(pts, degree, count, method).unwrap();
        let r = residual(&c, pts, &u);
        prop_assert!(
            r <= bezier + 1e-9 * scale,
            "residual {r} at {count} control points exceeds the Bezier fit's {bezier}"
        );
        if count == pts.len() {
            prop_assert!(
                r <= 1e-18 * scale,
                "full count residual {r} (scale {scale})"
            );
        }
    }
    Ok(())
}

/// The tolerance search meets the tolerance on every input point.
fn check_tolerance<P: FitPoint>(
    pts: &[P],
    degree: usize,
    share: f64,
    method: Parametrization,
) -> Result<(), TestCaseError> {
    // Tolerances from 100% down to 1e-4 of the size of the data.
    let tol = size(pts) * 10f64.powf(-4.0 * share);
    let c = approximate_to_tolerance(pts, degree, tol, method).unwrap();
    let u = parameters(pts, method).unwrap();
    for (k, (&uk, &q)) in u.iter().zip(pts).enumerate() {
        let d = (c.point(uk) - q).length();
        prop_assert!(d <= tol, "point {k} is {d} away (tolerance {tol})");
    }
    let (n, p) = (c.control_points().len(), c.degree());
    prop_assert!(n > p && n <= pts.len());
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn approximation_keeps_end_points_2d(
        (degree, pts) in degree_and_points2(1..=5),
        pick in 0usize..=999,
        m in method(),
    ) {
        check_end_points(&pts, degree, pick, m)?;
    }

    #[test]
    fn approximation_keeps_end_points_3d(
        (degree, pts) in degree_and_points3(1..=5),
        pick in 0usize..=999,
        m in method(),
    ) {
        check_end_points(&pts, degree, pick, m)?;
    }

    #[test]
    fn residual_beats_the_bezier_fit_and_vanishes_at_full_count_2d(
        (degree, pts) in degree_and_points2(1..=5),
        m in method(),
    ) {
        check_residual(&pts, degree, m)?;
    }

    #[test]
    fn residual_beats_the_bezier_fit_and_vanishes_at_full_count_3d(
        (degree, pts) in degree_and_points3(1..=5),
        m in method(),
    ) {
        check_residual(&pts, degree, m)?;
    }

    #[test]
    fn tolerance_search_meets_tolerance_2d(
        (degree, pts) in degree_and_points2(1..=5),
        share in 0.0f64..=1.0,
        m in method(),
    ) {
        check_tolerance(&pts, degree, share, m)?;
    }

    #[test]
    fn tolerance_search_meets_tolerance_3d(
        (degree, pts) in degree_and_points3(1..=5),
        share in 0.0f64..=1.0,
        m in method(),
    ) {
        check_tolerance(&pts, degree, share, m)?;
    }
}
