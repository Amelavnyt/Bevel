// SPDX-License-Identifier: GPL-3.0-or-later

//! Evaluation and derivatives.

use bevel_geom::{Homogeneous, NurbsCurve};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{any_curve2, any_curve3, config, dist, params, size, span_params};

/// `derivatives(u, k)` against central differences of `derivatives(u, k - 1)`
/// for k = 1, 2, away from knots (where derivatives may jump).
fn check_finite_differences<H: Homogeneous>(c: &NurbsCurve<H>) -> Result<(), TestCaseError> {
    let samples = span_params(c, 56, 0.05);
    prop_assert!(samples.len() >= 50);
    for (u, s0, s1) in samples {
        let h = 1e-5 * (s1 - s0);
        let d = c.derivatives(u, 2);
        let dp = c.derivatives(u + h, 1);
        let dm = c.derivatives(u - h, 1);
        for k in 1..=2 {
            let fd = (dp[k - 1] - dm[k - 1]) * (0.5 / h);
            // The error is relative to the typical size of a k-th derivative
            // on this span.
            let len = s1 - s0;
            let scale = H::point_length(d[k])
                + H::point_length(d[k - 1]) / len
                + size(c) / len.powi(i32::try_from(k).unwrap_or(2));
            let err = dist::<H>(fd, d[k]);
            prop_assert!(
                err <= 1e-5 * scale,
                "derivative {k} at {u}: exact {:?}, difference quotient {:?}",
                d[k],
                fd
            );
        }
    }
    Ok(())
}

/// The zeroth derivative is the point, everywhere including the domain ends.
fn check_point_is_zeroth_derivative<H: Homogeneous>(
    c: &NurbsCurve<H>,
) -> Result<(), TestCaseError> {
    let (a, b) = c.domain();
    for u in params(a, b, 64) {
        let d = c.derivatives(u, 3);
        prop_assert!(dist::<H>(d[0], c.point(u)) <= 1e-12 * size(c));
    }
    // Clamped curves interpolate their end control points.
    let pts = c.control_points();
    prop_assert!(dist::<H>(c.point(a), pts[0]) <= 1e-12 * size(c));
    prop_assert!(dist::<H>(c.point(b), pts[pts.len() - 1]) <= 1e-12 * size(c));
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn derivatives_match_finite_differences_3d(c in any_curve3()) {
        check_finite_differences(&c)?;
    }

    #[test]
    fn derivatives_match_finite_differences_2d(c in any_curve2()) {
        check_finite_differences(&c)?;
    }

    #[test]
    fn point_is_zeroth_derivative_3d(c in any_curve3()) {
        check_point_is_zeroth_derivative(&c)?;
    }

    #[test]
    fn point_is_zeroth_derivative_2d(c in any_curve2()) {
        check_point_is_zeroth_derivative(&c)?;
    }

    #[test]
    fn control_bounds_contain_the_curve(c in any_curve3()) {
        let bounds = c.control_bounds();
        let (a, b) = c.domain();
        let eps = 1e-9 * size(&c);
        for u in params(a, b, 64) {
            let p = c.point(u);
            prop_assert!(p.cmpge(bounds.min - eps).all() && p.cmple(bounds.max + eps).all());
        }
    }
}
