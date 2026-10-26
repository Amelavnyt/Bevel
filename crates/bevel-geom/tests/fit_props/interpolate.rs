// SPDX-License-Identifier: GPL-3.0-or-later

//! Interpolation, with and without end tangents.

use bevel_geom::fit::{
    FitPoint, Parametrization, interpolate, interpolate_with_tangents, parameters,
};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{
    config, degree_and_points2, degree_and_points3, method, size, tangent2, tangent3,
};

/// The curve passes through every point at its parameter, and has one
/// control point per point on the domain `[0, 1]`.
fn check_interpolation<P: FitPoint>(
    pts: &[P],
    degree: usize,
    method: Parametrization,
) -> Result<(), TestCaseError> {
    let c = interpolate(pts, degree, method).unwrap();
    prop_assert_eq!(c.degree(), degree);
    prop_assert_eq!(c.control_points().len(), pts.len());
    prop_assert!(!c.is_rational());
    let (a, b) = c.domain();
    prop_assert!((a, b) == (0.0, 1.0));
    let u = parameters(pts, method).unwrap();
    let tol = 1e-9 * size(pts);
    for (k, (&uk, &q)) in u.iter().zip(pts).enumerate() {
        let d = (c.point(uk) - q).length();
        prop_assert!(d <= tol, "point {k} missed by {d} (tol {tol})");
    }
    Ok(())
}

/// As [`check_interpolation`] with end derivatives, which must also match
/// in direction and magnitude.
fn check_tangents<P: FitPoint>(
    pts: &[P],
    d0: P,
    dn: P,
    degree: usize,
    method: Parametrization,
) -> Result<(), TestCaseError> {
    let c = interpolate_with_tangents(pts, d0, dn, degree, method).unwrap();
    prop_assert_eq!(c.degree(), degree);
    prop_assert_eq!(c.control_points().len(), pts.len() + 2);
    let u = parameters(pts, method).unwrap();
    let tol = 1e-9 * size(pts);
    for (k, (&uk, &q)) in u.iter().zip(pts).enumerate() {
        let d = (c.point(uk) - q).length();
        prop_assert!(d <= tol, "point {k} missed by {d} (tol {tol})");
    }
    for (u, want) in [(0.0, d0), (1.0, dn)] {
        let got = c.derivatives(u, 1)[1];
        let err = (got - want).length();
        let dtol = 1e-8 * (size(pts) + want.length());
        prop_assert!(err <= dtol, "derivative at {u}: {got:?}, wanted {want:?}");
        // The direction agrees as well: the cosine of the angle is 1.
        let cos = got.dot(want) / (got.length() * want.length());
        prop_assert!(cos > 1.0 - 1e-9, "directions differ at {u} (cos {cos})");
    }
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn interpolation_passes_through_points_2d(
        (degree, pts) in degree_and_points2(1..=5),
        m in method(),
    ) {
        check_interpolation(&pts, degree, m)?;
    }

    #[test]
    fn interpolation_passes_through_points_3d(
        (degree, pts) in degree_and_points3(1..=5),
        m in method(),
    ) {
        check_interpolation(&pts, degree, m)?;
    }

    #[test]
    fn tangent_interpolation_matches_points_and_end_derivatives_2d(
        (degree, pts) in degree_and_points2(2..=5),
        d0 in tangent2(),
        dn in tangent2(),
        m in method(),
    ) {
        check_tangents(&pts, d0, dn, degree, m)?;
    }

    #[test]
    fn tangent_interpolation_matches_points_and_end_derivatives_3d(
        (degree, pts) in degree_and_points3(2..=5),
        d0 in tangent3(),
        dn in tangent3(),
        m in method(),
    ) {
        check_tangents(&pts, d0, dn, degree, m)?;
    }
}
