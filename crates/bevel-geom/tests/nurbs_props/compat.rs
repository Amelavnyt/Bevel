// SPDX-License-Identifier: GPL-3.0-or-later

//! Making curves compatible keeps each curve's geometry, and ruled
//! surfaces interpolate their curves.

use bevel_geom::{NurbsCurve3, NurbsSurface, make_compatible};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{any_curve3, config, dist, params, size, unclamped_curve3};

/// Checks that `out`, on the domain `(a, b)`, is `curve` reparametrised
/// affinely onto that domain.
fn check_same(
    curve: &NurbsCurve3,
    out: &NurbsCurve3,
    (a, b): (f64, f64),
) -> Result<(), TestCaseError> {
    let (c, d) = curve.domain();
    let tol = 1e-9 * size(curve);
    for t in params(c, d, 64) {
        let s = a + (t - c) * (b - a) / (d - c);
        let e = dist::<bevel_math::DVec4>(curve.point(t), out.point(s));
        prop_assert!(e <= tol, "compatible curve differs by {e} at {t}");
    }
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn make_compatible_keeps_every_curve(
        a in any_curve3(),
        b in any_curve3(),
        c in unclamped_curve3(),
    ) {
        let curves = [a, b, c];
        let out = make_compatible(&curves).unwrap();
        let domain = curves[0].domain();
        let degree = curves.iter().map(NurbsCurve3::degree).max().unwrap();
        for (curve, o) in curves.iter().zip(&out) {
            prop_assert_eq!(o.degree(), degree);
            prop_assert_eq!(o.knots(), out[0].knots());
            prop_assert_eq!(o.domain(), domain);
            check_same(curve, o, domain)?;
        }
    }

    #[test]
    fn ruled_surface_interpolates_its_curves(a in any_curve3(), b in any_curve3()) {
        let s = NurbsSurface::ruled(&a, &b).unwrap();
        let ((ua, ub), (va, vb)) = s.domain();
        prop_assert_eq!((ua, ub), a.domain());
        prop_assert_eq!((va, vb), (0.0, 1.0));
        check_same(&a, &s.isocurve_v(0.0).unwrap(), (ua, ub))?;
        check_same(&b, &s.isocurve_v(1.0).unwrap(), (ua, ub))?;
    }
}
