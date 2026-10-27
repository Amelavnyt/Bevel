// SPDX-License-Identifier: GPL-3.0-or-later

//! Surface evaluation, derivatives, normals and curvatures.

use bevel_geom::NurbsSurface;
use bevel_math::DVec3;
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{
    any_surface, assert_same_surface, config, grid, knot_span_params, surface_size,
};

/// First and second partial derivatives against central differences of the
/// first ones, at parameters inside the knot-span patches (derivatives may
/// jump across knots).
fn check_finite_differences(s: &NurbsSurface) -> Result<(), TestCaseError> {
    let (du, dv) = s.domain();
    let us = knot_span_params(s.knots_u(), du, 12, 0.05);
    let vs = knot_span_params(s.knots_v(), dv, 12, 0.05);
    let size = surface_size(s);
    for &(u, u0, u1) in &us {
        for &(v, v0, v1) in &vs {
            let (lu, lv) = (u1 - u0, v1 - v0);
            let (hu, hv) = (1e-5 * lu, 1e-5 * lv);
            let d = s.derivatives(u, v, 2);
            let dup = s.derivatives(u + hu, v, 1);
            let dum = s.derivatives(u - hu, v, 1);
            let dvp = s.derivatives(u, v + hv, 1);
            let dvm = s.derivatives(u, v - hv, 1);
            // (value, difference quotient, scale) for S_u, S_v, S_uu, S_uv,
            // S_vv; the scale is the typical size of that derivative here.
            let cases = [
                (
                    d[1][0],
                    (dup[0][0] - dum[0][0]) / (2.0 * hu),
                    d[1][0].length() + size / lu,
                ),
                (
                    d[0][1],
                    (dvp[0][0] - dvm[0][0]) / (2.0 * hv),
                    d[0][1].length() + size / lv,
                ),
                (
                    d[2][0],
                    (dup[1][0] - dum[1][0]) / (2.0 * hu),
                    d[2][0].length() + d[1][0].length() / lu + size / (lu * lu),
                ),
                (
                    d[1][1],
                    (dvp[1][0] - dvm[1][0]) / (2.0 * hv),
                    d[1][1].length() + d[1][0].length() / lv + size / (lu * lv),
                ),
                (
                    d[0][2],
                    (dvp[0][1] - dvm[0][1]) / (2.0 * hv),
                    d[0][2].length() + d[0][1].length() / lv + size / (lv * lv),
                ),
            ];
            for (i, (exact, fd, scale)) in cases.into_iter().enumerate() {
                let err = (exact - fd).length();
                prop_assert!(
                    err <= 1e-5 * scale,
                    "derivative {i} at ({u}, {v}): exact {exact:?}, difference quotient {fd:?}"
                );
            }
        }
    }
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn surface_derivatives_match_finite_differences(s in any_surface()) {
        check_finite_differences(&s)?;
    }

    #[test]
    fn surface_point_is_zeroth_derivative(s in any_surface()) {
        let size = surface_size(&s);
        for (u, v) in grid(&s, 12) {
            let d = s.derivatives(u, v, 3);
            prop_assert_eq!(d.len(), 4);
            prop_assert!((d[0][0] - s.point(u, v)).length() <= 1e-12 * size);
        }
        // Clamped surfaces interpolate their corner control points.
        let ((ua, ub), (va, vb)) = s.domain();
        let pts = s.control_points();
        let (nu, nv) = (s.count_u(), s.count_v());
        for (u, v, i, j) in [(ua, va, 0, 0), (ub, va, nu - 1, 0), (ua, vb, 0, nv - 1), (ub, vb, nu - 1, nv - 1)] {
            prop_assert!((s.point(u, v) - pts[i * nv + j]).length() <= 1e-12 * size);
        }
    }

    #[test]
    fn surface_from_points_matches_new(s in any_surface()) {
        let t = NurbsSurface::from_points(
            s.degree_u(),
            s.degree_v(),
            s.knots_u().clone(),
            s.knots_v().clone(),
            &s.control_points(),
            Some(&s.weights()),
        )
        .unwrap();
        prop_assert_eq!(t.is_rational(), s.is_rational());
        assert_same_surface(&s, &t, |u, v| (u, v))?;
    }

    #[test]
    fn surface_normal_is_unit_and_orthogonal(s in any_surface()) {
        let mut found = 0;
        for (u, v) in grid(&s, 12) {
            let Some(n) = s.normal(u, v) else { continue };
            found += 1;
            let d = s.derivatives(u, v, 1);
            prop_assert!((n.length() - 1.0).abs() <= 1e-12);
            prop_assert!(n.dot(d[1][0]).abs() <= 1e-10 * d[1][0].length());
            prop_assert!(n.dot(d[0][1]).abs() <= 1e-10 * d[0][1].length());
            prop_assert!(n.dot(d[1][0].cross(d[0][1])) > 0.0);
        }
        prop_assert!(found > 0);
    }

    #[test]
    fn principal_curvatures_bound_normal_curvatures(s in any_surface()) {
        for (u, v) in grid(&s, 12) {
            let Some(c) = s.principal_curvatures(u, v) else { continue };
            let n = s.normal(u, v).unwrap();
            prop_assert!(c.k1 >= c.k2);
            prop_assert!((c.dir1.length() - 1.0).abs() <= 1e-12);
            prop_assert!((c.dir2.length() - 1.0).abs() <= 1e-12);
            prop_assert!(c.dir1.dot(n).abs() <= 1e-12 && c.dir1.dot(c.dir2).abs() <= 1e-12);
            // Normal curvatures along the parameter lines lie between the
            // principal ones (Euler's theorem).
            let d = s.derivatives(u, v, 2);
            let scale = c.k1.abs().max(c.k2.abs()) + 1e-300;
            for (first, second) in [(d[1][0], d[2][0]), (d[0][1], d[0][2])] {
                let kn = second.dot(n) / first.length_squared();
                prop_assert!(
                    kn <= c.k1 + 1e-7 * scale && kn >= c.k2 - 1e-7 * scale,
                    "normal curvature {kn} outside [{}, {}]",
                    c.k2,
                    c.k1
                );
            }
        }
    }

    #[test]
    fn surface_control_bounds_contain_the_surface(s in any_surface()) {
        let b = s.control_bounds();
        let eps = 1e-9 * surface_size(&s);
        for (u, v) in grid(&s, 12) {
            let p = s.point(u, v);
            prop_assert!(p.cmpge(b.min - DVec3::splat(eps)).all());
            prop_assert!(p.cmple(b.max + DVec3::splat(eps)).all());
        }
    }
}
