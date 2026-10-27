// SPDX-License-Identifier: GPL-3.0-or-later

//! Surface isocurves, Bezier decomposition, splitting, reversal, swapping
//! and transforms keep the geometry.

use bevel_geom::NurbsSurface;
use bevel_math::{DMat4, DQuat, DVec3};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{any_surface, assert_same_surface, config, grid, params, surface_size};

fn check_isocurves(s: &NurbsSurface) -> Result<(), TestCaseError> {
    let tol = 1e-9 * surface_size(s);
    let ((ua, ub), (va, vb)) = s.domain();
    for u in params(ua, ub, 12) {
        let c = s.isocurve_u(u).unwrap();
        prop_assert_eq!(c.domain(), (va, vb));
        for v in params(va, vb, 12) {
            let d = (c.point(v) - s.point(u, v)).length();
            prop_assert!(d <= tol, "isocurve_u({u}) differs by {d} at v = {v}");
        }
    }
    for v in params(va, vb, 12) {
        let c = s.isocurve_v(v).unwrap();
        prop_assert_eq!(c.domain(), (ua, ub));
        for u in params(ua, ub, 12) {
            let d = (c.point(u) - s.point(u, v)).length();
            prop_assert!(d <= tol, "isocurve_v({v}) differs by {d} at u = {u}");
        }
    }
    Ok(())
}

fn check_decompose(s: &NurbsSurface) -> Result<(), TestCaseError> {
    let ((ua, ub), (va, vb)) = s.domain();
    let inside = |b: Vec<f64>, lo: f64, hi: f64| -> Vec<f64> {
        b.into_iter().filter(|&t| t >= lo && t <= hi).collect()
    };
    let us = inside(s.knots_u().breakpoints(), ua, ub);
    let vs = inside(s.knots_v().breakpoints(), va, vb);
    let patches = s.decompose_bezier();
    prop_assert_eq!(patches.len(), us.len() - 1);
    let tol = 1e-9 * surface_size(s);
    for (row, wu) in patches.iter().zip(us.windows(2)) {
        prop_assert_eq!(row.len(), vs.len() - 1);
        for (patch, wv) in row.iter().zip(vs.windows(2)) {
            prop_assert_eq!(patch.domain(), ((wu[0], wu[1]), (wv[0], wv[1])));
            prop_assert_eq!(
                patch.control_weighted().len(),
                (s.degree_u() + 1) * (s.degree_v() + 1)
            );
            for u in params(wu[0], wu[1], 4) {
                for v in params(wv[0], wv[1], 4) {
                    let d = (patch.point(u, v) - s.point(u, v)).length();
                    prop_assert!(d <= tol, "patch differs by {d} at ({u}, {v})");
                }
            }
        }
    }
    Ok(())
}

fn check_split(s: &NurbsSurface, f: f64, in_u: bool) -> Result<(), TestCaseError> {
    let ((ua, ub), (va, vb)) = s.domain();
    let (lo, hi) = if in_u { (ua, ub) } else { (va, vb) };
    let t = lo + (hi - lo) * (0.01 + 0.98 * f);
    let (left, right) = if in_u { s.split_u(t) } else { s.split_v(t) }.unwrap();
    let tol = 1e-9 * surface_size(s);
    for half in [&left, &right] {
        if in_u {
            prop_assert_eq!(half.domain().1, (va, vb));
        } else {
            prop_assert_eq!(half.domain().0, (ua, ub));
        }
        for (u, v) in grid(half, 12) {
            let d = (half.point(u, v) - s.point(u, v)).length();
            prop_assert!(d <= tol, "half differs by {d} at ({u}, {v})");
        }
    }
    let ends = if in_u {
        (left.domain().0, right.domain().0)
    } else {
        (left.domain().1, right.domain().1)
    };
    prop_assert_eq!(ends.0.0.to_bits(), lo.to_bits());
    prop_assert_eq!(ends.1.1.to_bits(), hi.to_bits());
    prop_assert!((ends.0.1 - ends.1.0).abs() <= 0.0);
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn isocurves_match_the_surface(s in any_surface()) {
        check_isocurves(&s)?;
    }

    #[test]
    fn bezier_patches_match_the_surface(s in any_surface()) {
        check_decompose(&s)?;
    }

    #[test]
    fn split_halves_match_the_surface(s in any_surface(), f in 0.0f64..1.0, in_u in any::<bool>()) {
        check_split(&s, f, in_u)?;
    }

    #[test]
    fn reversed_surfaces_run_backwards(s in any_surface()) {
        let ((ua, ub), (va, vb)) = s.domain();
        let r = s.reversed_u();
        prop_assert_eq!(r.domain(), s.domain());
        assert_same_surface(&s, &r, |u, v| (ua + ub - u, v))?;
        let r = s.reversed_v();
        prop_assert_eq!(r.domain(), s.domain());
        assert_same_surface(&s, &r, |u, v| (u, va + vb - v))?;
        // Reversing twice gives back the knots, up to rounding.
        let back = s.reversed_u().reversed_u();
        let tol = s.knots_u().tolerance();
        for (a, b) in back.knots_u().as_slice().iter().zip(s.knots_u().as_slice()) {
            prop_assert!((a - b).abs() <= tol);
        }
    }

    #[test]
    fn swapped_surface_exchanges_parameters(s in any_surface()) {
        let t = s.swapped_uv();
        prop_assert_eq!(t.domain(), (s.domain().1, s.domain().0));
        assert_same_surface(&s, &t, |u, v| (v, u))?;
        // The normal flips.
        for (u, v) in grid(&s, 6) {
            if let (Some(a), Some(b)) = (s.normal(u, v), t.normal(v, u)) {
                prop_assert!((a + b).length() <= 1e-9);
            }
        }
    }

    #[test]
    fn rigid_motion_moves_the_surface(
        s in any_surface(),
        axis in prop::array::uniform3(-1.0f64..1.0),
        angle in -3.0f64..3.0,
        shift in prop::array::uniform3(-100.0f64..100.0),
    ) {
        let axis = DVec3::from_array(axis);
        prop_assume!(axis.length() > 0.1);
        let m = DMat4::from_rotation_translation(
            DQuat::from_axis_angle(axis.normalize(), angle),
            DVec3::from_array(shift),
        );
        let t = s.transformed(&m).unwrap();
        prop_assert_eq!(t.weights(), s.weights());
        let tol = 1e-9 * (surface_size(&s) + 200.0);
        for (u, v) in grid(&s, 12) {
            let d = (t.point(u, v) - m.transform_point3(s.point(u, v))).length();
            prop_assert!(d <= tol, "transformed surface off by {d} at ({u}, {v})");
        }
    }
}
