// SPDX-License-Identifier: GPL-3.0-or-later

//! Surface parameters, interpolation and least-squares approximation.

use bevel_geom::NurbsSurface;
use bevel_geom::fit::{
    Parametrization, approximate_surface, interpolate_surface, surface_parameters,
};
use proptest::prelude::*;

use crate::strategies::{config, method};
use crate::surface_strategies::{GridCase, grid_case};

/// The largest distance between a grid point and the surface at the point's
/// parameters.
fn max_deviation(s: &NurbsSurface, case: &GridCase, method: Parametrization) -> f64 {
    let (us, vs) = surface_parameters(&case.grid, case.rows, case.cols, method).unwrap();
    let mut worst = 0.0f64;
    for (k, &u) in us.iter().enumerate() {
        for (l, &v) in vs.iter().enumerate() {
            let q = case.grid[k * case.cols + l];
            worst = worst.max((s.point(u, v) - q).length());
        }
    }
    worst
}

/// Sanity bound on the deviation of a least-squares fit, as a multiple of
/// the size of the data (the largest coordinate magnitude). The two-pass fit
/// has no tight guarantee and the deviation is not monotone in the control
/// point counts. The worst case seen is a 2 x 2 net (the bilinear patch on
/// the four corners) with a deviation of about twice the size; a deviation
/// of several times that would mean a broken solve.
const DEVIATION_BOUND: f64 = 4.0;

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn parameters_are_increasing_from_zero_to_one(case in grid_case(), m in method()) {
        let (us, vs) = surface_parameters(&case.grid, case.rows, case.cols, m).unwrap();
        prop_assert_eq!(us.len(), case.rows);
        prop_assert_eq!(vs.len(), case.cols);
        for p in [&us, &vs] {
            prop_assert!(p[0].abs() < 1e-15 && (p[p.len() - 1] - 1.0).abs() < 1e-15);
            prop_assert!(p.windows(2).all(|w| w[1] > w[0]));
        }
    }

    #[test]
    fn interpolation_passes_through_every_grid_point(case in grid_case(), m in method()) {
        let s = interpolate_surface(
            &case.grid, case.rows, case.cols, case.degree_u, case.degree_v, m,
        )
        .unwrap();
        prop_assert_eq!((s.degree_u(), s.degree_v()), (case.degree_u, case.degree_v));
        prop_assert_eq!((s.count_u(), s.count_v()), (case.rows, case.cols));
        prop_assert!(!s.is_rational());
        prop_assert!(s.domain() == ((0.0, 1.0), (0.0, 1.0)));
        let tol = 1e-9 * case.size();
        let (us, vs) = surface_parameters(&case.grid, case.rows, case.cols, m).unwrap();
        for (k, &u) in us.iter().enumerate() {
            for (l, &v) in vs.iter().enumerate() {
                let d = (s.point(u, v) - case.grid[k * case.cols + l]).length();
                prop_assert!(d <= tol, "point ({k}, {l}) missed by {d} (tol {tol})");
            }
        }
    }

    #[test]
    fn approximation_keeps_the_corners_and_has_a_finite_deviation(
        case in grid_case(),
        pick_u in 0usize..=999,
        pick_v in 0usize..=999,
        m in method(),
    ) {
        // The counts spread over degree + 1 ..= rows (columns).
        let count_u = case.degree_u + 1 + pick_u * (case.rows - case.degree_u - 1) / 999;
        let count_v = case.degree_v + 1 + pick_v * (case.cols - case.degree_v - 1) / 999;
        let s = approximate_surface(
            &case.grid, case.rows, case.cols, case.degree_u, case.degree_v,
            count_u, count_v, m,
        )
        .unwrap();
        prop_assert_eq!((s.count_u(), s.count_v()), (count_u, count_v));
        prop_assert_eq!((s.degree_u(), s.degree_v()), (case.degree_u, case.degree_v));
        let ((ua, ub), (va, vb)) = s.domain();
        let tol = 1e-12 * case.size();
        let last = case.rows * case.cols - 1;
        let corners = [
            (ua, va, case.grid[0]),
            (ua, vb, case.grid[case.cols - 1]),
            (ub, va, case.grid[last + 1 - case.cols]),
            (ub, vb, case.grid[last]),
        ];
        for (u, v, q) in corners {
            let d = (s.point(u, v) - q).length();
            prop_assert!(d <= tol, "corner ({u}, {v}) off by {d} (tol {tol})");
        }
        let deviation = max_deviation(&s, &case, m);
        prop_assert!(deviation.is_finite());
        prop_assert!(
            deviation <= DEVIATION_BOUND * case.size(),
            "deviation {deviation} (size {}) with {count_u} x {count_v} control points",
            case.size()
        );
    }

    #[test]
    fn approximation_with_every_control_point_interpolates(
        case in grid_case(),
        m in method(),
    ) {
        let s = approximate_surface(
            &case.grid, case.rows, case.cols, case.degree_u, case.degree_v,
            case.rows, case.cols, m,
        )
        .unwrap();
        let tol = 1e-9 * case.size();
        let deviation = max_deviation(&s, &case, m);
        prop_assert!(deviation <= tol, "deviation {deviation} (tol {tol})");
        let i = interpolate_surface(
            &case.grid, case.rows, case.cols, case.degree_u, case.degree_v, m,
        )
        .unwrap();
        prop_assert!(s == i);
    }
}
