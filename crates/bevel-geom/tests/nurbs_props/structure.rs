// SPDX-License-Identifier: GPL-3.0-or-later

//! Clamping, decomposition, splitting, reversal and reparametrisation keep
//! the geometry.

use bevel_geom::{Homogeneous, NurbsCurve};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{
    any_curve2, any_curve3, assert_same_geometry, config, dist, params, size, unclamped_curve3,
};

fn check_decompose<H: Homogeneous>(c: &NurbsCurve<H>) -> Result<(), TestCaseError> {
    let segments = c.decompose_bezier();
    let (a, b) = c.domain();
    let spans: Vec<f64> = c
        .knots()
        .breakpoints()
        .into_iter()
        .filter(|&u| u >= a && u <= b)
        .collect();
    prop_assert_eq!(segments.len(), spans.len() - 1);
    let tol = 1e-9 * size(c);
    for (seg, w) in segments.iter().zip(spans.windows(2)) {
        prop_assert_eq!(seg.domain(), (w[0], w[1]));
        prop_assert_eq!(seg.control_weighted().len(), c.degree() + 1);
        for u in params(w[0], w[1], 16) {
            let d = dist::<H>(seg.point(u), c.point(u));
            prop_assert!(d <= tol, "segment differs by {d} at {u}");
        }
    }
    Ok(())
}

fn check_split<H: Homogeneous>(c: &NurbsCurve<H>, f: f64) -> Result<(), TestCaseError> {
    let (a, b) = c.domain();
    let u = a + (b - a) * (0.01 + 0.98 * f);
    let (left, right) = c.split(u).unwrap();
    prop_assert_eq!(left.domain().0.to_bits(), a.to_bits());
    prop_assert_eq!(right.domain().1.to_bits(), b.to_bits());
    let tol = 1e-9 * size(c);
    let (l0, l1) = left.domain();
    let (r0, r1) = right.domain();
    prop_assert!((l1 - r0).abs() <= 0.0, "halves do not meet: {l1} vs {r0}");
    for t in params(l0, l1, 32) {
        prop_assert!(dist::<H>(left.point(t), c.point(t)) <= tol);
    }
    for t in params(r0, r1, 32) {
        prop_assert!(dist::<H>(right.point(t), c.point(t)) <= tol);
    }
    Ok(())
}

fn check_reverse<H: Homogeneous>(c: &NurbsCurve<H>) -> Result<(), TestCaseError> {
    let (a, b) = c.domain();
    let r = c.reversed();
    prop_assert_eq!(r.domain(), (a, b));
    assert_same_geometry(c, &r, |u| a + b - u, 64)
}

fn check_reparametrize<H: Homogeneous>(
    c: &NurbsCurve<H>,
    start: f64,
    len: f64,
) -> Result<(), TestCaseError> {
    let (a, b) = c.domain();
    let end = start + len;
    let r = c.reparametrized(start, end).unwrap();
    prop_assert_eq!(r.domain(), (start, end));
    assert_same_geometry(c, &r, |u| start + (u - a) * (end - start) / (b - a), 64)
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn bezier_segments_match_the_curve_3d(c in any_curve3()) {
        check_decompose(&c)?;
    }

    #[test]
    fn bezier_segments_match_the_curve_2d(c in any_curve2()) {
        check_decompose(&c)?;
    }

    #[test]
    fn split_halves_match_the_curve_3d(c in any_curve3(), f in 0.0f64..1.0) {
        check_split(&c, f)?;
    }

    #[test]
    fn split_halves_match_the_curve_2d(c in any_curve2(), f in 0.0f64..1.0) {
        check_split(&c, f)?;
    }

    #[test]
    fn reversed_curve_runs_backwards_3d(c in any_curve3()) {
        check_reverse(&c)?;
    }

    #[test]
    fn reversed_curve_runs_backwards_2d(c in any_curve2()) {
        check_reverse(&c)?;
    }

    #[test]
    fn reparametrized_curve_matches_3d(c in any_curve3(), start in -50.0f64..50.0, len in 0.01f64..100.0) {
        check_reparametrize(&c, start, len)?;
    }

    #[test]
    fn reparametrized_curve_matches_2d(c in any_curve2(), start in -50.0f64..50.0, len in 0.01f64..100.0) {
        check_reparametrize(&c, start, len)?;
    }

    #[test]
    fn unclamped_curves_clamp_split_and_decompose(c in unclamped_curve3(), f in 0.0f64..1.0) {
        prop_assert!(!c.knots().is_clamped(c.degree()));
        let k = c.clamped();
        prop_assert!(k.knots().is_clamped(c.degree()));
        prop_assert_eq!(k.domain(), c.domain());
        assert_same_geometry(&c, &k, |u| u, 64)?;
        check_reverse(&c)?;
        check_decompose(&c)?;
        check_split(&c, f)?;
    }
}
