// SPDX-License-Identifier: GPL-3.0-or-later

//! Arc length and closest points.

use bevel_geom::{Homogeneous, NurbsCurve};
use bevel_math::{DVec2, DVec3};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{any_curve2, any_curve3, config, dist, params, polyline3, size};

fn check_additive<H: Homogeneous>(c: &NurbsCurve<H>, f: f64) -> Result<(), TestCaseError> {
    let (a, b) = c.domain();
    let tol = 1e-8 * size(c);
    let u = a + (b - a) * (0.01 + 0.98 * f);
    let whole = c.arc_length(a, b, tol).unwrap();
    let left = c.arc_length(a, u, tol).unwrap();
    let right = c.arc_length(u, b, tol).unwrap();
    prop_assert!(
        (whole - left - right).abs() <= 3.0 * tol,
        "{whole} vs {left} + {right}"
    );
    let (l, r) = c.split(u).unwrap();
    let (l0, l1) = l.domain();
    let (r0, r1) = r.domain();
    let halves = l.arc_length(l0, l1, tol).unwrap() + r.arc_length(r0, r1, tol).unwrap();
    prop_assert!(
        (whole - halves).abs() <= 3.0 * tol,
        "{whole} vs halves {halves}"
    );
    // Reversal keeps the length.
    let rev = c.reversed().arc_length(a, b, tol).unwrap();
    prop_assert!((whole - rev).abs() <= 2.0 * tol);
    Ok(())
}

fn check_closest_on_curve<H: Homogeneous>(c: &NurbsCurve<H>, f: f64) -> Result<(), TestCaseError> {
    let (a, b) = c.domain();
    let u = a + (b - a) * f;
    let p = c.point(u);
    let cp = c.closest_point(p).unwrap();
    prop_assert!(
        cp.distance < 1e-7,
        "distance {} for a point on the curve",
        cp.distance
    );
    prop_assert!(dist::<H>(cp.point, p) < 1e-7);
    prop_assert!(dist::<H>(c.point(cp.t), cp.point) <= 1e-12 * size(c));
    Ok(())
}

fn check_closest_off_curve<H: Homogeneous>(
    c: &NurbsCurve<H>,
    target: H::Point,
) -> Result<(), TestCaseError> {
    let cp = c.closest_point(target).unwrap();
    let (a, b) = c.domain();
    prop_assert!(cp.t >= a && cp.t <= b);
    prop_assert!((dist::<H>(c.point(cp.t), target) - cp.distance).abs() <= 1e-12 * size(c));
    for u in params(a, b, 200) {
        let d = dist::<H>(c.point(u), target);
        prop_assert!(
            cp.distance <= d + 1e-9,
            "closest distance {} but the sample at {u} is at {d}",
            cp.distance
        );
    }
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn polyline_length_is_the_sum_of_its_segments(c in polyline3()) {
        let pts = c.control_points();
        let expected: f64 = pts.windows(2).map(|w| (w[1] - w[0]).length()).sum();
        let (a, b) = c.domain();
        let tol = 1e-9 * size(&c);
        let l = c.arc_length(a, b, tol).unwrap();
        prop_assert!((l - expected).abs() <= tol, "{l} vs {expected}");
    }

    #[test]
    fn arc_length_is_additive_3d(c in any_curve3(), f in 0.0f64..1.0) {
        check_additive(&c, f)?;
    }

    #[test]
    fn arc_length_is_additive_2d(c in any_curve2(), f in 0.0f64..1.0) {
        check_additive(&c, f)?;
    }

    #[test]
    fn closest_point_of_a_curve_point_is_itself_3d(c in any_curve3(), f in 0.0f64..=1.0) {
        check_closest_on_curve(&c, f)?;
    }

    #[test]
    fn closest_point_of_a_curve_point_is_itself_2d(c in any_curve2(), f in 0.0f64..=1.0) {
        check_closest_on_curve(&c, f)?;
    }

    #[test]
    fn closest_point_beats_dense_sampling_3d(
        c in any_curve3(),
        t in prop::array::uniform3(-150.0f64..150.0),
    ) {
        check_closest_off_curve(&c, DVec3::from_array(t))?;
    }

    #[test]
    fn closest_point_beats_dense_sampling_2d(
        c in any_curve2(),
        t in prop::array::uniform2(-150.0f64..150.0),
    ) {
        check_closest_off_curve(&c, DVec2::from_array(t))?;
    }
}
