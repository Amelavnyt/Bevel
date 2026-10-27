// SPDX-License-Identifier: GPL-3.0-or-later

//! Degree elevation keeps the geometry; degree reduction stays within its
//! error bound.

use bevel_geom::{GeomError, Homogeneous, NurbsCurve};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{any_curve2, any_curve3, assert_same_geometry, config, dist, params, size};

fn check_elevate<H: Homogeneous>(c: &NurbsCurve<H>, t: usize) -> Result<(), TestCaseError> {
    let e = c.elevate_degree(t).unwrap();
    prop_assert_eq!(e.degree(), c.degree() + t);
    let (a, b) = c.domain();
    let interior = c
        .knots()
        .breakpoints()
        .into_iter()
        .filter(|&u| u > a && u < b)
        .count();
    prop_assert_eq!(
        e.control_weighted().len(),
        c.control_weighted().len() + t * (interior + 1)
    );
    for u in c.knots().breakpoints() {
        prop_assert_eq!(e.knots().multiplicity(u), c.knots().multiplicity(u) + t);
    }
    assert_same_geometry(c, &e, |u| u, 64)?;
    // And back down again.
    let mut r = e;
    for _ in 0..t {
        r = r.reduce_degree(1e-9 * size(c)).unwrap();
    }
    prop_assert_eq!(r.degree(), c.degree());
    prop_assert_eq!(r.knots(), c.knots());
    assert_same_geometry(c, &r, |u| u, 64)
}

fn check_reduction_bound<H: Homogeneous>(c: &NurbsCurve<H>) -> Result<(), TestCaseError> {
    prop_assume!(c.degree() >= 2);
    let bound = match c.reduce_degree(0.0) {
        Ok(_) => return Ok(()),
        Err(GeomError::ToleranceNotMet { error }) => error,
        Err(e) => return Err(TestCaseError::fail(format!("unexpected error {e}"))),
    };
    prop_assume!(bound.is_finite());
    let r = c.reduce_degree(bound).unwrap();
    prop_assert_eq!(r.degree(), c.degree() - 1);
    let (a, b) = c.domain();
    for u in params(a, b, 200) {
        let d = dist::<H>(c.point(u), r.point(u));
        prop_assert!(
            d <= bound * (1.0 + 1e-9) + 1e-12 * size(c),
            "deviation {d} exceeds the bound {bound} at {u}"
        );
    }
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn elevate_degree_keeps_geometry_3d(c in any_curve3(), t in 1usize..=3) {
        check_elevate(&c, t)?;
    }

    #[test]
    fn elevate_degree_keeps_geometry_2d(c in any_curve2(), t in 1usize..=3) {
        check_elevate(&c, t)?;
    }

    #[test]
    fn reduce_degree_stays_within_its_bound_3d(c in any_curve3()) {
        check_reduction_bound(&c)?;
    }

    #[test]
    fn reduce_degree_stays_within_its_bound_2d(c in any_curve2()) {
        check_reduction_bound(&c)?;
    }
}
