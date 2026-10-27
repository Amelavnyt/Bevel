// SPDX-License-Identifier: GPL-3.0-or-later

//! Knot insertion, refinement and removal keep the geometry.

use bevel_geom::{Homogeneous, NurbsCurve};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{any_curve2, any_curve3, assert_same_geometry, config, size};

/// A parameter strictly inside the domain: either the fraction `f` of the
/// way along it, or (when `on_knot`) an existing interior knot picked by `f`.
#[allow(clippy::cast_sign_loss)] // f is in [0, 1)
fn pick_param<H: Homogeneous>(c: &NurbsCurve<H>, f: f64, on_knot: bool) -> f64 {
    let (a, b) = c.domain();
    let interior: Vec<f64> = c
        .knots()
        .breakpoints()
        .into_iter()
        .filter(|&u| u > a && u < b)
        .collect();
    if on_knot && !interior.is_empty() {
        let i = ((f * interior.len() as f64) as usize).min(interior.len() - 1);
        interior[i]
    } else {
        a + (b - a) * (0.01 + 0.98 * f)
    }
}

fn check_insert<H: Homogeneous>(
    c: &NurbsCurve<H>,
    f: f64,
    on_knot: bool,
    r: usize,
) -> Result<(), TestCaseError> {
    let u = pick_param(c, f, on_knot);
    let room = c.degree() - c.knots().multiplicity(u);
    let r = r.min(room);
    let d = c.insert_knot(u, r).unwrap();
    prop_assert_eq!(d.control_weighted().len(), c.control_weighted().len() + r);
    prop_assert_eq!(d.knots().multiplicity(u), c.knots().multiplicity(u) + r);
    assert_same_geometry(c, &d, |u| u, 64)?;
    if r < room {
        prop_assert!(c.insert_knot(u, room + 1).is_err());
    }
    Ok(())
}

fn check_refine<H: Homogeneous>(c: &NurbsCurve<H>, fs: &[f64]) -> Result<(), TestCaseError> {
    // Keep only knots that fit under the multiplicity limit.
    let mut x: Vec<f64> = Vec::new();
    for (i, &f) in fs.iter().enumerate() {
        let u = pick_param(c, f, i % 3 == 0);
        let count = x.iter().filter(|&&v| (v - u).abs() <= 0.0).count();
        if c.knots().multiplicity(u) + count < c.degree() {
            x.push(u);
        }
    }
    let d = c.refine_knots(&x).unwrap();
    prop_assert_eq!(
        d.control_weighted().len(),
        c.control_weighted().len() + x.len()
    );
    prop_assert_eq!(
        d.knots().as_slice().len(),
        c.knots().as_slice().len() + x.len()
    );
    assert_same_geometry(c, &d, |u| u, 64)
}

fn check_insert_then_remove<H: Homogeneous>(
    c: &NurbsCurve<H>,
    f: f64,
    r: usize,
) -> Result<(), TestCaseError> {
    let u = pick_param(c, f, false);
    // A new knot (not one of the existing ones), inserted up to p times.
    prop_assume!(c.knots().multiplicity(u) == 0);
    let r = r.clamp(1, c.degree());
    let d = c.insert_knot(u, r).unwrap();
    let (e, removed) = d.remove_knot(u, r, 1e-9 * size(c));
    prop_assert_eq!(removed, r);
    prop_assert_eq!(e.control_weighted().len(), c.control_weighted().len());
    prop_assert_eq!(e.knots().as_slice().len(), c.knots().as_slice().len());
    assert_same_geometry(c, &e, |u| u, 64)
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn insert_knot_keeps_geometry_3d(c in any_curve3(), f in 0.0f64..1.0, k in any::<bool>(), r in 1usize..=5) {
        check_insert(&c, f, k, r)?;
    }

    #[test]
    fn insert_knot_keeps_geometry_2d(c in any_curve2(), f in 0.0f64..1.0, k in any::<bool>(), r in 1usize..=5) {
        check_insert(&c, f, k, r)?;
    }

    #[test]
    fn refine_knots_keeps_geometry_3d(c in any_curve3(), fs in prop::collection::vec(0.0f64..1.0, 1..12)) {
        check_refine(&c, &fs)?;
    }

    #[test]
    fn refine_knots_keeps_geometry_2d(c in any_curve2(), fs in prop::collection::vec(0.0f64..1.0, 1..12)) {
        check_refine(&c, &fs)?;
    }

    #[test]
    fn inserted_knot_can_be_removed_3d(c in any_curve3(), f in 0.0f64..1.0, r in 1usize..=5) {
        check_insert_then_remove(&c, f, r)?;
    }

    #[test]
    fn inserted_knot_can_be_removed_2d(c in any_curve2(), f in 0.0f64..1.0, r in 1usize..=5) {
        check_insert_then_remove(&c, f, r)?;
    }
}
