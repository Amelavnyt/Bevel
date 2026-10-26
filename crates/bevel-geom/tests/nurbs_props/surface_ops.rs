// SPDX-License-Identifier: GPL-3.0-or-later

//! Surface knot insertion, refinement and degree elevation keep the
//! geometry, in both directions.

use bevel_geom::{KnotVector, NurbsSurface};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{any_surface, assert_same_surface, config};

/// A parameter direction.
#[derive(Clone, Copy, Debug)]
enum Dir {
    U,
    V,
}

fn dir() -> impl Strategy<Value = Dir> {
    prop_oneof![Just(Dir::U), Just(Dir::V)]
}

/// Degree, knots and domain of `s` in direction `d`.
fn direction(s: &NurbsSurface, d: Dir) -> (usize, &KnotVector, (f64, f64)) {
    let (du, dv) = s.domain();
    match d {
        Dir::U => (s.degree_u(), s.knots_u(), du),
        Dir::V => (s.degree_v(), s.knots_v(), dv),
    }
}

/// A parameter strictly inside the domain: either the fraction `f` of the
/// way along it, or (when `on_knot`) an existing interior knot picked by `f`.
#[allow(clippy::cast_sign_loss)] // f is in [0, 1)
fn pick_param(knots: &KnotVector, (a, b): (f64, f64), f: f64, on_knot: bool) -> f64 {
    let interior: Vec<f64> = knots
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

fn check_insert(
    s: &NurbsSurface,
    d: Dir,
    f: f64,
    on_knot: bool,
    r: usize,
) -> Result<(), TestCaseError> {
    let (p, knots, domain) = direction(s, d);
    let t = pick_param(knots, domain, f, on_knot);
    let room = p - knots.multiplicity(t);
    let r = r.min(room);
    let e = match d {
        Dir::U => s.insert_knot_u(t, r),
        Dir::V => s.insert_knot_v(t, r),
    }
    .unwrap();
    let (_, new_knots, _) = direction(&e, d);
    prop_assert_eq!(new_knots.multiplicity(t), knots.multiplicity(t) + r);
    prop_assert_eq!(
        e.control_weighted().len(),
        s.control_weighted().len()
            + r * match d {
                Dir::U => s.count_v(),
                Dir::V => s.count_u(),
            }
    );
    assert_same_surface(s, &e, |u, v| (u, v))?;
    let too_many = match d {
        Dir::U => s.insert_knot_u(t, room + 1),
        Dir::V => s.insert_knot_v(t, room + 1),
    };
    prop_assert!(too_many.is_err());
    Ok(())
}

fn check_refine(s: &NurbsSurface, d: Dir, fs: &[f64]) -> Result<(), TestCaseError> {
    let (p, knots, domain) = direction(s, d);
    let mut x: Vec<f64> = Vec::new();
    for (i, &f) in fs.iter().enumerate() {
        let t = pick_param(knots, domain, f, i % 3 == 0);
        let count = x.iter().filter(|&&v| (v - t).abs() <= 0.0).count();
        if knots.multiplicity(t) + count < p {
            x.push(t);
        }
    }
    let e = match d {
        Dir::U => s.refine_knots_u(&x),
        Dir::V => s.refine_knots_v(&x),
    }
    .unwrap();
    let (_, new_knots, _) = direction(&e, d);
    prop_assert_eq!(new_knots.as_slice().len(), knots.as_slice().len() + x.len());
    assert_same_surface(s, &e, |u, v| (u, v))
}

fn check_elevate(s: &NurbsSurface, d: Dir, t: usize) -> Result<(), TestCaseError> {
    let (p, knots, _) = direction(s, d);
    let e = match d {
        Dir::U => s.elevate_degree_u(t),
        Dir::V => s.elevate_degree_v(t),
    }
    .unwrap();
    let (q, new_knots, _) = direction(&e, d);
    prop_assert_eq!(q, p + t);
    for u in knots.breakpoints() {
        prop_assert_eq!(new_knots.multiplicity(u), knots.multiplicity(u) + t);
    }
    assert_same_surface(s, &e, |u, v| (u, v))
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn surface_insert_knot_keeps_geometry(
        s in any_surface(),
        d in dir(),
        f in 0.0f64..1.0,
        k in any::<bool>(),
        r in 1usize..=4,
    ) {
        check_insert(&s, d, f, k, r)?;
    }

    #[test]
    fn surface_refine_knots_keeps_geometry(
        s in any_surface(),
        d in dir(),
        fs in prop::collection::vec(0.0f64..1.0, 1..8),
    ) {
        check_refine(&s, d, &fs)?;
    }

    #[test]
    fn surface_elevate_degree_keeps_geometry(s in any_surface(), d in dir(), t in 1usize..=2) {
        check_elevate(&s, d, t)?;
    }
}
