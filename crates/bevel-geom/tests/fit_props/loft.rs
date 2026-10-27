// SPDX-License-Identifier: GPL-3.0-or-later

//! Lofting: the surface contains every input curve.

use bevel_geom::fit::{Parametrization, loft, loft_parameters};
use bevel_geom::{GeomError, KnotVector, NurbsCurve3};
use bevel_math::DVec3;
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

use crate::strategies::{config, method};

/// Spacing between consecutive curves along `z`; the random offsets of the
/// control points stay well below it.
const LAYER: f64 = 12.0;

/// Degree, control point count and domain of a random curve.
#[derive(Clone, Copy, Debug)]
struct Spec {
    degree: usize,
    count: usize,
    start: f64,
    length: f64,
}

/// A degree of 1 to 3, 3 to 7 control points on a uniform clamped knot
/// vector and a random domain.
fn spec() -> impl Strategy<Value = Spec> {
    (1usize..=3, 3usize..=7, -5.0f64..5.0, 0.5f64..4.0).prop_map(
        |(degree, count, start, length)| Spec {
            degree,
            count: count.max(degree + 1),
            start,
            length,
        },
    )
}

/// `count` weights.
fn weights(count: usize) -> impl Strategy<Value = Vec<f64>> {
    prop::collection::vec(0.7f64..1.4, count)
}

/// A curve of shape `spec` in layer `layer` (placed along `z`) with random
/// control points and the given weights (a non-rational curve for `None`).
fn curve(
    layer: usize,
    spec: Spec,
    weights: Option<Vec<f64>>,
) -> impl Strategy<Value = NurbsCurve3> {
    prop::collection::vec((-20.0f64..20.0, -20.0f64..20.0, -3.0f64..3.0), spec.count).prop_map(
        move |coords| {
            let pts: Vec<DVec3> = coords
                .into_iter()
                .map(|(x, y, dz)| DVec3::new(x, y, LAYER * layer as f64 + dz))
                .collect();
            let uniform = KnotVector::clamped_uniform(spec.degree, spec.count).unwrap();
            let knots: Vec<f64> = uniform
                .as_slice()
                .iter()
                .map(|&u| spec.start + spec.length * u)
                .collect();
            let knots = KnotVector::new(knots, spec.degree).unwrap();
            NurbsCurve3::from_points(spec.degree, knots, &pts, weights.as_deref()).unwrap()
        },
    )
}

/// What kind of curves a case holds.
#[derive(Clone, Copy, Debug)]
enum Kind {
    /// Non-rational curves of unrelated shapes.
    Plain,
    /// Rational curves with the same shape and the same weights, so that the
    /// interpolated weights are constant and the loft cannot fail.
    SharedWeights,
    /// Rational curves of unrelated shapes and weights. The interpolation of
    /// the weights across the curves can overshoot to a non-positive value,
    /// which is rejected.
    FreeWeights,
}

/// 3 to 7 curves of the given kind stacked along `z` and a degree across
/// that the number of curves allows (1 to 3).
fn loft_case(kind: Kind) -> impl Strategy<Value = (Vec<NurbsCurve3>, usize)> {
    (3usize..=7, 1usize..=3).prop_flat_map(move |(count, degree_v)| {
        let curves: BoxedStrategy<Vec<NurbsCurve3>> = match kind {
            Kind::Plain => (0..count)
                .map(|k| spec().prop_flat_map(move |sp| curve(k, sp, None)))
                .collect::<Vec<_>>()
                .boxed(),
            Kind::SharedWeights => spec()
                .prop_flat_map(move |sp| (Just(sp), weights(sp.count)))
                .prop_flat_map(move |(sp, w)| {
                    (0..count)
                        .map(|k| curve(k, sp, Some(w.clone())))
                        .collect::<Vec<_>>()
                })
                .boxed(),
            Kind::FreeWeights => (0..count)
                .map(|k| {
                    spec()
                        .prop_flat_map(move |sp| (Just(sp), weights(sp.count)))
                        .prop_flat_map(move |(sp, w)| curve(k, sp, Some(w)))
                })
                .collect::<Vec<_>>()
                .boxed(),
        };
        (curves, Just(degree_v.min(count - 1)))
    })
}

/// The loft contains each input curve at its `v` parameter. If `may_reject`,
/// a surface with a non-positive weight is accepted as the outcome.
fn check_contains(
    curves: &[NurbsCurve3],
    degree_v: usize,
    m: Parametrization,
    may_reject: bool,
) -> Result<(), TestCaseError> {
    let s = match loft(curves, degree_v, m) {
        Ok(s) => s,
        Err(GeomError::InvalidWeight { .. }) if may_reject => return Ok(()),
        Err(e) => return Err(TestCaseError::fail(format!("loft failed: {e}"))),
    };
    let vs = loft_parameters(curves, m).unwrap();
    prop_assert_eq!(vs.len(), curves.len());
    prop_assert!(vs[0].abs() < 1e-15 && (vs[vs.len() - 1] - 1.0).abs() < 1e-15);
    prop_assert!(vs.windows(2).all(|w| w[1] > w[0]));
    prop_assert_eq!(s.degree_v(), degree_v);
    prop_assert_eq!(s.count_v(), curves.len());
    let degree_u = curves.iter().map(NurbsCurve3::degree).max().unwrap();
    prop_assert_eq!(s.degree_u(), degree_u);
    let (a, b) = curves[0].domain();
    prop_assert!(s.domain().0 == (a, b));
    // 1e-9 of the size of the data (coordinates up to LAYER * count).
    let tol = 1e-9 * LAYER * curves.len() as f64;
    for (c, &v) in curves.iter().zip(&vs) {
        let (lo, hi) = c.domain();
        for i in 0..50 {
            let t = lo + (hi - lo) * f64::from(i) / 49.0;
            let u = a + (t - lo) * (b - a) / (hi - lo);
            let d = (s.point(u, v) - c.point(t)).length();
            prop_assert!(
                d <= tol,
                "curve at t = {t} is {d} from the surface (tol {tol})"
            );
        }
    }
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn loft_contains_every_plain_curve(
        (curves, degree_v) in loft_case(Kind::Plain),
        m in method(),
    ) {
        check_contains(&curves, degree_v, m, false)?;
        let s = loft(&curves, degree_v, m).unwrap();
        prop_assert!(!s.is_rational());
    }

    #[test]
    fn loft_contains_every_rational_curve_with_shared_weights(
        (curves, degree_v) in loft_case(Kind::SharedWeights),
        m in method(),
    ) {
        check_contains(&curves, degree_v, m, false)?;
    }

    #[test]
    fn loft_contains_rational_curves_or_rejects_the_weights(
        (curves, degree_v) in loft_case(Kind::FreeWeights),
        m in method(),
    ) {
        check_contains(&curves, degree_v, m, true)?;
    }
}
