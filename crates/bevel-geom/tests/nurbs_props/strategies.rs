// SPDX-License-Identifier: GPL-3.0-or-later

//! Random valid knot vectors and curves for the property tests.

use bevel_geom::{Homogeneous, KnotVector, NurbsCurve, NurbsCurve2, NurbsCurve3};
use bevel_math::{DVec2, DVec3};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed, TestCaseError};

/// A fixed-seed configuration so that runs are reproducible.
///
/// For exploratory runs, `BEVEL_PROPTEST_SEED` replaces the seed and
/// `BEVEL_PROPTEST_CASES` the number of cases.
pub fn config(cases: u32) -> Config {
    let env = |name: &str| std::env::var(name).ok().and_then(|v| v.parse::<u64>().ok());
    Config {
        cases: env("BEVEL_PROPTEST_CASES").map_or(cases, |c| u32::try_from(c).unwrap_or(cases)),
        rng_seed: RngSeed::Fixed(env("BEVEL_PROPTEST_SEED").unwrap_or(0x5eed_b0e1)),
        failure_persistence: None,
        ..Config::default()
    }
}

/// A clamped knot vector of degree `degree` with `extra` interior knots,
/// on a random domain. Interior knots are separated by random gaps and may
/// repeat up to multiplicity `degree`.
pub fn clamped_knots(degree: usize, extra: usize) -> impl Strategy<Value = Vec<f64>> {
    (
        prop::collection::vec((0.05f64..1.0, any::<bool>()), extra),
        0.05f64..1.0,
        -10.0f64..10.0,
        0.5f64..20.0,
    )
        .prop_map(move |(items, last_gap, start, length)| {
            let mut interior = Vec::with_capacity(extra);
            let mut cur = 0.0;
            let mut run = 0;
            for (i, (gap, repeat)) in items.into_iter().enumerate() {
                if repeat && i > 0 && run < degree {
                    run += 1;
                } else {
                    cur += gap;
                    run = 1;
                }
                interior.push(cur);
            }
            let end = cur + last_gap;
            let map = |u: f64| start + length * u / end;
            let mut knots = vec![start; degree + 1];
            knots.extend(interior.into_iter().map(map));
            knots.extend(std::iter::repeat_n(start + length, degree + 1));
            knots
        })
}

/// A degree in `1..=5` with a clamped knot vector for `degree + 1 + extra`
/// control points, `extra` in `2..=10`.
pub fn degree_and_knots() -> impl Strategy<Value = (usize, Vec<f64>)> {
    degree_and_knots_in(1..=5)
}

/// As [`degree_and_knots`] with the degree drawn from `degrees`.
fn degree_and_knots_in(
    degrees: std::ops::RangeInclusive<usize>,
) -> impl Strategy<Value = (usize, Vec<f64>)> {
    (degrees, 2usize..=10)
        .prop_flat_map(|(p, extra)| clamped_knots(p, extra).prop_map(move |k| (p, k)))
}

/// `count` parameters spread evenly over `[a, b]`, ends included.
pub fn params(a: f64, b: f64, count: usize) -> Vec<f64> {
    (0..count)
        .map(|i| a + (b - a) * i as f64 / (count - 1) as f64)
        .collect()
}

/// Control point coordinates.
fn coord() -> impl Strategy<Value = f64> {
    -100.0f64..100.0
}

/// Raw ingredients of a curve.
#[derive(Clone, Debug)]
struct CurveParts {
    degree: usize,
    knots: Vec<f64>,
    coords: Vec<Vec<f64>>,
    weights: Option<Vec<f64>>,
}

/// Degree, knots, coordinates and (when rational) weights for a random
/// curve of dimension `dim`.
fn curve_parts(
    dim: usize,
    rational: bool,
    degrees: std::ops::RangeInclusive<usize>,
) -> impl Strategy<Value = CurveParts> {
    degree_and_knots_in(degrees).prop_flat_map(move |(p, knots)| {
        let count = knots.len() - p - 1;
        (
            Just(p),
            Just(knots),
            prop::collection::vec(prop::collection::vec(coord(), dim), count),
            prop::collection::vec(0.2f64..5.0, count),
        )
            .prop_map(move |(degree, knots, coords, weights)| CurveParts {
                degree,
                knots,
                coords,
                weights: rational.then_some(weights),
            })
    })
}

/// A random 3D curve; rational when `rational` is set.
pub fn curve3(rational: bool) -> impl Strategy<Value = NurbsCurve3> {
    curve3_of_degree(rational, 1..=5)
}

/// A random degree-1 3D curve (a polyline), rational or not.
pub fn polyline3() -> impl Strategy<Value = NurbsCurve3> {
    prop_oneof![
        curve3_of_degree(true, 1..=1),
        curve3_of_degree(false, 1..=1)
    ]
}

/// A random 3D curve with its degree drawn from `degrees`.
fn curve3_of_degree(
    rational: bool,
    degrees: std::ops::RangeInclusive<usize>,
) -> impl Strategy<Value = NurbsCurve3> {
    curve_parts(3, rational, degrees).prop_map(|c| {
        let pts: Vec<DVec3> = c
            .coords
            .iter()
            .map(|v| DVec3::new(v[0], v[1], v[2]))
            .collect();
        let kv = KnotVector::new(c.knots, c.degree).unwrap();
        NurbsCurve3::from_points(c.degree, kv, &pts, c.weights.as_deref()).unwrap()
    })
}

/// A random 2D curve; rational when `rational` is set.
pub fn curve2(rational: bool) -> impl Strategy<Value = NurbsCurve2> {
    curve_parts(2, rational, 1..=5).prop_map(|c| {
        let pts: Vec<DVec2> = c.coords.iter().map(|v| DVec2::new(v[0], v[1])).collect();
        let kv = KnotVector::new(c.knots, c.degree).unwrap();
        NurbsCurve2::from_points(c.degree, kv, &pts, c.weights.as_deref()).unwrap()
    })
}

/// A random 3D curve, rational or not.
pub fn any_curve3() -> impl Strategy<Value = NurbsCurve3> {
    prop_oneof![curve3(true), curve3(false)]
}

/// A random 2D curve, rational or not.
pub fn any_curve2() -> impl Strategy<Value = NurbsCurve2> {
    prop_oneof![curve2(true), curve2(false)]
}

/// `c` on an unclamped knot vector: the `p` knots before the domain and the
/// `p` after it are spread out instead of repeated. The domain stays the
/// same but the curve no longer interpolates its end control points.
pub fn unclamp<H: Homogeneous>(c: &NurbsCurve<H>) -> NurbsCurve<H> {
    let p = c.degree();
    let (a, b) = c.domain();
    let delta = 0.1 * (b - a);
    let mut knots = c.knots().as_slice().to_vec();
    let m = knots.len() - 1;
    for i in 0..p {
        knots[i] = a - (p - i) as f64 * delta;
        knots[m - i] = b + (p - i) as f64 * delta;
    }
    let kv = KnotVector::new(knots, p).unwrap();
    NurbsCurve::new(p, kv, c.control_weighted().to_vec()).unwrap()
}

/// A random unclamped 3D curve, rational or not.
pub fn unclamped_curve3() -> impl Strategy<Value = NurbsCurve3> {
    any_curve3().prop_map(|c| unclamp(&c))
}

/// The length scale of a curve: the largest control point distance from the
/// origin, at least 1.
pub fn size<H: Homogeneous>(c: &NurbsCurve<H>) -> f64 {
    c.control_points()
        .into_iter()
        .map(H::point_length)
        .fold(1.0, f64::max)
}

/// Distance between two model-space points.
pub fn dist<H: Homogeneous>(a: H::Point, b: H::Point) -> f64 {
    H::point_length(a - b)
}

/// At least `total` parameters spread over the knot spans of the domain (at
/// least 8 per span), kept `margin` (relative to the span length) away from
/// the span ends, as `(u, span_start, span_end)`.
pub fn span_params<H: Homogeneous>(
    c: &NurbsCurve<H>,
    total: usize,
    margin: f64,
) -> Vec<(f64, f64, f64)> {
    let (a, b) = c.domain();
    let breaks: Vec<f64> = c
        .knots()
        .breakpoints()
        .into_iter()
        .filter(|&u| u >= a && u <= b)
        .collect();
    let spans = breaks.len() - 1;
    let per_span = total.div_ceil(spans).max(8);
    let mut out = Vec::new();
    for w in breaks.windows(2) {
        let (s0, s1) = (w[0], w[1]);
        for j in 0..per_span {
            let t = margin + (1.0 - 2.0 * margin) * j as f64 / (per_span - 1) as f64;
            out.push((s0 + (s1 - s0) * t, s0, s1));
        }
    }
    out
}

/// Asserts that `b` at `map(u)` matches `a` at `u` for `count` parameters
/// across `a`'s domain, to within 1e-9 of the size of `a`.
pub fn assert_same_geometry<H: Homogeneous>(
    a: &NurbsCurve<H>,
    b: &NurbsCurve<H>,
    map: impl Fn(f64) -> f64,
    count: usize,
) -> Result<(), TestCaseError> {
    let (s, e) = a.domain();
    let tol = 1e-9 * size(a);
    for u in params(s, e, count) {
        let d = dist::<H>(a.point(u), b.point(map(u)));
        prop_assert!(d <= tol, "curves differ by {d} at u = {u} (tol {tol})");
    }
    Ok(())
}
