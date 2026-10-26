// SPDX-License-Identifier: GPL-3.0-or-later

//! Random point sets, degrees and parametrizations for the fitting tests.

use bevel_geom::fit::{FitPoint, Parametrization};
use bevel_math::{DVec2, DVec3};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};

/// A fixed-seed configuration so that runs are reproducible.
///
/// For exploratory runs, `BEVEL_PROPTEST_SEED` replaces the seed and
/// `BEVEL_PROPTEST_CASES` the number of cases.
pub fn config(cases: u32) -> Config {
    let env = |name: &str| std::env::var(name).ok().and_then(|v| v.parse::<u64>().ok());
    Config {
        cases: env("BEVEL_PROPTEST_CASES").map_or(cases, |c| u32::try_from(c).unwrap_or(cases)),
        rng_seed: RngSeed::Fixed(env("BEVEL_PROPTEST_SEED").unwrap_or(0x5eed_f171)),
        failure_persistence: None,
        ..Config::default()
    }
}

/// Consecutive points of random data are at least this far apart.
const MIN_SEPARATION: f64 = 0.5;

/// One of the three parametrization methods.
pub fn method() -> impl Strategy<Value = Parametrization> {
    prop_oneof![
        Just(Parametrization::Uniform),
        Just(Parametrization::ChordLength),
        Just(Parametrization::Centripetal),
    ]
}

/// A point of `[-100, 100]^2`.
fn point2() -> impl Strategy<Value = DVec2> {
    (-100.0f64..100.0, -100.0f64..100.0).prop_map(|(x, y)| DVec2::new(x, y))
}

/// A point of `[-100, 100]^3`.
fn point3() -> impl Strategy<Value = DVec3> {
    (-100.0f64..100.0, -100.0f64..100.0, -100.0f64..100.0).prop_map(|(x, y, z)| DVec3::new(x, y, z))
}

/// `min_len..=40` points from `point` whose consecutive points are distinct.
fn points<P: FitPoint>(
    point: impl Strategy<Value = P>,
    min_len: usize,
) -> impl Strategy<Value = Vec<P>> {
    prop::collection::vec(point, min_len..=40).prop_filter("coincident neighbours", |pts| {
        pts.windows(2)
            .all(|w| (w[1] - w[0]).length() >= MIN_SEPARATION)
    })
}

/// A degree from `degrees` and 4 to 40 planar points, at least `degree + 1`
/// of them.
pub fn degree_and_points2(
    degrees: std::ops::RangeInclusive<usize>,
) -> impl Strategy<Value = (usize, Vec<DVec2>)> {
    degrees.prop_flat_map(|p| points(point2(), (p + 1).max(4)).prop_map(move |pts| (p, pts)))
}

/// A degree from `degrees` and 4 to 40 points in space, at least `degree + 1`
/// of them.
pub fn degree_and_points3(
    degrees: std::ops::RangeInclusive<usize>,
) -> impl Strategy<Value = (usize, Vec<DVec3>)> {
    degrees.prop_flat_map(|p| points(point3(), (p + 1).max(4)).prop_map(move |pts| (p, pts)))
}

/// A non-trivial vector of the plane, used as an end tangent.
pub fn tangent2() -> impl Strategy<Value = DVec2> {
    (-50.0f64..50.0, -50.0f64..50.0)
        .prop_map(|(x, y)| DVec2::new(x, y))
        .prop_filter("short tangent", |t| t.length() >= 0.5)
}

/// A non-trivial vector of space, used as an end tangent.
pub fn tangent3() -> impl Strategy<Value = DVec3> {
    (-50.0f64..50.0, -50.0f64..50.0, -50.0f64..50.0)
        .prop_map(|(x, y, z)| DVec3::new(x, y, z))
        .prop_filter("short tangent", |t| t.length() >= 0.5)
}

/// The length scale of the data: the largest coordinate magnitude, at least 1.
pub fn size<P: FitPoint>(points: &[P]) -> f64 {
    points
        .iter()
        .flat_map(|&p| (0..P::DIM).map(move |axis| p.coord(axis).abs()))
        .fold(1.0, f64::max)
}
