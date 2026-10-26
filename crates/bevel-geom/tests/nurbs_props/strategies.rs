// SPDX-License-Identifier: GPL-3.0-or-later

//! Random valid knot vectors and curves for the property tests.

use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};

/// A fixed-seed configuration so that runs are reproducible.
pub fn config(cases: u32) -> Config {
    Config {
        cases,
        rng_seed: RngSeed::Fixed(0x5eed_b0e1),
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
    (1usize..=5, 2usize..=10)
        .prop_flat_map(|(p, extra)| clamped_knots(p, extra).prop_map(move |k| (p, k)))
}

/// `count` parameters spread evenly over `[a, b]`, ends included.
pub fn params(a: f64, b: f64, count: usize) -> Vec<f64> {
    (0..count)
        .map(|i| a + (b - a) * i as f64 / (count - 1) as f64)
        .collect()
}
