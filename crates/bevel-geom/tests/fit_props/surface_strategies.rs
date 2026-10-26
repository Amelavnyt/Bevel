// SPDX-License-Identifier: GPL-3.0-or-later

//! Random grids of points sampled from smooth surfaces, for the surface
//! fitting tests.

use bevel_geom::{KnotVector, NurbsSurface};
use bevel_math::DVec3;
use proptest::prelude::*;

/// Consecutive grid points along either direction are at least this far
/// apart.
const MIN_SEPARATION: f64 = 0.05;

/// A rectangular grid of points with the degrees to fit it with.
#[derive(Clone, Debug)]
pub struct GridCase {
    /// `rows x cols` points, row-major (`k * cols + l`, `k` along `u`).
    pub grid: Vec<DVec3>,
    /// Number of rows (points along `u`).
    pub rows: usize,
    /// Number of columns (points along `v`).
    pub cols: usize,
    /// Degree in `u`, at most `rows - 1`.
    pub degree_u: usize,
    /// Degree in `v`, at most `cols - 1`.
    pub degree_v: usize,
}

impl GridCase {
    /// The length scale of the data: the largest coordinate magnitude, at
    /// least 1.
    pub fn size(&self) -> f64 {
        crate::strategies::size(&self.grid)
    }
}

/// `count` increasing coordinates from 0 with random gaps.
fn irregular_axis(count: usize) -> impl Strategy<Value = Vec<f64>> {
    prop::collection::vec(0.5f64..2.0, count).prop_map(|gaps| {
        let mut x = 0.0;
        gaps.into_iter()
            .map(|g| {
                x += g;
                x
            })
            .collect()
    })
}

/// A height field `z = f(x, y)` over an irregular rectangular grid.
fn height_field(rows: usize, cols: usize) -> impl Strategy<Value = Vec<DVec3>> {
    (
        irregular_axis(rows),
        irregular_axis(cols),
        prop::array::uniform4(0.0f64..8.0),
        prop::array::uniform4(0.1f64..1.5),
        prop::array::uniform2(0.0f64..std::f64::consts::TAU),
    )
        .prop_map(move |(xs, ys, amp, freq, phase)| {
            xs.iter()
                .flat_map(|&x| {
                    ys.iter().map(move |&y| {
                        let z = amp[0]
                            * (freq[0] * x + phase[0]).sin()
                            * (freq[1] * y + phase[1]).cos()
                            + amp[1] * (freq[2] * x).cos()
                            + amp[2] * (freq[3] * y).sin()
                            + amp[3] * 0.02 * x * y;
                        DVec3::new(x, y, z)
                    })
                })
                .collect()
        })
}

/// A random non-rational surface of degrees 2 or 3 with 4 to 6 control
/// points per direction, sampled on a uniform parameter grid.
fn sampled_surface(rows: usize, cols: usize) -> impl Strategy<Value = Vec<DVec3>> {
    (2usize..=3, 2usize..=3, 4usize..=6, 4usize..=6)
        .prop_flat_map(|(p, q, nu, nv)| {
            (
                Just((p, q, nu, nv)),
                prop::collection::vec(prop::array::uniform3(-50.0f64..50.0), nu * nv),
            )
        })
        .prop_map(move |((p, q, nu, nv), coords)| {
            let pts: Vec<DVec3> = coords.into_iter().map(DVec3::from_array).collect();
            let ku = KnotVector::clamped_uniform(p, nu).unwrap();
            let kv = KnotVector::clamped_uniform(q, nv).unwrap();
            let s = NurbsSurface::from_points(p, q, ku, kv, &pts, None).unwrap();
            let ((ua, ub), (va, vb)) = s.domain();
            (0..rows)
                .flat_map(|k| {
                    let s = &s;
                    (0..cols).map(move |l| {
                        s.point(
                            ua + (ub - ua) * k as f64 / (rows - 1) as f64,
                            va + (vb - va) * l as f64 / (cols - 1) as f64,
                        )
                    })
                })
                .collect()
        })
}

/// Whether neighbouring grid points are all well separated.
fn separated(grid: &[DVec3], rows: usize, cols: usize) -> bool {
    let far = |a: DVec3, b: DVec3| (a - b).length() >= MIN_SEPARATION;
    (0..rows).all(|k| (1..cols).all(|l| far(grid[k * cols + l - 1], grid[k * cols + l])))
        && (1..rows).all(|k| (0..cols).all(|l| far(grid[(k - 1) * cols + l], grid[k * cols + l])))
}

/// A grid of 4 to 12 rows and columns with degrees from 1 to 5 (capped by
/// `rows - 1` and `cols - 1`, so degrees above 3 only occur on the larger
/// grids).
pub fn grid_case() -> impl Strategy<Value = GridCase> {
    (4usize..=12, 4usize..=12, 1usize..=5, 1usize..=5)
        .prop_flat_map(|(rows, cols, du, dv)| {
            let grid = prop_oneof![height_field(rows, cols), sampled_surface(rows, cols)];
            (Just((rows, cols, du, dv)), grid)
        })
        .prop_filter("coincident neighbours", |((rows, cols, _, _), grid)| {
            separated(grid, *rows, *cols)
        })
        .prop_map(|((rows, cols, du, dv), grid)| GridCase {
            grid,
            rows,
            cols,
            degree_u: du.min(rows - 1),
            degree_v: dv.min(cols - 1),
        })
}
