// SPDX-License-Identifier: GPL-3.0-or-later

//! Dense linear algebra for the fitting code, on top of nalgebra.
//!
//! Every system has one right-hand side column per coordinate of the data
//! (two or three), so the points are stored as the rows of a matrix.

use nalgebra::DMatrix;

use super::FitPoint;
use crate::GeomError;

/// A square system whose smallest LU pivot is below this fraction of its
/// largest one is treated as singular. The collocation matrices of the
/// fitting code have ratios many orders of magnitude above this.
const PIVOT_RATIO_LIMIT: f64 = 1e-14;

/// A least-squares system whose smallest `R` diagonal entry is below this
/// fraction of the largest one is treated as rank deficient.
const RANK_RATIO_LIMIT: f64 = 1e-13;

/// The points as the rows of a matrix with one column per coordinate.
pub(super) fn points_to_matrix<P: FitPoint>(points: &[P]) -> DMatrix<f64> {
    DMatrix::from_fn(points.len(), P::DIM, |row, axis| {
        points.get(row).map_or(0.0, |p| p.coord(axis))
    })
}

/// The rows of a matrix as points; the inverse of [`points_to_matrix`].
pub(super) fn matrix_to_points<P: FitPoint>(m: &DMatrix<f64>) -> Vec<P> {
    m.row_iter()
        .map(|row| {
            let coords: Vec<f64> = row.iter().copied().collect();
            P::from_coords(&coords)
        })
        .collect()
}

/// Writes `value` at `(row, col)`; positions outside the matrix are ignored
/// (they cannot occur for the spans and counts the callers derive).
pub(super) fn set(m: &mut DMatrix<f64>, row: usize, col: usize, value: f64) {
    if let Some(entry) = m.get_mut((row, col)) {
        *entry = value;
    }
}

/// Smallest and largest magnitude on the diagonal of `m`.
fn diagonal_range(m: &DMatrix<f64>) -> (f64, f64) {
    m.diagonal()
        .iter()
        .map(|d| d.abs())
        .fold((f64::INFINITY, 0.0), |(lo, hi), d| (lo.min(d), hi.max(d)))
}

/// Solves `a x = b` for square `a` with LU factorisation and partial
/// pivoting.
///
/// # Errors
///
/// [`GeomError::NotConverged`] if `a` is singular to working precision (a
/// vanishing pivot, or a pivot ratio below 1e-14) or the solution is not
/// finite.
pub(super) fn solve_square(a: DMatrix<f64>, b: &DMatrix<f64>) -> Result<DMatrix<f64>, GeomError> {
    let lu = a.lu();
    let (lo, hi) = diagonal_range(&lu.u());
    if !(hi > 0.0 && lo > PIVOT_RATIO_LIMIT * hi) {
        return Err(GeomError::NotConverged);
    }
    let x = lu.solve(b).ok_or(GeomError::NotConverged)?;
    if x.iter().all(|v| v.is_finite()) {
        Ok(x)
    } else {
        Err(GeomError::NotConverged)
    }
}

/// Solves the least-squares problem `min |a x - b|` for a matrix `a` with at
/// least as many rows as columns, through a Householder QR factorisation:
/// `x = R^-1 Q^T b`.
///
/// # Errors
///
/// [`GeomError::NotConverged`] if `a` is rank deficient to working precision
/// (an `R` diagonal ratio below 1e-13) or the solution is not finite, and
/// [`GeomError::InvalidArgument`] if `a` has fewer rows than columns.
pub(super) fn solve_least_squares(
    a: DMatrix<f64>,
    b: &DMatrix<f64>,
) -> Result<DMatrix<f64>, GeomError> {
    if a.nrows() < a.ncols() {
        return Err(GeomError::InvalidArgument("fewer equations than unknowns"));
    }
    let qr = a.qr();
    let r = qr.r();
    let (lo, hi) = diagonal_range(&r);
    if !(hi > 0.0 && lo > RANK_RATIO_LIMIT * hi) {
        return Err(GeomError::NotConverged);
    }
    let qtb = qr.q().transpose() * b;
    let x = r
        .solve_upper_triangular(&qtb)
        .ok_or(GeomError::NotConverged)?;
    if x.iter().all(|v| v.is_finite()) {
        Ok(x)
    } else {
        Err(GeomError::NotConverged)
    }
}
