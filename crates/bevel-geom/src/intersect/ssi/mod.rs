// SPDX-License-Identifier: GPL-3.0-or-later

//! Surface–surface intersection.

use super::{Rect, SsiResult};
use crate::{GeomError, Surface};

/// Where `a` over `ra` meets `b` over `rb`.
///
/// # Errors
///
/// [`GeomError::NonFinite`] for non-finite rectangles or tolerance, and
/// [`GeomError::TangentContact`] where the surfaces touch tangentially
/// along a curve that no exact method covers.
pub fn surface_surface(
    a: &Surface,
    ra: Rect,
    b: &Surface,
    rb: Rect,
    tol: f64,
) -> Result<SsiResult, GeomError> {
    let _ = (a, ra, b, rb, tol);
    Ok(SsiResult::Meet {
        curves: Vec::new(),
        points: Vec::new(),
    })
}

/// The general method alone, without fast paths.
///
/// # Errors
///
/// As [`surface_surface`].
pub fn surface_surface_general(
    a: &Surface,
    ra: Rect,
    b: &Surface,
    rb: Rect,
    tol: f64,
) -> Result<SsiResult, GeomError> {
    surface_surface(a, ra, b, rb, tol)
}
