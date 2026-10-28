// SPDX-License-Identifier: GPL-3.0-or-later

//! Curve–surface intersection.
//!
//! Lines against planes, spheres, cylinders, cones and tori, and circles
//! against planes, take closed-form fast paths. Every other pair goes
//! through the general method: recursive subdivision of the curve's range
//! and the surface's rectangle with box culling, Newton refinement of the
//! three equations `c(t) = S(u, v)`, and growing each solution into the
//! stretch of the curve that stays within the tolerance of the surface,
//! which tells crossings and touches (points) from stretches lying on the
//! surface (overlaps).

mod analytic;
mod general;

use super::curve_curve::common::{Found, check_range, check_tol};
use super::{CurveSurfaceHit, Rect};
use crate::{Curve3, CurveGeom, GeomError, Surface, SurfaceGeom};
use bevel_math::Interval;

/// Where `c` over `rc` meets `s` over `rs`.
///
/// Points are reported where the curve comes within `tol` (a distance in
/// model units) of the surface patch, tangential touches included, refined
/// by Newton to full precision where it converges; the ends of the range
/// and the sides of the rectangle count. Where the curve lies on the patch
/// (within `tol`) along a stretch longer than `tol`, one
/// [`CurveSurfaceHit::Overlap`] is reported instead (with no points inside
/// it). Hits are sorted by the curve parameter, then `u`, and duplicates
/// within `tol` are merged; a periodic direction of the rectangle longer
/// than its period covers the surface more than once, and each pass
/// counts.
///
/// # Errors
///
/// [`GeomError::NonFinite`] for a non-finite range, rectangle or
/// tolerance, [`GeomError::InvalidArgument`] for a tolerance that is not
/// positive or a range without `min < max`,
/// [`GeomError::ParameterOutOfDomain`] for a range outside a bounded
/// domain (a NURBS, or the latitudes of a sphere), and
/// [`GeomError::NotConverged`] if the subdivision of the general method
/// exceeds its depth or work limit.
pub fn curve_surface(
    c: &Curve3,
    rc: Interval,
    s: &Surface,
    rs: Rect,
    tol: f64,
) -> Result<Vec<CurveSurfaceHit>, GeomError> {
    check_tol(tol)?;
    let rc = check_range(rc, c.domain())?;
    let rs = Rect::new(
        check_range(rs.u, s.domain_u())?,
        check_range(rs.v, s.domain_v())?,
    );
    let found = match analytic::curve_surface(c, rc, s, rs, tol) {
        Some(found) => found,
        None => general::intersect(c, rc, s, rs, tol)?,
    };
    Ok(found
        .into_iter()
        .map(|f| match f {
            Found::Point(r) => CurveSurfaceHit::Point {
                t: r.s,
                u: r.x[0],
                v: r.x[1],
                point: CurveGeom::point(c, r.s),
            },
            Found::Overlap { lo, hi, .. } => CurveSurfaceHit::Overlap {
                t: Interval { min: lo, max: hi },
            },
        })
        .collect())
}
