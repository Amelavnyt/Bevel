// SPDX-License-Identifier: GPL-3.0-or-later

//! Curve–curve intersection.
//!
//! Lines and circles take closed-form fast paths (2D line–line,
//! line–circle and circle–circle; 3D line–line, line–circle and
//! circle–circle, coplanar or not). Every other pair goes through the
//! general method: recursive subdivision of both ranges with box culling,
//! Newton refinement of the surviving pairs, and growing each solution into
//! the stretch where the curves stay within the tolerance, which tells
//! crossings and touches (points) from coincident stretches (overlaps).

mod analytic;
pub(super) mod common;
mod general;

use super::CurveCurveHit;
use crate::{Curve2, Curve3, CurveGeom, GeomError};
use bevel_math::{Interval, Point2, Point3};
use common::{Found, check_range, check_tol};

/// Where `a` over `ra` meets `b` over `rb`, in the plane.
///
/// Points are reported where the curves come within `tol` (a distance in
/// model units) of each other, tangential touches included, refined by
/// Newton to full precision where it converges; an endpoint of a range
/// counts. Where the curves coincide within `tol` along a stretch longer
/// than `tol`, one [`CurveCurveHit::Overlap`] is reported instead (with no
/// points inside it). Hits are sorted by the parameter on `a`, then on `b`,
/// and duplicates within `tol` are merged. A range longer than the period
/// of a circle covers it more than once, and each pass counts.
///
/// # Errors
///
/// [`GeomError::NonFinite`] for a non-finite range or tolerance,
/// [`GeomError::InvalidArgument`] for a tolerance that is not positive or
/// a range without `min < max`, [`GeomError::ParameterOutOfDomain`] for a
/// range outside a NURBS curve's domain, and [`GeomError::NotConverged`] if
/// the subdivision of the general method exceeds its depth or work limit.
pub fn curve_curve_2d(
    a: &Curve2,
    ra: Interval,
    b: &Curve2,
    rb: Interval,
    tol: f64,
) -> Result<Vec<CurveCurveHit<Point2>>, GeomError> {
    check_tol(tol)?;
    let ra = check_range(ra, a.domain())?;
    let rb = check_range(rb, b.domain())?;
    let found = match analytic::curves_2d(a, ra, b, rb, tol) {
        Some(found) => found,
        None => general::intersect(a, ra, b, rb, tol)?,
    };
    Ok(hits(found, |s| CurveGeom::point(a, s)))
}

/// Where `a` over `ra` meets `b` over `rb`, in space.
///
/// As [`curve_curve_2d`]; skew curves passing within `tol` of each other
/// meet.
///
/// # Errors
///
/// As [`curve_curve_2d`].
pub fn curve_curve_3d(
    a: &Curve3,
    ra: Interval,
    b: &Curve3,
    rb: Interval,
    tol: f64,
) -> Result<Vec<CurveCurveHit<Point3>>, GeomError> {
    check_tol(tol)?;
    let ra = check_range(ra, a.domain())?;
    let rb = check_range(rb, b.domain())?;
    let found = match analytic::curves_3d(a, ra, b, rb, tol) {
        Some(found) => found,
        None => general::intersect(a, ra, b, rb, tol)?,
    };
    Ok(hits(found, |s| CurveGeom::point(a, s)))
}

/// The public hits for the found points and overlaps.
fn hits<P>(found: Vec<Found<1>>, point: impl Fn(f64) -> P) -> Vec<CurveCurveHit<P>> {
    found
        .into_iter()
        .map(|f| match f {
            Found::Point(r) => CurveCurveHit::Point {
                a: r.s,
                b: r.x[0],
                point: point(r.s),
            },
            Found::Overlap { lo, hi, x_lo, x_hi } => CurveCurveHit::Overlap {
                a: Interval { min: lo, max: hi },
                b: Interval::new(x_lo[0], x_hi[0]),
                reversed: x_hi[0] < x_lo[0],
            },
        })
        .collect()
}
