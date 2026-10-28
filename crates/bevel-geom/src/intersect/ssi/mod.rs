// SPDX-License-Identifier: GPL-3.0-or-later

//! Surface–surface intersection (SSI).
//!
//! Analytic pairs with closed-form answers go through exact fast paths
//! ([`analytic`]); everything else, and every pair in the cross-checking
//! tests, goes through the general method ([`general`]). Coincident
//! surfaces are recognised before either runs.

mod analytic;
mod general;
mod pieces;

use super::{Rect, SsiResult};
use crate::{GeomError, Surface, SurfaceGeom};
use bevel_math::Point2;
use general::Problem;

/// Where `a` over `ra` meets `b` over `rb`.
///
/// # Errors
///
/// [`GeomError::InvalidArgument`] for a tolerance that is not positive,
/// [`GeomError::NonFinite`] for non-finite rectangles or tolerance,
/// [`GeomError::TangentContact`] where the surfaces touch tangentially
/// along a curve that no exact method covers, and
/// [`GeomError::NotConverged`] or [`GeomError::ToleranceNotMet`] if the
/// general method fails.
pub fn surface_surface(
    a: &Surface,
    ra: Rect,
    b: &Surface,
    rb: Rect,
    tol: f64,
) -> Result<SsiResult, GeomError> {
    check(ra, rb, tol)?;
    let ba = a.bounds(ra.u, ra.v)?;
    let bb = b.bounds(rb.u, rb.v)?;
    let grow = |x: bevel_math::Aabb3| {
        let g = bevel_math::Vector3::splat(tol);
        bevel_math::Aabb3::new(x.min - g, x.max + g)
    };
    if !grow(ba).intersects(&grow(bb)) {
        return Ok(SsiResult::Meet {
            curves: Vec::new(),
            points: Vec::new(),
        });
    }
    // Lines are cut to where both patches can be.
    let lo = ba.min.max(bb.min);
    let hi = ba.max.min(bb.max);
    let common = bevel_math::Aabb3::new(lo.min(hi), hi.max(lo));
    match analytic::exact(a, b, &common, tol) {
        Some(Ok(analytic::Exact::Coincident(same_sense))) => {
            Ok(SsiResult::Coincident { same_sense })
        }
        Some(Ok(analytic::Exact::Curves(found, touching))) => {
            let pair = pieces::Pair { a, ra, b, rb };
            let mut curves = Vec::new();
            for (curve, range, contact) in &found {
                curves.extend(pieces::clip(curve, *range, *contact, &pair, tol)?);
            }
            sort_curves(&mut curves);
            let points = pieces::points(&touching, a, &ra, b, &rb);
            Ok(SsiResult::Meet { curves, points })
        }
        Some(Err(e)) => Err(e),
        None => general_method(a, ra, b, rb, tol),
    }
}

/// The pcurve on `s` of `curve` over `range`, a curve lying on `s`, with
/// the curve's own parameter: `s` at the pcurve's point equals `curve`'s
/// point within the returned error bound. Exact for lines and conics on
/// planes and wherever the surface parameters change linearly along the
/// curve; otherwise a cubic refined until it maps within half of `tol`.
/// The parameters start near the middle of `rect` and are followed
/// continuously from there.
///
/// # Errors
///
/// [`GeomError::NotConverged`] if a point cannot be inverted onto `s`, and
/// [`GeomError::ToleranceNotMet`] if no cubic of reasonable size fits.
pub fn pcurve_on(
    curve: &crate::Curve3,
    range: bevel_math::Interval,
    s: &Surface,
    rect: Rect,
    tol: f64,
) -> Result<(crate::Curve2, f64), GeomError> {
    pieces::pcurve(curve, range, s, &rect, tol)
}

fn check(ra: Rect, rb: Rect, tol: f64) -> Result<(), GeomError> {
    let values = [
        ra.u.min, ra.u.max, ra.v.min, ra.v.max, rb.u.min, rb.u.max, rb.v.min, rb.v.max, tol,
    ];
    if values.iter().any(|v| !v.is_finite()) {
        return Err(GeomError::NonFinite);
    }
    if tol <= 0.0 {
        return Err(GeomError::InvalidArgument("the tolerance must be positive"));
    }
    if [ra.u, ra.v, rb.u, rb.v].iter().any(|r| r.length() <= 0.0) {
        return Err(GeomError::InvalidArgument(
            "parameter rectangles must not be empty",
        ));
    }
    Ok(())
}

/// The general method alone, without fast paths; tests cross-check the
/// fast paths against it.
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
    check(ra, rb, tol)?;
    general_method(a, ra, b, rb, tol)
}

fn general_method(
    a: &Surface,
    ra: Rect,
    b: &Surface,
    rb: Rect,
    tol: f64,
) -> Result<SsiResult, GeomError> {
    let ba = a.bounds(ra.u, ra.v)?;
    let bb = b.bounds(rb.u, rb.v)?;
    let size = (ba.max - ba.min)
        .length()
        .max((bb.max - bb.min).length())
        .max(tol);
    let pr = Problem {
        a,
        ra,
        b,
        rb,
        tol,
        size,
    };
    if let Some(same_sense) = coincident(&pr) {
        return Ok(SsiResult::Coincident { same_sense });
    }
    let seeds = general::seeds(&pr)?;
    let (branches, candidates) = general::branches(&pr, &seeds)?;
    let points = general::touching(&pr, &candidates, &branches)?;
    let mut curves = Vec::new();
    for branch in &branches {
        if let Some(c) = general::fit_branch(&pr, branch)? {
            curves.push(c);
        }
    }
    sort_curves(&mut curves);
    Ok(SsiResult::Meet { curves, points })
}

/// Orders curves by their start points, so results do not depend on the
/// order seeds were found in.
fn sort_curves(curves: &mut [super::SsiCurve]) {
    use crate::CurveGeom as _;
    curves.sort_by(|x, y| {
        let (p, q) = (x.curve.point(x.range.min), y.curve.point(y.range.min));
        p.x.total_cmp(&q.x)
            .then(p.y.total_cmp(&q.y))
            .then(p.z.total_cmp(&q.z))
    });
}

/// Whether the patches lie on one surface where they overlap: every point
/// of a grid over `a` whose nearest point on `b` falls in `b`'s rectangle
/// lies on `b`, with parallel normals, and there are enough such points to
/// span an area. Returns whether the normals agree.
fn coincident(pr: &Problem<'_>) -> Option<bool> {
    const N: usize = 9;
    let mut on = 0;
    let mut same = 0;
    for i in 0..N {
        for j in 0..N {
            let fu = (i as f64 + 0.5) / N as f64;
            let fv = (j as f64 + 0.5) / N as f64;
            let ua = pr.ra.u.min + fu * pr.ra.u.length();
            let va = pr.ra.v.min + fv * pr.ra.v.length();
            let p = pr.a.point(ua, va);
            let Ok(c) = pr.b.closest_point(p) else {
                continue;
            };
            let Some(cb) = in_rect(pr.b, &pr.rb, Point2::new(c.u, c.v)) else {
                continue;
            };
            if c.distance > pr.tol {
                return None;
            }
            let (Some(na), Some(nb)) = (pr.a.normal(ua, va), pr.b.normal(cb.x, cb.y)) else {
                return None;
            };
            if na.cross(nb).length() > 1e-6 {
                return None;
            }
            on += 1;
            if na.dot(nb) > 0.0 {
                same += 1;
            }
        }
    }
    (on >= 4).then_some(2 * same >= on)
}

/// `p`, shifted by whole periods where `s` is periodic, if it then lies in
/// `r`.
fn in_rect(s: &Surface, r: &Rect, p: Point2) -> Option<Point2> {
    let shift = |x: f64, range: bevel_math::Interval, period: Option<f64>| match period {
        Some(per) if per > 0.0 => {
            let k = ((range.min - x) / per).ceil();
            let y = x + k * per;
            range.contains_with(y, 1e-9).then_some(y)
        }
        _ => range.contains_with(x, 1e-9).then_some(x),
    };
    Some(Point2::new(
        shift(p.x, r.u, s.period_u())?,
        shift(p.y, r.v, s.period_v())?,
    ))
}
