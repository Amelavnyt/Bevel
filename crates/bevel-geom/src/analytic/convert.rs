// SPDX-License-Identifier: GPL-3.0-or-later

//! Exact NURBS forms of the analytic types and the maps between their
//! parameters.
//!
//! Every analytic curve and surface converts exactly to a NURBS over a
//! parameter range (unbounded and periodic geometry needs one):
//!
//! - lines become degree-1 curves and planes bilinear patches; the NURBS
//!   parameters are the analytic ones ([`ParamMap::Identity`]);
//! - circles and ellipses become the rational quadratic arcs of
//!   [`circle_arc`](crate::circle_arc) and [`ellipse_arc`](crate::ellipse_arc)
//!   (for 2D curves the same construction in the curve's own axes);
//! - cylinders and cones revolve a line segment, spheres a meridian arc and
//!   tori a tube circle (or arc) with
//!   [`NurbsSurface::revolution`](crate::NurbsSurface::revolution); `u` is
//!   the direction of rotation and `v` the profile's.
//!
//! # Parameters of the rational arcs
//!
//! No rational parametrisation of a circle is linear in the angle, so a
//! NURBS arc's parameter `s` and the analytic angle `t` agree only at the
//! knots: the arc over `[t0, t1]` is cut into `n` equal segments of at most
//! 90 degrees with the knots `b_i = t0 + i (t1 - t0) / n` (each a double
//! knot), and the domain is `[t0, t1]` itself. Inside the segment
//! `[b_i, b_(i+1)]` of width `h`, with `tau = (s - b_i) / h`, the rational
//! quadratic Bezier arc with weights `1, cos(h / 2), 1` runs through the
//! angle
//!
//! `t = b_i + h / 2 + 2 atan(tan(h / 4) (2 tau - 1))`,
//!
//! and back, `tau = (1 + tan((t - b_i - h / 2) / 2) / tan(h / 4)) / 2`.
//! (The arc is the image of the half-angle substitution `tan((t - m) / 2)`,
//! which is affine in `tau` for a symmetric segment.) Both directions are
//! exact up to rounding; [`ParamMap::Arc`] carries the knots `b_i` and
//! evaluates them. For an ellipse the same map holds for the eccentric
//! angle, since the ellipse is an affine image of the circle.
//!
//! On surfaces the angular directions (`u` of every surface of revolution,
//! `v` of spheres and tori) map this way; the linear ones (`v` of cylinders
//! and cones, both directions of planes) are the identity.

use bevel_math::{Frame3, Interval, Point2, Vector2};
use serde::{Deserialize, Serialize};

use super::check_range;
use crate::conics::unit_arc;
use crate::{
    GeomError, Homogeneous, KnotVector, NurbsCurve, NurbsCurve2, NurbsCurve3, NurbsSurface,
};

/// How the parameter of a NURBS form relates to the analytic parameter it
/// was made from (see the module documentation).
///
/// Deserialization requires the breaks of an arc to be finite and strictly
/// increasing, with at least two of them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "ParamMapRaw")]
pub enum ParamMap {
    /// The NURBS parameter is the analytic one.
    Identity,
    /// A rational quadratic arc: the parameters agree at the segment
    /// boundaries `breaks` (the distinct knots, increasing, from the start
    /// to the end of the range) and follow the half-angle map in between.
    Arc {
        /// The segment boundaries, where both parameters are equal.
        breaks: Vec<f64>,
    },
}

/// The serialized form of a [`ParamMap`], checked before it becomes one.
#[derive(Deserialize)]
enum ParamMapRaw {
    Identity,
    Arc { breaks: Vec<f64> },
}

impl TryFrom<ParamMapRaw> for ParamMap {
    type Error = GeomError;

    fn try_from(raw: ParamMapRaw) -> Result<Self, GeomError> {
        match raw {
            ParamMapRaw::Identity => Ok(Self::Identity),
            ParamMapRaw::Arc { breaks } => {
                if breaks.iter().any(|b| !b.is_finite()) {
                    return Err(GeomError::NonFinite);
                }
                if breaks.len() < 2 || breaks.windows(2).any(|w| w[1] <= w[0]) {
                    return Err(GeomError::InvalidArgument(
                        "the breaks of an arc map must be at least two increasing values",
                    ));
                }
                Ok(Self::Arc { breaks })
            }
        }
    }
}

impl ParamMap {
    /// The map of a rational arc on the knot vector `knots`.
    pub(crate) fn arc(knots: &KnotVector) -> Self {
        Self::Arc {
            breaks: knots.breakpoints(),
        }
    }

    /// The NURBS parameter of the point at analytic parameter `t`. For an
    /// arc, `t` is clamped to the range first.
    pub fn nurbs_parameter(&self, t: f64) -> f64 {
        match self {
            Self::Identity => t,
            Self::Arc { breaks } => match segment(breaks, t) {
                Some((t, b0, h)) => {
                    let psi = t - b0 - 0.5 * h;
                    let tau = 0.5 * (1.0 + (0.5 * psi).tan() / (0.25 * h).tan());
                    b0 + tau * h
                }
                None => t,
            },
        }
    }

    /// The analytic parameter of the point at NURBS parameter `s`. For an
    /// arc, `s` is clamped to the domain first.
    pub fn analytic_parameter(&self, s: f64) -> f64 {
        match self {
            Self::Identity => s,
            Self::Arc { breaks } => match segment(breaks, s) {
                Some((s, b0, h)) => {
                    let tau = (s - b0) / h;
                    b0 + 0.5 * h + 2.0 * ((0.25 * h).tan() * (2.0 * tau - 1.0)).atan()
                }
                None => s,
            },
        }
    }
}

/// `x` clamped to `[breaks[0], breaks[n]]`, with the start and the width of
/// the segment that contains it; `None` for fewer than two breaks or a
/// non-finite `x`.
fn segment(breaks: &[f64], x: f64) -> Option<(f64, f64, f64)> {
    let (&first, &last) = (breaks.first()?, breaks.last()?);
    if breaks.len() < 2 || !x.is_finite() {
        return None;
    }
    let x = x.clamp(first, last);
    let w = breaks
        .windows(2)
        .find(|w| x <= w[1])
        .or_else(|| breaks.windows(2).last())?;
    Some((x, w[0], w[1] - w[0]))
}

/// A curve converted to NURBS, with the parameter map.
#[derive(Clone, Debug, PartialEq)]
pub struct ConvertedCurve<H: Homogeneous> {
    /// The exact NURBS form over the requested range.
    pub nurbs: NurbsCurve<H>,
    /// How its parameter relates to the original one.
    pub map: ParamMap,
}

/// A surface converted to NURBS, with the parameter maps of both
/// directions.
#[derive(Clone, Debug, PartialEq)]
pub struct ConvertedSurface {
    /// The exact NURBS form over the requested rectangle.
    pub nurbs: NurbsSurface,
    /// How its `u` relates to the original `u`.
    pub map_u: ParamMap,
    /// How its `v` relates to the original `v`.
    pub map_v: ParamMap,
}

/// Checks that a range is finite and of positive length.
pub(crate) fn check_open_range(range: Interval) -> Result<(), GeomError> {
    check_range(range)?;
    if range.length() > 0.0 {
        Ok(())
    } else {
        Err(GeomError::InvalidArgument(
            "a parameter range must have positive length",
        ))
    }
}

/// The degree-1 segment from `a` (at `range.min`) to `b` (at `range.max`).
pub(crate) fn segment_curve<H: Homogeneous>(
    a: H::Point,
    b: H::Point,
    range: Interval,
) -> Result<NurbsCurve<H>, GeomError> {
    check_open_range(range)?;
    let knots = KnotVector::clamped_from_interior(1, range.min, range.max, &[])?;
    NurbsCurve::from_points(1, knots, &[a, b], None)
}

/// The 2D arc of `c + a cos(t) x + b sin(t) y` over `range`: the unit
/// circle arc of A7.1 mapped by that affine map (exact), on the domain
/// `range`.
pub(crate) fn ellipse_arc_axes_2d(
    c: Point2,
    x: Vector2,
    y: Vector2,
    a: f64,
    b: f64,
    range: Interval,
) -> Result<NurbsCurve2, GeomError> {
    check_range(range)?;
    let arc = unit_arc(range.min, range.max)?;
    let points: Vec<Point2> = arc
        .points
        .iter()
        .map(|p| c + x * (a * p.x) + y * (b * p.y))
        .collect();
    NurbsCurve2::from_points(2, arc.knots, &points, Some(&arc.weights))
}

/// The surface swept by revolving `profile` (placed in the half-plane at
/// angle `u.min`) about the `z` axis of `frame` from `u.min` to `u.max`,
/// with its domain in `u` moved from `[0, u.max - u.min]` to `u`, so that
/// the NURBS parameter equals the angle at the knots.
pub(crate) fn revolve(
    frame: &Frame3,
    profile: &NurbsCurve3,
    u: Interval,
) -> Result<NurbsSurface, GeomError> {
    check_open_range(u)?;
    let s = NurbsSurface::revolution(profile, frame.origin, frame.z, u.length())?;
    let old = s.knots_u().as_slice();
    let last = old.len().saturating_sub(3);
    let knots: Vec<f64> = old
        .iter()
        .enumerate()
        .map(|(i, &k)| {
            if i < 3 {
                u.min
            } else if i >= last {
                u.max
            } else {
                u.min + k
            }
        })
        .collect();
    let knots_u = KnotVector::new(knots, 2)?;
    NurbsSurface::new(
        2,
        s.degree_v(),
        knots_u,
        s.knots_v().clone(),
        s.control_weighted().to_vec(),
    )
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{PI, TAU};

    use super::*;
    use crate::circle_arc;

    #[test]
    fn arc_map_inverts_and_matches_the_nurbs_circle() {
        for (t0, t1) in [(0.0, TAU), (-1.0, 0.5), (2.0, 2.0 + 1.5 * PI)] {
            let c = circle_arc(&Frame3::WORLD, 1.0, t0, t1).unwrap();
            let map = ParamMap::arc(c.knots());
            for i in 0..=50 {
                let s = t0 + (t1 - t0) * f64::from(i) / 50.0;
                let t = map.analytic_parameter(s);
                let p = c.point(s);
                assert!((p.x - t.cos()).abs() < 1e-15 && (p.y - t.sin()).abs() < 1e-15);
                assert!((map.nurbs_parameter(t) - s).abs() < 1e-14);
            }
            // Clamped outside the range.
            assert!((map.analytic_parameter(t1 + 1.0) - t1).abs() < 1e-15);
        }
        assert!((ParamMap::Identity.nurbs_parameter(3.5) - 3.5).abs() < 1e-300);
        let empty = ParamMap::Arc { breaks: vec![] };
        assert!((empty.analytic_parameter(1.5) - 1.5).abs() < 1e-300);
    }
}
