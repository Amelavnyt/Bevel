// SPDX-License-Identifier: GPL-3.0-or-later

//! Analytic curves and surfaces: lines, circles, ellipses, planes,
//! cylinders, cones, spheres and tori.
//!
//! These keep their identity (a circle stays a circle with a centre and a
//! radius) so that intersections, measurements and exports can be exact.
//! All of them are unbounded or periodic: trims and parameter ranges live
//! in the topology, and every operation that needs bounds (bounding boxes,
//! conversion to NURBS) takes the parameter range as an argument.
//!
//! # Parametrizations
//!
//! Lengths are in millimetres and angles in radians. Every 3D type is
//! placed by a right-handed orthonormal [`Frame3`] `(o, x, y, z)`; `e(u) =
//! cos(u) x + sin(u) y` is the unit vector at angle `u` from `x`
//! counter-clockwise about `z`.
//!
//! Curves:
//!
//! - [`Line2`], [`Line3`]: `C(t) = o + t d` with a unit direction `d`, so
//!   `t` is the signed distance from the origin `o`. Unbounded.
//! - [`Circle2`], [`Circle3`]: `C(t) = c + r (cos(t) x + sin(t) y)`, `t` the
//!   angle from the frame's `x` axis, counter-clockwise about its `z` axis
//!   in 3D. A [`Circle2`] carries its own axes and may run clockwise.
//!   Periodic with period `2 pi`.
//! - [`Ellipse2`], [`Ellipse3`]: `C(t) = c + a cos(t) x + b sin(t) y` with
//!   the major radius `a` along `x` and the minor radius `b <= a` along
//!   `y`; `t` is the eccentric angle. Periodic with period `2 pi`.
//!
//! Surfaces (each module documents its normal, curvatures and degenerate
//! closest points):
//!
//! - [`Plane`]: `S(u, v) = o + u x + v y`, both unbounded; normal `z`.
//! - [`Cylinder`]: `S(u, v) = o + r e(u) + v z`; `u` periodic, `v` the
//!   height, unbounded.
//! - [`Cone`]: `S(u, v) = o + (r0 + v tan(alpha)) e(u) + v z` with the
//!   radius `r0` at `v = 0` and the half-angle `alpha` in `(0, pi / 2)`;
//!   `v` is the distance along the axis, unbounded (the double cone), and
//!   the apex is at `v = -r0 / tan(alpha)`.
//! - [`Sphere`]: `S(u, v) = o + r (cos(v) e(u) + sin(v) z)`; `u` the
//!   longitude, periodic, `v` the latitude in `[-pi / 2, pi / 2]`.
//! - [`Torus`]: `S(u, v) = o + (R + r cos(v)) e(u) + r sin(v) z` with
//!   `R > r > 0`; both angles periodic.
//!
//! The normals of the closed surfaces (and of cylinders and cones) point
//! away from the axis or centre; that of a plane is its frame's `z` axis.
//!
//! The domain of a curve, or of a surface in each direction, is a
//! [`Domain`]. Evaluation accepts any finite parameter: periodic
//! parameters repeat, unbounded ones extend forever.
//!
//! # Conventions shared with the NURBS types
//!
//! Derivatives come in the layout of [`NurbsCurve::derivatives`]
//! (`result[k]` is the `k`-th derivative, capped at
//! [`MAX_DERIVATIVE_ORDER`]), closest points are
//! [`ClosestPoint`]s, and the signed curvature of a planar curve is
//! positive where it turns counter-clockwise, as for
//! [`NurbsCurve2::curvature`]. Surface derivatives come in the layout of
//! [`NurbsSurface::derivatives`], closest points are
//! [`SurfaceClosestPoint`]s, and principal curvatures are [`Curvatures`]
//! with the same sign convention as
//! [`NurbsSurface::principal_curvatures`]: positive where the surface bends
//! towards the normal, so a sphere with its outward normal has both
//! curvatures `-1 / r`.
//!
//! [`NurbsCurve::derivatives`]: crate::NurbsCurve::derivatives
//! [`NurbsCurve2::curvature`]: crate::NurbsCurve2::curvature
//! [`MAX_DERIVATIVE_ORDER`]: crate::MAX_DERIVATIVE_ORDER
//! [`ClosestPoint`]: crate::ClosestPoint
//! [`NurbsSurface::derivatives`]: crate::NurbsSurface::derivatives
//! [`NurbsSurface::principal_curvatures`]: crate::NurbsSurface::principal_curvatures
//! [`SurfaceClosestPoint`]: crate::SurfaceClosestPoint
//! [`Curvatures`]: crate::Curvatures

pub mod convert;

mod circle;
mod cone;
mod cylinder;
mod ellipse;
mod line;
mod plane;
mod revolved;
mod sphere;
mod torus;
pub mod transform;

pub use circle::{Circle2, Circle3};
pub use cone::Cone;
pub use convert::{ConvertedCurve, ConvertedSurface, ParamMap};
pub use cylinder::Cylinder;
pub use ellipse::{Ellipse2, Ellipse3};
pub use line::{Line2, Line3};
pub use plane::Plane;
pub use sphere::Sphere;
pub use torus::Torus;

use std::f64::consts::{PI, TAU};

use bevel_math::{Frame3, Interval, Point2, Point3, Vector2, Vector3};

use crate::{GeomError, MAX_DERIVATIVE_ORDER};

/// Tolerance on the orthonormality of a frame's axes, as for
/// [`circle_arc`](crate::circle_arc).
const FRAME_TOLERANCE: f64 = 1e-9;

/// The parameter domain of a curve, or of a surface in one direction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Domain {
    /// The parameter is restricted to the interval (NURBS).
    Bounded(Interval),
    /// The geometry repeats with the length of the interval as its period;
    /// the interval is the base period (`[0, 2 pi]` for every analytic
    /// angle) and any finite parameter is valid.
    Periodic(Interval),
    /// Every finite parameter is valid and the geometry does not repeat
    /// (lines, the axial direction of cylinders and cones, planes).
    Unbounded,
}

impl Domain {
    /// The period of a periodic domain.
    pub fn period(&self) -> Option<f64> {
        match self {
            Self::Periodic(i) => Some(i.length()),
            Self::Bounded(_) | Self::Unbounded => None,
        }
    }

    /// Returns `true` for a periodic domain.
    pub fn is_periodic(&self) -> bool {
        matches!(self, Self::Periodic(_))
    }

    /// The interval of a bounded domain, or the base period of a periodic
    /// one; `None` when unbounded.
    pub fn interval(&self) -> Option<Interval> {
        match self {
            Self::Bounded(i) | Self::Periodic(i) => Some(*i),
            Self::Unbounded => None,
        }
    }
}

/// The full turn `[0, 2 pi]`, the base period of every analytic angle.
pub(crate) const FULL_TURN: Interval = Interval { min: 0.0, max: TAU };

/// `t` wrapped into `[0, 2 pi)`.
pub(crate) fn wrap_angle(t: f64) -> f64 {
    let w = t.rem_euclid(TAU);
    // rem_euclid can round up to exactly 2 pi for tiny negative input.
    if w >= TAU { 0.0 } else { w }
}

/// The `k`-th derivatives of `cos` and `sin` at an angle whose cosine and
/// sine are `c` and `s`: `(cos^(k), sin^(k))`, exact (no phase shift).
pub(crate) fn trig_derivative(c: f64, s: f64, k: usize) -> (f64, f64) {
    match k % 4 {
        0 => (c, s),
        1 => (-s, c),
        2 => (-c, -s),
        _ => (s, -c),
    }
}

/// The number of derivative entries to compute for a request of order `n`.
pub(crate) fn derivative_count(n: usize) -> usize {
    n.min(MAX_DERIVATIVE_ORDER) + 1
}

/// Pushes the parameters in `range` where `cos(t - phase)` has an extremum
/// (`t = phase + m pi`), clamped into the range. Ranges longer than a full
/// turn are treated as one full turn from their start, which already holds
/// every extremum.
pub(crate) fn sinusoid_extrema(range: Interval, phase: f64, out: &mut Vec<f64>) {
    let lo = range.min;
    let hi = range.max.min(lo + TAU);
    let first = ((lo - phase) / PI).ceil();
    for i in 0..3 {
        let t = phase + (first + f64::from(i)) * PI;
        if t <= hi {
            out.push(t.clamp(lo, hi));
        }
    }
}

/// Checks that a parameter range is finite.
pub(crate) fn check_range(range: Interval) -> Result<(), GeomError> {
    if range.min.is_finite() && range.max.is_finite() {
        Ok(())
    } else {
        Err(GeomError::NonFinite)
    }
}

/// Checks that a radius is finite and positive.
pub(crate) fn check_radius(r: f64) -> Result<(), GeomError> {
    if !r.is_finite() {
        Err(GeomError::NonFinite)
    } else if r <= 0.0 {
        Err(GeomError::InvalidArgument("a radius must be positive"))
    } else {
        Ok(())
    }
}

/// Checks that the frame is finite, orthonormal and right-handed.
pub(crate) fn check_frame(frame: &Frame3) -> Result<(), GeomError> {
    let Frame3 { origin, x, y, z } = *frame;
    if !(origin.is_finite() && x.is_finite() && y.is_finite() && z.is_finite()) {
        return Err(GeomError::NonFinite);
    }
    let unit = |v: Vector3| (v.length() - 1.0).abs() <= FRAME_TOLERANCE;
    if !(unit(x) && unit(y) && unit(z))
        || x.dot(y).abs() > FRAME_TOLERANCE
        || (x.cross(y) - z).length() > FRAME_TOLERANCE
    {
        return Err(GeomError::InvalidArgument(
            "the frame must be orthonormal and right-handed",
        ));
    }
    Ok(())
}

/// `v` scaled to unit length, or an error for a zero or non-finite vector.
pub(crate) fn unit3(v: Vector3, what: &'static str) -> Result<Vector3, GeomError> {
    if !v.is_finite() {
        return Err(GeomError::NonFinite);
    }
    v.try_normalize().ok_or(GeomError::InvalidArgument(what))
}

/// `v` scaled to unit length, or an error for a zero or non-finite vector.
pub(crate) fn unit2(v: Vector2, what: &'static str) -> Result<Vector2, GeomError> {
    if !v.is_finite() {
        return Err(GeomError::NonFinite);
    }
    v.try_normalize().ok_or(GeomError::InvalidArgument(what))
}

/// Checks that a 3D point is finite.
pub(crate) fn check_point3(p: Point3) -> Result<(), GeomError> {
    if p.is_finite() {
        Ok(())
    } else {
        Err(GeomError::NonFinite)
    }
}

/// Checks that a 2D point is finite.
pub(crate) fn check_point2(p: Point2) -> Result<(), GeomError> {
    if p.is_finite() {
        Ok(())
    } else {
        Err(GeomError::NonFinite)
    }
}

/// The point of the ellipse `x^2 / a^2 + y^2 / b^2 = 1` (`a >= b > 0`)
/// closest to `p`, in the ellipse's own coordinates.
///
/// This is Eberly's robust method ("Distance from a point to an ellipse, an
/// ellipsoid, or a hyperellipsoid"): by symmetry the work is done in the
/// first quadrant, where the closest point is `x_i = e_i^2 y_i / (t +
/// e_i^2)` for the unique root `t` of a function that is monotone on a
/// known bracket; the root is found by bisection to full precision. On the
/// major axis inside the evolute the answer is the closed form of that
/// limit. For the centre, the end of the minor axis on the side of `p` is
/// returned (both ends are equally close).
pub(crate) fn closest_on_ellipse(a: f64, b: f64, p: Point2) -> Point2 {
    let (y0, y1) = (p.x.abs(), p.y.abs());
    let (x0, x1) = if y1 > 0.0 {
        if y0 > 0.0 {
            let (z0, z1) = (y0 / a, y1 / b);
            let g = z0 * z0 + z1 * z1 - 1.0;
            let r0 = (a / b) * (a / b);
            let s = ellipse_root(r0, z0, z1, g);
            (r0 * y0 / (s + r0), y1 / (s + 1.0))
        } else {
            (0.0, b)
        }
    } else {
        let numer = a * y0;
        let denom = a * a - b * b;
        if numer < denom {
            let xa = numer / denom;
            (a * xa, b * (1.0 - xa * xa).max(0.0).sqrt())
        } else {
            (a, 0.0)
        }
    };
    Point2::new(
        if p.x < 0.0 { -x0 } else { x0 },
        if p.y < 0.0 { -x1 } else { x1 },
    )
}

/// Bisection limit for [`ellipse_root`]; each step halves the bracket, so
/// this is far more than double precision needs.
const ELLIPSE_BISECTIONS: usize = 200;

/// The root `s` of `(r0 z0 / (s + r0))^2 + (z1 / (s + 1))^2 - 1` on Eberly's
/// bracket, by bisection until the midpoint no longer moves.
fn ellipse_root(r0: f64, z0: f64, z1: f64, g: f64) -> f64 {
    let n0 = r0 * z0;
    let mut s0 = z1 - 1.0;
    let mut s1 = if g < 0.0 { 0.0 } else { n0.hypot(z1) - 1.0 };
    let mut s = 0.5 * (s0 + s1);
    for _ in 0..ELLIPSE_BISECTIONS {
        s = 0.5 * (s0 + s1);
        if s <= s0 || s >= s1 {
            break;
        }
        let ratio0 = n0 / (s + r0);
        let ratio1 = z1 / (s + 1.0);
        let g = ratio0 * ratio0 + ratio1 * ratio1 - 1.0;
        if g > 0.0 {
            s0 = s;
        } else if g < 0.0 {
            s1 = s;
        } else {
            break;
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_angle_stays_in_one_turn() {
        assert!(wrap_angle(-1e-300) < TAU);
        assert!((wrap_angle(-PI) - PI).abs() < 1e-15);
        assert!((wrap_angle(7.0) - (7.0 - TAU)).abs() < 1e-15);
        assert!(wrap_angle(0.0).abs() < 1e-300);
    }

    #[test]
    fn sinusoid_extrema_are_in_range() {
        let mut out = Vec::new();
        sinusoid_extrema(Interval::new(0.0, TAU), 0.3, &mut out);
        assert_eq!(out.len(), 2);
        assert!((out[0] - 0.3).abs() < 1e-15 && (out[1] - 0.3 - PI).abs() < 1e-15);
        out.clear();
        // A long range holds a maximum and a minimum within its first turn.
        sinusoid_extrema(Interval::new(-100.0, 100.0), 0.0, &mut out);
        assert!(out.len() >= 2 && out.iter().all(|t| (-100.0..=-100.0 + TAU).contains(t)));
        out.clear();
        sinusoid_extrema(Interval::new(0.5, 1.0), 0.0, &mut out);
        assert!(out.is_empty());
    }

    #[test]
    fn ellipse_closest_point_handles_the_axes() {
        let (a, b) = (3.0, 1.0);
        // Inside the evolute on the major axis: off-axis answer.
        let q = closest_on_ellipse(a, b, Point2::new(1.0, 0.0));
        assert!(((q.x / a).powi(2) + (q.y / b).powi(2) - 1.0).abs() < 1e-14);
        assert!(q.y > 0.0);
        // Beyond it: the vertex.
        let q = closest_on_ellipse(a, b, Point2::new(-5.0, 0.0));
        assert!((q - Point2::new(-3.0, 0.0)).length() < 1e-15);
        // On the minor axis.
        let q = closest_on_ellipse(a, b, Point2::new(0.0, -0.5));
        assert!((q - Point2::new(0.0, -1.0)).length() < 1e-15);
        // A point on the ellipse is its own closest point.
        let t: f64 = 0.7;
        let p = Point2::new(a * t.cos(), b * t.sin());
        assert!((closest_on_ellipse(a, b, p) - p).length() < 1e-14);
    }
}
