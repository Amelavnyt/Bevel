// SPDX-License-Identifier: GPL-3.0-or-later

//! Homogeneous (weighted) control points.
//!
//! A rational curve `C(u) = sum N_i(u) w_i P_i / sum N_i(u) w_i` is the
//! perspective projection of the non-rational curve
//! `Cw(u) = sum N_i(u) Pw_i` with `Pw_i = (w_i P_i, w_i)`, one dimension up
//! (The NURBS Book, section 4.2). Knot insertion, refinement, removal,
//! degree elevation and the like are linear in the control points, so they
//! are carried out on the `Pw_i` exactly as for non-rational curves; only
//! evaluation projects back.
//!
//! [`Homogeneous`] abstracts over the two point types so that every
//! algorithm is written once:
//!
//! - [`DVec3`] is a 2D point `(w x, w y, w)`,
//! - [`DVec4`] is a 3D point `(w x, w y, w z, w)`.

use std::fmt::Debug;
use std::ops::{Add, Mul, Sub};

use bevel_math::{DVec2, DVec3, DVec4};

/// A control point in homogeneous coordinates: the weighted point followed by
/// its weight.
pub trait Homogeneous:
    Copy + Add<Output = Self> + Sub<Output = Self> + Mul<f64, Output = Self> + Debug + PartialEq
{
    /// The projected (model-space) point type.
    type Point: Copy
        + Add<Output = Self::Point>
        + Sub<Output = Self::Point>
        + Mul<f64, Output = Self::Point>
        + Debug;

    /// The zero vector (all coordinates and the weight zero).
    const ZERO: Self;

    /// The homogeneous point `(w p, w)`.
    fn from_point(p: Self::Point, w: f64) -> Self;

    /// The weight, the last coordinate.
    fn weight(self) -> f64;

    /// The weighted point `w p`, the first coordinates.
    fn weighted(self) -> Self::Point;

    /// The model-space point `w p / w`.
    fn project(self) -> Self::Point;

    /// Dot product of two model-space points.
    fn point_dot(a: Self::Point, b: Self::Point) -> f64;

    /// Euclidean length of a model-space point.
    fn point_length(p: Self::Point) -> f64;

    /// Returns `true` if every coordinate, including the weight, is finite.
    fn is_finite(self) -> bool;

    /// The zero model-space point.
    fn point_zero() -> Self::Point {
        Self::ZERO.weighted()
    }

    /// Euclidean distance between two homogeneous points, counting the
    /// weight as an ordinary coordinate (the `Distance4D` of The NURBS Book).
    fn distance(a: Self, b: Self) -> f64 {
        let d = a - b;
        let p = d.weighted();
        (Self::point_dot(p, p) + d.weight() * d.weight()).sqrt()
    }
}

impl Homogeneous for DVec3 {
    type Point = DVec2;

    const ZERO: Self = DVec3::ZERO;

    fn from_point(p: DVec2, w: f64) -> Self {
        DVec3::new(w * p.x, w * p.y, w)
    }

    fn weight(self) -> f64 {
        self.z
    }

    fn weighted(self) -> DVec2 {
        self.truncate()
    }

    fn project(self) -> DVec2 {
        self.truncate() / self.z
    }

    fn point_dot(a: DVec2, b: DVec2) -> f64 {
        a.dot(b)
    }

    fn point_length(p: DVec2) -> f64 {
        p.length()
    }

    fn is_finite(self) -> bool {
        DVec3::is_finite(self)
    }
}

impl Homogeneous for DVec4 {
    type Point = DVec3;

    const ZERO: Self = DVec4::ZERO;

    fn from_point(p: DVec3, w: f64) -> Self {
        DVec4::new(w * p.x, w * p.y, w * p.z, w)
    }

    fn weight(self) -> f64 {
        self.w
    }

    fn weighted(self) -> DVec3 {
        self.truncate()
    }

    fn project(self) -> DVec3 {
        self.truncate() / self.w
    }

    fn point_dot(a: DVec3, b: DVec3) -> f64 {
        a.dot(b)
    }

    fn point_length(p: DVec3) -> f64 {
        p.length()
    }

    fn is_finite(self) -> bool {
        DVec4::is_finite(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_through_homogeneous_coordinates() {
        let p = DVec3::new(1.0, -2.0, 3.0);
        let h = DVec4::from_point(p, 0.5);
        assert!((h.weight() - 0.5).abs() < 1e-15);
        assert!((h.weighted() - 0.5 * p).length() < 1e-15);
        assert!((h.project() - p).length() < 1e-15);

        let q = DVec2::new(4.0, 5.0);
        let g = DVec3::from_point(q, 2.0);
        assert!((g.project() - q).length() < 1e-15);
        assert!((DVec3::distance(g, DVec3::ZERO) - g.length()).abs() < 1e-15);
    }
}
