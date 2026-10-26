// SPDX-License-Identifier: GPL-3.0-or-later

//! Tolerance policy.
//!
//! Floats are never compared with `==` in geometry code; use the helpers in
//! this module instead. Parametric tolerances are derived per curve or
//! surface from [`LINEAR`] and the local parametric speed.

use crate::{Point3, Vector3};

/// Linear resolution in millimetres: two points closer than this are the same point.
pub const LINEAR: f64 = 1e-6;

/// Angular resolution in radians: two directions closer than this are parallel.
pub const ANGULAR: f64 = 1e-11;

/// Half-size of the modeling space in millimetres. Geometry outside
/// `[-MODEL_EXTENT, MODEL_EXTENT]` on any axis is rejected.
pub const MODEL_EXTENT: f64 = 1e5;

/// Returns `true` if two lengths differ by at most [`LINEAR`].
pub fn eq_length(a: f64, b: f64) -> bool {
    (a - b).abs() <= LINEAR
}

/// Returns `true` if two points are within [`LINEAR`] of each other.
pub fn eq_point(a: Point3, b: Point3) -> bool {
    a.distance_squared(b) <= LINEAR * LINEAR
}

/// Returns `true` if two non-zero vectors point in the same or opposite
/// direction, to within [`ANGULAR`].
///
/// Returns `false` if either vector is shorter than [`LINEAR`].
pub fn parallel(a: Vector3, b: Vector3) -> bool {
    let (la, lb) = (a.length(), b.length());
    if la <= LINEAR || lb <= LINEAR {
        return false;
    }
    // |a x b| = |a||b| sin(theta), and sin(theta) ~ theta near zero.
    a.cross(b).length() <= ANGULAR * la * lb
}

/// Returns `true` if every coordinate of `p` lies inside the modeling space.
pub fn in_model_space(p: Point3) -> bool {
    p.is_finite() && p.abs().max_element() <= MODEL_EXTENT
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lengths_within_resolution_are_equal() {
        assert!(eq_length(1.0, 1.0 + 0.5 * LINEAR));
        assert!(!eq_length(1.0, 1.0 + 2.0 * LINEAR));
    }

    #[test]
    fn parallel_detects_opposite_directions_and_rejects_zero() {
        assert!(parallel(Vector3::X, -3.0 * Vector3::X));
        assert!(!parallel(Vector3::X, Vector3::new(1.0, 1e-6, 0.0)));
        assert!(!parallel(Vector3::ZERO, Vector3::X));
    }

    #[test]
    fn model_space_rejects_far_and_non_finite_points() {
        assert!(in_model_space(Point3::splat(MODEL_EXTENT)));
        assert!(!in_model_space(Point3::new(0.0, 0.0, 2.0 * MODEL_EXTENT)));
        assert!(!in_model_space(Point3::new(f64::NAN, 0.0, 0.0)));
    }
}
