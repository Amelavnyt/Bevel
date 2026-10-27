// SPDX-License-Identifier: GPL-3.0-or-later

//! Robust geometric predicates.
//!
//! Each function returns a value whose **sign is exact** for the given
//! `f64` inputs, however close the configuration is to degenerate. The
//! magnitude is only a rough size of the underlying determinant. The
//! implementation is the [`robust`] crate, a port of Jonathan Richard
//! Shewchuk's adaptive-precision predicates:
//!
//! J. R. Shewchuk, "Adaptive Precision Floating-Point Arithmetic and Fast
//! Robust Geometric Predicates", Discrete & Computational Geometry 18(3),
//! 1997.
//!
//! Use the sign, compared against zero, to decide orientation; do not apply
//! a tolerance to the result.

use crate::{Point2, Point3};
use robust::{Coord, Coord3D};

fn c2(p: Point2) -> Coord<f64> {
    Coord { x: p.x, y: p.y }
}

fn c3(p: Point3) -> Coord3D<f64> {
    Coord3D {
        x: p.x,
        y: p.y,
        z: p.z,
    }
}

/// Orientation of the triangle `a, b, c` in the plane.
///
/// Positive if `a, b, c` are in counter-clockwise order (`c` lies to the
/// left of the directed line `a -> b`), negative if clockwise, and zero if
/// the three points are exactly collinear. The magnitude is twice the
/// signed area of the triangle.
pub fn orient2d(a: Point2, b: Point2, c: Point2) -> f64 {
    robust::orient2d(c2(a), c2(b), c2(c))
}

/// Orientation of the point `d` relative to the plane through `a, b, c`.
///
/// Positive if `d` lies **below** the plane, that is, on the side from
/// which `a, b, c` appear in clockwise order (equivalently, when `a, b, c`
/// are counter-clockwise as seen from above, `d` is below). Negative if `d`
/// is above, and zero if the four points are exactly coplanar. This is
/// Shewchuk's convention, and is the opposite of the "right-hand rule
/// normal" convention: for `a = (0,0,0)`, `b = (1,0,0)`, `c = (0,1,0)`,
/// the point `d = (0,0,-1)` gives a positive value.
pub fn orient3d(a: Point3, b: Point3, c: Point3, d: Point3) -> f64 {
    robust::orient3d(c3(a), c3(b), c3(c), c3(d))
}

/// Whether `d` lies inside the circle through `a, b, c`.
///
/// Positive if `d` is inside the circle, negative if outside, and zero if
/// exactly on it, **provided** `a, b, c` are in counter-clockwise order
/// (see [`orient2d`]). If they are clockwise the signs are reversed.
pub fn incircle(a: Point2, b: Point2, c: Point2, d: Point2) -> f64 {
    robust::incircle(c2(a), c2(b), c2(c), c2(d))
}

/// Whether `e` lies inside the sphere through `a, b, c, d`.
///
/// Positive if `e` is inside the sphere, negative if outside, and zero if
/// exactly on it, **provided** `a, b, c, d` are ordered so that
/// [`orient3d`]`(a, b, c, d)` is positive. With the opposite orientation
/// the signs are reversed.
pub fn insphere(a: Point3, b: Point3, c: Point3, d: Point3, e: Point3) -> f64 {
    robust::insphere(c3(a), c3(b), c3(c), c3(d), c3(e))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Scale that turns the test coordinates (multiples of 2^-53, below 32)
    /// into exact integers.
    const SCALE: f64 = 9_007_199_254_740_992.0; // 2^53

    fn to_int(x: f64) -> i128 {
        (x * SCALE) as i128
    }

    /// Exact sign of the orientation determinant, in integer arithmetic.
    fn exact_orient2d(a: Point2, b: Point2, c: Point2) -> i32 {
        let (ax, ay) = (to_int(a.x), to_int(a.y));
        let (bx, by) = (to_int(b.x), to_int(b.y));
        let (cx, cy) = (to_int(c.x), to_int(c.y));
        let det = (ax - cx) * (by - cy) - (ay - cy) * (bx - cx);
        det.signum() as i32
    }

    fn sign(x: f64) -> i32 {
        if x > 0.0 {
            1
        } else if x < 0.0 {
            -1
        } else {
            0
        }
    }

    #[test]
    fn orient2d_counter_clockwise_is_positive() {
        let (a, b, c) = (
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(0.0, 1.0),
        );
        assert!(orient2d(a, b, c) > 0.0);
        assert!(orient2d(a, c, b) < 0.0);
        assert!(orient2d(a, b, Point2::new(2.0, 0.0)).abs() < 1e-300);
    }

    #[test]
    fn orient2d_is_exact_for_nearly_collinear_points() {
        // Points near the line y = x at the scale of one ulp of 0.5.
        let u = 1.0 / SCALE;
        let b = Point2::new(12.0, 12.0);
        let c = Point2::new(24.0, 24.0);
        let mut naive_wrong = 0;
        for i in 0..32 {
            for j in 0..32 {
                let a = Point2::new(0.5 + f64::from(i) * u, 0.5 + f64::from(j) * u);
                let exact = exact_orient2d(a, b, c);
                assert_eq!(sign(orient2d(a, b, c)), exact, "i={i} j={j}");
                let naive = (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x);
                if sign(naive) != exact {
                    naive_wrong += 1;
                }
            }
        }
        // The point of the exercise: plain floating point gets many of these wrong.
        assert!(naive_wrong > 0, "naive evaluation was unexpectedly exact");
    }

    #[test]
    fn orient3d_sign_convention() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(0.0, 1.0, 0.0);
        assert!(orient3d(a, b, c, Point3::new(0.0, 0.0, -1.0)) > 0.0);
        assert!(orient3d(a, b, c, Point3::new(0.0, 0.0, 1.0)) < 0.0);
        assert!(orient3d(a, b, c, Point3::new(5.0, 7.0, 0.0)).abs() < 1e-300);
    }

    #[test]
    fn orient3d_detects_tiny_offsets_from_a_plane() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1e3, 0.0, 0.0);
        let c = Point3::new(0.0, 1e3, 0.0);
        let d = Point3::new(300.0, 300.0, -1e-300);
        assert!(orient3d(a, b, c, d) > 0.0);
    }

    #[test]
    fn incircle_sign_convention() {
        // Unit circle through three counter-clockwise points.
        let a = Point2::new(1.0, 0.0);
        let b = Point2::new(0.0, 1.0);
        let c = Point2::new(-1.0, 0.0);
        assert!(orient2d(a, b, c) > 0.0);
        assert!(incircle(a, b, c, Point2::ZERO) > 0.0);
        assert!(incircle(a, b, c, Point2::new(2.0, 0.0)) < 0.0);
        assert!(incircle(a, b, c, Point2::new(0.0, -1.0)).abs() < 1e-300);
    }

    #[test]
    fn insphere_sign_convention() {
        // Unit sphere through four points with positive orient3d.
        let a = Point3::new(1.0, 0.0, 0.0);
        let b = Point3::new(0.0, 1.0, 0.0);
        let c = Point3::new(-1.0, 0.0, 0.0);
        let d = Point3::new(0.0, 0.0, -1.0);
        assert!(orient3d(a, b, c, d) > 0.0);
        assert!(insphere(a, b, c, d, Point3::ZERO) > 0.0);
        assert!(insphere(a, b, c, d, Point3::new(2.0, 0.0, 0.0)) < 0.0);
        assert!(insphere(a, b, c, d, Point3::new(0.0, 0.0, 1.0)).abs() < 1e-300);
    }
}
