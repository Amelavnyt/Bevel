// SPDX-License-Identifier: GPL-3.0-or-later

//! Damped Newton iteration in one, two and three unknowns.
//!
//! The caller supplies a function returning the value **and** the
//! derivative (Jacobian) at a point. Every iteration:
//!
//! 1. computes the full Newton step `-J⁻¹ F`;
//! 2. clamps the target point `x + step` into the search bounds;
//! 3. accepts the clamped step if it reduces `‖F‖`; otherwise halves it,
//!    up to 10 times (backtracking), and takes the smallest trial if none
//!    reduced the residual.
//!
//! **Convergence.** The iteration succeeds as soon as either
//! - the residual is tiny: `‖F(x)‖ <= tol²`, or
//! - the full (undamped, unclamped) Newton step is short:
//!   `‖step‖ <= tol`, in which case the point after that step is returned.
//!
//! Because the step test uses the unclamped step, a start that is pressed
//! against a bound with the root outside the bounds does not count as
//! converged; it fails with [`MathError::NotConverged`].
//!
//! **Failure.** [`MathError::SingularJacobian`] if the derivative is zero
//! or (in 2D and 3D) `|det J|` is below `1e-14 ‖J‖ⁿ` for the Frobenius
//! norm `‖J‖`; [`MathError::NotConverged`] if `max_iter` iterations pass
//! (or the iterate is stuck on a bound); [`MathError::InvalidInput`] for a
//! non-positive or non-finite `tol`, a non-finite start, or a function that
//! returns non-finite values.

use crate::{DMat2, DMat3, DVec2, DVec3, Interval, MathError};
use std::ops::{Add, Mul, Neg, Sub};

/// Maximum number of step halvings per iteration.
const MAX_HALVINGS: usize = 10;
/// Determinants below this multiple of `‖J‖ⁿ` count as singular.
const SINGULAR_TOL: f64 = 1e-14;

/// A space Newton's method can run in.
trait Space:
    Copy + Add<Output = Self> + Sub<Output = Self> + Mul<f64, Output = Self> + Neg<Output = Self>
{
    /// The derivative type.
    type Jacobian: Copy;
    /// The bounds type.
    type Bounds: Copy;

    fn norm(self) -> f64;
    fn is_finite(self) -> bool;
    fn clamp_to(self, bounds: Self::Bounds) -> Self;
    /// Solves `J d = rhs`.
    fn solve(jacobian: Self::Jacobian, rhs: Self) -> Result<Self, MathError>;
}

impl Space for f64 {
    type Jacobian = f64;
    type Bounds = Interval;

    fn norm(self) -> f64 {
        self.abs()
    }

    fn is_finite(self) -> bool {
        f64::is_finite(self)
    }

    fn clamp_to(self, bounds: Interval) -> Self {
        bounds.clamp(self)
    }

    fn solve(jacobian: f64, rhs: f64) -> Result<f64, MathError> {
        if jacobian.abs() > f64::MIN_POSITIVE {
            Ok(rhs / jacobian)
        } else {
            Err(MathError::SingularJacobian)
        }
    }
}

impl Space for DVec2 {
    type Jacobian = DMat2;
    type Bounds = [Interval; 2];

    fn norm(self) -> f64 {
        self.length()
    }

    fn is_finite(self) -> bool {
        DVec2::is_finite(self)
    }

    fn clamp_to(self, b: [Interval; 2]) -> Self {
        DVec2::new(b[0].clamp(self.x), b[1].clamp(self.y))
    }

    fn solve(j: DMat2, rhs: DVec2) -> Result<DVec2, MathError> {
        let frobenius_sq = j.x_axis.length_squared() + j.y_axis.length_squared();
        if j.determinant().abs() > SINGULAR_TOL * frobenius_sq {
            Ok(j.inverse() * rhs)
        } else {
            Err(MathError::SingularJacobian)
        }
    }
}

impl Space for DVec3 {
    type Jacobian = DMat3;
    type Bounds = [Interval; 3];

    fn norm(self) -> f64 {
        self.length()
    }

    fn is_finite(self) -> bool {
        DVec3::is_finite(self)
    }

    fn clamp_to(self, b: [Interval; 3]) -> Self {
        DVec3::new(b[0].clamp(self.x), b[1].clamp(self.y), b[2].clamp(self.z))
    }

    fn solve(j: DMat3, rhs: DVec3) -> Result<DVec3, MathError> {
        let frobenius_sq =
            j.x_axis.length_squared() + j.y_axis.length_squared() + j.z_axis.length_squared();
        if j.determinant().abs() > SINGULAR_TOL * frobenius_sq * frobenius_sq.sqrt() {
            Ok(j.inverse() * rhs)
        } else {
            Err(MathError::SingularJacobian)
        }
    }
}

/// The shared damped Newton loop.
fn damped_newton<S: Space>(
    f: impl Fn(S) -> (S, S::Jacobian),
    x0: S,
    bounds: S::Bounds,
    tol: f64,
    max_iter: usize,
) -> Result<S, MathError> {
    if !(tol > 0.0 && tol.is_finite()) {
        return Err(MathError::InvalidInput(
            "tolerance must be positive and finite",
        ));
    }
    if !x0.is_finite() {
        return Err(MathError::InvalidInput("start point must be finite"));
    }
    let mut x = x0.clamp_to(bounds);
    let (mut fx, mut jx) = f(x);
    let mut iteration = 0;
    loop {
        if !fx.is_finite() {
            return Err(MathError::InvalidInput(
                "function returned a non-finite value",
            ));
        }
        let residual = fx.norm();
        if residual <= tol * tol {
            return Ok(x);
        }
        if iteration >= max_iter {
            return Err(MathError::NotConverged {
                iterations: iteration,
            });
        }
        let newton = S::solve(jx, -fx)?;
        if !newton.is_finite() {
            return Err(MathError::SingularJacobian);
        }
        let target = (x + newton).clamp_to(bounds);
        if newton.norm() <= tol {
            return Ok(target);
        }
        let step = target - x;
        if step.norm() <= 0.0 {
            // Pressed against a bound with the root outside it.
            return Err(MathError::NotConverged {
                iterations: iteration,
            });
        }

        let mut lambda = 1.0;
        let mut trial = x + step;
        let mut value = f(trial);
        for _ in 0..MAX_HALVINGS {
            if value.0.norm() < residual {
                break;
            }
            lambda *= 0.5;
            trial = x + step * lambda;
            value = f(trial);
        }
        x = trial;
        (fx, jx) = value;
        iteration += 1;
    }
}

/// Finds a root of a scalar function by damped Newton iteration.
///
/// `f` returns `(value, derivative)`. The iterate is kept inside `bounds`
/// (the start is clamped into it). See the [module documentation](self) for
/// the convergence criterion and the errors.
///
/// # Errors
/// [`MathError::SingularJacobian`] for a zero derivative,
/// [`MathError::NotConverged`] when `max_iter` is exhausted, and
/// [`MathError::InvalidInput`] for invalid `tol`, `x0` or function values.
pub fn newton_1d(
    f: impl Fn(f64) -> (f64, f64),
    x0: f64,
    bounds: Interval,
    tol: f64,
    max_iter: usize,
) -> Result<f64, MathError> {
    damped_newton(f, x0, bounds, tol, max_iter)
}

/// Finds a root of `F: R² -> R²` by damped Newton iteration.
///
/// `f` returns `(F(x), J(x))`, where column `i` of the matrix is the
/// derivative of `F` with respect to `x[i]`. The iterate is kept inside the
/// box `bounds[0] × bounds[1]`. See the [module documentation](self).
///
/// # Errors
/// As for [`newton_1d`]; the Jacobian is singular when its determinant is
/// negligible relative to its norm.
pub fn newton_2d(
    f: impl Fn(DVec2) -> (DVec2, DMat2),
    x0: DVec2,
    bounds: [Interval; 2],
    tol: f64,
    max_iter: usize,
) -> Result<DVec2, MathError> {
    damped_newton(f, x0, bounds, tol, max_iter)
}

/// Finds a root of `F: R³ -> R³` by damped Newton iteration.
///
/// `f` returns `(F(x), J(x))`, where column `i` of the matrix is the
/// derivative of `F` with respect to `x[i]`. The iterate is kept inside the
/// box `bounds[0] × bounds[1] × bounds[2]`. See the [module documentation](self).
///
/// # Errors
/// As for [`newton_2d`].
pub fn newton_3d(
    f: impl Fn(DVec3) -> (DVec3, DMat3),
    x0: DVec3,
    bounds: [Interval; 3],
    tol: f64,
    max_iter: usize,
) -> Result<DVec3, MathError> {
    damped_newton(f, x0, bounds, tol, max_iter)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    const WIDE: Interval = Interval {
        min: -1e3,
        max: 1e3,
    };

    #[test]
    fn newton_1d_finds_sqrt_two() {
        let r = newton_1d(|x| (x * x - 2.0, 2.0 * x), 1.0, WIDE, 1e-14, 50).unwrap();
        assert!((r - 2.0_f64.sqrt()).abs() < 1e-14);
    }

    #[test]
    fn newton_1d_respects_bounds() {
        // Both +-sqrt(2) are roots; the bounds select the negative one.
        let r = newton_1d(
            |x| (x * x - 2.0, 2.0 * x),
            1.0,
            Interval::new(-3.0, -0.5),
            1e-14,
            50,
        )
        .unwrap();
        assert!((r + 2.0_f64.sqrt()).abs() < 1e-14);
    }

    #[test]
    fn newton_1d_returns_a_root_at_the_start() {
        let r = newton_1d(|x| (x - 3.0, 1.0), 3.0, WIDE, 1e-12, 0).unwrap();
        assert!((r - 3.0).abs() < 1e-15);
    }

    #[test]
    fn newton_1d_singular_start() {
        assert_eq!(
            newton_1d(|x| (x * x - 1.0, 2.0 * x), 0.0, WIDE, 1e-12, 50),
            Err(MathError::SingularJacobian)
        );
    }

    #[test]
    fn newton_1d_reports_failure_to_converge() {
        // x² + 1 has no real root.
        let r = newton_1d(|x| (x * x + 1.0, 2.0 * x), 2.0, WIDE, 1e-12, 60);
        assert!(r.is_err(), "{r:?}");
        // Root outside the bounds: stuck on the bound.
        let r = newton_1d(|x| (x - 5.0, 1.0), 0.5, Interval::new(0.0, 1.0), 1e-12, 50);
        assert!(matches!(r, Err(MathError::NotConverged { .. })), "{r:?}");
        // Too few iterations.
        let r = newton_1d(|x| (x * x - 2.0, 2.0 * x), 100.0, WIDE, 1e-14, 2);
        assert_eq!(r, Err(MathError::NotConverged { iterations: 2 }));
    }

    #[test]
    fn newton_1d_rejects_bad_arguments() {
        let f = |x: f64| (x, 1.0);
        assert!(matches!(
            newton_1d(f, 1.0, WIDE, 0.0, 10),
            Err(MathError::InvalidInput(_))
        ));
        assert!(matches!(
            newton_1d(f, f64::NAN, WIDE, 1e-9, 10),
            Err(MathError::InvalidInput(_))
        ));
        assert!(matches!(
            newton_1d(|_| (f64::NAN, 1.0), 1.0, WIDE, 1e-9, 10),
            Err(MathError::InvalidInput(_))
        ));
    }

    #[test]
    fn damping_rescues_an_overshooting_step() {
        // atan(x) diverges under pure Newton from x0 = 2; backtracking converges.
        let r = newton_1d(|x| (x.atan(), 1.0 / (1.0 + x * x)), 2.0, WIDE, 1e-12, 100).unwrap();
        assert!(r.abs() < 1e-10);
    }

    fn circle_line(r: f64, c: f64) -> impl Fn(DVec2) -> (DVec2, DMat2) {
        // x² + y² = r² and y = x + c.
        move |p| {
            (
                DVec2::new(p.x * p.x + p.y * p.y - r * r, p.y - p.x - c),
                DMat2::from_cols(DVec2::new(2.0 * p.x, -1.0), DVec2::new(2.0 * p.y, 1.0)),
            )
        }
    }

    #[test]
    fn newton_2d_circle_and_line() {
        let bounds = [WIDE, WIDE];
        // x² + (x + 1)² = 25 -> x = 3 or -4.
        let p = newton_2d(
            circle_line(5.0, 1.0),
            DVec2::new(2.5, 3.0),
            bounds,
            1e-13,
            50,
        )
        .unwrap();
        assert!((p - DVec2::new(3.0, 4.0)).length() < 1e-12);
        let p = newton_2d(
            circle_line(5.0, 1.0),
            DVec2::new(-3.0, -2.0),
            bounds,
            1e-13,
            50,
        )
        .unwrap();
        assert!((p - DVec2::new(-4.0, -3.0)).length() < 1e-12);
    }

    #[test]
    fn newton_2d_singular_start() {
        // The Jacobian at the origin is singular for the circle.
        let r = newton_2d(circle_line(5.0, 1.0), DVec2::ZERO, [WIDE, WIDE], 1e-12, 50);
        assert_eq!(r, Err(MathError::SingularJacobian));
    }

    fn sphere_planes(target: DVec3) -> impl Fn(DVec3) -> (DVec3, DMat3) {
        let r2 = target.length_squared();
        let s = target.x + target.y + target.z;
        let d = target.x - target.y;
        move |p| {
            (
                DVec3::new(p.length_squared() - r2, p.x + p.y + p.z - s, p.x - p.y - d),
                DMat3::from_cols(
                    DVec3::new(2.0 * p.x, 1.0, 1.0),
                    DVec3::new(2.0 * p.y, 1.0, -1.0),
                    DVec3::new(2.0 * p.z, 1.0, 0.0),
                ),
            )
        }
    }

    #[test]
    fn newton_3d_sphere_and_planes() {
        let target = DVec3::new(1.0, 0.5, 3.0);
        let w = Interval::new(-10.0, 10.0);
        let p = newton_3d(
            sphere_planes(target),
            target + DVec3::new(0.2, -0.1, 0.15),
            [w, w, w],
            1e-13,
            50,
        )
        .unwrap();
        assert!((p - target).length() < 1e-12);
    }

    #[test]
    fn newton_3d_singular_start() {
        let w = Interval::new(-10.0, 10.0);
        let r = newton_3d(
            sphere_planes(DVec3::new(1.0, 0.5, 3.0)),
            DVec3::ZERO,
            [w, w, w],
            1e-12,
            50,
        );
        assert_eq!(r, Err(MathError::SingularJacobian));
    }

    proptest! {
        #[test]
        fn sqrt_from_nearby_start(a in 0.01..1e4f64, scale in 0.5..2.0f64) {
            let root = a.sqrt();
            let r = newton_1d(
                |x| (x * x - a, 2.0 * x),
                root * scale,
                Interval::new(1e-6, 1e6),
                1e-13,
                100,
            ).unwrap();
            prop_assert!((r - root).abs() <= 1e-10 * root);
        }

        #[test]
        fn circle_line_from_nearby_start(
            r in 1.0..10.0f64,
            angle in 0.3..1.2f64,
            dx in -0.05..0.05f64,
            dy in -0.05..0.05f64,
        ) {
            // Line y = x + c passing through the point at `angle` on the circle.
            let target = DVec2::new(r * angle.cos(), r * angle.sin());
            let c = target.y - target.x;
            let start = target + DVec2::new(dx, dy) * r;
            let w = Interval::new(-100.0, 100.0);
            let p = newton_2d(circle_line(r, c), start, [w, w], 1e-13, 100).unwrap();
            // The line cuts the circle twice; the result must lie on both.
            prop_assert!((p.length() - r).abs() <= 1e-10);
            prop_assert!((p.y - p.x - c).abs() <= 1e-10);
        }

        #[test]
        fn sphere_planes_from_nearby_start(
            x in 0.5..1.5f64,
            y in 0.5..1.5f64,
            z in 3.0..4.0f64,
            d in (-0.1..0.1f64, -0.1..0.1f64, -0.1..0.1f64),
        ) {
            let target = DVec3::new(x, y, z);
            let w = Interval::new(-20.0, 20.0);
            let start = target + DVec3::new(d.0, d.1, d.2);
            let p = newton_3d(sphere_planes(target), start, [w, w, w], 1e-13, 100).unwrap();
            prop_assert!((p - target).length() <= 1e-9);
        }
    }
}
