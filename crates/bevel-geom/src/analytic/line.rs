// SPDX-License-Identifier: GPL-3.0-or-later

//! Straight lines in the plane and in space.
//!
//! A line is `C(t) = o + t d` with a unit direction `d`: the parameter is
//! the signed distance from the origin `o` along `d`, and the domain is
//! unbounded (the topology bounds edges). Reversal negates `d` and keeps
//! `o`, so the reversed line at `-t` is the original at `t`.

use bevel_math::{Aabb2, Aabb3, Interval, Point2, Point3, Vector2, Vector3};

use super::{Domain, check_point2, check_point3, check_range, derivative_count, unit2, unit3};
use crate::{ClosestPoint, GeomError};

/// An unbounded straight line in the plane: `C(t) = origin + t direction`
/// with a unit direction.
#[derive(Clone, Copy, Debug, PartialEq)]
#[must_use]
pub struct Line2 {
    origin: Point2,
    direction: Vector2,
}

/// An unbounded straight line in space: `C(t) = origin + t direction` with
/// a unit direction.
#[derive(Clone, Copy, Debug, PartialEq)]
#[must_use]
pub struct Line3 {
    origin: Point3,
    direction: Vector3,
}

/// Generates the parts shared by [`Line2`] and [`Line3`].
macro_rules! line_common {
    ($line:ident, $point:ty, $vector:ty, $aabb:ty, $check:ident, $unit:ident) => {
        impl $line {
            /// The line through `origin` along `direction`, which is
            /// normalised (any non-zero length is accepted).
            ///
            /// # Errors
            ///
            /// [`GeomError::NonFinite`] for non-finite input and
            /// [`GeomError::InvalidArgument`] for a zero direction.
            pub fn new(origin: $point, direction: $vector) -> Result<Self, GeomError> {
                $check(origin)?;
                let direction = $unit(direction, "a line direction must be non-zero")?;
                Ok(Self { origin, direction })
            }

            /// The line through `a` (at `t = 0`) and `b` (at `t = |b - a|`).
            ///
            /// # Errors
            ///
            /// As [`Self::new`]; coincident points give a zero direction.
            pub fn through(a: $point, b: $point) -> Result<Self, GeomError> {
                Self::new(a, b - a)
            }

            /// The point at `t = 0`.
            pub fn origin(&self) -> $point {
                self.origin
            }

            /// The unit direction.
            pub fn direction(&self) -> $vector {
                self.direction
            }

            /// The point at signed distance `t` from the origin.
            pub fn point(&self, t: f64) -> $point {
                self.origin + self.direction * t
            }

            /// The point and the derivatives at `t` up to order `n`: the
            /// point, the direction, then zeros (see the module
            /// documentation of [`analytic`](crate::analytic) for the
            /// layout).
            pub fn derivatives(&self, t: f64, n: usize) -> Vec<$point> {
                let mut out = vec![<$vector>::ZERO; derivative_count(n)];
                if let Some(p) = out.first_mut() {
                    *p = self.point(t);
                }
                if let Some(d) = out.get_mut(1) {
                    *d = self.direction;
                }
                out
            }

            /// The unit tangent: the direction, everywhere.
            pub fn tangent(&self, _t: f64) -> Option<$vector> {
                Some(self.direction)
            }

            /// The curvature: zero everywhere.
            pub fn curvature(&self, _t: f64) -> Option<f64> {
                Some(0.0)
            }

            /// [`Domain::Unbounded`].
            pub fn domain(&self) -> Domain {
                Domain::Unbounded
            }

            /// The foot of the perpendicular from `target`.
            ///
            /// # Errors
            ///
            /// [`GeomError::NonFinite`] if `target` is not finite.
            pub fn closest_point(&self, target: $point) -> Result<ClosestPoint<$point>, GeomError> {
                $check(target)?;
                let t = (target - self.origin).dot(self.direction);
                let point = self.point(t);
                Ok(ClosestPoint {
                    t,
                    point,
                    distance: (target - point).length(),
                })
            }

            /// The exact bounding box of the segment over `range`.
            ///
            /// # Errors
            ///
            /// [`GeomError::NonFinite`] for a non-finite range.
            pub fn bounds(&self, range: Interval) -> Result<$aabb, GeomError> {
                check_range(range)?;
                Ok(<$aabb>::new(self.point(range.min), self.point(range.max)))
            }

            /// The same line traversed backwards: the direction is negated
            /// and the origin kept, so the reversed line at `-t` is this one
            /// at `t`.
            pub fn reversed(&self) -> Self {
                Self {
                    origin: self.origin,
                    direction: -self.direction,
                }
            }

            /// The parameter on [`Self::reversed`] of the point at `t`: `-t`.
            pub fn reversed_parameter(&self, t: f64) -> f64 {
                -t
            }
        }
    };
}

line_common!(Line2, Point2, Vector2, Aabb2, check_point2, unit2);
line_common!(Line3, Point3, Vector3, Aabb3, check_point3, unit3);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_is_parametrised_by_distance() {
        let l = Line3::new(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 3.0, 4.0)).unwrap();
        assert!((l.direction().length() - 1.0).abs() < 1e-15);
        assert!((l.point(5.0) - Point3::new(1.0, 5.0, 7.0)).length() < 1e-14);
        let cp = l
            .closest_point(Point3::new(1.0, 5.0, 7.0) + Vector3::X * 2.0)
            .unwrap();
        assert!((cp.t - 5.0).abs() < 1e-14 && (cp.distance - 2.0).abs() < 1e-14);
        let r = l.reversed();
        assert!((r.point(r.reversed_parameter(1.5)) - l.point(1.5)).length() < 1e-15);
        let d = l.derivatives(0.0, 3);
        assert_eq!(d.len(), 4);
        assert!(d[2].length() < 1e-300 && d[3].length() < 1e-300);
    }

    #[test]
    fn degenerate_lines_are_rejected() {
        assert_eq!(
            Line2::new(Point2::ZERO, Vector2::ZERO),
            Err(GeomError::InvalidArgument(
                "a line direction must be non-zero"
            ))
        );
        assert_eq!(
            Line3::through(Point3::ONE, Point3::ONE),
            Err(GeomError::InvalidArgument(
                "a line direction must be non-zero"
            ))
        );
        assert_eq!(
            Line3::new(Point3::new(f64::NAN, 0.0, 0.0), Vector3::X),
            Err(GeomError::NonFinite)
        );
        assert_eq!(
            Line2::new(Point2::ZERO, Vector2::new(f64::INFINITY, 0.0)),
            Err(GeomError::NonFinite)
        );
    }
}
