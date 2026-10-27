// SPDX-License-Identifier: GPL-3.0-or-later

//! Circles in the plane and in space.
//!
//! A circle is `C(t) = c + r (cos(t) x + sin(t) y)` with orthonormal axes
//! `x`, `y`: the parameter is the angle from `x` towards `y`, periodic with
//! period `2 pi` (base period `[0, 2 pi]`). A [`Circle3`] takes its axes
//! from a right-handed [`Frame3`], so it runs counter-clockwise about the
//! frame's `z` axis, its normal. A [`Circle2`] has `y = +-perp(x)` and runs
//! counter-clockwise or clockwise.
//!
//! Reversal negates `y` (and `z`), so the reversed circle at `-t` is the
//! original at `t`. The closest point to a point on the axis (the centre in
//! 2D), where every point of the circle is equally close, is the one at
//! `t = 0`.

use bevel_math::{Aabb2, Aabb3, DVec3, DVec4, Frame3, Interval, Point2, Point3, Vector2, Vector3};
use serde::{Deserialize, Serialize};

use super::convert::{ConvertedCurve, ParamMap, ellipse_arc_axes_2d};

use super::{
    Domain, FULL_TURN, check_frame, check_point2, check_point3, check_radius, check_range,
    check_stored2, derivative_count, sinusoid_extrema, trig_derivative, unit2, unit3, wrap_angle,
};
use crate::{ClosestPoint, GeomError, NurbsCurve, NurbsCurve2, NurbsCurve3, circle_arc};

/// A circle in the plane with its own axes: `C(t) = center + radius
/// (cos(t) x + sin(t) y)` with `y = perp(x)` (counter-clockwise) or
/// `y = -perp(x)` (clockwise).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "Circle2Raw")]
#[must_use]
pub struct Circle2 {
    center: Point2,
    x: Vector2,
    y: Vector2,
    radius: f64,
}

/// The serialized form of a [`Circle2`], checked by [`Circle2::from_axis`].
#[derive(Deserialize)]
struct Circle2Raw {
    center: Point2,
    x: Vector2,
    y: Vector2,
    radius: f64,
}

impl TryFrom<Circle2Raw> for Circle2 {
    type Error = GeomError;

    fn try_from(raw: Circle2Raw) -> Result<Self, GeomError> {
        let ccw = raw.x.perp_dot(raw.y) > 0.0;
        let c = Self::from_axis(raw.center, raw.x, ccw, raw.radius)?;
        check_stored2(raw.x, c.x)?;
        check_stored2(raw.y, c.y)?;
        Ok(Self {
            x: raw.x,
            y: raw.y,
            ..c
        })
    }
}

/// A circle in space: `C(t) = o + radius (cos(t) x + sin(t) y)` in the
/// frame `(o, x, y, z)`, counter-clockwise about `z`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "Circle3Raw")]
#[must_use]
pub struct Circle3 {
    frame: Frame3,
    radius: f64,
}

/// The serialized form of a [`Circle3`], checked by [`Circle3::new`].
#[derive(Deserialize)]
struct Circle3Raw {
    frame: Frame3,
    radius: f64,
}

impl TryFrom<Circle3Raw> for Circle3 {
    type Error = GeomError;

    fn try_from(raw: Circle3Raw) -> Result<Self, GeomError> {
        Self::new(raw.frame, raw.radius)
    }
}

impl Circle2 {
    /// The counter-clockwise circle with `t = 0` on the positive x axis.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for non-finite input and
    /// [`GeomError::InvalidArgument`] for a non-positive radius.
    pub fn new(center: Point2, radius: f64) -> Result<Self, GeomError> {
        Self::from_axis(center, Vector2::X, true, radius)
    }

    /// The circle with `t = 0` in the direction `x_axis` (normalised),
    /// running counter-clockwise or clockwise.
    ///
    /// # Errors
    ///
    /// As [`Circle2::new`], plus [`GeomError::InvalidArgument`] for a zero
    /// axis.
    pub fn from_axis(
        center: Point2,
        x_axis: Vector2,
        counter_clockwise: bool,
        radius: f64,
    ) -> Result<Self, GeomError> {
        check_point2(center)?;
        check_radius(radius)?;
        let x = unit2(x_axis, "a circle axis must be non-zero")?;
        let y = if counter_clockwise {
            x.perp()
        } else {
            -x.perp()
        };
        Ok(Self {
            center,
            x,
            y,
            radius,
        })
    }

    /// The centre.
    pub fn center(&self) -> Point2 {
        self.center
    }

    /// The radius.
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// The unit direction from the centre to the point at `t = 0`.
    pub fn x_axis(&self) -> Vector2 {
        self.x
    }

    /// The unit direction from the centre to the point at `t = pi / 2`.
    pub fn y_axis(&self) -> Vector2 {
        self.y
    }

    /// Returns `true` if the circle runs counter-clockwise.
    pub fn is_counter_clockwise(&self) -> bool {
        self.x.perp_dot(self.y) > 0.0
    }

    /// The signed curvature `+-1 / radius` (positive counter-clockwise).
    pub fn curvature(&self, _t: f64) -> Option<f64> {
        Some(self.x.perp_dot(self.y) / self.radius)
    }

    /// The point of the circle closest to `target`; for the centre, the
    /// point at `t = 0`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    pub fn closest_point(&self, target: Point2) -> Result<ClosestPoint<Point2>, GeomError> {
        check_point2(target)?;
        let d = target - self.center;
        let (lx, ly) = (d.dot(self.x), d.dot(self.y));
        let t = if lx.hypot(ly) > 0.0 {
            wrap_angle(ly.atan2(lx))
        } else {
            0.0
        };
        let point = self.point(t);
        Ok(ClosestPoint {
            t,
            point,
            distance: (target - point).length(),
        })
    }

    /// The same circle traversed backwards: `y` is negated, so the reversed
    /// circle at `-t` is this one at `t`.
    pub fn reversed(&self) -> Self {
        Self {
            y: -self.y,
            ..*self
        }
    }

    fn axes(&self) -> (Point2, Vector2, Vector2) {
        (self.center, self.x, self.y)
    }

    fn nurbs_arc(&self, range: Interval) -> Result<NurbsCurve2, GeomError> {
        ellipse_arc_axes_2d(self.center, self.x, self.y, self.radius, self.radius, range)
    }
}

impl Circle3 {
    /// The circle of radius `radius` in the XY plane of `frame`, centred on
    /// its origin.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for non-finite input and
    /// [`GeomError::InvalidArgument`] for a non-positive radius or a frame
    /// that is not orthonormal and right-handed (to 1e-9).
    pub fn new(frame: Frame3, radius: f64) -> Result<Self, GeomError> {
        check_frame(&frame)?;
        check_radius(radius)?;
        Ok(Self { frame, radius })
    }

    /// The circle around `center` in the plane with the given normal; the
    /// frame is [`Frame3::from_normal`].
    ///
    /// # Errors
    ///
    /// As [`Circle3::new`], plus [`GeomError::InvalidArgument`] for a zero
    /// normal.
    pub fn from_normal(center: Point3, normal: Vector3, radius: f64) -> Result<Self, GeomError> {
        check_point3(center)?;
        let n = unit3(normal, "a circle normal must be non-zero")?;
        let frame = Frame3::from_normal(center, n)
            .map_err(|_| GeomError::InvalidArgument("a circle normal must be non-zero"))?;
        Self::new(frame, radius)
    }

    /// The frame: the origin is the centre and `z` the normal.
    pub fn frame(&self) -> &Frame3 {
        &self.frame
    }

    /// The centre.
    pub fn center(&self) -> Point3 {
        self.frame.origin
    }

    /// The unit normal, about which the circle runs counter-clockwise.
    pub fn normal(&self) -> Vector3 {
        self.frame.z
    }

    /// The radius.
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// The curvature `1 / radius`.
    pub fn curvature(&self, _t: f64) -> Option<f64> {
        Some(1.0 / self.radius)
    }

    /// The point of the circle closest to `target`; for a point on the
    /// axis, the point at `t = 0`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    pub fn closest_point(&self, target: Point3) -> Result<ClosestPoint<Point3>, GeomError> {
        check_point3(target)?;
        let local = self.frame.to_local(target);
        let t = if local.x.hypot(local.y) > 0.0 {
            wrap_angle(local.y.atan2(local.x))
        } else {
            0.0
        };
        let point = self.point(t);
        Ok(ClosestPoint {
            t,
            point,
            distance: (target - point).length(),
        })
    }

    /// The same circle traversed backwards: the frame becomes `(o, x, -y,
    /// -z)`, so the reversed circle at `-t` is this one at `t`.
    pub fn reversed(&self) -> Self {
        let f = self.frame;
        Self {
            frame: Frame3 {
                y: -f.y,
                z: -f.z,
                ..f
            },
            radius: self.radius,
        }
    }

    fn axes(&self) -> (Point3, Vector3, Vector3) {
        (self.frame.origin, self.frame.x, self.frame.y)
    }

    fn nurbs_arc(&self, range: Interval) -> Result<NurbsCurve3, GeomError> {
        circle_arc(&self.frame, self.radius, range.min, range.max)
    }
}

/// Generates the parts shared by [`Circle2`] and [`Circle3`].
macro_rules! circle_common {
    ($circle:ident, $point:ty, $vector:ty, $aabb:ty, $h:ty, $dims:expr) => {
        impl $circle {
            /// The point at angle `t`.
            pub fn point(&self, t: f64) -> $point {
                let (c, x, y) = self.axes();
                let (s, co) = t.sin_cos();
                c + (x * co + y * s) * self.radius
            }

            /// The point and the derivatives at `t` up to order `n`:
            /// `C^(k)(t) = r (cos^(k)(t) x + sin^(k)(t) y)` for `k >= 1`.
            pub fn derivatives(&self, t: f64, n: usize) -> Vec<$point> {
                let (c, x, y) = self.axes();
                let (s, co) = t.sin_cos();
                (0..derivative_count(n))
                    .map(|k| {
                        let (dc, ds) = trig_derivative(co, s, k);
                        let v = (x * dc + y * ds) * self.radius;
                        if k == 0 { c + v } else { v }
                    })
                    .collect()
            }

            /// The unit tangent `-sin(t) x + cos(t) y`.
            pub fn tangent(&self, t: f64) -> Option<$vector> {
                let (_, x, y) = self.axes();
                let (s, co) = t.sin_cos();
                Some(y * co - x * s)
            }

            /// [`Domain::Periodic`] on `[0, 2 pi]`.
            pub fn domain(&self) -> Domain {
                Domain::Periodic(FULL_TURN)
            }

            /// The exact (tight) bounding box of the arc over `range`; a
            /// range of a full turn or more gives the whole circle.
            ///
            /// # Errors
            ///
            /// [`GeomError::NonFinite`] for a non-finite range.
            pub fn bounds(&self, range: Interval) -> Result<$aabb, GeomError> {
                check_range(range)?;
                let (_, x, y) = self.axes();
                let mut params = vec![range.min, range.max];
                for k in 0..$dims {
                    sinusoid_extrema(range, y[k].atan2(x[k]), &mut params);
                }
                Ok(<$aabb>::from_points(
                    params.into_iter().map(|t| self.point(t)),
                ))
            }

            /// The parameter on [`Self::reversed`] of the point at `t`: `-t`.
            pub fn reversed_parameter(&self, t: f64) -> f64 {
                -t
            }

            /// The exact rational quadratic arc over `range` (see
            /// [`analytic::convert`](crate::analytic::convert) for its
            /// segments and parameter map), on the domain `range`.
            ///
            /// # Errors
            ///
            /// [`GeomError::NonFinite`] for a non-finite range and
            /// [`GeomError::InvalidArgument`] unless its length is in
            /// `(0, 2 pi]`.
            pub fn to_nurbs_mapped(
                &self,
                range: Interval,
            ) -> Result<ConvertedCurve<$h>, GeomError> {
                let nurbs = self.nurbs_arc(range)?;
                let map = ParamMap::arc(nurbs.knots());
                Ok(ConvertedCurve { nurbs, map })
            }

            /// [`Self::to_nurbs_mapped`] without the map.
            ///
            /// # Errors
            ///
            /// As [`Self::to_nurbs_mapped`].
            pub fn to_nurbs(&self, range: Interval) -> Result<NurbsCurve<$h>, GeomError> {
                self.nurbs_arc(range)
            }

            /// The whole curve as a NURBS: [`Self::to_nurbs`] over
            /// `[0, 2 pi]`.
            ///
            /// # Errors
            ///
            /// None in practice; the signature matches [`Self::to_nurbs`].
            pub fn to_nurbs_full(&self) -> Result<NurbsCurve<$h>, GeomError> {
                self.nurbs_arc(FULL_TURN)
            }
        }
    };
}

circle_common!(Circle2, Point2, Vector2, Aabb2, DVec3, 2);
circle_common!(Circle3, Point3, Vector3, Aabb3, DVec4, 3);

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;

    #[test]
    fn circle3_runs_counter_clockwise_about_its_normal() {
        let c = Circle3::new(Frame3::WORLD, 2.0).unwrap();
        assert!((c.point(FRAC_PI_2) - Point3::new(0.0, 2.0, 0.0)).length() < 1e-15);
        let t = c.tangent(0.0).unwrap();
        assert!((t - Vector3::Y).length() < 1e-15);
        let r = c.reversed();
        assert!((r.point(-0.4) - c.point(0.4)).length() < 1e-15);
        assert!((r.normal() + Vector3::Z).length() < 1e-15);
        // On the axis every point is closest; t = 0 is returned.
        let cp = c.closest_point(Point3::new(0.0, 0.0, 3.0)).unwrap();
        assert!(cp.t.abs() < 1e-300 && (cp.distance - 13f64.sqrt()).abs() < 1e-14);
    }

    #[test]
    fn circle2_orientation_sets_the_curvature_sign() {
        let ccw = Circle2::new(Point2::ZERO, 4.0).unwrap();
        let cw = Circle2::from_axis(Point2::ZERO, Vector2::X, false, 4.0).unwrap();
        assert!(ccw.is_counter_clockwise() && !cw.is_counter_clockwise());
        assert!((ccw.curvature(0.3).unwrap() - 0.25).abs() < 1e-15);
        assert!((cw.curvature(0.3).unwrap() + 0.25).abs() < 1e-15);
        assert!((cw.point(FRAC_PI_2) - Point2::new(0.0, -4.0)).length() < 1e-15);
        assert_eq!(ccw.reversed(), cw);
    }

    #[test]
    fn degenerate_circles_are_rejected() {
        assert_eq!(
            Circle2::new(Point2::ZERO, 0.0),
            Err(GeomError::InvalidArgument("a radius must be positive"))
        );
        assert_eq!(
            Circle3::new(Frame3::WORLD, -1.0),
            Err(GeomError::InvalidArgument("a radius must be positive"))
        );
        assert_eq!(
            Circle3::new(Frame3::WORLD, f64::NAN),
            Err(GeomError::NonFinite)
        );
        assert!(Circle3::from_normal(Point3::ZERO, Vector3::ZERO, 1.0).is_err());
        assert!(Circle2::from_axis(Point2::ZERO, Vector2::ZERO, true, 1.0).is_err());
        let skew = Frame3 {
            x: Vector3::new(1.0, 0.1, 0.0),
            ..Frame3::WORLD
        };
        assert!(matches!(
            Circle3::new(skew, 1.0),
            Err(GeomError::InvalidArgument(_))
        ));
    }
}
