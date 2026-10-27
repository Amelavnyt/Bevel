// SPDX-License-Identifier: GPL-3.0-or-later

//! Ellipses in the plane and in space.
//!
//! An ellipse is `C(t) = c + a cos(t) x + b sin(t) y` with orthonormal axes
//! `x` (the major axis) and `y`, the major radius `a` and the minor radius
//! `b`, `a >= b > 0`. The parameter `t` is the eccentric angle (not the
//! polar angle of the point), periodic with period `2 pi`. As for circles,
//! an [`Ellipse3`] takes its axes from a right-handed [`Frame3`] and an
//! [`Ellipse2`] has `y = +-perp(x)`. A circle is the ellipse with `a = b`.
//!
//! Reversal negates `y` (and `z`), so the reversed ellipse at `-t` is the
//! original at `t`.
//!
//! The closest point is found in the plane of the ellipse with Eberly's
//! robust bisection (see the source of `closest_on_ellipse`); a point off
//! the plane has the closest point of its projection. For the centre the
//! answer is the end of the minor axis at `t = pi / 2` (both ends are
//! equally close; for `a = b`, `t = 0`).

use bevel_math::{Aabb2, Aabb3, DVec3, DVec4, Frame3, Interval, Point2, Point3, Vector2, Vector3};

use super::convert::{ConvertedCurve, ParamMap, ellipse_arc_axes_2d};

use super::{
    Domain, FULL_TURN, check_frame, check_point2, check_point3, check_range, closest_on_ellipse,
    derivative_count, sinusoid_extrema, trig_derivative, unit2, wrap_angle,
};
use crate::{ClosestPoint, GeomError, NurbsCurve, NurbsCurve2, NurbsCurve3, ellipse_arc};

/// An ellipse in the plane with its own axes:
/// `C(t) = center + a cos(t) x + b sin(t) y` with the major axis `x` and
/// `y = +-perp(x)`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[must_use]
pub struct Ellipse2 {
    center: Point2,
    x: Vector2,
    y: Vector2,
    major: f64,
    minor: f64,
}

/// An ellipse in space: `C(t) = o + a cos(t) x + b sin(t) y` in the frame
/// `(o, x, y, z)`, with the major axis along `x`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[must_use]
pub struct Ellipse3 {
    frame: Frame3,
    major: f64,
    minor: f64,
}

/// Checks `major >= minor > 0`, both finite.
fn check_radii(major: f64, minor: f64) -> Result<(), GeomError> {
    if !major.is_finite() || !minor.is_finite() {
        Err(GeomError::NonFinite)
    } else if minor <= 0.0 {
        Err(GeomError::InvalidArgument("a radius must be positive"))
    } else if major < minor {
        Err(GeomError::InvalidArgument(
            "the major radius of an ellipse must be at least its minor radius",
        ))
    } else {
        Ok(())
    }
}

impl Ellipse2 {
    /// The counter-clockwise ellipse around `center` with the major radius
    /// along `major_axis` (normalised).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for non-finite input and
    /// [`GeomError::InvalidArgument`] for a zero axis or unless
    /// `major_radius >= minor_radius > 0`.
    pub fn new(
        center: Point2,
        major_axis: Vector2,
        major_radius: f64,
        minor_radius: f64,
    ) -> Result<Self, GeomError> {
        Self::from_axis(center, major_axis, true, major_radius, minor_radius)
    }

    /// As [`Ellipse2::new`], running counter-clockwise or clockwise.
    ///
    /// # Errors
    ///
    /// As [`Ellipse2::new`].
    pub fn from_axis(
        center: Point2,
        major_axis: Vector2,
        counter_clockwise: bool,
        major_radius: f64,
        minor_radius: f64,
    ) -> Result<Self, GeomError> {
        check_point2(center)?;
        check_radii(major_radius, minor_radius)?;
        let x = unit2(major_axis, "an ellipse axis must be non-zero")?;
        let y = if counter_clockwise {
            x.perp()
        } else {
            -x.perp()
        };
        Ok(Self {
            center,
            x,
            y,
            major: major_radius,
            minor: minor_radius,
        })
    }

    /// The centre.
    pub fn center(&self) -> Point2 {
        self.center
    }

    /// The unit major axis, towards the point at `t = 0`.
    pub fn x_axis(&self) -> Vector2 {
        self.x
    }

    /// The unit minor axis, towards the point at `t = pi / 2`.
    pub fn y_axis(&self) -> Vector2 {
        self.y
    }

    /// Returns `true` if the ellipse runs counter-clockwise.
    pub fn is_counter_clockwise(&self) -> bool {
        self.x.perp_dot(self.y) > 0.0
    }

    /// The signed curvature `+-a b / |C'|^3` (positive counter-clockwise).
    pub fn curvature(&self, t: f64) -> Option<f64> {
        let speed = self.speed(t);
        Some(self.x.perp_dot(self.y) * self.major * self.minor / (speed * speed * speed))
    }

    /// The point of the ellipse closest to `target` (see the module
    /// documentation).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    pub fn closest_point(&self, target: Point2) -> Result<ClosestPoint<Point2>, GeomError> {
        check_point2(target)?;
        let d = target - self.center;
        let t = self.local_closest(d.dot(self.x), d.dot(self.y));
        let point = self.point(t);
        Ok(ClosestPoint {
            t,
            point,
            distance: (target - point).length(),
        })
    }

    /// The same ellipse traversed backwards: `y` is negated, so the reversed
    /// ellipse at `-t` is this one at `t`.
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
        ellipse_arc_axes_2d(self.center, self.x, self.y, self.major, self.minor, range)
    }
}

impl Ellipse3 {
    /// The ellipse in the XY plane of `frame`, centred on its origin, with
    /// the major radius along the frame's `x` axis.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for non-finite input and
    /// [`GeomError::InvalidArgument`] for a frame that is not orthonormal
    /// and right-handed (to 1e-9) or unless `major_radius >= minor_radius >
    /// 0`.
    pub fn new(frame: Frame3, major_radius: f64, minor_radius: f64) -> Result<Self, GeomError> {
        check_frame(&frame)?;
        check_radii(major_radius, minor_radius)?;
        Ok(Self {
            frame,
            major: major_radius,
            minor: minor_radius,
        })
    }

    /// The frame: the origin is the centre, `x` the major axis and `z` the
    /// normal.
    pub fn frame(&self) -> &Frame3 {
        &self.frame
    }

    /// The centre.
    pub fn center(&self) -> Point3 {
        self.frame.origin
    }

    /// The unit normal, about which the ellipse runs counter-clockwise.
    pub fn normal(&self) -> Vector3 {
        self.frame.z
    }

    /// The curvature `a b / |C'|^3`.
    pub fn curvature(&self, t: f64) -> Option<f64> {
        let speed = self.speed(t);
        Some(self.major * self.minor / (speed * speed * speed))
    }

    /// The point of the ellipse closest to `target` (see the module
    /// documentation).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    pub fn closest_point(&self, target: Point3) -> Result<ClosestPoint<Point3>, GeomError> {
        check_point3(target)?;
        let local = self.frame.to_local(target);
        let t = self.local_closest(local.x, local.y);
        let point = self.point(t);
        Ok(ClosestPoint {
            t,
            point,
            distance: (target - point).length(),
        })
    }

    /// The same ellipse traversed backwards: the frame becomes `(o, x, -y,
    /// -z)`, so the reversed ellipse at `-t` is this one at `t`.
    pub fn reversed(&self) -> Self {
        let f = self.frame;
        Self {
            frame: Frame3 {
                y: -f.y,
                z: -f.z,
                ..f
            },
            ..*self
        }
    }

    fn axes(&self) -> (Point3, Vector3, Vector3) {
        (self.frame.origin, self.frame.x, self.frame.y)
    }

    fn nurbs_arc(&self, range: Interval) -> Result<NurbsCurve3, GeomError> {
        ellipse_arc(&self.frame, self.major, self.minor, range.min, range.max)
    }
}

/// Generates the parts shared by [`Ellipse2`] and [`Ellipse3`].
macro_rules! ellipse_common {
    ($ellipse:ident, $point:ty, $vector:ty, $aabb:ty, $h:ty, $dims:expr) => {
        impl $ellipse {
            /// The major radius `a`, along the `x` axis.
            pub fn major_radius(&self) -> f64 {
                self.major
            }

            /// The minor radius `b`, along the `y` axis.
            pub fn minor_radius(&self) -> f64 {
                self.minor
            }

            /// The point at eccentric angle `t`.
            pub fn point(&self, t: f64) -> $point {
                let (c, x, y) = self.axes();
                let (s, co) = t.sin_cos();
                c + x * (self.major * co) + y * (self.minor * s)
            }

            /// The point and the derivatives at `t` up to order `n`:
            /// `C^(k)(t) = a cos^(k)(t) x + b sin^(k)(t) y` for `k >= 1`.
            pub fn derivatives(&self, t: f64, n: usize) -> Vec<$point> {
                let (c, x, y) = self.axes();
                let (s, co) = t.sin_cos();
                (0..derivative_count(n))
                    .map(|k| {
                        let (dc, ds) = trig_derivative(co, s, k);
                        let v = x * (self.major * dc) + y * (self.minor * ds);
                        if k == 0 { c + v } else { v }
                    })
                    .collect()
            }

            /// The speed `|C'(t)| = sqrt(a^2 sin^2 t + b^2 cos^2 t)`, at least
            /// `b`.
            fn speed(&self, t: f64) -> f64 {
                let (s, co) = t.sin_cos();
                (self.major * s).hypot(self.minor * co)
            }

            /// The unit tangent `C'(t) / |C'(t)|`.
            pub fn tangent(&self, t: f64) -> Option<$vector> {
                let (_, x, y) = self.axes();
                let (s, co) = t.sin_cos();
                let d = y * (self.minor * co) - x * (self.major * s);
                Some(d / self.speed(t))
            }

            /// [`Domain::Periodic`] on `[0, 2 pi]`.
            pub fn domain(&self) -> Domain {
                Domain::Periodic(FULL_TURN)
            }

            /// The exact (tight) bounding box of the arc over `range`; a
            /// range of a full turn or more gives the whole ellipse.
            ///
            /// # Errors
            ///
            /// [`GeomError::NonFinite`] for a non-finite range.
            pub fn bounds(&self, range: Interval) -> Result<$aabb, GeomError> {
                check_range(range)?;
                let (_, x, y) = self.axes();
                let mut params = vec![range.min, range.max];
                for k in 0..$dims {
                    let phase = (self.minor * y[k]).atan2(self.major * x[k]);
                    sinusoid_extrema(range, phase, &mut params);
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

            /// The eccentric angle of the point closest to the point with
            /// coordinates `(lx, ly)` along the axes.
            fn local_closest(&self, lx: f64, ly: f64) -> f64 {
                let q = closest_on_ellipse(self.major, self.minor, Point2::new(lx, ly));
                wrap_angle((q.y / self.minor).atan2(q.x / self.major))
            }
        }
    };
}

ellipse_common!(Ellipse2, Point2, Vector2, Aabb2, DVec3, 2);
ellipse_common!(Ellipse3, Point3, Vector3, Aabb3, DVec4, 3);

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;

    #[test]
    fn ellipse_vertices_and_curvature() {
        let e = Ellipse3::new(Frame3::WORLD, 4.0, 1.0).unwrap();
        assert!((e.point(0.0) - Point3::new(4.0, 0.0, 0.0)).length() < 1e-15);
        assert!((e.point(FRAC_PI_2) - Point3::new(0.0, 1.0, 0.0)).length() < 1e-15);
        // a / b^2 at the end of the major axis, b / a^2 at the minor.
        assert!((e.curvature(0.0).unwrap() - 4.0).abs() < 1e-14);
        assert!((e.curvature(FRAC_PI_2).unwrap() - 1.0 / 16.0).abs() < 1e-15);
        let e2 = Ellipse2::from_axis(Point2::ZERO, Vector2::Y, false, 4.0, 1.0).unwrap();
        assert!((e2.curvature(0.0).unwrap() + 4.0).abs() < 1e-14);
        // The centre maps to the end of the minor axis.
        let cp = e.closest_point(Point3::ZERO).unwrap();
        assert!((cp.t - FRAC_PI_2).abs() < 1e-15 && (cp.distance - 1.0).abs() < 1e-15);
    }

    #[test]
    fn degenerate_ellipses_are_rejected() {
        assert_eq!(
            Ellipse3::new(Frame3::WORLD, 1.0, 2.0),
            Err(GeomError::InvalidArgument(
                "the major radius of an ellipse must be at least its minor radius"
            ))
        );
        assert_eq!(
            Ellipse3::new(Frame3::WORLD, 1.0, 0.0),
            Err(GeomError::InvalidArgument("a radius must be positive"))
        );
        assert_eq!(
            Ellipse2::new(Point2::ZERO, Vector2::ZERO, 2.0, 1.0),
            Err(GeomError::InvalidArgument(
                "an ellipse axis must be non-zero"
            ))
        );
        assert_eq!(
            Ellipse2::new(Point2::ZERO, Vector2::X, f64::INFINITY, 1.0),
            Err(GeomError::NonFinite)
        );
        assert!(Ellipse3::new(Frame3::WORLD, 2.0, 2.0).is_ok());
    }
}
