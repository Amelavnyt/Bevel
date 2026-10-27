// SPDX-License-Identifier: GPL-3.0-or-later

//! [`CurveGeom`] and the curve enums [`Curve2`] and [`Curve3`].

use bevel_math::{Aabb2, Aabb3, DMat3, DMat4, DVec3, DVec4, Interval, Point2, Point3};

use super::restriction;
use crate::analytic::transform::{check_invertible3, check_invertible4};
use crate::analytic::{ConvertedCurve, Domain, ParamMap};
use crate::{
    Circle2, Circle3, ClosestPoint, Ellipse2, Ellipse3, GeomError, Homogeneous, Line2, Line3,
    NurbsCurve, NurbsCurve2, NurbsCurve3,
};

/// What every curve provides, in the plane or in space (see the
/// [module documentation](crate::geometry) for the design).
pub trait CurveGeom {
    /// The point type, [`Point2`] or [`Point3`]; vectors (derivatives,
    /// tangents) use it too.
    type Point: Copy;

    /// The homogeneous control point type of the NURBS form: [`DVec3`] for
    /// [`NurbsCurve2`], [`DVec4`] for [`NurbsCurve3`].
    type Homogeneous: Homogeneous<Point = Self::Point>;

    /// The bounding box type, [`Aabb2`] or [`Aabb3`].
    type Bounds;

    /// The transform matrix type: [`DMat3`] acting on `(x, y, 1)` in the
    /// plane, [`DMat4`] acting on `(x, y, z, 1)` in space.
    type Transform;

    /// The curve enum a transform returns, [`Curve2`] or [`Curve3`].
    type Curve;

    /// The point at parameter `t`.
    fn point(&self, t: f64) -> Self::Point;

    /// The point and the derivatives at `t` up to order `n`: `result[k]` is
    /// the `k`-th derivative, with at most
    /// [`MAX_DERIVATIVE_ORDER`](crate::MAX_DERIVATIVE_ORDER)` + 1` entries.
    fn derivatives(&self, t: f64, n: usize) -> Vec<Self::Point>;

    /// The unit tangent at `t`, or `None` where the first derivative
    /// vanishes.
    fn tangent(&self, t: f64) -> Option<Self::Point>;

    /// The curvature at `t` (signed, positive counter-clockwise, in the
    /// plane; non-negative in space), or `None` where the first derivative
    /// vanishes.
    fn curvature(&self, t: f64) -> Option<f64>;

    /// The parameter domain.
    fn domain(&self) -> Domain;

    /// The point of the curve closest to `target` (over the whole domain).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    fn closest_point(&self, target: Self::Point) -> Result<ClosestPoint<Self::Point>, GeomError>;

    /// A box containing the curve over the parameter range `range`: exact
    /// for the analytic types, the box of the control points of that piece
    /// for NURBS.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range and, for NURBS,
    /// [`GeomError::ParameterOutOfDomain`] for one outside the domain.
    fn bounds(&self, range: Interval) -> Result<Self::Bounds, GeomError>;

    /// The curve under the transform `m`, which must be invertible. The
    /// analytic types need an affine `m` and keep their kind where the map
    /// allows (a circle may become an ellipse; see
    /// [`analytic::transform`](crate::analytic::transform) for the
    /// parameters); NURBS curves follow any invertible `m`, projective
    /// ones included.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite matrix,
    /// [`GeomError::InvalidArgument`] for a singular one (or a projective
    /// one for analytic curves) and, for NURBS,
    /// [`GeomError::InvalidWeight`] if a weight stops being positive.
    fn transformed(&self, m: &Self::Transform) -> Result<Self::Curve, GeomError>;

    /// The same curve traversed backwards; [`CurveGeom::reversed_parameter`]
    /// maps the parameters.
    #[must_use]
    fn reversed(&self) -> Self
    where
        Self: Sized;

    /// The parameter on [`CurveGeom::reversed`] of the point at `t`: `-t`
    /// for the analytic types, `a + b - t` for a NURBS on `[a, b]`.
    fn reversed_parameter(&self, t: f64) -> f64;

    /// The exact NURBS form of the curve over `range`, with the map between
    /// the parameters (see [`analytic::convert`](crate::analytic::convert)).
    /// A NURBS curve gives its piece over `range` with
    /// [`ParamMap::Identity`].
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range,
    /// [`GeomError::InvalidArgument`] for one of zero length (or, for
    /// periodic curves, longer than a period) and, for NURBS,
    /// [`GeomError::ParameterOutOfDomain`] for one outside the domain.
    fn to_nurbs_mapped(
        &self,
        range: Interval,
    ) -> Result<ConvertedCurve<Self::Homogeneous>, GeomError>;

    /// [`CurveGeom::to_nurbs_mapped`] without the map.
    ///
    /// # Errors
    ///
    /// As [`CurveGeom::to_nurbs_mapped`].
    fn to_nurbs(&self, range: Interval) -> Result<NurbsCurve<Self::Homogeneous>, GeomError> {
        self.to_nurbs_mapped(range).map(|c| c.nurbs)
    }

    /// The NURBS form over the whole domain (one period of a periodic
    /// curve).
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidArgument`] for an unbounded curve, and as
    /// [`CurveGeom::to_nurbs`].
    fn to_nurbs_full(&self) -> Result<NurbsCurve<Self::Homogeneous>, GeomError> {
        let range = self.domain().interval().ok_or(GeomError::InvalidArgument(
            "an unbounded curve needs a parameter range to become a NURBS",
        ))?;
        self.to_nurbs(range)
    }
}

/// A curve in the plane: analytic or NURBS.
#[derive(Clone, Debug, PartialEq)]
pub enum Curve2 {
    /// A straight line.
    Line(Line2),
    /// A circle.
    Circle(Circle2),
    /// An ellipse.
    Ellipse(Ellipse2),
    /// A NURBS curve.
    Nurbs(NurbsCurve2),
}

/// A curve in space: analytic or NURBS.
#[derive(Clone, Debug, PartialEq)]
pub enum Curve3 {
    /// A straight line.
    Line(Line3),
    /// A circle.
    Circle(Circle3),
    /// An ellipse.
    Ellipse(Ellipse3),
    /// A NURBS curve.
    Nurbs(NurbsCurve3),
}

/// Implements [`CurveGeom`] for an analytic type by delegating to its own
/// methods of the same names.
macro_rules! analytic_curve {
    ($ty:ty, $point:ty, $h:ty, $aabb:ty, $m:ty, $enum:ty) => {
        impl CurveGeom for $ty {
            type Point = $point;
            type Homogeneous = $h;
            type Bounds = $aabb;
            type Transform = $m;
            type Curve = $enum;

            fn transformed(&self, m: &$m) -> Result<$enum, GeomError> {
                <$ty>::transformed(self, m)
            }

            fn point(&self, t: f64) -> $point {
                <$ty>::point(self, t)
            }

            fn derivatives(&self, t: f64, n: usize) -> Vec<$point> {
                <$ty>::derivatives(self, t, n)
            }

            fn tangent(&self, t: f64) -> Option<$point> {
                <$ty>::tangent(self, t)
            }

            fn curvature(&self, t: f64) -> Option<f64> {
                <$ty>::curvature(self, t)
            }

            fn domain(&self) -> Domain {
                <$ty>::domain(self)
            }

            fn closest_point(&self, target: $point) -> Result<ClosestPoint<$point>, GeomError> {
                <$ty>::closest_point(self, target)
            }

            fn bounds(&self, range: Interval) -> Result<$aabb, GeomError> {
                <$ty>::bounds(self, range)
            }

            fn reversed(&self) -> Self {
                <$ty>::reversed(self)
            }

            fn reversed_parameter(&self, t: f64) -> f64 {
                <$ty>::reversed_parameter(self, t)
            }

            fn to_nurbs_mapped(&self, range: Interval) -> Result<ConvertedCurve<$h>, GeomError> {
                <$ty>::to_nurbs_mapped(self, range)
            }
        }
    };
}

analytic_curve!(Line2, Point2, DVec3, Aabb2, DMat3, Curve2);
analytic_curve!(Line3, Point3, DVec4, Aabb3, DMat4, Curve3);
analytic_curve!(Circle2, Point2, DVec3, Aabb2, DMat3, Curve2);
analytic_curve!(Circle3, Point3, DVec4, Aabb3, DMat4, Curve3);
analytic_curve!(Ellipse2, Point2, DVec3, Aabb2, DMat3, Curve2);
analytic_curve!(Ellipse3, Point3, DVec4, Aabb3, DMat4, Curve3);

/// The piece of `c` over `range` (the curve itself when `range` covers the
/// domain).
fn restricted<H: Homogeneous>(
    c: &NurbsCurve<H>,
    range: Interval,
) -> Result<NurbsCurve<H>, GeomError> {
    let tol = c.knots().tolerance();
    let Some(r) = restriction(c.domain(), range, tol)? else {
        return Ok(c.clone());
    };
    if r.length() <= tol {
        return Err(GeomError::InvalidArgument(
            "a parameter range must have positive length",
        ));
    }
    let (a, b) = c.domain();
    let mut piece = c.clone();
    if r.min > a + tol {
        piece = piece.split(r.min)?.1;
    }
    if r.max < b - tol {
        piece = piece.split(r.max)?.0;
    }
    Ok(piece)
}

/// Implements [`CurveGeom`] for a NURBS curve type by delegating to its own
/// methods.
macro_rules! nurbs_curve {
    ($ty:ty, $point:ty, $h:ty, $aabb:ty, $m:ty, $enum:ident, $check:ident) => {
        impl CurveGeom for $ty {
            type Point = $point;
            type Homogeneous = $h;
            type Bounds = $aabb;
            type Transform = $m;
            type Curve = $enum;

            /// The transformed control points (exact), after checking that
            /// `m` is invertible.
            fn transformed(&self, m: &$m) -> Result<$enum, GeomError> {
                $check(m)?;
                <$ty>::transformed(self, m).map($enum::Nurbs)
            }

            fn point(&self, t: f64) -> $point {
                <$ty>::point(self, t)
            }

            fn derivatives(&self, t: f64, n: usize) -> Vec<$point> {
                <$ty>::derivatives(self, t, n)
            }

            fn tangent(&self, t: f64) -> Option<$point> {
                <$ty>::tangent(self, t)
            }

            fn curvature(&self, t: f64) -> Option<f64> {
                <$ty>::curvature(self, t)
            }

            /// [`Domain::Bounded`] on the knot domain.
            fn domain(&self) -> Domain {
                let (a, b) = <$ty>::domain(self);
                Domain::Bounded(Interval::new(a, b))
            }

            fn closest_point(&self, target: $point) -> Result<ClosestPoint<$point>, GeomError> {
                <$ty>::closest_point(self, target)
            }

            /// The box of the control points of the piece over `range`
            /// (which contains it, by the convex hull property).
            fn bounds(&self, range: Interval) -> Result<$aabb, GeomError> {
                let tol = self.knots().tolerance();
                if let Some(r) = restriction(<$ty>::domain(self), range, tol)? {
                    if r.length() <= tol {
                        return Ok(<$aabb>::from_points([<$ty>::point(self, r.min)]));
                    }
                }
                Ok(restricted(self, range)?.control_bounds())
            }

            fn reversed(&self) -> Self {
                <$ty>::reversed(self)
            }

            /// `a + b - t` on the domain `[a, b]`.
            fn reversed_parameter(&self, t: f64) -> f64 {
                let (a, b) = <$ty>::domain(self);
                (a + b) - t
            }

            fn to_nurbs_mapped(&self, range: Interval) -> Result<ConvertedCurve<$h>, GeomError> {
                Ok(ConvertedCurve {
                    nurbs: restricted(self, range)?,
                    map: ParamMap::Identity,
                })
            }
        }
    };
}

nurbs_curve!(
    NurbsCurve2,
    Point2,
    DVec3,
    Aabb2,
    DMat3,
    Curve2,
    check_invertible3
);
nurbs_curve!(
    NurbsCurve3,
    Point3,
    DVec4,
    Aabb3,
    DMat4,
    Curve3,
    check_invertible4
);

/// Implements [`CurveGeom`] and the `From` conversions for a curve enum by
/// dispatching to the variant.
macro_rules! curve_enum {
    ($enum:ident, $point:ty, $h:ty, $aabb:ty, $m:ty, $line:ty, $circle:ty, $ellipse:ty, $nurbs:ty) => {
        impl From<$line> for $enum {
            fn from(c: $line) -> Self {
                Self::Line(c)
            }
        }

        impl From<$circle> for $enum {
            fn from(c: $circle) -> Self {
                Self::Circle(c)
            }
        }

        impl From<$ellipse> for $enum {
            fn from(c: $ellipse) -> Self {
                Self::Ellipse(c)
            }
        }

        impl From<$nurbs> for $enum {
            fn from(c: $nurbs) -> Self {
                Self::Nurbs(c)
            }
        }

        impl $enum {
            /// Returns `true` for an analytic (not NURBS) curve.
            pub fn is_analytic(&self) -> bool {
                !matches!(self, Self::Nurbs(_))
            }
        }

        impl CurveGeom for $enum {
            type Point = $point;
            type Homogeneous = $h;
            type Bounds = $aabb;
            type Transform = $m;
            type Curve = $enum;

            fn transformed(&self, m: &$m) -> Result<$enum, GeomError> {
                dispatch!($enum, self, c => CurveGeom::transformed(c, m))
            }

            fn point(&self, t: f64) -> $point {
                dispatch!($enum, self, c => CurveGeom::point(c, t))
            }

            fn derivatives(&self, t: f64, n: usize) -> Vec<$point> {
                dispatch!($enum, self, c => CurveGeom::derivatives(c, t, n))
            }

            fn tangent(&self, t: f64) -> Option<$point> {
                dispatch!($enum, self, c => CurveGeom::tangent(c, t))
            }

            fn curvature(&self, t: f64) -> Option<f64> {
                dispatch!($enum, self, c => CurveGeom::curvature(c, t))
            }

            fn domain(&self) -> Domain {
                dispatch!($enum, self, c => CurveGeom::domain(c))
            }

            fn closest_point(&self, target: $point) -> Result<ClosestPoint<$point>, GeomError> {
                dispatch!($enum, self, c => CurveGeom::closest_point(c, target))
            }

            fn bounds(&self, range: Interval) -> Result<$aabb, GeomError> {
                dispatch!($enum, self, c => CurveGeom::bounds(c, range))
            }

            fn reversed(&self) -> Self {
                match self {
                    Self::Line(c) => Self::Line(CurveGeom::reversed(c)),
                    Self::Circle(c) => Self::Circle(CurveGeom::reversed(c)),
                    Self::Ellipse(c) => Self::Ellipse(CurveGeom::reversed(c)),
                    Self::Nurbs(c) => Self::Nurbs(CurveGeom::reversed(c)),
                }
            }

            fn reversed_parameter(&self, t: f64) -> f64 {
                dispatch!($enum, self, c => CurveGeom::reversed_parameter(c, t))
            }

            fn to_nurbs_mapped(&self, range: Interval) -> Result<ConvertedCurve<$h>, GeomError> {
                dispatch!($enum, self, c => CurveGeom::to_nurbs_mapped(c, range))
            }
        }
    };
}

/// `match` over the variants of a curve enum, binding each to `$c`.
macro_rules! dispatch {
    ($enum:ident, $self:expr, $c:ident => $e:expr) => {
        match $self {
            $enum::Line($c) => $e,
            $enum::Circle($c) => $e,
            $enum::Ellipse($c) => $e,
            $enum::Nurbs($c) => $e,
        }
    };
}

curve_enum!(
    Curve2,
    Point2,
    DVec3,
    Aabb2,
    DMat3,
    Line2,
    Circle2,
    Ellipse2,
    NurbsCurve2
);
curve_enum!(
    Curve3,
    Point3,
    DVec4,
    Aabb3,
    DMat4,
    Line3,
    Circle3,
    Ellipse3,
    NurbsCurve3
);
