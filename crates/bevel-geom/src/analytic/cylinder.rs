// SPDX-License-Identifier: GPL-3.0-or-later

//! Circular cylinders.
//!
//! A cylinder of radius `r` about the `z` axis of its frame is
//! `S(u, v) = o + r e(u) + v z` with `e(u) = cos(u) x + sin(u) y`: `u` is
//! the angle about the axis from `x` (periodic, base period `[0, 2 pi]`)
//! and `v` the height along the axis (unbounded).
//!
//! The normal `S_u x S_v / |S_u x S_v| = e(u)` points away from the axis.
//! The principal curvatures, with the sign convention of
//! [`Curvatures`] (positive where the surface bends towards the normal),
//! are `k1 = 0` along the axis (`dir1 = z`) and `k2 = -1 / r` around it.
//!
//! The closest point to a point on the axis, where the whole circle at that
//! height is equally close, is the one at `u = 0`.

use bevel_math::{Aabb3, DVec4, Frame3, Interval, Point3, Vector3};

use super::convert::{ConvertedSurface, ParamMap, revolve, segment_curve};
use super::revolved::{axial_angle, radial, revolved_bounds, revolved_derivatives};
use super::{
    Circle3, Domain, FULL_TURN, Line3, check_frame, check_point3, check_radius, check_range,
};
use crate::{Curvatures, GeomError, NurbsSurface, SurfaceClosestPoint};

/// An unbounded circular cylinder: `S(u, v) = o + r e(u) + v z` in the
/// frame `(o, x, y, z)`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[must_use]
pub struct Cylinder {
    frame: Frame3,
    radius: f64,
}

impl Cylinder {
    /// The cylinder of radius `radius` about the `z` axis of `frame`.
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

    /// The frame; its `z` axis is the axis of the cylinder.
    pub fn frame(&self) -> &Frame3 {
        &self.frame
    }

    /// The radius.
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// The point at `(u, v)`.
    pub fn point(&self, u: f64, v: f64) -> Point3 {
        self.frame.origin + radial(&self.frame, u).0 * self.radius + self.frame.z * v
    }

    /// The point and the partial derivatives up to total order `d` (capped
    /// at [`MAX_DERIVATIVE_ORDER`](crate::MAX_DERIVATIVE_ORDER)) in the
    /// layout of [`NurbsSurface::derivatives`](crate::NurbsSurface::derivatives).
    pub fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vector3>> {
        revolved_derivatives(&self.frame, u, d, |l| match l {
            0 => (self.radius, v),
            1 => (0.0, 1.0),
            _ => (0.0, 0.0),
        })
    }

    /// The first partial derivatives `S_u = r e'(u)`, `S_v = z`.
    pub fn partials(&self, u: f64, _v: f64) -> (Vector3, Vector3) {
        (radial(&self.frame, u).1 * self.radius, self.frame.z)
    }

    /// The unit normal `e(u)`, away from the axis.
    pub fn normal(&self, u: f64, _v: f64) -> Option<Vector3> {
        Some(radial(&self.frame, u).0)
    }

    /// `k1 = 0` along the axis (`dir1 = z`) and `k2 = -1 / r` around it
    /// (`dir2 = e(u) x z = -e'(u)`).
    pub fn principal_curvatures(&self, u: f64, _v: f64) -> Option<Curvatures> {
        let (e, _) = radial(&self.frame, u);
        Some(Curvatures {
            k1: 0.0,
            k2: -1.0 / self.radius,
            dir1: self.frame.z,
            dir2: e.cross(self.frame.z),
        })
    }

    /// [`Domain::Periodic`] on `[0, 2 pi]`.
    pub fn domain_u(&self) -> Domain {
        Domain::Periodic(FULL_TURN)
    }

    /// [`Domain::Unbounded`].
    pub fn domain_v(&self) -> Domain {
        Domain::Unbounded
    }

    /// The point of the cylinder closest to `target`, with `u` in `[0, 2
    /// pi)`; for a point on the axis, the one at `u = 0`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    pub fn closest_point(&self, target: Point3) -> Result<SurfaceClosestPoint, GeomError> {
        check_point3(target)?;
        let local = self.frame.to_local(target);
        let (u, _) = axial_angle(local);
        let point = self.point(u, local.z);
        Ok(SurfaceClosestPoint {
            u,
            v: local.z,
            point,
            distance: (target - point).length(),
        })
    }

    /// The isocurve at fixed `u`: the ruling through `S(u, 0)` along `z`,
    /// whose parameter is `v`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `u`.
    pub fn isocurve_u(&self, u: f64) -> Result<Line3, GeomError> {
        Line3::new(self.point(u, 0.0), self.frame.z)
    }

    /// The isocurve at fixed `v`: the circle of radius `r` at height `v`,
    /// whose parameter is `u`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `v`.
    pub fn isocurve_v(&self, v: f64) -> Result<Circle3, GeomError> {
        let frame = Frame3 {
            origin: self.frame.origin + self.frame.z * v,
            ..self.frame
        };
        Circle3::new(frame, self.radius)
    }

    /// The exact bounding box of the patch over `u x v` (both finite; a
    /// `u` range of a full turn or more gives the whole circumference).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range.
    pub fn bounds(&self, u: Interval, v: Interval) -> Result<Aabb3, GeomError> {
        revolved_bounds(&self.frame, u, v, false, |u, v| self.point(u, v))
    }

    /// The patch over `u x v` as an exact NURBS: the line segment at angle
    /// `u.min` revolved about the axis with
    /// [`NurbsSurface::revolution`], on the domain `u x v`. `u` follows
    /// the arc map of [`analytic::convert`](crate::analytic::convert) and
    /// `v` is unchanged.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range and
    /// [`GeomError::InvalidArgument`] for one of zero length, or a `u` range longer than a full turn.
    pub fn to_nurbs_mapped(&self, u: Interval, v: Interval) -> Result<ConvertedSurface, GeomError> {
        check_range(u)?;
        let profile =
            segment_curve::<DVec4>(self.point(u.min, v.min), self.point(u.min, v.max), v)?;
        let nurbs = revolve(&self.frame, &profile, u)?;
        let map_u = ParamMap::arc(nurbs.knots_u());
        Ok(ConvertedSurface {
            nurbs,
            map_u,
            map_v: ParamMap::Identity,
        })
    }

    /// [`Self::to_nurbs_mapped`] without the maps.
    ///
    /// # Errors
    ///
    /// As [`Self::to_nurbs_mapped`].
    pub fn to_nurbs(&self, u: Interval, v: Interval) -> Result<NurbsSurface, GeomError> {
        self.to_nurbs_mapped(u, v).map(|c| c.nurbs)
    }
}
