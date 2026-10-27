// SPDX-License-Identifier: GPL-3.0-or-later

//! Ring tori.
//!
//! A torus with major radius `R` (from the axis to the centre of the tube)
//! and minor radius `r` (of the tube), `R > r > 0`, about the `z` axis of
//! its frame is `S(u, v) = o + (R + r cos(v)) e(u) + r sin(v) z` with
//! `e(u) = cos(u) x + sin(u) y`: `u` is the angle about the axis from `x`
//! and `v` the angle around the tube, measured from the outer equator
//! towards `+z`. Both are periodic with base period `[0, 2 pi]`.
//!
//! The normal `cos(v) e(u) + sin(v) z` points out of the tube. The
//! principal curvatures, with the sign convention of [`Curvatures`]
//! (positive where the surface bends towards the normal), are
//! `k1 = -cos(v) / (R + r cos(v))` around the axis (`dir1 = e'(u)`, the
//! direction of `S_u`) and `k2 = -1 / r` around the tube; `k1 > k2`
//! everywhere since `R > 0`.
//!
//! The closest point lies in the half-plane through the axis and the
//! target: the angle `u` of the target, then `v` from the tube's centre
//! circle. For a point on the axis (all of a circle is closest) `u = 0`;
//! for a point on the centre circle of the tube (all of a circle around the
//! tube is closest) `v = 0`.

use bevel_math::{Aabb3, Frame3, Interval, Point3, Vector3};

use super::convert::{ConvertedSurface, ParamMap, revolve};
use super::revolved::{axial_angle, radial, revolved_bounds, revolved_derivatives};
use super::{
    Circle3, Domain, FULL_TURN, check_frame, check_point3, check_radius, check_range,
    trig_derivative, wrap_angle,
};
use crate::{Curvatures, GeomError, NurbsSurface, SurfaceClosestPoint, circle_arc};

/// A ring torus: `S(u, v) = o + (R + r cos(v)) e(u) + r sin(v) z` in the
/// frame `(o, x, y, z)`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[must_use]
pub struct Torus {
    frame: Frame3,
    major: f64,
    minor: f64,
}

impl Torus {
    /// The ring torus about the `z` axis of `frame`, centred on its origin.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for non-finite input and
    /// [`GeomError::InvalidArgument`] for a frame that is not orthonormal
    /// and right-handed (to 1e-9) or unless `major_radius > minor_radius >
    /// 0` (horn and spindle tori are not supported).
    pub fn new(frame: Frame3, major_radius: f64, minor_radius: f64) -> Result<Self, GeomError> {
        check_frame(&frame)?;
        check_radius(major_radius)?;
        check_radius(minor_radius)?;
        if minor_radius >= major_radius {
            return Err(GeomError::InvalidArgument(
                "the minor radius of a torus must be less than its major radius",
            ));
        }
        Ok(Self {
            frame,
            major: major_radius,
            minor: minor_radius,
        })
    }

    /// The frame; its `z` axis is the axis of the torus.
    pub fn frame(&self) -> &Frame3 {
        &self.frame
    }

    /// The major radius `R`, from the axis to the centre of the tube.
    pub fn major_radius(&self) -> f64 {
        self.major
    }

    /// The minor radius `r` of the tube.
    pub fn minor_radius(&self) -> f64 {
        self.minor
    }

    /// The point at `(u, v)`.
    pub fn point(&self, u: f64, v: f64) -> Point3 {
        let (s, c) = v.sin_cos();
        self.frame.origin
            + radial(&self.frame, u).0 * (self.major + self.minor * c)
            + self.frame.z * (self.minor * s)
    }

    /// The point and the partial derivatives up to total order `d` (capped
    /// at [`MAX_DERIVATIVE_ORDER`](crate::MAX_DERIVATIVE_ORDER)) in the
    /// layout of [`NurbsSurface::derivatives`](crate::NurbsSurface::derivatives).
    pub fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vector3>> {
        let (s, c) = v.sin_cos();
        let (big, r) = (self.major, self.minor);
        revolved_derivatives(&self.frame, u, d, |l| {
            let (dc, ds) = trig_derivative(c, s, l);
            (if l == 0 { big + r * dc } else { r * dc }, r * ds)
        })
    }

    /// The first partial derivatives `S_u = (R + r cos(v)) e'(u)`,
    /// `S_v = r (-sin(v) e(u) + cos(v) z)`.
    pub fn partials(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        let (e, de) = radial(&self.frame, u);
        let (s, c) = v.sin_cos();
        (
            de * (self.major + self.minor * c),
            (self.frame.z * c - e * s) * self.minor,
        )
    }

    /// The unit normal `cos(v) e(u) + sin(v) z`, out of the tube.
    pub fn normal(&self, u: f64, v: f64) -> Option<Vector3> {
        let (s, c) = v.sin_cos();
        Some(radial(&self.frame, u).0 * c + self.frame.z * s)
    }

    /// `k1 = -cos(v) / (R + r cos(v))` along `e'(u)` and `k2 = -1 / r`
    /// around the tube.
    pub fn principal_curvatures(&self, u: f64, v: f64) -> Option<Curvatures> {
        let (e, de) = radial(&self.frame, u);
        let (s, c) = v.sin_cos();
        let n = e * c + self.frame.z * s;
        Some(Curvatures {
            k1: -c / (self.major + self.minor * c),
            k2: -1.0 / self.minor,
            dir1: de,
            dir2: n.cross(de),
        })
    }

    /// [`Domain::Periodic`] on `[0, 2 pi]`.
    pub fn domain_u(&self) -> Domain {
        Domain::Periodic(FULL_TURN)
    }

    /// [`Domain::Periodic`] on `[0, 2 pi]`.
    pub fn domain_v(&self) -> Domain {
        Domain::Periodic(FULL_TURN)
    }

    /// The point of the torus closest to `target`, with `u` and `v` in
    /// `[0, 2 pi)` (see the module documentation for the axis and the
    /// centre circle).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    pub fn closest_point(&self, target: Point3) -> Result<SurfaceClosestPoint, GeomError> {
        check_point3(target)?;
        let local = self.frame.to_local(target);
        let (u, rho) = axial_angle(local);
        let (qx, qz) = (rho - self.major, local.z);
        let v = if qx.hypot(qz) > 0.0 {
            wrap_angle(qz.atan2(qx))
        } else {
            0.0
        };
        let point = self.point(u, v);
        Ok(SurfaceClosestPoint {
            u,
            v,
            point,
            distance: (target - point).length(),
        })
    }

    /// The isocurve at fixed `u`: the circle of the tube in the frame
    /// `(o + R e(u), e(u), z, e(u) x z)`, whose parameter is `v`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `u`.
    pub fn isocurve_u(&self, u: f64) -> Result<Circle3, GeomError> {
        if !u.is_finite() {
            return Err(GeomError::NonFinite);
        }
        let (e, _) = radial(&self.frame, u);
        let frame = Frame3 {
            origin: self.frame.origin + e * self.major,
            x: e,
            y: self.frame.z,
            z: e.cross(self.frame.z),
        };
        Circle3::new(frame, self.minor)
    }

    /// The isocurve at fixed `v`: the circle of radius `R + r cos(v)` at
    /// height `r sin(v)` about the axis, whose parameter is `u`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `v`.
    pub fn isocurve_v(&self, v: f64) -> Result<Circle3, GeomError> {
        if !v.is_finite() {
            return Err(GeomError::NonFinite);
        }
        let (s, c) = v.sin_cos();
        let frame = Frame3 {
            origin: self.frame.origin + self.frame.z * (self.minor * s),
            ..self.frame
        };
        Circle3::new(frame, self.major + self.minor * c)
    }

    /// The exact bounding box of the patch over `u x v` (both finite).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range.
    pub fn bounds(&self, u: Interval, v: Interval) -> Result<Aabb3, GeomError> {
        revolved_bounds(&self.frame, u, v, true, |u, v| self.point(u, v))
    }

    /// The patch over `u x v` as an exact NURBS: the arc of the tube circle
    /// at angle `u.min` revolved about the axis with
    /// [`NurbsSurface::revolution`], on the domain `u x v`. Both
    /// directions follow the arc map of
    /// [`analytic::convert`](crate::analytic::convert).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range and
    /// [`GeomError::InvalidArgument`] for one of zero length, or a range longer than a full turn.
    pub fn to_nurbs_mapped(&self, u: Interval, v: Interval) -> Result<ConvertedSurface, GeomError> {
        check_range(u)?;
        let (e, _) = radial(&self.frame, u.min);
        let tube = Frame3 {
            origin: self.frame.origin + e * self.major,
            x: e,
            y: self.frame.z,
            z: e.cross(self.frame.z),
        };
        let profile = circle_arc(&tube, self.minor, v.min, v.max)?;
        let nurbs = revolve(&self.frame, &profile, u)?;
        let map_u = ParamMap::arc(nurbs.knots_u());
        let map_v = ParamMap::arc(nurbs.knots_v());
        Ok(ConvertedSurface {
            nurbs,
            map_u,
            map_v,
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

    /// The whole surface as a NURBS: [`Self::to_nurbs`] over `[0, 2 pi] x [0, 2 pi]`.
    ///
    /// # Errors
    ///
    /// None in practice; the signature matches [`Self::to_nurbs`].
    pub fn to_nurbs_full(&self) -> Result<NurbsSurface, GeomError> {
        self.to_nurbs(FULL_TURN, FULL_TURN)
    }
}
