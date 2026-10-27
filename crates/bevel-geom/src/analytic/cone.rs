// SPDX-License-Identifier: GPL-3.0-or-later

//! Circular cones.
//!
//! A cone about the `z` axis of its frame with radius `r0` at `v = 0` and
//! half-angle `alpha` in `(0, pi / 2)` is
//! `S(u, v) = o + rho(v) e(u) + v z` with `rho(v) = r0 + v tan(alpha)` and
//! `e(u) = cos(u) x + sin(u) y`: `u` is the angle about the axis from `x`
//! (periodic, base period `[0, 2 pi]`) and `v` the distance along the axis
//! from the plane of the frame (unbounded). The cone widens towards `+z`;
//! its apex is at `v = -r0 / tan(alpha)`, the point `o - (r0 / tan(alpha))
//! z`. Beyond the apex `rho(v)` is negative and the parametrization covers
//! the second nappe (`S(u, v)` then lies at angle `u + pi`), so the
//! surface is the full double cone.
//!
//! The normal `S_u x S_v / |S_u x S_v| = sign(rho) (cos(alpha) e(u) -
//! sin(alpha) z)` points away from the axis on both nappes and does not
//! exist at the apex. The principal curvatures, with the sign convention
//! of [`Curvatures`] (positive where the surface bends towards the normal),
//! are `k1 = 0` along the ruling (`dir1 = sin(alpha) e(u) + cos(alpha) z`)
//! and `k2 = -cos(alpha) / |rho(v)|` around the axis.
//!
//! The closest point is found in the plane through the axis and the
//! target, which meets the cone in two lines through the apex (the rulings
//! at `u` and `u + pi`): the nearer foot of the perpendiculars wins, the
//! ruling at `u` on a tie. For a point on the axis `u = 0`.

use std::f64::consts::{FRAC_PI_2, PI};

use bevel_math::{Aabb3, DVec4, Frame3, Interval, Point3, Vector3};

use super::convert::{ConvertedSurface, ParamMap, revolve, segment_curve};
use super::revolved::{axial_angle, radial, revolved_bounds, revolved_derivatives};
use super::{
    Circle3, Domain, FULL_TURN, Line3, check_frame, check_point3, check_range, wrap_angle,
};
use crate::{Curvatures, GeomError, NurbsSurface, SurfaceClosestPoint};

/// Relative size of `rho(v)` at or below which `v` is the apex.
const APEX_TOLERANCE: f64 = 1e-12;

/// An unbounded circular (double) cone: `S(u, v) = o + (r0 + v
/// tan(alpha)) e(u) + v z` in the frame `(o, x, y, z)`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[must_use]
pub struct Cone {
    frame: Frame3,
    radius: f64,
    half_angle: f64,
}

impl Cone {
    /// The cone about the `z` axis of `frame` with radius `radius` (zero
    /// puts the apex at the frame's origin) in the frame's XY plane and
    /// half-angle `half_angle`, widening towards `+z`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for non-finite input and
    /// [`GeomError::InvalidArgument`] for a negative radius, a half-angle
    /// outside `(0, pi / 2)` or a frame that is not orthonormal and
    /// right-handed (to 1e-9).
    pub fn new(frame: Frame3, radius: f64, half_angle: f64) -> Result<Self, GeomError> {
        check_frame(&frame)?;
        if !radius.is_finite() || !half_angle.is_finite() {
            return Err(GeomError::NonFinite);
        }
        if radius < 0.0 {
            return Err(GeomError::InvalidArgument(
                "the radius of a cone must not be negative",
            ));
        }
        if half_angle <= 0.0 || half_angle >= FRAC_PI_2 {
            return Err(GeomError::InvalidArgument(
                "the half-angle of a cone must lie strictly between 0 and pi / 2",
            ));
        }
        Ok(Self {
            frame,
            radius,
            half_angle,
        })
    }

    /// The frame; its `z` axis is the axis of the cone.
    pub fn frame(&self) -> &Frame3 {
        &self.frame
    }

    /// The radius `r0` at `v = 0`.
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// The half-angle `alpha`, between the axis and a ruling.
    pub fn half_angle(&self) -> f64 {
        self.half_angle
    }

    /// The signed radius `rho(v) = r0 + v tan(alpha)`; negative beyond the
    /// apex.
    pub fn radius_at(&self, v: f64) -> f64 {
        self.radius + v * self.half_angle.tan()
    }

    /// The parameter `v = -r0 / tan(alpha)` of the apex.
    pub fn apex_parameter(&self) -> f64 {
        -self.radius / self.half_angle.tan()
    }

    /// The apex.
    pub fn apex(&self) -> Point3 {
        self.frame.origin + self.frame.z * self.apex_parameter()
    }

    /// The point at `(u, v)`.
    pub fn point(&self, u: f64, v: f64) -> Point3 {
        self.frame.origin + radial(&self.frame, u).0 * self.radius_at(v) + self.frame.z * v
    }

    /// Returns `true` if `v` is the apex up to rounding: `|rho(v)| <=
    /// 1e-12 (r0 + |v| tan(alpha))`.
    fn at_apex(&self, v: f64) -> bool {
        let tan = self.half_angle.tan();
        self.radius_at(v).abs() <= APEX_TOLERANCE * (self.radius + v.abs() * tan)
    }

    /// The point and the partial derivatives up to total order `d` (capped
    /// at [`MAX_DERIVATIVE_ORDER`](crate::MAX_DERIVATIVE_ORDER)) in the
    /// layout of [`NurbsSurface::derivatives`](crate::NurbsSurface::derivatives).
    pub fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vector3>> {
        let tan = self.half_angle.tan();
        revolved_derivatives(&self.frame, u, d, |l| match l {
            0 => (self.radius_at(v), v),
            1 => (tan, 1.0),
            _ => (0.0, 0.0),
        })
    }

    /// The first partial derivatives `S_u = rho(v) e'(u)`,
    /// `S_v = tan(alpha) e(u) + z`.
    pub fn partials(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        let (e, de) = radial(&self.frame, u);
        (
            de * self.radius_at(v),
            e * self.half_angle.tan() + self.frame.z,
        )
    }

    /// The unit normal, away from the axis; `None` at the apex (to a
    /// relative 1e-12).
    pub fn normal(&self, u: f64, v: f64) -> Option<Vector3> {
        if self.at_apex(v) {
            return None;
        }
        let (e, _) = radial(&self.frame, u);
        let (s, c) = self.half_angle.sin_cos();
        let n = e * c - self.frame.z * s;
        Some(if self.radius_at(v) > 0.0 { n } else { -n })
    }

    /// `k1 = 0` along the ruling and `k2 = -cos(alpha) / |rho(v)|` around
    /// the axis; `None` at the apex.
    pub fn principal_curvatures(&self, u: f64, v: f64) -> Option<Curvatures> {
        let n = self.normal(u, v)?;
        let (e, _) = radial(&self.frame, u);
        let (s, c) = self.half_angle.sin_cos();
        let dir1 = e * s + self.frame.z * c;
        Some(Curvatures {
            k1: 0.0,
            k2: -c / self.radius_at(v).abs(),
            dir1,
            dir2: n.cross(dir1),
        })
    }

    /// [`Domain::Periodic`] on `[0, 2 pi]`.
    pub fn domain_u(&self) -> Domain {
        Domain::Periodic(FULL_TURN)
    }

    /// [`Domain::Unbounded`] (the double cone; see the module
    /// documentation).
    pub fn domain_v(&self) -> Domain {
        Domain::Unbounded
    }

    /// The point of the double cone closest to `target`, with `u` in `[0, 2
    /// pi)` (see the module documentation).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    pub fn closest_point(&self, target: Point3) -> Result<SurfaceClosestPoint, GeomError> {
        check_point3(target)?;
        let local = self.frame.to_local(target);
        let (u, rho) = axial_angle(local);
        let z = local.z;
        let (s, c) = self.half_angle.sin_cos();
        let r0 = self.radius;
        // The ruling at u: (r0, 0) + w (sin, cos) in the half-plane; the one
        // at u + pi: (-r0, 0) + w (-sin, cos). v = w cos(alpha).
        let d1 = ((rho - r0) * c - z * s).abs();
        let d2 = ((rho + r0) * c + z * s).abs();
        let (u, v) = if d1 <= d2 {
            (u, c * ((rho - r0) * s + z * c))
        } else {
            (wrap_angle(u + PI), c * (z * c - (rho + r0) * s))
        };
        let point = self.point(u, v);
        Ok(SurfaceClosestPoint {
            u,
            v,
            point,
            distance: (target - point).length(),
        })
    }

    /// The isocurve at fixed `u`: the ruling through `S(u, 0)` along
    /// `sin(alpha) e(u) + cos(alpha) z`. Its parameter is the distance
    /// along the ruling, `t = v / cos(alpha)`, not `v`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `u`.
    pub fn isocurve_u(&self, u: f64) -> Result<Line3, GeomError> {
        let (e, _) = radial(&self.frame, u);
        let (s, c) = self.half_angle.sin_cos();
        Line3::new(self.point(u, 0.0), e * s + self.frame.z * c)
    }

    /// The isocurve at fixed `v`: the circle of radius `|rho(v)|` at height
    /// `v`, whose parameter is `u` (its frame is turned by `pi` beyond the
    /// apex).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `v` and
    /// [`GeomError::InvalidArgument`] at the apex (to a relative 1e-12),
    /// where the isocurve is a single point.
    pub fn isocurve_v(&self, v: f64) -> Result<Circle3, GeomError> {
        if !v.is_finite() {
            return Err(GeomError::NonFinite);
        }
        let rho = self.radius_at(v);
        if self.at_apex(v) {
            return Err(GeomError::InvalidArgument(
                "the isocurve of a cone at its apex is a point",
            ));
        }
        let f = self.frame;
        let (x, y) = if rho > 0.0 { (f.x, f.y) } else { (-f.x, -f.y) };
        let frame = Frame3 {
            origin: f.origin + f.z * v,
            x,
            y,
            z: f.z,
        };
        Circle3::new(frame, rho.abs())
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
