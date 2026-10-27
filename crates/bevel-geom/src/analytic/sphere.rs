// SPDX-License-Identifier: GPL-3.0-or-later

//! Spheres.
//!
//! A sphere of radius `r` centred on the origin of its frame is
//! `S(u, v) = o + r cos(v) e(u) + r sin(v) z` with `e(u) = cos(u) x +
//! sin(u) y`: `u` is the longitude about `z` from `x` (periodic, base
//! period `[0, 2 pi]`) and `v` the latitude, bounded to `[-pi / 2, pi / 2]`
//! (evaluation extends the formula beyond, over the poles). The poles
//! `o +- r z` are at `v = +-pi / 2`, where `S_u` vanishes.
//!
//! The normal `(S(u, v) - o) / r` points outwards and is returned at the
//! poles too, where the surface is smooth although the parametrization is
//! not. Both principal curvatures are `-1 / r` with the sign convention of
//! [`Curvatures`] (positive where the surface bends towards the normal);
//! every direction is principal, and `dir1 = e'(u)`, the direction of
//! `S_u` (defined at the poles too).
//!
//! The closest point to the centre, where every point is equally close, is
//! the one at `(0, 0)`; points on the axis map to a pole with `u = 0`.

use std::f64::consts::FRAC_PI_2;

use bevel_math::{Aabb3, Frame3, Interval, Point3, Vector3};

use super::convert::{ConvertedSurface, ParamMap, revolve};
use super::revolved::{axial_angle, radial, revolved_bounds, revolved_derivatives};
use super::{
    Circle3, Domain, FULL_TURN, check_frame, check_point3, check_radius, check_range,
    trig_derivative,
};
use crate::{Curvatures, GeomError, NurbsSurface, SurfaceClosestPoint, circle_arc};

/// How far outside `[-pi / 2, pi / 2]` a latitude range may reach (it is
/// clamped).
const LATITUDE_SLACK: f64 = 1e-12;

/// `cos(v)` at or below which a latitude counts as a pole.
const POLE_TOLERANCE: f64 = 1e-12;

/// The latitude range `[-pi / 2, pi / 2]` of a sphere.
pub(crate) const LATITUDES: Interval = Interval {
    min: -FRAC_PI_2,
    max: FRAC_PI_2,
};

/// A sphere: `S(u, v) = o + r (cos(v) e(u) + sin(v) z)` in the frame `(o,
/// x, y, z)`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[must_use]
pub struct Sphere {
    frame: Frame3,
    radius: f64,
}

impl Sphere {
    /// The sphere of radius `radius` centred on the origin of `frame`,
    /// with its poles on the frame's `z` axis.
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

    /// The frame: the origin is the centre and `z` points to the north
    /// pole.
    pub fn frame(&self) -> &Frame3 {
        &self.frame
    }

    /// The centre.
    pub fn center(&self) -> Point3 {
        self.frame.origin
    }

    /// The radius.
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// The point at longitude `u` and latitude `v`.
    pub fn point(&self, u: f64, v: f64) -> Point3 {
        self.frame.origin + self.unit_normal(u, v) * self.radius
    }

    /// `cos(v) e(u) + sin(v) z`.
    fn unit_normal(&self, u: f64, v: f64) -> Vector3 {
        let (s, c) = v.sin_cos();
        radial(&self.frame, u).0 * c + self.frame.z * s
    }

    /// The point and the partial derivatives up to total order `d` (capped
    /// at [`MAX_DERIVATIVE_ORDER`](crate::MAX_DERIVATIVE_ORDER)) in the
    /// layout of [`NurbsSurface::derivatives`](crate::NurbsSurface::derivatives).
    pub fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vector3>> {
        let (s, c) = v.sin_cos();
        let r = self.radius;
        revolved_derivatives(&self.frame, u, d, |l| {
            let (dc, ds) = trig_derivative(c, s, l);
            (r * dc, r * ds)
        })
    }

    /// The first partial derivatives `S_u = r cos(v) e'(u)`,
    /// `S_v = r (-sin(v) e(u) + cos(v) z)`.
    pub fn partials(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        let (e, de) = radial(&self.frame, u);
        let (s, c) = v.sin_cos();
        let r = self.radius;
        (de * (r * c), (self.frame.z * c - e * s) * r)
    }

    /// The outward unit normal, at the poles too.
    pub fn normal(&self, u: f64, v: f64) -> Option<Vector3> {
        Some(self.unit_normal(u, v))
    }

    /// Both curvatures are `-1 / r`; `dir1 = e'(u)`, `dir2 = normal x
    /// dir1`.
    pub fn principal_curvatures(&self, u: f64, v: f64) -> Option<Curvatures> {
        let n = self.unit_normal(u, v);
        let (_, dir1) = radial(&self.frame, u);
        let k = -1.0 / self.radius;
        Some(Curvatures {
            k1: k,
            k2: k,
            dir1,
            dir2: n.cross(dir1),
        })
    }

    /// [`Domain::Periodic`] on `[0, 2 pi]`.
    pub fn domain_u(&self) -> Domain {
        Domain::Periodic(FULL_TURN)
    }

    /// [`Domain::Bounded`] on `[-pi / 2, pi / 2]`.
    pub fn domain_v(&self) -> Domain {
        Domain::Bounded(LATITUDES)
    }

    /// The point of the sphere closest to `target`, with `u` in `[0, 2 pi)`
    /// and `v` in `[-pi / 2, pi / 2]` (see the module documentation for the
    /// centre and the axis).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    pub fn closest_point(&self, target: Point3) -> Result<SurfaceClosestPoint, GeomError> {
        check_point3(target)?;
        let local = self.frame.to_local(target);
        let (u, rho) = axial_angle(local);
        let v = if local.length() > 0.0 {
            local.z.atan2(rho)
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

    /// The isocurve at fixed `u`: the meridian, a great circle in the frame
    /// `(o, e(u), z, e(u) x z)`, whose parameter is `v` (the sphere covers
    /// `[-pi / 2, pi / 2]` of it).
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
            origin: self.frame.origin,
            x: e,
            y: self.frame.z,
            z: e.cross(self.frame.z),
        };
        Circle3::new(frame, self.radius)
    }

    /// The isocurve at fixed `v`: the parallel of radius `r cos(v)` at
    /// height `r sin(v)`, whose parameter is `u`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `v` and
    /// [`GeomError::InvalidArgument`] at a pole (`cos(v) <= 1e-12`), where
    /// the isocurve is a single point, or beyond one.
    pub fn isocurve_v(&self, v: f64) -> Result<Circle3, GeomError> {
        if !v.is_finite() {
            return Err(GeomError::NonFinite);
        }
        let (s, c) = v.sin_cos();
        if c <= POLE_TOLERANCE {
            return Err(GeomError::InvalidArgument(
                "the isocurve of a sphere at a pole is a point",
            ));
        }
        let r = self.radius * c;
        let frame = Frame3 {
            origin: self.frame.origin + self.frame.z * (self.radius * s),
            ..self.frame
        };
        Circle3::new(frame, r)
    }

    /// The exact bounding box of the patch over `u x v` (both finite).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range.
    pub fn bounds(&self, u: Interval, v: Interval) -> Result<Aabb3, GeomError> {
        revolved_bounds(&self.frame, u, v, true, |u, v| self.point(u, v))
    }

    /// The patch over `u x v` as an exact NURBS: the meridian arc at
    /// longitude `u.min` revolved about the axis with
    /// [`NurbsSurface::revolution`], on the domain `u x v`. Both
    /// directions follow the arc map of
    /// [`analytic::convert`](crate::analytic::convert). A pole in the
    /// range gives a row of coincident control points.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range and
    /// [`GeomError::InvalidArgument`] for one of zero length, a `u` range longer than a full turn, and
    /// [`GeomError::ParameterOutOfDomain`] for a latitude outside
    /// `[-pi / 2, pi / 2]` (by more than 1e-12).
    pub fn to_nurbs_mapped(&self, u: Interval, v: Interval) -> Result<ConvertedSurface, GeomError> {
        check_range(u)?;
        check_range(v)?;
        for t in [v.min, v.max] {
            if !LATITUDES.contains_with(t, LATITUDE_SLACK) {
                return Err(GeomError::ParameterOutOfDomain { t });
            }
        }
        let v = Interval::new(v.min.max(LATITUDES.min), v.max.min(LATITUDES.max));
        let (e, _) = radial(&self.frame, u.min);
        let meridian = Frame3 {
            origin: self.frame.origin,
            x: e,
            y: self.frame.z,
            z: e.cross(self.frame.z),
        };
        let profile = circle_arc(&meridian, self.radius, v.min, v.max)?;
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

    /// The whole surface as a NURBS: [`Self::to_nurbs`] over `[0, 2 pi] x [-pi / 2, pi / 2]`.
    ///
    /// # Errors
    ///
    /// None in practice; the signature matches [`Self::to_nurbs`].
    pub fn to_nurbs_full(&self) -> Result<NurbsSurface, GeomError> {
        self.to_nurbs(FULL_TURN, LATITUDES)
    }
}
