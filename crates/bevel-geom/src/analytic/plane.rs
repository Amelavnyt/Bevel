// SPDX-License-Identifier: GPL-3.0-or-later

//! Planes.
//!
//! A plane is `S(u, v) = o + u x + v y` in its frame `(o, x, y, z)`: `u`
//! and `v` are the coordinates along the frame's `x` and `y` axes, both
//! unbounded, and the normal is the frame's `z` axis (`S_u x S_v`). Both
//! principal curvatures are zero, with `dir1 = x` and `dir2 = y`.

use bevel_math::{Aabb3, Frame3, Interval, Point3, Vector3};
use serde::{Deserialize, Serialize};

use super::convert::{ConvertedSurface, ParamMap, check_open_range};
use super::{Domain, Line3, check_frame, check_point3, check_range, unit3};
use crate::{
    Curvatures, GeomError, KnotVector, MAX_DERIVATIVE_ORDER, NurbsSurface, SurfaceClosestPoint,
};

/// An unbounded plane: `S(u, v) = o + u x + v y` in the frame `(o, x, y,
/// z)`, with normal `z`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "PlaneRaw")]
#[must_use]
pub struct Plane {
    frame: Frame3,
}

/// The serialized form of a [`Plane`], checked by [`Plane::new`].
#[derive(Deserialize)]
struct PlaneRaw {
    frame: Frame3,
}

impl TryFrom<PlaneRaw> for Plane {
    type Error = GeomError;

    fn try_from(raw: PlaneRaw) -> Result<Self, GeomError> {
        Self::new(raw.frame)
    }
}

impl Plane {
    /// The plane of the frame's `x` and `y` axes.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite frame and
    /// [`GeomError::InvalidArgument`] for one that is not orthonormal and
    /// right-handed (to 1e-9).
    pub fn new(frame: Frame3) -> Result<Self, GeomError> {
        check_frame(&frame)?;
        Ok(Self { frame })
    }

    /// The plane through `origin` with the given normal; the frame is
    /// [`Frame3::from_normal`].
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for non-finite input and
    /// [`GeomError::InvalidArgument`] for a zero normal.
    pub fn from_normal(origin: Point3, normal: Vector3) -> Result<Self, GeomError> {
        check_point3(origin)?;
        let n = unit3(normal, "a plane normal must be non-zero")?;
        let frame = Frame3::from_normal(origin, n)
            .map_err(|_| GeomError::InvalidArgument("a plane normal must be non-zero"))?;
        Self::new(frame)
    }

    /// The frame.
    pub fn frame(&self) -> &Frame3 {
        &self.frame
    }

    /// The point at `(0, 0)`.
    pub fn origin(&self) -> Point3 {
        self.frame.origin
    }

    /// The point at `(u, v)`.
    pub fn point(&self, u: f64, v: f64) -> Point3 {
        self.frame.origin + self.frame.x * u + self.frame.y * v
    }

    /// The point and the partial derivatives up to total order `d` (capped
    /// at [`MAX_DERIVATIVE_ORDER`]) in the layout of
    /// [`NurbsSurface::derivatives`](crate::NurbsSurface::derivatives):
    /// `S_u = x`, `S_v = y` and zero above the first order.
    pub fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vector3>> {
        let d = d.min(MAX_DERIVATIVE_ORDER);
        let mut out: Vec<Vec<Vector3>> = (0..=d).map(|k| vec![Vector3::ZERO; d - k + 1]).collect();
        if let Some(row) = out.first_mut() {
            if let Some(p) = row.first_mut() {
                *p = self.point(u, v);
            }
            if let Some(sv) = row.get_mut(1) {
                *sv = self.frame.y;
            }
        }
        if let Some(su) = out.get_mut(1).and_then(|row| row.first_mut()) {
            *su = self.frame.x;
        }
        out
    }

    /// The first partial derivatives `(S_u, S_v) = (x, y)`.
    pub fn partials(&self, _u: f64, _v: f64) -> (Vector3, Vector3) {
        (self.frame.x, self.frame.y)
    }

    /// The unit normal, the frame's `z` axis.
    pub fn normal(&self, _u: f64, _v: f64) -> Option<Vector3> {
        Some(self.frame.z)
    }

    /// Both principal curvatures are zero; `dir1 = x`, `dir2 = y`.
    pub fn principal_curvatures(&self, _u: f64, _v: f64) -> Option<Curvatures> {
        Some(Curvatures {
            k1: 0.0,
            k2: 0.0,
            dir1: self.frame.x,
            dir2: self.frame.y,
        })
    }

    /// [`Domain::Unbounded`].
    pub fn domain_u(&self) -> Domain {
        Domain::Unbounded
    }

    /// [`Domain::Unbounded`].
    pub fn domain_v(&self) -> Domain {
        Domain::Unbounded
    }

    /// The orthogonal projection of `target` onto the plane.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    pub fn closest_point(&self, target: Point3) -> Result<SurfaceClosestPoint, GeomError> {
        check_point3(target)?;
        let local = self.frame.to_local(target);
        let point = self.point(local.x, local.y);
        Ok(SurfaceClosestPoint {
            u: local.x,
            v: local.y,
            point,
            distance: (target - point).length(),
        })
    }

    /// The signed distance of `p` from the plane, positive on the side of
    /// the normal.
    pub fn signed_distance(&self, p: Point3) -> f64 {
        (p - self.frame.origin).dot(self.frame.z)
    }

    /// The isocurve at fixed `u`: the line through `S(u, 0)` along `y`,
    /// whose parameter is `v`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `u`.
    pub fn isocurve_u(&self, u: f64) -> Result<Line3, GeomError> {
        Line3::new(self.point(u, 0.0), self.frame.y)
    }

    /// The isocurve at fixed `v`: the line through `S(0, v)` along `x`,
    /// whose parameter is `u`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `v`.
    pub fn isocurve_v(&self, v: f64) -> Result<Line3, GeomError> {
        Line3::new(self.point(0.0, v), self.frame.x)
    }

    /// The exact bounding box of the rectangle `u x v`: that of its four
    /// corners.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range.
    pub fn bounds(&self, u: Interval, v: Interval) -> Result<Aabb3, GeomError> {
        check_range(u)?;
        check_range(v)?;
        Ok(Aabb3::from_points([
            self.point(u.min, v.min),
            self.point(u.max, v.min),
            self.point(u.min, v.max),
            self.point(u.max, v.max),
        ]))
    }

    /// The rectangle `u x v` as a bilinear NURBS patch on the domain
    /// `u x v`, whose parameters are the plane's ([`ParamMap::Identity`]
    /// in both directions).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range and
    /// [`GeomError::InvalidArgument`] for one of zero length.
    pub fn to_nurbs_mapped(&self, u: Interval, v: Interval) -> Result<ConvertedSurface, GeomError> {
        check_open_range(u)?;
        check_open_range(v)?;
        let ku = KnotVector::clamped_from_interior(1, u.min, u.max, &[])?;
        let kv = KnotVector::clamped_from_interior(1, v.min, v.max, &[])?;
        let points = [
            self.point(u.min, v.min),
            self.point(u.min, v.max),
            self.point(u.max, v.min),
            self.point(u.max, v.max),
        ];
        let nurbs = NurbsSurface::from_points(1, 1, ku, kv, &points, None)?;
        Ok(ConvertedSurface {
            nurbs,
            map_u: ParamMap::Identity,
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
