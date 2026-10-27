// SPDX-License-Identifier: GPL-3.0-or-later

//! Transforms of the analytic types.
//!
//! A transform is an affine map of space given as a 4x4 matrix acting on
//! `(x, y, z, 1)` (a 3x3 matrix acting on `(x, y, 1)` for 2D curves). Its
//! last row must be `0 0 0 1` (to 1e-12): the analytic types reject
//! projective maps, which only their NURBS forms can follow. The linear
//! part must be invertible: `|det|` above `1e-12` of the cube (square in
//! 2D) of its RMS column length.
//!
//! What survives depends on the linear part `L`:
//!
//! - Conformal maps (rotations, reflections, uniform scales by `s`,
//!   translations) keep every type. Angular parameters are unchanged, and
//!   lengths scale by `s`: the line parameter, both plane parameters and
//!   the `v` of cylinders and cones (the image of `S(u, v)` is `S'(u, s v)`).
//!   Rigid motions therefore keep every parameter.
//! - Under a reflection, circles and ellipses keep their parameter (their
//!   normal, `x x y`, reverses) and planes keep `(u, v)` (their normal, the
//!   new `x x y`, is then opposite the image of the old one). Cylinders,
//!   cones, spheres and tori keep their normal pointing outwards instead,
//!   which reverses `u`: the image of `S(u, v)` is `S'(-u, s v)` (or
//!   `S'(-u, v)`).
//! - Any invertible affine map keeps lines (the parameter scales by
//!   `|L d|`) and planes (whose frame becomes the Gram-Schmidt frame of
//!   `L x`, `L y`; the new parameters are then the affine functions
//!   `u' = |L x| u + (L y . x') v`, `v' = (L y . y') v`).
//! - A circle or ellipse with conjugate semi-diameters `p = a L x`,
//!   `q = b L y` becomes the ellipse with major and minor radii along the
//!   principal axes `p cos(t0) + q sin(t0)` and `-p sin(t0) + q cos(t0)`,
//!   `t0 = atan2(2 p . q, |p|^2 - |q|^2) / 2`; its eccentric angle is
//!   `t - t0`. Perpendicular semi-diameters of equal length give a circle
//!   (parameter kept), and of unequal length an ellipse with `t0` either 0
//!   or `pi / 2`.
//! - A cylinder or cone stays one when the map is conformal across the
//!   axis (`L x` and `L y` perpendicular and of equal length `s`) and keeps
//!   the axis perpendicular to them: a stretch along the axis by `k`
//!   scales `v` by `k`, and turns a cone's `tan(alpha)` into
//!   `tan(alpha) s / k`. Any other map turns them into elliptic cylinders
//!   or cones, which have no finite NURBS form either, so it is rejected:
//!   convert a bounded patch with `to_nurbs` and transform that.
//! - A sphere or torus under a non-conformal map becomes the NURBS form of
//!   its whole domain ([`Sphere::to_nurbs_full`], [`Torus::to_nurbs_full`]),
//!   transformed exactly; its parameters relate to the old ones by the
//!   [`ParamMap`](super::ParamMap)s of that conversion.

use bevel_math::{DMat2, DMat3, DMat4, DVec2, DVec3, Frame3, Point2, Point3, Vector2, Vector3};

use super::{
    Circle2, Circle3, Cone, Cylinder, Ellipse2, Ellipse3, Line2, Line3, Plane, Sphere, Torus,
};
use crate::{Curve2, Curve3, GeomError, NurbsSurface, Surface};

/// Tolerance on the last row of an affine matrix.
const AFFINE_TOLERANCE: f64 = 1e-12;

/// Relative tolerance under which vectors count as perpendicular and
/// lengths as equal when classifying a linear map.
const CONFORMAL_TOLERANCE: f64 = 1e-12;

/// `|det|` at or below this fraction of the cubed (squared in 2D) RMS
/// column length makes a linear map singular.
const SINGULAR_TOLERANCE: f64 = 1e-12;

/// Error for a singular matrix.
const SINGULAR: GeomError = GeomError::InvalidArgument("the transform must be invertible");

/// The affine map of space given by a matrix whose last row is `0 0 0 1`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Affine3 {
    linear: DMat3,
    translation: DVec3,
}

impl Affine3 {
    /// The affine map of `m`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite matrix and
    /// [`GeomError::InvalidArgument`] for a projective or singular one.
    pub(crate) fn new(m: &DMat4) -> Result<Self, GeomError> {
        check_invertible4(m)?;
        let row = m.row(3);
        if row.truncate().abs().max_element() > AFFINE_TOLERANCE
            || (row.w - 1.0).abs() > AFFINE_TOLERANCE
        {
            return Err(GeomError::InvalidArgument(
                "analytic geometry needs an affine transform (last row 0 0 0 1)",
            ));
        }
        Ok(Self {
            linear: DMat3::from_mat4(*m),
            translation: m.w_axis.truncate(),
        })
    }

    fn point(&self, p: Point3) -> Point3 {
        self.linear * p + self.translation
    }

    fn vector(&self, v: Vector3) -> Vector3 {
        self.linear * v
    }
}

/// The affine map of the plane given by a matrix whose last row is `0 0 1`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Affine2 {
    linear: DMat2,
    translation: DVec2,
}

impl Affine2 {
    /// The affine map of `m`.
    ///
    /// # Errors
    ///
    /// As [`Affine3::new`].
    pub(crate) fn new(m: &DMat3) -> Result<Self, GeomError> {
        check_invertible3(m)?;
        let row = m.row(2);
        if row.truncate().abs().max_element() > AFFINE_TOLERANCE
            || (row.z - 1.0).abs() > AFFINE_TOLERANCE
        {
            return Err(GeomError::InvalidArgument(
                "analytic geometry needs an affine transform (last row 0 0 1)",
            ));
        }
        Ok(Self {
            linear: DMat2::from_cols(m.x_axis.truncate(), m.y_axis.truncate()),
            translation: m.z_axis.truncate(),
        })
    }

    fn point(&self, p: Point2) -> Point2 {
        self.linear * p + self.translation
    }

    fn vector(&self, v: Vector2) -> Vector2 {
        self.linear * v
    }
}

/// Checks that a 4x4 matrix is finite and invertible (its determinant is
/// not negligible next to the fourth power of its RMS column length).
///
/// # Errors
///
/// [`GeomError::NonFinite`] or [`GeomError::InvalidArgument`].
pub(crate) fn check_invertible4(m: &DMat4) -> Result<(), GeomError> {
    if !m.is_finite() {
        return Err(GeomError::NonFinite);
    }
    // The linear part decides for affine maps; the whole matrix for
    // projective ones.
    let affine = m.row(3).truncate().abs().max_element() <= AFFINE_TOLERANCE;
    // For an affine map the determinant is det(L) w.
    let (det, scale) = if affine {
        let l = DMat3::from_mat4(*m);
        let rms =
            ((l.x_axis.length_squared() + l.y_axis.length_squared() + l.z_axis.length_squared())
                / 3.0)
                .sqrt();
        (l.determinant() * m.w_axis.w, rms.powi(3) * m.w_axis.w.abs())
    } else {
        let rms = ((m.x_axis.length_squared()
            + m.y_axis.length_squared()
            + m.z_axis.length_squared()
            + m.w_axis.length_squared())
            / 4.0)
            .sqrt();
        (m.determinant(), rms.powi(4))
    };
    if det.abs() > SINGULAR_TOLERANCE * scale {
        Ok(())
    } else {
        Err(SINGULAR)
    }
}

/// The 2D counterpart of [`check_invertible4`] for a 3x3 matrix acting on
/// `(x, y, 1)`.
///
/// # Errors
///
/// [`GeomError::NonFinite`] or [`GeomError::InvalidArgument`].
pub(crate) fn check_invertible3(m: &DMat3) -> Result<(), GeomError> {
    if !m.is_finite() {
        return Err(GeomError::NonFinite);
    }
    let affine = m.row(2).truncate().abs().max_element() <= AFFINE_TOLERANCE;
    let (det, scale) = if affine {
        let (a, b) = (m.x_axis.truncate(), m.y_axis.truncate());
        let rms = (0.5 * (a.length_squared() + b.length_squared())).sqrt();
        (a.perp_dot(b) * m.z_axis.z, rms * rms * m.z_axis.z.abs())
    } else {
        let rms =
            ((m.x_axis.length_squared() + m.y_axis.length_squared() + m.z_axis.length_squared())
                / 3.0)
                .sqrt();
        (m.determinant(), rms.powi(3))
    };
    if det.abs() > SINGULAR_TOLERANCE * scale {
        Ok(())
    } else {
        Err(SINGULAR)
    }
}

/// Returns `true` if `a` and `b` are perpendicular to the classification
/// tolerance.
fn perpendicular(a: DVec3, b: DVec3) -> bool {
    a.dot(b).abs() <= CONFORMAL_TOLERANCE * a.length() * b.length()
}

/// Returns `true` if two lengths are equal to the classification tolerance.
fn same_length(a: f64, b: f64) -> bool {
    (a - b).abs() <= CONFORMAL_TOLERANCE * a.max(b)
}

/// The right-handed frame at `origin` with `x` along `x` and `y` in the
/// plane of `x` and `y_hint` (Gram-Schmidt).
fn frame_from_axes(origin: Point3, x: Vector3, y_hint: Vector3) -> Result<Frame3, GeomError> {
    Frame3::from_axes(origin, x, y_hint).map_err(|_| SINGULAR)
}

/// The images of a frame's origin and axes (not normalised).
struct FrameImage {
    origin: Point3,
    ex: Vector3,
    ey: Vector3,
    ez: Vector3,
}

impl FrameImage {
    fn new(a: &Affine3, f: &Frame3) -> Self {
        Self {
            origin: a.point(f.origin),
            ex: a.vector(f.x),
            ey: a.vector(f.y),
            ez: a.vector(f.z),
        }
    }

    /// The scale across the axis if the map is conformal there and keeps
    /// the axis perpendicular to the frame's plane.
    fn axial_scale(&self) -> Option<f64> {
        let (lx, ly) = (self.ex.length(), self.ey.length());
        (same_length(lx, ly)
            && perpendicular(self.ex, self.ey)
            && perpendicular(self.ex, self.ez)
            && perpendicular(self.ey, self.ez))
        .then_some(0.5 * (lx + ly))
    }

    /// The scale of a conformal map.
    fn conformal_scale(&self) -> Option<f64> {
        self.axial_scale()
            .filter(|&s| same_length(s, self.ez.length()))
    }

    /// The frame of the image of a surface of revolution: `x` along the
    /// image of `x`, `z` along the image of the axis, and `y` along the
    /// image of `y` unless the map reflects, where it is opposite (so that
    /// `u` reverses).
    fn axis_frame(&self) -> Result<Frame3, GeomError> {
        let reflects = self.ex.cross(self.ey).dot(self.ez) < 0.0;
        let y = if reflects { -self.ey } else { self.ey };
        frame_from_axes(self.origin, self.ex, y)
    }
}

/// The curve `c + cos(t) p + sin(t) q`: a circle or an ellipse (see the
/// module documentation for the parameter).
fn conic_from_conjugates3(c: Point3, p: Vector3, q: Vector3) -> Result<Curve3, GeomError> {
    let (lp, lq) = (p.length(), q.length());
    if perpendicular(p, q) {
        if same_length(lp, lq) {
            let frame = frame_from_axes(c, p, q)?;
            return Ok(Circle3::new(frame, 0.5 * (lp + lq))?.into());
        }
        return Ok(if lp >= lq {
            Ellipse3::new(frame_from_axes(c, p, q)?, lp, lq)?
        } else {
            Ellipse3::new(frame_from_axes(c, q, -p)?, lq, lp)?
        }
        .into());
    }
    let t0 = 0.5 * (2.0 * p.dot(q)).atan2(lp * lp - lq * lq);
    let (s, co) = t0.sin_cos();
    let (x, y) = (p * co + q * s, q * co - p * s);
    let (a, b) = (x.length(), y.length());
    Ok(Ellipse3::new(frame_from_axes(c, x, y)?, a.max(b), a.min(b))?.into())
}

/// The 2D counterpart of [`conic_from_conjugates3`].
fn conic_from_conjugates2(c: Point2, p: Vector2, q: Vector2) -> Result<Curve2, GeomError> {
    let (lp, lq) = (p.length(), q.length());
    if perpendicular(p.extend(0.0), q.extend(0.0)) {
        let ccw = p.perp_dot(q) > 0.0;
        if same_length(lp, lq) {
            return Ok(Circle2::from_axis(c, p, ccw, 0.5 * (lp + lq))?.into());
        }
        return Ok(if lp >= lq {
            Ellipse2::from_axis(c, p, ccw, lp, lq)?
        } else {
            // The major axis is q and the minor -p, which keeps the sense.
            Ellipse2::from_axis(c, q, ccw, lq, lp)?
        }
        .into());
    }
    let t0 = 0.5 * (2.0 * p.dot(q)).atan2(lp * lp - lq * lq);
    let (s, co) = t0.sin_cos();
    let (x, y) = (p * co + q * s, q * co - p * s);
    let (a, b) = (x.length(), y.length());
    Ok(Ellipse2::from_axis(c, x, x.perp_dot(y) > 0.0, a.max(b), a.min(b))?.into())
}

impl Line2 {
    /// The line under the affine map `m` of the plane (see
    /// [`analytic::transform`](crate::analytic::transform)): always a line,
    /// whose parameter scales by `|L d|`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite matrix and
    /// [`GeomError::InvalidArgument`] for a projective or singular one.
    pub fn transformed(&self, m: &DMat3) -> Result<Curve2, GeomError> {
        let a = Affine2::new(m)?;
        Ok(Line2::new(a.point(self.origin()), a.vector(self.direction()))?.into())
    }
}

impl Line3 {
    /// The line under the affine map `m` (see
    /// [`analytic::transform`](crate::analytic::transform)): always a line,
    /// whose parameter scales by `|L d|`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite matrix and
    /// [`GeomError::InvalidArgument`] for a projective or singular one.
    pub fn transformed(&self, m: &DMat4) -> Result<Curve3, GeomError> {
        let a = Affine3::new(m)?;
        Ok(Line3::new(a.point(self.origin()), a.vector(self.direction()))?.into())
    }
}

impl Circle2 {
    /// The circle under the affine map `m` of the plane (see
    /// [`analytic::transform`](crate::analytic::transform)): a circle with
    /// the same parameter under a conformal map, otherwise an ellipse.
    ///
    /// # Errors
    ///
    /// As [`Line2::transformed`].
    pub fn transformed(&self, m: &DMat3) -> Result<Curve2, GeomError> {
        let a = Affine2::new(m)?;
        let r = self.radius();
        conic_from_conjugates2(
            a.point(self.center()),
            a.vector(self.x_axis()) * r,
            a.vector(self.y_axis()) * r,
        )
    }
}

impl Circle3 {
    /// The circle under the affine map `m` (see
    /// [`analytic::transform`](crate::analytic::transform)): a circle with
    /// the same parameter when the map is conformal in its plane, otherwise
    /// an ellipse.
    ///
    /// # Errors
    ///
    /// As [`Line3::transformed`].
    pub fn transformed(&self, m: &DMat4) -> Result<Curve3, GeomError> {
        let a = Affine3::new(m)?;
        let (f, r) = (self.frame(), self.radius());
        conic_from_conjugates3(a.point(f.origin), a.vector(f.x) * r, a.vector(f.y) * r)
    }
}

impl Ellipse2 {
    /// The ellipse under the affine map `m` of the plane (see
    /// [`analytic::transform`](crate::analytic::transform)): an ellipse (a
    /// circle if the map makes it round).
    ///
    /// # Errors
    ///
    /// As [`Line2::transformed`].
    pub fn transformed(&self, m: &DMat3) -> Result<Curve2, GeomError> {
        let a = Affine2::new(m)?;
        conic_from_conjugates2(
            a.point(self.center()),
            a.vector(self.x_axis()) * self.major_radius(),
            a.vector(self.y_axis()) * self.minor_radius(),
        )
    }
}

impl Ellipse3 {
    /// The ellipse under the affine map `m` (see
    /// [`analytic::transform`](crate::analytic::transform)): an ellipse (a
    /// circle if the map makes it round).
    ///
    /// # Errors
    ///
    /// As [`Line3::transformed`].
    pub fn transformed(&self, m: &DMat4) -> Result<Curve3, GeomError> {
        let a = Affine3::new(m)?;
        let f = self.frame();
        conic_from_conjugates3(
            a.point(f.origin),
            a.vector(f.x) * self.major_radius(),
            a.vector(f.y) * self.minor_radius(),
        )
    }
}

impl Plane {
    /// The plane under the affine map `m` (see
    /// [`analytic::transform`](crate::analytic::transform)): always a
    /// plane.
    ///
    /// # Errors
    ///
    /// As [`Line3::transformed`].
    pub fn transformed(&self, m: &DMat4) -> Result<Surface, GeomError> {
        let a = Affine3::new(m)?;
        let img = FrameImage::new(&a, self.frame());
        Ok(Plane::new(frame_from_axes(img.origin, img.ex, img.ey)?)?.into())
    }
}

/// The error for a cylinder or cone under a map that makes it elliptic.
const ELLIPTIC: GeomError = GeomError::InvalidArgument(
    "the transform makes the unbounded surface elliptic; convert a bounded patch to NURBS first",
);

impl Cylinder {
    /// The cylinder under the affine map `m` (see
    /// [`analytic::transform`](crate::analytic::transform)).
    ///
    /// # Errors
    ///
    /// As [`Line3::transformed`], and [`GeomError::InvalidArgument`] for a
    /// map that is not conformal across the axis or tilts the axis.
    pub fn transformed(&self, m: &DMat4) -> Result<Surface, GeomError> {
        let a = Affine3::new(m)?;
        let img = FrameImage::new(&a, self.frame());
        let s = img.axial_scale().ok_or(ELLIPTIC)?;
        Ok(Cylinder::new(img.axis_frame()?, self.radius() * s)?.into())
    }
}

impl Cone {
    /// The cone under the affine map `m` (see
    /// [`analytic::transform`](crate::analytic::transform)).
    ///
    /// # Errors
    ///
    /// As [`Cylinder::transformed`].
    pub fn transformed(&self, m: &DMat4) -> Result<Surface, GeomError> {
        let a = Affine3::new(m)?;
        let img = FrameImage::new(&a, self.frame());
        let s = img.axial_scale().ok_or(ELLIPTIC)?;
        let k = img.ez.length();
        let half_angle = (self.half_angle().tan() * s / k).atan();
        Ok(Cone::new(img.axis_frame()?, self.radius() * s, half_angle)?.into())
    }
}

impl Sphere {
    /// The sphere under the affine map `m` (see
    /// [`analytic::transform`](crate::analytic::transform)): a sphere under
    /// a conformal map, otherwise the transformed NURBS form of the whole
    /// sphere.
    ///
    /// # Errors
    ///
    /// As [`Line3::transformed`].
    pub fn transformed(&self, m: &DMat4) -> Result<Surface, GeomError> {
        let a = Affine3::new(m)?;
        let img = FrameImage::new(&a, self.frame());
        match img.conformal_scale() {
            Some(s) => Ok(Sphere::new(img.axis_frame()?, self.radius() * s)?.into()),
            None => nurbs_image(&self.to_nurbs_full()?, m),
        }
    }
}

impl Torus {
    /// The torus under the affine map `m` (see
    /// [`analytic::transform`](crate::analytic::transform)): a torus under
    /// a conformal map, otherwise the transformed NURBS form of the whole
    /// torus.
    ///
    /// # Errors
    ///
    /// As [`Line3::transformed`].
    pub fn transformed(&self, m: &DMat4) -> Result<Surface, GeomError> {
        let a = Affine3::new(m)?;
        let img = FrameImage::new(&a, self.frame());
        match img.conformal_scale() {
            Some(s) => Ok(Torus::new(
                img.axis_frame()?,
                self.major_radius() * s,
                self.minor_radius() * s,
            )?
            .into()),
            None => nurbs_image(&self.to_nurbs_full()?, m),
        }
    }
}

/// The NURBS surface `s` under `m`.
fn nurbs_image(s: &NurbsSurface, m: &DMat4) -> Result<Surface, GeomError> {
    Ok(Surface::Nurbs(s.transformed(m)?))
}
