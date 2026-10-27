// SPDX-License-Identifier: GPL-3.0-or-later

//! [`SurfaceGeom`] and the surface enum [`Surface`].

use bevel_math::{Aabb3, Interval, Point3, Vector3};

use super::{Curve3, CurveGeom, restriction};
use crate::analytic::{ConvertedSurface, Domain, ParamMap};
use crate::{
    Cone, Curvatures, Cylinder, GeomError, NurbsSurface, Plane, Sphere, SurfaceClosestPoint, Torus,
};

/// What every surface provides (see the
/// [module documentation](crate::geometry) for the design).
pub trait SurfaceGeom {
    /// The point at `(u, v)`.
    fn point(&self, u: f64, v: f64) -> Point3;

    /// The point and the partial derivatives up to total order `d`:
    /// `result[k][l]` is `d^(k+l) S / du^k dv^l` for `k + l <= d` (capped
    /// at [`MAX_DERIVATIVE_ORDER`](crate::MAX_DERIVATIVE_ORDER)), as for
    /// [`NurbsSurface::derivatives`].
    fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vector3>>;

    /// The first partial derivatives `(S_u, S_v)`.
    fn partials(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        let d = self.derivatives(u, v, 1);
        let su = d.get(1).and_then(|row| row.first()).copied();
        let sv = d.first().and_then(|row| row.get(1)).copied();
        (su.unwrap_or(Vector3::ZERO), sv.unwrap_or(Vector3::ZERO))
    }

    /// The unit normal at `(u, v)`, or `None` where it does not exist. For
    /// NURBS it is `S_u x S_v` normalised; the analytic closed surfaces
    /// (and cylinders and cones) point away from their centre or axis, and
    /// a plane along its frame's `z` axis.
    fn normal(&self, u: f64, v: f64) -> Option<Vector3>;

    /// The principal curvatures and directions at `(u, v)`, with the sign
    /// convention of [`Curvatures`] (positive where the surface bends
    /// towards [`SurfaceGeom::normal`]), or `None` where there is no
    /// normal.
    fn principal_curvatures(&self, u: f64, v: f64) -> Option<Curvatures>;

    /// The domain in `u`.
    fn domain_u(&self) -> Domain;

    /// The domain in `v`.
    fn domain_v(&self) -> Domain;

    /// The period in `u`, if the surface is periodic in `u`.
    fn period_u(&self) -> Option<f64> {
        self.domain_u().period()
    }

    /// The period in `v`, if the surface is periodic in `v`.
    fn period_v(&self) -> Option<f64> {
        self.domain_v().period()
    }

    /// The point of the surface closest to `target` (point inversion, over
    /// the whole domain).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] if `target` is not finite.
    fn closest_point(&self, target: Point3) -> Result<SurfaceClosestPoint, GeomError>;

    /// The isocurve at fixed `u`, running along `v`. For the analytic types
    /// it is a line or a circle whose parameter is `v` (except along a
    /// cone's ruling; see [`Cone::isocurve_u`]).
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `u`.
    fn isocurve_u(&self, u: f64) -> Result<Curve3, GeomError>;

    /// The isocurve at fixed `v`, running along `u`; for the analytic types
    /// a line or a circle whose parameter is `u`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `v`, and
    /// [`GeomError::InvalidArgument`] where it is a single point (a pole of
    /// a sphere, the apex of a cone).
    fn isocurve_v(&self, v: f64) -> Result<Curve3, GeomError>;

    /// A box containing the patch over `u x v`: exact for the analytic
    /// types, the box of the control points of that patch for NURBS.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range and, for NURBS,
    /// [`GeomError::ParameterOutOfDomain`] for one outside the domain.
    fn bounds(&self, u: Interval, v: Interval) -> Result<Aabb3, GeomError>;

    /// The exact NURBS form of the patch over `u x v`, with the maps
    /// between the parameters (see
    /// [`analytic::convert`](crate::analytic::convert)). A NURBS surface
    /// gives its patch over the rectangle with [`ParamMap::Identity`].
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite range,
    /// [`GeomError::InvalidArgument`] for one of zero length (or longer
    /// than a period) and [`GeomError::ParameterOutOfDomain`] for one
    /// outside a bounded domain.
    fn to_nurbs_mapped(&self, u: Interval, v: Interval) -> Result<ConvertedSurface, GeomError>;

    /// [`SurfaceGeom::to_nurbs_mapped`] without the maps.
    ///
    /// # Errors
    ///
    /// As [`SurfaceGeom::to_nurbs_mapped`].
    fn to_nurbs(&self, u: Interval, v: Interval) -> Result<NurbsSurface, GeomError> {
        self.to_nurbs_mapped(u, v).map(|c| c.nurbs)
    }

    /// The NURBS form over the whole domain (one period in a periodic
    /// direction).
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidArgument`] for a surface unbounded in either
    /// direction, and as [`SurfaceGeom::to_nurbs`].
    fn to_nurbs_full(&self) -> Result<NurbsSurface, GeomError> {
        let unbounded = GeomError::InvalidArgument(
            "an unbounded surface needs a parameter range to become a NURBS",
        );
        let u = self.domain_u().interval().ok_or(unbounded)?;
        let v = self.domain_v().interval().ok_or(unbounded)?;
        self.to_nurbs(u, v)
    }
}

/// A surface: analytic or NURBS.
#[derive(Clone, Debug, PartialEq)]
pub enum Surface {
    /// A plane.
    Plane(Plane),
    /// A circular cylinder.
    Cylinder(Cylinder),
    /// A circular cone.
    Cone(Cone),
    /// A sphere.
    Sphere(Sphere),
    /// A ring torus.
    Torus(Torus),
    /// A NURBS surface.
    Nurbs(NurbsSurface),
}

/// Implements [`SurfaceGeom`] for an analytic type by delegating to its
/// own methods of the same names.
macro_rules! analytic_surface {
    ($ty:ident) => {
        impl SurfaceGeom for $ty {
            fn point(&self, u: f64, v: f64) -> Point3 {
                $ty::point(self, u, v)
            }

            fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vector3>> {
                $ty::derivatives(self, u, v, d)
            }

            fn partials(&self, u: f64, v: f64) -> (Vector3, Vector3) {
                $ty::partials(self, u, v)
            }

            fn normal(&self, u: f64, v: f64) -> Option<Vector3> {
                $ty::normal(self, u, v)
            }

            fn principal_curvatures(&self, u: f64, v: f64) -> Option<Curvatures> {
                $ty::principal_curvatures(self, u, v)
            }

            fn domain_u(&self) -> Domain {
                $ty::domain_u(self)
            }

            fn domain_v(&self) -> Domain {
                $ty::domain_v(self)
            }

            fn closest_point(&self, target: Point3) -> Result<SurfaceClosestPoint, GeomError> {
                $ty::closest_point(self, target)
            }

            fn isocurve_u(&self, u: f64) -> Result<Curve3, GeomError> {
                $ty::isocurve_u(self, u).map(Curve3::from)
            }

            fn isocurve_v(&self, v: f64) -> Result<Curve3, GeomError> {
                $ty::isocurve_v(self, v).map(Curve3::from)
            }

            fn bounds(&self, u: Interval, v: Interval) -> Result<Aabb3, GeomError> {
                $ty::bounds(self, u, v)
            }

            fn to_nurbs_mapped(
                &self,
                u: Interval,
                v: Interval,
            ) -> Result<ConvertedSurface, GeomError> {
                $ty::to_nurbs_mapped(self, u, v)
            }
        }

        impl From<$ty> for Surface {
            fn from(s: $ty) -> Self {
                Self::$ty(s)
            }
        }
    };
}

analytic_surface!(Plane);
analytic_surface!(Cylinder);
analytic_surface!(Cone);
analytic_surface!(Sphere);
analytic_surface!(Torus);

/// The patch of `s` over `u x v` (the surface itself when the rectangle
/// covers the domain).
fn restricted(s: &NurbsSurface, u: Interval, v: Interval) -> Result<NurbsSurface, GeomError> {
    let ((ua, ub), (va, vb)) = s.domain();
    let (tu, tv) = (s.knots_u().tolerance(), s.knots_v().tolerance());
    let ru = restriction((ua, ub), u, tu)?;
    let rv = restriction((va, vb), v, tv)?;
    let short = GeomError::InvalidArgument("a parameter range must have positive length");
    let mut piece = s.clone();
    if let Some(r) = ru {
        if r.length() <= tu {
            return Err(short);
        }
        if r.min > ua + tu {
            piece = piece.split_u(r.min)?.1;
        }
        if r.max < ub - tu {
            piece = piece.split_u(r.max)?.0;
        }
    }
    if let Some(r) = rv {
        if r.length() <= tv {
            return Err(short);
        }
        if r.min > va + tv {
            piece = piece.split_v(r.min)?.1;
        }
        if r.max < vb - tv {
            piece = piece.split_v(r.max)?.0;
        }
    }
    Ok(piece)
}

impl SurfaceGeom for NurbsSurface {
    fn point(&self, u: f64, v: f64) -> Point3 {
        NurbsSurface::point(self, u, v)
    }

    fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vector3>> {
        NurbsSurface::derivatives(self, u, v, d)
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vector3> {
        NurbsSurface::normal(self, u, v)
    }

    fn principal_curvatures(&self, u: f64, v: f64) -> Option<Curvatures> {
        NurbsSurface::principal_curvatures(self, u, v)
    }

    /// [`Domain::Bounded`] on the knot domain in `u`; whether the surface
    /// closes up is [`NurbsSurface::is_closed_u`].
    fn domain_u(&self) -> Domain {
        let ((a, b), _) = self.domain();
        Domain::Bounded(Interval::new(a, b))
    }

    /// [`Domain::Bounded`] on the knot domain in `v`; whether the surface
    /// closes up is [`NurbsSurface::is_closed_v`].
    fn domain_v(&self) -> Domain {
        let (_, (a, b)) = self.domain();
        Domain::Bounded(Interval::new(a, b))
    }

    fn closest_point(&self, target: Point3) -> Result<SurfaceClosestPoint, GeomError> {
        NurbsSurface::closest_point(self, target)
    }

    fn isocurve_u(&self, u: f64) -> Result<Curve3, GeomError> {
        NurbsSurface::isocurve_u(self, u).map(Curve3::Nurbs)
    }

    fn isocurve_v(&self, v: f64) -> Result<Curve3, GeomError> {
        NurbsSurface::isocurve_v(self, v).map(Curve3::Nurbs)
    }

    /// The box of the control points of the patch over `u x v` (which
    /// contains it, by the convex hull property).
    fn bounds(&self, u: Interval, v: Interval) -> Result<Aabb3, GeomError> {
        let ((ua, ub), (va, vb)) = self.domain();
        let (tu, tv) = (self.knots_u().tolerance(), self.knots_v().tolerance());
        let point_u = restriction((ua, ub), u, tu)?.is_some_and(|r| r.length() <= tu);
        let point_v = restriction((va, vb), v, tv)?.is_some_and(|r| r.length() <= tv);
        match (point_u, point_v) {
            (true, true) => Ok(Aabb3::from_points([NurbsSurface::point(
                self, u.min, v.min,
            )])),
            (true, false) => CurveGeom::bounds(&NurbsSurface::isocurve_u(self, u.min)?, v),
            (false, true) => CurveGeom::bounds(&NurbsSurface::isocurve_v(self, v.min)?, u),
            (false, false) => Ok(restricted(self, u, v)?.control_bounds()),
        }
    }

    fn to_nurbs_mapped(&self, u: Interval, v: Interval) -> Result<ConvertedSurface, GeomError> {
        Ok(ConvertedSurface {
            nurbs: restricted(self, u, v)?,
            map_u: ParamMap::Identity,
            map_v: ParamMap::Identity,
        })
    }
}

impl From<NurbsSurface> for Surface {
    fn from(s: NurbsSurface) -> Self {
        Self::Nurbs(s)
    }
}

/// `match` over the variants of [`Surface`], binding each to `$s`.
macro_rules! dispatch {
    ($self:expr, $s:ident => $e:expr) => {
        match $self {
            Surface::Plane($s) => $e,
            Surface::Cylinder($s) => $e,
            Surface::Cone($s) => $e,
            Surface::Sphere($s) => $e,
            Surface::Torus($s) => $e,
            Surface::Nurbs($s) => $e,
        }
    };
}

impl Surface {
    /// Returns `true` for an analytic (not NURBS) surface.
    pub fn is_analytic(&self) -> bool {
        !matches!(self, Self::Nurbs(_))
    }
}

impl SurfaceGeom for Surface {
    fn point(&self, u: f64, v: f64) -> Point3 {
        dispatch!(self, s => SurfaceGeom::point(s, u, v))
    }

    fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vector3>> {
        dispatch!(self, s => SurfaceGeom::derivatives(s, u, v, d))
    }

    fn partials(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        dispatch!(self, s => SurfaceGeom::partials(s, u, v))
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vector3> {
        dispatch!(self, s => SurfaceGeom::normal(s, u, v))
    }

    fn principal_curvatures(&self, u: f64, v: f64) -> Option<Curvatures> {
        dispatch!(self, s => SurfaceGeom::principal_curvatures(s, u, v))
    }

    fn domain_u(&self) -> Domain {
        dispatch!(self, s => SurfaceGeom::domain_u(s))
    }

    fn domain_v(&self) -> Domain {
        dispatch!(self, s => SurfaceGeom::domain_v(s))
    }

    fn closest_point(&self, target: Point3) -> Result<SurfaceClosestPoint, GeomError> {
        dispatch!(self, s => SurfaceGeom::closest_point(s, target))
    }

    fn isocurve_u(&self, u: f64) -> Result<Curve3, GeomError> {
        dispatch!(self, s => SurfaceGeom::isocurve_u(s, u))
    }

    fn isocurve_v(&self, v: f64) -> Result<Curve3, GeomError> {
        dispatch!(self, s => SurfaceGeom::isocurve_v(s, v))
    }

    fn bounds(&self, u: Interval, v: Interval) -> Result<Aabb3, GeomError> {
        dispatch!(self, s => SurfaceGeom::bounds(s, u, v))
    }

    fn to_nurbs_mapped(&self, u: Interval, v: Interval) -> Result<ConvertedSurface, GeomError> {
        dispatch!(self, s => SurfaceGeom::to_nurbs_mapped(s, u, v))
    }
}
