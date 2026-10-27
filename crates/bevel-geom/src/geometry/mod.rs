// SPDX-License-Identifier: GPL-3.0-or-later

//! The curve and surface enums ([`Curve2`], [`Curve3`], [`Surface`]) and
//! the traits every curve and surface implements ([`CurveGeom`],
//! [`SurfaceGeom`]).
//!
//! # Design
//!
//! The topology stores a [`Curve3`] per edge, a [`Curve2`] per coedge
//! (pcurve) and a [`Surface`] per face; each is an analytic type that keeps
//! its identity or a NURBS. The traits are implemented by every analytic
//! type, by [`NurbsCurve2`](crate::NurbsCurve2),
//! [`NurbsCurve3`](crate::NurbsCurve3) and
//! [`NurbsSurface`](crate::NurbsSurface) (delegating to their own
//! methods), and by the enums (dispatching to the variant), so algorithms
//! can be written once, generically or on the enums.
//!
//! There is one curve trait for both dimensions: [`CurveGeom`] names its
//! point type ([`Point2`](bevel_math::Point2) or
//! [`Point3`](bevel_math::Point3), also used for vectors), the
//! [`Homogeneous`](crate::Homogeneous) control point type of its NURBS
//! form (which fixes `NurbsCurve2` or `NurbsCurve3`) and its bounding box
//! type as associated types. Generic code over `C: CurveGeom` then works
//! in the plane and in space alike; the one difference in meaning is the
//! curvature, signed (positive counter-clockwise) for planar curves and
//! non-negative in space, as for the NURBS curves.
//!
//! Parameters follow each type's own documented parametrization (the
//! [`analytic`](crate::analytic) module for the analytic types). NURBS
//! evaluation clamps parameters to the domain; analytic evaluation accepts
//! every finite parameter. Bounds and NURBS forms take a parameter range,
//! since analytic geometry is unbounded or periodic and trims live in the
//! topology.
//!
//! Method-call syntax prefers a type's own (inherent) methods over trait
//! methods of the same name. The analytic types' own methods are the trait
//! methods, but the NURBS types keep their established signatures: on a
//! `NurbsCurve3`, `c.domain()` is the `(min, max)` pair and
//! `s.isocurve_u(u)` a `NurbsCurve3`. Call the trait version by path
//! (`CurveGeom::domain(&c)`) or through an enum when it matters.

mod curve;
mod surface;

pub use curve::{Curve2, Curve3, CurveGeom};
pub use surface::{Surface, SurfaceGeom};

use bevel_math::Interval;

use crate::GeomError;

/// The part of `domain` covered by `range`, for restricting a NURBS: `None`
/// when `range` covers the whole domain (to the knot tolerance `tol`).
///
/// # Errors
///
/// [`GeomError::NonFinite`] for a non-finite range and
/// [`GeomError::ParameterOutOfDomain`] for one reaching outside the domain
/// by more than `tol`.
pub(crate) fn restriction(
    domain: (f64, f64),
    range: Interval,
    tol: f64,
) -> Result<Option<Interval>, GeomError> {
    crate::analytic::check_range(range)?;
    let (a, b) = domain;
    for t in [range.min, range.max] {
        if t < a - tol || t > b + tol {
            return Err(GeomError::ParameterOutOfDomain { t });
        }
    }
    let (lo, hi) = (range.min.max(a), range.max.min(b));
    if lo <= a + tol && hi >= b - tol {
        Ok(None)
    } else {
        Ok(Some(Interval::new(lo, hi)))
    }
}
