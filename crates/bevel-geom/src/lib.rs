// SPDX-License-Identifier: GPL-3.0-or-later

//! Curves and surfaces for Bevel.
//!
//! Every curve and surface is exactly representable as a (rational) NURBS,
//! while analytic types (lines, circles, planes, cylinders, ...) keep their
//! identity so they can be intersected, measured and exported exactly.
//! Trims and parameter ranges live in the topology (`bevel-kernel`), not here.
//!
//! The NURBS code follows Piegl and Tiller, *The NURBS Book* (2nd ed.); each
//! algorithm cites its number there (A2.1, A5.1, ...).
//!
//! # Status
//!
//! Implemented:
//!
//! - [`KnotVector`]: validated knot vectors and span search (A2.1).
//! - [`basis`]: B-spline basis functions and their derivatives (A2.2, A2.3).
//! - [`Homogeneous`]: weighted control points, so that every algorithm is
//!   written once for 2D and 3D.
//! - [`NurbsCurve`] ([`NurbsCurve2`], [`NurbsCurve3`]): construction,
//!   evaluation, derivatives (A3.2, A4.1, A4.2), tangent, curvature,
//!   transforms and control-point bounds; knot insertion, refinement and
//!   removal (A5.1, A5.4, A5.8); clamping, Bezier decomposition (A5.6),
//!   splitting, reversal and reparametrisation; degree elevation (A5.9) and
//!   reduction with an error bound; arc length by adaptive Gauss-Legendre
//!   quadrature; closest point by sampling, Newton (section 6.1) and a
//!   branch-and-bound check over Bezier segments.
//! - [`fit`]: curve fitting through points; parametrization and averaged
//!   knots (section 9.2.1, eq. 9.8), global interpolation (A9.1) with optional
//!   end tangents, and least-squares approximation with a tolerance search
//!   (section 9.4.1).
//! - [`fit`] also fits surfaces: grid parameters (section 9.2.5), global
//!   surface interpolation (A9.4) and two-pass least-squares approximation.
//! - [`conics`]: exact circular and elliptical arcs (A7.1) and open conic
//!   arcs through a point (A7.2, A7.3) as rational quadratics.
//! - [`NurbsSurface`]: construction, evaluation and partial derivatives
//!   (A3.5, A3.6, A4.3, A4.4), normals, principal curvatures ([`Curvatures`])
//!   and control-point bounds; knot insertion (A5.3), refinement (A5.5) and
//!   degree elevation (A5.9 on every column) in either direction; Bezier
//!   decomposition (A5.7), splitting, reversal, swapping `u` and `v`,
//!   isocurves, transforms and closedness checks; closest point
//!   ([`SurfaceClosestPoint`]) by sampling, Newton (section 6.1) and a
//!   branch-and-bound check over Bezier patches; exact surfaces of
//!   revolution (A8.1), extrusions (general cylinders), ruled surfaces and
//!   bilinear patches.
//! - [`make_compatible`]: curves brought to a common degree, domain and
//!   knot vector, as ruled surfaces and lofts need.
//!
//! The curve arc length still uses a local copy of the Gauss-Legendre
//! rules; the closest-point iterations keep their own Newton loops, since
//! the book's convergence tests and the distance-decreasing step control
//! differ from the residual-based iteration in `bevel_math::newton`.
//!
//! Next: surface interpolation and least-squares fitting, analytic curves
//! and surfaces with the `CurveGeom` and `SurfaceGeom` traits, then
//! intersections for booleans.

pub mod basis;
pub mod conics;
pub mod fit;

mod error;
mod gauss_legendre;
mod homogeneous;
mod knots;
mod nurbs;

pub use conics::{circle_arc, circle_arc_2d, conic_arc, ellipse_arc, ellipse_arc_2d};
pub use error::GeomError;
pub use homogeneous::Homogeneous;
pub use knots::KnotVector;
pub use nurbs::{
    ClosestPoint, Curvatures, NurbsCurve, NurbsCurve2, NurbsCurve3, NurbsSurface,
    SurfaceClosestPoint, make_compatible,
};

/// The highest supported degree for curves and surfaces.
///
/// Work tables for basis functions are fixed-size arrays of this size, so
/// evaluation never allocates. Degree 25 matches the limit of the common
/// exchange kernels.
pub const MAX_DEGREE: usize = 25;

/// The highest derivative order [`NurbsCurve::derivatives`] and
/// [`NurbsSurface::derivatives`] compute.
///
/// Geometry needs at most third derivatives; the cap keeps a careless
/// caller from requesting an enormous allocation.
pub const MAX_DERIVATIVE_ORDER: usize = 16;
