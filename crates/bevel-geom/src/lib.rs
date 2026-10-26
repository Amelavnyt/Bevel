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
//!   knots (eq. 9.4, 9.5, 9.8), global interpolation (A9.1) with optional
//!   end tangents, and least-squares approximation with a tolerance search
//!   (section 9.4.1).
//!
//! The Gauss-Legendre rules and the Newton iteration are local and private
//! for now; they move to the numerics in `bevel-math` once it has them.
//!
//! Next: curve construction (exact circles, arcs and conics as rational
//! quadratics; interpolation and least-squares fitting), analytic curves and
//! the `CurveGeom` trait, then NURBS and analytic surfaces, and
//! intersections with booleans.

pub mod basis;
pub mod fit;

mod error;
mod gauss_legendre;
mod homogeneous;
mod knots;
mod nurbs;

pub use error::GeomError;
pub use homogeneous::Homogeneous;
pub use knots::KnotVector;
pub use nurbs::{ClosestPoint, NurbsCurve, NurbsCurve2, NurbsCurve3};

/// The highest supported degree for curves and surfaces.
///
/// Work tables for basis functions are fixed-size arrays of this size, so
/// evaluation never allocates. Degree 25 matches the limit of the common
/// exchange kernels.
pub const MAX_DEGREE: usize = 25;

/// The highest derivative order [`NurbsCurve::derivatives`] computes.
///
/// Geometry needs at most third derivatives; the cap keeps a careless
/// caller from requesting an enormous allocation.
pub const MAX_DERIVATIVE_ORDER: usize = 16;
