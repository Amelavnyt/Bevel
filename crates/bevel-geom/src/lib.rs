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
//!   reduction with an error bound.
//!
//! Next: arc length and point inversion for curves, then analytic curves, NURBS and analytic surfaces, and
//! intersections with booleans.

pub mod basis;

mod aabb2;
mod error;
mod homogeneous;
mod knots;
mod nurbs;

pub use aabb2::Aabb2;
pub use error::GeomError;
pub use homogeneous::Homogeneous;
pub use knots::KnotVector;
pub use nurbs::{NurbsCurve, NurbsCurve2, NurbsCurve3};

/// The highest supported degree for curves and surfaces.
///
/// Work tables for basis functions are fixed-size arrays of this size, so
/// evaluation never allocates. Degree 25 matches the limit of the common
/// exchange kernels.
pub const MAX_DEGREE: usize = 25;
