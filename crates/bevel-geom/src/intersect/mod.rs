// SPDX-License-Identifier: GPL-3.0-or-later

//! Intersections of curves and surfaces.
//!
//! Every function works on bounded pieces: a curve over a parameter range
//! and a surface over a parameter rectangle, as edges and faces use them.
//! Results are sorted deterministically and deduplicated within the
//! tolerance.
//!
//! - [`curve_curve_2d`] and [`curve_curve_3d`]: points where two curves
//!   meet, and stretches where they overlap.
//! - [`curve_surface`]: points where a curve meets a surface, and
//!   stretches where it lies on it.
//! - [`surface_surface`]: the curves where two surfaces cross, each with
//!   its parameter-space curve (pcurve) on both surfaces, plus isolated
//!   touching points, or the verdict that the surfaces coincide.
//!
//! The general method works for any curve or surface, NURBS included:
//! recursive subdivision of the parameter domains, culling pieces whose
//! bounding boxes do not overlap, then Newton refinement. Pairs of
//! analytic types with closed-form answers (plane–plane, plane–sphere,
//! sphere–sphere, ...) take exact fast paths, which tests cross-check
//! against the general method.

mod curve_curve;
mod curve_surface;
mod ssi;

pub use curve_curve::{curve_curve_2d, curve_curve_3d};
pub use curve_surface::curve_surface;
pub use ssi::{surface_surface, surface_surface_general};

use bevel_math::Interval;

use crate::{Curve2, Curve3};

/// A parameter rectangle of a surface: the part of it a face uses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rect {
    /// The `u` range.
    pub u: Interval,
    /// The `v` range.
    pub v: Interval,
}

impl Rect {
    /// The rectangle `u x v`.
    pub fn new(u: Interval, v: Interval) -> Self {
        Self { u, v }
    }

    /// Whether `(u, v)` lies in the rectangle, widened by `tol` in both
    /// directions.
    pub fn contains_with(&self, u: f64, v: f64, tol: f64) -> bool {
        self.u.contains_with(u, tol) && self.v.contains_with(v, tol)
    }
}

/// Where two curves meet.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CurveCurveHit<P> {
    /// They cross or touch at one point.
    Point {
        /// The parameter on the first curve.
        a: f64,
        /// The parameter on the second curve.
        b: f64,
        /// The point (on the first curve).
        point: P,
    },
    /// They coincide over a stretch.
    Overlap {
        /// The stretch on the first curve.
        a: Interval,
        /// The same stretch on the second curve, as an interval
        /// (`b.min < b.max`): `b.min` matches `a.min`, or `a.max` when
        /// `reversed` is set.
        b: Interval,
        /// Whether the second curve runs the other way along the overlap.
        reversed: bool,
    },
}

/// Where a curve meets a surface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CurveSurfaceHit {
    /// They meet at one point.
    Point {
        /// The curve parameter.
        t: f64,
        /// The surface `u`.
        u: f64,
        /// The surface `v`.
        v: f64,
        /// The point (on the curve).
        point: bevel_math::Point3,
    },
    /// The curve lies on the surface over a stretch.
    Overlap {
        /// The stretch of the curve parameter.
        t: Interval,
    },
}

/// How two surfaces meet along a curve.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Contact {
    /// They cross: the normals differ along the curve.
    Transversal,
    /// They touch without crossing: the normals are parallel along the
    /// curve (a cylinder lying on a plane).
    Tangent,
}

/// One curve where two surfaces meet, with its parameter-space curves.
#[derive(Clone, Debug, PartialEq)]
pub struct SsiCurve {
    /// The curve in space.
    pub curve: Curve3,
    /// The parameter range of `curve` that lies in both rectangles.
    pub range: Interval,
    /// The curve in the first surface's parameter space; it shares
    /// `curve`'s parameter: the first surface at `pcurve_a(t)` is
    /// `curve(t)` within `tolerance`.
    pub pcurve_a: Curve2,
    /// The same on the second surface.
    pub pcurve_b: Curve2,
    /// How far the curve and its pcurves (mapped through their surfaces)
    /// may be from the exact intersection, in millimetres; zero for exact
    /// analytic results.
    pub tolerance: f64,
    /// How the surfaces meet along the curve.
    pub contact: Contact,
}

/// An isolated point where two surfaces touch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SsiPoint {
    /// The point.
    pub point: bevel_math::Point3,
    /// Its parameters on the first surface.
    pub a: bevel_math::Point2,
    /// Its parameters on the second surface.
    pub b: bevel_math::Point2,
}

/// The intersection of two surface patches.
#[derive(Clone, Debug, PartialEq)]
pub enum SsiResult {
    /// The patches meet in these curves and isolated points (both possibly
    /// empty).
    Meet {
        /// The intersection curves, ordered deterministically.
        curves: Vec<SsiCurve>,
        /// Points where the patches touch without a curve through them.
        points: Vec<SsiPoint>,
    },
    /// The surfaces are the same surface (possibly with opposite normals)
    /// wherever the patches overlap.
    Coincident {
        /// Whether the normals point the same way.
        same_sense: bool,
    },
}
