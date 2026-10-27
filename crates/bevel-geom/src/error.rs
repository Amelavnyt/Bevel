// SPDX-License-Identifier: GPL-3.0-or-later

//! Errors returned by the curve and surface code.

use thiserror::Error;

/// An error from constructing or operating on curves and surfaces.
///
/// Every constructor validates its input and every operation that can fail
/// on bad input or bad geometry returns one of these instead of panicking.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum GeomError {
    /// The knot vector is not valid for the requested degree; the message
    /// says which rule it breaks.
    #[error("invalid knot vector: {0}")]
    InvalidKnotVector(&'static str),

    /// The degree is outside the supported range
    /// (`1..=`[`MAX_DEGREE`](crate::MAX_DEGREE)) or too high for the
    /// operation.
    #[error("unsupported degree {degree}")]
    InvalidDegree {
        /// The rejected degree.
        degree: usize,
    },

    /// The number of control points (or weights) does not match what the
    /// knot vector and degree require.
    #[error("expected {expected} control points, found {found}")]
    ControlPointCount {
        /// The number required.
        expected: usize,
        /// The number given.
        found: usize,
    },

    /// A weight is zero, negative or not finite. Weights must be finite and
    /// strictly positive so that the curve stays inside the convex hull of
    /// its control points.
    #[error("weight {index} is not a finite positive number")]
    InvalidWeight {
        /// Index of the offending control point.
        index: usize,
    },

    /// A parameter lies outside the curve's domain, or on its boundary where
    /// the operation needs an interior parameter.
    #[error("parameter {t} is outside the domain")]
    ParameterOutOfDomain {
        /// The rejected parameter.
        t: f64,
    },

    /// An iterative method did not converge within its iteration budget.
    #[error("the computation did not converge")]
    NotConverged,

    /// The result cannot be kept within the requested tolerance; `error` is
    /// the error bound the operation would have produced.
    #[error("tolerance not met: the error bound is {error}")]
    ToleranceNotMet {
        /// The error bound in model units (millimetres).
        error: f64,
    },

    /// A coordinate, knot or other input is NaN or infinite.
    #[error("input is not finite")]
    NonFinite,

    /// An argument other than those covered above is invalid; the message
    /// says which one and why.
    #[error("invalid argument: {0}")]
    InvalidArgument(&'static str),
}
