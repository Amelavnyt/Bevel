// SPDX-License-Identifier: GPL-3.0-or-later

//! Curve and surface fitting: interpolation and least-squares
//! approximation of points, and lofting through curves.
//!
//! Surface fitting (grids of points, and lofts) lives in the `surface`
//! submodule and reuses the curve machinery described below one direction
//! at a time. The curve functions build a non-rational
//! [`NurbsCurve`](crate::NurbsCurve) of a chosen degree from an ordered list
//! of points. They are generic over
//! the point type: pass `&[DVec2]` to get a
//! [`NurbsCurve2`](crate::NurbsCurve2) and `&[DVec3]` to get a
//! [`NurbsCurve3`](crate::NurbsCurve3); [`FitPoint`] links the two. Everything
//! follows Piegl and Tiller, *The NURBS Book* (2nd ed.), chapter 9.
//!
//! # Parameters and knots
//!
//! A fit first assigns a parameter `u_k` in `[0, 1]` to every data point
//! `Q_k` ([`parameters`], section 9.2.1): uniform spacing, chord length
//! or the centripetal method (section 9.2.1). The knot vector is then
//! derived from the parameters. For interpolation, [`averaging_knots`]
//! averages `p` consecutive parameters (eq. 9.8), which guarantees that the
//! collocation matrix is non-singular (the Schoenberg-Whitney conditions)
//! and well conditioned.
//!
//! # Interpolation
//!
//! [`interpolate`] is global curve interpolation (A9.1): with `n + 1` points
//! it solves the `(n + 1) x (n + 1)` system `sum_j N_j(u_k) P_j = Q_k` for the
//! control points `P_j`, one column per coordinate. The system is banded,
//! with at most `p + 1` non-zeros per row, but is solved here with nalgebra's
//! dense LU factorisation with partial pivoting; the cost is cubic in the
//! point count, which is no concern for the point counts of sketches and
//! imported data. A (numerically) singular system is reported as
//! [`GeomError::NotConverged`].
//!
//! [`interpolate_with_tangents`] adds the end derivatives (section 9.2.2):
//! two more unknowns and two more equations, `C'(0) = D_0` and `C'(1) = D_n`,
//! with the knot vector of the book's derivative-constrained variant of
//! eq. 9.8.
//!
//! # Least-squares approximation
//!
//! [`approximate`] (section 9.4.1) fits fewer control points than data
//! points: the first and last point are interpolated and the interior
//! control points minimise the sum of squared distances `|Q_k - C(u_k)|^2`
//! of the other points (section 9.4.1). The knots are spread to
//! follow the data parameters (section 9.4.1), so every knot span holds
//! at least one parameter. The over-determined system is solved through a
//! Householder QR factorisation of the design matrix instead of the normal
//! equations `N^T N P = R`: the normal equations square the condition number,
//! while QR does not, at the cost of a little more work. A rank-deficient
//! design matrix is reported as [`GeomError::NotConverged`].
//! [`approximate_to_tolerance`] searches for a control point count that keeps
//! the deviation below a tolerance.
//!
//! All results are deterministic: the same input always gives bit-identical
//! output.

mod approximate;
mod interpolate;
mod solve;
mod surface;

use std::fmt::Debug;
use std::ops::{Add, Mul, Sub};

use bevel_math::{DVec2, DVec3, DVec4};

use crate::knots::check_degree;
use crate::{GeomError, Homogeneous, KnotVector};

pub use approximate::{approximate, approximate_to_tolerance};
pub(crate) use interpolate::interpolate_at;
pub use interpolate::{interpolate, interpolate_with_tangents};
pub use surface::{
    approximate_surface, interpolate_surface, loft, loft_parameters, surface_parameters,
};

/// Consecutive data points closer than this fraction of the data extent are
/// treated as coincident.
const COINCIDENT_TOLERANCE: f64 = 1e-12;

/// A model-space point that curves can be fitted through: [`DVec2`] for
/// planar curves and [`DVec3`] for curves in space.
///
/// The trait gives the fitting code coordinate access (to assemble the linear
/// systems, one column per coordinate) and ties the point type to the
/// homogeneous type of the resulting curve.
pub trait FitPoint:
    Copy + Add<Output = Self> + Sub<Output = Self> + Mul<f64, Output = Self> + Debug
{
    /// The homogeneous control point type of curves through these points.
    type Homogeneous: Homogeneous<Point = Self>;

    /// The number of coordinates of a point.
    const DIM: usize;

    /// The coordinate along `axis`, or 0 if `axis >= DIM`.
    fn coord(self, axis: usize) -> f64;

    /// The point with the given leading coordinates; missing ones are 0 and
    /// extra ones are ignored.
    fn from_coords(coords: &[f64]) -> Self;

    /// The Euclidean length of the vector from the origin.
    fn length(self) -> f64 {
        <Self::Homogeneous as Homogeneous>::point_length(self)
    }

    /// The dot product with `other`.
    fn dot(self, other: Self) -> f64 {
        <Self::Homogeneous as Homogeneous>::point_dot(self, other)
    }

    /// Returns `true` if every coordinate is finite.
    fn is_finite(self) -> bool {
        (0..Self::DIM).all(|axis| self.coord(axis).is_finite())
    }
}

/// The coordinate at `index`, or 0 beyond the end of the slice.
fn at(coords: &[f64], index: usize) -> f64 {
    coords.get(index).copied().unwrap_or(0.0)
}

impl FitPoint for DVec2 {
    type Homogeneous = DVec3;
    const DIM: usize = 2;

    fn coord(self, axis: usize) -> f64 {
        match axis {
            0 => self.x,
            1 => self.y,
            _ => 0.0,
        }
    }

    fn from_coords(coords: &[f64]) -> Self {
        DVec2::new(at(coords, 0), at(coords, 1))
    }
}

impl FitPoint for DVec3 {
    type Homogeneous = DVec4;
    const DIM: usize = 3;

    fn coord(self, axis: usize) -> f64 {
        match axis {
            0 => self.x,
            1 => self.y,
            2 => self.z,
            _ => 0.0,
        }
    }

    fn from_coords(coords: &[f64]) -> Self {
        DVec3::new(at(coords, 0), at(coords, 1), at(coords, 2))
    }
}

/// How parameters are assigned to the data points of a fit (The NURBS Book,
/// section 9.2.1).
///
/// With `d_k = |Q_k - Q_{k-1}|` the distance between consecutive points, the
/// parameter of point `k` is the share `(w_1 + ... + w_k) / (w_1 + ... + w_n)`
/// of the total, where the segment weight `w_k` depends on the method.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum Parametrization {
    /// Equally spaced parameters, `u_k = k / n` (`w_k = 1`). Ignores the
    /// geometry, so it suits data that is itself sampled evenly.
    Uniform,
    /// Parameters proportional to the accumulated chord length (section 9.2.1,
    /// `w_k = d_k`): the usual choice and a good default.
    ChordLength,
    /// Parameters proportional to the accumulated square root of the chord
    /// length (section 9.2.1, `w_k = sqrt(d_k)`): the centripetal method, which
    /// avoids overshoot and loops at sharp turns in unevenly spaced data.
    Centripetal,
}

/// Rejects non-finite coordinates and fewer than `min` points.
fn check_points<P: FitPoint>(points: &[P], min: usize) -> Result<(), GeomError> {
    if points.iter().any(|&p| !p.is_finite()) {
        return Err(GeomError::NonFinite);
    }
    if points.len() < min {
        return Err(GeomError::InvalidArgument(
            "too few data points for the requested degree",
        ));
    }
    Ok(())
}

/// Parameters in `[0, 1]` for the points, with the first at 0, the last at 1
/// and the rest strictly increasing in between (The NURBS Book, section
/// 9.2.1).
///
/// # Errors
///
/// [`GeomError::InvalidArgument`] for fewer than two points or if two
/// consecutive points coincide (closer than 1e-12 of the extent of the
/// data): they would get equal parameters. [`GeomError::NonFinite`] for NaN
/// or infinite coordinates.
pub fn parameters<P: FitPoint>(
    points: &[P],
    method: Parametrization,
) -> Result<Vec<f64>, GeomError> {
    check_points(points, 2)?;
    let Some(&first) = points.first() else {
        return Err(GeomError::InvalidArgument("no data points"));
    };
    let extent = points
        .iter()
        .map(|&p| (p - first).length())
        .fold(0.0, f64::max);
    let chords: Vec<f64> = points.windows(2).map(|w| (w[1] - w[0]).length()).collect();
    if chords.iter().any(|&d| d <= COINCIDENT_TOLERANCE * extent) {
        return Err(GeomError::InvalidArgument(
            "consecutive data points coincide",
        ));
    }
    let weight = |d: f64| match method {
        Parametrization::Uniform => 1.0,
        Parametrization::ChordLength => d,
        Parametrization::Centripetal => d.sqrt(),
    };
    let total: f64 = chords.iter().map(|&d| weight(d)).sum();
    let mut params = Vec::with_capacity(points.len());
    let mut acc = 0.0;
    params.push(0.0);
    for &d in &chords {
        acc += weight(d);
        params.push(acc / total);
    }
    // The last parameter is exactly 1 whatever the rounding.
    if let Some(last) = params.last_mut() {
        *last = 1.0;
    }
    if params.windows(2).any(|w| w[1] <= w[0]) {
        return Err(GeomError::InvalidArgument(
            "the data points are too close together to give distinct parameters",
        ));
    }
    Ok(params)
}

/// Checks that `params` is finite and strictly increasing.
fn check_params(params: &[f64]) -> Result<(), GeomError> {
    if params.iter().any(|u| !u.is_finite()) {
        return Err(GeomError::NonFinite);
    }
    if params.windows(2).any(|w| w[1] <= w[0]) {
        return Err(GeomError::InvalidArgument(
            "parameters must be strictly increasing",
        ));
    }
    Ok(())
}

/// A clamped knot vector for interpolating at `params`, by averaging `degree`
/// consecutive parameters (The NURBS Book, eq. 9.8):
/// `u_{j+p} = (u*_j + ... + u*_{j+p-1}) / p` for `j = 1, ..., n - p`.
///
/// The domain is `[params[0], params[n]]` (`[0, 1]` for the output of
/// [`parameters`]). With `n + 1` parameters the vector fits `n + 1` control
/// points, and every parameter lies where the Schoenberg-Whitney conditions
/// need it, so the interpolation system is non-singular.
///
/// # Errors
///
/// [`GeomError::InvalidDegree`] for a degree outside `1..=MAX_DEGREE`,
/// [`GeomError::InvalidArgument`] for fewer than `degree + 1` parameters or
/// parameters that are not strictly increasing, and [`GeomError::NonFinite`]
/// for NaN or infinite parameters.
pub fn averaging_knots(params: &[f64], degree: usize) -> Result<KnotVector, GeomError> {
    check_degree(degree)?;
    check_params(params)?;
    if params.len() < degree + 1 {
        return Err(GeomError::InvalidArgument(
            "at least degree + 1 parameters are needed",
        ));
    }
    let (Some(&start), Some(&end)) = (params.first(), params.last()) else {
        return Err(GeomError::InvalidArgument("no parameters"));
    };
    // Windows start at j = 0; j = 1..=n-p are the interior knots.
    let interior: Vec<f64> = params
        .windows(degree)
        .skip(1)
        .take(params.len() - 1 - degree)
        .map(|w| w.iter().sum::<f64>() / degree as f64)
        .collect();
    KnotVector::clamped_from_interior(degree, start, end, &interior)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn collinear(count: usize) -> Vec<DVec3> {
        (0..count)
            .map(|k| DVec3::new(1.0, -2.0, 0.5) + DVec3::new(3.0, 1.0, -2.0) * k as f64)
            .collect()
    }

    #[test]
    fn equally_spaced_collinear_points_get_uniform_parameters() {
        let pts = collinear(7);
        for method in [
            Parametrization::Uniform,
            Parametrization::ChordLength,
            Parametrization::Centripetal,
        ] {
            let u = parameters(&pts, method).unwrap();
            assert_eq!(u.len(), 7);
            assert!(u[0].abs() < 1e-15);
            assert!((u[6] - 1.0).abs() < 1e-15);
            for (k, &v) in u.iter().enumerate() {
                assert!((v - k as f64 / 6.0).abs() < 1e-14, "{method:?} {k}");
            }
        }
    }

    #[test]
    fn chord_length_and_centripetal_follow_the_distances() {
        // Segments of length 1, 3 and 4.
        let pts = [
            DVec2::new(0.0, 0.0),
            DVec2::new(1.0, 0.0),
            DVec2::new(1.0, 3.0),
            DVec2::new(5.0, 3.0),
        ];
        let c = parameters(&pts, Parametrization::ChordLength).unwrap();
        for (a, b) in c.iter().zip([0.0, 1.0 / 8.0, 4.0 / 8.0, 1.0]) {
            assert!((a - b).abs() < 1e-15);
        }
        let s = [0.0, 1.0, 1.0 + 3f64.sqrt(), 3.0 + 3f64.sqrt()];
        let z = parameters(&pts, Parametrization::Centripetal).unwrap();
        for (a, b) in z.iter().zip(s) {
            assert!((a - b / s[3]).abs() < 1e-15);
        }
    }

    #[test]
    fn parameters_reject_bad_points() {
        let one = [DVec2::new(1.0, 1.0)];
        assert!(matches!(
            parameters(&one, Parametrization::Uniform),
            Err(GeomError::InvalidArgument(_))
        ));
        let dup = [
            DVec2::new(0.0, 0.0),
            DVec2::new(1.0, 0.0),
            DVec2::new(1.0, 0.0),
        ];
        for method in [
            Parametrization::Uniform,
            Parametrization::ChordLength,
            Parametrization::Centripetal,
        ] {
            assert!(matches!(
                parameters(&dup, method),
                Err(GeomError::InvalidArgument(_))
            ));
        }
        let nan = [DVec2::new(0.0, 0.0), DVec2::new(f64::NAN, 0.0)];
        assert_eq!(
            parameters(&nan, Parametrization::Uniform),
            Err(GeomError::NonFinite)
        );
        let same = [DVec2::new(2.0, 2.0); 3];
        assert!(parameters(&same, Parametrization::ChordLength).is_err());
    }

    #[test]
    fn averaging_knots_match_the_nurbs_book_example() {
        // Example 9.1: Q = (0,0), (3,4), (-1,4), (-4,0), (-4,-3), chord
        // length parameters 0, 5/17, 9/17, 14/17, 1 and degree 3 give
        // U = {0, 0, 0, 0, 28/51, 1, 1, 1, 1}.
        let pts = [
            DVec2::new(0.0, 0.0),
            DVec2::new(3.0, 4.0),
            DVec2::new(-1.0, 4.0),
            DVec2::new(-4.0, 0.0),
            DVec2::new(-4.0, -3.0),
        ];
        let u = parameters(&pts, Parametrization::ChordLength).unwrap();
        for (a, b) in u.iter().zip([0.0, 5.0, 9.0, 14.0, 17.0]) {
            assert!((a - b / 17.0).abs() < 1e-15);
        }
        let k = averaging_knots(&u, 3).unwrap();
        let expected = [0.0, 0.0, 0.0, 0.0, 28.0 / 51.0, 1.0, 1.0, 1.0, 1.0];
        assert_eq!(k.as_slice().len(), expected.len());
        for (a, b) in k.as_slice().iter().zip(expected) {
            assert!((a - b).abs() < 1e-15);
        }
        // Degree 2 averages pairs: (5 + 9) / 34 and (9 + 14) / 34.
        let k = averaging_knots(&u, 2).unwrap();
        let expected = [0.0, 0.0, 0.0, 14.0 / 34.0, 23.0 / 34.0, 1.0, 1.0, 1.0];
        for (a, b) in k.as_slice().iter().zip(expected) {
            assert!((a - b).abs() < 1e-15);
        }
        // Degree 1 uses the interior parameters themselves.
        let k = averaging_knots(&u, 1).unwrap();
        assert_eq!(k.as_slice().len(), 7);
        assert!((k.as_slice()[3] - 9.0 / 17.0).abs() < 1e-15);
    }

    #[test]
    fn averaging_knots_reject_bad_input() {
        let u = [0.0, 0.3, 0.6, 1.0];
        assert!(matches!(
            averaging_knots(&u, 0),
            Err(GeomError::InvalidDegree { degree: 0 })
        ));
        assert!(matches!(
            averaging_knots(&u, 4),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            averaging_knots(&[0.0, 0.5, 0.5, 1.0], 2),
            Err(GeomError::InvalidArgument(_))
        ));
        assert_eq!(
            averaging_knots(&[0.0, f64::NAN, 1.0], 2),
            Err(GeomError::NonFinite)
        );
        // Exactly degree + 1 parameters: a Bezier knot vector.
        let k = averaging_knots(&u, 3).unwrap();
        assert_eq!(k.as_slice(), &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
    }
}
