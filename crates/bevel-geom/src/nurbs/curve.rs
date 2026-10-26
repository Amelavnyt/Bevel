// SPDX-License-Identifier: GPL-3.0-or-later

//! NURBS curves.
//!
//! A NURBS curve of degree `p` is `C(u) = sum N_{i,p}(u) w_i P_i / sum
//! N_{i,p}(u) w_i` on the domain `[u_p, u_{n+1}]` of its knot vector. It is
//! stored as homogeneous control points `Pw_i = (w_i P_i, w_i)` (see
//! [`Homogeneous`]), so every algorithm is written once for 2D and 3D and
//! the representation-changing operations are linear in the stored points.
//!
//! # Evaluation
//!
//! - [`NurbsCurve::point`] sums the non-zero basis functions (A2.2) against
//!   the homogeneous points and projects (A4.1). For a non-rational curve
//!   all weights are equal, the weight sum is that weight and this is the
//!   B-spline point algorithm A3.1.
//! - [`NurbsCurve::derivatives`] computes the derivatives of the homogeneous
//!   curve with A3.2 (on top of A2.3), then those of the projected curve
//!   with A4.2, the Leibniz rule applied to `A(u) = w(u) C(u)`:
//!   `C^(k) = (A^(k) - sum_{i=1..k} binom(k, i) w^(i) C^(k-i)) / w`.
//!   Rational curves have non-zero derivatives of every order; for a
//!   non-rational curve those above the degree are zero.
//!
//! # Representation changes
//!
//! Knot insertion, refinement and removal, degree elevation and reduction,
//! Bezier decomposition, splitting, reversal and reparametrisation live in
//! the submodules; each one documents the algorithm it uses. All of them
//! except degree reduction and knot removal leave the geometry unchanged up
//! to rounding; those two stay within the tolerance they are given.

use bevel_math::{Aabb3, DMat3, DMat4, DVec3, DVec4};

use crate::basis::{basis_funs_raw, ders_basis_funs_raw};
use crate::knots::check_degree;
use crate::{Aabb2, GeomError, Homogeneous, KnotVector, MAX_DEGREE};

/// Relative difference between weights above which a curve counts as
/// rational.
const RATIONAL_WEIGHT_TOLERANCE: f64 = 1e-14;

/// A non-uniform rational B-spline curve over homogeneous control points `H`.
///
/// Use [`NurbsCurve2`] for curves in the plane (sketches, parameter space)
/// and [`NurbsCurve3`] for curves in model space.
#[derive(Clone, Debug, PartialEq)]
pub struct NurbsCurve<H: Homogeneous> {
    /// Invariant: `1 <= degree <= MAX_DEGREE`.
    degree: usize,
    /// Invariant: valid for `degree`, with
    /// `knots.len() == control.len() + degree + 1`.
    knots: KnotVector,
    /// Invariant: finite, with strictly positive weights.
    control: Vec<H>,
    /// Cached: some weight differs from the first.
    rational: bool,
}

/// A NURBS curve in the plane; control points are `(w x, w y, w)`.
pub type NurbsCurve2 = NurbsCurve<DVec3>;

/// A NURBS curve in 3D model space; control points are `(w x, w y, w z, w)`.
pub type NurbsCurve3 = NurbsCurve<DVec4>;

impl<H: Homogeneous> NurbsCurve<H> {
    /// A curve from its degree, knot vector and homogeneous control points
    /// `(w P, w)`.
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidDegree`] or [`GeomError::InvalidKnotVector`] if
    /// the knot vector is not valid for `degree`,
    /// [`GeomError::ControlPointCount`] unless there are
    /// `knots.len() - degree - 1` control points, [`GeomError::InvalidWeight`]
    /// for a weight that is not finite and positive, and
    /// [`GeomError::NonFinite`] for a non-finite coordinate.
    pub fn new(
        degree: usize,
        knots: KnotVector,
        control_weighted: Vec<H>,
    ) -> Result<Self, GeomError> {
        check_degree(degree)?;
        knots.validate(degree)?;
        let expected = knots.as_slice().len() - degree - 1;
        if control_weighted.len() != expected {
            return Err(GeomError::ControlPointCount {
                expected,
                found: control_weighted.len(),
            });
        }
        for (index, c) in control_weighted.iter().enumerate() {
            let w = c.weight();
            if !(w.is_finite() && w > 0.0) {
                return Err(GeomError::InvalidWeight { index });
            }
            if !c.is_finite() {
                return Err(GeomError::NonFinite);
            }
        }
        Ok(Self::from_parts(degree, knots, control_weighted))
    }

    /// A curve from model-space control points and optional weights; without
    /// weights the curve is non-rational (all weights 1).
    ///
    /// # Errors
    ///
    /// As [`NurbsCurve::new`], with [`GeomError::ControlPointCount`] also when
    /// `weights` and `points` differ in length.
    pub fn from_points(
        degree: usize,
        knots: KnotVector,
        points: &[H::Point],
        weights: Option<&[f64]>,
    ) -> Result<Self, GeomError> {
        let control = match weights {
            None => points.iter().map(|&p| H::from_point(p, 1.0)).collect(),
            Some(w) => {
                if w.len() != points.len() {
                    return Err(GeomError::ControlPointCount {
                        expected: points.len(),
                        found: w.len(),
                    });
                }
                if let Some(index) = w.iter().position(|&w| !(w.is_finite() && w > 0.0)) {
                    return Err(GeomError::InvalidWeight { index });
                }
                points
                    .iter()
                    .zip(w)
                    .map(|(&p, &w)| H::from_point(p, w))
                    .collect()
            }
        };
        Self::new(degree, knots, control)
    }

    /// The non-rational Bezier curve of degree `points.len() - 1` on `[0, 1]`.
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidDegree`] for fewer than two or more than
    /// `MAX_DEGREE + 1` points, and [`GeomError::NonFinite`] for non-finite
    /// coordinates.
    pub fn bezier(points: &[H::Point]) -> Result<Self, GeomError> {
        let degree = points.len().saturating_sub(1);
        check_degree(degree)?;
        let knots = KnotVector::clamped_from_interior(degree, 0.0, 1.0, &[])?;
        Self::from_points(degree, knots, points, None)
    }

    /// Assembles a curve from parts that already satisfy the type invariants.
    pub(crate) fn from_parts(degree: usize, knots: KnotVector, control: Vec<H>) -> Self {
        let rational = match control.first() {
            Some(first) => {
                let w0 = first.weight();
                control
                    .iter()
                    .any(|c| (c.weight() - w0).abs() > RATIONAL_WEIGHT_TOLERANCE * w0)
            }
            None => false,
        };
        Self {
            degree,
            knots,
            control,
            rational,
        }
    }

    /// The polynomial degree.
    pub fn degree(&self) -> usize {
        self.degree
    }

    /// The knot vector.
    pub fn knots(&self) -> &KnotVector {
        &self.knots
    }

    /// The model-space control points.
    pub fn control_points(&self) -> Vec<H::Point> {
        self.control.iter().map(|c| c.project()).collect()
    }

    /// The weights of the control points.
    pub fn weights(&self) -> Vec<f64> {
        self.control.iter().map(|c| c.weight()).collect()
    }

    /// The homogeneous control points `(w P, w)`.
    pub fn control_weighted(&self) -> &[H] {
        &self.control
    }

    /// Returns `true` if some weight differs from the first by more than
    /// 1e-14 relative; such a curve is genuinely rational.
    pub fn is_rational(&self) -> bool {
        self.rational
    }

    /// The parameter domain `[u_p, u_{n+1}]`.
    pub fn domain(&self) -> (f64, f64) {
        self.knots.domain(self.degree)
    }

    /// `u` clamped to the domain; NaN stays NaN.
    pub(crate) fn clamp_param(&self, u: f64) -> f64 {
        let (a, b) = self.domain();
        if u.is_nan() { u } else { u.clamp(a, b) }
    }

    /// The largest distance of a control point from the origin, but at least
    /// 1: the length scale for relative tolerances.
    pub(crate) fn size(&self) -> f64 {
        self.control
            .iter()
            .map(|c| H::point_length(c.project()))
            .fold(1.0, f64::max)
    }

    /// The point at parameter `u` (The NURBS Book, A4.1; A3.1 when
    /// non-rational).
    ///
    /// Parameters outside the domain are clamped to it; NaN gives a NaN point.
    pub fn point(&self, u: f64) -> H::Point {
        let p = self.degree;
        let u = self.clamp_param(u);
        let span = self.knots.span(p, u);
        let n = basis_funs_raw(self.knots.as_slice(), span, u, p);
        // span - p + j <= span <= n, and control.len() == n + 1.
        let cw = self.control[span - p..=span]
            .iter()
            .zip(&n)
            .fold(H::ZERO, |acc, (&c, &b)| acc + c * b);
        cw.project()
    }

    /// The point and the derivatives at `u` up to order `n`:
    /// `result[k]` is `C^(k)(u)`, so `result[0]` is [`NurbsCurve::point`]
    /// (The NURBS Book, A3.2 and A4.2).
    ///
    /// Parameters outside the domain are clamped to it. At a knot the
    /// derivatives are those of the span that starts there (the last span at
    /// the end of the domain).
    pub fn derivatives(&self, u: f64, n: usize) -> Vec<H::Point> {
        let mut out = vec![H::point_zero(); n + 1];
        self.derivatives_into(u, &mut out);
        out
    }

    /// [`NurbsCurve::derivatives`] into a caller-provided buffer of
    /// `n + 1` points.
    pub(crate) fn derivatives_into(&self, u: f64, out: &mut [H::Point]) {
        let Some(n) = out.len().checked_sub(1) else {
            return;
        };
        let p = self.degree;
        let u = self.clamp_param(u);
        let span = self.knots.span(p, u);
        let du = n.min(p);
        let ders = ders_basis_funs_raw(self.knots.as_slice(), span, u, p, du);
        // A3.2 on the homogeneous points; derivatives above the degree vanish.
        let mut aders = [H::ZERO; MAX_DEGREE + 1];
        for (ak, row) in aders.iter_mut().zip(&ders).take(du + 1) {
            *ak = self.control[span - p..=span]
                .iter()
                .zip(row)
                .fold(H::ZERO, |acc, (&c, &b)| acc + c * b);
        }
        // A4.2.
        let inv_w = 1.0 / aders[0].weight();
        for k in 0..=n {
            let mut v = if k <= du {
                aders[k].weighted()
            } else {
                H::point_zero()
            };
            if self.rational {
                let mut binom = 1.0;
                for i in 1..=k.min(du) {
                    binom = binom * (k + 1 - i) as f64 / i as f64;
                    v = v - out[k - i] * (binom * aders[i].weight());
                }
            }
            out[k] = v * inv_w;
        }
    }

    /// Speeds below this count as a vanishing first derivative: moving at it
    /// across the whole domain covers 1e-12 of the curve's size.
    pub(crate) fn speed_floor(&self) -> f64 {
        let (a, b) = self.domain();
        1e-12 * self.size() / (b - a)
    }

    /// The unit tangent at `u`, or `None` where the first derivative
    /// vanishes (see [`NurbsCurve::derivatives`] for parameters at knots).
    pub fn tangent(&self, u: f64) -> Option<H::Point> {
        let mut d = [H::point_zero(); 2];
        self.derivatives_into(u, &mut d);
        let speed = H::point_length(d[1]);
        (speed > self.speed_floor()).then(|| d[1] * (1.0 / speed))
    }

    /// First and second derivative at `u` and the speed, or `None` where the
    /// first derivative vanishes.
    fn first_two_derivatives(&self, u: f64) -> Option<(H::Point, H::Point, f64)> {
        let mut d = [H::point_zero(); 3];
        self.derivatives_into(u, &mut d);
        let speed = H::point_length(d[1]);
        (speed > self.speed_floor()).then_some((d[1], d[2], speed))
    }
}

impl NurbsCurve2 {
    /// The signed curvature `(x' y'' - y' x'') / |C'|^3` at `u`: positive
    /// where the curve turns counter-clockwise. `None` where the first
    /// derivative vanishes.
    pub fn curvature(&self, u: f64) -> Option<f64> {
        let (d1, d2, speed) = self.first_two_derivatives(u)?;
        Some(d1.perp_dot(d2) / (speed * speed * speed))
    }

    /// The curve under the 2D homogeneous transform `m` (a 3x3 matrix acting
    /// on `(x, y, 1)`).
    ///
    /// The matrix is applied to the homogeneous control points. For an affine
    /// `m` (last row `0 0 1`) this moves the control points and keeps the
    /// weights; NURBS are invariant under such maps, so the result is exactly
    /// the transformed curve. A projective `m` is applied exactly too, as long
    /// as all new weights stay positive.
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidWeight`] if a weight becomes zero or negative and
    /// [`GeomError::NonFinite`] for a non-finite matrix.
    pub fn transformed(&self, m: &DMat3) -> Result<Self, GeomError> {
        let control = self.control.iter().map(|&c| *m * c).collect();
        Self::new(self.degree, self.knots.clone(), control)
    }

    /// The axis-aligned box of the control points. Because the weights are
    /// positive the curve lies in their convex hull, so the box contains the
    /// whole curve.
    pub fn control_bounds(&self) -> Aabb2 {
        Aabb2::from_points(self.control.iter().map(|c| c.project()))
    }
}

impl NurbsCurve3 {
    /// The curvature `|C' x C''| / |C'|^3` at `u`, or `None` where the first
    /// derivative vanishes.
    pub fn curvature(&self, u: f64) -> Option<f64> {
        let (d1, d2, speed) = self.first_two_derivatives(u)?;
        Some(d1.cross(d2).length() / (speed * speed * speed))
    }

    /// The curve under the transform `m` of model space.
    ///
    /// The matrix is applied to the homogeneous control points. For an affine
    /// `m` (last row `0 0 0 1`) this moves the control points and keeps the
    /// weights; NURBS are invariant under such maps, so the result is exactly
    /// the transformed curve. A projective `m` is applied exactly too, as long
    /// as all new weights stay positive.
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidWeight`] if a weight becomes zero or negative and
    /// [`GeomError::NonFinite`] for a non-finite matrix.
    pub fn transformed(&self, m: &DMat4) -> Result<Self, GeomError> {
        let control = self.control.iter().map(|&c| *m * c).collect();
        Self::new(self.degree, self.knots.clone(), control)
    }

    /// The axis-aligned box of the control points. Because the weights are
    /// positive the curve lies in their convex hull, so the box contains the
    /// whole curve.
    pub fn control_bounds(&self) -> Aabb3 {
        Aabb3::from_points(self.control.iter().map(|c| c.project()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevel_math::DVec2;

    fn p2(x: f64, y: f64) -> DVec2 {
        DVec2::new(x, y)
    }

    fn quarter_circle(r: f64) -> NurbsCurve2 {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let knots = KnotVector::clamped_from_interior(2, 0.0, 1.0, &[]).unwrap();
        NurbsCurve2::from_points(
            2,
            knots,
            &[p2(r, 0.0), p2(r, r), p2(0.0, r)],
            Some(&[1.0, h, 1.0]),
        )
        .unwrap()
    }

    #[test]
    fn rational_quarter_circle_has_constant_curvature() {
        let r = 7.5;
        let c = quarter_circle(r);
        assert!(c.is_rational());
        for i in 0..=100 {
            let u = f64::from(i) / 100.0;
            let p = c.point(u);
            assert!((p.length() - r).abs() < 1e-12, "radius at {u}");
            let k = c.curvature(u).unwrap();
            assert!((k - 1.0 / r).abs() < 1e-9, "curvature {k} at {u}");
            let t = c.tangent(u).unwrap();
            assert!(t.dot(p).abs() < 1e-9, "tangent not perpendicular at {u}");
        }
    }

    #[test]
    fn derivatives_start_with_the_point() {
        let c = quarter_circle(2.0);
        for u in [0.0, 0.25, 0.5, 1.0, -3.0, 4.0] {
            let d = c.derivatives(u, 4);
            assert_eq!(d.len(), 5);
            assert!((d[0] - c.point(u)).length() < 1e-15);
        }
        assert!(c.point(f64::NAN).is_nan());
    }

    #[test]
    fn straight_line_has_zero_curvature_and_unit_tangent() {
        let c = NurbsCurve3::bezier(&[
            DVec3::ZERO,
            DVec3::new(1.0, 2.0, 2.0),
            DVec3::new(2.0, 4.0, 4.0),
        ])
        .unwrap();
        assert!(!c.is_rational());
        let t = c.tangent(0.3).unwrap();
        assert!((t - DVec3::new(1.0, 2.0, 2.0) / 3.0).length() < 1e-15);
        assert!(c.curvature(0.3).unwrap().abs() < 1e-12);
        // A degenerate curve has no tangent.
        let c = NurbsCurve3::bezier(&[DVec3::ONE, DVec3::ONE]).unwrap();
        assert!(c.tangent(0.5).is_none());
        assert!(c.curvature(0.5).is_none());
    }

    #[test]
    fn invalid_inputs_are_rejected() {
        let knots = KnotVector::clamped_uniform(2, 4).unwrap();
        let pts = [p2(0.0, 0.0), p2(1.0, 0.0), p2(1.0, 1.0), p2(0.0, 1.0)];
        assert!(matches!(
            NurbsCurve2::from_points(2, knots.clone(), &pts[..3], None),
            Err(GeomError::ControlPointCount {
                expected: 4,
                found: 3
            })
        ));
        assert!(matches!(
            NurbsCurve2::from_points(2, knots.clone(), &pts, Some(&[1.0, 0.0, 1.0, 1.0])),
            Err(GeomError::InvalidWeight { index: 1 })
        ));
        assert!(matches!(
            NurbsCurve2::from_points(2, knots.clone(), &pts, Some(&[1.0, 1.0, 1.0, -2.0])),
            Err(GeomError::InvalidWeight { index: 3 })
        ));
        assert!(matches!(
            NurbsCurve2::from_points(2, knots.clone(), &pts, Some(&[1.0, f64::NAN, 1.0, 1.0])),
            Err(GeomError::InvalidWeight { index: 1 })
        ));
        assert!(matches!(
            NurbsCurve2::from_points(2, knots.clone(), &pts, Some(&[1.0, 1.0])),
            Err(GeomError::ControlPointCount {
                expected: 4,
                found: 2
            })
        ));
        let mut bad = pts;
        bad[2].x = f64::NAN;
        assert!(matches!(
            NurbsCurve2::from_points(2, knots.clone(), &bad, None),
            Err(GeomError::NonFinite)
        ));
        // A knot vector valid for degree 2 is not valid for degree 1 here
        // (the interior knot count no longer matches), and degree 0 is
        // unsupported.
        assert!(matches!(
            NurbsCurve2::from_points(1, knots.clone(), &pts, None),
            Err(GeomError::ControlPointCount { .. } | GeomError::InvalidKnotVector(_))
        ));
        assert!(matches!(
            NurbsCurve2::from_points(0, knots, &pts, None),
            Err(GeomError::InvalidDegree { degree: 0 })
        ));
        assert!(matches!(
            NurbsCurve2::bezier(&pts[..1]),
            Err(GeomError::InvalidDegree { degree: 0 })
        ));
    }

    #[test]
    fn affine_transform_moves_the_curve() {
        let c = quarter_circle(1.0);
        let m = DMat3::from_scale_angle_translation(DVec2::splat(2.0), 0.5, DVec2::new(3.0, -1.0));
        let t = c.transformed(&m).unwrap();
        for i in 0..=10 {
            let u = f64::from(i) / 10.0;
            let expected = m.transform_point2(c.point(u));
            assert!((t.point(u) - expected).length() < 1e-13);
        }
        assert_eq!(t.weights(), c.weights());

        let c3 = NurbsCurve3::bezier(&[DVec3::ZERO, DVec3::X, DVec3::ONE]).unwrap();
        let m = DMat4::from_rotation_translation(
            bevel_math::DQuat::from_rotation_z(1.0),
            DVec3::new(1.0, 2.0, 3.0),
        );
        let t = c3.transformed(&m).unwrap();
        assert!((t.point(0.4) - m.transform_point3(c3.point(0.4))).length() < 1e-14);
        assert!(c3.transformed(&DMat4::ZERO).is_err());
    }

    #[test]
    fn control_bounds_contain_the_curve() {
        let c = quarter_circle(3.0);
        let b = c.control_bounds();
        assert_eq!(b.min, p2(0.0, 0.0));
        assert_eq!(b.max, p2(3.0, 3.0));
        for i in 0..=20 {
            assert!(b.contains(c.point(f64::from(i) / 20.0)));
        }
    }
}
