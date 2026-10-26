// SPDX-License-Identifier: GPL-3.0-or-later

//! NURBS surfaces.
//!
//! A NURBS surface of degrees `(p, q)` is the tensor product
//! `S(u, v) = sum_i sum_j N_{i,p}(u) N_{j,q}(v) w_ij P_ij /
//! sum_i sum_j N_{i,p}(u) N_{j,q}(v) w_ij` over a knot vector in `u` and one
//! in `v` (The NURBS Book, eq. 4.15). As for curves it is stored as
//! homogeneous control points `Pw_ij = (w_ij P_ij, w_ij)` (ADR-0016), so
//! the surface is the projection of the non-rational surface `Sw(u, v)`
//! one dimension up, and every representation change is linear in the
//! stored points.
//!
//! # Control net layout
//!
//! The control net has `count_u` rows (index `i`, along `u`) of `count_v`
//! points each (index `j`, along `v`), stored row by row: point `(i, j)` is
//! at index `i * count_v + j`.
//!
//! # Evaluation
//!
//! - [`NurbsSurface::point`] is A4.3 (A3.5 on the homogeneous points, then
//!   the projection): the `p + 1` basis functions in `u` are applied down
//!   each of the `q + 1` affected columns, and the `q + 1` results are
//!   combined with the basis functions in `v`.
//! - [`NurbsSurface::derivatives`] computes the derivatives
//!   `Aw^(k,l)` of the homogeneous surface with A3.6 (on top of A2.3 in
//!   each direction), then those of the projected surface with A4.4, the
//!   two-variable Leibniz rule applied to `A = w S` (eq. 4.20): `w S^(k,l)`
//!   is `A^(k,l)` minus the sum of `binom(k,i) binom(l,j) w^(i,j)
//!   S^(k-i,l-j)` over all `0 <= i <= k`, `0 <= j <= l` other than
//!   `i = j = 0`.
//! - [`NurbsSurface::normal`] is the unit vector along `S_u x S_v`.
//! - [`NurbsSurface::principal_curvatures`] solves the eigenproblem of the
//!   second fundamental form `II = (L, M, N)` (second partials dotted with
//!   the normal) relative to the first, `I = (E, F, G)` (dot products of
//!   the first partials), as in classical differential geometry: the principal
//!   curvatures are the roots of `(EG - F^2) k^2 - (EN - 2FM + GL) k +
//!   (LN - M^2) = 0`, and the principal directions are the null vectors of
//!   `II - k I`, mapped to model space through `S_u` and `S_v`.
//!
//! # Representation changes
//!
//! Knot insertion and refinement, degree elevation, Bezier decomposition,
//! splitting, reversal, isocurves and transforms live in the submodules;
//! each one documents the algorithm it uses.

use bevel_math::{Aabb3, DVec3, DVec4, Point3, Vector3};

use crate::basis::{basis_funs_raw, ders_basis_funs_raw};
use crate::knots::check_degree;
use crate::{GeomError, Homogeneous, KnotVector, MAX_DEGREE, MAX_DERIVATIVE_ORDER};

/// Relative difference between weights above which a surface counts as
/// rational (as for curves).
const RATIONAL_WEIGHT_TOLERANCE: f64 = 1e-14;

/// Relative tolerance on `|S_u x S_v| / (|S_u| |S_v|)` (the sine of the
/// angle between the partials) below which the surface has no normal.
const NORMAL_SINE_TOLERANCE: f64 = 1e-12;

/// Relative tolerance on the difference of the principal curvatures below
/// which a point counts as umbilic (every direction is principal).
const UMBILIC_TOLERANCE: f64 = 1e-10;

/// A non-uniform rational B-spline surface in 3D model space.
///
/// See the module documentation for the layout of the control net.
#[derive(Clone, Debug, PartialEq)]
#[must_use]
pub struct NurbsSurface {
    /// Invariant: `1 <= degree_u <= MAX_DEGREE`.
    degree_u: usize,
    /// Invariant: `1 <= degree_v <= MAX_DEGREE`.
    degree_v: usize,
    /// Invariant: valid for `degree_u`.
    knots_u: KnotVector,
    /// Invariant: valid for `degree_v`.
    knots_v: KnotVector,
    /// Invariant: `count_u * count_v` finite points with strictly positive
    /// weights, row-major (`i * count_v + j`), where `count_u =
    /// knots_u.len() - degree_u - 1` and likewise for `v`.
    control: Vec<DVec4>,
    /// Cached: some weight differs from the first.
    rational: bool,
}

/// The principal curvatures and directions of a surface at a point.
///
/// Curvatures are signed relative to [`NurbsSurface::normal`]: positive
/// where the surface bends towards the normal (the normal curvature
/// `II(t, t) / I(t, t)` with `II` measured along the normal). For a sphere
/// whose normal points outwards both curvatures are `-1 / r`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Curvatures {
    /// The larger principal curvature.
    pub k1: f64,
    /// The smaller principal curvature (`k2 <= k1`).
    pub k2: f64,
    /// Unit direction of `k1`, tangent to the surface.
    pub dir1: Vector3,
    /// Unit direction of `k2`: `normal x dir1`, so that `dir1`, `dir2` and
    /// the normal form a right-handed orthonormal frame.
    pub dir2: Vector3,
}

/// `binom(n, k)` as a float, for the small orders used in A4.4.
fn binomial(n: usize, k: usize) -> f64 {
    let k = k.min(n.saturating_sub(k));
    (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64)
}

impl NurbsSurface {
    /// A surface from its degrees, knot vectors and homogeneous control
    /// points `(w P, w)` in row-major order (see the module documentation).
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidDegree`] or [`GeomError::InvalidKnotVector`] if a
    /// knot vector is not valid for its degree,
    /// [`GeomError::ControlPointCount`] unless there are `count_u * count_v`
    /// control points with `count_u = knots_u.len() - degree_u - 1` (and
    /// likewise for `v`), [`GeomError::InvalidWeight`] for a weight that is
    /// not finite and positive, and [`GeomError::NonFinite`] for a
    /// non-finite coordinate.
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        knots_u: KnotVector,
        knots_v: KnotVector,
        control_weighted: Vec<DVec4>,
    ) -> Result<Self, GeomError> {
        check_degree(degree_u)?;
        check_degree(degree_v)?;
        knots_u.validate(degree_u)?;
        knots_v.validate(degree_v)?;
        let count_u = knots_u.as_slice().len() - degree_u - 1;
        let count_v = knots_v.as_slice().len() - degree_v - 1;
        let expected = count_u * count_v;
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
        Ok(Self::from_parts(
            degree_u,
            degree_v,
            knots_u,
            knots_v,
            control_weighted,
        ))
    }

    /// A surface from model-space control points (row-major) and optional
    /// weights; without weights the surface is non-rational (all weights 1).
    ///
    /// # Errors
    ///
    /// As [`NurbsSurface::new`], with [`GeomError::ControlPointCount`] also
    /// when `weights` and `points` differ in length.
    pub fn from_points(
        degree_u: usize,
        degree_v: usize,
        knots_u: KnotVector,
        knots_v: KnotVector,
        points: &[Point3],
        weights: Option<&[f64]>,
    ) -> Result<Self, GeomError> {
        let control = match weights {
            None => points.iter().map(|&p| DVec4::from_point(p, 1.0)).collect(),
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
                    .map(|(&p, &w)| DVec4::from_point(p, w))
                    .collect()
            }
        };
        Self::new(degree_u, degree_v, knots_u, knots_v, control)
    }

    /// Assembles a surface from parts that already satisfy the type
    /// invariants.
    pub(crate) fn from_parts(
        degree_u: usize,
        degree_v: usize,
        knots_u: KnotVector,
        knots_v: KnotVector,
        control: Vec<DVec4>,
    ) -> Self {
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
            degree_u,
            degree_v,
            knots_u,
            knots_v,
            control,
            rational,
        }
    }

    /// The degree in `u`.
    pub fn degree_u(&self) -> usize {
        self.degree_u
    }

    /// The degree in `v`.
    pub fn degree_v(&self) -> usize {
        self.degree_v
    }

    /// The knot vector in `u`.
    pub fn knots_u(&self) -> &KnotVector {
        &self.knots_u
    }

    /// The knot vector in `v`.
    pub fn knots_v(&self) -> &KnotVector {
        &self.knots_v
    }

    /// The number of rows of control points (along `u`).
    pub fn count_u(&self) -> usize {
        self.knots_u.as_slice().len() - self.degree_u - 1
    }

    /// The number of control points in each row (along `v`).
    pub fn count_v(&self) -> usize {
        self.knots_v.as_slice().len() - self.degree_v - 1
    }

    /// The model-space control points, row-major.
    pub fn control_points(&self) -> Vec<Point3> {
        self.control.iter().map(|c| c.project()).collect()
    }

    /// The weights of the control points, row-major.
    pub fn weights(&self) -> Vec<f64> {
        self.control.iter().map(|c| c.weight()).collect()
    }

    /// The homogeneous control points `(w P, w)`, row-major.
    pub fn control_weighted(&self) -> &[DVec4] {
        &self.control
    }

    /// Returns `true` if some weight differs from the first by more than
    /// 1e-14 relative; such a surface is genuinely rational.
    pub fn is_rational(&self) -> bool {
        self.rational
    }

    /// The parameter domain `((u_min, u_max), (v_min, v_max))`.
    pub fn domain(&self) -> ((f64, f64), (f64, f64)) {
        (
            self.knots_u.domain(self.degree_u),
            self.knots_v.domain(self.degree_v),
        )
    }

    /// `(u, v)` clamped to the domain; NaN stays NaN.
    pub(crate) fn clamp_params(&self, u: f64, v: f64) -> (f64, f64) {
        let ((ua, ub), (va, vb)) = self.domain();
        let c = |t: f64, a: f64, b: f64| if t.is_nan() { t } else { t.clamp(a, b) };
        (c(u, ua, ub), c(v, va, vb))
    }

    /// The knot spans of `(u, v)` (A2.1 in each direction).
    pub(crate) fn spans(&self, u: f64, v: f64) -> (usize, usize) {
        (
            self.knots_u.span(self.degree_u, u),
            self.knots_v.span(self.degree_v, v),
        )
    }

    /// The largest distance of a control point from the origin, but at
    /// least 1: the length scale for relative tolerances.
    pub(crate) fn size(&self) -> f64 {
        self.control
            .iter()
            .map(|c| c.project().length())
            .fold(1.0, f64::max)
    }

    /// The axis-aligned box of the control points. Because the weights are
    /// positive the surface lies in their convex hull, so the box contains
    /// the whole surface.
    pub fn control_bounds(&self) -> Aabb3 {
        Aabb3::from_points(self.control.iter().map(|c| c.project()))
    }

    /// The point at `(u, v)` (The NURBS Book, A4.3; A3.5 when
    /// non-rational).
    ///
    /// Parameters outside the domain are clamped to it; NaN gives a NaN
    /// point.
    pub fn point(&self, u: f64, v: f64) -> Point3 {
        let (u, v) = self.clamp_params(u, v);
        let (su, sv) = self.spans(u, v);
        self.point_in_spans(u, v, su, sv)
    }

    /// [`NurbsSurface::point`] using the polynomial piece of the given knot
    /// spans (which must be non-empty spans from [`KnotVector::span`]).
    pub(crate) fn point_in_spans(&self, u: f64, v: f64, su: usize, sv: usize) -> Point3 {
        let (p, q) = (self.degree_u, self.degree_v);
        let nu = basis_funs_raw(self.knots_u.as_slice(), su, u, p);
        let nv = basis_funs_raw(self.knots_v.as_slice(), sv, v, q);
        self.combine(su, sv, &nu, &nv).project()
    }

    /// `sum_r sum_s nu[r] nv[s] Pw[su - p + r][sv - q + s]`: the homogeneous
    /// point for the given basis function values.
    pub(crate) fn combine(&self, su: usize, sv: usize, nu: &[f64], nv: &[f64]) -> DVec4 {
        let (p, q) = (self.degree_u, self.degree_v);
        let count_v = self.count_v();
        let mut sw = DVec4::ZERO;
        for (r, &bu) in nu.iter().enumerate().take(p + 1) {
            let row = (su - p + r) * count_v + sv - q;
            let temp = self.control[row..=row + q]
                .iter()
                .zip(nv)
                .fold(DVec4::ZERO, |acc, (&c, &bv)| acc + c * bv);
            sw += temp * bu;
        }
        sw
    }

    /// The point and the partial derivatives at `(u, v)` up to total order
    /// `d` (The NURBS Book, A3.6 and A4.4): `result[k][l]` is
    /// `d^(k+l) S / du^k dv^l` for `k + l <= d`, so `result[k]` has
    /// `d - k + 1` entries and `result[0][0]` is [`NurbsSurface::point`].
    ///
    /// Parameters outside the domain are clamped to it. At a knot the
    /// derivatives are those of the patch that starts there (the last patch
    /// at the end of the domain). Orders above [`MAX_DERIVATIVE_ORDER`] are
    /// not computed: `d` is capped at it.
    pub fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Point3>> {
        let d = d.min(MAX_DERIVATIVE_ORDER);
        let (u, v) = self.clamp_params(u, v);
        let (su, sv) = self.spans(u, v);
        let w = d + 1;
        let mut aders = vec![DVec4::ZERO; w * w];
        let mut skl = vec![DVec3::ZERO; w * w];
        self.derivatives_in_spans(u, v, su, sv, d, &mut aders, &mut skl);
        (0..=d)
            .map(|k| skl[k * w..k * w + (d - k + 1)].to_vec())
            .collect()
    }

    /// The point and derivatives up to order 2 at `(u, v)` in the given
    /// spans, as a 3x3 table `[k][l]` (entries with `k + l > 2` are zero).
    pub(crate) fn derivatives2_in_spans(
        &self,
        u: f64,
        v: f64,
        su: usize,
        sv: usize,
    ) -> [[DVec3; 3]; 3] {
        let mut aders = [DVec4::ZERO; 9];
        let mut skl = [DVec3::ZERO; 9];
        self.derivatives_in_spans(u, v, su, sv, 2, &mut aders, &mut skl);
        [
            [skl[0], skl[1], skl[2]],
            [skl[3], skl[4], skl[5]],
            [skl[6], skl[7], skl[8]],
        ]
    }

    /// A3.6 and A4.4 at `(u, v)` with the polynomial pieces of the given
    /// non-empty spans. `aders` and `skl` are `(d + 1) x (d + 1)` row-major
    /// work and output tables; `skl[k * (d + 1) + l]` receives
    /// `S^(k,l)` for `k + l <= d` and zero elsewhere.
    #[allow(clippy::too_many_arguments)]
    fn derivatives_in_spans(
        &self,
        u: f64,
        v: f64,
        su: usize,
        sv: usize,
        d: usize,
        aders: &mut [DVec4],
        skl: &mut [DVec3],
    ) {
        let (p, q) = (self.degree_u, self.degree_v);
        let w = d + 1;
        let du = d.min(p);
        let dv = d.min(q);
        let nu = ders_basis_funs_raw(self.knots_u.as_slice(), su, u, p, du);
        let nv = ders_basis_funs_raw(self.knots_v.as_slice(), sv, v, q, dv);
        let count_v = self.count_v();
        aders.fill(DVec4::ZERO);
        skl.fill(DVec3::ZERO);
        // A3.6: derivatives of the homogeneous surface. Those with k > p or
        // l > q vanish.
        let mut temp = [DVec4::ZERO; MAX_DEGREE + 1];
        for k in 0..=du {
            for (s, t) in temp.iter_mut().enumerate().take(q + 1) {
                *t = (0..=p).fold(DVec4::ZERO, |acc, r| {
                    acc + self.control[(su - p + r) * count_v + sv - q + s] * nu[k][r]
                });
            }
            for l in 0..=(d - k).min(dv) {
                aders[k * w + l] = temp[..=q]
                    .iter()
                    .zip(&nv[l])
                    .fold(DVec4::ZERO, |acc, (&t, &b)| acc + t * b);
            }
        }
        // A4.4: derivatives of the projected surface.
        let inv_w = 1.0 / aders[0].w;
        for k in 0..=d {
            for l in 0..=d - k {
                let mut val = aders[k * w + l].truncate();
                if self.rational {
                    for j in 1..=l {
                        val -= skl[k * w + l - j] * (binomial(l, j) * aders[j].w);
                    }
                    for i in 1..=k {
                        let bki = binomial(k, i);
                        val -= skl[(k - i) * w + l] * (bki * aders[i * w].w);
                        let mut v2 = DVec3::ZERO;
                        for j in 1..=l {
                            v2 += skl[(k - i) * w + l - j] * (binomial(l, j) * aders[i * w + j].w);
                        }
                        val -= v2 * bki;
                    }
                }
                skl[k * w + l] = val * inv_w;
            }
        }
    }

    /// Partial derivatives below these lengths count as vanishing: moving
    /// at them across the whole domain covers 1e-12 of the surface's size.
    pub(crate) fn speed_floors(&self) -> (f64, f64) {
        let ((ua, ub), (va, vb)) = self.domain();
        let s = 1e-12 * self.size();
        (s / (ub - ua), s / (vb - va))
    }

    /// The unit normal from first partials `su`, `sv`, or `None` where they
    /// vanish or are parallel.
    pub(crate) fn normal_from(&self, su: Vector3, sv: Vector3) -> Option<Vector3> {
        let (fu, fv) = self.speed_floors();
        let (lu, lv) = (su.length(), sv.length());
        if lu <= fu || lv <= fv {
            return None;
        }
        let n = su.cross(sv);
        let ln = n.length();
        (ln > NORMAL_SINE_TOLERANCE * lu * lv).then(|| n / ln)
    }

    /// The unit normal `S_u x S_v / |S_u x S_v|` at `(u, v)`, or `None`
    /// where a partial derivative vanishes or the two are parallel (see
    /// [`NurbsSurface::derivatives`] for parameters at knots).
    pub fn normal(&self, u: f64, v: f64) -> Option<Vector3> {
        let (u, v) = self.clamp_params(u, v);
        let (su, sv) = self.spans(u, v);
        let mut aders = [DVec4::ZERO; 4];
        let mut skl = [DVec3::ZERO; 4];
        self.derivatives_in_spans(u, v, su, sv, 1, &mut aders, &mut skl);
        self.normal_from(skl[2], skl[1])
    }

    /// The principal curvatures and directions at `(u, v)`, or `None` where
    /// there is no normal (see [`Curvatures`] for the sign convention and
    /// the module documentation for the method).
    ///
    /// At an umbilic point (equal curvatures, as everywhere on a sphere or a
    /// plane) every tangent direction is principal; `dir1` is then the
    /// direction of `S_u`.
    pub fn principal_curvatures(&self, u: f64, v: f64) -> Option<Curvatures> {
        let (u, v) = self.clamp_params(u, v);
        let (su, sv) = self.spans(u, v);
        let d = self.derivatives2_in_spans(u, v, su, sv);
        let (s_u, s_v) = (d[1][0], d[0][1]);
        let n = self.normal_from(s_u, s_v)?;
        let (e, f, g) = (s_u.dot(s_u), s_u.dot(s_v), s_v.dot(s_v));
        let (l, m, nn) = (d[2][0].dot(n), d[1][1].dot(n), d[0][2].dot(n));
        let det = e * g - f * f;
        if det.is_nan() || det <= 0.0 {
            return None;
        }
        let mean = (e * nn - 2.0 * f * m + g * l) / (2.0 * det);
        let gauss = (l * nn - m * m) / det;
        let disc = (mean * mean - gauss).max(0.0).sqrt();
        let (k1, k2) = (mean + disc, mean - disc);
        let scale = k1.abs().max(k2.abs());
        let dir1 = if disc <= UMBILIC_TOLERANCE * scale {
            Some(s_u / s_u.length())
        } else {
            principal_direction(k1, (e, f, g), (l, m, nn), s_u, s_v)
        }?;
        // Remove any normal component left by rounding.
        let dir1 = (dir1 - n * dir1.dot(n)).try_normalize()?;
        Some(Curvatures {
            k1,
            k2,
            dir1,
            dir2: n.cross(dir1),
        })
    }
}

/// The model-space direction of the null vector of `II - k I` for the
/// principal curvature `k`: of the two rows `(L - kE, M - kF)` and
/// `(M - kF, N - kG)`, the longer one gives the more accurate null vector.
fn principal_direction(
    k: f64,
    (e, f, g): (f64, f64, f64),
    (l, m, n): (f64, f64, f64),
    s_u: Vector3,
    s_v: Vector3,
) -> Option<Vector3> {
    let (a, b, c) = (l - k * e, m - k * f, n - k * g);
    // (du, dv) orthogonal to the longer row.
    let (du, dv) = if a * a + b * b >= b * b + c * c {
        (-b, a)
    } else {
        (-c, b)
    };
    (s_u * du + s_v * dv).try_normalize()
}

#[cfg(test)]
mod tests {
    use bevel_math::DVec3;

    use super::*;

    /// A biquadratic patch of the paraboloid `z = x^2 + y^2` over
    /// `[-1, 1]^2` (exact: the paraboloid is quadratic in x and y).
    fn paraboloid() -> NurbsSurface {
        let kv = KnotVector::clamped_from_interior(2, -1.0, 1.0, &[]).unwrap();
        let mut pts = Vec::new();
        for x in [-1.0, 0.0, 1.0] {
            for y in [-1.0, 0.0, 1.0] {
                // Control net of x^2 on [-1, 1]: 1, -1, 1.
                let zx: f64 = if x == 0.0 { -1.0 } else { 1.0 };
                let zy: f64 = if y == 0.0 { -1.0 } else { 1.0 };
                pts.push(DVec3::new(x, y, zx + zy));
            }
        }
        NurbsSurface::from_points(2, 2, kv.clone(), kv, &pts, None).unwrap()
    }

    #[test]
    fn paraboloid_points_and_curvatures() {
        let s = paraboloid();
        assert_eq!((s.count_u(), s.count_v()), (3, 3));
        assert!(!s.is_rational());
        for (x, y) in [(0.0, 0.0), (0.3, -0.4), (-1.0, 1.0), (0.9, 0.2)] {
            let p = s.point(x, y);
            assert!((p - DVec3::new(x, y, x * x + y * y)).length() < 1e-14);
            let d = s.derivatives(x, y, 2);
            assert_eq!(d.len(), 3);
            assert_eq!(d[0].len(), 3);
            assert_eq!(d[2].len(), 1);
            assert!((d[1][0] - DVec3::new(1.0, 0.0, 2.0 * x)).length() < 1e-13);
            assert!((d[0][1] - DVec3::new(0.0, 1.0, 2.0 * y)).length() < 1e-13);
            assert!((d[2][0] - DVec3::new(0.0, 0.0, 2.0)).length() < 1e-13);
            assert!(d[1][1].length() < 1e-13);
        }
        // At the apex both curvatures are 2, towards the upward normal.
        let n = s.normal(0.0, 0.0).unwrap();
        assert!((n - DVec3::Z).length() < 1e-14);
        let c = s.principal_curvatures(0.0, 0.0).unwrap();
        assert!((c.k1 - 2.0).abs() < 1e-12 && (c.k2 - 2.0).abs() < 1e-12);
        // Elsewhere: k = 2 / (1 + 4 r^2)^(3/2) along the meridian and
        // 2 / (1 + 4 r^2)^(1/2) along the parallel.
        let (x, y) = (0.6, 0.0);
        let c = s.principal_curvatures(x, y).unwrap();
        let q = 1.0 + 4.0 * x * x;
        assert!((c.k1 - 2.0 / q.sqrt()).abs() < 1e-12, "{c:?}");
        assert!((c.k2 - 2.0 / q.powf(1.5)).abs() < 1e-12, "{c:?}");
        assert!(c.dir1.y.abs() > 0.999_999);
        let n = s.normal(x, y).unwrap();
        assert!(c.dir1.dot(n).abs() < 1e-14 && c.dir2.dot(n).abs() < 1e-14);
        assert!((c.dir1.cross(c.dir2) - n).length() < 1e-14);
    }

    #[test]
    fn derivatives_are_capped_and_triangular() {
        let s = paraboloid();
        let d = s.derivatives(0.1, 0.2, usize::MAX);
        assert_eq!(d.len(), MAX_DERIVATIVE_ORDER + 1);
        for (k, row) in d.iter().enumerate() {
            assert_eq!(row.len(), MAX_DERIVATIVE_ORDER + 1 - k);
        }
        assert!(d[3][0].length() < 1e-12);
        let d0 = s.derivatives(0.1, 0.2, 0);
        assert_eq!(d0.len(), 1);
        assert!((d0[0][0] - s.point(0.1, 0.2)).length() < 1e-15);
        assert!(s.point(f64::NAN, 0.0).is_nan());
    }

    #[test]
    fn degenerate_points_have_no_normal() {
        // A patch whose v = 0 row collapses to a point.
        let kv = KnotVector::clamped_from_interior(1, 0.0, 1.0, &[]).unwrap();
        let pts = [
            DVec3::ZERO,
            DVec3::X,
            DVec3::ZERO,
            DVec3::new(1.0, 1.0, 0.0),
        ];
        let s = NurbsSurface::from_points(1, 1, kv.clone(), kv, &pts, None).unwrap();
        assert!(s.normal(0.5, 0.0).is_none());
        assert!(s.principal_curvatures(0.5, 0.0).is_none());
        // S(u, v) = (v, u v, 0), so S_u x S_v = (0, 0, -v).
        let n = s.normal(0.5, 0.5).unwrap();
        assert!((n + DVec3::Z).length() < 1e-14);
        let c = s.principal_curvatures(0.5, 0.5).unwrap();
        assert!(c.k1.abs() < 1e-12 && c.k2.abs() < 1e-12);
    }

    #[test]
    fn invalid_inputs_are_rejected() {
        let k2 = KnotVector::clamped_uniform(2, 3).unwrap();
        let k1 = KnotVector::clamped_uniform(1, 2).unwrap();
        let pts = vec![DVec3::ONE; 6];
        assert!(NurbsSurface::from_points(2, 1, k2.clone(), k1.clone(), &pts, None).is_ok());
        assert!(matches!(
            NurbsSurface::from_points(2, 1, k2.clone(), k1.clone(), &pts[..5], None),
            Err(GeomError::ControlPointCount {
                expected: 6,
                found: 5
            })
        ));
        assert!(matches!(
            NurbsSurface::from_points(1, 1, k2.clone(), k1.clone(), &pts, None),
            Err(GeomError::ControlPointCount { .. } | GeomError::InvalidKnotVector(_))
        ));
        assert!(matches!(
            NurbsSurface::from_points(2, 0, k2.clone(), k1.clone(), &pts, None),
            Err(GeomError::InvalidDegree { degree: 0 })
        ));
        let w = [1.0, 1.0, 1.0, 0.0, 1.0, 1.0];
        assert!(matches!(
            NurbsSurface::from_points(2, 1, k2.clone(), k1.clone(), &pts, Some(&w)),
            Err(GeomError::InvalidWeight { index: 3 })
        ));
        assert!(matches!(
            NurbsSurface::from_points(2, 1, k2.clone(), k1.clone(), &pts, Some(&w[..2])),
            Err(GeomError::ControlPointCount { .. })
        ));
        let mut bad = pts.clone();
        bad[4].y = f64::INFINITY;
        assert!(matches!(
            NurbsSurface::from_points(2, 1, k2.clone(), k1.clone(), &bad, None),
            Err(GeomError::NonFinite)
        ));
        let control = vec![DVec4::new(1.0, 1.0, 1.0, -1.0); 6];
        assert!(matches!(
            NurbsSurface::new(2, 1, k2, k1, control),
            Err(GeomError::InvalidWeight { index: 0 })
        ));
    }

    #[test]
    fn control_bounds_contain_the_surface() {
        let s = paraboloid();
        let b = s.control_bounds();
        assert_eq!(b.min, DVec3::new(-1.0, -1.0, -2.0));
        assert_eq!(b.max, DVec3::new(1.0, 1.0, 2.0));
    }
}
