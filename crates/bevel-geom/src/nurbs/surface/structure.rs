// SPDX-License-Identifier: GPL-3.0-or-later

//! Bezier decomposition, splitting, reversal, isocurves, transforms and
//! closedness.
//!
//! - [`NurbsSurface::decompose_bezier`] is A5.7 (`DecomposeSurface`): the
//!   curve decomposition A5.6 applied to every column splits the surface
//!   into strips that are Bezier in `u`, and applied to every row of each
//!   strip, into patches that are Bezier in both directions.
//! - [`NurbsSurface::split_u`] splits every column with the curve's split
//!   (knot insertion to multiplicity `p`, A5.1), giving two surfaces that
//!   share the boundary row at the split.
//! - [`NurbsSurface::reversed_u`] reverses the order of the rows and maps the
//!   knots in `u` by `u -> a + b - u` (eq. 6.13).
//! - [`NurbsSurface::isocurve_u`] evaluates the `u` basis functions at the
//!   fixed parameter and applies them to every column of homogeneous
//!   points (the first half of A4.3), which gives the control points of the
//!   exact curve along `v` (The NURBS Book, section 4.4: isoparametric
//!   curves are NURBS curves on the other knot vector).
//! - [`NurbsSurface::transformed`] applies a matrix to the homogeneous
//!   control points, as for curves (section 4.4).

use bevel_math::{DMat4, DVec4};

use crate::basis::basis_funs_raw;
use crate::{GeomError, Homogeneous, KnotVector, NurbsCurve3, NurbsSurface};

/// Relative tolerance on the ratio of corresponding weights of two
/// boundary curves for them to count as the same curve.
const CLOSED_WEIGHT_TOLERANCE: f64 = 1e-12;

impl NurbsSurface {
    /// Applies a curve operation that yields several curves (one per piece)
    /// to every column and assembles one surface per piece, in order.
    fn map_u_pieces(
        &self,
        f: impl Fn(&NurbsCurve3) -> Result<Vec<NurbsCurve3>, GeomError>,
    ) -> Result<Vec<Self>, GeomError> {
        let pieces = self
            .columns()
            .iter()
            .map(f)
            .collect::<Result<Vec<_>, _>>()?;
        let count = pieces.first().map_or(0, Vec::len);
        if pieces.iter().any(|p| p.len() != count) {
            return Err(GeomError::InvalidKnotVector(
                "the curves of a surface along one direction must share their knot vector",
            ));
        }
        (0..count)
            .map(|k| {
                let columns: Vec<NurbsCurve3> = pieces.iter().map(|p| p[k].clone()).collect();
                Self::from_columns(&columns, self.degree_v, self.knots_v.clone())
            })
            .collect()
    }

    /// The Bezier patches of the surface (The NURBS Book, A5.7):
    /// `result[i][j]` is the patch on the `i`-th non-empty knot span in `u`
    /// and the `j`-th in `v`, with degrees `(p, q)`, `(p + 1) (q + 1)`
    /// control points and that span as its domain; it equals the surface
    /// there.
    ///
    /// The decomposition cannot fail for a valid surface; the result is
    /// only empty if an internal consistency check ever did.
    pub fn decompose_bezier(&self) -> Vec<Vec<Self>> {
        self.try_decompose_bezier().unwrap_or_default()
    }

    /// [`NurbsSurface::decompose_bezier`] with its consistency checks
    /// reported.
    pub(crate) fn try_decompose_bezier(&self) -> Result<Vec<Vec<Self>>, GeomError> {
        let strips = self.map_u_pieces(|c| Ok(c.decompose_bezier()))?;
        strips
            .iter()
            .map(|strip| {
                let patches = strip
                    .swapped_uv()
                    .map_u_pieces(|c| Ok(c.decompose_bezier()))?;
                Ok(patches.iter().map(Self::swapped_uv).collect())
            })
            .collect()
    }

    /// Splits the surface at `u = t` into the part on `[a, t]` and the part
    /// on `[t, b]` in `u`, where `[a, b]` is the domain in `u`. Both parts
    /// are clamped in `u` and keep the original parametrisation.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `t` and
    /// [`GeomError::ParameterOutOfDomain`] unless `t` lies strictly inside
    /// the domain in `u`.
    pub fn split_u(&self, t: f64) -> Result<(Self, Self), GeomError> {
        let pieces = self.map_u_pieces(|c| c.split(t).map(|(l, r)| vec![l, r]))?;
        let mut pieces = pieces.into_iter();
        match (pieces.next(), pieces.next()) {
            (Some(l), Some(r)) => Ok((l, r)),
            _ => Err(GeomError::ParameterOutOfDomain { t }),
        }
    }

    /// Splits the surface at `v = t` into the part on `[c, t]` and the part
    /// on `[t, d]` in `v`, where `[c, d]` is the domain in `v`.
    ///
    /// # Errors
    ///
    /// As [`NurbsSurface::split_u`], for the domain in `v`.
    pub fn split_v(&self, t: f64) -> Result<(Self, Self), GeomError> {
        let (l, r) = self.swapped_uv().split_u(t)?;
        Ok((l.swapped_uv(), r.swapped_uv()))
    }

    /// The same surface with `u` running backwards: the new surface at
    /// `(a + b - u, v)` is the original at `(u, v)`, where `[a, b]` is the
    /// domain in `u`. This reverses the orientation of the normal.
    pub fn reversed_u(&self) -> Self {
        let (nu, nv) = (self.count_u(), self.count_v());
        let control = (0..nu)
            .rev()
            .flat_map(|i| self.control[i * nv..(i + 1) * nv].iter().copied())
            .collect();
        Self::from_parts(
            self.degree_u,
            self.degree_v,
            self.knots_u.reversed(self.degree_u),
            self.knots_v.clone(),
            control,
        )
    }

    /// The same surface with `v` running backwards: the new surface at
    /// `(u, c + d - v)` is the original at `(u, v)`, where `[c, d]` is the
    /// domain in `v`. This reverses the orientation of the normal.
    pub fn reversed_v(&self) -> Self {
        let nv = self.count_v();
        let control = self
            .control
            .chunks_exact(nv)
            .flat_map(|row| row.iter().rev().copied())
            .collect();
        Self::from_parts(
            self.degree_u,
            self.degree_v,
            self.knots_u.clone(),
            self.knots_v.reversed(self.degree_v),
            control,
        )
    }

    /// The isoparametric curve at fixed `u`, running along `v`: the curve
    /// at `v` is the surface at `(u, v)`. It has the degree and knots of the
    /// surface in `v`. A parameter outside the domain is clamped to it.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `u`.
    pub fn isocurve_u(&self, u: f64) -> Result<NurbsCurve3, GeomError> {
        if !u.is_finite() {
            return Err(GeomError::NonFinite);
        }
        let ((a, b), _) = self.domain();
        let u = u.clamp(a, b);
        let p = self.degree_u;
        let span = self.knots_u.span(p, u);
        let n = basis_funs_raw(self.knots_u.as_slice(), span, u, p);
        let nv = self.count_v();
        let control: Vec<DVec4> = (0..nv)
            .map(|j| {
                (0..=p).fold(DVec4::ZERO, |acc, r| {
                    acc + self.control[(span - p + r) * nv + j] * n[r]
                })
            })
            .collect();
        Ok(NurbsCurve3::from_parts(
            self.degree_v,
            self.knots_v.clone(),
            control,
        ))
    }

    /// The isoparametric curve at fixed `v`, running along `u`: the curve
    /// at `u` is the surface at `(u, v)`. It has the degree and knots of the
    /// surface in `u`. A parameter outside the domain is clamped to it.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `v`.
    pub fn isocurve_v(&self, v: f64) -> Result<NurbsCurve3, GeomError> {
        self.swapped_uv().isocurve_u(v)
    }

    /// The surface under the transform `m` of model space, applied to the
    /// homogeneous control points. For an affine `m` this moves the control
    /// points and keeps the weights, which transforms the surface exactly;
    /// a projective `m` is applied exactly too, as long as all new weights
    /// stay positive.
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidWeight`] if a weight becomes zero or negative and
    /// [`GeomError::NonFinite`] for a non-finite matrix.
    pub fn transformed(&self, m: &DMat4) -> Result<Self, GeomError> {
        let control = self.control.iter().map(|&c| *m * c).collect();
        Self::new(
            self.degree_u,
            self.degree_v,
            self.knots_u.clone(),
            self.knots_v.clone(),
            control,
        )
    }

    /// Returns `true` if the surface is closed in `u`: its boundary curves
    /// at the two ends of the domain in `u` are the same curve. They are,
    /// when their control points agree to within `tol` (model units) and
    /// their weights are proportional (to 1e-12); for a clamped knot vector
    /// those curves are the first and last rows of the control net.
    ///
    /// A negative or NaN `tol` gives `false`.
    pub fn is_closed_u(&self, tol: f64) -> bool {
        let ((a, b), _) = self.domain();
        match (self.isocurve_u(a), self.isocurve_u(b)) {
            (Ok(first), Ok(last)) => same_control(&first, &last, tol),
            _ => false,
        }
    }

    /// Returns `true` if the surface is closed in `v`: its boundary curves
    /// at the two ends of the domain in `v` are the same curve (see
    /// [`NurbsSurface::is_closed_u`]).
    pub fn is_closed_v(&self, tol: f64) -> bool {
        self.swapped_uv().is_closed_u(tol)
    }
}

/// Returns `true` if two curves on the same knot vector have control points
/// within `tol` of each other and proportional weights.
fn same_control(a: &NurbsCurve3, b: &NurbsCurve3, tol: f64) -> bool {
    if tol.is_nan() || tol < 0.0 || !same_knots(a.knots(), b.knots()) {
        return false;
    }
    let (ca, cb) = (a.control_weighted(), b.control_weighted());
    let (Some(a0), Some(b0)) = (ca.first(), cb.first()) else {
        return false;
    };
    let ratio = a0.weight() / b0.weight();
    ca.len() == cb.len()
        && ca.iter().zip(cb).all(|(&x, &y)| {
            (x.project() - y.project()).length() <= tol
                && (x.weight() - ratio * y.weight()).abs() <= CLOSED_WEIGHT_TOLERANCE * x.weight()
        })
}

/// Returns `true` if two knot vectors have the same knots.
fn same_knots(a: &KnotVector, b: &KnotVector) -> bool {
    a.as_slice().len() == b.as_slice().len()
        && a.as_slice()
            .iter()
            .zip(b.as_slice())
            .all(|(&x, &y)| a.same(x, y))
}

#[cfg(test)]
mod tests {
    use bevel_math::{DMat4, DQuat, DVec3};

    use crate::{GeomError, KnotVector, NurbsSurface};

    /// A bicubic-by-quadratic tube-like surface whose first and last rows
    /// coincide when `closed`.
    fn net(closed: bool) -> NurbsSurface {
        let ku = KnotVector::clamped_from_interior(3, 0.0, 1.0, &[0.25, 0.5, 0.5]).unwrap();
        let kv = KnotVector::clamped_from_interior(2, -1.0, 1.0, &[0.0]).unwrap();
        let (nu, nv) = (7, 4);
        let mut pts = Vec::new();
        let mut w = Vec::new();
        for i in 0..nu {
            // When closed, the last row is exactly the first.
            let k = if closed { i % (nu - 1) } else { i };
            let a = std::f64::consts::TAU * f64::from(k) / f64::from(nu - 1);
            let a = if closed { a } else { a * 0.9 };
            for j in 0..nv {
                let r = 2.0 + 0.3 * f64::from(j);
                pts.push(DVec3::new(r * a.cos(), r * a.sin(), f64::from(j)));
                w.push(1.0 + 0.2 * f64::from(j));
            }
        }
        NurbsSurface::from_points(3, 2, ku, kv, &pts, Some(&w)).unwrap()
    }

    #[test]
    fn closedness_compares_the_boundary_curves() {
        let s = net(true);
        assert!(s.is_closed_u(1e-12));
        assert!(!s.is_closed_v(1e-6));
        assert!(!s.is_closed_u(-1.0));
        assert!(!net(false).is_closed_u(1e-6));
        assert!(s.swapped_uv().is_closed_v(1e-12));
        // Scaling all weights of the surface keeps it closed.
        let m = DMat4::from_scale(DVec3::ONE) * 3.0;
        assert!(s.transformed(&m).unwrap().is_closed_u(1e-12));
    }

    #[test]
    fn decomposition_yields_patches_on_every_span_pair() {
        let s = net(false);
        let patches = s.decompose_bezier();
        assert_eq!(patches.len(), 3);
        assert!(patches.iter().all(|row| row.len() == 2));
        assert_eq!(patches[1][0].domain(), ((0.25, 0.5), (-1.0, 0.0)));
        assert_eq!(patches[2][1].control_weighted().len(), 12);
    }

    #[test]
    fn isocurves_and_splits_validate_parameters() {
        let s = net(false);
        assert!(matches!(s.isocurve_u(f64::NAN), Err(GeomError::NonFinite)));
        assert!(matches!(
            s.isocurve_v(f64::INFINITY),
            Err(GeomError::NonFinite)
        ));
        // Outside the domain the isocurve is that of the boundary.
        let c = s.isocurve_u(7.0).unwrap();
        assert!((c.point(0.5) - s.point(1.0, 0.5)).length() < 1e-13);
        assert!(matches!(
            s.split_u(0.0),
            Err(GeomError::ParameterOutOfDomain { .. })
        ));
        assert!(matches!(s.split_v(f64::NAN), Err(GeomError::NonFinite)));
        let (l, r) = s.split_v(0.0).unwrap();
        assert_eq!(l.domain().1, (-1.0, 0.0));
        assert_eq!(r.domain().1, (0.0, 1.0));
    }

    #[test]
    fn transforms_validate_weights() {
        let s = net(false);
        assert!(matches!(
            s.transformed(&DMat4::ZERO),
            Err(GeomError::InvalidWeight { .. })
        ));
        let m = DMat4::from_rotation_translation(
            DQuat::from_rotation_y(0.7),
            DVec3::new(1.0, -2.0, 0.5),
        );
        let t = s.transformed(&m).unwrap();
        assert!((t.point(0.3, 0.2) - m.transform_point3(s.point(0.3, 0.2))).length() < 1e-13);
    }
}
