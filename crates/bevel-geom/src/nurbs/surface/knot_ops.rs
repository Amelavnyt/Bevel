// SPDX-License-Identifier: GPL-3.0-or-later

//! Knot insertion, refinement and degree elevation in either parameter
//! direction, and the column machinery they share.
//!
//! For fixed `v`, a tensor-product surface is a curve in `u` whose control
//! points are the curves along `v`; for the representation-changing
//! algorithms this means that an operation in `u` acts on every column of
//! the control net (the points `Pw_0j, ..., Pw_nj` for one `j`) as on the
//! control polygon of a curve with knot vector `U`. The coefficients of
//! those algorithms depend on the knots only, so every column is
//! transformed the same way and the results share their new knot vector.
//! This is how The NURBS Book builds its surface algorithms from the curve
//! ones:
//!
//! - [`NurbsSurface::insert_knot_u`] is A5.3 (`SurfaceKnotIns`), which runs
//!   Boehm's algorithm A5.1 down every column with shared coefficients;
//!   here each column goes through the curve's A5.1 implementation.
//! - [`NurbsSurface::refine_knots_u`] is A5.5 (`RefineKnotVectSurface`), A5.4
//!   on every column.
//! - [`NurbsSurface::elevate_degree_u`] applies the curve's A5.9 to every
//!   column (the row-wise application the book describes in section 5.5,
//!   rather than its fused surface version A5.10); the result is the same.
//!
//! The `v` versions work on the transposed net
//! ([`NurbsSurface::swapped_uv`]). All of them leave the geometry unchanged
//! up to rounding.

use bevel_math::DVec4;

use crate::{GeomError, KnotVector, NurbsCurve3, NurbsSurface};

impl NurbsSurface {
    /// The same surface with `u` and `v` exchanged: the new surface at
    /// `(v, u)` is the original at `(u, v)`. The control net is transposed,
    /// which also reverses the orientation of the normal.
    pub fn swapped_uv(&self) -> Self {
        let (nu, nv) = (self.count_u(), self.count_v());
        let mut control = Vec::with_capacity(self.control.len());
        for j in 0..nv {
            for i in 0..nu {
                control.push(self.control[i * nv + j]);
            }
        }
        Self::from_parts(
            self.degree_v,
            self.degree_u,
            self.knots_v.clone(),
            self.knots_u.clone(),
            control,
        )
    }

    /// The curves along `u` that make up the control net: curve `j` has
    /// degree `degree_u`, the knots in `u` and the control points
    /// `Pw_0j, ..., Pw_nj`.
    pub(crate) fn columns(&self) -> Vec<NurbsCurve3> {
        let (nu, nv) = (self.count_u(), self.count_v());
        (0..nv)
            .map(|j| {
                let control: Vec<DVec4> = (0..nu).map(|i| self.control[i * nv + j]).collect();
                NurbsCurve3::from_parts(self.degree_u, self.knots_u.clone(), control)
            })
            .collect()
    }

    /// The surface whose columns are `columns` (see
    /// [`NurbsSurface::columns`]), with degree and knots `degree_v`,
    /// `knots_v` in `v` (which must fit `columns.len()`).
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidKnotVector`] unless the columns share their
    /// degree and knot vector, and [`GeomError::ControlPointCount`] if there
    /// are not as many columns as `knots_v` needs.
    pub(crate) fn from_columns(
        columns: &[NurbsCurve3],
        degree_v: usize,
        knots_v: KnotVector,
    ) -> Result<Self, GeomError> {
        let expected = knots_v.as_slice().len().saturating_sub(degree_v + 1);
        let Some(first) = columns.first() else {
            return Err(GeomError::ControlPointCount { expected, found: 0 });
        };
        if columns.len() != expected {
            return Err(GeomError::ControlPointCount {
                expected,
                found: columns.len(),
            });
        }
        let (degree_u, knots_u) = (first.degree(), first.knots());
        let same_knots = |k: &KnotVector| {
            k.as_slice().len() == knots_u.as_slice().len()
                && k.as_slice()
                    .iter()
                    .zip(knots_u.as_slice())
                    .all(|(&a, &b)| knots_u.same(a, b))
        };
        if columns
            .iter()
            .any(|c| c.degree() != degree_u || !same_knots(c.knots()))
        {
            return Err(GeomError::InvalidKnotVector(
                "the curves of a surface along one direction must share their knot vector",
            ));
        }
        let nu = first.control_weighted().len();
        let mut control = Vec::with_capacity(nu * columns.len());
        for i in 0..nu {
            for c in columns {
                control.push(c.control_weighted()[i]);
            }
        }
        Ok(Self::from_parts(
            degree_u,
            degree_v,
            knots_u.clone(),
            knots_v,
            control,
        ))
    }

    /// Applies a curve operation to every column (see the module
    /// documentation).
    pub(crate) fn map_u(
        &self,
        f: impl Fn(&NurbsCurve3) -> Result<NurbsCurve3, GeomError>,
    ) -> Result<Self, GeomError> {
        let columns = self
            .columns()
            .iter()
            .map(f)
            .collect::<Result<Vec<_>, _>>()?;
        Self::from_columns(&columns, self.degree_v, self.knots_v.clone())
    }

    /// Applies a curve operation to every row: the curves along `v`.
    pub(crate) fn map_v(
        &self,
        f: impl Fn(&NurbsCurve3) -> Result<NurbsCurve3, GeomError>,
    ) -> Result<Self, GeomError> {
        Ok(self.swapped_uv().map_u(f)?.swapped_uv())
    }

    /// The surface with knot `u` inserted `r` more times into the knot
    /// vector in `u` (The NURBS Book, A5.3). The geometry is unchanged.
    ///
    /// # Errors
    ///
    /// As [`NurbsCurve3::insert_knot`]: [`GeomError::NonFinite`],
    /// [`GeomError::ParameterOutOfDomain`] unless `u` lies strictly inside
    /// the domain, and [`GeomError::InvalidKnotVector`] if the multiplicity
    /// would exceed the degree.
    pub fn insert_knot_u(&self, u: f64, r: usize) -> Result<Self, GeomError> {
        self.map_u(|c| c.insert_knot(u, r))
    }

    /// The surface with knot `v` inserted `r` more times into the knot
    /// vector in `v` (A5.3). The geometry is unchanged.
    ///
    /// # Errors
    ///
    /// As [`NurbsSurface::insert_knot_u`].
    pub fn insert_knot_v(&self, v: f64, r: usize) -> Result<Self, GeomError> {
        self.map_v(|c| c.insert_knot(v, r))
    }

    /// The surface with all of `new_knots` inserted into the knot vector in
    /// `u` (The NURBS Book, A5.5). The knots may come in any order and may
    /// repeat; the geometry is unchanged.
    ///
    /// # Errors
    ///
    /// As [`NurbsSurface::insert_knot_u`] for each knot.
    pub fn refine_knots_u(&self, new_knots: &[f64]) -> Result<Self, GeomError> {
        self.map_u(|c| c.refine_knots(new_knots))
    }

    /// The surface with all of `new_knots` inserted into the knot vector in
    /// `v` (A5.5). The geometry is unchanged.
    ///
    /// # Errors
    ///
    /// As [`NurbsSurface::insert_knot_u`] for each knot.
    pub fn refine_knots_v(&self, new_knots: &[f64]) -> Result<Self, GeomError> {
        self.map_v(|c| c.refine_knots(new_knots))
    }

    /// The same surface with its degree in `u` raised by `t` (A5.9 on every
    /// column; see the module documentation). Every knot in `u` gains
    /// multiplicity `t`, and the knot vector in `u` comes out clamped.
    ///
    /// # Errors
    ///
    /// [`GeomError::InvalidDegree`] if the new degree would exceed
    /// [`MAX_DEGREE`](crate::MAX_DEGREE).
    pub fn elevate_degree_u(&self, t: usize) -> Result<Self, GeomError> {
        self.map_u(|c| c.elevate_degree(t))
    }

    /// The same surface with its degree in `v` raised by `t`. Every knot in
    /// `v` gains multiplicity `t`, and the knot vector in `v` comes out
    /// clamped.
    ///
    /// # Errors
    ///
    /// As [`NurbsSurface::elevate_degree_u`].
    pub fn elevate_degree_v(&self, t: usize) -> Result<Self, GeomError> {
        self.map_v(|c| c.elevate_degree(t))
    }
}

#[cfg(test)]
mod tests {
    use bevel_math::DVec3;

    use crate::{GeomError, KnotVector, NurbsCurve3, NurbsSurface};

    fn saddle() -> NurbsSurface {
        let ku = KnotVector::clamped_from_interior(2, 0.0, 2.0, &[1.0]).unwrap();
        let kv = KnotVector::clamped_from_interior(1, 0.0, 1.0, &[0.5]).unwrap();
        let mut pts = Vec::new();
        for i in 0..4 {
            for j in 0..3 {
                let (x, y) = (f64::from(i), f64::from(j));
                pts.push(DVec3::new(x, y, (x - 1.5) * (y - 1.0)));
            }
        }
        let w: Vec<f64> = (0..12).map(|k| 1.0 + 0.1 * f64::from(k % 5)).collect();
        NurbsSurface::from_points(2, 1, ku, kv, &pts, Some(&w)).unwrap()
    }

    #[test]
    fn swapping_twice_is_the_identity() {
        let s = saddle();
        let t = s.swapped_uv();
        assert_eq!((t.degree_u(), t.degree_v()), (1, 2));
        assert_eq!((t.count_u(), t.count_v()), (3, 4));
        assert!((t.point(0.3, 1.2) - s.point(1.2, 0.3)).length() < 1e-14);
        assert_eq!(t.swapped_uv(), s);
    }

    #[test]
    fn columns_round_trip() {
        let s = saddle();
        let cols = s.columns();
        assert_eq!(cols.len(), 3);
        let t = NurbsSurface::from_columns(&cols, 1, s.knots_v().clone()).unwrap();
        assert_eq!(t, s);
        // Columns with different knots, or the wrong number of them, do not
        // make a surface.
        let mut bad = cols.clone();
        bad[1] = bad[1].insert_knot(0.5, 1).unwrap();
        assert!(matches!(
            NurbsSurface::from_columns(&bad, 1, s.knots_v().clone()),
            Err(GeomError::InvalidKnotVector(_))
        ));
        assert!(matches!(
            NurbsSurface::from_columns(&cols[..2], 1, s.knots_v().clone()),
            Err(GeomError::ControlPointCount { .. })
        ));
        let none: [NurbsCurve3; 0] = [];
        assert!(NurbsSurface::from_columns(&none, 1, s.knots_v().clone()).is_err());
    }

    #[test]
    fn operations_report_curve_errors() {
        let s = saddle();
        assert!(matches!(
            s.insert_knot_u(0.0, 1),
            Err(GeomError::ParameterOutOfDomain { .. })
        ));
        assert!(matches!(
            s.insert_knot_v(0.5, 1),
            Err(GeomError::InvalidKnotVector(_))
        ));
        assert!(matches!(
            s.refine_knots_u(&[f64::NAN]),
            Err(GeomError::NonFinite)
        ));
        assert!(matches!(
            s.elevate_degree_v(usize::MAX),
            Err(GeomError::InvalidDegree { .. })
        ));
        let e = s.elevate_degree_v(2).unwrap();
        assert_eq!((e.degree_u(), e.degree_v()), (2, 3));
        assert_eq!(e.knots_v().multiplicity(0.5), 3);
        assert!((e.point(1.3, 0.7) - s.point(1.3, 0.7)).length() < 1e-13);
        let r = s.refine_knots_v(&[0.25, 0.75, 0.9]).unwrap();
        assert_eq!(r.count_v(), 6);
        assert!((r.point(0.4, 0.8) - s.point(0.4, 0.8)).length() < 1e-13);
    }
}
