// SPDX-License-Identifier: GPL-3.0-or-later

//! Knot insertion, refinement and removal.
//!
//! - Insertion (Boehm's algorithm, The NURBS Book A5.1) adds a knot `r`
//!   times by replacing `p - s` control points with `p - s + r` affine
//!   combinations of their neighbours, where `s` is the existing
//!   multiplicity. The curve is unchanged.
//! - Refinement (A5.4) inserts many knots in one pass, working backwards
//!   from the end of the new knot vector. The curve is unchanged.
//! - Removal (A5.8) inverts insertion: it solves for the control points of
//!   the curve without the knot from both ends of the affected range and
//!   accepts the removal when the two solutions meet. When they do not
//!   meet exactly, the curve changes by at most their distance (eq. 5.29
//!   and the discussion after A5.8), which is how the tolerance is
//!   enforced.
//!
//! All three work on the homogeneous control points, so they apply to
//! rational curves unchanged. Knot removal measures its deviation in
//! homogeneous space. If the homogeneous curve `(A, W)` moves by at most
//! `da` in its point part and `dw` in its weight, the projected curve
//! `C = A / W` moves by at most `(da + max |C| dw) / W~` with `W~` the new
//! weight function, since `C~ - C = (dA - C dW) / W~`. With `max |C| <=
//! max |P_i|` and `W~ >= min w_i` this gives the model-space bound used
//! here (`ModelBound`); it is the book's eq. 5.30 with the point and
//! weight parts kept apart, which is much tighter when the weights barely
//! change. For a non-rational curve `dw` is zero and the bound is exact.
//! The minimum weight is taken from the curve before removal, so for
//! rational curves the bound is first order, as in the book. Unlike A5.8,
//! which tests each removal against the tolerance on its own, the bounds of
//! successive removals are summed, so the total deviation stays within the
//! tolerance.

use crate::{GeomError, Homogeneous, KnotVector, NurbsCurve};

impl<H: Homogeneous> NurbsCurve<H> {
    /// The curve with knot `u` inserted `r` more times (The NURBS Book,
    /// A5.1). The geometry is unchanged.
    ///
    /// A parameter within [`KnotVector::tolerance`] of an existing knot
    /// raises that knot's multiplicity.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite `u`,
    /// [`GeomError::ParameterOutOfDomain`] unless `u` lies strictly inside
    /// the domain, and [`GeomError::InvalidKnotVector`] if the multiplicity
    /// would exceed the degree.
    pub fn insert_knot(&self, u: f64, r: usize) -> Result<Self, GeomError> {
        let u = self.interior_knot_param(u)?;
        if r == 0 {
            return Ok(self.clone());
        }
        let p = self.degree;
        let s = self.knots.multiplicity(u);
        if s + r > p {
            return Err(GeomError::InvalidKnotVector(
                "knot multiplicity would exceed the degree",
            ));
        }
        let k = self.knots.span(p, u);
        let (knots, control) = insert_raw(p, self.knots.as_slice(), &self.control, u, k, s, r);
        Ok(Self::from_parts(p, KnotVector::from_raw(knots), control))
    }

    /// The curve with all the knots in `new_knots` inserted (The NURBS
    /// Book, A5.4). The knots may come in any order and may repeat; the
    /// geometry is unchanged.
    ///
    /// # Errors
    ///
    /// As [`NurbsCurve::insert_knot`] for each knot.
    pub fn refine_knots(&self, new_knots: &[f64]) -> Result<Self, GeomError> {
        let mut x = Vec::with_capacity(new_knots.len());
        for &u in new_knots {
            x.push(self.interior_knot_param(u)?);
        }
        if x.is_empty() {
            return Ok(self.clone());
        }
        x.sort_by(f64::total_cmp);
        let p = self.degree;
        let mut i = 0;
        while let Some(&u) = x.get(i) {
            let count = x[i..]
                .iter()
                .take_while(|&&v| self.knots.same(u, v))
                .count();
            if count + self.knots.multiplicity(u) > p {
                return Err(GeomError::InvalidKnotVector(
                    "knot multiplicity would exceed the degree",
                ));
            }
            i += count;
        }
        let (knots, control) = refine_raw(p, &self.knots, &self.control, &x);
        Ok(Self::from_parts(p, KnotVector::from_raw(knots), control))
    }

    /// Tries to remove the interior knot `u` up to `num` times while moving
    /// the curve by at most `tol` (in model units) anywhere (The NURBS Book,
    /// A5.8). Returns the new curve and how many times the knot was removed.
    ///
    /// Returns the curve unchanged and 0 if `u` is not an interior knot,
    /// `num` is 0 or `tol` is negative or NaN.
    pub fn remove_knot(&self, u: f64, num: usize, tol: f64) -> (Self, usize) {
        let unchanged = || (self.clone(), 0);
        if num == 0 || tol.is_nan() || tol < 0.0 {
            return unchanged();
        }
        let Ok(u) = self.interior_knot_param(u) else {
            return unchanged();
        };
        let Some(r) = self.knots.last_index_of(u) else {
            return unchanged();
        };
        let s = self.knots.multiplicity(u);
        let scale = ModelBound::of(&self.control);
        let mut used = 0.0;
        let mut accept = |d: H| {
            let e = scale.bound_of(d);
            if used + e <= tol {
                used += e;
                true
            } else {
                false
            }
        };
        let (knots, control, removed) = remove_raw(
            self.degree,
            self.knots.as_slice(),
            &self.control,
            u,
            r,
            s,
            num.min(s),
            &mut accept,
        );
        if removed == 0 {
            return unchanged();
        }
        (
            Self::from_parts(self.degree, KnotVector::from_raw(knots), control),
            removed,
        )
    }

    /// `u` snapped to the knot vector, checked to lie strictly inside the
    /// domain.
    pub(crate) fn interior_knot_param(&self, u: f64) -> Result<f64, GeomError> {
        if !u.is_finite() {
            return Err(GeomError::NonFinite);
        }
        let u = self.knots.snap_param(u);
        let (a, b) = self.domain();
        if u > a && u < b && !self.knots.same(u, a) && !self.knots.same(u, b) {
            Ok(u)
        } else {
            Err(GeomError::ParameterOutOfDomain { t: u })
        }
    }
}

/// Converts deviations of the homogeneous curve into model-space bounds
/// (see the module documentation).
#[derive(Clone, Copy, Debug)]
pub(crate) struct ModelBound {
    /// Largest distance of a control point from the origin.
    p_max: f64,
    /// Smallest weight.
    w_min: f64,
}

impl ModelBound {
    /// The bound for a curve with control points `control`, using the
    /// smallest weight of `weights_from` (the curve after the change, when
    /// it is known).
    pub(crate) fn with_weights<H: Homogeneous>(control: &[H], weights_from: &[H]) -> Self {
        Self {
            p_max: control
                .iter()
                .map(|c| H::point_length(c.project()))
                .fold(0.0, f64::max),
            w_min: weights_from
                .iter()
                .map(|c| c.weight())
                .fold(f64::INFINITY, f64::min),
        }
    }

    /// The bound for a curve with these control points.
    pub(crate) fn of<H: Homogeneous>(control: &[H]) -> Self {
        Self::with_weights(control, control)
    }

    /// The model-space bound for a homogeneous deviation with point part
    /// `da` and weight part `dw`.
    pub(crate) fn bound(self, da: f64, dw: f64) -> f64 {
        (da + self.p_max * dw) / self.w_min
    }

    /// The model-space bound for the homogeneous deviation vector `d`.
    pub(crate) fn bound_of<H: Homogeneous>(self, d: H) -> f64 {
        self.bound(H::point_length(d.weighted()), d.weight().abs())
    }
}

/// A5.1: inserts `u`, which lies in span `k` and has multiplicity `s`, `r`
/// times. Returns the new knots and control points.
///
/// Requires a valid knot vector and control polygon for degree `p`,
/// `U[k] <= u < U[k+1]` with `U[k+1] > U[k]`, `s` the multiplicity of `u` and
/// `1 <= r` with `r + s <= p`; then every index below is in range.
pub(crate) fn insert_raw<H: Homogeneous>(
    p: usize,
    up: &[f64],
    pw: &[H],
    u: f64,
    k: usize,
    s: usize,
    r: usize,
) -> (Vec<f64>, Vec<H>) {
    let np = pw.len() - 1;
    let mut uq = Vec::with_capacity(up.len() + r);
    uq.extend_from_slice(&up[..=k]);
    uq.extend(std::iter::repeat_n(u, r));
    uq.extend_from_slice(&up[k + 1..]);

    let mut qw = vec![H::ZERO; np + 1 + r];
    qw[..=k - p].copy_from_slice(&pw[..=k - p]);
    qw[k - s + r..].copy_from_slice(&pw[k - s..]);
    let mut rw: Vec<H> = pw[k - p..=k - s].to_vec();
    let mut l = 0;
    for j in 1..=r {
        l = k - p + j;
        for i in 0..=p - j - s {
            let alpha = (u - up[l + i]) / (up[i + k + 1] - up[l + i]);
            rw[i] = rw[i + 1] * alpha + rw[i] * (1.0 - alpha);
        }
        qw[l] = rw[0];
        qw[k + r - j - s] = rw[p - j - s];
    }
    // The remaining points between the two ends written above.
    if k - s > l + 1 {
        qw[l + 1..k - s].copy_from_slice(&rw[1..k - s - l]);
    }
    (uq, qw)
}

/// A5.4: inserts the sorted knots `x`, all strictly inside the domain and
/// snapped to `knots`, without exceeding multiplicity `p`.
fn refine_raw<H: Homogeneous>(
    p: usize,
    knots: &KnotVector,
    pw: &[H],
    x: &[f64],
) -> (Vec<f64>, Vec<H>) {
    let u = knots.as_slice();
    let n = pw.len() - 1;
    let m = n + p + 1;
    let r = x.len() - 1;
    let tol = knots.tolerance();
    let a = knots.span(p, x[0]);
    let b = knots.span(p, x[r]) + 1;
    let mut qw = vec![H::ZERO; n + r + 2];
    let mut ubar = vec![0.0; m + r + 2];
    qw[..=a - p].copy_from_slice(&pw[..=a - p]);
    qw[b + r..].copy_from_slice(&pw[b - 1..]);
    ubar[..=a].copy_from_slice(&u[..=a]);
    ubar[b + p + r + 1..].copy_from_slice(&u[b + p..]);
    let mut i = b + p - 1;
    let mut k = b + p + r;
    for &xj in x.iter().rev() {
        while xj <= u[i] && i > a {
            qw[k - p - 1] = pw[i - p - 1];
            ubar[k] = u[i];
            k -= 1;
            i -= 1;
        }
        qw[k - p - 1] = qw[k - p];
        for l in 1..=p {
            let ind = k - p + l;
            let alfa = ubar[k + l] - xj;
            if alfa.abs() <= tol {
                qw[ind - 1] = qw[ind];
            } else {
                let alfa = alfa / (ubar[k + l] - u[i + l - p]);
                qw[ind - 1] = qw[ind - 1] * alfa + qw[ind] * (1.0 - alfa);
            }
        }
        ubar[k] = xj;
        k -= 1;
    }
    (ubar, qw)
}

/// A5.8: removes the knot `u` (last index `r`, multiplicity `s`) up to `num`
/// times, asking `accept` with the homogeneous deviation vector of each
/// removal whether to go ahead. Returns the new knots, control points and
/// the number of removals.
///
/// Requires a valid curve of degree `p` and `u` strictly inside the domain
/// with `1 <= num <= s`; then `r >= p + s`, so every index below is in
/// range.
#[allow(clippy::too_many_arguments)]
pub(crate) fn remove_raw<H: Homogeneous>(
    p: usize,
    knots: &[f64],
    pw: &[H],
    u: f64,
    r: usize,
    s: usize,
    num: usize,
    accept: &mut dyn FnMut(H) -> bool,
) -> (Vec<f64>, Vec<H>, usize) {
    let mut uk = knots.to_vec();
    let mut pw = pw.to_vec();
    let n = pw.len() - 1;
    let m = n + p + 1;
    let ord = p + 1;
    let fout = (2 * r - s - p) / 2;
    let mut last = r - s;
    let mut first = r - p;
    let mut temp = vec![H::ZERO; 2 * p + 3];
    let mut t = 0;
    while t < num {
        // Eq. 5.28: solve from both ends of the affected range.
        let off = first - 1;
        temp[0] = pw[off];
        temp[last + 1 - off] = pw[last + 1];
        let (mut i, mut j) = (first, last);
        let (mut ii, mut jj) = (1, last - off);
        while j > i + t {
            let alfi = (u - uk[i]) / (uk[i + ord + t] - uk[i]);
            let alfj = (u - uk[j - t]) / (uk[j + ord] - uk[j - t]);
            temp[ii] = (pw[i] - temp[ii - 1] * (1.0 - alfi)) * (1.0 / alfi);
            temp[jj] = (pw[j] - temp[jj + 1] * alfj) * (1.0 / (1.0 - alfj));
            i += 1;
            ii += 1;
            j -= 1;
            jj -= 1;
        }
        let deviation = if j < i + t {
            temp[ii - 1] - temp[jj + 1]
        } else {
            let alfi = (u - uk[i]) / (uk[i + ord + t] - uk[i]);
            pw[i] - (temp[ii + t + 1] * alfi + temp[ii - 1] * (1.0 - alfi))
        };
        if !deviation.is_finite() || !accept(deviation) {
            break;
        }
        let (mut i, mut j) = (first, last);
        while j > i + t {
            pw[i] = temp[i - off];
            pw[j] = temp[j - off];
            i += 1;
            j -= 1;
        }
        first -= 1;
        last += 1;
        t += 1;
    }
    if t == 0 {
        return (uk, pw, 0);
    }
    for k in r + 1..=m {
        uk[k - t] = uk[k];
    }
    uk.truncate(m + 1 - t);
    // Control points j..=i are now redundant; shift the rest down over them.
    let mut j = fout;
    let mut i = j;
    for k in 1..t {
        if k % 2 == 1 {
            i += 1;
        } else {
            j -= 1;
        }
    }
    for k in i + 1..=n {
        pw[j] = pw[k];
        j += 1;
    }
    pw.truncate(n + 1 - t);
    (uk, pw, t)
}

#[cfg(test)]
mod tests {
    use bevel_math::DVec3;

    use crate::{GeomError, KnotVector, NurbsCurve3};

    fn polyline() -> NurbsCurve3 {
        let knots = KnotVector::clamped_from_interior(1, 0.0, 2.0, &[1.0]).unwrap();
        NurbsCurve3::from_points(
            1,
            knots,
            &[
                DVec3::ZERO,
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(2.0, 0.0, 0.0),
            ],
            None,
        )
        .unwrap()
    }

    #[test]
    fn insertion_respects_the_degree() {
        let c = polyline();
        assert!(matches!(
            c.insert_knot(1.0, 1),
            Err(GeomError::InvalidKnotVector(_))
        ));
        assert!(matches!(
            c.insert_knot(0.0, 1),
            Err(GeomError::ParameterOutOfDomain { .. })
        ));
        assert!(matches!(
            c.insert_knot(f64::NAN, 1),
            Err(GeomError::NonFinite)
        ));
        let d = c.insert_knot(0.5, 1).unwrap();
        assert_eq!(d.control_weighted().len(), 4);
        assert_eq!(d.knots().as_slice(), &[0.0, 0.0, 0.5, 1.0, 2.0, 2.0]);
        assert!(c.refine_knots(&[0.5, 1.0]).is_err());
        assert_eq!(c.insert_knot(0.5, 0).unwrap(), c);
    }

    #[test]
    fn a_corner_is_removable_only_within_a_large_tolerance() {
        let c = polyline();
        // Removing the corner knot replaces the polyline by its chord, which
        // is 1 away from the corner.
        let (same, removed) = c.remove_knot(1.0, 1, 0.5);
        assert_eq!(removed, 0);
        assert_eq!(same, c);
        let (chord, removed) = c.remove_knot(1.0, 1, 1.0 + 1e-12);
        assert_eq!(removed, 1);
        assert_eq!(chord.control_weighted().len(), 2);
        assert!((chord.point(1.0) - DVec3::new(1.0, 0.0, 0.0)).length() < 1e-12);
        // Not a knot, an end knot, or a bad tolerance: nothing happens.
        assert_eq!(c.remove_knot(0.5, 1, 10.0).1, 0);
        assert_eq!(c.remove_knot(2.0, 1, 10.0).1, 0);
        assert_eq!(c.remove_knot(1.0, 1, f64::NAN).1, 0);
    }

    #[test]
    fn refinement_matches_repeated_insertion() {
        // Pins down the knot indices of the inner loop of A5.4, whose
        // denominator is `Ubar[k+l] - U[i-p+l]`.
        let knots = KnotVector::clamped_from_interior(3, 0.0, 1.0, &[0.3, 0.6]).unwrap();
        let pts: Vec<DVec3> = (0..6)
            .map(|i| DVec3::new(f64::from(i), f64::from(i * i % 5), 1.0))
            .collect();
        let c = NurbsCurve3::from_points(3, knots, &pts, None).unwrap();
        let x = [0.2, 0.3, 0.45, 0.7, 0.7];
        let refined = c.refine_knots(&x).unwrap();
        let mut inserted = c.clone();
        for u in x {
            inserted = inserted.insert_knot(u, 1).unwrap();
        }
        assert_eq!(refined.knots(), inserted.knots());
        for (a, b) in refined
            .control_weighted()
            .iter()
            .zip(inserted.control_weighted())
        {
            assert!((*a - *b).length() < 1e-13);
        }
    }

    /// Found by the property tests: a knot inserted four times right next to
    /// a triple knot of a rational quartic. Removing it again is
    /// ill-conditioned, and bounding the homogeneous deviation by
    /// `d (1 + max |P|) / min w` overestimated the model-space error a
    /// hundredfold, so the last removal was refused.
    #[test]
    #[allow(clippy::unreadable_literal)]
    fn removal_bound_keeps_point_and_weight_parts_apart() {
        let knots = KnotVector::new(
            vec![
                8.148539022648709,
                8.148539022648709,
                8.148539022648709,
                8.148539022648709,
                8.148539022648709,
                10.437677148502258,
                10.437677148502258,
                10.437677148502258,
                10.614203679240932,
                12.058965514897007,
                12.058965514897007,
                14.170110624319278,
                14.814780300040834,
                14.814780300040834,
                14.814780300040834,
                17.083424459540247,
                17.083424459540247,
                17.083424459540247,
                17.083424459540247,
                17.083424459540247,
            ],
            4,
        )
        .unwrap();
        let mut control = vec![bevel_math::DVec4::new(0.0, 0.0, 0.0, 0.2); 15];
        control[11] = bevel_math::DVec4::new(0.0, -74.6231994142235, 0.0, 1.5700243083734513);
        control[12] = bevel_math::DVec4::new(0.0, -3.0416488004882005, 0.0, 0.2);
        let c = NurbsCurve3::new(4, knots, control).unwrap();
        let u = 14.818471589600048;
        let d = c.insert_knot(u, 4).unwrap();
        let (e, removed) = d.remove_knot(u, 4, 1e-8);
        assert_eq!(removed, 4);
        for i in 0..=100 {
            let t = 8.15 + 8.9 * f64::from(i) / 100.0;
            assert!((e.point(t) - c.point(t)).length() < 1e-8);
        }
    }

    #[test]
    fn inserted_knots_come_out_again() {
        let knots = KnotVector::clamped_uniform(3, 6).unwrap();
        let pts: Vec<DVec3> = (0..6)
            .map(|i| {
                let t = f64::from(i);
                DVec3::new(t, (t * 1.3).sin(), (t * 0.7).cos())
            })
            .collect();
        let c = NurbsCurve3::from_points(3, knots, &pts, Some(&[1.0, 2.0, 0.5, 1.0, 3.0, 1.0]))
            .unwrap();
        let d = c.insert_knot(0.4, 3).unwrap();
        assert_eq!(d.knots().multiplicity(0.4), 3);
        let (e, removed) = d.remove_knot(0.4, 3, 1e-9);
        assert_eq!(removed, 3);
        assert_eq!(e.control_weighted().len(), 6);
        for i in 0..=20 {
            let u = f64::from(i) / 20.0;
            assert!((e.point(u) - c.point(u)).length() < 1e-10);
            assert!((d.point(u) - c.point(u)).length() < 1e-12);
        }
    }
}
