// SPDX-License-Identifier: GPL-3.0-or-later

//! Making curves compatible: the same degree, domain and knot vector.
//!
//! Ruled surfaces, lofts and Gordon surfaces combine curves control point
//! by control point, which needs the curves to share their degree and knot
//! vector (The NURBS Book, sections 8.4 and 10.3). [`make_compatible`] gets
//! there in three steps, each of which leaves every curve's geometry
//! unchanged:
//!
//! 1. reparametrise every curve affinely onto the domain of the first
//!    (section 6.1, see [`NurbsCurve::reparametrized`]);
//! 2. raise every curve to the highest degree among them (A5.9), which also
//!    clamps it;
//! 3. merge the knot vectors: every distinct knot of any curve, with the
//!    largest multiplicity it has in any curve, and insert into each curve
//!    the knots it lacks (A5.4).
//!
//! Knots of different curves that agree to within the knot tolerance are
//! the same knot; afterwards every curve gets the one merged knot vector,
//! so they are bit-identical (this moves a knot by at most the tolerance,
//! as snapping in [`KnotVector::new`] does).

use crate::{GeomError, Homogeneous, KnotVector, NurbsCurve};

/// The curves made compatible: the same degree (the highest among them),
/// the same clamped knot vector and the domain of the first curve, each
/// with its geometry unchanged (see the module documentation). Curve `i`
/// of the result at `a + (t - c_i) (b - a) / (d_i - c_i)` is curve `i` of
/// the input at `t`, where `[a, b]` is the domain of the first curve and
/// `[c_i, d_i]` that of curve `i`.
///
/// # Errors
///
/// [`GeomError::InvalidArgument`] for an empty list,
/// [`GeomError::InvalidKnotVector`] if the domain of the first curve is too
/// short to keep the knots of another apart.
pub fn make_compatible<H: Homogeneous>(
    curves: &[NurbsCurve<H>],
) -> Result<Vec<NurbsCurve<H>>, GeomError> {
    let Some(first) = curves.first() else {
        return Err(GeomError::InvalidArgument(
            "make_compatible needs at least one curve",
        ));
    };
    let (a, b) = first.domain();
    let degree = curves.iter().map(NurbsCurve::degree).max().unwrap_or(1);
    // Steps 1 and 2.
    let mut prepared = Vec::with_capacity(curves.len());
    for (i, c) in curves.iter().enumerate() {
        let c = if i == 0 {
            c.clone()
        } else {
            c.reparametrized(a, b)?
        };
        prepared.push(c.elevate_degree(degree - c.degree())?.clamped());
    }
    // Step 3: the merged knots as (value, multiplicity), sorted.
    let tol = prepared
        .iter()
        .map(|c| c.knots().tolerance())
        .fold(0.0, f64::max);
    let mut all: Vec<(f64, usize)> = prepared.iter().flat_map(|c| c.knots().distinct()).collect();
    all.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut merged: Vec<(f64, usize)> = Vec::new();
    for (u, m) in all {
        match merged.last_mut() {
            Some((v, n)) if (u - *v).abs() <= tol => *n = (*n).max(m),
            _ => merged.push((u, m)),
        }
    }
    // The ends are exactly those of the first curve.
    if let Some(e) = merged.first_mut() {
        e.0 = a;
    }
    if let Some(e) = merged.last_mut() {
        e.0 = b;
    }
    let mut knots = Vec::new();
    for &(u, m) in &merged {
        knots.extend(std::iter::repeat_n(u, m));
    }
    let knots = KnotVector::new(knots, degree)?;
    let interior = merged.get(1..merged.len().saturating_sub(1)).unwrap_or(&[]);
    prepared
        .iter()
        .map(|c| {
            let mut missing = Vec::new();
            for &(u, m) in interior {
                let have = c.knots().multiplicity(u);
                missing.extend(std::iter::repeat_n(u, m.saturating_sub(have)));
            }
            let refined = c.refine_knots(&missing)?;
            let fits = refined.knots().as_slice().len() == knots.as_slice().len()
                && refined
                    .knots()
                    .as_slice()
                    .iter()
                    .zip(knots.as_slice())
                    .all(|(&x, &y)| (x - y).abs() <= tol);
            if !fits {
                return Err(GeomError::InvalidKnotVector(
                    "the knot vectors of the curves could not be merged",
                ));
            }
            Ok(NurbsCurve::from_parts(
                degree,
                knots.clone(),
                refined.control_weighted().to_vec(),
            ))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use bevel_math::DVec3;

    use super::make_compatible;
    use crate::{GeomError, KnotVector, NurbsCurve3};

    #[test]
    fn compatible_curves_share_degree_and_knots() {
        let line = NurbsCurve3::bezier(&[DVec3::ZERO, DVec3::new(4.0, 0.0, 0.0)]).unwrap();
        let knots = KnotVector::clamped_from_interior(3, -2.0, 5.0, &[0.0, 0.0, 1.5]).unwrap();
        let pts: Vec<DVec3> = (0..7)
            .map(|i| DVec3::new(f64::from(i), f64::from(i * i % 3), 1.0))
            .collect();
        let w = [1.0, 2.0, 0.5, 1.0, 1.5, 1.0, 0.8];
        let cubic = NurbsCurve3::from_points(3, knots, &pts, Some(&w)).unwrap();
        let out = make_compatible(&[line.clone(), cubic.clone()]).unwrap();
        assert_eq!(out.len(), 2);
        assert_eq!(out[0].degree(), 3);
        assert_eq!(out[0].knots(), out[1].knots());
        assert_eq!(out[0].domain(), (0.0, 1.0));
        // The cubic's knots map to 2/7, 2/7 and 3.5/7; the line gains them.
        assert_eq!(out[0].knots().multiplicity(2.0 / 7.0), 2);
        for i in 0..=50 {
            let t = f64::from(i) / 50.0;
            assert!((out[0].point(t) - line.point(t)).length() < 1e-12);
            assert!((out[1].point(t) - cubic.point(-2.0 + 7.0 * t)).length() < 1e-11);
        }
        assert!(matches!(
            make_compatible::<bevel_math::DVec4>(&[]),
            Err(GeomError::InvalidArgument(_))
        ));
        assert_eq!(
            make_compatible(std::slice::from_ref(&line)).unwrap(),
            vec![line]
        );
    }
}
