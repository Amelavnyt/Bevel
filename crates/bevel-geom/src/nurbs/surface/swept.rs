// SPDX-License-Identifier: GPL-3.0-or-later

//! Surfaces swept from curves: revolution, extrusion, ruled and bilinear
//! surfaces, all exact.
//!
//! - [`NurbsSurface::revolution`] is `MakeRevolvedSurf`, The NURBS Book A8.1
//!   (section 8.5): every control point `P_j` of the profile (weight `w_j`)
//!   sweeps a circle about the axis, built as in A7.1 (see
//!   [`circle_arc`](crate::circle_arc)): with `O_j` the foot of `P_j` on
//!   the axis, `X_j = P_j - O_j` and `Y_j = T x X_j` for the unit axis
//!   direction `T`, the control point `(i, j)` is `O_j + c_i X_j + s_i Y_j`
//!   with weight `a_i w_j`, where `(c_i, s_i)` and `a_i` are the control
//!   points and weights of the unit-circle arc. The direction of the circles
//!   is `u` (degree 2, rational, on the domain `[0, angle]` with the
//!   parameter equal to the angle at the knots), the profile direction is
//!   `v`. A profile point on the axis gives a row of coincident points (a
//!   pole).
//! - [`NurbsSurface::extrusion`] is the general cylinder of section 8.3:
//!   `S(u, v) = C(u) + v d` on `v` in `[0, 1]`, degree 1 in `v`, each
//!   homogeneous control point `(w P, w)` paired with `(w (P + d), w)`.
//! - [`NurbsSurface::ruled`] is the ruled surface of section 8.4: the two
//!   curves are made compatible ([`make_compatible`]) and paired control
//!   point by control point, degree 1 in `v` on `[0, 1]`. The pairing is in
//!   homogeneous space, so the rulings are straight lines between the
//!   points of the two curves at the same parameter.
//! - [`NurbsSurface::bilinear`] is the degree (1, 1) patch on four corners.

use bevel_math::{DVec4, Point3, Vector3};

use crate::conics::unit_arc;
use crate::{GeomError, Homogeneous, KnotVector, NurbsCurve3, NurbsSurface, make_compatible};

impl NurbsSurface {
    /// The surface swept by revolving `profile` by `angle` (radians,
    /// counter-clockwise by the right-hand rule) about the axis through
    /// `axis_origin` along `axis_dir` (The NURBS Book, A8.1). It is exact
    /// and rational in `u`, the direction of rotation, on the domain
    /// `[0, angle]`; `v` is the profile's parameter, with its degree and
    /// knots. The surface at `(0, v)` is the profile at `v`. An angle of
    /// `2 pi` gives a surface closed in `u`, whose last row of control points
    /// is exactly the first.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for non-finite input and
    /// [`GeomError::InvalidArgument`] for a zero axis direction or an angle
    /// outside `(0, 2 pi]`.
    pub fn revolution(
        profile: &NurbsCurve3,
        axis_origin: Point3,
        axis_dir: Vector3,
        angle: f64,
    ) -> Result<Self, GeomError> {
        if !axis_origin.is_finite() || !axis_dir.is_finite() {
            return Err(GeomError::NonFinite);
        }
        let Some(t) = axis_dir.try_normalize() else {
            return Err(GeomError::InvalidArgument(
                "the axis direction must be non-zero",
            ));
        };
        let arc = unit_arc(0.0, angle)?;
        let profile_control = profile.control_weighted();
        let nv = profile_control.len();
        let rows = arc.points.len();
        let mut control = Vec::with_capacity(rows * nv);
        for (i, (c, &a)) in arc.points.iter().zip(&arc.weights).enumerate() {
            let first_row = i == 0 || (arc.full && i + 1 == rows);
            for pw in profile_control {
                let (p, w) = (pw.project(), pw.weight());
                let point = if first_row {
                    p
                } else {
                    let o = axis_origin + t * (p - axis_origin).dot(t);
                    let x = p - o;
                    o + x * c.x + t.cross(x) * c.y
                };
                control.push(DVec4::from_point(point, a * w));
            }
        }
        Self::new(
            2,
            profile.degree(),
            arc.knots,
            profile.knots().clone(),
            control,
        )
    }

    /// The general cylinder `S(u, v) = profile(u) + v direction` for `v` in
    /// `[0, 1]` (The NURBS Book, section 8.3): `u` is the profile's
    /// parameter, with its degree and knots, and the surface has degree 1 in
    /// `v`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite direction and
    /// [`GeomError::InvalidArgument`] for a zero one.
    pub fn extrusion(profile: &NurbsCurve3, direction: Vector3) -> Result<Self, GeomError> {
        if !direction.is_finite() {
            return Err(GeomError::NonFinite);
        }
        if direction.length() <= f64::MIN_POSITIVE {
            return Err(GeomError::InvalidArgument(
                "the extrusion direction must be non-zero",
            ));
        }
        let mut control = Vec::with_capacity(2 * profile.control_weighted().len());
        for &pw in profile.control_weighted() {
            control.push(pw);
            control.push(pw + (direction * pw.weight()).extend(0.0));
        }
        Self::new(
            profile.degree(),
            1,
            profile.knots().clone(),
            unit_linear_knots()?,
            control,
        )
    }

    /// The ruled surface between `a` (at `v = 0`) and `b` (at `v = 1`)
    /// (The NURBS Book, section 8.4). The curves are made compatible first
    /// ([`make_compatible`]), so `u` runs over the domain of `a`, with the
    /// highest of their degrees and the union of their knots, and `b` is
    /// reparametrised affinely onto that domain. The surface has degree 1 in
    /// `v`: the line from `a(u)` to `b(u)`, where `b(u)` is the
    /// reparametrised `b`, is the ruling at `u`.
    ///
    /// # Errors
    ///
    /// As [`make_compatible`].
    pub fn ruled(a: &NurbsCurve3, b: &NurbsCurve3) -> Result<Self, GeomError> {
        let curves = make_compatible(&[a.clone(), b.clone()])?;
        let (Some(ca), Some(cb)) = (curves.first(), curves.get(1)) else {
            return Err(GeomError::InvalidArgument(
                "a ruled surface needs two curves",
            ));
        };
        let mut control = Vec::with_capacity(2 * ca.control_weighted().len());
        for (&pa, &pb) in ca.control_weighted().iter().zip(cb.control_weighted()) {
            control.push(pa);
            control.push(pb);
        }
        Self::new(
            ca.degree(),
            1,
            ca.knots().clone(),
            unit_linear_knots()?,
            control,
        )
    }

    /// The bilinear patch on `[0, 1]^2` through four corners: `p00` at
    /// `(0, 0)`, `p10` at `(1, 0)`, `p01` at `(0, 1)` and `p11` at `(1, 1)`.
    ///
    /// # Errors
    ///
    /// [`GeomError::NonFinite`] for a non-finite corner.
    pub fn bilinear(p00: Point3, p10: Point3, p01: Point3, p11: Point3) -> Result<Self, GeomError> {
        let k = unit_linear_knots()?;
        Self::from_points(1, 1, k.clone(), k, &[p00, p01, p10, p11], None)
    }
}

/// The knot vector `[0, 0, 1, 1]`.
fn unit_linear_knots() -> Result<KnotVector, GeomError> {
    KnotVector::clamped_from_interior(1, 0.0, 1.0, &[])
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_PI_2, PI, TAU};

    use bevel_math::{DVec3, Frame3};

    use crate::{GeomError, NurbsCurve3, NurbsSurface, circle_arc, make_compatible};

    /// A `count x count` grid of parameters strictly inside the domain.
    fn inner_grid(s: &NurbsSurface, count: usize) -> Vec<(f64, f64)> {
        let ((ua, ub), (va, vb)) = s.domain();
        let mut out = Vec::new();
        for i in 0..count {
            for j in 0..count {
                let fu = (i as f64 + 0.5) / count as f64;
                let fv = (j as f64 + 0.5) / count as f64;
                out.push((ua + (ub - ua) * fu, va + (vb - va) * fv));
            }
        }
        out
    }

    fn line(a: DVec3, b: DVec3) -> NurbsCurve3 {
        NurbsCurve3::bezier(&[a, b]).unwrap()
    }

    #[test]
    fn revolving_a_parallel_line_gives_a_cylinder() {
        let r = 3.0;
        let profile = line(DVec3::new(r, 0.0, -1.0), DVec3::new(r, 0.0, 4.0));
        let s = NurbsSurface::revolution(&profile, DVec3::ZERO, DVec3::Z, TAU).unwrap();
        assert_eq!(s.domain().0, (0.0, TAU));
        assert!(s.is_closed_u(0.0));
        assert!(!s.is_closed_v(1e-6));
        for (u, v) in inner_grid(&s, 15) {
            let p = s.point(u, v);
            assert!(
                (p.truncate().length() - r).abs() <= 1e-12 * r,
                "radius at ({u}, {v})"
            );
            // The normal points away from the axis, so the surface bends
            // away from it: curvatures 0 and -1/r.
            let n = s.normal(u, v).unwrap();
            assert!((n - p.truncate().extend(0.0) / r).length() < 1e-12);
            let c = s.principal_curvatures(u, v).unwrap();
            assert!(c.k1.abs() < 1e-12, "{c:?}");
            assert!((c.k2 + 1.0 / r).abs() < 1e-12, "{c:?}");
            assert!(c.dir1.z.abs() > 1.0 - 1e-12);
        }
        // The profile is the first row, and the angle is the parameter at
        // the knots.
        assert!((s.point(0.0, 0.3) - profile.point(0.3)).length() < 1e-15);
        assert!((s.point(PI, 0.0) - DVec3::new(-r, 0.0, -1.0)).length() < 1e-12);
    }

    #[test]
    fn revolving_a_quarter_circle_about_a_diameter_gives_a_sphere() {
        let r = 2.0;
        let c = DVec3::new(1.0, -1.0, 0.5);
        // The quarter circle from the equator to the pole in the XZ plane.
        let frame = Frame3::from_axes(c, DVec3::X, DVec3::Z).unwrap();
        let profile = circle_arc(&frame, r, 0.0, FRAC_PI_2).unwrap();
        let s = NurbsSurface::revolution(&profile, c, DVec3::Z, 1.5 * PI).unwrap();
        for (u, v) in inner_grid(&s, 15) {
            let p = s.point(u, v);
            assert!(((p - c).length() - r).abs() <= 1e-12 * r);
            let k = s.principal_curvatures(u, v).unwrap();
            assert!(
                (k.k1 + 1.0 / r).abs() < 1e-10 && (k.k2 + 1.0 / r).abs() < 1e-10,
                "{k:?}"
            );
        }
        // The pole has no normal.
        assert!(s.normal(1.0, FRAC_PI_2).is_none());
        assert!((s.point(1.0, FRAC_PI_2) - (c + DVec3::Z * r)).length() < 1e-12);
    }

    #[test]
    fn revolving_a_full_circle_gives_a_torus() {
        let (big, small) = (5.0, 1.5);
        let frame = Frame3::from_axes(DVec3::new(big, 0.0, 0.0), DVec3::X, DVec3::Z).unwrap();
        let profile = circle_arc(&frame, small, 0.0, TAU).unwrap();
        let s = NurbsSurface::revolution(&profile, DVec3::ZERO, DVec3::Z, TAU).unwrap();
        assert!(s.is_closed_u(0.0) && s.is_closed_v(0.0));
        for (u, v) in inner_grid(&s, 16) {
            let p = s.point(u, v);
            let rho = p.truncate().length();
            let tube = ((rho - big).powi(2) + p.z * p.z).sqrt();
            assert!(
                (tube - small).abs() <= 1e-12 * big,
                "tube radius at ({u}, {v})"
            );
        }
        // Every isocurve along u is a circle about the axis; along v, a
        // circle of the tube.
        let ((ua, ub), (va, vb)) = s.domain();
        for k in 0..8 {
            let v = va + (vb - va) * f64::from(k) / 8.0;
            let iso = s.isocurve_v(v).unwrap();
            let p0 = iso.point(ua);
            for i in 0..=40 {
                let p = iso.point(ua + (ub - ua) * f64::from(i) / 40.0);
                assert!((p.truncate().length() - p0.truncate().length()).abs() < 1e-11);
                assert!((p.z - p0.z).abs() < 1e-12);
            }
        }
        // On the outer equator the curvatures are -1/r (across the tube)
        // and -1/(R + r) (around the axis).
        let c = s.principal_curvatures(1.0, 0.0).unwrap();
        assert!((c.k2 + 1.0 / small).abs() < 1e-10, "{c:?}");
        assert!((c.k1 + 1.0 / (big + small)).abs() < 1e-10, "{c:?}");
    }

    #[test]
    fn extruding_an_arc_gives_a_cylinder_patch() {
        let r = 4.0;
        let arc = circle_arc(&Frame3::WORLD, r, 0.2, 2.9).unwrap();
        let s = NurbsSurface::extrusion(&arc, DVec3::new(0.0, 0.0, 7.0)).unwrap();
        assert_eq!((s.degree_u(), s.degree_v()), (2, 1));
        for (u, v) in inner_grid(&s, 12) {
            let p = s.point(u, v);
            assert!((p.truncate().length() - r).abs() <= 1e-12 * r);
            assert!((p.z - 7.0 * v).abs() <= 1e-12);
            let c = s.principal_curvatures(u, v).unwrap();
            assert!(
                c.k1.abs() < 1e-12 && (c.k2 + 1.0 / r).abs() < 1e-12,
                "{c:?}"
            );
        }
    }

    #[test]
    fn ruled_surface_interpolates_both_curves() {
        let a = line(DVec3::new(0.0, 0.0, 0.0), DVec3::new(6.0, 0.0, 0.0));
        let frame = Frame3 {
            origin: DVec3::new(3.0, 0.0, 4.0),
            ..Frame3::WORLD
        };
        let b = circle_arc(&frame, 3.0, PI, TAU).unwrap();
        let s = NurbsSurface::ruled(&a, &b).unwrap();
        assert_eq!(s.domain(), ((0.0, 1.0), (0.0, 1.0)));
        assert_eq!(s.degree_u(), 2);
        for i in 0..=40 {
            let u = f64::from(i) / 40.0;
            assert!((s.point(u, 0.0) - a.point(u)).length() < 1e-12);
            let bu = b.point(PI + PI * u);
            assert!((s.point(u, 1.0) - bu).length() < 1e-12);
            // The ruling at u is the straight segment between the two.
            for k in 1..4 {
                let v = f64::from(k) / 4.0;
                let off = (s.point(u, v) - a.point(u)).cross(bu - a.point(u)).length();
                assert!(off < 1e-10, "ruling not straight at ({u}, {v})");
            }
        }
    }

    #[test]
    fn bilinear_patch_hits_its_corners() {
        let (p00, p10, p01, p11) = (
            DVec3::ZERO,
            DVec3::new(2.0, 0.0, 1.0),
            DVec3::new(0.0, 3.0, -1.0),
            DVec3::new(2.0, 3.0, 5.0),
        );
        let s = NurbsSurface::bilinear(p00, p10, p01, p11).unwrap();
        for (u, v, p) in [
            (0.0, 0.0, p00),
            (1.0, 0.0, p10),
            (0.0, 1.0, p01),
            (1.0, 1.0, p11),
        ] {
            assert!((s.point(u, v) - p).length() < 1e-15);
        }
        let mid = (p00 + p10 + p01 + p11) * 0.25;
        assert!((s.point(0.5, 0.5) - mid).length() < 1e-15);
        assert!(matches!(
            NurbsSurface::bilinear(p00, p10, p01, DVec3::splat(f64::NAN)),
            Err(GeomError::NonFinite)
        ));
    }

    #[test]
    fn make_compatible_keeps_the_ruled_curves() {
        let a = line(DVec3::ZERO, DVec3::X);
        let b = circle_arc(&Frame3::WORLD, 1.0, 0.0, PI).unwrap();
        let out = make_compatible(&[a, b.clone()]).unwrap();
        for i in 0..=20 {
            let t = f64::from(i) / 20.0;
            assert!((out[1].point(t) - b.point(PI * t)).length() < 1e-12);
        }
    }

    #[test]
    fn invalid_sweeps_are_rejected() {
        let profile = line(DVec3::X, DVec3::new(1.0, 0.0, 1.0));
        let bad =
            |r: Result<NurbsSurface, GeomError>| matches!(r, Err(GeomError::InvalidArgument(_)));
        assert!(bad(NurbsSurface::revolution(
            &profile,
            DVec3::ZERO,
            DVec3::ZERO,
            1.0
        )));
        assert!(bad(NurbsSurface::revolution(
            &profile,
            DVec3::ZERO,
            DVec3::Z,
            0.0
        )));
        assert!(bad(NurbsSurface::revolution(
            &profile,
            DVec3::ZERO,
            DVec3::Z,
            -1.0
        )));
        assert!(bad(NurbsSurface::revolution(
            &profile,
            DVec3::ZERO,
            DVec3::Z,
            7.0
        )));
        assert!(matches!(
            NurbsSurface::revolution(&profile, DVec3::ZERO, DVec3::Z, f64::NAN),
            Err(GeomError::NonFinite)
        ));
        assert!(matches!(
            NurbsSurface::revolution(&profile, DVec3::splat(f64::INFINITY), DVec3::Z, 1.0),
            Err(GeomError::NonFinite)
        ));
        assert!(bad(NurbsSurface::extrusion(&profile, DVec3::ZERO)));
        assert!(matches!(
            NurbsSurface::extrusion(&profile, DVec3::new(0.0, f64::NAN, 0.0)),
            Err(GeomError::NonFinite)
        ));
    }
}
