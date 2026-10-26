// SPDX-License-Identifier: GPL-3.0-or-later

//! Shared closed forms for the surfaces of revolution (cylinder, cone,
//! sphere, torus).
//!
//! Each of them is `S(u, v) = o + rho(v) e(u) + h(v) z` in its frame, with
//! `e(u) = cos(u) x + sin(u) y` and a profile `(rho(v), h(v))` in the
//! half-plane at angle `u`. Its partial derivatives are
//! `d^(k+l) S / du^k dv^l = rho^(l)(v) e^(k)(u) + [k = 0] h^(l)(v) z`, and
//! each coordinate is a sinusoid in `u` for fixed `v`, which gives exact
//! bounding boxes.

use bevel_math::{Aabb3, Frame3, Interval, Point3, Vector3};

use super::{check_range, sinusoid_extrema, trig_derivative};
use crate::{GeomError, MAX_DERIVATIVE_ORDER};

/// The partial derivatives up to total order `d` (capped at
/// [`MAX_DERIVATIVE_ORDER`]) in the layout of
/// [`NurbsSurface::derivatives`](crate::NurbsSurface::derivatives), for the
/// profile whose `l`-th derivatives at `v` are `profile(l) = (rho^(l),
/// h^(l))`.
pub(crate) fn revolved_derivatives(
    frame: &Frame3,
    u: f64,
    d: usize,
    profile: impl Fn(usize) -> (f64, f64),
) -> Vec<Vec<Vector3>> {
    let d = d.min(MAX_DERIVATIVE_ORDER);
    let (s, c) = u.sin_cos();
    let profiles: Vec<(f64, f64)> = (0..=d).map(&profile).collect();
    (0..=d)
        .map(|k| {
            let (dc, ds) = trig_derivative(c, s, k);
            let e = frame.x * dc + frame.y * ds;
            profiles
                .iter()
                .take(d - k + 1)
                .enumerate()
                .map(|(l, &(rho, h))| {
                    let mut val = e * rho;
                    if k == 0 {
                        val += frame.z * h;
                        if l == 0 {
                            val += frame.origin;
                        }
                    }
                    val
                })
                .collect()
        })
        .collect()
}

/// The exact bounding box of a surface of revolution over a parameter
/// rectangle. `sinusoidal_v` says whether the profile is a circle in `v`
/// (sphere, torus: every coordinate is `A cos(v - phase) + B` for fixed
/// `u`) or a line (cylinder, cone: linear in `v`, so its extremes are at
/// the ends).
///
/// Every extreme of a coordinate over the rectangle lies where it is
/// extremal in both directions or on an edge: at `u` an end or a
/// stationary angle of the sinusoid in `u`, and at `v` an end or (for a
/// circular profile) a stationary angle of the sinusoid in `v` at that
/// `u`. The box is that of the surface points at those parameters.
pub(crate) fn revolved_bounds(
    frame: &Frame3,
    u: Interval,
    v: Interval,
    sinusoidal_v: bool,
    point: impl Fn(f64, f64) -> Point3,
) -> Result<Aabb3, GeomError> {
    check_range(u)?;
    check_range(v)?;
    let mut points = Vec::new();
    for k in 0..3 {
        let (xk, yk, zk) = (frame.x[k], frame.y[k], frame.z[k]);
        let mut us = vec![u.min, u.max];
        sinusoid_extrema(u, yk.atan2(xk), &mut us);
        for &uu in &us {
            let (s, c) = uu.sin_cos();
            let mut vs = vec![v.min, v.max];
            if sinusoidal_v {
                sinusoid_extrema(v, zk.atan2(c * xk + s * yk), &mut vs);
            }
            points.extend(vs.into_iter().map(|vv| point(uu, vv)));
        }
    }
    Ok(Aabb3::from_points(points))
}

/// `e(u) = cos(u) x + sin(u) y` and its derivative `e'(u)`.
pub(crate) fn radial(frame: &Frame3, u: f64) -> (Vector3, Vector3) {
    let (s, c) = u.sin_cos();
    (frame.x * c + frame.y * s, frame.y * c - frame.x * s)
}

/// The angle of the local point `(x, y)` about the frame's axis in `[0, 2
/// pi)`, and its distance from the axis; the angle is 0 on the axis.
pub(crate) fn axial_angle(local: Point3) -> (f64, f64) {
    let rho = local.x.hypot(local.y);
    let u = if rho > 0.0 {
        super::wrap_angle(local.y.atan2(local.x))
    } else {
        0.0
    };
    (u, rho)
}
