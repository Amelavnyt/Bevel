// SPDX-License-Identifier: GPL-3.0-or-later

//! Random frames, radii and parameters, and shared checks.

use std::f64::consts::TAU;
use std::fmt::Debug;
use std::ops::{Add, Mul, Sub};

use bevel_geom::{ClosestPoint, Curvatures, SurfaceClosestPoint};
use bevel_math::{DQuat, DVec2, DVec3, Frame3, Point2, Point3, Vector3};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed, TestCaseError};

/// A fixed-seed configuration so that runs are reproducible.
///
/// For exploratory runs, `BEVEL_PROPTEST_SEED` replaces the seed and
/// `BEVEL_PROPTEST_CASES` the number of cases.
pub fn config(cases: u32) -> Config {
    let env = |name: &str| std::env::var(name).ok().and_then(|v| v.parse::<u64>().ok());
    Config {
        cases: env("BEVEL_PROPTEST_CASES").map_or(cases, |c| u32::try_from(c).unwrap_or(cases)),
        rng_seed: RngSeed::Fixed(env("BEVEL_PROPTEST_SEED").unwrap_or(0xa7a1_7c00)),
        failure_persistence: None,
        ..Config::default()
    }
}

/// A random rotation.
pub fn rotation() -> impl Strategy<Value = DQuat> {
    (prop::array::uniform3(-1.0f64..1.0), 0.0f64..TAU).prop_map(|(a, angle)| {
        let axis = DVec3::from_array(a).try_normalize().unwrap_or(DVec3::Z);
        DQuat::from_axis_angle(axis, angle)
    })
}

/// A random point with coordinates in `[-r, r]`.
pub fn point3(r: f64) -> impl Strategy<Value = Point3> {
    prop::array::uniform3(-r..r).prop_map(DVec3::from_array)
}

/// A random point with coordinates in `[-r, r]`.
pub fn point2(r: f64) -> impl Strategy<Value = Point2> {
    prop::array::uniform2(-r..r).prop_map(DVec2::from_array)
}

/// A random right-handed orthonormal frame with its origin within 50 of
/// the world origin.
pub fn frame() -> impl Strategy<Value = Frame3> {
    (rotation(), point3(50.0)).prop_map(|(q, origin)| Frame3 {
        origin,
        x: q * DVec3::X,
        y: q * DVec3::Y,
        z: q * DVec3::Z,
    })
}

/// A random radius.
pub fn radius() -> impl Strategy<Value = f64> {
    0.1f64..40.0
}

/// A random angle, not restricted to one turn.
pub fn angle() -> impl Strategy<Value = f64> {
    -10.0f64..10.0
}

/// `count` parameters spread evenly over `[a, b]`, ends included.
pub fn params(a: f64, b: f64, count: usize) -> Vec<f64> {
    (0..count)
        .map(|i| a + (b - a) * i as f64 / (count - 1) as f64)
        .collect()
}

/// The vector operations the checks need, for 2D and 3D points.
pub trait Pt:
    Copy + Debug + Add<Output = Self> + Sub<Output = Self> + Mul<f64, Output = Self>
{
    /// Euclidean length.
    fn len(self) -> f64;
    /// The curvature from the first two derivatives: signed in 2D.
    fn curvature_of(d1: Self, d2: Self) -> f64;
    /// The coordinates.
    fn coords(self) -> Vec<f64>;
}

impl Pt for DVec2 {
    fn len(self) -> f64 {
        self.length()
    }
    fn curvature_of(d1: Self, d2: Self) -> f64 {
        d1.perp_dot(d2) / d1.length().powi(3)
    }
    fn coords(self) -> Vec<f64> {
        self.to_array().to_vec()
    }
}

impl Pt for DVec3 {
    fn len(self) -> f64 {
        self.length()
    }
    fn curvature_of(d1: Self, d2: Self) -> f64 {
        d1.cross(d2).length() / d1.length().powi(3)
    }
    fn coords(self) -> Vec<f64> {
        self.to_array().to_vec()
    }
}

/// The closed forms of a curve under test.
pub struct CurveCase<'a, P> {
    /// The point at a parameter.
    pub point: &'a dyn Fn(f64) -> P,
    /// The derivatives at a parameter up to an order.
    pub ders: &'a dyn Fn(f64, usize) -> Vec<P>,
    /// The unit tangent.
    pub tangent: &'a dyn Fn(f64) -> Option<P>,
    /// The curvature.
    pub curvature: &'a dyn Fn(f64) -> Option<f64>,
    /// The closest point.
    pub closest: &'a dyn Fn(P) -> ClosestPoint<P>,
    /// The length scale of the curve (its largest radius, or 1).
    pub scale: f64,
}

/// Derivatives against central differences of the previous order, the
/// tangent and the curvature against the derivatives, at `t`.
pub fn check_curve_differentials<P: Pt>(c: &CurveCase<P>, t: f64) -> Result<(), TestCaseError> {
    let h = 1e-5;
    let d = (c.ders)(t, 4);
    prop_assert_eq!(d.len(), 5);
    prop_assert!((d[0] - (c.point)(t)).len() <= 1e-14 * c.scale.max(100.0));
    let dp = (c.ders)(t + h, 3);
    let dm = (c.ders)(t - h, 3);
    for k in 1..=4 {
        let fd = (dp[k - 1] - dm[k - 1]) * (0.5 / h);
        prop_assert!(
            (fd - d[k]).len() <= 1e-8 * (c.scale + d[k - 1].len()),
            "derivative {k} at {t}: {:?} vs {:?}",
            d[k],
            fd
        );
    }
    let tangent = (c.tangent)(t).unwrap();
    prop_assert!((tangent - d[1] * (1.0 / d[1].len())).len() <= 1e-14);
    let k = (c.curvature)(t).unwrap();
    let expected = P::curvature_of(d[1], d[2]);
    prop_assert!(
        (k - expected).abs() <= 1e-12 * expected.abs().max(1e-3),
        "curvature {k} vs {expected}"
    );
    Ok(())
}

/// The closest point to `target` against `samples` parameters of the curve.
pub fn check_curve_closest<P: Pt>(
    c: &CurveCase<P>,
    target: P,
    samples: &[f64],
) -> Result<(), TestCaseError> {
    let cp = (c.closest)(target);
    prop_assert!(((c.point)(cp.t) - cp.point).len() <= 1e-12 * c.scale.max(100.0));
    prop_assert!(((target - cp.point).len() - cp.distance).abs() <= 1e-12 * c.scale.max(100.0));
    for &t in samples {
        let d = ((c.point)(t) - target).len();
        prop_assert!(
            cp.distance <= d + 1e-9 * c.scale.max(1.0),
            "closest distance {} but the sample at {t} is at {d}",
            cp.distance
        );
    }
    Ok(())
}

/// A bounding box (as min and max coordinates) against dense samples of the
/// range: it contains them and each face is touched to within `slack`.
pub fn check_bounds<P: Pt>(min: P, max: P, points: &[P], slack: f64) -> Result<(), TestCaseError> {
    let (lo, hi) = (min.coords(), max.coords());
    for (k, (&lo, &hi)) in lo.iter().zip(&hi).enumerate() {
        let values: Vec<f64> = points.iter().map(|p| p.coords()[k]).collect();
        let smin = values.iter().copied().fold(f64::INFINITY, f64::min);
        let smax = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        prop_assert!(lo <= smin + 1e-12 * slack.max(1.0) && hi >= smax - 1e-12 * slack.max(1.0));
        prop_assert!(
            smin - lo <= slack && hi - smax <= slack,
            "axis {k}: box [{lo}, {hi}], samples [{smin}, {smax}]"
        );
    }
    Ok(())
}

/// The closed forms of a surface under test.
pub struct SurfaceCase<'a> {
    /// The point at `(u, v)`.
    pub point: &'a dyn Fn(f64, f64) -> Point3,
    /// The partial derivatives up to a total order.
    pub ders: &'a dyn Fn(f64, f64, usize) -> Vec<Vec<Vector3>>,
    /// The first partials.
    pub partials: &'a dyn Fn(f64, f64) -> (Vector3, Vector3),
    /// The unit normal.
    pub normal: &'a dyn Fn(f64, f64) -> Option<Vector3>,
    /// The principal curvatures.
    pub curvatures: &'a dyn Fn(f64, f64) -> Option<Curvatures>,
    /// The closest point.
    pub closest: &'a dyn Fn(Point3) -> SurfaceClosestPoint,
    /// The length scale of the surface (its largest radius, or 1).
    pub scale: f64,
}

/// Partial derivatives against central differences, the normal against
/// `S_u x S_v`, and the principal curvatures against the normal curvature
/// `II(t, t)` along their directions, at `(u, v)` (away from singular
/// points of the parametrization).
pub fn check_surface_differentials(c: &SurfaceCase, u: f64, v: f64) -> Result<(), TestCaseError> {
    let h = 1e-5;
    let d = (c.ders)(u, v, 3);
    prop_assert_eq!(d.len(), 4);
    prop_assert!((d[0][0] - (c.point)(u, v)).length() <= 1e-14 * c.scale.max(200.0));
    let (su, sv) = (c.partials)(u, v);
    prop_assert!((su - d[1][0]).length() <= 1e-14 * c.scale.max(1.0));
    prop_assert!((sv - d[0][1]).length() <= 1e-14 * c.scale.max(1.0));
    let (dup, dum) = ((c.ders)(u + h, v, 2), (c.ders)(u - h, v, 2));
    let (dvp, dvm) = ((c.ders)(u, v + h, 2), (c.ders)(u, v - h, 2));
    for k in 0..=2 {
        for l in 0..=2 - k {
            let tol = 1e-8 * (c.scale + d[k][l].length());
            let fu = (dup[k][l] - dum[k][l]) * (0.5 / h);
            prop_assert!(
                (fu - d[k + 1][l]).length() <= tol,
                "d/du of S^({k},{l}) at ({u}, {v}): {:?} vs {:?}",
                d[k + 1][l],
                fu
            );
            let fv = (dvp[k][l] - dvm[k][l]) * (0.5 / h);
            prop_assert!(
                (fv - d[k][l + 1]).length() <= tol,
                "d/dv of S^({k},{l}) at ({u}, {v}): {:?} vs {:?}",
                d[k][l + 1],
                fv
            );
        }
    }
    let n = (c.normal)(u, v).unwrap();
    let cross = su.cross(sv);
    prop_assert!((n - cross / cross.length()).length() <= 1e-9);
    let k = (c.curvatures)(u, v).unwrap();
    prop_assert!(k.k1 >= k.k2);
    prop_assert!(k.dir1.dot(n).abs() <= 1e-12 && k.dir2.dot(n).abs() <= 1e-12);
    prop_assert!((k.dir1.cross(k.dir2) - n).length() <= 1e-12);
    let (e, f, g) = (su.dot(su), su.dot(sv), sv.dot(sv));
    let (l2, m2, n2) = (d[2][0].dot(n), d[1][1].dot(n), d[0][2].dot(n));
    let det = e * g - f * f;
    let normal_curvature = |t: Vector3| {
        let (a, b) = (t.dot(su), t.dot(sv));
        let alpha = (a * g - b * f) / det;
        let beta = (b * e - a * f) / det;
        l2 * alpha * alpha + 2.0 * m2 * alpha * beta + n2 * beta * beta
    };
    let scale = k.k1.abs().max(k.k2.abs()).max(1.0 / c.scale);
    for (dir, expected) in [(k.dir1, k.k1), (k.dir2, k.k2)] {
        let got = normal_curvature(dir);
        prop_assert!(
            (got - expected).abs() <= 1e-9 * scale,
            "normal curvature {got} vs principal {expected} at ({u}, {v})"
        );
    }
    Ok(())
}

/// The closest point to `target` against a grid of sampled parameters.
pub fn check_surface_closest(
    c: &SurfaceCase,
    target: Point3,
    us: &[f64],
    vs: &[f64],
) -> Result<SurfaceClosestPoint, TestCaseError> {
    let cp = (c.closest)(target);
    let size = c.scale.max(300.0);
    prop_assert!(((c.point)(cp.u, cp.v) - cp.point).length() <= 1e-12 * size);
    prop_assert!(((target - cp.point).length() - cp.distance).abs() <= 1e-12 * size);
    for &u in us {
        for &v in vs {
            let d = ((c.point)(u, v) - target).length();
            prop_assert!(
                cp.distance <= d + 1e-9 * size,
                "closest distance {} but the sample at ({u}, {v}) is at {d}",
                cp.distance
            );
        }
    }
    Ok(cp)
}
