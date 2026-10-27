// SPDX-License-Identifier: GPL-3.0-or-later

//! Strategies and closed forms shared by the property tests: fixed-seed
//! configuration, random rigid frames, log-uniform lengths, and the five
//! primitives with their exact volume, area, centroid and inertia.

// Each test binary uses a different part of this module.
#![allow(dead_code, clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};
use std::sync::Arc;

use bevel_kernel::{
    Body, BoxParams, ConeParams, CylinderParams, OpContext, OpTag, SphereParams, TorusParams, box_,
    cone, cylinder, sphere, torus,
};
use bevel_math::{DMat3, DQuat, DVec3, Frame3, Point3, Vector3};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};

/// A fixed-seed configuration so that runs are reproducible.
///
/// For exploratory runs, `BEVEL_PROPTEST_SEED` replaces the seed and
/// `BEVEL_PROPTEST_CASES` the number of cases.
pub fn config(cases: u32) -> Config {
    let env = |name: &str| std::env::var(name).ok().and_then(|v| v.parse::<u64>().ok());
    Config {
        cases: env("BEVEL_PROPTEST_CASES").map_or(cases, |c| u32::try_from(c).unwrap_or(cases)),
        rng_seed: RngSeed::Fixed(env("BEVEL_PROPTEST_SEED").unwrap_or(0x00b0_d1e5)),
        failure_persistence: None,
        ..Config::default()
    }
}

/// A random rigid frame with its origin within 500 mm of the world origin.
pub fn frame() -> impl Strategy<Value = Frame3> {
    (
        prop::array::uniform3(-1.0f64..1.0),
        0.0f64..TAU,
        prop::array::uniform3(-500.0f64..500.0),
    )
        .prop_map(|(a, angle, o)| {
            let axis = DVec3::from_array(a).try_normalize().unwrap_or(DVec3::Z);
            let q = DQuat::from_axis_angle(axis, angle);
            Frame3 {
                origin: DVec3::from_array(o),
                x: q * DVec3::X,
                y: q * DVec3::Y,
                z: q * DVec3::Z,
            }
        })
}

/// A random length in millimetres, log-uniform from 1 um to 20 m, so that
/// tiny, huge and badly proportioned primitives all appear.
pub fn length() -> impl Strategy<Value = f64> {
    (-3.0f64..4.3).prop_map(|e| 10f64.powf(e))
}

/// A cone radius: zero (an apex) a quarter of the time.
pub fn cone_radius() -> impl Strategy<Value = f64> {
    prop_oneof![1 => Just(0.0), 3 => length()]
}

/// Returns `true` for cone parameters the primitive accepts (the same
/// filter as `primitive_props.rs`).
pub fn cone_ok(r0: f64, r1: f64, h: f64) -> bool {
    let dr = (r1 - r0).abs();
    dr > 1e-3 * r0.max(r1) && 4.0 * f64::EPSILON * dr * (dr / h).max(1.0) <= 1e-7
}

/// Returns `true` for torus parameters the primitive accepts.
pub fn torus_ok(big: f64, small: f64) -> bool {
    big - small > 1e-4
}

/// A primitive with its parameters.
#[derive(Clone, Copy, Debug)]
pub enum Shape {
    /// `box_` with the size.
    Box(Frame3, Vector3),
    /// `cylinder` with radius and height.
    Cylinder(Frame3, f64, f64),
    /// `sphere` with the radius.
    Sphere(Frame3, f64),
    /// `cone` with bottom radius, top radius and height.
    Cone(Frame3, f64, f64, f64),
    /// `torus` with major and minor radius.
    Torus(Frame3, f64, f64),
}

/// The exact mass properties of a primitive, in world coordinates.
#[derive(Clone, Copy, Debug)]
pub struct Exact {
    /// The volume.
    pub volume: f64,
    /// The surface area.
    pub area: f64,
    /// The centre of mass.
    pub centroid: Point3,
    /// The inertia tensor about the centroid at unit density.
    pub inertia: DMat3,
}

impl Shape {
    /// The placement frame.
    pub fn frame(&self) -> Frame3 {
        match *self {
            Self::Box(f, ..)
            | Self::Cylinder(f, ..)
            | Self::Sphere(f, ..)
            | Self::Cone(f, ..)
            | Self::Torus(f, ..) => f,
        }
    }

    /// Builds the body.
    pub fn build(&self) -> Arc<Body> {
        let cx = &mut OpContext::new();
        let tag = OpTag(1);
        let out = match *self {
            Self::Box(frame, size) => box_(&BoxParams { frame, size }, tag, cx),
            Self::Cylinder(frame, radius, height) => cylinder(
                &CylinderParams {
                    frame,
                    radius,
                    height,
                },
                tag,
                cx,
            ),
            Self::Sphere(frame, radius) => sphere(&SphereParams { frame, radius }, tag, cx),
            Self::Cone(frame, bottom_radius, top_radius, height) => cone(
                &ConeParams {
                    frame,
                    bottom_radius,
                    top_radius,
                    height,
                },
                tag,
                cx,
            ),
            Self::Torus(frame, major_radius, minor_radius) => torus(
                &TorusParams {
                    frame,
                    major_radius,
                    minor_radius,
                },
                tag,
                cx,
            ),
        };
        out.unwrap().bodies.remove(0)
    }

    /// The closed-form mass properties.
    ///
    /// In the placement frame, with `m` the volume (unit density) and the
    /// inertia about the centroid:
    ///
    /// ```text
    /// box a x b x c:
    ///   m = a b c,  area = 2 (ab + bc + ca),  centroid = (a, b, c) / 2,
    ///   I = m / 12 diag(b^2 + c^2, a^2 + c^2, a^2 + b^2)
    /// frustum, r(z) = r0 + (r1 - r0) z / h (a cylinder has r0 = r1, a cone
    /// with an apex a zero radius):
    ///   m      = pi h (r0^2 + r0 r1 + r1^2) / 3
    ///   z_c    = (int z pi r^2 dz) / m
    ///          = h (r0^2 + 2 r0 r1 + 3 r1^2) / (4 (r0^2 + r0 r1 + r1^2))
    ///   I_zz   = int pi r^4 / 2 dz
    ///          = pi h (r0^4 + r0^3 r1 + r0^2 r1^2 + r0 r1^3 + r1^4) / 10
    ///   int x^2 dV = int y^2 dV = I_zz / 2
    ///   int z^2 dV = int pi r^2 z^2 dz = pi h^3 (r0^2 + 3 r0 r1 + 6 r1^2) / 30
    ///   I_xx = I_yy = I_zz / 2 + int z^2 dV - m z_c^2   (parallel axes)
    ///   area   = pi (r0^2 + r1^2) + pi (r0 + r1) sqrt(h^2 + (r1 - r0)^2)
    /// sphere r:
    ///   m = 4 pi r^3 / 3,  area = 4 pi r^2,  I = 2 m r^2 / 5
    /// torus (R, r):
    ///   m = 2 pi^2 R r^2,  area = 4 pi^2 R r,
    ///   I_zz = m (R^2 + 3 r^2 / 4),  I_xx = I_yy = m (R^2 / 2 + 5 r^2 / 8)
    /// ```
    ///
    /// The world tensor is `F I F^T` with `F` the frame's axes as columns.
    pub fn exact(&self) -> Exact {
        let (volume, area, c, diag) = match *self {
            Self::Box(_, s) => {
                let m = s.x * s.y * s.z;
                let i = DVec3::new(
                    s.y * s.y + s.z * s.z,
                    s.x * s.x + s.z * s.z,
                    s.x * s.x + s.y * s.y,
                ) * (m / 12.0);
                (m, 2.0 * (s.x * s.y + s.y * s.z + s.z * s.x), s * 0.5, i)
            }
            Self::Cylinder(_, r, h) => frustum(r, r, h),
            Self::Cone(_, r0, r1, h) => frustum(r0, r1, h),
            Self::Sphere(_, r) => {
                let m = 4.0 * PI * r * r * r / 3.0;
                (
                    m,
                    4.0 * PI * r * r,
                    DVec3::ZERO,
                    DVec3::splat(0.4 * m * r * r),
                )
            }
            Self::Torus(_, big, small) => {
                let m = 2.0 * PI * PI * big * small * small;
                let ixx = m * (big * big / 2.0 + 5.0 * small * small / 8.0);
                let izz = m * (big * big + 0.75 * small * small);
                (
                    m,
                    4.0 * PI * PI * big * small,
                    DVec3::ZERO,
                    DVec3::new(ixx, ixx, izz),
                )
            }
        };
        let f = self.frame();
        let axes = DMat3::from_cols(f.x, f.y, f.z);
        Exact {
            volume,
            area,
            centroid: f.to_world(c),
            inertia: axes * DMat3::from_diagonal(diag) * axes.transpose(),
        }
    }
}

/// `(volume, area, centroid, principal inertia)` of a frustum in its frame.
fn frustum(r0: f64, r1: f64, h: f64) -> (f64, f64, DVec3, DVec3) {
    let s2 = r0 * r0 + r0 * r1 + r1 * r1;
    let m = PI * h * s2 / 3.0;
    let zc = h * (r0 * r0 + 2.0 * r0 * r1 + 3.0 * r1 * r1) / (4.0 * s2);
    let s4 = r0.powi(4) + r0.powi(3) * r1 + r0 * r0 * r1 * r1 + r0 * r1.powi(3) + r1.powi(4);
    let izz = PI * h * s4 / 10.0;
    let zz = PI * h.powi(3) * (r0 * r0 + 3.0 * r0 * r1 + 6.0 * r1 * r1) / 30.0;
    let ixx = izz / 2.0 + zz - m * zc * zc;
    let area = PI * (r0 * r0 + r1 * r1) + PI * (r0 + r1) * h.hypot(r1 - r0);
    (m, area, DVec3::new(0.0, 0.0, zc), DVec3::new(ixx, ixx, izz))
}

/// The largest absolute entry of a matrix.
pub fn max_abs(m: &DMat3) -> f64 {
    m.to_cols_array()
        .iter()
        .fold(0.0f64, |a, &b| a.max(b.abs()))
}
