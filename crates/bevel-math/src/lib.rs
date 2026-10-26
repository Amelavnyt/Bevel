// SPDX-License-Identifier: GPL-3.0-or-later

//! Vectors, transforms, bounding boxes and tolerances for Bevel.
//!
//! All geometry uses `f64`; lengths are in millimetres and angles in
//! radians. The vector types are [glam]'s double-precision types behind
//! names that say what a value means (`Point3` vs `Vector3`).
//!
//! Status: the type aliases, tolerances and [`Aabb3`] exist; intervals,
//! robust predicates, root finding, Newton iteration and quadrature arrive
//! in the "Math & NURBS" phase of the roadmap.

pub mod predicates;
pub mod roots;
pub mod tol;

mod aabb;
mod error;
mod frame;
mod interval;

pub use aabb::{Aabb2, Aabb3};
pub use error::MathError;
pub use frame::Frame3;
pub use glam::{DMat3, DMat4, DQuat, DVec2, DVec3, DVec4};
pub use interval::Interval;

/// View and projection matrix constructors (glam's camera module).
pub use glam::dcamera;

/// A position in 2D (sketch or parameter space), in millimetres.
pub type Point2 = DVec2;
/// A displacement or direction in 2D.
pub type Vector2 = DVec2;
/// A position in 3D model space, in millimetres.
pub type Point3 = DVec3;
/// A displacement or direction in 3D.
pub type Vector3 = DVec3;
/// An affine transform of 3D model space.
pub type Transform3 = DMat4;
