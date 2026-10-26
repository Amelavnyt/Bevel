// SPDX-License-Identifier: GPL-3.0-or-later

//! Vectors, transforms, bounding boxes and tolerances for Bevel.
//!
//! All geometry uses `f64`; lengths are in millimetres and angles in
//! radians. The vector types are [glam]'s double-precision types behind
//! names that say what a value means (`Point3` vs `Vector3`).
//!
//! Status: the type aliases, tolerances, bounding boxes ([`Aabb2`],
//! [`Aabb3`]), [`Interval`], coordinate frames ([`Frame3`]) and
//! [`MathError`] exist, together with the numerical building blocks:
//! robust geometric [`predicates`], polynomial [`roots`], damped
//! [`newton`] iteration and Gauss–Legendre [`quadrature`]. Curves, surfaces
//! and NURBS evaluation build on these in the "Math & NURBS" phase of the
//! roadmap.

pub mod newton;
pub mod predicates;
pub mod quadrature;
pub mod roots;
pub mod tol;

mod aabb;
mod error;
mod frame;
mod interval;

pub use aabb::{Aabb2, Aabb3};
pub use error::MathError;
pub use frame::Frame3;
pub use glam::{DMat2, DMat3, DMat4, DQuat, DVec2, DVec3, DVec4};
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
