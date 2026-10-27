// SPDX-License-Identifier: GPL-3.0-or-later

//! wgpu renderer for the Bevel viewport.
//!
//! The renderer draws into its own offscreen targets (multisampled colour
//! and depth, resolved into a sampled texture) that the application shows
//! as an image in the UI. Owning the targets lets later passes (id buffer
//! for picking, silhouettes, section caps) read depth and colour freely.
//!
//! This crate knows nothing about kernel or model types. It draws plain
//! triangle meshes and line sets ([`TriangleMesh`], [`LineSet`]) that carry
//! opaque `u32` ids (0 means "not pickable"); turning bodies into those
//! buffers is the application's job. Edges are drawn as thick screen-space
//! lines, faces and edges whose id is hovered or selected are tinted
//! ([`ViewportRenderer::set_highlight`]), and the id under a pixel can be
//! read back without blocking ([`ViewportRenderer::request_pick`],
//! [`ViewportRenderer::poll_pick`]).
//!
//! Conventions:
//! - World space is Z-up, right-handed, in millimetres; the ground grid lies
//!   in the XY plane.
//! - Depth is reverse-Z (1 at the near plane, 0 at infinity) with an
//!   infinite far plane, which keeps precision across the whole model space.
//! - CPU-side camera math is `f64`; values are converted to `f32` only when
//!   they are written to GPU buffers.

mod camera;
mod mesh;
mod pack;
mod pick;
mod pipelines;
mod renderer;

pub use camera::{Camera, Projection, StandardView};
pub use mesh::{LineSet, TriangleMesh};
pub use pick::PickResult;
pub use renderer::{RenderOutput, ViewportOverlays, ViewportRenderer, ViewportTheme};
