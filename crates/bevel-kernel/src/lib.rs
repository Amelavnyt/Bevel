// SPDX-License-Identifier: GPL-3.0-or-later

//! Boundary-representation modeling kernel.
//!
//! Arena-based B-rep topology (vertices, edges, coedges, loops, faces,
//! shells, solids), validation, modeling operations, booleans, blends,
//! tessellation, mass properties and persistent topological naming.
//!
//! The kernel knows nothing about features, sketches or documents. Callers
//! pass an opaque [`OpTag`] so the kernel can name what it creates.
//!
//! # Status
//!
//! Implemented:
//!
//! - [`topology`]: the B-rep data structure ([`Body`] and its entities,
//!   typed slotmap keys, read-only accessors and adjacency queries,
//!   bounds, Euler-Poincare counts) and its conventions: orientation,
//!   pcurves sharing the edge parameter, seams on periodic faces,
//!   degenerate edges at poles and apexes, loop ordering and determinism.
//! - [`naming`]: persistent names ([`TopoName`], [`Source`], [`Role`]),
//!   operation [`History`], [`OpOutput`] and [`OpContext`] with its
//!   tolerance and cancellation token.
//! - [`builder`]: [`BodyBuilder`], which keeps references consistent at
//!   every step and validates on [`BodyBuilder::finish`].
//! - [`validate`](mod@validate): [`validate`](fn@validate) and the [`Issue`]s it reports
//!   (references, loop closure in 3D and in `(u, v)`, 2-manifold closed
//!   shells, orientation, pcurve and vertex agreement, seams, degenerate
//!   edges, Euler-Poincare, unique names).
//! - [`primitives`]: box, cylinder, sphere, cone and torus with exact
//!   analytic geometry, outward normals and named faces, edges and
//!   vertices.
//! - Serialization: bodies serialize with serde in a flat, index-based
//!   form and re-validate on deserialization, so a corrupt file never
//!   produces an invalid body (see "Serialization" on [`Body`]). Names,
//!   histories and primitive parameters serialize too.
//!
//! Next: tessellation for display, then extrude, revolve and booleans.

pub mod builder;
pub mod naming;
pub mod primitives;
pub mod topology;
pub mod validate;

mod error;
mod serial;
#[cfg(test)]
mod tests;

pub use builder::BodyBuilder;
pub use error::KernelError;
pub use naming::{
    EntityKind, History, HistoryEntry, OpContext, OpOutput, OpTag, Role, Source, TopoName,
};
pub use primitives::{
    BoxParams, ConeParams, CylinderParams, SphereParams, TorusParams, box_, cone, cylinder, sphere,
    torus,
};
pub use topology::{
    Body, BodyKind, Coedge, CoedgeKey, Edge, EdgeKey, EntityKey, EulerCounts, Face, FaceKey, Loop,
    LoopKey, Shell, ShellKey, Vertex, VertexKey,
};
pub use validate::{Issue, validate};
