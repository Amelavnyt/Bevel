// SPDX-License-Identifier: GPL-3.0-or-later

//! Mesh and CAD exchange formats.
//!
//! STL and 3MF export of tessellated bodies, and STEP (ISO 10303-21) import
//! and export of exact B-rep bodies. This crate depends only on kernel-level
//! crates; importing produces kernel bodies that the application turns into
//! an `ImportBody` command.
//!
//! Status: the plain mesh type ([`ExportMesh`]) and the STL writers and
//! reader ([`stl`]) exist. The writers take a kernel-independent mesh, so an
//! adapter from kernel tessellations can be added without touching them.
//! 3MF arrives next; STEP follows in the "STEP" phase.
//!
//! All coordinates handed to the writers are in millimetres.

mod error;
mod mesh;
pub mod stl;

#[cfg(test)]
mod test_util;

pub use error::IoError;
pub use mesh::ExportMesh;
