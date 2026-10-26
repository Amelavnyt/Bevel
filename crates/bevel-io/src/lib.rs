// SPDX-License-Identifier: GPL-3.0-or-later

//! Mesh and CAD exchange formats.
//!
//! STL and 3MF export of tessellated bodies, and STEP (ISO 10303-21) import
//! and export of exact B-rep bodies. This crate depends only on kernel-level
//! crates; importing produces kernel bodies that the application turns into
//! an `ImportBody` command.
//!
//! Status: placeholder; STL and 3MF arrive in the "B-rep & display" phase,
//! STEP in the "STEP" phase.
