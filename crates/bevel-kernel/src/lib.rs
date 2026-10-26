// SPDX-License-Identifier: GPL-3.0-or-later

//! Boundary-representation modeling kernel.
//!
//! Arena-based B-rep topology (vertices, edges, coedges, loops, faces,
//! shells, solids), validation, modeling operations, booleans, blends,
//! tessellation, mass properties and persistent topological naming.
//!
//! The kernel knows nothing about features, sketches or documents. Callers
//! pass an opaque `OpTag` so the kernel can name what it creates.
//!
//! Status: placeholder; topology and primitives arrive in the
//! "B-rep & display" phase.
