// SPDX-License-Identifier: GPL-3.0-or-later

//! 2D sketches.
//!
//! Sketch entities, geometric constraints and dimensions, the sketch's own
//! timeline of sketch ops, and profile regions built from the solved
//! geometry. A sketch knows nothing about 3D bodies: its plane and projected
//! geometry are handed in as plain inputs by the sketch feature in
//! `bevel-model`.
//!
//! Status: placeholder; implemented in the "Sketcher" phase.
