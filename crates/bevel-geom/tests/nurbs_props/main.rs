// SPDX-License-Identifier: GPL-3.0-or-later

//! Property tests for the NURBS core: random valid knot vectors, curves
//! (2D and 3D, degrees 1 to 5) and surfaces (degrees 1 to 4 in each
//! direction), rational and not, with repeated interior knots, all with a
//! fixed seed.

// Helpers outside `#[test]` functions unwrap freely too.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod basis;
mod degree;
mod eval;
mod knot_ops;
mod measure;
mod strategies;
mod structure;
mod surface_eval;
