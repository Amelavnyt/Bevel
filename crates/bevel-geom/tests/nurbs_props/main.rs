// SPDX-License-Identifier: GPL-3.0-or-later

//! Property tests for the NURBS core: random valid knot vectors and curves
//! (2D and 3D, degrees 1 to 5, rational and not, repeated interior knots)
//! with a fixed seed.

// Helpers outside `#[test]` functions unwrap freely too.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod basis;
mod eval;
mod knot_ops;
mod strategies;
mod structure;
