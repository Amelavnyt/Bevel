// SPDX-License-Identifier: GPL-3.0-or-later

//! Property tests for curve fitting: random point sets (2D and 3D, 4 to 40
//! points, distinct consecutive points), degrees 1 to 5 and all three
//! parametrizations, with a fixed seed.

// Helpers outside `#[test]` functions unwrap freely too.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod approximate;
mod interpolate;
mod strategies;
