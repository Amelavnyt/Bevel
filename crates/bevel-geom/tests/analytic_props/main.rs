// SPDX-License-Identifier: GPL-3.0-or-later

//! Tests for the analytic curves and surfaces: closed forms against finite
//! differences and dense sampling, with random frames, radii and
//! parameters drawn with a fixed seed.

// Helpers outside `#[test]` functions unwrap freely too.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod curves;
mod strategies;
