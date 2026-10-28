// SPDX-License-Identifier: GPL-3.0-or-later

//! Tests for curve–curve and curve–surface intersection: known
//! configurations, the closed-form fast paths cross-checked against the
//! general method on the same geometry converted to NURBS, and properties
//! of random line and circle pairs drawn with a fixed seed.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    reason = "tests"
)]

mod curves;
mod helpers;
mod props;
mod surfaces;
