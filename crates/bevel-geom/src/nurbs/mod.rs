// SPDX-License-Identifier: GPL-3.0-or-later

//! Non-uniform rational B-splines.

mod curve;
mod surface;

pub use curve::{ClosestPoint, NurbsCurve, NurbsCurve2, NurbsCurve3, make_compatible};
pub use surface::{Curvatures, NurbsSurface, SurfaceClosestPoint};
