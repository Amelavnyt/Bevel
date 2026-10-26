// SPDX-License-Identifier: GPL-3.0-or-later

//! Curves and surfaces for Bevel.
//!
//! Every curve and surface is exactly representable as a (rational) NURBS,
//! while analytic types (lines, circles, planes, cylinders, ...) keep their
//! identity so they can be intersected, measured and exported exactly.
//! Trims and parameter ranges live in the topology (`bevel-kernel`), not here.
//!
//! Status: placeholder. The NURBS core and analytic types arrive in the
//! "Math & NURBS" phase of the roadmap; intersections arrive with booleans.
