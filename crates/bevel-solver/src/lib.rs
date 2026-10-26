// SPDX-License-Identifier: GPL-3.0-or-later

//! Generic nonlinear constraint solver.
//!
//! The solver works on plain variables and residual functions with analytic
//! Jacobians. It knows nothing about sketches: `bevel-sketch` translates its
//! entities and constraints into solver problems.
//!
//! Planned pipeline: split the problem into independent clusters, solve each
//! with Levenberg-Marquardt / dogleg, and report degrees of freedom and
//! redundant or conflicting constraints from the Jacobian rank.
//!
//! Status: placeholder; implemented in the "Sketcher" phase.
