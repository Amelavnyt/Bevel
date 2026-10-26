// SPDX-License-Identifier: GPL-3.0-or-later

//! Parts, features and regeneration.
//!
//! A part owns its features, sketches, reference geometry and parameters.
//! Regeneration replays the part timeline up to the rollback marker and
//! produces a model state: bodies with persistent names, solved sketches
//! and diagnostics. Stored geometry references (`EntityRef`) are resolved
//! here.
//!
//! Status: placeholder; implemented in the "Document & timeline" phase.
