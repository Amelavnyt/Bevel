// SPDX-License-Identifier: GPL-3.0-or-later

//! Generic history timeline.
//!
//! One engine serves both the part timeline (features) and every sketch
//! timeline (sketch ops): ordered steps, a rollback marker, suppression,
//! dependency-checked reordering and a replay cache keyed by step content.
//!
//! Status: placeholder; implemented in the "Document & timeline" phase.
