// SPDX-License-Identifier: GPL-3.0-or-later

//! Units-aware expressions and parameter tables.
//!
//! Expressions such as `width * 2 + 3 mm` are parsed with a Pratt parser that
//! keeps source spans, checked for dimensional consistency (length, angle,
//! area, unitless) and evaluated against a parameter table with cycle
//! detection.
//!
//! Status: placeholder; implemented in the "Document & timeline" phase.
