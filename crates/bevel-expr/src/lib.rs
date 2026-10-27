// SPDX-License-Identifier: GPL-3.0-or-later

//! Units-aware expressions and parameter tables.
//!
//! Expressions such as `width * 2 + 3 mm` are parsed with a Pratt parser that
//! keeps source spans, checked for dimensional consistency (length, angle,
//! area, unitless) and evaluated against a parameter table with cycle
//! detection.
//!
//! Internally every length is in millimetres and every angle in radians; a
//! [`Quantity`] pairs such a value with a [`Dim`]. A unit symbol (`mm cm m um
//! in ft deg rad`) binds to the number literal directly before it, so
//! `10 mm ^ 2` is `(10 mm)^2` and `16 mm^2` is `(16 mm)^2`, an area of 256.
//!
//! Parameter values are evaluated with [`Expr::eval`], so a bare `40` stays
//! unitless. A feature field that reads it later calls [`eval_as`], which
//! applies the document's default unit at that point.

mod error;
mod eval;
mod lexer;
mod params;
mod parser;
mod units;

pub use error::{ExprError, ExprErrorKind};
pub use eval::{EvalContext, Expr, eval_as};
pub use params::{Evaluated, ParamError, ParamKind, Parameter, ParameterTable};
pub use units::{AngleUnit, Dim, LengthUnit, Quantity, Units, format_quantity};
