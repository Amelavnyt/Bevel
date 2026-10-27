// SPDX-License-Identifier: GPL-3.0-or-later

//! Expression errors.

use std::ops::Range;

use thiserror::Error;

use crate::units::Dim;

/// An error with the byte span of the source text it refers to.
#[derive(Clone, Debug, PartialEq, Error)]
#[error("{kind}")]
pub struct ExprError {
    /// Byte range in the expression source.
    pub span: Range<usize>,
    /// What went wrong.
    pub kind: ExprErrorKind,
}

impl ExprError {
    pub(crate) fn new(span: Range<usize>, kind: ExprErrorKind) -> Self {
        Self { span, kind }
    }
}

fn unexpected(found: &str, expected: Option<&str>) -> String {
    match expected {
        Some(e) => format!("expected {e}, found `{found}`"),
        None => format!("unexpected `{found}`"),
    }
}

/// The kinds of expression error.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum ExprErrorKind {
    /// A character that is not part of the syntax.
    #[error("unexpected character `{0}`")]
    UnexpectedChar(char),
    /// The expression ends where more input is needed.
    #[error("unexpected end of expression")]
    UnexpectedEnd,
    /// A token in the wrong place.
    #[error("{}", unexpected(.found, *.expected))]
    UnexpectedToken {
        /// The offending source text.
        found: String,
        /// What would have been accepted, if a single thing.
        expected: Option<&'static str>,
    },
    /// A call to a function that does not exist.
    #[error("unknown function `{0}`")]
    UnknownFunction(String),
    /// A function called with the wrong number of arguments.
    #[error("`{name}` takes {expected} argument(s), found {found}")]
    WrongArgCount {
        /// Function name.
        name: String,
        /// Accepted count, e.g. `2` or `2 or more`.
        expected: String,
        /// Number given.
        found: usize,
    },
    /// A parameter name with no definition.
    #[error("unknown parameter `{0}`")]
    UnknownParameter(String),
    /// A result of the wrong dimension for its field.
    #[error("expected {expected}, found {found}")]
    DimensionMismatch {
        /// Dimension the field needs.
        expected: Dim,
        /// Dimension the expression produced.
        found: Dim,
    },
    /// Operands that an operator or function cannot take.
    #[error("{0}")]
    InvalidOperand(String),
    /// Division by zero.
    #[error("division by zero")]
    DivisionByZero,
    /// An argument outside the function's domain.
    #[error("{0}")]
    Domain(String),
    /// A result that is infinite or not a number.
    #[error("result is not a finite number")]
    NonFinite,
    /// Nesting beyond the supported depth.
    #[error("expression is nested too deeply")]
    TooDeep,
    /// A parameter that depends on itself, listed in cycle order.
    #[error("circular reference: {}", .0.join(" -> "))]
    Cycle(Vec<String>),
    /// A parameter that reads another parameter which failed.
    #[error("depends on `{0}`, which has an error")]
    DependsOnError(String),
}
