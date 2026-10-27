// SPDX-License-Identifier: GPL-3.0-or-later

//! Expressions and their dimension-checked evaluation.

use std::collections::BTreeSet;
use std::ops::Range;

use crate::error::{ExprError, ExprErrorKind};
use crate::parser::{BinOp, Kind, Node, parse};
use crate::units::{AngleUnit, Dim, LengthUnit, Quantity, Units};

/// A parsed expression: its source text and syntax tree.
#[derive(Clone, Debug, PartialEq)]
pub struct Expr {
    source: String,
    root: Node,
}

/// What an expression can read while it is evaluated.
pub struct EvalContext<'a> {
    /// Document default units.
    pub units: Units,
    /// Returns a parameter's value, or `None` if the name is unknown.
    pub lookup: &'a dyn Fn(&str) -> Option<Quantity>,
}

impl Expr {
    /// Parses `source`.
    ///
    /// # Errors
    /// Returns a syntax error with the span of the offending text.
    pub fn parse(source: &str) -> Result<Expr, ExprError> {
        let root = parse(source)?;
        Ok(Expr {
            source: source.to_owned(),
            root,
        })
    }

    /// The source text.
    #[must_use]
    pub fn source(&self) -> &str {
        &self.source
    }

    /// Parameter names the expression reads, sorted, without duplicates
    /// (never includes `pi`).
    #[must_use]
    pub fn identifiers(&self) -> BTreeSet<String> {
        let mut out = BTreeSet::new();
        self.root.visit_idents(&mut |n, _| {
            out.insert(n.to_owned());
        });
        out
    }

    /// Every identifier occurrence with its byte span, in source order.
    pub(crate) fn identifier_spans(&self) -> Vec<(String, Range<usize>)> {
        let mut out = Vec::new();
        self.root
            .visit_idents(&mut |n, s| out.push((n.to_owned(), s.clone())));
        out.sort_by_key(|(_, s)| s.start);
        out
    }

    /// Evaluates the expression. A bare number stays unitless; see
    /// [`eval_as`] for fields that apply default units.
    ///
    /// # Errors
    /// Returns the first dimension, domain or lookup error, with its span.
    pub fn eval(&self, cx: &EvalContext<'_>) -> Result<Quantity, ExprError> {
        eval_node(&self.root, cx)
    }
}

/// Evaluates `expr` for a field that expects `expected`.
///
/// A unitless result in a length field is read in `units.length`, in an angle
/// field in `units.angle` (typing `40` in a length field means 40 mm when the
/// document is in mm). Any other mismatch is an error spanning the whole
/// expression.
///
/// # Errors
/// Evaluation errors, or [`ExprErrorKind::DimensionMismatch`].
pub fn eval_as(expr: &Expr, expected: Dim, cx: &EvalContext<'_>) -> Result<Quantity, ExprError> {
    let q = expr.eval(cx)?;
    let whole = 0..expr.source.len();
    let q = if q.dim == expected {
        q
    } else if q.dim == Dim::UNITLESS && expected == Dim::LENGTH {
        Quantity::length(cx.units.length.to_internal(q.value))
    } else if q.dim == Dim::UNITLESS && expected == Dim::ANGLE {
        Quantity::angle(cx.units.angle.to_internal(q.value))
    } else {
        return Err(ExprError::new(
            whole,
            ExprErrorKind::DimensionMismatch {
                expected,
                found: q.dim,
            },
        ));
    };
    finite(q, whole)
}

fn finite(q: Quantity, span: Range<usize>) -> Result<Quantity, ExprError> {
    if q.value.is_finite() {
        Ok(q)
    } else {
        Err(ExprError::new(span, ExprErrorKind::NonFinite))
    }
}

fn err<T>(span: &Range<usize>, kind: ExprErrorKind) -> Result<T, ExprError> {
    Err(ExprError::new(span.clone(), kind))
}

fn eval_node(n: &Node, cx: &EvalContext<'_>) -> Result<Quantity, ExprError> {
    match &n.kind {
        Kind::Num(q) => Ok(*q),
        Kind::Ident(name) => match (cx.lookup)(name) {
            Some(q) => finite(q, n.span.clone()),
            None => err(&n.span, ExprErrorKind::UnknownParameter(name.clone())),
        },
        Kind::Neg(a) => {
            let q = eval_node(a, cx)?;
            Ok(Quantity {
                value: -q.value,
                dim: q.dim,
            })
        }
        Kind::Pos(a) => eval_node(a, cx),
        Kind::Binary { op, lhs, rhs } => {
            let l = eval_node(lhs, cx)?;
            let r = eval_node(rhs, cx)?;
            let q = binary(*op, l, r, &n.span)?;
            finite(q, n.span.clone())
        }
        Kind::Call { name, args } => {
            let vals = args
                .iter()
                .map(|a| eval_node(a, cx))
                .collect::<Result<Vec<_>, _>>()?;
            let q = call(name, &vals, cx.units, &n.span)?;
            finite(q, n.span.clone())
        }
    }
}

fn too_large<T>(span: &Range<usize>) -> Result<T, ExprError> {
    err(
        span,
        ExprErrorKind::InvalidOperand("dimension exponent is too large".to_owned()),
    )
}

fn combine(a: Dim, b: Dim, sign: i8, span: &Range<usize>) -> Result<Dim, ExprError> {
    let l = sign
        .checked_mul(b.length)
        .and_then(|bl| a.length.checked_add(bl));
    let g = sign
        .checked_mul(b.angle)
        .and_then(|ba| a.angle.checked_add(ba));
    match (l, g) {
        (Some(length), Some(angle)) => Ok(Dim { length, angle }),
        _ => too_large(span),
    }
}

fn binary(op: BinOp, l: Quantity, r: Quantity, span: &Range<usize>) -> Result<Quantity, ExprError> {
    match op {
        BinOp::Add | BinOp::Sub => {
            if l.dim != r.dim {
                let msg = if op == BinOp::Add {
                    format!("cannot add {} and {}", l.dim, r.dim)
                } else {
                    format!("cannot subtract {} from {}", r.dim, l.dim)
                };
                return err(span, ExprErrorKind::InvalidOperand(msg));
            }
            let value = if op == BinOp::Add {
                l.value + r.value
            } else {
                l.value - r.value
            };
            Ok(Quantity { value, dim: l.dim })
        }
        BinOp::Mul => Ok(Quantity {
            value: l.value * r.value,
            dim: combine(l.dim, r.dim, 1, span)?,
        }),
        BinOp::Div => {
            if r.value.abs() < f64::MIN_POSITIVE {
                return err(span, ExprErrorKind::DivisionByZero);
            }
            Ok(Quantity {
                value: l.value / r.value,
                dim: combine(l.dim, r.dim, -1, span)?,
            })
        }
        BinOp::Pow => power(l, r, span),
    }
}

fn power(base: Quantity, exp: Quantity, span: &Range<usize>) -> Result<Quantity, ExprError> {
    if exp.dim != Dim::UNITLESS {
        return err(
            span,
            ExprErrorKind::InvalidOperand("exponent must be unitless".to_owned()),
        );
    }
    let zero_base = base.value.abs() < f64::MIN_POSITIVE;
    if zero_base && exp.value < 0.0 {
        return err(span, ExprErrorKind::DivisionByZero);
    }
    let value = base.value.powf(exp.value);
    if value.is_nan() {
        return err(
            span,
            ExprErrorKind::Domain("power of a negative number needs a whole exponent".to_owned()),
        );
    }
    if base.dim == Dim::UNITLESS {
        return Ok(Quantity::unitless(value));
    }
    let n = exp.value.round();
    if (exp.value - n).abs() > 1e-9 {
        return err(
            span,
            ExprErrorKind::InvalidOperand(format!(
                "exponent of a {} must be a whole number",
                base.dim
            )),
        );
    }
    if n.abs() > 127.0 {
        return too_large(span);
    }
    #[allow(clippy::cast_possible_truncation, reason = "range checked above")]
    let n = n as i8;
    match (
        base.dim.length.checked_mul(n),
        base.dim.angle.checked_mul(n),
    ) {
        (Some(length), Some(angle)) => Ok(Quantity {
            value,
            dim: Dim { length, angle },
        }),
        _ => too_large(span),
    }
}

/// Splits `args` into exactly `N` values, or reports a wrong count.
fn exactly<const N: usize>(
    name: &str,
    args: &[Quantity],
    span: &Range<usize>,
) -> Result<[Quantity; N], ExprError> {
    <[Quantity; N]>::try_from(args).or_else(|_| {
        err(
            span,
            ExprErrorKind::WrongArgCount {
                name: name.to_owned(),
                expected: N.to_string(),
                found: args.len(),
            },
        )
    })
}

fn call(
    name: &str,
    args: &[Quantity],
    units: Units,
    span: &Range<usize>,
) -> Result<Quantity, ExprError> {
    match name {
        "sin" | "cos" | "tan" => {
            let [a] = exactly::<1>(name, args, span)?;
            let rad = if a.dim == Dim::ANGLE {
                a.value
            } else if a.dim == Dim::UNITLESS {
                units.angle.to_internal(a.value)
            } else {
                return err(
                    span,
                    ExprErrorKind::InvalidOperand(format!(
                        "{name} needs an angle or a plain number, found {}",
                        a.dim
                    )),
                );
            };
            let v = match name {
                "sin" => rad.sin(),
                "cos" => rad.cos(),
                _ => rad.tan(),
            };
            Ok(Quantity::unitless(v))
        }
        "asin" | "acos" | "atan" => {
            let [a] = exactly::<1>(name, args, span)?;
            if a.dim != Dim::UNITLESS {
                return err(
                    span,
                    ExprErrorKind::InvalidOperand(format!(
                        "{name} needs a plain number, found {}",
                        a.dim
                    )),
                );
            }
            if name != "atan" && a.value.abs() > 1.0 {
                return err(
                    span,
                    ExprErrorKind::Domain(format!("{name} needs a value between -1 and 1")),
                );
            }
            let v = match name {
                "asin" => a.value.asin(),
                "acos" => a.value.acos(),
                _ => a.value.atan(),
            };
            Ok(Quantity::angle(v))
        }
        "atan2" => {
            let [y, x] = exactly::<2>(name, args, span)?;
            if y.dim != x.dim {
                return err(
                    span,
                    ExprErrorKind::InvalidOperand(format!(
                        "atan2 needs matching dimensions, found {} and {}",
                        y.dim, x.dim
                    )),
                );
            }
            Ok(Quantity::angle(y.value.atan2(x.value)))
        }
        "sqrt" => {
            let [a] = exactly::<1>(name, args, span)?;
            sqrt(a, span)
        }
        "abs" => {
            let [a] = exactly::<1>(name, args, span)?;
            Ok(Quantity {
                value: a.value.abs(),
                dim: a.dim,
            })
        }
        "round" | "floor" | "ceil" => {
            let [a] = exactly::<1>(name, args, span)?;
            let f: fn(f64) -> f64 = match name {
                "round" => f64::round,
                "floor" => f64::floor,
                _ => f64::ceil,
            };
            Ok(Quantity {
                value: round_in_default(a, units, f),
                dim: a.dim,
            })
        }
        "min" | "max" => min_max(name, args, span),
        _ => err(span, ExprErrorKind::UnknownFunction(name.to_owned())),
    }
}

fn sqrt(a: Quantity, span: &Range<usize>) -> Result<Quantity, ExprError> {
    if a.dim.length % 2 != 0 || a.dim.angle % 2 != 0 {
        return err(
            span,
            ExprErrorKind::InvalidOperand(format!("sqrt needs an even power of {}", a.dim)),
        );
    }
    if a.value < 0.0 {
        return err(
            span,
            ExprErrorKind::Domain("sqrt of a negative number".to_owned()),
        );
    }
    Ok(Quantity {
        value: a.value.sqrt(),
        dim: Dim {
            length: a.dim.length / 2,
            angle: a.dim.angle / 2,
        },
    })
}

fn min_max(name: &str, args: &[Quantity], span: &Range<usize>) -> Result<Quantity, ExprError> {
    let Some((first, rest)) = args.split_first() else {
        return err(
            span,
            ExprErrorKind::WrongArgCount {
                name: name.to_owned(),
                expected: "2 or more".to_owned(),
                found: 0,
            },
        );
    };
    let mut acc = first.value;
    for q in rest {
        if q.dim != first.dim {
            return err(
                span,
                ExprErrorKind::InvalidOperand(format!(
                    "{name} needs matching dimensions, found {} and {}",
                    first.dim, q.dim
                )),
            );
        }
        acc = if name == "min" {
            acc.min(q.value)
        } else {
            acc.max(q.value)
        };
    }
    Ok(Quantity {
        value: acc,
        dim: first.dim,
    })
}

/// Applies `f` to the value expressed in the default unit for length and
/// angle quantities, and to the internal value otherwise.
fn round_in_default(a: Quantity, units: Units, f: fn(f64) -> f64) -> f64 {
    if a.dim == Dim::LENGTH {
        let u: LengthUnit = units.length;
        u.to_internal(f(u.from_internal(a.value)))
    } else if a.dim == Dim::ANGLE {
        let u: AngleUnit = units.angle;
        u.to_internal(f(u.from_internal(a.value)))
    } else {
        f(a.value)
    }
}
