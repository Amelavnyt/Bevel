// SPDX-License-Identifier: GPL-3.0-or-later

//! Pratt parser producing a span-carrying AST.

use std::ops::Range;

use crate::error::{ExprError, ExprErrorKind};
use crate::lexer::{Tok, Token, UnitSym, lex};
use crate::units::Quantity;

/// Maximum nesting of parentheses, unary chains and calls.
pub(crate) const MAX_DEPTH: usize = 256;
/// Maximum height of the syntax tree (bounds evaluation recursion).
pub(crate) const MAX_HEIGHT: usize = 400;

/// Accepted argument counts `(min, max)` of a function, `None` max = unbounded.
fn arity(name: &str) -> Option<(usize, Option<usize>)> {
    match name {
        "sin" | "cos" | "tan" | "asin" | "acos" | "atan" | "sqrt" | "abs" | "round" | "floor"
        | "ceil" => Some((1, Some(1))),
        "atan2" => Some((2, Some(2))),
        "min" | "max" => Some((2, None)),
        _ => None,
    }
}

/// Whether `name` is a built-in function (or the constant `pi`).
pub(crate) fn is_reserved(name: &str) -> bool {
    name == "pi" || arity(name).is_some()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BinOp {
    Add,
    Sub,
    Mul,
    Div,
    Pow,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Kind {
    Num(Quantity),
    Ident(String),
    Neg(Box<Node>),
    Pos(Box<Node>),
    Binary {
        op: BinOp,
        lhs: Box<Node>,
        rhs: Box<Node>,
    },
    Call {
        name: String,
        args: Vec<Node>,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Node {
    pub span: Range<usize>,
    pub height: usize,
    pub kind: Kind,
}

impl Node {
    fn new(span: Range<usize>, height: usize, kind: Kind) -> Result<Self, ExprError> {
        if height > MAX_HEIGHT {
            return Err(ExprError::new(span, ExprErrorKind::TooDeep));
        }
        Ok(Self { span, height, kind })
    }

    /// Calls `f` for every identifier node with its name and span.
    pub fn visit_idents<'a>(&'a self, f: &mut impl FnMut(&'a str, &Range<usize>)) {
        match &self.kind {
            Kind::Num(_) => {}
            Kind::Ident(n) => f(n, &self.span),
            Kind::Neg(a) | Kind::Pos(a) => a.visit_idents(f),
            Kind::Binary { lhs, rhs, .. } => {
                lhs.visit_idents(f);
                rhs.visit_idents(f);
            }
            Kind::Call { args, .. } => {
                for a in args {
                    a.visit_idents(f);
                }
            }
        }
    }
}

const PREFIX_BP: u8 = 30;

struct Parser<'a> {
    src: &'a str,
    toks: Vec<Token>,
    pos: usize,
    depth: usize,
}

impl Parser<'_> {
    fn peek(&self) -> Option<&Token> {
        self.toks.get(self.pos)
    }

    fn end_error(&self) -> ExprError {
        ExprError::new(self.src.len()..self.src.len(), ExprErrorKind::UnexpectedEnd)
    }

    fn token_error(&self, t: &Token, expected: Option<&'static str>) -> ExprError {
        let found = self.src.get(t.span.clone()).unwrap_or_default().to_owned();
        ExprError::new(
            t.span.clone(),
            ExprErrorKind::UnexpectedToken { found, expected },
        )
    }

    fn expr(&mut self, min_bp: u8) -> Result<Node, ExprError> {
        self.depth += 1;
        let r = if self.depth > MAX_DEPTH {
            let span = self
                .peek()
                .map_or(self.src.len()..self.src.len(), |t| t.span.clone());
            Err(ExprError::new(span, ExprErrorKind::TooDeep))
        } else {
            self.expr_inner(min_bp)
        };
        self.depth -= 1;
        r
    }

    fn expr_inner(&mut self, min_bp: u8) -> Result<Node, ExprError> {
        let mut lhs = self.prefix()?;
        while let Some(t) = self.peek() {
            let (op, lbp, rbp) = match t.tok {
                Tok::Plus => (BinOp::Add, 10, 11),
                Tok::Minus => (BinOp::Sub, 10, 11),
                Tok::Star => (BinOp::Mul, 20, 21),
                Tok::Slash => (BinOp::Div, 20, 21),
                Tok::Caret => (BinOp::Pow, 40, 39),
                _ => break,
            };
            if lbp < min_bp {
                break;
            }
            self.pos += 1;
            let rhs = self.expr(rbp)?;
            let span = lhs.span.start..rhs.span.end;
            let height = 1 + lhs.height.max(rhs.height);
            lhs = Node::new(
                span,
                height,
                Kind::Binary {
                    op,
                    lhs: Box::new(lhs),
                    rhs: Box::new(rhs),
                },
            )?;
        }
        Ok(lhs)
    }

    fn prefix(&mut self) -> Result<Node, ExprError> {
        let Some(t) = self.toks.get(self.pos).cloned() else {
            return Err(self.end_error());
        };
        self.pos += 1;
        match t.tok {
            Tok::Num(v, unit) => {
                let q = match unit {
                    None => Quantity::unitless(v),
                    Some(UnitSym::Length(u)) => Quantity::length(u.to_internal(v)),
                    Some(UnitSym::Angle(u)) => Quantity::angle(u.to_internal(v)),
                };
                if !q.value.is_finite() {
                    return Err(ExprError::new(t.span, ExprErrorKind::NonFinite));
                }
                Node::new(t.span, 1, Kind::Num(q))
            }
            Tok::Ident(name) => {
                if matches!(self.peek().map(|n| &n.tok), Some(Tok::LParen)) {
                    self.call(name, t.span)
                } else if name == "pi" {
                    let q = Quantity::unitless(std::f64::consts::PI);
                    Node::new(t.span, 1, Kind::Num(q))
                } else {
                    Node::new(t.span, 1, Kind::Ident(name))
                }
            }
            Tok::Minus | Tok::Plus => {
                let operand = self.expr(PREFIX_BP)?;
                let span = t.span.start..operand.span.end;
                let height = operand.height + 1;
                let operand = Box::new(operand);
                let kind = if t.tok == Tok::Minus {
                    Kind::Neg(operand)
                } else {
                    Kind::Pos(operand)
                };
                Node::new(span, height, kind)
            }
            Tok::LParen => {
                let inner = self.expr(0)?;
                match self.toks.get(self.pos).cloned() {
                    Some(close) if close.tok == Tok::RParen => {
                        self.pos += 1;
                        Node::new(t.span.start..close.span.end, inner.height, inner.kind)
                    }
                    Some(other) => Err(self.token_error(&other, Some("`)`"))),
                    None => Err(self.end_error()),
                }
            }
            _ => Err(self.token_error(&t, None)),
        }
    }

    fn call(&mut self, name: String, name_span: Range<usize>) -> Result<Node, ExprError> {
        let Some((min, max)) = arity(&name) else {
            return Err(ExprError::new(
                name_span,
                ExprErrorKind::UnknownFunction(name),
            ));
        };
        self.pos += 1; // `(`
        let mut args = Vec::new();
        if matches!(self.peek().map(|n| &n.tok), Some(Tok::RParen)) {
            // no arguments
        } else {
            loop {
                args.push(self.expr(0)?);
                match self.toks.get(self.pos).cloned() {
                    Some(t) if t.tok == Tok::Comma => self.pos += 1,
                    Some(t) if t.tok == Tok::RParen => break,
                    Some(t) => return Err(self.token_error(&t, Some("`,` or `)`"))),
                    None => return Err(self.end_error()),
                }
            }
        }
        let Some(close) = self.toks.get(self.pos).cloned() else {
            return Err(self.end_error());
        };
        self.pos += 1;
        let span = name_span.start..close.span.end;
        let count = args.len();
        if count < min || max.is_some_and(|m| count > m) {
            let expected = match max {
                Some(m) if m == min => min.to_string(),
                _ => format!("{min} or more"),
            };
            return Err(ExprError::new(
                span,
                ExprErrorKind::WrongArgCount {
                    name,
                    expected,
                    found: count,
                },
            ));
        }
        let height = 1 + args.iter().map(|a| a.height).max().unwrap_or(0);
        Node::new(span, height, Kind::Call { name, args })
    }
}

/// Parses `src` into a syntax tree.
pub(crate) fn parse(src: &str) -> Result<Node, ExprError> {
    let toks = lex(src)?;
    let mut p = Parser {
        src,
        toks,
        pos: 0,
        depth: 0,
    };
    let node = p.expr(0)?;
    if let Some(t) = p.peek() {
        let expected = (t.tok == Tok::RParen).then_some("end of expression");
        return Err(p.token_error(t, expected));
    }
    Ok(node)
}
