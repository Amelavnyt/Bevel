// SPDX-License-Identifier: GPL-3.0-or-later

//! Tokeniser.

use std::ops::Range;

use crate::error::{ExprError, ExprErrorKind};
use crate::units::{AngleUnit, LengthUnit};

/// A unit symbol attached to a number literal.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum UnitSym {
    Length(LengthUnit),
    Angle(AngleUnit),
}

impl UnitSym {
    fn from_symbol(s: &str) -> Option<Self> {
        LengthUnit::from_symbol(s)
            .map(Self::Length)
            .or_else(|| AngleUnit::from_symbol(s).map(Self::Angle))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Tok {
    Num(f64, Option<UnitSym>),
    Ident(String),
    Plus,
    Minus,
    Star,
    Slash,
    Caret,
    LParen,
    RParen,
    Comma,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Token {
    pub tok: Tok,
    pub span: Range<usize>,
}

fn scan_while(bytes: &[u8], mut i: usize, pred: impl Fn(u8) -> bool) -> usize {
    while let Some(&b) = bytes.get(i) {
        if !pred(b) {
            break;
        }
        i += 1;
    }
    i
}

fn is_ident_start(b: u8) -> bool {
    b.is_ascii_alphabetic() || b == b'_'
}

fn is_ident_part(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_'
}

/// Scans a number literal starting at `start`; returns its end.
fn scan_number(bytes: &[u8], start: usize) -> Option<usize> {
    let int_end = scan_while(bytes, start, |b| b.is_ascii_digit());
    let mut j = int_end;
    let mut has_digits = int_end > start;
    if bytes.get(j) == Some(&b'.') {
        let frac_end = scan_while(bytes, j + 1, |b| b.is_ascii_digit());
        has_digits |= frac_end > j + 1;
        j = frac_end;
    }
    if !has_digits {
        return None;
    }
    if matches!(bytes.get(j), Some(b'e' | b'E')) {
        let mut k = j + 1;
        if matches!(bytes.get(k), Some(b'+' | b'-')) {
            k += 1;
        }
        let exp_end = scan_while(bytes, k, |b| b.is_ascii_digit());
        if exp_end > k {
            j = exp_end;
        }
    }
    Some(j)
}

/// Splits `src` into tokens.
pub(crate) fn lex(src: &str) -> Result<Vec<Token>, ExprError> {
    let bytes = src.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while let Some(&c) = bytes.get(i) {
        let simple = match c {
            b'+' => Some(Tok::Plus),
            b'-' => Some(Tok::Minus),
            b'*' => Some(Tok::Star),
            b'/' => Some(Tok::Slash),
            b'^' => Some(Tok::Caret),
            b'(' => Some(Tok::LParen),
            b')' => Some(Tok::RParen),
            b',' => Some(Tok::Comma),
            _ => None,
        };
        if let Some(tok) = simple {
            out.push(Token {
                tok,
                span: i..i + 1,
            });
            i += 1;
        } else if c == b' ' || c == b'\t' || c == b'\n' || c == b'\r' {
            i += 1;
        } else if c.is_ascii_digit() || c == b'.' {
            let Some(mut end) = scan_number(bytes, i) else {
                return Err(ExprError::new(i..i + 1, ExprErrorKind::UnexpectedChar('.')));
            };
            let value = src
                .get(i..end)
                .and_then(|t| t.parse::<f64>().ok())
                .filter(|v| v.is_finite())
                .ok_or_else(|| ExprError::new(i..end, ExprErrorKind::NonFinite))?;
            // An optional unit symbol, possibly after spaces.
            let k = scan_while(bytes, end, |b| b == b' ' || b == b'\t');
            let mut unit = None;
            if bytes.get(k).copied().is_some_and(is_ident_start) {
                let e = scan_while(bytes, k, is_ident_part);
                if let Some(u) = src.get(k..e).and_then(UnitSym::from_symbol) {
                    unit = Some(u);
                    end = e;
                }
            }
            out.push(Token {
                tok: Tok::Num(value, unit),
                span: i..end,
            });
            i = end;
        } else if is_ident_start(c) {
            let end = scan_while(bytes, i, is_ident_part);
            out.push(Token {
                tok: Tok::Ident(src.get(i..end).unwrap_or_default().to_owned()),
                span: i..end,
            });
            i = end;
        } else {
            let ch = src.get(i..).and_then(|s| s.chars().next()).unwrap_or('?');
            return Err(ExprError::new(
                i..i + ch.len_utf8(),
                ExprErrorKind::UnexpectedChar(ch),
            ));
        }
    }
    Ok(out)
}
