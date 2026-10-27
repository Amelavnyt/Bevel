// SPDX-License-Identifier: GPL-3.0-or-later

//! Real roots of polynomials.
//!
//! Polynomials are given by their coefficients in **ascending** power
//! order: `coeffs[i]` multiplies `t^i`, so `[c0, c1, c2]` is
//! `c0 + c1 t + c2 t²`.
//!
//! All root-returning functions return roots sorted ascending, with roots
//! closer than `1e-12 * max(1, |r|)` merged into one. A multiple root is
//! therefore reported once. The zero polynomial (all coefficients zero) is
//! zero everywhere; it is reported as having **no** roots.
//!
//! - [`quadratic`], [`cubic`] and [`quartic`] use closed forms and polish
//!   every root with two Newton steps on the original polynomial. A leading
//!   coefficient that is zero, or negligible next to the others (below
//!   `1e-14` of the largest coefficient), makes the function fall back to
//!   the next lower degree. A multiple root of a cubic or quartic is
//!   recognised when the polynomial vanishes (to rounding error) at a
//!   critical point; it is then reported exactly at that point.
//! - [`roots_in`] handles any degree: it isolates roots with a Sturm
//!   sequence and refines them by safeguarded Newton/bisection. Sturm
//!   sequences are evaluated in floating point, so accuracy degrades for
//!   high degrees (beyond roughly 15) or for roots that are extremely close
//!   together.

use crate::Interval;

/// Roots closer than this (relative to `max(1, |r|)`) are merged.
const MERGE_TOL: f64 = 1e-12;
/// Leading coefficients below this fraction of the largest are dropped.
const LEAD_TOL: f64 = 1e-14;
/// Remainders below this fraction of their dividend count as zero.
const REMAINDER_TOL: f64 = 1e-12;
/// How far past the ends of the interval `roots_in` still accepts a root,
/// relative to the interval's half-length.
const END_SLACK: f64 = 1e-12;

/// Evaluates the polynomial at `t` with Horner's scheme.
///
/// An empty coefficient slice is the zero polynomial.
pub fn eval(coeffs: &[f64], t: f64) -> f64 {
    coeffs.iter().rev().fold(0.0, |acc, &c| acc * t + c)
}

/// Coefficients of the derivative, in ascending order.
///
/// The derivative of a constant (or of the empty polynomial) is the empty
/// slice, which is the zero polynomial.
pub fn derivative(coeffs: &[f64]) -> Vec<f64> {
    coeffs
        .iter()
        .enumerate()
        .skip(1)
        .map(|(i, &c)| c * i as f64)
        .collect()
}

/// Real roots of `c0 + c1 t + c2 t²`, ascending.
///
/// Uses the cancellation-free form `q = -(b + sign(b) sqrt(disc)) / 2`,
/// `t = q / a` and `t = c / q`. A negligible `c2` falls back to the linear
/// case.
pub fn quadratic(c0: f64, c1: f64, c2: f64) -> Vec<f64> {
    low_degree(&[c0, c1, c2])
}

/// Real roots of `c0 + c1 t + c2 t² + c3 t³`, ascending.
///
/// Uses the trigonometric form when there are three real roots and
/// Cardano's formula when there is one. A negligible `c3` falls back to
/// [`quadratic`].
pub fn cubic(c0: f64, c1: f64, c2: f64, c3: f64) -> Vec<f64> {
    low_degree(&[c0, c1, c2, c3])
}

/// Real roots of `c0 + c1 t + c2 t² + c3 t³ + c4 t⁴`, ascending.
///
/// Uses Ferrari's method through the resolvent cubic of the depressed
/// quartic. A negligible `c4` falls back to [`cubic`].
pub fn quartic(c0: f64, c1: f64, c2: f64, c3: f64, c4: f64) -> Vec<f64> {
    low_degree(&[c0, c1, c2, c3, c4])
}

/// All real roots of the polynomial inside the closed `interval`, ascending.
///
/// Works for any degree. Roots are isolated with a Sturm sequence and
/// refined by safeguarded Newton/bisection to about `1e-14` relative
/// accuracy. Multiple roots are found by first dividing out the greatest
/// common divisor of the polynomial and its derivative, so `(t - 1)²` has
/// the single root `1`. Roots within `1e-12` of an end of the interval
/// (relative to its half-length) count as inside and are clamped to it.
///
/// Returns an empty vector for the zero polynomial, for a nonzero constant,
/// for non-finite coefficients and for an inverted or non-finite interval.
pub fn roots_in(coeffs: &[f64], interval: Interval) -> Vec<f64> {
    if !coeffs.iter().all(|c| c.is_finite())
        || !interval.min.is_finite()
        || !interval.max.is_finite()
        || interval.min > interval.max
    {
        return Vec::new();
    }
    let radius = interval.min.abs().max(interval.max.abs()).max(1.0);
    let poly = trim_for_radius(coeffs, radius);
    if poly.len() < 2 {
        return Vec::new();
    }
    let length = interval.length();
    if length <= 0.0 {
        let t = interval.min;
        let bound = 64.0 * f64::EPSILON * eval_abs(poly, t);
        return if eval(poly, t).abs() <= bound {
            vec![t]
        } else {
            Vec::new()
        };
    }

    // Work in s in [-1, 1] with t = mid + half * s, which keeps the
    // Sturm sequence well scaled whatever the interval is.
    let mid = interval.mid();
    let half = 0.5 * length;
    let mut q = normalized(&shift_scale(poly, mid, half));
    let mut chain = sturm_chain(&q);
    if let Some(g) = chain.last()
        && g.len() >= 2
    {
        q = normalized(&poly_div(&q, g));
        chain = sturm_chain(&q);
    }

    let lo = -1.0 - END_SLACK;
    let hi = 1.0 + END_SLACK;
    let mut found = Vec::new();
    let mut stack = vec![(
        lo,
        hi,
        sign_changes(&chain, lo),
        sign_changes(&chain, hi),
        0,
    )];
    while let Some((a, b, va, vb, depth)) = stack.pop() {
        match va.saturating_sub(vb) {
            0 => {}
            1 => found.extend(refine(&q, a, b)),
            _ if depth >= 80 || b - a <= 1e-15 => found.push(0.5 * (a + b)),
            _ => {
                let m = 0.5 * (a + b);
                let vm = sign_changes(&chain, m);
                stack.push((a, m, va, vm, depth + 1));
                stack.push((m, b, vm, vb, depth + 1));
            }
        }
    }
    let roots = found
        .into_iter()
        .map(|s| interval.clamp(mid + half * s))
        .collect();
    merge(roots)
}

// ---------------------------------------------------------------------
// Closed forms
// ---------------------------------------------------------------------

/// Roots of a polynomial of degree at most 4: closed form, snap of
/// multiple roots, Newton polish, merge.
fn low_degree(coeffs: &[f64]) -> Vec<f64> {
    if !coeffs.iter().all(|c| c.is_finite()) {
        return Vec::new();
    }
    let c = trim_leading(coeffs);
    let mut roots = solve_raw(c);
    if c.len() >= 4 {
        snap_multiple_roots(c, &mut roots);
    }
    let d = derivative(c);
    for r in &mut roots {
        *r = polish(c, &d, *r);
    }
    merge(roots)
}

/// Closed-form roots, unsorted, unpolished, possibly duplicated.
fn solve_raw(c: &[f64]) -> Vec<f64> {
    match *c {
        [c0, c1] => vec![-c0 / c1],
        [c0, c1, c2] => quadratic_raw(c2, c1, c0),
        [c0, c1, c2, c3] => cubic_raw(c0, c1, c2, c3),
        [c0, c1, c2, c3, c4] => quartic_raw(c0, c1, c2, c3, c4),
        _ => Vec::new(),
    }
}

/// `a t² + b t + c` with `a != 0`.
fn quadratic_raw(a: f64, b: f64, c: f64) -> Vec<f64> {
    let four_ac = 4.0 * a * c;
    let mut disc = b * b - four_ac;
    // A discriminant that is negative only by rounding error means a double root.
    if disc < 0.0 && disc >= -8.0 * f64::EPSILON * (b * b + four_ac.abs()) {
        disc = 0.0;
    }
    if disc < 0.0 {
        return Vec::new();
    }
    let sign = if b >= 0.0 { 1.0 } else { -1.0 };
    let q = -0.5 * (b + sign * disc.sqrt());
    if q == 0.0 {
        // b == 0 and disc == 0, so c == 0: a double root at zero.
        return vec![0.0];
    }
    vec![q / a, c / q]
}

/// `c0 + c1 t + c2 t² + c3 t³` with `c3 != 0`.
fn cubic_raw(c0: f64, c1: f64, c2: f64, c3: f64) -> Vec<f64> {
    let (a, b, c) = (c2 / c3, c1 / c3, c0 / c3);
    // Depressed cubic x³ + p x + q with t = x - a/3.
    let shift = a / 3.0;
    let p = b - a * a / 3.0;
    let q = 2.0 * a * a * a / 27.0 - a * b / 3.0 + c;
    let delta = q * q / 4.0 + p * p * p / 27.0;
    if delta > 0.0 {
        // One real root (Cardano), choosing the sign that avoids cancellation.
        let s = delta.sqrt();
        let w = if q >= 0.0 { -0.5 * q - s } else { -0.5 * q + s };
        let u = w.cbrt();
        let x = if u == 0.0 { 0.0 } else { u - p / (3.0 * u) };
        vec![x - shift]
    } else if p == 0.0 {
        vec![-shift]
    } else {
        // Three real roots (counting multiplicity): trigonometric form.
        let m = 2.0 * (-p / 3.0).sqrt();
        let arg = (3.0 * q / (2.0 * p) * (-3.0 / p).sqrt()).clamp(-1.0, 1.0);
        let phi = arg.acos() / 3.0;
        let two_thirds_pi = 2.0 * std::f64::consts::FRAC_PI_3;
        (0..3)
            .map(|k| m * (phi - f64::from(k) * two_thirds_pi).cos() - shift)
            .collect()
    }
}

/// `c0 + c1 t + c2 t² + c3 t³ + c4 t⁴` with `c4 != 0` (Ferrari).
fn quartic_raw(c0: f64, c1: f64, c2: f64, c3: f64, c4: f64) -> Vec<f64> {
    let (a, b, c, d) = (c3 / c4, c2 / c4, c1 / c4, c0 / c4);
    // Depressed quartic y⁴ + p y² + q y + r with t = y - a/4.
    let shift = a / 4.0;
    let p = b - 3.0 * a * a / 8.0;
    let q = c - a * b / 2.0 + a * a * a / 8.0;
    let r = d - a * c / 4.0 + a * a * b / 16.0 - 3.0 * a * a * a * a / 256.0;

    let length = p
        .abs()
        .sqrt()
        .max(q.abs().cbrt())
        .max(r.abs().sqrt().sqrt());
    let mut ys = Vec::new();
    if q.abs() <= 1e-14 * length * length * length {
        // Biquadratic: y⁴ + p y² + r = 0 in z = y².
        for z in quadratic_raw(1.0, p, r) {
            let tol = 1e-14 * length * length;
            if z >= 0.0 {
                let y = z.sqrt();
                ys.push(y);
                ys.push(-y);
            } else if z >= -tol {
                ys.push(0.0);
            }
        }
    } else {
        // Resolvent cubic 8 m³ + 8 p m² + (2 p² - 8 r) m - q² = 0 has a
        // positive root because its value at 0 is -q² < 0.
        let m = cubic_raw(-q * q, 2.0 * p * p - 8.0 * r, 8.0 * p, 8.0)
            .into_iter()
            .fold(f64::NEG_INFINITY, f64::max);
        if m > 0.0 && m.is_finite() {
            let s = (2.0 * m).sqrt();
            let half_q_over_s = q / (2.0 * s);
            ys.extend(quadratic_raw(1.0, -s, 0.5 * p + m + half_q_over_s));
            ys.extend(quadratic_raw(1.0, s, 0.5 * p + m - half_q_over_s));
        }
    }
    ys.into_iter().map(|y| y - shift).collect()
}

/// Replaces candidate roots near a multiple root by that root.
///
/// A root of multiplicity two or more of `coeffs` is a root of its
/// derivative. For each critical point where the polynomial vanishes to
/// rounding error, drop candidates within `1e-6` (relative) and add the
/// critical point itself.
fn snap_multiple_roots(coeffs: &[f64], roots: &mut Vec<f64>) {
    let d = derivative(coeffs);
    for xc in solve_raw(trim_leading(&d)) {
        let bound = 64.0 * f64::EPSILON * eval_abs(coeffs, xc);
        if eval(coeffs, xc).abs() <= bound {
            let reach = 1e-6 * xc.abs().max(1.0);
            roots.retain(|r| (r - xc).abs() > reach);
            roots.push(xc);
        }
    }
}

/// Two Newton steps on the polynomial, each kept only if `|p|` does not grow.
fn polish(coeffs: &[f64], deriv: &[f64], root: f64) -> f64 {
    let mut x = root;
    for _ in 0..2 {
        let f = eval(coeffs, x);
        let df = eval(deriv, x);
        if f == 0.0 || df == 0.0 {
            break;
        }
        let next = x - f / df;
        if next.is_finite() && eval(coeffs, next).abs() <= f.abs() {
            x = next;
        }
    }
    x
}

// ---------------------------------------------------------------------
// Polynomial helpers
// ---------------------------------------------------------------------

fn max_abs(c: &[f64]) -> f64 {
    c.iter().fold(0.0, |m, x| m.max(x.abs()))
}

/// `coeffs` without negligible leading coefficients.
fn trim_leading(coeffs: &[f64]) -> &[f64] {
    let limit = LEAD_TOL * max_abs(coeffs);
    let n = coeffs
        .iter()
        .rposition(|c| c.abs() > limit)
        .map_or(0, |i| i + 1);
    coeffs.get(..n).unwrap_or(&[])
}

/// Like [`trim_leading`], but judges each coefficient by its size on
/// `|t| <= radius`.
fn trim_for_radius(coeffs: &[f64], radius: f64) -> &[f64] {
    let weighted = |i: usize, c: f64| c.abs() * radius.powi(i32::try_from(i).unwrap_or(i32::MAX));
    let largest = coeffs
        .iter()
        .enumerate()
        .fold(0.0, |m, (i, &c)| f64::max(m, weighted(i, c)));
    let n = coeffs
        .iter()
        .enumerate()
        .rposition(|(i, &c)| weighted(i, c) > 1e-16 * largest)
        .map_or(0, |i| i + 1);
    coeffs.get(..n).unwrap_or(&[])
}

/// Horner evaluation of the sum of absolute values; bounds the rounding error.
fn eval_abs(coeffs: &[f64], t: f64) -> f64 {
    let t = t.abs();
    coeffs.iter().rev().fold(0.0, |acc, &c| acc * t + c.abs())
}

/// Sorts ascending and merges roots closer than `MERGE_TOL`.
fn merge(mut roots: Vec<f64>) -> Vec<f64> {
    roots.sort_by(f64::total_cmp);
    let mut out: Vec<f64> = Vec::with_capacity(roots.len());
    for r in roots {
        match out.last() {
            Some(&last) if r - last <= MERGE_TOL * r.abs().max(1.0) => {}
            _ => out.push(r),
        }
    }
    out
}

/// Coefficients of `p(mid + half * s)` in `s`.
fn shift_scale(coeffs: &[f64], mid: f64, half: f64) -> Vec<f64> {
    let mut c = coeffs.to_vec();
    let n = c.len();
    // Taylor shift by repeated synthetic division.
    for i in 0..n.saturating_sub(1) {
        for j in (i..n - 1).rev() {
            let next = c.get(j + 1).copied().unwrap_or(0.0);
            if let Some(cj) = c.get_mut(j) {
                *cj += mid * next;
            }
        }
    }
    let mut scale = 1.0;
    for ci in &mut c {
        *ci *= scale;
        scale *= half;
    }
    c
}

/// The polynomial divided by its largest coefficient (positive scale only).
fn normalized(c: &[f64]) -> Vec<f64> {
    let m = max_abs(c);
    if m > 0.0 {
        c.iter().map(|x| x / m).collect()
    } else {
        Vec::new()
    }
}

/// Removes leading coefficients no larger than `limit`.
fn trim_above(c: &mut Vec<f64>, limit: f64) {
    while c.last().is_some_and(|x| x.abs() <= limit) {
        c.pop();
    }
}

/// Remainder of `a / b`, with negligible leading terms removed.
fn poly_rem(a: &[f64], b: &[f64]) -> Vec<f64> {
    let mut r = a.to_vec();
    let (Some(&lead), Some(db)) = (b.last(), b.len().checked_sub(1)) else {
        return r;
    };
    if lead == 0.0 {
        return r;
    }
    while r.len() > db {
        let Some(top) = r.pop() else { break };
        let f = top / lead;
        let k = r.len().saturating_sub(db);
        for (i, &bi) in b.iter().enumerate().take(db) {
            if let Some(ri) = r.get_mut(k + i) {
                *ri -= f * bi;
            }
        }
    }
    trim_above(&mut r, REMAINDER_TOL * max_abs(a));
    r
}

/// Quotient of `a / b` (the remainder is discarded).
fn poly_div(a: &[f64], b: &[f64]) -> Vec<f64> {
    let mut r = a.to_vec();
    let (Some(&lead), Some(db)) = (b.last(), b.len().checked_sub(1)) else {
        return r;
    };
    if lead == 0.0 || r.len() <= db {
        return Vec::new();
    }
    let mut quotient = vec![0.0; r.len() - db];
    while r.len() > db {
        let Some(top) = r.pop() else { break };
        let f = top / lead;
        let k = r.len().saturating_sub(db);
        if let Some(qk) = quotient.get_mut(k) {
            *qk = f;
        }
        for (i, &bi) in b.iter().enumerate().take(db) {
            if let Some(ri) = r.get_mut(k + i) {
                *ri -= f * bi;
            }
        }
    }
    quotient
}

/// The Sturm sequence `p, p', -rem(p, p'), ...`, each term scaled to
/// largest coefficient 1. It ends with the greatest common divisor of `p`
/// and `p'` (a constant when `p` is square-free).
fn sturm_chain(p: &[f64]) -> Vec<Vec<f64>> {
    let mut chain = vec![p.to_vec(), normalized(&derivative(p))];
    while let (Some(prev), Some(last)) = (
        chain.len().checked_sub(2).and_then(|i| chain.get(i)),
        chain.last(),
    ) {
        if last.is_empty() {
            chain.pop();
            break;
        }
        if last.len() == 1 {
            break;
        }
        let rem: Vec<f64> = poly_rem(prev, last).iter().map(|x| -x).collect();
        if rem.is_empty() {
            break;
        }
        chain.push(normalized(&rem));
    }
    chain
}

/// Number of sign changes in the chain evaluated at `s`, ignoring zeros.
fn sign_changes(chain: &[Vec<f64>], s: f64) -> usize {
    let mut changes = 0;
    let mut previous = 0.0_f64;
    for poly in chain {
        let v = eval(poly, s);
        if v == 0.0 {
            continue;
        }
        if previous != 0.0 && (v > 0.0) != (previous > 0.0) {
            changes += 1;
        }
        previous = v;
    }
    changes
}

/// The single root of the square-free polynomial in `(a, b]`, by
/// safeguarded Newton: a Newton step is used when it stays inside the
/// bracket, bisection otherwise.
fn refine(q: &[f64], a: f64, b: f64) -> Option<f64> {
    let dq = derivative(q);
    let fb = eval(q, b);
    if fb == 0.0 {
        return Some(b);
    }
    // The interval is open at `a`: a root exactly at `a` belongs to the
    // neighbouring interval, so use the sign of q just to the right of it.
    let mut fa = eval(q, a);
    if fa == 0.0 {
        fa = eval(&dq, a);
    }
    if fa == 0.0 || (fa > 0.0) == (fb > 0.0) {
        return None;
    }
    // Keep f(lo) < 0 < f(hi).
    let (mut lo, mut hi) = if fa < 0.0 { (a, b) } else { (b, a) };
    let mut x = 0.5 * (a + b);
    for _ in 0..200 {
        let f = eval(q, x);
        if f == 0.0 {
            return Some(x);
        }
        if f < 0.0 {
            lo = x;
        } else {
            hi = x;
        }
        let (inner_lo, inner_hi) = (lo.min(hi), lo.max(hi));
        let df = eval(&dq, x);
        let mut next = if df == 0.0 { f64::NAN } else { x - f / df };
        if !(next > inner_lo && next < inner_hi) {
            next = 0.5 * (lo + hi);
        }
        if (next - x).abs() <= 1e-15 || (inner_hi - inner_lo) <= 2e-16 {
            return Some(next);
        }
        x = next;
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    /// Ascending coefficients of `lead * prod (t - r)`.
    fn from_roots(roots: &[f64], lead: f64) -> Vec<f64> {
        let mut c = vec![lead];
        for &r in roots {
            let mut next = vec![0.0; c.len() + 1];
            for (i, &ci) in c.iter().enumerate() {
                next[i + 1] += ci;
                next[i] -= r * ci;
            }
            c = next;
        }
        c
    }

    fn assert_roots(got: &[f64], want: &[f64], tol: f64) {
        assert_eq!(got.len(), want.len(), "got {got:?}, want {want:?}");
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() <= tol, "got {got:?}, want {want:?}");
        }
    }

    #[test]
    fn eval_and_derivative() {
        let p = [1.0, -3.0, 0.0, 2.0]; // 1 - 3t + 2t³
        assert!((eval(&p, 2.0) - 11.0).abs() < 1e-15);
        assert_eq!(derivative(&p), vec![-3.0, 0.0, 6.0]);
        assert!(derivative(&[5.0]).is_empty());
        assert!(eval(&[], 3.0).abs() < 1e-15);
    }

    #[test]
    fn quadratic_cases() {
        assert_roots(&quadratic(-6.0, 1.0, 1.0), &[-3.0, 2.0], 1e-14);
        assert!(quadratic(1.0, 0.0, 1.0).is_empty());
        assert_roots(&quadratic(1.0, -2.0, 1.0), &[1.0], 1e-14);
        assert_roots(&quadratic(0.0, 0.0, 1.0), &[0.0], 1e-14);
        // Linear fallback.
        assert_roots(&quadratic(-4.0, 2.0, 0.0), &[2.0], 1e-14);
        assert!(quadratic(3.0, 0.0, 0.0).is_empty());
        assert!(quadratic(0.0, 0.0, 0.0).is_empty());
    }

    #[test]
    fn quadratic_avoids_cancellation() {
        // Roots near 1e-8 and 1e8 both need full relative accuracy.
        let r = quadratic(1.0, -(1e8 + 1e-8), 1.0);
        assert_eq!(r.len(), 2);
        assert!((r[0] - 1e-8).abs() <= 1e-22);
        assert!((r[1] - 1e8).abs() <= 1e-6);
    }

    #[test]
    fn cubic_cases() {
        assert_roots(&cubic(-6.0, 11.0, -6.0, 1.0), &[1.0, 2.0, 3.0], 1e-13);
        // One real root: t³ + t + 2 = (t + 1)(t² - t + 2).
        assert_roots(&cubic(2.0, 1.0, 0.0, 1.0), &[-1.0], 1e-14);
        // Double root: (t - 1)²(t + 2).
        assert_roots(&cubic(2.0, -3.0, 0.0, 1.0), &[-2.0, 1.0], 1e-12);
        // Triple root.
        assert_roots(&cubic(-1.0, 3.0, -3.0, 1.0), &[1.0], 1e-12);
        // Falls back to a quadratic.
        assert_roots(&cubic(-6.0, 1.0, 1.0, 0.0), &[-3.0, 2.0], 1e-14);
    }

    #[test]
    fn quartic_cases() {
        let c = from_roots(&[-3.0, -0.5, 1.0, 4.0], 2.0);
        assert_roots(
            &quartic(c[0], c[1], c[2], c[3], c[4]),
            &[-3.0, -0.5, 1.0, 4.0],
            1e-12,
        );
        // Biquadratic t⁴ - 5t² + 4.
        assert_roots(
            &quartic(4.0, 0.0, -5.0, 0.0, 1.0),
            &[-2.0, -1.0, 1.0, 2.0],
            1e-13,
        );
        // No real roots.
        assert!(quartic(1.0, 0.0, 0.0, 0.0, 1.0).is_empty());
        // Two real roots: (t² + 1)(t - 1)(t + 3).
        assert_roots(&quartic(-3.0, 2.0, -2.0, 2.0, 1.0), &[-3.0, 1.0], 1e-13);
        // Double root: (t - 1)²(t - 2)(t + 3).
        let c = from_roots(&[1.0, 1.0, 2.0, -3.0], 1.0);
        assert_roots(
            &quartic(c[0], c[1], c[2], c[3], c[4]),
            &[-3.0, 1.0, 2.0],
            1e-7,
        );
        // Two double roots: (t - 1)²(t + 2)².
        let c = from_roots(&[1.0, 1.0, -2.0, -2.0], 1.0);
        assert_roots(&quartic(c[0], c[1], c[2], c[3], c[4]), &[-2.0, 1.0], 1e-7);
        // Falls back to a cubic.
        assert_roots(
            &quartic(-6.0, 11.0, -6.0, 1.0, 0.0),
            &[1.0, 2.0, 3.0],
            1e-12,
        );
    }

    #[test]
    fn non_finite_coefficients_give_no_roots() {
        assert!(quadratic(f64::NAN, 1.0, 1.0).is_empty());
        assert!(roots_in(&[f64::INFINITY, 1.0], Interval::new(-1.0, 1.0)).is_empty());
    }

    #[test]
    fn roots_in_basic() {
        let c = from_roots(&[-2.0, 0.5, 3.0, 7.0, 8.5], 1.0);
        assert_roots(
            &roots_in(&c, Interval::new(-10.0, 10.0)),
            &[-2.0, 0.5, 3.0, 7.0, 8.5],
            1e-10,
        );
        assert_roots(
            &roots_in(&c, Interval::new(0.0, 7.0)),
            &[0.5, 3.0, 7.0],
            1e-10,
        );
        assert_roots(&roots_in(&c, Interval::new(9.0, 10.0)), &[], 0.0);
    }

    #[test]
    fn roots_in_high_degree() {
        // Chebyshev-like spread of 9 roots.
        let want: Vec<f64> = (0..9).map(|i| -4.0 + f64::from(i)).collect();
        let c = from_roots(&want, 1.0);
        assert_roots(&roots_in(&c, Interval::new(-5.0, 5.0)), &want, 1e-8);
    }

    #[test]
    fn roots_in_includes_roots_at_the_ends() {
        let c = from_roots(&[-1.0, 2.0], 1.0);
        assert_roots(&roots_in(&c, Interval::new(-1.0, 2.0)), &[-1.0, 2.0], 1e-13);
    }

    #[test]
    fn roots_in_double_root() {
        let r = roots_in(&[1.0, -2.0, 1.0], Interval::new(-5.0, 5.0));
        assert_roots(&r, &[1.0], 1e-12);
        // (t - 1)²(t - 3)³(t + 2)
        let c = from_roots(&[1.0, 1.0, 3.0, 3.0, 3.0, -2.0], 1.0);
        assert_roots(
            &roots_in(&c, Interval::new(-5.0, 5.0)),
            &[-2.0, 1.0, 3.0],
            1e-6,
        );
    }

    #[test]
    fn roots_in_degenerate_inputs() {
        let i = Interval::new(-1.0, 1.0);
        assert!(roots_in(&[], i).is_empty());
        assert!(roots_in(&[0.0, 0.0, 0.0], i).is_empty());
        assert!(roots_in(&[3.0], i).is_empty());
        assert!(roots_in(&[0.0], i).is_empty());
        // Inverted (empty) interval.
        assert!(roots_in(&[-1.0, 1.0], Interval { min: 2.0, max: 0.0 }).is_empty());
        // Zero-length interval, on and off a root.
        assert_roots(
            &roots_in(&[-1.0, 1.0], Interval::new(1.0, 1.0)),
            &[1.0],
            0.0,
        );
        assert!(roots_in(&[-1.0, 1.0], Interval::new(2.0, 2.0)).is_empty());
        // Negligible leading coefficients are ignored.
        assert_roots(&roots_in(&[-1.0, 1.0, 0.0, 0.0], i), &[1.0], 1e-13);
    }

    #[test]
    fn roots_in_far_from_the_origin() {
        let c = from_roots(&[100.25, 100.5], 1.0);
        assert_roots(
            &roots_in(&c, Interval::new(100.0, 101.0)),
            &[100.25, 100.5],
            1e-9,
        );
    }

    #[test]
    fn shift_scale_matches_substitution() {
        let p = [0.0, 0.0, 1.0];
        assert_eq!(shift_scale(&p, 1.0, 1.0), vec![1.0, 2.0, 1.0]);
        assert_eq!(shift_scale(&p, 1.0, 2.0), vec![1.0, 4.0, 4.0]);
    }

    /// Accuracy to demand of root `r` of `c`: 1e-9, or more where the
    /// root is so ill-conditioned that rounding the coefficients alone moves it.
    fn allowed_error(c: &[f64], r: f64) -> f64 {
        let slope = eval(&derivative(c), r).abs();
        (100.0 * f64::EPSILON * eval_abs(c, r) / slope).max(1e-9)
    }

    /// 1 to 4 roots in [-10, 10], more than 1e-3 apart.
    fn separated_roots() -> impl Strategy<Value = Vec<f64>> {
        prop::collection::vec(-10.0..10.0f64, 1..=4).prop_filter("roots too close", |v| {
            let mut s = v.clone();
            s.sort_by(f64::total_cmp);
            s.windows(2).all(|w| w[1] - w[0] > 1e-3)
        })
    }

    proptest! {
        #[test]
        fn roots_in_recovers_known_roots(
            roots in separated_roots(),
            lead in prop_oneof![-5.0..-0.5f64, 0.5..5.0f64],
        ) {
            let mut want = roots.clone();
            want.sort_by(f64::total_cmp);
            let c = from_roots(&roots, lead);
            let got = roots_in(&c, Interval::new(-10.5, 10.5));
            prop_assert_eq!(got.len(), want.len());
            for (g, w) in got.iter().zip(&want) {
                prop_assert!((g - w).abs() <= allowed_error(&c, *w), "got {:?}, want {:?}", got, want);
            }
        }

        #[test]
        fn closed_forms_recover_known_roots(
            roots in separated_roots(),
            lead in prop_oneof![-5.0..-0.5f64, 0.5..5.0f64],
        ) {
            let mut want = roots.clone();
            want.sort_by(f64::total_cmp);
            let c = from_roots(&roots, lead);
            let got = match *c.as_slice() {
                [c0, c1, c2] => quadratic(c0, c1, c2),
                [c0, c1, c2, c3] => cubic(c0, c1, c2, c3),
                [c0, c1, c2, c3, c4] => quartic(c0, c1, c2, c3, c4),
                [c0, c1] => quadratic(c0, c1, 0.0),
                _ => Vec::new(),
            };
            prop_assert_eq!(got.len(), want.len(), "got {:?}, want {:?}", got, want);
            for (g, w) in got.iter().zip(&want) {
                prop_assert!((g - w).abs() <= allowed_error(&c, *w), "got {:?}, want {:?}", got, want);
            }
        }
    }
}
