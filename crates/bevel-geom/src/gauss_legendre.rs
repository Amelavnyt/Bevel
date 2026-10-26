// SPDX-License-Identifier: GPL-3.0-or-later

//! Fixed 8- and 16-point Gauss-Legendre rules.
//!
//! A local stand-in for the quadrature module planned in `bevel-math`; it
//! is private and kept minimal so that it can be swapped for that one.
//! The `n`-point rule integrates polynomials of degree up to `2n - 1`
//! exactly. Nodes are the roots of the Legendre polynomial `P_n`, weights
//! `2 / ((1 - x^2) P_n'(x)^2)`; both are symmetric about zero, so only the
//! positive half is stored.

/// Positive nodes and their weights of the 8-point rule on `[-1, 1]`.
pub(crate) const GL8: [(f64, f64); 4] = [
    (0.183_434_642_495_649_8, 0.362_683_783_378_362),
    (0.525_532_409_916_329, 0.313_706_645_877_887_27),
    (0.796_666_477_413_626_7, 0.222_381_034_453_374_48),
    (0.960_289_856_497_536_3, 0.101_228_536_290_376_26),
];

/// Positive nodes and their weights of the 16-point rule on `[-1, 1]`.
pub(crate) const GL16: [(f64, f64); 8] = [
    (0.095_012_509_837_637_44, 0.189_450_610_455_068_5),
    (0.281_603_550_779_258_9, 0.182_603_415_044_923_58),
    (0.458_016_777_657_227_37, 0.169_156_519_395_002_54),
    (0.617_876_244_402_643_8, 0.149_595_988_816_576_74),
    (0.755_404_408_355_003, 0.124_628_971_255_533_88),
    (0.865_631_202_387_831_8, 0.095_158_511_682_492_79),
    (0.944_575_023_073_232_6, 0.062_253_523_938_647_894),
    (0.989_400_934_991_649_9, 0.027_152_459_411_754_096),
];

/// Integrates `f` over `[a, b]` with a symmetric rule given by its positive
/// half.
pub(crate) fn integrate(rule: &[(f64, f64)], a: f64, b: f64, mut f: impl FnMut(f64) -> f64) -> f64 {
    let c = 0.5 * (a + b);
    let h = 0.5 * (b - a);
    let sum: f64 = rule
        .iter()
        .map(|&(x, w)| w * (f(c - h * x) + f(c + h * x)))
        .sum();
    sum * h
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check_exact(rule: &[(f64, f64)], max_degree: i32) {
        let weight_sum: f64 = rule.iter().map(|&(_, w)| 2.0 * w).sum();
        assert!((weight_sum - 2.0).abs() < 1e-14);
        for k in 0..=max_degree {
            let got = integrate(rule, -1.0, 1.0, |x| x.powi(k));
            let expected = if k % 2 == 0 {
                2.0 / f64::from(k + 1)
            } else {
                0.0
            };
            assert!((got - expected).abs() < 1e-14, "x^{k}: {got} vs {expected}");
        }
        // Shifted interval: the integral of x^(2n-1) over [1, 3].
        let n = max_degree + 1;
        let got = integrate(rule, 1.0, 3.0, |x| x.powi(n - 1));
        let expected = (3.0_f64.powi(n) - 1.0) / f64::from(n);
        assert!((got - expected).abs() < 1e-14 * expected);
    }

    #[test]
    fn rules_are_exact_for_their_polynomial_degree() {
        check_exact(&GL8, 15);
        check_exact(&GL16, 31);
    }
}
