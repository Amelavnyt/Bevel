// SPDX-License-Identifier: GPL-3.0-or-later

//! Basis functions: non-negativity and partition of unity.

use bevel_geom::KnotVector;
use bevel_geom::basis::{basis_funs, ders_basis_funs};
use proptest::prelude::*;

use crate::strategies::{config, degree_and_knots, params};

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn basis_is_a_non_negative_partition_of_unity((p, knots) in degree_and_knots()) {
        let kv = KnotVector::new(knots, p).unwrap();
        let (a, b) = kv.domain(p);
        let mut n = vec![0.0; p + 1];
        let mut d = vec![0.0; (p + 1) * (p + 1)];
        for u in params(a, b, 64) {
            basis_funs(&kv, p, u, &mut n).unwrap();
            prop_assert!(n.iter().all(|&v| v >= -1e-14), "negative basis value at {u}: {n:?}");
            let sum: f64 = n.iter().sum();
            prop_assert!((sum - 1.0).abs() < 1e-12, "sum {sum} at {u}");

            ders_basis_funs(&kv, p, u, p, &mut d).unwrap();
            for (j, v) in n.iter().enumerate() {
                prop_assert!((d[j] - v).abs() < 1e-14);
            }
            for k in 1..=p {
                let row = &d[k * (p + 1)..(k + 1) * (p + 1)];
                let sum: f64 = row.iter().sum();
                let scale: f64 = row.iter().map(|v| v.abs()).sum::<f64>().max(1.0);
                prop_assert!(sum.abs() <= 1e-11 * scale, "derivative {k} sums to {sum} at {u}");
            }
        }
    }
}
