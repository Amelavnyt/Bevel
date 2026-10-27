// SPDX-License-Identifier: GPL-3.0-or-later

//! Closest points on surfaces.

use bevel_math::DVec3;
use proptest::prelude::*;

use crate::strategies::{any_surface, config, grid, surface_size};

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn closest_point_of_a_surface_point_is_itself(
        s in any_surface(),
        f in prop::array::uniform2(0.0f64..=1.0),
    ) {
        let ((ua, ub), (va, vb)) = s.domain();
        let (u, v) = (ua + (ub - ua) * f[0], va + (vb - va) * f[1]);
        let p = s.point(u, v);
        let cp = s.closest_point(p).unwrap();
        prop_assert!(cp.distance < 1e-7, "distance {} for a point on the surface", cp.distance);
        prop_assert!((cp.point - p).length() < 1e-7);
        prop_assert!((s.point(cp.u, cp.v) - cp.point).length() <= 1e-12 * surface_size(&s));
    }

    #[test]
    fn closest_point_beats_dense_surface_sampling(
        s in any_surface(),
        t in prop::array::uniform3(-150.0f64..150.0),
    ) {
        let target = DVec3::from_array(t);
        let cp = s.closest_point(target).unwrap();
        let ((ua, ub), (va, vb)) = s.domain();
        prop_assert!(cp.u >= ua && cp.u <= ub && cp.v >= va && cp.v <= vb);
        prop_assert!(
            ((s.point(cp.u, cp.v) - target).length() - cp.distance).abs()
                <= 1e-12 * surface_size(&s)
        );
        for (u, v) in grid(&s, 60) {
            let d = (s.point(u, v) - target).length();
            prop_assert!(
                cp.distance <= d + 1e-9,
                "closest distance {} but the sample at ({u}, {v}) is at {d}",
                cp.distance
            );
        }
    }
}
