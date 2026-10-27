// SPDX-License-Identifier: GPL-3.0-or-later

//! Property tests for mass properties: every primitive with random sizes
//! under random rigid frames matches the closed-form volume, area,
//! centroid and inertia tensor (see `Shape::exact` in `common`) to 1e-9,
//! and its principal moments and axes diagonalize the tensor.

// Helpers outside `#[test]` functions unwrap freely too.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod common;

use bevel_kernel::{OpContext, mass_properties};
use bevel_math::{DVec3, Vector3};
use common::{Shape, cone_ok, cone_radius, config, frame, length, max_abs, torus_ok};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

/// The relative accuracy required: of the volume and the area, of the
/// centroid against the body's diagonal, and of the tensor and its
/// eigenvalues against the tensor's largest entry.
const RELATIVE: f64 = 1e-9;

fn check(shape: &Shape) -> Result<(), TestCaseError> {
    let body = shape.build();
    let m = mass_properties(&body, &OpContext::new()).unwrap();
    let e = shape.exact();
    let rel = |a: f64, b: f64| (a - b).abs() / b.abs();
    prop_assert!(
        rel(m.volume, e.volume) <= RELATIVE,
        "volume {} vs {}",
        m.volume,
        e.volume
    );
    prop_assert!(
        rel(m.area, e.area) <= RELATIVE,
        "area {} vs {}",
        m.area,
        e.area
    );
    let diagonal = body.bounds().size().length();
    let off = m.centroid.distance(e.centroid);
    prop_assert!(
        off <= RELATIVE * diagonal,
        "centroid off by {off} (size {diagonal})"
    );

    let scale = max_abs(&e.inertia);
    let error = max_abs(&(m.inertia - e.inertia));
    prop_assert!(
        error <= RELATIVE * scale,
        "inertia off by {error} of {scale}:\n{:?}\nvs\n{:?}",
        m.inertia,
        e.inertia
    );

    // Principal moments: the sorted eigenvalues of the exact tensor
    // (its diagonal in the placement frame).
    let f = shape.frame();
    let local = |v: Vector3| v.dot(e.inertia * v);
    let mut exact = [local(f.x), local(f.y), local(f.z)];
    exact.sort_by(f64::total_cmp);
    let exact = DVec3::from_array(exact);
    let moment_error = (m.principal_moments - exact).abs().max_element();
    prop_assert!(
        moment_error <= RELATIVE * scale,
        "moments {:?} vs {:?}",
        m.principal_moments,
        exact
    );
    // The axes form a rotation and diagonalize the computed tensor.
    let a = m.principal_axes;
    prop_assert!((a.determinant() - 1.0).abs() < 1e-12);
    for k in 0..3 {
        let axis = a.col(k);
        prop_assert!((axis.length() - 1.0).abs() < 1e-12);
        let residual = m.inertia * axis - axis * m.principal_moments[k];
        prop_assert!(
            residual.length() <= 1e-12 * scale,
            "axis {k} residual {residual:?}"
        );
    }
    Ok(())
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn box_mass_properties_are_exact(frame in frame(), x in length(), y in length(), z in length()) {
        check(&Shape::Box(frame, Vector3::new(x, y, z)))?;
    }

    #[test]
    fn cylinder_mass_properties_are_exact(frame in frame(), r in length(), h in length()) {
        check(&Shape::Cylinder(frame, r, h))?;
    }

    #[test]
    fn sphere_mass_properties_are_exact(frame in frame(), r in length()) {
        check(&Shape::Sphere(frame, r))?;
    }

    #[test]
    fn cone_mass_properties_are_exact(
        frame in frame(), r0 in cone_radius(), r1 in cone_radius(), h in length(),
    ) {
        prop_assume!(cone_ok(r0, r1, h));
        check(&Shape::Cone(frame, r0, r1, h))?;
    }

    #[test]
    fn torus_mass_properties_are_exact(frame in frame(), big in length(), fraction in 0.02f64..0.98) {
        let small = big * fraction;
        prop_assume!(torus_ok(big, small));
        check(&Shape::Torus(frame, big, small))?;
    }
}
