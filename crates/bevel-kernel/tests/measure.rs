// SPDX-License-Identifier: GPL-3.0-or-later

//! Face areas and edge lengths of the primitives against closed forms.

#![allow(clippy::unwrap_used, clippy::panic, reason = "test helpers")]

use bevel_kernel::{
    Body, BoxParams, CylinderParams, OpContext, OpTag, Role, SphereParams, TopoName, box_,
    cylinder, edge_length, face_area, sphere,
};
use bevel_math::{Frame3, Vector3};
use std::f64::consts::PI;
use std::sync::Arc;

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * b.abs().max(1.0)
}

fn role(name: &TopoName) -> Option<Role> {
    match name {
        TopoName::Generated { role, .. } => Some(*role),
        _ => None,
    }
}

fn single(output: bevel_kernel::OpOutput) -> Arc<Body> {
    output.bodies.into_iter().next().unwrap()
}

#[test]
fn box_faces_and_edges() {
    let mut cx = OpContext::new();
    let body = single(
        box_(
            &BoxParams {
                frame: Frame3::WORLD,
                size: Vector3::new(10.0, 20.0, 30.0),
            },
            OpTag(1),
            &mut cx,
        )
        .unwrap(),
    );
    let mut total = 0.0;
    for (key, face) in body.faces() {
        let area = face_area(&body, key, &cx).unwrap();
        let expected = match role(&face.name) {
            Some(Role::Top | Role::Bottom) => 200.0,
            Some(Role::Front | Role::Back) => 300.0,
            Some(Role::Left | Role::Right) => 600.0,
            other => panic!("unexpected face role {other:?}"),
        };
        assert!(
            close(area, expected),
            "{:?}: {area} vs {expected}",
            face.name
        );
        total += area;
    }
    assert!(close(total, 2.0 * (200.0 + 300.0 + 600.0)));
    let total_length: f64 = body
        .edges()
        .map(|(k, _)| edge_length(&body, k).unwrap())
        .sum();
    assert!(close(total_length, 4.0 * (10.0 + 20.0 + 30.0)));
}

#[test]
fn cylinder_side_caps_and_circles() {
    let mut cx = OpContext::new();
    let (r, h) = (7.0, 12.0);
    let body = single(
        cylinder(
            &CylinderParams {
                frame: Frame3::WORLD,
                radius: r,
                height: h,
            },
            OpTag(1),
            &mut cx,
        )
        .unwrap(),
    );
    let mut areas: Vec<f64> = body
        .faces()
        .map(|(k, _)| face_area(&body, k, &cx).unwrap())
        .collect();
    areas.sort_by(f64::total_cmp);
    assert!(
        close(areas[0], PI * r * r) && close(areas[1], PI * r * r),
        "{areas:?}"
    );
    assert!(close(areas[2], 2.0 * PI * r * h), "{areas:?}");
    let mut lengths: Vec<f64> = body
        .edges()
        .map(|(k, _)| edge_length(&body, k).unwrap())
        .collect();
    lengths.sort_by(f64::total_cmp);
    // The seam has the height; the two circles the circumference.
    assert!(close(lengths[0], h), "{lengths:?}");
    assert!(
        close(lengths[1], 2.0 * PI * r) && close(lengths[2], 2.0 * PI * r),
        "{lengths:?}"
    );
}

#[test]
fn sphere_area() {
    let mut cx = OpContext::new();
    let r = 9.0;
    let body = single(
        sphere(
            &SphereParams {
                frame: Frame3::WORLD,
                radius: r,
            },
            OpTag(1),
            &mut cx,
        )
        .unwrap(),
    );
    let (key, _) = body.faces().next().unwrap();
    assert!(close(face_area(&body, key, &cx).unwrap(), 4.0 * PI * r * r));
}
