// SPDX-License-Identifier: GPL-3.0-or-later

//! Regeneration of primitive features driven by parameters.

#![allow(clippy::unwrap_used, clippy::panic, reason = "tests")]

use bevel_expr::{LengthUnit, ParamKind, Parameter, Units};
use bevel_kernel::{OpContext, mass_properties};
use bevel_model::{
    BodyMode, Feature, FeatureDef, FeatureError, FeatureId, ModelState, OriginPlane, Part,
    PrimitiveDef, PrimitiveKind, Regenerator,
};
use bevel_timeline::StepStatus;
use std::collections::BTreeMap;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

fn param(part: &mut Part, name: &str, expression: &str) {
    part.parameters
        .add(Parameter {
            name: name.to_owned(),
            expression: expression.to_owned(),
            comment: String::new(),
            kind: ParamKind::User,
        })
        .unwrap();
}

/// A primitive whose field `key` reads parameter `value` for every pair.
fn primitive(id: u64, kind: PrimitiveKind, inputs: &[(&str, &str)]) -> Feature {
    Feature {
        id: FeatureId(id),
        name: format!("{}{id}", kind.name()),
        def: FeatureDef::Primitive(PrimitiveDef {
            kind,
            plane: OriginPlane::Xy,
            inputs: inputs
                .iter()
                .map(|(k, v)| ((*k).to_owned(), (*v).to_owned()))
                .collect::<BTreeMap<_, _>>(),
            body_mode: BodyMode::NewBody,
        }),
    }
}

/// A part with a box (`size` x `size` x `size`) and a sphere (`r`) beside it.
fn part() -> Part {
    let mut part = Part::default();
    param(&mut part, "size", "10 mm");
    param(&mut part, "r", "5 mm");
    param(&mut part, "zero", "0 mm");
    param(&mut part, "far", "size * 3");
    part.features
        .insert(primitive(
            1,
            PrimitiveKind::Box,
            &[
                ("length", "size"),
                ("width", "size"),
                ("height", "size"),
                ("x", "zero"),
                ("y", "zero"),
                ("z", "zero"),
            ],
        ))
        .unwrap();
    part.features
        .insert(primitive(
            2,
            PrimitiveKind::Sphere,
            &[("radius", "r"), ("x", "far"), ("y", "zero"), ("z", "zero")],
        ))
        .unwrap();
    part
}

fn volume(state: &ModelState, i: usize) -> f64 {
    mass_properties(&state.bodies[i].body, &OpContext::default())
        .unwrap()
        .volume
}

#[test]
fn features_build_bodies_from_parameters() {
    let part = part();
    let r = Regenerator::new().regenerate(&part, None).unwrap();
    assert_eq!(r.state.bodies.len(), 2);
    assert!(r.statuses.iter().all(|(_, s)| *s == StepStatus::Ok));
    assert!((volume(&r.state, 0) - 1000.0).abs() < 1e-6);
    let sphere = r.state.bodies[1].body.bounds();
    assert!(
        (sphere.min.x - 25.0).abs() < 1e-9,
        "placed at x = 3 * size - r"
    );
}

#[test]
fn a_parameter_change_reruns_only_its_readers() {
    let mut part = part();
    let mut regen = Regenerator::new();
    assert_eq!(regen.regenerate(&part, None).unwrap().evaluated, 2);
    assert_eq!(regen.regenerate(&part, None).unwrap().evaluated, 0);
    part.parameters
        .set_expression("r", "6 mm".to_owned())
        .unwrap();
    let r = regen.regenerate(&part, None).unwrap();
    assert_eq!(r.evaluated, 1, "only the sphere reads r");
    // `far` depends on `size`, so both features read a changed value.
    part.parameters
        .set_expression("size", "12 mm".to_owned())
        .unwrap();
    assert_eq!(regen.regenerate(&part, None).unwrap().evaluated, 2);
}

#[test]
fn bare_numbers_use_the_document_unit() {
    let mut part = part();
    part.parameters
        .set_expression("size", "1".to_owned())
        .unwrap();
    part.units = Units {
        length: LengthUnit::Inch,
        ..Units::default()
    };
    let r = Regenerator::new().regenerate(&part, None).unwrap();
    assert!((volume(&r.state, 0) - 25.4f64.powi(3)).abs() < 1e-6);
}

#[test]
fn failures_are_reported_per_feature() {
    let mut part = part();
    part.parameters
        .set_expression("size", "30 deg".to_owned())
        .unwrap();
    part.parameters
        .set_expression("r", "-2 mm".to_owned())
        .unwrap();
    let r = Regenerator::new().regenerate(&part, None).unwrap();
    assert!(r.state.bodies.is_empty());
    match r.statuses[0].1 {
        StepStatus::Error(FeatureError::Parameter { field, ref message }) => {
            assert_eq!(field, "Length");
            assert!(message.contains("angle"), "{message}");
        }
        ref other => panic!("{other:?}"),
    }
    // The sphere reads `far`, which reads the angle `size`: still a
    // parameter error, not a skip (features don't depend on each other).
    assert!(matches!(r.statuses[1].1, StepStatus::Error(_)));
    part.parameters
        .set_expression("size", "10 mm".to_owned())
        .unwrap();
    let r = Regenerator::new().regenerate(&part, None).unwrap();
    assert!(matches!(
        r.statuses[1].1,
        StepStatus::Error(FeatureError::Kernel(_))
    ));
    assert_eq!(r.state.bodies.len(), 1);
}

#[test]
fn suppressed_and_rolled_back_features_build_nothing() {
    let mut part = part();
    part.features.set_suppressed(FeatureId(1), true).unwrap();
    let r = Regenerator::new().regenerate(&part, None).unwrap();
    assert_eq!(r.state.bodies.len(), 1);
    assert_eq!(r.statuses[0].1, StepStatus::Suppressed);
    part.features.set_marker(0).unwrap();
    let r = Regenerator::new().regenerate(&part, None).unwrap();
    assert!(r.state.bodies.is_empty());
    assert!(
        r.statuses
            .iter()
            .all(|(_, s)| *s == StepStatus::AfterMarker)
    );
}

#[test]
fn cancelling_stops_and_forgets_partial_work() {
    let part = part();
    let mut regen = Regenerator::new();
    let cancel = Arc::new(AtomicBool::new(true));
    assert!(regen.regenerate(&part, Some(&cancel)).is_err());
    let r = regen.regenerate(&part, None).unwrap();
    assert_eq!(r.evaluated, 2);
    assert_eq!(r.state.bodies.len(), 2);
}

#[test]
fn body_ids_follow_their_feature() {
    let mut part = part();
    let before = Regenerator::new().regenerate(&part, None).unwrap();
    part.features.move_to(FeatureId(2), 0).unwrap();
    let after = Regenerator::new().regenerate(&part, None).unwrap();
    assert_eq!(before.state.bodies[1].id, after.state.bodies[0].id);
}
