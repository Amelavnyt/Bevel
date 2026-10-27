// SPDX-License-Identifier: GPL-3.0-or-later

//! Commands, undo and the `.bevel` file format.

#![allow(clippy::unwrap_used, clippy::panic, reason = "tests")]

use bevel_doc::file::{self, FORMAT_VERSION};
use bevel_doc::{
    BodyProps, Command, CommandError, DocSession, DocState, FileError, ParamEdit, add_primitive,
    sample,
};
use bevel_expr::{ParamKind, Parameter};
use bevel_model::{BodyId, FeatureDef, FeatureId, OriginPlane, PrimitiveKind, Regenerator};
use bevel_timeline::StepStatus;
use std::collections::BTreeMap;
use std::path::PathBuf;

fn add_box(session: &mut DocSession) -> FeatureId {
    let cmd = add_primitive(
        session.state(),
        PrimitiveKind::Box,
        OriginPlane::Xy,
        &BTreeMap::new(),
    );
    session.execute(&cmd).unwrap();
    FeatureId(session.state().next_feature_id - 1)
}

fn param_names(state: &DocState) -> Vec<String> {
    state
        .part
        .parameters
        .params()
        .iter()
        .map(|p| p.name.clone())
        .collect()
}

#[test]
fn the_sample_regenerates_without_errors() {
    let state = sample::primitives();
    assert_eq!(state.part.features.len(), 5);
    let r = Regenerator::new().regenerate(&state.part, None).unwrap();
    assert_eq!(r.state.bodies.len(), 5);
    assert!(
        r.statuses.iter().all(|(_, s)| *s == StepStatus::Ok),
        "{:?}",
        r.statuses
    );
    let names: Vec<_> = state
        .part
        .features
        .steps()
        .iter()
        .map(|s| s.op().name.clone())
        .collect();
    assert_eq!(names, ["Box1", "Cylinder1", "Sphere1", "Cone1", "Torus1"]);
}

#[test]
fn adding_a_primitive_creates_model_parameters() {
    let mut session = DocSession::default();
    let id = add_box(&mut session);
    let state = session.state();
    assert_eq!(param_names(state), ["d1", "d2", "d3", "d4", "d5", "d6"]);
    assert!(
        state
            .part
            .parameters
            .params()
            .iter()
            .all(|p| p.kind == ParamKind::Model)
    );
    assert_eq!(state.feature(id).unwrap().name, "Box1");
    assert_eq!(session.undo_label(), Some("Add Box"));
    let second = add_box(&mut session);
    assert_eq!(session.state().feature(second).unwrap().name, "Box2");
    let d12 = session
        .state()
        .part
        .parameters
        .get("d12")
        .map(|p| p.expression.clone());
    assert_eq!(d12.as_deref(), Some("0 mm"));
}

#[test]
fn undo_and_redo_restore_states_and_skip_marker_moves() {
    let mut session = DocSession::default();
    let a = add_box(&mut session);
    add_box(&mut session);
    session.execute(&Command::SetMarker { index: 1 }).unwrap();
    assert_eq!(
        session.undo_label(),
        Some("Add Box"),
        "marker moves are not undo steps"
    );
    let rename = Command::RenameFeature {
        id: a,
        name: "Base".to_owned(),
    };
    session.execute(&rename).unwrap();
    assert_eq!(session.undo().as_deref(), Some("Rename Box1"));
    assert_eq!(session.state().feature(a).unwrap().name, "Box1");
    assert_eq!(session.redo().as_deref(), Some("Rename Box1"));
    assert_eq!(session.state().feature(a).unwrap().name, "Base");
    session.undo();
    session.undo();
    session.undo();
    assert!(session.state().part.features.is_empty());
    assert!(session.undo().is_none());
}

#[test]
fn refused_commands_change_nothing() {
    let mut session = DocSession::default();
    let id = add_box(&mut session);
    let before = file::write(session.state()).unwrap();
    let batch = Command::Batch {
        label: "Two things".to_owned(),
        cmds: vec![
            Command::RenameFeature {
                id,
                name: "Fine".to_owned(),
            },
            Command::RenameFeature {
                id,
                name: "  ".to_owned(),
            },
        ],
    };
    assert_eq!(session.execute(&batch), Err(CommandError::EmptyName));
    assert_eq!(file::write(session.state()).unwrap(), before);
    let missing = Command::DeleteFeature {
        id: FeatureId(99),
        cascade: false,
    };
    assert_eq!(
        session.execute(&missing),
        Err(CommandError::NoFeature(FeatureId(99)))
    );
    let in_use = Command::Parameters(ParamEdit::Remove {
        name: "d1".to_owned(),
    });
    assert!(matches!(
        session.execute(&in_use),
        Err(CommandError::ParameterInUse { .. })
    ));
}

#[test]
fn deleting_a_feature_deletes_its_model_parameters() {
    let mut session = DocSession::default();
    let a = add_box(&mut session);
    let twice = Command::Parameters(ParamEdit::Add(Parameter {
        name: "twice".to_owned(),
        expression: "d1 * 2".to_owned(),
        comment: String::new(),
        kind: ParamKind::User,
    }));
    session.execute(&twice).unwrap();
    session
        .execute(&Command::DeleteFeature {
            id: a,
            cascade: false,
        })
        .unwrap();
    assert_eq!(
        param_names(session.state()),
        ["d1", "twice"],
        "d1 stays: `twice` reads it"
    );
}

#[test]
fn renaming_a_parameter_follows_into_features() {
    let mut session = DocSession::default();
    let a = add_box(&mut session);
    let rename = Command::Parameters(ParamEdit::Rename {
        old: "d1".to_owned(),
        new: "length".to_owned(),
    });
    session.execute(&rename).unwrap();
    let FeatureDef::Primitive(p) = &session.state().feature(a).unwrap().def;
    assert_eq!(p.inputs["length"], "length");
    let r = Regenerator::new()
        .regenerate(&session.state().part, None)
        .unwrap();
    assert_eq!(r.state.bodies.len(), 1);
}

#[test]
fn commands_serialize() {
    let state = DocState::new();
    let cmd = add_primitive(
        &state,
        PrimitiveKind::Torus,
        OriginPlane::Yz,
        &BTreeMap::new(),
    );
    let back: Command = serde_json::from_str(&serde_json::to_string(&cmd).unwrap()).unwrap();
    assert_eq!(back, cmd);
    let props = Command::SetBodyProps {
        body: BodyId {
            feature: FeatureId(1),
            index: 0,
        },
        props: BodyProps {
            name: Some("Lid".to_owned()),
            color: Some([1, 2, 3]),
            visible: false,
        },
    };
    let back: Command = serde_json::from_str(&serde_json::to_string(&props).unwrap()).unwrap();
    assert_eq!(back, props);
}

#[test]
fn files_round_trip_byte_for_byte() {
    let mut session = DocSession::new(sample::primitives());
    let suppress = Command::SetSuppressed {
        id: FeatureId(2),
        suppressed: true,
    };
    session.execute(&suppress).unwrap();
    session.execute(&Command::SetMarker { index: 4 }).unwrap();
    let hide = Command::SetBodyProps {
        body: BodyId {
            feature: FeatureId(1),
            index: 0,
        },
        props: BodyProps {
            visible: false,
            ..BodyProps::default()
        },
    };
    session.execute(&hide).unwrap();
    let bytes = file::write(session.state()).unwrap();
    assert_eq!(
        file::write(session.state()).unwrap(),
        bytes,
        "deterministic"
    );
    let back = file::read(&bytes).unwrap();
    assert_eq!(file::write(&back).unwrap(), bytes);
    assert_eq!(back.part.features.marker(), 4);
    assert!(back.part.features.get(FeatureId(2)).unwrap().suppressed());
    assert_eq!(back.next_feature_id, 6);
    assert!(
        !back
            .body_props(BodyId {
                feature: FeatureId(1),
                index: 0
            })
            .visible
    );
}

#[test]
fn damaged_and_foreign_files_are_refused() {
    assert!(matches!(
        file::read(b"not a zip"),
        Err(FileError::Container(_))
    ));
    let bytes = file::write(&sample::primitives()).unwrap();
    for i in (0..bytes.len()).step_by(97) {
        let mut damaged = bytes.clone();
        damaged[i] ^= 0x55;
        // Never a panic, and never a silently different document.
        if let Ok(state) = file::read(&damaged) {
            assert_eq!(file::write(&state).unwrap(), bytes, "byte {i}");
        }
    }
    assert!(matches!(
        file::migrate(serde_json::Value::Null, FORMAT_VERSION + 1),
        Err(FileError::NewerVersion { .. })
    ));
}

fn fixtures() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/bevel")
}

/// Every stored `.bevel` fixture, from every past format version, still
/// loads and regenerates. Set `BEVEL_WRITE_FIXTURES=1` to (re)write the
/// current version's fixtures.
#[test]
fn every_fixture_version_loads() {
    let current = fixtures().join(format!("v{FORMAT_VERSION}"));
    if std::env::var_os("BEVEL_WRITE_FIXTURES").is_some() {
        std::fs::create_dir_all(&current).unwrap();
        let bytes = file::write(&sample::primitives()).unwrap();
        std::fs::write(current.join("primitives.bevel"), bytes).unwrap();
    }
    let mut loaded = 0;
    for version in std::fs::read_dir(fixtures()).unwrap() {
        for entry in std::fs::read_dir(version.unwrap().path()).unwrap() {
            let path = entry.unwrap().path();
            let state = file::read(&std::fs::read(&path).unwrap())
                .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
            let r = Regenerator::new().regenerate(&state.part, None).unwrap();
            assert!(
                r.statuses
                    .iter()
                    .all(|(_, s)| !matches!(s, StepStatus::Error(_))),
                "{}",
                path.display()
            );
            loaded += 1;
        }
    }
    assert!(loaded > 0, "no fixtures in {}", fixtures().display());
}
