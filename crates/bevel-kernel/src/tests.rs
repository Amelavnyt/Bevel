// SPDX-License-Identifier: GPL-3.0-or-later

//! Bodies broken on purpose: each defect must produce the expected issue,
//! and the validator must not panic whatever the arenas hold.

use std::sync::Arc;

use bevel_geom::{Curve2, Line2};
use bevel_math::{Frame3, Point2, Point3, Vector2, Vector3};

use crate::{
    Body, BoxParams, CylinderParams, EntityKey, Issue, OpContext, OpTag, SphereParams, box_,
    cylinder, sphere, validate,
};

fn unwrap_body(out: crate::OpOutput) -> Body {
    let arc = out.bodies.into_iter().next().unwrap();
    Arc::try_unwrap(arc).unwrap()
}

fn unit_box() -> Body {
    let params = BoxParams {
        frame: Frame3::WORLD,
        size: Vector3::ONE,
    };
    unwrap_body(box_(&params, OpTag(1), &mut OpContext::new()).unwrap())
}

fn unit_cylinder() -> Body {
    let params = CylinderParams {
        frame: Frame3::WORLD,
        radius: 1.0,
        height: 2.0,
    };
    unwrap_body(cylinder(&params, OpTag(1), &mut OpContext::new()).unwrap())
}

/// Asserts that some issue matches `pred`, printing all issues otherwise.
fn expect(issues: &[Issue], what: &str, pred: impl Fn(&Issue) -> bool) {
    assert!(
        issues.iter().any(pred),
        "expected {what}, got:\n{}",
        issues
            .iter()
            .map(|i| format!("  {i}"))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

#[test]
fn primitives_start_out_valid() {
    assert_eq!(validate(&unit_box()), vec![]);
    assert_eq!(validate(&unit_cylinder()), vec![]);
}

#[test]
fn missing_coedge_is_reported() {
    let mut b = unit_box();
    let (ck, _) = b.coedges.iter().next().unwrap();
    b.coedges.remove(ck);
    let issues = validate(&b);
    expect(
        &issues,
        "a loop pointing at the missing coedge",
        |i| matches!(i, Issue::MissingEntity { from: EntityKey::Loop(_), to } if *to == EntityKey::Coedge(ck)),
    );
    // The edge has lost one of its two uses.
    expect(&issues, "an edge with one coedge", |i| {
        matches!(i, Issue::EdgeUseCount { count: 1, .. })
    });
}

#[test]
fn flipped_coedge_is_reported() {
    let mut b = unit_box();
    let (ck, c) = b.coedges.iter_mut().next().unwrap();
    c.reversed = !c.reversed;
    let edge = c.edge;
    let issues = validate(&b);
    expect(
        &issues,
        "inconsistent orientation",
        |i| matches!(i, Issue::InconsistentOrientation { edge: e, coedges } if *e == edge && coedges.contains(&ck)),
    );
    expect(&issues, "an open loop", |i| {
        matches!(i, Issue::LoopVertexGap { .. })
    });
}

#[test]
fn open_loop_is_reported() {
    let mut b = unit_box();
    let (lk, l) = b.loops.iter_mut().next().unwrap();
    let dropped = l.coedges.pop().unwrap();
    let issues = validate(&b);
    expect(
        &issues,
        "a vertex gap in the loop",
        |i| matches!(i, Issue::LoopVertexGap { loop_, .. } if *loop_ == lk),
    );
    expect(
        &issues,
        "a coedge no loop lists",
        |i| matches!(i, Issue::BackReference { entity, .. } if *entity == EntityKey::Coedge(dropped)),
    );
}

#[test]
fn mismatched_pcurve_is_reported() {
    let mut b = unit_box();
    let (ck, c) = b.coedges.iter_mut().next().unwrap();
    c.pcurve = Curve2::Line(Line2::new(Point2::new(0.25, 0.25), Vector2::X).unwrap());
    let issues = validate(&b);
    expect(
        &issues,
        "a pcurve mismatch",
        |i| matches!(i, Issue::PcurveMismatch { coedge, distance } if *coedge == ck && (*distance - 0.25 * std::f64::consts::SQRT_2).abs() < 1e-12),
    );
    expect(&issues, "a pcurve gap", |i| {
        matches!(i, Issue::PcurveGap { .. })
    });
}

#[test]
fn wrong_vertex_position_is_reported() {
    let mut b = unit_box();
    let (vk, v) = b.vertices.iter_mut().next().unwrap();
    v.point += Vector3::new(0.0, 0.0, 0.1);
    let issues = validate(&b);
    let mismatches = issues
        .iter()
        .filter(|i| matches!(i, Issue::EdgeVertexMismatch { vertex, .. } if *vertex == vk))
        .count();
    assert_eq!(mismatches, 3, "three edges meet at a box corner");
    expect(
        &issues,
        "the vertex off an edge",
        |i| matches!(i, Issue::VertexOffEdge { vertex, .. } if *vertex == vk),
    );
}

#[test]
fn duplicate_name_is_reported() {
    let mut b = unit_box();
    let mut faces = b.faces.iter_mut();
    let (k0, f0) = faces.next().unwrap();
    let (k1, f1) = faces.next().unwrap();
    f1.name = f0.name.clone();
    let issues = validate(&b);
    assert_eq!(
        issues,
        vec![Issue::DuplicateName {
            name: b.faces[k0].name.clone(),
            entities: vec![EntityKey::Face(k0), EntityKey::Face(k1)],
        }]
    );
}

#[test]
fn reversed_face_turns_the_wrong_way() {
    let mut b = unit_box();
    let (fk, f) = b.faces.iter_mut().next().unwrap();
    f.reversed = true;
    let outer = f.loops[0];
    let issues = validate(&b);
    assert!(matches!(
        issues.as_slice(),
        [Issue::LoopOrientation { loop_, area }] if *loop_ == outer && *area > 0.0
    ));
    assert!(issues[0].to_string().contains("wrong way"), "{fk:?}");
}

#[test]
fn extra_loop_breaks_euler_poincare() {
    let mut b = unit_box();
    let (_, f) = b.faces.iter_mut().next().unwrap();
    let outer = f.loops[0];
    f.loops.push(outer);
    let issues = validate(&b);
    expect(
        &issues,
        "an Euler-Poincare mismatch",
        |i| matches!(i, Issue::EulerPoincare { counts, .. } if counts.loops == 7 && counts.characteristic() == 1),
    );
    expect(&issues, "a loop listed twice", |i| {
        matches!(i, Issue::ListedTwice { .. })
    });
}

#[test]
fn false_degenerate_flag_is_reported() {
    let mut b = unit_box();
    let (ek, e) = b.edges.iter_mut().next().unwrap();
    e.degenerate = true;
    let issues = validate(&b);
    expect(
        &issues,
        "a degenerate mismatch",
        |i| matches!(i, Issue::DegenerateMismatch { edge, .. } if *edge == ek),
    );
    expect(
        &issues,
        "a degenerate edge with two uses",
        |i| matches!(i, Issue::EdgeUseCount { edge, count: 2 } if *edge == ek),
    );
}

#[test]
fn unmarked_degenerate_edge_is_reported() {
    let params = SphereParams {
        frame: Frame3::WORLD,
        radius: 1.0,
    };
    let mut b = unwrap_body(sphere(&params, OpTag(1), &mut OpContext::new()).unwrap());
    let (ek, e) = b.edges.iter_mut().find(|(_, e)| e.degenerate).unwrap();
    e.degenerate = false;
    let issues = validate(&b);
    expect(
        &issues,
        "a zero-length edge not marked degenerate",
        |i| matches!(i, Issue::DegenerateMismatch { edge, .. } if *edge == ek),
    );
}

#[test]
fn seam_on_one_side_is_reported() {
    let mut b = unit_cylinder();
    // Move the seam's downward pcurve (u = 0) onto the upward one (u = 2 pi).
    let seam = b.edges.iter().find(|(k, _)| b.is_seam(*k)).unwrap().0;
    let (_, c) = b
        .coedges
        .iter_mut()
        .find(|(_, c)| c.edge == seam && c.reversed)
        .unwrap();
    c.pcurve =
        Curve2::Line(Line2::new(Point2::new(std::f64::consts::TAU, 0.0), Vector2::Y).unwrap());
    let issues = validate(&b);
    expect(
        &issues,
        "a bad seam",
        |i| matches!(i, Issue::BadSeam { edge, .. } if *edge == seam),
    );
}

#[test]
fn open_shell_in_a_solid_is_reported() {
    let mut b = unit_box();
    let (sk, s) = b.shell_arena.iter_mut().next().unwrap();
    s.closed = false;
    assert_eq!(validate(&b), vec![Issue::OpenShellInSolid { shell: sk }]);
}

#[test]
fn bad_tolerance_and_far_vertex_are_reported() {
    let mut b = unit_box();
    let (vk, v) = b.vertices.iter_mut().next().unwrap();
    v.tol = f64::NAN;
    let (ek, e) = b.edges.iter_mut().next().unwrap();
    e.tol = 0.0;
    let issues = validate(&b);
    expect(
        &issues,
        "a NaN vertex tolerance",
        |i| matches!(i, Issue::InvalidTolerance { entity, .. } if *entity == EntityKey::Vertex(vk)),
    );
    expect(
        &issues,
        "a zero edge tolerance",
        |i| matches!(i, Issue::InvalidTolerance { entity, .. } if *entity == EntityKey::Edge(ek)),
    );
    let mut b = unit_box();
    let (vk, v) = b.vertices.iter_mut().next().unwrap();
    v.point = Point3::new(2e5, 0.0, 0.0);
    expect(
        &validate(&b),
        "a vertex outside the model space",
        |i| matches!(i, Issue::OutsideModelSpace { vertex } if *vertex == vk),
    );
}

#[test]
fn validator_survives_arbitrary_removals() {
    // Remove every entity of one kind at a time, then one of each kind.
    type Mutation = fn(&mut Body);
    let mutations: [Mutation; 7] = [
        |b| b.vertices.clear(),
        |b| b.edges.clear(),
        |b| b.coedges.clear(),
        |b| b.loops.clear(),
        |b| b.faces.clear(),
        |b| b.shell_arena.clear(),
        |b| {
            let v = b.vertices.keys().next().unwrap();
            b.vertices.remove(v);
            let e = b.edges.keys().nth(1).unwrap();
            b.edges.remove(e);
            let l = b.loops.keys().nth(1).unwrap();
            b.loops.remove(l);
            b.shells.clear();
        },
    ];
    for body in [unit_box(), unit_cylinder()] {
        for m in mutations {
            let mut b = body.clone();
            m(&mut b);
            let issues = validate(&b);
            assert!(!issues.is_empty());
            for i in &issues {
                assert!(!i.to_string().is_empty());
            }
            // A broken body cannot be serialized, and equals nothing.
            assert!(serde_json::to_string(&b).is_err());
            assert_ne!(b, body);
        }
    }
}
