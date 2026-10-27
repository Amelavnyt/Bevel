// SPDX-License-Identifier: GPL-3.0-or-later

//! Serialization of curves and surfaces: round trips are exact, and
//! corrupt data is refused by the constructors' checks.

// Helpers outside `#[test]` functions unwrap freely too.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::FRAC_PI_6;

use bevel_geom::{
    Circle2, Circle3, Cone, Curve2, Curve3, Cylinder, Ellipse2, Ellipse3, KnotVector, Line2, Line3,
    NurbsCurve2, NurbsCurve3, NurbsSurface, ParamMap, Plane, Sphere, Surface, Torus, circle_arc,
};
use bevel_math::{DQuat, DVec3, Frame3, Interval, Point2, Point3, Vector2, Vector3};
use serde::Serialize;
use serde::de::DeserializeOwned;

/// Serializes `value` to JSON and back, checking that nothing changed.
fn round_trip<T: Serialize + DeserializeOwned + PartialEq + std::fmt::Debug>(value: &T) -> String {
    let json = serde_json::to_string(value).unwrap();
    let back: T = serde_json::from_str(&json).unwrap();
    assert_eq!(&back, value, "{json}");
    json
}

/// A frame that is not axis aligned, so rounding in the axes is exercised.
fn tilted() -> Frame3 {
    let q = DQuat::from_axis_angle(DVec3::new(1.0, 2.0, -0.5).normalize(), 0.7);
    Frame3 {
        origin: Point3::new(3.0, -4.0, 5.5),
        x: q * DVec3::X,
        y: q * DVec3::Y,
        z: q * DVec3::Z,
    }
}

#[test]
fn analytic_curves_round_trip() {
    let f = tilted();
    round_trip(&Line2::new(Point2::new(1.0, 2.0), Vector2::new(3.0, -1.0)).unwrap());
    round_trip(&Line3::new(f.origin, Vector3::new(0.3, -2.0, 0.7)).unwrap());
    round_trip(
        &Circle2::from_axis(Point2::new(1.0, 2.0), Vector2::new(1.0, 1.0), false, 2.5).unwrap(),
    );
    round_trip(&Circle3::new(f, 4.0).unwrap());
    round_trip(&Ellipse2::new(Point2::new(-1.0, 0.0), Vector2::new(0.2, 1.0), 3.0, 1.0).unwrap());
    round_trip(&Ellipse3::new(f, 3.0, 2.0).unwrap());
}

#[test]
fn analytic_surfaces_round_trip() {
    let f = tilted();
    round_trip(&Plane::new(f).unwrap());
    round_trip(&Cylinder::new(f, 2.0).unwrap());
    round_trip(&Cone::new(f, 0.0, FRAC_PI_6).unwrap());
    round_trip(&Sphere::new(f, 3.0).unwrap());
    round_trip(&Torus::new(f, 5.0, 1.5).unwrap());
}

#[test]
fn nurbs_and_enums_round_trip() {
    let arc = circle_arc(&tilted(), 2.0, 0.0, 4.0).unwrap();
    assert!(arc.is_rational());
    let json = round_trip(&arc);
    assert!(json.contains("\"degree\":2"));
    let back: NurbsCurve3 = serde_json::from_str(&json).unwrap();
    assert!(back.is_rational(), "the cached rational flag is recomputed");

    let planar = NurbsCurve2::bezier(&[Point2::ZERO, Point2::new(1.0, 2.0), Point2::X]).unwrap();
    round_trip(&planar);
    let sphere = Sphere::new(tilted(), 3.0).unwrap();
    let nurbs = sphere.to_nurbs_full().unwrap();
    round_trip(&nurbs);

    round_trip(&Curve3::Nurbs(arc.clone()));
    round_trip(&Curve3::Circle(Circle3::new(tilted(), 1.0).unwrap()));
    round_trip(&Curve2::Line(Line2::new(Point2::ZERO, Vector2::Y).unwrap()));
    round_trip(&Surface::Sphere(sphere));
    round_trip(&Surface::Nurbs(nurbs));

    let converted = Circle3::new(tilted(), 2.0)
        .unwrap()
        .to_nurbs_mapped(Interval::new(0.0, 5.0))
        .unwrap();
    round_trip(&converted.map);
    round_trip(&ParamMap::Identity);
}

#[test]
fn json_is_readable() {
    let line = Line3::new(Point3::new(1.0, 2.0, 3.0), Vector3::Z).unwrap();
    assert_eq!(
        serde_json::to_string(&Curve3::Line(line)).unwrap(),
        r#"{"Line":{"origin":[1.0,2.0,3.0],"direction":[0.0,0.0,1.0]}}"#
    );
    let knots = KnotVector::new(vec![0.0, 0.0, 0.5, 1.0, 1.0], 1).unwrap();
    assert_eq!(
        serde_json::to_string(&knots).unwrap(),
        "[0.0,0.0,0.5,1.0,1.0]"
    );
}

/// Asserts that `json` does not deserialize as a `T`.
fn refused<T: DeserializeOwned + std::fmt::Debug>(json: &str) {
    let result = serde_json::from_str::<T>(json);
    assert!(result.is_err(), "accepted {json}: {result:?}");
}

#[test]
fn corrupt_knot_vectors_are_refused() {
    refused::<KnotVector>("[0.0, 0.0, 2.0, 1.0, 3.0, 3.0]");
    refused::<KnotVector>("[0.0, 1.0, 1.0]");
    refused::<KnotVector>("[1.0, 1.0, 1.0, 1.0]");
    // A vector fine on its own but not for the curve's degree.
    let curve = r#"{"degree":3,"knots":[0,0,1,1],"control":[[0,0,0,1],[1,0,0,1]]}"#;
    refused::<NurbsCurve3>(curve);
    // A decreasing knot vector inside a curve.
    let curve =
        r#"{"degree":1,"knots":[0,0,2,1,3,3],"control":[[0,0,0,1],[1,0,0,1],[2,0,0,1],[3,0,0,1]]}"#;
    refused::<NurbsCurve3>(curve);
}

#[test]
fn corrupt_nurbs_are_refused() {
    let wrong_count = r#"{"degree":1,"knots":[0,0,1,1],"control":[[0,0,1]]}"#;
    refused::<NurbsCurve2>(wrong_count);
    let zero_weight = r#"{"degree":1,"knots":[0,0,1,1],"control":[[0,0,1],[0,0,0]]}"#;
    refused::<NurbsCurve2>(zero_weight);
    let negative_weight = r#"{"degree":1,"knots":[0,0,1,1],"control":[[0,0,1],[1,1,-1]]}"#;
    refused::<NurbsCurve2>(negative_weight);
    let surface = r#"{"degree_u":1,"degree_v":1,"knots_u":[0,0,1,1],"knots_v":[0,0,1,1],
        "control":[[0,0,0,1],[1,0,0,1],[0,1,0,1]]}"#;
    refused::<NurbsSurface>(surface);
}

#[test]
fn corrupt_analytic_geometry_is_refused() {
    let world = r#"{"origin":[0,0,0],"x":[1,0,0],"y":[0,1,0],"z":[0,0,1]}"#;
    let skewed = r#"{"origin":[0,0,0],"x":[1,0,0],"y":[0.5,1,0],"z":[0,0,1]}"#;
    refused::<Circle3>(&format!(r#"{{"frame":{world},"radius":0.0}}"#));
    refused::<Circle3>(&format!(r#"{{"frame":{world},"radius":-1.0}}"#));
    refused::<Circle3>(&format!(r#"{{"frame":{skewed},"radius":1.0}}"#));
    refused::<Cylinder>(&format!(r#"{{"frame":{world},"radius":0.0}}"#));
    refused::<Sphere>(&format!(r#"{{"frame":{world},"radius":0.0}}"#));
    refused::<Cone>(&format!(
        r#"{{"frame":{world},"radius":1.0,"half_angle":2.0}}"#
    ));
    refused::<Cone>(&format!(
        r#"{{"frame":{world},"radius":-1.0,"half_angle":0.5}}"#
    ));
    refused::<Torus>(&format!(r#"{{"frame":{world},"major":1.0,"minor":2.0}}"#));
    refused::<Ellipse3>(&format!(r#"{{"frame":{world},"major":1.0,"minor":2.0}}"#));
    refused::<Plane>(&format!(r#"{{"frame":{skewed}}}"#));
    refused::<Line3>(r#"{"origin":[0,0,0],"direction":[0,0,0]}"#);
    refused::<Line3>(r#"{"origin":[0,0,0],"direction":[0,0,2]}"#);
    refused::<Line2>(r#"{"origin":[0,0],"direction":[0.5,0]}"#);
    refused::<Circle2>(r#"{"center":[0,0],"x":[1,0],"y":[0.5,0.5],"radius":1}"#);
    refused::<Ellipse2>(r#"{"center":[0,0],"x":[1,0],"y":[0,1],"major":1,"minor":0}"#);
    refused::<ParamMap>(r#"{"Arc":{"breaks":[0.0]}}"#);
    refused::<ParamMap>(r#"{"Arc":{"breaks":[0.0,2.0,1.0]}}"#);
    refused::<Surface>(&format!(r#"{{"Sphere":{{"frame":{world},"radius":0.0}}}}"#));
}

#[test]
fn round_tripped_geometry_still_evaluates_the_same() {
    let c = Circle3::new(tilted(), 2.0).unwrap();
    let back: Circle3 = serde_json::from_str(&serde_json::to_string(&c).unwrap()).unwrap();
    for t in [0.0, 1.0, 2.5] {
        assert!(c.point(t).distance(back.point(t)) <= 1e-15);
    }
}
