// SPDX-License-Identifier: GPL-3.0-or-later

//! Property tests for tessellation: every primitive with random sizes
//! under random rigid frames, at every preset, gives a watertight,
//! outward, non-degenerate mesh on its surfaces, within the tolerances,
//! that converges to the exact volume, tightens monotonically and is
//! deterministic.

// Helpers outside `#[test]` functions unwrap freely too.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod common;

use std::collections::BTreeMap;

use bevel_geom::{CurveGeom, SurfaceGeom};
use bevel_kernel::{Body, BodyMesh, OpContext, TessellationOptions, tessellate};
use bevel_math::{Point3, Vector3};
use common::{Shape, cone_ok, cone_radius, config, frame, length, torus_ok};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

/// Relative accuracy of points that should lie exactly on a surface or
/// curve: `1e-9` of the body's diagonal, plus the rounding of coordinates
/// up to 500 mm from the origin (`4 eps |p|`).
const ON_SURFACE: f64 = 1e-9;

/// Slack on the chordal test of triangle centroids, as a fraction of the
/// body's diagonal: rounding only, since refinement checks the deviation
/// at every centroid against an upper bound of the distance.
const CHORDAL_SLACK: f64 = 1e-9;

/// The most triangles a primitive may get, per preset. The worst case is
/// a thin torus, whose tube needs as many rings at its small radius as at
/// the body's size; the bounds are about four times the largest counts
/// these tests see with the fixed seed (2714, 11 312 and 53 174).
const MAX_TRIANGLES: [usize; 3] = [12_000, 50_000, 220_000];

/// The volume error bound `C` in `|V_mesh - V| <= C chordal area`: a flat
/// triangle stays within about `chordal` of the surface, so the slab
/// between mesh and surface has at most `chordal * area` volume; `C = 2`
/// leaves room for the deviation between the sampled points.
const VOLUME_FACTOR: f64 = 2.0;

/// The presets in tightening order.
fn presets(body: &Body) -> [TessellationOptions; 3] {
    let b = body.bounds();
    [
        TessellationOptions::coarse(&b),
        TessellationOptions::fine(&b),
        TessellationOptions::export(&b),
    ]
}

/// The bits of a point, for welding.
fn bits(p: Point3) -> [u64; 3] {
    [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]
}

/// The volume enclosed by the mesh (divergence theorem over the
/// triangles), about `origin` to avoid cancellation.
fn mesh_volume(mesh: &BodyMesh, origin: Point3) -> f64 {
    let mut v = 0.0;
    for f in &mesh.faces {
        for t in &f.triangles {
            let [a, b, c] = t.map(|i| f.positions[i as usize] - origin);
            v += a.dot(b.cross(c));
        }
    }
    v / 6.0
}

/// Checks one mesh of `body` made with `options`.
#[allow(clippy::too_many_lines)]
fn check_mesh(
    body: &Body,
    mesh: &BodyMesh,
    options: &TessellationOptions,
    max_triangles: usize,
) -> Result<(), TestCaseError> {
    let diagonal = body.bounds().size().length();
    let near = |p: Point3| ON_SURFACE * diagonal + 4.0 * f64::EPSILON * p.abs().max_element();

    // One mesh per face, in order.
    let face_keys: Vec<_> = body.faces().map(|(k, _)| k).collect();
    let mesh_keys: Vec<_> = mesh.faces.iter().map(|f| f.face).collect();
    prop_assert_eq!(face_keys, mesh_keys);
    prop_assert!(
        mesh.triangle_count() <= max_triangles,
        "{} triangles",
        mesh.triangle_count()
    );

    // Watertight: weld by bits, every undirected edge used exactly once in
    // each direction.
    let mut weld: BTreeMap<[u64; 3], usize> = BTreeMap::new();
    let mut directed: BTreeMap<(usize, usize), u32> = BTreeMap::new();
    for f in &mesh.faces {
        let face = body.face(f.face).unwrap();
        prop_assert_eq!(f.positions.len(), f.normals.len());
        prop_assert!(!f.triangles.is_empty(), "{} has no triangles", face.name);
        let ids: Vec<usize> = f
            .positions
            .iter()
            .map(|&p| {
                let n = weld.len();
                *weld.entry(bits(p)).or_insert(n)
            })
            .collect();
        for t in &f.triangles {
            prop_assert!(t.iter().all(|&i| (i as usize) < f.positions.len()));
            let [a, b, c] = t.map(|i| f.positions[i as usize]);
            let cross = (b - a).cross(c - a);
            prop_assert!(cross.length() > 0.0, "zero-area triangle in {}", face.name);
            let w = t.map(|i| ids[i as usize]);
            for k in 0..3 {
                let e = (w[k], w[(k + 1) % 3]);
                prop_assert!(e.0 != e.1, "collapsed edge in {}", face.name);
                *directed.entry(e).or_insert(0) += 1;
            }

            // The centroid is within chordal of the surface, and the
            // triangle faces the same way as the surface there.
            let centroid = (a + b + c) / 3.0;
            let cp = face.surface.closest_point(centroid).unwrap();
            prop_assert!(
                cp.distance <= options.chordal + CHORDAL_SLACK * diagonal,
                "{}: centroid {} from the surface, chordal {}",
                face.name,
                cp.distance,
                options.chordal
            );
            if let Some(n) = face.surface.normal(cp.u, cp.v) {
                let n = if face.reversed { -n } else { n };
                prop_assert!(cross.dot(n) > 0.0, "{}: triangle faces inwards", face.name);
            }
        }
        // Positions on the surface; normals unit, outward and within
        // `angular` of the exact normal.
        for (p, n) in f.positions.iter().zip(&f.normals) {
            let cp = face.surface.closest_point(*p).unwrap();
            prop_assert!(
                cp.distance <= near(*p),
                "{}: a point {} off the surface",
                face.name,
                cp.distance
            );
            prop_assert!((n.length() - 1.0).abs() < 1e-12);
            if let Some(exact) = face.surface.normal(cp.u, cp.v) {
                let exact = if face.reversed { -exact } else { exact };
                let angle = n.cross(exact).length().atan2(n.dot(exact));
                prop_assert!(
                    angle <= options.angular + 1e-9,
                    "{}: vertex normal {} rad off",
                    face.name,
                    angle
                );
            }
        }
    }
    for (&(a, b), &count) in &directed {
        prop_assert_eq!(count, 1, "edge {:?} used {} times one way", (a, b), count);
        prop_assert_eq!(
            directed.get(&(b, a)).copied(),
            Some(1),
            "edge {:?} is open",
            (a, b)
        );
    }

    // Edge polylines: one per non-degenerate edge, from vertex to vertex,
    // on the curve.
    let edges: Vec<_> = body
        .edges()
        .filter(|(_, e)| !e.degenerate)
        .map(|(k, _)| k)
        .collect();
    let polyline_edges: Vec<_> = mesh.edges.iter().map(|p| p.edge).collect();
    prop_assert_eq!(edges, polyline_edges);
    for pl in &mesh.edges {
        let e = body.edge(pl.edge).unwrap();
        let start = body.vertex(e.start).unwrap().point;
        let end = body.vertex(e.end).unwrap().point;
        prop_assert!(pl.points.len() >= 2);
        prop_assert_eq!(bits(pl.points[0]), bits(start));
        prop_assert_eq!(bits(*pl.points.last().unwrap()), bits(end));
        for &p in &pl.points {
            let cp = e.curve.closest_point(p).unwrap();
            prop_assert!(
                cp.distance <= near(p),
                "a polyline point {} off its curve",
                cp.distance
            );
        }
    }
    Ok(())
}

/// Every check, at every preset, on one primitive.
fn check_shape(shape: &Shape) -> Result<(), TestCaseError> {
    let body = shape.build();
    let exact = shape.exact();
    let cx = OpContext::new();
    let mut previous = 0;
    for (k, options) in presets(&body).iter().enumerate() {
        let mesh = tessellate(&body, options, &cx).unwrap();
        check_mesh(&body, &mesh, options, MAX_TRIANGLES[k])?;

        // Tightening never reduces the triangle count.
        let count = mesh.triangle_count();
        prop_assert!(
            count >= previous,
            "preset {k}: {count} < {previous} triangles"
        );
        previous = count;

        // The mesh volume converges to the exact volume.
        let v = mesh_volume(&mesh, shape.frame().origin);
        let bound = VOLUME_FACTOR * options.chordal * exact.area;
        prop_assert!(
            (v - exact.volume).abs() <= bound,
            "preset {k}: volume {v} vs {} (bound {bound})",
            exact.volume
        );

        // Deterministic, bit for bit.
        if k == 1 {
            let again = tessellate(&body, options, &cx).unwrap();
            prop_assert!(same_mesh(&mesh, &again), "tessellating twice differs");
        }
    }
    Ok(())
}

/// Returns `true` if two meshes are identical to the bit.
fn same_mesh(a: &BodyMesh, b: &BodyMesh) -> bool {
    let vbits = |v: &[Vector3]| v.iter().map(|&p| bits(p)).collect::<Vec<_>>();
    a.faces.len() == b.faces.len()
        && a.edges.len() == b.edges.len()
        && a.faces.iter().zip(&b.faces).all(|(x, y)| {
            x.face == y.face
                && vbits(&x.positions) == vbits(&y.positions)
                && vbits(&x.normals) == vbits(&y.normals)
                && x.triangles == y.triangles
        })
        && a.edges
            .iter()
            .zip(&b.edges)
            .all(|(x, y)| x.edge == y.edge && vbits(&x.points) == vbits(&y.points))
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn box_meshes_are_watertight_and_accurate(
        frame in frame(), x in length(), y in length(), z in length(),
    ) {
        check_shape(&Shape::Box(frame, Vector3::new(x, y, z)))?;
    }

    #[test]
    fn cylinder_meshes_are_watertight_and_accurate(frame in frame(), r in length(), h in length()) {
        check_shape(&Shape::Cylinder(frame, r, h))?;
    }

    #[test]
    fn sphere_meshes_are_watertight_and_accurate(frame in frame(), r in length()) {
        check_shape(&Shape::Sphere(frame, r))?;
    }

    #[test]
    fn cone_meshes_are_watertight_and_accurate(
        frame in frame(), r0 in cone_radius(), r1 in cone_radius(), h in length(),
    ) {
        prop_assume!(cone_ok(r0, r1, h));
        check_shape(&Shape::Cone(frame, r0, r1, h))?;
    }

    #[test]
    fn torus_meshes_are_watertight_and_accurate(
        frame in frame(), big in length(), fraction in 0.02f64..0.98,
    ) {
        let small = big * fraction;
        prop_assume!(torus_ok(big, small));
        check_shape(&Shape::Torus(frame, big, small))?;
    }
}
