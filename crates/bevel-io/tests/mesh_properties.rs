// SPDX-License-Identifier: GPL-3.0-or-later

//! Property tests: random closed meshes stay watertight and survive the
//! 3MF (exactly) and STL (to `f32` rounding) round trips.
//!
//! The generator is seeded, so a failure reproduces on every run.

#![allow(clippy::float_cmp, clippy::unwrap_used)]

use std::collections::BTreeMap;
use std::io::Cursor;

use bevel_io::{ExportMesh, stl, threemf};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};

/// An axis-aligned box with the given size and corner, 8 vertices and 12
/// outward counter-clockwise triangles.
fn box_mesh(size: [f64; 3], origin: [f64; 3]) -> ExportMesh {
    let corner = |x: f64, y: f64, z: f64| {
        [
            origin[0] + x * size[0],
            origin[1] + y * size[1],
            origin[2] + z * size[2],
        ]
    };
    ExportMesh {
        name: "box".to_owned(),
        color: None,
        positions: vec![
            corner(0.0, 0.0, 0.0),
            corner(1.0, 0.0, 0.0),
            corner(1.0, 1.0, 0.0),
            corner(0.0, 1.0, 0.0),
            corner(0.0, 0.0, 1.0),
            corner(1.0, 0.0, 1.0),
            corner(1.0, 1.0, 1.0),
            corner(0.0, 1.0, 1.0),
        ],
        triangles: vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ],
    }
}

/// An octahedron subdivided `levels` times and projected onto a sphere.
fn sphere_mesh(radius: f64, levels: u32) -> ExportMesh {
    let mut positions: Vec<[f64; 3]> = vec![
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0],
    ];
    let mut triangles: Vec<[u32; 3]> = vec![
        [0, 2, 4],
        [2, 1, 4],
        [1, 3, 4],
        [3, 0, 4],
        [2, 0, 5],
        [1, 2, 5],
        [3, 1, 5],
        [0, 3, 5],
    ];
    for _ in 0..levels {
        let mut midpoints: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        let mut next = Vec::with_capacity(triangles.len() * 4);
        for t in &triangles {
            let mut mid = [0u32; 3];
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let key = (a.min(b), a.max(b));
                mid[k] = *midpoints.entry(key).or_insert_with(|| {
                    let (p, q) = (positions[a as usize], positions[b as usize]);
                    positions.push([
                        f64::midpoint(p[0], q[0]),
                        f64::midpoint(p[1], q[1]),
                        f64::midpoint(p[2], q[2]),
                    ]);
                    u32::try_from(positions.len() - 1).unwrap()
                });
            }
            next.push([t[0], mid[0], mid[2]]);
            next.push([t[1], mid[1], mid[0]]);
            next.push([t[2], mid[2], mid[1]]);
            next.push([mid[0], mid[1], mid[2]]);
        }
        triangles = next;
    }
    for p in &mut positions {
        let len = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        *p = p.map(|c| c / len * radius);
    }
    ExportMesh {
        name: "sphere".to_owned(),
        color: Some([10, 200, 90, 255]),
        positions,
        triangles,
    }
}

/// Rotates the vertex numbering so indices do not follow construction order.
fn renumbered(mut mesh: ExportMesh, shift: usize) -> ExportMesh {
    let n = mesh.positions.len();
    mesh.positions.rotate_left(shift % n);
    let shift = u32::try_from(shift % n).unwrap();
    let n = u32::try_from(n).unwrap();
    for t in &mut mesh.triangles {
        for v in t.iter_mut() {
            *v = (*v + n - shift) % n;
        }
    }
    mesh
}

fn closed_mesh() -> impl Strategy<Value = ExportMesh> {
    let boxes = (
        prop::array::uniform3(0.001f64..1000.0),
        prop::array::uniform3(-100.0f64..100.0),
        0usize..8,
    )
        .prop_map(|(size, origin, shift)| renumbered(box_mesh(size, origin), shift));
    let spheres = (0.01f64..500.0, 0u32..3, 0usize..6)
        .prop_map(|(radius, levels, shift)| renumbered(sphere_mesh(radius, levels), shift));
    prop_oneof![boxes, spheres]
}

fn resolved(m: &ExportMesh) -> Vec<[[f64; 3]; 3]> {
    m.triangles
        .iter()
        .map(|t| t.map(|v| m.positions[v as usize]))
        .collect()
}

proptest! {
    #![proptest_config(Config {
        cases: 64,
        rng_seed: RngSeed::Fixed(0x00BE_7E10),
        failure_persistence: None,
        ..Config::default()
    })]

    #[test]
    fn closed_meshes_round_trip(mesh in closed_mesh()) {
        prop_assert!(mesh.validate().is_ok());
        prop_assert!(mesh.is_watertight());

        // 3MF: positions and connectivity come back exactly.
        let mut cursor = Cursor::new(Vec::new());
        threemf::write(std::slice::from_ref(&mesh), &mut cursor).unwrap();
        let back = threemf::read(cursor.get_ref()).unwrap();
        prop_assert_eq!(back.len(), 1);
        prop_assert!(back[0].is_watertight());
        prop_assert_eq!(&back[0].name, &mesh.name);
        prop_assert_eq!(back[0].color, mesh.color);
        prop_assert_eq!(resolved(&back[0]), resolved(&mesh));

        // STL: the same triangles to f32 rounding, in both encodings.
        let expected: Vec<[[f32; 3]; 3]> = resolved(&mesh)
            .iter()
            .map(|t| t.map(|p| p.map(|c| c as f32)))
            .collect();
        let mut binary = Vec::new();
        stl::write_binary(std::slice::from_ref(&mesh), &mut binary).unwrap();
        prop_assert_eq!(stl::read(&binary).unwrap(), expected.clone());
        let mut ascii = Vec::new();
        stl::write_ascii(std::slice::from_ref(&mesh), &mut ascii).unwrap();
        prop_assert_eq!(stl::read(&ascii).unwrap(), expected);
    }
}
