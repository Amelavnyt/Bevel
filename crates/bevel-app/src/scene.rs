// SPDX-License-Identifier: GPL-3.0-or-later

//! Placeholder scene shown until the document model exists: a lit test
//! cube with its edges, standing on the grid at the origin.

use bevel_render::{LineSet, TriangleMesh};

/// Edge length of the test cube, in millimetres.
const SIZE: f32 = 40.0;

pub fn test_scene() -> (Vec<TriangleMesh>, Vec<LineSet>) {
    (vec![test_cube()], vec![test_cube_edges()])
}

/// Cube corner `i`, with bit 0 selecting +X, bit 1 +Y and bit 2 +Z.
fn corner(i: usize) -> [f32; 3] {
    let h = 0.5 * SIZE;
    [
        if i & 1 == 0 { -h } else { h },
        if i & 2 == 0 { -h } else { h },
        if i & 4 == 0 { 0.0 } else { SIZE },
    ]
}

fn test_cube() -> TriangleMesh {
    // Each face: outward normal and its corners counter-clockwise seen from outside.
    let faces: [([f32; 3], [usize; 4]); 6] = [
        ([1.0, 0.0, 0.0], [1, 3, 7, 5]),
        ([-1.0, 0.0, 0.0], [2, 0, 4, 6]),
        ([0.0, 1.0, 0.0], [3, 2, 6, 7]),
        ([0.0, -1.0, 0.0], [0, 1, 5, 4]),
        ([0.0, 0.0, 1.0], [4, 5, 7, 6]),
        ([0.0, 0.0, -1.0], [0, 2, 3, 1]),
    ];
    let mut mesh = TriangleMesh {
        color: [0.48, 0.52, 0.54, 1.0],
        ..TriangleMesh::default()
    };
    for (normal, corners) in faces {
        let base = mesh.positions.len() as u32;
        for c in corners {
            mesh.positions.push(corner(c));
            mesh.normals.push(normal);
            mesh.ids.push(0);
        }
        mesh.indices
            .extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    mesh
}

fn test_cube_edges() -> LineSet {
    let mut lines = LineSet {
        color: [0.02, 0.02, 0.03, 1.0],
        ..LineSet::default()
    };
    // Corners that differ in exactly one bit share an edge.
    for a in 0..8 {
        for bit in [1, 2, 4] {
            if a & bit == 0 {
                lines.positions.push(corner(a));
                lines.positions.push(corner(a | bit));
                lines.ids.push(0);
            }
        }
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
        [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
    }

    fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }

    #[test]
    fn cube_triangles_wind_counter_clockwise_around_their_normals() {
        let mesh = test_cube();
        assert_eq!(mesh.indices.len(), 36);
        for tri in mesh.indices.chunks_exact(3) {
            let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[tri[k] as usize]);
            let n = cross(sub(b, a), sub(c, a));
            let expected = mesh.normals[tri[0] as usize];
            let dot = n[0] * expected[0] + n[1] * expected[1] + n[2] * expected[2];
            assert!(dot > 0.0, "triangle {tri:?} is wound inwards");
        }
    }

    #[test]
    fn cube_has_twelve_edges() {
        assert_eq!(test_cube_edges().positions.len(), 24);
    }

    #[test]
    fn cube_has_one_id_per_vertex_and_per_edge() {
        let mesh = test_cube();
        assert_eq!(mesh.ids.len(), mesh.positions.len());
        let edges = test_cube_edges();
        assert_eq!(edges.ids.len(), edges.positions.len() / 2);
    }
}
