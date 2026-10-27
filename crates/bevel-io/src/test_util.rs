// SPDX-License-Identifier: GPL-3.0-or-later

//! Shared fixtures for unit tests.

use crate::ExportMesh;

/// A unit cube from (0,0,0) to (1,1,1) with 8 vertices and 12 outward
/// counter-clockwise triangles.
pub(crate) fn unit_cube() -> ExportMesh {
    ExportMesh {
        name: "Cube".to_owned(),
        color: Some([200, 100, 50, 255]),
        positions: vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
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
