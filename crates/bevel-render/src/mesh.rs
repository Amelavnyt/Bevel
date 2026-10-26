// SPDX-License-Identifier: GPL-3.0-or-later

use bevel_math::{Aabb3, Point3};

/// An indexed triangle mesh with per-vertex normals and a single colour.
#[derive(Clone, Debug, Default)]
pub struct TriangleMesh {
    /// Vertex positions in world space.
    pub positions: Vec<[f32; 3]>,
    /// Unit normals, one per position.
    pub normals: Vec<[f32; 3]>,
    /// Triangle corner indices into `positions`, three per triangle,
    /// counter-clockwise when seen from the front.
    pub indices: Vec<u32>,
    /// Linear RGBA colour.
    pub color: [f32; 4],
}

impl TriangleMesh {
    /// Bounding box of the vertex positions.
    pub fn bounds(&self) -> Aabb3 {
        bounds_of(&self.positions)
    }
}

/// A set of line segments with a single colour.
#[derive(Clone, Debug, Default)]
pub struct LineSet {
    /// Segment end points in world space, two per segment.
    pub positions: Vec<[f32; 3]>,
    /// Linear RGBA colour.
    pub color: [f32; 4],
}

impl LineSet {
    /// Bounding box of the segment end points.
    pub fn bounds(&self) -> Aabb3 {
        bounds_of(&self.positions)
    }
}

fn bounds_of(positions: &[[f32; 3]]) -> Aabb3 {
    Aabb3::from_points(
        positions
            .iter()
            .map(|p| Point3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))),
    )
}
