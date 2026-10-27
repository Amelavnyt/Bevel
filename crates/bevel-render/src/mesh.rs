// SPDX-License-Identifier: GPL-3.0-or-later

use bevel_math::{Aabb3, Point3};

/// An indexed triangle mesh with per-vertex normals, ids and a single colour.
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
    /// The id of each vertex, one per position. Id 0 means "not pickable".
    ///
    /// The id is a flat per-vertex attribute (it is not interpolated), so
    /// every triangle must have the same id at all three corners. That holds
    /// when a mesh covers a single face, or when vertices are not shared
    /// across id boundaries, which is how face meshes are built anyway
    /// (they need separate normals at creases).
    pub ids: Vec<u32>,
}

impl TriangleMesh {
    /// Bounding box of the vertex positions.
    pub fn bounds(&self) -> Aabb3 {
        bounds_of(&self.positions)
    }

    /// Why the mesh can't be drawn, if it can't.
    pub(crate) fn check(&self) -> Result<(), &'static str> {
        if self.normals.len() != self.positions.len() {
            return Err("normal count does not match position count");
        }
        if self.ids.len() != self.positions.len() {
            return Err("id count does not match position count");
        }
        let vertex_count = u32::try_from(self.positions.len()).map_err(|_| "too many vertices")?;
        u32::try_from(self.indices.len()).map_err(|_| "too many indices")?;
        if self.indices.iter().any(|&i| i >= vertex_count) {
            return Err("index out of range");
        }
        Ok(())
    }
}

/// A set of line segments with a single colour and width.
#[derive(Clone, Debug)]
pub struct LineSet {
    /// Segment end points in world space, two per segment.
    pub positions: Vec<[f32; 3]>,
    /// The id of each segment, one per segment (`positions.len() / 2`).
    /// Id 0 means "not pickable". The segments of one polyline normally
    /// share the id of the edge they belong to.
    pub ids: Vec<u32>,
    /// Linear RGBA colour.
    pub color: [f32; 4],
    /// Line width in pixels; see [`LineSet::DEFAULT_WIDTH`].
    pub width: f32,
}

impl LineSet {
    /// The width of a default line set, in pixels.
    pub const DEFAULT_WIDTH: f32 = 1.5;

    /// Bounding box of the segment end points.
    pub fn bounds(&self) -> Aabb3 {
        bounds_of(&self.positions)
    }

    /// Number of segments (`positions.len() / 2`, rounded down).
    pub fn segment_count(&self) -> usize {
        self.positions.len() / 2
    }

    /// Why the line set can't be drawn, if it can't.
    pub(crate) fn check(&self) -> Result<(), &'static str> {
        if !self.positions.len().is_multiple_of(2) {
            return Err("odd number of line end points");
        }
        if self.ids.len() != self.segment_count() {
            return Err("id count does not match segment count");
        }
        if !(self.width.is_finite() && self.width > 0.0) {
            return Err("line width is not a positive number");
        }
        Ok(())
    }
}

impl Default for LineSet {
    fn default() -> Self {
        Self {
            positions: Vec::new(),
            ids: Vec::new(),
            color: [0.0; 4],
            width: Self::DEFAULT_WIDTH,
        }
    }
}

fn bounds_of(positions: &[[f32; 3]]) -> Aabb3 {
    Aabb3::from_points(
        positions
            .iter()
            .map(|p| Point3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn triangle() -> TriangleMesh {
        TriangleMesh {
            positions: vec![[0.0; 3]; 3],
            normals: vec![[0.0, 0.0, 1.0]; 3],
            indices: vec![0, 1, 2],
            color: [1.0; 4],
            ids: vec![7; 3],
        }
    }

    #[test]
    fn valid_mesh_passes() {
        assert_eq!(triangle().check(), Ok(()));
    }

    #[test]
    fn mesh_with_wrong_id_count_is_rejected() {
        let mut mesh = triangle();
        mesh.ids.pop();
        assert!(mesh.check().is_err());
        mesh.ids = Vec::new();
        assert!(mesh.check().is_err());
    }

    #[test]
    fn mesh_with_wrong_normal_count_or_bad_index_is_rejected() {
        let mut mesh = triangle();
        mesh.normals.pop();
        assert!(mesh.check().is_err());
        let mut mesh = triangle();
        mesh.indices[2] = 3;
        assert!(mesh.check().is_err());
    }

    fn segment() -> LineSet {
        LineSet {
            positions: vec![[0.0; 3], [1.0, 0.0, 0.0]],
            ids: vec![3],
            ..LineSet::default()
        }
    }

    #[test]
    fn default_line_width_is_one_and_a_half_pixels() {
        assert!((LineSet::default().width - 1.5).abs() < f32::EPSILON);
    }

    #[test]
    fn valid_line_set_passes() {
        assert_eq!(segment().check(), Ok(()));
        assert_eq!(segment().segment_count(), 1);
        assert_eq!(LineSet::default().check(), Ok(()));
    }

    #[test]
    fn line_set_with_wrong_id_count_is_rejected() {
        let mut lines = segment();
        lines.ids.push(4);
        assert!(lines.check().is_err());
        lines.ids = Vec::new();
        assert!(lines.check().is_err());
    }

    #[test]
    fn line_set_with_odd_points_or_bad_width_is_rejected() {
        let mut lines = segment();
        lines.positions.push([0.0; 3]);
        assert!(lines.check().is_err());
        for width in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            let lines = LineSet { width, ..segment() };
            assert!(lines.check().is_err(), "width {width}");
        }
    }
}
