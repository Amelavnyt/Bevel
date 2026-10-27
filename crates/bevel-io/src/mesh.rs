// SPDX-License-Identifier: GPL-3.0-or-later

//! A plain triangle mesh for export, independent of the kernel.
//!
//! Positions are `f64` millimetres; triangles index into the position list
//! and wind counter-clockwise when seen from outside the solid, so the
//! right-hand normal points out.

use std::collections::BTreeMap;

use crate::IoError;

/// One exportable body: a named, optionally coloured triangle mesh.
#[derive(Debug, Clone, PartialEq)]
pub struct ExportMesh {
    /// Name of the body, written to the file where the format allows.
    pub name: String,
    /// Display colour as non-premultiplied sRGBA, or `None` for no colour.
    pub color: Option<[u8; 4]>,
    /// Vertex positions in millimetres.
    pub positions: Vec<[f64; 3]>,
    /// Triangles as indices into `positions`, counter-clockwise seen from
    /// outside.
    pub triangles: Vec<[u32; 3]>,
}

/// Unique positions and re-indexed triangles, see `ExportMesh::welded`.
pub(crate) type Welded = (Vec<[f64; 3]>, Vec<[u32; 3]>);

impl ExportMesh {
    /// Checks that the mesh can be written.
    ///
    /// Every index must be in range, every coordinate finite, and no
    /// triangle may use the same index twice.
    ///
    /// # Errors
    ///
    /// [`IoError::InvalidMesh`] describing the first problem found.
    pub fn validate(&self) -> Result<(), IoError> {
        self.validate_indices_and_coords()?;
        for (i, t) in self.triangles.iter().enumerate() {
            if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                return Err(IoError::mesh(
                    &self.name,
                    format!("triangle {i} uses the same vertex more than once"),
                ));
            }
        }
        Ok(())
    }

    /// Like [`validate`](Self::validate) but tolerates degenerate triangles
    /// (used when reading files from other tools).
    pub(crate) fn validate_indices_and_coords(&self) -> Result<(), IoError> {
        for (i, p) in self.positions.iter().enumerate() {
            if !p.iter().all(|c| c.is_finite()) {
                return Err(IoError::mesh(
                    &self.name,
                    format!("vertex {i} has a coordinate that is not a finite number"),
                ));
            }
        }
        let n = self.positions.len();
        for (i, t) in self.triangles.iter().enumerate() {
            if t.iter().any(|&v| v as usize >= n) {
                return Err(IoError::mesh(
                    &self.name,
                    format!("triangle {i} refers to a vertex that does not exist"),
                ));
            }
        }
        Ok(())
    }

    /// Whether the mesh is a closed, consistently oriented surface.
    ///
    /// Vertices at exactly the same position (compared bit for bit, with
    /// `-0.0` equal to `0.0`) are welded first. The mesh is watertight when
    /// every undirected edge is used by exactly two triangles, in opposite
    /// directions. A mesh with no triangles, invalid indices or a
    /// triangle that collapses after welding is not watertight.
    pub fn is_watertight(&self) -> bool {
        let Some((_, tris)) = self.welded() else {
            return false;
        };
        if tris.is_empty() {
            return false;
        }
        let mut directed: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        for t in &tris {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if a == b {
                    return false;
                }
                *directed.entry((a, b)).or_insert(0) += 1;
            }
        }
        directed
            .iter()
            .all(|(&(a, b), &n)| n == 1 && directed.get(&(b, a)) == Some(&1))
    }

    /// Welds vertices at identical positions and drops unused ones.
    ///
    /// Returns the unique positions and the triangles re-indexed into them,
    /// both numbered in order of first use, so the result is deterministic.
    /// `None` if a triangle index is out of range.
    pub(crate) fn welded(&self) -> Option<Welded> {
        let mut ids: BTreeMap<[u64; 3], u32> = BTreeMap::new();
        let mut positions: Vec<[f64; 3]> = Vec::new();
        let mut triangles = Vec::with_capacity(self.triangles.len());
        for t in &self.triangles {
            let mut out = [0u32; 3];
            for (slot, &v) in out.iter_mut().zip(t) {
                let p = *self.positions.get(v as usize)?;
                let key = p.map(|c| (c + 0.0).to_bits());
                *slot = if let Some(&id) = ids.get(&key) {
                    id
                } else {
                    let id = u32::try_from(positions.len()).ok()?;
                    ids.insert(key, id);
                    positions.push(p);
                    id
                };
            }
            triangles.push(out);
        }
        Some((positions, triangles))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::unit_cube;

    #[test]
    fn cube_is_valid_and_watertight() {
        let cube = unit_cube();
        assert!(cube.validate().is_ok());
        assert!(cube.is_watertight());
        assert_eq!(cube.triangles.len(), 12);
    }

    #[test]
    fn missing_triangle_breaks_watertightness() {
        let mut cube = unit_cube();
        cube.triangles.pop();
        assert!(!cube.is_watertight());
    }

    #[test]
    fn flipped_triangle_breaks_watertightness() {
        let mut cube = unit_cube();
        cube.triangles[3].swap(0, 1);
        assert!(!cube.is_watertight());
    }

    #[test]
    fn duplicate_vertices_are_welded() {
        let mut cube = unit_cube();
        // Give every triangle its own copy of its vertices.
        let mut positions = Vec::new();
        for t in &mut cube.triangles {
            for v in t.iter_mut() {
                positions.push(cube.positions[*v as usize]);
                *v = u32::try_from(positions.len() - 1).unwrap();
            }
        }
        cube.positions = positions;
        assert!(cube.is_watertight());
    }

    #[test]
    fn validate_rejects_bad_index() {
        let mut cube = unit_cube();
        cube.triangles[0][2] = 99;
        assert!(matches!(cube.validate(), Err(IoError::InvalidMesh { .. })));
        assert!(!cube.is_watertight());
    }

    #[test]
    fn validate_rejects_nan_and_infinity() {
        let mut cube = unit_cube();
        cube.positions[2][1] = f64::NAN;
        assert!(cube.validate().is_err());
        cube.positions[2][1] = f64::INFINITY;
        assert!(cube.validate().is_err());
    }

    #[test]
    fn validate_rejects_degenerate_triangle() {
        let mut cube = unit_cube();
        cube.triangles[5] = [1, 1, 2];
        assert!(cube.validate().is_err());
    }

    #[test]
    fn empty_mesh_is_not_watertight() {
        let mut cube = unit_cube();
        cube.triangles.clear();
        assert!(!cube.is_watertight());
    }
}
