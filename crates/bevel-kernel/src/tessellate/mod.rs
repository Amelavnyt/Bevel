// SPDX-License-Identifier: GPL-3.0-or-later

//! Tessellation: triangle meshes of faces and polylines of edges, to a
//! chordal and an angular tolerance, for display and export.
//!
//! # Method
//!
//! 1. **Edges, once.** Every edge is cut at parameters chosen to meet the
//!    tolerances (see [`TessellationOptions`]): the curve stays within
//!    `chordal` of each chord, its tangent turns by at most `angular` per
//!    segment and no chord is longer than `max_edge`. The ends are the
//!    vertices' points exactly. Both faces of an edge, and both sides of a
//!    seam, use exactly these parameters and these 3D points, so the mesh
//!    is watertight by construction: after welding positions by their
//!    bits, every mesh edge of a closed shell is used by exactly two
//!    triangles, in opposite directions. A degenerate edge (a pole, an
//!    apex) takes as many points as the surface needs along it (the
//!    sphere's equator, for example), all at its vertex.
//! 2. **Faces in `(u, v)`.** The boundary points are mapped into the
//!    face's parameter space through each coedge's pcurve at the shared
//!    edge parameters (pcurves share the edge parameter; see
//!    [`topology`](crate::topology#pcurves)). A constrained Delaunay
//!    triangulation (`spade`) of the loops, in `(u, v)` scaled by the
//!    average `|S_u|` and `|S_v|` over the face so that triangles are not
//!    slivers in 3D, is refined until every triangle meets the
//!    tolerances in 3D. The details are in the module `face`
//!    (boundary, scaling, refinement).
//! 3. **Mesh.** Interior positions are exact surface points `S(u, v)`,
//!    boundary positions the shared edge points. Normals are the exact
//!    unit surface normals, negated on reversed faces so they point out of
//!    the material, and triangles are counter-clockwise seen from outside.
//!    Triangles that collapse in 3D (two corners at the same pole or apex)
//!    are dropped, so every triangle has positive area.
//!
//! # Poles and apexes
//!
//! Where the surface has no normal (a cone's apex), each mesh vertex there
//! takes the limit along its triangles: the surface normal at a point a
//! millionth of the way from the vertex towards the average `(u, v)`
//! centroid of the triangles around it. On a cone that is exactly the
//! normal of the ruling through the vertex, so the tip shades smoothly.
//! If the surface has no normal there either, the vertex takes the
//! normalized, area-weighted average of its triangles' normals. A
//! sphere's poles have a normal (the surface provides it), which is used.
//!
//! # Limits
//!
//! Every edge and face may use at most [`MAX_POINTS`] points; a tolerance
//! so small that more would be needed is an error
//! ([`KernelError::PointLimit`]), never a silently coarser mesh.
//!
//! # Determinism
//!
//! Edges and faces are visited in construction order, the triangulation is
//! built by inserting the same points in the same order, and nothing
//! iterates a hash map, so the same body and options give a bit-identical
//! mesh.

mod edge;
pub(crate) mod face;

use bevel_geom::SurfaceGeom;
use bevel_math::{Aabb3, Point2, Point3, Vector3};

use crate::KernelError;
use crate::naming::OpContext;
use crate::topology::{Body, EdgeKey, EntityKey, FaceKey};

pub(crate) use edge::discretize_edges;
pub(crate) use face::{FaceDomain, triangulate_face};

/// The most points one edge or face may be tessellated with.
pub const MAX_POINTS: usize = 1 << 20;

/// How far a vertex at a singular point is moved towards its triangles to
/// evaluate the limit normal, as a fraction of the way.
const LIMIT_STEP: f64 = 1e-6;

/// Tolerances for [`tessellate`].
///
/// The presets scale with the body: `d` below is the length of the
/// diagonal of the body's bounding box ([`Body::bounds`]), or 1 mm if that
/// is empty or not finite.
///
/// | Preset | `chordal` | `angular` | `max_edge` | Use |
/// |---|---|---|---|---|
/// | [`coarse`](Self::coarse) | `2e-3 d` | 0.5 rad (28.6°) | none | while dragging |
/// | [`fine`](Self::fine) | `5e-4 d` | 0.2 rad (11.5°) | none | display at rest |
/// | [`export`](Self::export) | `1e-4 d` | 0.1 rad (5.7°) | none | STL, 3MF and similar |
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TessellationOptions {
    /// The largest distance between the mesh and the surface, in
    /// millimetres; finite and positive. Edge chords and triangle
    /// centroids stay within it.
    pub chordal: f64,
    /// The largest change of normal (for faces) or tangent (for edges)
    /// across one triangle or segment, in radians; finite, in `(0, pi]`.
    pub angular: f64,
    /// The longest triangle edge or segment, in millimetres; finite and
    /// positive if given.
    pub max_edge: Option<f64>,
}

impl TessellationOptions {
    /// The coarse preset (see the [type documentation](Self)).
    pub fn coarse(bounds: &Aabb3) -> Self {
        Self::relative(bounds, 2e-3, 0.5)
    }

    /// The fine preset (see the [type documentation](Self)).
    pub fn fine(bounds: &Aabb3) -> Self {
        Self::relative(bounds, 5e-4, 0.2)
    }

    /// The export preset (see the [type documentation](Self)).
    pub fn export(bounds: &Aabb3) -> Self {
        Self::relative(bounds, 1e-4, 0.1)
    }

    /// `chordal = fraction * d`, the given angular tolerance, no edge
    /// limit.
    fn relative(bounds: &Aabb3, fraction: f64, angular: f64) -> Self {
        let d = if bounds.is_empty() {
            1.0
        } else {
            bounds.size().length()
        };
        let d = if d.is_finite() && d > 0.0 { d } else { 1.0 };
        Self {
            chordal: fraction * d,
            angular,
            max_edge: None,
        }
    }

    /// Checks the documented ranges.
    ///
    /// # Errors
    ///
    /// [`KernelError::InvalidParameter`] naming the offending field.
    pub fn check(&self) -> Result<(), KernelError> {
        if !(self.chordal.is_finite() && self.chordal > 0.0) {
            return Err(KernelError::InvalidParameter {
                name: "chordal tolerance",
                value: self.chordal,
                reason: "must be finite and positive",
            });
        }
        if !(self.angular.is_finite() && self.angular > 0.0 && self.angular <= std::f64::consts::PI)
        {
            return Err(KernelError::InvalidParameter {
                name: "angular tolerance",
                value: self.angular,
                reason: "must be finite, positive and at most pi",
            });
        }
        if let Some(m) = self.max_edge
            && !(m.is_finite() && m > 0.0)
        {
            return Err(KernelError::InvalidParameter {
                name: "maximum edge length",
                value: m,
                reason: "must be finite and positive",
            });
        }
        Ok(())
    }
}

/// The mesh of a body: one [`FaceMesh`] per face and one
/// [`EdgePolyline`] per non-degenerate edge, in construction order.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyMesh {
    /// The faces' meshes, in the body's face order.
    pub faces: Vec<FaceMesh>,
    /// The edges' polylines, in the body's edge order; degenerate edges
    /// (poles, apexes) are omitted.
    pub edges: Vec<EdgePolyline>,
}

impl BodyMesh {
    /// The number of triangles over all faces.
    pub fn triangle_count(&self) -> usize {
        self.faces.iter().map(|f| f.triangles.len()).sum()
    }
}

/// The triangles of one face.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceMesh {
    /// The face.
    pub face: FaceKey,
    /// The vertex positions: exact surface points inside the face, the
    /// shared edge points on its boundary.
    pub positions: Vec<Point3>,
    /// The unit normal at each position, pointing out of the material
    /// (`face.reversed` applied).
    pub normals: Vec<Vector3>,
    /// Triangles as indices into `positions`, counter-clockwise seen from
    /// outside (where the normals point), each with positive area.
    pub triangles: Vec<[u32; 3]>,
}

/// The polyline of one edge.
#[derive(Clone, Debug, PartialEq)]
pub struct EdgePolyline {
    /// The edge.
    pub edge: EdgeKey,
    /// The points from the start vertex to the end vertex (both exact),
    /// the same points the adjacent faces' meshes use on this edge.
    pub points: Vec<Point3>,
}

/// Tessellates `body` (see the [module documentation](self)).
///
/// # Errors
///
/// [`KernelError::InvalidParameter`] for options out of range (see
/// [`TessellationOptions::check`]), [`KernelError::PointLimit`] when an
/// edge or face would need more than [`MAX_POINTS`] points,
/// [`KernelError::Tessellation`] for a face whose discretized boundary
/// crosses or touches itself in `(u, v)`, and [`KernelError::Cancelled`].
pub fn tessellate(
    body: &Body,
    options: &TessellationOptions,
    cx: &OpContext,
) -> Result<BodyMesh, KernelError> {
    tessellate_capped(body, options, MAX_POINTS, cx)
}

/// [`tessellate`] with a given point limit.
pub(crate) fn tessellate_capped(
    body: &Body,
    options: &TessellationOptions,
    cap: usize,
    cx: &OpContext,
) -> Result<BodyMesh, KernelError> {
    options.check()?;
    cx.check_cancelled()?;
    let cap = cap.min(MAX_POINTS);
    let samples = discretize_edges(body, options, cap, cx)?;
    let mut faces = Vec::with_capacity(body.face_count());
    for (key, face) in body.faces() {
        cx.check_cancelled()?;
        let domain = triangulate_face(body, key, &samples, options, cap, cx)?;
        faces.push(face_mesh(key, face, &domain)?);
    }
    let edges = body
        .edges()
        .filter(|(_, e)| !e.degenerate)
        .filter_map(|(key, _)| {
            samples.get(key).map(|s| EdgePolyline {
                edge: key,
                points: s.points.clone(),
            })
        })
        .collect();
    Ok(BodyMesh { faces, edges })
}

/// The display mesh of a triangulated face domain.
fn face_mesh(
    key: FaceKey,
    face: &crate::topology::Face,
    d: &FaceDomain,
) -> Result<FaceMesh, KernelError> {
    let entity = EntityKey::Face(key);
    let n = d.uv.len();
    let at = |v: &[Point3], i: u32| v.get(i as usize).copied();
    // Keep the triangles with three distinct corners in 3D.
    let mut kept: Vec<[u32; 3]> = Vec::with_capacity(d.triangles.len());
    for &t in &d.triangles {
        let (Some(a), Some(b), Some(c)) = (at(&d.pos, t[0]), at(&d.pos, t[1]), at(&d.pos, t[2]))
        else {
            return Err(KernelError::Tessellation {
                entity,
                reason: "a triangle refers to a point that does not exist",
            });
        };
        if face::same_bits(a, b) || face::same_bits(b, c) || face::same_bits(c, a) {
            continue;
        }
        if (b - a).cross(c - a).length_squared() > 0.0 {
            kept.push(t);
        }
    }
    let sign = if face.reversed { -1.0 } else { 1.0 };

    let normals = vertex_normals(face, d, &kept, sign);

    // Drop the points no triangle uses (pole points between collapsed
    // triangles) and renumber.
    let mut used = vec![false; n];
    for t in &kept {
        for &i in t {
            if let Some(u) = used.get_mut(i as usize) {
                *u = true;
            }
        }
    }
    let mut new_index = vec![u32::MAX; n];
    let mut positions = Vec::new();
    let mut out_normals = Vec::new();
    for i in 0..n {
        if !used.get(i).copied().unwrap_or(false) {
            continue;
        }
        let normal = normals
            .get(i)
            .copied()
            .flatten()
            .ok_or(KernelError::Tessellation {
                entity,
                reason: "no normal could be found at a singular point",
            })?;
        let (Some(slot), Some(&p)) = (new_index.get_mut(i), d.pos.get(i)) else {
            continue;
        };
        *slot = u32::try_from(positions.len()).map_err(|_| KernelError::PointLimit {
            entity,
            limit: MAX_POINTS,
        })?;
        positions.push(p);
        out_normals.push(normal);
    }
    let remap = |i: u32| new_index.get(i as usize).copied().unwrap_or(u32::MAX);
    let triangles = kept
        .iter()
        .map(|t| {
            let [a, b, c] = t.map(remap);
            if face.reversed { [a, c, b] } else { [a, b, c] }
        })
        .collect();
    Ok(FaceMesh {
        face: key,
        positions,
        normals: out_normals,
        triangles,
    })
}

/// The oriented normal at each point of `d`: the surface normal times
/// `sign`, or at a singular point the limit along the kept triangles
/// around it (see the [module documentation](self#poles-and-apexes));
/// `None` only for a point no triangle uses.
fn vertex_normals(
    face: &crate::topology::Face,
    d: &FaceDomain,
    kept: &[[u32; 3]],
    sign: f64,
) -> Vec<Option<Vector3>> {
    let n = d.uv.len();
    let mut normals: Vec<Option<Vector3>> = d.normal.iter().map(|n| n.map(|n| n * sign)).collect();
    if normals.iter().any(Option::is_none) {
        let mut centroid_sum = vec![Point2::ZERO; n];
        let mut count = vec![0u32; n];
        let mut area_normal = vec![Vector3::ZERO; n];
        for t in kept {
            let uv = t.map(|i| d.uv.get(i as usize).copied().unwrap_or(Point2::ZERO));
            let p = t.map(|i| d.pos.get(i as usize).copied().unwrap_or(Point3::ZERO));
            let c = (uv[0] + uv[1] + uv[2]) / 3.0;
            // Counter-clockwise in (u, v) means along S_u x S_v.
            let facet = (p[1] - p[0]).cross(p[2] - p[0]) * sign;
            for &i in t {
                let i = i as usize;
                if let (Some(s), Some(k), Some(a)) = (
                    centroid_sum.get_mut(i),
                    count.get_mut(i),
                    area_normal.get_mut(i),
                ) {
                    *s += c;
                    *k += 1;
                    *a += facet;
                }
            }
        }
        for (i, slot) in normals.iter_mut().enumerate() {
            if slot.is_some() {
                continue;
            }
            let (Some(&uv), Some(&k)) = (d.uv.get(i), count.get(i)) else {
                continue;
            };
            if k == 0 {
                continue;
            }
            let towards = centroid_sum.get(i).copied().unwrap_or(uv) / f64::from(k);
            let q = uv + (towards - uv) * LIMIT_STEP;
            *slot = face
                .surface
                .normal(q.x, q.y)
                .map(|n| n * sign)
                .or_else(|| area_normal.get(i).and_then(|a| a.try_normalize()));
        }
    }

    normals
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};

    use bevel_math::Frame3;

    use super::*;
    use crate::{OpTag, SphereParams, sphere};

    fn ball() -> Arc<Body> {
        let params = SphereParams {
            frame: Frame3::WORLD,
            radius: 25.0,
        };
        sphere(&params, OpTag(1), &mut OpContext::new())
            .unwrap()
            .bodies
            .remove(0)
    }

    #[test]
    fn invalid_options_are_refused() {
        let body = ball();
        let good = TessellationOptions::fine(&body.bounds());
        let cx = OpContext::new();
        let cases = [
            TessellationOptions {
                chordal: 0.0,
                ..good
            },
            TessellationOptions {
                chordal: -1.0,
                ..good
            },
            TessellationOptions {
                chordal: f64::NAN,
                ..good
            },
            TessellationOptions {
                chordal: f64::INFINITY,
                ..good
            },
            TessellationOptions {
                angular: 0.0,
                ..good
            },
            TessellationOptions {
                angular: f64::NAN,
                ..good
            },
            TessellationOptions {
                angular: 4.0,
                ..good
            },
            TessellationOptions {
                max_edge: Some(0.0),
                ..good
            },
            TessellationOptions {
                max_edge: Some(f64::NAN),
                ..good
            },
        ];
        for options in cases {
            let err = tessellate(&body, &options, &cx).unwrap_err();
            assert!(
                matches!(err, KernelError::InvalidParameter { .. }),
                "{options:?}: {err}"
            );
        }
        assert!(tessellate(&body, &good, &cx).is_ok());
    }

    #[test]
    fn the_point_cap_is_an_error() {
        let body = ball();
        let options = TessellationOptions::export(&body.bounds());
        let err = tessellate_capped(&body, &options, 500, &OpContext::new()).unwrap_err();
        assert!(
            matches!(err, KernelError::PointLimit { limit: 500, .. }),
            "{err}"
        );
        // A tolerance far too fine for the default cap fails the same way
        // (on the edges, before any face work).
        let tiny = TessellationOptions {
            chordal: 1e-12,
            ..options
        };
        let err = tessellate(&body, &tiny, &OpContext::new()).unwrap_err();
        assert!(matches!(err, KernelError::PointLimit { .. }), "{err}");
    }

    #[test]
    fn cancellation_stops_tessellation() {
        let body = ball();
        let token = Arc::new(AtomicBool::new(false));
        token.store(true, Ordering::Relaxed);
        let cx = OpContext::with_cancel(token);
        let options = TessellationOptions::fine(&body.bounds());
        assert_eq!(
            tessellate(&body, &options, &cx),
            Err(KernelError::Cancelled)
        );
    }

    #[test]
    fn presets_scale_with_the_body_and_tighten() {
        let b = Aabb3::new(Point3::ZERO, Point3::new(30.0, 40.0, 0.0));
        let (c, f, e) = (
            TessellationOptions::coarse(&b),
            TessellationOptions::fine(&b),
            TessellationOptions::export(&b),
        );
        assert!((c.chordal - 0.1).abs() < 1e-12);
        assert!(c.chordal > f.chordal && f.chordal > e.chordal);
        assert!(c.angular > f.angular && f.angular > e.angular);
        // An empty box falls back to 1 mm.
        let empty = TessellationOptions::fine(&Aabb3::EMPTY);
        assert!((empty.chordal - 5e-4).abs() < 1e-15);
        assert!(empty.check().is_ok());
    }
}
