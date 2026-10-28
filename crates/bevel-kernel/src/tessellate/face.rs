// SPDX-License-Identifier: GPL-3.0-or-later

//! The triangulation of one face's trimmed `(u, v)` domain: a constrained
//! Delaunay triangulation of the discretized loops, refined until every
//! triangle meets the tolerances in 3D.
//!
//! # Boundary
//!
//! Each loop becomes a closed polygon in `(u, v)`: every coedge
//! contributes its edge's shared parameters (see [`super::edge`]) in
//! travel order, mapped through its pcurve, except the last one, which is
//! where the next coedge starts. The 3D position of a boundary point is
//! the edge's shared point, not `S(pcurve(t))`, so the faces on both sides
//! of an edge (and both sides of a seam) have bit-identical boundary
//! points. The segments of the polygons are constraints of the
//! triangulation and are never split.
//!
//! # Scaling
//!
//! The triangulation is Delaunay in the scaled coordinates `(s_u (u -
//! u0), s_v (v - v0))`, where `s_u` and `s_v` are the averages of `|S_u|`
//! and `|S_v|` over an 8 x 8 grid of cell centres of the face's `(u, v)`
//! box and `(u0, v0)` its lower corner. Without it a cylinder of radius
//! 100 and height 1 would be triangulated as if `2 pi` in `u` were as long
//! as 1 in `v`, and the triangles would be slivers in 3D. A scale that is
//! zero or not finite (a fully degenerate surface) falls back to the other
//! one, or to 1.
//!
//! # Refinement
//!
//! Passes are repeated until no triangle needs refining. A triangle inside
//! the face needs refining if
//!
//! - its 3D deviation exceeds `chordal`: the distance between the surface
//!   point `S(u, v)` at the triangle's `(u, v)` centroid (and at the
//!   midpoints of its interior edges; boundary edges are governed by the
//!   edge discretization) and the point of the flat triangle with the same
//!   barycentric coordinates. This is an upper bound on the distance from
//!   the flat triangle's point to the surface;
//! - the exact normals at its vertices and at its centroid differ pairwise
//!   by more than `angular` (vertices where the normal is undefined, such
//!   as a cone's apex, are left out); or
//! - one of its 3D edges is longer than `max_edge`.
//!
//! A triangle that collapses to a segment in 3D (two corners at the same
//! pole or apex) is never refined; it is dropped from the display mesh.
//! A triangle needing refinement contributes one new point: the `(u, v)`
//! midpoint of its longest (scaled) edge if that edge is interior, else
//! its centroid (longest-edge bisection keeps triangles well shaped).
//! Points are inserted in triangle order; a point that would land on a
//! vertex, on a constraint or outside the face is skipped (the triangle is
//! looked at again in the next pass). Every inserted point lies inside the
//! face, so the faces around it are inside too; that is how the inside
//! flags are kept up to date without a new flood fill. A pass that inserts
//! nothing ends the refinement. More than the point cap is an error.

use std::collections::VecDeque;

use bevel_geom::{CurveGeom, SurfaceGeom};
use bevel_math::{Aabb2, Aabb3, Point2, Point3, Vector3};
use slotmap::SecondaryMap;
use spade::handles::{FixedDirectedEdgeHandle, FixedFaceHandle, FixedVertexHandle, InnerTag};
use spade::{ConstrainedDelaunayTriangulation, PositionInTriangulation, Triangulation};

use super::TessellationOptions;
use super::edge::{EdgeSamples, angle};
use crate::KernelError;
use crate::naming::OpContext;
use crate::topology::{Body, CoedgeKey, EdgeKey, EntityKey, Face, FaceKey};

/// Grid cells per direction for the metric scale factors.
const SCALE_GRID: u32 = 8;

/// A triangle whose longest scaled edge is shorter than this fraction of
/// the face's scaled size is never refined (rounding would dominate).
const MIN_REFINE_SIZE: f64 = 1e-12;

/// Insertions between cancellation checks.
const CANCEL_EVERY: usize = 1024;

/// Per-face state in the triangulation.
#[derive(Clone, Copy, Debug, Default)]
struct FaceState {
    /// The triangle lies inside the face.
    inside: bool,
    /// The triangle has been checked and passed.
    good: bool,
}

type Cdt = ConstrainedDelaunayTriangulation<spade::Point2<f64>, (), (), FaceState>;

/// One segment of a face's boundary polygon: the coedge's pcurve from
/// edge parameter `t0` to `t1` (in travel order), between the polygon
/// vertices `a` and `b`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Segment {
    /// The coedge whose pcurve the segment follows.
    pub coedge: CoedgeKey,
    /// The edge parameter at `a`.
    pub t0: f64,
    /// The edge parameter at `b`.
    pub t1: f64,
    /// The polygon vertex where the segment starts.
    pub a: u32,
    /// The polygon vertex where it ends.
    pub b: u32,
}

/// The triangulated `(u, v)` domain of a face.
#[derive(Clone, Debug)]
pub(crate) struct FaceDomain {
    /// The points in `(u, v)`.
    pub uv: Vec<Point2>,
    /// Their 3D positions: shared edge points on the boundary, `S(u, v)`
    /// inside.
    pub pos: Vec<Point3>,
    /// The surface normal (not flipped for reversed faces) at each point,
    /// `None` where the surface has none.
    pub normal: Vec<Option<Vector3>>,
    /// The triangles inside the face, counter-clockwise in `(u, v)`.
    pub triangles: Vec<[u32; 3]>,
    /// The boundary polygon's segments, loop by loop.
    pub segments: Vec<Segment>,
}

/// Triangulates the domain of `face_key` (see the
/// [module documentation](self)).
///
/// # Errors
///
/// [`KernelError::PointLimit`] beyond `cap` points,
/// [`KernelError::Tessellation`] for a boundary that touches or crosses
/// itself in `(u, v)` or has non-finite points, [`KernelError::UnknownKey`]
/// for broken references and [`KernelError::Cancelled`].
pub(crate) fn triangulate_face(
    body: &Body,
    face_key: FaceKey,
    edges: &SecondaryMap<EdgeKey, EdgeSamples>,
    options: &TessellationOptions,
    cap: usize,
    cx: &OpContext,
) -> Result<FaceDomain, KernelError> {
    let entity = EntityKey::Face(face_key);
    let face = body.face(face_key).ok_or(KernelError::UnknownKey(entity))?;
    let mut d = FaceDomain {
        uv: Vec::new(),
        pos: Vec::new(),
        normal: Vec::new(),
        triangles: Vec::new(),
        segments: Vec::new(),
    };
    let loops = boundary(body, face, edges, &mut d, entity)?;
    if d.uv.len() > cap {
        return Err(KernelError::PointLimit { entity, limit: cap });
    }
    let bbox = Aabb2::from_points(d.uv.iter().copied());
    if !(bbox.min.is_finite() && bbox.max.is_finite()) {
        return Err(KernelError::Tessellation {
            entity,
            reason: "its boundary has points that are not finite in (u, v)",
        });
    }
    let extent = Aabb3::from_points(d.pos.iter().copied()).size().length();
    let mut t = Tri {
        face,
        options,
        entity,
        scale: metric_scale(face, bbox, extent, options),
        origin: bbox.min,
        cdt: Cdt::new(),
        d,
        size: 0.0,
    };
    let size = t.scaled(bbox.max);
    t.size = size.x.hypot(size.y);
    t.insert_boundary(&loops)?;
    t.classify();
    t.refine(cap, cx)?;
    t.collect()?;
    Ok(t.d)
}

/// Fills the boundary points of `d` (uv, positions and normals) and its
/// segments; returns the polygon vertex indices loop by loop.
fn boundary(
    body: &Body,
    face: &Face,
    edges: &SecondaryMap<EdgeKey, EdgeSamples>,
    d: &mut FaceDomain,
    entity: EntityKey,
) -> Result<Vec<Vec<u32>>, KernelError> {
    let mut loops = Vec::with_capacity(face.loops.len());
    for &loop_key in &face.loops {
        let l = body
            .loop_(loop_key)
            .ok_or(KernelError::UnknownKey(EntityKey::Loop(loop_key)))?;
        let first = d.uv.len();
        let mut pending: Vec<(CoedgeKey, f64, f64)> = Vec::new();
        for &ck in &l.coedges {
            let c = body
                .coedge(ck)
                .ok_or(KernelError::UnknownKey(EntityKey::Coedge(ck)))?;
            let samples = edges
                .get(c.edge)
                .ok_or(KernelError::UnknownKey(EntityKey::Edge(c.edge)))?;
            let n = samples.params.len();
            if n < 2 || samples.points.len() != n {
                return Err(KernelError::Tessellation {
                    entity,
                    reason: "an edge of its boundary has no discretization",
                });
            }
            // Travel order, without the last point (the next coedge's
            // first).
            let order: Vec<usize> = if c.reversed {
                (1..n).rev().collect()
            } else {
                (0..n - 1).collect()
            };
            for &i in &order {
                let (Some(&t), Some(&p)) = (samples.params.get(i), samples.points.get(i)) else {
                    continue;
                };
                let next = if c.reversed { i - 1 } else { i + 1 };
                let t_next = samples.params.get(next).copied().unwrap_or(t);
                let uv = c.pcurve.point(t);
                d.uv.push(uv);
                d.pos.push(p);
                d.normal.push(face.surface.normal(uv.x, uv.y));
                pending.push((ck, t, t_next));
            }
        }
        let count = d.uv.len() - first;
        if count < 3 {
            return Err(KernelError::Tessellation {
                entity,
                reason: "a loop has fewer than three points once discretized",
            });
        }
        let ids: Vec<u32> = (first..d.uv.len())
            .map(|i| to_u32(i, entity))
            .collect::<Result<_, _>>()?;
        for (k, &(coedge, t0, t1)) in pending.iter().enumerate() {
            let (Some(&a), Some(&b)) = (ids.get(k), ids.get((k + 1) % ids.len())) else {
                continue;
            };
            d.segments.push(Segment {
                coedge,
                t0,
                t1,
                a,
                b,
            });
        }
        loops.push(ids);
    }
    Ok(loops)
}

/// `i` as a `u32` mesh index.
fn to_u32(i: usize, entity: EntityKey) -> Result<u32, KernelError> {
    u32::try_from(i).map_err(|_| KernelError::PointLimit {
        entity,
        limit: u32::MAX as usize,
    })
}

/// The scale factors `(s_u, s_v)` (see the [module documentation](self)):
/// the average density each direction needs, `|S_w| max(k_w / angular,
/// sqrt(k_w / (8 chordal)), 1 / size)`, with `k_w = |S_ww . n| / |S_w|^2`
/// the normal curvature along `w` and `size` the 3D size of the face's
/// boundary.
fn metric_scale(face: &Face, bbox: Aabb2, size: f64, options: &TessellationOptions) -> Point2 {
    let (mut su, mut sv) = (0.0, 0.0);
    let n = f64::from(SCALE_GRID);
    let size = if size.is_finite() && size > 0.0 {
        size
    } else {
        1.0
    };
    let density = |speed: f64, second: Vector3, normal: Option<Vector3>| -> f64 {
        let flat = speed / size;
        let Some(normal) = normal else {
            return flat;
        };
        if !greater(speed, 0.0) {
            return flat;
        }
        let bend = second.dot(normal).abs() / speed; // k_w |S_w|
        let curvature = bend / speed;
        let need = (bend / options.angular)
            .max(speed * (curvature / (8.0 * options.chordal)).sqrt())
            .max(flat);
        if need.is_finite() { need } else { flat }
    };
    for i in 0..SCALE_GRID {
        for j in 0..SCALE_GRID {
            let u = bbox.min.x + (bbox.max.x - bbox.min.x) * ((f64::from(i) + 0.5) / n);
            let v = bbox.min.y + (bbox.max.y - bbox.min.y) * ((f64::from(j) + 0.5) / n);
            let d = face.surface.derivatives(u, v, 2);
            let at = |k: usize, l: usize| {
                d.get(k)
                    .and_then(|row| row.get(l))
                    .copied()
                    .unwrap_or(Vector3::ZERO)
            };
            let normal = face.surface.normal(u, v);
            su += density(at(1, 0).length(), at(2, 0), normal);
            sv += density(at(0, 1).length(), at(0, 2), normal);
        }
    }
    let (su, sv) = (su / (n * n), sv / (n * n));
    let ok = |s: f64| s.is_finite() && s > f64::MIN_POSITIVE;
    match (ok(su), ok(sv)) {
        (true, true) => Point2::new(su, sv),
        (true, false) => Point2::new(su, su),
        (false, true) => Point2::new(sv, sv),
        (false, false) => Point2::ONE,
    }
}

/// The triangulation being built.
struct Tri<'a> {
    face: &'a Face,
    options: &'a TessellationOptions,
    entity: EntityKey,
    scale: Point2,
    origin: Point2,
    cdt: Cdt,
    d: FaceDomain,
    /// The diagonal of the face's box in scaled coordinates.
    size: f64,
}

impl Tri<'_> {
    /// `uv` in the scaled coordinates of the triangulation.
    fn scaled(&self, uv: Point2) -> Point2 {
        (uv - self.origin) * self.scale
    }

    fn spade_point(&self, uv: Point2) -> spade::Point2<f64> {
        let s = self.scaled(uv);
        spade::Point2::new(s.x, s.y)
    }

    fn error(&self, reason: &'static str) -> KernelError {
        KernelError::Tessellation {
            entity: self.entity,
            reason,
        }
    }

    /// Inserts a point that is new to the triangulation and checks that it
    /// got the next vertex index.
    fn insert_vertex(&mut self, uv: Point2) -> Result<FixedVertexHandle, KernelError> {
        let p = self.spade_point(uv);
        if !(p.x.is_finite() && p.y.is_finite()) {
            return Err(self.error("a point is not finite in (u, v)"));
        }
        let before = self.cdt.num_vertices();
        let handle = self
            .cdt
            .insert(p)
            .map_err(|_| self.error("a point is out of range in (u, v)"))?;
        if self.cdt.num_vertices() != before + 1 || handle.index() != before {
            return Err(self.error("its boundary touches itself in (u, v)"));
        }
        Ok(handle)
    }

    /// Inserts the boundary points and the constraint segments.
    fn insert_boundary(&mut self, loops: &[Vec<u32>]) -> Result<(), KernelError> {
        for i in 0..self.d.uv.len() {
            let uv = self.d.uv.get(i).copied().unwrap_or(Point2::ZERO);
            self.insert_vertex(uv)?;
        }
        for ids in loops {
            for (k, &a) in ids.iter().enumerate() {
                let Some(&b) = ids.get((k + 1) % ids.len()) else {
                    continue;
                };
                let (ha, hb) = (
                    FixedVertexHandle::from_index(a as usize),
                    FixedVertexHandle::from_index(b as usize),
                );
                let added = self.cdt.try_add_constraint(ha, hb);
                if added.len() != 1 {
                    return Err(self.error(
                        "its boundary crosses or touches itself in (u, v) once discretized",
                    ));
                }
            }
        }
        Ok(())
    }

    /// Marks the triangles inside the face: a 0-1 breadth-first search
    /// from the outer face counts the constraints crossed, and a triangle
    /// is inside if that count is odd.
    fn classify(&mut self) {
        let n = self.cdt.num_all_faces();
        let mut handles: Vec<Option<FixedFaceHandle<InnerTag>>> = vec![None; n];
        for face in self.cdt.inner_faces() {
            if let Some(slot) = handles.get_mut(face.fix().index()) {
                *slot = Some(face.fix());
            }
        }
        let mut depth = vec![usize::MAX; n];
        let mut queue: VecDeque<(usize, usize)> = VecDeque::new();
        for face in self.cdt.inner_faces() {
            for e in face.adjacent_edges() {
                if e.rev().face().is_outer() {
                    let crossing = usize::from(e.is_constraint_edge());
                    let i = face.fix().index();
                    if let Some(slot) = depth.get_mut(i)
                        && crossing < *slot
                    {
                        *slot = crossing;
                        if crossing == 0 {
                            queue.push_front((i, 0));
                        } else {
                            queue.push_back((i, 1));
                        }
                    }
                }
            }
        }
        while let Some((i, d)) = queue.pop_front() {
            if depth.get(i).copied() != Some(d) {
                continue;
            }
            let Some(handle) = handles.get(i).copied().flatten() else {
                continue;
            };
            let face = self.cdt.face(handle);
            for e in face.adjacent_edges() {
                let Some(other) = e.rev().face().as_inner() else {
                    continue;
                };
                let j = other.fix().index();
                let nd = d + usize::from(e.is_constraint_edge());
                if let Some(slot) = depth.get_mut(j)
                    && nd < *slot
                {
                    *slot = nd;
                    if nd == d {
                        queue.push_front((j, nd));
                    } else {
                        queue.push_back((j, nd));
                    }
                }
            }
        }
        for h in handles.into_iter().flatten() {
            let inside = depth.get(h.index()).is_some_and(|&d| d % 2 == 1);
            *self.cdt.face_data_mut(h) = FaceState {
                inside,
                good: false,
            };
        }
    }

    /// The refinement passes (see the [module documentation](self)).
    fn refine(&mut self, cap: usize, cx: &OpContext) -> Result<(), KernelError> {
        loop {
            cx.check_cancelled()?;
            let mut candidates = Vec::new();
            let faces: Vec<FixedFaceHandle<InnerTag>> = self
                .cdt
                .inner_faces()
                .filter(|f| {
                    let s = f.data();
                    s.inside && !s.good
                })
                .map(|f| f.fix())
                .collect();
            for h in faces {
                match self.check(h) {
                    Some(uv) => candidates.push(uv),
                    None => self.cdt.face_data_mut(h).good = true,
                }
            }
            if candidates.is_empty() {
                return Ok(());
            }
            let mut inserted = 0;
            for uv in candidates {
                if self.try_insert(uv, cap)? {
                    inserted += 1;
                    if inserted % CANCEL_EVERY == 0 {
                        cx.check_cancelled()?;
                    }
                }
            }
            if inserted == 0 {
                return Ok(());
            }
        }
    }

    /// The vertex ids, the scaled positions and the constraint flags of
    /// the edges `(v0 v1, v1 v2, v2 v0)` of a triangle.
    fn triangle(&self, h: FixedFaceHandle<InnerTag>) -> ([usize; 3], [bool; 3]) {
        let f = self.cdt.face(h);
        let e = f.adjacent_edges();
        let ids = [
            e[0].from().fix().index(),
            e[1].from().fix().index(),
            e[2].from().fix().index(),
        ];
        let constraint = [
            e[0].is_constraint_edge(),
            e[1].is_constraint_edge(),
            e[2].is_constraint_edge(),
        ];
        (ids, constraint)
    }

    /// The point to insert for triangle `h`, or `None` if it passes.
    fn check(&self, h: FixedFaceHandle<InnerTag>) -> Option<Point2> {
        let (ids, constraint) = self.triangle(h);
        let uv = ids.map(|i| self.d.uv.get(i).copied().unwrap_or(Point2::ZERO));
        let pos = ids.map(|i| self.d.pos.get(i).copied().unwrap_or(Point3::ZERO));
        let normals = ids.map(|i| self.d.normal.get(i).copied().flatten());
        if same_bits(pos[0], pos[1]) || same_bits(pos[1], pos[2]) || same_bits(pos[2], pos[0]) {
            return None;
        }
        // Edge k runs from corner k to corner k + 1.
        let lengths = [0, 1, 2].map(|k| {
            let (a, b) = (uv[k], uv[(k + 1) % 3]);
            self.scaled(a).distance(self.scaled(b))
        });
        let (longest, longest_len) =
            lengths
                .iter()
                .copied()
                .enumerate()
                .fold(
                    (0, 0.0),
                    |(bi, bl), (i, l)| if l > bl { (i, l) } else { (bi, bl) },
                );
        if !greater(longest_len, MIN_REFINE_SIZE * self.size) {
            return None;
        }
        if !self.needs_refinement(uv, pos, normals, constraint) {
            return None;
        }
        if constraint[longest] {
            Some((uv[0] + uv[1] + uv[2]) / 3.0)
        } else {
            Some((uv[longest] + uv[(longest + 1) % 3]) * 0.5)
        }
    }

    /// The tests of the [module documentation](self).
    fn needs_refinement(
        &self,
        uv: [Point2; 3],
        pos: [Point3; 3],
        normals: [Option<Vector3>; 3],
        constraint: [bool; 3],
    ) -> bool {
        let o = self.options;
        let s = &self.face.surface;
        // Orientation: counter-clockwise in (u, v) must face along
        // S_u x S_v in 3D (near a pole or an apex a triangle can fold).
        // Not for a triangle thinner than `chordal` against a boundary
        // segment: the boundary's chord in (u, v) maps up to `chordal` away
        // from its 3D chord, so such a triangle can look folded however
        // finely it is refined, and it lies within tolerance anyway.
        let thin = (0..3).any(|k| {
            let (a, b) = (pos[k], pos[(k + 1) % 3]);
            let d = b - a;
            constraint[k]
                && greater(d.length_squared(), 0.0)
                && d.cross(pos[(k + 2) % 3] - a).length() <= o.chordal * d.length()
        });
        let centroid_uv = (uv[0] + uv[1] + uv[2]) / 3.0;
        let facet = (pos[1] - pos[0]).cross(pos[2] - pos[0]);
        if !thin
            && let Some(n) = s.normal(centroid_uv.x, centroid_uv.y)
            && !greater(facet.dot(n), 0.0)
        {
            return true;
        }
        // Chordal: the centroid and the interior edge midpoints. The
        // orientation is checked at the centroid's foot point too.
        let centroid = (pos[0] + pos[1] + pos[2]) / 3.0;
        let (distance, foot) = deviation(s, centroid_uv, centroid);
        if distance > o.chordal {
            return true;
        }
        if !thin
            && let Some(n) = s.normal(foot.x, foot.y)
            && !greater(facet.dot(n), 0.0)
        {
            return true;
        }
        for k in 0..3 {
            if constraint[k] {
                continue;
            }
            let j = (k + 1) % 3;
            let m_uv = (uv[k] + uv[j]) * 0.5;
            let m = (pos[k] + pos[j]) * 0.5;
            if deviation(s, m_uv, m).0 > o.chordal {
                return true;
            }
        }
        // Angular: the defined normals at the corners and the centroid.
        let mut ns: Vec<Vector3> = normals.iter().flatten().copied().collect();
        if let Some(n) = s.normal(centroid_uv.x, centroid_uv.y) {
            ns.push(n);
        }
        for (i, a) in ns.iter().enumerate() {
            for b in ns.iter().skip(i + 1) {
                if angle(*a, *b) > o.angular {
                    return true;
                }
            }
        }
        // Edge length.
        if let Some(max_edge) = o.max_edge {
            for k in 0..3 {
                if pos[k].distance(pos[(k + 1) % 3]) > max_edge {
                    return true;
                }
            }
        }
        false
    }

    /// Inserts `uv` if it lands strictly inside the face away from the
    /// constraints and existing vertices; returns whether it did.
    fn try_insert(&mut self, uv: Point2, cap: usize) -> Result<bool, KernelError> {
        let p = self.spade_point(uv);
        if !(p.x.is_finite() && p.y.is_finite()) {
            return Ok(false);
        }
        let ok = match self.cdt.locate(p) {
            PositionInTriangulation::OnFace(f) => self.cdt.face(f).data().inside,
            PositionInTriangulation::OnEdge(e) => self.interior_edge(e),
            _ => false,
        };
        if !ok {
            return Ok(false);
        }
        if self.d.uv.len() >= cap {
            return Err(KernelError::PointLimit {
                entity: self.entity,
                limit: cap,
            });
        }
        let handle = self.insert_vertex(uv)?;
        let s = &self.face.surface;
        self.d.uv.push(uv);
        self.d.pos.push(s.point(uv.x, uv.y));
        self.d.normal.push(s.normal(uv.x, uv.y));
        // Every face the insertion changed is around the new vertex, and
        // inside the face since the vertex is.
        let around: Vec<FixedFaceHandle<InnerTag>> = self
            .cdt
            .vertex(handle)
            .out_edges()
            .filter_map(|e| e.face().as_inner().map(|f| f.fix()))
            .collect();
        for f in around {
            *self.cdt.face_data_mut(f) = FaceState {
                inside: true,
                good: false,
            };
        }
        Ok(true)
    }

    /// Returns `true` if `e` is not a constraint and both its sides are
    /// inside the face.
    fn interior_edge(&self, e: FixedDirectedEdgeHandle) -> bool {
        let e = self.cdt.directed_edge(e);
        if e.is_constraint_edge() {
            return false;
        }
        let inside = |f: Option<spade::handles::FaceHandle<'_, InnerTag, _, _, _, FaceState>>| {
            f.is_some_and(|f| f.data().inside)
        };
        inside(e.face().as_inner()) && inside(e.rev().face().as_inner())
    }

    /// Copies the inside triangles to the domain.
    fn collect(&mut self) -> Result<(), KernelError> {
        let mut triangles = Vec::new();
        for f in self.cdt.inner_faces() {
            if !f.data().inside {
                continue;
            }
            let v = f.vertices();
            triangles.push([
                to_u32(v[0].fix().index(), self.entity)?,
                to_u32(v[1].fix().index(), self.entity)?,
                to_u32(v[2].fix().index(), self.entity)?,
            ]);
        }
        self.d.triangles = triangles;
        Ok(())
    }
}

/// `a > b`, false if either is NaN.
fn greater(a: f64, b: f64) -> bool {
    a > b
}

/// Gauss-Newton steps of [`deviation`].
const FOOT_STEPS: usize = 2;

/// An upper bound on the distance from `p` to the surface `s` and the
/// `(u, v)` of the surface point that gives it, starting
/// from the guess `uv`: the distance to `S(uv)` after up to
/// [`FOOT_STEPS`] Gauss-Newton steps towards the foot point (each kept
/// only if it brings the surface point closer). Any surface point bounds
/// the distance from above; the steps remove the tangential part of the
/// offset that comes from the parametrization rather than from the
/// geometry (a fan of triangles at a cone's apex, whose `(u, v)` corners
/// at the apex are arbitrary, would otherwise look far off).
fn deviation(s: &bevel_geom::Surface, uv: Point2, p: Point3) -> (f64, Point2) {
    let mut uv = uv;
    let mut best = s.point(uv.x, uv.y).distance(p);
    for _ in 0..FOOT_STEPS {
        let q = s.point(uv.x, uv.y);
        let r = p - q;
        let (su, sv) = s.partials(uv.x, uv.y);
        // Normal equations of min |S_u du + S_v dv - r|.
        let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
        let det = a * c - b * b;
        if !greater(det, f64::EPSILON * a * c) {
            break;
        }
        let (ru, rv) = (su.dot(r), sv.dot(r));
        let next = uv + Point2::new(c * ru - b * rv, a * rv - b * ru) / det;
        let d = s.point(next.x, next.y).distance(p);
        if !greater(best, d) {
            break;
        }
        best = d;
        uv = next;
    }
    (best, uv)
}

/// Returns `true` if two points have bit-identical coordinates (the same
/// mesh vertex after welding).
pub(crate) fn same_bits(a: Point3, b: Point3) -> bool {
    a.x.to_bits() == b.x.to_bits()
        && a.y.to_bits() == b.y.to_bits()
        && a.z.to_bits() == b.z.to_bits()
}
