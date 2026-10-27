// SPDX-License-Identifier: GPL-3.0-or-later

//! Boundary-representation topology: vertices, edges, coedges, loops,
//! faces and shells in per-body arenas.
//!
//! # Data structure
//!
//! A [`Body`] owns one [`slotmap::SlotMap`] arena per entity type and
//! refers to entities by typed keys ([`VertexKey`], [`EdgeKey`],
//! [`CoedgeKey`], [`LoopKey`], [`FaceKey`], [`ShellKey`]). A key is only
//! meaningful in the body that issued it.
//!
//! - A [`Vertex`] is a point with a tolerance.
//! - An [`Edge`] is a bounded piece of a 3D curve between two vertices.
//! - A [`Coedge`] (half-edge, fin) is one use of an edge by a loop, in the
//!   edge's direction or against it, with the edge's curve in the face's
//!   parameter space (the pcurve).
//! - A [`Loop`] is a closed cycle of coedges bounding a face.
//! - A [`Face`] is a region of a surface bounded by loops.
//! - A [`Shell`] is a connected set of faces, closed or open.
//! - A [`Body`] is a list of shells: a solid (`shells[0]` the outer
//!   boundary, the rest voids) or a sheet (open shells, for intermediate
//!   results).
//!
//! Back-references are stored and must agree: a coedge names its loop,
//! a loop its face, a face its shell. Bodies are built with
//! [`BodyBuilder`](crate::BodyBuilder), validated by
//! [`validate`](fn@crate::validate) on completion, and immutable afterwards:
//! nothing hands out mutable access, so share them as `Arc<Body>` and make
//! new bodies for changes. The entity structs have public fields, which
//! are read-only through the `&` references the body hands out.
//!
//! # Orientation
//!
//! - An edge runs along its curve's parameter from `range.min` (at its
//!   `start` vertex) to `range.max` (at its `end` vertex).
//! - A coedge with `reversed == false` travels its edge from `start` to
//!   `end`; with `reversed == true` from `end` to `start`.
//! - A face's *oriented normal* is its surface's normal
//!   ([`SurfaceGeom::normal`], the direction of `S_u x S_v`), negated when
//!   `face.reversed` is set. It points out of the material: out of the
//!   solid for the outer shell, into the void for a void shell.
//! - Loops keep the face on their left: walking along each coedge on the
//!   surface with the oriented normal pointing up, the face lies to the
//!   left. In the face's `(u, v)` space (where `S_u x S_v` is the surface
//!   normal) that means: if the face is not reversed its outer loop runs
//!   counter-clockwise and its inner loops clockwise; if it is reversed,
//!   the other way round. `face.loops[0]` is the outer loop.
//! - Orientation is consistent across edges: in a closed shell every
//!   non-degenerate edge has exactly two coedges, one with
//!   `reversed == false` and one with `reversed == true`.
//!
//! # Pcurves
//!
//! A coedge's pcurve is a curve in its face's `(u, v)` space that shares
//! the edge's parameter: for every `t` in `edge.range`,
//! `surface(pcurve(t))` is `edge.curve(t)` to within the edge's tolerance
//! (the "same parameter" property). A pcurve therefore runs along its
//! *edge*, not along its coedge: a reversed coedge travels its pcurve from
//! `range.max` to `range.min`. Both coedges of an edge use the same
//! parameter, which keeps the agreement check and the shared edge
//! discretization of tessellation simple.
//!
//! # Seams
//!
//! A face on a periodic surface (the angle `u` of cylinders, cones,
//! spheres and tori, and `v` of tori) is cut open along seam edges so that
//! each of its loops is a closed curve in `(u, v)` without jumps. A seam
//! edge is used twice by the same face, once in each direction, with its
//! two pcurves one period apart: on a cylinder the seam's pcurves are the
//! lines `u = 2 pi` (travelled upwards) and `u = 0` (downwards). An edge
//! whose two coedges lie in the same face is a seam by definition
//! ([`Body::is_seam`]); the validator checks that its pcurves are exactly
//! one period apart in exactly one periodic direction. The validator also
//! accepts a junction between consecutive pcurves that jumps by exactly
//! one period, for data from elsewhere, but every face the kernel builds
//! has gap-free loops.
//!
//! # Degenerate edges
//!
//! An edge of zero length (at a sphere's pole or a cone's apex) has
//! `degenerate == true`. Its `start` and `end` are the same vertex, its
//! curve is constant (a degree-1 NURBS whose two control points are the
//! vertex; see [`BodyBuilder::add_degenerate_edge`](crate::BodyBuilder::add_degenerate_edge)),
//! and its `range` is the parameter range of its pcurve, which runs along
//! the singular side of the face's parameter domain (`v = +-pi / 2` on a
//! sphere, from `u = 0` to `u = 2 pi`). It has exactly one coedge. A
//! non-degenerate edge must have positive length.
//!
//! # Euler-Poincare
//!
//! For each closed shell, `V - E + F - (L - F) = 2 (1 - G)` with `V` the
//! vertices, `E` the non-degenerate edges, `F` the faces, `L` the loops
//! and `G` the genus. Seams are ordinary edges; degenerate edges are not
//! counted, since they are points rather than arcs of the surface. A box
//! gives `8 - 12 + 6 - 0 = 2`, a cylinder `2 - 3 + 3 - 0 = 2`, a sphere
//! `2 - 1 + 1 - 0 = 2` and a torus `1 - 2 + 1 - 0 = 0` (genus 1). See
//! [`Body::euler_counts`].
//!
//! # Determinism
//!
//! Slotmap iteration visits slots in index order. Bodies are only ever
//! built by inserting (never removing), so every iterator here visits
//! entities in construction order, which is the same for the same
//! sequence of builder calls. No hash map iteration reaches any output.

use std::fmt;

use bevel_geom::{Curve2, Curve3, CurveGeom, Surface, SurfaceGeom};
use bevel_math::{Aabb2, Aabb3, Interval, Point2, Point3};
use slotmap::{Key, SlotMap};

use crate::naming::TopoName;

slotmap::new_key_type! {
    /// Key of a [`Vertex`] in its [`Body`].
    pub struct VertexKey;
    /// Key of an [`Edge`] in its [`Body`].
    pub struct EdgeKey;
    /// Key of a [`Coedge`] in its [`Body`].
    pub struct CoedgeKey;
    /// Key of a [`Loop`] in its [`Body`].
    pub struct LoopKey;
    /// Key of a [`Face`] in its [`Body`].
    pub struct FaceKey;
    /// Key of a [`Shell`] in its [`Body`].
    pub struct ShellKey;
}

/// A key of any entity type, for messages and validation issues.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EntityKey {
    /// A vertex.
    Vertex(VertexKey),
    /// An edge.
    Edge(EdgeKey),
    /// A coedge.
    Coedge(CoedgeKey),
    /// A loop.
    Loop(LoopKey),
    /// A face.
    Face(FaceKey),
    /// A shell.
    Shell(ShellKey),
}

impl fmt::Display for EntityKey {
    /// `vertex 3v1`, `edge 2v1`, ...: the kind and the slot index and
    /// version of the key.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Vertex(k) => write!(f, "vertex {:?}", k.data()),
            Self::Edge(k) => write!(f, "edge {:?}", k.data()),
            Self::Coedge(k) => write!(f, "coedge {:?}", k.data()),
            Self::Loop(k) => write!(f, "loop {:?}", k.data()),
            Self::Face(k) => write!(f, "face {:?}", k.data()),
            Self::Shell(k) => write!(f, "shell {:?}", k.data()),
        }
    }
}

/// A vertex: a point with a tolerance.
#[derive(Clone, Debug, PartialEq)]
pub struct Vertex {
    /// The position in model space.
    pub point: Point3,
    /// The vertex tolerance in millimetres: the edges that end here meet
    /// within this distance of `point`.
    pub tol: f64,
    /// The persistent name.
    pub name: TopoName,
}

/// An edge: the piece of `curve` over `range`, from `start` to `end`.
#[derive(Clone, Debug, PartialEq)]
pub struct Edge {
    /// The 3D curve; constant for a degenerate edge.
    pub curve: Curve3,
    /// The parameter range on `curve`, finite with `min < max`.
    pub range: Interval,
    /// The vertex at `range.min`.
    pub start: VertexKey,
    /// The vertex at `range.max` (the same as `start` for a closed or
    /// degenerate edge).
    pub end: VertexKey,
    /// `true` for a zero-length edge (see the
    /// [module documentation](self#degenerate-edges)).
    pub degenerate: bool,
    /// The edge tolerance in millimetres: how far the curve, its pcurves
    /// mapped through their surfaces and its end vertices may disagree.
    pub tol: f64,
    /// The persistent name.
    pub name: TopoName,
}

impl Edge {
    /// The point of the curve at `t`.
    pub fn point(&self, t: f64) -> Point3 {
        self.curve.point(t)
    }

    /// Returns `true` if the edge starts and ends at the same vertex.
    pub fn is_closed(&self) -> bool {
        self.start == self.end
    }
}

/// One use of an edge by a loop.
#[derive(Clone, Debug, PartialEq)]
pub struct Coedge {
    /// The edge used.
    pub edge: EdgeKey,
    /// `true` if the loop travels the edge from `end` to `start`.
    pub reversed: bool,
    /// The edge in the face's `(u, v)` space, with the edge's parameter
    /// (see the [module documentation](self#pcurves)).
    pub pcurve: Curve2,
    /// The loop this coedge belongs to.
    pub loop_: LoopKey,
}

impl Coedge {
    /// The edge parameter where the coedge starts: `range.max` of the edge
    /// if reversed, `range.min` otherwise.
    pub fn start_param(&self, edge: &Edge) -> f64 {
        if self.reversed {
            edge.range.max
        } else {
            edge.range.min
        }
    }

    /// The edge parameter where the coedge ends.
    pub fn end_param(&self, edge: &Edge) -> f64 {
        if self.reversed {
            edge.range.min
        } else {
            edge.range.max
        }
    }

    /// The vertex where the coedge starts.
    pub fn start_vertex(&self, edge: &Edge) -> VertexKey {
        if self.reversed { edge.end } else { edge.start }
    }

    /// The vertex where the coedge ends.
    pub fn end_vertex(&self, edge: &Edge) -> VertexKey {
        if self.reversed { edge.start } else { edge.end }
    }

    /// The point in `(u, v)` where the coedge starts.
    pub fn start_uv(&self, edge: &Edge) -> Point2 {
        self.pcurve.point(self.start_param(edge))
    }

    /// The point in `(u, v)` where the coedge ends.
    pub fn end_uv(&self, edge: &Edge) -> Point2 {
        self.pcurve.point(self.end_param(edge))
    }
}

/// A closed cycle of coedges bounding a face, in travel order.
#[derive(Clone, Debug, PartialEq)]
pub struct Loop {
    /// The coedges in the order the loop travels them.
    pub coedges: Vec<CoedgeKey>,
    /// The face this loop bounds.
    pub face: FaceKey,
}

/// A face: the region of `surface` bounded by its loops.
#[derive(Clone, Debug, PartialEq)]
pub struct Face {
    /// The underlying (untrimmed) surface.
    pub surface: Surface,
    /// `true` if the oriented normal is the opposite of the surface normal
    /// (see the [module documentation](self#orientation)).
    pub reversed: bool,
    /// The loops: the outer loop first, then the holes.
    pub loops: Vec<LoopKey>,
    /// The shell this face belongs to.
    pub shell: ShellKey,
    /// The persistent name.
    pub name: TopoName,
}

/// A connected set of faces.
#[derive(Clone, Debug, PartialEq)]
pub struct Shell {
    /// The faces.
    pub faces: Vec<FaceKey>,
    /// `true` if the shell is closed: every non-degenerate edge is shared
    /// by exactly two coedges.
    pub closed: bool,
}

/// What a body is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum BodyKind {
    /// A solid: every shell is closed; `shells[0]` is the outer boundary
    /// and the others bound voids.
    Solid,
    /// A sheet: open shells, used for intermediate results.
    Sheet,
}

/// The counts that enter the Euler-Poincare formula for one shell (see
/// the [module documentation](self#euler-poincare)).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EulerCounts {
    /// Distinct vertices used by the shell's edges.
    pub vertices: usize,
    /// Distinct non-degenerate edges used by the shell's faces.
    pub edges: usize,
    /// Faces.
    pub faces: usize,
    /// Loops of those faces.
    pub loops: usize,
}

impl EulerCounts {
    /// `V - E + F - (L - F)`, which is `2 (1 - G)` for a valid closed
    /// shell of genus `G`.
    pub fn characteristic(&self) -> i64 {
        let n = |x: usize| i64::try_from(x).unwrap_or(i64::MAX);
        n(self.vertices) - n(self.edges) + 2 * n(self.faces) - n(self.loops)
    }

    /// The genus `G = 1 - characteristic / 2`, or `None` if the
    /// characteristic is odd or greater than 2, which no closed orientable
    /// shell has.
    pub fn genus(&self) -> Option<u32> {
        let c = self.characteristic();
        if c > 2 || c.rem_euclid(2) != 0 {
            return None;
        }
        u32::try_from((2 - c) / 2).ok()
    }
}

/// A boundary-representation body (see the [module documentation](self)).
///
/// # Serialization
///
/// A body serializes in a flat, index-based form rather than as its
/// arenas, so files do not depend on slot numbers and read naturally:
///
/// ```text
/// { "kind": "Solid",
///   "vertices": [ { "name", "point", "tol" }, ... ],
///   "edges":    [ { "name", "start", "end", "curve", "range", "degenerate", "tol" }, ... ],
///   "faces":    [ { "name", "surface", "reversed",
///                   "loops": [ [ { "edge", "reversed", "pcurve" }, ... ], ... ] }, ... ],
///   "shells":   [ { "faces", "closed" }, ... ] }
/// ```
///
/// Vertices, edges and faces appear in construction order and refer to
/// each other by position in these lists; loops (outer first) and their
/// coedges (in travel order) are nested in their faces; shells appear in
/// body order. Unknown fields are refused.
///
/// Deserialization rebuilds the body with
/// [`BodyBuilder`](crate::BodyBuilder) in that order and finishes it,
/// which runs [`validate`](fn@crate::validate): data that describes an
/// invalid body (or contains invalid geometry, which the geometry types
/// refuse themselves) fails to deserialize, with the issues in the error
/// message. A deserialized body is therefore always valid. Rebuilding
/// assigns keys afresh, so equality ignores keys: two bodies are equal when
/// their flat forms are (the same entities in the same order, with the same
/// geometry, names and references), and a body equals its round trip.
#[derive(Clone, Debug)]
pub struct Body {
    pub(crate) vertices: SlotMap<VertexKey, Vertex>,
    pub(crate) edges: SlotMap<EdgeKey, Edge>,
    pub(crate) coedges: SlotMap<CoedgeKey, Coedge>,
    pub(crate) loops: SlotMap<LoopKey, Loop>,
    pub(crate) faces: SlotMap<FaceKey, Face>,
    pub(crate) shell_arena: SlotMap<ShellKey, Shell>,
    /// The shells in order: for a solid, the outer one first.
    pub(crate) shells: Vec<ShellKey>,
    pub(crate) kind: BodyKind,
}

impl Body {
    /// Whether the body is a solid or a sheet.
    pub fn kind(&self) -> BodyKind {
        self.kind
    }

    /// The vertex with key `k`.
    pub fn vertex(&self, k: VertexKey) -> Option<&Vertex> {
        self.vertices.get(k)
    }

    /// The edge with key `k`.
    pub fn edge(&self, k: EdgeKey) -> Option<&Edge> {
        self.edges.get(k)
    }

    /// The coedge with key `k`.
    pub fn coedge(&self, k: CoedgeKey) -> Option<&Coedge> {
        self.coedges.get(k)
    }

    /// The loop with key `k`.
    pub fn loop_(&self, k: LoopKey) -> Option<&Loop> {
        self.loops.get(k)
    }

    /// The face with key `k`.
    pub fn face(&self, k: FaceKey) -> Option<&Face> {
        self.faces.get(k)
    }

    /// The shell with key `k`.
    pub fn shell(&self, k: ShellKey) -> Option<&Shell> {
        self.shell_arena.get(k)
    }

    /// All vertices, in construction order.
    pub fn vertices(&self) -> impl Iterator<Item = (VertexKey, &Vertex)> {
        self.vertices.iter()
    }

    /// All edges, in construction order.
    pub fn edges(&self) -> impl Iterator<Item = (EdgeKey, &Edge)> {
        self.edges.iter()
    }

    /// All coedges, in construction order.
    pub fn coedges(&self) -> impl Iterator<Item = (CoedgeKey, &Coedge)> {
        self.coedges.iter()
    }

    /// All loops, in construction order.
    pub fn loops(&self) -> impl Iterator<Item = (LoopKey, &Loop)> {
        self.loops.iter()
    }

    /// All faces, in construction order.
    pub fn faces(&self) -> impl Iterator<Item = (FaceKey, &Face)> {
        self.faces.iter()
    }

    /// The shells in body order (for a solid, the outer one first).
    pub fn shells(&self) -> impl Iterator<Item = (ShellKey, &Shell)> {
        self.shells
            .iter()
            .filter_map(|&k| self.shell_arena.get(k).map(|s| (k, s)))
    }

    /// The keys of the shells in body order.
    pub fn shell_keys(&self) -> &[ShellKey] {
        &self.shells
    }

    /// The number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// The number of edges, degenerate ones included.
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// The number of coedges.
    pub fn coedge_count(&self) -> usize {
        self.coedges.len()
    }

    /// The number of loops.
    pub fn loop_count(&self) -> usize {
        self.loops.len()
    }

    /// The number of faces.
    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// The number of shells.
    pub fn shell_count(&self) -> usize {
        self.shells.len()
    }

    /// The coedges of every loop of `face`, outer loop first, each loop in
    /// travel order. Empty for a key that does not exist.
    pub fn face_coedges(&self, face: FaceKey) -> impl Iterator<Item = CoedgeKey> + '_ {
        self.faces
            .get(face)
            .into_iter()
            .flat_map(|f| f.loops.iter())
            .filter_map(|&l| self.loops.get(l))
            .flat_map(|l| l.coedges.iter().copied())
    }

    /// The distinct edges of `face`, in order of first use (a seam once).
    pub fn face_edges(&self, face: FaceKey) -> Vec<EdgeKey> {
        let mut out: Vec<EdgeKey> = Vec::new();
        for c in self.face_coedges(face) {
            if let Some(co) = self.coedges.get(c)
                && !out.contains(&co.edge)
            {
                out.push(co.edge);
            }
        }
        out
    }

    /// The coedges that use `edge`, in construction order. Linear in the
    /// number of coedges.
    pub fn edge_coedges(&self, edge: EdgeKey) -> Vec<CoedgeKey> {
        self.coedges
            .iter()
            .filter(|(_, c)| c.edge == edge)
            .map(|(k, _)| k)
            .collect()
    }

    /// The distinct faces whose loops use `edge` (a seam's face once), in
    /// construction order of the coedges.
    pub fn edge_faces(&self, edge: EdgeKey) -> Vec<FaceKey> {
        let mut out: Vec<FaceKey> = Vec::new();
        for c in self.edge_coedges(edge) {
            if let Some(f) = self.coedge_face(c)
                && !out.contains(&f)
            {
                out.push(f);
            }
        }
        out
    }

    /// The edges that start or end at `vertex`, in construction order.
    /// Linear in the number of edges.
    pub fn vertex_edges(&self, vertex: VertexKey) -> Vec<EdgeKey> {
        self.edges
            .iter()
            .filter(|(_, e)| e.start == vertex || e.end == vertex)
            .map(|(k, _)| k)
            .collect()
    }

    /// The face whose loop contains `coedge`.
    pub fn coedge_face(&self, coedge: CoedgeKey) -> Option<FaceKey> {
        let c = self.coedges.get(coedge)?;
        self.loops.get(c.loop_).map(|l| l.face)
    }

    /// The vertex where `coedge` starts (taking `reversed` into account).
    pub fn coedge_start_vertex(&self, coedge: CoedgeKey) -> Option<VertexKey> {
        let c = self.coedges.get(coedge)?;
        self.edges.get(c.edge).map(|e| c.start_vertex(e))
    }

    /// The vertex where `coedge` ends (taking `reversed` into account).
    pub fn coedge_end_vertex(&self, coedge: CoedgeKey) -> Option<VertexKey> {
        let c = self.coedges.get(coedge)?;
        self.edges.get(c.edge).map(|e| c.end_vertex(e))
    }

    /// Returns `true` if `edge` is a seam: it has two coedges and both lie
    /// in the same face.
    pub fn is_seam(&self, edge: EdgeKey) -> bool {
        match self.edge_coedges(edge).as_slice() {
            [a, b] => {
                let (fa, fb) = (self.coedge_face(*a), self.coedge_face(*b));
                fa.is_some() && fa == fb
            }
            _ => false,
        }
    }

    /// The entity named `name`, if any (names are unique in a valid body).
    pub fn find_name(&self, name: &TopoName) -> Option<EntityKey> {
        if let Some((k, _)) = self.vertices.iter().find(|(_, v)| &v.name == name) {
            return Some(EntityKey::Vertex(k));
        }
        if let Some((k, _)) = self.edges.iter().find(|(_, e)| &e.name == name) {
            return Some(EntityKey::Edge(k));
        }
        self.faces
            .iter()
            .find(|(_, f)| &f.name == name)
            .map(|(k, _)| EntityKey::Face(k))
    }

    /// The Euler-Poincare counts of `shell` (see the
    /// [module documentation](self#euler-poincare)), or `None` for a key
    /// that does not exist.
    pub fn euler_counts(&self, shell: ShellKey) -> Option<EulerCounts> {
        let s = self.shell_arena.get(shell)?;
        let mut vertices: Vec<VertexKey> = Vec::new();
        let mut edges: Vec<EdgeKey> = Vec::new();
        let mut loops = 0;
        for &f in &s.faces {
            let Some(face) = self.faces.get(f) else {
                continue;
            };
            loops += face.loops.len();
            for c in self.face_coedges(f) {
                let Some(edge_key) = self.coedges.get(c).map(|c| c.edge) else {
                    continue;
                };
                let Some(edge) = self.edges.get(edge_key) else {
                    continue;
                };
                for v in [edge.start, edge.end] {
                    if !vertices.contains(&v) {
                        vertices.push(v);
                    }
                }
                if !edge.degenerate && !edges.contains(&edge_key) {
                    edges.push(edge_key);
                }
            }
        }
        Some(EulerCounts {
            vertices: vertices.len(),
            edges: edges.len(),
            faces: s.faces.len(),
            loops,
        })
    }

    /// A box containing the body: the vertices, the edges' curve bounds
    /// over their ranges and, for faces that are not planar, the surface
    /// bounds over the `(u, v)` box of the face's pcurves. Exact for the
    /// primitives' analytic geometry; a planar face lies in the convex hull
    /// of its boundary, so its edges bound it. [`Aabb3::EMPTY`] for an
    /// empty body.
    pub fn bounds(&self) -> Aabb3 {
        let mut b = Aabb3::from_points(self.vertices.values().map(|v| v.point));
        for e in self.edges.values() {
            match e.curve.bounds(e.range) {
                Ok(eb) => b = b.union(eb),
                Err(_) => {
                    b = b
                        .include(e.point(e.range.min))
                        .include(e.point(e.range.max));
                }
            }
        }
        for (fk, f) in &self.faces {
            if matches!(f.surface, Surface::Plane(_)) {
                continue;
            }
            let mut uv = Aabb2::EMPTY;
            for c in self.face_coedges(fk) {
                let Some(co) = self.coedges.get(c) else {
                    continue;
                };
                let Some(e) = self.edges.get(co.edge) else {
                    continue;
                };
                if let Ok(pb) = co.pcurve.bounds(e.range) {
                    uv = uv.union(pb);
                }
            }
            if uv.is_empty() {
                continue;
            }
            let (u, v) = (
                Interval::new(uv.min.x, uv.max.x),
                Interval::new(uv.min.y, uv.max.y),
            );
            if let Ok(sb) = f.surface.bounds(u, v) {
                b = b.union(sb);
            }
        }
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn euler_counts_give_the_genus() {
        let counts = |vertices, edges, faces, loops| EulerCounts {
            vertices,
            edges,
            faces,
            loops,
        };
        // Box, cylinder, sphere (degenerate edges not counted), torus.
        assert_eq!(counts(8, 12, 6, 6).genus(), Some(0));
        assert_eq!(counts(2, 3, 3, 3).genus(), Some(0));
        assert_eq!(counts(2, 1, 1, 1).genus(), Some(0));
        assert_eq!(counts(1, 2, 1, 1).characteristic(), 0);
        assert_eq!(counts(1, 2, 1, 1).genus(), Some(1));
        // A double torus has characteristic -2.
        assert_eq!(counts(1, 4, 1, 1).genus(), Some(2));
        // Odd or too large characteristics have no genus.
        assert_eq!(counts(8, 12, 6, 7).genus(), None);
        assert_eq!(counts(9, 12, 6, 6).genus(), None);
        assert_eq!(counts(4, 0, 1, 1).genus(), None);
    }

    #[test]
    fn keys_display_their_kind() {
        let mut vertices: SlotMap<VertexKey, ()> = SlotMap::with_key();
        let k = vertices.insert(());
        assert_eq!(EntityKey::Vertex(k).to_string(), "vertex 1v1");
        let face = EntityKey::Face(FaceKey::null()).to_string();
        assert!(face.starts_with("face "), "{face}");
    }
}
