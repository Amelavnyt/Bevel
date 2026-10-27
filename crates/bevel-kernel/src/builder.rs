// SPDX-License-Identifier: GPL-3.0-or-later

//! Building bodies step by step.
//!
//! [`BodyBuilder`] is used by the primitives now and by every operation
//! later. It is a direct builder rather than a set of Euler operators, but
//! keeps their spirit: every step checks the keys it is given and keeps
//! the stored references consistent, so the structure is sound after each
//! call.
//!
//! - [`BodyBuilder::add_vertex`], [`BodyBuilder::add_edge`] and
//!   [`BodyBuilder::add_degenerate_edge`] add the wireframe; an edge must
//!   name vertices that exist.
//! - [`BodyBuilder::add_face`] adds a face on a surface with no loops yet.
//! - [`BodyBuilder::add_loop`] adds a loop to a face, creating one coedge
//!   per `(edge, reversed, pcurve)` triple; the coedges must join up
//!   vertex to vertex (taking `reversed` into account). The first loop of a
//!   face is its outer loop.
//! - [`BodyBuilder::add_shell`] groups faces into a shell; each face joins
//!   exactly one shell.
//! - [`BodyBuilder::finish`] runs [`validate`](fn@crate::validate) and returns
//!   the body only if it has no issues.
//!
//! Geometry (vertex positions, pcurve agreement, orientation) is checked
//! once, by validation in [`BodyBuilder::finish`], since it needs the whole
//! body. Entities are inserted in call order and never removed, so the
//! keys and iteration order of the finished body follow the sequence of
//! calls (see the determinism note in [`crate::topology`]).

use bevel_geom::{Curve2, Curve3, KnotVector, NurbsCurve3};
use bevel_math::{Interval, Point3};
use slotmap::Key;

use crate::KernelError;
use crate::naming::TopoName;
use crate::topology::{
    Body, BodyKind, Coedge, CoedgeKey, Edge, EdgeKey, EntityKey, Face, FaceKey, Loop, LoopKey,
    Shell, ShellKey, Vertex, VertexKey,
};
use crate::validate::validate;
use bevel_geom::Surface;

/// Builds a [`Body`] step by step (see the [module documentation](self)).
#[derive(Clone, Debug)]
pub struct BodyBuilder {
    body: Body,
}

impl Default for BodyBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl BodyBuilder {
    /// An empty builder.
    pub fn new() -> Self {
        Self {
            body: Body::empty(BodyKind::Solid),
        }
    }

    /// The body built so far, for queries such as [`Body::bounds`] before
    /// finishing. It is not validated.
    pub fn body(&self) -> &Body {
        &self.body
    }

    /// Adds a vertex at `point` with tolerance `tol`.
    pub fn add_vertex(&mut self, point: Point3, tol: f64, name: TopoName) -> VertexKey {
        self.body.vertices.insert(Vertex { point, tol, name })
    }

    /// Adds the edge along `curve` over `range` from `start` (at
    /// `range.min`) to `end` (at `range.max`).
    ///
    /// # Errors
    ///
    /// [`KernelError::UnknownKey`] if either vertex does not exist.
    pub fn add_edge(
        &mut self,
        curve: Curve3,
        range: Interval,
        start: VertexKey,
        end: VertexKey,
        tol: f64,
        name: TopoName,
    ) -> Result<EdgeKey, KernelError> {
        self.insert_edge(Edge {
            curve,
            range,
            start,
            end,
            degenerate: false,
            tol,
            name,
        })
    }

    /// Adds a degenerate (zero-length) edge at `vertex` whose pcurves will
    /// share the parameter range `range`; its curve is the constant
    /// degree-1 NURBS with both control points at the vertex.
    ///
    /// # Errors
    ///
    /// [`KernelError::UnknownKey`] if the vertex does not exist and
    /// [`KernelError::Geometry`] if `range` is not finite with positive
    /// length.
    pub fn add_degenerate_edge(
        &mut self,
        vertex: VertexKey,
        range: Interval,
        tol: f64,
        name: TopoName,
    ) -> Result<EdgeKey, KernelError> {
        let point = self
            .body
            .vertices
            .get(vertex)
            .ok_or(KernelError::UnknownKey(EntityKey::Vertex(vertex)))?
            .point;
        let knots = KnotVector::clamped_from_interior(1, range.min, range.max, &[])?;
        let curve = NurbsCurve3::from_points(1, knots, &[point, point], None)?;
        self.insert_edge(Edge {
            curve: Curve3::Nurbs(curve),
            range,
            start: vertex,
            end: vertex,
            degenerate: true,
            tol,
            name,
        })
    }

    /// Inserts an edge as given, after checking its vertices exist.
    pub(crate) fn insert_edge(&mut self, edge: Edge) -> Result<EdgeKey, KernelError> {
        for v in [edge.start, edge.end] {
            if !self.body.vertices.contains_key(v) {
                return Err(KernelError::UnknownKey(EntityKey::Vertex(v)));
            }
        }
        Ok(self.body.edges.insert(edge))
    }

    /// Adds a face on `surface` with no loops yet; `reversed` flips its
    /// oriented normal (see [`crate::topology`]).
    pub fn add_face(&mut self, surface: Surface, reversed: bool, name: TopoName) -> FaceKey {
        self.body.faces.insert(Face {
            surface,
            reversed,
            loops: Vec::new(),
            shell: ShellKey::null(),
            name,
        })
    }

    /// Adds a loop to `face` with one coedge per `(edge, reversed,
    /// pcurve)`, in travel order. The face's first loop is its outer loop.
    ///
    /// # Errors
    ///
    /// [`KernelError::UnknownKey`] for a face or edge that does not exist,
    /// [`KernelError::InvalidInput`] for an empty list, and
    /// [`KernelError::OpenLoop`] if consecutive coedges (cyclically) do not
    /// share the vertex at their junction.
    pub fn add_loop(
        &mut self,
        face: FaceKey,
        coedges: &[(EdgeKey, bool, Curve2)],
    ) -> Result<LoopKey, KernelError> {
        if !self.body.faces.contains_key(face) {
            return Err(KernelError::UnknownKey(EntityKey::Face(face)));
        }
        if coedges.is_empty() {
            return Err(KernelError::InvalidInput {
                what: "loop",
                reason: "a loop needs at least one coedge",
            });
        }
        let mut ends = Vec::with_capacity(coedges.len());
        for &(e, reversed, _) in coedges {
            let edge = self
                .body
                .edges
                .get(e)
                .ok_or(KernelError::UnknownKey(EntityKey::Edge(e)))?;
            ends.push(if reversed {
                (edge.end, edge.start)
            } else {
                (edge.start, edge.end)
            });
        }
        for (i, &(_, end)) in ends.iter().enumerate() {
            let next_start = ends.get((i + 1) % ends.len()).map(|&(s, _)| s);
            if next_start != Some(end) {
                return Err(KernelError::OpenLoop { index: i });
            }
        }
        let loop_key = self.body.loops.insert(Loop {
            coedges: Vec::with_capacity(coedges.len()),
            face,
        });
        let keys: Vec<CoedgeKey> = coedges
            .iter()
            .map(|(edge, reversed, pcurve)| {
                self.body.coedges.insert(Coedge {
                    edge: *edge,
                    reversed: *reversed,
                    pcurve: pcurve.clone(),
                    loop_: loop_key,
                })
            })
            .collect();
        if let Some(l) = self.body.loops.get_mut(loop_key) {
            l.coedges = keys;
        }
        if let Some(f) = self.body.faces.get_mut(face) {
            f.loops.push(loop_key);
        }
        Ok(loop_key)
    }

    /// Groups `faces` into a new shell, appended to the body's shell list
    /// (for a solid, the first shell is the outer one).
    ///
    /// # Errors
    ///
    /// [`KernelError::InvalidInput`] for an empty list or a face listed
    /// twice, [`KernelError::UnknownKey`] for a face that does not exist and
    /// [`KernelError::AlreadyOwned`] for one already in a shell.
    pub fn add_shell(&mut self, faces: &[FaceKey], closed: bool) -> Result<ShellKey, KernelError> {
        if faces.is_empty() {
            return Err(KernelError::InvalidInput {
                what: "shell",
                reason: "a shell needs at least one face",
            });
        }
        for (i, &f) in faces.iter().enumerate() {
            let face = self
                .body
                .faces
                .get(f)
                .ok_or(KernelError::UnknownKey(EntityKey::Face(f)))?;
            if !face.shell.is_null() {
                return Err(KernelError::AlreadyOwned {
                    entity: EntityKey::Face(f),
                    owner: EntityKey::Shell(face.shell),
                });
            }
            if faces.get(..i).is_some_and(|before| before.contains(&f)) {
                return Err(KernelError::InvalidInput {
                    what: "shell",
                    reason: "a face is listed twice",
                });
            }
        }
        let shell = self.body.shell_arena.insert(Shell {
            faces: faces.to_vec(),
            closed,
        });
        for &f in faces {
            if let Some(face) = self.body.faces.get_mut(f) {
                face.shell = shell;
            }
        }
        self.body.shells.push(shell);
        Ok(shell)
    }

    /// Validates the body and returns it as a body of `kind`.
    ///
    /// # Errors
    ///
    /// [`KernelError::InvalidBody`] with every issue
    /// [`validate`](fn@crate::validate) found.
    pub fn finish(self, kind: BodyKind) -> Result<Body, KernelError> {
        let body = self.finish_unchecked(kind);
        let issues = validate(&body);
        if issues.is_empty() {
            Ok(body)
        } else {
            Err(KernelError::InvalidBody { issues })
        }
    }

    /// The body as built, without validation (tests break bodies on
    /// purpose this way).
    pub(crate) fn finish_unchecked(mut self, kind: BodyKind) -> Body {
        self.body.kind = kind;
        self.body
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::naming::{OpTag, Role, Source};
    use bevel_geom::{Line2, Line3, Plane};
    use bevel_math::{Point2, Vector2, Vector3};

    fn name(role: Role) -> TopoName {
        TopoName::generated(OpTag(1), Source::Primitive, role)
    }

    #[test]
    fn steps_check_their_keys() {
        let mut b = BodyBuilder::new();
        let v = b.add_vertex(Point3::ZERO, 1e-6, name(Role::Vertex(0)));
        let w = b.add_vertex(Point3::X, 1e-6, name(Role::Vertex(1)));
        let line = Curve3::Line(Line3::new(Point3::ZERO, Vector3::X).unwrap());
        let e = b
            .add_edge(
                line.clone(),
                Interval::new(0.0, 1.0),
                v,
                w,
                1e-6,
                name(Role::Edge(0)),
            )
            .unwrap();
        let missing = VertexKey::null();
        assert!(matches!(
            b.add_edge(
                line,
                Interval::new(0.0, 1.0),
                v,
                missing,
                1e-6,
                name(Role::Edge(1))
            ),
            Err(KernelError::UnknownKey(EntityKey::Vertex(_)))
        ));
        let plane = Surface::Plane(Plane::from_normal(Point3::ZERO, Vector3::Z).unwrap());
        let f = b.add_face(plane, false, name(Role::Top));
        let pc = Curve2::Line(Line2::new(Point2::ZERO, Vector2::X).unwrap());
        // One coedge from v to w does not close.
        assert_eq!(
            b.add_loop(f, &[(e, false, pc.clone())]),
            Err(KernelError::OpenLoop { index: 0 })
        );
        assert!(b.add_loop(f, &[]).is_err());
        // There and back closes (topologically; validation would object).
        let l = b
            .add_loop(f, &[(e, false, pc.clone()), (e, true, pc)])
            .unwrap();
        assert_eq!(b.body().loop_(l).unwrap().coedges.len(), 2);
        assert_eq!(b.body().face(f).unwrap().loops, vec![l]);
        let s = b.add_shell(&[f], false).unwrap();
        assert!(matches!(
            b.add_shell(&[f], false),
            Err(KernelError::AlreadyOwned { .. })
        ));
        assert!(b.add_shell(&[], false).is_err());
        assert_eq!(b.body().face(f).unwrap().shell, s);
        assert!(matches!(
            b.finish(BodyKind::Sheet),
            Err(KernelError::InvalidBody { .. })
        ));
    }

    #[test]
    fn degenerate_edges_are_constant() {
        let mut b = BodyBuilder::new();
        let p = Point3::new(1.0, 2.0, 3.0);
        let v = b.add_vertex(p, 1e-6, name(Role::Vertex(0)));
        let e = b
            .add_degenerate_edge(v, Interval::new(0.0, 6.0), 1e-6, name(Role::Pole(0)))
            .unwrap();
        let edge = b.body().edge(e).unwrap();
        assert!(edge.degenerate && edge.is_closed());
        for t in [0.0, 1.5, 6.0] {
            assert!(edge.point(t).distance(p) < 1e-15);
        }
        assert!(matches!(
            b.add_degenerate_edge(v, Interval::new(1.0, 1.0), 1e-6, name(Role::Pole(1))),
            Err(KernelError::Geometry(_))
        ));
    }
}
