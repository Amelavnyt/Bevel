// SPDX-License-Identifier: GPL-3.0-or-later

//! Body validation.
//!
//! [`validate`] checks a [`Body`] against the conventions of
//! [`crate::topology`] and returns every [`Issue`] it finds; an empty list
//! means the body is valid. It never panics, whatever the body contains:
//! missing keys are reported and the checks that need them skipped.
//!
//! The checks, in order:
//!
//! 1. **References.** Every key an entity holds exists; back-references
//!    agree (a coedge's loop lists it, a loop's face lists it, a face's
//!    shell lists it, and the other way round); nothing is listed twice;
//!    nothing is left unused; a body has at least one shell.
//! 2. **Values.** Tolerances are finite and at least
//!    [`tol::LINEAR`]; vertices lie in the modelling space; edge ranges are
//!    finite with `min < max`, no longer than one period of a periodic
//!    curve and inside a bounded curve's domain.
//! 3. **Edge geometry.** The curve's ends meet the vertices; the vertices
//!    lie on the curve; a degenerate edge starts and ends at one vertex and
//!    its curve stays there, and a non-degenerate edge has positive length.
//! 4. **Pcurves.** For each coedge, the pcurve mapped through the face's
//!    surface agrees with the edge curve at the same parameter
//!    ([`SAMPLES`]` + 1` parameters across the range).
//! 5. **Loops.** Consecutive coedges share the vertex at their junction
//!    (taking `reversed` into account), and their pcurves meet in
//!    `(u, v)`, allowing a jump of exactly one period across a seam. The
//!    tolerance in each parameter direction is the 3D tolerance divided by
//!    the surface's parametric speed there, floored at a speed of 1 mm per
//!    unit (so a pole, where `S_u` vanishes, is checked to the 3D
//!    tolerance in `u`). A loop without jumps must turn the right way:
//!    the signed area of its `(u, v)` polygon is positive for an outer loop
//!    of a face that is not reversed and for an inner loop of a reversed
//!    face, negative otherwise.
//! 6. **Edge uses.** A degenerate edge has one coedge; in a closed shell a
//!    non-degenerate edge has exactly two, in an open shell one or two; two
//!    coedges of an edge have opposite `reversed` flags (consistent
//!    orientation); if both lie in the same face the edge is a seam and its
//!    pcurves must be one period apart in exactly one periodic direction;
//!    an edge belongs to one shell.
//! 7. **Shells.** A solid's shells are closed; each shell is connected
//!    through shared edges; each closed shell satisfies Euler-Poincare,
//!    `V - E + F - (L - F) = 2 (1 - G)` for some genus `G >= 0`, counted as
//!    in [`Body::euler_counts`].
//! 8. **Names.** Every vertex, edge and face name is unique in the body.
//!
//! Not checked: self-intersections, and whether void shells lie inside the
//! outer shell.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use bevel_geom::{CurveGeom, Domain, SurfaceGeom};
use bevel_math::{Interval, Point2, tol};
use slotmap::Key;
use thiserror::Error;

use crate::naming::TopoName;
use crate::topology::{
    Body, BodyKind, CoedgeKey, EdgeKey, EntityKey, EulerCounts, FaceKey, LoopKey, ShellKey,
    VertexKey,
};

/// The number of intervals each edge range is sampled with when comparing
/// geometry.
pub const SAMPLES: usize = 16;

/// One problem [`validate`] found. Each variant carries the keys of the
/// entities involved; its `Display` is a plain-English description.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum Issue {
    /// `from` refers to `to`, which does not exist.
    #[error("{from} refers to {to}, which does not exist")]
    MissingEntity {
        /// The referring entity.
        from: EntityKey,
        /// The missing one.
        to: EntityKey,
    },

    /// The body's shell list names a shell that does not exist.
    #[error("the body lists {}, which does not exist", EntityKey::Shell(*.shell))]
    MissingShell {
        /// The missing shell.
        shell: ShellKey,
    },

    /// `entity` and `owner` disagree about their relation: one refers to
    /// the other, but not the other way round.
    #[error("{entity} and {owner} disagree about whether one belongs to the other")]
    BackReference {
        /// The owned entity (coedge, loop, face).
        entity: EntityKey,
        /// Its owner (loop, face, shell).
        owner: EntityKey,
    },

    /// `entity` is listed more than once by its owners.
    #[error("{entity} is listed more than once")]
    ListedTwice {
        /// The entity.
        entity: EntityKey,
    },

    /// Nothing uses `entity`.
    #[error("{entity} is not used by anything")]
    Orphan {
        /// The unused entity.
        entity: EntityKey,
    },

    /// The body has no shells.
    #[error("the body has no shells")]
    NoShells,

    /// A loop has no coedges.
    #[error("{} has no coedges", EntityKey::Loop(*.loop_))]
    EmptyLoop {
        /// The loop.
        loop_: LoopKey,
    },

    /// A face has no loops.
    #[error("{} has no loops", EntityKey::Face(*.face))]
    FaceWithoutLoops {
        /// The face.
        face: FaceKey,
    },

    /// A shell has no faces.
    #[error("{} has no faces", EntityKey::Shell(*.shell))]
    EmptyShell {
        /// The shell.
        shell: ShellKey,
    },

    /// A tolerance is not finite or below [`tol::LINEAR`].
    #[error("{entity} has tolerance {value}, which is not finite or below 1e-6 mm")]
    InvalidTolerance {
        /// The vertex or edge.
        entity: EntityKey,
        /// Its tolerance.
        value: f64,
    },

    /// A vertex is not finite or lies outside the modelling space.
    #[error("{} lies outside the modelling space", EntityKey::Vertex(*.vertex))]
    OutsideModelSpace {
        /// The vertex.
        vertex: VertexKey,
    },

    /// An edge's range is not finite, empty, longer than a period of its
    /// curve or outside a bounded curve's domain.
    #[error("{} has an invalid parameter range [{}, {}]", EntityKey::Edge(*.edge), .range.min, .range.max)]
    InvalidRange {
        /// The edge.
        edge: EdgeKey,
        /// Its range.
        range: Interval,
    },

    /// The curve's point at one end of an edge is not at the vertex there.
    #[error("{} ends {distance} mm away from {}", EntityKey::Edge(*.edge), EntityKey::Vertex(*.vertex))]
    EdgeVertexMismatch {
        /// The edge.
        edge: EdgeKey,
        /// The vertex at that end.
        vertex: VertexKey,
        /// The distance between them.
        distance: f64,
    },

    /// A vertex does not lie on the curve of an edge that ends there.
    #[error("{} lies {distance} mm off the curve of {}", EntityKey::Vertex(*.vertex), EntityKey::Edge(*.edge))]
    VertexOffEdge {
        /// The edge.
        edge: EdgeKey,
        /// The vertex.
        vertex: VertexKey,
        /// Its distance from the curve.
        distance: f64,
    },

    /// An edge's `degenerate` flag does not match its geometry.
    #[error("{}: {reason}", EntityKey::Edge(*.edge))]
    DegenerateMismatch {
        /// The edge.
        edge: EdgeKey,
        /// What is wrong.
        reason: &'static str,
    },

    /// A pcurve mapped through its face's surface strays from the edge
    /// curve.
    #[error("the pcurve of {} strays {distance} mm from its edge", EntityKey::Coedge(*.coedge))]
    PcurveMismatch {
        /// The coedge.
        coedge: CoedgeKey,
        /// The largest distance found.
        distance: f64,
    },

    /// Consecutive coedges of a loop do not share a vertex.
    #[error(
        "{} is open: {} ends at a different vertex than {} starts",
        EntityKey::Loop(*.loop_), EntityKey::Coedge(*.coedge), EntityKey::Coedge(*.next)
    )]
    LoopVertexGap {
        /// The loop.
        loop_: LoopKey,
        /// The coedge whose end is wrong.
        coedge: CoedgeKey,
        /// The coedge after it.
        next: CoedgeKey,
    },

    /// Consecutive pcurves of a loop do not meet in `(u, v)`, even
    /// allowing one period.
    #[error(
        "the pcurves of {} and {} in {} are {gap} apart in (u, v)",
        EntityKey::Coedge(*.coedge), EntityKey::Coedge(*.next), EntityKey::Loop(*.loop_)
    )]
    PcurveGap {
        /// The loop.
        loop_: LoopKey,
        /// The coedge whose end is wrong.
        coedge: CoedgeKey,
        /// The coedge after it.
        next: CoedgeKey,
        /// The size of the gap in parameter units.
        gap: f64,
    },

    /// A loop turns the wrong way in `(u, v)` for its place in its face
    /// (or encloses no area).
    #[error("{} turns the wrong way in (u, v) (signed area {area})", EntityKey::Loop(*.loop_))]
    LoopOrientation {
        /// The loop.
        loop_: LoopKey,
        /// The signed area of its `(u, v)` polygon.
        area: f64,
    },

    /// An edge has the wrong number of coedges for its kind and shell.
    #[error("{} is used by {count} coedges", EntityKey::Edge(*.edge))]
    EdgeUseCount {
        /// The edge.
        edge: EdgeKey,
        /// How many coedges use it.
        count: usize,
    },

    /// The two coedges of an edge travel it in the same direction, so the
    /// faces on either side are oriented inconsistently.
    #[error(
        "{} and {} both travel {} the same way",
        EntityKey::Coedge(.coedges[0]), EntityKey::Coedge(.coedges[1]), EntityKey::Edge(*.edge)
    )]
    InconsistentOrientation {
        /// The edge.
        edge: EdgeKey,
        /// Its two coedges.
        coedges: [CoedgeKey; 2],
    },

    /// An edge used twice by one face is not a proper seam: its pcurves are
    /// not one period apart in exactly one periodic direction.
    #[error("{} is used twice by {} but is not a proper seam", EntityKey::Edge(*.edge), EntityKey::Face(*.face))]
    BadSeam {
        /// The edge.
        edge: EdgeKey,
        /// The face.
        face: FaceKey,
    },

    /// An edge is used by faces of different shells.
    #[error("{} is used by more than one shell", EntityKey::Edge(*.edge))]
    EdgeInSeveralShells {
        /// The edge.
        edge: EdgeKey,
    },

    /// A shell's faces are not all connected through shared edges.
    #[error("the faces of {} are not connected", EntityKey::Shell(*.shell))]
    ShellNotConnected {
        /// The shell.
        shell: ShellKey,
    },

    /// A solid has a shell that is not closed.
    #[error("{} of a solid is not closed", EntityKey::Shell(*.shell))]
    OpenShellInSolid {
        /// The shell.
        shell: ShellKey,
    },

    /// A closed shell's counts give no valid genus.
    #[error(
        "{} fails Euler-Poincare: V {} - E {} + F {} - (L {} - F) = {} is not 2 (1 - G) for any genus G",
        EntityKey::Shell(*.shell), .counts.vertices, .counts.edges, .counts.faces,
        .counts.loops, .counts.characteristic()
    )]
    EulerPoincare {
        /// The shell.
        shell: ShellKey,
        /// Its counts.
        counts: EulerCounts,
    },

    /// Several entities share a name.
    #[error("{} entities share the name {name}", .entities.len())]
    DuplicateName {
        /// The shared name.
        name: TopoName,
        /// The entities that carry it.
        entities: Vec<EntityKey>,
    },
}

/// Checks `body` and returns every issue found; empty means valid (see the
/// [module documentation](self) for the checks).
pub fn validate(body: &Body) -> Vec<Issue> {
    let mut v = Validator {
        body,
        issues: Vec::new(),
    };
    v.references();
    v.values();
    v.edge_geometry();
    v.pcurves();
    v.loops();
    v.edge_uses();
    v.shells();
    v.names();
    v.issues
}

/// Returns `true` if `value <= limit`; `false` if either is NaN, so a NaN
/// distance always counts as out of tolerance.
pub(crate) fn within(value: f64, limit: f64) -> bool {
    matches!(
        value.partial_cmp(&limit),
        Some(Ordering::Less | Ordering::Equal)
    )
}

/// `SAMPLES + 1` evenly spaced parameters across `r`, ends included.
fn params(r: Interval) -> impl Iterator<Item = f64> {
    (0..=SAMPLES).map(move |i| r.lerp(i as f64 / SAMPLES as f64))
}

/// Returns `true` if `r` is a usable edge range.
fn usable(r: Interval) -> bool {
    r.min.is_finite() && r.max.is_finite() && r.min < r.max
}

/// `d` reduced by the nearest multiple `k` of `period` with `|k| <= 1`:
/// the residual and `k`.
fn reduce(d: f64, period: Option<f64>) -> (f64, i32) {
    match period {
        Some(p) if p > 0.0 => {
            let k = (d / p).round().clamp(-1.0, 1.0);
            (d - k * p, k as i32)
        }
        _ => (d, 0),
    }
}

struct Validator<'a> {
    body: &'a Body,
    issues: Vec<Issue>,
}

impl Validator<'_> {
    fn push(&mut self, issue: Issue) {
        self.issues.push(issue);
    }

    /// Check 1: keys exist, back-references agree, nothing is listed twice
    /// or left unused.
    fn references(&mut self) {
        self.wire_references();
        self.loop_references();
        self.face_references();
        self.shell_references();
    }

    /// References between vertices, edges, coedges and loops.
    fn wire_references(&mut self) {
        let b = self.body;

        let mut used_vertices = BTreeSet::new();
        for (ek, e) in &b.edges {
            for v in [e.start, e.end] {
                if b.vertices.contains_key(v) {
                    used_vertices.insert(v);
                } else {
                    self.push(Issue::MissingEntity {
                        from: EntityKey::Edge(ek),
                        to: EntityKey::Vertex(v),
                    });
                }
            }
        }
        for vk in b.vertices.keys() {
            if !used_vertices.contains(&vk) {
                self.push(Issue::Orphan {
                    entity: EntityKey::Vertex(vk),
                });
            }
        }

        let mut used_edges = BTreeSet::new();
        for (ck, c) in &b.coedges {
            if b.edges.contains_key(c.edge) {
                used_edges.insert(c.edge);
            } else {
                self.push(Issue::MissingEntity {
                    from: EntityKey::Coedge(ck),
                    to: EntityKey::Edge(c.edge),
                });
            }
            match b.loops.get(c.loop_) {
                None => self.push(Issue::MissingEntity {
                    from: EntityKey::Coedge(ck),
                    to: EntityKey::Loop(c.loop_),
                }),
                Some(l) if !l.coedges.contains(&ck) => self.push(Issue::BackReference {
                    entity: EntityKey::Coedge(ck),
                    owner: EntityKey::Loop(c.loop_),
                }),
                Some(_) => {}
            }
        }
        for ek in b.edges.keys() {
            if !used_edges.contains(&ek) {
                self.push(Issue::Orphan {
                    entity: EntityKey::Edge(ek),
                });
            }
        }
    }

    /// References between loops, their coedges and their faces.
    fn loop_references(&mut self) {
        let b = self.body;
        let mut coedge_lists: BTreeMap<CoedgeKey, usize> = BTreeMap::new();
        for (lk, l) in &b.loops {
            if l.coedges.is_empty() {
                self.push(Issue::EmptyLoop { loop_: lk });
            }
            for &ck in &l.coedges {
                *coedge_lists.entry(ck).or_default() += 1;
                match b.coedges.get(ck) {
                    None => self.push(Issue::MissingEntity {
                        from: EntityKey::Loop(lk),
                        to: EntityKey::Coedge(ck),
                    }),
                    Some(c) if c.loop_ != lk => self.push(Issue::BackReference {
                        entity: EntityKey::Coedge(ck),
                        owner: EntityKey::Loop(lk),
                    }),
                    Some(_) => {}
                }
            }
            match b.faces.get(l.face) {
                None => self.push(Issue::MissingEntity {
                    from: EntityKey::Loop(lk),
                    to: EntityKey::Face(l.face),
                }),
                Some(f) if !f.loops.contains(&lk) => self.push(Issue::BackReference {
                    entity: EntityKey::Loop(lk),
                    owner: EntityKey::Face(l.face),
                }),
                Some(_) => {}
            }
        }
        for ck in b.coedges.keys() {
            self.count_listing(EntityKey::Coedge(ck), coedge_lists.get(&ck).copied());
        }
    }

    /// References between faces, their loops and their shells.
    fn face_references(&mut self) {
        let b = self.body;
        let mut loop_lists: BTreeMap<LoopKey, usize> = BTreeMap::new();
        for (fk, f) in &b.faces {
            if f.loops.is_empty() {
                self.push(Issue::FaceWithoutLoops { face: fk });
            }
            for &lk in &f.loops {
                *loop_lists.entry(lk).or_default() += 1;
                match b.loops.get(lk) {
                    None => self.push(Issue::MissingEntity {
                        from: EntityKey::Face(fk),
                        to: EntityKey::Loop(lk),
                    }),
                    Some(l) if l.face != fk => self.push(Issue::BackReference {
                        entity: EntityKey::Loop(lk),
                        owner: EntityKey::Face(fk),
                    }),
                    Some(_) => {}
                }
            }
            if !f.shell.is_null() {
                match b.shell_arena.get(f.shell) {
                    None => self.push(Issue::MissingEntity {
                        from: EntityKey::Face(fk),
                        to: EntityKey::Shell(f.shell),
                    }),
                    Some(s) if !s.faces.contains(&fk) => self.push(Issue::BackReference {
                        entity: EntityKey::Face(fk),
                        owner: EntityKey::Shell(f.shell),
                    }),
                    Some(_) => {}
                }
            }
        }
        for lk in b.loops.keys() {
            self.count_listing(EntityKey::Loop(lk), loop_lists.get(&lk).copied());
        }
    }

    /// References between shells, their faces and the body.
    fn shell_references(&mut self) {
        let b = self.body;
        let mut face_lists: BTreeMap<FaceKey, usize> = BTreeMap::new();
        for (sk, s) in &b.shell_arena {
            if s.faces.is_empty() {
                self.push(Issue::EmptyShell { shell: sk });
            }
            for &fk in &s.faces {
                *face_lists.entry(fk).or_default() += 1;
                match b.faces.get(fk) {
                    None => self.push(Issue::MissingEntity {
                        from: EntityKey::Shell(sk),
                        to: EntityKey::Face(fk),
                    }),
                    Some(f) if f.shell != sk => self.push(Issue::BackReference {
                        entity: EntityKey::Face(fk),
                        owner: EntityKey::Shell(sk),
                    }),
                    Some(_) => {}
                }
            }
        }
        for fk in b.faces.keys() {
            self.count_listing(EntityKey::Face(fk), face_lists.get(&fk).copied());
        }

        let mut shell_lists: BTreeMap<ShellKey, usize> = BTreeMap::new();
        for &sk in &b.shells {
            *shell_lists.entry(sk).or_default() += 1;
            if !b.shell_arena.contains_key(sk) {
                self.push(Issue::MissingShell { shell: sk });
            }
        }
        for sk in b.shell_arena.keys() {
            self.count_listing(EntityKey::Shell(sk), shell_lists.get(&sk).copied());
        }
        if b.shells.is_empty() {
            self.push(Issue::NoShells);
        }
    }

    /// Reports an entity listed by no owner or by more than one.
    fn count_listing(&mut self, entity: EntityKey, count: Option<usize>) {
        match count {
            None | Some(0) => self.push(Issue::Orphan { entity }),
            Some(1) => {}
            Some(_) => self.push(Issue::ListedTwice { entity }),
        }
    }

    /// Check 2: tolerances, vertex positions and edge ranges.
    fn values(&mut self) {
        let b = self.body;
        let bad_tol = |t: f64| !(t.is_finite() && t >= tol::LINEAR);
        for (vk, v) in &b.vertices {
            if bad_tol(v.tol) {
                self.push(Issue::InvalidTolerance {
                    entity: EntityKey::Vertex(vk),
                    value: v.tol,
                });
            }
            if !tol::in_model_space(v.point) {
                self.push(Issue::OutsideModelSpace { vertex: vk });
            }
        }
        for (ek, e) in &b.edges {
            if bad_tol(e.tol) {
                self.push(Issue::InvalidTolerance {
                    entity: EntityKey::Edge(ek),
                    value: e.tol,
                });
            }
            let r = e.range;
            let fits = match e.curve.domain() {
                Domain::Periodic(p) => r.length() <= p.length() * (1.0 + 1e-12),
                Domain::Bounded(d) => {
                    let slack = 1e-12 * d.length().max(1.0);
                    d.contains_with(r.min, slack) && d.contains_with(r.max, slack)
                }
                Domain::Unbounded => true,
            };
            if !usable(r) || !fits {
                self.push(Issue::InvalidRange { edge: ek, range: r });
            }
        }
    }

    /// Check 3: edge ends, vertices on edges, degenerate edges.
    fn edge_geometry(&mut self) {
        let b = self.body;
        for (ek, e) in &b.edges {
            if !usable(e.range) {
                continue;
            }
            for (vk, t) in [(e.start, e.range.min), (e.end, e.range.max)] {
                let Some(v) = b.vertices.get(vk) else {
                    continue;
                };
                let tol = e.tol.max(v.tol);
                let distance = e.point(t).distance(v.point);
                if !within(distance, tol) {
                    self.push(Issue::EdgeVertexMismatch {
                        edge: ek,
                        vertex: vk,
                        distance,
                    });
                }
                if !e.degenerate
                    && let Ok(cp) = e.curve.closest_point(v.point)
                    && !within(cp.distance, tol)
                {
                    self.push(Issue::VertexOffEdge {
                        edge: ek,
                        vertex: vk,
                        distance: cp.distance,
                    });
                }
            }
            if e.degenerate {
                if e.start != e.end {
                    self.push(Issue::DegenerateMismatch {
                        edge: ek,
                        reason: "a degenerate edge must start and end at the same vertex",
                    });
                }
                if let Some(v) = b.vertices.get(e.start) {
                    let tol = e.tol.max(v.tol);
                    if params(e.range).any(|t| !within(e.point(t).distance(v.point), tol)) {
                        self.push(Issue::DegenerateMismatch {
                            edge: ek,
                            reason: "the curve of a degenerate edge must stay at its vertex",
                        });
                    }
                }
            } else {
                let points: Vec<_> = params(e.range).map(|t| e.point(t)).collect();
                let length: f64 = points.windows(2).map(|w| w[0].distance(w[1])).sum();
                if within(length, e.tol) || length.is_nan() {
                    self.push(Issue::DegenerateMismatch {
                        edge: ek,
                        reason: "the edge has no length but is not marked degenerate",
                    });
                }
            }
        }
    }

    /// Check 4: every pcurve agrees with its edge through the face's
    /// surface.
    fn pcurves(&mut self) {
        let b = self.body;
        for (ck, c) in &b.coedges {
            let Some(e) = b.edges.get(c.edge) else {
                continue;
            };
            let Some(face) = b.loops.get(c.loop_).and_then(|l| b.faces.get(l.face)) else {
                continue;
            };
            if !usable(e.range) {
                continue;
            }
            let distance = params(e.range)
                .map(|t| {
                    let uv = c.pcurve.point(t);
                    face.surface.point(uv.x, uv.y).distance(e.point(t))
                })
                // `f64::max` ignores NaN; keep it, so a NaN sample is
                // reported.
                .fold(0.0, |acc: f64, d| {
                    if acc.is_nan() || d.is_nan() {
                        f64::NAN
                    } else {
                        acc.max(d)
                    }
                });
            if !within(distance, e.tol) {
                self.push(Issue::PcurveMismatch {
                    coedge: ck,
                    distance,
                });
            }
        }
    }

    /// The parametric tolerances `(tol_u, tol_v)` at `uv` on `face`'s
    /// surface for the 3D tolerance `tol`.
    fn uv_tolerance(&self, face: FaceKey, uv: Point2, tol: f64) -> (f64, f64) {
        match self.body.faces.get(face) {
            Some(f) => {
                let (su, sv) = f.surface.partials(uv.x, uv.y);
                (tol / su.length().max(1.0), tol / sv.length().max(1.0))
            }
            None => (tol, tol),
        }
    }

    /// Check 5: loops close in 3D and in `(u, v)`, and turn the right way.
    fn loops(&mut self) {
        let b = self.body;
        for (lk, l) in &b.loops {
            let Some(face) = b.faces.get(l.face) else {
                continue;
            };
            let periods = (face.surface.period_u(), face.surface.period_v());
            let mut jumped = false;
            let mut broken = false;
            let next_keys = l.coedges.iter().cycle().skip(1);
            for (&ck, &nk) in l.coedges.iter().zip(next_keys) {
                let (Some(c), Some(nc)) = (b.coedges.get(ck), b.coedges.get(nk)) else {
                    broken = true;
                    continue;
                };
                let (Some(e), Some(ne)) = (b.edges.get(c.edge), b.edges.get(nc.edge)) else {
                    broken = true;
                    continue;
                };
                let vertex = c.end_vertex(e);
                if vertex != nc.start_vertex(ne) {
                    self.push(Issue::LoopVertexGap {
                        loop_: lk,
                        coedge: ck,
                        next: nk,
                    });
                    broken = true;
                    continue;
                }
                if !usable(e.range) || !usable(ne.range) {
                    broken = true;
                    continue;
                }
                let (p, q) = (c.end_uv(e), nc.start_uv(ne));
                let vertex_tol = b.vertices.get(vertex).map_or(0.0, |v| v.tol);
                let tol = e.tol.max(ne.tol).max(vertex_tol);
                let (tol_u, tol_v) = self.uv_tolerance(l.face, p, tol);
                let (du, ku) = reduce(q.x - p.x, periods.0);
                let (dv, kv) = reduce(q.y - p.y, periods.1);
                jumped |= ku != 0 || kv != 0;
                if !(within(du.abs(), tol_u) && within(dv.abs(), tol_v)) {
                    self.push(Issue::PcurveGap {
                        loop_: lk,
                        coedge: ck,
                        next: nk,
                        gap: du.hypot(dv),
                    });
                    broken = true;
                }
            }
            if l.coedges.is_empty() || jumped || broken {
                continue;
            }
            let area = self.loop_area(&l.coedges);
            let outer = face.loops.first() == Some(&lk);
            let counter_clockwise = outer != face.reversed;
            let right_way = if counter_clockwise {
                area > 0.0
            } else {
                area < 0.0
            };
            if !right_way {
                self.push(Issue::LoopOrientation { loop_: lk, area });
            }
        }
    }

    /// The signed area (shoelace formula) of the `(u, v)` polygon through
    /// `SAMPLES` points of each coedge's pcurve, in travel order.
    fn loop_area(&self, coedges: &[CoedgeKey]) -> f64 {
        let b = self.body;
        let mut points: Vec<Point2> = Vec::new();
        for &ck in coedges {
            let Some(c) = b.coedges.get(ck) else {
                continue;
            };
            let Some(e) = b.edges.get(c.edge) else {
                continue;
            };
            let (t0, t1) = (c.start_param(e), c.end_param(e));
            for i in 0..SAMPLES {
                let s = i as f64 / SAMPLES as f64;
                points.push(c.pcurve.point(t0 + s * (t1 - t0)));
            }
        }
        let next = points.iter().cycle().skip(1);
        0.5 * points
            .iter()
            .zip(next)
            .map(|(p, q)| p.perp_dot(*q))
            .sum::<f64>()
    }

    /// Check 6: how many coedges use each edge, orientation across edges,
    /// seams.
    fn edge_uses(&mut self) {
        let b = self.body;
        let mut uses: BTreeMap<EdgeKey, Vec<CoedgeKey>> = BTreeMap::new();
        for (ck, c) in &b.coedges {
            uses.entry(c.edge).or_default().push(ck);
        }
        for (ek, e) in &b.edges {
            let Some(list) = uses.get(&ek) else {
                continue;
            };
            let mut shells: Vec<ShellKey> = Vec::new();
            for &ck in list {
                let shell = b
                    .coedge_face(ck)
                    .and_then(|f| b.faces.get(f))
                    .map(|f| f.shell);
                if let Some(s) = shell
                    && !shells.contains(&s)
                {
                    shells.push(s);
                }
            }
            if shells.len() > 1 {
                self.push(Issue::EdgeInSeveralShells { edge: ek });
            }
            let closed = shells
                .first()
                .and_then(|&s| b.shell_arena.get(s))
                .is_some_and(|s| s.closed);
            let count = list.len();
            let count_ok = if e.degenerate {
                count == 1
            } else if closed {
                count == 2
            } else {
                count <= 2
            };
            if !count_ok {
                self.push(Issue::EdgeUseCount { edge: ek, count });
            }
            if let [a, c] = list.as_slice() {
                let (Some(ca), Some(cc)) = (b.coedges.get(*a), b.coedges.get(*c)) else {
                    continue;
                };
                if ca.reversed == cc.reversed {
                    self.push(Issue::InconsistentOrientation {
                        edge: ek,
                        coedges: [*a, *c],
                    });
                }
                let (fa, fc) = (b.coedge_face(*a), b.coedge_face(*c));
                if let (Some(fa), Some(fc)) = (fa, fc)
                    && fa == fc
                    && usable(e.range)
                    && !self.proper_seam(fa, &ca.pcurve, &cc.pcurve, e)
                {
                    self.push(Issue::BadSeam { edge: ek, face: fa });
                }
            }
        }
    }

    /// Returns `true` if the two pcurves of a seam are one period apart in
    /// exactly one periodic direction of `face`, at every sample.
    fn proper_seam(
        &self,
        face: FaceKey,
        a: &bevel_geom::Curve2,
        c: &bevel_geom::Curve2,
        edge: &crate::topology::Edge,
    ) -> bool {
        let Some(f) = self.body.faces.get(face) else {
            return false;
        };
        let periods = (f.surface.period_u(), f.surface.period_v());
        let mut shift: Option<(i32, i32)> = None;
        for t in params(edge.range) {
            let (pa, pc) = (a.point(t), c.point(t));
            let (du, ku) = reduce(pc.x - pa.x, periods.0);
            let (dv, kv) = reduce(pc.y - pa.y, periods.1);
            let (tol_u, tol_v) = self.uv_tolerance(face, pa, edge.tol);
            if !(within(du.abs(), tol_u) && within(dv.abs(), tol_v)) {
                return false;
            }
            if (ku != 0) == (kv != 0) {
                return false;
            }
            match shift {
                None => shift = Some((ku, kv)),
                Some(s) if s != (ku, kv) => return false,
                Some(_) => {}
            }
        }
        true
    }

    /// Check 7: shells of solids are closed, shells are connected, closed
    /// shells satisfy Euler-Poincare.
    fn shells(&mut self) {
        let b = self.body;
        for (sk, s) in &b.shell_arena {
            if b.kind == BodyKind::Solid && !s.closed {
                self.push(Issue::OpenShellInSolid { shell: sk });
            }
            if !self.connected(&s.faces) {
                self.push(Issue::ShellNotConnected { shell: sk });
            }
            if s.closed
                && let Some(counts) = b.euler_counts(sk)
                && counts.genus().is_none()
            {
                self.push(Issue::EulerPoincare { shell: sk, counts });
            }
        }
    }

    /// Returns `true` if `faces` are connected through shared edges.
    fn connected(&self, faces: &[FaceKey]) -> bool {
        let b = self.body;
        let Some(&first) = faces.first() else {
            return true;
        };
        let wanted: BTreeSet<FaceKey> = faces.iter().copied().collect();
        let mut edge_faces: BTreeMap<EdgeKey, Vec<FaceKey>> = BTreeMap::new();
        for &f in &wanted {
            for e in b.face_edges(f) {
                edge_faces.entry(e).or_default().push(f);
            }
        }
        let mut seen = BTreeSet::from([first]);
        let mut stack = vec![first];
        while let Some(f) = stack.pop() {
            for e in b.face_edges(f) {
                for &g in edge_faces.get(&e).into_iter().flatten() {
                    if seen.insert(g) {
                        stack.push(g);
                    }
                }
            }
        }
        seen.len() == wanted.len()
    }

    /// Check 8: names are unique.
    fn names(&mut self) {
        let b = self.body;
        let mut names: BTreeMap<&TopoName, Vec<EntityKey>> = BTreeMap::new();
        for (k, v) in &b.vertices {
            names.entry(&v.name).or_default().push(EntityKey::Vertex(k));
        }
        for (k, e) in &b.edges {
            names.entry(&e.name).or_default().push(EntityKey::Edge(k));
        }
        for (k, f) in &b.faces {
            names.entry(&f.name).or_default().push(EntityKey::Face(k));
        }
        for (name, entities) in names {
            if entities.len() > 1 {
                self.push(Issue::DuplicateName {
                    name: name.clone(),
                    entities,
                });
            }
        }
    }
}
