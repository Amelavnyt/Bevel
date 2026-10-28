// SPDX-License-Identifier: GPL-3.0-or-later

//! Imprinting: splitting each face into fragments along the pieces that
//! cross it.
//!
//! In a face's parameter space the pieces form a planar graph: boundary
//! pieces once, in their loops' direction, and pieces crossing the face
//! (intersections and imprints) in both directions. Walking it so that the
//! face always stays on the left (at each node, the next piece is the one
//! that turns least clockwise from the way back) traces cycles: those with
//! positive area bound fragments, those with negative area are holes,
//! each assigned to the smallest fragment around it.

use super::cut::Inputs;
use super::pieces::{FaceRef, Piece};
use crate::KernelError;
use crate::topology::EntityKey;
use bevel_geom::{CurveGeom, SurfaceGeom};
use bevel_math::predicates::orient2d;
use bevel_math::{Point2, Point3};
use std::collections::BTreeMap;
use std::f64::consts::TAU;

/// A piece travelled one way on one face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct HalfEdge {
    pub piece: usize,
    /// Which of the piece's uses (its pcurve on this face).
    pub use_: usize,
    /// Along the piece's parameter, or against it.
    pub forward: bool,
}

/// A region of a face between pieces.
#[derive(Clone, Debug)]
pub(crate) struct Fragment {
    pub face: FaceRef,
    /// The outer cycle first, then the holes.
    pub cycles: Vec<Vec<HalfEdge>>,
    /// A point inside, in parameter space and in space.
    pub uv: Point2,
    pub point: Point3,
}

/// Samples along each half-edge for areas and containment tests.
const SAMPLES: usize = 16;

/// How far (in units of the node tolerance) the end of a half-edge may
/// sit from the start of the next in parameter space.
const NEAR_COPY: f64 = 1e3;

struct Graph<'p> {
    pieces: &'p [Piece],
    /// `-1` when the face is reversed: parameter space is mirrored in `v`
    /// so that the face is always on the left of its loops.
    sign: f64,
    edges: Vec<HalfEdge>,
}

impl Graph<'_> {
    /// The point at fraction `s` along `h` (in travel order), mirrored.
    fn at(&self, h: HalfEdge, s: f64) -> Point2 {
        let p = &self.pieces[h.piece];
        let s = if h.forward { s } else { 1.0 - s };
        let t = p.range.min + s * p.range.length();
        let q = p.uses[h.use_].pcurve.point(t);
        Point2::new(q.x, self.sign * q.y)
    }

    fn start_vertex(&self, h: HalfEdge) -> usize {
        let p = &self.pieces[h.piece];
        if h.forward { p.start } else { p.end }
    }

    fn end_vertex(&self, h: HalfEdge) -> usize {
        let p = &self.pieces[h.piece];
        if h.forward { p.end } else { p.start }
    }

    /// The direction `h` leaves its start in, as an angle.
    fn leave_angle(&self, h: HalfEdge) -> f64 {
        let d = self.at(h, 1e-4) - self.at(h, 0.0);
        let d = if d.length() > 0.0 {
            d
        } else {
            self.at(h, 1e-2) - self.at(h, 0.0)
        };
        d.y.atan2(d.x)
    }

    /// The direction back along `h` from its end, as an angle.
    fn back_angle(&self, h: HalfEdge) -> f64 {
        let d = self.at(h, 1.0 - 1e-4) - self.at(h, 1.0);
        let d = if d.length() > 0.0 {
            d
        } else {
            self.at(h, 1.0 - 1e-2) - self.at(h, 1.0)
        };
        d.y.atan2(d.x)
    }

    /// Whether `a`'s end and `b`'s start are the same node: the same vertex
    /// at the same place in parameter space (a seam's two sides are not).
    fn meets(&self, a: HalfEdge, b: HalfEdge, eps: f64) -> bool {
        self.end_vertex(a) == self.start_vertex(b)
            && (self.at(a, 1.0) - self.at(b, 0.0)).length() <= eps
    }

    fn twins(a: HalfEdge, b: HalfEdge) -> bool {
        a.piece == b.piece && a.use_ == b.use_ && a.forward != b.forward
    }

    /// The half-edge after `h`: of those leaving `h`'s end, the first
    /// clockwise from the way back.
    fn next(&self, h: HalfEdge, eps: f64) -> Option<HalfEdge> {
        let back = self.back_angle(h);
        // The half-edges leaving the same copy of the end vertex. Near a
        // pole a pcurve's end can sit well over `eps` off in `u` while a
        // hair off in space, so allow more: the other copies (across a
        // seam, elsewhere along a pole) are much further.
        let end = self.at(h, 1.0);
        let others: Vec<HalfEdge> = self
            .edges
            .iter()
            .copied()
            .filter(|&o| {
                self.end_vertex(h) == self.start_vertex(o)
                    && !Self::twins(h, o)
                    && (self.at(o, 0.0) - end).length() <= NEAR_COPY * eps
            })
            .collect();
        let pool = if others.is_empty() {
            self.edges
                .iter()
                .copied()
                .filter(|&o| self.meets(h, o, eps))
                .collect()
        } else {
            others
        };
        pool.into_iter().min_by(|&x, &y| {
            let turn = |o: HalfEdge| {
                let d = (back - self.leave_angle(o)).rem_euclid(TAU);
                if d <= 1e-12 { TAU } else { d }
            };
            turn(x).total_cmp(&turn(y))
        })
    }

    fn polygon(&self, cycle: &[HalfEdge]) -> Vec<Point2> {
        let mut pts = Vec::with_capacity(cycle.len() * SAMPLES + 1);
        for &h in cycle {
            for i in 0..SAMPLES {
                pts.push(self.at(h, i as f64 / SAMPLES as f64));
            }
        }
        if let Some(&first) = pts.first() {
            pts.push(first);
        }
        pts
    }
}

fn area(poly: &[Point2]) -> f64 {
    0.5 * poly
        .windows(2)
        .map(|w| w[0].x * w[1].y - w[1].x * w[0].y)
        .sum::<f64>()
}

fn winding(poly: &[Point2], p: Point2) -> i32 {
    let mut w = 0;
    for s in poly.windows(2) {
        let (a, b) = (s[0], s[1]);
        if a.y <= p.y {
            if b.y > p.y && orient2d(a, b, p) > 0.0 {
                w += 1;
            }
        } else if b.y <= p.y && orient2d(a, b, p) < 0.0 {
            w -= 1;
        }
    }
    w
}

fn distance_to(poly: &[Point2], p: Point2) -> f64 {
    poly.windows(2)
        .map(|s| super::domain::segment_distance(p, s[0], s[1]))
        .fold(f64::INFINITY, f64::min)
}

/// Splits every face of both inputs into fragments.
///
/// # Errors
///
/// [`KernelError::InvalidInput`] if the pieces on a face do not form
/// closed cycles.
pub(crate) fn fragments(
    inputs: &Inputs<'_>,
    pieces: &[Piece],
) -> Result<Vec<Fragment>, KernelError> {
    let mut out = Vec::new();
    for (b, body) in inputs.bodies.iter().enumerate() {
        for (fk, _) in body.faces() {
            out.extend(face_fragments(inputs, pieces, (b, fk))?);
        }
    }
    Ok(out)
}

#[expect(
    clippy::too_many_lines,
    reason = "building the face's graph and tracing it are one pass"
)]
fn face_fragments(
    inputs: &Inputs<'_>,
    pieces: &[Piece],
    f: FaceRef,
) -> Result<Vec<Fragment>, KernelError> {
    let body = inputs.bodies[f.0];
    let face = inputs.face(f)?;
    let rect = inputs.domain(f)?.rect;
    let eps = 1e-7 * rect.u.length().hypot(rect.v.length());
    let mut edges = Vec::new();
    // Boundary: each coedge's pieces, in loop order and direction.
    for &lk in &face.loops {
        let l = body
            .loop_(lk)
            .ok_or(KernelError::UnknownKey(EntityKey::Loop(lk)))?;
        for &ck in &l.coedges {
            let c = body
                .coedge(ck)
                .ok_or(KernelError::UnknownKey(EntityKey::Coedge(ck)))?;
            // The coedge's pieces in order along its edge, then in its
            // travel direction.
            let mut own: Vec<(f64, HalfEdge)> = pieces
                .iter()
                .enumerate()
                .filter_map(|(pi, p)| {
                    let ui = p
                        .uses
                        .iter()
                        .position(|u| u.coedge == Some(ck) && u.face == f)?;
                    let u = &p.uses[ui];
                    Some((
                        u.order,
                        HalfEdge {
                            piece: pi,
                            use_: ui,
                            forward: c.reversed != u.along,
                        },
                    ))
                })
                .collect();
            own.sort_by(|x, y| x.0.total_cmp(&y.0));
            let mut own: Vec<HalfEdge> = own.into_iter().map(|(_, h)| h).collect();
            if c.reversed {
                own.reverse();
            }
            edges.extend(own);
        }
    }
    // Pieces crossing the face: both ways.
    for (pi, p) in pieces.iter().enumerate() {
        for (ui, u) in p.uses.iter().enumerate() {
            if u.face == f && u.coedge.is_none() {
                edges.push(HalfEdge {
                    piece: pi,
                    use_: ui,
                    forward: true,
                });
                edges.push(HalfEdge {
                    piece: pi,
                    use_: ui,
                    forward: false,
                });
            }
        }
    }
    prune_dangling(pieces, &mut edges);
    let graph = Graph {
        pieces,
        sign: if face.reversed { -1.0 } else { 1.0 },
        edges,
    };
    // Trace the cycles.
    let mut used = vec![false; graph.edges.len()];
    let mut outers: Vec<(Vec<HalfEdge>, Vec<Point2>, f64)> = Vec::new();
    let mut holes: Vec<(Vec<HalfEdge>, Vec<Point2>)> = Vec::new();
    for start in 0..graph.edges.len() {
        if used[start] {
            continue;
        }
        let mut cycle = Vec::new();
        let mut h = graph.edges[start];
        loop {
            let Some(i) = graph.edges.iter().position(|&e| e == h) else {
                return Err(broken());
            };
            if used[i] {
                if i == start {
                    break;
                }
                return Err(broken());
            }
            used[i] = true;
            cycle.push(h);
            h = graph.next(h, eps).ok_or_else(broken)?;
            if cycle.len() > graph.edges.len() {
                return Err(broken());
            }
        }
        let poly = graph.polygon(&cycle);
        let a = area(&poly);
        if a > 0.0 {
            outers.push((cycle, poly, a));
        } else if a < 0.0 {
            holes.push((cycle, poly));
        }
    }
    // Holes go to the smallest fragment around them.
    let mut cycles: Vec<Vec<Vec<HalfEdge>>> =
        outers.iter().map(|(c, _, _)| vec![c.clone()]).collect();
    let mut hole_polys: Vec<Vec<Vec<Point2>>> = vec![Vec::new(); outers.len()];
    for (cycle, poly) in holes {
        // Just to the left of the hole's longest half-edge: on the face's
        // side, so the fragment around the hole contains it but the
        // fragment the hole's pieces bound from the other side does not.
        let probe = left_of(&graph, &cycle, eps).ok_or_else(broken)?;
        let around = outers
            .iter()
            .enumerate()
            .filter(|(_, (_, op, _))| winding(op, probe) != 0)
            .min_by(|x, y| x.1.2.total_cmp(&y.1.2))
            .map(|(i, _)| i);
        let Some(i) = around else {
            return Err(broken());
        };
        cycles[i].push(cycle);
        hole_polys[i].push(poly);
    }
    let mut out = Vec::new();
    for (i, (outer, poly, _)) in outers.iter().enumerate() {
        let uv = interior(&graph, outer, poly, &hole_polys[i]).ok_or_else(broken)?;
        let uv = Point2::new(uv.x, graph.sign * uv.y);
        out.push(Fragment {
            face: f,
            cycles: cycles[i].clone(),
            uv,
            point: face.surface.point(uv.x, uv.y),
        });
    }
    Ok(out)
}

/// Drops the pieces crossing a face that end at a vertex nothing else on
/// the face reaches (repeatedly, as dropping one can free another): they
/// bound no region. A tool edge grazing the face within tolerance leaves
/// such a stretch.
fn prune_dangling(pieces: &[Piece], edges: &mut Vec<HalfEdge>) {
    loop {
        let mut degree: BTreeMap<usize, usize> = BTreeMap::new();
        for h in edges.iter().filter(|h| h.forward) {
            let p = &pieces[h.piece];
            *degree.entry(p.start).or_default() += 1;
            *degree.entry(p.end).or_default() += 1;
        }
        // The boundary's pieces appear once each, in one direction.
        for h in edges.iter().filter(|h| !h.forward) {
            let p = &pieces[h.piece];
            if p.uses[h.use_].coedge.is_some() {
                *degree.entry(p.start).or_default() += 1;
                *degree.entry(p.end).or_default() += 1;
            }
        }
        let free = |v: usize| degree.get(&v).copied().unwrap_or(0) <= 1;
        let before = edges.len();
        edges.retain(|h| {
            let p = &pieces[h.piece];
            p.uses[h.use_].coedge.is_some() || p.start == p.end || !(free(p.start) || free(p.end))
        });
        if edges.len() == before {
            return;
        }
    }
}

/// A point just to the left of the middle of the longest half-edge of
/// `cycle`, by `eps` (in mirrored parameter space).
fn left_of(graph: &Graph<'_>, cycle: &[HalfEdge], eps: f64) -> Option<Point2> {
    let h = cycle.iter().copied().max_by(|&x, &y| {
        (graph.at(x, 1.0) - graph.at(x, 0.0))
            .length()
            .total_cmp(&(graph.at(y, 1.0) - graph.at(y, 0.0)).length())
    })?;
    let m = graph.at(h, 0.5);
    let d = graph.at(h, 0.5 + 1e-4) - graph.at(h, 0.5 - 1e-4);
    (d.length() > 0.0).then(|| m + Point2::new(-d.y, d.x) / d.length() * (10.0 * eps))
}

/// A point well inside the fragment bounded by `outer` (with `poly` its
/// polygon) minus the hole polygons: just left of the middle of its longest
/// half-edge, as far in as possible.
fn interior(
    graph: &Graph<'_>,
    outer: &[HalfEdge],
    poly: &[Point2],
    holes: &[Vec<Point2>],
) -> Option<Point2> {
    let longest = outer.iter().copied().max_by(|&x, &y| {
        (graph.at(x, 1.0) - graph.at(x, 0.0))
            .length()
            .total_cmp(&(graph.at(y, 1.0) - graph.at(y, 0.0)).length())
    })?;
    let size = poly.windows(2).map(|w| (w[1] - w[0]).length()).sum::<f64>();
    let mut best: Option<(Point2, f64)> = None;
    for &h in outer.iter().chain(std::iter::once(&longest)) {
        let m = graph.at(h, 0.5);
        let d = graph.at(h, 0.5 + 1e-4) - graph.at(h, 0.5 - 1e-4);
        if d.length() <= 0.0 {
            continue;
        }
        let left = Point2::new(-d.y, d.x) / d.length();
        let mut step = 0.25 * size;
        for _ in 0..40 {
            let q = m + left * step;
            if winding(poly, q) != 0 && holes.iter().all(|hp| winding(hp, q) == 0) {
                let clearance = holes
                    .iter()
                    .map(|hp| distance_to(hp, q))
                    .fold(distance_to(poly, q), f64::min);
                if best.is_none_or(|(_, c)| clearance > c) {
                    best = Some((q, clearance));
                }
            }
            step *= 0.5;
        }
    }
    best.map(|(q, _)| q)
}

fn broken() -> KernelError {
    KernelError::InvalidInput {
        what: "boolean",
        reason: "the pieces on a face do not form closed regions",
    }
}
