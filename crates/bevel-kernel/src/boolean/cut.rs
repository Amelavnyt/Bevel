// SPDX-License-Identifier: GPL-3.0-or-later

//! Cutting: where the faces of the two inputs meet, as pieces.
//!
//! Every pair of faces whose boxes overlap is intersected. Each
//! intersection curve is split where it crosses either face's edges (those
//! edges are split there too), and the parts inside both faces become
//! intersection pieces. A part that runs along an edge of one face and
//! through the other face is that edge's piece, given a pcurve on the
//! other face. Faces on the same surface imprint their edges on each other
//! instead.

use super::classify::{Solid, into_rect};
use super::domain::{Domain, Where};
use super::pieces::{FaceRef, Origin, Piece, Use, Vertices};
use crate::KernelError;
use crate::topology::{Body, EdgeKey, EntityKey};
use bevel_geom::intersect::{
    CurveCurveHit, Rect, SsiResult, curve_curve_3d, pcurve_on, surface_surface,
};
use bevel_geom::{Curve2, Curve3, CurveGeom, SurfaceGeom};
use bevel_math::{Aabb3, Interval, Point2, Vector3};
use std::collections::BTreeMap;

/// An intersection piece, before its ends are tied to vertices.
pub(crate) struct CutPiece {
    pub curve: Curve3,
    pub range: Interval,
    pub tol: f64,
    pub faces: [FaceRef; 2],
    pub pcurves: [Curve2; 2],
}

/// A stretch of an input edge that runs through a face of the other input
/// (along a face it touches, or across a coincident face).
pub(crate) struct Imprint {
    pub body: usize,
    pub edge: EdgeKey,
    pub range: Interval,
    pub face: FaceRef,
}

/// Everything cutting found.
#[derive(Default)]
pub(crate) struct Cuts {
    /// Parameters where input edges are split, by `(body, edge)`.
    pub splits: BTreeMap<(usize, EdgeKey), Vec<f64>>,
    pub pieces: Vec<CutPiece>,
    pub imprints: Vec<Imprint>,
}

/// The inputs prepared for cutting.
pub(crate) struct Inputs<'b> {
    pub bodies: [&'b Body; 2],
    pub solids: [Solid<'b>; 2],
    pub tol: f64,
}

impl Inputs<'_> {
    pub(crate) fn face(&self, f: FaceRef) -> Result<&crate::topology::Face, KernelError> {
        self.bodies[f.0]
            .face(f.1)
            .ok_or(KernelError::UnknownKey(EntityKey::Face(f.1)))
    }

    pub(crate) fn domain(&self, f: FaceRef) -> Result<&Domain, KernelError> {
        self.solids[f.0]
            .domain(f.1)
            .ok_or(KernelError::UnknownKey(EntityKey::Face(f.1)))
    }

    /// Where the point `p` of `face`'s surface (with parameters `uv`, any
    /// period) lies in the face.
    pub(crate) fn where_in(&self, f: FaceRef, uv: Point2) -> Result<(Where, Point2), KernelError> {
        let face = self.face(f)?;
        let d = self.domain(f)?;
        let uv = into_rect(&face.surface, d, uv);
        Ok((
            d.classify(uv, self.solids[f.0].margin(&face.surface, uv)),
            uv,
        ))
    }

    fn face_box(&self, f: FaceRef) -> Result<Aabb3, KernelError> {
        let face = self.face(f)?;
        let r = self.domain(f)?.rect;
        let b = face.surface.bounds(r.u, r.v)?;
        let g = Vector3::splat(self.tol);
        Ok(Aabb3::new(b.min - g, b.max + g))
    }
}

/// Cuts the faces of the two inputs against each other.
///
/// # Errors
///
/// Intersection failures, including [`bevel_geom::GeomError::TangentContact`].
pub(crate) fn cut(inputs: &Inputs<'_>) -> Result<Cuts, KernelError> {
    let mut cuts = Cuts::default();
    let faces = |b: usize| {
        inputs.bodies[b]
            .faces()
            .map(move |(k, _)| (b, k))
            .collect::<Vec<FaceRef>>()
    };
    let (fa, fb) = (faces(0), faces(1));
    let boxes_b = fb
        .iter()
        .map(|f| inputs.face_box(*f))
        .collect::<Result<Vec<_>, _>>()?;
    for a in &fa {
        let box_a = inputs.face_box(*a)?;
        for (b, box_b) in fb.iter().zip(&boxes_b) {
            if !box_a.intersects(box_b) {
                continue;
            }
            let (sa, sb) = (&inputs.face(*a)?.surface, &inputs.face(*b)?.surface);
            let (ra, rb) = (inputs.domain(*a)?.rect, inputs.domain(*b)?.rect);
            match surface_surface(sa, ra, sb, rb, inputs.tol)? {
                SsiResult::Meet { curves, .. } => {
                    for c in curves {
                        split_curve(
                            inputs,
                            &mut cuts,
                            [*a, *b],
                            &c.curve,
                            c.range,
                            [&c.pcurve_a, &c.pcurve_b],
                            c.tolerance,
                        )?;
                    }
                }
                SsiResult::Coincident { .. } => {
                    imprint_edges(inputs, &mut cuts, *b, *a)?;
                    imprint_edges(inputs, &mut cuts, *a, *b)?;
                }
            }
        }
    }
    Ok(cuts)
}

/// The edges of `f`, skipping degenerate ones (points).
fn edges_of(inputs: &Inputs<'_>, f: FaceRef) -> Vec<(EdgeKey, Curve3, Interval)> {
    let body = inputs.bodies[f.0];
    body.face_edges(f.1)
        .into_iter()
        .filter_map(|k| {
            let e = body.edge(k)?;
            (!e.degenerate).then(|| (k, e.curve.clone(), e.range))
        })
        .collect()
}

/// Splits an intersection curve where it crosses the edges of its two
/// faces and keeps the parts inside both.
fn split_curve(
    inputs: &Inputs<'_>,
    cuts: &mut Cuts,
    faces: [FaceRef; 2],
    curve: &Curve3,
    range: Interval,
    pcurves: [&Curve2; 2],
    tol: f64,
) -> Result<(), KernelError> {
    // A fitted curve was checked at its spans' midpoints; the validator
    // samples elsewhere, so give it room.
    let tol = (2.0 * tol).max(inputs.tol);
    let mut params = vec![range.min, range.max];
    for f in faces {
        for (ek, ec, er) in edges_of(inputs, f) {
            for hit in curve_curve_3d(curve, range, &ec, er, inputs.tol)? {
                let (on_curve, on_edge) = match hit {
                    CurveCurveHit::Point { a, b, .. } => (vec![a], vec![b]),
                    CurveCurveHit::Overlap { a, b, .. } => (vec![a.min, a.max], vec![b.min, b.max]),
                };
                params.extend(on_curve);
                cuts.splits.entry((f.0, ek)).or_default().extend(on_edge);
            }
        }
    }
    let params = split_params(curve, range, params, inputs.tol);
    for w in params.windows(2) {
        let part = Interval::new(w[0], w[1]);
        // Slivers go; a closed curve's whole range has coinciding ends but
        // a middle elsewhere.
        if part.length() <= 0.0
            || ((curve.point(part.max) - curve.point(part.min)).length() <= inputs.tol
                && (curve.point(part.mid()) - curve.point(part.min)).length() <= inputs.tol)
        {
            continue;
        }
        let t = part.mid();
        let (wa, _) = inputs.where_in(faces[0], pcurves[0].point(t))?;
        let (wb, _) = inputs.where_in(faces[1], pcurves[1].point(t))?;
        match (wa, wb) {
            (Where::Inside, Where::Inside) => cuts.pieces.push(CutPiece {
                curve: curve.clone(),
                range: part,
                tol,
                faces,
                pcurves: [pcurves[0].clone(), pcurves[1].clone()],
            }),
            // Along an edge of one face, through the other: that edge's
            // stretch goes through the other face.
            (Where::Inside, Where::Boundary) => {
                along_edge(inputs, cuts, faces[1], faces[0], curve, part)?;
            }
            (Where::Boundary, Where::Inside) => {
                along_edge(inputs, cuts, faces[0], faces[1], curve, part)?;
            }
            _ => {}
        }
    }
    Ok(())
}

/// `t` shifted by whole periods of a periodic `curve` to lie in (or
/// nearest) `range`, where an edge's pcurves are defined.
pub(crate) fn into_range(curve: &Curve3, range: Interval, t: f64) -> f64 {
    match curve.domain().period() {
        Some(per) if per > 0.0 => {
            let shifted = t + ((range.mid() - t) / per).round() * per;
            // Prefer a copy inside the range when one is.
            [shifted - per, shifted, shifted + per]
                .into_iter()
                .find(|c| range.contains_with(*c, 1e-9))
                .unwrap_or(shifted)
        }
        _ => t,
    }
}

/// Sorted split parameters over `range`, merging those at the same point
/// but keeping both ends of the range (a closed curve's ends coincide in
/// space but bound its whole range).
fn split_params(curve: &Curve3, range: Interval, mut params: Vec<f64>, tol: f64) -> Vec<f64> {
    params.retain(|t| range.contains_with(*t, 0.0));
    params.sort_by(f64::total_cmp);
    let mut out: Vec<f64> = vec![range.min];
    for t in params {
        let last = out.last().copied().unwrap_or(range.min);
        if (curve.point(t) - curve.point(last)).length() > tol {
            out.push(t);
        }
    }
    // The end: replace a last split at the same point, else append.
    let n = out.len();
    let last = out.last().copied().unwrap_or(range.min);
    if n > 1 && (curve.point(range.max) - curve.point(last)).length() <= tol {
        out[n - 1] = range.max;
    } else if last < range.max {
        out.push(range.max);
    }
    out
}

/// Records that the edge of `owner` along which `curve` runs over `part`
/// passes through the face `through`.
fn along_edge(
    inputs: &Inputs<'_>,
    cuts: &mut Cuts,
    owner: FaceRef,
    through: FaceRef,
    curve: &Curve3,
    part: Interval,
) -> Result<(), KernelError> {
    let (p, q) = (curve.point(part.min), curve.point(part.max));
    let m = curve.point(part.mid());
    for (ek, ec, er) in edges_of(inputs, owner) {
        let cm = ec.closest_point(m)?;
        if cm.distance > inputs.tol {
            continue;
        }
        let tm = into_range(&ec, er, cm.t);
        let near = |t: f64| {
            // The ends, on the same turn as the middle.
            let t = into_range(&ec, er, t);
            match ec.domain().period() {
                Some(per) if (t - tm).abs() > per / 2.0 => t - per * ((t - tm) / per).round(),
                _ => t,
            }
        };
        let (tp, tq) = (near(ec.closest_point(p)?.t), near(ec.closest_point(q)?.t));
        let (lo, hi) = (tp.min(tq), tp.max(tq));
        if hi - lo <= 0.0 || !er.contains_with(tm, 1e-9) {
            continue;
        }
        let range = Interval::new(lo.max(er.min), hi.min(er.max));
        cuts.splits
            .entry((owner.0, ek))
            .or_default()
            .extend([range.min, range.max]);
        cuts.imprints.push(Imprint {
            body: owner.0,
            edge: ek,
            range,
            face: through,
        });
        return Ok(());
    }
    Ok(())
}

/// Imprints the edges of `from` on the coincident face `onto`: the
/// stretches of `from`'s edges inside `onto`.
fn imprint_edges(
    inputs: &Inputs<'_>,
    cuts: &mut Cuts,
    from: FaceRef,
    onto: FaceRef,
) -> Result<(), KernelError> {
    let surface = &inputs.face(onto)?.surface;
    let boundary = edges_of(inputs, onto);
    for (ek, ec, er) in edges_of(inputs, from) {
        let mut params = vec![er.min, er.max];
        for (ok, oc, or) in &boundary {
            for hit in curve_curve_3d(&ec, er, oc, *or, inputs.tol)? {
                let (on_this, on_other) = match hit {
                    CurveCurveHit::Point { a, b, .. } => (vec![a], vec![b]),
                    CurveCurveHit::Overlap { a, b, .. } => (vec![a.min, a.max], vec![b.min, b.max]),
                };
                params.extend(on_this.iter().copied());
                cuts.splits.entry((from.0, ek)).or_default().extend(on_this);
                cuts.splits
                    .entry((onto.0, *ok))
                    .or_default()
                    .extend(on_other);
            }
        }
        let params = split_params(&ec, er, params, inputs.tol);
        for w in params.windows(2) {
            let part = Interval::new(w[0], w[1]);
            if part.length() <= 0.0 {
                continue;
            }
            let c = surface.closest_point(ec.point(part.mid()))?;
            if c.distance > inputs.tol {
                continue;
            }
            if inputs.where_in(onto, Point2::new(c.u, c.v))?.0 == Where::Inside {
                cuts.imprints.push(Imprint {
                    body: from.0,
                    edge: ek,
                    range: part,
                    face: onto,
                });
            }
        }
    }
    Ok(())
}

/// The pcurve on face `f` of `curve` over `range` (a curve lying on it).
pub(crate) fn pcurve_in(
    inputs: &Inputs<'_>,
    f: FaceRef,
    curve: &Curve3,
    range: Interval,
) -> Result<Curve2, KernelError> {
    let face = inputs.face(f)?;
    let d = inputs.domain(f)?;
    // Centre the parameters on where the curve actually is in the face.
    let mid = face.surface.closest_point(curve.point(range.mid()))?;
    let centre = into_rect(&face.surface, d, Point2::new(mid.u, mid.v));
    let rect = Rect::new(
        Interval::new(
            centre.x - 0.5 * d.rect.u.length(),
            centre.x + 0.5 * d.rect.u.length(),
        ),
        Interval::new(
            centre.y - 0.5 * d.rect.v.length(),
            centre.y + 0.5 * d.rect.v.length(),
        ),
    );
    Ok(pcurve_on(curve, range, &face.surface, rect, inputs.tol)?.0)
}

/// Splits every input edge at its recorded parameters into pieces, and
/// turns intersection pieces and imprints into pieces too.
#[expect(
    clippy::too_many_lines,
    reason = "one pass over input edges, imprints and cuts"
)]
pub(crate) fn pieces(
    inputs: &Inputs<'_>,
    cuts: &Cuts,
    vertices: &mut Vertices,
) -> Result<Vec<Piece>, KernelError> {
    let pole_splits = pole_splits(inputs, cuts)?;
    let mut out = Vec::new();
    for (b, body) in inputs.bodies.iter().enumerate() {
        for (ek, e) in body.edges() {
            let start = body
                .vertex(e.start)
                .ok_or(KernelError::UnknownKey(EntityKey::Vertex(e.start)))?;
            let end = body
                .vertex(e.end)
                .ok_or(KernelError::UnknownKey(EntityKey::Vertex(e.end)))?;
            let mut params: Vec<f64> = cuts
                .splits
                .get(&(b, ek))
                .map(|v| {
                    v.iter()
                        .copied()
                        .filter(|t| e.range.contains_with(*t, 0.0))
                        .collect()
                })
                .unwrap_or_default();
            params.push(e.range.min);
            params.push(e.range.max);
            params.sort_by(f64::total_cmp);
            if e.degenerate {
                // A pole is one point in space but a side of the domain:
                // intersection pieces ending at it split that side.
                params = vec![e.range.min, e.range.max];
                if let Some(extra) = pole_splits.get(&(b, ek)) {
                    params.extend(
                        extra
                            .iter()
                            .copied()
                            .filter(|t| *t > e.range.min && *t < e.range.max),
                    );
                }
                params.sort_by(f64::total_cmp);
                params.dedup_by(|x, y| (*x - *y).abs() <= 1e-9 * e.range.length());
            } else {
                // Splits within the tolerance of each other or of an end
                // are one point.
                params.dedup_by(|x, y| {
                    (e.curve.point(*x) - e.curve.point(*y)).length() <= inputs.tol
                });
                if let Some(last) = params.last_mut() {
                    *last = e.range.max;
                }
                if let Some(first) = params.first_mut() {
                    *first = e.range.min;
                }
                // A closed edge's end may have been merged into its start.
                if params.len() == 1 {
                    params.push(e.range.max);
                }
            }
            let count = u32::try_from(params.len() - 1).unwrap_or(u32::MAX);
            let coedges: Vec<_> = body.edge_coedges(ek);
            for (i, w) in params.windows(2).enumerate() {
                let s = if i == 0 {
                    vertices.at(start.point, Some(&start.name))
                } else {
                    vertices.at(e.curve.point(w[0]), None)
                };
                let t = if i + 2 == params.len() {
                    vertices.at(end.point, Some(&end.name))
                } else {
                    vertices.at(e.curve.point(w[1]), None)
                };
                let mut uses = Vec::new();
                for &ck in &coedges {
                    let c = body
                        .coedge(ck)
                        .ok_or(KernelError::UnknownKey(EntityKey::Coedge(ck)))?;
                    let face = body
                        .coedge_face(ck)
                        .ok_or(KernelError::UnknownKey(EntityKey::Coedge(ck)))?;
                    uses.push(Use {
                        face: (b, face),
                        pcurve: c.pcurve.clone(),
                        coedge: Some(ck),
                        along: true,
                        order: w[0],
                    });
                }
                out.push(Piece {
                    curve: e.curve.clone(),
                    range: Interval::new(w[0], w[1]),
                    start: s,
                    end: t,
                    degenerate: e.degenerate,
                    tol: e.tol.max(inputs.tol),
                    origin: Origin::Edge {
                        body: b,
                        edge: ek,
                        name: e.name.clone(),
                        index: u32::try_from(i).unwrap_or(u32::MAX),
                        count,
                    },
                    uses,
                });
            }
        }
    }
    // Imprints: the edge pieces inside each imprint's stretch also run
    // through that face, used both ways there.
    for im in &cuts.imprints {
        let face = im.face;
        let mut extra = Vec::new();
        for (i, p) in out.iter().enumerate() {
            let Origin::Edge { body, edge, .. } = &p.origin else {
                continue;
            };
            if *body != im.body
                || *edge != im.edge
                || p.degenerate
                || !im.range.contains_with(p.range.mid(), 0.0)
            {
                continue;
            }
            if p.uses.iter().any(|u| u.face == face) {
                continue;
            }
            extra.push((i, pcurve_in(inputs, face, &p.curve, p.range)?));
        }
        for (i, pcurve) in extra {
            out[i].uses.push(Use {
                face,
                pcurve,
                coedge: None,
                along: true,
                order: 0.0,
            });
        }
    }
    for c in &cuts.pieces {
        let names = [
            inputs.face(c.faces[0])?.name.clone(),
            inputs.face(c.faces[1])?.name.clone(),
        ];
        let closed =
            (c.curve.point(c.range.min) - c.curve.point(c.range.max)).length() <= inputs.tol;
        let s = vertices.at(c.curve.point(c.range.min), None);
        let t = if closed {
            s
        } else {
            vertices.at(c.curve.point(c.range.max), None)
        };
        out.push(Piece {
            curve: c.curve.clone(),
            range: c.range,
            start: s,
            end: t,
            degenerate: false,
            tol: c.tol,
            origin: Origin::Cut { faces: names },
            uses: vec![
                Use {
                    face: c.faces[0],
                    pcurve: c.pcurves[0].clone(),
                    coedge: None,
                    along: true,
                    order: 0.0,
                },
                Use {
                    face: c.faces[1],
                    pcurve: c.pcurves[1].clone(),
                    coedge: None,
                    along: true,
                    order: 0.0,
                },
            ],
        });
    }
    let mut out = merge_coincident(inputs, out)?;
    snap_ends(inputs, &mut out)?;
    Ok(out)
}

/// Moves the ends of fitted pcurves onto the exact pcurves they meet.
///
/// A cut piece's pcurve starts and ends where its curve's ends invert
/// to, a tolerance away from the vertex in space; near a pole that can be
/// far off in `u`. The input edges' pcurves are exact, so where a fitted
/// pcurve ends at a vertex on one of them, its end control point is moved
/// there (which moves the curve by at most that much, only near the end).
fn snap_ends(inputs: &Inputs<'_>, pieces: &mut [Piece]) -> Result<(), KernelError> {
    // The exact (u, v) of each vertex on each face, from the input edges.
    let mut exact: BTreeMap<(usize, FaceRef), Vec<Point2>> = BTreeMap::new();
    for p in pieces.iter() {
        for u in p.uses.iter().filter(|u| u.coedge.is_some()) {
            for (v, t) in [(p.start, p.range.min), (p.end, p.range.max)] {
                exact
                    .entry((v, u.face))
                    .or_default()
                    .push(u.pcurve.point(t));
            }
        }
    }
    for p in pieces.iter_mut() {
        let (start, end, range) = (p.start, p.end, p.range);
        for u in p.uses.iter_mut().filter(|u| u.coedge.is_none()) {
            let Curve2::Nurbs(n) = &u.pcurve else {
                continue;
            };
            let rect = inputs.domain(u.face)?.rect;
            let limit = SNAP_LIMIT * rect.u.length().hypot(rect.v.length());
            let mut points = n.control_points();
            let last = points.len().saturating_sub(1);
            let mut moved = false;
            for (v, t, i) in [(start, range.min, 0), (end, range.max, last)] {
                let at = n.point(t);
                let target = exact.get(&(v, u.face)).and_then(|list| {
                    list.iter()
                        .copied()
                        .min_by(|a, b| (*a - at).length().total_cmp(&(*b - at).length()))
                });
                if let (Some(q), Some(cp)) = (target, points.get_mut(i))
                    && (q - at).length() <= limit
                {
                    *cp += q - at;
                    moved = true;
                }
            }
            if moved {
                let weighted = points
                    .iter()
                    .zip(n.weights())
                    .map(|(c, w)| Vector3::new(c.x * w, c.y * w, w))
                    .collect();
                let snapped =
                    bevel_geom::NurbsCurve2::new(n.degree(), n.knots().clone(), weighted)?;
                u.pcurve = Curve2::Nurbs(snapped);
            }
        }
    }
    Ok(())
}

/// How far (relative to the face's parameter rectangle) a fitted pcurve's
/// end may be moved onto an exact one.
const SNAP_LIMIT: f64 = 1e-4;

/// Where intersection pieces end on degenerate edges (poles), as
/// parameters of those edges, from the pieces' pcurve ends.
fn pole_splits(
    inputs: &Inputs<'_>,
    cuts: &Cuts,
) -> Result<BTreeMap<(usize, EdgeKey), Vec<f64>>, KernelError> {
    let mut out: BTreeMap<(usize, EdgeKey), Vec<f64>> = BTreeMap::new();
    for c in &cuts.pieces {
        for (f, pc) in c.faces.iter().zip(&c.pcurves) {
            let body = inputs.bodies[f.0];
            for ck in body.face_coedges(f.1) {
                let Some(co) = body.coedge(ck) else { continue };
                let Some(e) = body.edge(co.edge) else {
                    continue;
                };
                if !e.degenerate {
                    continue;
                }
                for t in [c.range.min, c.range.max] {
                    let uv = pc.point(t);
                    let near = co.pcurve.closest_point(uv)?;
                    // On the pole's side of the domain, up to a period.
                    let d = inputs.domain(*f)?;
                    let face = inputs.face(*f)?;
                    let uv = into_rect(&face.surface, d, uv);
                    let near2 = co.pcurve.closest_point(uv)?;
                    let best = if near2.distance < near.distance {
                        near2
                    } else {
                        near
                    };
                    if best.distance <= 1e-6 * d.rect.u.length().hypot(d.rect.v.length()) {
                        out.entry((f.0, co.edge)).or_default().push(best.t);
                    }
                }
            }
        }
    }
    Ok(out)
}

/// Merges pieces that are the same stretch of space (both inputs have an
/// edge there, or an intersection runs along an edge): the first keeps
/// its curve, and the others' uses move onto it with pcurves refitted to
/// its parameter.
fn merge_coincident(inputs: &Inputs<'_>, pieces: Vec<Piece>) -> Result<Vec<Piece>, KernelError> {
    let mut out: Vec<Piece> = Vec::with_capacity(pieces.len());
    for p in pieces {
        let same = out.iter().position(|q| {
            !p.degenerate
                && !q.degenerate
                && ((q.start == p.start && q.end == p.end)
                    || (q.start == p.end && q.end == p.start))
                && (q.midpoint() - p.midpoint()).length() <= 10.0 * inputs.tol
        });
        let Some(j) = same else {
            out.push(p);
            continue;
        };
        let flipped = out[j].start != p.start
            || (out[j].start == out[j].end && {
                // A closed piece: compare the directions at the start.
                let a = out[j]
                    .curve
                    .point(out[j].range.min + 1e-3 * out[j].range.length())
                    - out[j].curve.point(out[j].range.min);
                let b = p.curve.point(p.range.min + 1e-3 * p.range.length())
                    - p.curve.point(p.range.min);
                a.dot(b) < 0.0
            });
        let mut moved = Vec::with_capacity(p.uses.len());
        for u in &p.uses {
            if out[j]
                .uses
                .iter()
                .any(|v| v.face == u.face && v.coedge == u.coedge)
            {
                continue;
            }
            let pcurve = refit(inputs, u, &p, &out[j])?;
            moved.push(Use {
                pcurve,
                along: u.along != flipped,
                ..u.clone()
            });
        }
        let q = &mut out[j];
        q.tol = q.tol.max(p.tol);
        q.uses.extend(moved);
    }
    Ok(out)
}

/// The pcurve of use `u` (of piece `from`) carried over to piece `to`'s
/// curve and parameter, on the same periodic copy of the face's domain.
fn refit(inputs: &Inputs<'_>, u: &Use, from: &Piece, to: &Piece) -> Result<Curve2, KernelError> {
    let face = inputs.face(u.face)?;
    let d = inputs.domain(u.face)?;
    let m = to.curve.point(to.range.mid());
    let t = into_range(&from.curve, from.range, from.curve.closest_point(m)?.t);
    let centre = u.pcurve.point(t);
    let rect = Rect::new(
        Interval::new(
            centre.x - 0.5 * d.rect.u.length(),
            centre.x + 0.5 * d.rect.u.length(),
        ),
        Interval::new(
            centre.y - 0.5 * d.rect.v.length(),
            centre.y + 0.5 * d.rect.v.length(),
        ),
    );
    Ok(pcurve_on(&to.curve, to.range, &face.surface, rect, inputs.tol)?.0)
}
