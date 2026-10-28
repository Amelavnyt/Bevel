// SPDX-License-Identifier: GPL-3.0-or-later

//! Edge discretization: the parameters (and 3D points) at which every edge
//! is cut, computed once per edge and shared by every face that uses it.
//!
//! A curve is cut into equal parameter spans and each span is split again
//! until it meets the tolerances (see [`curve_params`]). A degenerate edge
//! (a pole or an apex) has no length to measure, so it takes its count from
//! the surface along its pcurve (see [`degenerate_params`]).

use bevel_geom::{Curve2, Curve3, CurveGeom, Surface, SurfaceGeom};
use bevel_math::{Aabb2, Interval, Point3, Vector3};
use slotmap::SecondaryMap;

use super::TessellationOptions;
use crate::KernelError;
use crate::naming::OpContext;
use crate::topology::{Body, Edge, EdgeKey, EntityKey};

/// Spans a curve that is not a line starts with: enough that no span of a
/// closed curve has a zero chord.
const INITIAL_SPANS: usize = 4;

/// The most parts one test splits a span into; finer splits come from
/// recursion.
const MAX_SPLIT: f64 = 1024.0;

/// The deepest recursion before a span is declared impossible to meet the
/// tolerances with.
const MAX_DEPTH: u32 = 48;

/// A margin on the chordal estimate. The sagitta of a circular arc shrinks
/// a little slower than the square of its angle (`sin(x / n) >= sin(x) /
/// n`), by at most 11 % in the square root for spans up to a quarter turn,
/// so splitting by the plain square root would often need a second round.
const CHORDAL_MARGIN: f64 = 1.15;

/// Relative slack on the angular and length tests, so that a span exactly
/// at the limit (up to rounding) passes.
const SLACK: f64 = 1e-9;

/// Where an edge is cut.
#[derive(Clone, Debug)]
pub(crate) struct EdgeSamples {
    /// The edge parameters, increasing from `range.min` to `range.max`.
    pub params: Vec<f64>,
    /// The points at those parameters: the start and end vertices at the
    /// ends (exactly), the curve in between; every point is the vertex for
    /// a degenerate edge.
    pub points: Vec<Point3>,
}

/// Cuts every edge of `body` (see the [module documentation](self)).
///
/// # Errors
///
/// [`KernelError::PointLimit`] if an edge needs more than `cap` points,
/// [`KernelError::Tessellation`] for an edge whose curve does not
/// converge, [`KernelError::InvalidBody`]-like reference errors as
/// [`KernelError::UnknownKey`], and [`KernelError::Cancelled`].
pub(crate) fn discretize_edges(
    body: &Body,
    options: &TessellationOptions,
    cap: usize,
    cx: &OpContext,
) -> Result<SecondaryMap<EdgeKey, EdgeSamples>, KernelError> {
    // The pcurves of each edge with their surfaces: a span must also be
    // fine enough that its chord in (u, v) maps close to its 3D chord, or
    // the faces' boundary polygons stray from their edges.
    let mut on: SecondaryMap<EdgeKey, Vec<(&Curve2, &Surface)>> = SecondaryMap::new();
    for (_, c) in body.coedges() {
        let face = body
            .loop_(c.loop_)
            .and_then(|l| body.face(l.face))
            .ok_or(KernelError::UnknownKey(EntityKey::Loop(c.loop_)))?;
        if let Some(list) = on.entry(c.edge) {
            list.or_default().push((&c.pcurve, &face.surface));
        }
    }
    let mut out = SecondaryMap::new();
    for (key, edge) in body.edges() {
        cx.check_cancelled()?;
        let entity = EntityKey::Edge(key);
        let params = if edge.degenerate {
            degenerate_params(body, key, edge, options, cap)?
        } else {
            let pcurves = on.get(key).map_or(&[][..], Vec::as_slice);
            cut_curve(&edge.curve, edge.range, pcurves, options, cap, entity)?
        };
        let start = body
            .vertex(edge.start)
            .ok_or(KernelError::UnknownKey(EntityKey::Vertex(edge.start)))?
            .point;
        let end = body
            .vertex(edge.end)
            .ok_or(KernelError::UnknownKey(EntityKey::Vertex(edge.end)))?
            .point;
        let last = params.len().saturating_sub(1);
        let points = params
            .iter()
            .enumerate()
            .map(|(i, &t)| {
                if edge.degenerate || i == 0 {
                    start
                } else if i == last {
                    end
                } else {
                    edge.curve.point(t)
                }
            })
            .collect();
        out.insert(key, EdgeSamples { params, points });
    }
    Ok(out)
}

/// The parameters at which to cut `curve` over `range`, ends included.
///
/// The range is first cut into equal spans (one for a line,
/// [`INITIAL_SPANS`] otherwise). A span `[a, b]` then passes if
///
/// - the curve at `a + (b - a) / 4`, the middle and `a + 3 (b - a) / 4`
///   lies within `chordal` of the chord,
/// - the tangent turns by at most `angular` (the angle between the unit
///   tangents at `a` and the middle plus the one between the middle and
///   `b`), and
/// - the chord is no longer than `max_edge`.
///
/// Otherwise it is cut into `n` equal spans, `n` estimated from the worst
/// ratio (the square root of the deviation ratio, since the deviation of
/// a smooth curve from its chord shrinks with the square of the span),
/// and each of those is checked again. Spans are processed in order, so
/// the result is deterministic. A curve that is straight but not a line
/// (a degree-1 NURBS) still starts with four spans, which then pass.
///
/// # Errors
///
/// [`KernelError::PointLimit`] beyond `cap` points and
/// [`KernelError::Tessellation`] if a span does not pass after
/// [`MAX_DEPTH`] rounds of splitting.
pub(crate) fn curve_params(
    curve: &Curve3,
    range: Interval,
    options: &TessellationOptions,
    cap: usize,
    entity: EntityKey,
) -> Result<Vec<f64>, KernelError> {
    cut_curve(curve, range, &[], options, cap, entity)
}

/// [`curve_params`], with every span also tested on `pcurves`: the
/// points of each pcurve's chord in `(u, v)` (at a quarter, the middle
/// and three quarters), mapped through its surface, must lie within
/// `chordal` of the 3D chord (for a pcurve that is straight in `(u, v)`
/// this is the plain chordal test again), and both the surface normal
/// along the pcurve and the pcurve's own tangent in `(u, v)` must turn by
/// at most `angular` (for the lines and circles bounding primitives'
/// faces the pcurve turns as the curve does). Face refinement cannot
/// split a boundary segment, so a segment failing either test would make
/// it refine without end.
fn cut_curve(
    curve: &Curve3,
    range: Interval,
    pcurves: &[(&Curve2, &Surface)],
    options: &TessellationOptions,
    cap: usize,
    entity: EntityKey,
) -> Result<Vec<f64>, KernelError> {
    if !(range.min.is_finite() && range.max.is_finite() && range.min < range.max) {
        return Err(KernelError::Tessellation {
            entity,
            reason: "its parameter range is not a finite, increasing interval",
        });
    }
    let spans = if matches!(curve, Curve3::Line(_)) {
        1
    } else {
        INITIAL_SPANS
    };
    let mut out = vec![range.min];
    let mut cutter = Cutter {
        curve,
        pcurves,
        options,
        cap,
        entity,
        out: &mut out,
    };
    let mut a = range.min;
    for k in 1..=spans {
        let b = span_end(range.min, range.max, k, spans);
        cutter.cut(a, b, 0)?;
        a = b;
    }
    Ok(out)
}

/// The end of span `k` of `n` equal spans of `[a, b]`, exactly `b` for the
/// last one.
fn span_end(a: f64, b: f64, k: usize, n: usize) -> f64 {
    if k >= n {
        b
    } else {
        a + (b - a) * (k as f64 / n as f64)
    }
}

/// The recursive span splitter of [`curve_params`].
struct Cutter<'a> {
    curve: &'a Curve3,
    pcurves: &'a [(&'a Curve2, &'a Surface)],
    options: &'a TessellationOptions,
    cap: usize,
    entity: EntityKey,
    out: &'a mut Vec<f64>,
}

impl Cutter<'_> {
    /// Appends the cuts in `(a, b]`.
    fn cut(&mut self, a: f64, b: f64, depth: u32) -> Result<(), KernelError> {
        let n = self.parts(a, b);
        if n <= 1 {
            if self.out.len() >= self.cap {
                return Err(KernelError::PointLimit {
                    entity: self.entity,
                    limit: self.cap,
                });
            }
            self.out.push(b);
            return Ok(());
        }
        if depth >= MAX_DEPTH {
            return Err(KernelError::Tessellation {
                entity: self.entity,
                reason: "its curve does not meet the tolerances however finely it is cut",
            });
        }
        let mut start = a;
        for k in 1..=n {
            let end = span_end(a, b, k, n);
            self.cut(start, end, depth + 1)?;
            start = end;
        }
        Ok(())
    }

    /// How many equal parts `[a, b]` needs (1 if it passes).
    fn parts(&self, a: f64, b: f64) -> usize {
        let o = self.options;
        let c = self.curve;
        let (pa, pb) = (c.point(a), c.point(b));
        let chord = pb - pa;
        let mut deviation: f64 = 0.0;
        for s in [0.25, 0.5, 0.75] {
            let p = c.point(a + (b - a) * s);
            deviation = deviation.max(distance_to_segment(p, pa, chord));
            for (pc, surface) in self.pcurves {
                let (qa, qb) = (pc.point(a), pc.point(b));
                let q = qa + (qb - qa) * s;
                let p = surface.point(q.x, q.y);
                deviation = deviation.max(distance_to_segment(p, pa, chord));
            }
        }
        let mid = 0.5 * (a + b);
        let (ta, tm, tb) = (c.tangent(a), c.tangent(mid), c.tangent(b));
        let mut turn = angle_opt(ta, tm) + angle_opt(tm, tb);
        // The faces' normals along the edge turn no more than their
        // triangles allow either.
        for (pc, surface) in self.pcurves {
            let normal = |t: f64| {
                let q = pc.point(t);
                surface.normal(q.x, q.y)
            };
            let (na, nm, nb) = (normal(a), normal(mid), normal(b));
            turn = turn.max(angle_opt(na, nm) + angle_opt(nm, nb));
            // Nor does the pcurve turn more in (u, v), where the faces
            // integrate along it: near a pole it can swing round while
            // the 3D curve barely bends.
            let tangent = |t: f64| pc.tangent(t).map(|d| Vector3::new(d.x, d.y, 0.0));
            let ts = [0.0, 0.25, 0.5, 0.75, 1.0].map(|s| tangent(a + (b - a) * s));
            turn = turn.max(ts.windows(2).map(|w| angle_opt(w[0], w[1])).sum());
        }

        let mut need: f64 = 1.0;
        if deviation > o.chordal {
            need = need.max(CHORDAL_MARGIN * (deviation / o.chordal).sqrt());
        }
        if turn > o.angular * (1.0 + SLACK) {
            need = need.max(turn / o.angular);
        }
        if let Some(max_edge) = o.max_edge {
            let length = chord.length();
            if length > max_edge * (1.0 + SLACK) {
                need = need.max(length / max_edge);
            }
        }
        if !need.is_finite() {
            // A non-finite curve point: split in two and let the depth
            // limit report it.
            return 2;
        }
        if need <= 1.0 {
            1
        } else {
            // At least 2, at most MAX_SPLIT, so the cast is exact.
            count(need.ceil().clamp(2.0, MAX_SPLIT))
        }
    }
}

/// `x` as a count; `x` is a whole number between 1 and the point cap.
#[allow(clippy::cast_sign_loss)]
fn count(x: f64) -> usize {
    x as usize
}

/// The distance from `p` to the segment from `a` along `d`.
fn distance_to_segment(p: Point3, a: Point3, d: Vector3) -> f64 {
    let len2 = d.length_squared();
    let w = p - a;
    if len2 > 0.0 {
        let s = (w.dot(d) / len2).clamp(0.0, 1.0);
        (w - d * s).length()
    } else {
        w.length()
    }
}

/// The angle between two unit vectors, 0 if either is missing.
fn angle_opt(a: Option<Vector3>, b: Option<Vector3>) -> f64 {
    match (a, b) {
        (Some(a), Some(b)) => angle(a, b),
        _ => 0.0,
    }
}

/// The angle between two unit vectors, accurate for small angles too.
pub(crate) fn angle(a: Vector3, b: Vector3) -> f64 {
    a.cross(b).length().atan2(a.dot(b))
}

/// The parameters of a degenerate edge: as many equal spans as the
/// surface needs along the edge's pcurve.
///
/// The edge has no length, but its single pcurve runs along a side of the
/// face's `(u, v)` domain (a sphere's pole, a cone's apex), and the fan of
/// triangles there is best when its spacing matches the rest of the face.
/// The count is that of the isoparametric curve in the same direction
/// through the centre of the face's `(u, v)` box (the equator of a sphere,
/// the middle circle of a cone), discretized like an edge over the
/// pcurve's span; if the surface has no such isocurve, it is the span
/// divided by `angular` (the angle step of a revolved surface).
fn degenerate_params(
    body: &Body,
    key: EdgeKey,
    edge: &Edge,
    options: &TessellationOptions,
    cap: usize,
) -> Result<Vec<f64>, KernelError> {
    let entity = EntityKey::Edge(key);
    let range = edge.range;
    if !(range.min.is_finite() && range.max.is_finite() && range.min < range.max) {
        return Err(KernelError::Tessellation {
            entity,
            reason: "its parameter range is not a finite, increasing interval",
        });
    }
    let coedge_key = body
        .edge_coedges(key)
        .first()
        .copied()
        .ok_or(KernelError::Tessellation {
            entity,
            reason: "the degenerate edge is not used by any face",
        })?;
    let coedge = body
        .coedge(coedge_key)
        .ok_or(KernelError::UnknownKey(EntityKey::Coedge(coedge_key)))?;
    let face_key = body
        .coedge_face(coedge_key)
        .ok_or(KernelError::UnknownKey(EntityKey::Coedge(coedge_key)))?;
    let face = body
        .face(face_key)
        .ok_or(KernelError::UnknownKey(EntityKey::Face(face_key)))?;

    let (a, b) = (
        coedge.pcurve.point(range.min),
        coedge.pcurve.point(range.max),
    );
    let mut uv = Aabb2::EMPTY;
    for c in body.face_coedges(face_key) {
        let Some(co) = body.coedge(c) else { continue };
        let Some(e) = body.edge(co.edge) else {
            continue;
        };
        if let Ok(bb) = co.pcurve.bounds(e.range) {
            uv = uv.union(bb);
        }
    }
    let centre = uv.center();
    let along_u = (b.x - a.x).abs() >= (b.y - a.y).abs();
    let (iso, lo, hi) = if along_u {
        (
            face.surface.isocurve_v(centre.y),
            a.x.min(b.x),
            a.x.max(b.x),
        )
    } else {
        (
            face.surface.isocurve_u(centre.x),
            a.y.min(b.y),
            a.y.max(b.y),
        )
    };
    let span = hi - lo;
    let spans = match iso {
        Ok(curve) if uv.min.is_finite() && uv.max.is_finite() && span > 0.0 => {
            curve_params(&curve, Interval::new(lo, hi), options, cap, entity)?.len() - 1
        }
        _ => {
            let n = (span.abs() / options.angular).ceil();
            if n.is_finite() && n >= 1.0 {
                count(n.min(cap as f64))
            } else {
                1
            }
        }
    }
    .max(1);
    if spans >= cap {
        return Err(KernelError::PointLimit { entity, limit: cap });
    }
    Ok((0..=spans)
        .map(|k| span_end(range.min, range.max, k, spans))
        .collect())
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{PI, TAU};

    use bevel_geom::{Circle3, Line3};
    use bevel_math::Frame3;
    use slotmap::Key;

    use super::*;

    fn options(chordal: f64, angular: f64, max_edge: Option<f64>) -> TessellationOptions {
        TessellationOptions {
            chordal,
            angular,
            max_edge,
        }
    }

    fn circle(r: f64) -> Curve3 {
        Curve3::Circle(Circle3::new(Frame3::WORLD, r).unwrap())
    }

    #[test]
    fn a_line_is_one_segment_unless_too_long() {
        let line = Curve3::Line(Line3::new(Point3::ZERO, Vector3::X).unwrap());
        let e = EntityKey::Edge(EdgeKey::null());
        let r = Interval::new(0.0, 10.0);
        let p = curve_params(&line, r, &options(0.1, 0.1, None), 100, e).unwrap();
        assert_eq!(p, vec![0.0, 10.0]);
        let p = curve_params(&line, r, &options(0.1, 0.1, Some(3.0)), 100, e).unwrap();
        assert_eq!(p.len(), 5);
        assert!(p.windows(2).all(|w| w[1] - w[0] <= 3.0));
    }

    #[test]
    fn a_circle_is_cut_evenly_to_the_tolerances() {
        let e = EntityKey::Edge(EdgeKey::null());
        let r = Interval::new(0.0, TAU);
        // The angular tolerance alone: 2 pi / 0.5 rounds up to 13, but
        // each quarter is cut separately, into 4.
        let p = curve_params(&circle(1.0), r, &options(10.0, 0.5, None), 100, e).unwrap();
        assert_eq!(p.len(), 17);
        // The chordal tolerance alone: the sagitta r (1 - cos(x / 2)) stays
        // within the tolerance and the spans are even.
        let chordal = 1e-3;
        let p = curve_params(&circle(10.0), r, &options(chordal, PI, None), 10_000, e).unwrap();
        let steps: Vec<f64> = p.windows(2).map(|w| w[1] - w[0]).collect();
        for s in &steps {
            assert!(10.0 * (1.0 - (s / 2.0).cos()) <= chordal);
            assert!((s - steps[0]).abs() < 1e-12);
        }
        // Not wastefully fine: one quarter fewer segments would fail.
        let n = steps.len() as f64;
        let coarser = TAU / (n * 0.75);
        assert!(10.0 * (1.0 - (coarser / 2.0).cos()) > chordal);
    }

    #[test]
    fn the_point_cap_is_an_error() {
        let e = EntityKey::Edge(EdgeKey::null());
        let r = Interval::new(0.0, TAU);
        let err = curve_params(&circle(1.0), r, &options(1e-6, 0.01, None), 50, e).unwrap_err();
        assert_eq!(
            err,
            KernelError::PointLimit {
                entity: e,
                limit: 50
            }
        );
    }
}
