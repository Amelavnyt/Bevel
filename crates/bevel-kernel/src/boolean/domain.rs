// SPDX-License-Identifier: GPL-3.0-or-later

//! A face's domain in `(u, v)`: its loops as closed polygons, for telling
//! whether a parameter point lies inside the face.
//!
//! Each coedge's pcurve is sampled at its edge's discretization (the one
//! the tessellator uses, at the export preset), so the polygons follow the
//! true boundary to well within the classification margins the boolean
//! pipeline uses; points that close to the boundary are reported as on it.

use crate::tessellate::edge::curve_params;
use crate::topology::{Body, EntityKey, FaceKey};
use crate::{KernelError, TessellationOptions};
use bevel_geom::intersect::Rect;
use bevel_geom::{Curve2, CurveGeom};
use bevel_math::predicates::orient2d;
use bevel_math::{Interval, Point2};

/// Where a parameter point lies relative to a face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Where {
    /// Strictly inside, farther than the margin from the boundary.
    Inside,
    /// Outside, farther than the margin from the boundary.
    Outside,
    /// Within the margin of the boundary.
    Boundary,
}

/// A face's loops as closed polygons in `(u, v)`.
#[derive(Clone, Debug)]
pub(crate) struct Domain {
    /// The bounding rectangle of the loops.
    pub rect: Rect,
    /// One closed polygon per loop (the last point repeats the first).
    pub loops: Vec<Vec<Point2>>,
    /// For each polygon segment, the stretch of pcurve it stands for.
    spans: Vec<Vec<Span>>,
    /// The pcurves the spans refer to.
    pcurves: Vec<Curve2>,
}

/// The stretch of pcurve between two polygon points, and how far it
/// strays from their segment.
#[derive(Clone, Copy, Debug)]
struct Span {
    /// The pcurve (an index into [`Domain::pcurves`]), `None` for a
    /// straight segment.
    pcurve: Option<usize>,
    t0: f64,
    t1: f64,
    /// A bound on the distance from the pcurve to the segment.
    dev: f64,
}

impl Domain {
    /// The domain of `face`, sampling pcurves with `options`.
    ///
    /// # Errors
    ///
    /// Broken references as [`KernelError::UnknownKey`], and sampling
    /// failures from the edge discretization.
    pub(crate) fn of(
        body: &Body,
        face: FaceKey,
        options: &TessellationOptions,
    ) -> Result<Self, KernelError> {
        let f = body
            .face(face)
            .ok_or(KernelError::UnknownKey(EntityKey::Face(face)))?;
        let mut loops = Vec::with_capacity(f.loops.len());
        let mut spans = Vec::with_capacity(f.loops.len());
        let mut pcurves = Vec::new();
        let (mut lo, mut hi) = (
            Point2::splat(f64::INFINITY),
            Point2::splat(f64::NEG_INFINITY),
        );
        for &lk in &f.loops {
            let l = body
                .loop_(lk)
                .ok_or(KernelError::UnknownKey(EntityKey::Loop(lk)))?;
            let mut poly: Vec<Point2> = Vec::new();
            // The pcurve and parameter of each polygon point.
            let mut at: Vec<(Option<usize>, f64)> = Vec::new();
            for &ck in &l.coedges {
                let c = body
                    .coedge(ck)
                    .ok_or(KernelError::UnknownKey(EntityKey::Coedge(ck)))?;
                let e = body
                    .edge(c.edge)
                    .ok_or(KernelError::UnknownKey(EntityKey::Edge(c.edge)))?;
                let mut params = if e.degenerate {
                    // A degenerate edge is a point in space but a segment of
                    // the domain: its pcurve is a straight line.
                    vec![e.range.min, e.range.max]
                } else {
                    curve_params(
                        &e.curve,
                        e.range,
                        options,
                        usize::MAX,
                        EntityKey::Edge(c.edge),
                    )?
                };
                if c.reversed {
                    params.reverse();
                }
                let index = (!e.degenerate).then_some(pcurves.len());
                if index.is_some() {
                    pcurves.push(c.pcurve.clone());
                }
                for (i, t) in params.into_iter().enumerate() {
                    let p = c.pcurve.point(t);
                    if poly.last().is_none_or(|q| (*q - p).length() > 0.0) {
                        poly.push(p);
                        // A coedge's first point ends the segment from the
                        // previous coedge, which is straight.
                        at.push((if i == 0 { None } else { index }, t));
                    }
                }
            }
            if let Some(&first) = poly.first()
                && poly.last().is_some_and(|q| (*q - first).length() > 0.0)
            {
                poly.push(first);
                at.push((None, 0.0));
            }
            let span = |k: usize| -> Span {
                let ((_, t0), (pc, t1)) = (at[k], at[k + 1]);
                let Some(i) = pc else {
                    return Span {
                        pcurve: None,
                        t0,
                        t1,
                        dev: 0.0,
                    };
                };
                let (a, b) = (poly[k], poly[k + 1]);
                let dev = [0.25, 0.5, 0.75]
                    .into_iter()
                    .map(|s| segment_distance(pcurves[i].point(t0 + (t1 - t0) * s), a, b))
                    .fold(0.0, f64::max);
                Span {
                    pcurve: Some(i),
                    t0,
                    t1,
                    dev: DEV_MARGIN * dev,
                }
            };
            spans.push((0..poly.len().saturating_sub(1)).map(span).collect());
            for p in &poly {
                lo = lo.min(*p);
                hi = hi.max(*p);
            }
            loops.push(poly);
        }
        if !(lo.x < hi.x && lo.y < hi.y) {
            return Err(KernelError::InvalidInput {
                what: "face",
                reason: "a face's loops do not enclose an area in parameter space",
            });
        }
        Ok(Self {
            rect: Rect::new(Interval::new(lo.x, hi.x), Interval::new(lo.y, hi.y)),
            loops,
            spans,
            pcurves,
        })
    }

    /// Where `p` lies, treating points within `margin` (in parameter
    /// units) of the boundary as on it.
    pub(crate) fn classify(&self, p: Point2, margin: f64) -> Where {
        let mut winding = 0i32;
        for (poly, spans) in self.loops.iter().zip(&self.spans) {
            for (w, span) in poly.windows(2).zip(spans) {
                let seg = Seg {
                    a: w[0],
                    b: w[1],
                    span: *span,
                };
                if !self.cross(p, margin, seg, 0, &mut winding) {
                    return Where::Boundary;
                }
            }
        }
        // Outer loops run counter-clockwise and holes clockwise on a face
        // that is not reversed, the other way round on a reversed one:
        // inside means a non-zero winding either way.
        if winding == 0 {
            Where::Outside
        } else {
            Where::Inside
        }
    }
}

/// A polygon segment from `a` to `b` and the pcurve stretch it stands
/// for.
#[derive(Clone, Copy)]
struct Seg {
    a: Point2,
    b: Point2,
    span: Span,
}

impl Domain {
    /// Adds the crossings of `seg`'s pcurve stretch with the ray from `p`
    /// to `winding`; `false` if `p` is within `margin` of it. Where `p`
    /// lies closer to the segment than the stretch can stray, the stretch
    /// is halved until the pieces follow it closely enough, so the answer
    /// is exact near the boundary too.
    fn cross(&self, p: Point2, margin: f64, seg: Seg, depth: u32, winding: &mut i32) -> bool {
        let Seg { a, b, span } = seg;
        let d = segment_distance(p, a, b);
        if let Some(pc) = span.pcurve.and_then(|i| self.pcurves.get(i))
            && d <= span.dev + margin
            && span.dev > 0.25 * margin
            && depth < MAX_DEPTH
        {
            let tm = 0.5 * (span.t0 + span.t1);
            let m = pc.point(tm);
            let half = |t0: f64, t1: f64, a: Point2, b: Point2| {
                let dev = segment_distance(pc.point(0.5 * (t0 + t1)), a, b);
                Seg {
                    a,
                    b,
                    span: Span {
                        pcurve: span.pcurve,
                        t0,
                        t1,
                        dev: DEV_MARGIN * dev,
                    },
                }
            };
            return self.cross(p, margin, half(span.t0, tm, a, m), depth + 1, winding)
                && self.cross(p, margin, half(tm, span.t1, m, b), depth + 1, winding);
        }
        if d <= margin {
            return false;
        }
        // Winding number by upward and downward crossings, with exact
        // orientation tests.
        if a.y <= p.y {
            if b.y > p.y && orient2d(a, b, p) > 0.0 {
                *winding += 1;
            }
        } else if b.y <= p.y && orient2d(a, b, p) < 0.0 {
            *winding -= 1;
        }
        true
    }
}

/// A safety factor on a span's measured deviation from its segment.
const DEV_MARGIN: f64 = 2.0;

/// The deepest halving of a span near a point.
const MAX_DEPTH: u32 = 48;

/// The distance from `p` to the segment `a b`.
pub(crate) fn segment_distance(p: Point2, a: Point2, b: Point2) -> f64 {
    let d = b - a;
    let len2 = d.length_squared();
    let t = if len2 > 0.0 {
        ((p - a).dot(d) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (a + d * t - p).length()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "tests")]

    use super::*;
    use crate::naming::OpTag;
    use crate::primitives::{BoxParams, CylinderParams, SphereParams, box_, cylinder, sphere};
    use crate::{OpContext, TessellationOptions};
    use bevel_math::{Frame3, Vector3};
    use std::f64::consts::PI;

    fn options(body: &Body) -> TessellationOptions {
        TessellationOptions::export(&body.bounds())
    }

    #[test]
    fn box_faces_are_rectangles() {
        let mut cx = OpContext::new();
        let params = BoxParams {
            frame: Frame3::WORLD,
            size: Vector3::new(10.0, 20.0, 30.0),
        };
        let body = &box_(&params, OpTag(1), &mut cx).unwrap().bodies[0];
        for (k, _) in body.faces() {
            let d = Domain::of(body, k, &options(body)).unwrap();
            let c = Point2::new(d.rect.u.mid(), d.rect.v.mid());
            assert_eq!(d.classify(c, 1e-9), Where::Inside);
            let corner = Point2::new(d.rect.u.min, d.rect.v.min);
            assert_eq!(d.classify(corner, 1e-9), Where::Boundary);
            let out = Point2::new(d.rect.u.max + 1.0, d.rect.v.mid());
            assert_eq!(d.classify(out, 1e-9), Where::Outside);
        }
    }

    #[test]
    fn caps_are_discs_and_periodic_faces_full_rectangles() {
        let mut cx = OpContext::new();
        let params = CylinderParams {
            frame: Frame3::WORLD,
            radius: 5.0,
            height: 10.0,
        };
        let body = &cylinder(&params, OpTag(1), &mut cx).unwrap().bodies[0];
        for (k, f) in body.faces() {
            let d = Domain::of(body, k, &options(body)).unwrap();
            if let bevel_geom::Surface::Plane(_) = f.surface {
                // The disc: the plane's own coordinates, so the corner
                // of its bounding square is outside.
                let c = Point2::new(d.rect.u.mid(), d.rect.v.mid());
                assert_eq!(d.classify(c, 1e-6), Where::Inside);
                let corner = Point2::new(d.rect.u.min + 0.5, d.rect.v.min + 0.5);
                assert_eq!(d.classify(corner, 1e-6), Where::Outside);
            } else {
                assert!((d.rect.u.length() - 2.0 * PI).abs() < 1e-9);
                assert_eq!(d.classify(Point2::new(1.0, 5.0), 1e-6), Where::Inside);
            }
        }
        let params = SphereParams {
            frame: Frame3::WORLD,
            radius: 3.0,
        };
        let body = &sphere(&params, OpTag(2), &mut cx).unwrap().bodies[0];
        let (k, _) = body.faces().next().unwrap();
        let d = Domain::of(body, k, &options(body)).unwrap();
        assert_eq!(d.classify(Point2::new(2.0, 0.3), 1e-6), Where::Inside);
        assert_eq!(d.classify(Point2::new(2.0, PI), 1e-6), Where::Outside);
    }
}
