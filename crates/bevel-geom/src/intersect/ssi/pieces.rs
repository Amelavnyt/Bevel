// SPDX-License-Identifier: GPL-3.0-or-later

//! Turning exact intersection curves into results: clipping a curve to the
//! parts whose parameters lie in both rectangles, and giving each part a
//! pcurve on both surfaces that shares the curve's parameter.
//!
//! A pcurve is exact where possible: a line or conic on a plane maps to a
//! line or conic in the plane's coordinates, and wherever the surface
//! parameters change linearly along the curve (a ruling or a coaxial
//! circle of a cylinder, a latitude of a sphere) the pcurve is a line.
//! Otherwise it is a cubic through inverted samples at the curve's own
//! parameters, refined until it maps within tolerance.

use super::super::{Contact, Rect, SsiCurve, SsiPoint};
use crate::{Curve2, Curve3, CurveGeom, Ellipse2, GeomError, Line2, Surface, SurfaceGeom, fit};
use bevel_math::{Interval, Point2, Point3};
use std::f64::consts::PI;

/// Samples used to find where a curve enters and leaves the rectangles.
const CLIP_SAMPLES: usize = 512;
/// The most samples a fitted pcurve may use.
const MAX_PCURVE_SAMPLES: usize = 4097;

/// `p`'s parameters on `s`, shifted by whole periods to be nearest `near`
/// (continuity along a curve), or into `r` when there is nothing to follow.
pub(super) fn invert(s: &Surface, r: &Rect, p: Point3, near: Option<Point2>) -> Option<Point2> {
    let c = s.closest_point(p).ok()?;
    let fit_period = |x: f64, period: Option<f64>, target: f64| match period {
        Some(per) if per > 0.0 => x + ((target - x) / per).round() * per,
        _ => x,
    };
    let (tu, tv) = match near {
        Some(n) => (n.x, n.y),
        None => (r.u.mid(), r.v.mid()),
    };
    Some(Point2::new(
        fit_period(c.u, s.period_u(), tu),
        fit_period(c.v, s.period_v(), tv),
    ))
}

/// The touching points that lie in both rectangles.
pub(super) fn points(
    points: &[Point3],
    a: &Surface,
    ra: &Rect,
    b: &Surface,
    rb: &Rect,
) -> Vec<SsiPoint> {
    points
        .iter()
        .filter_map(|&p| {
            let pa = invert(a, ra, p, None)?;
            let pb = invert(b, rb, p, None)?;
            (ra.contains_with(pa.x, pa.y, 1e-9) && rb.contains_with(pb.x, pb.y, 1e-9)).then_some(
                SsiPoint {
                    point: p,
                    a: pa,
                    b: pb,
                },
            )
        })
        .collect()
}

/// Two surface patches.
pub(super) struct Pair<'s> {
    pub a: &'s Surface,
    pub ra: Rect,
    pub b: &'s Surface,
    pub rb: Rect,
}

/// The parts of `curve` over `range` whose parameters lie in both
/// rectangles, each with its pcurves.
pub(super) fn clip(
    curve: &Curve3,
    range: Interval,
    contact: Contact,
    pair: &Pair<'_>,
    tol: f64,
) -> Result<Vec<SsiCurve>, GeomError> {
    // Walk the samples, following the parameters continuously within a
    // part. A part ends where the continued parameters leave a rectangle,
    // at its edge or at a periodic seam (where the next part starts at
    // once), or jump, through a pole or an apex.
    let n = CLIP_SAMPLES;
    let ts: Vec<f64> = (0..=n)
        .map(|i| range.min + range.length() * i as f64 / n as f64)
        .collect();
    let mut parts = Vec::new();
    let mut open: Option<(f64, Uv)> = None;
    let mut prev_t = range.min;
    for (i, &t) in ts.iter().enumerate() {
        if let Some((start, near)) = open {
            if let Some(uv) = pair.follow(curve, t, near) {
                open = Some((start, uv));
                prev_t = t;
                continue;
            }
            let end = pair.leave(curve, prev_t, t, near);
            parts.push(Interval::new(start, end));
            open = pair.fresh(curve, t).map(|uv| (end, uv));
        } else if let Some(uv) = pair.fresh(curve, t) {
            let start = if i == 0 {
                t
            } else {
                pair.enter(curve, prev_t, t)
            };
            open = Some((start, uv));
        }
        prev_t = t;
    }
    if let Some((start, _)) = open {
        parts.push(Interval::new(start, range.max));
    }
    // Drop slivers (a closed curve's ends coincide, so measure to the middle).
    parts.retain(|p| {
        let (s, m, e) = (curve.point(p.min), curve.point(p.mid()), curve.point(p.max));
        p.length() > 0.0 && ((m - s).length() > tol || (e - s).length() > tol)
    });
    // A closed curve whose run wraps through the range ends is one part.
    if parts.len() >= 2 && is_closed(curve, range, tol) {
        let (first, last) = (parts[0], parts[parts.len() - 1]);
        if (first.min - range.min).abs() <= f64::EPSILON * range.length().max(1.0)
            && (last.max - range.max).abs() <= f64::EPSILON * range.length().max(1.0)
        {
            let period = range.length();
            parts.pop();
            parts[0] = Interval::new(last.min - period, first.max);
        }
    }
    parts
        .into_iter()
        .map(|part| {
            let (pa, ea) = pcurve(curve, part, pair.a, &pair.ra, tol)?;
            let (pb, eb) = pcurve(curve, part, pair.b, &pair.rb, tol)?;
            Ok(SsiCurve {
                curve: curve.clone(),
                range: part,
                pcurve_a: pa,
                pcurve_b: pb,
                tolerance: ea.max(eb).max(1e-9 * tol),
                contact,
            })
        })
        .collect()
}

fn is_closed(curve: &Curve3, range: Interval, tol: f64) -> bool {
    matches!(curve, Curve3::Circle(_) | Curve3::Ellipse(_))
        && (curve.point(range.min) - curve.point(range.max)).length() <= tol
}

/// Parameters on both surfaces.
type Uv = (Point2, Point2);

impl Pair<'_> {
    fn inside(&self, uv: Uv) -> bool {
        self.ra.contains_with(uv.0.x, uv.0.y, 1e-9) && self.rb.contains_with(uv.1.x, uv.1.y, 1e-9)
    }

    /// The parameters at `t`, shifted into the rectangles, if they fit.
    fn fresh(&self, curve: &Curve3, t: f64) -> Option<Uv> {
        let p = curve.point(t);
        let uv = (
            invert(self.a, &self.ra, p, None)?,
            invert(self.b, &self.rb, p, None)?,
        );
        self.inside(uv).then_some(uv)
    }

    /// The parameters at `t` continued from `near`, if they stay in the
    /// rectangles without jumping.
    fn follow(&self, curve: &Curve3, t: f64, near: Uv) -> Option<Uv> {
        let p = curve.point(t);
        let uv = (
            invert(self.a, &self.ra, p, Some(near.0))?,
            invert(self.b, &self.rb, p, Some(near.1))?,
        );
        let jump = (uv.0 - near.0).length() > PI / 2.0 || (uv.1 - near.1).length() > PI / 2.0;
        (self.inside(uv) && !jump).then_some(uv)
    }

    /// Where a part that is inside at `inn` (with parameters `near`) ends
    /// before `out`, by bisection.
    fn leave(&self, curve: &Curve3, inn: f64, out: f64, near: Uv) -> f64 {
        let (mut inn, mut out, mut near) = (inn, out, near);
        for _ in 0..60 {
            let mid = 0.5 * (inn + out);
            match self.follow(curve, mid, near) {
                Some(uv) => {
                    inn = mid;
                    near = uv;
                }
                None => out = mid,
            }
        }
        inn
    }

    /// Where the curve enters the rectangles between `out` and `inn`, by
    /// bisection.
    fn enter(&self, curve: &Curve3, out: f64, inn: f64) -> f64 {
        let (mut out, mut inn) = (out, inn);
        for _ in 0..60 {
            let mid = 0.5 * (out + inn);
            if self.fresh(curve, mid).is_some() {
                inn = mid;
            } else {
                out = mid;
            }
        }
        inn
    }
}

/// The pcurve of `curve` over `part` on `s`, and its error bound.
fn pcurve(
    curve: &Curve3,
    part: Interval,
    s: &Surface,
    r: &Rect,
    tol: f64,
) -> Result<(Curve2, f64), GeomError> {
    if let Some(exact) = plane_pcurve(curve, s) {
        return Ok((exact, 0.0));
    }
    // Inverted parameters at evenly spaced curve parameters, following
    // periods from the middle of the part.
    let sample = |count: usize| -> Option<(Vec<f64>, Vec<Point2>)> {
        let ts: Vec<f64> = (0..count)
            .map(|i| part.min + part.length() * i as f64 / (count - 1) as f64)
            .collect();
        // Anchor in the middle, where the parameters are clearly inside
        // the rectangle, then follow them out to both ends.
        let mid = count / 2;
        let mut uv = vec![Point2::ZERO; count];
        let anchor = invert(s, r, curve.point(*ts.get(mid)?), None)?;
        uv[mid] = anchor;
        let mut near = anchor;
        for i in mid + 1..count {
            near = invert(s, r, curve.point(ts[i]), Some(near))?;
            uv[i] = near;
        }
        near = anchor;
        for i in (0..mid).rev() {
            near = invert(s, r, curve.point(ts[i]), Some(near))?;
            uv[i] = near;
        }
        Some((ts, uv))
    };
    let error = |pc: &Curve2, ts: &[f64]| -> f64 {
        ts.windows(2)
            .map(|w| {
                let t = 0.5 * (w[0] + w[1]);
                let q = pc.point(t);
                (s.point(q.x, q.y) - curve.point(t)).length()
            })
            .fold(0.0, f64::max)
    };
    // A line when the parameters change linearly.
    let (ts, uv) = sample(9).ok_or(GeomError::NotConverged)?;
    if let (Some(&p0), Some(&p1)) = (uv.first(), uv.last()) {
        let dir = (p1 - p0) / part.length();
        if dir.length() > 0.0 {
            let speed = dir.length();
            let line = Line2::new(p0 - dir * part.min, dir)?;
            // Line2 runs at unit speed: rescale the parameter through a
            // straight NURBS instead when the speed is not one.
            let line = if (speed - 1.0).abs() <= 1e-12 {
                Curve2::Line(line)
            } else {
                Curve2::Nurbs(fit::interpolate_at(&[p0, p1], &[part.min, part.max], 1)?)
            };
            let err = error(&line, &ts);
            if err <= 1e-3 * tol {
                return Ok((line, err));
            }
        }
    }
    // A cubic through the samples, with samples added where a span's
    // midpoint maps too far from the curve (near a pole the parameters
    // change fast, elsewhere slowly).
    let (mut ts, mut uv) = sample(17).ok_or(GeomError::NotConverged)?;
    while ts.len() <= MAX_PCURVE_SAMPLES {
        let nurbs = Curve2::Nurbs(fit::interpolate_at(&uv, &ts, 3)?);
        let mut worst = 0.0_f64;
        let mut bad = Vec::new();
        for (i, w) in ts.windows(2).enumerate() {
            let t = 0.5 * (w[0] + w[1]);
            let q = nurbs.point(t);
            let err = (s.point(q.x, q.y) - curve.point(t)).length();
            worst = worst.max(err);
            if err > 0.5 * tol {
                bad.push(i);
            }
        }
        if bad.is_empty() {
            return Ok((nurbs, worst));
        }
        for &i in bad.iter().rev() {
            let t = 0.5 * (ts[i] + ts[i + 1]);
            let q = invert(s, r, curve.point(t), Some(0.5 * (uv[i] + uv[i + 1])))
                .ok_or(GeomError::NotConverged)?;
            ts.insert(i + 1, t);
            uv.insert(i + 1, q);
        }
    }
    Err(GeomError::ToleranceNotMet {
        error: f64::INFINITY,
    })
}

/// The exact pcurve of a line or conic lying in a plane: the same curve in
/// the plane's coordinates, with the same parameter.
fn plane_pcurve(curve: &Curve3, s: &Surface) -> Option<Curve2> {
    let Surface::Plane(plane) = s else {
        return None;
    };
    let f = plane.frame();
    let map = |p: Point3| Point2::new((p - f.origin).dot(f.x), (p - f.origin).dot(f.y));
    let map_dir = |d: bevel_math::Vector3| Point2::new(d.dot(f.x), d.dot(f.y));
    match curve {
        Curve3::Line(l) => {
            let d = map_dir(l.direction());
            // A line in the plane keeps its unit direction.
            ((d.length() - 1.0).abs() <= 1e-9)
                .then(|| Line2::new(map(l.origin()), d).ok().map(Curve2::Line))?
        }
        Curve3::Circle(c) => {
            let cf = c.frame();
            let ccw = cf.z.dot(f.z) > 0.0;
            Ellipse2::from_axis(map(cf.origin), map_dir(cf.x), ccw, c.radius(), c.radius())
                .ok()
                .map(Curve2::Ellipse)
        }
        Curve3::Ellipse(e) => {
            let ef = e.frame();
            let ccw = ef.z.dot(f.z) > 0.0;
            Ellipse2::from_axis(
                map(ef.origin),
                map_dir(ef.x),
                ccw,
                e.major_radius(),
                e.minor_radius(),
            )
            .ok()
            .map(Curve2::Ellipse)
        }
        Curve3::Nurbs(_) => None,
    }
}
