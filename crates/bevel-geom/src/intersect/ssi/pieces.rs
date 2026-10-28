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
use crate::intersect::{CurveCurveHit, curve_curve_3d};
use crate::{Curve2, Curve3, CurveGeom, Ellipse2, GeomError, Line2, Surface, SurfaceGeom, fit};
use bevel_math::{Interval, Point2, Point3};

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
    // Break the curve where it crosses the rectangles' sides (exactly, by
    // intersecting it with their boundary curves) and keep the stretches
    // whose middles lie inside both. A side that is a periodic seam or a
    // pole also breaks it, so each part's parameters run without a jump.
    let mut breaks = vec![range.min, range.max];
    breaks.extend(crossings(curve, range, pair.a, &pair.ra, tol)?);
    breaks.extend(crossings(curve, range, pair.b, &pair.rb, tol)?);
    breaks.retain(|t| range.contains_with(*t, 0.0));
    breaks.sort_by(f64::total_cmp);
    breaks.dedup_by(|x, y| (*x - *y).abs() <= 1e-12 * range.length().max(1.0));
    let mut parts: Vec<Interval> = Vec::new();
    for w in breaks.windows(2) {
        let part = Interval::new(w[0], w[1]);
        if part.length() <= 0.0 || pair.fresh(curve, part.mid()).is_none() {
            continue;
        }
        // A touch of a side is not a break: join the parts when the
        // parameters run on across it.
        if let Some(last) = parts.last_mut()
            && (last.max - part.min).abs() <= f64::EPSILON * range.length().max(1.0)
            && pair.continuous_at(curve, *last, part)
        {
            last.max = part.max;
            continue;
        }
        parts.push(part);
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
            && pair.continuous_at(curve, last, first)
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

/// Steps in which [`Pair::continuous_at`] follows a part's parameters
/// from its middle to its end.
const FOLLOW_STEPS: u32 = 16;

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

    /// Whether the parameters run on without a jump from part `left` into
    /// the next part `right` at their common end.
    fn continuous_at(&self, curve: &Curve3, left: Interval, right: Interval) -> bool {
        let (Some(l), Some(r)) = (
            self.fresh(curve, left.mid()),
            self.fresh(curve, right.mid()),
        ) else {
            return false;
        };
        // Follow each part's parameters from its middle out to the common
        // end: inverting the end point alone cannot tell the two sides of a
        // periodic seam apart.
        let follow = |from: f64, mut uv: Uv, to: f64| -> Option<Uv> {
            for i in 1..=FOLLOW_STEPS {
                let t = from + (to - from) * f64::from(i) / f64::from(FOLLOW_STEPS);
                let p = curve.point(t);
                uv = (
                    invert(self.a, &self.ra, p, Some(uv.0))?,
                    invert(self.b, &self.rb, p, Some(uv.1))?,
                );
            }
            Some(uv)
        };
        let (Some(x), Some(y)) = (
            follow(left.mid(), l, left.max),
            follow(right.mid(), r, right.min),
        ) else {
            return false;
        };
        let size = |r: &Rect| r.u.length().hypot(r.v.length());
        (x.0 - y.0).length() <= 1e-6 * size(&self.ra)
            && (x.1 - y.1).length() <= 1e-6 * size(&self.rb)
            && !singular(self.a, x.0)
            && !singular(self.b, x.1)
    }
}

/// Where `curve` over `range` crosses the sides of `r` on `s`: the
/// parameters where it meets each side's boundary curve, and where it
/// passes through a side that is a single point (a pole).
fn crossings(
    curve: &Curve3,
    range: Interval,
    s: &Surface,
    r: &Rect,
    tol: f64,
) -> Result<Vec<f64>, GeomError> {
    let mut out = Vec::new();
    // Each side: its boundary curve over its parameter range, or `None`
    // where the side is a single point (a sphere's pole), with that point.
    let side = |u: Option<f64>, v: Option<f64>| -> (Option<(Curve3, Interval)>, Point3) {
        match (u, v) {
            (Some(u), _) => (
                s.isocurve_u(u).ok().map(|c| (c, r.v)),
                s.point(u, r.v.mid()),
            ),
            (_, Some(v)) => (
                s.isocurve_v(v).ok().map(|c| (c, r.u)),
                s.point(r.u.mid(), v),
            ),
            _ => (None, Point3::ZERO),
        }
    };
    let sides = [
        side(Some(r.u.min), None),
        side(Some(r.u.max), None),
        side(None, Some(r.v.min)),
        side(None, Some(r.v.max)),
    ];
    for (iso, mid) in &sides {
        let point = match iso {
            Some((c, along)) => {
                let (p0, pm, p1) = (c.point(along.min), c.point(along.mid()), c.point(along.max));
                ((p0 - pm).length() <= tol && (p1 - pm).length() <= tol).then_some(pm)
            }
            None => Some(*mid),
        };
        if let Some(pole) = point {
            // A pole: the curve breaks where it passes through it.
            let c = curve.closest_point(pole)?;
            if c.distance <= tol {
                out.push(into_range(curve, range, c.t));
            }
            continue;
        }
        if let Some((c, along)) = iso {
            for hit in curve_curve_3d(curve, range, c, *along, tol)? {
                match hit {
                    CurveCurveHit::Point { a, .. } => out.push(a),
                    CurveCurveHit::Overlap { a, .. } => out.extend([a.min, a.max]),
                }
            }
        }
    }
    Ok(out)
}

/// `t` shifted by whole periods of a periodic `curve` into `range` when a
/// copy lies there.
fn into_range(curve: &Curve3, range: Interval, t: f64) -> f64 {
    match curve.domain().period() {
        Some(per) if per > 0.0 => {
            let base = t + ((range.mid() - t) / per).round() * per;
            [base - per, base, base + per]
                .into_iter()
                .find(|c| range.contains_with(*c, 1e-12))
                .unwrap_or(base)
        }
        _ => t,
    }
}

/// The pcurve of `curve` over `part` on `s`, and its error bound.
pub(super) fn pcurve(
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
        settle_poles(s, &mut uv);
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

/// Whether `uv` is a singular point of `s`'s parametrization (a pole).
fn singular(s: &Surface, uv: Point2) -> bool {
    let (su, sv) = s.partials(uv.x, uv.y);
    let scale = su.length().max(sv.length()).max(f64::MIN_POSITIVE);
    su.length() <= 1e-9 * scale || sv.length() <= 1e-9 * scale
}

/// At an end on a singular point of the parametrization (a pole, where
/// one parameter can take any value), takes that parameter from the
/// neighbouring sample: the limit along the curve, not the arbitrary value
/// inversion gives there.
fn settle_poles(s: &Surface, uv: &mut [Point2]) {
    let n = uv.len();
    if n < 2 {
        return;
    }
    for (end, next) in [(0, 1), (n - 1, n - 2)] {
        let (su, sv) = s.partials(uv[end].x, uv[end].y);
        let scale = su.length().max(sv.length()).max(f64::MIN_POSITIVE);
        if su.length() <= 1e-9 * scale {
            uv[end].x = uv[next].x;
        }
        if sv.length() <= 1e-9 * scale {
            uv[end].y = uv[next].y;
        }
    }
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
