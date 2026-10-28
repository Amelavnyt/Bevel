// SPDX-License-Identifier: GPL-3.0-or-later

//! Conversions to NURBS, hit summaries and shared checks.

use bevel_geom::intersect::{CurveCurveHit, CurveSurfaceHit, Rect};
use bevel_geom::{Curve2, Curve3, CurveGeom, Surface, SurfaceGeom};
use bevel_math::{Interval, Point2, Point3};

/// The default tolerance of the tests.
pub const TOL: f64 = 1e-6;

/// The NURBS form of a plane curve over `r`, with its domain.
pub fn nurbs2(c: &Curve2, r: Interval) -> (Curve2, Interval) {
    let n = c.to_nurbs(r).unwrap();
    let (a, b) = n.domain();
    (Curve2::Nurbs(n), Interval::new(a, b))
}

/// The NURBS form of a space curve over `r`, with its domain.
pub fn nurbs3(c: &Curve3, r: Interval) -> (Curve3, Interval) {
    let n = c.to_nurbs(r).unwrap();
    let (a, b) = n.domain();
    (Curve3::Nurbs(n), Interval::new(a, b))
}

/// The NURBS form of a surface over `r`, with its rectangle.
pub fn nurbs_surface(s: &Surface, r: Rect) -> (Surface, Rect) {
    let n = s.to_nurbs(r.u, r.v).unwrap();
    let ((ua, ub), (va, vb)) = n.domain();
    (
        Surface::Nurbs(n),
        Rect::new(Interval::new(ua, ub), Interval::new(va, vb)),
    )
}

/// A point or the ends of an overlap, in space.
#[derive(Clone, Copy, Debug)]
pub enum Spot<P> {
    Point(P),
    Overlap(P, P),
}

/// The hits of two plane curves in space (overlaps by the ends on `a`).
pub fn spots2(a: &Curve2, hits: &[CurveCurveHit<Point2>]) -> Vec<Spot<Point2>> {
    hits.iter()
        .map(|h| match *h {
            CurveCurveHit::Point { point, .. } => Spot::Point(point),
            CurveCurveHit::Overlap { a: r, .. } => Spot::Overlap(a.point(r.min), a.point(r.max)),
        })
        .collect()
}

/// The hits of two space curves in space.
pub fn spots3(a: &Curve3, hits: &[CurveCurveHit<Point3>]) -> Vec<Spot<Point3>> {
    hits.iter()
        .map(|h| match *h {
            CurveCurveHit::Point { point, .. } => Spot::Point(point),
            CurveCurveHit::Overlap { a: r, .. } => Spot::Overlap(a.point(r.min), a.point(r.max)),
        })
        .collect()
}

/// The hits of a curve and a surface in space.
pub fn spots_cs(c: &Curve3, hits: &[CurveSurfaceHit]) -> Vec<Spot<Point3>> {
    hits.iter()
        .map(|h| match *h {
            CurveSurfaceHit::Point { point, .. } => Spot::Point(point),
            CurveSurfaceHit::Overlap { t } => Spot::Overlap(c.point(t.min), c.point(t.max)),
        })
        .collect()
}

/// Distances for both point types.
pub trait Dist: Copy + std::fmt::Debug {
    fn dist(self, other: Self) -> f64;
}

impl Dist for Point2 {
    fn dist(self, other: Self) -> f64 {
        (self - other).length()
    }
}

impl Dist for Point3 {
    fn dist(self, other: Self) -> f64 {
        (self - other).length()
    }
}

/// Asserts that two lists of hits name the same places within `slack`, in
/// any order (an overlap may run either way).
pub fn assert_same_spots<P: Dist>(got: &[Spot<P>], want: &[Spot<P>], slack: f64) {
    assert_eq!(
        got.len(),
        want.len(),
        "hit counts differ:\n got  {got:?}\n want {want:?}"
    );
    let matches = |g: &Spot<P>, w: &Spot<P>| match (g, w) {
        (Spot::Point(p), Spot::Point(q)) => p.dist(*q) <= slack,
        (Spot::Overlap(p0, p1), Spot::Overlap(q0, q1)) => {
            (p0.dist(*q0) <= slack && p1.dist(*q1) <= slack)
                || (p0.dist(*q1) <= slack && p1.dist(*q0) <= slack)
        }
        _ => false,
    };
    let mut used = vec![false; want.len()];
    for g in got {
        let k = want
            .iter()
            .enumerate()
            .position(|(i, w)| !used[i] && matches(g, w))
            .unwrap_or_else(|| panic!("{g:?} is not among {want:?}"));
        used[k] = true;
    }
}

/// The points of plane hits.
pub fn points2(hits: &[CurveCurveHit<Point2>]) -> Vec<Point2> {
    hits.iter()
        .filter_map(|h| match *h {
            CurveCurveHit::Point { point, .. } => Some(point),
            CurveCurveHit::Overlap { .. } => None,
        })
        .collect()
}

/// Asserts that every point hit of two plane curves lies on both within
/// `tol`, with its parameters in the ranges, and that hits are sorted.
pub fn check_hits2(
    a: &Curve2,
    ra: Interval,
    b: &Curve2,
    rb: Interval,
    hits: &[CurveCurveHit<Point2>],
    tol: f64,
) {
    let mut last = f64::NEG_INFINITY;
    for h in hits {
        match *h {
            CurveCurveHit::Point { a: s, b: t, point } => {
                assert!(ra.contains(s) && rb.contains(t), "{h:?} outside the ranges");
                assert!((a.point(s) - point).length() <= 1e-12 * (1.0 + point.length()));
                assert!((b.point(t) - point).length() <= tol, "{h:?} is not on b");
                assert!(s >= last, "hits are not sorted");
                last = s;
            }
            CurveCurveHit::Overlap { a: r, b: q, .. } => {
                assert!(ra.contains(r.min) && ra.contains(r.max) && r.min < r.max);
                assert!(rb.contains_with(q.min, 1e-12) && rb.contains_with(q.max, 1e-12));
                assert!(r.min >= last, "hits are not sorted");
                last = r.min;
            }
        }
    }
}

/// Asserts that every point hit of a curve and a surface lies on both
/// within `tol`, with its parameters in the range and the rectangle.
pub fn check_hits_cs(
    c: &Curve3,
    rc: Interval,
    s: &Surface,
    rs: Rect,
    hits: &[CurveSurfaceHit],
    tol: f64,
) {
    for h in hits {
        match *h {
            CurveSurfaceHit::Point { t, u, v, point } => {
                assert!(
                    rc.contains(t) && rs.contains_with(u, v, 0.0),
                    "{h:?} outside"
                );
                assert!((c.point(t) - point).length() <= 1e-12 * (1.0 + point.length()));
                assert!(
                    (s.point(u, v) - point).length() <= tol,
                    "{h:?} is not on the surface"
                );
            }
            CurveSurfaceHit::Overlap { t } => {
                assert!(rc.contains(t.min) && rc.contains(t.max) && t.min < t.max);
                for k in 0..=8 {
                    let p = c.point(t.lerp(f64::from(k) / 8.0));
                    let d = s.closest_point(p).unwrap().distance;
                    assert!(d <= 2.0 * tol, "{h:?} leaves the surface by {d}");
                }
            }
        }
    }
}
