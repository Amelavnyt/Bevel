// SPDX-License-Identifier: GPL-3.0-or-later

//! The general curve–curve method: subdivision with box culling, then
//! Newton refinement.
//!
//! Pairs of pieces whose boxes (widened by the tolerance) meet are
//! examined:
//!
//! - pieces whose capsules (chord plus deviation from it) are farther apart
//!   than the tolerance are dropped;
//! - flat pieces with disjoint tangent cones meet at most once (the chord
//!   between two common points would be parallel to a tangent of both), so
//!   Newton from their midpoints settles the pair;
//! - flat, nearly parallel pieces whose sampled points all lie within the
//!   tolerance of the other curve are a seed of a tangential touch or an
//!   overlap, left to [`resolve`];
//! - anything else is split (the piece with the larger box), down to boxes
//!   of a small multiple of the tolerance, where Newton runs from the
//!   midpoints.

use std::marker::PhantomData;

use bevel_math::{Aabb3, Interval};

use super::common::{
    CONE_MARGIN, Ctx, CurveTarget, FLAT_ANGLE, Found, MAX_LEVEL, MAX_WORK, Path, Raw, Shape, TINY,
    boxes_meet, curve_residual, diagonal, line_angle, minimize, resolve, scale_of,
    segment_distance, shape_of, splittable, widened,
};
use crate::GeomError;

/// A piece of a curve over a sub-range.
#[derive(Clone, Copy, Debug)]
struct Piece {
    r: Interval,
    bbox: Aabb3,
    shape: Option<Shape>,
    level: u32,
}

impl Piece {
    fn new<P: Path>(path: &P, r: Interval, level: u32) -> Result<Self, GeomError> {
        let (bbox, points) = path.hull(r)?;
        Ok(Self {
            r,
            bbox,
            shape: points.as_deref().and_then(shape_of),
            level,
        })
    }

    fn halves<P: Path>(&self, path: &P) -> Result<[Self; 2], GeomError> {
        if self.level >= MAX_LEVEL {
            return Err(GeomError::NotConverged);
        }
        let (l, r) = self.r.split(self.r.mid());
        Ok([
            Self::new(path, l, self.level + 1)?,
            Self::new(path, r, self.level + 1)?,
        ])
    }
}

/// The pieces of both curves and the raw solutions found so far.
struct Search<'a, P, Q> {
    ctx: Ctx<'a, P, CurveTarget<'a, Q>, 1>,
    b: &'a Q,
    rb: Interval,
    raws: Vec<Raw<1>>,
}

impl<P: Path, Q: Path> Search<'_, P, Q> {
    /// Newton from the midpoints of the pair, kept near it.
    fn leaf(&mut self, x: &Piece, y: &Piece) {
        let bounds = [widened(x.r, self.ctx.ra), widened(y.r, self.rb)];
        self.refine([x.r.mid(), y.r.mid()], &bounds);
    }

    /// Newton from `start` within `bounds`; keeps the result if the curves
    /// meet there.
    fn refine(&mut self, start: [f64; 2], bounds: &[Interval; 2]) {
        let (a, b) = (self.ctx.a, self.b);
        let solved = minimize(|v| curve_residual(a, b, v), start, bounds, self.ctx.scale);
        if let Some((v, d)) = solved
            && d <= self.ctx.tol
        {
            self.raws.push(Raw {
                s: v[0],
                x: [v[1]],
                d,
            });
        }
    }

    /// If every sampled point of `x` lies within the tolerance of the
    /// second curve, records the middle one (and Newton from it) and
    /// returns `true`.
    fn seed(&mut self, x: &Piece, y: &Piece) -> bool {
        let mut t = [y.r.mid()];
        let mut middle = None;
        for (k, f) in [0.0, 0.25, 0.5, 0.75, 1.0].into_iter().enumerate() {
            let s = x.r.lerp(f);
            let Some((xp, d)) = self.ctx.project_local(self.ctx.a.at(s), t) else {
                return false;
            };
            if d > self.ctx.tol {
                return false;
            }
            t = xp;
            if k == 2 {
                middle = Some(Raw { s, x: xp, d });
            }
        }
        if let Some(m) = middle {
            self.raws.push(m);
            let bounds = [self.ctx.ra, self.rb];
            self.refine([m.s, m.x[0]], &bounds);
        }
        true
    }

    /// Settles the pair if it can be settled without splitting.
    fn settled(&mut self, x: &Piece, y: &Piece) -> bool {
        let tol = self.ctx.tol;
        if !boxes_meet(&x.bbox, &y.bbox, tol) {
            return true;
        }
        let (Some(sx), Some(sy)) = (x.shape, y.shape) else {
            return false;
        };
        if segment_distance(sx.p0, sx.p1, sy.p0, sy.p1) - sx.dev - sy.dev > tol {
            return true;
        }
        if sx.half > FLAT_ANGLE || sy.half > FLAT_ANGLE {
            return false;
        }
        if line_angle(sx.axis, sy.axis) > sx.half + sy.half + CONE_MARGIN {
            self.leaf(x, y);
            return true;
        }
        self.seed(x, y)
    }
}

/// Where `a` over `ra` meets `b` over `rb`, by subdivision.
///
/// # Errors
///
/// [`GeomError::NotConverged`] if the subdivision exceeds its depth or
/// work limit, and the errors of evaluating the curves' bounds.
pub(in crate::intersect) fn intersect<P: Path, Q: Path>(
    a: &P,
    ra: Interval,
    b: &Q,
    rb: Interval,
    tol: f64,
) -> Result<Vec<Found<1>>, GeomError> {
    let mut pa = vec![Piece::new(a, ra, 0)?];
    let mut pb = vec![Piece::new(b, rb, 0)?];
    let scale = scale_of(&[pa[0].bbox, pb[0].bbox]);
    let target = CurveTarget {
        path: b,
        bounds: [rb],
    };
    let mut search = Search {
        ctx: Ctx {
            a,
            ra,
            target: &target,
            tol,
            scale,
            dims: PhantomData,
        },
        b,
        rb,
        raws: Vec::new(),
    };
    let mut stack = vec![(0_usize, 0_usize)];
    let mut work = 0_usize;
    while let Some((ia, ib)) = stack.pop() {
        work += 1;
        if work > MAX_WORK {
            return Err(GeomError::NotConverged);
        }
        let (x, y) = (pa[ia], pb[ib]);
        if search.settled(&x, &y) {
            continue;
        }
        let (dx, dy) = (diagonal(&x.bbox), diagonal(&y.bbox));
        let (can_a, can_b) = (splittable(x.r), splittable(y.r));
        if dx.max(dy) <= TINY * tol || !(can_a || can_b) {
            search.leaf(&x, &y);
            continue;
        }
        if can_a && (dx >= dy || !can_b) {
            let halves = x.halves(a)?;
            let first = pa.len();
            pa.extend(halves);
            stack.push((first + 1, ib));
            stack.push((first, ib));
        } else {
            let halves = y.halves(b)?;
            let first = pb.len();
            pb.extend(halves);
            stack.push((ia, first + 1));
            stack.push((ia, first));
        }
    }
    let Search { ctx, raws, .. } = search;
    Ok(resolve(&ctx, raws))
}
