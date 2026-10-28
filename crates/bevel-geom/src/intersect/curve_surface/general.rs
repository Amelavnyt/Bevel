// SPDX-License-Identifier: GPL-3.0-or-later

//! The general curve–surface method: subdivision of the curve's range and
//! the surface's rectangle with box culling, then Newton refinement.
//!
//! As for two curves (see `curve_curve::general`): pairs of a curve piece
//! and a surface patch whose boxes meet are dropped when the curve's
//! capsule misses the patch's slab (the plane through its centre, widened
//! by the control net's deviation from it), settled by Newton when both are
//! flat and the curve's tangent cone stays away from the patch's tangent
//! planes (then they meet at most once), recorded as a seed when the
//! curve's sampled points all lie within the tolerance of the surface, and
//! split otherwise.

use std::f64::consts::FRAC_PI_2;

use std::marker::PhantomData;

use bevel_math::{Aabb3, DVec3, Interval};

use super::super::Rect;
use super::super::curve_curve::common::{
    CONE_MARGIN, Ctx, FLAT_ANGLE, Found, MAX_LEVEL, MAX_WORK, Path, Raw, Shape, SurfaceTarget,
    TINY, angle, boxes_meet, diagonal, line_angle, minimize, resolve, scale_of, shape_of,
    splittable, surface_residual, widened,
};
use crate::{GeomError, Surface, SurfaceGeom};

/// A curve piece over a sub-range.
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

/// The plane through a patch's centre with its normal there, how far the
/// control net strays from it, and the half-angle of a cone about the
/// normal holding the patch's normals (estimated from samples).
#[derive(Clone, Copy, Debug)]
struct Slab {
    center: DVec3,
    normal: DVec3,
    dev: f64,
    half: f64,
}

/// A surface patch over a sub-rectangle.
#[derive(Clone, Copy, Debug)]
struct Patch {
    u: Interval,
    v: Interval,
    bbox: Aabb3,
    slab: Option<Slab>,
    level: u32,
}

/// The box of a patch and, where it converts to a NURBS, its control net.
fn hull(s: &Surface, u: Interval, v: Interval) -> Result<(Aabb3, Option<Vec<DVec3>>), GeomError> {
    if let Surface::Nurbs(n) = s {
        return Ok(match SurfaceGeom::to_nurbs(n, u, v) {
            Ok(p) => (p.control_bounds(), Some(p.control_points())),
            Err(_) => (SurfaceGeom::bounds(n, u, v)?, None),
        });
    }
    let bounds = SurfaceGeom::bounds(s, u, v)?;
    let net = SurfaceGeom::to_nurbs(s, u, v)
        .ok()
        .map(|p| p.control_points());
    Ok((bounds, net))
}

/// The [`Slab`] of a patch with the control net `net`; `None` where a
/// normal is missing.
fn slab_of(s: &Surface, u: Interval, v: Interval, net: &[DVec3]) -> Option<Slab> {
    let (um, vm) = (u.mid(), v.mid());
    let center = SurfaceGeom::point(s, um, vm);
    let normal = SurfaceGeom::normal(s, um, vm)?;
    let mut half = 0.0_f64;
    for fu in [0.0, 0.5, 1.0] {
        for fv in [0.0, 0.5, 1.0] {
            let n = SurfaceGeom::normal(s, u.lerp(fu), v.lerp(fv))?;
            half = half.max(angle(normal, n));
        }
    }
    let dev = net
        .iter()
        .map(|&p| (p - center).dot(normal).abs())
        .fold(0.0, f64::max);
    dev.is_finite().then_some(Slab {
        center,
        normal,
        dev,
        half: 1.5 * half + 0.01,
    })
}

impl Patch {
    fn new(s: &Surface, u: Interval, v: Interval, level: u32) -> Result<Self, GeomError> {
        let (bbox, net) = hull(s, u, v)?;
        let slab = net.and_then(|net| slab_of(s, u, v, &net));
        Ok(Self {
            u,
            v,
            bbox,
            slab,
            level,
        })
    }

    /// The patch split across its longer direction (in space), or `None`
    /// if neither direction can be split.
    fn halves(&self, s: &Surface) -> Result<Option<[Self; 2]>, GeomError> {
        if self.level >= MAX_LEVEL {
            return Err(GeomError::NotConverged);
        }
        let (can_u, can_v) = (splittable(self.u), splittable(self.v));
        if !(can_u || can_v) {
            return Ok(None);
        }
        let p = |u: f64, v: f64| SurfaceGeom::point(s, u, v);
        let (um, vm) = (self.u.mid(), self.v.mid());
        let along_u =
            (p(self.u.min, vm) - p(um, vm)).length() + (p(um, vm) - p(self.u.max, vm)).length();
        let along_v =
            (p(um, self.v.min) - p(um, vm)).length() + (p(um, vm) - p(um, self.v.max)).length();
        let level = self.level + 1;
        Ok(Some(if can_u && (along_u >= along_v || !can_v) {
            let (l, r) = self.u.split(um);
            [
                Self::new(s, l, self.v, level)?,
                Self::new(s, r, self.v, level)?,
            ]
        } else {
            let (l, r) = self.v.split(vm);
            [
                Self::new(s, self.u, l, level)?,
                Self::new(s, self.u, r, level)?,
            ]
        }))
    }
}

/// The pieces, patches and raw solutions of one intersection.
struct Search<'a, P> {
    ctx: Ctx<'a, P, SurfaceTarget<'a>, 2>,
    target: &'a SurfaceTarget<'a>,
    rs: Rect,
    raws: Vec<Raw<2>>,
}

impl<P: Path> Search<'_, P> {
    /// Newton from the middles of the pair, kept near it.
    fn leaf(&mut self, x: &Piece, y: &Patch) {
        let bounds = [
            widened(x.r, self.ctx.ra),
            widened(y.u, self.rs.u),
            widened(y.v, self.rs.v),
        ];
        self.refine([x.r.mid(), y.u.mid(), y.v.mid()], &bounds);
    }

    /// Newton from `start` within `bounds`; keeps the result if the curve
    /// meets the surface there.
    fn refine(&mut self, start: [f64; 3], bounds: &[Interval; 3]) {
        let (c, s) = (self.ctx.a, self.target);
        let solved = minimize(|x| surface_residual(c, s, x), start, bounds, self.ctx.scale);
        if let Some((x, d)) = solved
            && d <= self.ctx.tol
        {
            self.raws.push(Raw {
                s: x[0],
                x: [x[1], x[2]],
                d,
            });
        }
    }

    /// If every sampled point of the curve piece lies within the tolerance
    /// of the surface, records the middle one (and Newton from it) and
    /// returns `true`.
    fn seed(&mut self, x: &Piece, y: &Patch) -> bool {
        let mut uv = [y.u.mid(), y.v.mid()];
        let mut middle = None;
        for (k, f) in [0.0, 0.25, 0.5, 0.75, 1.0].into_iter().enumerate() {
            let t = x.r.lerp(f);
            let Some((xp, d)) = self.ctx.project_local(self.ctx.a.at(t), uv) else {
                return false;
            };
            if d > self.ctx.tol {
                return false;
            }
            uv = xp;
            if k == 2 {
                middle = Some(Raw { s: t, x: xp, d });
            }
        }
        if let Some(m) = middle {
            self.raws.push(m);
            let bounds = [self.ctx.ra, self.rs.u, self.rs.v];
            self.refine([m.s, m.x[0], m.x[1]], &bounds);
        }
        true
    }

    /// Settles the pair if it can be settled without splitting.
    fn settled(&mut self, x: &Piece, y: &Patch) -> bool {
        let tol = self.ctx.tol;
        if !boxes_meet(&x.bbox, &y.bbox, tol) {
            return true;
        }
        let (Some(sh), Some(sl)) = (x.shape, y.slab) else {
            return false;
        };
        let h0 = (sh.p0 - sl.center).dot(sl.normal);
        let h1 = (sh.p1 - sl.center).dot(sl.normal);
        if h0.min(h1) - sh.dev > sl.dev + tol || h0.max(h1) + sh.dev < -sl.dev - tol {
            return true;
        }
        if sh.half > FLAT_ANGLE || sl.half > FLAT_ANGLE {
            return false;
        }
        if line_angle(sh.axis, sl.normal) + sh.half + sl.half + CONE_MARGIN < FRAC_PI_2 {
            self.leaf(x, y);
            return true;
        }
        self.seed(x, y)
    }
}

/// Where `c` over `rc` meets `s` over `rs`, by subdivision.
///
/// # Errors
///
/// [`GeomError::NotConverged`] if the subdivision exceeds its depth or
/// work limit, and the errors of evaluating the bounds.
pub(super) fn intersect<P: Path>(
    c: &P,
    rc: Interval,
    s: &Surface,
    rs: Rect,
    tol: f64,
) -> Result<Vec<Found<2>>, GeomError> {
    let mut pieces = vec![Piece::new(c, rc, 0)?];
    let mut patches = vec![Patch::new(s, rs.u, rs.v, 0)?];
    let scale = scale_of(&[pieces[0].bbox, patches[0].bbox]);
    let target = SurfaceTarget {
        surface: s,
        bounds: [rs.u, rs.v],
    };
    let mut search = Search {
        ctx: Ctx {
            a: c,
            ra: rc,
            target: &target,
            tol,
            scale,
            dims: PhantomData,
        },
        target: &target,
        rs,
        raws: Vec::new(),
    };
    let mut stack = vec![(0_usize, 0_usize)];
    let mut work = 0_usize;
    while let Some((ic, is)) = stack.pop() {
        work += 1;
        if work > MAX_WORK {
            return Err(GeomError::NotConverged);
        }
        let (x, y) = (pieces[ic], patches[is]);
        if search.settled(&x, &y) {
            continue;
        }
        let (dx, dy) = (diagonal(&x.bbox), diagonal(&y.bbox));
        let can_c = splittable(x.r);
        let can_s = splittable(y.u) || splittable(y.v);
        if dx.max(dy) <= TINY * tol || !(can_c || can_s) {
            search.leaf(&x, &y);
            continue;
        }
        if can_c && (dx >= dy || !can_s) {
            let halves = x.halves(c)?;
            let first = pieces.len();
            pieces.extend(halves);
            stack.push((first + 1, is));
            stack.push((first, is));
        } else if let Some(halves) = y.halves(s)? {
            let first = patches.len();
            patches.extend(halves);
            stack.push((ic, first + 1));
            stack.push((ic, first));
        } else {
            search.leaf(&x, &y);
        }
    }
    let Search { ctx, raws, .. } = search;
    Ok(resolve(&ctx, raws))
}
