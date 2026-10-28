// SPDX-License-Identifier: GPL-3.0-or-later

//! Whether a point lies inside, outside or on a solid.
//!
//! On the boundary: within the tolerance of a face (its surface's nearest
//! point lies in the face's domain). Otherwise a ray is cast and the faces
//! it crosses are counted; an odd count means inside. A ray that grazes a
//! surface, runs along one, or passes near a face's boundary proves nothing
//! and is replaced by the next of a fixed list of directions; two clean
//! rays must agree, and if they do not, the majority of five decides.

use super::domain::{Domain, Where};
use crate::topology::{Body, FaceKey};
use crate::{KernelError, TessellationOptions};
use bevel_geom::intersect::{CurveSurfaceHit, curve_surface};
use bevel_geom::{Curve3, Line3, Surface, SurfaceGeom};
use bevel_math::{Aabb3, Interval, Point2, Point3, Vector3};

/// Where a point lies relative to a solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Containment {
    /// Inside the material.
    In,
    /// Outside.
    Out,
    /// On the boundary, within the tolerance.
    On,
}

/// Ray directions, fixed so results are deterministic; none is parallel to
/// a coordinate axis or plane, where model faces tend to lie.
const DIRECTIONS: [[f64; 3]; 9] = [
    [0.536_212, 0.274_651, 0.798_115],
    [-0.312_791, 0.845_103, 0.433_127],
    [0.701_419, -0.590_337, 0.399_872],
    [-0.461_538, -0.369_231, 0.806_590],
    [0.183_421, 0.917_331, -0.353_204],
    [-0.871_402, 0.142_675, -0.469_396],
    [0.391_116, -0.214_597, -0.894_967],
    [-0.120_745, -0.981_034, 0.151_872],
    [0.667_983, 0.583_105, -0.462_416],
];

/// A solid prepared for point classification.
pub(crate) struct Solid<'b> {
    body: &'b Body,
    faces: Vec<(FaceKey, Domain)>,
    bounds: Aabb3,
    tol: f64,
}

/// A ray's verdict.
enum Ray {
    /// It crossed this many face interiors.
    Count(usize),
    /// It grazed, ran along a surface or passed too near a boundary.
    Unclear,
}

impl<'b> Solid<'b> {
    /// Prepares `body`, with face domains sampled for `options`.
    ///
    /// # Errors
    ///
    /// Failures building the face domains.
    pub(crate) fn new(
        body: &'b Body,
        options: &TessellationOptions,
        tol: f64,
    ) -> Result<Self, KernelError> {
        let faces = body
            .faces()
            .map(|(k, _)| Domain::of(body, k, options).map(|d| (k, d)))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            body,
            faces,
            bounds: body.bounds(),
            tol,
        })
    }

    /// The face domains, by face.
    pub(crate) fn domain(&self, face: FaceKey) -> Option<&Domain> {
        self.faces.iter().find(|(k, _)| *k == face).map(|(_, d)| d)
    }

    /// The margin, in parameter units, within which a point counts as on
    /// a face's boundary: ten tolerances over the parametric speed, floored
    /// at a speed of 1 mm per unit.
    pub(crate) fn margin(&self, surface: &Surface, uv: Point2) -> f64 {
        let (su, sv) = surface.partials(uv.x, uv.y);
        10.0 * self.tol / su.length().min(sv.length()).max(1.0)
    }

    /// Whether `p` lies on a face of the solid; the face and its parameters
    /// if so.
    pub(crate) fn on_face(&self, p: Point3) -> Result<Option<(FaceKey, Point2)>, KernelError> {
        for (k, d) in &self.faces {
            let Some(face) = self.body.face(*k) else {
                continue;
            };
            let c = face.surface.closest_point(p)?;
            if c.distance > self.tol {
                continue;
            }
            let uv = into_rect(&face.surface, d, Point2::new(c.u, c.v));
            if d.classify(uv, self.margin(&face.surface, uv)) != Where::Outside {
                return Ok(Some((*k, uv)));
            }
        }
        Ok(None)
    }

    /// Where `p` lies.
    ///
    /// # Errors
    ///
    /// Geometry failures, or [`KernelError::InvalidInput`] if no ray gives
    /// a clear answer.
    pub(crate) fn classify(&self, p: Point3) -> Result<Containment, KernelError> {
        if self.on_face(p)?.is_some() {
            return Ok(Containment::On);
        }
        let mut votes = Vec::new();
        for dir in DIRECTIONS {
            if let Ray::Count(n) = self.ray(p, Vector3::from_array(dir).normalize())? {
                votes.push(n % 2 == 1);
                if votes.len() == 2 && votes[0] == votes[1] {
                    break;
                }
                if votes.len() == 5 {
                    break;
                }
            }
        }
        let inside = votes.iter().filter(|v| **v).count();
        match votes.len() {
            0 => Err(KernelError::InvalidInput {
                what: "point classification",
                reason: "no ray gave a clear answer",
            }),
            n => Ok(if 2 * inside > n {
                Containment::In
            } else {
                Containment::Out
            }),
        }
    }

    fn ray(&self, p: Point3, dir: Vector3) -> Result<Ray, KernelError> {
        let reach =
            (self.bounds.max - self.bounds.min).length() + (p - self.bounds.center()).length();
        let line = Curve3::Line(Line3::new(p, dir)?);
        let range = Interval::new(0.0, 2.0 * reach + 1.0);
        let mut count = 0;
        for (k, d) in &self.faces {
            let Some(face) = self.body.face(*k) else {
                continue;
            };
            for hit in curve_surface(&line, range, &face.surface, d.rect, self.tol)? {
                let CurveSurfaceHit::Point { t, u, v, .. } = hit else {
                    return Ok(Ray::Unclear);
                };
                let uv = Point2::new(u, v);
                match d.classify(uv, self.margin(&face.surface, uv)) {
                    Where::Outside => {}
                    Where::Boundary => return Ok(Ray::Unclear),
                    Where::Inside => {
                        let grazing = face
                            .surface
                            .normal(u, v)
                            .is_none_or(|n| n.dot(dir).abs() < 1e-6);
                        if grazing || t <= self.tol {
                            return Ok(Ray::Unclear);
                        }
                        count += 1;
                    }
                }
            }
        }
        Ok(Ray::Count(count))
    }
}

/// `uv` shifted by whole periods of `s` towards the centre of `d`'s
/// rectangle.
pub(crate) fn into_rect(s: &Surface, d: &Domain, uv: Point2) -> Point2 {
    let shift = |x: f64, target: f64, period: Option<f64>| match period {
        Some(per) if per > 0.0 => x + ((target - x) / per).round() * per,
        _ => x,
    };
    Point2::new(
        shift(uv.x, d.rect.u.mid(), s.period_u()),
        shift(uv.y, d.rect.v.mid(), s.period_v()),
    )
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "tests")]

    use super::*;
    use crate::OpContext;
    use crate::naming::OpTag;
    use crate::primitives::{BoxParams, SphereParams, TorusParams, box_, sphere, torus};
    use bevel_math::Frame3;

    fn solid_of(body: &Body) -> Solid<'_> {
        Solid::new(body, &TessellationOptions::export(&body.bounds()), 1e-6).unwrap()
    }

    #[test]
    fn points_around_a_box() {
        let mut cx = OpContext::new();
        let params = BoxParams {
            frame: Frame3::WORLD,
            size: Vector3::new(10.0, 20.0, 30.0),
        };
        let body = &box_(&params, OpTag(1), &mut cx).unwrap().bodies[0];
        let s = solid_of(body);
        assert_eq!(
            s.classify(Point3::new(5.0, 10.0, 15.0)).unwrap(),
            Containment::In
        );
        assert_eq!(
            s.classify(Point3::new(15.0, 10.0, 15.0)).unwrap(),
            Containment::Out
        );
        assert_eq!(
            s.classify(Point3::new(10.0, 10.0, 15.0)).unwrap(),
            Containment::On
        );
        assert_eq!(
            s.classify(Point3::new(10.0, 20.0, 30.0)).unwrap(),
            Containment::On,
            "a corner"
        );
        // On the plane of a face but beside it.
        assert_eq!(
            s.classify(Point3::new(10.0, 25.0, 15.0)).unwrap(),
            Containment::Out
        );
    }

    #[test]
    fn points_around_curved_solids() {
        let mut cx = OpContext::new();
        let params = SphereParams {
            frame: Frame3::WORLD,
            radius: 5.0,
        };
        let body = &sphere(&params, OpTag(1), &mut cx).unwrap().bodies[0];
        let s = solid_of(body);
        assert_eq!(
            s.classify(Point3::new(1.0, 2.0, -3.0)).unwrap(),
            Containment::In
        );
        assert_eq!(
            s.classify(Point3::new(3.0, 3.0, 3.0)).unwrap(),
            Containment::Out
        );
        assert_eq!(
            s.classify(Point3::new(0.0, 0.0, 5.0)).unwrap(),
            Containment::On,
            "the pole"
        );
        let params = TorusParams {
            frame: Frame3::WORLD,
            major_radius: 10.0,
            minor_radius: 3.0,
        };
        let body = &torus(&params, OpTag(2), &mut cx).unwrap().bodies[0];
        let s = solid_of(body);
        assert_eq!(
            s.classify(Point3::ZERO).unwrap(),
            Containment::Out,
            "the hole"
        );
        assert_eq!(
            s.classify(Point3::new(0.0, -10.0, 1.0)).unwrap(),
            Containment::In
        );
        assert_eq!(
            s.classify(Point3::new(0.0, 13.0, 0.0)).unwrap(),
            Containment::On
        );
    }
}
