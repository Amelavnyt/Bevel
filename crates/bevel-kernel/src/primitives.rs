// SPDX-License-Identifier: GPL-3.0-or-later

//! Primitive solids: box, cylinder, sphere, cone and torus.
//!
//! Each primitive is placed by a [`Frame3`] and built with exact analytic
//! geometry: [`Plane`], [`Cylinder`], [`Cone`], [`Sphere`] and [`Torus`]
//! faces, [`Line3`] and [`Circle3`] edges, and pcurves that are lines in
//! `(u, v)` (or circles on planes). The result is one closed shell with
//! every oriented normal pointing out of the solid, validated before it is
//! returned, and a [`History`] that records every vertex, edge and face as
//! generated from nothing. Every name is
//! `Generated { op: tag, source: Primitive, role }`, with the roles below.
//!
//! # Parameters
//!
//! Lengths must be finite and at least `10 * tolerance` (the context's
//! tolerance, 1e-5 mm by default), so no edge or face collapses within
//! tolerance; the frame must be valid ([`Frame3::is_valid`]) with its
//! origin in the modelling space, and the whole solid must fit inside
//! `±`[`tol::MODEL_EXTENT`] on every axis.
//!
//! # Placement, topology and roles
//!
//! Coordinates below are in the placement frame `(o, x, y, z)`.
//!
//! - **Box** ([`box_`]): the corner is at the frame origin and the box
//!   spans `[0, sx] x [0, sy] x [0, sz]`. Vertex `Vertex(i + 2 j + 4 k)` is
//!   the corner `(i sx, j sy, k sz)` for `i, j, k` in `{0, 1}`. Edge
//!   `Edge(4 a + b1 + 2 b2)` runs along axis `a` (0 = x, 1 = y, 2 = z) in
//!   the positive direction, at the corner bits `b1` and `b2` of the next
//!   two axes in cyclic order (`a + 1` and `a + 2`, modulo 3). Faces:
//!   `Bottom` (`z = 0`), `Top` (`z = sz`), `Front` (`y = 0`), `Back`
//!   (`y = sy`), `Left` (`x = 0`) and `Right` (`x = sx`), each on a plane
//!   whose normal points outwards (no face is reversed), with its `(u, v)`
//!   origin at the face's lowest corner. 8 vertices, 12 edges, 6 faces.
//! - **Cylinder** ([`cylinder`]): radius `r` about the `z` axis, from
//!   `z = 0` to `z = h`. `Side(0)` lies on a [`Cylinder`] with the frame of
//!   the placement; `Bottom` is the plane of the frame, reversed so that it
//!   faces `-z`; `Top` is that plane moved to `z = h`. `Edge(0)` and
//!   `Edge(1)` are the bottom and top circles (counter-clockwise about
//!   `z`); `Seam(0)` is the line at `u = 0` (the `+x` side) from
//!   `Vertex(0)` at `(r, 0, 0)` up to `Vertex(1)` at `(r, 0, h)`. 2
//!   vertices, 3 edges, 3 faces.
//! - **Cone** ([`cone`]): bottom radius `r0` at `z = 0`, top radius `r1`
//!   at `z = h`, unequal, one of them possibly zero (an apex). The side
//!   lies on a [`Cone`] whose frame widens towards the larger radius: the
//!   placement frame if `r0 < r1`, otherwise the frame moved to `z = h`
//!   with `y` and `z` negated. Its `v` runs from the narrow end (`v = 0`)
//!   to the wide end (`v = h`). Roles as for the cylinder (`Bottom`, `Top`,
//!   `Side(0)`, `Edge(0)` bottom circle, `Edge(1)` top circle, `Seam(0)`,
//!   `Vertex(0)` bottom, `Vertex(1)` top), except that at an apex there is
//!   no cap and no circle but a degenerate edge `Apex`, and the vertex
//!   there is the apex itself. 2 vertices, 3 edges (one degenerate at an
//!   apex), 3 faces (2 at an apex). The seam's pcurves are degree-1 NURBS
//!   lines, since the seam's 3D parameter (arc length along the ruling)
//!   runs faster than `v` by `1 / cos(half-angle)`.
//! - **Sphere** ([`sphere`]): radius `r` about the frame origin, poles on
//!   the `z` axis. One face `Side(0)` on a [`Sphere`]; `Seam(0)` is the
//!   half circle at longitude `u = 0` from `Vertex(0)` (the south pole
//!   `-r z`) to `Vertex(1)` (the north pole `+r z`); `Pole(0)` and `Pole(1)`
//!   are the degenerate edges at the poles. 2 vertices, 3 edges (two
//!   degenerate), 1 face.
//! - **Torus** ([`torus`]): major radius `R`, minor radius `r`,
//!   `R - r >= 10 * tolerance`, about the `z` axis, centred at the origin.
//!   One face `Side(0)` on a [`Torus`]; `Seam(0)` is the tube circle at
//!   `u = 0` and `Seam(1)` the outer equator (`v = 0`), both through
//!   `Vertex(0)` at `(R + r, 0, 0)`. 1 vertex, 2 edges, 1 face, genus 1.
//!
//! Every curved face's single loop is the rectangle of its parameter
//! domain, counter-clockwise in `(u, v)`: along `v = v0` (`u` from 0 to
//! `2 pi`), up the seam at `u = 2 pi`, back along `v = v1` and down the
//! seam at `u = 0` (for the torus, `v` from 0 to `2 pi`).

use std::f64::consts::{FRAC_PI_2, TAU};
use std::sync::Arc;

use bevel_geom::{
    Circle2, Circle3, Cone, Curve2, Curve3, Cylinder, KnotVector, Line2, Line3, NurbsCurve2, Plane,
    Sphere, Surface, Torus,
};
use bevel_math::{Frame3, Interval, Point2, Point3, Vector2, Vector3, tol};
use serde::{Deserialize, Serialize};
use slotmap::Key;

use crate::KernelError;
use crate::builder::BodyBuilder;
use crate::naming::{History, OpContext, OpOutput, OpTag, Role, Source, TopoName};
use crate::topology::{BodyKind, EdgeKey, FaceKey, VertexKey};
use crate::validate::within;

/// The smallest length a primitive accepts, as a multiple of the
/// tolerance.
pub const MIN_LENGTH_FACTOR: f64 = 10.0;

/// A safety factor on the rounding error of a cone's radius, which grows
/// like `eps * dr^2 / h` for flat cones (see [`cone`]).
const CONE_ERROR_FACTOR: f64 = 4.0;

/// A box with one corner at the frame origin.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoxParams {
    /// The placement: the box spans `[0, size.x] x [0, size.y] x
    /// [0, size.z]` in this frame.
    pub frame: Frame3,
    /// The edge lengths along the frame's axes.
    pub size: Vector3,
}

/// A cylinder standing on the frame's XY plane.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CylinderParams {
    /// The placement: the axis is the frame's `z` axis, the bottom circle
    /// is centred on the origin.
    pub frame: Frame3,
    /// The radius.
    pub radius: f64,
    /// The height along `z`.
    pub height: f64,
}

/// A sphere centred on the frame origin.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SphereParams {
    /// The placement: the centre is the origin, the poles lie on `z`.
    pub frame: Frame3,
    /// The radius.
    pub radius: f64,
}

/// A cone or frustum standing on the frame's XY plane.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConeParams {
    /// The placement: the axis is the frame's `z` axis, the bottom circle
    /// is centred on the origin.
    pub frame: Frame3,
    /// The radius at `z = 0`; zero puts an apex there.
    pub bottom_radius: f64,
    /// The radius at `z = height`; zero puts an apex there.
    pub top_radius: f64,
    /// The height along `z`.
    pub height: f64,
}

/// A ring torus centred on the frame origin.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TorusParams {
    /// The placement: the axis is the frame's `z` axis.
    pub frame: Frame3,
    /// The distance from the axis to the centre of the tube.
    pub major_radius: f64,
    /// The radius of the tube, less than the major radius.
    pub minor_radius: f64,
}

/// Shared state while building a primitive.
struct Prim {
    b: BodyBuilder,
    tag: OpTag,
    tol: f64,
}

impl Prim {
    fn new(tag: OpTag, cx: &OpContext) -> Result<Self, KernelError> {
        cx.check_cancelled()?;
        Ok(Self {
            b: BodyBuilder::new(),
            tag,
            tol: cx.checked_tolerance()?,
        })
    }

    fn name(&self, role: Role) -> TopoName {
        TopoName::generated(self.tag, Source::Primitive, role)
    }

    /// Checks a length parameter (see the module documentation).
    fn length(&self, name: &'static str, value: f64) -> Result<f64, KernelError> {
        if !value.is_finite() {
            return Err(KernelError::InvalidParameter {
                name,
                value,
                reason: "must be a finite number",
            });
        }
        if value < MIN_LENGTH_FACTOR * self.tol {
            return Err(KernelError::InvalidParameter {
                name,
                value,
                reason: "must be positive and at least 10 times the modelling tolerance",
            });
        }
        if value > 2.0 * tol::MODEL_EXTENT {
            return Err(KernelError::InvalidParameter {
                name,
                value,
                reason: "is larger than the modelling space",
            });
        }
        Ok(value)
    }

    fn vertex(&mut self, point: Point3, role: Role) -> VertexKey {
        let name = self.name(role);
        self.b.add_vertex(point, self.tol, name)
    }

    fn edge(
        &mut self,
        curve: Curve3,
        range: Interval,
        start: VertexKey,
        end: VertexKey,
        role: Role,
    ) -> Result<EdgeKey, KernelError> {
        let name = self.name(role);
        self.b.add_edge(curve, range, start, end, self.tol, name)
    }

    fn face(&mut self, surface: Surface, reversed: bool, role: Role) -> FaceKey {
        let name = self.name(role);
        self.b.add_face(surface, reversed, name)
    }

    /// Checks cancellation and the model space, validates, and wraps the
    /// body with its history.
    fn finish(self, faces: &[FaceKey], cx: &OpContext) -> Result<OpOutput, KernelError> {
        let mut b = self.b;
        b.add_shell(faces, true)?;
        cx.check_cancelled()?;
        let bounds = b.body().bounds();
        let extent = bounds.min.abs().max(bounds.max.abs()).max_element();
        if !within(extent, tol::MODEL_EXTENT) {
            return Err(KernelError::OutsideModelSpace { extent });
        }
        let body = b.finish(BodyKind::Solid)?;
        let history = History::generated(&body)?;
        Ok(OpOutput {
            bodies: vec![Arc::new(body)],
            history,
        })
    }
}

/// Checks a placement frame.
fn check_frame(frame: &Frame3) -> Result<(), KernelError> {
    if frame.is_valid() && tol::in_model_space(frame.origin) {
        Ok(())
    } else {
        Err(KernelError::InvalidFrame)
    }
}

/// `frame` moved by `d` along its `z` axis.
fn lifted(frame: &Frame3, d: f64) -> Frame3 {
    Frame3 {
        origin: frame.origin + frame.z * d,
        ..*frame
    }
}

/// The pcurve `origin + t direction` in `(u, v)`.
fn uv_line(origin: Point2, direction: Vector2) -> Result<Curve2, KernelError> {
    Ok(Curve2::Line(Line2::new(origin, direction)?))
}

/// The pcurve of `circle` on `plane`, with the circle's parameter: the
/// circle's centre and axes expressed in the plane's `(u, v)` coordinates.
fn circle_on_plane(circle: &Circle3, plane: &Plane) -> Result<Curve2, KernelError> {
    let p = plane.frame();
    let c = circle.frame();
    let uv = |v: Vector3| Vector2::new(v.dot(p.x), v.dot(p.y));
    let center = uv(c.origin - p.origin);
    let (x, y) = (uv(c.x), uv(c.y));
    let ccw = x.perp_dot(y) > 0.0;
    Ok(Curve2::Circle(Circle2::from_axis(
        center,
        x,
        ccw,
        circle.radius(),
    )?))
}

/// A box's size in its frame, with its corners and axes.
struct BoxShape {
    frame: Frame3,
    size: [f64; 3],
}

impl BoxShape {
    /// The frame axis `a` (0 = x, 1 = y, 2 = z).
    fn axis(&self, a: usize) -> Vector3 {
        match a {
            0 => self.frame.x,
            1 => self.frame.y,
            _ => self.frame.z,
        }
    }

    /// The corner with the given bits along x, y and z.
    fn corner(&self, bits: [u32; 3]) -> Point3 {
        self.frame.to_world(Point3::new(
            f64::from(bits[0]) * self.size[0],
            f64::from(bits[1]) * self.size[1],
            f64::from(bits[2]) * self.size[2],
        ))
    }
}

/// The index `i + 2 j + 4 k` of the corner with bits `(i, j, k)`.
fn corner_index(bits: [u32; 3]) -> usize {
    (bits[0] + 2 * bits[1] + 4 * bits[2]) as usize
}

/// The index `4 a + b1 + 2 b2` of the edge along axis `a` whose lower
/// corner has the bits `b1`, `b2` on the next two axes (cyclically).
fn box_edge_index(a: usize, bits: [u32; 3]) -> usize {
    4 * a + bits[(a + 1) % 3] as usize + 2 * bits[(a + 2) % 3] as usize
}

/// Builds a box (see the [module documentation](self)).
///
/// # Errors
///
/// [`KernelError::InvalidParameter`] for a size or tolerance out of range,
/// [`KernelError::InvalidFrame`], [`KernelError::OutsideModelSpace`] and
/// [`KernelError::Cancelled`].
pub fn box_(params: &BoxParams, tag: OpTag, cx: &mut OpContext) -> Result<OpOutput, KernelError> {
    let mut p = Prim::new(tag, cx)?;
    check_frame(&params.frame)?;
    let shape = BoxShape {
        frame: params.frame,
        size: [
            p.length("box size x", params.size.x)?,
            p.length("box size y", params.size.y)?,
            p.length("box size z", params.size.z)?,
        ],
    };
    let vertices: [VertexKey; 8] = std::array::from_fn(|n| {
        let n = n as u32;
        let bits = [n & 1, (n >> 1) & 1, (n >> 2) & 1];
        p.vertex(shape.corner(bits), Role::Vertex(n))
    });
    let edges = box_edges(&mut p, &shape, &vertices)?;
    let faces = box_faces(&mut p, &shape, &edges)?;
    p.finish(&faces, cx)
}

/// The twelve edges, each running in the positive direction of its axis.
fn box_edges(
    p: &mut Prim,
    shape: &BoxShape,
    vertices: &[VertexKey; 8],
) -> Result<[EdgeKey; 12], KernelError> {
    let mut edges = [EdgeKey::null(); 12];
    for (m, slot) in (0u32..).zip(edges.iter_mut()) {
        let a = (m / 4) as usize;
        let mut bits = [0u32; 3];
        bits[(a + 1) % 3] = m & 1;
        bits[(a + 2) % 3] = (m >> 1) & 1;
        let start = shape.corner(bits);
        let lo = vertices[corner_index(bits)];
        bits[a] = 1;
        let hi = vertices[corner_index(bits)];
        *slot = p.edge(
            Curve3::Line(Line3::new(start, shape.axis(a))?),
            Interval::new(0.0, shape.size[a]),
            lo,
            hi,
            Role::Edge(m),
        )?;
    }
    Ok(edges)
}

/// The six faces with their loops, in the order Bottom, Top, Front, Back,
/// Left, Right.
fn box_faces(
    p: &mut Prim,
    shape: &BoxShape,
    edges: &[EdgeKey; 12],
) -> Result<Vec<FaceKey>, KernelError> {
    // (normal axis, side, role) in face order.
    let specs = [
        (2, 0, Role::Bottom),
        (2, 1, Role::Top),
        (1, 0, Role::Front),
        (1, 1, Role::Back),
        (0, 0, Role::Left),
        (0, 1, Role::Right),
    ];
    let mut faces = Vec::with_capacity(6);
    let mut loops = Vec::with_capacity(6);
    for (a, side, role) in specs {
        // (u, v) axes with u x v pointing out of the box.
        let (bu, bv) = if side == 1 {
            ((a + 1) % 3, (a + 2) % 3)
        } else {
            ((a + 2) % 3, (a + 1) % 3)
        };
        let mut base = [0u32; 3];
        base[a] = side;
        let (x, y) = (shape.axis(bu), shape.axis(bv));
        let plane = Plane::new(Frame3 {
            origin: shape.corner(base),
            x,
            y,
            z: x.cross(y),
        })?;
        faces.push(p.face(Surface::Plane(plane), false, role));

        let (su, sv) = (shape.size[bu], shape.size[bv]);
        // The edge along axis `e` whose lower corner is at (du, dv).
        let edge = |e: usize, du: u32, dv: u32| {
            let mut bits = base;
            bits[bu] = du;
            bits[bv] = dv;
            edges[box_edge_index(e, bits)]
        };
        // Counter-clockwise in (u, v): (0,0) -> (su,0) -> (su,sv) -> (0,sv).
        loops.push([
            (edge(bu, 0, 0), false, uv_line(Point2::ZERO, Vector2::X)?),
            (
                edge(bv, 1, 0),
                false,
                uv_line(Point2::new(su, 0.0), Vector2::Y)?,
            ),
            (
                edge(bu, 0, 1),
                true,
                uv_line(Point2::new(0.0, sv), Vector2::X)?,
            ),
            (edge(bv, 0, 0), true, uv_line(Point2::ZERO, Vector2::Y)?),
        ]);
    }
    for (&face, coedges) in faces.iter().zip(&loops) {
        p.b.add_loop(face, coedges)?;
    }
    Ok(faces)
}

/// One end of a cylinder or cone side: a circle of positive radius or an
/// apex.
struct End {
    /// The vertex on the seam (the apex itself at an apex).
    vertex: VertexKey,
    /// Its position.
    point: Point3,
    /// The circle edge, or the degenerate edge at an apex.
    edge: EdgeKey,
    /// The circle and its cap plane, absent at an apex.
    cap: Option<(Circle3, Plane)>,
}

/// Makes one end at the origin of `frame` (the side frame moved to that
/// end): the vertex at `u = 0` and the circle edge, or the apex vertex and
/// its degenerate edge for a zero radius.
fn make_end(
    p: &mut Prim,
    frame: Frame3,
    radius: f64,
    vertex_role: Role,
    edge_role: Role,
) -> Result<End, KernelError> {
    let full = Interval::new(0.0, TAU);
    if radius <= 0.0 {
        let vertex = p.vertex(frame.origin, vertex_role);
        let name = p.name(Role::Apex);
        let edge = p.b.add_degenerate_edge(vertex, full, p.tol, name)?;
        return Ok(End {
            vertex,
            point: frame.origin,
            edge,
            cap: None,
        });
    }
    let point = frame.to_world(Point3::new(radius, 0.0, 0.0));
    let vertex = p.vertex(point, vertex_role);
    let circle = Circle3::new(frame, radius)?;
    let edge = p.edge(Curve3::Circle(circle), full, vertex, vertex, edge_role)?;
    Ok(End {
        vertex,
        point,
        edge,
        cap: Some((circle, Plane::new(frame)?)),
    })
}

/// The side of a cylinder or cone between the narrow end (`v = 0` of the
/// side frame `c`) and the wide end (`v = h`).
struct Side {
    c: Frame3,
    surface: Surface,
    narrow_radius: f64,
    wide_radius: f64,
    height: f64,
    /// `true` if the narrow end is the bottom of the placement.
    narrow_is_bottom: bool,
}

/// A pcurve of the seam: the line `u = u0` from `v = 0` to `v = h` over
/// the seam's parameter range `[0, length]`. A [`Line2`] when the seam's
/// parameter is `v` itself (a cylinder), otherwise the degree-1 NURBS with
/// that parametrization (a cone, where `length > h`).
fn seam_pcurve(u0: f64, h: f64, length: f64, cylinder: bool) -> Result<Curve2, KernelError> {
    if cylinder {
        return uv_line(Point2::new(u0, 0.0), Vector2::Y);
    }
    let knots = KnotVector::clamped_from_interior(1, 0.0, length, &[])?;
    let points = [Point2::new(u0, 0.0), Point2::new(u0, h)];
    Ok(Curve2::Nurbs(NurbsCurve2::from_points(
        1, knots, &points, None,
    )?))
}

/// Builds the vertices, edges, faces and loops of a cylinder or cone and
/// returns the faces in order (bottom cap, top cap, side).
fn revolved(p: &mut Prim, s: &Side) -> Result<Vec<FaceKey>, KernelError> {
    let h = s.height;
    let (narrow_frame, wide_frame) = (s.c, lifted(&s.c, h));
    // The ends, bottom first.
    let (narrow, wide) = if s.narrow_is_bottom {
        let n = make_end(
            p,
            narrow_frame,
            s.narrow_radius,
            Role::Vertex(0),
            Role::Edge(0),
        )?;
        let w = make_end(p, wide_frame, s.wide_radius, Role::Vertex(1), Role::Edge(1))?;
        (n, w)
    } else {
        let w = make_end(p, wide_frame, s.wide_radius, Role::Vertex(0), Role::Edge(0))?;
        let n = make_end(
            p,
            narrow_frame,
            s.narrow_radius,
            Role::Vertex(1),
            Role::Edge(1),
        )?;
        (n, w)
    };

    // The seam at u = 0 from the narrow end to the wide end.
    let cylinder = matches!(s.surface, Surface::Cylinder(_));
    let (seam_line, seam_length) = if cylinder {
        (Line3::new(narrow.point, s.c.z)?, h)
    } else {
        (
            Line3::through(narrow.point, wide.point)?,
            narrow.point.distance(wide.point),
        )
    };
    let seam = p.edge(
        Curve3::Line(seam_line),
        Interval::new(0.0, seam_length),
        narrow.vertex,
        wide.vertex,
        Role::Seam(0),
    )?;

    // Faces: bottom cap, top cap, side. Both caps lie on planes of the
    // side frame, whose normal points from the narrow end to the wide end,
    // so the narrow cap is reversed to face outwards.
    let (bottom, top) = if s.narrow_is_bottom {
        ((&narrow, true), (&wide, false))
    } else {
        ((&wide, false), (&narrow, true))
    };
    let mut faces = Vec::with_capacity(3);
    let mut cap_coedges = Vec::with_capacity(2);
    for ((end, reversed), role) in [(bottom, Role::Bottom), (top, Role::Top)] {
        if let Some((circle, plane)) = &end.cap {
            faces.push(p.face(Surface::Plane(*plane), reversed, role));
            cap_coedges.push((end.edge, reversed, circle_on_plane(circle, plane)?));
        }
    }
    let side = p.face(s.surface.clone(), false, Role::Side(0));
    for (&face, coedge) in faces.iter().zip(&cap_coedges) {
        p.b.add_loop(face, std::slice::from_ref(coedge))?;
    }
    // Counter-clockwise in (u, v): along the narrow end, up the seam at
    // u = 2 pi, back along the wide end, down the seam at u = 0.
    p.b.add_loop(
        side,
        &[
            (narrow.edge, false, uv_line(Point2::ZERO, Vector2::X)?),
            (seam, false, seam_pcurve(TAU, h, seam_length, cylinder)?),
            (wide.edge, true, uv_line(Point2::new(0.0, h), Vector2::X)?),
            (seam, true, seam_pcurve(0.0, h, seam_length, cylinder)?),
        ],
    )?;
    faces.push(side);
    Ok(faces)
}

/// Builds a cylinder (see the [module documentation](self)).
///
/// # Errors
///
/// As [`box_`].
pub fn cylinder(
    params: &CylinderParams,
    tag: OpTag,
    cx: &mut OpContext,
) -> Result<OpOutput, KernelError> {
    let mut p = Prim::new(tag, cx)?;
    check_frame(&params.frame)?;
    let r = p.length("cylinder radius", params.radius)?;
    let h = p.length("cylinder height", params.height)?;
    let side = Side {
        c: params.frame,
        surface: Surface::Cylinder(Cylinder::new(params.frame, r)?),
        narrow_radius: r,
        wide_radius: r,
        height: h,
        narrow_is_bottom: true,
    };
    let faces = revolved(&mut p, &side)?;
    p.finish(&faces, cx)
}

/// Builds a cone or frustum (see the [module documentation](self)).
///
/// # Errors
///
/// As [`box_`], with [`KernelError::InvalidParameter`] also for a negative
/// radius, two zero radii, or radii closer than `10 * tolerance` (use a
/// cylinder). A radius below `10 * tolerance` is taken as zero (an apex).
/// A cone so flat that [`Cone`]'s half-angle cannot represent it within a
/// tenth of the tolerance (roughly `4 eps dr^2 / h > tolerance / 10`, with
/// `dr` the difference of the radii) is refused as an invalid height.
pub fn cone(params: &ConeParams, tag: OpTag, cx: &mut OpContext) -> Result<OpOutput, KernelError> {
    let mut p = Prim::new(tag, cx)?;
    check_frame(&params.frame)?;
    // A radius below the smallest length is an apex.
    let radius = |p: &Prim, name, value: f64| {
        if value >= 0.0 && value < MIN_LENGTH_FACTOR * p.tol {
            Ok(0.0)
        } else {
            p.length(name, value)
        }
    };
    let r0 = radius(&p, "cone bottom radius", params.bottom_radius)?;
    let r1 = radius(&p, "cone top radius", params.top_radius)?;
    let h = p.length("cone height", params.height)?;
    if (r1 - r0).abs() < MIN_LENGTH_FACTOR * p.tol {
        return Err(KernelError::InvalidParameter {
            name: "cone top radius",
            value: r1,
            reason: "must differ from the bottom radius by at least 10 times the tolerance \
                     (equal radii make a cylinder; two zero radii nothing)",
        });
    }
    // The cone surface stores its half-angle, and tan(half-angle) loses
    // relative precision near 90 degrees: the radius at the wide end is
    // off by about eps * dr * (dr / h). Refuse cones too flat to stay well
    // within tolerance.
    let dr = (r1 - r0).abs();
    if CONE_ERROR_FACTOR * f64::EPSILON * dr * (dr / h).max(1.0) > 0.1 * p.tol {
        return Err(KernelError::InvalidParameter {
            name: "cone height",
            value: h,
            reason: "is too small for the difference of the radii: a cone this flat \
                     cannot be represented within tolerance",
        });
    }
    let f = params.frame;
    let narrow_is_bottom = r0 < r1;
    let (c, rn, rw) = if narrow_is_bottom {
        (f, r0, r1)
    } else {
        (
            Frame3 {
                origin: f.origin + f.z * h,
                x: f.x,
                y: -f.y,
                z: -f.z,
            },
            r1,
            r0,
        )
    };
    let half_angle = (rw - rn).atan2(h);
    let side = Side {
        c,
        surface: Surface::Cone(Cone::new(c, rn, half_angle)?),
        narrow_radius: rn,
        wide_radius: rw,
        height: h,
        narrow_is_bottom,
    };
    let faces = revolved(&mut p, &side)?;
    p.finish(&faces, cx)
}

/// Builds a sphere (see the [module documentation](self)).
///
/// # Errors
///
/// As [`box_`].
pub fn sphere(
    params: &SphereParams,
    tag: OpTag,
    cx: &mut OpContext,
) -> Result<OpOutput, KernelError> {
    let mut p = Prim::new(tag, cx)?;
    check_frame(&params.frame)?;
    let r = p.length("sphere radius", params.radius)?;
    let f = params.frame;
    let south = p.vertex(f.origin - f.z * r, Role::Vertex(0));
    let north = p.vertex(f.origin + f.z * r, Role::Vertex(1));
    // The meridian at u = 0: angle t from x towards z is the latitude.
    let meridian = Frame3 {
        origin: f.origin,
        x: f.x,
        y: f.z,
        z: f.x.cross(f.z),
    };
    let seam = p.edge(
        Curve3::Circle(Circle3::new(meridian, r)?),
        Interval::new(-FRAC_PI_2, FRAC_PI_2),
        south,
        north,
        Role::Seam(0),
    )?;
    let full = Interval::new(0.0, TAU);
    let name = p.name(Role::Pole(0));
    let south_pole = p.b.add_degenerate_edge(south, full, p.tol, name)?;
    let name = p.name(Role::Pole(1));
    let north_pole = p.b.add_degenerate_edge(north, full, p.tol, name)?;
    let face = p.face(Surface::Sphere(Sphere::new(f, r)?), false, Role::Side(0));
    p.b.add_loop(
        face,
        &[
            (
                south_pole,
                false,
                uv_line(Point2::new(0.0, -FRAC_PI_2), Vector2::X)?,
            ),
            (seam, false, uv_line(Point2::new(TAU, 0.0), Vector2::Y)?),
            (
                north_pole,
                true,
                uv_line(Point2::new(0.0, FRAC_PI_2), Vector2::X)?,
            ),
            (seam, true, uv_line(Point2::ZERO, Vector2::Y)?),
        ],
    )?;
    p.finish(&[face], cx)
}

/// Builds a torus (see the [module documentation](self)).
///
/// # Errors
///
/// As [`box_`], with [`KernelError::InvalidParameter`] also unless the
/// major radius exceeds the minor one by at least `10 * tolerance`.
pub fn torus(
    params: &TorusParams,
    tag: OpTag,
    cx: &mut OpContext,
) -> Result<OpOutput, KernelError> {
    let mut p = Prim::new(tag, cx)?;
    check_frame(&params.frame)?;
    let big = p.length("torus major radius", params.major_radius)?;
    let small = p.length("torus minor radius", params.minor_radius)?;
    if big - small < MIN_LENGTH_FACTOR * p.tol {
        return Err(KernelError::InvalidParameter {
            name: "torus minor radius",
            value: small,
            reason: "must be less than the major radius by at least 10 times the tolerance",
        });
    }
    let f = params.frame;
    let v = p.vertex(
        f.to_world(Point3::new(big + small, 0.0, 0.0)),
        Role::Vertex(0),
    );
    // The tube circle at u = 0: angle t from x towards z is v.
    let meridian = Frame3 {
        origin: f.origin + f.x * big,
        x: f.x,
        y: f.z,
        z: f.x.cross(f.z),
    };
    let full = Interval::new(0.0, TAU);
    let tube = p.edge(
        Curve3::Circle(Circle3::new(meridian, small)?),
        full,
        v,
        v,
        Role::Seam(0),
    )?;
    let equator = p.edge(
        Curve3::Circle(Circle3::new(f, big + small)?),
        full,
        v,
        v,
        Role::Seam(1),
    )?;
    let face = p.face(
        Surface::Torus(Torus::new(f, big, small)?),
        false,
        Role::Side(0),
    );
    p.b.add_loop(
        face,
        &[
            (equator, false, uv_line(Point2::ZERO, Vector2::X)?),
            (tube, false, uv_line(Point2::new(TAU, 0.0), Vector2::Y)?),
            (equator, true, uv_line(Point2::new(0.0, TAU), Vector2::X)?),
            (tube, true, uv_line(Point2::ZERO, Vector2::Y)?),
        ],
    )?;
    p.finish(&[face], cx)
}
