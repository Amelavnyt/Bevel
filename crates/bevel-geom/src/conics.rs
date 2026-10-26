// SPDX-License-Identifier: GPL-3.0-or-later

//! Exact circular, elliptical and general conic arcs as rational quadratic
//! NURBS.
//!
//! # Circles and ellipses
//!
//! [`circle_arc`] is `MakeNurbsCircle`, The NURBS Book A7.1. A sweep `theta`
//! is cut into `n` equal segments of at most 90 degrees (`n = 1` up to 90
//! degrees, 2 up to 180, 3 up to 270, 4 beyond). Each segment is a rational
//! quadratic Bezier arc: its end points lie on the circle, its middle
//! control point is the intersection of the end tangents, at distance
//! `r / cos(dtheta / 2)` from the centre on the bisector, and has weight
//! `cos(dtheta / 2)` (eq. 7.33). The segments meet at double knots, so the
//! curve is `C1` in geometry but only `C0` in its parametrisation there.
//! The middle points are computed in closed form instead of by intersecting
//! the tangent lines as A7.1 does; the result is the same up to rounding.
//!
//! The parameter domain is `[start, end]` (the angles themselves), with the
//! double interior knots at the segment boundaries `start + i dtheta`. The
//! parameter equals the angle at the knots and at the ends; in between it
//! is close to the angle but not equal, since no rational parametrisation of
//! a circle is by arc length. A full circle (a sweep of `2 pi`) ends exactly
//! on its first control point.
//!
//! An ellipse is the image of the unit circle under the affine map
//! `(x, y) -> (a x, b y)`, and NURBS are invariant under affine maps
//! (section 4.4), so [`ellipse_arc`] applies that map to the control points
//! of the unit-circle arc and keeps the weights. Its angles are eccentric
//! angles: the point at angle `t` is `(a cos t, b sin t)`.
//!
//! # General conics
//!
//! [`conic_arc`] is `MakeOpenConic`, A7.3. The conic through `P0` and `P2`
//! with end tangents `T0`, `T2` that passes through `P` is a single
//! rational quadratic Bezier arc whose middle control point `P1` is the
//! intersection of the tangent lines; its weight `w1` follows from `P`
//! (`MakeOneArc`, A7.2, eqs. 7.35 to 7.38). The weight classifies the conic:
//! `w1 < 1` is an ellipse, `w1 = 1` a parabola, `w1 > 1` a hyperbola. An
//! ellipse arc wider than the triangle `P0 P1 P2` suggests (an inner angle
//! at `P1` of at most 60 degrees, or a negative weight, where the arc goes
//! the long way round) is split at its middle (`SplitArc`, eqs. 7.39 to
//! 7.42) into two or four segments with positive weights. When the tangents
//! are parallel the middle control point is at infinity, a homogeneous
//! point `(V, 0)` (the second case of A7.2), and the arc is split in two.
//!
//! The result has the book's parameter domain `[0, 1]`.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use bevel_math::{DVec2, DVec3, Frame3, Point2, Point3, Vector2, Vector3};

use crate::{GeomError, KnotVector, NurbsCurve2, NurbsCurve3};

/// Relative tolerance on angles: sweeps within this of a full turn are a
/// full turn, and of a quarter, half or three-quarter turn need no extra
/// segment.
const ANGLE_TOLERANCE: f64 = 1e-12;

/// Tolerance on the orthonormality of a frame's axes.
const FRAME_TOLERANCE: f64 = 1e-9;

/// Relative tolerance under which vectors count as parallel and points as
/// lying on a line or plane in [`conic_arc`].
const CONIC_TOLERANCE: f64 = 1e-10;

/// An arc of the unit circle as a rational quadratic NURBS in the plane:
/// control points, weights and a clamped knot vector on `[start, end]`.
#[derive(Clone, Debug)]
pub(crate) struct UnitArc {
    /// The knot vector, degree 2.
    pub(crate) knots: KnotVector,
    /// Control points (not weighted); the end points of each segment lie
    /// on the unit circle.
    pub(crate) points: Vec<DVec2>,
    /// Weights: 1 at the segment ends, `cos(dtheta / 2)` in the middle.
    pub(crate) weights: Vec<f64>,
    /// `true` for a full turn, where the last point is a copy of the first.
    pub(crate) full: bool,
}

/// The arc of the unit circle from angle `start` to `end`
/// counter-clockwise, split as in A7.1 (see the module documentation).
///
/// # Errors
///
/// [`GeomError::NonFinite`] for non-finite angles and
/// [`GeomError::InvalidArgument`] unless `0 < end - start <= 2 pi`.
pub(crate) fn unit_arc(start: f64, end: f64) -> Result<UnitArc, GeomError> {
    if !start.is_finite() || !end.is_finite() {
        return Err(GeomError::NonFinite);
    }
    let sweep = end - start;
    if sweep.is_nan() || sweep <= 0.0 {
        return Err(GeomError::InvalidArgument(
            "the end angle must be greater than the start angle",
        ));
    }
    let slack = ANGLE_TOLERANCE * TAU.max(start.abs()).max(end.abs());
    if sweep > TAU + slack {
        return Err(GeomError::InvalidArgument(
            "an arc can sweep at most a full turn",
        ));
    }
    let full = sweep >= TAU - slack;
    let sweep = if full { TAU } else { sweep };
    let segments: usize = if sweep <= FRAC_PI_2 + slack {
        1
    } else if sweep <= PI + slack {
        2
    } else if sweep <= 1.5 * PI + slack {
        3
    } else {
        4
    };
    let dtheta = sweep / segments as f64;
    let w1 = (0.5 * dtheta).cos();
    let on_circle = |a: f64| DVec2::new(a.cos(), a.sin());
    let mut points = Vec::with_capacity(2 * segments + 1);
    let mut weights = Vec::with_capacity(2 * segments + 1);
    let mut interior = Vec::with_capacity(2 * segments);
    let first = on_circle(start);
    points.push(first);
    weights.push(1.0);
    for i in 1..=segments {
        let a0 = start + (i - 1) as f64 * dtheta;
        points.push(on_circle(a0 + 0.5 * dtheta) / w1);
        weights.push(w1);
        let last = i == segments;
        points.push(match (last, full) {
            (true, true) => first,
            (true, false) => on_circle(end),
            _ => on_circle(start + i as f64 * dtheta),
        });
        weights.push(1.0);
        if !last {
            let k = start + i as f64 * dtheta;
            interior.extend([k, k]);
        }
    }
    let knots = KnotVector::clamped_from_interior(2, start, end, &interior)?;
    Ok(UnitArc {
        knots,
        points,
        weights,
        full,
    })
}

/// Checks that the frame is finite, orthonormal and right-handed.
fn check_frame(frame: &Frame3) -> Result<(), GeomError> {
    let Frame3 { origin, x, y, z } = *frame;
    if !(origin.is_finite() && x.is_finite() && y.is_finite() && z.is_finite()) {
        return Err(GeomError::NonFinite);
    }
    let unit = |v: Vector3| (v.length() - 1.0).abs() <= FRAME_TOLERANCE;
    if !(unit(x) && unit(y) && unit(z))
        || x.dot(y).abs() > FRAME_TOLERANCE
        || (x.cross(y) - z).length() > FRAME_TOLERANCE
    {
        return Err(GeomError::InvalidArgument(
            "the frame must be orthonormal and right-handed",
        ));
    }
    Ok(())
}

/// Checks that a radius is finite and positive.
fn check_radius(r: f64) -> Result<(), GeomError> {
    if !r.is_finite() {
        Err(GeomError::NonFinite)
    } else if r <= 0.0 {
        Err(GeomError::InvalidArgument("a radius must be positive"))
    } else {
        Ok(())
    }
}

/// The exact circular arc of radius `radius` in the XY plane of `frame`,
/// centred on its origin, from angle `start_angle` to `end_angle` measured
/// from the frame's X axis counter-clockwise about its Z axis (The NURBS
/// Book, A7.1). A sweep of `2 pi` gives the full circle.
///
/// The domain is `[start_angle, end_angle]`; see the module documentation
/// for the segments and the parametrisation.
///
/// # Errors
///
/// [`GeomError::NonFinite`] for non-finite input,
/// [`GeomError::InvalidArgument`] for a non-positive radius, a frame that is
/// not orthonormal and right-handed, or a sweep `end_angle - start_angle`
/// outside `(0, 2 pi]`.
pub fn circle_arc(
    frame: &Frame3,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<NurbsCurve3, GeomError> {
    ellipse_arc(frame, radius, radius, start_angle, end_angle)
}

/// The exact arc of the ellipse with semi-axis `major_radius` along the X
/// axis of `frame` and `minor_radius` along its Y axis, centred on its
/// origin, from eccentric angle `start_angle` to `end_angle`
/// counter-clockwise about the frame's Z axis: the point at angle `t` is
/// `origin + major_radius cos(t) X + minor_radius sin(t) Y`.
///
/// It is the affine image of the unit-circle arc of [`circle_arc`], with the
/// same knots and weights. Either semi-axis may be the longer one.
///
/// # Errors
///
/// As [`circle_arc`], for either radius.
pub fn ellipse_arc(
    frame: &Frame3,
    major_radius: f64,
    minor_radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<NurbsCurve3, GeomError> {
    check_frame(frame)?;
    check_radius(major_radius)?;
    check_radius(minor_radius)?;
    let arc = unit_arc(start_angle, end_angle)?;
    let points: Vec<Point3> = arc
        .points
        .iter()
        .map(|c| frame.origin + frame.x * (major_radius * c.x) + frame.y * (minor_radius * c.y))
        .collect();
    NurbsCurve3::from_points(2, arc.knots, &points, Some(&arc.weights))
}

/// The exact circular arc of radius `radius` around `center` in the plane,
/// from angle `start_angle` to `end_angle` measured counter-clockwise from
/// the positive x axis. The 2D counterpart of [`circle_arc`].
///
/// # Errors
///
/// [`GeomError::NonFinite`] for non-finite input and
/// [`GeomError::InvalidArgument`] for a non-positive radius or a sweep
/// outside `(0, 2 pi]`.
pub fn circle_arc_2d(
    center: Point2,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<NurbsCurve2, GeomError> {
    ellipse_arc_2d(center, Vector2::X, radius, radius, start_angle, end_angle)
}

/// The exact arc of the ellipse around `center` with semi-axis
/// `major_radius` along `major_axis` (any non-zero direction) and
/// `minor_radius` perpendicular to it, from eccentric angle `start_angle`
/// to `end_angle` counter-clockwise. The 2D counterpart of [`ellipse_arc`].
///
/// # Errors
///
/// As [`circle_arc_2d`], plus [`GeomError::InvalidArgument`] for a zero
/// `major_axis`.
pub fn ellipse_arc_2d(
    center: Point2,
    major_axis: Vector2,
    major_radius: f64,
    minor_radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<NurbsCurve2, GeomError> {
    if !center.is_finite() || !major_axis.is_finite() {
        return Err(GeomError::NonFinite);
    }
    let len = major_axis.length();
    if len <= f64::MIN_POSITIVE {
        return Err(GeomError::InvalidArgument(
            "the major axis must be non-zero",
        ));
    }
    let x = major_axis / len;
    let y = x.perp();
    check_radius(major_radius)?;
    check_radius(minor_radius)?;
    let arc = unit_arc(start_angle, end_angle)?;
    let points: Vec<Point2> = arc
        .points
        .iter()
        .map(|c| center + x * (major_radius * c.x) + y * (minor_radius * c.y))
        .collect();
    NurbsCurve2::from_points(2, arc.knots, &points, Some(&arc.weights))
}

/// Returns `true` if `0 < x < 1`.
fn strictly_between_0_and_1(x: f64) -> bool {
    x > 0.0 && x < 1.0
}

/// `a x b` for 2D vectors.
fn cross2(a: DVec2, b: DVec2) -> f64 {
    a.perp_dot(b)
}

/// A rational quadratic Bezier arc in standard form (end weights 1), in
/// the plane of the conic: end points, the middle control point in
/// homogeneous form `(w P1, w)` (or `(V, 0)` for a point at infinity).
#[derive(Clone, Copy, Debug)]
struct Arc2 {
    p0: DVec2,
    mid: DVec3,
    p2: DVec2,
}

impl Arc2 {
    /// Splits the arc at its parameter midpoint (`SplitArc`, eqs. 7.39 to
    /// 7.42) into two arcs in standard form. Requires a middle weight above
    /// -1, which keeps every new weight positive.
    fn split(self) -> (Self, Self) {
        let a = self.p0.extend(1.0);
        let c = self.p2.extend(1.0);
        let q = (a + self.mid) * 0.5;
        let r = (self.mid + c) * 0.5;
        let s = (q + r) * 0.5;
        // q, r and s all have weight (1 + w) / 2; in standard form the new
        // middle weight is w_q / sqrt(w_a w_s) = sqrt((1 + w) / 2).
        let w = q.z / s.z.sqrt();
        let sp = s.truncate() / s.z;
        let qp = q.truncate() / q.z;
        let rp = r.truncate() / r.z;
        (
            Self {
                p0: self.p0,
                mid: (qp * w).extend(w),
                p2: sp,
            },
            Self {
                p0: sp,
                mid: (rp * w).extend(w),
                p2: self.p2,
            },
        )
    }
}

/// The open conic arc from `p0` to `p2` with end tangent directions `t0`
/// and `t2` that passes through `p` (The NURBS Book, A7.3 with A7.2 for
/// the weight). The arc leaves `p0` along `t0` and arrives at `p2` along
/// `t2`; the lengths of the tangents do not matter. The domain is
/// `[0, 1]`; see the module documentation for the segments.
///
/// # Errors
///
/// [`GeomError::NonFinite`] for non-finite input, and
/// [`GeomError::InvalidArgument`] (with the reason) when no such arc
/// exists: coincident end points, a zero tangent or one along the chord
/// `p2 - p0`, data that do not lie in one plane, parallel tangents that
/// point the same way, tangent directions that do not fit together, or a
/// point `p` on the chord, outside the region between the tangent lines,
/// on the wrong side of the chord for the tangents, or reachable only
/// through infinity (a middle weight of -1 or less, the book's "outside
/// convex hull" case).
pub fn conic_arc(
    p0: Point3,
    t0: Vector3,
    p2: Point3,
    t2: Vector3,
    p: Point3,
) -> Result<NurbsCurve3, GeomError> {
    use GeomError::InvalidArgument as Bad;
    if ![p0, t0, p2, t2, p].iter().all(|v| v.is_finite()) {
        return Err(GeomError::NonFinite);
    }
    let chord = p2 - p0;
    let length = chord.length();
    let size = p0.length().max(p2.length()).max(p.length()).max(1.0);
    if length <= CONIC_TOLERANCE * size {
        return Err(Bad("the end points of a conic arc must differ"));
    }
    let (Some(t0), Some(t2)) = (t0.try_normalize(), t2.try_normalize()) else {
        return Err(Bad("the tangents of a conic arc must be non-zero"));
    };
    // A frame in the plane of the conic: x along the chord, y on the side
    // of the start tangent.
    let ex = chord / length;
    let normal = ex.cross(t0);
    if normal.length() <= CONIC_TOLERANCE || ex.cross(t2).length() <= CONIC_TOLERANCE {
        return Err(Bad("a tangent of a conic arc points along the chord"));
    }
    let normal = normal.normalize();
    if t2.dot(normal).abs() > CONIC_TOLERANCE
        || (p - p0).dot(normal).abs() > CONIC_TOLERANCE * length
    {
        return Err(Bad(
            "the end points, tangents and the point of a conic arc must lie in one plane",
        ));
    }
    let ey = normal.cross(ex);
    let local = |v: Vector3| DVec2::new(v.dot(ex), v.dot(ey));
    let (q0, q2, q) = (DVec2::ZERO, DVec2::new(length, 0.0), local(p - p0));
    let (d0, d2) = (local(t0), local(t2));
    if q.y.abs() <= CONIC_TOLERANCE * length {
        return Err(Bad("the point of a conic arc lies on the chord"));
    }

    let arc = one_arc(q0, d0, q2, d2, q, length)?;
    // `MakeOpenConic`: the number of segments.
    let w1 = arc.mid.z;
    let segments = if w1 >= 1.0 {
        1
    } else if w1 > 0.0 {
        let p1 = arc.mid.truncate() / w1;
        let (a, b) = (q0 - p1, q2 - p1);
        let angle = cross2(a, b).abs().atan2(a.dot(b));
        if angle > PI / 3.0 { 1 } else { 2 }
    } else if w1 < 0.0 {
        let p1 = arc.mid.truncate() / w1;
        let (a, b) = (q0 - p1, q2 - p1);
        let angle = cross2(a, b).abs().atan2(a.dot(b));
        if angle > FRAC_PI_2 { 4 } else { 2 }
    } else {
        2
    };
    let pieces = match segments {
        1 => vec![arc],
        2 => {
            let (l, r) = arc.split();
            vec![l, r]
        }
        _ => {
            let (l, r) = arc.split();
            let (ll, lr) = l.split();
            let (rl, rr) = r.split();
            vec![ll, lr, rl, rr]
        }
    };
    let to_world = |v: DVec2| p0 + ex * v.x + ey * v.y;
    let mut control = Vec::with_capacity(2 * pieces.len() + 1);
    control.push(to_world(q0).extend(1.0));
    let mut interior = Vec::with_capacity(2 * pieces.len());
    for (i, piece) in pieces.iter().enumerate() {
        let w = piece.mid.z;
        let m = piece.mid.truncate();
        // (w P1, w) in the plane maps to (w to_world(P1), w).
        let mid3 = p0 * w + ex * m.x + ey * m.y;
        control.push(mid3.extend(w));
        let end = if i + 1 == pieces.len() {
            p2
        } else {
            to_world(piece.p2)
        };
        control.push(end.extend(1.0));
        if i + 1 < pieces.len() {
            let k = (i + 1) as f64 / pieces.len() as f64;
            interior.extend([k, k]);
        }
    }
    let knots = KnotVector::clamped_from_interior(2, 0.0, 1.0, &interior)?;
    NurbsCurve3::new(2, knots, control)
}

/// `MakeOneArc` (A7.2) in the plane of the conic: the single rational
/// quadratic arc from `q0` (the origin) to `q2` (on the positive x axis at
/// distance `length`) with tangent directions `d0`, `d2` through `q`.
/// `d0` points into the upper half plane.
fn one_arc(
    q0: DVec2,
    d0: DVec2,
    q2: DVec2,
    d2: DVec2,
    q: DVec2,
    length: f64,
) -> Result<Arc2, GeomError> {
    use GeomError::InvalidArgument as Bad;
    let chord = q2 - q0;
    let denom = cross2(d0, d2);
    if denom.abs() <= CONIC_TOLERANCE {
        // Parallel tangents: the middle control point is at infinity.
        if d0.dot(d2) > 0.0 {
            return Err(Bad(
                "parallel tangents of a conic arc must point in opposite directions",
            ));
        }
        // q + alf0 d0 = q0 + alf2 chord, on the x axis.
        let alf0 = -q.y / d0.y;
        let alf2 = (q.x + alf0 * d0.x) / length;
        if alf0 >= 0.0 {
            return Err(Bad(
                "the point of a conic arc is on the wrong side of the chord for the tangents",
            ));
        }
        if !strictly_between_0_and_1(alf2) {
            return Err(Bad(
                "the point of a conic arc must lie between the tangent lines",
            ));
        }
        let a = (alf2 / (1.0 - alf2)).sqrt();
        let u = a / (1.0 + a);
        let b = 2.0 * u * (1.0 - u);
        let b = -alf0 * (1.0 - b) / b;
        return Ok(Arc2 {
            p0: q0,
            mid: (d0 * b).extend(0.0),
            p2: q2,
        });
    }
    // q0 + alf d0 = q2 + bet d2.
    let alf = cross2(chord, d2) / denom;
    let bet = cross2(chord, d0) / denom;
    if alf * bet >= 0.0 {
        return Err(Bad(
            "the tangent directions of a conic arc do not fit together",
        ));
    }
    let p1 = q0 + d0 * alf;
    let v1 = q - p1;
    // p1 + s v1 = q0 + alf2 chord: the line from p1 through q meets the
    // chord at alf2.
    let det = cross2(v1, chord);
    if v1.length() <= CONIC_TOLERANCE * length
        || det.abs() <= CONIC_TOLERANCE * v1.length() * length
    {
        return Err(Bad(
            "the point of a conic arc must lie between the tangent lines",
        ));
    }
    let alf2 = cross2(v1, p1 - q0) / det;
    if !strictly_between_0_and_1(alf2) {
        return Err(Bad(
            "the point of a conic arc must lie between the tangent lines",
        ));
    }
    let a = (alf2 / (1.0 - alf2)).sqrt();
    let u = a / (1.0 + a);
    let num = (1.0 - u) * (1.0 - u) * (q - q0).dot(p1 - q) + u * u * (q - q2).dot(p1 - q);
    let den = 2.0 * u * (1.0 - u) * (p1 - q).dot(p1 - q);
    let w1 = num / den;
    if !w1.is_finite() || w1.abs() <= CONIC_TOLERANCE {
        return Err(Bad("the point of a conic arc lies on the chord"));
    }
    if (w1 > 0.0) != (alf > 0.0) {
        return Err(Bad(
            "the point of a conic arc is on the wrong side of the chord for the tangents",
        ));
    }
    if w1 <= -1.0 {
        return Err(Bad(
            "no conic arc with these tangents reaches the point without passing through infinity",
        ));
    }
    Ok(Arc2 {
        p0: q0,
        mid: (p1 * w1).extend(w1),
        p2: q2,
    })
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI, TAU};

    use bevel_math::{DVec2, DVec3, Frame3};

    use super::*;

    fn params(c: &NurbsCurve3, count: usize) -> Vec<f64> {
        let (a, b) = c.domain();
        (0..=count)
            .map(|i| a + (b - a) * i as f64 / count as f64)
            .collect()
    }

    fn tilted_frame() -> Frame3 {
        Frame3::from_axes(
            DVec3::new(3.0, -2.0, 5.0),
            DVec3::new(1.0, 2.0, -0.5),
            DVec3::new(-0.3, 0.4, 2.0),
        )
        .unwrap()
    }

    #[test]
    fn circle_arcs_lie_on_the_circle() {
        let f = tilted_frame();
        let r = 7.25;
        for (start, end, segments) in [
            (0.0, FRAC_PI_2, 1),
            (0.3, 0.3 + PI, 2),
            (-1.0, 3.5, 3),
            (2.0, 2.0 + 4.9, 4),
            (0.0, TAU, 4),
        ] {
            let c = circle_arc(&f, r, start, end).unwrap();
            assert_eq!(c.degree(), 2);
            assert_eq!(c.control_weighted().len(), 2 * segments + 1);
            assert_eq!(c.domain(), (start, end));
            assert!(c.is_rational());
            for u in params(&c, 400) {
                let local = f.to_local(c.point(u));
                assert!(local.z.abs() <= 1e-12 * r, "off the plane at {u}");
                let radius = local.truncate().length();
                assert!((radius - r).abs() <= 1e-12 * r, "radius {radius} at {u}");
                let k = c.curvature(u).unwrap();
                assert!((k - 1.0 / r).abs() <= 1e-9 / r, "curvature {k} at {u}");
            }
            // The ends sit at the requested angles, with tangents along the
            // direction of travel.
            for (angle, u) in [(start, start), (end, end)] {
                let expected = f.to_world(DVec3::new(r * angle.cos(), r * angle.sin(), 0.0));
                assert!((c.point(u) - expected).length() <= 1e-12 * r);
                let t = c.tangent(u).unwrap();
                let expected_t = f.vector_to_world(DVec3::new(-angle.sin(), angle.cos(), 0.0));
                assert!((t - expected_t).length() <= 1e-12);
            }
            // The parameter equals the angle at the knots.
            for k in c.knots().breakpoints() {
                let expected = f.to_world(DVec3::new(r * k.cos(), r * k.sin(), 0.0));
                assert!((c.point(k) - expected).length() <= 1e-12 * r);
            }
        }
    }

    #[test]
    fn full_circle_is_closed_and_has_the_right_length() {
        let r = 2.5;
        let c = circle_arc(&Frame3::WORLD, r, 0.4, 0.4 + TAU).unwrap();
        let pts = c.control_weighted();
        assert_eq!(pts.first(), pts.last());
        let (a, b) = c.domain();
        let l = c.arc_length(a, b, 1e-12).unwrap();
        assert!((l - TAU * r).abs() <= 1e-10 * r, "length {l}");
        let half = c.arc_length(a, a + PI, 1e-12).unwrap();
        assert!((half - PI * r).abs() <= 1e-10 * r, "half length {half}");
    }

    #[test]
    fn circle_arc_2d_matches_the_3d_arc() {
        let c2 = circle_arc_2d(DVec2::new(1.0, -2.0), 3.0, -0.5, 2.5).unwrap();
        let f = Frame3 {
            origin: DVec3::new(1.0, -2.0, 0.0),
            ..Frame3::WORLD
        };
        let c3 = circle_arc(&f, 3.0, -0.5, 2.5).unwrap();
        assert_eq!(c2.knots(), c3.knots());
        for u in params(&c3, 50) {
            let p = c3.point(u);
            assert!((c2.point(u) - p.truncate()).length() < 1e-14);
            assert!((c2.curvature(u).unwrap() - 1.0 / 3.0).abs() < 1e-12);
        }
    }

    #[test]
    fn ellipse_arcs_lie_on_the_ellipse() {
        let f = tilted_frame();
        let (a, b) = (5.0, 2.0);
        let c = ellipse_arc(&f, a, b, 0.2, 0.2 + 5.5).unwrap();
        for u in params(&c, 400) {
            let l = f.to_local(c.point(u));
            let implicit = (l.x / a).powi(2) + (l.y / b).powi(2) - 1.0;
            assert!(implicit.abs() <= 1e-12, "implicit {implicit} at {u}");
            assert!(l.z.abs() <= 1e-12 * a);
        }
        let end: f64 = 0.2 + 5.5;
        let expected = f.to_world(DVec3::new(a * end.cos(), b * end.sin(), 0.0));
        assert!((c.point(end) - expected).length() <= 1e-12 * a);
        // Curvature of an ellipse at the end of its major axis is a / b^2.
        let full = ellipse_arc(&f, a, b, 0.0, TAU).unwrap();
        assert!((full.curvature(0.0).unwrap() - a / (b * b)).abs() < 1e-10);
        assert!((full.curvature(FRAC_PI_2).unwrap() - b / (a * a)).abs() < 1e-10);

        let e2 = ellipse_arc_2d(DVec2::new(1.0, 1.0), DVec2::new(0.0, 2.0), a, b, 0.0, PI).unwrap();
        for i in 0..=100 {
            let p = e2.point(PI * f64::from(i) / 100.0) - DVec2::new(1.0, 1.0);
            // The major axis is along +y, the minor along -x.
            let implicit = (p.y / a).powi(2) + (p.x / b).powi(2) - 1.0;
            assert!(implicit.abs() <= 1e-12);
        }
        assert!((e2.point(FRAC_PI_2) - DVec2::new(1.0 - b, 1.0)).length() < 1e-12);
    }

    #[test]
    fn invalid_arcs_are_rejected() {
        let f = Frame3::WORLD;
        assert!(matches!(
            circle_arc(&f, 0.0, 0.0, 1.0),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            circle_arc(&f, f64::NAN, 0.0, 1.0),
            Err(GeomError::NonFinite)
        ));
        assert!(matches!(
            circle_arc(&f, 1.0, 1.0, 1.0),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            circle_arc(&f, 1.0, 1.0, 0.0),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            circle_arc(&f, 1.0, 0.0, 6.3),
            Err(GeomError::InvalidArgument(_))
        ));
        assert!(matches!(
            circle_arc(&f, 1.0, 0.0, f64::INFINITY),
            Err(GeomError::NonFinite)
        ));
        let skew = Frame3 {
            y: DVec3::new(0.1, 1.0, 0.0),
            ..Frame3::WORLD
        };
        assert!(matches!(
            circle_arc(&skew, 1.0, 0.0, 1.0),
            Err(GeomError::InvalidArgument(_))
        ));
        let left_handed = Frame3 {
            z: -DVec3::Z,
            ..Frame3::WORLD
        };
        assert!(circle_arc(&left_handed, 1.0, 0.0, 1.0).is_err());
        assert!(ellipse_arc(&f, 1.0, -1.0, 0.0, 1.0).is_err());
        assert!(ellipse_arc_2d(DVec2::ZERO, DVec2::ZERO, 1.0, 1.0, 0.0, 1.0).is_err());
        // A sweep a rounding error past a full turn is a full turn.
        let c = circle_arc(&f, 1.0, 0.1, 0.1 + TAU * (1.0 + 1e-15)).unwrap();
        assert_eq!(c.control_weighted().len(), 9);
    }

    fn check_conic(c: &NurbsCurve3, implicit: impl Fn(DVec3) -> f64, scale: f64) {
        for u in params(c, 400) {
            let v = implicit(c.point(u));
            assert!(v.abs() <= 1e-12 * scale, "implicit {v} at {u}");
        }
        assert!(c.weights().iter().all(|&w| w > 0.0));
    }

    fn check_ends(c: &NurbsCurve3, p0: DVec3, t0: DVec3, p2: DVec3, t2: DVec3) {
        assert!((c.point(0.0) - p0).length() < 1e-12);
        assert!((c.point(1.0) - p2).length() < 1e-12);
        assert!((c.tangent(0.0).unwrap() - t0.normalize()).length() < 1e-12);
        assert!((c.tangent(1.0).unwrap() - t2.normalize()).length() < 1e-12);
    }

    /// The closest sample of the curve to `p`.
    fn passes_through(c: &NurbsCurve3, p: DVec3) -> bool {
        c.closest_point(p).unwrap().distance < 1e-10
    }

    #[test]
    fn conic_through_a_circle_point_is_the_circle() {
        let (p0, t0) = (DVec3::X, DVec3::Y);
        let (p2, t2) = (DVec3::Y, -DVec3::X);
        let p = DVec3::new(FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0);
        let c = conic_arc(p0, t0, p2, t2, p).unwrap();
        assert_eq!(c.control_weighted().len(), 3);
        assert!((c.weights()[1] - FRAC_1_SQRT_2).abs() < 1e-14);
        check_conic(&c, |q| q.length() - 1.0, 1.0);
        check_ends(&c, p0, t0, p2, t2);
        assert!(passes_through(&c, p));
    }

    #[test]
    fn conic_classification_covers_every_case() {
        // Ellipse x^2/4 + y^2 = 1, a short arc through an off-centre point:
        // one segment.
        let ell = |q: DVec3| (q.x / 2.0).powi(2) + q.y * q.y - 1.0;
        let pt = |t: f64| DVec3::new(2.0 * t.cos(), t.sin(), 0.0);
        let tg = |t: f64| DVec3::new(-2.0 * t.sin(), t.cos(), 0.0);
        let c = conic_arc(pt(0.0), tg(0.0), pt(FRAC_PI_2), tg(FRAC_PI_2), pt(0.7)).unwrap();
        check_conic(&c, ell, 1.0);
        check_ends(&c, pt(0.0), tg(0.0), pt(FRAC_PI_2), tg(FRAC_PI_2));
        assert!(passes_through(&c, pt(0.7)));

        // A narrow ellipse arc (inner angle under 60 degrees): two segments.
        let c = conic_arc(pt(-1.2), tg(-1.2), pt(1.2), tg(1.2), pt(0.3)).unwrap();
        assert_eq!(c.control_weighted().len(), 5);
        check_conic(&c, ell, 1.0);
        assert!(passes_through(&c, pt(0.3)));

        // Three quarters of a circle: negative weight, two segments.
        let circle = |q: DVec3| q.length() - 1.0;
        let a = |t: f64| DVec3::new(t.cos(), t.sin(), 0.0);
        let ta = |t: f64| DVec3::new(-t.sin(), t.cos(), 0.0);
        let end = 1.5 * PI;
        let c = conic_arc(a(0.0), ta(0.0), a(end), ta(end), a(PI)).unwrap();
        assert_eq!(c.control_weighted().len(), 5);
        check_conic(&c, circle, 1.0);
        check_ends(&c, a(0.0), ta(0.0), a(end), ta(end));
        let (s, e) = c.domain();
        let l = c.arc_length(s, e, 1e-12).unwrap();
        assert!((l - end).abs() < 1e-10, "length {l}");

        // 300 degrees of a circle: four segments.
        let end = 5.0 * PI / 3.0;
        let c = conic_arc(a(0.0), ta(0.0), a(end), ta(end), a(PI)).unwrap();
        assert_eq!(c.control_weighted().len(), 9);
        check_conic(&c, circle, 1.0);
        let l = c.arc_length(0.0, 1.0, 1e-12).unwrap();
        assert!((l - end).abs() < 1e-10, "length {l}");

        // Parabola y = x^2.
        let c = conic_arc(
            DVec3::new(-1.0, 1.0, 0.0),
            DVec3::new(1.0, -2.0, 0.0),
            DVec3::new(1.0, 1.0, 0.0),
            DVec3::new(1.0, 2.0, 0.0),
            DVec3::ZERO,
        )
        .unwrap();
        check_conic(&c, |q| q.y - q.x * q.x, 1.0);
        assert!(passes_through(&c, DVec3::ZERO));

        // Hyperbola x^2 - y^2 = 1 between t = -1 and t = 1.
        let h = |t: f64| DVec3::new(t.cosh(), t.sinh(), 0.0);
        let th = |t: f64| DVec3::new(t.sinh(), t.cosh(), 0.0);
        let c = conic_arc(h(-1.0), th(-1.0), h(1.0), th(1.0), h(0.4)).unwrap();
        assert_eq!(c.control_weighted().len(), 3);
        assert!(c.weights()[1] > 1.0);
        check_conic(&c, |q| q.x * q.x - q.y * q.y - 1.0, 3.0);
        check_ends(&c, h(-1.0), th(-1.0), h(1.0), th(1.0));
        assert!(passes_through(&c, h(0.4)));
    }

    #[test]
    fn conic_with_parallel_tangents_is_a_half_ellipse() {
        // Half of x^2/9 + y^2/4 = 1 in a tilted plane, through an
        // off-centre point.
        let f = tilted_frame();
        let pt = |t: f64| f.to_world(DVec3::new(3.0 * t.cos(), 2.0 * t.sin(), 0.0));
        let tg = |t: f64| f.vector_to_world(DVec3::new(-3.0 * t.sin(), 2.0 * t.cos(), 0.0));
        let c = conic_arc(pt(0.0), tg(0.0), pt(PI), tg(PI), pt(1.1)).unwrap();
        assert_eq!(c.control_weighted().len(), 5);
        let ell = |q: DVec3| {
            let l = f.to_local(q);
            (l.x / 3.0).powi(2) + (l.y / 2.0).powi(2) - 1.0 + l.z.abs()
        };
        check_conic(&c, ell, 1.0);
        check_ends(&c, pt(0.0), tg(0.0), pt(PI), tg(PI));
        assert!(passes_through(&c, pt(1.1)));
    }

    #[test]
    fn impossible_conics_are_rejected() {
        let bad = |p0, t0, p2, t2, p| {
            matches!(
                conic_arc(p0, t0, p2, t2, p),
                Err(GeomError::InvalidArgument(_))
            )
        };
        let (x, y, z) = (DVec3::X, DVec3::Y, DVec3::Z);
        let mid = DVec3::new(0.7, 0.7, 0.0);
        // Coincident ends, zero tangent, tangent along the chord.
        assert!(bad(x, y, x, -x, mid));
        assert!(bad(x, DVec3::ZERO, y, -x, mid));
        assert!(bad(x, y - x, y, -x, mid));
        // Not planar.
        assert!(bad(x, y, y, -x, mid + z * 0.1));
        assert!(bad(x, y + z, y, -x, mid));
        // Parallel tangents pointing the same way.
        assert!(bad(x, y, -x, y, y));
        // Parallel tangents with the point on the wrong side or outside.
        assert!(bad(x, y, -x, -y, -y));
        assert!(bad(x, y, -x, -y, DVec3::new(3.0, 1.0, 0.0)));
        // The point on the chord, outside the tangent lines, or behind.
        assert!(bad(x, y, y, -x, DVec3::new(0.5, 0.5, 0.0)));
        assert!(bad(x, y, y, -x, DVec3::new(1.5, 0.2, 0.0)));
        assert!(bad(x, y, y, -x, DVec3::new(0.2, 0.2, 0.0)));
        // Tangents that do not fit together.
        assert!(bad(x, y, y, x, mid));
        // The point at the apex of the tangents.
        assert!(bad(x, y, y, -x, DVec3::new(1.0, 1.0, 0.0)));
        assert!(matches!(
            conic_arc(x, y, y, -x, DVec3::new(f64::NAN, 0.0, 0.0)),
            Err(GeomError::NonFinite)
        ));
    }
}
