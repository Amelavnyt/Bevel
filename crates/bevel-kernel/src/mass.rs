// SPDX-License-Identifier: GPL-3.0-or-later

//! Mass properties of solids: volume, surface area, centre of mass and
//! the inertia tensor at unit density, with its principal moments and
//! axes.
//!
//! # Method
//!
//! Every volume integral becomes a surface integral by the divergence
//! theorem, `int_V div F dV = sum over faces of int_face F . n dA`, and on
//! a face `n dA = s (S_u x S_v) du dv` with `s = -1` for a reversed face
//! and `+1` otherwise. With `x = S(u, v) - c` relative to a reference
//! point `c` (which keeps cancellation small for bodies far from the
//! origin) and `N = s S_u x S_v`:
//!
//! ```text
//! volume          V    = int_V 1 dV       = 1/3 sum int (x . N) du dv    F = x / 3
//! area            A    = sum int |S_u x S_v| du dv
//! first moments   M_i  = int_V x_i dV     = 1/4 sum int x_i (x . N)      F = x_i x / 4
//! second moments  P_ij = int_V x_i x_j dV = 1/5 sum int x_i x_j (x . N)  F = x_i x_j x / 5
//! ```
//!
//! (`div(x_i x) = 4 x_i` and `div(x_i x_j x) = 5 x_i x_j`.) These fields
//! are radial, so every integrand is a multiple of `x . N` like the
//! volume's: the result does not depend on the orientation of the world
//! axes, `P` is symmetric by construction, and a face that misses its
//! neighbour by a hair (a very flat cone stores its half-angle, so its rim
//! can sit 1e-8 mm off its cap) leaks flux only in proportion to `x . n`
//! across the gap. The axis-aligned fields `x_i^2 / 2 e_i` would leak in
//! proportion to `x_i^2` and turn such a gap into a centroid error of
//! 1e-4 mm. The centroid is `g = c + M / V`; the second moments are then
//! taken again with `c = g`, so the inertia `I = trace(P) E - P` is about
//! the centroid without a parallel-axis subtraction.
//!
//! # Quadrature
//!
//! Each face's trimmed `(u, v)` domain is triangulated as for
//! tessellation (angular tolerance 0.3 rad, no chordal or length limit,
//! so the triangles are small in angle but not over-refined), and each
//! `(u, v)` triangle is integrated with an 8 x 8 Gauss-Legendre product
//! rule on the collapsed square `(s, t) -> a + s (b - a) + (1 - s) t (c -
//! a)` (Jacobian `(1 - s)` times twice the area), of the exact surface
//! integrands above. The rule is exact for polynomials of degree 14 in
//! `(u, v)`, and the trigonometric integrands of the analytic surfaces
//! vary by well under a radian over a triangle, so the quadrature error is
//! at rounding level.
//!
//! The triangulation's boundary is a polygon through the pcurves'
//! points, which is exact where a pcurve is straight in `(u, v)` (every
//! side face of a primitive). A curved pcurve (the circle bounding a
//! planar cap) adds the sliver between each polygon segment `A B` and its
//! arc `C(t)`, `t` from `t0` to `t1`, parametrized as `Q(tau, sigma) = L +
//! sigma (C(t) - L)` with `L = A + tau (B - A)` and `t = t0 + tau (t1 -
//! t0)`, integrated with the same rule and Jacobian `Q_tau x Q_sigma`. Its
//! sign follows the loop's orientation (the face lies left of its loops in
//! `(u, v)` unless it is reversed), so a bulge outwards adds and one
//! inwards subtracts. The result is exact up to quadrature and rounding
//! for any smooth trim, not just for straight ones.
//!
//! # Principal axes
//!
//! The principal moments and axes are the eigenvalues and eigenvectors of
//! the symmetric inertia tensor, by cyclic Jacobi rotations (which are
//! accurate for symmetric matrices, repeated eigenvalues included).
//! Moments are sorted in increasing order, and the axes are the columns
//! of a rotation matrix (orthonormal and right-handed).

use bevel_geom::{Curve2, CurveGeom, SurfaceGeom};
use bevel_math::quadrature::gauss_legendre;
use bevel_math::{DMat3, DVec3, Point2, Point3, Vector3};

use crate::KernelError;
use crate::naming::OpContext;
use crate::tessellate::face::FaceDomain;
use crate::tessellate::{MAX_POINTS, TessellationOptions, discretize_edges, triangulate_face};
use crate::topology::{Body, BodyKind, EntityKey, Face, FaceKey};

/// The angular tolerance of the triangulation used for integration.
const ANGULAR: f64 = 0.3;

/// Gauss-Legendre points per direction.
const GAUSS_POINTS: usize = 8;

/// Jacobi sweeps before giving up (convergence is quadratic; 3x3 needs a
/// handful).
const JACOBI_SWEEPS: usize = 64;

/// The mass properties of a solid at unit density (see the
/// [module documentation](self)).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    /// The enclosed volume in mm³ (positive).
    pub volume: f64,
    /// The total area of the faces in mm².
    pub area: f64,
    /// The centre of mass.
    pub centroid: Point3,
    /// The inertia tensor about the centroid at unit density, in mm⁵:
    /// `I_xx = int (y^2 + z^2) dV`, `I_xy = -int x y dV`, and so on.
    pub inertia: DMat3,
    /// The principal moments, the eigenvalues of `inertia`, in increasing
    /// order.
    pub principal_moments: DVec3,
    /// The principal axes as the columns of a rotation matrix: column `k`
    /// is the axis of `principal_moments[k]`.
    pub principal_axes: DMat3,
}

/// One quadrature node: the point relative to the reference point, the
/// oriented `S_u x S_v` there and the weight.
#[derive(Clone, Copy, Debug)]
struct Node {
    x: Vector3,
    n: Vector3,
    w: f64,
}

/// The mass properties of a solid (see the [module documentation](self)).
///
/// # Errors
///
/// [`KernelError::InvalidInput`] for a sheet body (which encloses no
/// volume; its area alone is not computed) or a body whose faces enclose
/// no positive volume, the tessellation errors of the integration
/// domain ([`KernelError::PointLimit`], [`KernelError::Tessellation`]),
/// and [`KernelError::Cancelled`].
pub fn mass_properties(body: &Body, cx: &OpContext) -> Result<MassProperties, KernelError> {
    if body.kind() != BodyKind::Solid {
        return Err(KernelError::InvalidInput {
            what: "body",
            reason: "mass properties need a solid body; a sheet encloses no volume",
        });
    }
    cx.check_cancelled()?;
    let bounds = body.bounds();
    let size = bounds.size().length();
    if bounds.is_empty() || !(size.is_finite() && size > 0.0) {
        return Err(KernelError::InvalidInput {
            what: "body",
            reason: "it has no finite, positive extent",
        });
    }
    let options = TessellationOptions {
        chordal: size,
        angular: ANGULAR,
        max_edge: None,
    };
    let rule = rule()?;
    let edges = discretize_edges(body, &options, MAX_POINTS, cx)?;
    let reference = bounds.center();
    let mut nodes: Vec<Node> = Vec::new();
    for (key, face) in body.faces() {
        cx.check_cancelled()?;
        let domain = triangulate_face(body, key, &edges, &options, MAX_POINTS, cx)?;
        face_nodes(body, key, face, &domain, &rule, reference, &mut nodes)?;
    }

    // Pass 1: volume, area, centroid.
    let (mut volume, mut area, mut first) = (0.0, 0.0, Vector3::ZERO);
    for q in &nodes {
        let flux = q.w * q.x.dot(q.n);
        volume += flux;
        area += q.w * q.n.length();
        first += q.x * flux;
    }
    volume /= 3.0;
    first /= 4.0;
    if !(volume.is_finite() && volume > 0.0) {
        return Err(KernelError::InvalidInput {
            what: "body",
            reason: "its faces enclose no positive volume",
        });
    }
    let shift = first / volume;
    let centroid = reference + shift;

    // Pass 2: second moments about the centroid.
    let mut p = [[0.0f64; 3]; 3];
    for q in &nodes {
        let x = q.x - shift;
        let flux = q.w * x.dot(q.n) / 5.0;
        let x = x.to_array();
        for (i, row) in p.iter_mut().enumerate() {
            for (j, pij) in row.iter_mut().enumerate().skip(i) {
                *pij += flux * x[i] * x[j];
            }
        }
    }
    let trace = p[0][0] + p[1][1] + p[2][2];
    let mut inertia = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            let pij = if i <= j { p[i][j] } else { p[j][i] };
            inertia[i][j] = if i == j { trace - pij } else { -pij };
        }
    }
    let (moments, axes) = symmetric_eigen(inertia);
    let to_mat = |m: [[f64; 3]; 3]| {
        DMat3::from_cols(
            DVec3::new(m[0][0], m[1][0], m[2][0]),
            DVec3::new(m[0][1], m[1][1], m[2][1]),
            DVec3::new(m[0][2], m[1][2], m[2][2]),
        )
    };
    Ok(MassProperties {
        volume,
        area,
        centroid,
        inertia: to_mat(inertia),
        principal_moments: DVec3::from_array(moments),
        principal_axes: to_mat(axes),
    })
}

/// The Gauss-Legendre rule on `[0, 1]`: `(nodes, weights)`.
fn rule() -> Result<Vec<(f64, f64)>, KernelError> {
    let (x, w) = gauss_legendre(GAUSS_POINTS).map_err(|_| KernelError::InvalidInput {
        what: "quadrature rule",
        reason: "the Gauss-Legendre rule could not be computed",
    })?;
    Ok(x.iter()
        .zip(&w)
        .map(|(&x, &w)| (0.5 * (x + 1.0), 0.5 * w))
        .collect())
}

/// Appends the quadrature nodes of one face: its `(u, v)` triangles and
/// the slivers of its curved boundary segments.
fn face_nodes(
    body: &Body,
    key: FaceKey,
    face: &Face,
    d: &FaceDomain,
    rule: &[(f64, f64)],
    reference: Point3,
    nodes: &mut Vec<Node>,
) -> Result<(), KernelError> {
    let sign = if face.reversed { -1.0 } else { 1.0 };
    let surface = &face.surface;
    let mut push = |uv: Point2, w: f64| {
        let (su, sv) = surface.partials(uv.x, uv.y);
        nodes.push(Node {
            x: surface.point(uv.x, uv.y) - reference,
            n: su.cross(sv) * sign,
            w,
        });
    };
    let uv = |i: u32| d.uv.get(i as usize).copied();
    for t in &d.triangles {
        let (Some(a), Some(b), Some(c)) = (uv(t[0]), uv(t[1]), uv(t[2])) else {
            return Err(KernelError::Tessellation {
                entity: EntityKey::Face(key),
                reason: "a triangle refers to a point that does not exist",
            });
        };
        let (ab, ac) = (b - a, c - a);
        let jacobian = ab.perp_dot(ac);
        for &(s, ws) in rule {
            for &(r, wr) in rule {
                let p = a + ab * s + ac * ((1.0 - s) * r);
                push(p, ws * wr * (1.0 - s) * jacobian);
            }
        }
    }
    // Slivers between curved pcurves and their chords. The face lies left
    // of its loops in (u, v) unless it is reversed.
    let orientation = sign;
    for seg in &d.segments {
        let Some(coedge) = body.coedge(seg.coedge) else {
            return Err(KernelError::UnknownKey(EntityKey::Coedge(seg.coedge)));
        };
        let pcurve = &coedge.pcurve;
        if matches!(pcurve, Curve2::Line(_)) {
            continue;
        }
        let (Some(a), Some(b)) = (uv(seg.a), uv(seg.b)) else {
            continue;
        };
        let dt = seg.t1 - seg.t0;
        for &(tau, wt) in rule {
            let t = seg.t0 + dt * tau;
            let d = pcurve.derivatives(t, 1);
            let (Some(&c), Some(&dc)) = (d.first(), d.get(1)) else {
                continue;
            };
            let l = a + (b - a) * tau;
            let q_sigma = c - l;
            for &(sigma, ws) in rule {
                let q = l + q_sigma * sigma;
                let q_tau = (b - a) * (1.0 - sigma) + dc * (sigma * dt);
                let jacobian = q_tau.perp_dot(q_sigma);
                push(q, -orientation * wt * ws * jacobian);
            }
        }
    }
    Ok(())
}

/// The eigenvalues (increasing) and eigenvectors (the columns of a
/// rotation matrix, `v[row][column]`) of a symmetric 3x3 matrix, by cyclic
/// Jacobi rotations.
pub(crate) fn symmetric_eigen(m: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut a = m;
    // Symmetrize from the upper triangle.
    a[1][0] = a[0][1];
    a[2][0] = a[0][2];
    a[2][1] = a[1][2];
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let norm2: f64 = a.iter().flatten().map(|x| x * x).sum();
    for _ in 0..JACOBI_SWEEPS {
        let off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        if off <= f64::EPSILON * f64::EPSILON * norm2 * 1e-4 || !off.is_finite() {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            rotate(&mut a, &mut v, p, q);
        }
    }
    let mut order = [0usize, 1, 2];
    order.sort_by(|&i, &j| a[i][i].total_cmp(&a[j][j]));
    let values = order.map(|k| a[k][k]);
    let mut vectors = [[0.0f64; 3]; 3];
    for (col, &k) in order.iter().enumerate() {
        for row in 0..3 {
            vectors[row][col] = v[row][k];
        }
    }
    // Right-handed: column 2 = column 0 x column 1.
    let c = |j: usize| DVec3::new(vectors[0][j], vectors[1][j], vectors[2][j]);
    if c(0).cross(c(1)).dot(c(2)) < 0.0 {
        for row in &mut vectors {
            row[2] = -row[2];
        }
    }
    (values, vectors)
}

/// One Jacobi rotation zeroing `a[p][q]` (`p < q`), accumulated into `v`.
// The updates read and write two rows or columns of `a` at once.
#[allow(clippy::needless_range_loop)]
fn rotate(a: &mut [[f64; 3]; 3], v: &mut [[f64; 3]; 3], p: usize, q: usize) {
    let apq = a[p][q];
    let scale = a[p][p].abs() + a[q][q].abs();
    if apq.abs() <= f64::MIN_POSITIVE || apq.abs() <= f64::EPSILON * 1e-3 * scale {
        a[p][q] = 0.0;
        a[q][p] = 0.0;
        return;
    }
    let theta = (a[q][q] - a[p][p]) / (2.0 * apq);
    let t = if theta.abs() > 1e150 {
        0.5 / theta
    } else {
        theta.signum() / (theta.abs() + theta.hypot(1.0))
    };
    let c = 1.0 / t.hypot(1.0);
    let s = t * c;
    for k in 0..3 {
        let (akp, akq) = (a[k][p], a[k][q]);
        a[k][p] = c * akp - s * akq;
        a[k][q] = s * akp + c * akq;
    }
    for k in 0..3 {
        let (apk, aqk) = (a[p][k], a[q][k]);
        a[p][k] = c * apk - s * aqk;
        a[q][k] = s * apk + c * aqk;
    }
    a[p][q] = 0.0;
    a[q][p] = 0.0;
    for row in v.iter_mut() {
        let (vp, vq) = (row[p], row[q]);
        row[p] = c * vp - s * vq;
        row[q] = s * vp + c * vq;
    }
}

#[cfg(test)]
#[allow(clippy::needless_range_loop)] // index-heavy matrix checks
mod tests {
    use std::sync::Arc;
    use std::sync::atomic::AtomicBool;

    use bevel_geom::{Curve2, Curve3, Line2, Line3, Plane, Surface};
    use bevel_math::{Frame3, Interval, Vector2};

    use super::*;
    use crate::{BodyBuilder, BoxParams, OpTag, Role, Source, TopoName, box_};

    /// A tiny deterministic generator for test matrices.
    struct Lcg(u64);

    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
        }
    }

    /// Checks `A v_k = lambda_k v_k`, orthonormality, right-handedness and
    /// the order.
    fn check_eigen(m: [[f64; 3]; 3]) {
        let (values, vectors) = symmetric_eigen(m);
        let norm = m
            .iter()
            .flatten()
            .fold(0.0f64, |a, b| a.max(b.abs()))
            .max(1e-300);
        let col = |j: usize| DVec3::new(vectors[0][j], vectors[1][j], vectors[2][j]);
        let a = DMat3::from_cols(
            DVec3::new(m[0][0], m[0][1], m[0][2]),
            DVec3::new(m[0][1], m[1][1], m[1][2]),
            DVec3::new(m[0][2], m[1][2], m[2][2]),
        );
        for k in 0..3 {
            let residual = a * col(k) - col(k) * values[k];
            assert!(residual.length() <= 1e-13 * norm, "{m:?}: {residual:?}");
            for j in 0..3 {
                let expected = if j == k { 1.0 } else { 0.0 };
                assert!((col(k).dot(col(j)) - expected).abs() < 1e-14);
            }
        }
        assert!(values[0] <= values[1] && values[1] <= values[2]);
        assert!((col(0).cross(col(1)).dot(col(2)) - 1.0).abs() < 1e-14);
    }

    #[test]
    fn jacobi_diagonalizes_symmetric_matrices() {
        let mut rng = Lcg(7);
        for _ in 0..500 {
            let mut m = [[0.0; 3]; 3];
            let scale = 10f64.powf(6.0 * rng.next());
            for i in 0..3 {
                for j in i..3 {
                    m[i][j] = rng.next() * scale;
                    m[j][i] = m[i][j];
                }
            }
            check_eigen(m);
        }
    }

    #[test]
    fn jacobi_handles_special_matrices() {
        check_eigen([[0.0; 3]; 3]);
        check_eigen([[3.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 2.0]]);
        // Repeated eigenvalues (a sphere's or cylinder's tensor, rotated).
        check_eigen([[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]);
        check_eigen([[2.0, 1.0, 0.0], [1.0, 2.0, 0.0], [0.0, 0.0, 3.0]]);
        // Nearly diagonal, and badly scaled.
        check_eigen([[1.0, 1e-17, 0.0], [1e-17, 1.0, 1e-20], [0.0, 1e-20, 1.0]]);
        check_eigen([[1e12, 1.0, 0.0], [1.0, 1e-12, 0.0], [0.0, 0.0, 1.0]]);
        let (values, _) = symmetric_eigen([[2.0, 1.0, 0.0], [1.0, 2.0, 0.0], [0.0, 0.0, 3.0]]);
        assert!((values[0] - 1.0).abs() < 1e-15);
        assert!((values[1] - 3.0).abs() < 1e-15 && (values[2] - 3.0).abs() < 1e-15);
    }

    fn unit_box() -> Arc<Body> {
        let params = BoxParams {
            frame: Frame3::WORLD,
            size: Vector3::new(1.0, 2.0, 3.0),
        };
        box_(&params, OpTag(1), &mut OpContext::new())
            .unwrap()
            .bodies
            .remove(0)
    }

    #[test]
    fn a_box_has_its_closed_forms() {
        let m = mass_properties(&unit_box(), &OpContext::new()).unwrap();
        assert!((m.volume - 6.0).abs() < 1e-13);
        assert!((m.area - 22.0).abs() < 1e-13);
        assert!(m.centroid.distance(Point3::new(0.5, 1.0, 1.5)) < 1e-14);
        let expected = DMat3::from_diagonal(DVec3::new(13.0, 10.0, 5.0) * (6.0 / 12.0));
        assert!(
            (m.inertia - expected)
                .to_cols_array()
                .iter()
                .all(|x| x.abs() < 1e-13)
        );
        assert!(
            (m.principal_moments - DVec3::new(2.5, 5.0, 6.5))
                .abs()
                .max_element()
                < 1e-13
        );
        // The smallest moment is about the long axis.
        assert!(m.principal_axes.col(0).z.abs() > 1.0 - 1e-14);
    }

    #[test]
    fn cancellation_stops_mass_properties() {
        let cx = OpContext::with_cancel(Arc::new(AtomicBool::new(true)));
        assert_eq!(
            mass_properties(&unit_box(), &cx),
            Err(KernelError::Cancelled)
        );
    }

    #[test]
    fn sheets_are_refused() {
        // One open unit square.
        let name = |role| TopoName::generated(OpTag(1), Source::Primitive, role);
        let mut b = BodyBuilder::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let vertices: Vec<_> = (0u32..)
            .zip(corners)
            .map(|(i, p)| b.add_vertex(p, 1e-6, name(Role::Vertex(i))))
            .collect();
        let face = b.add_face(
            Surface::Plane(Plane::new(Frame3::WORLD).unwrap()),
            false,
            name(Role::Top),
        );
        let mut coedges = Vec::new();
        for i in 0..4 {
            let (p, q) = (corners[i], corners[(i + 1) % 4]);
            let e = b
                .add_edge(
                    Curve3::Line(Line3::through(p, q).unwrap()),
                    Interval::new(0.0, 1.0),
                    vertices[i],
                    vertices[(i + 1) % 4],
                    1e-6,
                    name(Role::Edge(i as u32)),
                )
                .unwrap();
            let pcurve = Curve2::Line(
                Line2::new(Point2::new(p.x, p.y), Vector2::new(q.x - p.x, q.y - p.y)).unwrap(),
            );
            coedges.push((e, false, pcurve));
        }
        b.add_loop(face, &coedges).unwrap();
        b.add_shell(&[face], false).unwrap();
        let sheet = b.finish(BodyKind::Sheet).unwrap();
        let err = mass_properties(&sheet, &OpContext::new()).unwrap_err();
        assert!(
            matches!(err, KernelError::InvalidInput { what: "body", .. }),
            "{err}"
        );
    }
}
