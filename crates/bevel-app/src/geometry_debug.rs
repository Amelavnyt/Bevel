// SPDX-License-Identifier: GPL-3.0-or-later

//! The geometry debug scene (View > Geometry Debug): sample NURBS curves and
//! surfaces built with `bevel-geom`, sampled into the renderer's plain mesh
//! and line buffers. It exercises evaluation, normals, conics, fitting and
//! surfaces of revolution until real bodies exist.

use bevel_geom::fit::{self, Parametrization};
use bevel_geom::{GeomError, NurbsCurve3, NurbsSurface, circle_arc};
use bevel_math::{Frame3, Point3, Vector3};
use bevel_render::{LineSet, TriangleMesh};
use std::f64::consts::TAU;

/// Linear colour of sampled curves (the light signature teal).
const CURVE: [f32; 4] = [0.098, 0.40, 0.38, 1.0];
/// Linear colour of control polygons.
const CONTROL: [f32; 4] = [0.30, 0.31, 0.33, 1.0];
/// Linear colour of sampled surfaces.
const SURFACE: [f32; 4] = [0.48, 0.52, 0.54, 1.0];

/// Builds the debug scene.
///
/// # Errors
///
/// Any error from constructing the sample geometry.
pub fn scene() -> Result<(Vec<TriangleMesh>, Vec<LineSet>), GeomError> {
    let mut meshes = Vec::new();
    let mut lines = Vec::new();

    // An exact rational circle.
    let frame = Frame3 {
        origin: Point3::new(-110.0, 0.0, 0.0),
        ..Frame3::WORLD
    };
    lines.push(curve_lines(
        &circle_arc(&frame, 25.0, 0.0, TAU)?,
        128,
        CURVE,
    ));

    // A cubic interpolated through points, with its control polygon.
    let points = [
        Point3::new(-55.0, -20.0, 0.0),
        Point3::new(-40.0, 15.0, 20.0),
        Point3::new(-20.0, -10.0, 35.0),
        Point3::new(0.0, 20.0, 25.0),
        Point3::new(15.0, -15.0, 10.0),
        Point3::new(30.0, 10.0, 30.0),
    ];
    let spline: NurbsCurve3 = fit::interpolate(&points, 3, Parametrization::Centripetal)?;
    lines.push(curve_lines(&spline, 200, CURVE));
    lines.push(polyline(&spline.control_points(), CONTROL));
    lines.push(markers(&points, 1.5, CURVE));

    // A torus: a full circle revolved about the Z axis.
    let tube = Frame3::from_axes(
        Point3::new(95.0, 0.0, 9.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    )
    .map_err(|_| GeomError::InvalidArgument("degenerate tube frame"))?;
    let tube_circle = circle_arc(&tube, 8.0, 0.0, TAU)?;
    let torus =
        NurbsSurface::revolution(&tube_circle, Point3::new(75.0, 0.0, 0.0), Vector3::Z, TAU)?;
    meshes.push(surface_mesh(&torus, 64, 32, SURFACE));

    // A vase: an interpolated profile revolved about a vertical axis.
    let axis = Point3::new(0.0, 75.0, 0.0);
    let profile_points = [
        (14.0, 0.0),
        (18.0, 10.0),
        (12.0, 25.0),
        (8.0, 35.0),
        (11.0, 45.0),
    ]
    .map(|(r, z)| axis + Vector3::new(r, 0.0, z));
    let profile: NurbsCurve3 = fit::interpolate(&profile_points, 3, Parametrization::ChordLength)?;
    let vase = NurbsSurface::revolution(&profile, axis, Vector3::Z, TAU)?;
    meshes.push(surface_mesh(&vase, 64, 32, SURFACE));
    lines.push(curve_lines(&profile, 100, CURVE));

    Ok((meshes, lines))
}

fn to_f32(p: Point3) -> [f32; 3] {
    p.as_vec3().to_array()
}

/// The curve as `segments` straight segments, uniform in its parameter.
fn curve_lines(curve: &NurbsCurve3, segments: usize, color: [f32; 4]) -> LineSet {
    let (a, b) = curve.domain();
    let at = |i: usize| curve.point(a + (b - a) * i as f64 / segments as f64);
    let positions = (0..segments)
        .flat_map(|i| [to_f32(at(i)), to_f32(at(i + 1))])
        .collect();
    LineSet { positions, color }
}

/// Straight segments between consecutive points.
fn polyline(points: &[Point3], color: [f32; 4]) -> LineSet {
    let positions = points
        .windows(2)
        .flat_map(|w| [to_f32(w[0]), to_f32(w[1])])
        .collect();
    LineSet { positions, color }
}

/// A small three-axis cross at each point.
fn markers(points: &[Point3], size: f64, color: [f32; 4]) -> LineSet {
    let positions = points
        .iter()
        .flat_map(|&p| {
            [Vector3::X, Vector3::Y, Vector3::Z]
                .into_iter()
                .flat_map(move |d| [to_f32(p - size * d), to_f32(p + size * d)])
        })
        .collect();
    LineSet { positions, color }
}

/// The surface sampled on a `nu` x `nv` grid of quads, uniform in its
/// parameters, with exact normals where they exist.
fn surface_mesh(surface: &NurbsSurface, nu: usize, nv: usize, color: [f32; 4]) -> TriangleMesh {
    let ((u0, u1), (v0, v1)) = surface.domain();
    let mut mesh = TriangleMesh {
        color,
        ..TriangleMesh::default()
    };
    for i in 0..=nu {
        let u = u0 + (u1 - u0) * i as f64 / nu as f64;
        for j in 0..=nv {
            let v = v0 + (v1 - v0) * j as f64 / nv as f64;
            mesh.positions.push(to_f32(surface.point(u, v)));
            // Normals vanish at poles; any unit vector keeps the shading finite.
            let n = surface.normal(u, v).unwrap_or(Vector3::Z);
            mesh.normals.push(n.as_vec3().to_array());
        }
    }
    let row = nv as u32 + 1;
    for i in 0..nu as u32 {
        for j in 0..nv as u32 {
            let a = i * row + j;
            let (b, c, d) = (a + row, a + row + 1, a + 1);
            mesh.indices.extend([a, b, c, a, c, d]);
        }
    }
    mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scene_builds_and_is_finite() {
        let (meshes, lines) = scene().unwrap();
        assert_eq!(meshes.len(), 2);
        assert!(!lines.is_empty());
        for mesh in &meshes {
            assert!(mesh.positions.iter().flatten().all(|c| c.is_finite()));
            assert!(mesh.normals.iter().flatten().all(|c| c.is_finite()));
        }
    }

    #[test]
    fn surface_mesh_has_a_full_grid() {
        let (meshes, _) = scene().unwrap();
        let torus = &meshes[0];
        assert_eq!(torus.positions.len(), 65 * 33);
        assert_eq!(torus.indices.len(), 6 * 64 * 32);
        let count = torus.positions.len() as u32;
        assert!(torus.indices.iter().all(|&i| i < count));
    }

    #[test]
    fn torus_samples_lie_on_the_torus() {
        let (meshes, _) = scene().unwrap();
        // Major radius 20 about (75, 0), minor radius 8 around z = 9.
        for p in &meshes[0].positions {
            let (x, y, z) = (
                f64::from(p[0]) - 75.0,
                f64::from(p[1]),
                f64::from(p[2]) - 9.0,
            );
            let ring = (x.hypot(y) - 20.0).hypot(z);
            assert!((ring - 8.0).abs() < 1e-4, "off the torus by {}", ring - 8.0);
        }
    }
}
