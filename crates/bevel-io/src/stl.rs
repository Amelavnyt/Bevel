// SPDX-License-Identifier: GPL-3.0-or-later

//! STL reading and writing.
//!
//! STL has no formal standard; this module follows the de-facto
//! specification (3D Systems, "Stereolithography Interface Specification",
//! 1988, as commonly implemented). A binary file is an 80-byte header, a
//! little-endian `u32` triangle count, and 50 bytes per triangle (normal,
//! three vertices as `f32` triplets, and a 16-bit attribute word). An ASCII
//! file is a sequence of `solid <name>` blocks of `facet normal` /
//! `outer loop` / `vertex` records.
//!
//! STL carries no units. Everything written here is in millimetres, which
//! is what slicers and most CAD tools assume. Coordinates are rounded to
//! `f32`. Colours and names cannot be stored in binary STL, and several
//! meshes in one binary file simply become one run of triangles; ASCII
//! output keeps one `solid` block per mesh.
//!
//! Facet normals are computed from the triangle (unit length, zero for a
//! degenerate triangle) so readers that trust them see a consistent
//! outward orientation.

use std::io::Write;

use crate::{ExportMesh, IoError};

const HEADER_LEN: usize = 80;
const BINARY_PREFIX: usize = HEADER_LEN + 4;
const TRIANGLE_LEN: usize = 50;

/// Writes the meshes as one binary STL file.
///
/// The 80-byte header reads `Bevel` followed by spaces (it deliberately does
/// not start with `solid`, which would make some readers mistake the file
/// for ASCII). Coordinates are millimetres.
///
/// # Errors
///
/// [`IoError::InvalidMesh`] if a mesh fails [`ExportMesh::validate`], has a
/// coordinate too large for `f32`, or the total triangle count exceeds
/// `u32::MAX`; [`IoError::Io`] if writing fails.
pub fn write_binary(meshes: &[ExportMesh], out: &mut impl Write) -> Result<(), IoError> {
    let mut total: u64 = 0;
    for m in meshes {
        m.validate()?;
        total += m.triangles.len() as u64;
    }
    let count = u32::try_from(total).map_err(|_| {
        IoError::mesh(
            meshes.first().map_or("", |m| m.name.as_str()),
            "there are more triangles than a binary STL file can hold",
        )
    })?;

    let mut header = [b' '; HEADER_LEN];
    header[..5].copy_from_slice(b"Bevel");
    out.write_all(&header)?;
    out.write_all(&count.to_le_bytes())?;

    for m in meshes {
        for tri in &m.triangles {
            let corners = corners_f32(m, *tri)?;
            let normal = facet_normal(&corners);
            let mut record = [0u8; TRIANGLE_LEN];
            let floats = normal.iter().chain(corners.iter().flatten());
            for (chunk, v) in record.chunks_exact_mut(4).zip(floats) {
                chunk.copy_from_slice(&v.to_le_bytes());
            }
            // The last two bytes are the attribute word, left at zero.
            out.write_all(&record)?;
        }
    }
    Ok(())
}

/// Writes the meshes as ASCII STL, one `solid` block per mesh.
///
/// Numbers use nine significant digits (`1.00000000e0`), enough to
/// reproduce any `f32` exactly. Mesh names are written after `solid`, with
/// line breaks and other control characters replaced by spaces. Coordinates
/// are millimetres.
///
/// # Errors
///
/// [`IoError::InvalidMesh`] if a mesh fails [`ExportMesh::validate`] or has a
/// coordinate too large for `f32`; [`IoError::Io`] if writing fails.
pub fn write_ascii(meshes: &[ExportMesh], out: &mut impl Write) -> Result<(), IoError> {
    for m in meshes {
        m.validate()?;
        let name: String = m
            .name
            .chars()
            .map(|c| if c.is_control() { ' ' } else { c })
            .collect();
        writeln!(out, "solid {name}")?;
        for tri in &m.triangles {
            let corners = corners_f32(m, *tri)?;
            let n = facet_normal(&corners);
            writeln!(
                out,
                "  facet normal {:.8e} {:.8e} {:.8e}",
                f64::from(n[0]),
                f64::from(n[1]),
                f64::from(n[2])
            )?;
            writeln!(out, "    outer loop")?;
            for c in &corners {
                writeln!(
                    out,
                    "      vertex {:.8e} {:.8e} {:.8e}",
                    f64::from(c[0]),
                    f64::from(c[1]),
                    f64::from(c[2])
                )?;
            }
            writeln!(out, "    endloop")?;
            writeln!(out, "  endfacet")?;
        }
        writeln!(out, "endsolid {name}")?;
    }
    Ok(())
}

/// Reads a binary or ASCII STL file into a flat list of triangles.
///
/// The format is detected from the size: if the bytes are exactly
/// `84 + 50 * n` long, where `n` is the count stored at byte 80, the file is
/// binary (even when its header starts with `solid`). Otherwise a file whose
/// first word is `solid` is parsed as ASCII. Facet normals are ignored; all
/// `solid` blocks of an ASCII file are concatenated.
///
/// # Errors
///
/// [`IoError::InvalidFile`] if the data is neither valid binary nor valid
/// ASCII STL, or contains a number that is not finite.
pub fn read(bytes: &[u8]) -> Result<Vec<[[f32; 3]; 3]>, IoError> {
    if looks_binary(bytes) {
        return read_binary(bytes);
    }
    let text = std::str::from_utf8(bytes).map_err(|_| {
        IoError::file(
            "STL",
            "the size does not match a binary file and the text is not ASCII",
        )
    })?;
    if !text.trim_start().starts_with("solid") {
        return Err(IoError::file(
            "STL",
            "it is neither a binary file of the right size nor an ASCII file starting with \"solid\"",
        ));
    }
    read_ascii(text)
}

fn looks_binary(bytes: &[u8]) -> bool {
    let Some(count) = bytes.get(HEADER_LEN..BINARY_PREFIX) else {
        return false;
    };
    let mut raw = [0u8; 4];
    raw.copy_from_slice(count);
    let n = u64::from(u32::from_le_bytes(raw));
    bytes.len() as u64 == BINARY_PREFIX as u64 + TRIANGLE_LEN as u64 * n
}

fn read_binary(bytes: &[u8]) -> Result<Vec<[[f32; 3]; 3]>, IoError> {
    let body = bytes.get(BINARY_PREFIX..).unwrap_or_default();
    let mut triangles = Vec::with_capacity(body.len() / TRIANGLE_LEN);
    for record in body.chunks_exact(TRIANGLE_LEN) {
        let mut v = [0f32; 12];
        for (slot, chunk) in v.iter_mut().zip(record.chunks_exact(4)) {
            let mut raw = [0u8; 4];
            raw.copy_from_slice(chunk);
            *slot = f32::from_le_bytes(raw);
        }
        if v.iter().any(|x| !x.is_finite()) {
            return Err(IoError::file("STL", "a coordinate is not a finite number"));
        }
        // v[0..3] is the stored normal, which is ignored.
        triangles.push([[v[3], v[4], v[5]], [v[6], v[7], v[8]], [v[9], v[10], v[11]]]);
    }
    Ok(triangles)
}

fn read_ascii(text: &str) -> Result<Vec<[[f32; 3]; 3]>, IoError> {
    let mut words = text.split_whitespace();
    let mut vertices: Vec<[f32; 3]> = Vec::new();
    while let Some(word) = words.next() {
        if !word.eq_ignore_ascii_case("vertex") {
            continue;
        }
        let mut p = [0f32; 3];
        for slot in &mut p {
            let token = words
                .next()
                .ok_or_else(|| IoError::file("STL", "a vertex is missing coordinates"))?;
            let value: f32 = token
                .parse()
                .map_err(|_| IoError::file("STL", format!("\"{token}\" is not a valid number")))?;
            if !value.is_finite() {
                return Err(IoError::file("STL", "a coordinate is not a finite number"));
            }
            *slot = value;
        }
        vertices.push(p);
    }
    if !vertices.len().is_multiple_of(3) {
        return Err(IoError::file(
            "STL",
            "the number of vertices is not a multiple of three",
        ));
    }
    Ok(vertices
        .chunks_exact(3)
        .map(|c| [c[0], c[1], c[2]])
        .collect())
}

/// The three corners of a triangle as `f32`, failing on overflow.
fn corners_f32(mesh: &ExportMesh, tri: [u32; 3]) -> Result<[[f32; 3]; 3], IoError> {
    let mut out = [[0f32; 3]; 3];
    for (slot, &v) in out.iter_mut().zip(&tri) {
        let p = mesh
            .positions
            .get(v as usize)
            .ok_or_else(|| IoError::mesh(&mesh.name, "a triangle refers to a missing vertex"))?;
        for (s, &c) in slot.iter_mut().zip(p) {
            let f = c as f32;
            if !f.is_finite() {
                return Err(IoError::mesh(
                    &mesh.name,
                    "a coordinate is too large for STL (limit about 3.4e38)",
                ));
            }
            *s = f;
        }
    }
    Ok(out)
}

/// Unit normal from the corner order (right-hand rule), zero if degenerate.
///
/// Computed in `f64` from the `f32` corners that are actually written.
fn facet_normal(c: &[[f32; 3]; 3]) -> [f32; 3] {
    let d = |a: usize, b: usize| -> [f64; 3] {
        [
            f64::from(c[b][0]) - f64::from(c[a][0]),
            f64::from(c[b][1]) - f64::from(c[a][1]),
            f64::from(c[b][2]) - f64::from(c[a][2]),
        ]
    };
    let (u, v) = (d(0, 1), d(0, 2));
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 0.0 && len.is_finite() {
        [
            (n[0] / len) as f32,
            (n[1] / len) as f32,
            (n[2] / len) as f32,
        ]
    } else {
        [0.0; 3]
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::test_util::unit_cube;

    fn cube_triangles() -> Vec<[[f32; 3]; 3]> {
        let cube = unit_cube();
        cube.triangles
            .iter()
            .map(|t| t.map(|v| cube.positions[v as usize].map(|c| c as f32)))
            .collect()
    }

    #[test]
    fn binary_round_trip() {
        let mut bytes = Vec::new();
        write_binary(&[unit_cube()], &mut bytes).unwrap();
        assert_eq!(bytes.len(), 84 + 50 * 12);
        assert_eq!(&bytes[..5], b"Bevel");
        assert!(bytes[5..80].iter().all(|&b| b == b' '));
        assert_eq!(read(&bytes).unwrap(), cube_triangles());
    }

    #[test]
    fn ascii_round_trip() {
        let mut bytes = Vec::new();
        write_ascii(&[unit_cube()], &mut bytes).unwrap();
        let text = String::from_utf8(bytes.clone()).unwrap();
        assert!(text.starts_with("solid Cube\n"));
        assert!(text.trim_end().ends_with("endsolid Cube"));
        assert_eq!(text.matches("facet normal").count(), 12);
        assert_eq!(read(&bytes).unwrap(), cube_triangles());
    }

    #[test]
    fn binary_normals_point_outward() {
        let mut bytes = Vec::new();
        write_binary(&[unit_cube()], &mut bytes).unwrap();
        let f = |i: usize| {
            let mut raw = [0u8; 4];
            raw.copy_from_slice(&bytes[84 + i * 4..88 + i * 4]);
            f32::from_le_bytes(raw)
        };
        // The first cube triangle is on the bottom face (z = 0).
        assert_eq!([f(0), f(1), f(2)], [0.0, 0.0, -1.0]);
    }

    #[test]
    fn degenerate_normal_is_zero() {
        let n = facet_normal(&[[0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);
        assert_eq!(n, [0.0; 3]);
    }

    #[test]
    fn binary_header_starting_with_solid_is_still_binary() {
        let mut bytes = Vec::new();
        write_binary(&[unit_cube()], &mut bytes).unwrap();
        bytes[..5].copy_from_slice(b"solid");
        assert_eq!(read(&bytes).unwrap(), cube_triangles());
    }

    #[test]
    fn rejects_garbage_and_truncation() {
        assert!(read(b"hello world").is_err());
        assert!(read(&[]).is_err());
        let mut bytes = Vec::new();
        write_binary(&[unit_cube()], &mut bytes).unwrap();
        bytes[0] = 0xFF;
        bytes.truncate(bytes.len() - 7);
        assert!(read(&bytes).is_err());
        assert!(read(b"solid x\nfacet normal 0 0 0\nvertex 1 2\n").is_err());
        assert!(read(b"solid x\nvertex 1 2 3\nvertex 4 5 6\n").is_err());
    }

    #[test]
    fn writers_reject_invalid_meshes() {
        let mut cube = unit_cube();
        cube.positions[0][0] = f64::NAN;
        assert!(write_binary(&[cube.clone()], &mut Vec::new()).is_err());
        assert!(write_ascii(&[cube], &mut Vec::new()).is_err());
        let mut big = unit_cube();
        big.positions[0][0] = 1e300;
        assert!(write_binary(&[big], &mut Vec::new()).is_err());
    }

    #[test]
    fn output_is_deterministic() {
        let meshes = [unit_cube(), unit_cube()];
        let run = |ascii: bool| {
            let mut v = Vec::new();
            if ascii {
                write_ascii(&meshes, &mut v).unwrap();
            } else {
                write_binary(&meshes, &mut v).unwrap();
            }
            v
        };
        assert_eq!(run(false), run(false));
        assert_eq!(run(true), run(true));
    }

    #[test]
    fn multiple_meshes_are_concatenated() {
        let mut bytes = Vec::new();
        write_binary(&[unit_cube(), unit_cube()], &mut bytes).unwrap();
        assert_eq!(read(&bytes).unwrap().len(), 24);
        let mut ascii = Vec::new();
        write_ascii(&[unit_cube(), unit_cube()], &mut ascii).unwrap();
        assert_eq!(read(&ascii).unwrap().len(), 24);
    }
}
