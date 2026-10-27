// SPDX-License-Identifier: GPL-3.0-or-later

//! CPU-side packing of vertex and instance data into the byte layouts the
//! shaders read.
//!
//! Everything is packed by hand into 32-bit words (`f32` values through
//! [`f32::to_bits`]), so no unsafe casts are needed. The layouts are
//! described by the `*_FORMATS` arrays, which the pipelines use for their
//! vertex buffer layouts; a test parses the shader sources and checks that
//! the attributes declared there match.

use crate::{LineSet, TriangleMesh};

/// Attributes of a mesh vertex, in shader location order: position,
/// normal, colour, id. See `VertexIn` in shaders/mesh.wgsl.
pub(crate) const MESH_VERTEX_FORMATS: [wgpu::VertexFormat; 4] = [
    wgpu::VertexFormat::Float32x3,
    wgpu::VertexFormat::Float32x3,
    wgpu::VertexFormat::Float32x4,
    wgpu::VertexFormat::Uint32,
];
/// Words in a packed mesh vertex.
pub(crate) const MESH_VERTEX_WORDS: usize = 3 + 3 + 4 + 1;

/// Attributes of a line segment instance, in shader location order: start,
/// end, colour, id, width. See `Instance` in shaders/lines.wgsl.
pub(crate) const LINE_INSTANCE_FORMATS: [wgpu::VertexFormat; 5] = [
    wgpu::VertexFormat::Float32x3,
    wgpu::VertexFormat::Float32x3,
    wgpu::VertexFormat::Float32x4,
    wgpu::VertexFormat::Uint32,
    wgpu::VertexFormat::Float32,
];
/// Words in a packed line segment instance.
pub(crate) const LINE_INSTANCE_WORDS: usize = 3 + 3 + 4 + 1 + 1;

/// Bytes between consecutive elements of a buffer with the given attributes.
pub(crate) fn stride_of(formats: &[wgpu::VertexFormat]) -> u64 {
    formats.iter().map(wgpu::VertexFormat::size).sum()
}

/// Little-endian bytes of the words.
pub(crate) fn words_to_bytes(words: &[u32]) -> Vec<u8> {
    words.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// Little-endian bytes of the floats.
pub(crate) fn floats_to_bytes(values: &[f32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}

fn push_floats(words: &mut Vec<u32>, values: &[f32]) {
    words.extend(values.iter().map(|v| v.to_bits()));
}

/// The vertices of `mesh` as words: position, normal, colour, id each.
///
/// The mesh must have passed its validity check, so the normal and id
/// slices are as long as the positions; shorter ones are padded with zero
/// rather than panicking.
pub(crate) fn pack_mesh_vertices(mesh: &TriangleMesh) -> Vec<u32> {
    let mut words = Vec::with_capacity(mesh.positions.len() * MESH_VERTEX_WORDS);
    for (i, position) in mesh.positions.iter().enumerate() {
        push_floats(&mut words, position);
        push_floats(&mut words, mesh.normals.get(i).unwrap_or(&[0.0; 3]));
        push_floats(&mut words, &mesh.color);
        words.push(mesh.ids.get(i).copied().unwrap_or(0));
    }
    words
}

/// One line segment instance: start, end, colour, id, width.
pub(crate) fn pack_line_instance(
    start: [f32; 3],
    end: [f32; 3],
    color: [f32; 4],
    id: u32,
    width: f32,
) -> [u32; LINE_INSTANCE_WORDS] {
    let [s0, s1, s2] = start.map(f32::to_bits);
    let [e0, e1, e2] = end.map(f32::to_bits);
    let [c0, c1, c2, c3] = color.map(f32::to_bits);
    [s0, s1, s2, e0, e1, e2, c0, c1, c2, c3, id, width.to_bits()]
}

/// The segments of all line sets as instances. Returns the words and the
/// number of segments.
pub(crate) fn pack_line_instances(sets: &[LineSet]) -> (Vec<u32>, usize) {
    let mut words = Vec::new();
    let mut count = 0;
    for set in sets {
        for (segment, &id) in set.positions.chunks_exact(2).zip(&set.ids) {
            words.extend(pack_line_instance(
                segment[0], segment[1], set.color, id, set.width,
            ));
            count += 1;
        }
    }
    (words, count)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MESH_SHADER: &str = include_str!("shaders/mesh.wgsl");
    const LINES_SHADER: &str = include_str!("shaders/lines.wgsl");

    /// The `@location(n) name: type,` members of a WGSL struct, as
    /// `(location, type)` in declaration order.
    fn struct_locations(source: &str, name: &str) -> Vec<(u32, String)> {
        let header = format!("struct {name} {{");
        let body = source
            .split_once(&header)
            .and_then(|(_, rest)| rest.split_once('}'))
            .map_or_else(|| panic!("struct {name} not found"), |(body, _)| body);
        body.lines()
            .filter_map(|line| {
                let line = line.trim().strip_prefix("@location(")?;
                let (location, rest) = line.split_once(')')?;
                let (_, ty) = rest.split_once(':')?;
                Some((
                    location.parse().expect("location number"),
                    ty.trim().trim_end_matches(',').to_owned(),
                ))
            })
            .collect()
    }

    fn format_of(wgsl_type: &str) -> wgpu::VertexFormat {
        match wgsl_type {
            "f32" => wgpu::VertexFormat::Float32,
            "u32" => wgpu::VertexFormat::Uint32,
            "vec2<f32>" => wgpu::VertexFormat::Float32x2,
            "vec3<f32>" => wgpu::VertexFormat::Float32x3,
            "vec4<f32>" => wgpu::VertexFormat::Float32x4,
            other => panic!("unexpected attribute type {other}"),
        }
    }

    /// Checks that the shader declares exactly the attributes in `formats`,
    /// at consecutive locations starting at 0.
    fn assert_shader_matches(source: &str, struct_name: &str, formats: &[wgpu::VertexFormat]) {
        let declared = struct_locations(source, struct_name);
        let locations: Vec<u32> = (0..formats.len() as u32).collect();
        assert_eq!(
            declared.iter().map(|(l, _)| *l).collect::<Vec<_>>(),
            locations
        );
        let declared: Vec<_> = declared.iter().map(|(_, t)| format_of(t)).collect();
        assert_eq!(declared, formats);
    }

    #[test]
    fn mesh_vertex_layout_matches_the_shader() {
        assert_shader_matches(MESH_SHADER, "VertexIn", &MESH_VERTEX_FORMATS);
        assert_eq!(
            stride_of(&MESH_VERTEX_FORMATS),
            (MESH_VERTEX_WORDS * 4) as u64
        );
        // Offsets in bytes: position 0, normal 12, colour 24, id 40.
        let mut offsets = Vec::new();
        let mut at = 0;
        for f in MESH_VERTEX_FORMATS {
            offsets.push(at);
            at += f.size();
        }
        assert_eq!(offsets, [0, 12, 24, 40]);
    }

    #[test]
    fn line_instance_layout_matches_the_shader() {
        assert_shader_matches(LINES_SHADER, "Instance", &LINE_INSTANCE_FORMATS);
        assert_eq!(
            stride_of(&LINE_INSTANCE_FORMATS),
            (LINE_INSTANCE_WORDS * 4) as u64
        );
        let mut offsets = Vec::new();
        let mut at = 0;
        for f in LINE_INSTANCE_FORMATS {
            offsets.push(at);
            at += f.size();
        }
        // Start 0, end 12, colour 24, id 40, width 44.
        assert_eq!(offsets, [0, 12, 24, 40, 44]);
    }

    #[test]
    fn mesh_vertices_are_packed_in_attribute_order() {
        let mesh = TriangleMesh {
            positions: vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
            normals: vec![[0.0, 0.0, 1.0], [0.0, 1.0, 0.0]],
            indices: vec![0, 1, 0],
            color: [0.1, 0.2, 0.3, 0.4],
            ids: vec![11, 22],
        };
        let words = pack_mesh_vertices(&mesh);
        assert_eq!(words.len(), 2 * MESH_VERTEX_WORDS);
        let second = &words[MESH_VERTEX_WORDS..];
        assert_eq!(second[0], 4.0f32.to_bits());
        assert_eq!(second[2], 6.0f32.to_bits());
        assert_eq!(second[4], 1.0f32.to_bits());
        assert_eq!(second[6], 0.1f32.to_bits());
        assert_eq!(second[9], 0.4f32.to_bits());
        assert_eq!(second[10], 22);
        assert_eq!(words[10], 11);
        assert_eq!(words_to_bytes(&words).len(), 2 * 44);
    }

    #[test]
    fn line_instances_carry_set_colour_width_and_segment_ids() {
        let a = LineSet {
            positions: vec![[0.0; 3], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]],
            ids: vec![5, 6],
            color: [1.0, 0.0, 0.0, 1.0],
            width: 2.0,
        };
        let b = LineSet {
            positions: vec![[0.0, 0.0, 9.0], [0.0, 0.0, 10.0]],
            ids: vec![7],
            ..LineSet::default()
        };
        let (words, count) = pack_line_instances(&[a, b]);
        assert_eq!(count, 3);
        assert_eq!(words.len(), 3 * LINE_INSTANCE_WORDS);
        let second = &words[LINE_INSTANCE_WORDS..2 * LINE_INSTANCE_WORDS];
        assert_eq!(second[0], 1.0f32.to_bits());
        assert_eq!(second[3], 1.0f32.to_bits());
        assert_eq!(second[4], 1.0f32.to_bits());
        assert_eq!(second[6], 1.0f32.to_bits());
        assert_eq!(second[10], 6);
        assert_eq!(second[11], 2.0f32.to_bits());
        let third = &words[2 * LINE_INSTANCE_WORDS..];
        assert_eq!(third[2], 9.0f32.to_bits());
        assert_eq!(third[10], 7);
        assert_eq!(third[11], LineSet::DEFAULT_WIDTH.to_bits());
    }
}
