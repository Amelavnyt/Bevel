// SPDX-License-Identifier: GPL-3.0-or-later
// Vertical gradient behind the scene, drawn as one full-screen triangle.

struct VertexOut {
    @builtin(position) position: vec4<f32>,
    @location(0) t: f32,
};

@vertex
fn vs_main(@builtin(vertex_index) index: u32) -> VertexOut {
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    var out: VertexOut;
    // Depth 0 is the far plane under reverse-Z.
    out.position = vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
    out.t = uv.y;
    return out;
}

@fragment
fn fs_main(in: VertexOut) -> @location(0) vec4<f32> {
    return to_srgb(mix(globals.background_bottom, globals.background_top, clamp(in.t, 0.0, 1.0)));
}
