// SPDX-License-Identifier: GPL-3.0-or-later
// Per-frame uniforms shared by every viewport pass. Keep in sync with
// `Globals` in renderer.rs.

struct Globals {
    view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    // xyz: eye position in world space.
    eye: vec4<f32>,
    // x: minor cell size, y: major cell size, z: fade radius, w: half extent.
    grid: vec4<f32>,
    // xy: grid centre in the XY plane.
    grid_center: vec4<f32>,
    // xy: viewport size in pixels, z: 1 for orthographic projection.
    viewport: vec4<f32>,
    background_top: vec4<f32>,
    background_bottom: vec4<f32>,
    grid_minor: vec4<f32>,
    grid_major: vec4<f32>,
    axis_x: vec4<f32>,
    axis_y: vec4<f32>,
};

@group(0) @binding(0) var<uniform> globals: Globals;

// The render target stores sRGB-encoded values in a UNORM format (see
// OUTPUT_FORMAT in renderer.rs), so every pass encodes its linear result.
fn to_srgb(color: vec4<f32>) -> vec4<f32> {
    let c = clamp(color.rgb, vec3<f32>(0.0), vec3<f32>(1.0));
    let low = c * 12.92;
    let high = 1.055 * pow(c, vec3<f32>(1.0 / 2.4)) - 0.055;
    return vec4<f32>(select(high, low, c <= vec3<f32>(0.0031308)), color.a);
}
