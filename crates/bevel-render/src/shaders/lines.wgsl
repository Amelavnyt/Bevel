// SPDX-License-Identifier: GPL-3.0-or-later
// One-pixel line segments (edges, axes). Lines are pulled slightly towards
// the camera so edges win the depth test against the faces they bound.

struct VertexIn {
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct VertexOut {
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(in: VertexIn) -> VertexOut {
    var out: VertexOut;
    var clip = globals.view_proj * vec4<f32>(in.position, 1.0);
    if globals.viewport.z > 0.5 {
        // Orthographic depth is linear: offset by a fixed fraction of the range.
        clip.z += 2e-5 * clip.w;
    } else {
        // Reverse-Z perspective depth is n / distance: scaling it moves the
        // point a fixed fraction of its distance towards the eye.
        clip.z *= 1.0015;
    }
    out.position = clip;
    out.color = in.color;
    return out;
}

@fragment
fn fs_main(in: VertexOut) -> @location(0) vec4<f32> {
    return to_srgb(in.color);
}
